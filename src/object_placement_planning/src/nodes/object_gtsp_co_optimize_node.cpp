// ROS node for object_gtsp_co_optimize (object_gtsp_co_optimizer.hpp): reads the parameters, builds the scene, runs the
// one descent, measures the result (planned length and time, reconfigurations, joint margin) and writes it out.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/object_gtsp_co_optimizer.hpp"
#include "object_placement_planning/object_placement.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"
#include "object_placement_planning/viewpoint_types.hpp"

namespace
{

using Joints = std::vector<double>;

// The surface the robot stands on (top face at the base plane), as in object_placement, so all methods see the same world.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;	// m
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	const std::string base_link = scene->getRobotModel()->getRootLinkName();
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = base_link;
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {size, size, thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * thickness;
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	scene->processCollisionObjectMsg(surface);
	// The base rests on it: allow every link fixed to the root.
	for (const moveit::core::LinkModel* link : scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == scene->getRobotModel()->getRootLink())
			scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// RViz marker for the mounting surface, as object_placement draws it.
visualization_msgs::msg::Marker GetMountingSurfaceMarker(const std::string& base_link, const rclcpp::Time& stamp)
{
	const double size = 4.0, thickness = 0.05;	// m, as AddMountingSurface
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = base_link;
	marker.header.stamp = stamp;
	marker.ns = "mounting_surface";
	marker.type = visualization_msgs::msg::Marker::CUBE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.z = -0.5 * thickness;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = thickness;
	marker.color = marker_utils::Rgba(0.55f, 0.45f, 0.35f, 0.6f);
	return marker;
}

}  // namespace

class ObjectGtspCoOptimizeNode : public rclcpp::Node
{
public:
	ObjectGtspCoOptimizeNode()
		: Node("object_gtsp_co_optimize", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = this->getParameter<std::string>("group_name", "panda_arm");
		std::string mesh_path = this->getParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->getParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->getParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = this->getParameter<std::string>("output_dir", "/tmp/object_gtsp_co_optimize_output");
		const Joints home_pose = this->getParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal = ToVector3(this->getParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d object_rotation = RotationFromRpyDeg(this->getParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		const Eigen::Vector3d start(this->getParameter<double>("initial_x", 0.5), this->getParameter<double>("initial_y", 0.0), this->getParameter<double>("initial_z", 0.15));
		const double planning_time = this->getParameter<double>("real_cost_planning_time", 0.5);
		const int planning_attempts = static_cast<int>(this->getParameter<int64_t>("real_cost_attempts", 3));
		const double unreachable_penalty = this->getParameter<double>("unreachable_penalty", 50.0);
		const bool keep_alive = this->getParameter<bool>("keep_alive", false);  // keep republishing the final markers until Ctrl-C (for RViz)
		// Drive the robot (real, or robot_sim.launch.py) through the final tour; needs move_group running.
		const bool execute_on_robot = this->getParameter<bool>("execute_on_robot", false);
		const double execution_planning_time = this->getParameter<double>("execution_planning_time", 5.0);
		const int execution_planning_attempts = static_cast<int>(this->getParameter<int64_t>("execution_planning_attempts", 5));
		const object_gtsp_co_optimize::DescentLimits descent_limits = this->readDescentLimits();
		const object_gtsp_co_optimize::TourSettings tour_settings = this->readTourSettings();
		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs.stl";

		const std::vector<Eigen::Isometry3d> viewpoints_nominal = LoadTourTcpPoses(tour_input_dir);	// abs, object at nominal
		std::vector<Eigen::Isometry3d> viewpoints = viewpoints_nominal;  // relative to the object position
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= nominal;
		const int num_viewpoint = static_cast<int>(viewpoints.size());

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		const planning_scene_monitor::PlanningSceneMonitorPtr scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, object_rotation);
		AddMountingSurface(scene);
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_markers", rclcpp::QoS(1).transient_local());
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_progress_markers", rclcpp::QoS(1).transient_local());
		visualization_msgs::msg::MarkerArray surface_markers;
		surface_markers.markers.push_back(GetMountingSurfaceMarker(model->getRootLinkName(), this->now()));
		marker_pub->publish(surface_markers);
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; co-optimizing in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s", num_viewpoint, descent_limits.search_box_lower_bound.x(), descent_limits.search_box_upper_bound.x(), descent_limits.search_box_lower_bound.y(), descent_limits.search_box_upper_bound.y(), descent_limits.search_box_lower_bound.z(), descent_limits.search_box_upper_bound.z(), descent_limits.time_budget);

		const auto start_time = std::chrono::steady_clock::now();
		auto GetElapsedSeconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
		object_gtsp_co_optimize::Optimizer optimizer(this->shared_from_this(), model, scene, group_name, viewpoints, object_rotation, home_pose, descent_limits, this->readMethodSwitches(), this->readCostWeights(), this->readStepSizes(), this->readIkSettings(), tour_settings, progress_pub, mesh_path, mesh_scale);
		// One descent from the given start, no restarts.
		const object_gtsp_co_optimize::ObjectPositionEvaluation best = optimizer.runDescent(start);
		const double search_seconds = GetElapsedSeconds();
		const bool all_reached = best.viewpoint_ik.num_reached == num_viewpoint;

		// Planned tour length and, for reference, execution time: straight moves where collision-free, else OMPL (camera spin
		// aside in the length; timed with TOTG for the time).
		const std::vector<Joints> tour_poses = optimizer.getPosesInTourOrder(best.viewpoint_ik, best.order);
		const std::vector<int>& counted_joints = optimizer.getCountedJoints();
		const joint_travel::EdgeWeights& edge_weights = optimizer.getEdgeWeights();
		scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(best.viewpoint_ik.position, object_rotation));
		const std::vector<double> edge_lengths = PlanTourEdgeLengths(this->shared_from_this(), model, scene, tour_poses, home_pose, group_name, planning_time, planning_attempts, /*count_last_joint=*/false);
		double planned_length = 0.0;
		int num_failed_length_edges = 0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_lengths.size() && edge_lengths[k] >= 0.0;
			num_failed_length_edges += edge_ok ? 0 : 1;
			planned_length += edge_ok ? edge_lengths[k] : joint_travel::JointDistance(k == 0 ? home_pose : tour_poses[k - 1], tour_poses[k], counted_joints) + unreachable_penalty;
		}
		const std::vector<double> edge_times = PlanTourEdgeTimes(this->shared_from_this(), model, scene, tour_poses, home_pose, group_name, planning_time, planning_attempts);
		double planned_time = 0.0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_times.size() && edge_times[k] >= 0.0;
			planned_time += edge_ok ? edge_times[k] : joint_travel::StraightMoveTime(k == 0 ? home_pose : tour_poses[k - 1], tour_poses[k], edge_weights.max_velocity, edge_weights.max_acceleration) + unreachable_penalty;
		}
		std::vector<Joints> poses_from_home = tour_poses;
		poses_from_home.insert(poses_from_home.begin(), home_pose);
		const int num_reconfigurations = joint_travel::CountReconfigs(poses_from_home, tour_settings.reconfig_threshold, counted_joints);
		const double smallest_margin = optimizer.getSmallestJointMargin(tour_poses);
		const double seconds = GetElapsedSeconds();
		if (all_reached)
			RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned length %.2f rad (%d failed edges), %d reconfigurations, smallest joint margin %.3f rad beyond the hard limit; planned time %.2f s for reference (%.1f s).", best.viewpoint_ik.position.x(), best.viewpoint_ik.position.y(), best.viewpoint_ik.position.z(), num_viewpoint, planned_length, num_failed_length_edges, num_reconfigurations, smallest_margin, planned_time, seconds);
		else
			RCLCPP_WARN(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%.1f s).", best.viewpoint_ik.position.x(), best.viewpoint_ik.position.y(), best.viewpoint_ik.position.z(), std::max(0, best.viewpoint_ik.num_reached), num_viewpoint, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = std::max(0, best.viewpoint_ik.num_reached);
		out["num_total"] = num_viewpoint;
		out["x"] = best.viewpoint_ik.position.x();
		out["y"] = best.viewpoint_ik.position.y();
		out["z"] = best.viewpoint_ik.position.z();
		out["planned_travel"] = all_reached ? planned_length : -1.0;  // rad
		out["planned_time"] = all_reached ? planned_time : -1.0;  // s, reference only
		out["straight_tour_cost"] = best.joint_travel_cost;
		out["failed_edges"] = num_failed_length_edges;
		out["reconfigurations"] = num_reconfigurations;
		out["smallest_joint_margin"] = smallest_margin;
		out["search_seconds"] = search_seconds;
		out["seconds"] = seconds;
		for (int viewpoint : best.order)
			out["tour_order"].append(viewpoint);
		for (const Joints& pose : tour_poses)
		{
			Json::Value joints(Json::arrayValue);
			for (double value : pose)
				joints.append(value);
			out["joint_solutions"].append(joints);
		}
		std::ofstream(output_dir + "/object_gtsp_co_optimize_result.json") << out;

		// Final markers on /object_placement_markers, as object_placement draws them: the object and the tour's viewpoints.
		ObjectPlacementResult result;
		result.ok = all_reached;
		result.x = best.viewpoint_ik.position.x();
		result.y = best.viewpoint_ik.position.y();
		result.z = best.viewpoint_ik.position.z();
		result.tour_order = best.order;
		for (int viewpoint : best.order)
			result.joint_solutions.push_back(best.viewpoint_ik.reached[static_cast<size_t>(viewpoint)] ? best.viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(viewpoint)].q : Joints());
		visualization_msgs::msg::MarkerArray final_markers = BuildObjectPlacementMarkerArray(this->now(), mesh_path, mesh_scale, nominal, object_rotation, viewpoints_nominal, result);
		final_markers.markers.push_back(GetMountingSurfaceMarker(model->getRootLinkName(), this->now()));
		marker_pub->publish(final_markers);
		optimizer.publishProgressMarkers(best.viewpoint_ik, best.order, Eigen::Vector3d::Zero());

		if (execute_on_robot && !all_reached)
			RCLCPP_WARN(this->get_logger(), "Not executing: the tour misses viewpoints.");
		else if (execute_on_robot)
		{
			RCLCPP_WARN(this->get_logger(), "execute_on_robot: driving the robot against the object at abs (%.4f, %.4f, %.4f) m -- on a real robot only correct if the object is actually fixtured there.", best.viewpoint_ik.position.x(), best.viewpoint_ik.position.y(), best.viewpoint_ik.position.z());
			const moveit::planning_interface::MoveGroupInterfacePtr move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), group_name);
			move_group->startStateMonitor();
			move_group->setPlanningTime(execution_planning_time);
			move_group->setNumPlanningAttempts(execution_planning_attempts);
			move_group->setMaxVelocityScalingFactor(1.0);
			move_group->setMaxAccelerationScalingFactor(1.0);
			move_group->setEndEffectorLink("tool0");
			// The unwrapped tour poses, so the robot turns full-turn joints the short way, as the tour length counts them.
			std::vector<ViewpointCandidate> tour_viewpoints(tour_poses.size());
			std::vector<const ViewpointCandidate*> selected;
			for (size_t k = 0; k < tour_poses.size(); ++k)
			{
				tour_viewpoints[k].tcp_pose = scene_geometry::ToPoseMsg(scene_geometry::Translation(best.viewpoint_ik.position) * viewpoints[static_cast<size_t>(best.order[k])]);
				tour_viewpoints[k].joint_solution = tour_poses[k];
				tour_viewpoints[k].reachable = true;
				selected.push_back(&tour_viewpoints[k]);
			}
			ExecuteTourOnRobot(this->shared_from_this(), model, scene, move_group, marker_pub, selected, home_pose, group_name, execution_planning_time, execution_planning_attempts);
		}
		while (keep_alive && rclcpp::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(2));
			marker_pub->publish(final_markers);
		}
	}

private:
	template <typename T>
	T getParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}

	// The search box, time budget and iteration limit.
	object_gtsp_co_optimize::DescentLimits readDescentLimits()
	{
		object_gtsp_co_optimize::DescentLimits descent_limits;
		descent_limits.search_box_lower_bound = Eigen::Vector3d(this->getParameter<double>("x_min", -0.6), this->getParameter<double>("y_min", -0.6), this->getParameter<double>("z_min", -0.2));
		descent_limits.search_box_upper_bound = Eigen::Vector3d(this->getParameter<double>("x_max", 0.6), this->getParameter<double>("y_max", 0.6), this->getParameter<double>("z_max", 0.2));
		descent_limits.time_budget = this->getParameter<double>("co_time_budget_sec", 300.0);
		descent_limits.max_steps = static_cast<int>(this->getParameter<int64_t>("co_max_steps", 200));
		return descent_limits;
	}

	// Parts of the method that can be turned off (ablations).
	object_gtsp_co_optimize::MethodSwitches readMethodSwitches()
	{
		object_gtsp_co_optimize::MethodSwitches method_switches;
		method_switches.use_room = this->getParameter<bool>("co_room", true);
		method_switches.use_neighbor_ik = this->getParameter<bool>("co_neighbor_ik", true);
		method_switches.use_posture_match = this->getParameter<bool>("co_posture_match", true);
		method_switches.use_slide = this->getParameter<bool>("co_slide", true);
		method_switches.tour_object_move = this->getParameter<bool>("co_tour_object_move", true);
		method_switches.freeze_object_when_reached = this->getParameter<bool>("co_freeze_object_when_reached", false);
		return method_switches;
	}

	// The weighted total cost's weights.
	object_gtsp_co_optimize::CostWeights readCostWeights()
	{
		object_gtsp_co_optimize::CostWeights cost_weights;
		cost_weights.initial_penalty = this->getParameter<double>("co_initial_penalty", 100.0);
		cost_weights.penalty_growth = this->getParameter<double>("co_penalty_growth", 10.0);
		cost_weights.max_penalty = this->getParameter<double>("co_max_penalty", 1e5);
		cost_weights.miss_cost = this->getParameter<double>("co_miss_cost", 10.0);
		cost_weights.reconfiguration_cost = this->getParameter<double>("co_reconfiguration_cost", 2.0);
		cost_weights.reconfiguration_weight = this->getParameter<double>("co_reconfiguration_weight", 10.0);
		cost_weights.blocked_edge_charge = this->getParameter<double>("co_blocked_edge_charge", 4.0);
		cost_weights.joint_room_band = this->getParameter<double>("co_joint_room_band", 0.2);
		cost_weights.joint_room_weight = this->getParameter<double>("co_joint_room_weight", 50.0);
		return cost_weights;
	}

	// Object step and pose slide sizes.
	object_gtsp_co_optimize::StepSizes readStepSizes()
	{
		object_gtsp_co_optimize::StepSizes step_sizes;
		step_sizes.max_line_search_tries = static_cast<int>(this->getParameter<int64_t>("co_max_line_search_tries", 4));
		step_sizes.initial_step = this->getParameter<double>("co_initial_step", 0.02);
		step_sizes.min_step = this->getParameter<double>("co_min_step", 0.001);
		step_sizes.max_step = this->getParameter<double>("co_max_step", 0.05);
		step_sizes.initial_slide_step = this->getParameter<double>("co_initial_slide_step", 0.3);
		step_sizes.slide_joint_margin = this->getParameter<double>("co_slide_joint_margin", 0.05);
		return step_sizes;
	}

	// Exact and closest IK; object_placement's names, so one params file serves all methods.
	object_gtsp_co_optimize::IkSettings readIkSettings()
	{
		object_gtsp_co_optimize::IkSettings ik_settings;
		closest_ik::Options ik_options;
		ik_options.iters = static_cast<int>(this->getParameter<int64_t>("closest_ik_iters", 60));
		ik_options.margin = this->getParameter<double>("closest_ik_margin", 0.02);
		ik_options.rot_scale = this->getParameter<double>("rot_metric_scale", 0.1);
		ik_options.limit_sharpness = this->getParameter<double>("manipulability_limit_sharpness", 20.0);
		ik_options.ignore_roll = this->getParameter<bool>("ignore_camera_roll", false);
		ik_options.exact_margin = this->getParameter<double>("exact_ik_margin", 0.0);
		ik_settings.exact_options = ik_options;
		ik_settings.exact_options.exact_only = true;
		ik_settings.closest_options = ik_options;
		ik_settings.closest_options.try_exact_first = false;
		ik_settings.jacobian_damping = this->getParameter<double>("jacobian_damping", 1e-3);
		ik_settings.exact_finish_gap = this->getParameter<double>("exact_ik_finish_gap", 0.05);
		ik_settings.random_ik_seed = static_cast<int>(this->getParameter<int64_t>("random_seed", 42));
		ik_settings.ik_attempts = static_cast<int>(this->getParameter<int64_t>("co_ik_attempts", 10));
		ik_settings.closest_gap_slack = this->getParameter<double>("co_closest_gap_slack", 0.005);
		ik_settings.closest_ik_extra_starts = static_cast<int>(this->getParameter<int64_t>("co_closest_ik_extra_starts", 3));
		return ik_settings;
	}

	// Tour edge cost and the GTSP solve; object_placement's names.
	object_gtsp_co_optimize::TourSettings readTourSettings()
	{
		object_gtsp_co_optimize::TourSettings tour_settings;
		tour_settings.edge_weights.joint_distance = this->getParameter<double>("joint_distance_weight", 1.0);
		tour_settings.edge_weights.max_turn = this->getParameter<double>("max_joint_deviation_weight", 1.0);
		tour_settings.edge_weights.time = this->getParameter<double>("travel_time_weight", 0.0);
		tour_settings.two_opt_rounds = static_cast<int>(this->getParameter<int64_t>("gtsp_two_opt_rounds", 5));
		tour_settings.reconfig_threshold = this->getParameter<double>("reconfig_threshold", 1.0);
		tour_settings.gtsp_blocked_rounds = static_cast<int>(this->getParameter<int64_t>("co_gtsp_blocked_rounds", 2));
		return tour_settings;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectGtspCoOptimizeNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
