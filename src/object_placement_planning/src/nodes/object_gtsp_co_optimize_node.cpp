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

	void Run()
	{
		const std::string group_name = this->GetParameter<std::string>("group_name", "panda_arm");
		std::string mesh_path = this->GetParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->GetParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->GetParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = this->GetParameter<std::string>("output_dir", "/tmp/object_gtsp_co_optimize_output");
		const Joints home = this->GetParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal = ToVector3(this->GetParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d rotation = RotationFromRpyDeg(this->GetParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		const Eigen::Vector3d start(this->GetParameter<double>("initial_x", 0.5), this->GetParameter<double>("initial_y", 0.0), this->GetParameter<double>("initial_z", 0.15));
		const double planning_time = this->GetParameter<double>("real_cost_planning_time", 0.5);
		const int planning_attempts = static_cast<int>(this->GetParameter<int64_t>("real_cost_attempts", 3));
		const double unreachable_penalty = this->GetParameter<double>("unreachable_penalty", 50.0);
		const bool keep_alive = this->GetParameter<bool>("keep_alive", false);  // keep republishing the final markers until Ctrl-C (for RViz)
		object_gtsp_co_optimize::Settings settings = this->ReadSettings();
		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs.stl";
		settings.mesh_path = mesh_path;
		settings.mesh_scale = mesh_scale;

		const std::vector<Eigen::Isometry3d> viewpoints_nominal = LoadTourTcpPoses(tour_input_dir);	// abs, object at nominal
		std::vector<Eigen::Isometry3d> viewpoints = viewpoints_nominal;  // relative to the object position
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= nominal;
		const int num_viewpoint = static_cast<int>(viewpoints.size());

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		const planning_scene_monitor::PlanningSceneMonitorPtr scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, rotation);
		AddMountingSurface(scene);
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_markers", rclcpp::QoS(1).transient_local());
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_progress_markers", rclcpp::QoS(1).transient_local());
		visualization_msgs::msg::MarkerArray surface_markers;
		surface_markers.markers.push_back(GetMountingSurfaceMarker(model->getRootLinkName(), this->now()));
		marker_pub->publish(surface_markers);
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; co-optimizing in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s", num_viewpoint, settings.lower_bound.x(), settings.upper_bound.x(), settings.lower_bound.y(), settings.upper_bound.y(), settings.lower_bound.z(), settings.upper_bound.z(), settings.time_budget);

		const auto start_time = std::chrono::steady_clock::now();
		auto GetElapsedSeconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
		object_gtsp_co_optimize::Optimizer optimizer(this->shared_from_this(), model, scene, group_name, viewpoints, rotation, home, settings, progress_pub);
		// One descent from the given start, no restarts.
		const object_gtsp_co_optimize::Descent best = optimizer.Descend(start);
		const double search_seconds = GetElapsedSeconds();
		const bool all_reached = best.placement.num_reached == num_viewpoint;

		// Planned tour length and, for reference, execution time: straight moves where collision-free, else OMPL (camera spin
		// aside in the length; timed with TOTG for the time).
		const std::vector<Joints> tour_poses = optimizer.GetTourPoses(best.placement, best.order);
		const std::vector<int>& travel_joints = optimizer.GetTravelJoints();
		const joint_travel::EdgeWeights& edge_weights = optimizer.GetEdgeWeights();
		scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(best.placement.position, rotation));
		const std::vector<double> edge_lengths = PlanTourEdgeLengths(this->shared_from_this(), model, scene, tour_poses, home, group_name, planning_time, planning_attempts, /*count_last_joint=*/false);
		double planned_length = 0.0;
		int num_failed_length_edges = 0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_lengths.size() && edge_lengths[k] >= 0.0;
			num_failed_length_edges += edge_ok ? 0 : 1;
			planned_length += edge_ok ? edge_lengths[k] : joint_travel::JointDistance(k == 0 ? home : tour_poses[k - 1], tour_poses[k], travel_joints) + unreachable_penalty;
		}
		const std::vector<double> edge_times = PlanTourEdgeTimes(this->shared_from_this(), model, scene, tour_poses, home, group_name, planning_time, planning_attempts);
		double planned_time = 0.0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_times.size() && edge_times[k] >= 0.0;
			planned_time += edge_ok ? edge_times[k] : joint_travel::StraightMoveTime(k == 0 ? home : tour_poses[k - 1], tour_poses[k], edge_weights.max_velocity, edge_weights.max_acceleration) + unreachable_penalty;
		}
		std::vector<Joints> poses_from_home = tour_poses;
		poses_from_home.insert(poses_from_home.begin(), home);
		const int num_reconfigurations = joint_travel::CountReconfigs(poses_from_home, settings.reconfig_threshold, travel_joints);
		const double smallest_margin = optimizer.GetSmallestJointMargin(tour_poses);
		const double seconds = GetElapsedSeconds();
		if (all_reached)
			RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned length %.2f rad (%d failed edges), %d reconfigurations, smallest joint margin %.3f rad beyond the hard limit; planned time %.2f s for reference (%.1f s).", best.placement.position.x(), best.placement.position.y(), best.placement.position.z(), num_viewpoint, planned_length, num_failed_length_edges, num_reconfigurations, smallest_margin, planned_time, seconds);
		else
			RCLCPP_WARN(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%.1f s).", best.placement.position.x(), best.placement.position.y(), best.placement.position.z(), std::max(0, best.placement.num_reached), num_viewpoint, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = std::max(0, best.placement.num_reached);
		out["num_total"] = num_viewpoint;
		out["x"] = best.placement.position.x();
		out["y"] = best.placement.position.y();
		out["z"] = best.placement.position.z();
		out["planned_travel"] = all_reached ? planned_length : -1.0;  // rad
		out["planned_time"] = all_reached ? planned_time : -1.0;  // s, reference only
		out["straight_tour_cost"] = best.tour_cost;
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
		result.x = best.placement.position.x();
		result.y = best.placement.position.y();
		result.z = best.placement.position.z();
		result.tour_order = best.order;
		for (int viewpoint : best.order)
			result.joint_solutions.push_back(best.placement.reached[static_cast<size_t>(viewpoint)] ? best.placement.poses[static_cast<size_t>(viewpoint)] : Joints());
		visualization_msgs::msg::MarkerArray final_markers = BuildObjectPlacementMarkerArray(this->now(), mesh_path, mesh_scale, nominal, rotation, viewpoints_nominal, result);
		final_markers.markers.push_back(GetMountingSurfaceMarker(model->getRootLinkName(), this->now()));
		marker_pub->publish(final_markers);
		optimizer.PublishProgress(best.placement, best.order, Eigen::Vector3d::Zero());
		while (keep_alive && rclcpp::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(2));
			marker_pub->publish(final_markers);
		}
	}

private:
	template <typename T>
	T GetParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}

	// The optimizer's tunable parameters; object_placement's names where shared, so one params file serves all methods.
	object_gtsp_co_optimize::Settings ReadSettings()
	{
		object_gtsp_co_optimize::Settings settings;
		settings.lower_bound = Eigen::Vector3d(this->GetParameter<double>("x_min", -0.6), this->GetParameter<double>("y_min", -0.6), this->GetParameter<double>("z_min", -0.2));
		settings.upper_bound = Eigen::Vector3d(this->GetParameter<double>("x_max", 0.6), this->GetParameter<double>("y_max", 0.6), this->GetParameter<double>("z_max", 0.2));
		settings.seed = static_cast<int>(this->GetParameter<int64_t>("random_seed", 42));
		closest_ik::Options ik_options;
		ik_options.iters = static_cast<int>(this->GetParameter<int64_t>("closest_ik_iters", 60));
		ik_options.margin = this->GetParameter<double>("closest_ik_margin", 0.02);
		ik_options.rot_scale = this->GetParameter<double>("rot_metric_scale", 0.1);
		ik_options.limit_sharpness = this->GetParameter<double>("manipulability_limit_sharpness", 20.0);
		ik_options.ignore_roll = this->GetParameter<bool>("ignore_camera_roll", false);
		ik_options.exact_margin = this->GetParameter<double>("exact_ik_margin", 0.0);
		settings.exact_options = ik_options;
		settings.exact_options.exact_only = true;
		settings.closest_options = ik_options;
		settings.closest_options.try_exact_first = false;
		settings.jacobian_damping = this->GetParameter<double>("jacobian_damping", 1e-3);
		settings.edge_weights.joint_distance = this->GetParameter<double>("joint_distance_weight", 1.0);
		settings.edge_weights.max_turn = this->GetParameter<double>("max_joint_deviation_weight", 1.0);
		settings.edge_weights.time = this->GetParameter<double>("travel_time_weight", 0.0);
		settings.two_opt_rounds = static_cast<int>(this->GetParameter<int64_t>("gtsp_two_opt_rounds", 5));
		settings.reconfig_threshold = this->GetParameter<double>("reconfig_threshold", 1.0);
		settings.exact_finish_gap = this->GetParameter<double>("exact_ik_finish_gap", 0.05);
		settings.time_budget = this->GetParameter<double>("co_time_budget_sec", 300.0);
		settings.max_steps = static_cast<int>(this->GetParameter<int64_t>("co_max_steps", 200));
		settings.max_line_search_tries = static_cast<int>(this->GetParameter<int64_t>("co_max_line_search_tries", 4));
		settings.ik_attempts = static_cast<int>(this->GetParameter<int64_t>("co_ik_attempts", 10));
		settings.closest_gap_slack = this->GetParameter<double>("co_closest_gap_slack", 0.005);
		settings.closest_ik_extra_starts = static_cast<int>(this->GetParameter<int64_t>("co_closest_ik_extra_starts", 3));
		settings.initial_step = this->GetParameter<double>("co_initial_step", 0.02);
		settings.min_step = this->GetParameter<double>("co_min_step", 0.001);
		settings.max_step = this->GetParameter<double>("co_max_step", 0.05);
		settings.initial_penalty = this->GetParameter<double>("co_initial_penalty", 100.0);
		settings.penalty_growth = this->GetParameter<double>("co_penalty_growth", 10.0);
		settings.max_penalty = this->GetParameter<double>("co_max_penalty", 1e5);
		settings.miss_cost = this->GetParameter<double>("co_miss_cost", 10.0);
		settings.use_room = this->GetParameter<bool>("co_room", true);
		settings.room_in_reach_direction = this->GetParameter<bool>("co_room_in_reach_direction", false);
		settings.use_neighbor_ik = this->GetParameter<bool>("co_neighbor_ik", true);
		settings.use_posture_match = this->GetParameter<bool>("co_posture_match", true);
		settings.reconfiguration_cost = this->GetParameter<double>("co_reconfiguration_cost", 2.0);
		settings.reconfiguration_weight = this->GetParameter<double>("co_reconfiguration_weight", 10.0);
		settings.blocked_edge_charge = this->GetParameter<double>("co_blocked_edge_charge", 4.0);
		settings.gtsp_blocked_rounds = static_cast<int>(this->GetParameter<int64_t>("co_gtsp_blocked_rounds", 2));
		settings.joint_room_band = this->GetParameter<double>("co_joint_room_band", 0.2);
		settings.joint_room_weight = this->GetParameter<double>("co_joint_room_weight", 50.0);
		settings.use_slide = this->GetParameter<bool>("co_slide", true);
		settings.initial_slide_step = this->GetParameter<double>("co_initial_slide_step", 0.3);
		settings.slide_joint_margin = this->GetParameter<double>("co_slide_joint_margin", 0.05);
		settings.tour_object_move = this->GetParameter<bool>("co_tour_object_move", true);
		settings.freeze_object_when_reached = this->GetParameter<bool>("co_freeze_object_when_reached", false);
		settings.progress_delay = this->GetParameter<double>("visualize_progress_delay_sec", 0.0);
		return settings;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectGtspCoOptimizeNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->Run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
