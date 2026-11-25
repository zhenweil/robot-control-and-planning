// Runs object_gtsp_co_optimize: reads parameters, runs the descent, plans the result and saves it.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
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
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/object_gtsp_co_optimize_setup.hpp"
#include "object_placement_planning/object_gtsp_co_optimizer.hpp"
#include "object_placement_planning/object_placement.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"
#include "object_placement_planning/viewpoint_types.hpp"

namespace
{

using Joints = std::vector<double>;
using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;

// Planned length or time per edge; negative if planning failed.
using EdgePlanner = std::function<std::vector<double>(const std::vector<Joints>& tour_poses, const Joints& start_pose, double planning_time, int planning_attempts)>;

// Final check of the tour, measured only.
struct PlannedTour
{
	double length = 0.0;  // rad
	double time = 0.0;  // s, for reference
	int num_failed_edges = 0;
};

}  // namespace

class ObjectGtspCoOptimizeNode : public rclcpp::Node
{
public:
	ObjectGtspCoOptimizeNode() : Node("object_gtsp_co_optimize", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		this->readParameters();
		const std::vector<Eigen::Isometry3d> viewpoints_at_nominal = LoadTourTcpPoses(this->tour_input_dir);  // abs
		std::vector<Eigen::Isometry3d> viewpoints = viewpoints_at_nominal;  // relative to the object position
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= this->nominal_position;
		const int num_viewpoints = static_cast<int>(viewpoints.size());

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		this->model = loader.getModel();
		this->scene = BuildLocalCollisionScene(this->shared_from_this(), this->mesh_path, this->mesh_scale, this->nominal_position, this->object_rotation);
		object_gtsp_co_optimize::AddMountingSurface(this->scene);
		const MarkerArrayPublisher marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_markers", rclcpp::QoS(1).transient_local());
		const MarkerArrayPublisher progress_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_progress_markers", rclcpp::QoS(1).transient_local());
		visualization_msgs::msg::MarkerArray surface_markers;
		surface_markers.markers.push_back(object_gtsp_co_optimize::GetMountingSurfaceMarker(this->model->getRootLinkName(), this->now()));
		marker_publisher->publish(surface_markers);

		const object_gtsp_co_optimize::Settings settings = object_gtsp_co_optimize::ReadSettings(*this);
		const object_gtsp_co_optimize::DescentLimits& limits = settings.descent_limits;
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; co-optimizing in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s", num_viewpoints, limits.search_box_lower_bound.x(), limits.search_box_upper_bound.x(), limits.search_box_lower_bound.y(), limits.search_box_upper_bound.y(), limits.search_box_lower_bound.z(), limits.search_box_upper_bound.z(), limits.time_budget);

		const auto start_time = std::chrono::steady_clock::now();
		auto get_elapsed_seconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
		const object_gtsp_co_optimize::ProgressDisplay progress_display{progress_publisher, this->mesh_path, this->mesh_scale, this->show_tool_path};
		object_gtsp_co_optimize::Optimizer optimizer(this->shared_from_this(), this->model, this->scene, this->group_name, viewpoints, this->object_rotation, this->home_pose, settings, progress_display);
		const object_gtsp_co_optimize::TourSolution solution = optimizer.runDescent(this->start_position);
		const double search_seconds = get_elapsed_seconds();
		const bool all_reached = solution.placement_ik.num_reached == num_viewpoints;
		const Eigen::Vector3d& object_position = solution.placement_ik.object_position;

		const std::vector<Joints> tour_poses = optimizer.getPosesInTourOrder(solution.placement_ik, solution.order);
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(object_position, this->object_rotation));
		const PlannedTour planned_tour = this->planTour(tour_poses, optimizer.getCountedJoints(), optimizer.getEdgeWeights());
		const double smallest_margin = optimizer.getSmallestJointMargin(tour_poses);
		const double seconds = get_elapsed_seconds();
		if (all_reached)
			RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned length %.2f rad (%d failed edges), %d reconfigurations, smallest joint margin %.3f rad beyond the hard limit; planned time %.2f s for reference (%.1f s).", object_position.x(), object_position.y(), object_position.z(), num_viewpoints, planned_tour.length, planned_tour.num_failed_edges, solution.num_reconfigurations, smallest_margin, planned_tour.time, seconds);
		else
			RCLCPP_WARN(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%.1f s).", object_position.x(), object_position.y(), object_position.z(), std::max(0, solution.placement_ik.num_reached), num_viewpoints, seconds);

		Json::Value result_json;
		result_json["ok"] = all_reached;
		result_json["num_reachable"] = std::max(0, solution.placement_ik.num_reached);
		result_json["num_total"] = num_viewpoints;
		result_json["x"] = object_position.x();
		result_json["y"] = object_position.y();
		result_json["z"] = object_position.z();
		result_json["planned_travel"] = all_reached ? planned_tour.length : -1.0;  // rad
		result_json["planned_time"] = all_reached ? planned_tour.time : -1.0;  // s
		result_json["straight_tour_cost"] = solution.tour_length;
		result_json["failed_edges"] = planned_tour.num_failed_edges;
		result_json["reconfigurations"] = solution.num_reconfigurations;
		result_json["smallest_joint_margin"] = smallest_margin;
		result_json["search_seconds"] = search_seconds;
		result_json["seconds"] = seconds;
		for (int viewpoint : solution.order)
			result_json["tour_order"].append(viewpoint);
		for (const Joints& pose : tour_poses)
		{
			Json::Value joints_json(Json::arrayValue);
			for (double value : pose)
				joints_json.append(value);
			result_json["joint_solutions"].append(joints_json);
		}
		std::filesystem::create_directories(this->output_dir);
		std::ofstream(this->output_dir + "/object_gtsp_co_optimize_result.json") << result_json;

		// Final markers, drawn as object_placement draws them.
		ObjectPlacementResult placement_result;
		placement_result.ok = all_reached;
		placement_result.x = object_position.x();
		placement_result.y = object_position.y();
		placement_result.z = object_position.z();
		placement_result.tour_order = solution.order;
		for (int viewpoint : solution.order)
			placement_result.joint_solutions.push_back(solution.placement_ik.is_reached[static_cast<size_t>(viewpoint)] ? solution.placement_ik.ik_solutions[static_cast<size_t>(viewpoint)].joints : Joints());
		visualization_msgs::msg::MarkerArray final_markers = BuildObjectPlacementMarkerArray(this->now(), this->mesh_path, this->mesh_scale, this->nominal_position, this->object_rotation, viewpoints_at_nominal, placement_result, this->show_tool_path);
		final_markers.markers.push_back(object_gtsp_co_optimize::GetMountingSurfaceMarker(this->model->getRootLinkName(), this->now()));
		marker_publisher->publish(final_markers);
		optimizer.publishProgressMarkers(solution.placement_ik, solution.order, Eigen::Vector3d::Zero());

		if (this->execute_on_robot && !all_reached)
			RCLCPP_WARN(this->get_logger(), "Not executing: the tour misses viewpoints.");
		else if (this->execute_on_robot)
			this->executeTour(solution, viewpoints, tour_poses, marker_publisher);
		while (this->keep_alive && rclcpp::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(2));
			marker_publisher->publish(final_markers);
		}
	}

private:
	template <typename T>
	T getParameter(const std::string& name, const T& fallback)
	{
		return object_gtsp_co_optimize::GetParameter<T>(*this, name, fallback);
	}

	void readParameters()
	{
		this->group_name = this->getParameter<std::string>("group_name", "panda_arm");
		this->mesh_path = this->getParameter<std::string>("mesh_path", "");
		if (this->mesh_path.empty())
			this->mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs_repaired_cm_binary.stl";
		this->mesh_scale = this->getParameter<double>("mesh_scale", 0.01);
		this->show_tool_path = this->getParameter<bool>("show_tour_lines", false);
		this->tour_input_dir = this->getParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		this->output_dir = this->getParameter<std::string>("output_dir", "/tmp/object_gtsp_co_optimize_output");
		this->home_pose = this->getParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		this->nominal_position = ToVector3(this->getParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		this->object_rotation = RotationFromRpyDeg(this->getParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		this->start_position = Eigen::Vector3d(this->getParameter<double>("initial_x", 0.5), this->getParameter<double>("initial_y", 0.0), this->getParameter<double>("initial_z", 0.15));
		this->planning_time = this->getParameter<double>("real_cost_planning_time", 0.5);
		this->planning_attempts = static_cast<int>(this->getParameter<int64_t>("real_cost_attempts", 3));
		this->unreachable_penalty = this->getParameter<double>("unreachable_penalty", 50.0);
		this->keep_alive = this->getParameter<bool>("keep_alive", false);  // keep republishing markers for RViz
		this->execute_on_robot = this->getParameter<bool>("execute_on_robot", false);  // needs move_group running
		this->execution_planning_time = this->getParameter<double>("execution_planning_time", 5.0);
		this->execution_planning_attempts = static_cast<int>(this->getParameter<int64_t>("execution_planning_attempts", 5));
	}

	// Plans every edge; a failed edge is retried with the execution budget.
	std::vector<double> planEdgesWithRetry(const std::vector<Joints>& tour_poses, const EdgePlanner& plan_edges)
	{
		std::vector<double> edge_values = plan_edges(tour_poses, this->home_pose, this->planning_time, this->planning_attempts);
		for (size_t edge_index = 0; edge_index < tour_poses.size() && edge_index < edge_values.size(); ++edge_index)
		{
			if (edge_values[edge_index] >= 0.0)
				continue;
			const Joints& start_pose = edge_index == 0 ? this->home_pose : tour_poses[edge_index - 1];
			const std::vector<double> retry = plan_edges({tour_poses[edge_index]}, start_pose, this->execution_planning_time, this->execution_planning_attempts);
			if (!retry.empty())
				edge_values[edge_index] = retry[0];
		}
		return edge_values;
	}

	// Planned length and time of the tour; a failed edge is penalized.
	PlannedTour planTour(const std::vector<Joints>& tour_poses, const std::vector<int>& counted_joints, const joint_travel::EdgeWeights& edge_weights)
	{
		const rclcpp::Node::SharedPtr node = this->shared_from_this();
		const EdgePlanner plan_lengths = [&](const std::vector<Joints>& poses, const Joints& start_pose, double time, int attempts) { return PlanTourEdgeLengths(node, this->model, this->scene, poses, start_pose, this->group_name, time, attempts, /*count_last_joint=*/false); };
		const EdgePlanner plan_times = [&](const std::vector<Joints>& poses, const Joints& start_pose, double time, int attempts) { return PlanTourEdgeTimes(node, this->model, this->scene, poses, start_pose, this->group_name, time, attempts); };
		const std::vector<double> edge_lengths = this->planEdgesWithRetry(tour_poses, plan_lengths);
		const std::vector<double> edge_times = this->planEdgesWithRetry(tour_poses, plan_times);
		PlannedTour planned_tour;
		for (size_t edge_index = 0; edge_index < tour_poses.size(); ++edge_index)
		{
			const Joints& start_pose = edge_index == 0 ? this->home_pose : tour_poses[edge_index - 1];
			const bool length_planned = edge_index < edge_lengths.size() && edge_lengths[edge_index] >= 0.0;
			planned_tour.num_failed_edges += length_planned ? 0 : 1;
			planned_tour.length += length_planned ? edge_lengths[edge_index] : joint_travel::JointDistance(start_pose, tour_poses[edge_index], counted_joints) + this->unreachable_penalty;
			const bool time_planned = edge_index < edge_times.size() && edge_times[edge_index] >= 0.0;
			planned_tour.time += time_planned ? edge_times[edge_index] : joint_travel::StraightMoveTime(start_pose, tour_poses[edge_index], edge_weights.max_velocity, edge_weights.max_acceleration) + this->unreachable_penalty;
		}
		return planned_tour;
	}

	// Drives the robot through the tour along the unwrapped poses.
	void executeTour(const object_gtsp_co_optimize::TourSolution& solution, const std::vector<Eigen::Isometry3d>& viewpoints, const std::vector<Joints>& tour_poses, const MarkerArrayPublisher& marker_publisher)
	{
		const Eigen::Vector3d& object_position = solution.placement_ik.object_position;
		RCLCPP_WARN(this->get_logger(), "execute_on_robot: driving the robot against the object at abs (%.4f, %.4f, %.4f) m -- on a real robot only correct if the object is actually fixtured there.", object_position.x(), object_position.y(), object_position.z());
		const moveit::planning_interface::MoveGroupInterfacePtr move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), this->group_name);
		move_group->startStateMonitor();
		move_group->setPlanningTime(this->execution_planning_time);
		move_group->setNumPlanningAttempts(this->execution_planning_attempts);
		move_group->setMaxVelocityScalingFactor(1.0);
		move_group->setMaxAccelerationScalingFactor(1.0);
		move_group->setEndEffectorLink("tool0");
		std::vector<ViewpointCandidate> tour_viewpoints(tour_poses.size());
		std::vector<const ViewpointCandidate*> selected_viewpoints;
		for (size_t tour_position = 0; tour_position < tour_poses.size(); ++tour_position)
		{
			tour_viewpoints[tour_position].tcp_pose = scene_geometry::ToPoseMsg(scene_geometry::Translation(object_position) * viewpoints[static_cast<size_t>(solution.order[tour_position])]);
			tour_viewpoints[tour_position].joint_solution = tour_poses[tour_position];
			tour_viewpoints[tour_position].reachable = true;
			selected_viewpoints.push_back(&tour_viewpoints[tour_position]);
		}
		ExecuteTourOnRobot(this->shared_from_this(), this->model, this->scene, move_group, marker_publisher, selected_viewpoints, this->home_pose, this->group_name, this->execution_planning_time, this->execution_planning_attempts);
	}

	std::string group_name;
	std::string mesh_path;
	double mesh_scale = 0.01;
	bool show_tool_path = false;
	std::string tour_input_dir;
	std::string output_dir;
	Joints home_pose;
	Eigen::Vector3d nominal_position = Eigen::Vector3d::Zero();  // abs; the viewpoint files assume the object here
	Eigen::Matrix3d object_rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d start_position = Eigen::Vector3d::Zero();  // abs
	double planning_time = 0.5;  // s per OMPL attempt
	int planning_attempts = 3;
	double unreachable_penalty = 50.0;  // rad (s for time) per failed edge
	bool keep_alive = false;
	bool execute_on_robot = false;
	double execution_planning_time = 5.0;
	int execution_planning_attempts = 5;
	moveit::core::RobotModelConstPtr model;
	planning_scene_monitor::PlanningSceneMonitorPtr scene;
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
