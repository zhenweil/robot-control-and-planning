#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <ompl/util/RandomNumbers.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/object_placement.hpp"
#include "panda_arm_control/object_placement_optimizer.hpp"
#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_io.hpp"
#include "panda_arm_control/viewpoint_types.hpp"

namespace
{

struct Params
{
	std::string mesh_path;
	double mesh_scale = 0.01;
	std::string group_name = "panda_arm";
	std::vector<double> object_translation_world = {0.2, 0.2, 0.38};
	std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
	std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};
	// Directory a prior viewpoint_planner_* run exported selected_robot_poses.json into.
	std::string tour_input_dir = "/tmp/viewpoint_planner_output";
	std::string output_dir = "/tmp/object_placement_output";

	// Object position bounds and start, abs (m, base frame).
	double x_min = -0.1, x_max = 1.1;
	double y_min = -0.6, y_max = 0.6;
	double z_min = -0.05, z_max = 0.35;
	double initial_x = 0.5, initial_y = 0.0, initial_z = 0.15;
	double rot_metric_scale = 0.3;  // m per rad: weighs a missed viewpoint's angle gap against its distance gap

	double joint_distance_weight = 1.0;
	double max_joint_deviation_weight = 1.0;
	double miss_gap_weight = 5000.0;
	double manipulability_limit_sharpness = 0.0;
	double manipulability_weight = 0.0;
	double softmin_manipulability_weight = 0.0;
	double softmin_tau = 0.01;
	double miss_gap_cap = 0.15;
	double closest_ik_margin = 0.02;
	int closest_ik_iters = 60;
	int closest_ik_starts = 5;
	double unreachable_penalty = 50.0;

	int max_solutions_per_candidate = 4;
	int ik_retries_per_point = 14;
	int gtsp_two_opt_rounds = 5;
	int gtsp_num_restart = 2;  // min-of-N inner solves wherever a committed cost matters

	double jump_softmin_weight = 100.0;
	int jump_max_moves = 10;
	int reach_patience = 3;
	int room_max_targets = 3;
	int descent_num_restart = 6;
	int max_kinematic_moves = 20;
	double initial_step = 0.02;
	double step_shrink = 0.25;
	int max_line_search_iters = 4;
	double jacobian_damping = 1e-3;

	double convergence_tolerance_offset = 0.002;
	double convergence_tolerance_cost = 1e-3;
	int patience = 3;
	bool stop_when_all_reached = false;
	int gtsp_restarts = 0;
	int gtsp_restart_ik_solutions = 8;
	int tour_placement_iterations = 0;
	int tour_placement_rounds = 1;
	double tour_placement_object_step = 0.02;
	double tour_placement_self_motion_step = 0.3;
	double tour_placement_joint_margin = 0.05;
	double reconfig_threshold = 1.0;
	bool travel_gradient = true;
	bool freeze_order = false;
	bool lock_input_order = false;
	double real_cost_planning_time = 0.0;
	int real_cost_attempts = 3;
	bool planned_travel_in_cost = false;
	bool room_then_reach = false;
	double reach_time_limit = 0.0;
	bool trace_line_search = false;
	int gtsp_restarts_at_reach = 0;

	bool fd_gradient_check = false;
	double fd_epsilon = 1e-4;

	double ik_timeout = 0.15;
	int ik_attempts = 0;
	int random_seed = 42;
	double visualize_progress_delay_sec = 0.0;

	// Drives the real robot through the recommended placement's tour by moving the (software-only)
	// collision object into the frame the base would see at the recommended offset. Only correct
	// if that matches reality -- see ApplyObjectPlacementToScene's doc comment. Defaults to false.
	bool execute_on_robot = false;
	double execution_planning_time = 5.0;
	int execution_planning_attempts = 5;
};

// The surface the robot is mounted on: a wide slab just under the base, so no arm pose (IK or closest IK) can reach
// below it. The base rests on it, so those pairs are allowed to touch.
constexpr double kSurfaceSize = 4.0;       // m
constexpr double kSurfaceThickness = 0.05;  // m

void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = kSurfaceSize, thickness = kSurfaceThickness;
	planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
	const std::string base_link = locked_scene->getRobotModel()->getRootLinkName();
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = base_link;
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {size, size, thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * thickness;  // top face at the base plane
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	locked_scene->processCollisionObjectMsg(surface);
	// The base rests on it: allow every link fixed to the root (a URDF's world link has no geometry of its own).
	for (const moveit::core::LinkModel* link : locked_scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == locked_scene->getRobotModel()->getRootLink())
			locked_scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// RViz marker for the mounting surface, in the robot's base frame.
visualization_msgs::msg::Marker MountingSurfaceMarker(const std::string& base_link, const rclcpp::Time& stamp)
{
	visualization_msgs::msg::Marker m;
	m.header.frame_id = base_link;
	m.header.stamp = stamp;
	m.ns = "mounting_surface";
	m.id = 0;
	m.type = visualization_msgs::msg::Marker::CUBE;
	m.action = visualization_msgs::msg::Marker::ADD;
	m.pose.position.z = -0.5 * kSurfaceThickness;
	m.pose.orientation.w = 1.0;
	m.scale.x = kSurfaceSize;
	m.scale.y = kSurfaceSize;
	m.scale.z = kSurfaceThickness;
	m.color.r = 0.55f;
	m.color.g = 0.45f;
	m.color.b = 0.35f;
	m.color.a = 0.6f;
	return m;
}

}  // namespace

class ObjectPlacementNode : public rclcpp::Node
{
public:
	explicit ObjectPlacementNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("object_placement", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
	{
	}

	void init()
	{
		this->declareToolParameters();
		this->loadParameters();

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		this->robot_model = loader.getModel();
		this->robot_state = std::make_shared<moveit::core::RobotState>(this->robot_model);
		this->jmg = this->robot_state->getJointModelGroup(this->params.group_name);

		this->robot_state->setToDefaultValues();
		this->robot_state->setJointGroupPositions(this->jmg, this->params.initial_joints);
		this->robot_state->update();
		this->robot_state->copyJointGroupPositions(this->jmg, this->home_joint_values);

		this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/object_placement_markers", rclcpp::QoS(1).transient_local());
		this->progress_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/object_placement_progress_markers", rclcpp::QoS(1).transient_local());

		if (this->params.execute_on_robot)
		{
			this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
				this->shared_from_this(), this->params.group_name);
			this->move_group->startStateMonitor();
			this->move_group->setPlanningTime(this->params.execution_planning_time);
			this->move_group->setNumPlanningAttempts(this->params.execution_planning_attempts);
			this->move_group->setMaxVelocityScalingFactor(1.0);
			this->move_group->setMaxAccelerationScalingFactor(1.0);
			this->move_group->setEndEffectorLink("tool0");
		}

		this->runPipeline();

		if (!rclcpp::ok())  // shut down mid-run (Ctrl-C)
			return;

		this->marker_timer = this->create_wall_timer(
			std::chrono::seconds(2), [this]() { this->marker_pub->publish(this->marker_array); });
	}

private:
	Params params;

	moveit::core::RobotModelPtr robot_model;
	moveit::core::RobotStatePtr robot_state;
	const moveit::core::JointModelGroup* jmg = nullptr;
	std::vector<double> home_joint_values;
	std::string resolved_mesh_path;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_marker_pub;
	rclcpp::TimerBase::SharedPtr marker_timer;
	visualization_msgs::msg::MarkerArray marker_array;

	moveit::planning_interface::MoveGroupInterfacePtr move_group;

	template <typename T>
	void declareIfNeeded(const std::string& name, const T& default_value)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, default_value);
	}

	void declareToolParameters()
	{
		this->declareIfNeeded("mesh_path", this->params.mesh_path);
		this->declareIfNeeded("mesh_scale", this->params.mesh_scale);
		this->declareIfNeeded("group_name", this->params.group_name);
		this->declareIfNeeded("object_translation_world", this->params.object_translation_world);
		this->declareIfNeeded("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->declareIfNeeded("initial_joints", this->params.initial_joints);
		this->declareIfNeeded("tour_input_dir", this->params.tour_input_dir);
		this->declareIfNeeded("output_dir", this->params.output_dir);
		this->declareIfNeeded("x_min", this->params.x_min);
		this->declareIfNeeded("x_max", this->params.x_max);
		this->declareIfNeeded("y_min", this->params.y_min);
		this->declareIfNeeded("y_max", this->params.y_max);
		this->declareIfNeeded("z_min", this->params.z_min);
		this->declareIfNeeded("z_max", this->params.z_max);
		this->declareIfNeeded("initial_x", this->params.initial_x);
		this->declareIfNeeded("initial_y", this->params.initial_y);
		this->declareIfNeeded("initial_z", this->params.initial_z);
		this->declareIfNeeded("rot_metric_scale", this->params.rot_metric_scale);
		this->declareIfNeeded("joint_distance_weight", this->params.joint_distance_weight);
		this->declareIfNeeded("max_joint_deviation_weight", this->params.max_joint_deviation_weight);
		this->declareIfNeeded("miss_gap_weight", this->params.miss_gap_weight);
		this->declareIfNeeded("manipulability_limit_sharpness", this->params.manipulability_limit_sharpness);
		this->declareIfNeeded("manipulability_weight", this->params.manipulability_weight);
		this->declareIfNeeded("softmin_manipulability_weight", this->params.softmin_manipulability_weight);
		this->declareIfNeeded("softmin_tau", this->params.softmin_tau);
		this->declareIfNeeded("miss_gap_cap", this->params.miss_gap_cap);
		this->declareIfNeeded("closest_ik_margin", this->params.closest_ik_margin);
		this->declareIfNeeded("closest_ik_iters", this->params.closest_ik_iters);
		this->declareIfNeeded("closest_ik_starts", this->params.closest_ik_starts);
		this->declareIfNeeded("unreachable_penalty", this->params.unreachable_penalty);
		this->declareIfNeeded("max_solutions_per_candidate", this->params.max_solutions_per_candidate);
		this->declareIfNeeded("ik_retries_per_point", this->params.ik_retries_per_point);
		this->declareIfNeeded("gtsp_num_restart", this->params.gtsp_num_restart);
		this->declareIfNeeded("gtsp_two_opt_rounds", this->params.gtsp_two_opt_rounds);
		this->declareIfNeeded("jump_softmin_weight", this->params.jump_softmin_weight);
		this->declareIfNeeded("jump_max_moves", this->params.jump_max_moves);
		this->declareIfNeeded("reach_patience", this->params.reach_patience);
		this->declareIfNeeded("room_max_targets", this->params.room_max_targets);
		this->declareIfNeeded("descent_num_restart", this->params.descent_num_restart);
		this->declareIfNeeded("max_kinematic_moves", this->params.max_kinematic_moves);
		this->declareIfNeeded("initial_step", this->params.initial_step);
		this->declareIfNeeded("step_shrink", this->params.step_shrink);
		this->declareIfNeeded("max_line_search_iters", this->params.max_line_search_iters);
		this->declareIfNeeded("jacobian_damping", this->params.jacobian_damping);
		this->declareIfNeeded("convergence_tolerance_offset", this->params.convergence_tolerance_offset);
		this->declareIfNeeded("convergence_tolerance_cost", this->params.convergence_tolerance_cost);
		this->declareIfNeeded("patience", this->params.patience);
		this->declareIfNeeded("stop_when_all_reached", this->params.stop_when_all_reached);
		this->declareIfNeeded("gtsp_restarts", this->params.gtsp_restarts);
		this->declareIfNeeded("gtsp_restart_ik_solutions", this->params.gtsp_restart_ik_solutions);
		this->declareIfNeeded("tour_placement_iterations", this->params.tour_placement_iterations);
		this->declareIfNeeded("tour_placement_rounds", this->params.tour_placement_rounds);
		this->declareIfNeeded("tour_placement_object_step", this->params.tour_placement_object_step);
		this->declareIfNeeded("tour_placement_self_motion_step", this->params.tour_placement_self_motion_step);
		this->declareIfNeeded("tour_placement_joint_margin", this->params.tour_placement_joint_margin);
		this->declareIfNeeded("reconfig_threshold", this->params.reconfig_threshold);
		this->declareIfNeeded("travel_gradient", this->params.travel_gradient);
		this->declareIfNeeded("freeze_order", this->params.freeze_order);
		this->declareIfNeeded("lock_input_order", this->params.lock_input_order);
		this->declareIfNeeded("real_cost_planning_time", this->params.real_cost_planning_time);
		this->declareIfNeeded("real_cost_attempts", this->params.real_cost_attempts);
		this->declareIfNeeded("planned_travel_in_cost", this->params.planned_travel_in_cost);
		this->declareIfNeeded("room_then_reach", this->params.room_then_reach);
		this->declareIfNeeded("reach_time_limit", this->params.reach_time_limit);
		this->declareIfNeeded("trace_line_search", this->params.trace_line_search);
		this->declareIfNeeded("gtsp_restarts_at_reach", this->params.gtsp_restarts_at_reach);
		this->declareIfNeeded("fd_gradient_check", this->params.fd_gradient_check);
		this->declareIfNeeded("fd_epsilon", this->params.fd_epsilon);
		this->declareIfNeeded("ik_timeout", this->params.ik_timeout);
		this->declareIfNeeded("ik_attempts", this->params.ik_attempts);
		this->declareIfNeeded("random_seed", this->params.random_seed);
		this->declareIfNeeded("visualize_progress_delay_sec", this->params.visualize_progress_delay_sec);
		this->declareIfNeeded("execute_on_robot", this->params.execute_on_robot);
		this->declareIfNeeded("execution_planning_time", this->params.execution_planning_time);
		this->declareIfNeeded("execution_planning_attempts", this->params.execution_planning_attempts);
	}

	void loadParameters()
	{
		this->get_parameter("mesh_path", this->params.mesh_path);
		this->get_parameter("mesh_scale", this->params.mesh_scale);
		this->get_parameter("group_name", this->params.group_name);
		this->get_parameter("object_translation_world", this->params.object_translation_world);
		this->get_parameter("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->get_parameter("initial_joints", this->params.initial_joints);
		this->get_parameter("tour_input_dir", this->params.tour_input_dir);
		this->get_parameter("output_dir", this->params.output_dir);
		this->get_parameter("x_min", this->params.x_min);
		this->get_parameter("x_max", this->params.x_max);
		this->get_parameter("y_min", this->params.y_min);
		this->get_parameter("y_max", this->params.y_max);
		this->get_parameter("z_min", this->params.z_min);
		this->get_parameter("z_max", this->params.z_max);
		this->get_parameter("initial_x", this->params.initial_x);
		this->get_parameter("initial_y", this->params.initial_y);
		this->get_parameter("initial_z", this->params.initial_z);
		this->get_parameter("rot_metric_scale", this->params.rot_metric_scale);
		this->get_parameter("joint_distance_weight", this->params.joint_distance_weight);
		this->get_parameter("max_joint_deviation_weight", this->params.max_joint_deviation_weight);
		this->get_parameter("miss_gap_weight", this->params.miss_gap_weight);
		this->get_parameter("manipulability_limit_sharpness", this->params.manipulability_limit_sharpness);
		this->get_parameter("manipulability_weight", this->params.manipulability_weight);
		this->get_parameter("softmin_manipulability_weight", this->params.softmin_manipulability_weight);
		this->get_parameter("softmin_tau", this->params.softmin_tau);
		this->get_parameter("miss_gap_cap", this->params.miss_gap_cap);
		this->get_parameter("closest_ik_margin", this->params.closest_ik_margin);
		this->get_parameter("closest_ik_iters", this->params.closest_ik_iters);
		this->get_parameter("closest_ik_starts", this->params.closest_ik_starts);
		this->get_parameter("unreachable_penalty", this->params.unreachable_penalty);
		this->get_parameter("max_solutions_per_candidate", this->params.max_solutions_per_candidate);
		this->get_parameter("ik_retries_per_point", this->params.ik_retries_per_point);
		this->get_parameter("gtsp_num_restart", this->params.gtsp_num_restart);
		this->get_parameter("gtsp_two_opt_rounds", this->params.gtsp_two_opt_rounds);
		this->get_parameter("jump_softmin_weight", this->params.jump_softmin_weight);
		this->get_parameter("jump_max_moves", this->params.jump_max_moves);
		this->get_parameter("reach_patience", this->params.reach_patience);
		this->get_parameter("room_max_targets", this->params.room_max_targets);
		this->get_parameter("descent_num_restart", this->params.descent_num_restart);
		this->get_parameter("max_kinematic_moves", this->params.max_kinematic_moves);
		this->get_parameter("initial_step", this->params.initial_step);
		this->get_parameter("step_shrink", this->params.step_shrink);
		this->get_parameter("max_line_search_iters", this->params.max_line_search_iters);
		this->get_parameter("jacobian_damping", this->params.jacobian_damping);
		this->get_parameter("convergence_tolerance_offset", this->params.convergence_tolerance_offset);
		this->get_parameter("convergence_tolerance_cost", this->params.convergence_tolerance_cost);
		this->get_parameter("patience", this->params.patience);
		this->get_parameter("stop_when_all_reached", this->params.stop_when_all_reached);
		this->get_parameter("gtsp_restarts", this->params.gtsp_restarts);
		this->get_parameter("gtsp_restart_ik_solutions", this->params.gtsp_restart_ik_solutions);
		this->get_parameter("tour_placement_iterations", this->params.tour_placement_iterations);
		this->get_parameter("tour_placement_rounds", this->params.tour_placement_rounds);
		this->get_parameter("tour_placement_object_step", this->params.tour_placement_object_step);
		this->get_parameter("tour_placement_self_motion_step", this->params.tour_placement_self_motion_step);
		this->get_parameter("tour_placement_joint_margin", this->params.tour_placement_joint_margin);
		this->get_parameter("reconfig_threshold", this->params.reconfig_threshold);
		this->get_parameter("travel_gradient", this->params.travel_gradient);
		this->get_parameter("freeze_order", this->params.freeze_order);
		this->get_parameter("lock_input_order", this->params.lock_input_order);
		this->get_parameter("real_cost_planning_time", this->params.real_cost_planning_time);
		this->get_parameter("real_cost_attempts", this->params.real_cost_attempts);
		this->get_parameter("planned_travel_in_cost", this->params.planned_travel_in_cost);
		this->get_parameter("room_then_reach", this->params.room_then_reach);
		this->get_parameter("reach_time_limit", this->params.reach_time_limit);
		this->get_parameter("trace_line_search", this->params.trace_line_search);
		this->get_parameter("gtsp_restarts_at_reach", this->params.gtsp_restarts_at_reach);
		this->get_parameter("fd_gradient_check", this->params.fd_gradient_check);
		this->get_parameter("fd_epsilon", this->params.fd_epsilon);
		this->get_parameter("ik_timeout", this->params.ik_timeout);
		this->get_parameter("ik_attempts", this->params.ik_attempts);
		this->get_parameter("random_seed", this->params.random_seed);
		this->get_parameter("visualize_progress_delay_sec", this->params.visualize_progress_delay_sec);
		this->get_parameter("execute_on_robot", this->params.execute_on_robot);
		this->get_parameter("execution_planning_time", this->params.execution_planning_time);
		this->get_parameter("execution_planning_attempts", this->params.execution_planning_attempts);
	}

	void runPipeline()
	{
		std::string mesh_path = this->params.mesh_path;
		if (mesh_path.empty())
			mesh_path =
				ament_index_cpp::get_package_share_directory("panda_arm_control") + "/meshes/bunny_holding_eggs.stl";
		this->resolved_mesh_path = mesh_path;

		std::vector<Eigen::Isometry3d> tour_tcp_poses = LoadTourTcpPoses(this->params.tour_input_dir);
		RCLCPP_INFO(
			this->get_logger(), "Loaded %zu tour poses from %s/selected_robot_poses.json", tour_tcp_poses.size(),
			this->params.tour_input_dir.c_str());

		if (tour_tcp_poses.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No tour poses loaded -- nothing to optimize a base for");
			return;
		}

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);

		planning_scene_monitor::PlanningSceneMonitorPtr local_scene = BuildLocalCollisionScene(
			this->shared_from_this(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world);
		AddMountingSurface(local_scene);
		visualization_msgs::msg::MarkerArray surface_markers;
		surface_markers.markers.push_back(MountingSurfaceMarker(this->robot_model->getRootLinkName(), this->now()));
		this->marker_pub->publish(surface_markers);

		ObjectPlacementParams bg;
		bg.bounds.x_min = this->params.x_min;
		bg.bounds.x_max = this->params.x_max;
		bg.bounds.y_min = this->params.y_min;
		bg.bounds.y_max = this->params.y_max;
		bg.bounds.z_min = this->params.z_min;
		bg.bounds.z_max = this->params.z_max;
		bg.initial_x = this->params.initial_x;
		bg.initial_y = this->params.initial_y;
		bg.initial_z = this->params.initial_z;
		bg.rot_metric_scale = this->params.rot_metric_scale;
		bg.edge_weights.joint_distance = this->params.joint_distance_weight;
		bg.edge_weights.max_turn = this->params.max_joint_deviation_weight;
		bg.miss_gap_weight = this->params.miss_gap_weight;
		bg.manipulability_limit_sharpness = this->params.manipulability_limit_sharpness;
		bg.manipulability_weight = this->params.manipulability_weight;
		bg.softmin_manipulability_weight = this->params.softmin_manipulability_weight;
		bg.softmin_tau = this->params.softmin_tau;
		bg.miss_gap_cap = this->params.miss_gap_cap;
		bg.closest_ik_margin = this->params.closest_ik_margin;
		bg.closest_ik_iters = this->params.closest_ik_iters;
		bg.closest_ik_starts = this->params.closest_ik_starts;
		bg.unreachable_penalty = this->params.unreachable_penalty;
		bg.max_solutions_per_candidate = this->params.max_solutions_per_candidate;
		bg.ik_timeout = this->params.ik_timeout;
		bg.ik_attempts = this->params.ik_attempts;
		bg.ik_retries_per_point = this->params.ik_retries_per_point;
		bg.gtsp_num_restart = this->params.gtsp_num_restart;
		bg.gtsp_two_opt_rounds = this->params.gtsp_two_opt_rounds;
		bg.jump_softmin_weight = this->params.jump_softmin_weight;
		bg.jump_max_moves = this->params.jump_max_moves;
		bg.reach_patience = this->params.reach_patience;
		bg.room_max_targets = this->params.room_max_targets;
		bg.descent_num_restart = this->params.descent_num_restart;
		bg.max_kinematic_moves = this->params.max_kinematic_moves;
		bg.initial_step = this->params.initial_step;
		bg.step_shrink = this->params.step_shrink;
		bg.max_line_search_iters = this->params.max_line_search_iters;
		bg.jacobian_damping = this->params.jacobian_damping;
		bg.convergence_tolerance_offset = this->params.convergence_tolerance_offset;
		bg.convergence_tolerance_cost = this->params.convergence_tolerance_cost;
		bg.patience = this->params.patience;
		bg.stop_when_all_reached = this->params.stop_when_all_reached;
		bg.gtsp_restarts = this->params.gtsp_restarts;
		bg.gtsp_restart_ik_solutions = this->params.gtsp_restart_ik_solutions;
		bg.tour_placement_iterations = this->params.tour_placement_iterations;
		bg.tour_placement_rounds = this->params.tour_placement_rounds;
		bg.tour_placement_object_step = this->params.tour_placement_object_step;
		bg.tour_placement_self_motion_step = this->params.tour_placement_self_motion_step;
		bg.tour_placement_joint_margin = this->params.tour_placement_joint_margin;
		bg.reconfig_threshold = this->params.reconfig_threshold;
		bg.travel_gradient = this->params.travel_gradient;
		bg.freeze_order = this->params.freeze_order;
		bg.lock_input_order = this->params.lock_input_order;
		bg.real_cost_planning_time = this->params.real_cost_planning_time;
		bg.real_cost_attempts = this->params.real_cost_attempts;
		bg.planned_travel_in_cost = this->params.planned_travel_in_cost;
		bg.room_then_reach = this->params.room_then_reach;
		bg.reach_time_limit = this->params.reach_time_limit;
		bg.trace_line_search = this->params.trace_line_search;
		bg.gtsp_restarts_at_reach = this->params.gtsp_restarts_at_reach;
		bg.random_seed = this->params.random_seed;
		bg.fd_gradient_check = this->params.fd_gradient_check;
		bg.fd_epsilon = this->params.fd_epsilon;
		bg.progress_pub = this->progress_marker_pub;
		bg.progress_mesh_path = this->resolved_mesh_path;
		bg.progress_mesh_scale = this->params.mesh_scale;
		bg.visualize_progress_delay_sec = this->params.visualize_progress_delay_sec;

		RCLCPP_INFO(
			this->get_logger(), "Descending the object position (x, y, z) for a %zu-pose tour...",
			tour_tcp_poses.size());

		object_placement::ObjectPlacementOptimizer optimizer(
			this->shared_from_this(), this->robot_model, local_scene, this->params.group_name, object_translation_world,
			object_rotation_world, tour_tcp_poses, this->home_joint_values, bg);
		ObjectPlacementResult result = optimizer.Optimize();

		ExportObjectPlacementResult(this->params.output_dir, result);

		this->marker_array = BuildObjectPlacementMarkerArray(
			this->now(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world, tour_tcp_poses, result);
		this->marker_array.markers.push_back(MountingSurfaceMarker(this->robot_model->getRootLinkName(), this->now()));
		this->marker_pub->publish(this->marker_array);

		if (!result.ok)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"Best base offset reaches only %d/%d tour poses -- not executing even if execute_on_robot is set",
				result.num_reachable, result.num_total);
			return;
		}

		if (this->params.execute_on_robot)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"execute_on_robot: driving the REAL robot against the object placed at abs (%.4f, %.4f, %.4f) m in "
				"software -- only correct if the physical object is actually fixtured to match.",
				result.x, result.y, result.z);

			ApplyObjectPlacementToScene(local_scene, object_rotation_world, result.x, result.y, result.z);

			const Eigen::Isometry3d object_offset =
				PlacementTransform(object_translation_world, result.x, result.y, result.z);

			std::vector<ViewpointCandidate> owned(result.tour_order.size());
			std::vector<const ViewpointCandidate*> selected;
			selected.reserve(result.tour_order.size());
			for (size_t k = 0; k < result.tour_order.size(); ++k)
			{
				Eigen::Isometry3d local_pose = object_offset * tour_tcp_poses[result.tour_order[k]];
				owned[k].tcp_pose.position.x = local_pose.translation().x();
				owned[k].tcp_pose.position.y = local_pose.translation().y();
				owned[k].tcp_pose.position.z = local_pose.translation().z();
				Eigen::Quaterniond q(local_pose.rotation());
				owned[k].tcp_pose.orientation.x = q.x();
				owned[k].tcp_pose.orientation.y = q.y();
				owned[k].tcp_pose.orientation.z = q.z();
				owned[k].tcp_pose.orientation.w = q.w();
				owned[k].joint_solution = result.joint_solutions[k];
				selected.push_back(&owned[k]);
			}

			ExecuteTourOnRobot(
				this->shared_from_this(), this->robot_model, local_scene, this->move_group, this->marker_pub, selected,
				this->home_joint_values, this->params.group_name, this->params.execution_planning_time,
				this->params.execution_planning_attempts);
		}
	}
};

int main(int argc, char* argv[])
{
	// Seed OMPL before any planner exists (it only takes a seed once), so planned travel repeats run to run.
	if (const char* seed = std::getenv("RANDOM_SEED"))
	{
		const unsigned long s = std::strtoul(seed, nullptr, 10);
		if (s != 0)
			ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(s));
	}
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectPlacementNode>();

	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
