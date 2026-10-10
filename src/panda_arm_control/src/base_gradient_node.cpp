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
#include <ompl/util/RandomNumbers.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/base_gradient.hpp"
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
	std::string output_dir = "/tmp/base_gradient_output";

	// Object position bounds and start, abs (m, base frame); roll/pitch are tilt (rad).
	double bg_x_min = -0.1, bg_x_max = 1.1;
	double bg_y_min = -0.6, bg_y_max = 0.6;
	double bg_z_min = -0.05, bg_z_max = 0.35;
	double bg_roll_min = -0.35, bg_roll_max = 0.35;	  // rad (~20 deg) -- object tip
	double bg_pitch_min = -0.35, bg_pitch_max = 0.35;  // rad -- object tilt
	double bg_yaw_min = 0.0, bg_yaw_max = 0.0;		  // rad -- object spin about z; equal = locked
	bool bg_yaw_after_translation = true;
	double bg_initial_x = 0.5, bg_initial_y = 0.0, bg_initial_z = 0.15;
	double bg_initial_roll = 0.0, bg_initial_pitch = 0.0, bg_initial_yaw = 0.0;
	double bg_rot_metric_scale = 0.3;  // m per rad, blends translation & tip/tilt in the step

	double bg_joint_distance_weight = 1.0;
	double bg_cartesian_distance_weight = 0.0;
	double bg_max_joint_deviation_weight = 1.0;
	double bg_manipulability_weight = 0.0;
	double bg_manipulability_weight_initial = 40.0;
	double bg_manipulability_weight_decay = 0.8;
	double bg_log_manipulability_weight = 2.0;
	double bg_miss_gap_weight = 5000.0;
	double bg_cost_slack = 0.0;
	bool bg_manipulability_include_missed = false;
	double bg_self_clearance_weight = 0.0;
	double bg_joint_limit_weight = 0.0;
	double bg_joint_limit_zone = 0.15;
	double bg_manipulability_limit_sharpness = 0.0;
	double bg_softmin_manipulability_weight = 0.0;
	double bg_softmin_tau = 0.01;
	double bg_miss_gap_cap = 0.15;
	double bg_closest_ik_margin = 0.02;
	int bg_closest_ik_iters = 60;
	int bg_closest_ik_starts = 5;
	double bg_unreachable_penalty = 50.0;

	int bg_max_solutions_per_candidate = 4;
	int bg_ik_retries_per_point = 14;
	int bg_gtsp_two_opt_rounds = 5;
	int bg_gtsp_num_restart = 2;  // min-of-N inner solves wherever a committed cost matters

	int bg_descent_num_restart = 6;
	double bg_descent_restart_perturbation = 0.05;
	int bg_max_outer_iterations = 50;
	double bg_initial_step = 0.02;
	double bg_step_shrink = 0.25;
	int bg_max_line_search_iters = 4;
	double bg_jacobian_damping = 1e-3;

	double bg_convergence_tolerance_offset = 0.002;
	double bg_convergence_tolerance_cost = 1e-3;
	int bg_patience = 3;
	bool bg_stop_when_all_reached = false;
	int bg_refine_solves = 0;
	bool bg_travel_gradient = true;
	bool bg_manipulability_gradient = true;
	bool bg_travel_in_cost = true;
	bool bg_freeze_order = false;
	bool bg_lock_input_order = false;
	bool bg_manipulability_after_reach = false;
	double bg_real_cost_planning_time = 0.0;
	int bg_real_cost_attempts = 3;
	bool bg_planned_travel_in_cost = false;
	bool bg_steering = true;
	bool bg_reach_probe = false;
	int bg_miss_tolerance = 0;
	bool bg_reach_rules = false;
	bool bg_track_probes = false;
	bool bg_room_then_reach = false;
	double bg_room_budget = 30.0;
	double bg_rule_limit_room = 0.02;
	double bg_rule_clearance = 0.005;
	bool bg_trace_line_search = false;
	int bg_refine_at_reach = 0;

	bool bg_fd_gradient_check = false;
	double bg_fd_epsilon = 1e-4;

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

std::vector<Eigen::Isometry3d> LoadTourTcpPoses(const std::string& tour_input_dir)
{
	std::string json_path = tour_input_dir + "/selected_robot_poses.json";
	std::ifstream file(json_path);
	if (!file.is_open())
		throw std::runtime_error("Could not open tour input file: " + json_path);

	Json::Value root;
	Json::CharReaderBuilder reader_builder;
	std::string errs;
	if (!Json::parseFromStream(reader_builder, file, &root, &errs))
		throw std::runtime_error("Failed to parse " + json_path + ": " + errs);

	std::vector<std::pair<int, Eigen::Isometry3d>> entries;
	for (const auto& item : root)
	{
		int id = item["id"].asInt();
		const auto& pos = item["tcp_position"];
		const auto& quat = item["tcp_quaternion_xyzw"];

		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		pose.translation() = Eigen::Vector3d(pos["x"].asDouble(), pos["y"].asDouble(), pos["z"].asDouble());
		pose.linear() = Eigen::Quaterniond(
							 quat["w"].asDouble(), quat["x"].asDouble(), quat["y"].asDouble(), quat["z"].asDouble())
							 .toRotationMatrix();
		entries.emplace_back(id, pose);
	}

	std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	std::vector<Eigen::Isometry3d> poses;
	poses.reserve(entries.size());
	for (const auto& e : entries)
		poses.push_back(e.second);
	return poses;
}

}  // namespace

class BaseGradientNode : public rclcpp::Node
{
public:
	explicit BaseGradientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("base_gradient", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
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
			"/base_gradient_markers", rclcpp::QoS(1).transient_local());
		this->progress_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/base_gradient_progress_markers", rclcpp::QoS(1).transient_local());

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
		this->declareIfNeeded("bg_x_min", this->params.bg_x_min);
		this->declareIfNeeded("bg_x_max", this->params.bg_x_max);
		this->declareIfNeeded("bg_y_min", this->params.bg_y_min);
		this->declareIfNeeded("bg_y_max", this->params.bg_y_max);
		this->declareIfNeeded("bg_z_min", this->params.bg_z_min);
		this->declareIfNeeded("bg_z_max", this->params.bg_z_max);
		this->declareIfNeeded("bg_roll_min", this->params.bg_roll_min);
		this->declareIfNeeded("bg_roll_max", this->params.bg_roll_max);
		this->declareIfNeeded("bg_pitch_min", this->params.bg_pitch_min);
		this->declareIfNeeded("bg_pitch_max", this->params.bg_pitch_max);
		this->declareIfNeeded("bg_yaw_min", this->params.bg_yaw_min);
		this->declareIfNeeded("bg_yaw_max", this->params.bg_yaw_max);
		this->declareIfNeeded("bg_yaw_after_translation", this->params.bg_yaw_after_translation);
		this->declareIfNeeded("bg_initial_x", this->params.bg_initial_x);
		this->declareIfNeeded("bg_initial_y", this->params.bg_initial_y);
		this->declareIfNeeded("bg_initial_z", this->params.bg_initial_z);
		this->declareIfNeeded("bg_initial_roll", this->params.bg_initial_roll);
		this->declareIfNeeded("bg_initial_pitch", this->params.bg_initial_pitch);
		this->declareIfNeeded("bg_initial_yaw", this->params.bg_initial_yaw);
		this->declareIfNeeded("bg_rot_metric_scale", this->params.bg_rot_metric_scale);
		this->declareIfNeeded("bg_joint_distance_weight", this->params.bg_joint_distance_weight);
		this->declareIfNeeded("bg_cartesian_distance_weight", this->params.bg_cartesian_distance_weight);
		this->declareIfNeeded("bg_max_joint_deviation_weight", this->params.bg_max_joint_deviation_weight);
		this->declareIfNeeded("bg_manipulability_weight", this->params.bg_manipulability_weight);
		this->declareIfNeeded(
			"bg_manipulability_weight_initial", this->params.bg_manipulability_weight_initial);
		this->declareIfNeeded("bg_manipulability_weight_decay", this->params.bg_manipulability_weight_decay);
		this->declareIfNeeded("bg_log_manipulability_weight", this->params.bg_log_manipulability_weight);
		this->declareIfNeeded("bg_miss_gap_weight", this->params.bg_miss_gap_weight);
		this->declareIfNeeded("bg_cost_slack", this->params.bg_cost_slack);
		this->declareIfNeeded("bg_manipulability_include_missed", this->params.bg_manipulability_include_missed);
		this->declareIfNeeded("bg_self_clearance_weight", this->params.bg_self_clearance_weight);
		this->declareIfNeeded("bg_joint_limit_weight", this->params.bg_joint_limit_weight);
		this->declareIfNeeded("bg_joint_limit_zone", this->params.bg_joint_limit_zone);
		this->declareIfNeeded("bg_manipulability_limit_sharpness", this->params.bg_manipulability_limit_sharpness);
		this->declareIfNeeded("bg_softmin_manipulability_weight", this->params.bg_softmin_manipulability_weight);
		this->declareIfNeeded("bg_softmin_tau", this->params.bg_softmin_tau);
		this->declareIfNeeded("bg_miss_gap_cap", this->params.bg_miss_gap_cap);
		this->declareIfNeeded("bg_closest_ik_margin", this->params.bg_closest_ik_margin);
		this->declareIfNeeded("bg_closest_ik_iters", this->params.bg_closest_ik_iters);
		this->declareIfNeeded("bg_closest_ik_starts", this->params.bg_closest_ik_starts);
		this->declareIfNeeded("bg_unreachable_penalty", this->params.bg_unreachable_penalty);
		this->declareIfNeeded("bg_max_solutions_per_candidate", this->params.bg_max_solutions_per_candidate);
		this->declareIfNeeded("bg_ik_retries_per_point", this->params.bg_ik_retries_per_point);
		this->declareIfNeeded("bg_gtsp_num_restart", this->params.bg_gtsp_num_restart);
		this->declareIfNeeded("bg_gtsp_two_opt_rounds", this->params.bg_gtsp_two_opt_rounds);
		this->declareIfNeeded("bg_descent_num_restart", this->params.bg_descent_num_restart);
		this->declareIfNeeded("bg_descent_restart_perturbation", this->params.bg_descent_restart_perturbation);
		this->declareIfNeeded("bg_max_outer_iterations", this->params.bg_max_outer_iterations);
		this->declareIfNeeded("bg_initial_step", this->params.bg_initial_step);
		this->declareIfNeeded("bg_step_shrink", this->params.bg_step_shrink);
		this->declareIfNeeded("bg_max_line_search_iters", this->params.bg_max_line_search_iters);
		this->declareIfNeeded("bg_jacobian_damping", this->params.bg_jacobian_damping);
		this->declareIfNeeded("bg_convergence_tolerance_offset", this->params.bg_convergence_tolerance_offset);
		this->declareIfNeeded("bg_convergence_tolerance_cost", this->params.bg_convergence_tolerance_cost);
		this->declareIfNeeded("bg_patience", this->params.bg_patience);
		this->declareIfNeeded("bg_stop_when_all_reached", this->params.bg_stop_when_all_reached);
		this->declareIfNeeded("bg_refine_solves", this->params.bg_refine_solves);
		this->declareIfNeeded("bg_travel_gradient", this->params.bg_travel_gradient);
		this->declareIfNeeded("bg_manipulability_gradient", this->params.bg_manipulability_gradient);
		this->declareIfNeeded("bg_travel_in_cost", this->params.bg_travel_in_cost);
		this->declareIfNeeded("bg_freeze_order", this->params.bg_freeze_order);
		this->declareIfNeeded("bg_lock_input_order", this->params.bg_lock_input_order);
		this->declareIfNeeded("bg_manipulability_after_reach", this->params.bg_manipulability_after_reach);
		this->declareIfNeeded("bg_real_cost_planning_time", this->params.bg_real_cost_planning_time);
		this->declareIfNeeded("bg_real_cost_attempts", this->params.bg_real_cost_attempts);
		this->declareIfNeeded("bg_planned_travel_in_cost", this->params.bg_planned_travel_in_cost);
		this->declareIfNeeded("bg_steering", this->params.bg_steering);
		this->declareIfNeeded("bg_reach_probe", this->params.bg_reach_probe);
		this->declareIfNeeded("bg_miss_tolerance", this->params.bg_miss_tolerance);
		this->declareIfNeeded("bg_reach_rules", this->params.bg_reach_rules);
		this->declareIfNeeded("bg_track_probes", this->params.bg_track_probes);
		this->declareIfNeeded("bg_room_then_reach", this->params.bg_room_then_reach);
		this->declareIfNeeded("bg_room_budget", this->params.bg_room_budget);
		this->declareIfNeeded("bg_rule_limit_room", this->params.bg_rule_limit_room);
		this->declareIfNeeded("bg_rule_clearance", this->params.bg_rule_clearance);
		this->declareIfNeeded("bg_trace_line_search", this->params.bg_trace_line_search);
		this->declareIfNeeded("bg_refine_at_reach", this->params.bg_refine_at_reach);
		this->declareIfNeeded("bg_fd_gradient_check", this->params.bg_fd_gradient_check);
		this->declareIfNeeded("bg_fd_epsilon", this->params.bg_fd_epsilon);
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
		this->get_parameter("bg_x_min", this->params.bg_x_min);
		this->get_parameter("bg_x_max", this->params.bg_x_max);
		this->get_parameter("bg_y_min", this->params.bg_y_min);
		this->get_parameter("bg_y_max", this->params.bg_y_max);
		this->get_parameter("bg_z_min", this->params.bg_z_min);
		this->get_parameter("bg_z_max", this->params.bg_z_max);
		this->get_parameter("bg_roll_min", this->params.bg_roll_min);
		this->get_parameter("bg_roll_max", this->params.bg_roll_max);
		this->get_parameter("bg_pitch_min", this->params.bg_pitch_min);
		this->get_parameter("bg_pitch_max", this->params.bg_pitch_max);
		this->get_parameter("bg_yaw_min", this->params.bg_yaw_min);
		this->get_parameter("bg_yaw_max", this->params.bg_yaw_max);
		this->get_parameter("bg_yaw_after_translation", this->params.bg_yaw_after_translation);
		this->get_parameter("bg_initial_x", this->params.bg_initial_x);
		this->get_parameter("bg_initial_y", this->params.bg_initial_y);
		this->get_parameter("bg_initial_z", this->params.bg_initial_z);
		this->get_parameter("bg_initial_roll", this->params.bg_initial_roll);
		this->get_parameter("bg_initial_pitch", this->params.bg_initial_pitch);
		this->get_parameter("bg_initial_yaw", this->params.bg_initial_yaw);
		this->get_parameter("bg_rot_metric_scale", this->params.bg_rot_metric_scale);
		this->get_parameter("bg_joint_distance_weight", this->params.bg_joint_distance_weight);
		this->get_parameter("bg_cartesian_distance_weight", this->params.bg_cartesian_distance_weight);
		this->get_parameter("bg_max_joint_deviation_weight", this->params.bg_max_joint_deviation_weight);
		this->get_parameter("bg_manipulability_weight", this->params.bg_manipulability_weight);
		this->get_parameter("bg_manipulability_weight_initial", this->params.bg_manipulability_weight_initial);
		this->get_parameter("bg_manipulability_weight_decay", this->params.bg_manipulability_weight_decay);
		this->get_parameter("bg_log_manipulability_weight", this->params.bg_log_manipulability_weight);
		this->get_parameter("bg_miss_gap_weight", this->params.bg_miss_gap_weight);
		this->get_parameter("bg_cost_slack", this->params.bg_cost_slack);
		this->get_parameter("bg_manipulability_include_missed", this->params.bg_manipulability_include_missed);
		this->get_parameter("bg_self_clearance_weight", this->params.bg_self_clearance_weight);
		this->get_parameter("bg_joint_limit_weight", this->params.bg_joint_limit_weight);
		this->get_parameter("bg_joint_limit_zone", this->params.bg_joint_limit_zone);
		this->get_parameter("bg_manipulability_limit_sharpness", this->params.bg_manipulability_limit_sharpness);
		this->get_parameter("bg_softmin_manipulability_weight", this->params.bg_softmin_manipulability_weight);
		this->get_parameter("bg_softmin_tau", this->params.bg_softmin_tau);
		this->get_parameter("bg_miss_gap_cap", this->params.bg_miss_gap_cap);
		this->get_parameter("bg_closest_ik_margin", this->params.bg_closest_ik_margin);
		this->get_parameter("bg_closest_ik_iters", this->params.bg_closest_ik_iters);
		this->get_parameter("bg_closest_ik_starts", this->params.bg_closest_ik_starts);
		this->get_parameter("bg_unreachable_penalty", this->params.bg_unreachable_penalty);
		this->get_parameter("bg_max_solutions_per_candidate", this->params.bg_max_solutions_per_candidate);
		this->get_parameter("bg_ik_retries_per_point", this->params.bg_ik_retries_per_point);
		this->get_parameter("bg_gtsp_num_restart", this->params.bg_gtsp_num_restart);
		this->get_parameter("bg_gtsp_two_opt_rounds", this->params.bg_gtsp_two_opt_rounds);
		this->get_parameter("bg_descent_num_restart", this->params.bg_descent_num_restart);
		this->get_parameter("bg_descent_restart_perturbation", this->params.bg_descent_restart_perturbation);
		this->get_parameter("bg_max_outer_iterations", this->params.bg_max_outer_iterations);
		this->get_parameter("bg_initial_step", this->params.bg_initial_step);
		this->get_parameter("bg_step_shrink", this->params.bg_step_shrink);
		this->get_parameter("bg_max_line_search_iters", this->params.bg_max_line_search_iters);
		this->get_parameter("bg_jacobian_damping", this->params.bg_jacobian_damping);
		this->get_parameter("bg_convergence_tolerance_offset", this->params.bg_convergence_tolerance_offset);
		this->get_parameter("bg_convergence_tolerance_cost", this->params.bg_convergence_tolerance_cost);
		this->get_parameter("bg_patience", this->params.bg_patience);
		this->get_parameter("bg_stop_when_all_reached", this->params.bg_stop_when_all_reached);
		this->get_parameter("bg_refine_solves", this->params.bg_refine_solves);
		this->get_parameter("bg_travel_gradient", this->params.bg_travel_gradient);
		this->get_parameter("bg_manipulability_gradient", this->params.bg_manipulability_gradient);
		this->get_parameter("bg_travel_in_cost", this->params.bg_travel_in_cost);
		this->get_parameter("bg_freeze_order", this->params.bg_freeze_order);
		this->get_parameter("bg_lock_input_order", this->params.bg_lock_input_order);
		this->get_parameter("bg_manipulability_after_reach", this->params.bg_manipulability_after_reach);
		this->get_parameter("bg_real_cost_planning_time", this->params.bg_real_cost_planning_time);
		this->get_parameter("bg_real_cost_attempts", this->params.bg_real_cost_attempts);
		this->get_parameter("bg_planned_travel_in_cost", this->params.bg_planned_travel_in_cost);
		this->get_parameter("bg_steering", this->params.bg_steering);
		this->get_parameter("bg_reach_probe", this->params.bg_reach_probe);
		this->get_parameter("bg_miss_tolerance", this->params.bg_miss_tolerance);
		this->get_parameter("bg_reach_rules", this->params.bg_reach_rules);
		this->get_parameter("bg_track_probes", this->params.bg_track_probes);
		this->get_parameter("bg_room_then_reach", this->params.bg_room_then_reach);
		this->get_parameter("bg_room_budget", this->params.bg_room_budget);
		this->get_parameter("bg_rule_limit_room", this->params.bg_rule_limit_room);
		this->get_parameter("bg_rule_clearance", this->params.bg_rule_clearance);
		this->get_parameter("bg_trace_line_search", this->params.bg_trace_line_search);
		this->get_parameter("bg_refine_at_reach", this->params.bg_refine_at_reach);
		this->get_parameter("bg_fd_gradient_check", this->params.bg_fd_gradient_check);
		this->get_parameter("bg_fd_epsilon", this->params.bg_fd_epsilon);
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

		BaseGradientParams bg;
		bg.bounds.x_min = this->params.bg_x_min;
		bg.bounds.x_max = this->params.bg_x_max;
		bg.bounds.y_min = this->params.bg_y_min;
		bg.bounds.y_max = this->params.bg_y_max;
		bg.bounds.z_min = this->params.bg_z_min;
		bg.bounds.z_max = this->params.bg_z_max;
		bg.bounds.roll_min = this->params.bg_roll_min;
		bg.bounds.roll_max = this->params.bg_roll_max;
		bg.bounds.pitch_min = this->params.bg_pitch_min;
		bg.bounds.pitch_max = this->params.bg_pitch_max;
		bg.bounds.yaw_min = this->params.bg_yaw_min;
		bg.bounds.yaw_max = this->params.bg_yaw_max;
		bg.yaw_after_translation = this->params.bg_yaw_after_translation;
		bg.initial_x = this->params.bg_initial_x;
		bg.initial_y = this->params.bg_initial_y;
		bg.initial_z = this->params.bg_initial_z;
		bg.initial_roll = this->params.bg_initial_roll;
		bg.initial_pitch = this->params.bg_initial_pitch;
		bg.initial_yaw = this->params.bg_initial_yaw;
		bg.rot_metric_scale = this->params.bg_rot_metric_scale;
		bg.joint_distance_weight = this->params.bg_joint_distance_weight;
		bg.cartesian_distance_weight = this->params.bg_cartesian_distance_weight;
		bg.max_joint_deviation_weight = this->params.bg_max_joint_deviation_weight;
		bg.manipulability_weight = this->params.bg_manipulability_weight;
		bg.manipulability_weight_initial = this->params.bg_manipulability_weight_initial;
		bg.manipulability_weight_decay = this->params.bg_manipulability_weight_decay;
		bg.log_manipulability_weight = this->params.bg_log_manipulability_weight;
		bg.miss_gap_weight = this->params.bg_miss_gap_weight;
		bg.cost_slack = this->params.bg_cost_slack;
		bg.manipulability_include_missed = this->params.bg_manipulability_include_missed;
		bg.self_clearance_weight = this->params.bg_self_clearance_weight;
		bg.joint_limit_weight = this->params.bg_joint_limit_weight;
		bg.joint_limit_zone = this->params.bg_joint_limit_zone;
		bg.manipulability_limit_sharpness = this->params.bg_manipulability_limit_sharpness;
		bg.softmin_manipulability_weight = this->params.bg_softmin_manipulability_weight;
		bg.softmin_tau = this->params.bg_softmin_tau;
		bg.miss_gap_cap = this->params.bg_miss_gap_cap;
		bg.closest_ik_margin = this->params.bg_closest_ik_margin;
		bg.closest_ik_iters = this->params.bg_closest_ik_iters;
		bg.closest_ik_starts = this->params.bg_closest_ik_starts;
		bg.unreachable_penalty = this->params.bg_unreachable_penalty;
		bg.max_solutions_per_candidate = this->params.bg_max_solutions_per_candidate;
		bg.ik_timeout = this->params.ik_timeout;
		bg.ik_attempts = this->params.ik_attempts;
		bg.ik_retries_per_point = this->params.bg_ik_retries_per_point;
		bg.gtsp_num_restart = this->params.bg_gtsp_num_restart;
		bg.gtsp_two_opt_rounds = this->params.bg_gtsp_two_opt_rounds;
		bg.descent_num_restart = this->params.bg_descent_num_restart;
		bg.descent_restart_perturbation = this->params.bg_descent_restart_perturbation;
		bg.max_outer_iterations = this->params.bg_max_outer_iterations;
		bg.initial_step = this->params.bg_initial_step;
		bg.step_shrink = this->params.bg_step_shrink;
		bg.max_line_search_iters = this->params.bg_max_line_search_iters;
		bg.jacobian_damping = this->params.bg_jacobian_damping;
		bg.convergence_tolerance_offset = this->params.bg_convergence_tolerance_offset;
		bg.convergence_tolerance_cost = this->params.bg_convergence_tolerance_cost;
		bg.patience = this->params.bg_patience;
		bg.stop_when_all_reached = this->params.bg_stop_when_all_reached;
		bg.refine_solves = this->params.bg_refine_solves;
		bg.travel_gradient = this->params.bg_travel_gradient;
		bg.manipulability_gradient = this->params.bg_manipulability_gradient;
		bg.travel_in_cost = this->params.bg_travel_in_cost;
		bg.freeze_order = this->params.bg_freeze_order;
		bg.lock_input_order = this->params.bg_lock_input_order;
		bg.manipulability_after_reach = this->params.bg_manipulability_after_reach;
		bg.real_cost_planning_time = this->params.bg_real_cost_planning_time;
		bg.real_cost_attempts = this->params.bg_real_cost_attempts;
		bg.planned_travel_in_cost = this->params.bg_planned_travel_in_cost;
		bg.steering = this->params.bg_steering;
		bg.reach_probe = this->params.bg_reach_probe;
		bg.miss_tolerance = this->params.bg_miss_tolerance;
		bg.reach_rules = this->params.bg_reach_rules;
		bg.track_probes = this->params.bg_track_probes;
		bg.room_then_reach = this->params.bg_room_then_reach;
		bg.room_budget = this->params.bg_room_budget;
		bg.rule_limit_room = this->params.bg_rule_limit_room;
		bg.rule_clearance = this->params.bg_rule_clearance;
		bg.trace_line_search = this->params.bg_trace_line_search;
		bg.refine_at_reach = this->params.bg_refine_at_reach;
		bg.random_seed = this->params.random_seed;
		bg.fd_gradient_check = this->params.bg_fd_gradient_check;
		bg.fd_epsilon = this->params.bg_fd_epsilon;
		bg.progress_pub = this->progress_marker_pub;
		bg.progress_mesh_path = this->resolved_mesh_path;
		bg.progress_mesh_scale = this->params.mesh_scale;
		bg.visualize_progress_delay_sec = this->params.visualize_progress_delay_sec;

		RCLCPP_INFO(
			this->get_logger(), "Descending the object offset (x, y, z, tip, tilt) for a %zu-pose tour...",
			tour_tcp_poses.size());

		BaseGradientResult result = SolveBaseGradient(
			this->shared_from_this(), this->robot_model, local_scene, this->params.group_name, object_translation_world,
			object_rotation_world, tour_tcp_poses, this->home_joint_values, bg);

		ExportBaseGradientResult(this->params.output_dir, result);

		this->marker_array = BuildBaseGradientMarkerArray(
			this->now(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world, tour_tcp_poses, result);
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
				"execute_on_robot: driving the REAL robot against the object placed at abs (%.4f, %.4f, %.4f) m + "
				"tip %.2f deg / tilt %.2f deg / spin %.2f deg in software -- only correct if the physical object is actually "
				"fixtured to match.",
				result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI,
				result.yaw * 180.0 / M_PI);

			ApplyObjectPlacementToScene(
				local_scene, object_rotation_world, result.x, result.y, result.z, result.roll, result.pitch, result.yaw);

			const Eigen::Isometry3d object_offset = PlacementTransform(
				object_translation_world, result.x, result.y, result.z, result.roll, result.pitch, result.yaw);

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
	auto node = std::make_shared<BaseGradientNode>();

	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
