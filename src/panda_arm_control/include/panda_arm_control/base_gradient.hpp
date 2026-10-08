#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Absolute object position bounds (m, base frame); roll/pitch tilt about the object's position.
struct BaseGradientBounds
{
	double x_min = -0.1, x_max = 1.1;
	double y_min = -0.6, y_max = 0.6;
	double z_min = -0.05, z_max = 0.35;
	double roll_min = -0.35, roll_max = 0.35;	// radians (~20 deg)
	double pitch_min = -0.35, pitch_max = 0.35;  // radians
};

struct BaseGradientParams
{
	BaseGradientBounds bounds;
	// Absolute start position (m, base frame), NaN = nominal; start tilt (rad).
	double initial_x = NAN, initial_y = NAN, initial_z = NAN, initial_roll = 0.0, initial_pitch = 0.0;

	// Weights for GTSP
	double joint_distance_weight = 1.0;
	double cartesian_distance_weight = 0.0;
	double max_joint_deviation_weight = 1.0;

	// Manupulability: sqrt(J*J^T)
	double manipulability_weight = 0.0;
	double manipulability_weight_initial = 40.0;
	double manipulability_weight_decay = 0.8;
	double log_manipulability_weight = 2.0;
	// A missed viewpoint costs unreachable_penalty + miss_gap_weight * min(gap, miss_gap_cap), where gap
	// is its closest-IK pose gap (m, rotation scaled by rot_metric_scale): gives misses a gradient.
	double miss_gap_weight = 5000.0;
	double miss_gap_cap = 0.15;

	// Collision-aware closest IK for missed viewpoints: keeps arm-arm and arm-object pairs at least
	// closest_ik_margin apart; iterations per start, and starts (warm seed + random) per viewpoint.
	double closest_ik_margin = 0.02;
	int closest_ik_iters = 60;
	int closest_ik_starts = 5;

	double unreachable_penalty = 50.0;

	// Parameters for GTSP
	int max_solutions_per_candidate = 4; // number of IK solutions per viewpoint
	double ik_timeout = 0.15;
	int ik_retries_per_point = 10;
	int gtsp_two_opt_rounds = 5;
	int gtsp_num_restart = 2;

	// Base gradient parameters
	int descent_num_restart = 3;
	double descent_restart_perturbation = 0.1; // sigma value of gaussian perturbation
	int max_outer_iterations = 50;
	double initial_step = 0.02;
	double step_shrink = 0.25;
	int max_line_search_iters = 4;
	double jacobian_damping = 1e-3;  // lambda in the damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1
	double rot_metric_scale = 0.3;   // meters per radian, for blending translation & tip/tilt steps

	double convergence_tolerance_offset = 0.002;  // stop optimization when object moves < 2mm
	double convergence_tolerance_cost = 1e-3;	  // stop optimization when cost improvement < 0.1%
	int patience = 3; // stop optimization if no improvement after this many iterations

	int random_seed = 42;
	// Log the analytic gradient next to a central-difference estimate every outer iteration.
	bool fd_gradient_check = false;
	double fd_epsilon = 1e-4;

	// Optional live convergence markers on this topic (base breadcrumb trail, -grad arrow, current
	// tour). nullptr (default) disables progress publishing.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	std::string progress_mesh_path;  // object mesh drawn in the progress markers; empty = no mesh
	double progress_mesh_scale = 0.01;
	double visualize_progress_delay_sec = 0.0;
};

struct BaseGradientResult
{
	bool ok = false;  // true iff every input pose is reachable at the returned placement
	int num_reachable = 0;
	int num_total = 0;
	// Absolute object position (m, base frame) and tilt about it (rad), relative to nominal orientation.
	double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0;

	// Input-pose indices in visit order (the inner GTSP's tour).
	std::vector<int> tour_order;
	// One joint vector per entry, parallel to tour_order; empty where unreachable.
	std::vector<std::vector<double>> joint_solutions;

	double total_joint_path_length = 0.0;  // sum ||dq||_2 over tour edges (home -> first -> ...)
	double total_weighted_cost = 0.0;	   // the full weighted objective at the returned placement

	int num_inner_solves = 0;

	// {restart, x, y, z, roll, pitch, weighted_cost} after each outer iteration across all
	// restarts -- for plotting the descents.
	std::vector<std::array<double, 7>> history;
};

// Finds the object placement (abs x, y, z, roll, pitch) that minimizes tour joint travel, alternating
// GTSP solves with gradient-descent steps; restores the object's scene pose before returning.
BaseGradientResult SolveBaseGradient(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_nominal,
	const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal,
	const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params);

// Writes base_gradient_result.json to output_dir.
void ExportBaseGradientResult(const std::string& output_dir, const BaseGradientResult& result);

// Rigid transform taking nominal (abs) object/viewpoint poses to the placement at abs (x, y, z), tilted.
Eigen::Isometry3d PlacementTransform(
	const Eigen::Vector3d& object_translation_nominal, double x, double y, double z, double roll, double pitch);

// Moves the scene's "object" to abs (x, y, z), tilted; only valid once the real object is re-fixtured to match.
void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal,
	double x,
	double y,
	double z,
	double roll,
	double pitch);

// Object mesh + tour polyline/waypoints re-expressed in the recommended base's frame (same idea
// as BuildBasePlacementMarkerArray). frame_id is "world".
visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal,
	const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal,
	const BaseGradientResult& result);
