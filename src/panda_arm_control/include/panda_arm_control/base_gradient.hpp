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

// Absolute object position bounds (m, base frame). The object keeps its nominal orientation.
struct BaseGradientBounds
{
	double x_min = -0.1, x_max = 1.1;
	double y_min = -0.6, y_max = 0.6;
	double z_min = -0.05, z_max = 0.35;
};

struct BaseGradientParams
{
	BaseGradientBounds bounds;
	// Absolute start position (m, base frame), NaN = nominal.
	double initial_x = NAN, initial_y = NAN, initial_z = NAN;

	// Weights for GTSP
	double joint_distance_weight = 1.0;
	double cartesian_distance_weight = 0.0;
	double max_joint_deviation_weight = 1.0;

	// Manupulability: sqrt(J*J^T)
	double manipulability_weight = 0.0;
	double manipulability_weight_initial = 40.0;
	double manipulability_weight_decay = 0.8;
	// Missed viewpoints add their closest-IK pose's manipulability too (cost and gradient).
	bool manipulability_include_missed = false;
	// Range-aware manipulability: each joint's Jacobian column is scaled by 1 - exp(-k (q-lo)(hi-q)/range^2),
	// so joints near a limit count less. k = this; 0 = plain manipulability.
	double manipulability_limit_sharpness = 0.0;
	// Soft-min manipulability: cost -= weight * (-tau log sum exp(-w / tau)), ~ the worst poses' w. Their
	// shares of the push add up to 1, so one bad pose can't take over (unlike log). 0 = off.
	double softmin_manipulability_weight = 0.0;
	double softmin_tau = 0.01;
	// Self-clearance: each arm pose costs -weight * sum over all arm-arm pairs of log(distance), so the
	// gradient always spreads the links apart (folded postures cost more). 0 = off.
	double self_clearance_weight = 0.0;
	// Joint-limit room: each arm pose costs weight * sum over joints within `zone` (share of range) of a limit of
	// ((zone - f) / zone)^2, f = share of range to the nearest limit. Warns before a joint runs out. 0 = off.
	double joint_limit_weight = 0.0;
	double joint_limit_zone = 0.15;
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
	// > 0: each IK call is this many single KDL attempts instead of ik_timeout seconds, so runs repeat.
	int ik_attempts = 0;
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
	double rot_metric_scale = 0.3;   // meters per radian: weighs a missed viewpoint's angle gap against its distance gap

	double convergence_tolerance_offset = 0.002;  // stop optimization when object moves < 2mm
	double convergence_tolerance_cost = 1e-3;	  // stop optimization when cost improvement < 0.1%
	int patience = 3; // stop optimization if no improvement after this many iterations
	bool stop_when_all_reached = false;  // end each descent as soon as every viewpoint is reached
	int refine_solves = 0;  // extra full solves at each descent's result, warm-started from the best so far
	// Elbow-swing smoothing, after refine: slide each arm pose along its self-motion and move the object with them
	// to shorten the tour's joint travel; kept only if the planned travel drops.
	bool elbow_swing = false;
	int elbow_swing_iters = 50;
	double elbow_swing_object_step = 0.01;  // m: first object step
	double elbow_swing_joint_step = 0.1;  // first step along each pose's self-motion (times the travel slope)
	double elbow_swing_jump_threshold = 1.0;  // rad: a move turning any joint more than this counts as a big move
	double elbow_swing_jump_weight = 10.0;  // penalty per rad^2 of turn past the threshold; 0 = path length only
	bool travel_gradient = true;  // false: the descent direction ignores tour travel (cost still includes it)
	bool manipulability_gradient = true;  // false: the descent direction ignores the manipulability terms
	bool travel_in_cost = true;  // false: the placement cost leaves out travel (GTSP still orders by travel)
	bool freeze_order = false;  // once all viewpoints are reached, keep the viewpoint order fixed
	bool lock_input_order = false;  // visit viewpoints in the input order throughout (no GTSP reordering)
	bool manipulability_after_reach = false;  // misses only until all are reached, then manipulability on
	// > 0: refine picks tours by real cost (OMPL-planned joint travel); seconds per planning attempt.
	double real_cost_planning_time = 0.0;
	int real_cost_attempts = 3;  // planning attempts per leg; the shortest joint path is kept
	// Travel in the cost = planned joint travel instead of the L2 estimate, whatever travel_in_cost says (needs real_cost_planning_time > 0).
	bool planned_travel_in_cost = false;
	// Probes track the current arm poses (one warm IK attempt per viewpoint, fixed order) instead of a fresh search.
	bool track_probes = false;
	// Room-then-reach: one-sided walls learned from lost viewpoints; when stuck, aim at a missed viewpoint,
	// and if a blocker is lost, first move to give it room, then aim again. Moves that lose nothing may raise the cost.
	bool room_then_reach = false;
	bool room_target_vote = true;  // the aimed-at viewpoint votes in room moves too (false: only lost viewpoints)
	bool tracking = false;  // internal: set on the params copy used for tracked solves
	bool trace_line_search = false;  // log every line-search probe and full solve against the current point
	int refine_at_reach = 0;  // log N refine solves at the first all-reached placement (descent continues)

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
	// Absolute object position (m, base frame); orientation stays nominal.
	double x = 0.0, y = 0.0, z = 0.0;

	// Input-pose indices in visit order (the inner GTSP's tour).
	std::vector<int> tour_order;
	// One joint vector per entry, parallel to tour_order; empty where unreachable.
	std::vector<std::vector<double>> joint_solutions;

	double total_joint_path_length = 0.0;  // sum ||dq||_2 over tour edges (home -> first -> ...)
	double total_weighted_cost = 0.0;	   // the full weighted objective at the returned placement

	int num_inner_solves = 0;

	// {restart, x, y, z, weighted_cost} after each outer iteration across all
	// restarts -- for plotting the descents.
	std::vector<std::array<double, 5>> history;
};

// Finds the object position (abs x, y, z) that minimizes tour joint travel, alternating
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

// Rigid transform taking nominal (abs) object/viewpoint poses to the placement at abs (x, y, z).
Eigen::Isometry3d PlacementTransform(const Eigen::Vector3d& object_translation_nominal, double x, double y, double z);

// Moves the scene's "object" to abs (x, y, z); only valid once the real object is re-fixtured to match.
void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal,
	double x,
	double y,
	double z);

// Object mesh + tour polyline/waypoints re-expressed in the recommended base's frame (same idea
// as BuildBstarPlacementMarkerArray). frame_id is "world".
visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal,
	const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal,
	const BaseGradientResult& result);
