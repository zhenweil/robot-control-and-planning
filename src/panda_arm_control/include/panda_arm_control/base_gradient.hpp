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

#include "panda_arm_control/joint_travel.hpp"

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
	// Absolute start position (m, base frame).
	double initial_x = 0.5, initial_y = 0.0, initial_z = 0.15;

	// Tour edge cost: joint distance and largest single-joint turn
	joint_travel::EdgeWeights edge_weights;

	// Manipulability sqrt(det(J J^T)) of the tour poses and the misses' closest poses: while viewpoints are missed, cost -=
	// manipulability_weight * sum + softmin_manipulability_weight * soft-min. Once all are reached the cost is travel only.
	double manipulability_weight = 0.0;
	double softmin_manipulability_weight = 0.0;
	// Range-aware manipulability: each joint's Jacobian column is scaled by 1 - exp(-k (q-lo)(hi-q)/range^2),
	// so joints near a limit count less. k = this; 0 = plain manipulability.
	double manipulability_limit_sharpness = 0.0;
	// Soft-min manipulability -tau log sum exp(-w / tau), ~ the worst poses' w.
	double softmin_tau = 0.01;
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
	// Before each restart: climb the reached viewpoints' manipulability (sum + this x soft-min) from the stuck spot.
	double jump_softmin_weight = 100.0;
	int jump_max_moves = 10;  // moves per manipulability jump
	// Outer loop on a reachability map (reachability_map.hpp; build it first with reachability_map.launch.py): once every
	// viewpoint is reached, step up the soft-min over all viewpoints of how well the arm reaches their position and
	// approach direction (no IK), solve again from there, keep the lower cost. The map settings below must match the ones
	// it was built with (config/reachability_map.yaml).
	int reach_outer_iterations = 0;  // 0 = off
	double reach_outer_step = 0.1;  // m, halved when a step doesn't lower the cost
	std::string reach_map_file;  // cache; empty = ~/.ros/reachability_<group>.bin
	int reach_map_samples = 1000000;
	bool reach_map_self_collision = true;
	double reach_map_voxel = 0.04;  // m
	int reach_map_directions = 100;  // approach-direction bins
	double reach_map_tau = 0.05;  // soft-min temperature (map values are 0..1)
	double reach_map_limit_sharpness = 20.0;  // map samples count by their joint-limit room; 0 = all count 1
	int reach_map_seed = 42;
	int max_kinematic_moves = 20;  // per reach attempt
	double initial_step = 0.02;
	double step_shrink = 0.25;
	int max_line_search_iters = 4;
	double jacobian_damping = 1e-3;  // lambda in the damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1
	double rot_metric_scale = 0.3;   // meters per radian: weighs a missed viewpoint's angle gap against its distance gap

	double convergence_tolerance_offset = 0.002;  // stop optimization when object moves < 2mm
	double convergence_tolerance_cost = 1e-3;	  // stop optimization when cost improvement < 0.1%
	int patience = 3; // stop optimization if no improvement after this many iterations
	bool stop_when_all_reached = false;  // end each descent as soon as every viewpoint is reached
	double reach_time_limit = 0.0;  // s per descent to reach every viewpoint; then it counts as stuck. 0 = none
	int reach_patience = 3;  // reach-phase iterations without gaining a viewpoint before the descent counts as stuck
	int refine_solves = 0;  // extra full solves at each descent's result, warm-started from the best so far
	// After refine: QP steps moving the object and each tour pose along its self-motion (order and IK branches fixed) to
	// shorten the tour's joint distance; kept if planned travel drops. 0 = off.
	int tour_placement_iterations = 0;
	double tour_placement_object_step = 0.02;  // m per axis, halved when a step is rejected
	double tour_placement_self_motion_step = 0.3;  // rad per pose, halved with the object step
	double tour_placement_joint_margin = 0.05;  // rad kept from each joint limit
	int compare_extra_refine_solves = 0;  // comparison: also run this many more refine solves from the same refined tour and log it (not used)
	int refine_ik_solutions = 8;  // IK solutions per viewpoint in refine solves: more same-branch choices for GTSP
	// Reconfiguration: a move turning any joint (joint 7 aside) more than this; refine logs how many are left.
	double reconfig_threshold = 1.0;
	bool travel_gradient = true;  // false: the descent direction ignores tour travel
	bool freeze_order = false;  // once all viewpoints are reached, keep the viewpoint order fixed
	bool lock_input_order = false;  // visit viewpoints in the input order throughout (no GTSP reordering)
	// > 0: OMPL-planned joint travel can be computed; seconds per planning attempt.
	double real_cost_planning_time = 0.0;
	int real_cost_attempts = 3;  // planning attempts per edge; the shortest joint path is kept
	// Travel in the cost = planned joint travel, once every viewpoint is reached (needs real_cost_planning_time > 0).
	bool planned_travel_in_cost = false;
	// Room-then-reach: when stuck with misses, aim at a missed viewpoint; if that loses reached viewpoints, back off
	// to raise the margins (joint limit, reach, stretch, collision) that ran out for them, then aim again.
	bool room_then_reach = false;
	int room_max_targets = 3;  // missed viewpoints room-then-reach tries per call, closest first; 0 = all
	bool tracking = false;  // internal: set on the params copy used for tracked solves
	// internal: a solve reaching fewer viewpoints than this skips the missed ones' closest-IK gaps (a probe that is
	// rejected on count anyway); 0 = never
	int gaps_min_reached = 0;
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

// The search itself is object_placement::ObjectPlacementOptimizer (object_placement_optimizer.hpp).

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
