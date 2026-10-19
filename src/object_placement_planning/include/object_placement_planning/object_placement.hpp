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

#include "object_placement_planning/joint_travel.hpp"

// Absolute object position bounds (m, base frame). The object keeps its nominal orientation.
struct ObjectPlacementBounds
{
	double x_min = -0.1, x_max = 1.1;
	double y_min = -0.6, y_max = 0.6;
	double z_min = -0.05, z_max = 0.35;
};

struct ObjectPlacementParams
{
	ObjectPlacementBounds bounds;
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
	double exact_ik_finish_gap = 0.05;	 // m (rotation at rot_metric_scale): a miss closer than this gets exact IK from its closest pose
	// IK for reached viewpoints: "kdl" (MoveIt's KDL plugin) or "closest" (closest_ik's exact stage, same solver as the
	// misses' closest poses). With "closest": ignore_camera_roll frees the camera's spin about its axis (also in the
	// misses' gaps), and an exact pose must keep exact_ik_margin (m) clearance (0 = collision-free only).
	std::string ik_solver = "kdl";
	bool ignore_camera_roll = false;
	double exact_ik_margin = 0.0;

	double unreachable_penalty = 50.0;

	// Parameters for GTSP
	int max_solutions_per_candidate = 4; // number of IK solutions per viewpoint
	double ik_timeout = 0.15;
	// > 0: each IK call is this many single KDL attempts instead of ik_timeout seconds, so runs repeat.
	int ik_attempts = 0;
	int ik_retries_per_point = 10;
	int gtsp_two_opt_rounds = 5;
	int gtsp_num_restart = 2;

	// Object placement parameters
	int descent_num_restart = 3;
	// Before each restart: climb the reached viewpoints' manipulability (sum + this x soft-min) from the stuck spot.
	double jump_softmin_weight = 100.0;
	int jump_max_moves = 10;  // moves per manipulability jump
	int max_kinematic_moves = 20;  // per reach attempt
	double initial_step = 0.02;
	double step_shrink = 0.25;
	double reach_first_step = 0.01;  // m: while viewpoints are missed, moves try this, doubling below initial_step, before initial_step; 0 = off
	int max_line_search_iters = 4;
	double jacobian_damping = 1e-3;  // lambda in the damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1
	double rot_metric_scale = 0.3;   // meters per radian: weighs a missed viewpoint's angle gap against its distance gap

	double convergence_tolerance_offset = 0.002;  // stop optimization when object moves < 2mm
	double convergence_tolerance_cost = 1e-3;	  // stop optimization when cost improvement < 0.1%
	int patience = 3; // stop optimization if no improvement after this many iterations
	bool stop_when_all_reached = false;  // end each descent as soon as every viewpoint is reached
	int reach_patience = 3;  // reach-phase iterations without gaining a viewpoint before the descent counts as stuck
	int gtsp_restarts = 0;  // extra full solves at each descent's result, warm-started from the best so far
	// After GTSP restarts: QP steps moving the object and each tour pose along its self-motion (order and IK branches fixed) to
	// shorten the tour's joint distance; kept if planned travel drops. 0 = off.
	int tour_placement_iterations = 0;
	int tour_placement_rounds = 1;  // GTSP restarts then tour-placement, repeated this many times
	double tour_placement_object_step = 0.02;  // m per axis, halved when a step is rejected; 0 = object held still
	double tour_placement_self_motion_step = 0.3;  // rad per pose, halved with the object step
	double tour_placement_joint_margin = 0.05;  // rad kept from each joint limit
	int gtsp_restart_ik_solutions = 8;  // IK solutions per viewpoint in GTSP restarts: more same-branch choices for GTSP
	// Reconfiguration: a move turning any joint (joint 7 aside) more than this; GTSP restarts log how many are left.
	double reconfig_threshold = 1.0;
	// Planned travel = execution time (s), stopping at every pose: straight joint moves where collision-free, else OMPL,
	// timed under the joint speed and acceleration limits. Pair with travel_time_weight for the GTSP.
	bool travel_cost_is_time = false;
	// Once all are reached, before the GTSP restarts + tour-placement rounds: fewer reconfigurations, then more margin.
	bool reduce_reconfigurations = false;  // IK in the neighbor's posture, then object moves for the rest
	int reconfiguration_max_moves = 10;  // object moves in that stage
	bool raise_margins = false;  // roomier IK poses, then object moves, until every margin reaches margin_target_floors
	double margin_target_floors = 2.0;  // smallest margin wanted, in floors (joint limit 0.05 rad, stretch 0.02, collision 0.01 m)
	int margin_max_moves = 10;  // object moves in that stage
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
	int room_max_targets = 3;
	bool room_for_missed_viewpoint = true;	 // room-then-reach: when aiming stops closing a missed viewpoint's gap, move to open what blocks its closest pose
	// Before a restart: > 0 runs up to restart_room_max_moves room-then-reach moves that may lose this many reached
	// viewpoints, instead of the manipulability jump (0).
	int restart_room_max_lost_viewpoints = 0;
	bool restart_switch_posture = false;  // restarts stay at the best spot with the reached viewpoints in other arm postures
	int restart_room_max_moves = 3;  // missed viewpoints room-then-reach tries per call, closest first; 0 = all
	bool tracking = false;  // internal: set on the params copy used for tracked solves
	// internal: a solve reaching fewer viewpoints than this skips the missed ones' closest-IK gaps (a probe that is
	// rejected on count anyway); 0 = never
	int gaps_min_reached = 0;
	bool trace_line_search = false;  // log every line-search probe and full solve against the current point
	int gtsp_restarts_at_reach = 0;  // log N GTSP restarts at the first all-reached placement (descent continues)

	int random_seed = 42;

	// Optional live convergence markers on this topic (base breadcrumb trail, -grad arrow, current
	// tour). nullptr (default) disables progress publishing.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	std::string progress_mesh_path;  // object mesh drawn in the progress markers; empty = no mesh
	double progress_mesh_scale = 0.01;
	double visualize_progress_delay_sec = 0.0;
};

struct ObjectPlacementResult
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

// Writes object_placement_result.json to output_dir.
void ExportObjectPlacementResult(const std::string& output_dir, const ObjectPlacementResult& result);

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
visualization_msgs::msg::MarkerArray BuildObjectPlacementMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal,
	const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal,
	const ObjectPlacementResult& result);
