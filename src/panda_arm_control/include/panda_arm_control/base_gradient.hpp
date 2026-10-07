#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

struct BaseGradientBounds
{
	double x_min = -0.6, x_max = 0.6;
	double y_min = -0.6, y_max = 0.6;
	double z_min = -0.2, z_max = 0.2;
	double roll_min = -0.35, roll_max = 0.35;	// radians (~20 deg)
	double pitch_min = -0.35, pitch_max = 0.35;  // radians
};

struct BaseGradientParams
{
	BaseGradientBounds bounds;
	// Where the descent starts, relative to the object's nominal pose (default 0 = nominal).
	double initial_x = 0.0, initial_y = 0.0, initial_z = 0.0, initial_roll = 0.0, initial_pitch = 0.0;

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
	bool ok = false;  // true iff every input pose is reachable at the returned offset
	int num_reachable = 0;
	int num_total = 0;
	// Object offset relative to its nominal pose, in the robot base frame: translation (m) + tip
	// roll / tilt pitch (rad). Applied as T(x,y,z) * Ry(pitch) * Rx(roll).
	double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0;

	// Input-pose indices in visit order (the inner GTSP's tour).
	std::vector<int> tour_order;
	// One joint vector per entry, parallel to tour_order; empty where unreachable.
	std::vector<std::vector<double>> joint_solutions;

	double total_joint_path_length = 0.0;  // sum ||dq||_2 over tour edges (home -> first -> ...)
	double total_weighted_cost = 0.0;	   // the full weighted objective at the returned offset

	// Full inner solves consumed by the descent (iteration-0 solve + one per accepted step, each
	// counting gtsp_num_restart sub-solves; line-search probes not included). A budget for a fair
	// random-search comparison.
	int num_inner_solves = 0;

	// {restart, x, y, z, roll, pitch, weighted_cost} after each outer iteration across all
	// restarts -- for plotting the descents.
	std::vector<std::array<double, 7>> history;
};

// Alternating minimization of tour joint travel over the object offset (x, y, z, roll, pitch):
// run the inner redundant-IK GTSP at the current offset, take the analytic gradient of the
// weighted tour cost w.r.t. the offset (via the manipulator Jacobian), backtracking-line-search a
// descent step, then re-run the GTSP -- until the offset and the cost both settle.
//
// object_translation_original/object_rotation_original and tour_tcp_poses_original follow the same
// current-mount-frame convention as SolveBasePlacement (see base_placement.hpp). The planning
// scene monitor must already have the object registered (id "object"); this function moves that
// object's pose during the search and restores it before returning.
BaseGradientResult SolveBaseGradient(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params);

// Writes base_gradient_result.json to output_dir.
void ExportBaseGradientResult(const std::string& output_dir, const BaseGradientResult& result);

// ---------------------------------------------------------------------------------------------
// Attribution experiment: is a descent's cost drop the object move, or just inner-solve
// (IK / GTSP) variance and warm-starting? For one random seed (sweep the seed across processes
// -- that sweep is "experiment 2"):
//   exp 1  cold inner solve (IK from scratch, every seed = home, full multi-branch GTSP) at
//          offset 0 and at the descent's recommended offset b*. Same solver effort both ends,
//          only the object pose differs -- so any cost gap is the move, not warm-starting.
//   exp 3  cold inner solve at `num_random_directions` random offsets whose mixed-metric
//          magnitude equals |b*|. The optimized direction should beat random ones of equal size.
//   exp 4  the same random offsets (and b*), warm inner solve -- IK seeds and GTSP order both
//          warm-started from the offset-0 cold solution, one hop. Isolates the warm-start
//          discount from the placement effect.
//   exp 5  random search of equal budget: cold-solve the descent's num_inner_solves worth of
//          offsets drawn uniformly in the bounds, keep the best. If the descent does not beat
//          this, the gradient is not earning its complexity.
// ---------------------------------------------------------------------------------------------
struct BaseGradientPointEval
{
	double weighted_cost = 0.0;  // as the solver scores it (includes the unreachable penalty)
	double honest_cost = 0.0;	 // penalty stripped -- comparable to BaseGradientResult::total_weighted_cost
	int num_reachable = 0;
	bool all_reachable = false;
	std::array<double, 5> offset{{0, 0, 0, 0, 0}};  // x, y, z, roll, pitch actually evaluated (post-bounds)
	double d_metric = 0.0;						   // sqrt(x^2+y^2+z^2 + (roll*s)^2 + (pitch*s)^2), s = rot_metric_scale
};

struct BaseGradientExperimentResult
{
	int seed = 0;
	int num_total = 0;
	double rot_metric_scale = 0.3;

	bool descent_ok = false;
	std::array<double, 5> descent_offset{{0, 0, 0, 0, 0}};
	double descent_d_metric = 0.0;
	double descent_reported_cost = 0.0;  // BaseGradientResult::total_weighted_cost from the internal descent

	double probe_d_metric = 0.0;  // radius used for the exp 3 / 4 random offsets
	bool probe_d_floored = false;  // true if |b*| was ~0 and probe_d_metric was raised to a floor

	int descent_inner_solves = 0;  // budget exp 5 matched (from BaseGradientResult::num_inner_solves)

	BaseGradientPointEval c0_cold;	  // exp 1
	BaseGradientPointEval copt_cold;  // exp 1
	BaseGradientPointEval copt_warm;  // exp 4 at b*
	std::vector<BaseGradientPointEval> rand_cold;  // exp 3
	std::vector<BaseGradientPointEval> rand_warm;  // exp 4 (offsets parallel to rand_cold)
	std::vector<BaseGradientPointEval> random_search;  // exp 5 (uniform-in-bounds, cold)
};

// Runs the descent once (for b*), then exp 1 / 3 / 4 / 5 above. Restores the scene before
// returning. random_search_budget <= 0 means "match the descent's num_inner_solves".
BaseGradientExperimentResult RunBaseGradientExperiment(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params,
	int num_random_directions,
	double min_probe_d_metric,
	int random_search_budget);

// Writes base_gradient_experiment_seed<seed>.json to output_dir.
void ExportBaseGradientExperimentResult(const std::string& output_dir, const BaseGradientExperimentResult& result);

// ---------------------------------------------------------------------------------------------
// Score one object offset with exactly the placement experiment's / descent's inner solve, so a
// separate placement optimizer (e.g. the B* baseline in base_placement.cpp) can share the
// objective verbatim. offset is (x, y, z, roll, pitch); it is clamped to params.bounds.
// min-of-params.gtsp_num_restart IK collections, then a full redundant-IK GTSP re-route -- or, if
// fixed_order is non-empty, an exact fixed-order branch DP over that visiting sequence. Restores
// the scene before returning.
// ---------------------------------------------------------------------------------------------
struct ObjectOffsetScore
{
	double weighted_cost = 0.0;  // solver metric: joint L2 + max-dev, + unreachable_penalty/pose
	double honest_cost = 0.0;	 // penalty stripped
	int num_reachable = 0;
	int num_total = 0;
	bool all_reachable = false;
	std::vector<int> tour;					 // visit order (viewpoint indices)
	std::vector<std::vector<double>> joints;  // parallel: chosen IK branch
};

ObjectOffsetScore ScoreObjectOffset(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params,
	const std::array<double, 5>& offset,
	const std::vector<int>& fixed_order);

// ---------------------------------------------------------------------------------------------
// Placement / order separability experiment. Answers three questions the descent leaves open:
//   1  with the visiting route frozen, does the object (x, y, z) position change the tour cost?
//      (rotation is held at 0 here.)
//   2  at the best position for a route, does re-routing (full GTSP) lower the cost further?
//   3  does the best object position move when the route changes -- are placement and routing
//      separable, or coupled?
// One sweep feeds all three: a grid_n^3 grid of (x, y, z) offsets over the bounds. At each grid
// point the IK branch set is collected once (min-of-gtsp_num_restart), then every reference route
// is scored against it with an exact fixed-order branch DP (SolveFixedOrder), and one free-
// routing full GTSP is run on the same branches. Reference routes are the full GTSP's output at
// the nominal offset and at 6 spread offsets, each padded to a full permutation.
// The grid can be sharded (grid_start / grid_count) across processes. The seed-pose RNG is
// re-seeded per grid point from (seed, flat index); MoveIt's IK plugin RNG still advances with
// the global call order, so a sharded run differs slightly from a single-process one -- within
// the min-of-gtsp_num_restart cost noise, not enough to move the flat-vs-structured verdict.
// ---------------------------------------------------------------------------------------------
struct PlacementGridPoint
{
	std::array<double, 3> offset{{0, 0, 0}};  // x, y, z actually evaluated
	int num_ik_reachable = 0;				 // viewpoints with >=1 collision-free IK branch here

	// Parallel to PlacementOrderExperimentResult::reference_orders -- the fixed-route branch-DP
	// score of each reference route at this offset (best over the gtsp_num_restart replicas).
	std::vector<double> order_weighted_cost;  // includes the unreachable penalty (solver metric)
	std::vector<double> order_honest_cost;	 // penalty stripped
	std::vector<int> order_num_reachable;

	// Free-routing full GTSP (reorder + branch) on the same branch set, best over replicas.
	double full_weighted_cost = 0.0;
	double full_honest_cost = 0.0;
	int full_num_reachable = 0;
	std::vector<int> full_tour;  // the re-optimized visiting order (viewpoint indices)
};

struct PlacementOrderExperimentResult
{
	int seed = 0;
	int num_total = 0;
	double unreachable_penalty = 50.0;

	std::array<int, 3> grid_shape{{0, 0, 0}};	   // (nx, ny, nz), all = grid_n
	std::array<double, 3> grid_min{{0, 0, 0}};	   // (x_min, y_min, z_min)
	std::array<double, 3> grid_max{{0, 0, 0}};
	int grid_start = 0;	 // flat index of the first grid point in `grid`
	int grid_count = 0;	 // number of grid points in `grid` (this shard's slice)

	std::vector<std::string> order_labels;
	std::vector<std::vector<int>> reference_orders;	// full permutations of 0..num_total-1
	std::vector<PlacementGridPoint> grid;			// the slice [grid_start, grid_start + grid_count)
};

// grid_count < 0 means "to the end of the grid from grid_start".
// reference_orders_file: if non-empty, load the reference routes from that JSON instead of
// solving for them -- so every shard of a parallel sweep scores against an identical route set
// (route generation at reachability-breaking offsets is IK-timeout / CPU-load sensitive and does
// not reproduce across processes). When empty, the routes are solved and written to
// <output_dir>/placement_reference_orders_seed<seed>.json for the grid shards to reuse.
PlacementOrderExperimentResult RunPlacementOrderExperiment(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params,
	int grid_n,
	int grid_start,
	int grid_count,
	const std::string& reference_orders_file,
	const std::string& output_dir);

// Writes placement_experiment_seed<seed>_g<grid_start>.json to output_dir.
void ExportPlacementOrderExperimentResult(const std::string& output_dir, const PlacementOrderExperimentResult& result);

// Moves the registered "object" collision object to its nominal pose adjusted by the offset
// T(x,y,z) * Ry(pitch) * Rx(roll) in the robot base frame. Only valid if that adjustment has
// actually been realized on the physical object (re-fixtured to match).
void ApplyObjectOffsetToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
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
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const BaseGradientResult& result);
