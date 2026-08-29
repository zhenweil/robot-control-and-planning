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

// What is optimized is the rigid pose adjustment applied to the OBJECT (and, rigidly with it, its
// viewpoints) expressed in the robot base frame (panda_link0): translation (x, y, z) plus tip
// (roll, about base x) and tilt (pitch, about base y). Physically realized by re-fixturing the
// object on a tip/tilt stage -- equivalent to, and easier than, repositioning/re-tilting the arm
// base. Yaw (about base z) is excluded: it is redundant with joint 1, which rotates the whole arm
// about that axis, so it leaves the joint-travel objective unchanged.
struct BaseGradientBounds
{
	double x_min = -0.3, x_max = 0.3;
	double y_min = -0.3, y_max = 0.3;
	double z_min = -0.2, z_max = 0.2;
	double roll_min = -0.35, roll_max = 0.35;	// radians (~20 deg)
	double pitch_min = -0.35, pitch_max = 0.35;  // radians
};

struct BaseGradientParams
{
	BaseGradientBounds bounds;
	// Where the descent starts, relative to the object's nominal pose (default 0 = nominal).
	double initial_x = 0.0, initial_y = 0.0, initial_z = 0.0, initial_roll = 0.0, initial_pitch = 0.0;

	// Objective weights (mirrors HierarchicalTourParams): total tour cost is
	//   sum_edges [ w_cart*||dp|| + w_joint*||dq||_2 + w_maxdev*max_k|dq_k| ].
	// The Cartesian term is invariant to the base pose, so it never affects the gradient -- it is
	// kept only so the inner GTSP orders the tour the same way the rest of the pipeline does.
	double joint_distance_weight = 1.0;
	double cartesian_distance_weight = 0.0;
	double max_joint_deviation_weight = 1.0;
	// Added to weighted_cost for every viewpoint left unreachable at an offset. Without it a
	// partial solution looks cheap only because it visits fewer poses; make it dominate any
	// plausible tour cost so full reachability always wins.
	double unreachable_penalty = 50.0;

	// Inner GTSP (redundant-IK generalized TSP): each viewpoint offers up to this many distinct IK
	// branches and the solver picks whichever ordering + branch minimizes reconfiguration. Used
	// only for the committed solve each iteration; line-search probes always use 1 (cheap).
	int max_solutions_per_candidate = 2;
	double ik_timeout = 0.1;
	int ik_retries_per_point = 8;
	int gtsp_two_opt_rounds = 5;

	// Basin hopping to escape local minima: restart 0 descends from initial_*, each later restart
	// descends from the best offset so far kicked by a Gaussian of std-dev restart_perturbation
	// (meters; the tip/tilt kick is that over rot_metric_scale). Staying near the incumbent keeps
	// restarts feasible, unlike sampling the whole bounds box. 1 = single descent.
	int num_restarts = 3;
	double restart_perturbation = 0.08;
	// Stop launching restarts once there is a fully-reachable result and this many consecutive
	// restarts failed to beat it. 0 disables (always run all num_restarts).
	int restart_patience = 2;

	// Gradient descent on the object offset. The descent direction is the unit-normalized negative
	// gradient in a mixed metric where 1 rad of tip/tilt counts as `rot_metric_scale` meters, so
	// `initial_step` / `min_step` are that blended displacement, not scaled by the raw gradient.
	int max_outer_iterations = 15;
	double initial_step = 0.05;
	double step_shrink = 0.5;
	double armijo_c = 1e-4;
	double min_step = 1e-4;
	int max_line_search_iters = 8;
	double jacobian_damping = 1e-3;  // lambda in the damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1
	double rot_metric_scale = 0.3;   // meters per radian, for blending translation & tip/tilt steps

	double convergence_tolerance_offset = 0.002;  // blended (see rot_metric_scale) offset move per iteration
	double convergence_tolerance_cost = 1e-3;	  // relative tour-cost improvement per outer iteration
	// Stop early once this many consecutive iterations each improve the cost by less than
	// convergence_tolerance_cost (relative) -- avoids grinding through many near-zero-gain steps.
	int patience = 3;

	int random_seed = 42;
	// Log the analytic gradient next to a central-difference estimate every outer iteration.
	bool fd_gradient_check = false;
	double fd_epsilon = 1e-4;

	// Optional live convergence markers on this topic (base breadcrumb trail, -grad arrow, current
	// tour). nullptr (default) disables progress publishing.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
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
