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

// Search bounds for the base offset (x, y, yaw), relative to the robot's actual mount
// (panda_link0) -- same convention as BasePlacementBounds in base_placement.hpp.
struct BaseGradientBounds
{
	double x_min = -0.3, x_max = 0.3;
	double y_min = -0.3, y_max = 0.3;
	double yaw_min = -M_PI, yaw_max = M_PI;
};

struct BaseGradientParams
{
	BaseGradientBounds bounds;
	// Where the descent starts, relative to today's mount (default 0 = the real mount).
	double initial_x = 0.0, initial_y = 0.0, initial_yaw = 0.0;

	// Base yaw is redundant with joint 1 (both rotate the arm about the mount's z axis), so for the
	// joint-travel objective it has no effect and is not optimized by default -- the base stays at
	// initial_yaw. Enable only if joint-1 limits at some viewpoints make the base angle matter.
	bool optimize_yaw = false;

	// Objective weights (mirrors HierarchicalTourParams): total tour cost is
	//   sum_edges [ w_cart*||dp|| + w_joint*||dq||_2 + w_maxdev*max_k|dq_k| ].
	// The Cartesian term is invariant to the base pose, so it never affects the gradient -- it is
	// kept only so the inner GTSP orders the tour the same way the rest of the pipeline does.
	double joint_distance_weight = 1.0;
	double cartesian_distance_weight = 0.0;
	double max_joint_deviation_weight = 1.0;

	// Inner GTSP (redundant-IK generalized TSP): each viewpoint offers up to this many distinct IK
	// branches and the solver picks whichever ordering + branch minimizes reconfiguration.
	int max_solutions_per_candidate = 3;
	double ik_timeout = 0.1;
	int ik_retries_per_point = 8;
	int gtsp_two_opt_rounds = 5;

	// Gradient descent on the base pose. The descent direction is the unit-normalized negative
	// gradient, so `initial_step` / `min_step` are actual base displacements (meters on x/y,
	// radians on yaw), not scaled by the raw (cost-per-meter) gradient magnitude.
	int max_outer_iterations = 15;
	double initial_step = 0.05;
	double step_shrink = 0.5;
	double armijo_c = 1e-4;
	double min_step = 1e-4;
	int max_line_search_iters = 12;
	double jacobian_damping = 1e-3;  // lambda in the damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1

	double convergence_tolerance_xy = 0.002;   // meters, base move per outer iteration
	double convergence_tolerance_yaw = 0.01;   // radians
	double convergence_tolerance_cost = 1e-3;  // relative tour-cost improvement per outer iteration
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
	bool ok = false;  // true iff every input pose is reachable at the returned (x, y, yaw)
	int num_reachable = 0;
	int num_total = 0;
	double x = 0.0, y = 0.0, yaw = 0.0;  // offset relative to today's actual mount

	// Input-pose indices in visit order (the inner GTSP's tour).
	std::vector<int> tour_order;
	// One joint vector per entry, parallel to tour_order; empty where unreachable.
	std::vector<std::vector<double>> joint_solutions;

	double total_joint_path_length = 0.0;  // sum ||dq||_2 over tour edges (home -> first -> ...)
	double total_weighted_cost = 0.0;	   // the full weighted objective at the returned base

	// {x, y, yaw, weighted_cost} after each outer iteration -- for plotting the descent.
	std::vector<std::array<double, 4>> history;
};

// Alternating minimization of tour joint travel over the base offset (x, y, yaw): run the inner
// redundant-IK GTSP at the current base, take the analytic gradient of the weighted tour cost
// w.r.t. the base pose (via the manipulator Jacobian), backtracking-line-search a descent step,
// then re-run the GTSP -- until the base and the cost both settle.
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
