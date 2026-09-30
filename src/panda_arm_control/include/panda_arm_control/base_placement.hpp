#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Base pose search bounds (x, y only -- no rotation search), applied as an object offset
// (object-offset duality).
struct BasePlacementBounds
{
	double x_min = -0.15, x_max = 0.15;
	double y_min = -0.15, y_max = 0.15;
};

// Search-strategy knobs for the two-layer B* algorithm (Zhao et al., arXiv:2504.12719).
struct BasePlacementParams
{
	BasePlacementBounds bounds;

	int num_restarts = 3;
	int random_seed = 42;

	// Step-1 relaxation (Sec. IV-A): joint IK over (arm dof, base x/y/theta) together via damped
	// least squares. Point 1 seeds from a random state; every later point seeds from the
	// preceding point's solution for continuity. A failure retries with a fresh random seed, up
	// to num_init_retries times, before that point is left unplaced.
	int num_init_retries = 20;
	int joint_ik_max_iterations = 300;
	double joint_ik_damping = 1e-3;
	// Levenberg-Marquardt: retries within one iteration, growing damping, until a step helps.
	int joint_ik_lm_max_escalations = 20;
	double ik_timeout = 3.0;

	// Outer layer (Eq. 9): mu(j) = mu_initial * mu_growth_factor^j penalizes per-point base poses
	// away from their mean, pulling them to one shared value.
	double mu_initial = 1.0;
	double mu_growth_factor = 2.0;
	int max_outer_iterations = 12;
	double outer_convergence_tolerance = 0.005;  // meters
	// Blends the end-effector's own orientation error into the FK residual tolerance below --
	// unrelated to object rotation, which isn't searched at all.
	double rot_metric_scale = 0.3;				  // meters per radian

	// Inner layer (Eq. 11): trust-region SLP, re-linearizing FK + collision each iteration.
	int max_inner_iterations = 25;
	// Caps each step's size since the FK/collision model is only a local linear approximation.
	double trust_region_initial = 0.1;
	double trust_region_shrink = 0.5;
	double trust_region_expand = 1.5;
	double trust_region_min = 1e-4;
	double trust_region_accept_ratio = 0.1;
	double trust_region_good_ratio = 0.75;
	// Small quadratic cost on dq/db so the LP's solver has a unique minimum to converge to.
	double trust_region_reg = 1e-3;
	// FK error (Eq. 5) is a minimized L1 cost, not a hard equality -- so the outer loop's
	// mu-pulling can never make one point's error infeasible for the whole combined LP.
	double fk_penalty_weight = 50.0;

	// Collision (Eq. 8): linearized signed-distance constraint for pairs within this threshold.
	double collision_distance_threshold = 0.1;
	double min_clearance = 0.01;

	// A viewpoint counts as reached in the final result if its FK residual is below this.
	double fk_residual_tolerance = 1e-3;

	// Logs analytic vs. finite-difference FK/collision Jacobians once per restart, to catch a bad
	// hand-derived Jacobian; this whole SLP is only as correct as those derivatives.
	bool fd_jacobian_check = false;
	double fd_epsilon = 1e-6;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	double visualize_progress_delay_sec = 0.0;
};

struct BasePlacementResult
{
	bool ok = false;
	int num_reachable = 0;
	int num_total = 0;
	double x = 0.0, y = 0.0;  // T(x,y,0), object frame

	std::vector<int> tour_order;
	std::vector<std::vector<double>> joint_solutions;

	double total_joint_path_length = 0.0;  // Eq. 4
	double fk_residual_max = 0.0;
};

// Faithful B* (Zhao et al., arXiv:2504.12719): fixed visiting order, outer-layer progressive
// constraint tightening over per-point relaxed base poses, inner-layer trust-region SLP (OSQP).
// Runs num_restarts independent instances, keeps the lowest-cost feasible one. Moves the
// registered "object" collision object during the search and restores it before returning.
BasePlacementResult SolveBasePlacement(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params);

// Writes base_placement_result.json to output_dir.
void ExportBasePlacementResult(const std::string& output_dir, const BasePlacementResult& result);

// Moves the registered "object" to its nominal pose adjusted by T(x, y, 0).
void ApplyBasePlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	double x,
	double y);

// Object mesh + tour polyline/waypoints re-expressed at the recommended offset. frame_id "world".
visualization_msgs::msg::MarkerArray BuildBasePlacementMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const BasePlacementResult& result);
