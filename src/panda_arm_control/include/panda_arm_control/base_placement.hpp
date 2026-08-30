#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/base_gradient.hpp"  // BaseGradientParams / ScoreObjectOffset -- the shared objective

// Search bounds for the candidate object offset (x, y, z) in the robot base frame, relative to
// the object's nominal pose. Rotation is held at 0 (the base_gradient placement experiment does
// the same -- tip/tilt is a separate axis and base yaw is redundant with joint 1). Kept small by
// default since re-fixturing an object usually can't move it far.
struct BasePlacementBounds
{
	double x_min = -0.15, x_max = 0.15;
	double y_min = -0.15, y_max = 0.15;
	double z_min = -0.15, z_max = 0.15;
};

// Search-strategy knobs. The *cost* every candidate offset is scored with comes from
// ScoreObjectOffset (see base_gradient.hpp) -- pass its BaseGradientParams separately.
struct BasePlacementParams
{
	BasePlacementBounds bounds;
	// B*-style: independent restarts from different initializations, keep the lowest-cost result.
	// Restart 0 starts from the mobile-base relaxation mean; the rest from random offsets.
	int num_restarts = 3;
	// Pattern-search iterations per restart (each probes +/- step on x, y, z and steps to the
	// best improving neighbour, else halves the step).
	int max_outer_iterations = 10;
	// Random feasible-pose candidates tried per point in the step-1 relaxation.
	int candidates_per_point = 16;
	// IK random-restart retries per point in the relaxation (the final score uses its own IK).
	int ik_retries_per_point = 4;
	double ik_timeout = 0.1;
	// Pattern-search step: starts at initial_step, shrinks by step_shrink when no probe improves,
	// stops at min_step.
	double initial_step = 0.06;
	double step_shrink = 0.5;
	double min_step = 0.005;
	// Stop a restart once an iteration improves the cost by less than this (absolute).
	double convergence_tolerance_cost = 1e-2;

	int random_seed = 42;

	// Optional live convergence markers (per-point relaxation guesses + running mean). nullptr
	// (default) disables progress publishing.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	double visualize_progress_delay_sec = 0.0;
};

struct BasePlacementResult
{
	bool ok = false;  // true iff every input pose is reachable at (x, y, z)
	int num_reachable = 0;
	int num_total = 0;
	// Object offset relative to its nominal pose, in the robot base frame (m). Rotation is 0.
	double x = 0.0, y = 0.0, z = 0.0;

	// The frozen visiting order (its reachable prefix at the winning offset) and the chosen IK
	// branch per stop -- parallel, length == num_reachable.
	std::vector<int> tour_order;
	std::vector<std::vector<double>> joint_solutions;

	double weighted_cost = 0.0;  // ScoreObjectOffset's metric (joint L2 + max-dev, + penalty/pose)
	double honest_cost = 0.0;	 // penalty stripped -- comparable to the experiment's honest_cost
	double total_joint_path_length = 0.0;  // sum |dq| (L1) over tour edges, home -> first -> ...
};

// B*-style fixed-base placement search, adapted as a baseline for the base_gradient experiments.
// Like the paper's B*, the visiting order is a fixed input -- here the tour as the
// viewpoint_planner_* run produced it (tour_tcp_poses_original is already in visiting order, so
// the frozen order is 0, 1, ..., n-1) -- and only the placement is optimized. It optimizes the
// object offset (x, y, z) (the same variable the placement experiment sweeps) and scores every
// candidate with ScoreObjectOffset restricted to that frozen order (an exact branch DP -- the
// same cost the experiment's frozen-route column, EXP 1, uses).
//
//   1. Relaxation: per point, independently find any collision-free object offset (mobile-base
//      relaxation). Their mean is restart 0's starting point.
//   2. Pattern search: from that start, probe +/- step on each axis, step to the lowest-cost
//      improving neighbour (cost = ScoreObjectOffset), shrink the step when none improves, until
//      the step floors or the gain stalls.
//   3. Repeat from random starts; keep the lowest-cost result across all restarts (unlike the
//      original which bailed on the first feasible one).
//
// object_translation_original/object_rotation_original: the object's pose relative to panda_link0
// as nominally mounted. tour_tcp_poses_original: tool0 targets in that same frame. The planning
// scene monitor must already have the object registered (id "object"); this moves it during the
// search and restores it before returning.
BasePlacementResult SolveBasePlacement(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params,
	const BaseGradientParams& score_params);

// Writes base_placement_result.json to output_dir.
void ExportBasePlacementResult(const std::string& output_dir, const BasePlacementResult& result);

// Moves the registered "object" collision object to its nominal pose adjusted by the offset
// T(x, y, z) in the robot base frame. Only valid if that offset has actually been realized on the
// physical object (re-fixtured to match).
void ApplyBasePlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	double x,
	double y,
	double z);

// Object mesh + tour polyline/waypoints re-expressed at the recommended offset. frame_id "world".
visualization_msgs::msg::MarkerArray BuildBasePlacementMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const BasePlacementResult& result);
