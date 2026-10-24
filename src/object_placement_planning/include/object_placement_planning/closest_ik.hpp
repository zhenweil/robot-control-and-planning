#pragma once
// Collision-aware closest IK: the collision-free arm pose that gets nearest to a target the arm can't reach exactly.

#include <atomic>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

#include "object_placement_planning/scene_geometry.hpp"

namespace closest_ik
{

// Per close pair: d(distance)/dq over the group's joints, the distance, d(distance)/d(object position) (zero unless
// one side is the object) and "link - link".
struct ClosePairs
{
	std::vector<Eigen::RowVectorXd> gradients;
	std::vector<double> dists;
	std::vector<Eigen::Vector3d> object_slopes;
	std::vector<std::string> names;
};

// Close pairs in one distance result, with their slopes.
ClosePairs ClosePairGradients(
	const collision_detection::DistanceResult& res, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, double margin);

// A distance request for the group's links, reporting pairs closer than threshold.
collision_detection::DistanceRequest CloseRequest(
	const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double threshold);

// Self and world distances at the state's pose, skipping pairs the scene always allows (as the validity check does).
std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> Distances(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state,
	const collision_detection::DistanceRequest& req);

// Self and world pairs closer than margin, with their slopes.
ClosePairs FindClosePairs(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, const collision_detection::DistanceRequest& req, double margin);

// Links rigid with the tool, plus what hangs off them (hand, fingers).
std::set<std::string> ToolSideLinks(const moveit::core::LinkModel* tool);

// True if a 1 cm rod from the tool to the target hits nothing (tool side excepted, and the object if ignore_object).
bool ClearPathToTarget(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::LinkModel* tool, const Eigen::Vector3d& target, const std::set<std::string>& tool_side, bool ignore_object = false);

struct Options
{
	int iters = 60;					 // closest stage
	double margin = 0.02;			 // m: clearance the closest stage keeps
	double rot_scale = 0.1;			 // m per rad in the gap
	double limit_sharpness = 20.0;	 // manipulability's joint-limit falloff
	bool ignore_roll = false;		 // the tool's spin about its z axis (the camera axis) is free
	bool try_exact_first = true;	 // exact stage first: plain damped Newton steps, one validity check at the end
	bool camera_can_enter_object = false;  // closest stage: the camera may go into the object (exact poses still may not)
	// Weighted closest stage: one cost, no priorities. Each term is weight x squared shortfall.
	bool use_weighted_cost = false;
	int weighted_iters = 20;			   // steps of the weighted closest stage
	double position_weight = 1.0;		   // per m^2 of position error to the target
	double angle_weight = 0.01;			   // per rad^2 of angle error to the target
	double collision_weight = 1.0;		   // per m^2 a close pair is short of margin
	double camera_object_weight = 1.0;	   // the same for the camera's pairs with the object
	double self_distance_weight = 0.0;	   // per m^2 the arm's own pairs are short of self_distance_margin (keeps it unfolded)
	double self_distance_margin = 0.08;	   // m
	double joint_room_weight = 1e-3;	   // per joint: x s^2 / (1 - s^2), s = (angle - range middle) / half range
	bool exact_only = false;		 // skip the closest stage
	int exact_iters = 100;
	double exact_margin = 0.0;				 // m: clearance an exact pose must keep; 0 = collision-free only
	double exact_position_tolerance = 1e-4;	 // m
	double exact_rotation_tolerance = 1e-3;	 // rad
};

struct Result
{
	bool found = false;
	bool exact = false;	 // within the exact tolerances
	scene_geometry::Vector6d pose_error = scene_geometry::Vector6d::Zero();
	double gap = std::numeric_limits<double>::max();
	std::vector<double> q;	// the best collision-free pose; empty if none
	double weighted_cost = std::numeric_limits<double>::max();	// weighted closest stage only
};

// Time spent in the weighted closest stage, summed over threads (for the log); reset by the caller.
struct WeightedSolveTiming
{
	std::atomic<long> solves{0};
	std::atomic<long> steps{0};
	std::atomic<long> distance_microseconds{0};	 // distance queries for the cost
	std::atomic<long> check_microseconds{0};	 // validity and clear-path checks of the kept pose
};
WeightedSolveTiming& GetWeightedSolveTiming();
void ResetWeightedSolveTiming();

// [position; rotation] error from tool to target; with ignore_roll the rotation only aligns the tool's z axis.
scene_geometry::Vector6d GetPoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool, bool ignore_roll);

// Exact IK if the target is reachable (exact stage), else the closest collision-free pose: damped steps toward the
// target that keep close pairs out to `margin` (first priority) and climb manipulability in the target's null space.
// Joints at a limit that the step pushes further are frozen and the step re-solved.
// With use_weighted_cost the closest stage instead lowers one weighted cost and returns the cheapest valid pose.
Result Solve(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target,
	const std::vector<double>& start, const Options& options);

}  // namespace closest_ik
