#pragma once
// Collision-aware closest IK: the collision-free arm pose that gets nearest to a target the arm can't reach exactly.

#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

#include "panda_arm_control/scene_geometry.hpp"

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

// True if a 1 cm rod from the tool to the target hits nothing (tool side excepted).
bool ClearPathToTarget(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::LinkModel* tool, const Eigen::Vector3d& target, const std::set<std::string>& tool_side);

struct Result
{
	bool found = false;
	scene_geometry::Vector6d pose_error = scene_geometry::Vector6d::Zero();
	double gap = std::numeric_limits<double>::max();
	std::vector<double> q;	// the best collision-free pose; empty if none
};

// Damped steps toward the target that keep close pairs out to `margin` (first priority) and climb manipulability in
// the target's null space. Joints at a limit that the step pushes further are frozen and the step re-solved.
Result Solve(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target,
	const std::vector<double>& start, int iters, double margin, double rot_scale, double limit_sharpness);

}  // namespace closest_ik
