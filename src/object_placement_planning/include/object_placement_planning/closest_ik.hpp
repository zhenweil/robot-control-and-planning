#pragma once
// Collision-aware IK: exact if the target is reachable, else the nearest collision-free pose.

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

// Close link pairs: distance, its joint gradient, its object-position slope, and "link - link".
struct ClosePairs
{
	std::vector<Eigen::RowVectorXd> gradients;
	std::vector<double> dists;
	std::vector<Eigen::Vector3d> object_slopes;  // zero unless one side is the object
	std::vector<std::string> names;
};

// Pairs closer than margin in one distance result, with their gradients.
ClosePairs ClosePairGradients(const collision_detection::DistanceResult& distance_result, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double margin);

// Distance request for the group's links, reporting pairs within threshold.
collision_detection::DistanceRequest CloseRequest(const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double threshold);

// Self and world distances, skipping pairs the scene always allows.
std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> Distances(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const collision_detection::DistanceRequest& request);

// Self and world pairs closer than margin, with their gradients.
ClosePairs FindClosePairs(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const collision_detection::DistanceRequest& request, double margin);

// Links rigid with the tool, plus what hangs off them.
std::set<std::string> ToolSideLinks(const moveit::core::LinkModel* tool);

// The tool side without its wrist link.
std::set<std::string> GetCameraLinks(const moveit::core::LinkModel* tool);

// True if either side of a "link - link" pair name is in links.
bool PairHasAnyLink(const std::string& pair_name, const std::set<std::string>& links);

// True if a thin rod from tool to target hits nothing.
bool ClearPathToTarget(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::LinkModel* tool, const Eigen::Vector3d& target, const std::set<std::string>& tool_side);

struct SolveSettings
{
	int iters = 60;  // closest stage steps
	double margin = 0.02;  // m kept from collisions in the closest stage
	double rot_scale = 0.1;  // m per rad in the gap
	double limit_sharpness = 20.0;  // manipulability falloff near joint limits
	bool ignore_roll = false;  // camera spin about its axis is free
	bool try_exact_first = true;
	// Weighted closest stage: one cost, each term weight x squared shortfall.
	bool use_weighted_cost = false;
	int weighted_iters = 20;
	double position_weight = 1.0;  // per m^2
	double angle_weight = 0.01;  // per rad^2
	double collision_weight = 1.0;  // per m^2 short of margin
	double camera_object_weight = 1.0;  // the same for camera-object pairs
	double self_distance_weight = 0.0;  // per m^2 short of self_distance_margin
	double self_distance_margin = 0.08;  // m
	double joint_room_weight = 1e-3;  // per joint, x s^2 / (1 - s^2)
	bool exact_only = false;  // skip the closest stage
	int exact_iters = 100;
	double exact_margin = 0.0;  // m an exact pose keeps from collisions; 0 = collision-free only
	double exact_position_tolerance = 1e-4;  // m
	double exact_rotation_tolerance = 1e-3;  // rad
};

struct Result
{
	bool found = false;
	bool exact = false;
	scene_geometry::Vector6d pose_error = scene_geometry::Vector6d::Zero();
	double gap = std::numeric_limits<double>::max();
	std::vector<double> q;  // empty if nothing found
	double weighted_cost = std::numeric_limits<double>::max();  // weighted stage only
};

// [position; rotation] error from tool to target; ignore_roll aligns only the z axis.
scene_geometry::Vector6d GetPoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool, bool ignore_roll);

// Tool Jacobian; ignore_roll drops the spin about the tool's z axis.
Eigen::MatrixXd GetToolJacobian(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const std::vector<double>& joints, bool ignore_roll);

// Exact IK if reachable, else the closest collision-free pose (priority or weighted stage).
Result Solve(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const SolveSettings& options);

}  // namespace closest_ik
