#pragma once
// Pose conversions, pose errors and planning-scene helpers shared by the object-placement code.

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

namespace scene_geometry
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Isometry3d Translation(const Eigen::Vector3d& p);

Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation);

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t);

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v);

// Sets the group to q and runs the scene's full validity check (collisions included).
bool IsStateValid(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* q);

// Moves the scene's "object" to `pose` (MOVE only updates its pose: cheap, no mesh reload).
void MoveObject(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const Eigen::Isometry3d& pose);

// [position; axis * angle] error from `tool` to `target`.
Vector6d PoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool);

// Pose gap in meters, rotation weighed by rot_scale (m per rad).
double PoseGap(const Vector6d& e, double rot_scale);

// J^T (J J^T + damping^2 I)^-1
Eigen::MatrixXd DampedPinv(const Eigen::MatrixXd& J, double damping);

}  // namespace scene_geometry
