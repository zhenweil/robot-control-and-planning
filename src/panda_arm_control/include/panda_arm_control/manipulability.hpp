#pragma once
// Manipulability of an arm pose, range-aware: joints near a limit count less.

#include <vector>

#include <Eigen/Dense>
#include <moveit/robot_state/robot_state.h>

namespace manipulability
{

// Per-joint room score s = 1 - exp(-k (q - lo)(hi - q) / range^2) (~1 mid-range, 0 at a limit) and ds/dq.
// k <= 0 or an unbounded joint: s = 1.
struct JointRoom
{
	Eigen::VectorXd score;
	Eigen::VectorXd slope;
};

JointRoom JointRoomScore(
	const moveit::core::JointModelGroup* group, const moveit::core::RobotModelConstPtr& model,
	const std::vector<double>& q, double k);

// 6 x dof tool Jacobian, [linear; angular] in the base frame. Sets `state` to q.
Eigen::MatrixXd ToolJacobian(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const std::vector<double>& q);

// w = sqrt(det(Js Js^T)), Js = J with each joint's column scaled by its room score (limit_sharpness 0 = plain).
double Value(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const std::vector<double>& q, double limit_sharpness);

// dw/dq.
Eigen::VectorXd Gradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const std::vector<double>& q, double limit_sharpness);

// -tau log(sum exp(-w / tau)), ~ the smallest w.
double SoftMin(const std::vector<double>& w, double tau);

// Each w's share of the soft-min's slope; the shares add up to 1.
std::vector<double> SoftMinShares(const std::vector<double>& w, double tau);

}  // namespace manipulability
