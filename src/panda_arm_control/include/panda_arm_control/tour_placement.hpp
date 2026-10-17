#pragma once
// Joint placement-tour step: with the viewpoint order and IK branches fixed, move the object and slide each tour pose
// along its self-motion (the 7-joint arm's one redundant direction per 6D viewpoint) to shorten the tour's joint
// distance. Each pose is linearized: q + dq_dp * object_move + self_motion * amount. One step is a QP.

#include <vector>

#include <Eigen/Dense>

namespace tour_placement
{

using Joints = std::vector<double>;

// One tour pose and how it moves: with the object (keeping the tool on its viewpoint) and along its self-motion.
struct LinearizedPose
{
	Joints q;
	Eigen::MatrixXd dq_dp;  // joints x 3
	Eigen::VectorXd self_motion;  // unit null-space direction of the tool Jacobian
};

struct StepLimits
{
	std::vector<double> joint_min, joint_max;
	double joint_margin = 0.05;  // rad kept from each joint limit
	double max_object_step = 0.02;  // m per axis
	double max_self_motion = 0.3;  // rad along each pose's self-motion
	Eigen::Vector3d free_axes = Eigen::Vector3d::Ones();  // 0 = that object axis is locked
	std::vector<bool> frozen_poses;  // per pose: true = no self-motion (empty = none frozen)
};

struct Step
{
	bool ok = false;
	Eigen::Vector3d object_move = Eigen::Vector3d::Zero();
	std::vector<double> self_motion_amounts;  // per pose
};

// Unit vector spanning the null space of a 6 x 7 tool Jacobian (the smallest right singular vector).
Eigen::VectorXd GetSelfMotionDirection(const Eigen::MatrixXd& jacobian);

// Minimizes sum over edges k of ||q_k - q_k-1||^2 / ||current edge|| (counted joints only; q_0 = start_pose, fixed),
// which approximates the summed joint distance, subject to joint limits and the step limits.
Step SolveStep(const Joints& start_pose, const std::vector<LinearizedPose>& poses, const std::vector<int>& counted_joints, const StepLimits& limits);

}  // namespace tour_placement
