#pragma once
// Joint-space travel between arm poses. Only the joints listed in `joints` (indices) count.

#include <vector>

namespace joint_travel
{

using Joints = std::vector<double>;

// Weights of one tour edge's cost.
struct EdgeWeights
{
	double joint_distance = 1.0;  // per rad of joint distance (L2)
	double max_turn = 1.0;	// per rad of the largest single-joint turn
	double time = 0.0;  // per second of a straight move that stops at both ends; needs the limits below
	std::vector<double> max_velocity;  // rad/s, every joint
	std::vector<double> max_acceleration;  // rad/s^2, every joint
};

// Indices 0 .. num_joints - 1.
std::vector<int> AllJoints(int num_joints);

// L2 joint distance.
double JointDistance(const Joints& a, const Joints& b, const std::vector<int>& joints);

// Largest single-joint turn.
double MaxJointTurn(const Joints& a, const Joints& b, const std::vector<int>& joints);

// Moves poses[k-1] -> poses[k] that turn any joint more than threshold.
int CountReconfigs(const std::vector<Joints>& poses, double threshold, const std::vector<int>& joints);

// Time of a straight joint-space move from rest to rest, all joints in step, under each joint's speed and acceleration
// limits (trapezoidal profile). Every joint counts: the arm stops for all of them.
double StraightMoveTime(const Joints& a, const Joints& b, const std::vector<double>& max_velocity, const std::vector<double>& max_acceleration);

// One edge's cost.
double EdgeCost(const Joints& a, const Joints& b, const EdgeWeights& w, const std::vector<int>& joints);

// Sum of edge costs poses[k-1] -> poses[k].
double TravelCost(const std::vector<Joints>& poses, const EdgeWeights& w, const std::vector<int>& joints);

}  // namespace joint_travel
