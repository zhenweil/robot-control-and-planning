#include "panda_arm_control/joint_travel.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace joint_travel
{

std::vector<int> AllJoints(int num_joints)
{
	std::vector<int> joints(static_cast<size_t>(std::max(0, num_joints)));
	std::iota(joints.begin(), joints.end(), 0);
	return joints;
}

double JointDistance(const Joints& a, const Joints& b, const std::vector<int>& joints)
{
	double d = 0.0;
	for (int j : joints)
	{
		const double diff = a[static_cast<size_t>(j)] - b[static_cast<size_t>(j)];
		d += diff * diff;
	}
	return std::sqrt(d);
}

double MaxJointTurn(const Joints& a, const Joints& b, const std::vector<int>& joints)
{
	double m = 0.0;
	for (int j : joints)
		m = std::max(m, std::abs(a[static_cast<size_t>(j)] - b[static_cast<size_t>(j)]));
	return m;
}

int CountReconfigs(const std::vector<Joints>& poses, double threshold, const std::vector<int>& joints)
{
	int count = 0;
	for (size_t k = 1; k < poses.size(); ++k)
		count += MaxJointTurn(poses[k - 1], poses[k], joints) > threshold ? 1 : 0;
	return count;
}

double EdgeCost(const Joints& a, const Joints& b, const EdgeWeights& w, const std::vector<int>& joints)
{
	return w.joint_distance * JointDistance(a, b, joints) + w.max_turn * MaxJointTurn(a, b, joints);
}

double TravelCost(const std::vector<Joints>& poses, const EdgeWeights& w, const std::vector<int>& joints)
{
	double total = 0.0;
	for (size_t k = 1; k < poses.size(); ++k)
		total += EdgeCost(poses[k - 1], poses[k], w, joints);
	return total;
}

}  // namespace joint_travel
