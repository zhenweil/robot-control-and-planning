#include "object_placement_planning/joint_travel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

double StraightMoveTime(const Joints& a, const Joints& b, const std::vector<double>& max_velocity, const std::vector<double>& max_acceleration)
{
	// Along s in [0, 1]: the move's speed and acceleration are capped by the joint that runs out first.
	double path_velocity = std::numeric_limits<double>::infinity();
	double path_acceleration = std::numeric_limits<double>::infinity();
	for (size_t j = 0; j < a.size() && j < max_velocity.size() && j < max_acceleration.size(); ++j)
	{
		const double delta = std::abs(b[j] - a[j]);
		if (delta < 1e-12)
			continue;
		path_velocity = std::min(path_velocity, max_velocity[j] / delta);
		path_acceleration = std::min(path_acceleration, max_acceleration[j] / delta);
	}
	if (std::isinf(path_velocity))
		return 0.0;
	// Reaches full speed if speeding up and slowing down take less than the whole path.
	if (path_velocity * path_velocity / path_acceleration <= 1.0)
		return 1.0 / path_velocity + path_velocity / path_acceleration;
	return 2.0 * std::sqrt(1.0 / path_acceleration);
}

double EdgeCost(const Joints& a, const Joints& b, const EdgeWeights& w, const std::vector<int>& joints)
{
	const double time = w.time > 0.0 ? w.time * StraightMoveTime(a, b, w.max_velocity, w.max_acceleration) : 0.0;
	return w.joint_distance * JointDistance(a, b, joints) + w.max_turn * MaxJointTurn(a, b, joints) + time;
}

double TravelCost(const std::vector<Joints>& poses, const EdgeWeights& w, const std::vector<int>& joints)
{
	double total = 0.0;
	for (size_t k = 1; k < poses.size(); ++k)
		total += EdgeCost(poses[k - 1], poses[k], w, joints);
	return total;
}

}  // namespace joint_travel
