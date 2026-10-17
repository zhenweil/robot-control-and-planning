#include "panda_arm_control/ik_sampling.hpp"

#include <algorithm>
#include <memory>

namespace ik_sampling
{

std::unique_ptr<random_numbers::RandomNumberGenerator> seeded_rng;

random_numbers::RandomNumberGenerator& SeededRng()
{
	if (!seeded_rng)
		seeded_rng = std::make_unique<random_numbers::RandomNumberGenerator>(42);
	return *seeded_rng;
}

void ReseedRng(int seed)
{
	seeded_rng = std::make_unique<random_numbers::RandomNumberGenerator>(static_cast<boost::uint32_t>(std::max(1, seed)));
}

bool ExactIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity)
{
	return ExactIk(state, group, target, attempts, timeout, validity, SeededRng());
}

bool ExactIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity, random_numbers::RandomNumberGenerator& rng)
{
	if (attempts <= 0)
		return state.setFromIK(group, target, "tool0", timeout, validity);
	const double single_attempt = 1e-9;  // s: KDL stops after its first attempt (0 means the default timeout)
	for (int a = 0; a < attempts; ++a)
	{
		if (a > 0)
			state.setToRandomPositions(group, rng);
		if (state.setFromIK(group, target, "tool0", single_attempt, validity))
			return true;
	}
	return false;
}

}  // namespace ik_sampling
