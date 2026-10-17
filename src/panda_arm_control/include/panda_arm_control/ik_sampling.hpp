#pragma once
// IK with repeatable random starts. One RNG for all random starts, reseeded before each solve: the same spot gets the
// same luck, so runs repeat and nearby spots compare fairly (MoveIt's own RNGs seed from the clock).

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>

namespace ik_sampling
{

random_numbers::RandomNumberGenerator& SeededRng();

void ReseedRng(int seed);

// attempts > 0: that many single KDL attempts (first from the current state, the rest random), a fixed amount of work
// so a seed repeats regardless of CPU speed. attempts <= 0: one time-limited call.
bool ExactIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity);

// The same with its own RNG for the random starts, so threads don't share one.
bool ExactIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity, random_numbers::RandomNumberGenerator& rng);

}  // namespace ik_sampling
