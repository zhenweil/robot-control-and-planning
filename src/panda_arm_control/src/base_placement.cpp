#include "panda_arm_control/base_placement.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <thread>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <json/json.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace
{

struct BasePose2D
{
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
};

Eigen::Isometry3d MakeBaseTransform(const BasePose2D& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = Eigen::Vector3d(p.x, p.y, 0.0);
	t.linear() = Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	return t;
}

// Eigen::Translation3d * Matrix3d yields an Affine3d (not Isometry3d, even for a rotation
// matrix), so build the Isometry directly instead of relying on operator* here.
Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = translation;
	t.linear() = rotation;
	return t;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

// Mirrors real_cost_planning.cpp's private helper of the same name/signature.
bool IsStateCollisionFree(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* joint_positions)
{
	state->setJointGroupPositions(group, joint_positions);
	state->update();

	planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
	return locked_scene->isStateValid(*state, group->getName());
}

// MOVE only updates the registered object's pose -- cheap (no mesh reload), unlike rebuilding the
// whole scene via BuildLocalCollisionScene for every candidate base pose.
void SetObjectPose(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;

	planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
	locked_scene->processCollisionObjectMsg(obj);
}

double WrapAngle(double a)
{
	while (a > M_PI)
		a -= 2.0 * M_PI;
	while (a < -M_PI)
		a += 2.0 * M_PI;
	return a;
}

double RandomUniform(std::mt19937& rng, double lo, double hi)
{
	std::uniform_real_distribution<double> dist(lo, hi);
	return dist(rng);
}

BasePose2D RandomInBounds(const BasePlacementBounds& b, std::mt19937& rng)
{
	BasePose2D p;
	p.x = RandomUniform(rng, b.x_min, b.x_max);
	p.y = RandomUniform(rng, b.y_min, b.y_max);
	p.yaw = RandomUniform(rng, b.yaw_min, b.yaw_max);
	return p;
}

bool IsFullYawRange(const BasePlacementBounds& b)
{
	return (b.yaw_max - b.yaw_min) >= 2.0 * M_PI - 1e-6;
}

BasePose2D RandomNear(
	const BasePose2D& center, double radius_xy, double radius_yaw, const BasePlacementBounds& b, std::mt19937& rng)
{
	BasePose2D p;
	p.x = std::clamp(center.x + RandomUniform(rng, -radius_xy, radius_xy), b.x_min, b.x_max);
	p.y = std::clamp(center.y + RandomUniform(rng, -radius_xy, radius_xy), b.y_min, b.y_max);
	double yaw = WrapAngle(center.yaw + RandomUniform(rng, -radius_yaw, radius_yaw));
	if (!IsFullYawRange(b))
		yaw = std::clamp(yaw, b.yaw_min, b.yaw_max);
	p.yaw = yaw;
	return p;
}

double PoseDistanceToMean(const BasePose2D& a, const BasePose2D& mean)
{
	double dx = a.x - mean.x;
	double dy = a.y - mean.y;
	// Rotation contributes little to reach/collision compared to translation on this scale --
	// weight yaw (radians) roughly like 0.1 m per radian so it still matters for convergence.
	return std::sqrt(dx * dx + dy * dy) + 0.1 * std::abs(WrapAngle(a.yaw - mean.yaw));
}

BasePose2D CircularMean(const std::vector<BasePose2D>& poses, const std::vector<bool>& placed)
{
	double sx = 0.0, sy = 0.0, ssin = 0.0, scos = 0.0;
	int n = 0;
	for (size_t i = 0; i < poses.size(); ++i)
	{
		if (!placed[i])
			continue;
		sx += poses[i].x;
		sy += poses[i].y;
		ssin += std::sin(poses[i].yaw);
		scos += std::cos(poses[i].yaw);
		++n;
	}
	BasePose2D mean;
	if (n == 0)
		return mean;
	mean.x = sx / n;
	mean.y = sy / n;
	mean.yaw = std::atan2(ssin, scos);
	return mean;
}

// One IK attempt at a single point, given a candidate base pose and a joint seed -- moves the
// registered object into the candidate base's frame, tries the seeded joint values first, then a
// few random-restart retries (redundant IK: a different elbow/wrist configuration may still
// reach it even if the seeded one can't).
bool TryReachPoint(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const Eigen::Isometry3d& target_pose_original,
	const BasePose2D& base_pose, const std::vector<double>& seed_joints, double ik_timeout, int ik_retries,
	std::vector<double>* out_joints)
{
	Eigen::Isometry3d base_transform_inv = MakeBaseTransform(base_pose).inverse();
	SetObjectPose(planning_scene_monitor, base_transform_inv * object_pose_original);
	geometry_msgs::msg::Pose target_local = ToPoseMsg(base_transform_inv * target_pose_original);

	auto validity_callback = [&planning_scene_monitor](
								  moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								  const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	state.setJointGroupPositions(jmg, seed_joints);
	bool ok = state.setFromIK(jmg, target_local, "tool0", ik_timeout, validity_callback);

	for (int attempt = 0; !ok && attempt < ik_retries; ++attempt)
	{
		state.setToRandomPositions(jmg);
		ok = state.setFromIK(jmg, target_local, "tool0", ik_timeout, validity_callback);
	}

	if (ok)
		state.copyJointGroupPositions(jmg, *out_joints);
	return ok;
}

double JointL1Distance(const std::vector<double>& a, const std::vector<double>& b)
{
	double d = 0.0;
	for (size_t k = 0; k < a.size() && k < b.size(); ++k)
		d += std::abs(a[k] - b[k]);
	return d;
}

// Publishes each point's current individual base-pose guess, the running mean, a circle for the
// current search radius, and a trail through every past mean this restart has visited, so the
// refinement loop's convergence -- and how the mean itself moved to get there -- can be watched
// live in RViz. No-op if params.progress_pub isn't set. mean_history.back() is the current mean.
void PublishProgress(
	const rclcpp::Node::SharedPtr& node, const BasePlacementParams& params,
	const std::vector<BasePose2D>& placements, const std::vector<bool>& placed,
	const std::vector<BasePose2D>& mean_history, double radius_xy)
{
	if (!params.progress_pub)
		return;

	const BasePose2D& mean = mean_history.back();
	rclcpp::Time stamp = node->now();
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	for (size_t i = 0; i < placements.size(); ++i)
	{
		visualization_msgs::msg::Marker dot;
		dot.header.frame_id = "world";
		dot.header.stamp = stamp;
		dot.ns = "base_placement_progress_points";
		dot.id = id++;
		dot.type = visualization_msgs::msg::Marker::SPHERE;
		dot.action = visualization_msgs::msg::Marker::ADD;
		dot.pose.position.x = placements[i].x;
		dot.pose.position.y = placements[i].y;
		dot.pose.position.z = 0.01;
		dot.pose.orientation.w = 1.0;
		dot.scale.x = dot.scale.y = dot.scale.z = 0.02;
		bool p = i < placed.size() && placed[i];
		dot.color.r = p ? 0.1f : 0.8f;
		dot.color.g = p ? 0.6f : 0.2f;
		dot.color.b = 0.9f;
		dot.color.a = 0.9f;
		markers.markers.push_back(dot);
	}

	visualization_msgs::msg::Marker mean_marker;
	mean_marker.header.frame_id = "world";
	mean_marker.header.stamp = stamp;
	mean_marker.ns = "base_placement_progress_mean";
	mean_marker.id = id++;
	mean_marker.type = visualization_msgs::msg::Marker::CYLINDER;
	mean_marker.action = visualization_msgs::msg::Marker::ADD;
	mean_marker.pose.position.x = mean.x;
	mean_marker.pose.position.y = mean.y;
	mean_marker.pose.position.z = 0.015;
	Eigen::Quaterniond mean_quat(Eigen::AngleAxisd(mean.yaw, Eigen::Vector3d::UnitZ()));
	mean_marker.pose.orientation.x = mean_quat.x();
	mean_marker.pose.orientation.y = mean_quat.y();
	mean_marker.pose.orientation.z = mean_quat.z();
	mean_marker.pose.orientation.w = mean_quat.w();
	mean_marker.scale.x = mean_marker.scale.y = 0.05;
	mean_marker.scale.z = 0.03;
	mean_marker.color.r = 1.0f;
	mean_marker.color.g = 0.85f;
	mean_marker.color.b = 0.0f;
	mean_marker.color.a = 1.0f;
	markers.markers.push_back(mean_marker);

	visualization_msgs::msg::Marker radius_circle;
	radius_circle.header.frame_id = "world";
	radius_circle.header.stamp = stamp;
	radius_circle.ns = "base_placement_progress_radius";
	radius_circle.id = id++;
	radius_circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
	radius_circle.action = visualization_msgs::msg::Marker::ADD;
	radius_circle.pose.orientation.w = 1.0;
	radius_circle.scale.x = 0.003;
	radius_circle.color.r = 1.0f;
	radius_circle.color.g = 0.85f;
	radius_circle.color.b = 0.0f;
	radius_circle.color.a = 0.6f;
	constexpr int kCircleSegments = 32;
	for (int k = 0; k <= kCircleSegments; ++k)
	{
		double angle = 2.0 * M_PI * k / kCircleSegments;
		geometry_msgs::msg::Point pt;
		pt.x = mean.x + radius_xy * std::cos(angle);
		pt.y = mean.y + radius_xy * std::sin(angle);
		pt.z = 0.01;
		radius_circle.points.push_back(pt);
	}
	markers.markers.push_back(radius_circle);

	if (mean_history.size() > 1)
	{
		visualization_msgs::msg::Marker trail;
		trail.header.frame_id = "world";
		trail.header.stamp = stamp;
		trail.ns = "base_placement_progress_mean_trail";
		trail.id = id++;
		trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
		trail.action = visualization_msgs::msg::Marker::ADD;
		trail.pose.orientation.w = 1.0;
		trail.scale.x = 0.004;
		trail.color.r = 1.0f;
		trail.color.g = 0.4f;
		trail.color.b = 0.1f;
		trail.color.a = 0.8f;
		for (const BasePose2D& past_mean : mean_history)
		{
			geometry_msgs::msg::Point pt;
			pt.x = past_mean.x;
			pt.y = past_mean.y;
			pt.z = 0.012;
			trail.points.push_back(pt);
		}
		markers.markers.push_back(trail);
	}

	// One small breadcrumb sphere per past mean (excluding the current one, already drawn as the
	// big yellow cylinder above) so each iteration's move is individually visible, not just the
	// connecting line.
	for (size_t h = 0; h + 1 < mean_history.size(); ++h)
	{
		visualization_msgs::msg::Marker breadcrumb;
		breadcrumb.header.frame_id = "world";
		breadcrumb.header.stamp = stamp;
		breadcrumb.ns = "base_placement_progress_mean_history";
		breadcrumb.id = id++;
		breadcrumb.type = visualization_msgs::msg::Marker::SPHERE;
		breadcrumb.action = visualization_msgs::msg::Marker::ADD;
		breadcrumb.pose.position.x = mean_history[h].x;
		breadcrumb.pose.position.y = mean_history[h].y;
		breadcrumb.pose.position.z = 0.012;
		breadcrumb.pose.orientation.w = 1.0;
		breadcrumb.scale.x = breadcrumb.scale.y = breadcrumb.scale.z = 0.025;
		breadcrumb.color.r = 1.0f;
		breadcrumb.color.g = 0.4f;
		breadcrumb.color.b = 0.1f;
		breadcrumb.color.a = 0.7f;
		markers.markers.push_back(breadcrumb);
	}

	params.progress_pub->publish(markers);

	if (params.visualize_progress_delay_sec > 0.0)
		std::this_thread::sleep_for(std::chrono::duration<double>(params.visualize_progress_delay_sec));
}

// One full attempt of the outer relaxation/tightening scheme (steps 1-3 of the header doc),
// from a single random initialization.
BasePlacementResult RunOneRestart(
	const rclcpp::Node::SharedPtr& node, int restart_number, int num_restarts, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints, const BasePlacementParams& params, std::mt19937& rng)
{
	const size_t n = tour_tcp_poses_original.size();
	std::vector<BasePose2D> placements(n);
	std::vector<std::vector<double>> joints(n);
	std::vector<bool> placed(n, false);

	// Step 1: mobile-base relaxation -- each point independently picks any reachable base pose.
	{
		std::vector<double> seed = start_reference_joints;
		for (size_t i = 0; i < n; ++i)
		{
			bool found = false;
			for (int attempt = 0; attempt < params.candidates_per_point && !found; ++attempt)
			{
				BasePose2D candidate = RandomInBounds(params.bounds, rng);
				std::vector<double> sol;
				if (TryReachPoint(
						state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i],
						candidate, seed, params.ik_timeout, params.ik_retries_per_point, &sol))
				{
					placements[i] = candidate;
					joints[i] = sol;
					placed[i] = true;
					seed = sol;
					found = true;
				}
			}
			if (!found)
				joints[i] = seed;  // carry the seed forward unchanged so later points still have one
		}

		int num_placed = std::count(placed.begin(), placed.end(), true);
		RCLCPP_DEBUG(
			node->get_logger(),
			"[restart %d/%d] STEP 1/3 (relaxation, no optimization yet): each point independently "
			"searched for any reachable base pose -- %d/%zu succeeded",
			restart_number, num_restarts, num_placed, n);
	}

	// Step 2: progressive tightening -- re-search each point within a shrinking radius around the
	// running mean, until every placed point converges to (near) the mean.
	BasePose2D mean = CircularMean(placements, placed);
	std::vector<BasePose2D> mean_history = {mean};  // every mean this restart has visited, in order
	double radius_xy = 0.5 * std::max(params.bounds.x_max - params.bounds.x_min, params.bounds.y_max - params.bounds.y_min);
	double radius_yaw = std::min(M_PI, 0.5 * (params.bounds.yaw_max - params.bounds.yaw_min));

	RCLCPP_DEBUG(
		node->get_logger(),
		"[restart %d/%d] STEP 2/3 (tightening) starting point: averaging step 1's %zu independent "
		"guesses gives mean=(%.4f, %.4f, %.4f rad) -- each iteration below re-searches every point "
		"near this mean and shrinks the search radius, which IS the optimization loop",
		restart_number, num_restarts, n, mean.x, mean.y, mean.yaw);

	PublishProgress(node, params, placements, placed, mean_history, radius_xy);

	for (int outer_iter = 0; outer_iter < params.max_outer_iterations; ++outer_iter)
	{
		std::vector<double> seed = start_reference_joints;
		for (size_t i = 0; i < n; ++i)
		{
			std::vector<double> sol;
			bool found = TryReachPoint(
				state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i], mean, seed,
				params.ik_timeout, params.ik_retries_per_point, &sol);
			BasePose2D best = mean;

			if (!found)
			{
				double best_dist = std::numeric_limits<double>::max();
				for (int attempt = 0; attempt < params.candidates_per_point; ++attempt)
				{
					BasePose2D candidate = RandomNear(mean, radius_xy, radius_yaw, params.bounds, rng);
					std::vector<double> candidate_sol;
					if (!TryReachPoint(
							state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i],
							candidate, seed, params.ik_timeout, params.ik_retries_per_point, &candidate_sol))
						continue;
					double dist = PoseDistanceToMean(candidate, mean);
					if (dist < best_dist)
					{
						best_dist = dist;
						best = candidate;
						sol = candidate_sol;
						found = true;
					}
				}
			}

			// Full-bounds rescue: this point has no feasible pose near the shrunk mean this
			// round -- widen back to global bounds rather than lose an otherwise-reachable point.
			if (!found)
			{
				double best_dist = std::numeric_limits<double>::max();
				for (int attempt = 0; attempt < params.candidates_per_point; ++attempt)
				{
					BasePose2D candidate = RandomInBounds(params.bounds, rng);
					std::vector<double> candidate_sol;
					if (!TryReachPoint(
							state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i],
							candidate, seed, params.ik_timeout, params.ik_retries_per_point, &candidate_sol))
						continue;
					double dist = PoseDistanceToMean(candidate, mean);
					if (dist < best_dist)
					{
						best_dist = dist;
						best = candidate;
						sol = candidate_sol;
						found = true;
					}
				}
			}

			if (found)
			{
				placements[i] = best;
				joints[i] = sol;
				placed[i] = true;
				seed = sol;
			}
			else
			{
				// Keep whatever this point had from the previous round rather than discarding it.
				seed = joints[i];
			}
		}

		BasePose2D new_mean = CircularMean(placements, placed);
		double moved_x = new_mean.x - mean.x;
		double moved_y = new_mean.y - mean.y;
		double moved_yaw = WrapAngle(new_mean.yaw - mean.yaw);
		double moved_dist = std::sqrt(moved_x * moved_x + moved_y * moved_y);
		mean = new_mean;
		mean_history.push_back(mean);
		radius_xy *= params.radius_shrink_factor;
		radius_yaw *= params.radius_shrink_factor;

		int num_converged = 0;
		int num_placed = 0;
		for (size_t i = 0; i < n; ++i)
		{
			if (!placed[i])
				continue;
			++num_placed;
			double dx = std::abs(placements[i].x - mean.x);
			double dy = std::abs(placements[i].y - mean.y);
			double dyaw = std::abs(WrapAngle(placements[i].yaw - mean.yaw));
			if (dx <= params.convergence_tolerance_xy && dy <= params.convergence_tolerance_xy &&
				dyaw <= params.convergence_tolerance_yaw)
				++num_converged;
		}

		bool iteration_converged = (num_converged == num_placed);
		RCLCPP_INFO(
			node->get_logger(),
			"  iteration %d/%d: base moved %.4f m (dx=%+.4f, dy=%+.4f, dyaw=%+.4f rad) -> now at "
			"(%.4f, %.4f, %.4f rad)%s",
			outer_iter + 1, params.max_outer_iterations, moved_dist, moved_x, moved_y, moved_yaw, mean.x, mean.y,
			mean.yaw, iteration_converged ? "  [settled, stopping]" : "");

		RCLCPP_DEBUG(
			node->get_logger(),
			"[restart %d/%d] iteration %d/%d detail: next search radius %.4f m/%.4f rad; %d/%zu points "
			"placed, %d/%d agree with mean",
			restart_number, num_restarts, outer_iter + 1, params.max_outer_iterations, radius_xy, radius_yaw,
			num_placed, n, num_converged, num_placed);

		PublishProgress(node, params, placements, placed, mean_history, radius_xy);

		if (iteration_converged)
			break;
	}

	// STEP 3/3 (finalize): fix the base at the converged mean and re-solve the whole sequence in
	// order, seeded point-to-point exactly as it will actually be executed.
	BasePlacementResult result;
	result.x = mean.x;
	result.y = mean.y;
	result.yaw = mean.yaw;
	result.num_total = static_cast<int>(n);
	result.joint_solutions.resize(n);

	std::vector<double> seed = start_reference_joints;
	const std::vector<double>* prev_joints = &start_reference_joints;
	for (size_t i = 0; i < n; ++i)
	{
		std::vector<double> sol;
		bool found = TryReachPoint(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original[i], mean, seed,
			params.ik_timeout, params.ik_retries_per_point * 3, &sol);
		if (found)
		{
			result.joint_solutions[i] = sol;
			result.num_reachable++;
			result.total_joint_path_length += JointL1Distance(sol, *prev_joints);
			prev_joints = &result.joint_solutions[i];
			seed = sol;
		}
		else
		{
			result.joint_solutions[i].clear();
			// seed/prev_joints unchanged -- next point still measures from the last reachable one.
		}
	}
	result.ok = (result.num_reachable == result.num_total);

	return result;
}

} // namespace

BasePlacementResult SolveBasePlacement(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params)
{
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);

	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);

	BasePlacementResult best;
	best.num_total = static_cast<int>(tour_tcp_poses_original.size());

	for (int restart = 0; restart < params.num_restarts; ++restart)
	{
		if (!rclcpp::ok())
			break;

		std::mt19937 rng(static_cast<unsigned int>(params.random_seed + restart));
		BasePlacementResult candidate = RunOneRestart(
			node, restart + 1, params.num_restarts, state, jmg, planning_scene_monitor, object_pose_original,
			tour_tcp_poses_original, start_reference_joints, params, rng);

		RCLCPP_DEBUG(
			node->get_logger(),
			"[restart %d/%d] STEP 3/3 (finalize) RESULT: base=(%.4f, %.4f, %.4f rad), reachable=%d/%d, "
			"total_joint_path_length=%.4f",
			restart + 1, params.num_restarts, candidate.x, candidate.y, candidate.yaw, candidate.num_reachable,
			candidate.num_total, candidate.total_joint_path_length);

		bool better = candidate.num_reachable > best.num_reachable ||
			(candidate.num_reachable == best.num_reachable &&
			 candidate.total_joint_path_length < best.total_joint_path_length);
		if (restart == 0 || better)
			best = candidate;

		if (best.ok)
			break;  // full reachability already achieved -- later restarts only chase path length
	}

	// Leave the scene as we found it -- callers (e.g. subsequent execution) expect the object at
	// its real, original pose unless they explicitly move it themselves.
	SetObjectPose(planning_scene_monitor, object_pose_original);

	if (best.ok)
	{
		RCLCPP_INFO(
			node->get_logger(), "Done. Best base position: x=%.4f, y=%.4f, yaw=%.4f rad -- reaches all %d viewpoints.",
			best.x, best.y, best.yaw, best.num_total);
	}
	else
	{
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Best base position found: x=%.4f, y=%.4f, yaw=%.4f rad -- only reaches %d/%d viewpoints "
			"(could not find one base position that reaches all of them).",
			best.x, best.y, best.yaw, best.num_reachable, best.num_total);
	}

	return best;
}

void ApplyBasePlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original, double x,
	double y, double yaw)
{
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	Eigen::Isometry3d base_transform_inv = MakeBaseTransform(BasePose2D{x, y, yaw}).inverse();
	SetObjectPose(planning_scene_monitor, base_transform_inv * object_pose_original);
}

void ExportBasePlacementResult(const std::string& output_dir, const BasePlacementResult& result)
{
	std::filesystem::create_directories(output_dir);

	Json::Value root;
	root["ok"] = result.ok;
	root["num_reachable"] = result.num_reachable;
	root["num_total"] = result.num_total;
	root["x"] = result.x;
	root["y"] = result.y;
	root["yaw"] = result.yaw;
	root["total_joint_path_length"] = result.total_joint_path_length;

	Json::Value joints(Json::arrayValue);
	for (const auto& sol : result.joint_solutions)
	{
		Json::Value entry(Json::arrayValue);
		for (double v : sol)
			entry.append(v);
		joints.append(entry);
	}
	root["joint_solutions"] = joints;

	std::string json_path = output_dir + "/base_placement_result.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved base placement result JSON: %s\n", json_path.c_str());
}

visualization_msgs::msg::MarkerArray BuildBasePlacementMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const BasePlacementResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	BasePose2D base_pose{result.x, result.y, result.yaw};
	Eigen::Isometry3d base_transform_inv = MakeBaseTransform(base_pose).inverse();
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	Eigen::Isometry3d object_pose_local = base_transform_inv * object_pose_original;

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "recommended_base_object";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose = ToPoseMsg(object_pose_local);
	mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = mesh_scale;
	mesh_marker.color.r = 0.7f;
	mesh_marker.color.g = 0.7f;
	mesh_marker.color.b = 0.7f;
	mesh_marker.color.a = 0.5f;
	markers.markers.push_back(mesh_marker);

	visualization_msgs::msg::Marker line;
	line.header.frame_id = "world";
	line.header.stamp = stamp;
	line.ns = "recommended_base_tour";
	line.id = id++;
	line.type = visualization_msgs::msg::Marker::LINE_STRIP;
	line.action = visualization_msgs::msg::Marker::ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.002;
	line.color.r = 0.1f;
	line.color.g = 0.9f;
	line.color.b = 0.1f;
	line.color.a = 0.9f;

	for (size_t i = 0; i < tour_tcp_poses_original.size(); ++i)
	{
		Eigen::Isometry3d local = base_transform_inv * tour_tcp_poses_original[i];

		visualization_msgs::msg::Marker sphere;
		sphere.header.frame_id = "world";
		sphere.header.stamp = stamp;
		sphere.ns = "recommended_base_waypoints";
		sphere.id = id++;
		sphere.type = visualization_msgs::msg::Marker::SPHERE;
		sphere.action = visualization_msgs::msg::Marker::ADD;
		sphere.pose.position.x = local.translation().x();
		sphere.pose.position.y = local.translation().y();
		sphere.pose.position.z = local.translation().z();
		sphere.pose.orientation.w = 1.0;
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		bool reachable = i < result.joint_solutions.size() && !result.joint_solutions[i].empty();
		sphere.color.r = reachable ? 0.1f : 0.9f;
		sphere.color.g = reachable ? 0.9f : 0.1f;
		sphere.color.b = 0.1f;
		sphere.color.a = 1.0f;
		markers.markers.push_back(sphere);

		line.points.push_back(sphere.pose.position);
	}
	markers.markers.push_back(line);

	return markers;
}
