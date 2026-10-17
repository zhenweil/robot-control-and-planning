#include "object_placement_planning/closest_ik.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <geometric_shapes/shapes.h>

#include "object_placement_planning/manipulability.hpp"

namespace closest_ik
{

ClosePairs ClosePairGradients(
	const collision_detection::DistanceResult& res, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, double margin)
{
	ClosePairs pairs;
	for (const auto& entry : res.distances)
		for (const auto& d : entry.second)
		{
			if (d.distance >= margin)
				continue;
			Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(group->getVariableCount());
			Eigen::Vector3d object_slope = Eigen::Vector3d::Zero();
			bool robot_side = false;
			for (int side = 0; side < 2; ++side)
			{
				// distance ~ distance0 + normal.(dp1 - dp0)
				const double sign = side == 0 ? -1.0 : 1.0;
				if (d.body_types[side] == collision_detection::BodyType::WORLD_OBJECT && d.link_names[side] == "object")
					object_slope += sign * d.normal;
				if (d.body_types[side] != collision_detection::BodyType::ROBOT_LINK &&
					d.body_types[side] != collision_detection::BodyType::ROBOT_ATTACHED)
					continue;
				const moveit::core::LinkModel* link = state.getRobotModel()->getLinkModel(d.link_names[side]);
				if (!link || !group->isLinkUpdated(link->getName()))
					continue;  // the group's joints don't move it; asking for its Jacobian logs an error
				const Eigen::Vector3d local = state.getGlobalLinkTransform(link).inverse() * d.nearest_points[side];
				Eigen::MatrixXd J;
				if (!state.getJacobian(group, link, local, J))
					continue;
				row += sign * (d.normal.transpose() * J.topRows<3>());
				robot_side = true;
			}
			if (robot_side && row.norm() > 1e-9)
			{
				pairs.gradients.push_back(row);
				pairs.dists.push_back(d.distance);
				pairs.object_slopes.push_back(object_slope);
				pairs.names.push_back(d.link_names[0] + " - " + d.link_names[1]);
			}
		}
	return pairs;
}

collision_detection::DistanceRequest CloseRequest(
	const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double threshold)
{
	collision_detection::DistanceRequest req;
	req.enable_nearest_points = true;
	req.enable_signed_distance = true;
	req.type = collision_detection::DistanceRequestType::ALL;
	req.group_name = group->getName();
	req.enableGroup(state.getRobotModel());
	req.distance_threshold = threshold;
	req.max_contacts_per_body = 2;
	return req;
}

std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> Distances(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state,
	const collision_detection::DistanceRequest& req)
{
	std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> res;
	collision_detection::DistanceRequest request = req;
	planning_scene_monitor::LockedPlanningSceneRO locked(scene);
	request.acm = &locked->getAllowedCollisionMatrix();
	request.enableGroup(locked->getRobotModel());  // MoveIt matches active links by pointer, not name
	locked->getCollisionEnv()->distanceSelf(request, res.first, state);
	locked->getCollisionEnv()->distanceRobot(request, res.second, state);
	return res;
}

ClosePairs FindClosePairs(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, const collision_detection::DistanceRequest& req, double margin)
{
	const auto [self_res, world_res] = Distances(scene, state, req);
	ClosePairs pairs = ClosePairGradients(self_res, state, group, margin);
	const ClosePairs world = ClosePairGradients(world_res, state, group, margin);
	pairs.gradients.insert(pairs.gradients.end(), world.gradients.begin(), world.gradients.end());
	pairs.dists.insert(pairs.dists.end(), world.dists.begin(), world.dists.end());
	pairs.object_slopes.insert(pairs.object_slopes.end(), world.object_slopes.begin(), world.object_slopes.end());
	pairs.names.insert(pairs.names.end(), world.names.begin(), world.names.end());
	return pairs;
}

// Links rigid with the tool, plus what hangs off them (hand, fingers).
std::set<std::string> ToolSideLinks(const moveit::core::LinkModel* tool)
{
	std::set<std::string> links;
	const moveit::core::LinkModel* wrist = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tool);
	links.insert(wrist->getName());
	for (const moveit::core::JointModel* joint : wrist->getChildJointModels())
		for (const moveit::core::LinkModel* link : joint->getDescendantLinkModels())
			links.insert(link->getName());
	return links;
}

// True if a 1 cm rod from the tool to the target hits nothing (tool side excepted): a closest pose with something in
// between can't slide onto the viewpoint, so its gap and slope would mislead.
bool ClearPathToTarget(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::LinkModel* tool, const Eigen::Vector3d& target, const std::set<std::string>& tool_side)
{
	const Eigen::Vector3d local = state.getGlobalLinkTransform(tool).inverse() * target;
	const double length = local.norm();
	if (length < 0.01)
		return true;
	Eigen::Isometry3d rod = Eigen::Isometry3d::Identity();
	rod.translation() = 0.5 * local;  // cylinder axis is z, centered on its pose
	rod.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), local / length).toRotationMatrix();
	const std::string id = "path_to_target";
	state.attachBody(
		id, Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Cylinder>(0.005, length)}, {rod}, tool_side,
		tool->getName());
	state.update();
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked(scene);
		locked->checkCollision(req, res, state);
	}
	state.clearAttachedBody(id);
	return !res.collision;
}

scene_geometry::Vector6d GetPoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool, bool ignore_roll)
{
	if (!ignore_roll)
		return scene_geometry::PoseError(target, tool);
	const Eigen::Vector3d tool_axis = tool.linear().col(2);
	const Eigen::Vector3d target_axis = target.linear().col(2);
	const Eigen::Vector3d cross = tool_axis.cross(target_axis);
	const double angle = std::atan2(cross.norm(), tool_axis.dot(target_axis));
	scene_geometry::Vector6d e;
	e.head<3>() = target.translation() - tool.translation();
	e.tail<3>() = cross.norm() > 1e-12 ? Eigen::Vector3d(cross.normalized() * angle) : Eigen::Vector3d(tool.linear().col(0) * angle);  // axes opposite: turn about any perpendicular
	return e;
}

namespace
{

// Tool Jacobian; with ignore_roll the angular rows drop the spin about the tool's z axis, so roll is unconstrained.
Eigen::MatrixXd GetToolJacobian(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const std::vector<double>& q, bool ignore_roll)
{
	Eigen::MatrixXd J = manipulability::ToolJacobian(state, group, tool, q);
	if (ignore_roll)
	{
		const Eigen::Vector3d axis = state.getGlobalLinkTransform(tool).linear().col(2);
		J.bottomRows<3>() = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * J.bottomRows<3>();
	}
	return J;
}

bool IsWithinExactTolerance(const scene_geometry::Vector6d& pose_error, const Options& options)
{
	return pose_error.head<3>().norm() < options.exact_position_tolerance && pose_error.tail<3>().norm() < options.exact_rotation_tolerance;
}

// Exact stage, like KDL: full damped Newton steps on the pose error, no collision terms; one validity check once within
// the tolerances. Not found if it doesn't converge or the pose is invalid.
Result SolveExact(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const Options& options)
{
	const double damping = 1e-3;
	const double max_joint_step = 0.5;	// rad
	std::vector<double> q = start;
	for (int it = 0; it < options.exact_iters; ++it)
	{
		state.setJointGroupPositions(group, q);
		state.update();
		const scene_geometry::Vector6d pose_error = GetPoseError(target, state.getGlobalLinkTransform(tool), options.ignore_roll);
		if (IsWithinExactTolerance(pose_error, options))
		{
			if (!scene_geometry::IsStateValid(scene, &state, group, q.data()))
				return {};
			if (options.exact_margin > 0.0 && !FindClosePairs(scene, state, group, CloseRequest(state, group, options.exact_margin), options.exact_margin).dists.empty())
				return {};
			return {true, true, pose_error, scene_geometry::PoseGap(pose_error, options.rot_scale), q};
		}
		Eigen::VectorXd dq = scene_geometry::DampedPinv(GetToolJacobian(state, group, tool, q, options.ignore_roll), damping) * pose_error;
		const double step_max = dq.cwiseAbs().maxCoeff();
		if (step_max > max_joint_step)
			dq *= max_joint_step / step_max;
		for (size_t k = 0; k < q.size(); ++k)
			q[k] += dq(static_cast<Eigen::Index>(k));
		state.setJointGroupPositions(group, q);
		state.enforceBounds(group);
		state.copyJointGroupPositions(group, q);
	}
	return {};
}

}  // namespace

Result Solve(
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target,
	const std::vector<double>& start, const Options& options)
{
	if (options.try_exact_first || options.exact_only)
	{
		const Result exact = SolveExact(scene, state, group, tool, target, start, options);
		if (exact.found || options.exact_only)
			return exact;
	}
	const int iters = options.iters;
	const double margin = options.margin;
	const double rot_scale = options.rot_scale;
	const double limit_sharpness = options.limit_sharpness;
	const double damping = 0.01;
	const double max_joint_step = 0.1;
	const double manip_step = 0.02;  // rad per iteration up the manipulability
	const int stall_window = 5;		 // stop once the best gap improves < 1 mm over this many steps
	const double stall_gain = 1e-3;
	const int dof = static_cast<int>(group->getVariableCount());
	const collision_detection::DistanceRequest req = CloseRequest(state, group, margin);

	std::vector<double> lower(static_cast<size_t>(dof)), upper(static_cast<size_t>(dof));
	for (size_t k = 0; k < lower.size(); ++k)
	{
		const moveit::core::VariableBounds& b = state.getRobotModel()->getVariableBounds(group->getVariableNames()[k]);
		lower[k] = b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::max();
		upper[k] = b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::max();
	}

	const std::set<std::string> tool_side = ToolSideLinks(tool);
	std::vector<double> q = start;
	Result best;
	std::vector<double> best_gap_history;
	for (int it = 0; it <= iters; ++it)
	{
		state.setJointGroupPositions(group, q);
		state.update();
		const scene_geometry::Vector6d pose_error = GetPoseError(target, state.getGlobalLinkTransform(tool), options.ignore_roll);
		const double gap = scene_geometry::PoseGap(pose_error, rot_scale);
		const ClosePairs pairs = FindClosePairs(scene, state, group, req, margin);
		// The full validity check, not just the distance query, which lets self-collisions near the base through.
		if (gap < best.gap && scene_geometry::IsStateValid(scene, &state, group, q.data()) &&
			ClearPathToTarget(scene, state, tool, target.translation(), tool_side))
		{
			best = {true, IsWithinExactTolerance(pose_error, options), pose_error, gap, q};
			if (gap < 1e-6)
				break;
		}
		if (it == iters)
			break;
		best_gap_history.push_back(std::min(gap, best_gap_history.empty() ? gap : best_gap_history.back()));
		if (static_cast<int>(best_gap_history.size()) > stall_window &&
			best_gap_history[best_gap_history.size() - 1 - stall_window] - best_gap_history.back() < stall_gain)
			break;

		const Eigen::MatrixXd J = GetToolJacobian(state, group, tool, q, options.ignore_roll);
		const auto num_pairs = static_cast<Eigen::Index>(pairs.gradients.size());
		Eigen::MatrixXd Jc(num_pairs, dof);
		Eigen::VectorXd push(num_pairs);
		for (Eigen::Index k = 0; k < num_pairs; ++k)
		{
			Jc.row(k) = pairs.gradients[static_cast<size_t>(k)];
			push(k) = margin - pairs.dists[static_cast<size_t>(k)];
		}
		const Eigen::VectorXd manip_grad = manipulability::Gradient(state, group, tool, q, limit_sharpness);
		const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
		auto masked_step = [&](const Eigen::VectorXd& mask) {
			const Eigen::MatrixXd Jm = J * mask.asDiagonal();
			const Eigen::MatrixXd Jm_pinv = scene_geometry::DampedPinv(Jm, damping);
			Eigen::VectorXd step = Jm_pinv * pose_error;
			const Eigen::VectorXd up = (I - Jm_pinv * Jm) * mask.asDiagonal() * manip_grad;
			if (up.norm() > 1e-12)
				step += manip_step * up / up.norm();
			if (num_pairs > 0)
			{
				const Eigen::MatrixXd Jcm = Jc * mask.asDiagonal();
				const Eigen::MatrixXd Jc_pinv = scene_geometry::DampedPinv(Jcm, damping);
				step = Jc_pinv * push + (I - Jc_pinv * Jcm) * step;
			}
			return step;
		};
		Eigen::VectorXd mask = Eigen::VectorXd::Ones(dof);
		Eigen::VectorXd dq = masked_step(mask);
		for (int pass = 0; pass < dof; ++pass)
		{
			bool changed = false;
			for (int k = 0; k < dof; ++k)
			{
				const size_t kk = static_cast<size_t>(k);
				if (mask(k) > 0.0 &&
					((q[kk] <= lower[kk] + 1e-4 && dq(k) < 0.0) || (q[kk] >= upper[kk] - 1e-4 && dq(k) > 0.0)))
				{
					mask(k) = 0.0;
					changed = true;
				}
			}
			if (!changed)
				break;
			dq = masked_step(mask);
		}
		const double step_max = dq.cwiseAbs().maxCoeff();
		if (step_max > max_joint_step)
			dq *= max_joint_step / step_max;
		for (int k = 0; k < dof; ++k)
			q[static_cast<size_t>(k)] += dq(k);
		state.setJointGroupPositions(group, q);
		state.enforceBounds(group);
		state.copyJointGroupPositions(group, q);
	}
	return best;
}

}  // namespace closest_ik
