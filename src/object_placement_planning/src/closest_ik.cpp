#include "object_placement_planning/closest_ik.hpp"

#include <algorithm>
#include <chrono>
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

WeightedSolveTiming& GetWeightedSolveTiming()
{
	static WeightedSolveTiming timing;
	return timing;
}

void ResetWeightedSolveTiming()
{
	WeightedSolveTiming& timing = GetWeightedSolveTiming();
	timing.solves = 0;
	timing.steps = 0;
	timing.distance_microseconds = 0;
	timing.check_microseconds = 0;
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

// The group's joint bounds; an unbounded joint gets +-max.
void GetJointBounds(const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, std::vector<double>& lower, std::vector<double>& upper)
{
	lower.resize(group->getVariableCount());
	upper.resize(group->getVariableCount());
	for (size_t k = 0; k < lower.size(); ++k)
	{
		const moveit::core::VariableBounds& b = state.getRobotModel()->getVariableBounds(group->getVariableNames()[k]);
		lower[k] = b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::max();
		upper[k] = b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::max();
	}
}

// The camera side: the tool side without its wrist link.
std::set<std::string> GetCameraLinks(const moveit::core::LinkModel* tool)
{
	std::set<std::string> camera_links = ToolSideLinks(tool);
	camera_links.erase(moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tool)->getName());
	return camera_links;
}

bool IsCameraObjectPair(const std::string& pair_name, const std::set<std::string>& camera_links)
{
	const size_t split = pair_name.find(" - ");
	const std::string first = pair_name.substr(0, split);
	const std::string second = pair_name.substr(split + 3);
	return (first == "object" && camera_links.count(second)) || (second == "object" && camera_links.count(first));
}

// Exact stage, like KDL: full damped Newton steps on the pose error, no collision terms; one validity check once within
// the tolerances. Not found if it doesn't converge or the pose is invalid.
// True if no arm link is within margin of anything. The camera side (links past the last wrist link) is left out: the
// viewpoint planner already set its standoff from the object.
bool HasArmClearance(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, double margin)
{
	const std::set<std::string> camera_side = GetCameraLinks(tool);
	const ClosePairs pairs = FindClosePairs(scene, state, group, CloseRequest(state, group, margin), margin);
	for (const std::string& name : pairs.names)
	{
		const size_t split = name.find(" - ");
		if (!camera_side.count(name.substr(0, split)) && !camera_side.count(name.substr(split + 3)))
			return false;
	}
	return true;
}

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
			if (options.exact_margin > 0.0 && !HasArmClearance(scene, state, group, tool, options.exact_margin))
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

// The weighted closest-stage cost at one pose: total = |residuals|^2.
struct WeightedCost
{
	double total = 0.0;
	scene_geometry::Vector6d pose_error = scene_geometry::Vector6d::Zero();
	Eigen::VectorXd residuals;
	Eigen::MatrixXd residual_slopes;  // d(residuals)/dq
};

WeightedCost GetWeightedCost(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& q, const Options& options, const std::set<std::string>& camera_links, const std::vector<double>& lower, const std::vector<double>& upper)
{
	const Eigen::Index dof = static_cast<Eigen::Index>(q.size());
	const Eigen::MatrixXd J = GetToolJacobian(state, group, tool, q, options.ignore_roll);
	WeightedCost cost;
	cost.pose_error = GetPoseError(target, state.getGlobalLinkTransform(tool), options.ignore_roll);
	std::vector<double> residuals;
	std::vector<Eigen::RowVectorXd> slopes;
	for (Eigen::Index row = 0; row < 6; ++row)
	{
		const double scale = std::sqrt(row < 3 ? options.position_weight : options.angle_weight);
		residuals.push_back(scale * cost.pose_error(row));
		slopes.push_back(-scale * J.row(row));	// the error is target minus tool
	}
	// One distance query out to the wider margin; the arm's own pairs pay the collision term inside margin and the
	// self-distance term inside self_distance_margin.
	const double self_margin = options.self_distance_weight > 0.0 ? std::max(options.margin, options.self_distance_margin) : options.margin;
	const auto distance_start = std::chrono::steady_clock::now();
	collision_detection::DistanceResult self_distances, world_distances;
	{
		// Separate queries: only the arm's own pairs need the wider threshold, and a small one lets the checker skip far pairs.
		planning_scene_monitor::LockedPlanningSceneRO locked(scene);
		collision_detection::DistanceRequest self_request = CloseRequest(state, group, self_margin);
		self_request.acm = &locked->getAllowedCollisionMatrix();
		self_request.enableGroup(locked->getRobotModel());
		locked->getCollisionEnv()->distanceSelf(self_request, self_distances, state);
		collision_detection::DistanceRequest world_request = CloseRequest(state, group, options.margin);
		world_request.acm = &locked->getAllowedCollisionMatrix();
		world_request.enableGroup(locked->getRobotModel());
		locked->getCollisionEnv()->distanceRobot(world_request, world_distances, state);
	}
	GetWeightedSolveTiming().distance_microseconds += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - distance_start).count();
	const ClosePairs world_pairs = ClosePairGradients(world_distances, state, group, options.margin);
	for (size_t pair_index = 0; pair_index < world_pairs.names.size(); ++pair_index)
	{
		const double scale = std::sqrt(IsCameraObjectPair(world_pairs.names[pair_index], camera_links) ? options.camera_object_weight : options.collision_weight);
		residuals.push_back(scale * (options.margin - world_pairs.dists[pair_index]));
		slopes.push_back(-scale * world_pairs.gradients[pair_index]);
	}
	const ClosePairs self_pairs = ClosePairGradients(self_distances, state, group, self_margin);
	for (size_t pair_index = 0; pair_index < self_pairs.names.size(); ++pair_index)
	{
		if (self_pairs.dists[pair_index] < options.margin)
		{
			const double scale = std::sqrt(options.collision_weight);
			residuals.push_back(scale * (options.margin - self_pairs.dists[pair_index]));
			slopes.push_back(-scale * self_pairs.gradients[pair_index]);
		}
		if (options.self_distance_weight > 0.0 && self_pairs.dists[pair_index] < options.self_distance_margin)
		{
			const double scale = std::sqrt(options.self_distance_weight);
			residuals.push_back(scale * (options.self_distance_margin - self_pairs.dists[pair_index]));
			slopes.push_back(-scale * self_pairs.gradients[pair_index]);
		}
	}
	// Joint room over the whole range: s^2 / (1 - s^2) with s the share of the half range from the middle. About s^2
	// mid-range, steep near a limit (s 0.9: 4.3, s 0.99: 49); s is capped at 0.999 since bounds put joints right on a limit.
	const double room_scale = std::sqrt(options.joint_room_weight);
	for (size_t joint_index = 0; joint_index < q.size(); ++joint_index)
	{
		if (upper[joint_index] - lower[joint_index] > 1e6)
			continue;  // unbounded
		const double half_range = 0.5 * (upper[joint_index] - lower[joint_index]);
		const double middle = 0.5 * (upper[joint_index] + lower[joint_index]);
		const double share = std::clamp((q[joint_index] - middle) / half_range, -0.999, 0.999);
		const double room_left = 1.0 - share * share;
		Eigen::RowVectorXd slope = Eigen::RowVectorXd::Zero(dof);
		slope(static_cast<Eigen::Index>(joint_index)) = room_scale / (room_left * std::sqrt(room_left) * half_range);
		residuals.push_back(room_scale * share / std::sqrt(room_left));
		slopes.push_back(slope);
	}
	cost.residuals = Eigen::Map<const Eigen::VectorXd>(residuals.data(), static_cast<Eigen::Index>(residuals.size()));
	cost.residual_slopes.resize(static_cast<Eigen::Index>(slopes.size()), dof);
	for (size_t row = 0; row < slopes.size(); ++row)
		cost.residual_slopes.row(static_cast<Eigen::Index>(row)) = slopes[row];
	cost.total = cost.residuals.squaredNorm();
	return cost;
}

// Weighted closest stage: Levenberg-Marquardt steps on the weighted cost (damping grows on a step that doesn't lower it).
// The checks run once at the end, from the cheapest pose passed through down, so most steps skip them; a pose within
// the exact tolerances is checked at once and ends the solve if it is exact.
Result SolveWeighted(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const Options& options)
{
	const double max_joint_step = 0.1;	// rad
	const int max_damping_tries = 3;
	const int stall_window = 5;	 // stop once the best gap improves < 1 mm over this many steps, as the priority stage
	const double stall_gain = 1e-3;
	WeightedSolveTiming& timing = GetWeightedSolveTiming();
	++timing.solves;
	std::vector<double> lower, upper;
	GetJointBounds(state, group, lower, upper);
	const std::set<std::string> tool_side = ToolSideLinks(tool);
	const std::set<std::string> camera_links = GetCameraLinks(tool);
	const Eigen::Index dof = static_cast<Eigen::Index>(start.size());
	// The kept-pose checks: valid, clear path, and whether it is exact.
	auto check_pose = [&](const std::vector<double>& q, const WeightedCost& cost) {
		const auto check_start = std::chrono::steady_clock::now();
		Result result;
		if (scene_geometry::IsStateValid(scene, &state, group, q.data()) && ClearPathToTarget(scene, state, tool, target.translation(), tool_side))
		{
			const bool exact = IsWithinExactTolerance(cost.pose_error, options) && (options.exact_margin <= 0.0 || HasArmClearance(scene, state, group, tool, options.exact_margin));
			result = {true, exact, cost.pose_error, scene_geometry::PoseGap(cost.pose_error, options.rot_scale), q, cost.total};
		}
		timing.check_microseconds += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - check_start).count();
		return result;
	};
	std::vector<double> q = start;
	WeightedCost cost = GetWeightedCost(scene, state, group, tool, target, q, options, camera_links, lower, upper);
	std::vector<std::pair<std::vector<double>, WeightedCost>> visited = {{q, cost}};
	std::vector<double> best_gap_history;
	double damping = 1e-3;
	for (int step = 0; step < options.weighted_iters; ++step)
	{
		if (IsWithinExactTolerance(cost.pose_error, options))
		{
			const Result result = check_pose(q, cost);
			if (result.exact)
				return result;
		}
		const double gap = scene_geometry::PoseGap(cost.pose_error, options.rot_scale);
		best_gap_history.push_back(std::min(gap, best_gap_history.empty() ? gap : best_gap_history.back()));
		if (static_cast<int>(best_gap_history.size()) > stall_window && best_gap_history[best_gap_history.size() - 1 - stall_window] - best_gap_history.back() < stall_gain)
			break;
		++timing.steps;
		const Eigen::VectorXd gradient = 2.0 * cost.residual_slopes.transpose() * cost.residuals;
		const Eigen::MatrixXd hessian = 2.0 * cost.residual_slopes.transpose() * cost.residual_slopes;
		bool lowered = false;
		for (int damping_try = 0; damping_try < max_damping_tries && !lowered; ++damping_try)
		{
			Eigen::VectorXd dq = -(hessian + damping * Eigen::MatrixXd::Identity(dof, dof)).ldlt().solve(gradient);
			const double step_max = dq.cwiseAbs().maxCoeff();
			if (step_max > max_joint_step)
				dq *= max_joint_step / step_max;
			std::vector<double> candidate = q;
			for (Eigen::Index k = 0; k < dof; ++k)
				candidate[static_cast<size_t>(k)] += dq(k);
			state.setJointGroupPositions(group, candidate);
			state.enforceBounds(group);
			state.copyJointGroupPositions(group, candidate);
			const WeightedCost candidate_cost = GetWeightedCost(scene, state, group, tool, target, candidate, options, camera_links, lower, upper);
			if (candidate_cost.total < cost.total)
			{
				q = candidate;
				cost = candidate_cost;
				visited.emplace_back(q, cost);
				damping = std::max(1e-6, 0.5 * damping);
				lowered = true;
			}
			else
				damping *= 10.0;
		}
		if (!lowered)
			break;
	}
	// The cost only falls, so the last pose is the cheapest: check back from it until one passes.
	for (auto visited_pose = visited.rbegin(); visited_pose != visited.rend(); ++visited_pose)
	{
		const Result result = check_pose(visited_pose->first, visited_pose->second);
		if (result.found)
			return result;
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
	if (options.use_weighted_cost)
		return SolveWeighted(scene, state, group, tool, target, start, options);
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

	std::vector<double> lower, upper;
	GetJointBounds(state, group, lower, upper);

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
			// Exact only with the exact poses' arm clearance too, as the exact stage requires.
			const bool exact = IsWithinExactTolerance(pose_error, options) && (options.exact_margin <= 0.0 || HasArmClearance(scene, state, group, tool, options.exact_margin));
			best = {true, exact, pose_error, gap, q};
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
