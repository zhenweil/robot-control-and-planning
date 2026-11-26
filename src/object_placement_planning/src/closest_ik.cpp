#include "object_placement_planning/closest_ik.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include <geometric_shapes/shapes.h>

#include "object_placement_planning/manipulability.hpp"

namespace closest_ik
{

ClosePairs ClosePairGradients(const collision_detection::DistanceResult& distance_result, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double margin)
{
	ClosePairs pairs;
	for (const auto& entry : distance_result.distances)
		for (const collision_detection::DistanceResultsData& pair : entry.second)
		{
			if (pair.distance >= margin)
				continue;
			Eigen::RowVectorXd gradient = Eigen::RowVectorXd::Zero(group->getVariableCount());
			Eigen::Vector3d object_slope = Eigen::Vector3d::Zero();
			bool has_robot_side = false;
			for (int side = 0; side < 2; ++side)
			{
				// distance ~ distance0 + normal . (moved point 1 - moved point 0)
				const double sign = side == 0 ? -1.0 : 1.0;
				if (pair.body_types[side] == collision_detection::BodyType::WORLD_OBJECT && pair.link_names[side] == "object")
					object_slope += sign * pair.normal;
				if (pair.body_types[side] != collision_detection::BodyType::ROBOT_LINK && pair.body_types[side] != collision_detection::BodyType::ROBOT_ATTACHED)
					continue;
				const moveit::core::LinkModel* link = state.getRobotModel()->getLinkModel(pair.link_names[side]);
				if (!link || !group->isLinkUpdated(link->getName()))
					continue;  // not moved by the group; its Jacobian would log an error
				const Eigen::Vector3d local_point = state.getGlobalLinkTransform(link).inverse() * pair.nearest_points[side];
				Eigen::MatrixXd point_jacobian;
				if (!state.getJacobian(group, link, local_point, point_jacobian))
					continue;
				gradient += sign * (pair.normal.transpose() * point_jacobian.topRows<3>());
				has_robot_side = true;
			}
			if (!has_robot_side || gradient.norm() <= 1e-9)
				continue;
			pairs.gradients.push_back(gradient);
			pairs.dists.push_back(pair.distance);
			pairs.object_slopes.push_back(object_slope);
			pairs.names.push_back(pair.link_names[0] + " - " + pair.link_names[1]);
		}
	return pairs;
}

collision_detection::DistanceRequest CloseRequest(const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double threshold)
{
	collision_detection::DistanceRequest request;
	request.enable_nearest_points = true;
	request.enable_signed_distance = true;
	request.type = collision_detection::DistanceRequestType::ALL;
	request.group_name = group->getName();
	request.enableGroup(state.getRobotModel());
	request.distance_threshold = threshold;
	request.max_contacts_per_body = 2;
	return request;
}

std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> Distances(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const collision_detection::DistanceRequest& request)
{
	std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> results;
	collision_detection::DistanceRequest scene_request = request;
	planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene);
	scene_request.acm = &locked_scene->getAllowedCollisionMatrix();
	scene_request.enableGroup(locked_scene->getRobotModel());  // MoveIt matches links by pointer
	locked_scene->getCollisionEnv()->distanceSelf(scene_request, results.first, state);
	locked_scene->getCollisionEnv()->distanceRobot(scene_request, results.second, state);
	return results;
}

ClosePairs FindClosePairs(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const collision_detection::DistanceRequest& request, double margin)
{
	const auto [self_result, world_result] = Distances(scene, state, request);
	ClosePairs pairs = ClosePairGradients(self_result, state, group, margin);
	const ClosePairs world_pairs = ClosePairGradients(world_result, state, group, margin);
	pairs.gradients.insert(pairs.gradients.end(), world_pairs.gradients.begin(), world_pairs.gradients.end());
	pairs.dists.insert(pairs.dists.end(), world_pairs.dists.begin(), world_pairs.dists.end());
	pairs.object_slopes.insert(pairs.object_slopes.end(), world_pairs.object_slopes.begin(), world_pairs.object_slopes.end());
	pairs.names.insert(pairs.names.end(), world_pairs.names.begin(), world_pairs.names.end());
	return pairs;
}

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

std::set<std::string> GetCameraLinks(const moveit::core::LinkModel* tool)
{
	std::set<std::string> camera_links = ToolSideLinks(tool);
	camera_links.erase(moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tool)->getName());
	return camera_links;
}

bool PairHasAnyLink(const std::string& pair_name, const std::set<std::string>& links)
{
	const size_t split = pair_name.find(" - ");
	return links.count(pair_name.substr(0, split)) || links.count(pair_name.substr(split + 3));
}

// Something between tool and target means the closest pose can't slide onto it.
bool ClearPathToTarget(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::LinkModel* tool, const Eigen::Vector3d& target, const std::set<std::string>& tool_side)
{
	const Eigen::Vector3d local_target = state.getGlobalLinkTransform(tool).inverse() * target;
	const double length = local_target.norm();
	if (length < 0.01)
		return true;
	Eigen::Isometry3d rod_pose = Eigen::Isometry3d::Identity();
	rod_pose.translation() = 0.5 * local_target;  // cylinder centred on its pose, axis z
	rod_pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), local_target / length).toRotationMatrix();
	const std::string rod_id = "path_to_target";
	state.attachBody(rod_id, Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Cylinder>(0.005, length)}, {rod_pose}, tool_side, tool->getName());
	state.update();
	collision_detection::CollisionRequest request;
	collision_detection::CollisionResult result;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(scene);
		locked_scene->checkCollision(request, result, state);
	}
	state.clearAttachedBody(rod_id);
	return !result.collision;
}

scene_geometry::Vector6d GetPoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool, bool ignore_roll)
{
	if (!ignore_roll)
		return scene_geometry::PoseError(target, tool);
	const Eigen::Vector3d tool_axis = tool.linear().col(2);
	const Eigen::Vector3d target_axis = target.linear().col(2);
	const Eigen::Vector3d cross = tool_axis.cross(target_axis);
	const double angle = std::atan2(cross.norm(), tool_axis.dot(target_axis));
	scene_geometry::Vector6d pose_error;
	pose_error.head<3>() = target.translation() - tool.translation();
	// Opposite axes: turn about any perpendicular.
	pose_error.tail<3>() = cross.norm() > 1e-12 ? Eigen::Vector3d(cross.normalized() * angle) : Eigen::Vector3d(tool.linear().col(0) * angle);
	return pose_error;
}

Eigen::MatrixXd GetToolJacobian(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const std::vector<double>& joints, bool ignore_roll)
{
	Eigen::MatrixXd jacobian = manipulability::ToolJacobian(state, group, tool, joints);
	if (ignore_roll)
	{
		const Eigen::Vector3d axis = state.getGlobalLinkTransform(tool).linear().col(2);
		jacobian.bottomRows<3>() = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * jacobian.bottomRows<3>();
	}
	return jacobian;
}

namespace
{

std::vector<double> AddToJoints(const std::vector<double>& joints, const Eigen::VectorXd& change)
{
	std::vector<double> result = joints;
	for (size_t joint_index = 0; joint_index < result.size(); ++joint_index)
		result[joint_index] += change(static_cast<Eigen::Index>(joint_index));
	return result;
}

// Scales the step down so no joint moves more than max_joint_step.
void LimitJointStep(Eigen::VectorXd& joint_step, double max_joint_step)
{
	const double largest = joint_step.cwiseAbs().maxCoeff();
	if (largest > max_joint_step)
		joint_step *= max_joint_step / largest;
}

// Sets the joints, clamps them into bounds, and reads them back.
void ClampIntoBounds(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, std::vector<double>& joints)
{
	state.setJointGroupPositions(group, joints);
	state.enforceBounds(group);
	state.copyJointGroupPositions(group, joints);
}

bool IsWithinExactTolerance(const scene_geometry::Vector6d& pose_error, const SolveSettings& options)
{
	return pose_error.head<3>().norm() < options.exact_position_tolerance && pose_error.tail<3>().norm() < options.exact_rotation_tolerance;
}

// Joint bounds; an unbounded joint gets +-max.
void GetJointBounds(const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, std::vector<double>& lower, std::vector<double>& upper)
{
	lower.resize(group->getVariableCount());
	upper.resize(group->getVariableCount());
	for (size_t joint_index = 0; joint_index < lower.size(); ++joint_index)
	{
		const moveit::core::VariableBounds& bounds = state.getRobotModel()->getVariableBounds(group->getVariableNames()[joint_index]);
		lower[joint_index] = bounds.position_bounded_ ? bounds.min_position_ : -std::numeric_limits<double>::max();
		upper[joint_index] = bounds.position_bounded_ ? bounds.max_position_ : std::numeric_limits<double>::max();
	}
}

bool IsCameraObjectPair(const std::string& pair_name, const std::set<std::string>& camera_links)
{
	const size_t split = pair_name.find(" - ");
	const std::string first = pair_name.substr(0, split);
	const std::string second = pair_name.substr(split + 3);
	return (first == "object" && camera_links.count(second)) || (second == "object" && camera_links.count(first));
}

// True if no arm link is within margin; the camera side is exempt.
bool HasArmClearance(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, double margin)
{
	const std::set<std::string> camera_links = GetCameraLinks(tool);
	const ClosePairs pairs = FindClosePairs(scene, state, group, CloseRequest(state, group, margin), margin);
	return std::all_of(pairs.names.begin(), pairs.names.end(), [&](const std::string& pair_name) { return PairHasAnyLink(pair_name, camera_links); });
}

bool IsExact(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const scene_geometry::Vector6d& pose_error, const SolveSettings& options)
{
	return IsWithinExactTolerance(pose_error, options) && (options.exact_margin <= 0.0 || HasArmClearance(scene, state, group, tool, options.exact_margin));
}

// Stops once the best gap improved under stall_gain over stall_window steps.
bool HasStalled(std::vector<double>& best_gap_history, double gap)
{
	const size_t stall_window = 5;
	const double stall_gain = 1e-3;  // m
	best_gap_history.push_back(std::min(gap, best_gap_history.empty() ? gap : best_gap_history.back()));
	return best_gap_history.size() > stall_window && best_gap_history[best_gap_history.size() - 1 - stall_window] - best_gap_history.back() < stall_gain;
}

// Damped Newton steps on the pose error only; validity checked once within tolerance.
Result SolveExact(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const SolveSettings& options)
{
	const double damping = 1e-3;
	const double max_joint_step = 0.5;  // rad
	std::vector<double> joints = start;
	for (int iteration = 0; iteration < options.exact_iters; ++iteration)
	{
		state.setJointGroupPositions(group, joints);
		state.update();
		const scene_geometry::Vector6d pose_error = GetPoseError(target, state.getGlobalLinkTransform(tool), options.ignore_roll);
		if (IsWithinExactTolerance(pose_error, options))
		{
			if (!scene_geometry::IsStateValid(scene, &state, group, joints.data()))
				return {};
			if (options.exact_margin > 0.0 && !HasArmClearance(scene, state, group, tool, options.exact_margin))
				return {};
			return {true, true, pose_error, scene_geometry::PoseGap(pose_error, options.rot_scale), joints};
		}
		Eigen::VectorXd joint_step = scene_geometry::DampedPinv(GetToolJacobian(state, group, tool, joints, options.ignore_roll), damping) * pose_error;
		LimitJointStep(joint_step, max_joint_step);
		joints = AddToJoints(joints, joint_step);
		ClampIntoBounds(state, group, joints);
	}
	return {};
}

// Weighted cost at one pose: total = |residuals|^2.
struct WeightedCost
{
	double total = 0.0;
	scene_geometry::Vector6d pose_error = scene_geometry::Vector6d::Zero();
	Eigen::VectorXd residuals;
	Eigen::MatrixXd residual_gradients;  // d(residuals)/d(joints)
};

// Context shared by every cost evaluation of one weighted solve.
struct WeightedSolveContext
{
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene;
	const moveit::core::JointModelGroup* group;
	const moveit::core::LinkModel* tool;
	const Eigen::Isometry3d& target;
	const SolveSettings& options;
	std::set<std::string> camera_links;
	std::vector<double> lower_bounds;
	std::vector<double> upper_bounds;
};

WeightedCost GetWeightedCost(const WeightedSolveContext& context, moveit::core::RobotState& state, const std::vector<double>& joints)
{
	const SolveSettings& options = context.options;
	const Eigen::Index num_joints = static_cast<Eigen::Index>(joints.size());
	const Eigen::MatrixXd jacobian = GetToolJacobian(state, context.group, context.tool, joints, options.ignore_roll);
	WeightedCost cost;
	cost.pose_error = GetPoseError(context.target, state.getGlobalLinkTransform(context.tool), options.ignore_roll);
	std::vector<double> residuals;
	std::vector<Eigen::RowVectorXd> gradients;
	auto add_residual = [&](double scale, double shortfall, const Eigen::RowVectorXd& shortfall_gradient)
	{
		residuals.push_back(scale * shortfall);
		gradients.push_back(scale * shortfall_gradient);
	};
	for (Eigen::Index row = 0; row < 6; ++row)
		add_residual(std::sqrt(row < 3 ? options.position_weight : options.angle_weight), cost.pose_error(row), -jacobian.row(row));  // error is target minus tool
	// Self pairs need the wider self-distance window; separate queries keep the world one small.
	const double self_window = options.self_distance_weight > 0.0 ? std::max(options.margin, options.self_distance_margin) : options.margin;
	collision_detection::DistanceResult self_distances, world_distances;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(context.scene);
		collision_detection::DistanceRequest self_request = CloseRequest(state, context.group, self_window);
		self_request.acm = &locked_scene->getAllowedCollisionMatrix();
		self_request.enableGroup(locked_scene->getRobotModel());
		locked_scene->getCollisionEnv()->distanceSelf(self_request, self_distances, state);
		collision_detection::DistanceRequest world_request = CloseRequest(state, context.group, options.margin);
		world_request.acm = &locked_scene->getAllowedCollisionMatrix();
		world_request.enableGroup(locked_scene->getRobotModel());
		locked_scene->getCollisionEnv()->distanceRobot(world_request, world_distances, state);
	}
	const ClosePairs world_pairs = ClosePairGradients(world_distances, state, context.group, options.margin);
	for (size_t pair_index = 0; pair_index < world_pairs.names.size(); ++pair_index)
	{
		const double weight = IsCameraObjectPair(world_pairs.names[pair_index], context.camera_links) ? options.camera_object_weight : options.collision_weight;
		add_residual(std::sqrt(weight), options.margin - world_pairs.dists[pair_index], -world_pairs.gradients[pair_index]);
	}
	const ClosePairs self_pairs = ClosePairGradients(self_distances, state, context.group, self_window);
	for (size_t pair_index = 0; pair_index < self_pairs.names.size(); ++pair_index)
	{
		const double distance = self_pairs.dists[pair_index];
		if (distance < options.margin)
			add_residual(std::sqrt(options.collision_weight), options.margin - distance, -self_pairs.gradients[pair_index]);
		if (options.self_distance_weight > 0.0 && distance < options.self_distance_margin)
			add_residual(std::sqrt(options.self_distance_weight), options.self_distance_margin - distance, -self_pairs.gradients[pair_index]);
	}
	// Joint room s^2 / (1 - s^2), s = offset from mid-range in half ranges.
	const double room_scale = std::sqrt(options.joint_room_weight);
	for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
	{
		const double lower = context.lower_bounds[joint_index];
		const double upper = context.upper_bounds[joint_index];
		if (upper - lower > 1e6)
			continue;  // unbounded
		const double half_range = 0.5 * (upper - lower);
		const double share = std::clamp((joints[joint_index] - 0.5 * (upper + lower)) / half_range, -0.999, 0.999);
		const double room_left = 1.0 - share * share;
		Eigen::RowVectorXd room_gradient = Eigen::RowVectorXd::Zero(num_joints);
		room_gradient(static_cast<Eigen::Index>(joint_index)) = room_scale / (room_left * std::sqrt(room_left) * half_range);
		residuals.push_back(room_scale * share / std::sqrt(room_left));
		gradients.push_back(room_gradient);
	}
	cost.residuals = Eigen::Map<const Eigen::VectorXd>(residuals.data(), static_cast<Eigen::Index>(residuals.size()));
	cost.residual_gradients.resize(static_cast<Eigen::Index>(gradients.size()), num_joints);
	for (size_t row = 0; row < gradients.size(); ++row)
		cost.residual_gradients.row(static_cast<Eigen::Index>(row)) = gradients[row];
	cost.total = cost.residuals.squaredNorm();
	return cost;
}

// Levenberg-Marquardt on the weighted cost; checks run at the end, cheapest pose first.
Result SolveWeighted(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const SolveSettings& options)
{
	const double max_joint_step = 0.1;  // rad
	const int max_damping_tries = 3;
	WeightedSolveContext context{scene, group, tool, target, options, GetCameraLinks(tool), {}, {}};
	GetJointBounds(state, group, context.lower_bounds, context.upper_bounds);
	const std::set<std::string> tool_side = ToolSideLinks(tool);
	const Eigen::Index num_joints = static_cast<Eigen::Index>(start.size());
	auto check_pose = [&](const std::vector<double>& joints, const WeightedCost& cost)
	{
		if (!scene_geometry::IsStateValid(scene, &state, group, joints.data()) || !ClearPathToTarget(scene, state, tool, target.translation(), tool_side))
			return Result();
		return Result{true, IsExact(scene, state, group, tool, cost.pose_error, options), cost.pose_error, scene_geometry::PoseGap(cost.pose_error, options.rot_scale), joints, cost.total};
	};
	std::vector<double> joints = start;
	WeightedCost cost = GetWeightedCost(context, state, joints);
	std::vector<std::pair<std::vector<double>, WeightedCost>> visited = {{joints, cost}};
	std::vector<double> best_gap_history;
	double damping = 1e-3;
	for (int step = 0; step < options.weighted_iters; ++step)
	{
		if (IsWithinExactTolerance(cost.pose_error, options))
		{
			const Result result = check_pose(joints, cost);
			if (result.exact)
				return result;
		}
		if (HasStalled(best_gap_history, scene_geometry::PoseGap(cost.pose_error, options.rot_scale)))
			break;
		const Eigen::VectorXd gradient = 2.0 * cost.residual_gradients.transpose() * cost.residuals;
		const Eigen::MatrixXd hessian = 2.0 * cost.residual_gradients.transpose() * cost.residual_gradients;
		bool lowered = false;
		for (int damping_try = 0; damping_try < max_damping_tries && !lowered; ++damping_try)
		{
			Eigen::VectorXd joint_step = -(hessian + damping * Eigen::MatrixXd::Identity(num_joints, num_joints)).ldlt().solve(gradient);
			LimitJointStep(joint_step, max_joint_step);
			std::vector<double> candidate = AddToJoints(joints, joint_step);
			ClampIntoBounds(state, group, candidate);
			const WeightedCost candidate_cost = GetWeightedCost(context, state, candidate);
			if (!(candidate_cost.total < cost.total))
			{
				damping *= 10.0;
				continue;
			}
			joints = candidate;
			cost = candidate_cost;
			visited.emplace_back(joints, cost);
			damping = std::max(1e-6, 0.5 * damping);
			lowered = true;
		}
		if (!lowered)
			break;
	}
	// The cost only falls, so check back from the last pose until one passes.
	for (auto visited_pose = visited.rbegin(); visited_pose != visited.rend(); ++visited_pose)
	{
		const Result result = check_pose(visited_pose->first, visited_pose->second);
		if (result.found)
			return result;
	}
	return {};
}

// Priority steps: close pairs, then pose error, then manipulability.
// Joints pushed past a limit are frozen and the step re-solved.
Result SolvePriority(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const SolveSettings& options)
{
	const double damping = 0.01;
	const double max_joint_step = 0.1;  // rad
	const double manipulability_step = 0.02;  // rad
	const int num_joints = static_cast<int>(group->getVariableCount());
	const collision_detection::DistanceRequest request = CloseRequest(state, group, options.margin);
	std::vector<double> lower, upper;
	GetJointBounds(state, group, lower, upper);
	const std::set<std::string> tool_side = ToolSideLinks(tool);
	const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(num_joints, num_joints);
	std::vector<double> joints = start;
	Result best;
	std::vector<double> best_gap_history;
	for (int iteration = 0; iteration <= options.iters; ++iteration)
	{
		state.setJointGroupPositions(group, joints);
		state.update();
		const scene_geometry::Vector6d pose_error = GetPoseError(target, state.getGlobalLinkTransform(tool), options.ignore_roll);
		const double gap = scene_geometry::PoseGap(pose_error, options.rot_scale);
		const ClosePairs pairs = FindClosePairs(scene, state, group, request, options.margin);
		// Full validity check: the distance query misses self-collisions near the base.
		if (gap < best.gap && scene_geometry::IsStateValid(scene, &state, group, joints.data()) && ClearPathToTarget(scene, state, tool, target.translation(), tool_side))
		{
			best = {true, IsExact(scene, state, group, tool, pose_error, options), pose_error, gap, joints};
			if (gap < 1e-6)
				break;
		}
		if (iteration == options.iters || HasStalled(best_gap_history, gap))
			break;
		const Eigen::MatrixXd jacobian = GetToolJacobian(state, group, tool, joints, options.ignore_roll);
		const Eigen::Index num_pairs = static_cast<Eigen::Index>(pairs.gradients.size());
		Eigen::MatrixXd pair_jacobian(num_pairs, num_joints);
		Eigen::VectorXd pair_push(num_pairs);
		for (Eigen::Index pair_index = 0; pair_index < num_pairs; ++pair_index)
		{
			pair_jacobian.row(pair_index) = pairs.gradients[static_cast<size_t>(pair_index)];
			pair_push(pair_index) = options.margin - pairs.dists[static_cast<size_t>(pair_index)];
		}
		const Eigen::VectorXd manipulability_gradient = manipulability::Gradient(state, group, tool, joints, options.limit_sharpness);
		auto get_step_with_free_joints = [&](const Eigen::VectorXd& free_joints)
		{
			const Eigen::MatrixXd free_jacobian = jacobian * free_joints.asDiagonal();
			const Eigen::MatrixXd free_pinv = scene_geometry::DampedPinv(free_jacobian, damping);
			Eigen::VectorXd step = free_pinv * pose_error;
			const Eigen::VectorXd climb = (identity - free_pinv * free_jacobian) * free_joints.asDiagonal() * manipulability_gradient;
			if (climb.norm() > 1e-12)
				step += manipulability_step * climb / climb.norm();
			if (num_pairs > 0)
			{
				const Eigen::MatrixXd free_pair_jacobian = pair_jacobian * free_joints.asDiagonal();
				const Eigen::MatrixXd pair_pinv = scene_geometry::DampedPinv(free_pair_jacobian, damping);
				step = pair_pinv * pair_push + (identity - pair_pinv * free_pair_jacobian) * step;
			}
			return step;
		};
		Eigen::VectorXd free_joints = Eigen::VectorXd::Ones(num_joints);
		Eigen::VectorXd joint_step = get_step_with_free_joints(free_joints);
		for (int pass = 0; pass < num_joints; ++pass)
		{
			bool changed = false;
			for (int joint_index = 0; joint_index < num_joints; ++joint_index)
			{
				const size_t index = static_cast<size_t>(joint_index);
				const bool pushed_below = joints[index] <= lower[index] + 1e-4 && joint_step(joint_index) < 0.0;
				const bool pushed_above = joints[index] >= upper[index] - 1e-4 && joint_step(joint_index) > 0.0;
				if (free_joints(joint_index) > 0.0 && (pushed_below || pushed_above))
				{
					free_joints(joint_index) = 0.0;
					changed = true;
				}
			}
			if (!changed)
				break;
			joint_step = get_step_with_free_joints(free_joints);
		}
		LimitJointStep(joint_step, max_joint_step);
		joints = AddToJoints(joints, joint_step);
		ClampIntoBounds(state, group, joints);
	}
	return best;
}

}  // namespace

Result Solve(const planning_scene_monitor::PlanningSceneMonitorPtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const std::vector<double>& start, const SolveSettings& options)
{
	if (options.try_exact_first || options.exact_only)
	{
		const Result exact = SolveExact(scene, state, group, tool, target, start, options);
		if (exact.found || options.exact_only)
			return exact;
	}
	if (options.use_weighted_cost)
		return SolveWeighted(scene, state, group, tool, target, start, options);
	return SolvePriority(scene, state, group, tool, target, start, options);
}

}  // namespace closest_ik
