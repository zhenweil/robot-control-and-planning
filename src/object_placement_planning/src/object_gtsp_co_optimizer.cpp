#include "object_placement_planning/object_gtsp_co_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <thread>
#include <utility>

#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>

#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/manipulability.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/room_margins.hpp"
#include "object_placement_planning/scene_geometry.hpp"

namespace object_gtsp_co_optimize
{

Optimizer::Optimizer(
	rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model, planning_scene_monitor::PlanningSceneMonitorPtr scene,
	const std::string& group_name, std::vector<Eigen::Isometry3d> viewpoints, const Eigen::Matrix3d& object_rotation, Joints home_pose,
	DescentLimits descent_limits, MethodSwitches method_switches, CostWeights cost_weights, StepSizes step_sizes, IkSettings ik_settings, TourSettings tour_settings, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub, std::string mesh_path, double mesh_scale)
	: node(std::move(node)), model(std::move(model)), scene(std::move(scene)), home_pose(std::move(home_pose)), step_sizes(step_sizes), ik_settings(std::move(ik_settings)),
	  cost_weights(cost_weights), tour_settings(std::move(tour_settings)), descent_limits(std::move(descent_limits)), method_switches(method_switches), viewpoints(std::move(viewpoints)), object_rotation(object_rotation), mesh_path(std::move(mesh_path)), mesh_scale(mesh_scale), progress_pub(std::move(progress_pub))
{
	this->state = std::make_unique<moveit::core::RobotState>(this->model);
	this->state->setToDefaultValues();
	this->group = this->state->getJointModelGroup(group_name);
	this->tool = this->model->getLinkModel("tool0");
	// The camera side, as exact IK's clearance check: links rigid with the tool and below, the wrist link itself aside.
	this->camera_side_links = closest_ik::ToolSideLinks(this->tool);
	this->camera_side_links.erase(moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(this->tool)->getName());
	SetJointSpeedLimits(this->model, group_name, this->tour_settings.edge_weights);
	this->counted_joints = joint_travel::AllJoints(static_cast<int>(this->home_pose.size()) - 1);  // the last joint only spins the camera
	this->num_threads = std::max(1u, std::min(std::thread::hardware_concurrency(), static_cast<unsigned int>(this->viewpoints.size())));
}

ObjectPositionEvaluation Optimizer::runDescent(const Eigen::Vector3d& start)
{
	this->start_time = std::chrono::steady_clock::now();
	return this->runDescentFromStart(this->clampToSearchBox(start));
}

std::vector<Joints> Optimizer::getPosesInTourOrder(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const
{
	std::vector<Joints> poses;
	for (int viewpoint : order)
		poses.push_back(placement.chosen_ik_per_viewpoint[static_cast<size_t>(viewpoint)].q);
	return poses;
}

double Optimizer::getSmallestJointMargin(const std::vector<Joints>& poses) const
{
	double smallest = std::numeric_limits<double>::infinity();
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	for (const Joints& pose : poses)
		for (int j : this->counted_joints)
		{
			const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(j)]->getVariableBounds()[0];
			if (!bounds.position_bounded_)
				continue;
			const double value = pose[static_cast<size_t>(j)];
			smallest = std::min(smallest, std::min(value - bounds.min_position_, bounds.max_position_ - value));
		}
	return smallest;
}

const std::vector<int>& Optimizer::getCountedJoints() const
{
	return this->counted_joints;
}

const joint_travel::EdgeWeights& Optimizer::getEdgeWeights() const
{
	return this->tour_settings.edge_weights;
}

ObjectPositionEvaluation Optimizer::runDescentFromStart(const Eigen::Vector3d& start)
{
	double penalty = this->cost_weights.initial_penalty;
	ObjectPositionEvaluation current;
	current.placement = this->solveViewpointIkAtPosition(start, std::vector<Joints>(this->viewpoints.size(), this->home_pose));
	current.order = this->reorderTourWithGtsp(current.placement, {});
	this->evaluateCurrentAndReorderedTour(current, penalty);
	const int num_viewpoint = static_cast<int>(this->viewpoints.size());
	double reach_step = this->step_sizes.initial_step;
	double tour_step = this->step_sizes.initial_step;
	this->object_trail.assign(1, start);
	this->publishProgressMarkers(current.placement, current.order, Eigen::Vector3d::Zero());
	this->logObjectPositionEvaluation("start", current);
	bool object_frozen = false;	 // freeze_object_when_reached: set once every viewpoint is reached
	for (int iteration = 1; iteration <= this->descent_limits.max_steps && rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - this->start_time).count() < this->descent_limits.time_budget; ++iteration)
	{
		bool changed = false;
		if (current.placement.num_reached < num_viewpoint && !object_frozen)
			changed = this->moveObjectToReachMore(current, reach_step, penalty, iteration) || changed;
		// Checked after the reach step, so the iteration that reaches the last viewpoint takes no tour move.
		if (this->method_switches.freeze_object_when_reached && !object_frozen && current.placement.num_reached == num_viewpoint)
		{
			object_frozen = true;
			RCLCPP_INFO(this->node->get_logger(), "iteration %d: all reached; object frozen at abs (%.4f, %.4f, %.4f)", iteration, current.placement.position.x(), current.placement.position.y(), current.placement.position.z());
		}
		changed = this->improveArmPosesWithObjectFixed(current, penalty, iteration) || changed;
		if (this->method_switches.tour_object_move && !object_frozen)
			changed = this->moveObjectToLowerCost(current, tour_step, penalty, iteration) || changed;
		if (changed)
			continue;
		if (current.placement.num_reached == num_viewpoint || penalty >= this->cost_weights.max_penalty)
			break;
		penalty = std::min(this->cost_weights.max_penalty, penalty * this->cost_weights.penalty_growth);
		this->updateWeightedTotalCost(current, penalty);
		reach_step = this->step_sizes.initial_step;
		tour_step = this->step_sizes.initial_step;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d: nothing changed with %d missed; penalty -> %.0f per m", iteration, num_viewpoint - current.placement.num_reached, penalty);
		this->logMissedViewpoints(current.placement);
	}
	this->logObjectPositionEvaluation("end", current);
	this->logMissedViewpoints(current.placement);
	return current;
}

Eigen::Vector3d Optimizer::clampToSearchBox(const Eigen::Vector3d& position) const
{
	return position.cwiseMax(this->descent_limits.search_box_lower_bound).cwiseMin(this->descent_limits.search_box_upper_bound);
}

ObjectPositionEvaluation Optimizer::evaluateObjectPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds, const std::vector<int>& order, double penalty)
{
	ObjectPositionEvaluation candidate;
	candidate.placement = this->solveViewpointIkAtPosition(this->clampToSearchBox(position), seeds, fallback_seeds);
	candidate.order = order;
	this->evaluateCurrentAndReorderedTour(candidate, penalty);
	return candidate;
}

void Optimizer::evaluateCurrentAndReorderedTour(ObjectPositionEvaluation& descent, double penalty)
{
	ObjectPositionEvaluation reordered = descent;
	this->swapInReconfigurationPostureMatches(descent);
	this->updateWeightedTotalCost(descent, penalty);
	reordered.order = this->reorderTourWithGtsp(reordered.placement, reordered.order);
	this->swapInReconfigurationPostureMatches(reordered);
	this->updateWeightedTotalCost(reordered, penalty);
	// Reach first: more viewpoints reached wins; at equal reach the lower weighted total cost.
	const bool reordered_reaches_more = reordered.placement.num_reached > descent.placement.num_reached;
	const bool reordered_cheaper = reordered.placement.num_reached == descent.placement.num_reached && reordered.weighted_total_cost < descent.weighted_total_cost;
	if (reordered_reaches_more || reordered_cheaper)
		descent = reordered;
}

void Optimizer::updateWeightedTotalCost(ObjectPositionEvaluation& descent, double penalty)
{
	this->unwrapFullTurnJointsAlongTour(descent.placement, descent.order);
	// Tour length, with each edge whose straight move collides charged its estimated detour (see getBlockedEdgeDetourCharges).
	double detour_charge = 0.0;
	const std::vector<std::pair<size_t, double>> blocked = this->getBlockedEdgeDetourCharges(descent.placement, descent.order);
	for (const std::pair<size_t, double>& edge : blocked)
		detour_charge += edge.second;
	descent.num_blocked_edges = static_cast<int>(blocked.size());
	std::vector<Joints> poses_from_home = this->getPosesInTourOrder(descent.placement, descent.order);
	poses_from_home.insert(poses_from_home.begin(), this->home_pose);
	descent.joint_travel_cost = joint_travel::TravelCost(poses_from_home, this->tour_settings.edge_weights, this->counted_joints) + detour_charge;
	descent.num_reconfigurations = this->countReconfigurations(descent.placement, descent.order);
	descent.posture_gap_sum = 0.0;
	for (const ReconfigurationPostureMatch& match : descent.matches)
		descent.posture_gap_sum += match.gap;
	descent.room_shortfall = 0.0;
	std::vector<Joints> reached_poses;
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		// Reached poses only: a missed viewpoint's closest pose presses on limits by design.
		if (!descent.placement.reached[v])
			continue;
		descent.room_shortfall += this->getJointRoomShortfall(descent.placement.chosen_ik_per_viewpoint[v].q, nullptr);
		reached_poses.push_back(descent.placement.chosen_ik_per_viewpoint[v].q);
	}
	descent.smallest_margin = this->getSmallestJointMargin(reached_poses);
	const int num_missed = static_cast<int>(this->viewpoints.size()) - descent.placement.num_reached;
	descent.weighted_total_cost = descent.joint_travel_cost + this->cost_weights.miss_cost * num_missed + penalty * descent.placement.gap_sum + this->cost_weights.reconfiguration_cost * descent.num_reconfigurations + this->cost_weights.reconfiguration_weight * descent.posture_gap_sum + this->cost_weights.joint_room_weight * descent.room_shortfall;
}

bool Optimizer::moveObjectToReachMore(ObjectPositionEvaluation& current, double& step, double penalty, int iteration)
{
	const Eigen::Vector3d aim = this->getReachDirection(current.placement);
	if (aim.norm() < 0.5)
		return false;
	Eigen::Vector3d direction = aim;
	bool tried_miss_room = false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && step >= this->step_sizes.min_step; ++attempt)
	{
		const ObjectPositionEvaluation candidate = this->evaluateObjectPosition(current.placement.position + step * direction, this->getChosenPoses(current.placement), {}, current.order, penalty);
		std::vector<size_t> lost;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
			if (current.placement.reached[v] && !candidate.placement.reached[v])
				lost.push_back(v);
		const bool kept = candidate.placement.num_reached > current.placement.num_reached || (candidate.placement.num_reached == current.placement.num_reached && candidate.placement.gap_sum < current.placement.gap_sum);
		RCLCPP_INFO(this->node->get_logger(), "iteration %d reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d (lost %zu)  gap sum %.4f m vs %.4f%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, lost.size(), candidate.placement.gap_sum, current.placement.gap_sum, kept ? "  <-- kept" : "");
		if (kept)
		{
			current = candidate;
			this->object_trail.push_back(current.placement.position);
			this->publishProgressMarkers(current.placement, current.order, direction);
			step = std::min(this->step_sizes.max_step, step * 1.5);
			// Misses nearly on target: say why they still miss.
			bool has_tiny_miss = false;
			for (size_t v = 0; v < this->viewpoints.size(); ++v)
				has_tiny_miss = has_tiny_miss || (!current.placement.reached[v] && current.placement.chosen_ik_per_viewpoint[v].gap < 1e-3);
			if (has_tiny_miss)
				this->logMissedViewpoints(current.placement);
			return true;
		}
		if (!lost.empty() && this->method_switches.use_room)
		{
			const Eigen::Vector3d room_direction = this->getRoomThenReachDirection(current.placement, lost, aim, step);
			if (room_direction.norm() > 0.5 && (room_direction - direction).norm() > 1e-3)
			{
				direction = room_direction;
				continue;
			}
		}
		// Room-then-reach for the misses: once per step, a move that frees what blocks them (joint limits, stretch,
		// collisions), kept if it raises their tightest margin and loses nothing; the next iteration aims again.
		if (this->method_switches.use_room && !tried_miss_room)
		{
			tried_miss_room = true;
			if (this->moveObjectToUnblockMisses(current, step, penalty, iteration))
				return true;
		}
		step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::improveArmPosesWithObjectFixed(ObjectPositionEvaluation& current, double penalty, int iteration)
{
	if (!this->method_switches.use_slide)
	{
		ObjectPositionEvaluation reordered = current;
		this->evaluateCurrentAndReorderedTour(reordered, penalty);
		const bool kept = reordered.weighted_total_cost < current.weighted_total_cost - 1e-9;
		if (kept)
			current = reordered;
		return kept;
	}
	const DescentDirection direction = this->getTourDirection(current);
	double slide_step = this->step_sizes.initial_slide_step;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && direction.has_slide; ++attempt)
	{
		const ObjectPositionEvaluation candidate = this->evaluateObjectPosition(current.placement.position, this->getPosesAfterJointSlides(current.placement, direction, slide_step), this->getChosenPoses(current.placement), current.order, penalty);
		const bool kept = candidate.weighted_total_cost < current.weighted_total_cost - 1e-9;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d poses: slide %.3f rad  reached %d  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, slide_step, candidate.placement.num_reached, candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.joint_travel_cost, candidate.smallest_margin, candidate.weighted_total_cost, current.weighted_total_cost, kept ? "  <-- kept" : "");
		if (kept)
		{
			current = candidate;
			return true;
		}
		slide_step *= 0.5;
	}
	return false;
}

bool Optimizer::moveObjectToLowerCost(ObjectPositionEvaluation& current, double& step, double penalty, int iteration)
{
	const DescentDirection direction = this->getTourDirection(current);
	if (direction.object_direction.norm() < 0.5)
		return false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && step >= this->step_sizes.min_step; ++attempt)
	{
		const ObjectPositionEvaluation candidate = this->evaluateObjectPosition(current.placement.position + step * direction.object_direction, this->getChosenPoses(current.placement), {}, current.order, penalty);
		const bool kept = candidate.weighted_total_cost < current.weighted_total_cost - 1e-9;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d tour: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, candidate.placement.gap_sum, candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.joint_travel_cost, candidate.smallest_margin, candidate.weighted_total_cost, current.weighted_total_cost, kept ? "  <-- kept" : "");
		if (kept)
		{
			current = candidate;
			this->object_trail.push_back(current.placement.position);
			this->publishProgressMarkers(current.placement, current.order, direction.object_direction);
			step = std::min(this->step_sizes.max_step, step * 1.5);
			return true;
		}
		step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::moveObjectToUnblockMisses(ObjectPositionEvaluation& current, double step, double penalty, int iteration)
{
	const double near_floors = 6.0;
	std::vector<size_t> blocked;
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		if (current.placement.reached[v])
			continue;
		const double gap = current.placement.chosen_ik_per_viewpoint[v].gap;
		const double blocked_share = gap < 1e-3 ? 1.0 : 1.0 - current.placement.chosen_ik_per_viewpoint[v].position_error.norm() / gap;
		if (blocked_share > 0.3)
			blocked.push_back(v);
	}
	if (blocked.empty())
		return false;
	const std::vector<room_margins::Margin> before = this->getBlockingMarginsOfViewpoints(current.placement, blocked, near_floors);
	if (before.empty())
		return false;
	const Eigen::Vector3d room_direction = room_margins::MostRoomDirection(before, step);
	if (room_direction.norm() < 0.5)
		return false;
	const ObjectPositionEvaluation candidate = this->evaluateObjectPosition(current.placement.position + step * room_direction, this->getChosenPoses(current.placement), {}, current.order, penalty);
	bool lost_any = false;
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
		lost_any = lost_any || (current.placement.reached[v] && !candidate.placement.reached[v]);
	const std::vector<room_margins::Margin> after = this->getBlockingMarginsOfViewpoints(candidate.placement, blocked, near_floors);
	const double tightest_before = room_margins::Tightest(before);
	const double tightest_after = after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(after);
	const bool kept = !lost_any && candidate.placement.num_reached >= current.placement.num_reached && tightest_after > tightest_before + 1e-3;
	RCLCPP_INFO(this->node->get_logger(), "iteration %d reach room: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  blocked misses %zu  tightest margin %.2f -> %.2f floors%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, blocked.size(), tightest_before, tightest_after, kept ? "  <-- kept" : "");
	if (!kept)
		return false;
	current = candidate;
	this->object_trail.push_back(current.placement.position);
	this->publishProgressMarkers(current.placement, current.order, room_direction);
	return true;
}

Eigen::Vector3d Optimizer::getReachDirection(const ViewpointIkAtPosition& placement)
{
	Eigen::Vector3d reach_move = Eigen::Vector3d::Zero();
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		if (placement.reached[v])
			continue;
		const double gap = placement.chosen_ik_per_viewpoint[v].gap;
		// The closest pose is a minimum over the joints, so only the gap's position part moves with the object.
		if (gap > 1e-9)
			reach_move -= placement.chosen_ik_per_viewpoint[v].position_error / gap;
		// Room blended in (room_in_reach_direction): a miss on target (gap under 1 mm) is all blocked. Off by default, room
		// then comes only as its own tries (moveObjectToUnblockMisses, getRoomThenReachDirection): its straight-line guess can point
		// the object the wrong way, and blended into every reach move it bends the whole path.
		const double blocked_share = gap < 1e-3 ? 1.0 : 1.0 - placement.chosen_ik_per_viewpoint[v].position_error.norm() / gap;
		if (this->method_switches.use_room && this->method_switches.room_in_reach_direction && blocked_share > 1e-3)
			reach_move += blocked_share * this->getRoomDirectionOfMissedViewpoint(placement, v);
	}
	return reach_move.norm() > 1e-12 ? Eigen::Vector3d(reach_move.normalized()) : Eigen::Vector3d::Zero();
}

DescentDirection Optimizer::getTourDirection(const ObjectPositionEvaluation& descent)
{
	const ViewpointIkAtPosition& placement = descent.placement;
	const std::vector<Eigen::VectorXd> pose_slopes = this->getTourCostSlopePerPose(placement, descent.order);
	Eigen::MatrixXd object_twist = Eigen::MatrixXd::Zero(6, 3);
	object_twist.topRows<3>() = Eigen::Matrix3d::Identity();
	Eigen::Vector3d reconfiguration_move = Eigen::Vector3d::Zero();
	Eigen::Vector3d length_move = Eigen::Vector3d::Zero();
	Eigen::Vector3d room_move = Eigen::Vector3d::Zero();
	DescentDirection direction;
	direction.joint_slides.resize(this->viewpoints.size());
	double largest_slide = 0.0;
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		const Eigen::MatrixXd jacobian = this->getToolJacobian(*this->state, placement.chosen_ik_per_viewpoint[v].q);
		const Eigen::MatrixXd follow_rate = scene_geometry::DampedPinv(jacobian, this->ik_settings.jacobian_damping) * object_twist;
		Eigen::VectorXd room_slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(placement.chosen_ik_per_viewpoint[v].q.size()));
		if (placement.reached[v])
			this->getJointRoomShortfall(placement.chosen_ik_per_viewpoint[v].q, &room_slope);
		length_move -= follow_rate.transpose() * pose_slopes[v];
		room_move -= this->cost_weights.joint_room_weight * follow_rate.transpose() * room_slope;
		if (!this->method_switches.use_slide || !placement.reached[v])
			continue;
		direction.joint_slides[v] = -this->getSelfMotionProjector(jacobian) * (pose_slopes[v] + this->cost_weights.joint_room_weight * room_slope);
		largest_slide = std::max(largest_slide, direction.joint_slides[v].norm());
	}
	for (Eigen::VectorXd& slide : direction.joint_slides)
		if (slide.size() > 0)
			slide = largest_slide > 1e-9 ? Eigen::VectorXd(slide / largest_slide) : Eigen::VectorXd();
	for (const ReconfigurationPostureMatch& match : descent.matches)
	{
		if (match.gap > 1e-9)
			reconfiguration_move -= this->cost_weights.reconfiguration_weight * match.position_error / match.gap;
		if (!this->method_switches.use_slide || !placement.reached[match.viewpoint_index])
			continue;
		// Toward the neighbour's joints along the self-motion, full slide step.
		const Eigen::VectorXd toward = Eigen::Map<const Eigen::VectorXd>(match.neighbor_pose.data(), static_cast<Eigen::Index>(match.neighbor_pose.size())) - Eigen::Map<const Eigen::VectorXd>(placement.chosen_ik_per_viewpoint[match.viewpoint_index].q.data(), static_cast<Eigen::Index>(match.neighbor_pose.size()));
		const Eigen::VectorXd slide = this->getSelfMotionProjector(this->getToolJacobian(*this->state, placement.chosen_ik_per_viewpoint[match.viewpoint_index].q)) * toward;
		if (slide.norm() > 1e-9)
			direction.joint_slides[match.viewpoint_index] = slide.normalized();
	}
	const Eigen::Vector3d object_move = this->combineMovesWithoutConflict({reconfiguration_move, length_move, room_move});
	if (object_move.norm() > 1e-12)
		direction.object_direction = object_move.normalized();
	direction.has_slide = std::any_of(direction.joint_slides.begin(), direction.joint_slides.end(), [](const Eigen::VectorXd& slide) { return slide.size() > 0; });
	return direction;
}

Eigen::Vector3d Optimizer::getRoomThenReachDirection(const ViewpointIkAtPosition& placement, const std::vector<size_t>& lost, const Eigen::Vector3d& aim, double step)
{
	const double lost_margin_floors = 6.0;  // margins up to this many floors count as at risk
	const std::vector<room_margins::Margin> margins = this->getBlockingMarginsOfViewpoints(placement, lost, lost_margin_floors);
	if (margins.empty())
		return Eigen::Vector3d::Zero();
	Eigen::Vector3d direction = aim;
	for (int round = 0; round < 20; ++round)
	{
		bool changed = false;
		for (const room_margins::Margin& margin : margins)
		{
			const double rate = margin.slope.dot(direction);
			if (rate >= -1e-12 || margin.slope.squaredNorm() < 1e-24)
				continue;
			direction -= rate / margin.slope.squaredNorm() * margin.slope;
			changed = true;
		}
		if (!changed)
			break;
	}
	if (direction.norm() < 0.3 * aim.norm())
		return room_margins::MostRoomDirection(margins, step);
	return direction.normalized();
}

Eigen::Vector3d Optimizer::getRoomDirectionOfMissedViewpoint(const ViewpointIkAtPosition& placement, size_t v)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	const std::vector<room_margins::Margin> margins = this->getBlockingMarginsOfPose(v, placement.chosen_ik_per_viewpoint[v].q);
	if (margins.empty())
		return Eigen::Vector3d::Zero();
	return room_margins::MostRoomDirection(margins, this->step_sizes.initial_step);
}

Eigen::Vector3d Optimizer::combineMovesWithoutConflict(const std::vector<Eigen::Vector3d>& moves) const
{
	Eigen::Vector3d total = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < moves.size(); ++i)
	{
		Eigen::Vector3d move = moves[i];
		for (size_t j = 0; j < moves.size(); ++j)
		{
			const double other_size = moves[j].squaredNorm();
			if (j == i || other_size < 1e-24)
				continue;
			const double against = move.dot(moves[j]);
			if (against < 0.0)
				move -= against / other_size * moves[j];
		}
		total += move;
	}
	return total;
}

std::vector<Eigen::VectorXd> Optimizer::getTourCostSlopePerPose(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const
{
	std::vector<Eigen::VectorXd> pose_slopes(this->viewpoints.size(), Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size())));
	for (size_t k = 0; k < order.size(); ++k)
	{
		const size_t to = static_cast<size_t>(order[k]);
		const Joints& from_pose = k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q;
		// d(edge cost)/d(to) by central differences; the cost depends on to - from only, so d/d(from) is its negative.
		const double delta = 1e-5;
		Eigen::VectorXd slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size()));
		Joints shifted = placement.chosen_ik_per_viewpoint[to].q;
		for (size_t j = 0; j < shifted.size(); ++j)
		{
			const double value = shifted[j];
			shifted[j] = value + delta;
			const double cost_up = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[j] = value - delta;
			const double cost_down = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[j] = value;
			slope[static_cast<Eigen::Index>(j)] = (cost_up - cost_down) / (2.0 * delta);
		}
		pose_slopes[to] += slope;
		if (k > 0)
			pose_slopes[static_cast<size_t>(order[k - 1])] -= slope;
	}
	return pose_slopes;
}

std::vector<Joints> Optimizer::getPosesAfterJointSlides(const ViewpointIkAtPosition& placement, const DescentDirection& direction, double slide_step) const
{
	std::vector<Joints> slid_poses = this->getChosenPoses(placement);
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	for (size_t v = 0; v < slid_poses.size(); ++v)
	{
		if (direction.joint_slides[v].size() == 0)
			continue;
		for (size_t j = 0; j < slid_poses[v].size(); ++j)
		{
			const moveit::core::VariableBounds& bounds = joints[j]->getVariableBounds()[0];
			const double value = slid_poses[v][j];
			slid_poses[v][j] += slide_step * direction.joint_slides[v][static_cast<Eigen::Index>(j)];
			if (bounds.position_bounded_)
				slid_poses[v][j] = std::clamp(slid_poses[v][j], std::min(value, bounds.min_position_ + this->step_sizes.slide_joint_margin), std::max(value, bounds.max_position_ - this->step_sizes.slide_joint_margin));
		}
	}
	return slid_poses;
}

Eigen::MatrixXd Optimizer::getToolJacobian(moveit::core::RobotState& state, const Joints& q) const
{
	Eigen::MatrixXd jacobian = manipulability::ToolJacobian(state, this->group, this->tool, q);
	if (this->ik_settings.exact_options.ignore_roll)
	{
		const Eigen::Vector3d axis = state.getGlobalLinkTransform(this->tool).linear().col(2);
		jacobian.bottomRows<3>() = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * jacobian.bottomRows<3>();
	}
	return jacobian;
}

Eigen::MatrixXd Optimizer::getSelfMotionProjector(const Eigen::MatrixXd& jacobian) const
{
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
	const Eigen::VectorXd& singular_values = svd.singularValues();
	const double tolerance = 1e-6 * std::max(1.0, singular_values.size() > 0 ? singular_values[0] : 0.0);
	const Eigen::Index rank = (singular_values.array() > tolerance).count();
	const Eigen::MatrixXd null_basis = svd.matrixV().rightCols(jacobian.cols() - rank);
	return null_basis * null_basis.transpose();
}

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfViewpoints(const ViewpointIkAtPosition& placement, const std::vector<size_t>& misses, double near_floors)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	std::vector<room_margins::Margin> margins;
	for (size_t v : misses)
	{
		const std::vector<room_margins::Margin> mine = this->getBlockingMarginsOfPose(v, placement.chosen_ik_per_viewpoint[v].q, near_floors);
		margins.insert(margins.end(), mine.begin(), mine.end());
	}
	return margins;
}

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfPose(size_t v, const Joints& q, double near_floors)
{
	const double joint_floor = 0.05;  // rad
	const double stretch_floor = 0.02;
	const double collision_floor = 0.01;  // m
	const double window = this->ik_settings.closest_options.margin + near_floors * collision_floor;  // m, collision pairs looked at
	const int dof = static_cast<int>(q.size());
	const Eigen::MatrixXd dq_dp = scene_geometry::DampedPinv(manipulability::ToolJacobian(*this->state, this->group, this->tool, q), this->ik_settings.jacobian_damping).leftCols<3>();
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	std::vector<room_margins::Margin> margins;
	for (int i = 0; i < dof; ++i)
	{
		const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(i)]->getVariableBounds()[0];
		if (!bounds.position_bounded_)
			continue;
		const Eigen::VectorXd unit = Eigen::VectorXd::Unit(dof, i);
		const std::string joint_name = joints[static_cast<size_t>(i)]->getName();
		const double low = q[static_cast<size_t>(i)] - bounds.min_position_;
		const double high = bounds.max_position_ - q[static_cast<size_t>(i)];
		if (low < near_floors * joint_floor)
			margins.push_back({static_cast<int>(v), joint_name + " low limit", low, joint_floor, dq_dp.transpose() * unit, unit});
		if (high < near_floors * joint_floor)
			margins.push_back({static_cast<int>(v), joint_name + " high limit", high, joint_floor, dq_dp.transpose() * -unit, -unit});
	}
	const double stretch = this->getSmallestJacobianSingularValue(q);
	if (stretch < near_floors * stretch_floor)
	{
		const double delta = 1e-5;  // rad
		Eigen::VectorXd stretch_slope(dof);
		for (int i = 0; i < dof; ++i)
		{
			Joints shifted = q;
			shifted[static_cast<size_t>(i)] += delta;
			stretch_slope(i) = (this->getSmallestJacobianSingularValue(shifted) - stretch) / delta;
		}
		margins.push_back({static_cast<int>(v), "stretch", stretch, stretch_floor, dq_dp.transpose() * stretch_slope, stretch_slope});
	}
	this->state->setJointGroupPositions(this->group, q);
	this->state->update();
	const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, window), window);
	for (size_t k = 0; k < pairs.dists.size(); ++k)
	{
		const Eigen::VectorXd joint_slope = pairs.gradients[k].transpose();
		margins.push_back({static_cast<int>(v), "collision " + pairs.names[k], pairs.dists[k], collision_floor, dq_dp.transpose() * joint_slope + pairs.object_slopes[k], joint_slope});
	}
	return margins;
}

double Optimizer::getSmallestJacobianSingularValue(const Joints& q)
{
	return Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(*this->state, this->group, this->tool, q)).singularValues().minCoeff();
}

double Optimizer::getJointRoomShortfall(const Joints& q, Eigen::VectorXd* slope) const
{
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	if (slope != nullptr)
		*slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(q.size()));
	double shortfall = 0.0;
	for (int j : this->counted_joints)
	{
		const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(j)]->getVariableBounds()[0];
		if (!bounds.position_bounded_)
			continue;
		const double value = q[static_cast<size_t>(j)];
		const double low = value - bounds.min_position_;
		const double high = bounds.max_position_ - value;
		if (low < this->cost_weights.joint_room_band)
		{
			shortfall += (this->cost_weights.joint_room_band - low) * (this->cost_weights.joint_room_band - low);
			if (slope != nullptr)
				(*slope)[j] -= 2.0 * (this->cost_weights.joint_room_band - low);
		}
		if (high < this->cost_weights.joint_room_band)
		{
			shortfall += (this->cost_weights.joint_room_band - high) * (this->cost_weights.joint_room_band - high);
			if (slope != nullptr)
				(*slope)[j] += 2.0 * (this->cost_weights.joint_room_band - high);
		}
	}
	return shortfall;
}

ViewpointIkAtPosition Optimizer::solveViewpointIkAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(position, this->object_rotation));
	ViewpointIkAtPosition placement;
	placement.position = position;
	placement.chosen_ik_per_viewpoint.assign(this->viewpoints.size(), ClosestIkCandidate());
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
		placement.chosen_ik_per_viewpoint[v].q = seeds[v];
	placement.reached.assign(this->viewpoints.size(), false);
	placement.closest_ik_candidates.assign(this->viewpoints.size(), {});
	std::vector<std::thread> threads;
	for (unsigned int t = 0; t < this->num_threads; ++t)
		threads.emplace_back([&, t]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t v = t; v < this->viewpoints.size(); v += this->num_threads)
			{
				// Random starts seeded by viewpoint and spot (0.1 mm grid): the same spot always gives the same poses.
				random_numbers::RandomNumberGenerator viewpoint_rng(this->getRandomIkSeed(position, v));
				const Eigen::Isometry3d target = scene_geometry::Translation(position) * this->viewpoints[v];
				closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, seeds[v], this->ik_settings.exact_options);
				if (!result.exact && v < fallback_seeds.size())
					result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, fallback_seeds[v], this->ik_settings.exact_options);
				std::vector<Joints> random_starts;
				for (int attempt = 0; attempt < this->ik_settings.ik_attempts && !result.exact; ++attempt)
				{
					Joints random_start;
					thread_state.setToRandomPositions(this->group, viewpoint_rng);
					thread_state.copyJointGroupPositions(this->group, random_start);
					random_starts.push_back(random_start);
					result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, random_start, this->ik_settings.exact_options);
				}
				if (!result.exact)
				{
					// Closest poses from its own pose and a few of the random starts (other postures). Those near the best gap
					// and with room are kept for GTSP to choose by tour; the one from its own pose is the default.
					std::vector<Joints> closest_starts = {seeds[v]};
					for (size_t k = 0; k < random_starts.size() && static_cast<int>(closest_starts.size()) <= this->ik_settings.closest_ik_extra_starts; ++k)
						closest_starts.push_back(random_starts[k]);
					const std::vector<closest_ik::Result> closest = this->solveClosestIkFromStarts(thread_state, target, closest_starts);
					result = closest_ik::Result();
					for (const closest_ik::Result& candidate : closest)
						if (candidate.exact)
							result = candidate;
					if (!result.exact && !closest.empty())
					{
						placement.closest_ik_candidates[v] = this->keepCandidatesNearBestGap(thread_state, closest);
						result = closest_ik::Result();
						result.found = true;
						result.q = placement.closest_ik_candidates[v].front().q;
					}
				}
				if (!result.found)
				{
					// No collision-free pose near: keep the last one and a fixed large gap, no pull.
					placement.chosen_ik_per_viewpoint[v].gap = this->ik_settings.closest_options.margin + 1.0;
					continue;
				}
				placement.reached[v] = result.exact;
				if (result.exact)
					placement.chosen_ik_per_viewpoint[v].q = result.q;
				else
					placement.chosen_ik_per_viewpoint[v] = placement.closest_ik_candidates[v].front();
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		placement.num_reached += placement.reached[v] ? 1 : 0;
		placement.gap_sum += placement.chosen_ik_per_viewpoint[v].gap;
	}
	return placement;
}

std::vector<closest_ik::Result> Optimizer::solveClosestIkFromStarts(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts)
{
	std::vector<closest_ik::Result> results;
	for (const Joints& start : starts)
	{
		closest_ik::Result result = closest_ik::Solve(this->scene, state, this->group, this->tool, target, start, this->ik_settings.closest_options);
		if (!result.found)
			continue;
		if (this->getClearanceShortfall(state, result.q) > 0.0)
			result = this->pushArmToExactClearance(state, target, result);
		else if (!result.exact && result.gap < this->ik_settings.exact_finish_gap)
		{
			// Close to the target: exact IK started from the closest pose (every exact check applies).
			const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, result.q, this->ik_settings.exact_options);
			if (exact.exact)
				result = exact;
		}
		results.push_back(result);
		if (result.exact)
			break;
	}
	return results;
}

std::vector<ClosestIkCandidate> Optimizer::keepCandidatesNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest)
{
	std::vector<ClosestIkCandidate> all;
	std::vector<bool> roomy;
	double best_gap = std::numeric_limits<double>::max();
	for (const closest_ik::Result& result : closest)
	{
		ClosestIkCandidate candidate;
		candidate.q = result.q;
		candidate.position_error = result.pose_error.head<3>();
		candidate.angle_error = result.pose_error.tail<3>().norm();
		candidate.clearance_shortfall = this->getClearanceShortfall(state, result.q);
		candidate.gap = scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + candidate.clearance_shortfall;
		best_gap = std::min(best_gap, candidate.gap);
		roomy.push_back(candidate.clearance_shortfall <= 0.0 && this->getSmallestJointMargin({result.q}) >= this->cost_weights.joint_room_band);
		all.push_back(candidate);
	}
	std::vector<ClosestIkCandidate> near, near_roomy;
	for (size_t k = 0; k < all.size(); ++k)
	{
		if (all[k].gap > best_gap + this->ik_settings.closest_gap_slack)
			continue;
		near.push_back(all[k]);
		if (roomy[k])
			near_roomy.push_back(all[k]);
	}
	return near_roomy.empty() ? near : near_roomy;
}

closest_ik::Result Optimizer::solveSmallestGapIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts)
{
	closest_ik::Result best;
	double best_gap = std::numeric_limits<double>::max();
	for (const closest_ik::Result& result : this->solveClosestIkFromStarts(state, target, starts))
	{
		if (result.exact)
			return result;
		const double gap = scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + this->getClearanceShortfall(state, result.q);
		if (gap < best_gap)
		{
			best = result;
			best_gap = gap;
		}
	}
	return best;
}

closest_ik::Result Optimizer::pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest)
{
	const double margin = this->ik_settings.exact_options.exact_margin;
	if (margin <= 0.0)
		return closest;
	const double goal_clearance = margin + 0.002;  // m: a little past the exact poses' clearance
	const double max_joint_step = 0.05;	 // rad per iteration
	const int max_iterations = 40;
	Joints q = closest.q;
	for (int iteration = 0; iteration < max_iterations; ++iteration)
	{
		state.setJointGroupPositions(this->group, q);
		state.update();
		const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, goal_clearance), goal_clearance);
		Eigen::VectorXd push = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(q.size()));
		for (size_t k = 0; k < pairs.names.size(); ++k)
		{
			const size_t split = pairs.names[k].find(" - ");
			if (this->camera_side_links.count(pairs.names[k].substr(0, split)) || this->camera_side_links.count(pairs.names[k].substr(split + 3)))
				continue;
			const Eigen::VectorXd gradient = pairs.gradients[k].transpose();
			push += (goal_clearance - pairs.dists[k]) * gradient / (gradient.squaredNorm() + 1e-9);
		}
		if (push.norm() < 1e-9)
			break;
		const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->ik_settings.exact_options.ignore_roll);
		const Eigen::MatrixXd jacobian = this->getToolJacobian(state, q);
		const Eigen::MatrixXd pseudo_inverse = scene_geometry::DampedPinv(jacobian, this->ik_settings.jacobian_damping);
		const Eigen::MatrixXd self_motion = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - pseudo_inverse * jacobian;
		Eigen::VectorXd step = pseudo_inverse * pose_error + self_motion * push;
		if (step.cwiseAbs().maxCoeff() > max_joint_step)
			step *= max_joint_step / step.cwiseAbs().maxCoeff();
		for (size_t j = 0; j < q.size(); ++j)
			q[j] += step[static_cast<Eigen::Index>(j)];
		state.setJointGroupPositions(this->group, q);
		state.enforceBounds(this->group);
		state.copyJointGroupPositions(this->group, q);
	}
	const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, q, this->ik_settings.exact_options);
	if (exact.exact)
		return exact;
	if (!scene_geometry::IsStateValid(this->scene, &state, this->group, q.data()))
		return closest;
	state.setJointGroupPositions(this->group, q);
	state.update();
	closest_ik::Result finished;
	finished.found = true;
	finished.exact = false;
	finished.pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->ik_settings.exact_options.ignore_roll);
	finished.gap = scene_geometry::PoseGap(finished.pose_error, this->ik_settings.closest_options.rot_scale);
	finished.q = q;
	const double finished_gap = finished.gap + this->getClearanceShortfall(state, q);
	const double closest_gap = scene_geometry::PoseGap(closest.pose_error, this->ik_settings.closest_options.rot_scale) + this->getClearanceShortfall(state, closest.q);
	return finished_gap < closest_gap ? finished : closest;
}

double Optimizer::getClearanceShortfall(moveit::core::RobotState& state, const Joints& q) const
{
	const double margin = this->ik_settings.exact_options.exact_margin;
	if (margin <= 0.0)
		return 0.0;
	state.setJointGroupPositions(this->group, q);
	state.update();
	const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, margin), margin);
	double clearance = margin;  // smallest arm link distance (camera side aside), looked for up to the margin
	for (size_t k = 0; k < pairs.names.size(); ++k)
	{
		const size_t split = pairs.names[k].find(" - ");
		if (this->camera_side_links.count(pairs.names[k].substr(0, split)) || this->camera_side_links.count(pairs.names[k].substr(split + 3)))
			continue;
		clearance = std::min(clearance, pairs.dists[k]);
	}
	return std::max(0.0, margin - clearance);
}

uint32_t Optimizer::getRandomIkSeed(const Eigen::Vector3d& position, size_t v) const
{
	const Eigen::Vector3d cell = (position * 1e4).array().round();
	uint64_t hash = static_cast<uint64_t>(this->ik_settings.random_ik_seed) * 1000003ULL + static_cast<uint64_t>(v) * 7919ULL;
	for (int axis = 0; axis < 3; ++axis)
		hash = hash * 6364136223846793005ULL + static_cast<uint64_t>(static_cast<int64_t>(cell[axis]));
	return static_cast<uint32_t>(hash ^ (hash >> 32));
}

std::vector<Joints> Optimizer::getChosenPoses(const ViewpointIkAtPosition& placement) const
{
	std::vector<Joints> poses;
	for (const ClosestIkCandidate& chosen_ik : placement.chosen_ik_per_viewpoint)
		poses.push_back(chosen_ik.q);
	return poses;
}

std::vector<int> Optimizer::reorderTourWithGtsp(ViewpointIkAtPosition& placement, const std::vector<int>& order)
{
	std::vector<std::vector<Joints>> candidates(this->viewpoints.size());
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		candidates[v].push_back(placement.chosen_ik_per_viewpoint[v].q);
		// A missed viewpoint's other closest poses (near its best gap, with room): GTSP picks by tour.
		if (!placement.reached[v] && v < placement.closest_ik_candidates.size())
			for (ClosestIkCandidate& candidate : placement.closest_ik_candidates[v])
			{
				// Same turn as the current pose (see unwrapFullTurnJointsAlongTour), so the GTSP choice can be matched back to its gap below.
				this->unwrapFullTurnJointsToward(candidate.q, placement.chosen_ik_per_viewpoint[v].q);
				if (candidate.q != placement.chosen_ik_per_viewpoint[v].q)
					candidates[v].push_back(candidate.q);
			}
	}
	if (this->method_switches.use_neighbor_ik && order.size() == this->viewpoints.size())
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
		std::vector<std::thread> threads;
		for (unsigned int t = 0; t < this->num_threads; ++t)
			threads.emplace_back([&, t]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t k = t; k < order.size(); k += this->num_threads)
				{
					const size_t v = static_cast<size_t>(order[k]);
					if (!placement.reached[v])
						continue;
					const Eigen::Isometry3d target = scene_geometry::Translation(placement.position) * this->viewpoints[v];
					std::vector<const Joints*> neighbors = {k == 0 ? &this->home_pose : &placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q};
					if (k + 1 < order.size())
						neighbors.push_back(&placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k + 1])].q);
					for (const Joints* neighbor : neighbors)
					{
						const closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *neighbor, this->ik_settings.exact_options);
						if (result.exact)
							candidates[v].push_back(result.q);
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
	}
	// Candidates in the same turn as the viewpoint's current pose, so GTSP doesn't see fake full-turn swings.
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
		for (Joints& candidate : candidates[v])
			this->unwrapFullTurnJointsToward(candidate, placement.chosen_ik_per_viewpoint[v].q);
	gtsp::Solver solver(candidates, this->home_pose, this->tour_settings.edge_weights, this->tour_settings.two_opt_rounds);
	solver.SetTravelJoints(this->counted_joints);
	gtsp::Tour tour = solver.Solve(order);
	// GTSP sees straight-line lengths only: charge the chosen edges whose straight move collides and solve again, so the
	// order can route around them (checking every pair up front would be too slow).
	for (int round = 0; round < this->tour_settings.gtsp_blocked_rounds && this->cost_weights.blocked_edge_charge > 0.0; ++round)
	{
		ViewpointIkAtPosition chosen = placement;
		for (size_t k = 0; k < tour.order.size(); ++k)
			chosen.chosen_ik_per_viewpoint[static_cast<size_t>(tour.order[k])].q = tour.poses[k];
		const std::vector<std::pair<size_t, double>> blocked = this->getBlockedEdgeDetourCharges(chosen, tour.order);
		if (blocked.empty())
			break;
		for (const std::pair<size_t, double>& edge : blocked)
		{
			const size_t k = edge.first;
			const size_t to = static_cast<size_t>(tour.order[k]);
			const std::pair<int, int> to_key = {static_cast<int>(to), this->findCandidateIndex(candidates[to], tour.poses[k])};
			const std::pair<int, int> from_key = k == 0 ? std::make_pair(-1, -1) : std::make_pair(tour.order[k - 1], this->findCandidateIndex(candidates[static_cast<size_t>(tour.order[k - 1])], tour.poses[k - 1]));
			solver.AddEdgeCost(from_key, to_key, edge.second);
		}
		tour = solver.Solve(tour.order);
	}
	for (size_t k = 0; k < tour.order.size(); ++k)
	{
		const size_t v = static_cast<size_t>(tour.order[k]);
		placement.chosen_ik_per_viewpoint[v].q = tour.poses[k];
		if (placement.reached[v] || v >= placement.closest_ik_candidates.size())
			continue;
		// A missed viewpoint's chosen closest pose brings its own gap.
		for (const ClosestIkCandidate& candidate : placement.closest_ik_candidates[v])
		{
			if (candidate.q != tour.poses[k])
				continue;
			placement.gap_sum += candidate.gap - placement.chosen_ik_per_viewpoint[v].gap;
			placement.chosen_ik_per_viewpoint[v] = candidate;
		}
	}
	return tour.order;
}

int Optimizer::findCandidateIndex(const std::vector<Joints>& candidates, const Joints& pose) const
{
	for (size_t i = 0; i < candidates.size(); ++i)
		if (candidates[i] == pose)
			return static_cast<int>(i);
	return 0;
}

void Optimizer::unwrapFullTurnJointsAlongTour(ViewpointIkAtPosition& placement, const std::vector<int>& order) const
{
	const Joints* previous = &this->home_pose;
	for (int viewpoint : order)
	{
		Joints& pose = placement.chosen_ik_per_viewpoint[static_cast<size_t>(viewpoint)].q;
		this->unwrapFullTurnJointsToward(pose, *previous);
		previous = &pose;
	}
}

void Optimizer::unwrapFullTurnJointsToward(Joints& pose, const Joints& reference) const
{
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	const double full_turn = 2.0 * M_PI;
	for (size_t j = 0; j < pose.size() && j < joints.size() && j < reference.size(); ++j)
	{
		const moveit::core::VariableBounds& bounds = joints[j]->getVariableBounds()[0];
		if (bounds.position_bounded_ && bounds.max_position_ - bounds.min_position_ < full_turn)
			continue;
		const double turns = std::round((reference[j] - pose[j]) / full_turn);
		for (double shift : {turns, turns - 1.0, turns + 1.0})
		{
			const double value = pose[j] + shift * full_turn;
			const bool inside = !bounds.position_bounded_ || (value >= bounds.min_position_ && value <= bounds.max_position_);
			if (inside && std::abs(value - reference[j]) < std::abs(pose[j] - reference[j]))
				pose[j] = value;
		}
	}
}

bool Optimizer::isReconfiguration(const Joints& from, const Joints& to) const
{
	return joint_travel::MaxJointTurn(from, to, this->counted_joints) > this->tour_settings.reconfig_threshold;
}

int Optimizer::countReconfigurations(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const
{
	int count = 0;
	for (size_t k = 0; k < order.size(); ++k)
		count += this->isReconfiguration(k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q, placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k])].q) ? 1 : 0;
	return count;
}

int Optimizer::countReconfigurationsAround(const ViewpointIkAtPosition& placement, const std::vector<int>& order, size_t k, const Joints& pose) const
{
	const Joints& before = k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q;
	int count = this->isReconfiguration(before, pose) ? 1 : 0;
	if (k + 1 < order.size())
		count += this->isReconfiguration(pose, placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k + 1])].q) ? 1 : 0;
	return count;
}

void Optimizer::swapInReconfigurationPostureMatches(ObjectPositionEvaluation& descent)
{
	this->unwrapFullTurnJointsAlongTour(descent.placement, descent.order);
	descent.matches.clear();
	if (!this->method_switches.use_posture_match)
		return;
	ViewpointIkAtPosition& placement = descent.placement;
	const std::vector<int>& order = descent.order;
	std::vector<size_t> position_of(this->viewpoints.size(), 0);
	for (size_t k = 0; k < order.size(); ++k)
		position_of[static_cast<size_t>(order[k])] = k;
	for (int round = 0; round < 2; ++round)
	{
		descent.matches = this->findReconfigurationPostureMatches(placement, order);
		bool swapped = false;
		for (const ReconfigurationPostureMatch& match : descent.matches)
		{
			const size_t k = position_of[match.viewpoint_index];
			const int before = this->countReconfigurationsAround(placement, order, k, placement.chosen_ik_per_viewpoint[match.viewpoint_index].q);
			const int after = this->countReconfigurationsAround(placement, order, k, match.pose_in_neighbor_posture);
			const bool removes_reconfiguration = match.exact && after < before;
			const bool closer_miss = !placement.reached[match.viewpoint_index] && !match.exact && match.gap < placement.chosen_ik_per_viewpoint[match.viewpoint_index].gap && after <= before;
			if (!removes_reconfiguration && !closer_miss)
				continue;
			placement.chosen_ik_per_viewpoint[match.viewpoint_index].q = match.pose_in_neighbor_posture;
			if (match.exact && !placement.reached[match.viewpoint_index])
			{
				placement.reached[match.viewpoint_index] = true;
				placement.num_reached += 1;
			}
			const double new_gap = match.exact ? 0.0 : match.gap;
			placement.gap_sum += new_gap - placement.chosen_ik_per_viewpoint[match.viewpoint_index].gap;
			placement.chosen_ik_per_viewpoint[match.viewpoint_index].gap = new_gap;
			placement.chosen_ik_per_viewpoint[match.viewpoint_index].clearance_shortfall = match.clearance_shortfall;
			placement.chosen_ik_per_viewpoint[match.viewpoint_index].angle_error = match.angle_error;
			placement.chosen_ik_per_viewpoint[match.viewpoint_index].position_error = match.exact ? Eigen::Vector3d::Zero() : match.position_error;
			swapped = true;
		}
		if (!swapped)
			return;
	}
	descent.matches = this->findReconfigurationPostureMatches(placement, order);
}

std::vector<ReconfigurationPostureMatch> Optimizer::findReconfigurationPostureMatches(const ViewpointIkAtPosition& placement, const std::vector<int>& order)
{
	std::vector<size_t> edges;  // tour position k of each reconfiguration edge order[k-1] (or home) -> order[k]
	for (size_t k = 0; k < order.size(); ++k)
		if (this->isReconfiguration(k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q, placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k])].q))
			edges.push_back(k);
	std::vector<ReconfigurationPostureMatch> matches(edges.size());
	std::vector<char> found(edges.size(), 0);  // one byte per edge: written from parallel threads
	if (edges.empty())
		return {};
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(edges.size()));
	std::vector<std::thread> threads;
	for (unsigned int t = 0; t < num_workers; ++t)
		threads.emplace_back([&, t]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t e = t; e < edges.size(); e += num_workers)
			{
				const size_t k = edges[e];
				const size_t to = static_cast<size_t>(order[k]);
				const Joints& from_pose = k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q;
				// (viewpoint, neighbour pose): B in A's posture, and A in B's (home has no viewpoint to move).
				std::vector<std::pair<size_t, const Joints*>> options = {{to, &from_pose}};
				if (k > 0)
					options.push_back({static_cast<size_t>(order[k - 1]), &placement.chosen_ik_per_viewpoint[to].q});
				for (const std::pair<size_t, const Joints*>& option : options)
				{
					const Eigen::Isometry3d target = scene_geometry::Translation(placement.position) * this->viewpoints[option.first];
					closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *option.second, this->ik_settings.exact_options);
					if (!result.exact)
						result = this->solveSmallestGapIk(thread_state, target, {*option.second});
					if (!result.found || this->isReconfiguration(*option.second, result.q))
						continue;
					const double clearance_shortfall = result.exact ? 0.0 : this->getClearanceShortfall(thread_state, result.q);
					const double gap = result.exact ? 0.0 : scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + clearance_shortfall;
					if (found[e] && gap >= matches[e].gap)
						continue;
					found[e] = true;
					matches[e].viewpoint_index = option.first;
					matches[e].neighbor_pose = *option.second;
					matches[e].pose_in_neighbor_posture = result.q;
					matches[e].position_error = result.exact ? Eigen::Vector3d::Zero() : Eigen::Vector3d(result.pose_error.head<3>());
					matches[e].gap = gap;
					matches[e].clearance_shortfall = clearance_shortfall;
					matches[e].angle_error = result.exact ? 0.0 : result.pose_error.tail<3>().norm();
					matches[e].exact = result.exact;
				}
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	std::vector<ReconfigurationPostureMatch> kept;
	for (size_t e = 0; e < edges.size(); ++e)
		if (found[e])
			kept.push_back(matches[e]);
	return kept;
}

bool Optimizer::isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& a, const Joints& b) const
{
	const double check_step = 0.02;	 // rad
	double largest = 0.0;
	for (size_t j = 0; j < a.size(); ++j)
		largest = std::max(largest, std::abs(b[j] - a[j]));
	const int steps = std::max(1, static_cast<int>(std::ceil(largest / check_step)));
	Joints q(a.size());
	for (int i = 1; i < steps; ++i)
	{
		const double fraction = static_cast<double>(i) / steps;
		for (size_t j = 0; j < a.size(); ++j)
			q[j] = a[j] + fraction * (b[j] - a[j]);
		if (!scene_geometry::IsStateValid(this->scene, &state, this->group, q.data()))
			return false;
	}
	return true;
}

std::vector<std::pair<size_t, double>> Optimizer::getBlockedEdgeDetourCharges(const ViewpointIkAtPosition& placement, const std::vector<int>& order)
{
	if (this->cost_weights.blocked_edge_charge <= 0.0 || order.empty())
		return {};
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	std::vector<double> charges(order.size(), -1.0);  // -1: straight move free
	const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(order.size()));
	std::vector<std::thread> threads;
	for (unsigned int t = 0; t < num_workers; ++t)
		threads.emplace_back([&, t]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t k = t; k < order.size(); k += num_workers)
			{
				const Joints& from_pose = k == 0 ? this->home_pose : placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k - 1])].q;
				const Joints& to_pose = placement.chosen_ik_per_viewpoint[static_cast<size_t>(order[k])].q;
				if (this->isStraightJointMoveCollisionFree(thread_state, from_pose, to_pose))
					continue;
				const double straight = joint_travel::EdgeCost(from_pose, to_pose, this->tour_settings.edge_weights, this->counted_joints);
				const bool via_home = k > 0 && this->isStraightJointMoveCollisionFree(thread_state, from_pose, this->home_pose) && this->isStraightJointMoveCollisionFree(thread_state, this->home_pose, to_pose);
				const double detour = joint_travel::EdgeCost(from_pose, this->home_pose, this->tour_settings.edge_weights, this->counted_joints) + joint_travel::EdgeCost(this->home_pose, to_pose, this->tour_settings.edge_weights, this->counted_joints);
				charges[k] = via_home ? std::max(0.0, detour - straight) : this->cost_weights.blocked_edge_charge * straight;
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	std::vector<std::pair<size_t, double>> blocked;
	for (size_t k = 0; k < order.size(); ++k)
		if (charges[k] >= 0.0)
			blocked.push_back({k, charges[k]});
	return blocked;
}

void Optimizer::logObjectPositionEvaluation(const std::string& label, const ObjectPositionEvaluation& descent)
{
	RCLCPP_INFO(this->node->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%zu  gap sum %.4f m  reconfigurations %d (posture gap sum %.3f m)  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f", label.c_str(), descent.placement.position.x(), descent.placement.position.y(), descent.placement.position.z(), descent.placement.num_reached, this->viewpoints.size(), descent.placement.gap_sum, descent.num_reconfigurations, descent.posture_gap_sum, descent.num_blocked_edges, descent.joint_travel_cost, descent.smallest_margin, descent.weighted_total_cost);
}

void Optimizer::logMissedViewpoints(const ViewpointIkAtPosition& placement)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	const double limit_band = 0.02;  // rad: a joint this close to its (hard) limit counts as at it
	const double contact_band = this->ik_settings.closest_options.margin + 0.002;  // m
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		if (placement.reached[v])
			continue;
		std::string blockers;
		for (size_t j = 0; j < joints.size() && j < placement.chosen_ik_per_viewpoint[v].q.size(); ++j)
		{
			const moveit::core::VariableBounds& bounds = joints[j]->getVariableBounds()[0];
			const double value = placement.chosen_ik_per_viewpoint[v].q[j];
			if (bounds.position_bounded_ && (value - bounds.min_position_ < limit_band || bounds.max_position_ - value < limit_band))
				blockers += " " + joints[j]->getName() + " at limit;";
		}
		this->state->setJointGroupPositions(this->group, placement.chosen_ik_per_viewpoint[v].q);
		this->state->update();
		const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, contact_band), contact_band);
		for (size_t k = 0; k < pairs.names.size(); ++k)
			blockers += " " + pairs.names[k] + " " + std::to_string(static_cast<int>(std::round(pairs.dists[k] * 1000.0))) + " mm;";
		const bool valid = scene_geometry::IsStateValid(this->scene, this->state.get(), this->group, placement.chosen_ik_per_viewpoint[v].q.data());
		RCLCPP_INFO(this->node->get_logger(), "  missed viewpoint %zu: gap %.2f mm (position %.2f mm, angle %.3f deg, clearance short %.2f mm, closest pose %s), blocked by:%s", v, 1000.0 * placement.chosen_ik_per_viewpoint[v].gap, 1000.0 * placement.chosen_ik_per_viewpoint[v].position_error.norm(), placement.chosen_ik_per_viewpoint[v].angle_error * 180.0 / M_PI, 1000.0 * placement.chosen_ik_per_viewpoint[v].clearance_shortfall, valid ? "valid" : "INVALID", blockers.empty() ? " nothing (reach)" : blockers.c_str());
	}
}

void Optimizer::publishProgressMarkers(const ViewpointIkAtPosition& placement, const std::vector<int>& order, const Eigen::Vector3d& object_direction)
{
	if (!this->progress_pub)
		return;
	const rclcpp::Time stamp = this->node->now();
	const Eigen::Isometry3d object_transform = scene_geometry::Translation(placement.position);
	visualization_msgs::msg::MarkerArray markers;
	// Markers update in place (same ns and id); the first publish clears what an earlier run left.
	if (this->progress_marker_keys.empty())
	{
		visualization_msgs::msg::Marker clear;
		clear.action = visualization_msgs::msg::Marker::DELETEALL;
		markers.markers.push_back(clear);
	}

	visualization_msgs::msg::Marker mesh = marker_utils::MakeMarker("object_placement_current", visualization_msgs::msg::Marker::MESH_RESOURCE, stamp);
	mesh.mesh_resource = "file://" + this->mesh_path;
	mesh.mesh_use_embedded_materials = false;
	mesh.pose = scene_geometry::ToPoseMsg(scene_geometry::MakeIsometry(placement.position, this->object_rotation));
	mesh.scale.x = mesh.scale.y = mesh.scale.z = this->mesh_scale;
	mesh.color = marker_utils::Rgba(1.0f, 0.85f, 0.0f, 0.6f);
	markers.markers.push_back(mesh);

	visualization_msgs::msg::Marker viewpoint_markers = marker_utils::MakeMarker("object_placement_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	viewpoint_markers.scale.x = viewpoint_markers.scale.y = viewpoint_markers.scale.z = 0.01;
	viewpoint_markers.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		viewpoint_markers.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[v]).translation()));
		viewpoint_markers.colors.push_back(placement.reached[v] ? marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f) : marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f));
	}
	markers.markers.push_back(viewpoint_markers);

	// Trail as one cylinder per move: line markers vanish from some viewing angles.
	visualization_msgs::msg::Marker crumbs = marker_utils::MakeMarker("object_placement_history", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.012;
	crumbs.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t k = 0; k < this->object_trail.size(); ++k)
	{
		crumbs.points.push_back(scene_geometry::ToPoint(this->object_trail[k]));
		const double length = k == 0 ? 0.0 : (this->object_trail[k] - this->object_trail[k - 1]).norm();
		if (length < 1e-6)
			continue;
		visualization_msgs::msg::Marker segment = marker_utils::MakeMarker("object_placement_trail", visualization_msgs::msg::Marker::CYLINDER, stamp, static_cast<int>(k));
		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		pose.translation() = 0.5 * (this->object_trail[k] + this->object_trail[k - 1]);
		pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), this->object_trail[k] - this->object_trail[k - 1]).toRotationMatrix();
		segment.pose = scene_geometry::ToPoseMsg(pose);
		segment.scale.x = segment.scale.y = 0.005;
		segment.scale.z = length;
		segment.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
		markers.markers.push_back(segment);
	}
	markers.markers.push_back(crumbs);

	if (object_direction.norm() > 1e-9)
	{
		visualization_msgs::msg::Marker arrow = marker_utils::MakeMarker("object_placement_descent_dir", visualization_msgs::msg::Marker::ARROW, stamp);
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.color = marker_utils::Rgba(0.1f, 0.5f, 1.0f, 0.95f);
		arrow.points = {scene_geometry::ToPoint(placement.position), scene_geometry::ToPoint(placement.position + 0.15 * object_direction.normalized())};
		markers.markers.push_back(arrow);
	}

	// The tour's tool path from home, through each pose (missed ones at their closest pose).
	visualization_msgs::msg::Marker tool_path = marker_utils::MakeMarker("object_placement_tool_path", visualization_msgs::msg::Marker::LINE_STRIP, stamp);
	tool_path.scale.x = 0.003;
	tool_path.color = marker_utils::Rgba(0.2f, 0.6f, 1.0f, 0.9f);
	moveit::core::RobotState ghost(*this->state);
	std::vector<Joints> path_poses = this->getPosesInTourOrder(placement, order);
	path_poses.insert(path_poses.begin(), this->home_pose);
	for (const Joints& pose : path_poses)
	{
		ghost.setJointGroupPositions(this->group, pose);
		ghost.update();
		tool_path.points.push_back(scene_geometry::ToPoint(ghost.getGlobalLinkTransform(this->tool).translation()));
	}
	if (tool_path.points.size() > 1)
		markers.markers.push_back(tool_path);

	visualization_msgs::msg::Marker gaps = marker_utils::MakeMarker("object_placement_miss_gap", visualization_msgs::msg::Marker::LINE_LIST, stamp);
	gaps.scale.x = 0.003;
	gaps.color = marker_utils::Rgba(1.0f, 0.2f, 1.0f, 1.0f);
	visualization_msgs::msg::MarkerArray arms;
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		if (placement.reached[v])
			continue;
		ghost.setJointGroupPositions(this->group, placement.chosen_ik_per_viewpoint[v].q);
		ghost.update();
		ghost.getRobotMarkers(arms, this->model->getLinkModelNamesWithCollisionGeometry(), marker_utils::Rgba(0.9f, 0.2f, 0.2f, 0.35f), "object_placement_miss_ik", rclcpp::Duration(0, 0));
		gaps.points.push_back(scene_geometry::ToPoint(ghost.getGlobalLinkTransform(this->tool).translation()));
		gaps.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[v]).translation()));
	}
	if (!gaps.points.empty())
		markers.markers.push_back(gaps);
	for (visualization_msgs::msg::Marker& arm : arms.markers)
	{
		arm.header.frame_id = "world";
		arm.header.stamp = stamp;
		markers.markers.push_back(arm);
	}

	// Delete markers the last publish had and this one doesn't (e.g. a miss now reached).
	std::set<std::pair<std::string, int>> keys;
	for (const visualization_msgs::msg::Marker& marker : markers.markers)
		if (marker.action == visualization_msgs::msg::Marker::ADD)
			keys.insert({marker.ns, marker.id});
	for (const std::pair<std::string, int>& key : this->progress_marker_keys)
		if (keys.count(key) == 0)
		{
			visualization_msgs::msg::Marker stale = marker_utils::MakeMarker(key.first, visualization_msgs::msg::Marker::SPHERE, stamp, key.second);
			stale.action = visualization_msgs::msg::Marker::DELETE;
			markers.markers.push_back(stale);
		}
	this->progress_marker_keys = std::move(keys);
	this->progress_pub->publish(markers);
}

}  // namespace object_gtsp_co_optimize
