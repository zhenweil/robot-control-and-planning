#include <set>
#include <cmath>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>
#include <algorithm>
#include <random_numbers/random_numbers.h>
#include <moveit/robot_state/robot_state.h>

#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/room_margins.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/manipulability.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/object_gtsp_co_optimizer.hpp"

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

ObjectPositionEvaluation Optimizer::runDescent(const Eigen::Vector3d& requested_start)
{
	this->start_time = std::chrono::steady_clock::now();
	const Eigen::Vector3d start = this->clampToSearchBox(requested_start);
	double miss_gap_penalty = this->cost_weights.initial_penalty;
	ObjectPositionEvaluation current_evaluation;

	// Find initial IK for all viewpoints, order viewpoints, and improve IK
	current_evaluation.viewpoint_ik = this->solveViewpointIkAtPosition(start, std::vector<Joints>(this->viewpoints.size(), this->home_pose));
	current_evaluation.order = this->reorderTourWithGtsp(current_evaluation.viewpoint_ik, {});
	this->improveOrderAndPosturesWithObjectFixed(current_evaluation, miss_gap_penalty);

	const int num_viewpoint = static_cast<int>(this->viewpoints.size());
	double reach_step = this->step_sizes.initial_step;
	double tour_step = this->step_sizes.initial_step;
	this->object_trail.assign(1, start);
	this->publishProgressMarkers(current_evaluation.viewpoint_ik, current_evaluation.order, Eigen::Vector3d::Zero());
	this->logObjectPositionEvaluation("start", current_evaluation);
	bool object_frozen = false;	 // freeze_object_when_reached: set once every viewpoint is reached
	
	for (int iteration = 1; iteration <= this->descent_limits.max_steps && rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - this->start_time).count() < this->descent_limits.time_budget; ++iteration)
	{
		bool any_step_accepted = false;
		if (current_evaluation.viewpoint_ik.num_reached < num_viewpoint && !object_frozen)
			any_step_accepted = this->moveObjectToReachMore(current_evaluation, reach_step, miss_gap_penalty, iteration) || any_step_accepted; // Room-then-reach if viewpoints are missing
			
		// Checked after the reach step, so the iteration that reaches the last viewpoint takes no tour move.
		if (this->method_switches.freeze_object_when_reached && !object_frozen && current_evaluation.viewpoint_ik.num_reached == num_viewpoint)
		{
			object_frozen = true;
			RCLCPP_INFO(this->node->get_logger(), "iteration %d: all reached; object frozen at abs (%.4f, %.4f, %.4f)", iteration, current_evaluation.viewpoint_ik.position.x(), current_evaluation.viewpoint_ik.position.y(), current_evaluation.viewpoint_ik.position.z());
		}
		any_step_accepted = this->improveArmPosesWithObjectFixed(current_evaluation, miss_gap_penalty, iteration) || any_step_accepted;
		if (this->method_switches.tour_object_move && !object_frozen)
			any_step_accepted = this->moveObjectToLowerCost(current_evaluation, tour_step, miss_gap_penalty, iteration) || any_step_accepted;
		if (any_step_accepted)
			continue;
		if (current_evaluation.viewpoint_ik.num_reached == num_viewpoint || miss_gap_penalty >= this->cost_weights.max_penalty)
			break;
		miss_gap_penalty = std::min(this->cost_weights.max_penalty, miss_gap_penalty * this->cost_weights.penalty_growth);
		this->updateWeightedTotalCost(current_evaluation, miss_gap_penalty);
		reach_step = this->step_sizes.initial_step;
		tour_step = this->step_sizes.initial_step;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d: nothing changed with %d missed; penalty -> %.0f per m", iteration, num_viewpoint - current_evaluation.viewpoint_ik.num_reached, miss_gap_penalty);
		this->logMissedViewpoints(current_evaluation.viewpoint_ik);
	}
	this->logObjectPositionEvaluation("end", current_evaluation);
	this->logMissedViewpoints(current_evaluation.viewpoint_ik);
	return current_evaluation;
}

std::vector<Joints> Optimizer::getPosesInTourOrder(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const
{
	std::vector<Joints> poses;
	for (int viewpoint : order)
		poses.push_back(viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(viewpoint)].q);
	return poses;
}

double Optimizer::getSmallestJointMargin(const std::vector<Joints>& poses) const
{
	double smallest_margin = std::numeric_limits<double>::infinity();
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	for (const Joints& pose : poses)
		for (int joint_index : this->counted_joints)
		{
			const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(joint_index)]->getVariableBounds()[0];
			if (!bounds.position_bounded_)
				continue;
			const double value = pose[static_cast<size_t>(joint_index)];
			smallest_margin = std::min(smallest_margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
		}
	return smallest_margin;
}

const std::vector<int>& Optimizer::getCountedJoints() const
{
	return this->counted_joints;
}

const joint_travel::EdgeWeights& Optimizer::getEdgeWeights() const
{
	return this->tour_settings.edge_weights;
}

Eigen::Vector3d Optimizer::clampToSearchBox(const Eigen::Vector3d& position) const
{
	return position.cwiseMax(this->descent_limits.search_box_lower_bound).cwiseMin(this->descent_limits.search_box_upper_bound);
}

ObjectPositionEvaluation Optimizer::solveAndImproveAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds, const std::vector<int>& order, double miss_gap_penalty)
{
	ObjectPositionEvaluation evaluation;
	evaluation.viewpoint_ik = this->solveViewpointIkAtPosition(this->clampToSearchBox(position), seeds, fallback_seeds);
	evaluation.order = order;
	this->improveOrderAndPosturesWithObjectFixed(evaluation, miss_gap_penalty);
	return evaluation;
}

void Optimizer::improveOrderAndPosturesWithObjectFixed(ObjectPositionEvaluation& evaluation, double miss_gap_penalty)
{
	// Timed for the log: posture matches, GTSP and the cost (mostly the blocked-edge checks).
	auto get_seconds_since = [](std::chrono::steady_clock::time_point since) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count(); };
	ObjectPositionEvaluation reordered_evaluation = evaluation;
	auto since = std::chrono::steady_clock::now();
	this->swapInReconfigurationPostureMatches(evaluation);
	double posture_match_seconds = get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	this->updateWeightedTotalCost(evaluation, miss_gap_penalty);
	double cost_seconds = get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	reordered_evaluation.order = this->reorderTourWithGtsp(reordered_evaluation.viewpoint_ik, reordered_evaluation.order);
	const double gtsp_seconds = get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	this->swapInReconfigurationPostureMatches(reordered_evaluation);
	posture_match_seconds += get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	this->updateWeightedTotalCost(reordered_evaluation, miss_gap_penalty);
	cost_seconds += get_seconds_since(since);
	const PostureSwapCounts& swaps = this->posture_swap_counts;
	RCLCPP_INFO(this->node->get_logger(), "  tour work: posture matches %.2f s, GTSP %.2f s, cost with blocked-edge checks %.2f s; posture swaps so far: later from earlier %d removed / %d closer, earlier from later %d removed / %d closer", posture_match_seconds, gtsp_seconds, cost_seconds, swaps.later_from_earlier_removed, swaps.later_from_earlier_closer, swaps.earlier_from_later_removed, swaps.earlier_from_later_closer);
	// Reach first: more viewpoints reached wins; at equal reach the lower weighted total cost.
	const bool reordered_reaches_more = reordered_evaluation.viewpoint_ik.num_reached > evaluation.viewpoint_ik.num_reached;
	const bool reordered_cheaper = reordered_evaluation.viewpoint_ik.num_reached == evaluation.viewpoint_ik.num_reached && reordered_evaluation.weighted_total_cost < evaluation.weighted_total_cost;
	if (reordered_reaches_more || reordered_cheaper)
		evaluation = reordered_evaluation;
}

void Optimizer::updateWeightedTotalCost(ObjectPositionEvaluation& evaluation, double miss_gap_penalty)
{
	this->unwrapFullTurnJointsAlongTour(evaluation.viewpoint_ik, evaluation.order);
	// Tour length, with each edge whose straight move collides charged its estimated detour (see getBlockedEdgeDetourCharges).
	double detour_charge = 0.0;
	const std::vector<std::pair<size_t, double>> blocked_edges = this->getBlockedEdgeDetourCharges(evaluation.viewpoint_ik, evaluation.order);
	for (const std::pair<size_t, double>& edge : blocked_edges)
		detour_charge += edge.second;
	evaluation.num_blocked_edges = static_cast<int>(blocked_edges.size());
	std::vector<Joints> poses_from_home = this->getPosesInTourOrder(evaluation.viewpoint_ik, evaluation.order);
	poses_from_home.insert(poses_from_home.begin(), this->home_pose);
	evaluation.joint_travel_cost = joint_travel::TravelCost(poses_from_home, this->tour_settings.edge_weights, this->counted_joints) + detour_charge;
	evaluation.num_reconfigurations = this->countReconfigurations(evaluation.viewpoint_ik, evaluation.order);
	evaluation.posture_gap_sum = 0.0;
	for (const ReconfigurationPostureMatch& match : evaluation.matches)
		evaluation.posture_gap_sum += match.gap;
	evaluation.room_shortfall = 0.0;
	std::vector<Joints> reached_poses;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		// Reached poses only: a missed viewpoint's closest pose presses on limits by design.
		if (!evaluation.viewpoint_ik.reached[viewpoint_index])
			continue;
		evaluation.room_shortfall += this->getJointRoomShortfall(evaluation.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, nullptr);
		reached_poses.push_back(evaluation.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
	}
	evaluation.smallest_margin = this->getSmallestJointMargin(reached_poses);
	const int num_missed = static_cast<int>(this->viewpoints.size()) - evaluation.viewpoint_ik.num_reached;
	evaluation.weighted_total_cost = evaluation.joint_travel_cost + this->cost_weights.miss_cost * num_missed + miss_gap_penalty * evaluation.viewpoint_ik.gap_sum + this->cost_weights.reconfiguration_cost * evaluation.num_reconfigurations + this->cost_weights.reconfiguration_weight * evaluation.posture_gap_sum + this->cost_weights.joint_room_weight * evaluation.room_shortfall;
}

bool Optimizer::moveObjectToReachMore(ObjectPositionEvaluation& current_evaluation, double& object_step, double miss_gap_penalty, int iteration)
{
	const Eigen::Vector3d aim = this->getReachDirection(current_evaluation.viewpoint_ik);
	if (aim.norm() < 0.5)
		return false;
	Eigen::Vector3d direction = aim;
	bool is_room_then_reach = false;  // the direction was turned to keep room for viewpoints the last try lost
	bool tried_miss_room = false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && object_step >= this->step_sizes.min_step; ++attempt)
	{
		const ObjectPositionEvaluation candidate_evaluation = this->solveAndImproveAtPosition(current_evaluation.viewpoint_ik.position + object_step * direction, this->getChosenPoses(current_evaluation.viewpoint_ik), {}, current_evaluation.order, miss_gap_penalty);
		std::vector<size_t> lost_viewpoints;
		for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
			if (current_evaluation.viewpoint_ik.reached[viewpoint_index] && !candidate_evaluation.viewpoint_ik.reached[viewpoint_index])
				lost_viewpoints.push_back(viewpoint_index);
		const bool kept = candidate_evaluation.viewpoint_ik.num_reached > current_evaluation.viewpoint_ik.num_reached || (candidate_evaluation.viewpoint_ik.num_reached == current_evaluation.viewpoint_ik.num_reached && candidate_evaluation.viewpoint_ik.gap_sum < current_evaluation.viewpoint_ik.gap_sum);
		RCLCPP_INFO(this->node->get_logger(), "iteration %d %s: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d (lost %zu)  gap sum %.4f m vs %.4f%s", iteration, is_room_then_reach ? "room-then-reach" : "reach", object_step, candidate_evaluation.viewpoint_ik.position.x(), candidate_evaluation.viewpoint_ik.position.y(), candidate_evaluation.viewpoint_ik.position.z(), candidate_evaluation.viewpoint_ik.num_reached, lost_viewpoints.size(), candidate_evaluation.viewpoint_ik.gap_sum, current_evaluation.viewpoint_ik.gap_sum, kept ? "  <-- kept" : "");
		if (kept)
		{
			current_evaluation = candidate_evaluation;
			this->object_trail.push_back(current_evaluation.viewpoint_ik.position);
			this->publishProgressMarkers(current_evaluation.viewpoint_ik, current_evaluation.order, direction);
			object_step = std::min(this->step_sizes.max_step, object_step * 1.5);
			// Misses nearly on target: say why they still miss.
			bool has_tiny_miss = false;
			for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
				has_tiny_miss = has_tiny_miss || (!current_evaluation.viewpoint_ik.reached[viewpoint_index] && current_evaluation.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap < 1e-3);
			if (has_tiny_miss)
				this->logMissedViewpoints(current_evaluation.viewpoint_ik);
			return true;
		}
		if (!lost_viewpoints.empty() && this->method_switches.use_room)
		{
			const Eigen::Vector3d room_direction = this->getRoomThenReachDirection(current_evaluation.viewpoint_ik, lost_viewpoints, aim, object_step);
			if (room_direction.norm() > 0.5 && (room_direction - direction).norm() > 1e-3)
			{
				direction = room_direction;
				is_room_then_reach = true;
				continue;
			}
		}
		// Room-then-reach for the misses: once per step, a move that frees what blocks them (joint limits, stretch,
		// collisions), kept if it raises their tightest margin and loses nothing; the next iteration aims again.
		if (this->method_switches.use_room && !tried_miss_room)
		{
			tried_miss_room = true;
			if (this->moveObjectToUnblockMisses(current_evaluation, object_step, miss_gap_penalty, iteration))
				return true;
		}
		object_step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	object_step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::improveArmPosesWithObjectFixed(ObjectPositionEvaluation& current_evaluation, double miss_gap_penalty, int iteration)
{
	if (!this->method_switches.use_slide)
	{
		ObjectPositionEvaluation reordered_evaluation = current_evaluation;
		this->improveOrderAndPosturesWithObjectFixed(reordered_evaluation, miss_gap_penalty);
		const bool kept = reordered_evaluation.weighted_total_cost < current_evaluation.weighted_total_cost - 1e-9;
		if (kept)
			current_evaluation = reordered_evaluation;
		return kept;
	}
	const DescentDirection direction = this->getTourDirection(current_evaluation);
	double slide_step = this->step_sizes.initial_slide_step;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && direction.has_slide; ++attempt)
	{
		const ObjectPositionEvaluation candidate_evaluation = this->solveAndImproveAtPosition(current_evaluation.viewpoint_ik.position, this->getPosesAfterJointSlides(current_evaluation.viewpoint_ik, direction, slide_step), this->getChosenPoses(current_evaluation.viewpoint_ik), current_evaluation.order, miss_gap_penalty);
		const bool kept = candidate_evaluation.weighted_total_cost < current_evaluation.weighted_total_cost - 1e-9;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d poses: slide %.3f rad  reached %d  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, slide_step, candidate_evaluation.viewpoint_ik.num_reached, candidate_evaluation.num_reconfigurations, candidate_evaluation.num_blocked_edges, candidate_evaluation.joint_travel_cost, candidate_evaluation.smallest_margin, candidate_evaluation.weighted_total_cost, current_evaluation.weighted_total_cost, kept ? "  <-- kept" : "");
		if (kept)
		{
			current_evaluation = candidate_evaluation;
			return true;
		}
		slide_step *= 0.5;
	}
	return false;
}

bool Optimizer::moveObjectToLowerCost(ObjectPositionEvaluation& current_evaluation, double& object_step, double miss_gap_penalty, int iteration)
{
	const DescentDirection direction = this->getTourDirection(current_evaluation);
	if (direction.object_direction.norm() < 0.5)
		return false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && object_step >= this->step_sizes.min_step; ++attempt)
	{
		const ObjectPositionEvaluation candidate_evaluation = this->solveAndImproveAtPosition(current_evaluation.viewpoint_ik.position + object_step * direction.object_direction, this->getChosenPoses(current_evaluation.viewpoint_ik), {}, current_evaluation.order, miss_gap_penalty);
		const bool kept = candidate_evaluation.weighted_total_cost < current_evaluation.weighted_total_cost - 1e-9;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d tour: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, object_step, candidate_evaluation.viewpoint_ik.position.x(), candidate_evaluation.viewpoint_ik.position.y(), candidate_evaluation.viewpoint_ik.position.z(), candidate_evaluation.viewpoint_ik.num_reached, candidate_evaluation.viewpoint_ik.gap_sum, candidate_evaluation.num_reconfigurations, candidate_evaluation.num_blocked_edges, candidate_evaluation.joint_travel_cost, candidate_evaluation.smallest_margin, candidate_evaluation.weighted_total_cost, current_evaluation.weighted_total_cost, kept ? "  <-- kept" : "");
		if (kept)
		{
			current_evaluation = candidate_evaluation;
			this->object_trail.push_back(current_evaluation.viewpoint_ik.position);
			this->publishProgressMarkers(current_evaluation.viewpoint_ik, current_evaluation.order, direction.object_direction);
			object_step = std::min(this->step_sizes.max_step, object_step * 1.5);
			return true;
		}
		object_step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	object_step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::moveObjectToUnblockMisses(ObjectPositionEvaluation& current_evaluation, double object_step, double miss_gap_penalty, int iteration)
{
	const double near_floors = 6.0;
	std::vector<size_t> blocked_viewpoints;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		if (current_evaluation.viewpoint_ik.reached[viewpoint_index])
			continue;
		const double gap = current_evaluation.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
		const double blocked_share = gap < 1e-3 ? 1.0 : 1.0 - current_evaluation.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].position_error.norm() / gap;
		if (blocked_share > 0.3)
			blocked_viewpoints.push_back(viewpoint_index);
	}
	if (blocked_viewpoints.empty())
		return false;
	const std::vector<room_margins::Margin> margins_before = this->getBlockingMarginsOfViewpoints(current_evaluation.viewpoint_ik, blocked_viewpoints, near_floors);
	if (margins_before.empty())
		return false;
	const Eigen::Vector3d room_direction = room_margins::MostRoomDirection(margins_before, object_step);
	if (room_direction.norm() < 0.5)
		return false;
	const ObjectPositionEvaluation candidate_evaluation = this->solveAndImproveAtPosition(current_evaluation.viewpoint_ik.position + object_step * room_direction, this->getChosenPoses(current_evaluation.viewpoint_ik), {}, current_evaluation.order, miss_gap_penalty);
	bool lost_any = false;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		lost_any = lost_any || (current_evaluation.viewpoint_ik.reached[viewpoint_index] && !candidate_evaluation.viewpoint_ik.reached[viewpoint_index]);
	const std::vector<room_margins::Margin> margins_after = this->getBlockingMarginsOfViewpoints(candidate_evaluation.viewpoint_ik, blocked_viewpoints, near_floors);
	const double tightest_before = room_margins::Tightest(margins_before);
	const double tightest_after = margins_after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(margins_after);
	const bool kept = !lost_any && candidate_evaluation.viewpoint_ik.num_reached >= current_evaluation.viewpoint_ik.num_reached && tightest_after > tightest_before + 1e-3;
	RCLCPP_INFO(this->node->get_logger(), "iteration %d reach room: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  blocked misses %zu  tightest margin %.2f -> %.2f floors%s", iteration, object_step, candidate_evaluation.viewpoint_ik.position.x(), candidate_evaluation.viewpoint_ik.position.y(), candidate_evaluation.viewpoint_ik.position.z(), candidate_evaluation.viewpoint_ik.num_reached, blocked_viewpoints.size(), tightest_before, tightest_after, kept ? "  <-- kept" : "");
	if (!kept)
		return false;
	current_evaluation = candidate_evaluation;
	this->object_trail.push_back(current_evaluation.viewpoint_ik.position);
	this->publishProgressMarkers(current_evaluation.viewpoint_ik, current_evaluation.order, room_direction);
	return true;
}

Eigen::Vector3d Optimizer::getReachDirection(const ViewpointIkAtPosition& viewpoint_ik)
{
	Eigen::Vector3d reach_move = Eigen::Vector3d::Zero();
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		if (viewpoint_ik.reached[viewpoint_index])
			continue;
		const double gap = viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
		// The closest pose is a minimum over the joints, so only the gap's position part moves with the object.
		if (gap > 1e-9)
			reach_move -= viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].position_error / gap;
	}
	return reach_move.norm() > 1e-12 ? Eigen::Vector3d(reach_move.normalized()) : Eigen::Vector3d::Zero();
}

DescentDirection Optimizer::getTourDirection(const ObjectPositionEvaluation& evaluation)
{
	const ViewpointIkAtPosition& viewpoint_ik = evaluation.viewpoint_ik;
	const std::vector<Eigen::VectorXd> pose_slopes = this->getTourCostSlopePerPose(viewpoint_ik, evaluation.order);
	Eigen::MatrixXd object_twist = Eigen::MatrixXd::Zero(6, 3);
	object_twist.topRows<3>() = Eigen::Matrix3d::Identity();
	Eigen::Vector3d reconfiguration_move = Eigen::Vector3d::Zero();
	Eigen::Vector3d length_move = Eigen::Vector3d::Zero();
	Eigen::Vector3d room_move = Eigen::Vector3d::Zero();
	DescentDirection direction;
	direction.joint_slides.resize(this->viewpoints.size());
	double largest_slide = 0.0;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		const Eigen::MatrixXd jacobian = this->getToolJacobian(*this->state, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		const Eigen::MatrixXd follow_rate = scene_geometry::DampedPinv(jacobian, this->ik_settings.jacobian_damping) * object_twist;
		Eigen::VectorXd room_slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q.size()));
		if (viewpoint_ik.reached[viewpoint_index])
			this->getJointRoomShortfall(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, &room_slope);
		length_move -= follow_rate.transpose() * pose_slopes[viewpoint_index];
		room_move -= this->cost_weights.joint_room_weight * follow_rate.transpose() * room_slope;
		if (!this->method_switches.use_slide || !viewpoint_ik.reached[viewpoint_index])
			continue;
		direction.joint_slides[viewpoint_index] = -this->getSelfMotionProjector(jacobian) * (pose_slopes[viewpoint_index] + this->cost_weights.joint_room_weight * room_slope);
		largest_slide = std::max(largest_slide, direction.joint_slides[viewpoint_index].norm());
	}
	for (Eigen::VectorXd& slide : direction.joint_slides)
		if (slide.size() > 0)
			slide = largest_slide > 1e-9 ? Eigen::VectorXd(slide / largest_slide) : Eigen::VectorXd();
	for (const ReconfigurationPostureMatch& match : evaluation.matches)
	{
		if (match.gap > 1e-9)
			reconfiguration_move -= this->cost_weights.reconfiguration_weight * match.position_error / match.gap;
		if (!this->method_switches.use_slide || !viewpoint_ik.reached[match.viewpoint_index])
			continue;
		// Toward the neighbour's joints along the self-motion, full slide step.
		const Eigen::VectorXd toward = Eigen::Map<const Eigen::VectorXd>(match.neighbor_pose.data(), static_cast<Eigen::Index>(match.neighbor_pose.size())) - Eigen::Map<const Eigen::VectorXd>(viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].q.data(), static_cast<Eigen::Index>(match.neighbor_pose.size()));
		const Eigen::VectorXd slide = this->getSelfMotionProjector(this->getToolJacobian(*this->state, viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].q)) * toward;
		if (slide.norm() > 1e-9)
			direction.joint_slides[match.viewpoint_index] = slide.normalized();
	}
	const Eigen::Vector3d object_move = this->combineMovesWithoutConflict({reconfiguration_move, length_move, room_move});
	if (object_move.norm() > 1e-12)
		direction.object_direction = object_move.normalized();
	direction.has_slide = std::any_of(direction.joint_slides.begin(), direction.joint_slides.end(), [](const Eigen::VectorXd& slide) { return slide.size() > 0; });
	return direction;
}

Eigen::Vector3d Optimizer::getRoomThenReachDirection(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<size_t>& lost_viewpoints, const Eigen::Vector3d& aim, double object_step)
{
	const double lost_margin_floors = 6.0;  // margins up to this many floors count as at risk
	const std::vector<room_margins::Margin> margins = this->getBlockingMarginsOfViewpoints(viewpoint_ik, lost_viewpoints, lost_margin_floors);
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
		return room_margins::MostRoomDirection(margins, object_step);
	return direction.normalized();
}

Eigen::Vector3d Optimizer::combineMovesWithoutConflict(const std::vector<Eigen::Vector3d>& moves) const
{
	Eigen::Vector3d combined_move = Eigen::Vector3d::Zero();
	for (size_t move_index = 0; move_index < moves.size(); ++move_index)
	{
		Eigen::Vector3d move = moves[move_index];
		for (size_t other_index = 0; other_index < moves.size(); ++other_index)
		{
			const double other_size = moves[other_index].squaredNorm();
			if (other_index == move_index || other_size < 1e-24)
				continue;
			const double against = move.dot(moves[other_index]);
			if (against < 0.0)
				move -= against / other_size * moves[other_index];
		}
		combined_move += move;
	}
	return combined_move;
}

std::vector<Eigen::VectorXd> Optimizer::getTourCostSlopePerPose(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const
{
	std::vector<Eigen::VectorXd> pose_slopes(this->viewpoints.size(), Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size())));
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
	{
		const size_t to_viewpoint = static_cast<size_t>(order[tour_position]);
		const Joints& from_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
		// d(edge cost)/d(to) by central differences; the cost depends on to - from only, so d/d(from) is its negative.
		const double delta = 1e-5;
		Eigen::VectorXd slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size()));
		Joints shifted = viewpoint_ik.chosen_ik_per_viewpoint[to_viewpoint].q;
		for (size_t joint_index = 0; joint_index < shifted.size(); ++joint_index)
		{
			const double value = shifted[joint_index];
			shifted[joint_index] = value + delta;
			const double cost_up = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value - delta;
			const double cost_down = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value;
			slope[static_cast<Eigen::Index>(joint_index)] = (cost_up - cost_down) / (2.0 * delta);
		}
		pose_slopes[to_viewpoint] += slope;
		if (tour_position > 0)
			pose_slopes[static_cast<size_t>(order[tour_position - 1])] -= slope;
	}
	return pose_slopes;
}

std::vector<Joints> Optimizer::getPosesAfterJointSlides(const ViewpointIkAtPosition& viewpoint_ik, const DescentDirection& direction, double slide_step) const
{
	std::vector<Joints> slid_poses = this->getChosenPoses(viewpoint_ik);
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	for (size_t viewpoint_index = 0; viewpoint_index < slid_poses.size(); ++viewpoint_index)
	{
		if (direction.joint_slides[viewpoint_index].size() == 0)
			continue;
		for (size_t joint_index = 0; joint_index < slid_poses[viewpoint_index].size(); ++joint_index)
		{
			const moveit::core::VariableBounds& bounds = joints[joint_index]->getVariableBounds()[0];
			const double value = slid_poses[viewpoint_index][joint_index];
			slid_poses[viewpoint_index][joint_index] += slide_step * direction.joint_slides[viewpoint_index][static_cast<Eigen::Index>(joint_index)];
			if (bounds.position_bounded_)
				slid_poses[viewpoint_index][joint_index] = std::clamp(slid_poses[viewpoint_index][joint_index], std::min(value, bounds.min_position_ + this->step_sizes.slide_joint_margin), std::max(value, bounds.max_position_ - this->step_sizes.slide_joint_margin));
		}
	}
	return slid_poses;
}

Eigen::MatrixXd Optimizer::getToolJacobian(moveit::core::RobotState& state, const Joints& joint_values) const
{
	Eigen::MatrixXd jacobian = manipulability::ToolJacobian(state, this->group, this->tool, joint_values);
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

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfViewpoints(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<size_t>& viewpoint_indices, double near_floors)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	std::vector<room_margins::Margin> margins;
	for (size_t viewpoint_index : viewpoint_indices)
	{
		const std::vector<room_margins::Margin> viewpoint_margins = this->getBlockingMarginsOfPose(viewpoint_index, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, near_floors);
		margins.insert(margins.end(), viewpoint_margins.begin(), viewpoint_margins.end());
	}
	return margins;
}

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfPose(size_t viewpoint_index, const Joints& joint_values, double near_floors)
{
	const double joint_floor = 0.05;  // rad
	const double stretch_floor = 0.02;
	const double collision_floor = 0.01;  // m
	const double window = this->ik_settings.closest_options.margin + near_floors * collision_floor;  // m, collision pairs looked at
	const int num_joints = static_cast<int>(joint_values.size());
	const Eigen::MatrixXd joint_rate_per_object_move = scene_geometry::DampedPinv(manipulability::ToolJacobian(*this->state, this->group, this->tool, joint_values), this->ik_settings.jacobian_damping).leftCols<3>();
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	std::vector<room_margins::Margin> margins;
	for (int joint_index = 0; joint_index < num_joints; ++joint_index)
	{
		const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(joint_index)]->getVariableBounds()[0];
		if (!bounds.position_bounded_)
			continue;
		const Eigen::VectorXd joint_direction = Eigen::VectorXd::Unit(num_joints, joint_index);
		const std::string joint_name = joints[static_cast<size_t>(joint_index)]->getName();
		const double room_to_lower_limit = joint_values[static_cast<size_t>(joint_index)] - bounds.min_position_;
		const double room_to_upper_limit = bounds.max_position_ - joint_values[static_cast<size_t>(joint_index)];
		if (room_to_lower_limit < near_floors * joint_floor)
			margins.push_back({static_cast<int>(viewpoint_index), joint_name + " low limit", room_to_lower_limit, joint_floor, joint_rate_per_object_move.transpose() * joint_direction, joint_direction});
		if (room_to_upper_limit < near_floors * joint_floor)
			margins.push_back({static_cast<int>(viewpoint_index), joint_name + " high limit", room_to_upper_limit, joint_floor, joint_rate_per_object_move.transpose() * -joint_direction, -joint_direction});
	}
	const double stretch = this->getSmallestJacobianSingularValue(joint_values);
	if (stretch < near_floors * stretch_floor)
	{
		const double delta = 1e-5;  // rad
		Eigen::VectorXd stretch_slope(num_joints);
		for (int joint_index = 0; joint_index < num_joints; ++joint_index)
		{
			Joints shifted = joint_values;
			shifted[static_cast<size_t>(joint_index)] += delta;
			stretch_slope(joint_index) = (this->getSmallestJacobianSingularValue(shifted) - stretch) / delta;
		}
		margins.push_back({static_cast<int>(viewpoint_index), "stretch", stretch, stretch_floor, joint_rate_per_object_move.transpose() * stretch_slope, stretch_slope});
	}
	this->state->setJointGroupPositions(this->group, joint_values);
	this->state->update();
	const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, window), window);
	for (size_t pair_index = 0; pair_index < close_pairs.dists.size(); ++pair_index)
	{
		const Eigen::VectorXd joint_slope = close_pairs.gradients[pair_index].transpose();
		margins.push_back({static_cast<int>(viewpoint_index), "collision " + close_pairs.names[pair_index], close_pairs.dists[pair_index], collision_floor, joint_rate_per_object_move.transpose() * joint_slope + close_pairs.object_slopes[pair_index], joint_slope});
	}
	return margins;
}

double Optimizer::getSmallestJacobianSingularValue(const Joints& joint_values)
{
	return Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(*this->state, this->group, this->tool, joint_values)).singularValues().minCoeff();
}

double Optimizer::getJointRoomShortfall(const Joints& joint_values, Eigen::VectorXd* slope) const
{
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	if (slope != nullptr)
		*slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_values.size()));
	double shortfall = 0.0;
	for (int joint_index : this->counted_joints)
	{
		const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(joint_index)]->getVariableBounds()[0];
		if (!bounds.position_bounded_)
			continue;
		const double value = joint_values[static_cast<size_t>(joint_index)];
		const double room_to_lower_limit = value - bounds.min_position_;
		const double room_to_upper_limit = bounds.max_position_ - value;
		if (room_to_lower_limit < this->cost_weights.joint_room_band)
		{
			shortfall += (this->cost_weights.joint_room_band - room_to_lower_limit) * (this->cost_weights.joint_room_band - room_to_lower_limit);
			if (slope != nullptr)
				(*slope)[joint_index] -= 2.0 * (this->cost_weights.joint_room_band - room_to_lower_limit);
		}
		if (room_to_upper_limit < this->cost_weights.joint_room_band)
		{
			shortfall += (this->cost_weights.joint_room_band - room_to_upper_limit) * (this->cost_weights.joint_room_band - room_to_upper_limit);
			if (slope != nullptr)
				(*slope)[joint_index] += 2.0 * (this->cost_weights.joint_room_band - room_to_upper_limit);
		}
	}
	return shortfall;
}

ViewpointIkAtPosition Optimizer::solveViewpointIkAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(position, this->object_rotation));
	const auto solve_start = std::chrono::steady_clock::now();
	closest_ik::ResetWeightedSolveTiming();
	ViewpointIkAtPosition viewpoint_ik;
	viewpoint_ik.position = position;
	viewpoint_ik.chosen_ik_per_viewpoint.assign(this->viewpoints.size(), ClosestIkCandidate());
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q = seeds[viewpoint_index];
	viewpoint_ik.reached.assign(this->viewpoints.size(), false);
	viewpoint_ik.closest_ik_candidates.assign(this->viewpoints.size(), {});
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < this->num_threads; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t viewpoint_index = thread_index; viewpoint_index < this->viewpoints.size(); viewpoint_index += this->num_threads)
			{
				// Random starts seeded by viewpoint and spot (0.1 mm grid): the same spot always gives the same poses.
				random_numbers::RandomNumberGenerator viewpoint_rng(this->getRandomIkSeed(position, viewpoint_index));
				const Eigen::Isometry3d target = scene_geometry::Translation(position) * this->viewpoints[viewpoint_index];
				closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, seeds[viewpoint_index], this->ik_settings.exact_options);
				if (!result.exact && viewpoint_index < fallback_seeds.size())
					result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, fallback_seeds[viewpoint_index], this->ik_settings.exact_options);
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
					std::vector<Joints> closest_starts = {seeds[viewpoint_index]};
					for (size_t random_start_index = 0; random_start_index < random_starts.size() && static_cast<int>(closest_starts.size()) <= this->ik_settings.closest_ik_extra_starts; ++random_start_index)
						closest_starts.push_back(random_starts[random_start_index]);
					const std::vector<closest_ik::Result> closest_results = this->solveClosestIkFromStarts(thread_state, target, closest_starts);
					result = closest_ik::Result();
					for (const closest_ik::Result& candidate : closest_results)
						if (candidate.exact)
							result = candidate;
					if (!result.exact && !closest_results.empty())
					{
						viewpoint_ik.closest_ik_candidates[viewpoint_index] = this->keepCandidatesNearBestGap(thread_state, closest_results);
						result = closest_ik::Result();
						result.found = true;
						result.q = viewpoint_ik.closest_ik_candidates[viewpoint_index].front().q;
					}
				}
				if (!result.found)
				{
					// No collision-free pose near: keep the last one and a fixed large gap, no pull.
					viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap = this->ik_settings.closest_options.margin + 1.0;
					continue;
				}
				viewpoint_ik.reached[viewpoint_index] = result.exact;
				if (result.exact)
					viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q = result.q;
				else
					viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index] = viewpoint_ik.closest_ik_candidates[viewpoint_index].front();
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		viewpoint_ik.num_reached += viewpoint_ik.reached[viewpoint_index] ? 1 : 0;
		viewpoint_ik.gap_sum += viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
	}
	const closest_ik::WeightedSolveTiming& timing = closest_ik::GetWeightedSolveTiming();
	if (timing.solves > 0)
		RCLCPP_INFO(this->node->get_logger(), "  IK at abs (%.4f, %.4f, %.4f): %.2f s wall; weighted closest IK %ld solves, %.1f steps each, distance queries %.2f s, checks %.2f s (summed over %u threads)", position.x(), position.y(), position.z(), std::chrono::duration<double>(std::chrono::steady_clock::now() - solve_start).count(), timing.solves.load(), static_cast<double>(timing.steps.load()) / static_cast<double>(timing.solves.load()), 1e-6 * static_cast<double>(timing.distance_microseconds.load()), 1e-6 * static_cast<double>(timing.check_microseconds.load()), this->num_threads);
	return viewpoint_ik;
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

std::vector<ClosestIkCandidate> Optimizer::keepCandidatesNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest_results)
{
	std::vector<ClosestIkCandidate> all_candidates;
	std::vector<bool> is_roomy;
	double best_gap = std::numeric_limits<double>::max();
	for (const closest_ik::Result& result : closest_results)
	{
		ClosestIkCandidate candidate;
		candidate.q = result.q;
		candidate.position_error = result.pose_error.head<3>();
		candidate.angle_error = result.pose_error.tail<3>().norm();
		candidate.clearance_shortfall = this->getClearanceShortfall(state, result.q);
		candidate.gap = scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + candidate.clearance_shortfall;
		best_gap = std::min(best_gap, candidate.gap);
		is_roomy.push_back(candidate.clearance_shortfall <= 0.0 && this->getSmallestJointMargin({result.q}) >= this->cost_weights.joint_room_band);
		all_candidates.push_back(candidate);
	}
	if (this->ik_settings.closest_options.use_weighted_cost)
	{
		// The weighted cost already counts room and clearance: the cheapest is the default, those within the gap slack of it GTSP choices.
		size_t cheapest_index = 0;
		for (size_t candidate_index = 1; candidate_index < closest_results.size(); ++candidate_index)
			if (closest_results[candidate_index].weighted_cost < closest_results[cheapest_index].weighted_cost)
				cheapest_index = candidate_index;
		std::vector<ClosestIkCandidate> near_candidates = {all_candidates[cheapest_index]};
		for (size_t candidate_index = 0; candidate_index < all_candidates.size(); ++candidate_index)
			if (candidate_index != cheapest_index && all_candidates[candidate_index].gap <= all_candidates[cheapest_index].gap + this->ik_settings.closest_gap_slack)
				near_candidates.push_back(all_candidates[candidate_index]);
		return near_candidates;
	}
	std::vector<ClosestIkCandidate> near_candidates, near_roomy_candidates;
	for (size_t candidate_index = 0; candidate_index < all_candidates.size(); ++candidate_index)
	{
		if (all_candidates[candidate_index].gap > best_gap + this->ik_settings.closest_gap_slack)
			continue;
		near_candidates.push_back(all_candidates[candidate_index]);
		if (is_roomy[candidate_index])
			near_roomy_candidates.push_back(all_candidates[candidate_index]);
	}
	return near_roomy_candidates.empty() ? near_candidates : near_roomy_candidates;
}

closest_ik::Result Optimizer::solveSmallestGapIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts)
{
	closest_ik::Result best_result;
	double best_gap = std::numeric_limits<double>::max();
	for (const closest_ik::Result& result : this->solveClosestIkFromStarts(state, target, starts))
	{
		if (result.exact)
			return result;
		const double gap = scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + this->getClearanceShortfall(state, result.q);
		if (gap < best_gap)
		{
			best_result = result;
			best_gap = gap;
		}
	}
	return best_result;
}

closest_ik::Result Optimizer::pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest_result)
{
	const double margin = this->ik_settings.exact_options.exact_margin;
	if (margin <= 0.0)
		return closest_result;
	const double goal_clearance = margin + 0.002;  // m: a little past the exact poses' clearance
	const double max_joint_step = 0.05;	 // rad per iteration
	const int max_iterations = 40;
	Joints joint_values = closest_result.q;
	for (int iteration = 0; iteration < max_iterations; ++iteration)
	{
		state.setJointGroupPositions(this->group, joint_values);
		state.update();
		const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, goal_clearance), goal_clearance);
		Eigen::VectorXd clearance_push = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_values.size()));
		for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
		{
			const size_t split = close_pairs.names[pair_index].find(" - ");
			if (this->camera_side_links.count(close_pairs.names[pair_index].substr(0, split)) || this->camera_side_links.count(close_pairs.names[pair_index].substr(split + 3)))
				continue;
			const Eigen::VectorXd gradient = close_pairs.gradients[pair_index].transpose();
			clearance_push += (goal_clearance - close_pairs.dists[pair_index]) * gradient / (gradient.squaredNorm() + 1e-9);
		}
		if (clearance_push.norm() < 1e-9)
			break;
		const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->ik_settings.exact_options.ignore_roll);
		const Eigen::MatrixXd jacobian = this->getToolJacobian(state, joint_values);
		const Eigen::MatrixXd pseudo_inverse = scene_geometry::DampedPinv(jacobian, this->ik_settings.jacobian_damping);
		const Eigen::MatrixXd self_motion = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - pseudo_inverse * jacobian;
		Eigen::VectorXd joint_step = pseudo_inverse * pose_error + self_motion * clearance_push;
		if (joint_step.cwiseAbs().maxCoeff() > max_joint_step)
			joint_step *= max_joint_step / joint_step.cwiseAbs().maxCoeff();
		for (size_t joint_index = 0; joint_index < joint_values.size(); ++joint_index)
			joint_values[joint_index] += joint_step[static_cast<Eigen::Index>(joint_index)];
		state.setJointGroupPositions(this->group, joint_values);
		state.enforceBounds(this->group);
		state.copyJointGroupPositions(this->group, joint_values);
	}
	const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, joint_values, this->ik_settings.exact_options);
	if (exact.exact)
		return exact;
	if (!scene_geometry::IsStateValid(this->scene, &state, this->group, joint_values.data()))
		return closest_result;
	state.setJointGroupPositions(this->group, joint_values);
	state.update();
	closest_ik::Result finished;
	finished.found = true;
	finished.exact = false;
	finished.pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->ik_settings.exact_options.ignore_roll);
	finished.gap = scene_geometry::PoseGap(finished.pose_error, this->ik_settings.closest_options.rot_scale);
	finished.q = joint_values;
	finished.weighted_cost = closest_result.weighted_cost;	// the push only moves the arm out to the exact clearance
	const double finished_gap = finished.gap + this->getClearanceShortfall(state, joint_values);
	const double closest_gap = scene_geometry::PoseGap(closest_result.pose_error, this->ik_settings.closest_options.rot_scale) + this->getClearanceShortfall(state, closest_result.q);
	return finished_gap < closest_gap ? finished : closest_result;
}

double Optimizer::getClearanceShortfall(moveit::core::RobotState& state, const Joints& joint_values) const
{
	const double exact_margin = this->ik_settings.exact_options.exact_margin;
	if (exact_margin <= 0.0)
		return 0.0;
	state.setJointGroupPositions(this->group, joint_values);
	state.update();
	const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, exact_margin), exact_margin);
	double clearance = exact_margin;  // smallest arm link distance (camera side aside), looked for up to the margin
	for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
	{
		const size_t split = close_pairs.names[pair_index].find(" - ");
		if (this->camera_side_links.count(close_pairs.names[pair_index].substr(0, split)) || this->camera_side_links.count(close_pairs.names[pair_index].substr(split + 3)))
			continue;
		clearance = std::min(clearance, close_pairs.dists[pair_index]);
	}
	return std::max(0.0, exact_margin - clearance);
}

uint32_t Optimizer::getRandomIkSeed(const Eigen::Vector3d& position, size_t viewpoint_index) const
{
	const Eigen::Vector3d cell = (position * 1e4).array().round();
	uint64_t hash = static_cast<uint64_t>(this->ik_settings.random_ik_seed) * 1000003ULL + static_cast<uint64_t>(viewpoint_index) * 7919ULL;
	for (int axis = 0; axis < 3; ++axis)
		hash = hash * 6364136223846793005ULL + static_cast<uint64_t>(static_cast<int64_t>(cell[axis]));
	return static_cast<uint32_t>(hash ^ (hash >> 32));
}

std::vector<Joints> Optimizer::getChosenPoses(const ViewpointIkAtPosition& viewpoint_ik) const
{
	std::vector<Joints> poses;
	for (const ClosestIkCandidate& chosen_ik : viewpoint_ik.chosen_ik_per_viewpoint)
		poses.push_back(chosen_ik.q);
	return poses;
}

std::vector<int> Optimizer::reorderTourWithGtsp(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	std::vector<std::vector<Joints>> candidates(this->viewpoints.size());
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		candidates[viewpoint_index].push_back(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		// A missed viewpoint's other closest poses (near its best gap, with room): GTSP picks by tour.
		if (!viewpoint_ik.reached[viewpoint_index] && viewpoint_index < viewpoint_ik.closest_ik_candidates.size())
			for (ClosestIkCandidate& candidate : viewpoint_ik.closest_ik_candidates[viewpoint_index])
			{
				// Same turn as the current pose (see unwrapFullTurnJointsAlongTour), so the GTSP choice can be matched back to its gap below.
				this->unwrapFullTurnJointsToward(candidate.q, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
				if (candidate.q != viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q)
					candidates[viewpoint_index].push_back(candidate.q);
			}
	}
	if (this->method_switches.use_neighbor_ik && order.size() == this->viewpoints.size())
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
		std::vector<std::thread> threads;
		for (unsigned int thread_index = 0; thread_index < this->num_threads; ++thread_index)
			threads.emplace_back([&, thread_index]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t tour_position = thread_index; tour_position < order.size(); tour_position += this->num_threads)
				{
					const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
					if (!viewpoint_ik.reached[viewpoint_index])
						continue;
					const Eigen::Isometry3d target = scene_geometry::Translation(viewpoint_ik.position) * this->viewpoints[viewpoint_index];
					std::vector<const Joints*> neighbor_poses = {tour_position == 0 ? &this->home_pose : &viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q};
					if (tour_position + 1 < order.size())
						neighbor_poses.push_back(&viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position + 1])].q);
					for (const Joints* neighbor_pose : neighbor_poses)
					{
						const closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *neighbor_pose, this->ik_settings.exact_options);
						if (result.exact)
							candidates[viewpoint_index].push_back(result.q);
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
	}
	// Candidates in the same turn as the viewpoint's current pose, so GTSP doesn't see fake full-turn swings.
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		for (Joints& candidate : candidates[viewpoint_index])
			this->unwrapFullTurnJointsToward(candidate, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
	gtsp::Solver solver(candidates, this->home_pose, this->tour_settings.edge_weights, this->tour_settings.two_opt_rounds);
	solver.SetTravelJoints(this->counted_joints);
	gtsp::Tour tour = solver.Solve(order);
	// GTSP sees straight-line lengths only: charge the chosen edges whose straight move collides and solve again, so the
	// order can route around them (checking every pair up front would be too slow).
	for (int round = 0; round < this->tour_settings.gtsp_blocked_rounds && this->cost_weights.blocked_edge_charge > 0.0; ++round)
	{
		ViewpointIkAtPosition tour_viewpoint_ik = viewpoint_ik;
		for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
			tour_viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(tour.order[tour_position])].q = tour.poses[tour_position];
		const std::vector<std::pair<size_t, double>> blocked_edges = this->getBlockedEdgeDetourCharges(tour_viewpoint_ik, tour.order);
		if (blocked_edges.empty())
			break;
		for (const std::pair<size_t, double>& edge : blocked_edges)
		{
			const size_t tour_position = edge.first;
			const size_t to_viewpoint = static_cast<size_t>(tour.order[tour_position]);
			const std::pair<int, int> to_key = {static_cast<int>(to_viewpoint), this->findCandidateIndex(candidates[to_viewpoint], tour.poses[tour_position])};
			const std::pair<int, int> from_key = tour_position == 0 ? std::make_pair(-1, -1) : std::make_pair(tour.order[tour_position - 1], this->findCandidateIndex(candidates[static_cast<size_t>(tour.order[tour_position - 1])], tour.poses[tour_position - 1]));
			solver.AddEdgeCost(from_key, to_key, edge.second);
		}
		tour = solver.Solve(tour.order);
	}
	for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
	{
		const size_t viewpoint_index = static_cast<size_t>(tour.order[tour_position]);
		viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q = tour.poses[tour_position];
		if (viewpoint_ik.reached[viewpoint_index] || viewpoint_index >= viewpoint_ik.closest_ik_candidates.size())
			continue;
		// A missed viewpoint's chosen closest pose brings its own gap.
		for (const ClosestIkCandidate& candidate : viewpoint_ik.closest_ik_candidates[viewpoint_index])
		{
			if (candidate.q != tour.poses[tour_position])
				continue;
			viewpoint_ik.gap_sum += candidate.gap - viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
			viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index] = candidate;
		}
	}
	return tour.order;
}

int Optimizer::findCandidateIndex(const std::vector<Joints>& candidates, const Joints& pose) const
{
	for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index)
		if (candidates[candidate_index] == pose)
			return static_cast<int>(candidate_index);
	return 0;
}

void Optimizer::unwrapFullTurnJointsAlongTour(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const
{
	const Joints* previous = &this->home_pose;
	for (int viewpoint : order)
	{
		Joints& pose = viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(viewpoint)].q;
		this->unwrapFullTurnJointsToward(pose, *previous);
		previous = &pose;
	}
}

void Optimizer::unwrapFullTurnJointsToward(Joints& pose, const Joints& reference) const
{
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	const double full_turn = 2.0 * M_PI;
	for (size_t joint_index = 0; joint_index < pose.size() && joint_index < joints.size() && joint_index < reference.size(); ++joint_index)
	{
		const moveit::core::VariableBounds& bounds = joints[joint_index]->getVariableBounds()[0];
		if (bounds.position_bounded_ && bounds.max_position_ - bounds.min_position_ < full_turn)
			continue;
		const double turns = std::round((reference[joint_index] - pose[joint_index]) / full_turn);
		for (double shift : {turns, turns - 1.0, turns + 1.0})
		{
			const double value = pose[joint_index] + shift * full_turn;
			const bool inside = !bounds.position_bounded_ || (value >= bounds.min_position_ && value <= bounds.max_position_);
			if (inside && std::abs(value - reference[joint_index]) < std::abs(pose[joint_index] - reference[joint_index]))
				pose[joint_index] = value;
		}
	}
}

bool Optimizer::isReconfiguration(const Joints& from, const Joints& to) const
{
	return joint_travel::MaxJointTurn(from, to, this->counted_joints) > this->tour_settings.reconfig_threshold;
}

int Optimizer::countReconfigurations(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const
{
	int num_reconfigurations = 0;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		num_reconfigurations += this->isReconfiguration(tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q, viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position])].q) ? 1 : 0;
	return num_reconfigurations;
}

int Optimizer::countReconfigurationsAround(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order, size_t tour_position, const Joints& pose) const
{
	const Joints& previous_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
	int num_reconfigurations = this->isReconfiguration(previous_pose, pose) ? 1 : 0;
	if (tour_position + 1 < order.size())
		num_reconfigurations += this->isReconfiguration(pose, viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position + 1])].q) ? 1 : 0;
	return num_reconfigurations;
}

void Optimizer::swapInReconfigurationPostureMatches(ObjectPositionEvaluation& evaluation)
{
	this->unwrapFullTurnJointsAlongTour(evaluation.viewpoint_ik, evaluation.order);
	evaluation.matches.clear();
	if (!this->method_switches.use_posture_match)
		return;
	ViewpointIkAtPosition& viewpoint_ik = evaluation.viewpoint_ik;
	const std::vector<int>& order = evaluation.order;
	std::vector<size_t> tour_position_of_viewpoint(this->viewpoints.size(), 0);
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		tour_position_of_viewpoint[static_cast<size_t>(order[tour_position])] = tour_position;
	for (int round = 0; round < 2; ++round)
	{
		evaluation.matches = this->findReconfigurationPostureMatches(viewpoint_ik, order);
		bool swapped = false;
		for (const ReconfigurationPostureMatch& match : evaluation.matches)
		{
			const size_t tour_position = tour_position_of_viewpoint[match.viewpoint_index];
			const int reconfigurations_before = this->countReconfigurationsAround(viewpoint_ik, order, tour_position, viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].q);
			const int reconfigurations_after = this->countReconfigurationsAround(viewpoint_ik, order, tour_position, match.pose_in_neighbor_posture);
			const bool removes_reconfiguration = match.exact && reconfigurations_after < reconfigurations_before;
			const bool closer_miss = !viewpoint_ik.reached[match.viewpoint_index] && !match.exact && match.gap < viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].gap && reconfigurations_after <= reconfigurations_before;
			if (!removes_reconfiguration && !closer_miss)
				continue;
			if (match.is_later_viewpoint)
				(removes_reconfiguration ? this->posture_swap_counts.later_from_earlier_removed : this->posture_swap_counts.later_from_earlier_closer) += 1;
			else
				(removes_reconfiguration ? this->posture_swap_counts.earlier_from_later_removed : this->posture_swap_counts.earlier_from_later_closer) += 1;
			viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].q = match.pose_in_neighbor_posture;
			if (match.exact && !viewpoint_ik.reached[match.viewpoint_index])
			{
				viewpoint_ik.reached[match.viewpoint_index] = true;
				viewpoint_ik.num_reached += 1;
			}
			const double new_gap = match.exact ? 0.0 : match.gap;
			viewpoint_ik.gap_sum += new_gap - viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].gap;
			viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].gap = new_gap;
			viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].clearance_shortfall = match.clearance_shortfall;
			viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].angle_error = match.angle_error;
			viewpoint_ik.chosen_ik_per_viewpoint[match.viewpoint_index].position_error = match.exact ? Eigen::Vector3d::Zero() : match.position_error;
			swapped = true;
		}
		if (!swapped)
			return;
	}
	evaluation.matches = this->findReconfigurationPostureMatches(viewpoint_ik, order);
}

std::vector<ReconfigurationPostureMatch> Optimizer::findReconfigurationPostureMatches(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	std::vector<size_t> reconfiguration_edges;  // tour position k of each reconfiguration edge order[k-1] (or home) -> order[k]
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (this->isReconfiguration(tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q, viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position])].q))
			reconfiguration_edges.push_back(tour_position);
	std::vector<ReconfigurationPostureMatch> matches(reconfiguration_edges.size());
	std::vector<char> found_per_edge(reconfiguration_edges.size(), 0);  // one byte per edge: written from parallel threads
	if (reconfiguration_edges.empty())
		return {};
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	if (!(this->posture_match_cache_position == viewpoint_ik.position))
	{
		this->posture_match_cache.clear();
		this->posture_match_cache_position = viewpoint_ik.position;
	}
	const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(reconfiguration_edges.size()));
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < num_workers; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t edge_index = thread_index; edge_index < reconfiguration_edges.size(); edge_index += num_workers)
			{
				const size_t tour_position = reconfiguration_edges[edge_index];
				const size_t to_viewpoint = static_cast<size_t>(order[tour_position]);
				const Joints& from_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
				// (viewpoint, neighbour pose): B in A's posture, and A in B's (home has no viewpoint to move).
				std::vector<std::pair<size_t, const Joints*>> posture_options = {{to_viewpoint, &from_pose}};
				if (tour_position > 0)
					posture_options.push_back({static_cast<size_t>(order[tour_position - 1]), &viewpoint_ik.chosen_ik_per_viewpoint[to_viewpoint].q});
				for (const std::pair<size_t, const Joints*>& posture_option : posture_options)
				{
					// The same viewpoint from the same start pose at this object position gives the same answer: solve it once.
					const std::pair<size_t, Joints> cache_key(posture_option.first, *posture_option.second);
					PostureMatchSolve solve;
					bool is_cached = false;
					{
						std::lock_guard<std::mutex> lock(this->posture_match_cache_mutex);
						const auto cached = this->posture_match_cache.find(cache_key);
						if (cached != this->posture_match_cache.end())
						{
							solve = cached->second;
							is_cached = true;
						}
					}
					if (!is_cached)
					{
						const Eigen::Isometry3d target = scene_geometry::Translation(viewpoint_ik.position) * this->viewpoints[posture_option.first];
						solve.result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *posture_option.second, this->ik_settings.exact_options);
						if (!solve.result.exact)
							solve.result = this->solveSmallestGapIk(thread_state, target, {*posture_option.second});
						solve.clearance_shortfall = solve.result.found && !solve.result.exact ? this->getClearanceShortfall(thread_state, solve.result.q) : 0.0;
						std::lock_guard<std::mutex> lock(this->posture_match_cache_mutex);
						this->posture_match_cache[cache_key] = solve;
					}
					const closest_ik::Result& result = solve.result;
					if (!result.found || this->isReconfiguration(*posture_option.second, result.q))
						continue;
					const double clearance_shortfall = solve.clearance_shortfall;
					const double gap = result.exact ? 0.0 : scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + clearance_shortfall;
					if (found_per_edge[edge_index] && gap >= matches[edge_index].gap)
						continue;
					found_per_edge[edge_index] = true;
					matches[edge_index].viewpoint_index = posture_option.first;
					matches[edge_index].neighbor_pose = *posture_option.second;
					matches[edge_index].pose_in_neighbor_posture = result.q;
					matches[edge_index].position_error = result.exact ? Eigen::Vector3d::Zero() : Eigen::Vector3d(result.pose_error.head<3>());
					matches[edge_index].gap = gap;
					matches[edge_index].clearance_shortfall = clearance_shortfall;
					matches[edge_index].angle_error = result.exact ? 0.0 : result.pose_error.tail<3>().norm();
					matches[edge_index].exact = result.exact;
					matches[edge_index].is_later_viewpoint = posture_option.first == to_viewpoint;
				}
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	std::vector<ReconfigurationPostureMatch> kept;
	for (size_t edge_index = 0; edge_index < reconfiguration_edges.size(); ++edge_index)
		if (found_per_edge[edge_index])
			kept.push_back(matches[edge_index]);
	return kept;
}

bool Optimizer::isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose) const
{
	const double check_step = 0.02;	 // rad
	double largest_joint_change = 0.0;
	for (size_t joint_index = 0; joint_index < from_pose.size(); ++joint_index)
		largest_joint_change = std::max(largest_joint_change, std::abs(to_pose[joint_index] - from_pose[joint_index]));
	const int num_checks = std::max(1, static_cast<int>(std::ceil(largest_joint_change / check_step)));
	Joints interpolated_pose(from_pose.size());
	for (int check_index = 1; check_index < num_checks; ++check_index)
	{
		const double fraction = static_cast<double>(check_index) / num_checks;
		for (size_t joint_index = 0; joint_index < from_pose.size(); ++joint_index)
			interpolated_pose[joint_index] = from_pose[joint_index] + fraction * (to_pose[joint_index] - from_pose[joint_index]);
		if (!scene_geometry::IsStateValid(this->scene, &state, this->group, interpolated_pose.data()))
			return false;
	}
	return true;
}

std::vector<std::pair<size_t, double>> Optimizer::getBlockedEdgeDetourCharges(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	if (this->cost_weights.blocked_edge_charge <= 0.0 || order.empty())
		return {};
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	std::vector<double> detour_charges(order.size(), -1.0);  // -1: straight move free
	const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(order.size()));
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < num_workers; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t tour_position = thread_index; tour_position < order.size(); tour_position += num_workers)
			{
				const Joints& from_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
				const Joints& to_pose = viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position])].q;
				if (this->isStraightJointMoveCollisionFree(thread_state, from_pose, to_pose))
					continue;
				const double straight_cost = joint_travel::EdgeCost(from_pose, to_pose, this->tour_settings.edge_weights, this->counted_joints);
				const bool via_home = tour_position > 0 && this->isStraightJointMoveCollisionFree(thread_state, from_pose, this->home_pose) && this->isStraightJointMoveCollisionFree(thread_state, this->home_pose, to_pose);
				const double detour_cost = joint_travel::EdgeCost(from_pose, this->home_pose, this->tour_settings.edge_weights, this->counted_joints) + joint_travel::EdgeCost(this->home_pose, to_pose, this->tour_settings.edge_weights, this->counted_joints);
				detour_charges[tour_position] = via_home ? std::max(0.0, detour_cost - straight_cost) : this->cost_weights.blocked_edge_charge * straight_cost;
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	std::vector<std::pair<size_t, double>> blocked_edges;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (detour_charges[tour_position] >= 0.0)
			blocked_edges.push_back({tour_position, detour_charges[tour_position]});
	return blocked_edges;
}

void Optimizer::logObjectPositionEvaluation(const std::string& label, const ObjectPositionEvaluation& evaluation)
{
	RCLCPP_INFO(this->node->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%zu  gap sum %.4f m  reconfigurations %d (posture gap sum %.3f m)  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f", label.c_str(), evaluation.viewpoint_ik.position.x(), evaluation.viewpoint_ik.position.y(), evaluation.viewpoint_ik.position.z(), evaluation.viewpoint_ik.num_reached, this->viewpoints.size(), evaluation.viewpoint_ik.gap_sum, evaluation.num_reconfigurations, evaluation.posture_gap_sum, evaluation.num_blocked_edges, evaluation.joint_travel_cost, evaluation.smallest_margin, evaluation.weighted_total_cost);
}

void Optimizer::logMissedViewpoints(const ViewpointIkAtPosition& viewpoint_ik)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	const double limit_band = 0.02;  // rad: a joint this close to its (hard) limit counts as at it
	const double contact_band = this->ik_settings.closest_options.margin + 0.002;  // m
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		if (viewpoint_ik.reached[viewpoint_index])
			continue;
		std::string blockers;
		for (size_t joint_index = 0; joint_index < joints.size() && joint_index < viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q.size(); ++joint_index)
		{
			const moveit::core::VariableBounds& bounds = joints[joint_index]->getVariableBounds()[0];
			const double value = viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q[joint_index];
			if (bounds.position_bounded_ && (value - bounds.min_position_ < limit_band || bounds.max_position_ - value < limit_band))
				blockers += " " + joints[joint_index]->getName() + " at limit;";
		}
		this->state->setJointGroupPositions(this->group, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		this->state->update();
		const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, contact_band), contact_band);
		for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
			blockers += " " + close_pairs.names[pair_index] + " " + std::to_string(static_cast<int>(std::round(close_pairs.dists[pair_index] * 1000.0))) + " mm;";
		const bool valid = scene_geometry::IsStateValid(this->scene, this->state.get(), this->group, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q.data());
		RCLCPP_INFO(this->node->get_logger(), "  missed viewpoint %zu: gap %.2f mm (position %.2f mm, angle %.3f deg, clearance short %.2f mm, closest pose %s), blocked by:%s", viewpoint_index, 1000.0 * viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap, 1000.0 * viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].position_error.norm(), viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].angle_error * 180.0 / M_PI, 1000.0 * viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].clearance_shortfall, valid ? "valid" : "INVALID", blockers.empty() ? " nothing (reach)" : blockers.c_str());
	}
}

void Optimizer::publishProgressMarkers(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order, const Eigen::Vector3d& object_direction)
{
	if (!this->progress_pub)
		return;
	const rclcpp::Time stamp = this->node->now();
	const Eigen::Isometry3d object_transform = scene_geometry::Translation(viewpoint_ik.position);
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
	mesh.pose = scene_geometry::ToPoseMsg(scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	mesh.scale.x = mesh.scale.y = mesh.scale.z = this->mesh_scale;
	mesh.color = marker_utils::Rgba(1.0f, 0.85f, 0.0f, 0.6f);
	markers.markers.push_back(mesh);

	visualization_msgs::msg::Marker viewpoint_markers = marker_utils::MakeMarker("object_placement_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	viewpoint_markers.scale.x = viewpoint_markers.scale.y = viewpoint_markers.scale.z = 0.01;
	viewpoint_markers.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		viewpoint_markers.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[viewpoint_index]).translation()));
		viewpoint_markers.colors.push_back(viewpoint_ik.reached[viewpoint_index] ? marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f) : marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f));
	}
	markers.markers.push_back(viewpoint_markers);

	// Trail as one cylinder per move: line markers vanish from some viewing angles.
	visualization_msgs::msg::Marker trail_spheres = marker_utils::MakeMarker("object_placement_history", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	trail_spheres.scale.x = trail_spheres.scale.y = trail_spheres.scale.z = 0.012;
	trail_spheres.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t trail_index = 0; trail_index < this->object_trail.size(); ++trail_index)
	{
		trail_spheres.points.push_back(scene_geometry::ToPoint(this->object_trail[trail_index]));
		const double length = trail_index == 0 ? 0.0 : (this->object_trail[trail_index] - this->object_trail[trail_index - 1]).norm();
		if (length < 1e-6)
			continue;
		visualization_msgs::msg::Marker segment = marker_utils::MakeMarker("object_placement_trail", visualization_msgs::msg::Marker::CYLINDER, stamp, static_cast<int>(trail_index));
		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		pose.translation() = 0.5 * (this->object_trail[trail_index] + this->object_trail[trail_index - 1]);
		pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), this->object_trail[trail_index] - this->object_trail[trail_index - 1]).toRotationMatrix();
		segment.pose = scene_geometry::ToPoseMsg(pose);
		segment.scale.x = segment.scale.y = 0.005;
		segment.scale.z = length;
		segment.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
		markers.markers.push_back(segment);
	}
	markers.markers.push_back(trail_spheres);

	if (object_direction.norm() > 1e-9)
	{
		visualization_msgs::msg::Marker arrow = marker_utils::MakeMarker("object_placement_descent_dir", visualization_msgs::msg::Marker::ARROW, stamp);
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.color = marker_utils::Rgba(0.1f, 0.5f, 1.0f, 0.95f);
		arrow.points = {scene_geometry::ToPoint(viewpoint_ik.position), scene_geometry::ToPoint(viewpoint_ik.position + 0.15 * object_direction.normalized())};
		markers.markers.push_back(arrow);
	}

	// The tour's tool path from home, through each pose (missed ones at their closest pose).
	visualization_msgs::msg::Marker tool_path = marker_utils::MakeMarker("object_placement_tool_path", visualization_msgs::msg::Marker::LINE_STRIP, stamp);
	tool_path.scale.x = 0.003;
	tool_path.color = marker_utils::Rgba(0.2f, 0.6f, 1.0f, 0.9f);
	moveit::core::RobotState marker_state(*this->state);
	std::vector<Joints> path_poses = this->getPosesInTourOrder(viewpoint_ik, order);
	path_poses.insert(path_poses.begin(), this->home_pose);
	for (const Joints& pose : path_poses)
	{
		marker_state.setJointGroupPositions(this->group, pose);
		marker_state.update();
		tool_path.points.push_back(scene_geometry::ToPoint(marker_state.getGlobalLinkTransform(this->tool).translation()));
	}
	if (tool_path.points.size() > 1)
		markers.markers.push_back(tool_path);

	visualization_msgs::msg::Marker miss_gap_lines = marker_utils::MakeMarker("object_placement_miss_gap", visualization_msgs::msg::Marker::LINE_LIST, stamp);
	miss_gap_lines.scale.x = 0.003;
	miss_gap_lines.color = marker_utils::Rgba(1.0f, 0.2f, 1.0f, 1.0f);
	visualization_msgs::msg::MarkerArray missed_arm_markers;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		if (viewpoint_ik.reached[viewpoint_index])
			continue;
		marker_state.setJointGroupPositions(this->group, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		marker_state.update();
		marker_state.getRobotMarkers(missed_arm_markers, this->model->getLinkModelNamesWithCollisionGeometry(), marker_utils::Rgba(0.9f, 0.2f, 0.2f, 0.35f), "object_placement_miss_ik", rclcpp::Duration(0, 0));
		miss_gap_lines.points.push_back(scene_geometry::ToPoint(marker_state.getGlobalLinkTransform(this->tool).translation()));
		miss_gap_lines.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[viewpoint_index]).translation()));
	}
	if (!miss_gap_lines.points.empty())
		markers.markers.push_back(miss_gap_lines);
	for (visualization_msgs::msg::Marker& arm : missed_arm_markers.markers)
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
