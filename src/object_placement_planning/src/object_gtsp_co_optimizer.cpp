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
	DescentLimits descent_limits, MethodSwitches method_switches, CostWeights cost_weights, StepSizes step_sizes, IkSettings ik_settings, TourSettings tour_settings, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub, std::string mesh_path, double mesh_scale, bool show_tour_lines)
	: node(std::move(node)), model(std::move(model)), scene(std::move(scene)), home_pose(std::move(home_pose)), step_sizes(step_sizes), ik_settings(std::move(ik_settings)),
	  cost_weights(cost_weights), tour_settings(std::move(tour_settings)), descent_limits(std::move(descent_limits)), method_switches(method_switches), viewpoints(std::move(viewpoints)), object_rotation(object_rotation), mesh_path(std::move(mesh_path)), mesh_scale(mesh_scale), show_tour_lines(show_tour_lines), progress_pub(std::move(progress_pub))
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
	random_numbers::RandomNumberGenerator reference_rng(static_cast<uint32_t>(this->ik_settings.random_ik_seed));
	moveit::core::RobotState reference_state(*this->state);
	for (int reference_index = 0; reference_index < this->tour_settings.num_reference_postures; ++reference_index)
	{
		Joints reference_posture;
		reference_state.setToRandomPositions(this->group, reference_rng);
		reference_state.copyJointGroupPositions(this->group, reference_posture);
		this->reference_postures.push_back(reference_posture);
	}
}

TourSolutionAtObjectPosition Optimizer::runDescent(const Eigen::Vector3d& requested_start)
{
	this->start_time = std::chrono::steady_clock::now();
	const Eigen::Vector3d start = this->clampToSearchBox(requested_start);
	this->miss_gap_penalty = this->cost_weights.initial_penalty;

	// Find initial IK for all viewpoints, order viewpoints, and improve IK
	TourSolutionAtObjectPosition current_solution = this->solveIkAndImproveTourAtPosition(start, std::vector<Joints>(this->viewpoints.size(), this->home_pose), {}, {});

	const int num_viewpoint = static_cast<int>(this->viewpoints.size());
	double reach_step = this->step_sizes.initial_step;
	double tour_step = this->step_sizes.initial_step;
	double reconfiguration_step = this->step_sizes.initial_step;
	this->object_trail.assign(1, start);
	this->publishProgressMarkers(current_solution.viewpoint_ik, current_solution.order, Eigen::Vector3d::Zero());
	this->logTourSolution("start", current_solution);
	bool object_frozen = false;	 // freeze_object_when_reached: set once every viewpoint is reached
	
	// Optimization starts.
	for (int iteration = 1; iteration <= this->descent_limits.max_steps && rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - this->start_time).count() < this->descent_limits.time_budget; ++iteration)
	{
		bool any_step_accepted = false;

		// Room-then-reach step: move the object toward the missed viewpoints (room-then-reach if that fails); once all are reached, free what blocks the reconfiguration edges, treated as misses.
		if (!object_frozen)
			any_step_accepted = this->moveObjectToReachMore(current_solution, reach_step, reconfiguration_step, iteration) || any_step_accepted;

		if (this->method_switches.freeze_object_when_reached && !object_frozen && current_solution.viewpoint_ik.num_reached == num_viewpoint)
		{
			object_frozen = true;
			RCLCPP_INFO(this->node->get_logger(), "iteration %d: all reached; object frozen at abs (%.4f, %.4f, %.4f)", iteration, current_solution.viewpoint_ik.position.x(), current_solution.viewpoint_ik.position.y(), current_solution.viewpoint_ik.position.z());
		}

		any_step_accepted = this->improveArmPosesWithObjectFixed(current_solution, iteration) || any_step_accepted;

		// Tour placement step: move the object to lower the tour cost.
		if (this->method_switches.tour_object_move && !object_frozen)
			any_step_accepted = this->moveObjectToLowerCost(current_solution, tour_step, iteration) || any_step_accepted;

		if (any_step_accepted)
			continue;
		if (current_solution.viewpoint_ik.num_reached == num_viewpoint || this->miss_gap_penalty >= this->cost_weights.max_penalty)
			break;

		this->miss_gap_penalty = std::min(this->cost_weights.max_penalty, this->miss_gap_penalty * this->cost_weights.penalty_growth);
		this->refinePosesAndComputeCost(current_solution);
		reach_step = this->step_sizes.initial_step;
		tour_step = this->step_sizes.initial_step;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d: nothing changed with %d missed; penalty -> %.0f per m", iteration, num_viewpoint - current_solution.viewpoint_ik.num_reached, this->miss_gap_penalty);
		this->logMissedViewpoints(current_solution.viewpoint_ik);
	}
	this->logTourSolution("end", current_solution);
	this->logMissedViewpoints(current_solution.viewpoint_ik);
	return current_solution;
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

// The tour solution at one object position: IK for every viewpoint from start_poses, then the order and poses improved and scored.
TourSolutionAtObjectPosition Optimizer::solveIkAndImproveTourAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const std::vector<int>& order)
{
	TourSolutionAtObjectPosition solution;
	solution.viewpoint_ik = this->solveViewpointIkAtPosition(this->clampToSearchBox(position), start_poses, fallback_start_poses);
	// No order yet (the descent start): a first one from GTSP, since an empty tour would score as almost free.
	if (order.empty())
		solution.order = this->reorderTourWithGtsp(solution.viewpoint_ik, {}); // Sample multiple IKs for each viewpoint once, then solve GTSP multiple times.
	else
		solution.order = order;
	this->improveOrderAndPosturesWithObjectFixed(solution);
	return solution;
}

void Optimizer::improveOrderAndPosturesWithObjectFixed(TourSolutionAtObjectPosition& solution_to_complete)
{
	auto get_seconds_since = [](std::chrono::steady_clock::time_point since) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count(); };
	TourSolutionAtObjectPosition reordered_solution = solution_to_complete;
	this->neighbor_ik_seconds = 0.0;
	auto since = std::chrono::steady_clock::now();
	this->refinePosesAndComputeCost(solution_to_complete); // Keeps order fixed and refine IK solutions.
	double cost_seconds = get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	reordered_solution.order = this->reorderTourWithGtsp(reordered_solution.viewpoint_ik, reordered_solution.order); // Sample multiple IKs for each viewpoint once, then solve GTSP multiple times.
	const double gtsp_seconds = get_seconds_since(since);
	since = std::chrono::steady_clock::now();
	this->refinePosesAndComputeCost(reordered_solution);
	cost_seconds += get_seconds_since(since);
	RCLCPP_INFO(this->node->get_logger(), "  tour work: GTSP %.2f s, cost with straight-path and blocked-edge checks %.2f s (of it, misses from neighbour poses %.2f s)", gtsp_seconds, cost_seconds, this->neighbor_ik_seconds);
	const bool reordered_reaches_more = reordered_solution.viewpoint_ik.num_reached > solution_to_complete.viewpoint_ik.num_reached;
	const bool reordered_cheaper = reordered_solution.viewpoint_ik.num_reached == solution_to_complete.viewpoint_ik.num_reached && reordered_solution.weighted_total_cost < solution_to_complete.weighted_total_cost;
	if (reordered_reaches_more || reordered_cheaper)
		solution_to_complete = reordered_solution;
}

void Optimizer::refinePosesAndComputeCost(TourSolutionAtObjectPosition& solution)
{
	// 1. For a missed viewpoints, re-solve its closest IK from its previous viewpoint IK and next viewpoint IK.
	// 2. For reached viewpoints, run a straight line path from viewpoint A to B. Update B's IK with that arriving IK.

	this->unwrapFullTurnJointsAlongTour(solution.viewpoint_ik, solution.order);
	// Misses first (they may come closer in a neighbour's posture), then reconfigurations: following the straight camera paths may change poses.
	this->bringMissesCloserFromNeighbors(solution.viewpoint_ik, solution.order);
	solution.num_reconfigurations = this->followTourAlongStraightCameraPaths(solution.viewpoint_ik, solution.order);
	// Tour length, with each edge whose straight move collides charged its estimated detour (see getBlockedEdgeDetourCharges).
	double detour_charge = 0.0;
	const std::vector<std::pair<size_t, double>> blocked_edges = this->getBlockedEdgeDetourCharges(solution.viewpoint_ik, solution.order);
	for (const std::pair<size_t, double>& edge : blocked_edges)
		detour_charge += edge.second;
	solution.num_blocked_edges = static_cast<int>(blocked_edges.size());
	std::vector<Joints> poses_from_home = this->getPosesInTourOrder(solution.viewpoint_ik, solution.order);
	poses_from_home.insert(poses_from_home.begin(), this->home_pose);
	solution.joint_travel_cost = joint_travel::TravelCost(poses_from_home, this->tour_settings.edge_weights, this->counted_joints) + detour_charge;
	solution.room_shortfall = 0.0;
	std::vector<Joints> reached_poses;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		// Collect joint limit info from reached viewpoints.
		if (!solution.viewpoint_ik.reached[viewpoint_index])
			continue;
		solution.room_shortfall += this->getJointRoomShortfall(solution.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, nullptr);
		reached_poses.push_back(solution.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
	}
	solution.smallest_margin = this->getSmallestJointMargin(reached_poses);
	const int num_missed = static_cast<int>(this->viewpoints.size()) - solution.viewpoint_ik.num_reached;
	solution.weighted_total_cost = solution.joint_travel_cost + this->cost_weights.miss_cost * num_missed + this->miss_gap_penalty * solution.viewpoint_ik.gap_sum + this->cost_weights.reconfiguration_cost * solution.num_reconfigurations + this->cost_weights.joint_room_weight * solution.room_shortfall;
}

// The room-then-reach step. Misses first: a move toward them, else room-then-reach. Once all are reached, the
// reconfiguration edges are treated as misses and their blockers freed (not with freeze_object_when_reached: the object stays).
bool Optimizer::moveObjectToReachMore(TourSolutionAtObjectPosition& current_solution, double& reach_step, double& reconfiguration_step, int iteration)
{
	bool move_accepted = false;
	if (current_solution.viewpoint_ik.num_reached < static_cast<int>(this->viewpoints.size()))
		move_accepted = this->moveObjectTowardMissedViewpoints(current_solution, reach_step, iteration);
	const bool all_reached = current_solution.viewpoint_ik.num_reached == static_cast<int>(this->viewpoints.size());
	if (this->method_switches.use_room && !this->method_switches.freeze_object_when_reached && all_reached && current_solution.num_reconfigurations > 0)
		move_accepted = this->moveObjectToUnblockReconfigurations(current_solution, reconfiguration_step, iteration) || move_accepted;
	return move_accepted;
}

bool Optimizer::moveObjectTowardMissedViewpoints(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration)
{
	const Eigen::Vector3d direction = this->getDirectionTowardMissedViewpoints(current_solution.viewpoint_ik);
	if (direction.isZero())
		return false;
	bool tried_room_then_reach = false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && object_step >= this->step_sizes.min_step; ++attempt)
	{
		// Simple pull using closest IK to close gap.
		const TourSolutionAtObjectPosition candidate_solution = this->solveIkAndImproveTourAtPosition(current_solution.viewpoint_ik.position + object_step * direction, this->getChosenPoses(current_solution.viewpoint_ik), {}, current_solution.order);
		std::vector<size_t> lost_viewpoints;
		for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
			if (current_solution.viewpoint_ik.reached[viewpoint_index] && !candidate_solution.viewpoint_ik.reached[viewpoint_index])
				lost_viewpoints.push_back(viewpoint_index);
		
		// Accept move if it reaches more viewpoints, or closer the miss gap without losing any viewpoints.
		const bool move_accepted = candidate_solution.viewpoint_ik.num_reached > current_solution.viewpoint_ik.num_reached || (candidate_solution.viewpoint_ik.num_reached == current_solution.viewpoint_ik.num_reached && candidate_solution.viewpoint_ik.gap_sum < current_solution.viewpoint_ik.gap_sum);
		RCLCPP_INFO(this->node->get_logger(), "iteration %d reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu (lost %zu)  gap sum %.4f m vs %.4f%s", iteration, object_step, candidate_solution.viewpoint_ik.position.x(), candidate_solution.viewpoint_ik.position.y(), candidate_solution.viewpoint_ik.position.z(), candidate_solution.viewpoint_ik.num_reached, this->viewpoints.size(), lost_viewpoints.size(), candidate_solution.viewpoint_ik.gap_sum, current_solution.viewpoint_ik.gap_sum, move_accepted ? "  <-- kept" : "");
		if (move_accepted)
		{
			current_solution = candidate_solution;
			this->object_trail.push_back(current_solution.viewpoint_ik.position);
			this->publishProgressMarkers(current_solution.viewpoint_ik, current_solution.order, direction);
			object_step = std::min(this->step_sizes.max_step, object_step * 1.5); // If this move accepted, next time move more.
			return true;
		}
		// Room-then-reach: once per step, a move that frees what blocks the misses; the next iteration aims again.
		if (this->method_switches.use_room && !tried_room_then_reach)
		{
			tried_room_then_reach = true;
			if (this->moveObjectToUnblockMisses(current_solution, object_step, iteration))
				return true;
		}
		object_step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	object_step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::improveArmPosesWithObjectFixed(TourSolutionAtObjectPosition& current_solution, int iteration)
{
	if (!this->method_switches.use_slide)
	{
		TourSolutionAtObjectPosition reordered_solution = current_solution;
		this->improveOrderAndPosturesWithObjectFixed(reordered_solution);
		const bool move_accepted = reordered_solution.weighted_total_cost < current_solution.weighted_total_cost - 1e-9;
		if (move_accepted)
			current_solution = reordered_solution;
		return move_accepted;
	}
	const std::vector<Eigen::VectorXd> joint_slides = this->getSelfMotionSlidesToLowerTourCost(current_solution);
	const bool has_slide = std::any_of(joint_slides.begin(), joint_slides.end(), [](const Eigen::VectorXd& slide) { return slide.size() > 0; });
	double slide_step = this->step_sizes.initial_slide_step;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && has_slide; ++attempt)
	{
		const TourSolutionAtObjectPosition candidate_solution = this->solveIkAndImproveTourAtPosition(current_solution.viewpoint_ik.position, this->getPosesAfterJointSlides(current_solution.viewpoint_ik, joint_slides, slide_step), this->getChosenPoses(current_solution.viewpoint_ik), current_solution.order);
		const bool move_accepted = candidate_solution.weighted_total_cost < current_solution.weighted_total_cost - this->step_sizes.min_slide_gain;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d poses: slide %.3f rad  reached %d/%zu  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, slide_step, candidate_solution.viewpoint_ik.num_reached, this->viewpoints.size(), candidate_solution.num_reconfigurations, candidate_solution.num_blocked_edges, candidate_solution.joint_travel_cost, candidate_solution.smallest_margin, candidate_solution.weighted_total_cost, current_solution.weighted_total_cost, move_accepted ? "  <-- kept" : "");
		if (move_accepted)
		{
			current_solution = candidate_solution;
			return true;
		}
		slide_step *= 0.5;
	}
	return false;
}

bool Optimizer::moveObjectToLowerCost(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration)
{
	const Eigen::Vector3d direction = this->getDirectionToLowerTourCost(current_solution);
	if (direction.isZero())
		return false;
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && object_step >= this->step_sizes.min_step; ++attempt)
	{
		const TourSolutionAtObjectPosition candidate_solution = this->solveIkAndImproveTourAtPosition(current_solution.viewpoint_ik.position + object_step * direction, this->getChosenPoses(current_solution.viewpoint_ik), {}, current_solution.order);
		// Once all are reached, a tour move may not add reconfigurations (it would undo the reconfiguration room moves).
		const bool adds_reconfiguration = this->method_switches.use_room && current_solution.viewpoint_ik.num_reached == static_cast<int>(this->viewpoints.size()) && candidate_solution.num_reconfigurations > current_solution.num_reconfigurations;
		const bool move_accepted = candidate_solution.weighted_total_cost < current_solution.weighted_total_cost - 1e-9 && !adds_reconfiguration;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d tour: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, object_step, candidate_solution.viewpoint_ik.position.x(), candidate_solution.viewpoint_ik.position.y(), candidate_solution.viewpoint_ik.position.z(), candidate_solution.viewpoint_ik.num_reached, this->viewpoints.size(), candidate_solution.viewpoint_ik.gap_sum, candidate_solution.num_reconfigurations, candidate_solution.num_blocked_edges, candidate_solution.joint_travel_cost, candidate_solution.smallest_margin, candidate_solution.weighted_total_cost, current_solution.weighted_total_cost, move_accepted ? "  <-- kept" : "");
		if (move_accepted)
		{
			current_solution = candidate_solution;
			this->object_trail.push_back(current_solution.viewpoint_ik.position);
			this->publishProgressMarkers(current_solution.viewpoint_ik, current_solution.order, direction);
			object_step = std::min(this->step_sizes.max_step, object_step * 1.5);
			return true;
		}
		object_step *= 0.5;
	}
	// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
	object_step = this->step_sizes.initial_step;
	return false;
}

bool Optimizer::moveObjectToUnblockMisses(TourSolutionAtObjectPosition& current_solution, double object_step, int iteration)
{
	const double blocking_distance_multiple = 6.0;  // a limit is blocking if closer than this multiple of its safe distance (joint 0.05 rad, stretch 0.02, collision 1 cm)
	// All misses: this runs after a reach step toward them failed; a miss with nothing limiting its pose adds no margin.
	std::vector<size_t> missed_viewpoints;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		if (!current_solution.viewpoint_ik.reached[viewpoint_index])
			missed_viewpoints.push_back(viewpoint_index);
	if (missed_viewpoints.empty())
		return false;
	const std::vector<room_margins::Margin> margins_before = this->getBlockingMarginsOfViewpoints(current_solution.viewpoint_ik, missed_viewpoints, blocking_distance_multiple);
	bool keeps_reached_clear = false;
	const Eigen::Vector3d move_direction = this->getRoomMoveDirection(current_solution.viewpoint_ik, margins_before, object_step, keeps_reached_clear);
	if (move_direction.isZero())
		return false;
	
	const TourSolutionAtObjectPosition candidate_solution = this->solveIkAndImproveTourAtPosition(current_solution.viewpoint_ik.position + object_step * move_direction, this->getChosenPoses(current_solution.viewpoint_ik), {}, current_solution.order);
	int num_lost = 0;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		num_lost += current_solution.viewpoint_ik.reached[viewpoint_index] && !candidate_solution.viewpoint_ik.reached[viewpoint_index] ? 1 : 0;
	const std::vector<room_margins::Margin> margins_after = this->getBlockingMarginsOfViewpoints(candidate_solution.viewpoint_ik, missed_viewpoints, blocking_distance_multiple);
	const double tightest_before = room_margins::Tightest(margins_before);
	const double tightest_after = margins_after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(margins_after);
	// Kept if it loses at most allowed_reach_loss of the reached viewpoints and reaches more or frees the misses.
	const bool loss_allowed = static_cast<double>(num_lost) <= this->cost_weights.allowed_reach_loss * static_cast<double>(current_solution.viewpoint_ik.num_reached);
	const bool move_accepted = loss_allowed && (candidate_solution.viewpoint_ik.num_reached > current_solution.viewpoint_ik.num_reached || tightest_after > tightest_before + 1e-3);
	RCLCPP_INFO(this->node->get_logger(), "iteration %d room-then-reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu (lost %d)  misses %zu  tightest margin %.2f -> %.2f floors  reached kept clear: %s%s", iteration, object_step, candidate_solution.viewpoint_ik.position.x(), candidate_solution.viewpoint_ik.position.y(), candidate_solution.viewpoint_ik.position.z(), candidate_solution.viewpoint_ik.num_reached, this->viewpoints.size(), num_lost, missed_viewpoints.size(), tightest_before, tightest_after, keeps_reached_clear ? "yes" : "no", move_accepted ? "  <-- kept" : "");
	if (!move_accepted)
		return false;
	current_solution = candidate_solution;
	this->object_trail.push_back(current_solution.viewpoint_ik.position);
	this->publishProgressMarkers(current_solution.viewpoint_ik, current_solution.order, move_direction);
	return true;
}

// Room-then-reach for reconfigurations, once every viewpoint is reached: frees what stops the straight camera paths where
// they stopped. Kept if no viewpoint is lost and there are fewer reconfigurations, or as many with their blockers freed.
bool Optimizer::moveObjectToUnblockReconfigurations(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration)
{
	const double blocking_distance_multiple = 6.0;  // as for the misses
	const std::vector<room_margins::Margin> margins_before = this->getBlockingMarginsOfReconfigurations(current_solution, blocking_distance_multiple);
	for (int attempt = 0; attempt < this->step_sizes.max_line_search_tries && object_step >= this->step_sizes.min_step; ++attempt)
	{
		bool keeps_reached_clear = false;
		const Eigen::Vector3d move_direction = this->getRoomMoveDirection(current_solution.viewpoint_ik, margins_before, object_step, keeps_reached_clear);
		if (move_direction.isZero())
			break;
		const TourSolutionAtObjectPosition candidate_solution = this->solveIkAndImproveTourAtPosition(current_solution.viewpoint_ik.position + object_step * move_direction, this->getChosenPoses(current_solution.viewpoint_ik), {}, current_solution.order);
		const std::vector<room_margins::Margin> margins_after = candidate_solution.num_reconfigurations > 0 ? this->getBlockingMarginsOfReconfigurations(candidate_solution, blocking_distance_multiple) : std::vector<room_margins::Margin>();
		const double tightest_before = room_margins::Tightest(margins_before);
		const double tightest_after = margins_after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(margins_after);
		const bool keeps_all_reached = candidate_solution.viewpoint_ik.num_reached == current_solution.viewpoint_ik.num_reached;
		const bool move_accepted = keeps_all_reached && (candidate_solution.num_reconfigurations < current_solution.num_reconfigurations || (candidate_solution.num_reconfigurations == current_solution.num_reconfigurations && tightest_after > tightest_before + 1e-3));
		RCLCPP_INFO(this->node->get_logger(), "iteration %d reconfiguration room: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu  reconfigurations %d vs %d  tightest margin %.2f -> %.2f floors  tour length %.2f  reached kept clear: %s%s", iteration, object_step, candidate_solution.viewpoint_ik.position.x(), candidate_solution.viewpoint_ik.position.y(), candidate_solution.viewpoint_ik.position.z(), candidate_solution.viewpoint_ik.num_reached, this->viewpoints.size(), candidate_solution.num_reconfigurations, current_solution.num_reconfigurations, tightest_before, tightest_after, candidate_solution.joint_travel_cost, keeps_reached_clear ? "yes" : "no", move_accepted ? "  <-- kept" : "");
		if (move_accepted)
		{
			current_solution = candidate_solution;
			this->object_trail.push_back(current_solution.viewpoint_ik.position);
			this->publishProgressMarkers(current_solution.viewpoint_ik, current_solution.order, move_direction);
			object_step = std::min(this->step_sizes.max_step, object_step * 1.5);
			return true;
		}
		object_step *= 0.5;
	}
	object_step = this->step_sizes.initial_step;
	return false;
}

// Most-room direction for these margins, with the parts that tighten what blocks the reached viewpoints dropped; if too
// little is left, the room direction itself (keeps_reached_clear false). Zero if no direction gives room.
Eigen::Vector3d Optimizer::getRoomMoveDirection(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<room_margins::Margin>& margins, double object_step, bool& keeps_reached_clear)
{
	keeps_reached_clear = false;
	if (margins.empty())
		return Eigen::Vector3d::Zero();
	const Eigen::Vector3d room_direction = room_margins::MostRoomDirection(margins, object_step);
	if (room_direction.isZero())
		return Eigen::Vector3d::Zero();
	const double reached_blocking_distance_multiple = 2.0;
	std::vector<size_t> reached_viewpoints;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		if (viewpoint_ik.reached[viewpoint_index])
			reached_viewpoints.push_back(viewpoint_index);
	const Eigen::Vector3d kept_clear_direction = this->dropPartsTighteningMargins(room_direction, this->getBlockingMarginsOfViewpoints(viewpoint_ik, reached_viewpoints, reached_blocking_distance_multiple));
	keeps_reached_clear = kept_clear_direction.norm() >= 0.3;
	return keeps_reached_clear ? Eigen::Vector3d(kept_clear_direction.normalized()) : room_direction;
}

// Margins at the poses where the tour's straight camera paths stopped (a pose on the path moves with the object like a
// viewpoint's pose), labelled with the edge's target viewpoint.
std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfReconfigurations(const TourSolutionAtObjectPosition& solution, double blocking_distance_multiple)
{
	std::vector<Joints> end_poses;
	const std::vector<size_t> reconfiguration_edges = this->getReconfigurationEdges(solution.viewpoint_ik, solution.order, &end_poses);
	std::vector<room_margins::Margin> margins;
	for (size_t tour_position : reconfiguration_edges)
	{
		if (end_poses[tour_position].empty())
			continue;
		const std::vector<room_margins::Margin> edge_margins = this->getBlockingMarginsOfPose(static_cast<size_t>(solution.order[tour_position]), end_poses[tour_position], blocking_distance_multiple);
		margins.insert(margins.end(), edge_margins.begin(), edge_margins.end());
	}
	return margins;
}

Eigen::Vector3d Optimizer::getDirectionTowardMissedViewpoints(const ViewpointIkAtPosition& viewpoint_ik)
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

// Downhill direction of the tour cost (tour length + joint room) for the object: unit, or zero.
Eigen::Vector3d Optimizer::getDirectionToLowerTourCost(const TourSolutionAtObjectPosition& solution)
{
	const ViewpointIkAtPosition& viewpoint_ik = solution.viewpoint_ik;
	// TODO: calculate gradient theoretically.
	const std::vector<Eigen::VectorXd> d_tour_length_d_joints = this->getDTourLengthDJointsPerViewpoint(viewpoint_ik, solution.order);
	Eigen::MatrixXd object_twist = Eigen::MatrixXd::Zero(6, 3);
	object_twist.topRows<3>() = Eigen::Matrix3d::Identity();
	Eigen::Vector3d object_move_to_shorten_tour = Eigen::Vector3d::Zero();
	Eigen::Vector3d object_move_to_gain_joint_room = Eigen::Vector3d::Zero();
	// Each pose follows the object by IK (J+): its joint gradients become object gradients, summed over the viewpoints.
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		const Eigen::MatrixXd jacobian = this->getToolJacobian(*this->state, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		const Eigen::MatrixXd d_joints_d_object_position = scene_geometry::DampedPinv(jacobian, this->ik_settings.jacobian_damping) * object_twist;
		Eigen::VectorXd d_room_shortfall_d_joints = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q.size()));
		if (viewpoint_ik.reached[viewpoint_index])
			this->getJointRoomShortfall(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, &d_room_shortfall_d_joints);
		object_move_to_shorten_tour -= d_joints_d_object_position.transpose() * d_tour_length_d_joints[viewpoint_index];
		object_move_to_gain_joint_room -= this->cost_weights.joint_room_weight * d_joints_d_object_position.transpose() * d_room_shortfall_d_joints;
	}
	const Eigen::Vector3d object_move = this->combineMovesWithoutConflict({object_move_to_shorten_tour, object_move_to_gain_joint_room});
	return object_move.norm() > 1e-12 ? Eigen::Vector3d(object_move.normalized()) : Eigen::Vector3d::Zero();
}

// Per reached viewpoint: the self-motion slide (camera stays on target) that lowers the tour cost (tour length + joint
// room), scaled so the largest over all poses has norm 1. Empty vector = no slide (missed, no self-motion, or slides off).
std::vector<Eigen::VectorXd> Optimizer::getSelfMotionSlidesToLowerTourCost(const TourSolutionAtObjectPosition& solution)
{
	const ViewpointIkAtPosition& viewpoint_ik = solution.viewpoint_ik;
	std::vector<Eigen::VectorXd> joint_slides(this->viewpoints.size());
	if (!this->method_switches.use_slide)
		return joint_slides;
	const std::vector<Eigen::VectorXd> d_tour_length_d_joints = this->getDTourLengthDJointsPerViewpoint(viewpoint_ik, solution.order);
	double largest_slide = 0.0;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		if (!viewpoint_ik.reached[viewpoint_index])
			continue;
		const Eigen::MatrixXd jacobian = this->getToolJacobian(*this->state, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		Eigen::VectorXd d_room_shortfall_d_joints = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q.size()));
		this->getJointRoomShortfall(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, &d_room_shortfall_d_joints);
		joint_slides[viewpoint_index] = -this->getSelfMotionProjector(jacobian) * (d_tour_length_d_joints[viewpoint_index] + this->cost_weights.joint_room_weight * d_room_shortfall_d_joints);
		largest_slide = std::max(largest_slide, joint_slides[viewpoint_index].norm());
	}
	for (Eigen::VectorXd& slide : joint_slides)
		if (slide.size() > 0)
			slide = largest_slide > 1e-9 ? Eigen::VectorXd(slide / largest_slide) : Eigen::VectorXd();
	return joint_slides;
}

// The move without its parts that would tighten any of the margins (repeated, as dropping one part can tighten another).
Eigen::Vector3d Optimizer::dropPartsTighteningMargins(const Eigen::Vector3d& move, const std::vector<room_margins::Margin>& margins) const
{
	Eigen::Vector3d direction = move;
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
	return direction;
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

// Per viewpoint: the gradient of the tour length (edge costs from home through the order) with respect to that viewpoint's joints.
std::vector<Eigen::VectorXd> Optimizer::getDTourLengthDJointsPerViewpoint(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const
{
	std::vector<Eigen::VectorXd> d_tour_length_d_joints(this->viewpoints.size(), Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size())));
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
	{
		const size_t to_viewpoint = static_cast<size_t>(order[tour_position]);
		const Joints& from_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
		// d(edge cost)/d(to) by central differences; the cost depends on to - from only, so d/d(from) is its negative.
		const double delta = 1e-5;
		Eigen::VectorXd d_edge_cost_d_to_joints = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home_pose.size()));
		Joints shifted = viewpoint_ik.chosen_ik_per_viewpoint[to_viewpoint].q;
		for (size_t joint_index = 0; joint_index < shifted.size(); ++joint_index)
		{
			const double value = shifted[joint_index];
			shifted[joint_index] = value + delta;
			const double cost_up = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value - delta;
			const double cost_down = joint_travel::EdgeCost(from_pose, shifted, this->tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value;
			d_edge_cost_d_to_joints[static_cast<Eigen::Index>(joint_index)] = (cost_up - cost_down) / (2.0 * delta);
		}
		d_tour_length_d_joints[to_viewpoint] += d_edge_cost_d_to_joints;
		if (tour_position > 0)
			d_tour_length_d_joints[static_cast<size_t>(order[tour_position - 1])] -= d_edge_cost_d_to_joints;
	}
	return d_tour_length_d_joints;
}

std::vector<Joints> Optimizer::getPosesAfterJointSlides(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<Eigen::VectorXd>& joint_slides, double slide_step) const
{
	std::vector<Joints> slid_poses = this->getChosenPoses(viewpoint_ik);
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	for (size_t viewpoint_index = 0; viewpoint_index < slid_poses.size(); ++viewpoint_index)
	{
		if (joint_slides[viewpoint_index].size() == 0)
			continue;
		for (size_t joint_index = 0; joint_index < slid_poses[viewpoint_index].size(); ++joint_index)
		{
			const moveit::core::VariableBounds& bounds = joints[joint_index]->getVariableBounds()[0];
			const double value = slid_poses[viewpoint_index][joint_index];
			slid_poses[viewpoint_index][joint_index] += slide_step * joint_slides[viewpoint_index][static_cast<Eigen::Index>(joint_index)];
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

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfViewpoints(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<size_t>& viewpoint_indices, double blocking_distance_multiple)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	std::vector<room_margins::Margin> margins;
	for (size_t viewpoint_index : viewpoint_indices)
	{
		const std::vector<room_margins::Margin> viewpoint_margins = this->getBlockingMarginsOfPose(viewpoint_index, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q, blocking_distance_multiple);
		margins.insert(margins.end(), viewpoint_margins.begin(), viewpoint_margins.end());
	}
	return margins;
}

std::vector<room_margins::Margin> Optimizer::getBlockingMarginsOfPose(size_t viewpoint_index, const Joints& joint_values, double blocking_distance_multiple)
{
	const double joint_safe_distance = 0.05;  // rad
	const double stretch_safe_distance = 0.02;
	const double collision_safe_distance = 0.01;  // m
	const double window = this->ik_settings.closest_options.margin + blocking_distance_multiple * collision_safe_distance;  // m, collision pairs looked at
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
		if (room_to_lower_limit < blocking_distance_multiple * joint_safe_distance)
			margins.push_back({static_cast<int>(viewpoint_index), joint_name + " low limit", room_to_lower_limit, joint_safe_distance, joint_rate_per_object_move.transpose() * joint_direction, joint_direction});
		if (room_to_upper_limit < blocking_distance_multiple * joint_safe_distance)
			margins.push_back({static_cast<int>(viewpoint_index), joint_name + " high limit", room_to_upper_limit, joint_safe_distance, joint_rate_per_object_move.transpose() * -joint_direction, -joint_direction});
	}
	const double stretch = this->getSmallestJacobianSingularValue(joint_values);
	if (stretch < blocking_distance_multiple * stretch_safe_distance)
	{
		const double delta = 1e-5;  // rad
		Eigen::VectorXd stretch_slope(num_joints);
		for (int joint_index = 0; joint_index < num_joints; ++joint_index)
		{
			Joints shifted = joint_values;
			shifted[static_cast<size_t>(joint_index)] += delta;
			stretch_slope(joint_index) = (this->getSmallestJacobianSingularValue(shifted) - stretch) / delta;
		}
		margins.push_back({static_cast<int>(viewpoint_index), "stretch", stretch, stretch_safe_distance, joint_rate_per_object_move.transpose() * stretch_slope, stretch_slope});
	}
	this->state->setJointGroupPositions(this->group, joint_values);
	this->state->update();
	const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, window), window);
	for (size_t pair_index = 0; pair_index < close_pairs.dists.size(); ++pair_index)
	{
		const Eigen::VectorXd joint_slope = close_pairs.gradients[pair_index].transpose();
		margins.push_back({static_cast<int>(viewpoint_index), "collision " + close_pairs.names[pair_index], close_pairs.dists[pair_index], collision_safe_distance, joint_rate_per_object_move.transpose() * joint_slope + close_pairs.object_slopes[pair_index], joint_slope});
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

// Given start IK solutions, find exact IK for each viewpoint. If no exact IK, find closest IK.
ViewpointIkAtPosition Optimizer::solveViewpointIkAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(position, this->object_rotation));
	const auto solve_start = std::chrono::steady_clock::now();
	closest_ik::ResetWeightedSolveTiming();
	ViewpointIkAtPosition viewpoint_ik;
	viewpoint_ik.position = position;
	viewpoint_ik.chosen_ik_per_viewpoint.assign(this->viewpoints.size(), ClosestIkCandidate());

	// Copy start IKs to viewpoint_IK. For the first iteration, start_poses is home pose.
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q = start_poses[viewpoint_index];

	viewpoint_ik.reached.assign(this->viewpoints.size(), false);
	viewpoint_ik.closest_ik_candidates.assign(this->viewpoints.size(), {});
	
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < this->num_threads; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t viewpoint_index = thread_index; viewpoint_index < this->viewpoints.size(); viewpoint_index += this->num_threads)
			{
				const Eigen::Isometry3d target = scene_geometry::Translation(position) * this->viewpoints[viewpoint_index];
				// Solve exact IK for each viewpoint from the start pose.
				closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, start_poses[viewpoint_index], this->ik_settings.exact_options);

				// If no solution, try exact IK using fallback start pose.
				if (!result.exact && viewpoint_index < fallback_start_poses.size())
					result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, fallback_start_poses[viewpoint_index], this->ik_settings.exact_options);

				// Still not exact: random starts (Not truely random though. Depends on viewpoint index, object position, and random seed).
				// TODO: use the same reference posture to start. 
				random_numbers::RandomNumberGenerator viewpoint_rng(this->getRandomIkSeed(position, viewpoint_index));
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
					// If still no exact IK, find closest IK.
					std::vector<Joints> closest_starts = {start_poses[viewpoint_index]};
					for (size_t random_start_index = 0; random_start_index < random_starts.size() && static_cast<int>(closest_starts.size()) <= this->ik_settings.closest_ik_extra_starts; ++random_start_index)
						closest_starts.push_back(random_starts[random_start_index]); // Start from start poses + 3 random IK
					const std::vector<closest_ik::Result> closest_results = this->solveClosestIkFromStarts(thread_state, target, closest_starts);
					result = closest_ik::Result();
					for (const closest_ik::Result& candidate : closest_results)
						if (candidate.exact)
							result = candidate;
					
					// Keep the good closest IK + backups within 5mm within the good closest IK.
					if (!result.exact && !closest_results.empty())
					{
						// TODO: test if 5mm gap and extra closest IK is needed.
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

// Sample multiple IKs for each viewpoint once, then solve GTSP multiple times
std::vector<int> Optimizer::reorderTourWithGtsp(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	std::vector<std::vector<Joints>> ik_candidates(this->viewpoints.size());
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		// Copy the already solved first round IK into ik_candidates.
		ik_candidates[viewpoint_index].push_back(viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
		// A missed viewpoint's other closest poses (near its best gap, with room): GTSP picks by tour.
		if (!viewpoint_ik.reached[viewpoint_index]) // For unreached viewpoints.
			for (ClosestIkCandidate& closest_ik_candidate : viewpoint_ik.closest_ik_candidates[viewpoint_index])
			{
				this->unwrapFullTurnJointsToward(closest_ik_candidate.q, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q); // Remove the fake full circle turn. e.g. Change from 370° to 10°.
				if (closest_ik_candidate.q != viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q) // Make sure an IK solution is not added twice.
					ik_candidates[viewpoint_index].push_back(closest_ik_candidate.q); 
			}
	}
	if (order.size() == this->viewpoints.size()) // Make sure we have an initial order to start with.
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation)); // Put object at the current solution position.
		std::vector<std::thread> threads;

		// Find extra IKs for each viewpoint. 
		for (unsigned int thread_index = 0; thread_index < this->num_threads; ++thread_index)
			threads.emplace_back([&, thread_index]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t tour_position = thread_index; tour_position < order.size(); tour_position += this->num_threads)
				{
					const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
					if (!viewpoint_ik.reached[viewpoint_index]) // Skip missed viewpoints. Will be solved afterwards.
						continue;
					const Eigen::Isometry3d target = scene_geometry::Translation(viewpoint_ik.position) * this->viewpoints[viewpoint_index];

					// Add previous and next viewpoint IK as a start point for exact IK.
					std::vector<const Joints*> neighbor_poses;
					if (tour_position == 0)
						neighbor_poses.push_back(&this->home_pose);
					else
						neighbor_poses.push_back(&viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q);
					if (tour_position + 1 < order.size())
						neighbor_poses.push_back(&viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position + 1])].q);
					
					// Now start exact IK solve.
					for (const Joints* neighbor_pose : neighbor_poses)
					{
						const closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *neighbor_pose, this->ik_settings.exact_options);
						if (result.exact)
							ik_candidates[viewpoint_index].push_back(result.q);
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
	}

	// For each viewpoint, solve more IKs from reference postures.
	if (!this->reference_postures.empty())
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
		const double same_pose_distance = 0.1;	// rad: closer than this to a candidate already there adds nothing.
		std::vector<std::thread> threads;

		// Solve exact IK using reference postures as start pose, unwrap, and not add them if they are too similar to other IKs.
		for (unsigned int thread_index = 0; thread_index < this->num_threads; ++thread_index)
			threads.emplace_back([&, thread_index]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t viewpoint_index = thread_index; viewpoint_index < this->viewpoints.size(); viewpoint_index += this->num_threads)
				{
					if (!viewpoint_ik.reached[viewpoint_index])
						continue;
					const Eigen::Isometry3d target = scene_geometry::Translation(viewpoint_ik.position) * this->viewpoints[viewpoint_index];
					for (const Joints& reference_posture : this->reference_postures)
					{
						closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, reference_posture, this->ik_settings.exact_options);
						if (!result.exact)
							continue;
						this->unwrapFullTurnJointsToward(result.q, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);
						const bool is_new = std::none_of(ik_candidates[viewpoint_index].begin(), ik_candidates[viewpoint_index].end(), [&](const Joints& candidate) { return joint_travel::JointDistance(candidate, result.q, this->counted_joints) < same_pose_distance; });
						if (is_new)
							ik_candidates[viewpoint_index].push_back(result.q);
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
	}

	// Additional unwrap to make sure robot doesn't do fake full turn.
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		for (Joints& candidate : ik_candidates[viewpoint_index])
			this->unwrapFullTurnJointsToward(candidate, viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q);

	// Solve GTSP.
	gtsp::Solver solver(ik_candidates, this->home_pose, this->tour_settings.edge_weights, this->tour_settings.two_opt_rounds);
	solver.SetTravelJoints(this->counted_joints);
	gtsp::Tour tour = solver.Solve(order);

	// Try to fix reconfiguration between viewpoints.
	for (int round = 0; round < this->tour_settings.gtsp_blocked_rounds; ++round)
	{
		ViewpointIkAtPosition tour_viewpoint_ik = viewpoint_ik;
		// Copy solved GTSP.
		for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
			tour_viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(tour.order[tour_position])].q = tour.poses[tour_position];

		std::vector<std::pair<size_t, double>> charged_edges;
		if (this->cost_weights.blocked_edge_charge > 0.0)
			charged_edges = this->getBlockedEdgeDetourCharges(tour_viewpoint_ik, tour.order); // Check if anything blocking the straight move from viewpoint A to B.

		for (size_t tour_position : this->getReconfigurationEdges(tour_viewpoint_ik, tour.order)) // Check if any reconfiguration is needed.
			charged_edges.push_back({tour_position, this->cost_weights.reconfiguration_cost});

		if (charged_edges.empty())
			break;

		for (const std::pair<size_t, double>& edge : charged_edges)
		{
			const size_t tour_position = edge.first;
			const size_t to_viewpoint = static_cast<size_t>(tour.order[tour_position]);
			const std::pair<int, int> to_key = {static_cast<int>(to_viewpoint), this->findCandidateIndex(ik_candidates[to_viewpoint], tour.poses[tour_position])};
			const std::pair<int, int> from_key = tour_position == 0 ? std::make_pair(-1, -1) : std::make_pair(tour.order[tour_position - 1], this->findCandidateIndex(ik_candidates[static_cast<size_t>(tour.order[tour_position - 1])], tour.poses[tour_position - 1]));
			solver.AddEdgeCost(from_key, to_key, edge.second); // Add extra cost to edges if they are either blocked or require reconfiguration.
		}
		tour = solver.Solve(tour.order); // Solve GTSP again with the new edge cost.
	}

	// Copy refined GTSP results to output.
	for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
	{
		const size_t viewpoint_index = static_cast<size_t>(tour.order[tour_position]);
		viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q = tour.poses[tour_position];
		if (viewpoint_ik.reached[viewpoint_index])
			continue;

		for (const ClosestIkCandidate& closest_ik_candidate : viewpoint_ik.closest_ik_candidates[viewpoint_index])
		{
			if (closest_ik_candidate.q != tour.poses[tour_position])
				continue;
			viewpoint_ik.gap_sum += closest_ik_candidate.gap - viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
			viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index] = closest_ik_candidate;
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

// Follows the camera from from_pose's camera pose to to_pose's on a straight line with an even turn, in small steps of one
// Jacobian update each (the next step corrects what is left); then one exact IK onto to_pose's camera pose gives the arrival
// pose. False at a joint jump, a joint near its limit or a collision on the way, or no exact IK at the end. end_pose: the
// arrival pose, or where the path stopped. Results are kept per (from pose, to pose) at this object position.
bool Optimizer::followStraightCameraPath(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose, Joints& end_pose)
{
	const std::pair<Joints, Joints> cache_key(from_pose, to_pose);
	{
		std::lock_guard<std::mutex> lock(this->straight_path_cache_mutex);
		const auto cached = this->straight_path_cache.find(cache_key);
		if (cached != this->straight_path_cache.end())
		{
			end_pose = cached->second.second;
			return cached->second.first;
		}
	}
	auto remember = [&](bool followed, const Joints& pose)
	{
		std::lock_guard<std::mutex> lock(this->straight_path_cache_mutex);
		this->straight_path_cache[cache_key] = {followed, pose};
		end_pose = pose;
		return followed;
	};
	state.setJointGroupPositions(this->group, from_pose);
	state.update();
	const Eigen::Isometry3d from_camera = state.getGlobalLinkTransform(this->tool);
	state.setJointGroupPositions(this->group, to_pose);
	state.update();
	const Eigen::Isometry3d to_camera = state.getGlobalLinkTransform(this->tool);
	const Eigen::Quaterniond from_rotation(from_camera.rotation());
	const Eigen::Quaterniond to_rotation(to_camera.rotation());
	const double camera_travel = (to_camera.translation() - from_camera.translation()).norm();
	const double camera_turn = from_rotation.angularDistance(to_rotation);
	const int num_steps = std::max(1, static_cast<int>(std::ceil(std::max(camera_travel / this->tour_settings.path_check_position_step, camera_turn / this->tour_settings.path_check_angle_step))));
	const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
	auto is_joint_step_ok = [&](const Joints& before, const Joints& after)
	{
		for (size_t joint_index = 0; joint_index < after.size(); ++joint_index)
		{
			if (std::abs(after[joint_index] - before[joint_index]) > this->tour_settings.path_check_max_joint_jump)
				return false;
			const moveit::core::VariableBounds& bounds = joints[joint_index]->getVariableBounds()[0];
			if (bounds.position_bounded_ && (after[joint_index] < bounds.min_position_ + this->tour_settings.path_check_joint_margin || after[joint_index] > bounds.max_position_ - this->tour_settings.path_check_joint_margin))
				return false;
		}
		return true;
	};
	Joints pose = from_pose;
	for (int step = 1; step < num_steps; ++step)
	{
		const double share = static_cast<double>(step) / static_cast<double>(num_steps);
		Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
		camera.translation() = (1.0 - share) * from_camera.translation() + share * to_camera.translation();
		camera.linear() = from_rotation.slerp(share, to_rotation).toRotationMatrix();
		state.setJointGroupPositions(this->group, pose);
		state.update();
		const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(camera, state.getGlobalLinkTransform(this->tool), this->ik_settings.exact_options.ignore_roll);
		const Eigen::VectorXd joint_step = scene_geometry::DampedPinv(this->getToolJacobian(state, pose), this->ik_settings.jacobian_damping) * pose_error;
		Joints next_pose = pose;
		for (size_t joint_index = 0; joint_index < next_pose.size(); ++joint_index)
			next_pose[joint_index] += joint_step(static_cast<Eigen::Index>(joint_index));
		if (!is_joint_step_ok(pose, next_pose) || !scene_geometry::IsStateValid(this->scene, &state, this->group, next_pose.data()))
			return remember(false, pose);
		pose = next_pose;
	}
	const closest_ik::Result arrival = closest_ik::Solve(this->scene, state, this->group, this->tool, to_camera, pose, this->ik_settings.exact_options);
	if (!arrival.exact || !is_joint_step_ok(pose, arrival.q))
		return remember(false, pose);
	return remember(true, arrival.q);
}

// Each missed viewpoint is also solved from its tour neighbours' poses (exact IK, else closest IK) and takes the result if
// its gap is smaller: the miss comes closer in a neighbour's posture.
void Optimizer::bringMissesCloserFromNeighbors(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	std::vector<size_t> miss_positions;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (!viewpoint_ik.reached[static_cast<size_t>(order[tour_position])])
			miss_positions.push_back(tour_position);
	if (miss_positions.empty())
		return;
	const auto since = std::chrono::steady_clock::now();
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	if (!(this->neighbor_ik_cache_position == viewpoint_ik.position))
	{
		this->neighbor_ik_cache.clear();
		this->neighbor_ik_cache_position = viewpoint_ik.position;
	}
	std::vector<ClosestIkCandidate> closer_poses(miss_positions.size());
	std::vector<char> found(miss_positions.size(), 0);  // one byte per miss: written from parallel threads
	const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(miss_positions.size()));
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < num_workers; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t miss_index = thread_index; miss_index < miss_positions.size(); miss_index += num_workers)
			{
				const size_t tour_position = miss_positions[miss_index];
				const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
				const Eigen::Isometry3d target = scene_geometry::Translation(viewpoint_ik.position) * this->viewpoints[viewpoint_index];
				std::vector<Joints> neighbor_poses = {tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q};
				if (tour_position + 1 < order.size())
					neighbor_poses.push_back(viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position + 1])].q);
				double best_gap = viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
				for (const Joints& neighbor_pose : neighbor_poses)
				{
					// The solve depends only on (viewpoint, start pose) at this object position, so a repeat is looked up.
					const std::pair<size_t, Joints> cache_key(viewpoint_index, neighbor_pose);
					NeighborIkResult neighbor_result;
					bool cached = false;
					{
						std::lock_guard<std::mutex> lock(this->neighbor_ik_cache_mutex);
						const auto entry = this->neighbor_ik_cache.find(cache_key);
						if (entry != this->neighbor_ik_cache.end())
						{
							neighbor_result = entry->second;
							cached = true;
						}
					}
					if (!cached)
					{
						closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, neighbor_pose, this->ik_settings.exact_options);
						if (!result.exact)
							result = this->solveSmallestGapIk(thread_state, target, {neighbor_pose});
						neighbor_result.found = result.found;
						neighbor_result.exact = result.exact;
						neighbor_result.candidate.q = result.q;
						if (result.found && !result.exact)
						{
							neighbor_result.candidate.clearance_shortfall = this->getClearanceShortfall(thread_state, result.q);
							neighbor_result.candidate.gap = scene_geometry::PoseGap(result.pose_error, this->ik_settings.closest_options.rot_scale) + neighbor_result.candidate.clearance_shortfall;
							neighbor_result.candidate.position_error = result.pose_error.head<3>();
							neighbor_result.candidate.angle_error = result.pose_error.tail<3>().norm();
						}
						std::lock_guard<std::mutex> lock(this->neighbor_ik_cache_mutex);
						this->neighbor_ik_cache[cache_key] = neighbor_result;
					}
					if (!neighbor_result.found)
						continue;
					if (neighbor_result.candidate.gap >= best_gap)
						continue;
					best_gap = neighbor_result.candidate.gap;
					closer_poses[miss_index] = neighbor_result.candidate;
					found[miss_index] = 1;
					if (neighbor_result.exact)
						break;
				}
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	for (size_t miss_index = 0; miss_index < miss_positions.size(); ++miss_index)
	{
		if (!found[miss_index])
			continue;
		const size_t viewpoint_index = static_cast<size_t>(order[miss_positions[miss_index]]);
		viewpoint_ik.gap_sum += closer_poses[miss_index].gap - viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].gap;
		viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index] = closer_poses[miss_index];
		if (closer_poses[miss_index].gap <= 0.0)
		{
			viewpoint_ik.reached[viewpoint_index] = true;
			viewpoint_ik.num_reached += 1;
		}
	}
	this->neighbor_ik_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

// Tour positions whose edge (from the previous pose, or home) the arm can't follow on the straight camera path, each edge
// from its start pose as it is now, in parallel; arrival_poses (if given) gets each followed edge's arrival pose.
std::vector<size_t> Optimizer::getReconfigurationEdges(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order, std::vector<Joints>* arrival_poses)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(viewpoint_ik.position, this->object_rotation));
	if (!(this->straight_path_cache_position == viewpoint_ik.position))
	{
		this->straight_path_cache.clear();
		this->straight_path_cache_position = viewpoint_ik.position;
	}
	std::vector<char> followed(order.size(), 0);  // one byte per edge: written from parallel threads
	std::vector<Joints> arrivals(order.size());
	const unsigned int num_workers = std::max(1u, std::min(this->num_threads, static_cast<unsigned int>(order.size())));
	std::vector<std::thread> threads;
	for (unsigned int thread_index = 0; thread_index < num_workers; ++thread_index)
		threads.emplace_back([&, thread_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t tour_position = thread_index; tour_position < order.size(); tour_position += num_workers)
			{
				const Joints& from_pose = tour_position == 0 ? this->home_pose : viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q;
				followed[tour_position] = this->followStraightCameraPath(thread_state, from_pose, viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position])].q, arrivals[tour_position]) ? 1 : 0;
			}
		});
	for (std::thread& thread : threads)
		thread.join();
	std::vector<size_t> reconfiguration_edges;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (!followed[tour_position])
			reconfiguration_edges.push_back(tour_position);
	if (arrival_poses)
		*arrival_poses = arrivals;
	return reconfiguration_edges;
}

// Walks the tour: each reached viewpoint whose edge the arm can follow takes the pose it arrives in. Edges are first
// followed in parallel from the poses as they are; an edge whose start pose then changed is followed again. Returns the
// number of reconfigurations.
int Optimizer::followTourAlongStraightCameraPaths(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order)
{
	const double same_pose_tolerance = 1e-3;  // rad: an arrival this near the pose in use leaves it as it is
	std::vector<Joints> arrival_poses;
	const std::vector<size_t> reconfiguration_edges = this->getReconfigurationEdges(viewpoint_ik, order, &arrival_poses);
	std::vector<char> followed(order.size(), 1);
	for (size_t tour_position : reconfiguration_edges)
		followed[tour_position] = 0;
	moveit::core::RobotState path_state(*this->state);
	int num_reconfigurations = 0;
	bool start_pose_changed = false;  // the previous viewpoint took an arrival pose, so this edge starts elsewhere
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
	{
		const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
		Joints& pose = viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q;
		if (start_pose_changed)
			followed[tour_position] = this->followStraightCameraPath(path_state, viewpoint_ik.chosen_ik_per_viewpoint[static_cast<size_t>(order[tour_position - 1])].q, pose, arrival_poses[tour_position]) ? 1 : 0;
		start_pose_changed = false;
		if (!followed[tour_position])
		{
			++num_reconfigurations;
			continue;
		}
		if (!viewpoint_ik.reached[viewpoint_index])
			continue;
		double largest_change = 0.0;
		for (size_t joint_index = 0; joint_index < pose.size(); ++joint_index)
			largest_change = std::max(largest_change, std::abs(arrival_poses[tour_position][joint_index] - pose[joint_index]));
		if (largest_change <= same_pose_tolerance)
			continue;
		pose = arrival_poses[tour_position];
		start_pose_changed = true;
	}
	return num_reconfigurations;
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

void Optimizer::logTourSolution(const std::string& label, const TourSolutionAtObjectPosition& solution)
{
	RCLCPP_INFO(this->node->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%zu  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f", label.c_str(), solution.viewpoint_ik.position.x(), solution.viewpoint_ik.position.y(), solution.viewpoint_ik.position.z(), solution.viewpoint_ik.num_reached, this->viewpoints.size(), solution.viewpoint_ik.gap_sum, solution.num_reconfigurations, solution.num_blocked_edges, solution.joint_travel_cost, solution.smallest_margin, solution.weighted_total_cost);
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
	if (this->show_tour_lines && tool_path.points.size() > 1)
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
