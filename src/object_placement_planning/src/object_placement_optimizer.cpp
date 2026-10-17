#include <cmath>
#include <atomic>
#include <thread>
#include <cstdio>
#include <numeric>
#include <algorithm>
#include <functional>

#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/ik_sampling.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/tour_placement.hpp"
#include "object_placement_planning/manipulability.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/object_placement_optimizer.hpp"

namespace object_placement
{

// ---------------------------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------------------------

Position ClampToBounds(Position p, const ObjectPlacementBounds& b)
{
	p.x() = std::clamp(p.x(), b.x_min, b.x_max);
	p.y() = std::clamp(p.y(), b.y_min, b.y_max);
	p.z() = std::clamp(p.z(), b.z_min, b.z_max);
	return p;
}

std::vector<int> GetLostViewpoints(const TourSolution& from, const TourSolution& to)
{
	std::vector<int> lost;
	for (int v : from.viewpoint_order)
		if (std::find(to.viewpoint_order.begin(), to.viewpoint_order.end(), v) == to.viewpoint_order.end())
			lost.push_back(v);
	return lost;
}

std::vector<Joints> GetIkStartPoses(const TourSolution& sol, const Joints& fallback, int num_viewpoint)
{
	std::vector<Joints> seeds(static_cast<size_t>(num_viewpoint), fallback);
	for (size_t k = 0; k < sol.viewpoint_order.size(); ++k)
		if (!sol.ik_poses[k].empty())
			seeds[static_cast<size_t>(sol.viewpoint_order[k])] = sol.ik_poses[k];
	return seeds;
}

// Take function and run in parallel
void RunInParallel(const moveit::core::RobotState& state_template, size_t count, uint32_t base_seed, const std::function<void(size_t, moveit::core::RobotState&, random_numbers::RandomNumberGenerator&)>& work)
{
	std::atomic<size_t> next_index{0};
	auto worker = [&]()
	{
		moveit::core::RobotState state(state_template);
		for (size_t i = next_index.fetch_add(1); i < count; i = next_index.fetch_add(1))
		{
			random_numbers::RandomNumberGenerator rng(static_cast<boost::uint32_t>(base_seed + 7919u * static_cast<uint32_t>(i)));
			work(i, state, rng);
		}
	};
	const size_t num_threads = std::min<size_t>(std::max(1u, std::thread::hardware_concurrency()), count);
	std::vector<std::thread> threads;
	for (size_t t = 1; t < num_threads; ++t)
		threads.emplace_back(worker);
	worker();
	for (std::thread& thread : threads)
		thread.join();
}

std::vector<Joints> PrependHomePose(const Joints& home, const std::vector<Joints>& tour)
{
	std::vector<Joints> path{home};
	path.insert(path.end(), tour.begin(), tour.end());
	return path;
}

// Joint distance from home through the poses in order.
double GetJointDistanceFromHome(const Joints& home, const std::vector<Joints>& poses, const std::vector<int>& joints)
{
	double distance = 0.0;
	for (size_t k = 0; k < poses.size(); ++k)
		distance += joint_travel::JointDistance(k == 0 ? home : poses[k - 1], poses[k], joints);
	return distance;
}

// The target's pose error if the tour misses it, else null.
const scene_geometry::Vector6d* FindMissedPoseError(const TourSolution& tour, int viewpoint_index)
{
	for (const MissedViewpoint& m : tour.missed_viewpoints)
		if (m.viewpoint_index == viewpoint_index)
			return &m.pose_error;
	return nullptr;
}

// ---------------------------------------------------------------------------------------------
// Setup and the run
// ---------------------------------------------------------------------------------------------

ObjectPlacementOptimizer::ObjectPlacementOptimizer(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const std::string& group_name,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& viewpoints_nominal, const Joints& home, const ObjectPlacementParams& params)
	: group_name(group_name), node(node), params(params), robot_model(robot_model), scene(scene),
	  object_pose(scene_geometry::MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal)),
	  object_rotation_nominal(object_rotation_nominal), object_translation_nominal(object_translation_nominal),
	  num_viewpoint(static_cast<int>(viewpoints_nominal.size())),
	  input_order(static_cast<size_t>(viewpoints_nominal.size())), robot_state(robot_model),
	  viewpoints(viewpoints_nominal), home_pose(home),
	  counted_joint_indices(joint_travel::AllJoints(static_cast<int>(home.size()) - 1))
{
	ik_sampling::ReseedRng(this->params.random_seed);

	for (Eigen::Isometry3d& t : this->viewpoints)
		t.translation() -= this->object_translation_nominal;

	std::iota(this->input_order.begin(), this->input_order.end(), 0);
	this->robot_state.setToDefaultValues();
	this->joint_group = this->robot_state.getJointModelGroup(this->group_name);
	this->tool_link = this->robot_model->getLinkModel("tool0");
	this->state_validity = [this](moveit::core::RobotState* s, const moveit::core::JointModelGroup* g, const double* q) {
		return scene_geometry::IsStateValid(this->scene, s, g, q);
	};
	if (this->params.ik_solver != "kdl" && this->params.ik_solver != "closest")
		RCLCPP_WARN(this->node->get_logger(), "unknown ik_solver '%s' -- using kdl", this->params.ik_solver.c_str());
	if (this->params.ignore_camera_roll && this->params.ik_solver != "closest")
		RCLCPP_WARN(this->node->get_logger(), "ignore_camera_roll needs ik_solver: closest -- roll stays fixed");
	this->can_plan = this->params.real_cost_planning_time > 0.0;
	this->use_planned_travel_in_cost = this->params.planned_travel_in_cost && this->can_plan;

	this->robot_state.setJointGroupPositions(this->joint_group, this->home_pose);
	this->robot_state.update();
	this->run_result.num_total = this->num_viewpoint;
}

ObjectPlacementResult ObjectPlacementOptimizer::Optimize()
{
	const Position start(this->params.initial_x, this->params.initial_y, this->params.initial_z);
	const ReachAttemptResult best_result = this->ReachAllViewpoints(start); // Try to reach all viewpoints

	const TourSolution& final_tour = best_result.tour_solution;
	this->run_result.x = best_result.position.x();
	this->run_result.y = best_result.position.y();
	this->run_result.z = best_result.position.z();
	this->run_result.tour_order = final_tour.viewpoint_order;
	this->run_result.joint_solutions = final_tour.ik_poses;
	this->run_result.total_weighted_cost = final_tour.weighted_cost_sum - this->params.unreachable_penalty * (this->num_viewpoint - final_tour.num_reachable);
	for (size_t k = 0; k < final_tour.ik_poses.size(); ++k)
		this->run_result.total_joint_path_length += joint_travel::JointDistance(k == 0 ? this->home_pose : final_tour.ik_poses[k - 1], final_tour.ik_poses[k], this->counted_joint_indices);
	this->run_result.num_reachable = final_tour.num_reachable;
	this->run_result.ok = best_result.all_reachable;

	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(this->object_translation_nominal, this->object_rotation_nominal));
	if (this->run_result.ok)
		RCLCPP_INFO(
			this->node->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned travel %.2f rad.",
			this->run_result.x, this->run_result.y, this->run_result.z, this->num_viewpoint, final_tour.planned_cost);
	else
		RCLCPP_WARN(
			this->node->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses.",
			this->run_result.x, this->run_result.y, this->run_result.z, this->run_result.num_reachable,
			this->num_viewpoint);
	return this->run_result;
}

ReachAttemptResult ObjectPlacementOptimizer::ReachAllViewpoints(const Position& from)
{
	const int num_tries = std::max(1, this->params.descent_num_restart);
	ReachAttemptResult best_attempt;
	for (int r = 0; r < num_tries && rclcpp::ok(); ++r)
	{	
		// First run start from input position. Later run at a manipulability jump from the best attempt so far
		const Position start = r == 0 ? from : this->RunManipulabilityJump(best_attempt.position, best_attempt.tour_solution, r);
		if (this->ResetAndSolveAtStart(start, r, r == 0 ? nullptr : &best_attempt.tour_solution))
		{
			// Kinematic moves while any viewpoint is missed.
			int moves_used = 0;
			int no_gain = 0;  // Moves since a viewpoint was last gained
			for (; moves_used < this->params.max_kinematic_moves && !this->current_tour.all_reachable && rclcpp::ok(); ++moves_used)
			{
				if (no_gain >= this->params.reach_patience)
				{
					RCLCPP_WARN(
						this->node->get_logger(), "  reach attempt %d: no viewpoint gained for reach_patience iterations with %d/%d reachable -- stuck", r + 1,
						this->current_tour.num_reachable, this->num_viewpoint);
					break;
				}
				const int reached = this->current_tour.num_reachable;
				if (!this->TryReachViewpoints(moves_used))
				{
					++moves_used;
					break;
				}
				no_gain = this->current_tour.num_reachable > reached ? 0 : no_gain + 1;
			}
			if (this->current_tour.all_reachable) // After all viewpoints are reached, keep moving to lower travel cost
				this->RefineGtspAndPlacement(moves_used);
		}
		const ReachAttemptResult d = this->best_in_attempt;

		// Improved is true if it's 1) first attempt or 2) reaches more viewpoints or 3) same viewpoints but lower cost
		const bool improved = r == 0 || d.tour_solution.num_reachable > best_attempt.tour_solution.num_reachable ||
			(d.tour_solution.num_reachable == best_attempt.tour_solution.num_reachable &&
			 d.weighted_cost_sum < best_attempt.weighted_cost_sum);
		RCLCPP_INFO(
			this->node->get_logger(),
			"reach attempt %d/%d done: total cost=%.4f  joint dist + largest turn + miss=%.2f  sum_w=%.3f  reachable %d/%d  abs (%.4f, "
			"%.4f, %.4f) m%s",
			r + 1, num_tries, d.weighted_cost_sum,
			d.tour_solution.joint_distance_plus_turn +
				this->params.unreachable_penalty * (this->num_viewpoint - d.tour_solution.num_reachable),
			d.tour_solution.manipulability_sum, d.tour_solution.num_reachable, this->num_viewpoint, d.position.x(),
			d.position.y(), d.position.z(), (r > 0 && improved) ? "  <-- new best" : "");
		if (improved)
			best_attempt = d;
		if (best_attempt.all_reachable)
			break;
	}
	return best_attempt;
}

bool ObjectPlacementOptimizer::ResetAndSolveAtStart(const Position& start, int restart_in, const TourSolution* warm)
{
	// Resets the per-try state (seed, IK starts, last poses, deadline, best) and runs the first full solve at `start`.
	// Return false if nothing is reachable there.
	this->attempt_index = restart_in;
	this->attempt_seed = this->params.random_seed + this->attempt_index;  // each restart draws its own IK luck
	this->current_position = ClampToBounds(start, this->params.bounds);
	this->ik_start_poses = warm ? GetIkStartPoses(*warm, this->home_pose, this->num_viewpoint) : std::vector<Joints>(static_cast<size_t>(this->num_viewpoint), this->home_pose);
	this->stall_count = 0;
	this->last_reached_poses.assign(static_cast<size_t>(this->num_viewpoint), {});
	this->last_closest_poses.assign(static_cast<size_t>(this->num_viewpoint), {});
	this->quick_solve_start_from_current_ik = true;
	this->logged_gtsp_restarts_at_first_reach = false;
	this->best_in_attempt = ReachAttemptResult{};
	this->best_in_attempt.position = this->current_position;

	ik_sampling::ReseedRng(this->attempt_seed);
	const std::vector<int>* start_lock = this->params.lock_input_order ? &this->input_order : nullptr;
	TourSolution first = this->EvaluateObjectPosition(this->current_position, this->params, this->ik_start_poses, warm ? &warm->viewpoint_order : nullptr, this->params.max_solutions_per_candidate, start_lock);
	this->run_result.num_inner_solves += 1;
	this->current_tour = this->RunGtspRestartsAndEvaluate(this->current_position, std::move(first), this->params.gtsp_num_restart - 1, false, start_lock);
	this->current_tour = this->AddPlannedTravelToCost(this->current_position, std::move(this->current_tour));
	if (this->current_tour.viewpoint_order.empty())
	{
		RCLCPP_WARN(
			this->node->get_logger(), "reach attempt %d: no tour pose reachable at the start placement",
			this->attempt_index + 1);
		return false;
	}
	this->RecordHistoryAndBest(this->current_position, this->current_tour);
	this->PublishProgressMarkers(Eigen::Vector3d::Zero());
	RCLCPP_INFO(
		this->node->get_logger(), "reach attempt %d/%d start: reachable %d/%d", this->attempt_index + 1, std::max(1, this->params.descent_num_restart),
		this->current_tour.num_reachable, this->num_viewpoint);
	return true;
}

// Once all are reached: kinematic moves to lower the cost (unless stop_when_all_reached), then rounds of GTSP restarts + tour-placement. `iter`: moves already used.
void ObjectPlacementOptimizer::RefineGtspAndPlacement(int iter)
{
	this->LogGtspRestartsAtFirstReach();
	this->quick_solve_start_from_current_ik = false;  // a dragged-along tour looks worse than the spot really is
	if (this->params.stop_when_all_reached)
		RCLCPP_INFO(
			this->node->get_logger(), "  reach attempt %d: all viewpoints reached -- stopping",
			this->attempt_index + 1);
	else
		for (; iter < this->params.max_kinematic_moves && rclcpp::ok(); ++iter)
			if (!this->TryReachViewpoints(iter))
				break;
	for (int round = 1; round <= std::max(1, this->params.tour_placement_rounds) && rclcpp::ok(); ++round)
	{
		RCLCPP_INFO(this->node->get_logger(), "  round %d/%d: GTSP restarts, then tour-placement", round, std::max(1, this->params.tour_placement_rounds));
		this->RestartGtsp();
		this->OptimizeTourAndPlacement();
	}
}

// One move of the object: gradient line search, else TryKinematicRoomMove (room-then-reach) while viewpoints are
// missed, then a full solve at the new spot. False when no move helps (converged or stuck).
bool ObjectPlacementOptimizer::TryReachViewpoints(int iter)
{
	this->move_start = std::chrono::steady_clock::now();
	this->ik_seconds = 0.0;
	this->closest_ik_seconds = 0.0;
	const Eigen::Vector3d g = this->ComputeCostGradient(this->current_tour, this->params);
	const double gnorm = g.norm();
	if (gnorm < 1e-6)
	{
		RCLCPP_INFO(this->node->get_logger(), "  reach attempt %d: gradient ~ 0 -- converged", this->attempt_index + 1);
		this->PublishProgressMarkers(-g);
		return false;
	}
	this->PrepareReachViewpoints();
	std::optional<MoveCandidate> step = this->LineSearch(-g / gnorm);
	if (!step && this->params.room_then_reach && !this->quick_solve_tour.missed_viewpoints.empty())
		step = this->TryKinematicRoomMove();
	std::optional<TourSolution> committed;
	if (step)
		committed = this->ConfirmImprovementWithFullSolve(*step);
	if (!committed)
	{
		if (this->current_tour.all_reachable)
			RCLCPP_INFO(this->node->get_logger(), "  converged");
		else
			RCLCPP_WARN(
				this->node->get_logger(), "  reach attempt %d: no step reduces the tour cost with %d/%d reachable -- stuck",
				this->attempt_index + 1, this->current_tour.num_reachable, this->num_viewpoint);
		this->PublishProgressMarkers(-g);
		return false;
	}
	return !this->CommitMoveAndCheckConvergence(*step, std::move(*committed), iter, g);
}

// Where IK starts per viewpoint, whether the order is fixed, and the baseline quick solve a try must beat.
void ObjectPlacementOptimizer::PrepareReachViewpoints()
{
	// Save each viewpoint's latest pose so its IK keeps starting from the same arm pose shape.
	for (size_t k = 0; k < this->current_tour.viewpoint_order.size(); ++k)
	{
		const size_t viewpoint_index = static_cast<size_t>(this->current_tour.viewpoint_order[k]);
		this->last_reached_poses[viewpoint_index] = this->current_tour.ik_poses[k];
		this->last_closest_poses[viewpoint_index].clear();
	}
	this->ik_start_poses = GetIkStartPoses(this->current_tour, this->home_pose, this->num_viewpoint);
	for (const MissedViewpoint& m : this->current_tour.missed_viewpoints)
	{
		const size_t v = static_cast<size_t>(m.viewpoint_index);
		if (!m.closest_pose.empty())
			this->last_closest_poses[v] = m.closest_pose;
		if (!this->last_closest_poses[v].empty())
			this->ik_start_poses[v] = this->last_closest_poses[v];
		else if (!this->last_reached_poses[v].empty())
			this->ik_start_poses[v] = this->last_reached_poses[v];
	}
	this->locked_order = this->current_tour.viewpoint_order;
	this->fixed_order = this->params.lock_input_order ? &this->input_order
		: (this->params.freeze_order && this->current_tour.all_reachable) ? &this->locked_order
														 : nullptr;
	this->quick_solve_tour = this->AddPlannedTravelToCost(this->current_position, this->QuickSolveTour(this->current_position, this->params));
	this->run_result.num_inner_solves += 1;
}

// Backtracking along `dir`: shrink the step until a probe beats the quick solve here (same IK effort, so a viewpoint
// the quick solve merely failed to find doesn't block the step).
std::optional<MoveCandidate> ObjectPlacementOptimizer::LineSearch(const Eigen::Vector3d& dir)
{
	ObjectPlacementParams probe_params = this->params;
	probe_params.gaps_min_reached = this->quick_solve_tour.num_reachable;  // a probe reaching fewer is rejected anyway, so it skips miss gaps
	double step = this->params.initial_step;
	for (int ls = 0; ls < this->params.max_line_search_iters; ++ls)
	{
		const Position cand = ClampToBounds(this->current_position + step * dir, this->params.bounds);
		TourSolution probe = this->QuickSolveTour(cand, probe_params);
		this->run_result.num_inner_solves += 1;
		if (probe.num_reachable >= this->quick_solve_tour.num_reachable)
			probe = this->AddPlannedTravelToCost(cand, std::move(probe));	// plan only probes that could be accepted
		if (this->params.trace_line_search)
			RCLCPP_INFO(
				this->node->get_logger(),
				"    probe step=%.4f m abs (%.4f, %.4f, %.4f): reached %d vs %d here, cost %.2f vs %.2f here "
				"(sum_w %.3f vs %.3f, planned %.2f vs %.2f)",
				step, cand.x(), cand.y(), cand.z(), probe.num_reachable, this->quick_solve_tour.num_reachable,
				probe.weighted_cost_sum, this->quick_solve_tour.weighted_cost_sum, probe.manipulability_sum,
				this->quick_solve_tour.manipulability_sum, probe.planned_cost, this->quick_solve_tour.planned_cost);
		if (this->HasMoreReachedOrLowerCost(probe, this->quick_solve_tour))
			return MoveCandidate{cand, step, std::move(probe)};
		step *= this->params.step_shrink;
	}
	return std::nullopt;
}

// Logs one room-then-reach candidate move when trace_line_search is on.
void ObjectPlacementOptimizer::TraceRoomMove(const std::string& what, int target, const Position& cand, const TourSolution& candidate_tour, const std::string& verdict)
{
	if (this->params.trace_line_search)
		RCLCPP_INFO(this->node->get_logger(), "    %s for vp %d abs (%.4f, %.4f, %.4f): reached %d vs %d here -> %s", what.c_str(), target, cand.x(), cand.y(), cand.z(), candidate_tour.num_reachable, this->quick_solve_tour.num_reachable, verdict.c_str());
}

// Room then reach. For each missed viewpoint (smallest gap first), aim a full step at it: reached with nothing lost is
// accepted, nothing lost walks on. If reached viewpoints are lost, back up instead along the move that most raises the
// margins that ran out for them (kept off the other viewpoints' limits), then aim again.
std::optional<MoveCandidate> ObjectPlacementOptimizer::TryKinematicRoomMove()
{
	const int kMaxRounds = 6;
	const int kMaxReplans = 3;
	const int kLostTries = 10;	   // other-IK candidates for a lost viewpoint
	const int kSqueezedTries = 6;  // and for one the room move would squeeze
	const double step = this->params.initial_step;
	std::vector<std::optional<Position>> searched_at(static_cast<size_t>(this->num_viewpoint));
	std::vector<MissedViewpoint> targets = this->quick_solve_tour.missed_viewpoints;
	std::sort(targets.begin(), targets.end(),
		[&](const MissedViewpoint& a, const MissedViewpoint& b)
		{
			return scene_geometry::PoseGap(a.pose_error, this->params.rot_metric_scale) <
				scene_geometry::PoseGap(b.pose_error, this->params.rot_metric_scale);
		});
	if (this->params.room_max_targets > 0 && static_cast<int>(targets.size()) > this->params.room_max_targets)
		targets.resize(static_cast<size_t>(this->params.room_max_targets));  // the far ones rarely succeed
		
	for (const MissedViewpoint& miss : targets)
	{
		const int target = miss.viewpoint_index;
		Position at = this->current_position;
		TourSolution here_at = this->quick_solve_tour;
		for (int round = 0; round < kMaxRounds; ++round)
		{
			// Aim: a full step at the target.
			const Eigen::Vector3d aim = this->GetMoveDirectionToCloseGap(target, here_at, round == 0);
			if (aim.norm() < 1e-9)
				break;
			const Position cand = ClampToBounds(at + step * aim, this->params.bounds);
			TourSolution candidate_tour = this->TestCandidateMove(here_at, cand);
			const std::vector<int> lost = GetLostViewpoints(here_at, candidate_tour);
			if (GetLostViewpoints(this->quick_solve_tour, candidate_tour).empty() && candidate_tour.num_reachable > this->quick_solve_tour.num_reachable)
			{
				this->TraceRoomMove("aim", target, cand, candidate_tour, "accepted");
				return MoveCandidate{cand, step, this->AddPlannedTravelToCost(cand, std::move(candidate_tour))};
			}
			if (lost.empty())
			{
				this->TraceRoomMove("aim", target, cand, candidate_tour, "walk on");
				at = cand;
				here_at = std::move(candidate_tour);
				continue;
			}

			// Back up. A viewpoint near a limit first takes its roomiest other IK solution here (once per spot).
			room_margins::ArmModel model = this->BuildRoomMarginArmModel(at, here_at);
			auto swap_pose = [&](int v, int tries) {
				std::optional<Position>& last = searched_at[static_cast<size_t>(v)];
				if (last && *last == at)
					return false;
				last = at;
				const std::optional<Joints> roomier = this->FindRoomierPose(v, at, model, tries);
				if (!roomier)
					return false;
				const auto it = std::find(here_at.viewpoint_order.begin(), here_at.viewpoint_order.end(), v);
				here_at.ik_poses[static_cast<size_t>(it - here_at.viewpoint_order.begin())] = *roomier;
				return true;
			};
			std::vector<room_margins::Margin> short_margins;
			std::vector<bool> joined(static_cast<size_t>(this->num_viewpoint), false);
			// Adds what ran out for viewpoint v on the move to `to`.
			auto add_shortage = [&](int v, const Position& to) {
				const room_margins::Shortage sh = this->GetMarginsThatRunOut(v, model, at, to - at);
				short_margins.insert(short_margins.end(), sh.margins.begin(), sh.margins.end());
				joined[static_cast<size_t>(v)] = true;
				return " vp " + std::to_string(v) + ": " + sh.why + ";";
			};
			bool swapped = false;
			for (int v : lost)
				swapped = swap_pose(v, kLostTries) || swapped;
			if (swapped)
				model = this->BuildRoomMarginArmModel(at, here_at);
			std::string why;
			for (int v : lost)
				why += add_shortage(v, cand);
			this->TraceRoomMove("aim", target, cand, candidate_tour, "back up (" + why + " )");
			Eigen::Vector3d room = room_margins::MostRoomDirection(short_margins, step);
			// Viewpoints the move would run out join in, after the same pose swap.
			for (int pass = 0; pass < 3 && room.norm() > 1e-9; ++pass)
			{
				std::vector<int> squeezed;
				for (const room_margins::Margin& m : model.margins)
					if (!joined[static_cast<size_t>(m.viewpoint_index)] && room_margins::RoomAlong({m}, room) < step &&
						std::find(squeezed.begin(), squeezed.end(), m.viewpoint_index) == squeezed.end())
						squeezed.push_back(m.viewpoint_index);
				if (squeezed.empty())
					break;
				bool swapped_any = false;
				for (int v : squeezed)
					swapped_any = swap_pose(v, kSqueezedTries) || swapped_any;
				if (swapped_any)
					model = this->BuildRoomMarginArmModel(at, here_at);
				for (int v : squeezed)
				{
					const std::vector<room_margins::Margin> mine = room_margins::MarginsOf(model, v);
					if (room_margins::RoomAlong(mine, room) >= step)
						continue;  // its new pose has room for the move
					for (const room_margins::Margin& m : mine)
						if (room_margins::RoomAlong({m}, room) < step)
							short_margins.push_back(m);
					joined[static_cast<size_t>(v)] = true;
				}
				room = room_margins::MostRoomDirection(short_margins, step);
			}
			if (room.norm() < 1e-9)
				break;

			// Room move: a full step; one that loses a viewpoint adds what ran out for it and is re-planned, last
			// half a step.
			bool moved = false;
			for (int plan = 0; plan <= kMaxReplans && !moved && room.norm() > 1e-9; ++plan)
			{
				const double size = plan < kMaxReplans ? step : 0.5 * step;
				const Position back = ClampToBounds(at + size * room, this->params.bounds);
				TourSolution room_move_tour = this->TestCandidateMove(here_at, back);
				const std::vector<int> back_lost = GetLostViewpoints(here_at, room_move_tour);
				char what[64];
				std::snprintf(what, sizeof(what), "room (%+.2f, %+.2f, %+.2f) x %.3f m", room(0), room(1), room(2), size);
				if (GetLostViewpoints(this->quick_solve_tour, room_move_tour).empty() && room_move_tour.num_reachable > this->quick_solve_tour.num_reachable)
				{
					this->TraceRoomMove(what, target, back, room_move_tour, "gained, accepted");
					return MoveCandidate{back, size, this->AddPlannedTravelToCost(back, std::move(room_move_tour))};
				}
				if (back_lost.empty())
				{
					this->TraceRoomMove(what, target, back, room_move_tour, "moved");
					at = back;
					here_at = std::move(room_move_tour);
					moved = true;
					break;
				}
				std::string room_why;
				for (int v : back_lost)
				{
					if (swap_pose(v, kLostTries))
						model = this->BuildRoomMarginArmModel(at, here_at);
					room_why += add_shortage(v, back);
				}
				this->TraceRoomMove(what, target, back, room_move_tour, "lost, add what ran out (" + room_why + " ) and replan");
				room = room_margins::MostRoomDirection(short_margins, step);
			}
			if (!moved)
				break;
		}
	}
	return std::nullopt;
}

// Full solve at the accepted spot (the probe's own solve stands if better); kept only if it beats the current one.
std::optional<TourSolution> ObjectPlacementOptimizer::ConfirmImprovementWithFullSolve(const MoveCandidate& step)
{
	const int num_restarts = step.tour_solution.all_reachable ? this->params.gtsp_num_restart - 1 : 0;  // the order hardly matters while viewpoints are missed
	ik_sampling::ReseedRng(this->attempt_seed);
	TourSolution first = this->EvaluateObjectPosition(step.next_position, this->params, this->ik_start_poses, &step.tour_solution.viewpoint_order, this->params.max_solutions_per_candidate, this->fixed_order);
	this->run_result.num_inner_solves += 1;
	TourSolution committed = this->AddPlannedTravelToCost(step.next_position, this->RunGtspRestartsAndEvaluate(step.next_position, std::move(first), num_restarts, false, this->fixed_order));
	if (this->HasMoreReachedOrLowerCost(step.tour_solution, committed))
		committed = step.tour_solution;
	const bool keep = this->HasMoreReachedOrLowerCost(committed, this->current_tour);
	if (this->params.trace_line_search)
		RCLCPP_INFO(
			this->node->get_logger(), "    full solve: reached %d vs %d current, cost %.2f vs %.2f current -> %s",
			committed.num_reachable, this->current_tour.num_reachable, committed.weighted_cost_sum,
			this->current_tour.weighted_cost_sum, keep ? "accepted" : "rejected");
	if (!keep)
		return std::nullopt;
	return committed;
}

// Moves to the accepted spot and logs it. Return true when the descent has converged.
bool ObjectPlacementOptimizer::CommitMoveAndCheckConvergence(const MoveCandidate& step, TourSolution committed, int iter, const Eigen::Vector3d& g)
{
	const double moved = (step.next_position - this->current_position).norm();
	const double rel_gain = (this->current_tour.weighted_cost_sum - committed.weighted_cost_sum) /
		std::max(std::abs(this->current_tour.weighted_cost_sum), 1e-9);
	const bool gained = committed.num_reachable > this->current_tour.num_reachable;  // progress even if the cost rose

	this->move_history.push_back({MoveType::KinematicRoomMove, this->current_position, step.next_position});
	this->current_position = step.next_position;
	this->current_tour = std::move(committed);
	this->RecordHistoryAndBest(this->current_position, this->current_tour);
	RCLCPP_INFO(
		this->node->get_logger(), "reach attempt %d/%d kinematic move %d/%d: reachable %d/%d  (%.1f s: IK %.1f s, closest IK %.1f s)", this->attempt_index + 1,
		std::max(1, this->params.descent_num_restart), iter + 1, this->params.max_kinematic_moves,
		this->current_tour.num_reachable, this->num_viewpoint, std::chrono::duration<double>(std::chrono::steady_clock::now() - this->move_start).count(), this->ik_seconds, this->closest_ik_seconds);
	this->PublishProgressMarkers(-g);

	if (gained || rel_gain >= this->params.convergence_tolerance_cost)
		this->stall_count = 0;
	else if (++this->stall_count >= std::max(1, this->params.patience))
	{
		RCLCPP_INFO(
			this->node->get_logger(), "  reach attempt %d: %d iterations with <%.1e relative gain -- stopping early",
			this->attempt_index + 1, this->stall_count, this->params.convergence_tolerance_cost);
		return true;
	}
	if (!gained && moved < this->params.convergence_tolerance_offset && rel_gain < this->params.convergence_tolerance_cost)
	{
		RCLCPP_INFO(this->node->get_logger(), "  reach attempt %d: placement settled -- converged", this->attempt_index + 1);
		return true;
	}
	return false;
}

// Manipulability jump score: sum of the reached poses' manipulability + jump_softmin_weight x soft-min.
double ObjectPlacementOptimizer::GetJumpScore(const TourSolution& tour) const
{
	return tour.manipulability_sum + this->params.jump_softmin_weight * tour.softmin_manipulability;
}

void ObjectPlacementOptimizer::LogJumpStep(int restart_in, int iter, double step)
{
	RCLCPP_INFO(this->node->get_logger(), "jump %d iter %d: abs (%.4f, %.4f, %.4f) m  reachable %d/%d  manipulability score %.4f (sum_w %.3f, soft-min %.4f)  step %.4f m", restart_in + 1, iter, this->current_position.x(), this->current_position.y(), this->current_position.z(), this->current_tour.num_reachable, this->num_viewpoint, this->GetJumpScore(this->current_tour), this->current_tour.manipulability_sum, this->current_tour.softmin_manipulability, step);
}

// Gradient ascent on the reached viewpoints' manipulability (sum + jump_softmin_weight x soft-min), whatever is
// reachable at each spot; misses and travel ignored. Backtracking line search; a step is kept when its full solve
// raises the score. Ends when no step does, or after `patience` steps of tiny gain.
Position ObjectPlacementOptimizer::RunManipulabilityJump(const Position& from, const TourSolution& warm, int restart_in)
{
	ObjectPlacementParams p = this->params;	// reached viewpoints' manipulability only; misses need no closest pose
	p.gaps_min_reached = std::numeric_limits<int>::max();
	p.max_solutions_per_candidate = 2;  // fewer IK poses per viewpoint: the jump only needs a manipulable one
	this->attempt_index = restart_in;
	this->attempt_seed = this->params.random_seed + this->attempt_index;
	this->current_position = from;
	this->quick_solve_start_from_current_ik = true;
	this->ik_start_poses = GetIkStartPoses(warm, this->home_pose, this->num_viewpoint);
	ik_sampling::ReseedRng(this->attempt_seed);
	this->current_tour = this->FindMostManipulablePoses(this->current_position, p);
	this->LogJumpStep(restart_in, 0, 0.0);
	this->stall_count = 0;
	for (int iter = 0; iter < this->params.jump_max_moves && rclcpp::ok(); ++iter)
	{
		const Eigen::Vector3d g = this->ComputeManipulabilityGradient(this->current_tour.ik_poses, 1.0, this->params.jump_softmin_weight);
		if (g.norm() < 1e-9)
			break;
		this->ik_start_poses = GetIkStartPoses(this->current_tour, this->home_pose, this->num_viewpoint);
		this->locked_order = this->current_tour.viewpoint_order;
		this->quick_solve_tour = this->QuickSolveTour(this->current_position, p);
		std::optional<MoveCandidate> step;
		double size = this->params.initial_step;
		for (int ls = 0; ls < this->params.max_line_search_iters && !step; ++ls, size *= this->params.step_shrink)
		{
			const Position cand = ClampToBounds(this->current_position - size * g.normalized(), this->params.bounds);
			TourSolution probe = this->QuickSolveTour(cand, p);
			this->run_result.num_inner_solves += 1;
			if (this->params.trace_line_search)
				RCLCPP_INFO(
					this->node->get_logger(), "    jump probe step=%.4f m: reached %d, score %.4f vs %.4f here", size,
					probe.num_reachable, this->GetJumpScore(probe), this->GetJumpScore(this->quick_solve_tour));
			if (this->GetJumpScore(probe) > this->GetJumpScore(this->quick_solve_tour))
				step = MoveCandidate{cand, size, std::move(probe)};
		}
		if (!step)
			break;
		// Fresh IK at the new spot can find more manipulable poses than the tracked probe; keep the better.
		ik_sampling::ReseedRng(this->attempt_seed);
		TourSolution committed = this->FindMostManipulablePoses(step->next_position, p);
		if (this->GetJumpScore(step->tour_solution) > this->GetJumpScore(committed))
			committed = step->tour_solution;
		if (this->GetJumpScore(committed) <= this->GetJumpScore(this->current_tour))
			break;
		const double gain =
			(this->GetJumpScore(committed) - this->GetJumpScore(this->current_tour)) / std::max(std::abs(this->GetJumpScore(this->current_tour)), 1e-9);
		this->move_history.push_back({MoveType::ManipulabilityJump, this->current_position, step->next_position});
		this->current_position = step->next_position;
		this->current_tour = std::move(committed);
		this->LogJumpStep(restart_in, iter + 1, step->step_length);
		this->PublishProgressMarkers(-g);
		if (gain >= this->params.convergence_tolerance_cost)
			this->stall_count = 0;
		else if (++this->stall_count >= std::max(1, this->params.patience))
			break;
	}
	RCLCPP_INFO(
		this->node->get_logger(), "jump %d: moved %.4f m from abs (%.4f, %.4f, %.4f) to (%.4f, %.4f, %.4f)",
		restart_in + 1, (this->current_position - from).norm(), from.x(), from.y(), from.z(), this->current_position.x(),
		this->current_position.y(), this->current_position.z());
	return this->current_position;
}

// GTSP restarts at the descent's result.
void ObjectPlacementOptimizer::RestartGtsp()
{
	if (this->params.gtsp_restarts <= 0 || this->best_in_attempt.tour_solution.viewpoint_order.empty())
		return;
	TourSolution start = this->best_in_attempt.tour_solution;
	if (this->can_plan && start.planned_cost < 0.0)
		start.planned_cost = this->ComputeRealPlannedTravelCost(this->best_in_attempt.position, start);
	const std::vector<int>* lock = this->params.lock_input_order ? &this->input_order : this->params.freeze_order ? &start.viewpoint_order : nullptr;
	this->best_in_attempt.tour_solution = this->RunGtspRestartsAndEvaluate(this->best_in_attempt.position, start, this->params.gtsp_restarts, true, lock);
	this->best_in_attempt.weighted_cost_sum = this->ComputeWeightedCost(this->best_in_attempt.tour_solution);
	this->best_in_attempt.all_reachable = this->best_in_attempt.tour_solution.all_reachable;
	const TourSolution& s = this->best_in_attempt.tour_solution;
	int reordered = 0;
	for (size_t k = 0; k < s.viewpoint_order.size(); ++k)
		reordered += (k >= start.viewpoint_order.size() || start.viewpoint_order[k] != s.viewpoint_order[k]) ? 1 : 0;
	RCLCPP_INFO(
		this->node->get_logger(),
		"  reach attempt %d: %d GTSP restarts at final abs (%.4f, %.4f, %.4f) m: joint dist + largest turn %.2f -> %.2f  planned travel %.2f -> "
		"%.2f  "
		"order changed at %d/%zu positions  reconfigurations %d -> %d",
		this->attempt_index + 1, this->params.gtsp_restarts, this->best_in_attempt.position.x(),
		this->best_in_attempt.position.y(), this->best_in_attempt.position.z(), start.joint_distance_plus_turn,
		s.joint_distance_plus_turn, start.planned_cost, s.planned_cost, reordered, s.viewpoint_order.size(),
		joint_travel::CountReconfigs(
			PrependHomePose(this->home_pose, start.ik_poses), this->params.reconfig_threshold, this->counted_joint_indices),
		joint_travel::CountReconfigs(
			PrependHomePose(this->home_pose, s.ik_poses), this->params.reconfig_threshold, this->counted_joint_indices));
}

// After GTSP restarts: with the viewpoint order and IK branches fixed, move the object and slide each tour pose along its
// self-motion to shorten the tour's joint distance (a QP per iteration on the linearized poses, then IK back onto the
// viewpoints). The result is kept if its planned travel is lower.
void ObjectPlacementOptimizer::OptimizeTourAndPlacement()
{
	if (this->params.tour_placement_iterations <= 0 || !this->best_in_attempt.all_reachable)
		return;
	const auto tour_placement_start = std::chrono::steady_clock::now();
	const std::vector<int>& order = this->best_in_attempt.tour_solution.viewpoint_order;
	tour_placement::StepLimits limits;
	for (const std::string& name : this->joint_group->getVariableNames())
	{
		limits.joint_min.push_back(this->robot_model->getVariableBounds(name).min_position_);
		limits.joint_max.push_back(this->robot_model->getVariableBounds(name).max_position_);
	}
	limits.joint_margin = this->params.tour_placement_joint_margin;
	limits.frozen_poses.assign(this->best_in_attempt.tour_solution.ik_poses.size(), false);
	double object_step = this->params.tour_placement_object_step;
	double self_motion_step = this->params.tour_placement_self_motion_step;

	Position position = this->best_in_attempt.position;
	std::vector<Joints> poses = this->best_in_attempt.tour_solution.ik_poses;
	double distance = GetJointDistanceFromHome(this->home_pose, poses, this->counted_joint_indices);
	int num_accepted = 0;
	for (int iter = 0; iter < this->params.tour_placement_iterations && object_step > 1e-4 && rclcpp::ok(); ++iter)
	{
		std::vector<tour_placement::LinearizedPose> linearized;
		for (const Joints& q : poses)
		{
			const Eigen::MatrixXd jacobian = manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q);
			linearized.push_back({q, Eigen::MatrixXd(scene_geometry::DampedPinv(jacobian, this->params.jacobian_damping).leftCols<3>()), tour_placement::GetSelfMotionDirection(jacobian)});
		}
		limits.max_object_step = object_step;
		limits.max_self_motion = self_motion_step;
		const tour_placement::Step step = tour_placement::SolveStep(this->home_pose, linearized, this->counted_joint_indices, limits);
		if (!step.ok)
		{
			RCLCPP_WARN(this->node->get_logger(), "  tour-placement %d: QP not solved -- stopping", iter + 1);
			break;
		}

		// IK-correct each predicted pose onto its viewpoint (IK seeded there); a pose that fails or jumps branch rejects the step.
		const Position next_position = ClampToBounds(position + step.object_move, this->params.bounds);
		scene_geometry::MoveObject(this->scene, scene_geometry::Translation(next_position) * this->object_pose);
		std::vector<Joints> next_poses;
		for (size_t k = 0; k < poses.size(); ++k)
		{
			const Eigen::VectorXd predicted = Eigen::Map<const Eigen::VectorXd>(poses[k].data(), static_cast<Eigen::Index>(poses[k].size())) + linearized[k].dq_dp * (next_position - position) + linearized[k].self_motion * step.self_motion_amounts[k];
			Joints seed(predicted.data(), predicted.data() + predicted.size());
			for (size_t j = 0; j < seed.size(); ++j)
				seed[j] = std::clamp(seed[j], limits.joint_min[j], limits.joint_max[j]);
			this->robot_state.setJointGroupPositions(this->joint_group, seed);
			const Eigen::Isometry3d target = scene_geometry::Translation(next_position) * this->viewpoints[static_cast<size_t>(order[k])];
			if (!this->SolveExactIk(this->robot_state, target, 1, this->params, ik_sampling::SeededRng()))
				break;
			Joints projected;
			this->robot_state.copyJointGroupPositions(this->joint_group, projected);
			if (joint_travel::JointDistance(projected, seed, joint_travel::AllJoints(static_cast<int>(seed.size()))) > 0.2)
				break;
			next_poses.push_back(projected);
		}
		const size_t failed_pose = next_poses.size();
		const double next_distance = failed_pose == poses.size() ? GetJointDistanceFromHome(this->home_pose, next_poses, this->counted_joint_indices) : std::numeric_limits<double>::infinity();
		if (next_distance >= distance)
		{
			scene_geometry::MoveObject(this->scene, scene_geometry::Translation(position) * this->object_pose);
			// A pose that failed IK correction keeps its self-motion from now on; retry at the same step size once per pose.
			if (failed_pose < poses.size() && !limits.frozen_poses[failed_pose])
			{
				limits.frozen_poses[failed_pose] = true;
				continue;
			}
			object_step *= 0.5;
			self_motion_step *= 0.5;
			continue;
		}
		position = next_position;
		poses = std::move(next_poses);
		distance = next_distance;
		++num_accepted;
		// Grow the step back after a success, up to the starting size.
		object_step = std::min(1.5 * object_step, this->params.tour_placement_object_step);
		self_motion_step = std::min(1.5 * self_motion_step, this->params.tour_placement_self_motion_step);
	}
	if (num_accepted == 0)
	{
		RCLCPP_INFO(this->node->get_logger(), "  tour-placement: no improvement");
		return;
	}

	TourSolution optimized = this->best_in_attempt.tour_solution;
	optimized.ik_poses = poses;
	optimized.joint_distance_plus_turn = joint_travel::TravelCost(PrependHomePose(this->home_pose, poses), this->params.edge_weights, this->counted_joint_indices);
	optimized.planned_cost = this->can_plan ? this->ComputeRealPlannedTravelCost(position, optimized) : -1.0;
	optimized.weighted_cost_sum = this->ComputeWeightedCost(optimized);
	const bool kept = optimized.weighted_cost_sum < this->best_in_attempt.weighted_cost_sum;
	RCLCPP_INFO(this->node->get_logger(), "  tour-placement (%.1f s): planned travel %.2f -> %.2f rad%s", std::chrono::duration<double>(std::chrono::steady_clock::now() - tour_placement_start).count(), this->best_in_attempt.tour_solution.planned_cost, optimized.planned_cost, kept ? "" : "  -- not kept");
	if (!kept)
		return;
	this->move_history.push_back({MoveType::KinematicRoomMove, this->best_in_attempt.position, position});
	this->best_in_attempt.position = position;
	this->best_in_attempt.tour_solution = std::move(optimized);
	this->best_in_attempt.weighted_cost_sum = this->best_in_attempt.tour_solution.weighted_cost_sum;
}

// ---------------------------------------------------------------------------------------------
// Solving and scoring
// ---------------------------------------------------------------------------------------------

// closest_ik settings from the params; with the kdl solver the closest pass stays as before (no exact stage, roll fixed).
closest_ik::Options ObjectPlacementOptimizer::GetClosestIkOptions(const ObjectPlacementParams& p, bool exact_only) const
{
	const bool use_closest_solver = p.ik_solver == "closest";
	closest_ik::Options options;
	options.iters = p.closest_ik_iters;
	options.margin = p.closest_ik_margin;
	options.rot_scale = p.rot_metric_scale;
	options.limit_sharpness = p.manipulability_limit_sharpness;
	options.ignore_roll = use_closest_solver && p.ignore_camera_roll;
	options.try_exact_first = use_closest_solver;
	options.exact_only = exact_only;
	options.exact_margin = p.exact_ik_margin;
	return options;
}

// Exact IK from the state's pose, then from random poses (attempts in all). On success the state holds the solution.
bool ObjectPlacementOptimizer::SolveExactIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, int attempts, const ObjectPlacementParams& p, random_numbers::RandomNumberGenerator& rng)
{
	if (p.ik_solver != "closest")
		return ik_sampling::ExactIk(state, this->joint_group, scene_geometry::ToPoseMsg(target), attempts, p.ik_timeout, this->state_validity, rng);
	const closest_ik::Options options = this->GetClosestIkOptions(p, true);
	for (int attempt = 0; attempt < std::max(1, attempts); ++attempt)
	{
		if (attempt > 0)
			state.setToRandomPositions(this->joint_group, rng);
		Joints start;
		state.copyJointGroupPositions(this->joint_group, start);
		const closest_ik::Result result = closest_ik::Solve(this->scene, state, this->joint_group, this->tool_link, target, start, options);
		if (result.exact)
		{
			state.setJointGroupPositions(this->joint_group, result.q);
			state.update();
			return true;
		}
	}
	return false;
}

// IK solutions per viewpoint: one from its seed (stays near it), the rest from random starts. Tracking: one attempt
// from the seed; random ones only if it fails.
std::vector<std::vector<Joints>> ObjectPlacementOptimizer::CollectIkPoses(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed_q, int max_solutions)
{
	const auto ik_start = std::chrono::steady_clock::now();
	const Eigen::Isometry3d xform = scene_geometry::Translation(at);
	scene_geometry::MoveObject(this->scene, xform * this->object_pose);
	std::vector<std::vector<Joints>> ik(this->viewpoints.size());
	const int max_sol = std::max(1, max_solutions);
	// Tracking: a viewpoint missed at the current spot tries only its seed (its closest pose); random starts rarely
	// reach it after a small move, and the committed solve still tries them.
	std::vector<bool> missed_now(this->viewpoints.size(), false);
	if (p.tracking)
		for (const MissedViewpoint& m : this->current_tour.missed_viewpoints)
			missed_now[static_cast<size_t>(m.viewpoint_index)] = true;
	const uint32_t base_seed = static_cast<uint32_t>(ik_sampling::SeededRng().uniformInteger(1, 1 << 30));
	RunInParallel(this->robot_state, this->viewpoints.size(), base_seed, [&](size_t v, moveit::core::RobotState& state, random_numbers::RandomNumberGenerator& rng)
	{
		const Eigen::Isometry3d target = xform * this->viewpoints[v];
		std::vector<Joints>& sols = ik[v];
		Joints q;
		state.setJointGroupPositions(this->joint_group, seed_q[v]);
		if (this->SolveExactIk(state, target, p.tracking ? 1 : p.ik_attempts, p, rng))
		{
			state.copyJointGroupPositions(this->joint_group, q);
			sols.push_back(q);
		}
		for (int attempt = 0; (!p.tracking || (sols.empty() && !missed_now[v])) && attempt < max_sol * 3 + p.ik_retries_per_point && static_cast<int>(sols.size()) < max_sol; ++attempt)
		{
			if (sols.empty() && attempt >= 4)
				break;	// out of reach: every miss costs a full IK call
			state.setToRandomPositions(this->joint_group, rng);
			if (!this->SolveExactIk(state, target, p.ik_attempts, p, rng))
				continue;
			state.copyJointGroupPositions(this->joint_group, q);
			const bool dup = std::any_of(sols.begin(), sols.end(), [&](const Joints& s) { return joint_travel::JointDistance(s, q, this->counted_joint_indices) < 0.1; });
			if (!dup)
				sols.push_back(q);
		}
	});
	this->ik_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - ik_start).count();
	return ik;
}

// Collects IK poses for every viewpoint, then solves the GTSP over the reached ones (or follows the fixed order).
TourSolution ObjectPlacementOptimizer::SampleIkAndSolveGtsp(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed_q, const std::vector<int>* initial_order, int max_solutions, const std::vector<int>* fixed_order)
{
	gtsp::Solver tour_solver(this->CollectIkPoses(at, p, seed_q, max_solutions), this->home_pose, p.edge_weights, p.gtsp_two_opt_rounds);
	tour_solver.SetTravelJoints(this->counted_joint_indices);
	gtsp::Tour tour = fixed_order ? tour_solver.SolveFixedOrder(*fixed_order) : initial_order ? tour_solver.Solve(*initial_order) : tour_solver.Solve();
	TourSolution out;
	out.viewpoint_order = std::move(tour.order);
	out.ik_poses = std::move(tour.poses);
	out.num_reachable = static_cast<int>(out.viewpoint_order.size());
	out.all_reachable = out.num_reachable == this->num_viewpoint;
	out.joint_distance_plus_turn = joint_travel::TravelCost(PrependHomePose(this->home_pose, out.ik_poses), p.edge_weights, this->counted_joint_indices);
	return out;
}

// Closest pose per missed viewpoint, in parallel. Starts: the nearest reached viewpoint's tour pose (often the right
// posture family), then its own seed; random starts only while no collision-free pose is under the cap (a capped gap
// has no slope).
std::vector<MissedViewpoint> ObjectPlacementOptimizer::FindClosestPosesForMissedViewpoints(const Position& at, const ObjectPlacementParams& p, const TourSolution& tour, const std::vector<Joints>& seed_q)
{
	// A miss without a closest pose counts the largest gap, with no slope.
	const scene_geometry::Vector6d capped_gap = scene_geometry::Vector6d::UnitX() * p.miss_gap_cap;
	std::vector<bool> reached(this->viewpoints.size(), false);
	for (const int viewpoint_index : tour.viewpoint_order)
		reached[static_cast<size_t>(viewpoint_index)] = true;
	std::vector<size_t> misses;
	for (size_t v = 0; v < reached.size(); ++v)
		if (!reached[v])
			misses.push_back(v);
	std::vector<MissedViewpoint> missed;
	if (tour.num_reachable < p.gaps_min_reached)  // rejected on count anyway
	{
		for (const size_t v : misses)
			missed.push_back({static_cast<int>(v), {}, capped_gap});
		return missed;
	}

	const Eigen::Isometry3d xform = scene_geometry::Translation(at);
	const closest_ik::Options closest_options = this->GetClosestIkOptions(p, false);
	std::vector<closest_ik::Result> closest_poses(misses.size());
	const auto closest_start = std::chrono::steady_clock::now();
	const uint32_t base_seed = static_cast<uint32_t>(ik_sampling::SeededRng().uniformInteger(1, 1 << 30));
	RunInParallel(this->robot_state, misses.size(), base_seed, [&](size_t i, moveit::core::RobotState& state, random_numbers::RandomNumberGenerator& rng)
	{
		const size_t v = misses[i];
		const Eigen::Isometry3d target = xform * this->viewpoints[v];
		Joints neighbor;
		double neighbor_dist = std::numeric_limits<double>::max();
		for (size_t k = 0; k < tour.viewpoint_order.size() && !p.tracking; ++k)
		{
			const Eigen::Isometry3d other = xform * this->viewpoints[static_cast<size_t>(tour.viewpoint_order[k])];
			const double d = (other.translation() - target.translation()).norm() + p.rot_metric_scale * Eigen::AngleAxisd(other.linear() * target.linear().transpose()).angle();
			if (d < neighbor_dist)
			{
				neighbor_dist = d;
				neighbor = tour.ik_poses[k];
			}
		}
		const int fixed_starts = neighbor.empty() ? 1 : 2;
		closest_ik::Result& closest = closest_poses[i];
		for (int t = 0; t < std::max(fixed_starts, p.closest_ik_starts) && (t == 0 || !closest.found || closest.gap >= p.miss_gap_cap); ++t)
		{
			Joints start = seed_q[v];
			if (t == 0 && fixed_starts == 2)
				start = neighbor;
			else if (t > 0 && !(t == 1 && fixed_starts == 2))
			{
				state.setToRandomPositions(this->joint_group, rng);
				state.copyJointGroupPositions(this->joint_group, start);
			}
			const closest_ik::Result r = closest_ik::Solve(this->scene, state, this->joint_group, this->tool_link, target, start, closest_options);
			if (r.found && r.gap < closest.gap)
				closest = r;
		}
	});
	this->closest_ik_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - closest_start).count();
	for (size_t i = 0; i < misses.size(); ++i)
	{
		const closest_ik::Result& closest = closest_poses[i];
		missed.push_back({static_cast<int>(misses[i]), closest.q, closest.found ? closest.pose_error : capped_gap});
	}
	return missed;
}

// The tour at `at`: IK and GTSP, closest poses for the missed viewpoints, then the score.
TourSolution ObjectPlacementOptimizer::EvaluateObjectPosition(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed_q, const std::vector<int>* initial_order, int max_solutions, const std::vector<int>* fixed_order)
{
	TourSolution out = this->SampleIkAndSolveGtsp(at, p, seed_q, initial_order, max_solutions, fixed_order);
	out.missed_viewpoints = this->FindClosestPosesForMissedViewpoints(at, p, out, seed_q);

	// Manipulability of the tour poses and the misses' closest poses: in the cost only while viewpoints are missed.
	const double k = p.manipulability_limit_sharpness;
	std::vector<double> ws;
	for (const Joints& q : out.ik_poses)
	{
		ws.push_back(manipulability::Value(this->robot_state, this->joint_group, this->tool_link, q, k));
		out.manipulability_sum += ws.back();
	}
	for (const MissedViewpoint& m : out.missed_viewpoints)
		if (!m.closest_pose.empty())
		{
			ws.push_back(manipulability::Value(this->robot_state, this->joint_group, this->tool_link, m.closest_pose, k));
			out.manipulability_sum += ws.back();
		}
	out.softmin_manipulability = manipulability::SoftMin(ws, p.softmin_tau);

	for (const MissedViewpoint& m : out.missed_viewpoints)
		out.miss_gap_sum += std::min(p.miss_gap_cap, scene_geometry::PoseGap(m.pose_error, p.rot_metric_scale));
	out.miss_cost = p.miss_gap_weight * out.miss_gap_sum;
	out.weighted_cost_sum = p.unreachable_penalty * (this->num_viewpoint - out.num_reachable) + out.miss_cost;
	if (!out.all_reachable)
		out.weighted_cost_sum -= p.manipulability_weight * out.manipulability_sum + p.softmin_manipulability_weight * out.softmin_manipulability;
	return out;
}

// For the manipulability jump: fresh IK poses per viewpoint, keeping each one's most manipulable pose. No GTSP: the jump
// score ignores the viewpoint order and travel. Viewpoints in input order.
TourSolution ObjectPlacementOptimizer::FindMostManipulablePoses(const Position& at, const ObjectPlacementParams& p)
{
	const std::vector<std::vector<Joints>> ik_poses = this->CollectIkPoses(at, p, this->ik_start_poses, p.max_solutions_per_candidate);
	TourSolution out;
	std::vector<double> manipulabilities;
	for (int v = 0; v < this->num_viewpoint; ++v)
	{
		double best_manipulability = -std::numeric_limits<double>::infinity();
		const Joints* best_pose = nullptr;
		for (const Joints& q : ik_poses[static_cast<size_t>(v)])
		{
			const double manipulability = manipulability::Value(this->robot_state, this->joint_group, this->tool_link, q, p.manipulability_limit_sharpness);
			if (manipulability > best_manipulability)
			{
				best_manipulability = manipulability;
				best_pose = &q;
			}
		}
		if (!best_pose)
			continue;
		out.viewpoint_order.push_back(v);
		out.ik_poses.push_back(*best_pose);
		manipulabilities.push_back(best_manipulability);
		out.manipulability_sum += best_manipulability;
	}
	out.num_reachable = static_cast<int>(out.viewpoint_order.size());
	out.all_reachable = out.num_reachable == this->num_viewpoint;
	out.softmin_manipulability = manipulability::SoftMin(manipulabilities, p.softmin_tau);
	return out;
}

// Quick solve for comparing nearby spots. Tracked (reach phase): each viewpoint follows its current pose with one IK
// attempt in a fixed order.
TourSolution ObjectPlacementOptimizer::QuickSolveTour(const Position& at, ObjectPlacementParams p)
{
	ik_sampling::ReseedRng(this->attempt_seed);
	if (!this->quick_solve_start_from_current_ik)
		return this->EvaluateObjectPosition(
			at, p, this->ik_start_poses, &this->current_tour.viewpoint_order, this->params.max_solutions_per_candidate,
			this->fixed_order);
	p.tracking = true;
	return this->EvaluateObjectPosition(at, p, this->ik_start_poses, &this->current_tour.viewpoint_order, 1, &this->locked_order);
}

// Room-then-reach: evaluates a candidate move to `at`; each reached viewpoint's IK starts from its pose in from_tour.
TourSolution ObjectPlacementOptimizer::TestCandidateMove(const TourSolution& from_tour, const Position& at)
{
	for (size_t k = 0; k < from_tour.viewpoint_order.size(); ++k)
		this->ik_start_poses[static_cast<size_t>(from_tour.viewpoint_order[k])] = from_tour.ik_poses[k];
	this->run_result.num_inner_solves += 1;
	ObjectPlacementParams candidate_params = this->params;
	candidate_params.gaps_min_reached = from_tour.num_reachable;  // a candidate reaching fewer is rejected anyway, so it skips miss gaps
	return this->QuickSolveTour(at, candidate_params);
}

// More full solves at a fixed spot; keeps the lowest cost, never fewer viewpoints reached. Warm start: IK starts and
// order from the best so far, more IK choices, each solve OMPL-planned and logged.
TourSolution ObjectPlacementOptimizer::RunGtspRestartsAndEvaluate(const Position& at, TourSolution best_sol, int num_restarts, bool warm_start, const std::vector<int>* fixed_order)
{
	if (warm_start && this->can_plan && best_sol.planned_cost < 0.0)
		best_sol.planned_cost = this->ComputeRealPlannedTravelCost(at, best_sol);
	best_sol.weighted_cost_sum = this->ComputeWeightedCost(best_sol);
	if (warm_start)
		RCLCPP_INFO(this->node->get_logger(), "  GTSP restarts start: reachable %d/%d, joint dist + largest turn %.2f, planned travel %.2f rad", best_sol.num_reachable, this->num_viewpoint, best_sol.joint_distance_plus_turn, best_sol.planned_cost);
	for (int i = 0; i < num_restarts && rclcpp::ok(); ++i)
	{
		const std::vector<Joints> ik_starts = warm_start ? GetIkStartPoses(best_sol, this->home_pose, this->num_viewpoint) : this->ik_start_poses;
		TourSolution s = this->EvaluateObjectPosition(at, this->params, ik_starts, &best_sol.viewpoint_order, warm_start ? this->params.gtsp_restart_ik_solutions : this->params.max_solutions_per_candidate, fixed_order);
		this->run_result.num_inner_solves += 1;
		if (s.num_reachable < best_sol.num_reachable)
		{
			if (warm_start)
				RCLCPP_INFO(this->node->get_logger(), "  GTSP restart %d/%d: reachable %d/%d, fewer than the best -- skipped", i + 1, num_restarts, s.num_reachable, this->num_viewpoint);
			continue;
		}
		if (warm_start && this->can_plan)
			s.planned_cost = this->ComputeRealPlannedTravelCost(at, s);
		s.weighted_cost_sum = this->ComputeWeightedCost(s);
		const bool better = s.weighted_cost_sum < best_sol.weighted_cost_sum;
		if (warm_start)
			RCLCPP_INFO(this->node->get_logger(), "  GTSP restart %d/%d: joint dist + largest turn %.2f, planned travel %.2f rad vs %.2f best%s", i + 1, num_restarts, s.joint_distance_plus_turn, s.planned_cost, best_sol.planned_cost, better ? "  <-- new best" : "");
		if (better)
			best_sol = std::move(s);
	}
	return best_sol;
}

// Planned joint travel: every edge planned with OMPL. A failed edge counts its straight-line distance plus
// unreachable_penalty.
double ObjectPlacementOptimizer::ComputeRealPlannedTravelCost(const Position& at, const TourSolution& sol)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::Translation(at) * this->object_pose);
	const std::vector<double> edges = PlanTourJointPathLengths(
		this->node, this->robot_model, this->scene, sol.ik_poses, this->home_pose, this->group_name,
		this->params.real_cost_planning_time, this->params.real_cost_attempts,
		/*count_last_joint=*/false);
	double total = 0.0;
	for (size_t k = 0; k < sol.ik_poses.size(); ++k)
		total += (k < edges.size() && edges[k] >= 0.0)
			? edges[k]
			: joint_travel::JointDistance(
				  k == 0 ? this->home_pose : sol.ik_poses[k - 1], sol.ik_poses[k], this->counted_joint_indices) +
				this->params.unreachable_penalty;
	return total;
}

// Adds planned travel to the cost, only once every viewpoint is reached (before that comparisons go by count and gap).
TourSolution ObjectPlacementOptimizer::AddPlannedTravelToCost(const Position& at, TourSolution sol)
{
	if (!this->use_planned_travel_in_cost || !sol.all_reachable || sol.planned_cost >= 0.0)
		return sol;
	sol.planned_cost = this->ComputeRealPlannedTravelCost(at, sol);
	sol.weighted_cost_sum += sol.planned_cost;
	return sol;
}

double ObjectPlacementOptimizer::ComputeWeightedCost(const TourSolution& sol) const
{
	const double travel = this->use_planned_travel_in_cost && sol.planned_cost >= 0.0 ? sol.planned_cost : 0.0;
	const double reach_phase_comfort = sol.all_reachable ? 0.0 : this->params.manipulability_weight * sol.manipulability_sum + this->params.softmin_manipulability_weight * sol.softmin_manipulability;
	return travel + this->params.unreachable_penalty * (this->num_viewpoint - sol.num_reachable) + sol.miss_cost - reach_phase_comfort;
}

// Better if reached more viewpoints or cost is lower
bool ObjectPlacementOptimizer::HasMoreReachedOrLowerCost(const TourSolution& a, const TourSolution& b) const
{
	return a.num_reachable > b.num_reachable || (a.num_reachable == b.num_reachable && a.weighted_cost_sum < b.weighted_cost_sum);
}

// History entry, and the descent's best (most viewpoints reached, then cost).
void ObjectPlacementOptimizer::RecordHistoryAndBest(const Position& at, const TourSolution& sol)
{
	TourSolution s = sol;
	s.weighted_cost_sum = this->ComputeWeightedCost(sol);
	this->run_result.history.push_back(
		{static_cast<double>(this->attempt_index), at.x(), at.y(), at.z(), s.weighted_cost_sum});
	if (s.num_reachable > this->best_in_attempt.tour_solution.num_reachable ||
		(s.num_reachable == this->best_in_attempt.tour_solution.num_reachable &&
		 s.weighted_cost_sum < this->best_in_attempt.weighted_cost_sum))
	{
		this->best_in_attempt.position = at;
		this->best_in_attempt.weighted_cost_sum = s.weighted_cost_sum;
		this->best_in_attempt.all_reachable = s.all_reachable;
		this->best_in_attempt.tour_solution = std::move(s);
	}
}

// Move towards higher manipulability
Eigen::Vector3d ObjectPlacementOptimizer::ComputeManipulabilityGradient(const std::vector<Joints>& poses, double sum_weight, double softmin_weight)
{
	const double k = this->params.manipulability_limit_sharpness;
	std::vector<double> ws;
	std::vector<Eigen::Vector3d> dw;
	Eigen::Vector3d g = Eigen::Vector3d::Zero();
	for (const Joints& q : poses)
	{
		const Eigen::MatrixXd dq_dp = scene_geometry::DampedPinv(manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q), this->params.jacobian_damping).leftCols<3>();
		ws.push_back(std::max(1e-6, manipulability::Value(this->robot_state, this->joint_group, this->tool_link, q, k)));
		dw.push_back(dq_dp.transpose() * manipulability::Gradient(this->robot_state, this->joint_group, this->tool_link, q, k));
		g -= sum_weight * dw.back();
	}
	if (ws.empty() || softmin_weight <= 0.0)
		return g;
	const std::vector<double> shares = manipulability::SoftMinShares(ws, this->params.softmin_tau);
	for (size_t i = 0; i < shares.size(); ++i)
		g -= softmin_weight * shares[i] * dw[i];
	return g;
}

Eigen::Vector3d ObjectPlacementOptimizer::ComputeCostGradient(const TourSolution& sol, const ObjectPlacementParams& p)
{
	Eigen::Vector3d g = Eigen::Vector3d::Zero();
	// While viewpoints are missed, manipulability of the tour poses and the misses' closest poses keeps the arm away from limits.
	if (!sol.all_reachable)
	{
		std::vector<Joints> poses = sol.ik_poses;
		for (const MissedViewpoint& m : sol.missed_viewpoints)
			if (!m.closest_pose.empty())
				poses.push_back(m.closest_pose);
		g += this->ComputeManipulabilityGradient(poses, p.manipulability_weight, p.softmin_manipulability_weight);
	}
	// MissedViewpoint gaps: the viewpoint moves with the object; no slope past the cap.
	for (const MissedViewpoint& m : sol.missed_viewpoints)
	{
		const double gap = scene_geometry::PoseGap(m.pose_error, p.rot_metric_scale);
		if (gap >= 1e-9 && gap < p.miss_gap_cap)
			g += p.miss_gap_weight * Eigen::Vector3d(m.pose_error.head<3>()) / gap;
	}
	return g;
}

// Move object towards closet IK for missed viewpoints
Eigen::Vector3d ObjectPlacementOptimizer::GetMoveDirectionToCloseGap(int target, const TourSolution& here_at, bool first)
{
	const scene_geometry::Vector6d* gap = FindMissedPoseError(here_at, target);
	if (!gap)
		return Eigen::Vector3d::Zero();
	const scene_geometry::Vector6d* full = first ? FindMissedPoseError(this->current_tour, target) : nullptr;
	if (full &&
		scene_geometry::PoseGap(*full, this->params.rot_metric_scale) <
			scene_geometry::PoseGap(*gap, this->params.rot_metric_scale))
		gap = full;
	const Eigen::Vector3d aim = -gap->head<3>();
	return aim.norm() < 1e-9 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(aim.normalized());
}

// Model stores IK pose, dq_dp, and each pose's margins
room_margins::ArmModel ObjectPlacementOptimizer::BuildRoomMarginArmModel(const Position& at, const TourSolution& sol)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::Translation(at) * this->object_pose);
	room_margins::ArmModel model;
	model.q.resize(static_cast<size_t>(this->num_viewpoint));
	model.dq_dp.resize(static_cast<size_t>(this->num_viewpoint));
	for (size_t k = 0; k < sol.viewpoint_order.size(); ++k)
	{
		const Joints& q = sol.ik_poses[k];
		if (q.empty())
			continue;
		const size_t v = static_cast<size_t>(sol.viewpoint_order[k]);
		model.q[v] = q;
		model.dq_dp[v] = scene_geometry::DampedPinv(manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q), this->params.jacobian_damping).leftCols<3>();
		const std::vector<room_margins::Margin> m = this->GetIkPoseMargins(sol.viewpoint_order[k], q);
		model.margins.insert(model.margins.end(), m.begin(), m.end());
	}
	return model;
}

// Smallest singular value of the tool Jacobian: 0 when fully stretched or folded.
double ObjectPlacementOptimizer::GetStretch(const Joints& q)
{
	return Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q)).singularValues().minCoeff();
}

// 1) Distance of each joint to its limit 2) Smallest singular value (stretch) 3) Distance to collision object
// A 7-joint IK has 14 joint limit margin, 1 stretch margin, and a variable number of collision margins
std::vector<room_margins::Margin> ObjectPlacementOptimizer::GetIkPoseMargins(int v, const Joints& q)
{
	const double kJointFloor = 0.05;	 // rad
	const double kStretchFloor = 0.02;
	const double kCollisionFloor = 0.01;  // m
	const double kWindow = 0.1;			 // m
	const double h = 1e-5;				 // rad, finite-difference step
	const int dof = static_cast<int>(q.size());
	const Eigen::MatrixXd dq_dp = scene_geometry::DampedPinv(manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q), this->params.jacobian_damping).leftCols<3>();
	std::vector<room_margins::Margin> margins;
	for (int i = 0; i < dof; ++i)
	{
		const moveit::core::VariableBounds& b = this->robot_model->getVariableBounds(this->joint_group->getVariableNames()[static_cast<size_t>(i)]);
		if (!b.position_bounded_)
			continue;
		const Eigen::VectorXd e = Eigen::VectorXd::Unit(dof, i);
		const std::string joint = "joint " + std::to_string(i + 1);
		margins.push_back({v, joint + " low limit", q[static_cast<size_t>(i)] - b.min_position_, kJointFloor, dq_dp.transpose() * e, e});
		margins.push_back({v, joint + " high limit", b.max_position_ - q[static_cast<size_t>(i)], kJointFloor, dq_dp.transpose() * -e, -e});
	}
	const double stretch = this->GetStretch(q);
	Eigen::VectorXd stretch_slope(dof);  // finite difference
	for (int i = 0; i < dof; ++i)
	{
		Joints shifted = q;
		shifted[static_cast<size_t>(i)] += h;
		stretch_slope(i) = (this->GetStretch(shifted) - stretch) / h;
	}
	margins.push_back({v, "stretch", stretch, kStretchFloor, dq_dp.transpose() * stretch_slope, stretch_slope});
	this->robot_state.setJointGroupPositions(this->joint_group, q);
	this->robot_state.update();
	collision_detection::DistanceRequest req = closest_ik::CloseRequest(this->robot_state, this->joint_group, kWindow);
	const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, this->robot_state, this->joint_group, req, kWindow);

	for (size_t i = 0; i < pairs.dists.size(); ++i)
	{
		const Eigen::VectorXd joint_slope = pairs.gradients[i].transpose();
		margins.push_back({v, "collision " + pairs.names[i], pairs.dists[i], kCollisionFloor, dq_dp.transpose() * joint_slope + pairs.object_slopes[i], joint_slope});
	}

	return margins;
}

room_margins::Shortage ObjectPlacementOptimizer::GetMarginsThatRunOut(int v, const room_margins::ArmModel& model, const Position& at, const Eigen::Vector3d& d)
{
	const Eigen::Vector3d moved = ClampToBounds(at + d, this->params.bounds) - at;
	room_margins::Shortage out;
	for (const room_margins::Margin& m : model.margins)
		if (m.viewpoint_index == v && room_margins::RoomAlong({m}, moved.normalized()) < moved.norm())
		{
			out.margins.push_back(m);
			out.why += (out.why.empty() ? "" : ", ") + m.name;
		}
	if (out.why.empty())
		out.why = "nothing predicted";
	return out;
}

// Find another IKs for one viewpoint with more joint room
std::optional<Joints> ObjectPlacementOptimizer::FindRoomierPose(int v, const Position& at, const room_margins::ArmModel& model, int tries)
{
	const int resamples = tries / 2;
	const int nudges = tries - resamples;
	const double nudge_step = 0.1;	// rad: nudge i slides (i + 1) times this
	const std::vector<room_margins::Margin> mine = room_margins::MarginsOf(model, v);
	const double now = room_margins::Tightest(mine);
	if (now >= 1.0)
		return std::nullopt;
	std::string tightest;
	Eigen::VectorXd raise;	// d(tightest margin)/dq
	for (const room_margins::Margin& m : mine)
		if (m.value / m.floor == now)
		{
			char buf[96];
			std::snprintf(buf, sizeof(buf), "%s %.4f", m.name.c_str(), m.value);
			tightest = buf;
			raise = m.joint_slope;
		}
	scene_geometry::MoveObject(this->scene, scene_geometry::Translation(at) * this->object_pose);
	const Eigen::Isometry3d target = scene_geometry::Translation(at) * this->viewpoints[static_cast<size_t>(v)];
	std::vector<std::pair<Joints, std::string>> candidate_poses;  // pose and how it was found
	for (int i = 0; i < resamples; ++i)
	{
		this->robot_state.setToRandomPositions(this->joint_group, ik_sampling::SeededRng());
		if (!this->SolveExactIk(this->robot_state, target, 1, this->params, ik_sampling::SeededRng()))
			continue;
		Joints q;
		this->robot_state.copyJointGroupPositions(this->joint_group, q);
		candidate_poses.push_back({q, "resampled"});
	}
	const Joints& q0 = model.q[static_cast<size_t>(v)]; 
	const Eigen::MatrixXd J = manipulability::ToolJacobian(this->robot_state, this->joint_group, this->tool_link, q0);
	const Eigen::VectorXd slide = (Eigen::MatrixXd::Identity(J.cols(), J.cols()) - scene_geometry::DampedPinv(J, this->params.jacobian_damping) * J) * raise;
	for (int i = 0; i < nudges && slide.norm() > 1e-9; ++i)
	{
		Joints q = q0;
		const Eigen::VectorXd dq = (i + 1) * nudge_step * slide.normalized();
		for (size_t j = 0; j < q.size(); ++j)
			q[j] += dq(static_cast<Eigen::Index>(j));
		// Back onto the viewpoint: one IK attempt from the nudged pose stays near it (collision-free, within limits).
		this->robot_state.setJointGroupPositions(this->joint_group, q);
		if (!this->SolveExactIk(this->robot_state, target, 1, this->params, ik_sampling::SeededRng()))
			continue;
		Joints exact;
		this->robot_state.copyJointGroupPositions(this->joint_group, exact);
		if (this->robot_state.satisfiesBounds(this->joint_group))
			candidate_poses.push_back({exact, "nudged"});
	}
	double best_score = now + 1.0;
	std::optional<Joints> roomiest;
	std::string how;
	for (const auto& [pose, kind] : candidate_poses)
	{
		const double score = room_margins::Tightest(this->GetIkPoseMargins(v, pose));
		if (score > best_score)
		{
			best_score = score;
			roomiest = pose;
			how = kind;
		}
	}
	if (this->params.trace_line_search)
		RCLCPP_INFO(
			this->node->get_logger(), "    vp %d near a limit (smallest margin %.2f floors, %s): %s", v, now, tightest.c_str(),
			roomiest ? (how + " IK with " + std::to_string(best_score) + " floors swapped in").c_str() : "no roomier IK");
	return roomiest;
}

// ---------------------------------------------------------------------------------------------
// Logging and Visualization
// ---------------------------------------------------------------------------------------------

// Debug: log what refining right at the first all-reached spot would give.
void ObjectPlacementOptimizer::LogGtspRestartsAtFirstReach()
{
	if (this->params.gtsp_restarts_at_reach <= 0 || !this->current_tour.all_reachable || this->logged_gtsp_restarts_at_first_reach)
		return;
	this->logged_gtsp_restarts_at_first_reach = true;
	TourSolution start = this->current_tour;
	if (this->can_plan && start.planned_cost < 0.0)
		start.planned_cost = this->ComputeRealPlannedTravelCost(this->current_position, start);
	const std::vector<int>* lock = this->params.lock_input_order ? &this->input_order : this->params.freeze_order ? &start.viewpoint_order : nullptr;
	const TourSolution restarted = this->RunGtspRestartsAndEvaluate(this->current_position, start, this->params.gtsp_restarts_at_reach, true, lock);
	RCLCPP_INFO(
		this->node->get_logger(),
		"  reach attempt %d: %d GTSP restarts at first all-reached abs (%.4f, %.4f, %.4f) m: joint dist + largest turn %.2f -> %.2f  "
		"planned travel %.2f -> %.2f",
		this->attempt_index + 1, this->params.gtsp_restarts_at_reach, this->current_position.x(), this->current_position.y(),
		this->current_position.z(), start.joint_distance_plus_turn, restarted.joint_distance_plus_turn, start.planned_cost,
		restarted.planned_cost);
}

// Live markers: object mesh, every move so far (orange kinematic, white jump, cyan reachability), -gradient arrow, viewpoints (green reached, red missed), and
// each miss's closest pose with a line to its target. The previous publish is cleared first.
void ObjectPlacementOptimizer::PublishProgressMarkers(const Eigen::Vector3d& neg_grad)
{
	if (!this->params.progress_pub)
		return;
	const rclcpp::Time stamp = this->node->now();
	const Eigen::Isometry3d xform = scene_geometry::Translation(this->current_position);
	const Eigen::Vector3d object_at = (xform * this->object_pose).translation();
	// Markers update in place (same ns and id) instead of after a DELETEALL each time, which made RViz reload meshes and
	// flash. The first publish still clears what an earlier run left.
	visualization_msgs::msg::MarkerArray markers;
	if (this->progress_marker_keys.empty())
	{
		visualization_msgs::msg::Marker clear;
		clear.action = visualization_msgs::msg::Marker::DELETEALL;
		markers.markers.push_back(clear);
	}

	if (!this->params.progress_mesh_path.empty())
	{
		visualization_msgs::msg::Marker mesh = marker_utils::MakeMarker("object_placement_current", visualization_msgs::msg::Marker::MESH_RESOURCE, stamp);
		mesh.mesh_resource = "file://" + this->params.progress_mesh_path;
		mesh.mesh_use_embedded_materials = false;
		mesh.pose = scene_geometry::ToPoseMsg(xform * this->object_pose);
		mesh.scale.x = mesh.scale.y = mesh.scale.z = this->params.progress_mesh_scale;
		mesh.color = marker_utils::Rgba(1.0f, 0.85f, 0.0f, 0.6f);
		markers.markers.push_back(mesh);
	}

	visualization_msgs::msg::Marker vps = marker_utils::MakeMarker("object_placement_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	vps.scale.x = vps.scale.y = vps.scale.z = 0.01;
	vps.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t v = 0; v < this->viewpoints.size(); ++v)
	{
		const bool missed = std::any_of(
			this->current_tour.missed_viewpoints.begin(), this->current_tour.missed_viewpoints.end(),
			[&](const MissedViewpoint& m) { return m.viewpoint_index == static_cast<int>(v); });
		vps.points.push_back(scene_geometry::ToPoint((xform * this->viewpoints[v]).translation()));
		vps.colors.push_back(missed ? marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f) : marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f));
	}
	markers.markers.push_back(vps);

	if (!this->move_history.empty())
	{
		// One cylinder per move: line markers vanished from some viewing angles.
		visualization_msgs::msg::Marker crumbs = marker_utils::MakeMarker("object_placement_history", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
		crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.012;
		crumbs.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
		for (size_t k = 0; k < this->move_history.size(); ++k)
		{
			const MoveSegment& m = this->move_history[k];
			const std_msgs::msg::ColorRGBA c = m.type == MoveType::KinematicRoomMove ? marker_utils::Rgba(1.0f, 0.4f, 0.1f, 1.0f) : marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
			const Eigen::Vector3d from = (scene_geometry::Translation(m.from) * this->object_pose).translation();
			const Eigen::Vector3d to = (scene_geometry::Translation(m.to) * this->object_pose).translation();
			const double length = (to - from).norm();
			if (length > 1e-6)
			{
				visualization_msgs::msg::Marker segment = marker_utils::MakeMarker("object_placement_trail", visualization_msgs::msg::Marker::CYLINDER, stamp, static_cast<int>(k));
				Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
				pose.translation() = 0.5 * (from + to);
				pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), to - from).toRotationMatrix();
				segment.pose = scene_geometry::ToPoseMsg(pose);
				segment.scale.x = segment.scale.y = 0.005;
				segment.scale.z = length;
				segment.color = c;
				markers.markers.push_back(segment);
			}
			crumbs.points.push_back(scene_geometry::ToPoint(to));
			crumbs.colors.push_back(c);
		}
		markers.markers.push_back(crumbs);
	}

	const double gnorm = neg_grad.norm();
	if (gnorm > 1e-9)
	{
		visualization_msgs::msg::Marker arrow = marker_utils::MakeMarker("object_placement_descent_dir", visualization_msgs::msg::Marker::ARROW, stamp);
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.color = marker_utils::Rgba(0.1f, 0.5f, 1.0f, 0.95f);
		arrow.points = {scene_geometry::ToPoint(object_at), scene_geometry::ToPoint(object_at + neg_grad * (0.15 / gnorm))};
		markers.markers.push_back(arrow);
	}

	visualization_msgs::msg::Marker gaps = marker_utils::MakeMarker("object_placement_miss_gap", visualization_msgs::msg::Marker::LINE_LIST, stamp);
	gaps.scale.x = 0.003;
	gaps.color = marker_utils::Rgba(1.0f, 0.2f, 1.0f, 1.0f);
	visualization_msgs::msg::MarkerArray arms;
	moveit::core::RobotState ghost(this->robot_state);
	for (const MissedViewpoint& m : this->current_tour.missed_viewpoints)
	{
		if (m.closest_pose.empty())
			continue;
		ghost.setJointGroupPositions(this->joint_group, m.closest_pose);
		ghost.update();
		ghost.getRobotMarkers(arms, ghost.getRobotModel()->getLinkModelNamesWithCollisionGeometry(), marker_utils::Rgba(0.9f, 0.2f, 0.2f, 0.35f), "object_placement_miss_ik", rclcpp::Duration(0, 0));
		gaps.points.push_back(scene_geometry::ToPoint(ghost.getGlobalLinkTransform("tool0").translation()));
		gaps.points.push_back(scene_geometry::ToPoint((xform * this->viewpoints[static_cast<size_t>(m.viewpoint_index)]).translation()));
	}
	if (!gaps.points.empty())
		markers.markers.push_back(gaps);
		
	for (visualization_msgs::msg::Marker& m : arms.markers)
	{
		m.header.frame_id = "world";
		m.header.stamp = stamp;
		markers.markers.push_back(m);
	}
	std::set<std::pair<std::string, int>> keys;
	for (const visualization_msgs::msg::Marker& m : markers.markers)
		if (m.action == visualization_msgs::msg::Marker::ADD)
			keys.insert({m.ns, m.id});
	for (const std::pair<std::string, int>& key : this->progress_marker_keys)
		if (keys.count(key) == 0)
		{
			visualization_msgs::msg::Marker stale = marker_utils::MakeMarker(key.first, visualization_msgs::msg::Marker::SPHERE, stamp, key.second);
			stale.action = visualization_msgs::msg::Marker::DELETE;
			markers.markers.push_back(stale);
		}
	this->progress_marker_keys = std::move(keys);
	this->params.progress_pub->publish(markers);
	if (this->params.visualize_progress_delay_sec > 0.0)
		std::this_thread::sleep_for(std::chrono::duration<double>(this->params.visualize_progress_delay_sec));
}

}  // namespace object_placement
