#include "object_placement_planning/object_gtsp_co_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>

#include <random_numbers/random_numbers.h>

#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/manipulability.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"

namespace object_gtsp_co_optimize
{

namespace
{

// A margin blocks if closer than this many safe distances.
constexpr double miss_blocking_multiple = 6.0;
constexpr double reached_blocking_multiple = 2.0;
constexpr double smallest_reach_step = 0.005;  // m
constexpr double degrees_per_radian = 180.0 / M_PI;  // settings and logs use degrees
constexpr double radians_per_degree = M_PI / 180.0;
constexpr double smallest_kept_share = 0.3;  // a direction shorter than this after trimming is dropped

// Safe distances that one KME margin unit stands for.
constexpr double joint_safe_distance = 0.05;  // rad
constexpr double stretch_safe_distance = 0.02;
constexpr double collision_safe_distance = 0.01;  // m

Joints AddToJoints(const Joints& joints, const Eigen::VectorXd& change)
{
	Joints result = joints;
	for (size_t joint_index = 0; joint_index < result.size(); ++joint_index)
		result[joint_index] += change(static_cast<Eigen::Index>(joint_index));
	return result;
}

double GetLargestJointChange(const Joints& from, const Joints& to)
{
	double largest_change = 0.0;
	for (size_t joint_index = 0; joint_index < from.size(); ++joint_index)
		largest_change = std::max(largest_change, std::abs(to[joint_index] - from[joint_index]));
	return largest_change;
}

// Projector onto the arm's self-motion: joint moves that keep the camera still.
Eigen::MatrixXd GetSelfMotionProjector(const Eigen::MatrixXd& jacobian)
{
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
	const Eigen::VectorXd& singular_values = svd.singularValues();
	const double tolerance = 1e-6 * std::max(1.0, singular_values.size() > 0 ? singular_values[0] : 0.0);
	const Eigen::Index rank = (singular_values.array() > tolerance).count();
	const Eigen::MatrixXd null_basis = svd.matrixV().rightCols(jacobian.cols() - rank);
	return null_basis * null_basis.transpose();
}

int FindPoseIndex(const std::vector<Joints>& options, const Joints& pose)
{
	for (size_t option_index = 0; option_index < options.size(); ++option_index)
		if (options[option_index] == pose)
			return static_cast<int>(option_index);
	return 0;
}

}  // namespace

Optimizer::Optimizer(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model, planning_scene_monitor::PlanningSceneMonitorPtr scene, const std::string& group_name, std::vector<Eigen::Isometry3d> viewpoints, const Eigen::Matrix3d& object_rotation, Joints home_pose, Settings settings, ProgressDisplay progress_display)
	: node(std::move(node)), model(std::move(model)), scene(std::move(scene)), home_pose(std::move(home_pose)), settings(std::move(settings)), viewpoints(std::move(viewpoints)), object_rotation(object_rotation), progress_display(std::move(progress_display))
{
	this->state = std::make_unique<moveit::core::RobotState>(this->model);
	this->state->setToDefaultValues();
	this->group = this->state->getJointModelGroup(group_name);
	this->tool = this->model->getLinkModel("tool0");
	this->camera_side_links = closest_ik::GetCameraLinks(this->tool);
	SetJointSpeedLimits(this->model, group_name, this->settings.tour_settings.edge_weights);
	this->counted_joints = joint_travel::AllJoints(static_cast<int>(this->home_pose.size()) - 1);  // the last joint only spins the camera
	this->num_threads = std::max(1u, std::min(std::thread::hardware_concurrency(), static_cast<unsigned int>(this->viewpoints.size())));
	random_numbers::RandomNumberGenerator reference_rng(static_cast<uint32_t>(this->settings.ik_settings.random_ik_seed));
	moveit::core::RobotState reference_state(*this->state);
	for (int reference_index = 0; reference_index < this->settings.tour_settings.num_reference_postures; ++reference_index)
	{
		Joints reference_posture;
		reference_state.setToRandomPositions(this->group, reference_rng);
		reference_state.copyJointGroupPositions(this->group, reference_posture);
		this->reference_postures.push_back(reference_posture);
	}
}

// ---------------------------------------------------------------- Descent

TourSolution Optimizer::runDescent(const Eigen::Vector3d& start)
{
	this->start_time = std::chrono::steady_clock::now();
	const Eigen::Vector3d clamped_start = this->clampToSearchBox(start);
	TourSolution solution = this->solveTourAtPosition(clamped_start, std::vector<Joints>(this->viewpoints.size(), this->home_pose), {}, {});
	double cpd_step = this->settings.step_sizes.initial_step;
	double reconfiguration_step = this->settings.step_sizes.initial_step;
	this->object_trail.assign(1, clamped_start);
	this->publishProgressMarkers(solution.placement_ik, solution.order, Eigen::Vector3d::Zero());
	this->logTourSolution("start", solution);
	const MethodSwitches& switches = this->settings.method_switches;
	bool object_frozen = false;
	for (int iteration = 1; iteration <= this->settings.descent_limits.max_iterations && rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - this->start_time).count() < this->settings.descent_limits.time_budget; ++iteration)
	{
		bool any_step_kept = false;
		if (!object_frozen)
			any_step_kept = this->runKmeReachStep(solution, reconfiguration_step, iteration) || any_step_kept;
		if (switches.freeze_object_when_reached && !object_frozen && this->isAllReached(solution.placement_ik))
		{
			object_frozen = true;
			RCLCPP_INFO(this->node->get_logger(), "iteration %d: all reached; object frozen at abs (%.4f, %.4f, %.4f)", iteration, solution.placement_ik.object_position.x(), solution.placement_ik.object_position.y(), solution.placement_ik.object_position.z());
		}
		any_step_kept = this->runPoseSlideStep(solution, iteration) || any_step_kept;
		if (switches.use_cpd && !object_frozen)
			any_step_kept = this->runCpdStep(solution, cpd_step, iteration) || any_step_kept;
		if (!any_step_kept)
			break;
	}
	this->logTourSolution("end", solution);
	this->logMissedViewpoints(solution.placement_ik);
	return solution;
}

// Reach step and KME on misses; once all are reached, KME on reconfigurations.
bool Optimizer::runKmeReachStep(TourSolution& solution, double& reconfiguration_step, int iteration)
{
	const MethodSwitches& switches = this->settings.method_switches;
	bool moved = false;
	Eigen::Vector3d kept_reach_direction = Eigen::Vector3d::Zero();
	if (!this->isAllReached(solution.placement_ik))
		moved = this->moveTowardMisses(solution, kept_reach_direction, iteration);
	if (switches.use_kme && !this->isAllReached(solution.placement_ik))
		moved = this->expandMarginsOfMisses(solution, this->settings.step_sizes.initial_step, kept_reach_direction, iteration) || moved;
	// Reach and KME change IK only; the tour is scored once here.
	if (moved)
	{
		solution = this->improveTourForPlacement(solution.placement_ik, solution.order);
		this->logTourSolution("iteration " + std::to_string(iteration) + " reach/KME kept, tour scored", solution);
	}
	if (switches.use_kme && !switches.freeze_object_when_reached && this->isAllReached(solution.placement_ik) && solution.num_reconfigurations > 0)
		moved = this->expandMarginsOfReconfigurations(solution, reconfiguration_step, iteration) || moved;
	return moved;
}

// Object step along the mean miss error; kept if no fewer are reached.
bool Optimizer::moveTowardMisses(TourSolution& solution, Eigen::Vector3d& kept_direction, int iteration)
{
	const Eigen::Vector3d reach_move = this->computeMoveTowardMisses(solution.placement_ik);
	if (reach_move.norm() < 1e-12)
		return false;
	const Eigen::Vector3d direction = reach_move.normalized();
	double step = this->settings.step_sizes.initial_step;
	for (int attempt = 0; attempt < this->settings.step_sizes.max_line_search_tries && step >= smallest_reach_step - 1e-9; ++attempt, step *= 0.5)
	{
		const PlacementIkSolutions candidate = this->solveIkForAllViewpoints(this->clampToSearchBox(solution.placement_ik.object_position + step * direction), this->convertIkSolutionsToVectorOfJoints(solution.placement_ik), {});
		const bool kept = candidate.num_reached >= solution.placement_ik.num_reached;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu  gap sum %.4f m vs %.4f%s", iteration, step, candidate.object_position.x(), candidate.object_position.y(), candidate.object_position.z(), candidate.num_reached, this->viewpoints.size(), candidate.gap_sum, solution.placement_ik.gap_sum, kept ? "  <-- kept" : "");
		if (!kept)
			continue;
		solution.placement_ik = candidate;
		this->recordObjectMove(solution, direction);
		kept_direction = direction;
		return true;
	}
	return false;
}

// KME on misses: kept if misses gain reach or margin, losing few.
bool Optimizer::expandMarginsOfMisses(TourSolution& solution, double object_step, const Eigen::Vector3d& kept_reach_direction, int iteration)
{
	const std::vector<size_t> missed_viewpoints = this->getViewpointsWithReach(solution.placement_ik, false);
	if (missed_viewpoints.empty())
		return false;
	const std::vector<room_margins::Margin> margins_before = this->getMarginsOfViewpoints(solution.placement_ik, missed_viewpoints, miss_blocking_multiple);
	const double soft_min_before = room_margins::SoftMin(margins_before);
	for (int attempt = 0; attempt < this->settings.step_sizes.max_line_search_tries && object_step >= smallest_reach_step - 1e-9; ++attempt, object_step *= 0.5)
	{
		bool keeps_reached_clear = false;
		Eigen::Vector3d direction = this->computeKmeDirection(solution.placement_ik, margins_before, object_step, keeps_reached_clear);
		if (direction.isZero())
			return false;
		// Don't undo this iteration's reach move.
		const double against_reach = direction.dot(kept_reach_direction);
		if (against_reach < 0.0)
		{
			const Eigen::Vector3d kept_part = direction - against_reach * kept_reach_direction;
			RCLCPP_INFO(this->node->get_logger(), "iteration %d room-then-reach: direction (%.2f, %.2f, %.2f) undoes the reach move by %.2f; %.2f left%s", iteration, direction.x(), direction.y(), direction.z(), -against_reach, kept_part.norm(), kept_part.norm() < smallest_kept_share ? ", skipped" : "");
			if (kept_part.norm() < smallest_kept_share)
				return false;
			direction = kept_part.normalized();
		}
		const PlacementIkSolutions candidate = this->solveIkForAllViewpoints(this->clampToSearchBox(solution.placement_ik.object_position + object_step * direction), this->convertIkSolutionsToVectorOfJoints(solution.placement_ik), {});
		int num_lost = 0;
		for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
			num_lost += solution.placement_ik.is_reached[viewpoint_index] && !candidate.is_reached[viewpoint_index] ? 1 : 0;
		// Same poses carried with the object, so before and after compare like with like.
		const std::vector<room_margins::Margin> margins_after = this->getMarginsOfCarriedPoses(solution.placement_ik, missed_viewpoints, object_step * direction, miss_blocking_multiple);
		const double soft_min_after = room_margins::SoftMin(margins_after);
		const bool loss_allowed = static_cast<double>(num_lost) <= this->settings.cost_weights.allowed_reach_loss * static_cast<double>(solution.placement_ik.num_reached);
		const bool kept = loss_allowed && (candidate.num_reached > solution.placement_ik.num_reached || soft_min_after > soft_min_before + 1e-3);
		RCLCPP_INFO(this->node->get_logger(), "iteration %d room-then-reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu (lost %d)  misses %zu  soft-min margin %.2f -> %.2f floors  reached kept clear: %s%s", iteration, object_step, candidate.object_position.x(), candidate.object_position.y(), candidate.object_position.z(), candidate.num_reached, this->viewpoints.size(), num_lost, missed_viewpoints.size(), soft_min_before, soft_min_after, keeps_reached_clear ? "yes" : "no", kept ? "  <-- kept" : "");
		if (!kept)
			continue;
		solution.placement_ik = candidate;
		this->recordObjectMove(solution, direction);
		return true;
	}
	return false;
}

// KME on reconfigurations: kept if none are lost and reconfigurations drop or free up.
bool Optimizer::expandMarginsOfReconfigurations(TourSolution& solution, double& object_step, int iteration)
{
	const std::vector<room_margins::Margin> margins_before = this->getMarginsOfReconfigurations(solution, miss_blocking_multiple);
	const double tightest_before = room_margins::Tightest(margins_before);
	for (int attempt = 0; attempt < this->settings.step_sizes.max_line_search_tries && object_step >= this->settings.step_sizes.min_step; ++attempt, object_step *= 0.5)
	{
		bool keeps_reached_clear = false;
		const Eigen::Vector3d direction = this->computeKmeDirection(solution.placement_ik, margins_before, object_step, keeps_reached_clear);
		if (direction.isZero())
			break;
		const TourSolution candidate = this->solveTourAtPosition(solution.placement_ik.object_position + object_step * direction, this->convertIkSolutionsToVectorOfJoints(solution.placement_ik), {}, solution.order);
		const std::vector<room_margins::Margin> margins_after = candidate.num_reconfigurations > 0 ? this->getMarginsOfReconfigurations(candidate, miss_blocking_multiple) : std::vector<room_margins::Margin>();
		const double tightest_after = margins_after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(margins_after);
		const bool keeps_all_reached = candidate.placement_ik.num_reached == solution.placement_ik.num_reached;
		const bool fewer = candidate.num_reconfigurations < solution.num_reconfigurations;
		const bool as_many_but_freer = candidate.num_reconfigurations == solution.num_reconfigurations && tightest_after > tightest_before + 1e-3;
		const bool kept = keeps_all_reached && (fewer || as_many_but_freer);
		RCLCPP_INFO(this->node->get_logger(), "iteration %d reconfiguration room: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu  reconfigurations %d vs %d  tightest margin %.2f -> %.2f floors  tour length %.2f  reached kept clear: %s%s", iteration, object_step, candidate.placement_ik.object_position.x(), candidate.placement_ik.object_position.y(), candidate.placement_ik.object_position.z(), candidate.placement_ik.num_reached, this->viewpoints.size(), candidate.num_reconfigurations, solution.num_reconfigurations, tightest_before, tightest_after, candidate.tour_length, keeps_reached_clear ? "yes" : "no", kept ? "  <-- kept" : "");
		if (!kept)
			continue;
		solution = candidate;
		this->recordObjectMove(solution, direction);
		object_step = std::min(this->settings.step_sizes.max_step, object_step * 1.5);
		return true;
	}
	object_step = this->settings.step_sizes.initial_step;
	return false;
}

// Pose slides along self-motion, with the object held still.
bool Optimizer::runPoseSlideStep(TourSolution& solution, int iteration)
{
	const std::vector<Eigen::VectorXd> slides = this->computePoseSlidesDownTourLength(solution);
	const bool has_slide = std::any_of(slides.begin(), slides.end(), [](const Eigen::VectorXd& slide) { return slide.size() > 0; });
	double slide_scale = this->settings.step_sizes.initial_slide_scale_deg * radians_per_degree;
	for (int attempt = 0; attempt < this->settings.step_sizes.max_line_search_tries && has_slide; ++attempt, slide_scale *= 0.5)
	{
		// Misses did not slide and the object stayed: keep their IK.
		const TourSolution candidate = this->improveTourForPlacement(this->solveIkForAllViewpoints(solution.placement_ik.object_position, this->slideArmPoses(solution.placement_ik, slides, slide_scale), this->convertIkSolutionsToVectorOfJoints(solution.placement_ik), &solution.placement_ik), solution.order);
		const bool kept = candidate.total_cost < solution.total_cost - this->settings.step_sizes.min_slide_gain;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d poses: slide scale %.2f deg  reached %d/%zu  reconfigurations %d  blocked edges %d  tour length %.1f deg  smallest margin %.2f deg  cost %.2f vs %.2f%s", iteration, slide_scale * degrees_per_radian, candidate.placement_ik.num_reached, this->viewpoints.size(), candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.tour_length * degrees_per_radian, candidate.smallest_joint_margin * degrees_per_radian, candidate.total_cost, solution.total_cost, kept ? "  <-- kept" : "");
		if (!kept)
			continue;
		solution = candidate;
		return true;
	}
	return false;
}

// CPD: object step down the tour length, adding no reconfigurations.
bool Optimizer::runCpdStep(TourSolution& solution, double& object_step, int iteration)
{
	const Eigen::Vector3d direction = this->computeCpdDirection(solution);
	if (direction.isZero())
		return false;
	for (int attempt = 0; attempt < this->settings.step_sizes.max_line_search_tries && object_step >= this->settings.step_sizes.min_step; ++attempt, object_step *= 0.5)
	{
		const TourSolution candidate = this->solveTourAtPosition(solution.placement_ik.object_position + object_step * direction, this->convertIkSolutionsToVectorOfJoints(solution.placement_ik), {}, solution.order);
		const bool adds_reconfiguration = this->settings.method_switches.use_kme && this->isAllReached(solution.placement_ik) && candidate.num_reconfigurations > solution.num_reconfigurations;
		const bool kept = candidate.total_cost < solution.total_cost - 1e-9 && !adds_reconfiguration;
		RCLCPP_INFO(this->node->get_logger(), "iteration %d tour: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d/%zu  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.1f deg  smallest margin %.2f deg  cost %.2f vs %.2f%s", iteration, object_step, candidate.placement_ik.object_position.x(), candidate.placement_ik.object_position.y(), candidate.placement_ik.object_position.z(), candidate.placement_ik.num_reached, this->viewpoints.size(), candidate.placement_ik.gap_sum, candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.tour_length * degrees_per_radian, candidate.smallest_joint_margin * degrees_per_radian, candidate.total_cost, solution.total_cost, kept ? "  <-- kept" : "");
		if (!kept)
			continue;
		solution = candidate;
		this->recordObjectMove(solution, direction);
		object_step = std::min(this->settings.step_sizes.max_step, object_step * 1.5);
		return true;
	}
	object_step = this->settings.step_sizes.initial_step;
	return false;
}

void Optimizer::recordObjectMove(const TourSolution& solution, const Eigen::Vector3d& direction)
{
	this->object_trail.push_back(solution.placement_ik.object_position);
	this->publishProgressMarkers(solution.placement_ik, solution.order, direction);
}

// ---------------------------------------------------------------- Tour scoring

TourSolution Optimizer::solveTourAtPosition(const Eigen::Vector3d& object_position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const std::vector<int>& order)
{
	return this->improveTourForPlacement(this->solveIkForAllViewpoints(this->clampToSearchBox(object_position), start_poses, fallback_start_poses), order);
}

TourSolution Optimizer::improveTourForPlacement(const PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	TourSolution solution;
	solution.placement_ik = placement_ik;
	// No order yet: an empty tour would score as free.
	solution.order = order.empty() ? this->reorderTourWithGtsp(solution.placement_ik, {}) : order;
	this->runGtspAndScore(solution);
	return solution;
}

// Scores the given order and a GTSP reorder; keeps the better.
void Optimizer::runGtspAndScore(TourSolution& solution)
{
	TourSolution reordered = solution;
	this->refinePosesAndScore(solution);
	reordered.order = this->reorderTourWithGtsp(reordered.placement_ik, reordered.order);
	this->refinePosesAndScore(reordered);
	const bool reaches_more = reordered.placement_ik.num_reached > solution.placement_ik.num_reached;
	const bool cheaper = reordered.placement_ik.num_reached == solution.placement_ik.num_reached && reordered.total_cost < solution.total_cost;
	if (reaches_more || cheaper)
		solution = reordered;
}

void Optimizer::refinePosesAndScore(TourSolution& solution)
{
	this->unwrapFullTurnsAlongTour(solution.placement_ik, solution.order);
	// Misses first: following camera paths may change poses after.
	this->bringMissesCloserFromNeighbors(solution.placement_ik, solution.order);
	solution.num_reconfigurations = this->followTourAlongStraightCameraPaths(solution.placement_ik, solution.order);
	double blocked_charge = 0.0;
	const std::vector<std::pair<size_t, double>> blocked_edges = this->getBlockedEdgeCharges(solution.placement_ik, solution.order);
	for (const std::pair<size_t, double>& edge : blocked_edges)
		blocked_charge += edge.second;
	solution.num_blocked_edges = static_cast<int>(blocked_edges.size());
	std::vector<Joints> poses_from_home = this->getPosesInTourOrder(solution.placement_ik, solution.order);
	poses_from_home.insert(poses_from_home.begin(), this->home_pose);
	solution.tour_length = joint_travel::TravelCost(poses_from_home, this->settings.tour_settings.edge_weights, this->counted_joints) + blocked_charge;
	std::vector<Joints> reached_poses;
	for (size_t viewpoint_index : this->getViewpointsWithReach(solution.placement_ik, true))
		reached_poses.push_back(solution.placement_ik.ik_solutions[viewpoint_index].joints);
	solution.smallest_joint_margin = this->computeSmallestJointMargin(reached_poses);
	const CostWeights& weights = this->settings.cost_weights;
	solution.total_cost = solution.tour_length + weights.gap_penalty * solution.placement_ik.gap_sum + weights.reconfiguration_cost * solution.num_reconfigurations;
}

// ---------------------------------------------------------------- Step directions

// Least-squares object move onto the misses' cameras: minus the mean position error.
Eigen::Vector3d Optimizer::computeMoveTowardMisses(const PlacementIkSolutions& placement_ik) const
{
	Eigen::Vector3d move = Eigen::Vector3d::Zero();
	int num_missed = 0;
	for (size_t viewpoint_index : this->getViewpointsWithReach(placement_ik, false))
	{
		move -= placement_ik.ik_solutions[viewpoint_index].position_error;
		++num_missed;
	}
	return num_missed > 0 ? Eigen::Vector3d(move / num_missed) : Eigen::Vector3d::Zero();
}

// Unit object direction down the tour length; poses follow by IK.
Eigen::Vector3d Optimizer::computeCpdDirection(const TourSolution& solution)
{
	const std::vector<Eigen::VectorXd> length_gradients = this->computeTourLengthGradientPerViewpoint(solution.placement_ik, solution.order);
	Eigen::Vector3d move = Eigen::Vector3d::Zero();
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		const Eigen::MatrixXd jacobian = closest_ik::GetToolJacobian(*this->state, this->group, this->tool, solution.placement_ik.ik_solutions[viewpoint_index].joints, this->settings.ik_settings.exact_solve_settings.ignore_roll);
		const Eigen::MatrixXd joint_rate_per_object_move = scene_geometry::DampedPinv(jacobian, this->settings.ik_settings.jacobian_damping).leftCols<3>();
		move -= joint_rate_per_object_move.transpose() * length_gradients[viewpoint_index];
	}
	return move.norm() > 1e-12 ? Eigen::Vector3d(move.normalized()) : Eigen::Vector3d::Zero();
}

// Self-motion slide per reached pose, down the tour length; largest is 1.
std::vector<Eigen::VectorXd> Optimizer::computePoseSlidesDownTourLength(const TourSolution& solution)
{
	std::vector<Eigen::VectorXd> slides(this->viewpoints.size());
	const std::vector<Eigen::VectorXd> length_gradients = this->computeTourLengthGradientPerViewpoint(solution.placement_ik, solution.order);
	double largest_slide = 0.0;
	for (size_t viewpoint_index : this->getViewpointsWithReach(solution.placement_ik, true))
	{
		const Eigen::MatrixXd jacobian = closest_ik::GetToolJacobian(*this->state, this->group, this->tool, solution.placement_ik.ik_solutions[viewpoint_index].joints, this->settings.ik_settings.exact_solve_settings.ignore_roll);
		slides[viewpoint_index] = -GetSelfMotionProjector(jacobian) * length_gradients[viewpoint_index];
		largest_slide = std::max(largest_slide, slides[viewpoint_index].norm());
	}
	for (Eigen::VectorXd& slide : slides)
		if (slide.size() > 0)
			slide = largest_slide > 1e-9 ? Eigen::VectorXd(slide / largest_slide) : Eigen::VectorXd();
	return slides;
}

// Per viewpoint: gradient of the tour length with respect to its joints.
std::vector<Eigen::VectorXd> Optimizer::computeTourLengthGradientPerViewpoint(const PlacementIkSolutions& placement_ik, const std::vector<int>& order) const
{
	const Eigen::Index num_joints = static_cast<Eigen::Index>(this->home_pose.size());
	std::vector<Eigen::VectorXd> gradients(this->viewpoints.size(), Eigen::VectorXd::Zero(num_joints));
	const double delta = 1e-5;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
	{
		const size_t to_viewpoint = static_cast<size_t>(order[tour_position]);
		const Joints& from_pose = tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints;
		// Central differences; the edge cost depends on to - from only.
		Eigen::VectorXd edge_gradient = Eigen::VectorXd::Zero(num_joints);
		Joints shifted = placement_ik.ik_solutions[to_viewpoint].joints;
		for (size_t joint_index = 0; joint_index < shifted.size(); ++joint_index)
		{
			const double value = shifted[joint_index];
			shifted[joint_index] = value + delta;
			const double cost_up = joint_travel::EdgeCost(from_pose, shifted, this->settings.tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value - delta;
			const double cost_down = joint_travel::EdgeCost(from_pose, shifted, this->settings.tour_settings.edge_weights, this->counted_joints);
			shifted[joint_index] = value;
			edge_gradient[static_cast<Eigen::Index>(joint_index)] = (cost_up - cost_down) / (2.0 * delta);
		}
		gradients[to_viewpoint] += edge_gradient;
		if (tour_position > 0)
			gradients[static_cast<size_t>(order[tour_position - 1])] -= edge_gradient;
	}
	return gradients;
}

std::vector<Joints> Optimizer::slideArmPoses(const PlacementIkSolutions& placement_ik, const std::vector<Eigen::VectorXd>& slides, double slide_scale) const
{
	std::vector<Joints> slid_poses = this->convertIkSolutionsToVectorOfJoints(placement_ik);
	const double margin = this->settings.step_sizes.slide_joint_margin_deg * radians_per_degree;
	for (size_t viewpoint_index = 0; viewpoint_index < slid_poses.size(); ++viewpoint_index)
	{
		if (slides[viewpoint_index].size() == 0)
			continue;
		for (size_t joint_index = 0; joint_index < slid_poses[viewpoint_index].size(); ++joint_index)
		{
			const moveit::core::VariableBounds& bounds = this->getJointBounds(joint_index);
			double& value = slid_poses[viewpoint_index][joint_index];
			const double before = value;
			value += slide_scale * slides[viewpoint_index][static_cast<Eigen::Index>(joint_index)];
			// Stay off the limits, but never push a joint further out.
			if (bounds.position_bounded_)
				value = std::clamp(value, std::min(before, bounds.min_position_ + margin), std::max(before, bounds.max_position_ - margin));
		}
	}
	return slid_poses;
}

// Direction that most raises the margins, trimmed so reached viewpoints keep theirs if possible.
Eigen::Vector3d Optimizer::computeKmeDirection(const PlacementIkSolutions& placement_ik, const std::vector<room_margins::Margin>& margins, double object_step, bool& keeps_reached_clear)
{
	keeps_reached_clear = false;
	if (margins.empty())
		return Eigen::Vector3d::Zero();
	const Eigen::Vector3d room_direction = room_margins::MostRoomDirection(margins, object_step);
	if (room_direction.isZero())
		return Eigen::Vector3d::Zero();
	const std::vector<room_margins::Margin> reached_margins = this->getMarginsOfViewpoints(placement_ik, this->getViewpointsWithReach(placement_ik, true), reached_blocking_multiple);
	const Eigen::Vector3d trimmed_direction = this->removePartsTighteningMargins(room_direction, reached_margins);
	keeps_reached_clear = trimmed_direction.norm() >= smallest_kept_share;
	return keeps_reached_clear ? Eigen::Vector3d(trimmed_direction.normalized()) : room_direction;
}

// Removes the parts of move that tighten any margin, repeatedly.
Eigen::Vector3d Optimizer::removePartsTighteningMargins(const Eigen::Vector3d& move, const std::vector<room_margins::Margin>& margins) const
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

// ---------------------------------------------------------------- KME margins

// Joint-limit, stretch and collision margins of one pose that are near blocking.
std::vector<room_margins::Margin> Optimizer::getMarginsOfPose(size_t viewpoint_index, const Joints& joints, double blocking_multiple)
{
	const int viewpoint = static_cast<int>(viewpoint_index);
	const int num_joints = static_cast<int>(joints.size());
	const Eigen::MatrixXd joint_rate_per_object_move = scene_geometry::DampedPinv(manipulability::ToolJacobian(*this->state, this->group, this->tool, joints), this->settings.ik_settings.jacobian_damping).leftCols<3>();
	std::vector<room_margins::Margin> margins;
	for (int joint_index = 0; joint_index < num_joints; ++joint_index)
	{
		const moveit::core::VariableBounds& bounds = this->getJointBounds(static_cast<size_t>(joint_index));
		if (!bounds.position_bounded_)
			continue;
		const Eigen::VectorXd joint_direction = Eigen::VectorXd::Unit(num_joints, joint_index);
		const std::string joint_name = this->group->getActiveJointModels()[static_cast<size_t>(joint_index)]->getName();
		const double room_to_lower_limit = joints[static_cast<size_t>(joint_index)] - bounds.min_position_;
		const double room_to_upper_limit = bounds.max_position_ - joints[static_cast<size_t>(joint_index)];
		if (room_to_lower_limit < blocking_multiple * joint_safe_distance)
			margins.push_back({viewpoint, joint_name + " low limit", room_to_lower_limit, joint_safe_distance, joint_rate_per_object_move.transpose() * joint_direction, joint_direction});
		if (room_to_upper_limit < blocking_multiple * joint_safe_distance)
			margins.push_back({viewpoint, joint_name + " high limit", room_to_upper_limit, joint_safe_distance, joint_rate_per_object_move.transpose() * -joint_direction, -joint_direction});
	}
	const double stretch = Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(*this->state, this->group, this->tool, joints)).singularValues().minCoeff();
	if (stretch < blocking_multiple * stretch_safe_distance)
	{
		const double delta = 1e-5;  // rad
		Eigen::VectorXd stretch_slope(num_joints);
		for (int joint_index = 0; joint_index < num_joints; ++joint_index)
		{
			Joints shifted = joints;
			shifted[static_cast<size_t>(joint_index)] += delta;
			stretch_slope(joint_index) = (Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(*this->state, this->group, this->tool, shifted)).singularValues().minCoeff() - stretch) / delta;
		}
		margins.push_back({viewpoint, "stretch", stretch, stretch_safe_distance, joint_rate_per_object_move.transpose() * stretch_slope, stretch_slope});
	}
	const double window = this->settings.ik_settings.closest_solve_settings.margin + blocking_multiple * collision_safe_distance;  // m
	this->state->setJointGroupPositions(this->group, joints);
	this->state->update();
	const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, window), window);
	for (size_t pair_index = 0; pair_index < close_pairs.dists.size(); ++pair_index)
	{
		const Eigen::VectorXd joint_slope = close_pairs.gradients[pair_index].transpose();
		margins.push_back({viewpoint, "collision " + close_pairs.names[pair_index], close_pairs.dists[pair_index], collision_safe_distance, joint_rate_per_object_move.transpose() * joint_slope + close_pairs.object_slopes[pair_index], joint_slope});
	}
	return margins;
}

std::vector<room_margins::Margin> Optimizer::getMarginsOfViewpoints(const PlacementIkSolutions& placement_ik, const std::vector<size_t>& viewpoint_indices, double blocking_multiple)
{
	this->placeObjectAt(placement_ik.object_position);
	std::vector<room_margins::Margin> margins;
	for (size_t viewpoint_index : viewpoint_indices)
	{
		const std::vector<room_margins::Margin> pose_margins = this->getMarginsOfPose(viewpoint_index, placement_ik.ik_solutions[viewpoint_index].joints, blocking_multiple);
		margins.insert(margins.end(), pose_margins.begin(), pose_margins.end());
	}
	return margins;
}

// Margins after the object moves, poses carried along to first order.
std::vector<room_margins::Margin> Optimizer::getMarginsOfCarriedPoses(const PlacementIkSolutions& placement_ik, const std::vector<size_t>& viewpoint_indices, const Eigen::Vector3d& object_move, double blocking_multiple)
{
	this->placeObjectAt(placement_ik.object_position + object_move);
	std::vector<room_margins::Margin> margins;
	for (size_t viewpoint_index : viewpoint_indices)
	{
		const Joints& joints = placement_ik.ik_solutions[viewpoint_index].joints;
		const Joints carried_joints = AddToJoints(joints, scene_geometry::DampedPinv(manipulability::ToolJacobian(*this->state, this->group, this->tool, joints), this->settings.ik_settings.jacobian_damping).leftCols<3>() * object_move);
		const std::vector<room_margins::Margin> pose_margins = this->getMarginsOfPose(viewpoint_index, carried_joints, blocking_multiple);
		margins.insert(margins.end(), pose_margins.begin(), pose_margins.end());
	}
	return margins;
}

// Margins where the straight camera paths stopped, labelled with the edge's target viewpoint.
std::vector<room_margins::Margin> Optimizer::getMarginsOfReconfigurations(const TourSolution& solution, double blocking_multiple)
{
	std::vector<Joints> end_poses;
	const std::vector<size_t> reconfiguration_edges = this->getReconfigurationEdges(solution.placement_ik, solution.order, &end_poses);
	std::vector<room_margins::Margin> margins;
	for (size_t tour_position : reconfiguration_edges)
	{
		if (end_poses[tour_position].empty())
			continue;
		const std::vector<room_margins::Margin> edge_margins = this->getMarginsOfPose(static_cast<size_t>(solution.order[tour_position]), end_poses[tour_position], blocking_multiple);
		margins.insert(margins.end(), edge_margins.begin(), edge_margins.end());
	}
	return margins;
}

// ---------------------------------------------------------------- IK

// Exact IK per viewpoint from given then random starts; else closest IK.
PlacementIkSolutions Optimizer::solveIkForAllViewpoints(const Eigen::Vector3d& object_position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const PlacementIkSolutions* kept_misses)
{
	this->placeObjectAt(object_position);
	const IkSettings& ik_settings = this->settings.ik_settings;
	const size_t num_viewpoints = this->viewpoints.size();
	PlacementIkSolutions placement_ik;
	placement_ik.object_position = object_position;
	placement_ik.ik_solutions.assign(num_viewpoints, IkSolution());
	for (size_t viewpoint_index = 0; viewpoint_index < num_viewpoints; ++viewpoint_index)
		placement_ik.ik_solutions[viewpoint_index].joints = start_poses[viewpoint_index];
	placement_ik.is_reached.assign(num_viewpoints, false);
	placement_ik.closest_ik_candidates.assign(num_viewpoints, {});
	this->runInParallel(num_viewpoints, [&](moveit::core::RobotState& thread_state, size_t viewpoint_index)
	{
		if (kept_misses != nullptr && !kept_misses->is_reached[viewpoint_index])
		{
			placement_ik.ik_solutions[viewpoint_index] = kept_misses->ik_solutions[viewpoint_index];
			placement_ik.closest_ik_candidates[viewpoint_index] = kept_misses->closest_ik_candidates[viewpoint_index];
			return;
		}
		const Eigen::Isometry3d target = this->getViewpointTarget(object_position, viewpoint_index);
		closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, start_poses[viewpoint_index], ik_settings.exact_solve_settings);
		if (!result.exact && viewpoint_index < fallback_start_poses.size())
			result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, fallback_start_poses[viewpoint_index], ik_settings.exact_solve_settings);
		// Seeded by viewpoint and object position, so a rerun gives the same starts.
		random_numbers::RandomNumberGenerator viewpoint_rng(this->getRandomIkSeed(object_position, viewpoint_index));
		std::vector<Joints> random_starts;
		for (int attempt = 0; attempt < ik_settings.exact_ik_random_attempts && !result.exact; ++attempt)
		{
			Joints random_start;
			thread_state.setToRandomPositions(this->group, viewpoint_rng);
			thread_state.copyJointGroupPositions(this->group, random_start);
			random_starts.push_back(random_start);
			result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, random_start, ik_settings.exact_solve_settings);
		}
		if (!result.exact)
		{
			std::vector<Joints> closest_starts = {start_poses[viewpoint_index]};
			for (size_t random_index = 0; random_index < random_starts.size() && static_cast<int>(closest_starts.size()) <= ik_settings.closest_ik_random_attempts; ++random_index)
				closest_starts.push_back(random_starts[random_index]);
			const std::vector<closest_ik::Result> closest_results = this->solveClosestIkFromStarts(thread_state, target, closest_starts, true);
			result = closest_ik::Result();
			for (const closest_ik::Result& closest_result : closest_results)
				if (closest_result.exact)
					result = closest_result;
			if (!result.exact && !closest_results.empty())
			{
				placement_ik.closest_ik_candidates[viewpoint_index] = this->keepSolutionsNearBestGap(thread_state, closest_results);
				result = closest_ik::Result();
				result.found = true;
				result.q = placement_ik.closest_ik_candidates[viewpoint_index].front().joints;
			}
		}
		if (!result.found)
		{
			// Nothing collision-free near: keep the start pose with a large gap.
			placement_ik.ik_solutions[viewpoint_index].gap = ik_settings.closest_solve_settings.margin + 1.0;
			return;
		}
		placement_ik.is_reached[viewpoint_index] = result.exact;
		if (result.exact)
			placement_ik.ik_solutions[viewpoint_index].joints = result.q;
		else
			placement_ik.ik_solutions[viewpoint_index] = placement_ik.closest_ik_candidates[viewpoint_index].front();
	});
	for (size_t viewpoint_index = 0; viewpoint_index < num_viewpoints; ++viewpoint_index)
	{
		placement_ik.num_reached += placement_ik.is_reached[viewpoint_index] ? 1 : 0;
		placement_ik.gap_sum += placement_ik.ik_solutions[viewpoint_index].gap;
	}
	return placement_ik;
}

// Closest IK per start; then clearance push, or an exact finish if near.
std::vector<closest_ik::Result> Optimizer::solveClosestIkFromStarts(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts, bool push_for_clearance)
{
	std::vector<closest_ik::Result> results;
	for (const Joints& start : starts)
	{
		closest_ik::Result result = closest_ik::Solve(this->scene, state, this->group, this->tool, target, start, this->settings.ik_settings.closest_solve_settings);
		if (!result.found)
			continue;
		const bool short_of_clearance = this->computeClearanceShortfall(state, result.q) > 0.0;
		if (short_of_clearance && push_for_clearance)
			result = this->pushArmToExactClearance(state, target, result);
		else if (!short_of_clearance && !result.exact && result.gap < this->settings.ik_settings.max_closest_ik_gap_to_retry_exact_ik)
		{
			const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, result.q, this->settings.ik_settings.exact_solve_settings);
			if (exact.exact)
				result = exact;
		}
		results.push_back(result);
		if (result.exact)
			break;
	}
	return results;
}

// Moves the arm out to the exact clearance along self-motion while tracking the target.
closest_ik::Result Optimizer::pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest_result) const
{
	const IkSettings& ik_settings = this->settings.ik_settings;
	const double margin = ik_settings.exact_solve_settings.exact_margin;
	if (margin <= 0.0)
		return closest_result;
	const double max_joint_step = 0.05;  // rad
	const int max_iterations = 40;
	Joints joints = closest_result.q;
	for (int iteration = 0; iteration < max_iterations; ++iteration)
	{
		state.setJointGroupPositions(this->group, joints);
		state.update();
		const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, margin), margin);
		Eigen::VectorXd clearance_push = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joints.size()));
		for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
		{
			if (closest_ik::PairHasAnyLink(close_pairs.names[pair_index], this->camera_side_links))
				continue;
			const Eigen::VectorXd gradient = close_pairs.gradients[pair_index].transpose();
			clearance_push += (margin - close_pairs.dists[pair_index]) * gradient / (gradient.squaredNorm() + 1e-9);
		}
		if (clearance_push.norm() < 1e-9)
			break;
		const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), ik_settings.exact_solve_settings.ignore_roll);
		const Eigen::MatrixXd jacobian = closest_ik::GetToolJacobian(state, this->group, this->tool, joints, ik_settings.exact_solve_settings.ignore_roll);
		const Eigen::MatrixXd pseudo_inverse = scene_geometry::DampedPinv(jacobian, ik_settings.jacobian_damping);
		const Eigen::MatrixXd self_motion = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - pseudo_inverse * jacobian;
		Eigen::VectorXd joint_step = pseudo_inverse * pose_error + self_motion * clearance_push;
		if (joint_step.cwiseAbs().maxCoeff() > max_joint_step)
			joint_step *= max_joint_step / joint_step.cwiseAbs().maxCoeff();
		joints = AddToJoints(joints, joint_step);
		state.setJointGroupPositions(this->group, joints);
		state.enforceBounds(this->group);
		state.copyJointGroupPositions(this->group, joints);
	}
	const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, joints, ik_settings.exact_solve_settings);
	if (exact.exact)
		return exact;
	if (!scene_geometry::IsStateValid(this->scene, &state, this->group, joints.data()))
		return closest_result;
	state.setJointGroupPositions(this->group, joints);
	state.update();
	closest_ik::Result pushed;
	pushed.found = true;
	pushed.exact = false;
	pushed.pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), ik_settings.exact_solve_settings.ignore_roll);
	pushed.gap = scene_geometry::PoseGap(pushed.pose_error, ik_settings.closest_solve_settings.rot_scale);
	pushed.q = joints;
	pushed.weighted_cost = closest_result.weighted_cost;
	const double pushed_gap = pushed.gap + this->computeClearanceShortfall(state, joints);
	const double closest_gap = scene_geometry::PoseGap(closest_result.pose_error, ik_settings.closest_solve_settings.rot_scale) + this->computeClearanceShortfall(state, closest_result.q);
	return pushed_gap < closest_gap ? pushed : closest_result;
}

// Closest poses within max_gap_above_best_closest_ik_pose of the best.
std::vector<IkSolution> Optimizer::keepSolutionsNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest_results) const
{
	const IkSettings& ik_settings = this->settings.ik_settings;
	std::vector<IkSolution> all_solutions;
	double best_gap = std::numeric_limits<double>::max();
	for (const closest_ik::Result& result : closest_results)
	{
		const IkSolution pose = this->scoreIkResult(state, result);
		best_gap = std::min(best_gap, pose.gap);
		all_solutions.push_back(pose);
	}
	if (ik_settings.closest_solve_settings.use_weighted_cost)
	{
		// The weighted cost already counts room and clearance: start from its cheapest.
		size_t cheapest_index = 0;
		for (size_t result_index = 1; result_index < closest_results.size(); ++result_index)
			if (closest_results[result_index].weighted_cost < closest_results[cheapest_index].weighted_cost)
				cheapest_index = result_index;
		std::vector<IkSolution> near_solutions = {all_solutions[cheapest_index]};
		for (size_t pose_index = 0; pose_index < all_solutions.size(); ++pose_index)
			if (pose_index != cheapest_index && all_solutions[pose_index].gap <= all_solutions[cheapest_index].gap + ik_settings.max_gap_above_best_closest_ik_pose)
				near_solutions.push_back(all_solutions[pose_index]);
		return near_solutions;
	}
	std::vector<IkSolution> near_solutions;
	for (const IkSolution& pose : all_solutions)
		if (pose.gap <= best_gap + ik_settings.max_gap_above_best_closest_ik_pose)
			near_solutions.push_back(pose);
	return near_solutions;
}

IkSolution Optimizer::scoreIkResult(moveit::core::RobotState& state, const closest_ik::Result& result) const
{
	IkSolution pose;
	pose.joints = result.q;
	pose.position_error = result.pose_error.head<3>();
	pose.angle_error = result.pose_error.tail<3>().norm();
	pose.clearance_shortfall = this->computeClearanceShortfall(state, result.q);
	pose.gap = scene_geometry::PoseGap(result.pose_error, this->settings.ik_settings.closest_solve_settings.rot_scale) + pose.clearance_shortfall;
	return pose;
}

// How far the arm (camera side aside) is inside the exact clearance.
double Optimizer::computeClearanceShortfall(moveit::core::RobotState& state, const Joints& joints) const
{
	const double exact_margin = this->settings.ik_settings.exact_solve_settings.exact_margin;
	if (exact_margin <= 0.0)
		return 0.0;
	state.setJointGroupPositions(this->group, joints);
	state.update();
	const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, exact_margin), exact_margin);
	double clearance = exact_margin;
	for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
		if (!closest_ik::PairHasAnyLink(close_pairs.names[pair_index], this->camera_side_links))
			clearance = std::min(clearance, close_pairs.dists[pair_index]);
	return std::max(0.0, exact_margin - clearance);
}

uint32_t Optimizer::getRandomIkSeed(const Eigen::Vector3d& object_position, size_t viewpoint_index) const
{
	const Eigen::Vector3d cell = (object_position * 1e4).array().round();
	uint64_t hash = static_cast<uint64_t>(this->settings.ik_settings.random_ik_seed) * 1000003ULL + static_cast<uint64_t>(viewpoint_index) * 7919ULL;
	for (int axis = 0; axis < 3; ++axis)
		hash = hash * 6364136223846793005ULL + static_cast<uint64_t>(static_cast<int64_t>(cell[axis]));
	return static_cast<uint32_t>(hash ^ (hash >> 32));
}

// ---------------------------------------------------------------- Viewpoint order and edges

// GTSP over each viewpoint's pose options, re-solved with blocked and reconfiguration edges charged.
std::vector<int> Optimizer::reorderTourWithGtsp(PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	const std::vector<std::vector<Joints>> options = this->collectIkPoseCandidatesPerViewpointForGtsp(placement_ik, order);
	const TourSettings& tour_settings = this->settings.tour_settings;
	gtsp::Solver solver(options, this->home_pose, tour_settings.edge_weights, tour_settings.two_opt_rounds);
	solver.SetTravelJoints(this->counted_joints);
	gtsp::Tour tour = solver.Solve(order);
	for (int round = 0; round < tour_settings.gtsp_penalty_rounds; ++round)
	{
		PlacementIkSolutions tour_placement = placement_ik;
		for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
			tour_placement.ik_solutions[static_cast<size_t>(tour.order[tour_position])].joints = tour.poses[tour_position];
		std::vector<std::pair<size_t, double>> charged_edges = this->getBlockedEdgeCharges(tour_placement, tour.order);
		for (size_t tour_position : this->getReconfigurationEdges(tour_placement, tour.order))
			charged_edges.push_back({tour_position, this->settings.cost_weights.reconfiguration_cost});
		if (charged_edges.empty())
			break;
		for (const std::pair<size_t, double>& edge : charged_edges)
		{
			const size_t tour_position = edge.first;
			const size_t to_viewpoint = static_cast<size_t>(tour.order[tour_position]);
			const std::pair<int, int> to_key = {static_cast<int>(to_viewpoint), FindPoseIndex(options[to_viewpoint], tour.poses[tour_position])};
			const std::pair<int, int> from_key = tour_position == 0 ? std::make_pair(-1, -1) : std::make_pair(tour.order[tour_position - 1], FindPoseIndex(options[static_cast<size_t>(tour.order[tour_position - 1])], tour.poses[tour_position - 1]));
			solver.AddEdgeCost(from_key, to_key, edge.second);
		}
		tour = solver.Solve(tour.order);
	}
	// Take the tour's poses; a missed viewpoint also takes its candidate's gap.
	for (size_t tour_position = 0; tour_position < tour.order.size(); ++tour_position)
	{
		const size_t viewpoint_index = static_cast<size_t>(tour.order[tour_position]);
		placement_ik.ik_solutions[viewpoint_index].joints = tour.poses[tour_position];
		if (placement_ik.is_reached[viewpoint_index])
			continue;
		for (const IkSolution& candidate : placement_ik.closest_ik_candidates[viewpoint_index])
		{
			if (candidate.joints != tour.poses[tour_position])
				continue;
			placement_ik.gap_sum += candidate.gap - placement_ik.ik_solutions[viewpoint_index].gap;
			placement_ik.ik_solutions[viewpoint_index] = candidate;
		}
	}
	return tour.order;
}

// Per viewpoint: its pose, closest options, IK from neighbours and reference postures.
std::vector<std::vector<Joints>> Optimizer::collectIkPoseCandidatesPerViewpointForGtsp(PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	const size_t num_viewpoints = this->viewpoints.size();
	const closest_ik::SolveSettings& exact_solve_settings = this->settings.ik_settings.exact_solve_settings;
	std::vector<std::vector<Joints>> options(num_viewpoints);
	for (size_t viewpoint_index = 0; viewpoint_index < num_viewpoints; ++viewpoint_index)
	{
		const Joints& chosen_joints = placement_ik.ik_solutions[viewpoint_index].joints;
		options[viewpoint_index].push_back(chosen_joints);
		if (placement_ik.is_reached[viewpoint_index])
			continue;
		for (IkSolution& candidate : placement_ik.closest_ik_candidates[viewpoint_index])
		{
			this->unwrapFullTurns(candidate.joints, chosen_joints);
			if (candidate.joints != chosen_joints)
				options[viewpoint_index].push_back(candidate.joints);
		}
	}
	if (order.size() == num_viewpoints)
	{
		this->placeObjectAt(placement_ik.object_position);
		this->runInParallel(order.size(), [&](moveit::core::RobotState& thread_state, size_t tour_position)
		{
			const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
			if (!placement_ik.is_reached[viewpoint_index])
				return;
			const Eigen::Isometry3d target = this->getViewpointTarget(placement_ik.object_position, viewpoint_index);
			std::vector<const Joints*> neighbor_poses = {&(tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints)};
			if (tour_position + 1 < order.size())
				neighbor_poses.push_back(&placement_ik.ik_solutions[static_cast<size_t>(order[tour_position + 1])].joints);
			for (const Joints* neighbor_pose : neighbor_poses)
			{
				const closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *neighbor_pose, exact_solve_settings);
				if (result.exact)
					options[viewpoint_index].push_back(result.q);
			}
		});
	}
	if (!this->reference_postures.empty())
	{
		this->placeObjectAt(placement_ik.object_position);
		const double same_pose_distance = 0.1;  // rad
		this->runInParallel(num_viewpoints, [&](moveit::core::RobotState& thread_state, size_t viewpoint_index)
		{
			if (!placement_ik.is_reached[viewpoint_index])
				return;
			const Eigen::Isometry3d target = this->getViewpointTarget(placement_ik.object_position, viewpoint_index);
			for (const Joints& reference_posture : this->reference_postures)
			{
				closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, reference_posture, exact_solve_settings);
				if (!result.exact)
					continue;
				this->unwrapFullTurns(result.q, placement_ik.ik_solutions[viewpoint_index].joints);
				const bool is_new = std::none_of(options[viewpoint_index].begin(), options[viewpoint_index].end(), [&](const Joints& option) { return joint_travel::JointDistance(option, result.q, this->counted_joints) < same_pose_distance; });
				if (is_new)
					options[viewpoint_index].push_back(result.q);
			}
		});
	}
	for (size_t viewpoint_index = 0; viewpoint_index < num_viewpoints; ++viewpoint_index)
		for (Joints& option : options[viewpoint_index])
			this->unwrapFullTurns(option, placement_ik.ik_solutions[viewpoint_index].joints);
	return options;
}

// Each miss tries IK from its tour neighbours; a smaller gap wins.
void Optimizer::bringMissesCloserFromNeighbors(PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	std::vector<size_t> miss_positions;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (!placement_ik.is_reached[static_cast<size_t>(order[tour_position])])
			miss_positions.push_back(tour_position);
	if (miss_positions.empty())
		return;
	this->placeObjectAt(placement_ik.object_position);
	if (!(this->neighbor_ik_cache_position == placement_ik.object_position))
	{
		this->neighbor_ik_cache.clear();
		this->neighbor_ik_cache_position = placement_ik.object_position;
	}
	// One solve per (miss, neighbour pose), previous neighbour first.
	std::vector<size_t> pair_miss_index;
	std::vector<Joints> pair_start_pose;
	for (size_t miss_index = 0; miss_index < miss_positions.size(); ++miss_index)
	{
		const size_t tour_position = miss_positions[miss_index];
		pair_miss_index.push_back(miss_index);
		pair_start_pose.push_back(tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints);
		if (tour_position + 1 < order.size())
		{
			pair_miss_index.push_back(miss_index);
			pair_start_pose.push_back(placement_ik.ik_solutions[static_cast<size_t>(order[tour_position + 1])].joints);
		}
	}
	std::vector<NeighborIkResult> pair_results(pair_miss_index.size());
	this->runInParallel(pair_miss_index.size(), [&](moveit::core::RobotState& thread_state, size_t pair_index)
	{
		const size_t viewpoint_index = static_cast<size_t>(order[miss_positions[pair_miss_index[pair_index]]]);
		const Joints& start_pose = pair_start_pose[pair_index];
		const std::pair<size_t, Joints> cache_key(viewpoint_index, start_pose);
		{
			std::lock_guard<std::mutex> lock(this->neighbor_ik_cache_mutex);
			const auto entry = this->neighbor_ik_cache.find(cache_key);
			if (entry != this->neighbor_ik_cache.end())
			{
				pair_results[pair_index] = entry->second;
				return;
			}
		}
		const Eigen::Isometry3d target = this->getViewpointTarget(placement_ik.object_position, viewpoint_index);
		closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, start_pose, this->settings.ik_settings.exact_solve_settings);
		if (!result.exact)
		{
			// No clearance push: too slow for every (miss, neighbour) pair.
			const std::vector<closest_ik::Result> closest_results = this->solveClosestIkFromStarts(thread_state, target, {start_pose}, false);
			result = closest_results.empty() ? closest_ik::Result() : closest_results.front();
		}
		NeighborIkResult neighbor_result;
		neighbor_result.found = result.found;
		neighbor_result.exact = result.exact;
		neighbor_result.solution.joints = result.q;
		if (result.found && !result.exact)
			neighbor_result.solution = this->scoreIkResult(thread_state, result);
		pair_results[pair_index] = neighbor_result;
		std::lock_guard<std::mutex> lock(this->neighbor_ik_cache_mutex);
		this->neighbor_ik_cache[cache_key] = neighbor_result;
	});
	// Per miss: a smaller gap wins; an exact pose ends the search.
	std::vector<IkSolution> closer_solutions(miss_positions.size());
	std::vector<char> found(miss_positions.size(), 0);
	std::vector<char> miss_done(miss_positions.size(), 0);
	std::vector<double> best_gap(miss_positions.size());
	for (size_t miss_index = 0; miss_index < miss_positions.size(); ++miss_index)
		best_gap[miss_index] = placement_ik.ik_solutions[static_cast<size_t>(order[miss_positions[miss_index]])].gap;
	for (size_t pair_index = 0; pair_index < pair_miss_index.size(); ++pair_index)
	{
		const size_t miss_index = pair_miss_index[pair_index];
		const NeighborIkResult& neighbor_result = pair_results[pair_index];
		if (miss_done[miss_index] || !neighbor_result.found || neighbor_result.solution.gap >= best_gap[miss_index])
			continue;
		best_gap[miss_index] = neighbor_result.solution.gap;
		closer_solutions[miss_index] = neighbor_result.solution;
		found[miss_index] = 1;
		if (neighbor_result.exact)
			miss_done[miss_index] = 1;
	}
	for (size_t miss_index = 0; miss_index < miss_positions.size(); ++miss_index)
	{
		if (!found[miss_index])
			continue;
		const size_t viewpoint_index = static_cast<size_t>(order[miss_positions[miss_index]]);
		placement_ik.gap_sum += closer_solutions[miss_index].gap - placement_ik.ik_solutions[viewpoint_index].gap;
		placement_ik.ik_solutions[viewpoint_index] = closer_solutions[miss_index];
		if (closer_solutions[miss_index].gap <= 0.0)
		{
			placement_ik.is_reached[viewpoint_index] = true;
			placement_ik.num_reached += 1;
		}
	}
}

// Reached poses on followed edges take the arrival pose; returns reconfigurations.
int Optimizer::followTourAlongStraightCameraPaths(PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	const double same_pose_tolerance = 1e-3;  // rad
	std::vector<Joints> arrival_poses;
	const std::vector<size_t> reconfiguration_edges = this->getReconfigurationEdges(placement_ik, order, &arrival_poses);
	std::vector<char> followed(order.size(), 1);
	for (size_t tour_position : reconfiguration_edges)
		followed[tour_position] = 0;
	moveit::core::RobotState path_state(*this->state);
	int num_reconfigurations = 0;
	bool start_pose_changed = false;  // the edge must be followed again from the new pose
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
	{
		const size_t viewpoint_index = static_cast<size_t>(order[tour_position]);
		Joints& pose = placement_ik.ik_solutions[viewpoint_index].joints;
		if (start_pose_changed)
			followed[tour_position] = this->followStraightCameraPath(path_state, tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints, pose, arrival_poses[tour_position]) ? 1 : 0;
		start_pose_changed = false;
		if (!followed[tour_position])
		{
			++num_reconfigurations;
			continue;
		}
		if (!placement_ik.is_reached[viewpoint_index] || GetLargestJointChange(pose, arrival_poses[tour_position]) <= same_pose_tolerance)
			continue;
		pose = arrival_poses[tour_position];
		start_pose_changed = true;
	}
	return num_reconfigurations;
}

// Tour positions whose edge the camera can't follow straight.
std::vector<size_t> Optimizer::getReconfigurationEdges(const PlacementIkSolutions& placement_ik, const std::vector<int>& order, std::vector<Joints>* arrival_poses)
{
	this->placeObjectAt(placement_ik.object_position);
	if (!(this->straight_path_cache_position == placement_ik.object_position))
	{
		this->straight_path_cache.clear();
		this->straight_path_cache_position = placement_ik.object_position;
	}
	std::vector<char> followed(order.size(), 0);  // char, not bool: written from several threads
	std::vector<Joints> end_poses(order.size());
	this->runInParallel(order.size(), [&](moveit::core::RobotState& thread_state, size_t tour_position)
	{
		const Joints& to_pose = placement_ik.ik_solutions[static_cast<size_t>(order[tour_position])].joints;
		followed[tour_position] = this->followStraightCameraPath(thread_state, tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints, to_pose, end_poses[tour_position]) ? 1 : 0;
	});
	std::vector<size_t> reconfiguration_edges;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (!followed[tour_position])
			reconfiguration_edges.push_back(tour_position);
	if (arrival_poses)
		*arrival_poses = end_poses;
	return reconfiguration_edges;
}

// Straight camera path with an even turn, then exact IK at the end.
// Fails on a joint jump, joint limit, collision or no exact IK.
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
	const TourSettings& tour_settings = this->settings.tour_settings;
	const closest_ik::SolveSettings& exact_solve_settings = this->settings.ik_settings.exact_solve_settings;
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
	const int num_steps = std::max(1, static_cast<int>(std::ceil(std::max(camera_travel / tour_settings.reconfiguration_check_camera_move_per_step, camera_turn / (tour_settings.reconfiguration_check_camera_turn_per_step_deg * radians_per_degree)))));
	auto is_joint_step_ok = [&](const Joints& before, const Joints& after)
	{
		for (size_t joint_index = 0; joint_index < after.size(); ++joint_index)
		{
			if (std::abs(after[joint_index] - before[joint_index]) > tour_settings.path_check_max_joint_jump_deg * radians_per_degree)
				return false;
			const moveit::core::VariableBounds& bounds = this->getJointBounds(joint_index);
			if (bounds.position_bounded_ && (after[joint_index] < bounds.min_position_ || after[joint_index] > bounds.max_position_))
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
		const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(camera, state.getGlobalLinkTransform(this->tool), exact_solve_settings.ignore_roll);
		const Eigen::MatrixXd jacobian = closest_ik::GetToolJacobian(state, this->group, this->tool, pose, exact_solve_settings.ignore_roll);
		const Joints next_pose = AddToJoints(pose, scene_geometry::DampedPinv(jacobian, this->settings.ik_settings.jacobian_damping) * pose_error);
		if (!is_joint_step_ok(pose, next_pose) || !scene_geometry::IsStateValid(this->scene, &state, this->group, next_pose.data()))
			return remember(false, pose);
		pose = next_pose;
	}
	const closest_ik::Result arrival = closest_ik::Solve(this->scene, state, this->group, this->tool, to_camera, pose, exact_solve_settings);
	if (!arrival.exact || !is_joint_step_ok(pose, arrival.q))
		return remember(false, pose);
	return remember(true, arrival.q);
}

// Blocked edges and their charge: detour via home if free, else a multiple.
std::vector<std::pair<size_t, double>> Optimizer::getBlockedEdgeCharges(const PlacementIkSolutions& placement_ik, const std::vector<int>& order)
{
	const double charge_multiple = this->settings.cost_weights.blocked_edge_charge;
	if (charge_multiple <= 0.0 || order.empty())
		return {};
	this->placeObjectAt(placement_ik.object_position);
	const joint_travel::EdgeWeights& edge_weights = this->settings.tour_settings.edge_weights;
	std::vector<double> charges(order.size(), -1.0);  // -1: straight move is free
	this->runInParallel(order.size(), [&](moveit::core::RobotState& thread_state, size_t tour_position)
	{
		const Joints& from_pose = tour_position == 0 ? this->home_pose : placement_ik.ik_solutions[static_cast<size_t>(order[tour_position - 1])].joints;
		const Joints& to_pose = placement_ik.ik_solutions[static_cast<size_t>(order[tour_position])].joints;
		if (this->isStraightJointMoveCollisionFree(thread_state, from_pose, to_pose))
			return;
		const double straight_cost = joint_travel::EdgeCost(from_pose, to_pose, edge_weights, this->counted_joints);
		const bool via_home = tour_position > 0 && this->isStraightJointMoveCollisionFree(thread_state, from_pose, this->home_pose) && this->isStraightJointMoveCollisionFree(thread_state, this->home_pose, to_pose);
		const double detour_cost = joint_travel::EdgeCost(from_pose, this->home_pose, edge_weights, this->counted_joints) + joint_travel::EdgeCost(this->home_pose, to_pose, edge_weights, this->counted_joints);
		charges[tour_position] = via_home ? std::max(0.0, detour_cost - straight_cost) : charge_multiple * straight_cost;
	});
	std::vector<std::pair<size_t, double>> blocked_edges;
	for (size_t tour_position = 0; tour_position < order.size(); ++tour_position)
		if (charges[tour_position] >= 0.0)
			blocked_edges.push_back({tour_position, charges[tour_position]});
	return blocked_edges;
}

bool Optimizer::isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose) const
{
	const double check_step = 0.02;  // rad
	const int num_checks = std::max(1, static_cast<int>(std::ceil(GetLargestJointChange(from_pose, to_pose) / check_step)));
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

void Optimizer::unwrapFullTurnsAlongTour(PlacementIkSolutions& placement_ik, const std::vector<int>& order) const
{
	const Joints* previous = &this->home_pose;
	for (int viewpoint : order)
	{
		Joints& pose = placement_ik.ik_solutions[static_cast<size_t>(viewpoint)].joints;
		this->unwrapFullTurns(pose, *previous);
		previous = &pose;
	}
}

// Shifts full-turn joints by whole turns toward the reference.
void Optimizer::unwrapFullTurns(Joints& pose, const Joints& reference) const
{
	const size_t num_joints = this->group->getActiveJointModels().size();
	const double full_turn = 2.0 * M_PI;
	for (size_t joint_index = 0; joint_index < pose.size() && joint_index < num_joints && joint_index < reference.size(); ++joint_index)
	{
		const moveit::core::VariableBounds& bounds = this->getJointBounds(joint_index);
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

// ---------------------------------------------------------------- Small helpers

std::vector<Joints> Optimizer::getPosesInTourOrder(const PlacementIkSolutions& placement_ik, const std::vector<int>& order) const
{
	std::vector<Joints> tour_poses;
	for (int viewpoint : order)
		tour_poses.push_back(placement_ik.ik_solutions[static_cast<size_t>(viewpoint)].joints);
	return tour_poses;
}

// Smallest distance of any counted joint to its limit.
double Optimizer::computeSmallestJointMargin(const std::vector<Joints>& joint_poses) const
{
	double smallest_margin = std::numeric_limits<double>::infinity();
	for (const Joints& pose : joint_poses)
		for (int joint_index : this->counted_joints)
		{
			const moveit::core::VariableBounds& bounds = this->getJointBounds(static_cast<size_t>(joint_index));
			if (!bounds.position_bounded_)
				continue;
			const double value = pose[static_cast<size_t>(joint_index)];
			smallest_margin = std::min(smallest_margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
		}
	return smallest_margin;
}

const joint_travel::EdgeWeights& Optimizer::getEdgeWeights() const
{
	return this->settings.tour_settings.edge_weights;
}

const std::vector<int>& Optimizer::getCountedJoints() const
{
	return this->counted_joints;
}

std::vector<Joints> Optimizer::convertIkSolutionsToVectorOfJoints(const PlacementIkSolutions& placement_ik) const
{
	std::vector<Joints> joints;
	for (const IkSolution& pose : placement_ik.ik_solutions)
		joints.push_back(pose.joints);
	return joints;
}

const moveit::core::VariableBounds& Optimizer::getJointBounds(size_t joint_index) const
{
	return this->group->getActiveJointModels()[joint_index]->getVariableBounds()[0];
}

std::vector<size_t> Optimizer::getViewpointsWithReach(const PlacementIkSolutions& placement_ik, bool reached) const
{
	std::vector<size_t> viewpoint_indices;
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
		if (static_cast<bool>(placement_ik.is_reached[viewpoint_index]) == reached)
			viewpoint_indices.push_back(viewpoint_index);
	return viewpoint_indices;
}

Eigen::Isometry3d Optimizer::getViewpointTarget(const Eigen::Vector3d& object_position, size_t viewpoint_index) const
{
	return scene_geometry::Translation(object_position) * this->viewpoints[viewpoint_index];
}

void Optimizer::placeObjectAt(const Eigen::Vector3d& object_position)
{
	scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(object_position, this->object_rotation));
}

Eigen::Vector3d Optimizer::clampToSearchBox(const Eigen::Vector3d& position) const
{
	return position.cwiseMax(this->settings.descent_limits.search_box_lower_bound).cwiseMin(this->settings.descent_limits.search_box_upper_bound);
}

bool Optimizer::isAllReached(const PlacementIkSolutions& placement_ik) const
{
	return placement_ik.num_reached == static_cast<int>(this->viewpoints.size());
}

// Splits items over threads, each with its own robot state copy.
void Optimizer::runInParallel(size_t num_items, const std::function<void(moveit::core::RobotState& thread_state, size_t item_index)>& work) const
{
	const size_t num_workers = std::max<size_t>(1, std::min<size_t>(this->num_threads, num_items));
	std::vector<std::thread> threads;
	for (size_t worker_index = 0; worker_index < num_workers; ++worker_index)
		threads.emplace_back([&, worker_index]()
		{
			moveit::core::RobotState thread_state(*this->state);
			for (size_t item_index = worker_index; item_index < num_items; item_index += num_workers)
				work(thread_state, item_index);
		});
	for (std::thread& thread : threads)
		thread.join();
}

// ---------------------------------------------------------------- Logging and markers

void Optimizer::logTourSolution(const std::string& label, const TourSolution& solution) const
{
	RCLCPP_INFO(this->node->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%zu  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.1f deg  smallest margin %.2f deg  cost %.2f", label.c_str(), solution.placement_ik.object_position.x(), solution.placement_ik.object_position.y(), solution.placement_ik.object_position.z(), solution.placement_ik.num_reached, this->viewpoints.size(), solution.placement_ik.gap_sum, solution.num_reconfigurations, solution.num_blocked_edges, solution.tour_length * degrees_per_radian, solution.smallest_joint_margin * degrees_per_radian, solution.total_cost);
}

// Per missed viewpoint: its gap and what blocks it.
void Optimizer::logMissedViewpoints(const PlacementIkSolutions& placement_ik)
{
	this->placeObjectAt(placement_ik.object_position);
	const double limit_band = 0.02;  // rad
	const double contact_band = this->settings.ik_settings.closest_solve_settings.margin + 0.002;  // m
	const size_t num_joints = this->group->getActiveJointModels().size();
	for (size_t viewpoint_index : this->getViewpointsWithReach(placement_ik, false))
	{
		const IkSolution& pose = placement_ik.ik_solutions[viewpoint_index];
		std::string blockers;
		for (size_t joint_index = 0; joint_index < num_joints && joint_index < pose.joints.size(); ++joint_index)
		{
			const moveit::core::VariableBounds& bounds = this->getJointBounds(joint_index);
			const double value = pose.joints[joint_index];
			if (bounds.position_bounded_ && (value - bounds.min_position_ < limit_band || bounds.max_position_ - value < limit_band))
				blockers += " " + this->group->getActiveJointModels()[joint_index]->getName() + " at limit;";
		}
		this->state->setJointGroupPositions(this->group, pose.joints);
		this->state->update();
		const closest_ik::ClosePairs close_pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, contact_band), contact_band);
		for (size_t pair_index = 0; pair_index < close_pairs.names.size(); ++pair_index)
			blockers += " " + close_pairs.names[pair_index] + " " + std::to_string(static_cast<int>(std::round(close_pairs.dists[pair_index] * 1000.0))) + " mm;";
		const bool valid = scene_geometry::IsStateValid(this->scene, this->state.get(), this->group, pose.joints.data());
		RCLCPP_INFO(this->node->get_logger(), "  missed viewpoint %zu: gap %.2f mm (position %.2f mm, angle %.3f deg, clearance short %.2f mm, closest pose %s), blocked by:%s", viewpoint_index, 1000.0 * pose.gap, 1000.0 * pose.position_error.norm(), pose.angle_error * degrees_per_radian, 1000.0 * pose.clearance_shortfall, valid ? "valid" : "INVALID", blockers.empty() ? " nothing (reach)" : blockers.c_str());
	}
}

void Optimizer::publishProgressMarkers(const PlacementIkSolutions& placement_ik, const std::vector<int>& order, const Eigen::Vector3d& object_direction)
{
	if (!this->progress_display.publisher)
		return;
	const rclcpp::Time stamp = this->node->now();
	const Eigen::Isometry3d object_transform = scene_geometry::Translation(placement_ik.object_position);
	visualization_msgs::msg::MarkerArray markers;
	// The first publish clears what an earlier run left.
	if (this->published_marker_keys.empty())
	{
		visualization_msgs::msg::Marker clear;
		clear.action = visualization_msgs::msg::Marker::DELETEALL;
		markers.markers.push_back(clear);
	}

	visualization_msgs::msg::Marker mesh = marker_utils::MakeMarker("object_placement_current", visualization_msgs::msg::Marker::MESH_RESOURCE, stamp);
	mesh.mesh_resource = "file://" + this->progress_display.mesh_path;
	mesh.mesh_use_embedded_materials = false;
	mesh.pose = scene_geometry::ToPoseMsg(scene_geometry::MakeIsometry(placement_ik.object_position, this->object_rotation));
	mesh.scale.x = mesh.scale.y = mesh.scale.z = this->progress_display.mesh_scale;
	mesh.color = marker_utils::Rgba(1.0f, 0.85f, 0.0f, 0.6f);
	markers.markers.push_back(mesh);

	visualization_msgs::msg::Marker viewpoint_markers = marker_utils::MakeMarker("object_placement_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
	viewpoint_markers.scale.x = viewpoint_markers.scale.y = viewpoint_markers.scale.z = 0.01;
	viewpoint_markers.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t viewpoint_index = 0; viewpoint_index < this->viewpoints.size(); ++viewpoint_index)
	{
		viewpoint_markers.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[viewpoint_index]).translation()));
		viewpoint_markers.colors.push_back(placement_ik.is_reached[viewpoint_index] ? marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f) : marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f));
	}
	markers.markers.push_back(viewpoint_markers);

	// Trail as cylinders: line markers vanish from some viewing angles.
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
		arrow.points = {scene_geometry::ToPoint(placement_ik.object_position), scene_geometry::ToPoint(placement_ik.object_position + 0.15 * object_direction.normalized())};
		markers.markers.push_back(arrow);
	}

	// Tool path from home through each pose (misses at their closest pose).
	visualization_msgs::msg::Marker tool_path = marker_utils::MakeMarker("object_placement_tool_path", visualization_msgs::msg::Marker::LINE_STRIP, stamp);
	tool_path.scale.x = 0.003;
	tool_path.color = marker_utils::Rgba(0.2f, 0.6f, 1.0f, 0.9f);
	moveit::core::RobotState marker_state(*this->state);
	std::vector<Joints> path_poses = this->getPosesInTourOrder(placement_ik, order);
	path_poses.insert(path_poses.begin(), this->home_pose);
	for (const Joints& pose : path_poses)
	{
		marker_state.setJointGroupPositions(this->group, pose);
		marker_state.update();
		tool_path.points.push_back(scene_geometry::ToPoint(marker_state.getGlobalLinkTransform(this->tool).translation()));
	}
	if (this->progress_display.show_tool_path && tool_path.points.size() > 1)
		markers.markers.push_back(tool_path);

	visualization_msgs::msg::Marker miss_gap_lines = marker_utils::MakeMarker("object_placement_miss_gap", visualization_msgs::msg::Marker::LINE_LIST, stamp);
	miss_gap_lines.scale.x = 0.003;
	miss_gap_lines.color = marker_utils::Rgba(1.0f, 0.2f, 1.0f, 1.0f);
	visualization_msgs::msg::MarkerArray missed_arm_markers;
	for (size_t viewpoint_index : this->getViewpointsWithReach(placement_ik, false))
	{
		marker_state.setJointGroupPositions(this->group, placement_ik.ik_solutions[viewpoint_index].joints);
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

	// Delete markers the last publish had and this one doesn't.
	std::set<std::pair<std::string, int>> keys;
	for (const visualization_msgs::msg::Marker& marker : markers.markers)
		if (marker.action == visualization_msgs::msg::Marker::ADD)
			keys.insert({marker.ns, marker.id});
	for (const std::pair<std::string, int>& key : this->published_marker_keys)
		if (keys.count(key) == 0)
		{
			visualization_msgs::msg::Marker stale = marker_utils::MakeMarker(key.first, visualization_msgs::msg::Marker::SPHERE, stamp, key.second);
			stale.action = visualization_msgs::msg::Marker::DELETE;
			markers.markers.push_back(stale);
		}
	this->published_marker_keys = std::move(keys);
	this->progress_display.publisher->publish(markers);
}

}  // namespace object_gtsp_co_optimize
