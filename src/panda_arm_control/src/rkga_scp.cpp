#include "panda_arm_control/rkga_scp.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <thread>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rcutils/logging.h>
#include <shape_msgs/msg/mesh.hpp>

#include "panda_arm_control/pose_utils.hpp"

planning_scene_monitor::PlanningSceneMonitorPtr BuildLocalCollisionScene(
	const rclcpp::Node::SharedPtr& node,
	const std::string& mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world)
{
	auto planning_scene_monitor =
		std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");

	Eigen::Vector3d scale(mesh_scale, mesh_scale, mesh_scale);
	shapes::Mesh* m = shapes::createMeshFromResource("file://" + mesh_path, scale);

	shape_msgs::msg::Mesh mesh_msg;
	shapes::ShapeMsg mesh_tmp;
	shapes::constructMsgFromShape(m, mesh_tmp);
	mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_tmp);

	Eigen::Quaterniond q(object_rotation_world);
	geometry_msgs::msg::Pose mesh_pose;
	mesh_pose.position.x = object_translation_world.x();
	mesh_pose.position.y = object_translation_world.y();
	mesh_pose.position.z = object_translation_world.z();
	mesh_pose.orientation.x = q.x();
	mesh_pose.orientation.y = q.y();
	mesh_pose.orientation.z = q.z();
	mesh_pose.orientation.w = q.w();

	// "world" isn't a real link in the robot model -- it's only the SRDF virtual joint's parent
	// frame, which normally becomes known via an external TF broadcast (panda_arm.launch.py's
	// static_transform_publisher). This local scene doesn't sync with any external TF, so
	// PlanningScene::knowsFrameTransform("world") would fail (verified against
	// RobotState::knowsFrameTransform, which only checks actual robot links). Use panda_link0
	// instead -- panda_arm.launch.py defines world->panda_link0 as an all-zero (identity)
	// transform, so object_translation_world/object_rotation_world are valid unchanged here.
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.meshes.push_back(mesh_msg);
	obj.mesh_poses.push_back(mesh_pose);
	obj.operation = obj.ADD;

	{
		planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
		locked_scene->processCollisionObjectMsg(obj);

		// Verify the object actually landed in the scene rather than assuming -- if this comes
		// back false/without "object" in it, every reachability check downstream is silently
		// only checking self-collision, not collision with the actual object.
		bool object_present = locked_scene->getWorld()->hasObject("object");
		RCLCPP_INFO(
			node->get_logger(), "BuildLocalCollisionScene: object registered in local scene: %s",
			object_present ? "yes" : "NO -- collision checks will NOT see the object");
	}

	return planning_scene_monitor;
}

namespace
{
bool IsStateCollisionFree(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* joint_positions)
{
	state->setJointGroupPositions(group, joint_positions);
	state->update();

	planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
	return locked_scene->isStateValid(*state, group->getName());
}

bool AreJointSolutionsSimilar(const std::vector<double>& a, const std::vector<double>& b, double threshold_rad)
{
	double dist_sq = 0.0;
	for (size_t i = 0; i < a.size(); ++i)
	{
		double d = a[i] - b[i];
		dist_sq += d * d;
	}
	return std::sqrt(dist_sq) < threshold_rad;
}

constexpr int kMaxJointSolutionsPerCandidate = 5;
constexpr double kJointSolutionDedupThresholdRad = 0.1; // ~5.7 degrees of combined joint distance

} // namespace

std::vector<ViewpointCandidate> ComputeReachabilityWithCollisionCheck(
	std::vector<ViewpointCandidate> candidates,
	const moveit::core::RobotStatePtr& robot_state,
	const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera,
	double ik_timeout)
{
	auto validity_callback = [&planning_scene_monitor](
								  moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
								  const double* joint_positions) {
		return IsStateCollisionFree(planning_scene_monitor, state, group, joint_positions);
	};

	for (size_t i = 0; i < candidates.size(); ++i)
	{
		auto& c = candidates[i];
		c.tcp_pose = ConvertViewpointToTcpPose(
			c.camera_pos, c.view_dir, object_translation_world, object_rotation_world, t_tcp_camera);

		c.reachable = robot_state->setFromIK(jmg, c.tcp_pose, "tool0", ik_timeout, validity_callback);

		if (!c.reachable)
			continue;

		robot_state->copyJointGroupPositions(jmg, c.joint_solution);
		c.joint_solutions.push_back(c.joint_solution);

		// Exploit the arm's redundancy: retry from several randomized seeds to find other
		// distinct valid (collision-free) configurations reaching the same tcp_pose, so the
		// RKGA-SCP cost model has real alternatives to choose from instead of being stuck with
		// whichever elbow/wrist configuration this first, arbitrarily-seeded solve happened to
		// find.
		for (int attempt = 0;
			 attempt < kMaxJointSolutionsPerCandidate * 3 &&
			 static_cast<int>(c.joint_solutions.size()) < kMaxJointSolutionsPerCandidate;
			 ++attempt)
		{
			robot_state->setToRandomPositions(jmg);
			if (!robot_state->setFromIK(jmg, c.tcp_pose, "tool0", ik_timeout, validity_callback))
				continue;

			std::vector<double> alt_solution;
			robot_state->copyJointGroupPositions(jmg, alt_solution);

			bool is_duplicate = false;
			for (const auto& existing : c.joint_solutions)
			{
				if (AreJointSolutionsSimilar(existing, alt_solution, kJointSolutionDedupThresholdRad))
				{
					is_duplicate = true;
					break;
				}
			}

			if (!is_duplicate)
				c.joint_solutions.push_back(std::move(alt_solution));
		}
	}

	return candidates;
}

std::vector<ViewpointCandidate> FilterByReachability(std::vector<ViewpointCandidate> candidates)
{
	std::vector<ViewpointCandidate> reachable;
	reachable.reserve(candidates.size());

	for (auto& c : candidates)
		if (c.reachable)
			reachable.push_back(std::move(c));

	return reachable;
}

double ComputeAreaVisibility(const MeshData& mesh, const std::vector<const ViewpointCandidate*>& selected)
{
	std::vector<uint8_t> covered(mesh.NumFaces(), 0);
	for (const ViewpointCandidate* c : selected)
	{
		const auto& mask = c->visible_mask;
		for (size_t f = 0; f < mesh.NumFaces(); ++f)
			if (mask[f])
				covered[f] = 1;
	}

	double area = 0.0;
	for (size_t f = 0; f < mesh.NumFaces(); ++f)
		if (covered[f])
			area += mesh.face_areas[f];

	return area / mesh.total_area;
}

namespace
{

// Sums tool0 position deltas between consecutive trajectory waypoints (forward kinematics), i.e.
// the actual Cartesian path length the end effector traveled -- not a straight-line estimate.
double MeasureTrajectoryEndEffectorDistance(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
	if (!trajectory || trajectory->getWayPointCount() < 2)
		return 0.0;

	double dist = 0.0;
	Eigen::Vector3d prev = trajectory->getWayPoint(0).getGlobalLinkTransform("tool0").translation();
	for (size_t i = 1; i < trajectory->getWayPointCount(); ++i)
	{
		Eigen::Vector3d cur = trajectory->getWayPoint(i).getGlobalLinkTransform("tool0").translation();
		dist += (cur - prev).norm();
		prev = cur;
	}
	return dist;
}

// Sums per-joint L2 deltas between consecutive trajectory waypoints, i.e. the actual joint-space
// path length the arm traveled -- from the same planned trajectory as
// MeasureTrajectoryEndEffectorDistance, not a separate straight-line joint estimate.
double MeasureTrajectoryJointDistance(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
	if (!trajectory || trajectory->getWayPointCount() < 2)
		return 0.0;

	const moveit::core::JointModelGroup* jmg = trajectory->getGroup();

	double dist = 0.0;
	std::vector<double> prev;
	trajectory->getWayPoint(0).copyJointGroupPositions(jmg, prev);
	for (size_t i = 1; i < trajectory->getWayPointCount(); ++i)
	{
		std::vector<double> cur;
		trajectory->getWayPoint(i).copyJointGroupPositions(jmg, cur);

		double step = 0.0;
		for (size_t j = 0; j < cur.size(); ++j)
			step += (cur[j] - prev[j]) * (cur[j] - prev[j]);
		dist += std::sqrt(step);

		prev = std::move(cur);
	}
	return dist;
}

} // namespace

TravelCostMatrix ComputeEndEffectorTravelDistanceMatrix(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<ViewpointCandidate>& candidates,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double planning_time)
{
	// generatePlan() logs an INFO line every single call ("Planner configuration ... will use
	// planner ..."), and this function calls it many times -- silence just this one noisy logger
	// (not the whole node) so the terminal doesn't get flooded.
	if (rcutils_logging_set_logger_level(
			"moveit.ompl_planning.model_based_planning_context", RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK)
		RCLCPP_WARN(node->get_logger(), "Failed to quiet moveit.ompl_planning.model_based_planning_context logger");

	planning_scene::PlanningSceneConstPtr scene = planning_scene_monitor->getPlanningScene();

	const size_t n = candidates.size();
	TravelCostMatrix matrix;
	matrix.cartesian_distance.assign(n + 1, std::vector<double>(n + 1, -1.0));
	matrix.joint_distance.assign(n + 1, std::vector<double>(n + 1, -1.0));

	std::vector<const std::vector<double>*> joints(n + 1);
	joints[0] = &start_reference_joints;
	for (size_t i = 0; i < n; ++i)
		joints[i + 1] = &candidates[i].joint_solution;

	std::vector<std::pair<size_t, size_t>> pair_list;
	pair_list.reserve((n + 1) * n / 2);
	for (size_t i = 0; i <= n; ++i)
		for (size_t j = i + 1; j <= n; ++j)
			pair_list.emplace_back(i, j);
	const int total_pairs = static_cast<int>(pair_list.size());
	RCLCPP_INFO(node->get_logger(), "ComputeEndEffectorTravelDistanceMatrix: %d pairs", total_pairs);

	std::atomic<size_t> next_pair{0};
	std::atomic<int> done{0};
	std::atomic<bool> interrupted{false};
	std::mutex log_mutex;

	unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());

	// One PlanningPipeline per thread, but built sequentially, here, before any worker thread
	// starts -- pluginlib::ClassLoader (used internally to load the OMPL planner plugin) is not
	// safe for concurrent construction from multiple threads. Racing that construction previously
	// corrupted the loader's global state, making every generatePlan() call fail with "No planning
	// plugin loaded" (not just in this function, but in any later PlanningPipeline construction in
	// the same process). Only the actual generatePlan() calls run concurrently below, each against
	// its own already-built pipeline instance.
	std::vector<std::unique_ptr<planning_pipeline::PlanningPipeline>> pipelines;
	pipelines.reserve(num_threads);
	for (unsigned int t = 0; t < num_threads; ++t)
	{
		pipelines.push_back(std::make_unique<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl"));
		pipelines.back()->displayComputedMotionPlans(false);
		pipelines.back()->checkSolutionPaths(false);
	}

	auto worker = [&](unsigned int thread_idx) {
		planning_pipeline::PlanningPipeline& pipeline = *pipelines[thread_idx];

		while (true)
		{
			// Without this, Ctrl+C during this loop just queues up -- generatePlan() is a long
			// blocking synchronous call and nothing here was checking for shutdown, so the whole
			// precomputation had to run to completion before the process could exit.
			if (!rclcpp::ok())
			{
				interrupted = true;
				return;
			}

			size_t idx = next_pair.fetch_add(1);
			if (idx >= pair_list.size())
				return;

			size_t i = pair_list[idx].first;
			size_t j = pair_list[idx].second;

			moveit::core::RobotState start_state(robot_model);
			start_state.setToDefaultValues();
			const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_name);
			start_state.setJointGroupPositions(jmg, *joints[i]);
			start_state.update();

			moveit::core::RobotState goal_state(robot_model);
			goal_state.setToDefaultValues();
			goal_state.setJointGroupPositions(jmg, *joints[j]);
			goal_state.update();

			planning_interface::MotionPlanRequest req;
			req.group_name = group_name;
			req.allowed_planning_time = planning_time;
			req.num_planning_attempts = 1;
			moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
			req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

			planning_interface::MotionPlanResponse res;
			bool ok = pipeline.generatePlan(scene, req, res);

			double cartesian_distance =
				(ok && res.trajectory_) ? MeasureTrajectoryEndEffectorDistance(res.trajectory_) : -1.0;
			double joint_distance = (ok && res.trajectory_) ? MeasureTrajectoryJointDistance(res.trajectory_) : -1.0;

			// Each (i, j) pair appears exactly once in pair_list, so distinct threads never write
			// the same matrix cells -- no lock needed here.
			matrix.cartesian_distance[i][j] = cartesian_distance;
			matrix.cartesian_distance[j][i] = cartesian_distance;
			matrix.joint_distance[i][j] = joint_distance;
			matrix.joint_distance[j][i] = joint_distance;

			int d = ++done;
			if (d % 25 == 0 || d == total_pairs)
			{
				std::lock_guard<std::mutex> lock(log_mutex);
				RCLCPP_INFO(
					node->get_logger(), "ComputeEndEffectorTravelDistanceMatrix: %d/%d pairs planned", d,
					total_pairs);
			}
		}
	};

	std::vector<std::thread> workers;
	workers.reserve(num_threads);
	for (unsigned int t = 0; t < num_threads; ++t)
		workers.emplace_back(worker, t);
	for (auto& w : workers)
		w.join();

	if (interrupted)
		RCLCPP_WARN(
			node->get_logger(), "ComputeEndEffectorTravelDistanceMatrix: interrupted at %d/%d pairs", done.load(),
			total_pairs);

	return matrix;
}

std::vector<TourTrajectory> PlanFinalTourTrajectories(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<const ViewpointCandidate*>& selected, const std::vector<double>& start_reference_joints,
	const std::string& group_name, double planning_time, int num_planning_attempts)
{
	if (rcutils_logging_set_logger_level(
			"moveit.ompl_planning.model_based_planning_context", RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK)
		RCLCPP_WARN(node->get_logger(), "Failed to quiet moveit.ompl_planning.model_based_planning_context logger");

	planning_pipeline::PlanningPipeline pipeline(robot_model, node, "ompl");
	pipeline.displayComputedMotionPlans(false);
	pipeline.checkSolutionPaths(false);

	planning_scene::PlanningSceneConstPtr scene = planning_scene_monitor->getPlanningScene();

	std::vector<TourTrajectory> legs;
	legs.reserve(selected.size());

	const std::vector<double>* start_joints = &start_reference_joints;

	for (size_t leg = 0; leg < selected.size(); ++leg)
	{
		if (!rclcpp::ok())
		{
			RCLCPP_WARN(
				node->get_logger(), "PlanFinalTourTrajectories: interrupted at leg %zu/%zu", leg, selected.size());
			return legs;
		}

		const std::vector<double>& goal_joints = selected[leg]->joint_solution;

		moveit::core::RobotState start_state(robot_model);
		start_state.setToDefaultValues();
		const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_name);
		start_state.setJointGroupPositions(jmg, *start_joints);
		start_state.update();

		moveit::core::RobotState goal_state(robot_model);
		goal_state.setToDefaultValues();
		goal_state.setJointGroupPositions(jmg, goal_joints);
		goal_state.update();

		planning_interface::MotionPlanRequest req;
		req.group_name = group_name;
		req.allowed_planning_time = planning_time;
		req.num_planning_attempts = 1;
		moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
		req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

		// Re-run RRTConnect (randomized) a few times and keep the shortest result -- mirrors
		// WaypointFollower::execute_waypoints' setNumPlanningAttempts(5) rationale, affordable here
		// since this is only selected.size() calls, not O(n^2).
		robot_trajectory::RobotTrajectoryPtr best_trajectory;
		double best_distance = std::numeric_limits<double>::max();
		for (int attempt = 0; attempt < num_planning_attempts; ++attempt)
		{
			planning_interface::MotionPlanResponse res;
			bool ok = pipeline.generatePlan(scene, req, res);
			if (!ok || !res.trajectory_)
				continue;

			double distance = MeasureTrajectoryEndEffectorDistance(res.trajectory_);
			if (distance < best_distance)
			{
				best_distance = distance;
				best_trajectory = res.trajectory_;
			}
		}

		TourTrajectory leg_result;
		if (best_trajectory)
		{
			leg_result.ok = true;
			best_trajectory->getRobotTrajectoryMsg(leg_result.trajectory);
		}
		else
		{
			RCLCPP_ERROR(
				node->get_logger(), "PlanFinalTourTrajectories: failed to plan leg %zu/%zu after %d attempts", leg,
				selected.size(), num_planning_attempts);
		}
		legs.push_back(leg_result);

		start_joints = &goal_joints;
	}

	return legs;
}

namespace
{

// Sorts candidate indices by chromosome key ascending, then walks that order greedily adding
// each candidate if it still covers previously-uncovered area -- same feasibility/stopping
// semantics as GreedySelectViewpoints, but the visiting order comes from the chromosome instead
// of "biggest gain first." No minimum-gain threshold: any candidate contributing new coverage
// (gain > 0) is accepted, however small.
std::vector<size_t> DecodeChromosome(
	const std::vector<double>& keys, const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates,
	double target_area_visibility)
{
	std::vector<size_t> order(candidates.size());
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return keys[a] < keys[b]; });

	const size_t n_faces = mesh.NumFaces();
	std::vector<uint8_t> uncovered(n_faces, 1);
	double visible_area = 0.0;

	std::vector<size_t> selected;

	for (size_t idx : order)
	{
		if (visible_area / mesh.total_area >= target_area_visibility)
			break;

		double gain = 0.0;
		const auto& mask = candidates[idx].visible_mask;
		for (size_t f = 0; f < n_faces; ++f)
			if (uncovered[f] && mask[f])
				gain += mesh.face_areas[f];

		if (gain <= 0.0)
			continue; // contributes nothing new -- skip, keep walking the sorted order

		selected.push_back(idx);
		for (size_t f = 0; f < n_faces; ++f)
			if (mask[f])
				uncovered[f] = 0;
		visible_area += gain;
	}

	return selected;
}

// Thin wrapper over the public ComputeAreaVisibility for use during decoding, where selections
// are index sets into `candidates` rather than pointer vectors.
double ComputeAreaVisibilityByIndex(
	const std::vector<size_t>& selected, const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates)
{
	std::vector<const ViewpointCandidate*> pointers;
	pointers.reserve(selected.size());
	for (size_t idx : selected)
		pointers.push_back(&candidates[idx]);

	return ComputeAreaVisibility(mesh, pointers);
}

// Real travel cost of traversing `selected` in order, via lookups into travel_cost_matrix (index 0
// = start reference, index idx+1 = candidates[idx]) instead of any straight-line estimate. Combines
// real Cartesian end-effector distance with real joint-space distance (weighted by
// joint_distance_weight), both measured from the same planned trajectory per transition. A -1.0
// cartesian_distance entry means no valid plan was found for that transition -- reject the whole
// sequence outright by returning the worst possible fitness, rather than silently treating an
// infeasible transition as free or ignoring it.
double EvaluateFitness(
	const std::vector<size_t>& selected, const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight)
{
	if (selected.empty())
		return std::numeric_limits<double>::max();

	double total = 0.0;
	size_t prev_matrix_idx = 0; // start reference

	for (size_t idx : selected)
	{
		size_t cur_matrix_idx = idx + 1;
		double cartesian_distance = travel_cost_matrix.cartesian_distance[prev_matrix_idx][cur_matrix_idx];
		if (cartesian_distance < 0.0)
			return std::numeric_limits<double>::max(); // infeasible transition

		double joint_distance = travel_cost_matrix.joint_distance[prev_matrix_idx][cur_matrix_idx];
		total += cartesian_distance + joint_distance_weight * joint_distance;
		prev_matrix_idx = cur_matrix_idx;
	}

	return total;
}

} // namespace

std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates,
	const TravelCostMatrix& travel_cost_matrix, const RkgaScpParams& params)
{
	if (candidates.empty())
		return {};

	std::mt19937 rng(params.random_seed);
	std::uniform_real_distribution<double> key_dist(0.0, 1.0);

	const size_t n = candidates.size();
	const size_t elite_count =
		std::max<size_t>(1, static_cast<size_t>(params.elite_fraction * params.population_size));
	const size_t mutant_count = static_cast<size_t>(params.mutant_fraction * params.population_size);

	std::vector<std::vector<double>> population(
		static_cast<size_t>(params.population_size), std::vector<double>(n));
	for (auto& individual : population)
		for (double& key : individual)
			key = key_dist(rng);

	std::vector<size_t> best_selected;
	double best_fitness = std::numeric_limits<double>::max();

	for (int gen = 0; gen < params.num_generations; ++gen)
	{
		std::vector<double> fitness(population.size());
		std::vector<std::vector<size_t>> decoded(population.size());
		for (size_t i = 0; i < population.size(); ++i)
		{
			decoded[i] = DecodeChromosome(population[i], mesh, candidates, params.target_area_visibility);
			fitness[i] = EvaluateFitness(decoded[i], travel_cost_matrix, params.joint_distance_weight);
		}

		std::vector<size_t> rank(population.size());
		std::iota(rank.begin(), rank.end(), 0);
		std::sort(rank.begin(), rank.end(), [&](size_t a, size_t b) { return fitness[a] < fitness[b]; });

		bool improved = fitness[rank[0]] < best_fitness;
		if (improved)
		{
			best_fitness = fitness[rank[0]];
			best_selected = decoded[rank[0]];
		}

		double this_gen_visibility = ComputeAreaVisibilityByIndex(decoded[rank[0]], mesh, candidates);
		double best_visibility = ComputeAreaVisibilityByIndex(best_selected, mesh, candidates);

		printf(
			"gen=%d/%d, best_this_gen=%.4f (%zu viewpoints, %.2f%% visibility), "
			"best_overall=%.4f (%zu viewpoints, %.2f%% visibility)%s\n",
			gen + 1, params.num_generations, fitness[rank[0]], decoded[rank[0]].size(), this_gen_visibility * 100.0,
			best_fitness, best_selected.size(), best_visibility * 100.0, improved ? " *" : "");

		std::vector<std::vector<double>> next_population;
		next_population.reserve(population.size());

		for (size_t e = 0; e < elite_count; ++e)
			next_population.push_back(population[rank[e]]);

		for (size_t m = 0; m < mutant_count; ++m)
		{
			std::vector<double> mutant(n);
			for (double& key : mutant)
				key = key_dist(rng);
			next_population.push_back(std::move(mutant));
		}

		std::uniform_int_distribution<size_t> elite_pick(0, elite_count - 1);
		std::uniform_int_distribution<size_t> any_pick(0, population.size() - 1);
		while (next_population.size() < population.size())
		{
			const std::vector<double>& elite_parent = population[rank[elite_pick(rng)]];
			const std::vector<double>& other_parent = population[any_pick(rng)];

			std::vector<double> child(n);
			for (size_t g = 0; g < n; ++g)
				child[g] = (key_dist(rng) < params.elite_bias) ? elite_parent[g] : other_parent[g];

			next_population.push_back(std::move(child));
		}

		population = std::move(next_population);
	}

	std::vector<const ViewpointCandidate*> result;
	result.reserve(best_selected.size());
	for (size_t idx : best_selected)
		result.push_back(&candidates[idx]);

	PrintTourCostBreakdown(candidates, result, travel_cost_matrix, params.joint_distance_weight);

	return result;
}

namespace
{

// candidates[idx] <-> travel_cost_matrix index idx+1 (index 0 is the start reference) -- see
// ComputeEndEffectorTravelDistanceMatrix. Relies on `ptr` pointing into `candidates` itself (not a
// copy), so pointer arithmetic recovers its index.
size_t MatrixIndexOf(const std::vector<ViewpointCandidate>& candidates, const ViewpointCandidate* ptr)
{
	return static_cast<size_t>(ptr - candidates.data()) + 1;
}

// cartesian_distance[a][b] == -1.0 means infeasible (see TravelCostMatrix) -- surface that as
// "infinitely expensive" so NN/2-opt naturally avoid it, matching EvaluateFitness's handling of the
// same sentinel.
double WeightedCost(const TravelCostMatrix& matrix, size_t a, size_t b, double joint_distance_weight)
{
	double cartesian = matrix.cartesian_distance[a][b];
	if (cartesian < 0.0)
		return std::numeric_limits<double>::max();
	return cartesian + joint_distance_weight * matrix.joint_distance[a][b];
}

} // namespace

std::vector<const ViewpointCandidate*> NearestNeighborOrderMatrix(
	const std::vector<ViewpointCandidate>& candidates, std::vector<const ViewpointCandidate*> selected,
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight)
{
	if (selected.size() < 2)
		return selected;

	std::vector<bool> visited(selected.size(), false);
	std::vector<const ViewpointCandidate*> ordered;
	ordered.reserve(selected.size());

	size_t current_matrix_idx = 0; // start reference (home)
	for (size_t step = 0; step < selected.size(); ++step)
	{
		double best = std::numeric_limits<double>::max();
		size_t best_i = selected.size(); // sentinel: none picked yet

		for (size_t i = 0; i < selected.size(); ++i)
		{
			if (visited[i])
				continue;

			size_t idx = MatrixIndexOf(candidates, selected[i]);
			double cost = WeightedCost(travel_cost_matrix, current_matrix_idx, idx, joint_distance_weight);
			if (best_i == selected.size() || cost < best)
			{
				best = cost;
				best_i = i;
			}
		}

		visited[best_i] = true;
		ordered.push_back(selected[best_i]);
		current_matrix_idx = MatrixIndexOf(candidates, selected[best_i]);
	}

	return ordered;
}

std::vector<const ViewpointCandidate*> TwoOptImproveMatrix(
	const std::vector<ViewpointCandidate>& candidates, std::vector<const ViewpointCandidate*> ordered,
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight)
{
	if (ordered.size() < 3)
		return ordered;

	auto matrix_idx = [&](size_t idx) { return MatrixIndexOf(candidates, ordered[idx]); };

	bool improved = true;
	while (improved)
	{
		improved = false;

		for (size_t i = 0; i + 1 < ordered.size(); ++i)
		{
			size_t prev_idx = (i == 0) ? 0 : matrix_idx(i - 1);

			for (size_t j = i + 1; j < ordered.size(); ++j)
			{
				double old_cost = WeightedCost(travel_cost_matrix, prev_idx, matrix_idx(i), joint_distance_weight);
				double new_cost = WeightedCost(travel_cost_matrix, prev_idx, matrix_idx(j), joint_distance_weight);

				if (j + 1 < ordered.size())
				{
					old_cost +=
						WeightedCost(travel_cost_matrix, matrix_idx(j), matrix_idx(j + 1), joint_distance_weight);
					new_cost +=
						WeightedCost(travel_cost_matrix, matrix_idx(i), matrix_idx(j + 1), joint_distance_weight);
				}

				if (new_cost < old_cost - 1e-9)
				{
					std::reverse(
						ordered.begin() + static_cast<std::ptrdiff_t>(i), ordered.begin() + static_cast<std::ptrdiff_t>(j) + 1);
					improved = true;
				}
			}
		}
	}

	return ordered;
}

void PrintTourCostBreakdown(
	const std::vector<ViewpointCandidate>& candidates, const std::vector<const ViewpointCandidate*>& selected,
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight)
{
	double total_cartesian = 0.0;
	double total_joint = 0.0;
	size_t prev_matrix_idx = 0; // start reference
	printf("-- final tour cost breakdown (joint_distance_weight=%.4f) --\n", joint_distance_weight);
	for (const ViewpointCandidate* ptr : selected)
	{
		size_t cur_matrix_idx = MatrixIndexOf(candidates, ptr);
		double cartesian = travel_cost_matrix.cartesian_distance[prev_matrix_idx][cur_matrix_idx];
		double joint = travel_cost_matrix.joint_distance[prev_matrix_idx][cur_matrix_idx];
		printf(
			"  leg %zu -> %zu: cartesian=%.4f m, joint=%.4f rad\n", prev_matrix_idx, cur_matrix_idx, cartesian,
			joint);
		total_cartesian += cartesian;
		total_joint += joint;
		prev_matrix_idx = cur_matrix_idx;
	}
	printf(
		"-- totals: cartesian=%.4f m, joint=%.4f rad, weighted_total=%.4f --\n", total_cartesian, total_joint,
		total_cartesian + joint_distance_weight * total_joint);
}
