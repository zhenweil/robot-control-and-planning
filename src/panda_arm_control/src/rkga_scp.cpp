#include "panda_arm_control/rkga_scp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include "panda_arm_control/mesh_utils.hpp"
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

TravelCostBreakdown ComputeTravelCostBreakdown(
	const std::vector<const ViewpointCandidate*>& selected, const Eigen::Vector3d& start_reference_position,
	const std::vector<double>& start_reference_joints)
{
	TravelCostBreakdown breakdown;

	Eigen::Vector3d prev_pos = start_reference_position;
	const std::vector<double>* prev_joints = &start_reference_joints;

	for (const ViewpointCandidate* c : selected)
	{
		Eigen::Vector3d pos(c->tcp_pose.position.x, c->tcp_pose.position.y, c->tcp_pose.position.z);
		breakdown.cartesian_distance_m += (pos - prev_pos).norm();

		if (!prev_joints->empty() && !c->joint_solution.empty() && prev_joints->size() == c->joint_solution.size())
		{
			double joint_dist_sq = 0.0;
			for (size_t i = 0; i < prev_joints->size(); ++i)
			{
				double d = (*prev_joints)[i] - c->joint_solution[i];
				joint_dist_sq += d * d;
			}
			breakdown.joint_distance_rad += std::sqrt(joint_dist_sq);
		}

		prev_pos = pos;
		prev_joints = &c->joint_solution;
	}

	return breakdown;
}

namespace
{

Eigen::Vector3d TcpPositionOf(const ViewpointCandidate& c)
{
	return Eigen::Vector3d(c.tcp_pose.position.x, c.tcp_pose.position.y, c.tcp_pose.position.z);
}

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

// Minimum-cost way to traverse `selected` in order, via dynamic programming over each
// candidate's joint_solutions options: since the same TCP pose can be reached via different
// elbow/wrist configurations, a single fixed joint solution per candidate doesn't necessarily
// reflect the true achievable reconfiguration cost between two viewpoints. dp[j] tracks the
// minimum cumulative cost to reach option j of the current step, considering every option of the
// previous step -- so the result is the best possible joint-solution assignment for this
// particular visiting order, not just whatever solution happened to be recorded first. Fitness is
// travel cost alone -- no per-viewpoint penalty, so evolution only pressures toward cheaper travel,
// not toward using fewer viewpoints.
double EvaluateFitness(
	const std::vector<size_t>& selected, const std::vector<ViewpointCandidate>& candidates,
	const Eigen::Vector3d& start_reference_position, const std::vector<double>& start_reference_joints,
	double joint_distance_weight)
{
	if (selected.empty())
		return std::numeric_limits<double>::max();

	std::vector<double> dp = {0.0};
	std::vector<Eigen::Vector3d> prev_positions = {start_reference_position};
	std::vector<std::vector<double>> prev_joint_options = {start_reference_joints};

	for (size_t idx : selected)
	{
		const ViewpointCandidate& c = candidates[idx];
		Eigen::Vector3d pos = TcpPositionOf(c);

		// Copy (not reference) to sidestep any ambiguity about temporary lifetime extension when
		// the ternary's branches have different value categories -- these vectors are tiny
		// (<= a handful of 7-element joint vectors), so the copy cost is negligible.
		std::vector<std::vector<double>> options =
			c.joint_solutions.empty() ? std::vector<std::vector<double>>{c.joint_solution} : c.joint_solutions;

		std::vector<double> new_dp(options.size(), std::numeric_limits<double>::max());
		for (size_t j = 0; j < options.size(); ++j)
			for (size_t jp = 0; jp < dp.size(); ++jp)
				new_dp[j] = std::min(
					new_dp[j],
					dp[jp] + TourCost(prev_positions[jp], prev_joint_options[jp], pos, options[j], joint_distance_weight));

		dp = std::move(new_dp);
		prev_positions.assign(options.size(), pos);
		prev_joint_options = options;
	}

	return *std::min_element(dp.begin(), dp.end());
}

} // namespace

std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates,
	const Eigen::Vector3d& start_reference_position, const std::vector<double>& start_reference_joints,
	double joint_distance_weight, const RkgaScpParams& params)
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
			fitness[i] = EvaluateFitness(
				decoded[i], candidates, start_reference_position, start_reference_joints, joint_distance_weight);
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

	return result;
}
