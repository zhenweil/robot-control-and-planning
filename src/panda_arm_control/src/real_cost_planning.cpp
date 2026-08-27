#include "panda_arm_control/real_cost_planning.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rcutils/logging.h>
#include <shape_msgs/msg/mesh.hpp>

#include "panda_arm_control/pose_utils.hpp"
#include "panda_arm_control/viewpoint_io.hpp"

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

	// "world" isn't a real robot link and this local scene has no external TF sync -- use
	// panda_link0 instead (identity transform from world, per panda_arm.launch.py).
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.meshes.push_back(mesh_msg);
	obj.mesh_poses.push_back(mesh_pose);
	obj.operation = obj.ADD;

	{
		planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
		locked_scene->processCollisionObjectMsg(obj);

		// Verify the object landed in the scene -- if not, reachability checks silently only
		// test self-collision, not the actual object.
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

} // namespace

std::vector<ViewpointCandidate> ComputeReachabilityWithCollisionCheck(
	std::vector<ViewpointCandidate> candidates,
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera,
	double ik_timeout)
{
	const size_t n = candidates.size();
	std::atomic<size_t> next_index{0};
	std::atomic<int> done{0};
	const int total = static_cast<int>(n);

	unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
	num_threads = std::min(num_threads, static_cast<unsigned int>(std::max<size_t>(n, 1)));

	auto worker = [&]() {
		// Each thread owns its own RobotState -- setFromIK mutates it in place, so sharing one
		// across threads (as the old single-threaded version did) would race.
		moveit::core::RobotState local_state(robot_model);
		local_state.setToDefaultValues();
		const moveit::core::JointModelGroup* local_jmg = local_state.getJointModelGroup(group_name);

		auto validity_callback = [&planning_scene_monitor](
									  moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
									  const double* joint_positions) {
			return IsStateCollisionFree(planning_scene_monitor, state, group, joint_positions);
		};

		while (true)
		{
			if (!rclcpp::ok())
				return;

			size_t i = next_index.fetch_add(1);
			if (i >= n)
				return;

			ViewpointCandidate& c = candidates[i];
			c.tcp_pose = ConvertViewpointToTcpPose(
				c.camera_pos, c.view_dir, object_translation_world, object_rotation_world, t_tcp_camera);

			c.reachable = local_state.setFromIK(local_jmg, c.tcp_pose, "tool0", ik_timeout, validity_callback);
			if (c.reachable)
			{
				local_state.copyJointGroupPositions(local_jmg, c.joint_solution);
				// No longer retries for alternate IK solutions -- nothing reads that plural
				// field, only joint_solution. Kept single-element for the type to stay useful.
				c.joint_solutions.push_back(c.joint_solution);
			}

			int d = ++done;
			if (d % 100 == 0 || d == total)
				printf("reachability: %d/%d candidates checked\n", d, total);
		}
	};

	std::vector<std::thread> workers;
	workers.reserve(num_threads);
	for (unsigned int t = 0; t < num_threads; ++t)
		workers.emplace_back(worker);
	for (auto& w : workers)
		w.join();

	return candidates;
}

void CollectAlternateJointSolutions(
	const std::vector<const ViewpointCandidate*>& candidates,
	const moveit::core::RobotModelConstPtr& robot_model,
	const std::string& group_name,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera,
	double ik_timeout,
	int max_solutions_per_candidate)
{
	if (max_solutions_per_candidate <= 1)
		return;

	const double kDedupThresholdRad = 0.1;	 // ~5.7 degrees of combined joint distance
	const size_t n = candidates.size();

	std::atomic<size_t> next_index{0};
	unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
	num_threads = std::min(num_threads, static_cast<unsigned int>(std::max<size_t>(n, 1)));

	auto worker = [&]() {
		moveit::core::RobotState local_state(robot_model);
		local_state.setToDefaultValues();
		const moveit::core::JointModelGroup* local_jmg = local_state.getJointModelGroup(group_name);

		auto validity_callback = [&planning_scene_monitor](
									  moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
									  const double* joint_positions) {
			return IsStateCollisionFree(planning_scene_monitor, state, group, joint_positions);
		};

		while (true)
		{
			if (!rclcpp::ok())
				return;

			size_t i = next_index.fetch_add(1);
			if (i >= n)
				return;

			// Mutating through a const pointer is safe here: `candidates` is a read-only *view*
			// over storage the caller owns non-const (see header comment) -- same convention this
			// codebase already relies on for `selected` throughout the tour-ordering functions.
			ViewpointCandidate* c = const_cast<ViewpointCandidate*>(candidates[i]);
			geometry_msgs::msg::Pose tcp_pose = ConvertViewpointToTcpPose(
				c->camera_pos, c->view_dir, object_translation_world, object_rotation_world, t_tcp_camera);

			for (int attempt = 0;
				 attempt < max_solutions_per_candidate * 3 &&
				 static_cast<int>(c->joint_solutions.size()) < max_solutions_per_candidate;
				 ++attempt)
			{
				local_state.setToRandomPositions(local_jmg);
				if (!local_state.setFromIK(local_jmg, tcp_pose, "tool0", ik_timeout, validity_callback))
					continue;

				std::vector<double> alt_solution;
				local_state.copyJointGroupPositions(local_jmg, alt_solution);

				bool is_duplicate = false;
				for (const auto& existing : c->joint_solutions)
				{
					if (AreJointSolutionsSimilar(existing, alt_solution, kDedupThresholdRad))
					{
						is_duplicate = true;
						break;
					}
				}

				if (!is_duplicate)
					c->joint_solutions.push_back(std::move(alt_solution));
			}
		}
	};

	std::vector<std::thread> workers;
	workers.reserve(num_threads);
	for (unsigned int t = 0; t < num_threads; ++t)
		workers.emplace_back(worker);
	for (auto& w : workers)
		w.join();
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

// Sums per-joint L2 deltas between consecutive trajectory waypoints -- the actual joint-space
// path length traveled, from the same trajectory, not a separate straight-line estimate.
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

// Straight-line joint-space interpolation between start/goal, checked for collision at each step
// (~0.01 rad max per-joint step, matching typical MoveIt validity-checking resolution). If the
// whole interpolated path is collision-free, it's a real trajectory -- and since it's a single
// straight segment, it's necessarily the shortest possible one, cheaper and more consistent than a
// randomized RRTConnect solve. Returns nullptr if any step is invalid, so the caller can fall back
// to real motion planning only when a straight line genuinely isn't feasible.
robot_trajectory::RobotTrajectoryPtr TryStraightLineInterpolatedPath(
	const moveit::core::RobotModelConstPtr& robot_model, moveit::core::RobotState& scratch_state,
	const moveit::core::JointModelGroup* jmg, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<double>& start_joints, const std::vector<double>& goal_joints)
{
	double max_diff = 0.0;
	for (size_t k = 0; k < start_joints.size(); ++k)
		max_diff = std::max(max_diff, std::abs(goal_joints[k] - start_joints[k]));
	int num_steps = std::max(1, static_cast<int>(max_diff / 0.01));

	auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, jmg);

	for (int step = 0; step <= num_steps; ++step)
	{
		double t = static_cast<double>(step) / static_cast<double>(num_steps);
		std::vector<double> interp(start_joints.size());
		for (size_t k = 0; k < start_joints.size(); ++k)
			interp[k] = start_joints[k] + t * (goal_joints[k] - start_joints[k]);

		scratch_state.setJointGroupPositions(jmg, interp);
		scratch_state.update();

		{
			planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
			if (!locked_scene->isStateValid(scratch_state, jmg->getName()))
				return nullptr;
		}

		trajectory->addSuffixWayPoint(scratch_state, 0.0);
	}

	return trajectory;
}

// Building/using a PlanningPipeline logs adapter chatter that floods the terminal when many
// pipelines are built (esp. the hierarchical tour node). Parent prefixes cover all child loggers.
void QuietPlanningLoggers(const rclcpp::Node::SharedPtr& node)
{
	static const char* kLoggerPrefixesToQuiet[] = {
		"moveit.ompl_planning",
		"moveit.ros_planning",
		"moveit_ros",
	};
	for (const char* logger_name : kLoggerPrefixesToQuiet)
	{
		if (rcutils_logging_set_logger_level(logger_name, RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK)
			RCLCPP_WARN(node->get_logger(), "Failed to quiet %s logger", logger_name);
	}
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
	QuietPlanningLoggers(node);

	planning_scene::PlanningSceneConstPtr scene = planning_scene_monitor->getPlanningScene();

	const size_t n = candidates.size();
	TravelCostMatrix matrix;
	matrix.cartesian_distance.assign(n + 1, std::vector<double>(n + 1, -1.0));
	matrix.joint_distance.assign(n + 1, std::vector<double>(n + 1, -1.0));
	matrix.max_joint_deviation.assign(n + 1, std::vector<double>(n + 1, 0.0));

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
	// Pipeline construction is expensive relative to a tiny pair count -- never build more
	// pipelines than there's work for.
	num_threads = std::min(num_threads, static_cast<unsigned int>(total_pairs));

	// One PlanningPipeline per thread, built sequentially first -- pluginlib's class loader
	// isn't thread-safe to construct concurrently and racing it broke every later plugin load.
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
			// Checks shutdown -- generatePlan() blocks synchronously, so without this Ctrl+C
			// would queue up until the whole precomputation finished.
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

			// Endpoint-only, so it's the same regardless of straight-line vs. RRTConnect below.
			double max_joint_dev = 0.0;
			for (size_t k = 0; k < joints[i]->size(); ++k)
				max_joint_dev = std::max(max_joint_dev, std::abs((*joints[j])[k] - (*joints[i])[k]));
			matrix.max_joint_deviation[i][j] = max_joint_dev;
			matrix.max_joint_deviation[j][i] = max_joint_dev;

			moveit::core::RobotState start_state(robot_model);
			start_state.setToDefaultValues();
			const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_name);
			start_state.setJointGroupPositions(jmg, *joints[i]);
			start_state.update();

			moveit::core::RobotState goal_state(robot_model);
			goal_state.setToDefaultValues();
			goal_state.setJointGroupPositions(jmg, *joints[j]);
			goal_state.update();

			double cartesian_distance = -1.0;
			double joint_distance = -1.0;

			// A collision-free straight line is necessarily the shortest possible path -- cheaper
			// and less noisy than a randomized RRTConnect solve, so try it before falling back.
			moveit::core::RobotState interp_scratch(robot_model);
			interp_scratch.setToDefaultValues();
			robot_trajectory::RobotTrajectoryPtr straight_line = TryStraightLineInterpolatedPath(
				robot_model, interp_scratch, jmg, planning_scene_monitor, *joints[i], *joints[j]);

			if (straight_line)
			{
				cartesian_distance = MeasureTrajectoryEndEffectorDistance(straight_line);
				joint_distance = MeasureTrajectoryJointDistance(straight_line);
			}
			else
			{
				planning_interface::MotionPlanRequest req;
				req.group_name = group_name;
				req.allowed_planning_time = planning_time;
				req.num_planning_attempts = 1;
				moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
				req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

				planning_interface::MotionPlanResponse res;
				bool ok = pipeline.generatePlan(scene, req, res);

				cartesian_distance =
					(ok && res.trajectory_) ? MeasureTrajectoryEndEffectorDistance(res.trajectory_) : -1.0;
				joint_distance = (ok && res.trajectory_) ? MeasureTrajectoryJointDistance(res.trajectory_) : -1.0;
			}

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
	QuietPlanningLoggers(node);

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

		// Re-run RRTConnect a few times, keep the shortest -- affordable here since it's only
		// selected.size() calls, not O(n^2).
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

// candidates[idx] maps to matrix index idx+1 (0 is home). Relies on ptr pointing into
// candidates itself, not a copy.
size_t MatrixIndexOf(const std::vector<ViewpointCandidate>& candidates, const ViewpointCandidate* ptr)
{
	return static_cast<size_t>(ptr - candidates.data()) + 1;
}

// -1.0 means infeasible -- treat as infinitely expensive so NN/2-opt avoid it, matching the
// GA fitness function's handling.
double WeightedCost(
	const TravelCostMatrix& matrix, size_t a, size_t b, double joint_distance_weight,
	double max_joint_deviation_weight)
{
	double cartesian = matrix.cartesian_distance[a][b];
	if (cartesian < 0.0)
		return std::numeric_limits<double>::max();
	return cartesian + joint_distance_weight * matrix.joint_distance[a][b] +
		max_joint_deviation_weight * matrix.max_joint_deviation[a][b];
}

} // namespace

std::vector<const ViewpointCandidate*> NearestNeighborOrderMatrix(
	const std::vector<ViewpointCandidate>& candidates, std::vector<const ViewpointCandidate*> selected,
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight, double max_joint_deviation_weight)
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
			double cost = WeightedCost(
				travel_cost_matrix, current_matrix_idx, idx, joint_distance_weight, max_joint_deviation_weight);
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
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight, double max_joint_deviation_weight)
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
				double old_cost = WeightedCost(
					travel_cost_matrix, prev_idx, matrix_idx(i), joint_distance_weight, max_joint_deviation_weight);
				double new_cost = WeightedCost(
					travel_cost_matrix, prev_idx, matrix_idx(j), joint_distance_weight, max_joint_deviation_weight);

				if (j + 1 < ordered.size())
				{
					old_cost += WeightedCost(
						travel_cost_matrix, matrix_idx(j), matrix_idx(j + 1), joint_distance_weight,
						max_joint_deviation_weight);
					new_cost += WeightedCost(
						travel_cost_matrix, matrix_idx(i), matrix_idx(j + 1), joint_distance_weight,
						max_joint_deviation_weight);
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
	const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight, double max_joint_deviation_weight)
{
	double total_cartesian = 0.0;
	double total_joint = 0.0;
	double worst_max_joint_deviation = 0.0;
	size_t prev_matrix_idx = 0; // start reference
	printf(
		"-- final tour cost breakdown (joint_distance_weight=%.4f, max_joint_deviation_weight=%.4f) --\n",
		joint_distance_weight, max_joint_deviation_weight);
	for (const ViewpointCandidate* ptr : selected)
	{
		size_t cur_matrix_idx = MatrixIndexOf(candidates, ptr);
		double cartesian = travel_cost_matrix.cartesian_distance[prev_matrix_idx][cur_matrix_idx];
		double joint = travel_cost_matrix.joint_distance[prev_matrix_idx][cur_matrix_idx];
		double max_joint_dev = travel_cost_matrix.max_joint_deviation[prev_matrix_idx][cur_matrix_idx];
		printf(
			"  leg %zu -> %zu: cartesian=%.4f m, joint=%.4f rad, max_joint_deviation=%.4f rad\n", prev_matrix_idx,
			cur_matrix_idx, cartesian, joint, max_joint_dev);
		total_cartesian += cartesian;
		total_joint += joint;
		worst_max_joint_deviation = std::max(worst_max_joint_deviation, max_joint_dev);
		prev_matrix_idx = cur_matrix_idx;
	}
	printf(
		"-- totals: cartesian=%.4f m, joint=%.4f rad, worst_max_joint_deviation=%.4f rad, weighted_total=%.4f --\n",
		total_cartesian, total_joint, worst_max_joint_deviation,
		total_cartesian + joint_distance_weight * total_joint);
}

namespace
{
bool MoveToHome(
	const moveit::planning_interface::MoveGroupInterfacePtr& move_group, const std::vector<double>& home_joint_values)
{
	move_group->setStartStateToCurrentState();
	move_group->setJointValueTarget(home_joint_values);
	moveit::planning_interface::MoveGroupInterface::Plan home_plan;
	return static_cast<bool>(move_group->plan(home_plan)) &&
		move_group->execute(home_plan) == moveit::core::MoveItErrorCode::SUCCESS;
}
} // namespace

void ExecuteTourOnRobot(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const moveit::planning_interface::MoveGroupInterfacePtr& move_group,
	const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double execution_planning_time,
	int execution_planning_attempts)
{
	if (selected.empty())
	{
		RCLCPP_WARN(node->get_logger(), "No reachable/selected viewpoints -- nothing to execute on robot");
		return;
	}

	RCLCPP_INFO(
		node->get_logger(), "Planning %zu final tour legs at higher fidelity for execution...", selected.size());

	std::vector<TourTrajectory> legs = PlanFinalTourTrajectories(
		node, robot_model, planning_scene_monitor, selected, start_reference_joints, group_name,
		execution_planning_time, execution_planning_attempts);

	for (size_t i = 0; i < legs.size(); ++i)
	{
		if (!legs[i].ok)
		{
			RCLCPP_ERROR(node->get_logger(), "Aborting execution: failed to plan leg %zu/%zu", i, legs.size());
			return;
		}
	}

	// Legs were planned from start_reference_joints -- the robot must actually be there first
	// or the first leg won't match its state.
	RCLCPP_INFO(node->get_logger(), "Moving to tour start (robot home) before execution...");
	if (!MoveToHome(move_group, start_reference_joints))
	{
		RCLCPP_ERROR(node->get_logger(), "Aborting execution: failed to move to tour start (robot home)");
		return;
	}

	for (size_t i = 0; i < legs.size(); ++i)
	{
		RCLCPP_INFO(node->get_logger(), "Executing tour leg %zu/%zu", i + 1, legs.size());
		marker_pub->publish(BuildCurrentTargetMarkerArray(node->now(), selected[i]->tcp_pose));
		moveit::core::MoveItErrorCode result = move_group->execute(legs[i].trajectory);
		if (result != moveit::core::MoveItErrorCode::SUCCESS)
		{
			RCLCPP_ERROR(
				node->get_logger(), "Aborting execution: leg %zu/%zu failed to execute (error code %d)", i + 1,
				legs.size(), result.val);
			return;
		}
	}

	RCLCPP_INFO(node->get_logger(), "Finished executing %zu-leg tour on robot, returning to home...", legs.size());
	if (!MoveToHome(move_group, start_reference_joints))
		RCLCPP_ERROR(node->get_logger(), "Failed to return to home after executing tour");
}
