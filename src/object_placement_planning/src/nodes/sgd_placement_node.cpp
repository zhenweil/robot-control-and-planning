// SGD baseline (Mayer and Althoff, "Smart placement, faster robots", 2026, Sec. 2.4), position only: Adam steps along
// the averaged closest-IK gap gradients; restarts near the best position so far (not anywhere, as in the paper) until
// the time budget runs out.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// The surface the robot stands on (top face at the base plane), as in object_placement, so both see the same world.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;	// m
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	const std::string base_link = scene->getRobotModel()->getRootLinkName();
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = base_link;
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {size, size, thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * thickness;
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	scene->processCollisionObjectMsg(surface);
	// The base rests on it: allow every link fixed to the root (a URDF's world link has no geometry of its own).
	for (const moveit::core::LinkModel* link : scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == scene->getRobotModel()->getRootLink())
			scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// Closest IK of every viewpoint at one object position.
struct StepResult
{
	std::vector<closest_ik::Result> closest;  // per viewpoint
	int num_exact = 0;
	double mean_position_gap = 0.0;	// m, over all viewpoints (0 for exact ones)
};

}  // namespace

class SgdPlacementNode : public rclcpp::Node
{
public:
	SgdPlacementNode()
		: Node("sgd_placement", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = this->GetParameter<std::string>("group_name", "panda_arm");
		std::string mesh_path = this->GetParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->GetParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->GetParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = this->GetParameter<std::string>("output_dir", "/tmp/sgd_placement_output");
		const Joints home = this->GetParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal = ToVector3(this->GetParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d rotation = RotationFromRpyDeg(this->GetParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		// object_placement's parameter names, so one params file serves both.
		const Eigen::Vector3d lo(this->GetParameter<double>("x_min", -0.6), this->GetParameter<double>("y_min", -0.6), this->GetParameter<double>("z_min", -0.2));
		const Eigen::Vector3d hi(this->GetParameter<double>("x_max", 0.6), this->GetParameter<double>("y_max", 0.6), this->GetParameter<double>("z_max", 0.2));
		const Eigen::Vector3d start(this->GetParameter<double>("initial_x", 0.5), this->GetParameter<double>("initial_y", 0.0), this->GetParameter<double>("initial_z", 0.15));
		const int seed = static_cast<int>(this->GetParameter<int64_t>("random_seed", 42));
		closest_ik::Options ik_options;
		ik_options.iters = static_cast<int>(this->GetParameter<int64_t>("closest_ik_iters", 60));
		ik_options.margin = this->GetParameter<double>("closest_ik_margin", 0.02);
		ik_options.rot_scale = this->GetParameter<double>("rot_metric_scale", 0.1);
		ik_options.limit_sharpness = this->GetParameter<double>("manipulability_limit_sharpness", 20.0);
		ik_options.ignore_roll = this->GetParameter<bool>("ignore_camera_roll", false);
		ik_options.exact_margin = this->GetParameter<double>("exact_ik_margin", 0.0);
		ik_options.try_exact_first = true;
		// The same closest IK as object_gtsp_co_optimize, so the comparison is between the methods only.
		ik_options.use_weighted_cost = this->GetParameter<bool>("closest_ik_weighted_cost", false);
		ik_options.weighted_iters = static_cast<int>(this->GetParameter<int64_t>("closest_ik_weighted_iters", 20));
		ik_options.position_weight = this->GetParameter<double>("closest_ik_position_weight", 1.0);
		ik_options.angle_weight = this->GetParameter<double>("closest_ik_angle_weight", 0.01);
		ik_options.collision_weight = this->GetParameter<double>("closest_ik_collision_weight", 1.0);
		ik_options.camera_object_weight = this->GetParameter<double>("closest_ik_camera_object_weight", 1.0);
		ik_options.self_distance_weight = this->GetParameter<double>("closest_ik_self_distance_weight", 0.0);
		ik_options.self_distance_margin = this->GetParameter<double>("closest_ik_self_distance_margin", 0.08);
		ik_options.joint_room_weight = this->GetParameter<double>("closest_ik_joint_room_weight", 1e-3);
		// Adam (Kingma and Ba, 2015) on the object position; the paper tunes these with Optuna, values not published.
		const double time_budget = this->GetParameter<double>("sgd_time_budget_sec", 300.0);
		const int max_steps = static_cast<int>(this->GetParameter<int64_t>("sgd_max_steps", 50));	// per restart
		const int ik_attempts = static_cast<int>(this->GetParameter<int64_t>("sgd_ik_attempts", 10));	// random start poses after the seed fails
		const double learning_rate = this->GetParameter<double>("sgd_learning_rate", 0.02);	// m
		const double beta1 = this->GetParameter<double>("sgd_beta1", 0.9);
		const double beta2 = this->GetParameter<double>("sgd_beta2", 0.999);
		const double restart_radius = this->GetParameter<double>("sgd_restart_radius", 0.1);	// m per axis around the best position
		// Restarts sampled around where the last descent ended instead: a normal distribution, so nearer spots are likelier.
		const bool restart_near_last = this->GetParameter<bool>("sgd_restart_near_last", false);
		const double restart_sigma = this->GetParameter<double>("sgd_restart_sigma", 0.1);	// m per axis
		const double adam_epsilon = 1e-8;
		// Tour at the solution: object_placement's names and defaults.
		const int two_opt_rounds = static_cast<int>(this->GetParameter<int64_t>("gtsp_two_opt_rounds", 5));
		// GTSP restarts once all are reached, as object_placement's: IK from the best tour's poses with more IK choices.
		const int gtsp_restarts = static_cast<int>(this->GetParameter<int64_t>("sgd_gtsp_restarts", 3));
		const int restart_ik_solutions = static_cast<int>(this->GetParameter<int64_t>("gtsp_restart_ik_solutions", 8));
		const double planning_time = this->GetParameter<double>("real_cost_planning_time", 0.5);
		const int planning_attempts = static_cast<int>(this->GetParameter<int64_t>("real_cost_attempts", 3));
		const double unreachable_penalty = this->GetParameter<double>("unreachable_penalty", 50.0);
		// An all-reached spot also needs a plannable tour (GTSP over its exact poses, every edge planned; a failed edge is
		// replanned with the execution budget before it counts); otherwise SGD restarts.
		const bool require_plannable_tour = this->GetParameter<bool>("sgd_require_plannable_tour", true);
		const double execution_planning_time = this->GetParameter<double>("execution_planning_time", 5.0);
		const int execution_planning_attempts = static_cast<int>(this->GetParameter<int64_t>("execution_planning_attempts", 5));
		joint_travel::EdgeWeights edge_weights;
		edge_weights.joint_distance = this->GetParameter<double>("joint_distance_weight", 1.0);
		edge_weights.max_turn = this->GetParameter<double>("max_joint_deviation_weight", 1.0);
		edge_weights.time = this->GetParameter<double>("travel_time_weight", 0.0);
		// Planned travel = execution time (s), stopping at every pose, as object_placement's travel_cost_is_time.
		const bool travel_cost_is_time = this->GetParameter<bool>("travel_cost_is_time", false);
		const std::vector<int> travel_joints = joint_travel::AllJoints(static_cast<int>(home.size()) - 1);  // the last joint only spins the camera

		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs.stl";
		std::vector<Eigen::Isometry3d> viewpoints = LoadTourTcpPoses(tour_input_dir);	// relative to the object position below
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= nominal;
		const int num_viewpoint = static_cast<int>(viewpoints.size());
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; SGD in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s, %d steps per restart, learning rate %.3f m", num_viewpoint, lo.x(), hi.x(), lo.y(), hi.y(), lo.z(), hi.z(), time_budget, max_steps, learning_rate);

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		moveit::core::RobotState state(model);
		state.setToDefaultValues();
		const moveit::core::JointModelGroup* group = state.getJointModelGroup(group_name);
		const moveit::core::LinkModel* tool = model->getLinkModel("tool0");
		SetJointSpeedLimits(model, group_name, edge_weights);
		const planning_scene_monitor::PlanningSceneMonitorPtr scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, rotation);
		AddMountingSurface(scene);

		// Closest IK of every viewpoint at `position`, in parallel, each started from seeds[v].
		const unsigned int num_threads = std::max(1u, std::min(std::thread::hardware_concurrency(), static_cast<unsigned int>(num_viewpoint)));
		closest_ik::Options exact_options = ik_options;
		exact_options.exact_only = true;
		closest_ik::Options closest_options = ik_options;
		closest_options.try_exact_first = false;
		int num_ik_calls = 0;  // seeds the random start poses, so runs repeat
		// Exact IK from the seed, then from up to ik_attempts random poses; if none is exact, the closest pose from the seed.
		auto SolveClosestIk = [&](const Eigen::Vector3d& position, const std::vector<Joints>& seeds)
		{
			++num_ik_calls;
			scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(position, rotation));
			StepResult step;
			step.closest.resize(viewpoints.size());
			std::vector<std::thread> threads;
			for (unsigned int t = 0; t < num_threads; ++t)
				threads.emplace_back([&, t]()
				{
					moveit::core::RobotState thread_state(state);
					random_numbers::RandomNumberGenerator thread_rng(static_cast<uint32_t>(seed * 100003 + num_ik_calls * 101 + static_cast<int>(t)));
					for (size_t v = t; v < viewpoints.size(); v += num_threads)
					{
						const Eigen::Isometry3d target = scene_geometry::Translation(position) * viewpoints[v];
						closest_ik::Result result = closest_ik::Solve(scene, thread_state, group, tool, target, seeds[v], exact_options);
						for (int attempt = 0; attempt < ik_attempts && !result.exact; ++attempt)
						{
							Joints random_start;
							thread_state.setToRandomPositions(group, thread_rng);
							thread_state.copyJointGroupPositions(group, random_start);
							result = closest_ik::Solve(scene, thread_state, group, tool, target, random_start, exact_options);
						}
						step.closest[v] = result.exact ? result : closest_ik::Solve(scene, thread_state, group, tool, target, seeds[v], closest_options);
					}
				});
			for (std::thread& thread : threads)
				thread.join();
			for (const closest_ik::Result& result : step.closest)
			{
				step.num_exact += result.exact ? 1 : 0;
				step.mean_position_gap += result.exact ? 0.0 : (result.found ? result.pose_error.head<3>().norm() : ik_options.margin + 1.0);
			}
			step.mean_position_gap /= static_cast<double>(num_viewpoint);
			return step;
		};

		std::mt19937 rng(static_cast<unsigned int>(seed));
		std::uniform_real_distribution<double> restart_offset(-restart_radius, restart_radius);
		std::normal_distribution<double> normal_offset(0.0, restart_sigma);
		// A restart spot near `center`, resampled until inside the box (clamped if that keeps failing).
		auto SampleNear = [&](const Eigen::Vector3d& center)
		{
			Eigen::Vector3d sample = center;
			for (int tries = 0; tries < 100; ++tries)
			{
				sample = center + Eigen::Vector3d(normal_offset(rng), normal_offset(rng), normal_offset(rng));
				if ((sample.array() >= lo.array()).all() && (sample.array() <= hi.array()).all())
					return sample;
			}
			return Eigen::Vector3d(sample.cwiseMax(lo).cwiseMin(hi));
		};
		// Tour edges at `position` that fail to plan: GTSP over the step's exact poses, each edge planned (straight move if free,
		// else OMPL), a failed one replanned with the execution budget.
		auto CountFailedTourEdges = [&](const StepResult& step, const Eigen::Vector3d& position)
		{
			std::vector<std::vector<Joints>> step_exact_poses(viewpoints.size());
			for (size_t v = 0; v < viewpoints.size(); ++v)
				step_exact_poses[v].push_back(step.closest[v].q);
			gtsp::Solver step_solver(step_exact_poses, home, edge_weights, two_opt_rounds);
			step_solver.SetTravelJoints(travel_joints);
			const gtsp::Tour tour = step_solver.Solve();
			scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(position, rotation));
			const std::vector<double> edges = PlanTourEdgeLengths(this->shared_from_this(), model, scene, tour.poses, home, group_name, planning_time, planning_attempts, /*count_last_joint=*/false);
			int num_failed = 0;
			for (size_t k = 0; k < tour.poses.size(); ++k)
			{
				if (k < edges.size() && edges[k] >= 0.0)
					continue;
				const std::vector<double> retry = PlanTourEdgeLengths(this->shared_from_this(), model, scene, {tour.poses[k]}, k == 0 ? home : tour.poses[k - 1], group_name, execution_planning_time, execution_planning_attempts, /*count_last_joint=*/false);
				num_failed += (retry.empty() || retry[0] < 0.0) ? 1 : 0;
			}
			return num_failed;
		};
		const auto start_time = std::chrono::steady_clock::now();
		auto GetElapsedSeconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };

		Eigen::Vector3d best_position = start.cwiseMax(lo).cwiseMin(hi);
		StepResult best_step;
		best_step.num_exact = -1;
		Eigen::Vector3d last_position = best_position;	// where the last descent ended
		int restart = 0, total_steps = 0;
		bool all_reached = false;
		// The first all-reached spot whose tour fails to plan: used if the budget ends before a plannable one is found.
		bool has_unplannable_fallback = false;
		StepResult unplannable_fallback_step;
		Eigen::Vector3d unplannable_fallback_position = Eigen::Vector3d::Zero();
		while (!all_reached && rclcpp::ok() && GetElapsedSeconds() < time_budget)
		{
			// The first descent starts at the given start (as object_placement), later ones at a random offset from the best position
			// (or near where the last descent ended).
			Eigen::Vector3d position = restart == 0 ? start.cwiseMax(lo).cwiseMin(hi) : restart_near_last ? SampleNear(last_position) : Eigen::Vector3d(Eigen::Vector3d(best_position + Eigen::Vector3d(restart_offset(rng), restart_offset(rng), restart_offset(rng))).cwiseMax(lo).cwiseMin(hi));
			std::vector<Joints> seeds(viewpoints.size(), home);
			Eigen::Vector3d first_moment = Eigen::Vector3d::Zero(), second_moment = Eigen::Vector3d::Zero();
			for (int step_index = 1; step_index <= max_steps && rclcpp::ok() && GetElapsedSeconds() < time_budget; ++step_index)
			{
				const StepResult step = SolveClosestIk(position, seeds);
				RCLCPP_INFO(this->get_logger(), "restart %d step %d: abs (%.4f, %.4f, %.4f) m  exact IK %d/%d  mean gap %.4f m", restart + 1, step_index, position.x(), position.y(), position.z(), step.num_exact, num_viewpoint, step.mean_position_gap);
				++total_steps;
				if (step.num_exact > best_step.num_exact || (step.num_exact == best_step.num_exact && step.mean_position_gap < best_step.mean_position_gap))
				{
					best_step = step;
					best_position = position;
				}
				if (step.num_exact == num_viewpoint)
				{
					const int num_failed_edges = require_plannable_tour ? CountFailedTourEdges(step, position) : 0;
					if (num_failed_edges == 0)
					{
						all_reached = true;
						best_step = step;
						best_position = position;
						break;
					}
					RCLCPP_INFO(this->get_logger(), "restart %d step %d: all reached but %d tour edges fail to plan; restarting", restart + 1, step_index, num_failed_edges);
					if (!has_unplannable_fallback)
					{
						has_unplannable_fallback = true;
						unplannable_fallback_step = step;
						unplannable_fallback_position = position;
					}
					break;
				}
				// The goal moves with the object, so d|goal - tool|/d(position) = (goal - tool) / |goal - tool|; averaged as in the paper.
				Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
				for (size_t v = 0; v < viewpoints.size(); ++v)
				{
					const closest_ik::Result& result = step.closest[v];
					if (!result.found)
						continue;
					seeds[v] = result.q;
					const Eigen::Vector3d position_error = result.pose_error.head<3>();
					if (!result.exact && position_error.norm() > 1e-9)
						gradient += position_error / position_error.norm();
				}
				gradient /= static_cast<double>(num_viewpoint);
				first_moment = beta1 * first_moment + (1.0 - beta1) * gradient;
				second_moment = beta2 * second_moment + (1.0 - beta2) * gradient.cwiseProduct(gradient);
				const Eigen::Vector3d first_unbiased = first_moment / (1.0 - std::pow(beta1, step_index));
				const Eigen::Vector3d second_unbiased = second_moment / (1.0 - std::pow(beta2, step_index));
				position -= learning_rate * first_unbiased.cwiseQuotient((second_unbiased.array().sqrt() + adam_epsilon).matrix());
				position = position.cwiseMax(lo).cwiseMin(hi);
			}
			last_position = position;
			++restart;
		}
		if (!all_reached && has_unplannable_fallback)
		{
			RCLCPP_WARN(this->get_logger(), "No all-reached spot with a plannable tour within the budget; using the first all-reached one at abs (%.4f, %.4f, %.4f)", unplannable_fallback_position.x(), unplannable_fallback_position.y(), unplannable_fallback_position.z());
			all_reached = true;
			best_step = unplannable_fallback_step;
			best_position = unplannable_fallback_position;
		}
		const double search_seconds = GetElapsedSeconds();

		// Tour at the result: GTSP over the exact poses, OMPL-planned travel once all are reached.
		std::vector<std::vector<Joints>> exact_poses(viewpoints.size());
		for (size_t v = 0; v < viewpoints.size(); ++v)
			if (best_step.closest.size() == viewpoints.size() && best_step.closest[v].exact)
				exact_poses[v].push_back(best_step.closest[v].q);
		gtsp::Solver tour_solver(exact_poses, home, edge_weights, two_opt_rounds);
		tour_solver.SetTravelJoints(travel_joints);
		gtsp::Tour best_tour = tour_solver.Solve();
		scene_geometry::MoveObject(scene, scene_geometry::MakeIsometry(best_position, rotation));
		auto GetPlannedTravel = [&](const gtsp::Tour& tour)
		{
			double travel = 0.0;
			if (travel_cost_is_time)
			{
				const std::vector<double> times = PlanTourEdgeTimes(this->shared_from_this(), model, scene, tour.poses, home, group_name, planning_time, planning_attempts);
				for (size_t k = 0; k < tour.poses.size(); ++k)
					travel += (k < times.size() && times[k] >= 0.0) ? times[k] : joint_travel::StraightMoveTime(k == 0 ? home : tour.poses[k - 1], tour.poses[k], edge_weights.max_velocity, edge_weights.max_acceleration) + unreachable_penalty;
				return travel;
			}
			const std::vector<double> edges = PlanTourJointPathLengths(this->shared_from_this(), model, scene, tour.poses, home, group_name, planning_time, planning_attempts, /*count_last_joint=*/false);
			for (size_t k = 0; k < tour.poses.size(); ++k)
				travel += (k < edges.size() && edges[k] >= 0.0) ? edges[k] : joint_travel::JointDistance(k == 0 ? home : tour.poses[k - 1], tour.poses[k], travel_joints) + unreachable_penalty;
			return travel;
		};
		double planned_travel = all_reached ? GetPlannedTravel(best_tour) : -1.0;
		const double travel_at_reach = planned_travel;

		// GTSP restarts: each viewpoint's IK from its pose in the best tour, then random starts, up to restart_ik_solutions
		// distinct poses; GTSP from the best order; kept if the planned travel is lower.
		for (int restart_index = 0; restart_index < gtsp_restarts && all_reached && rclcpp::ok(); ++restart_index)
		{
			std::vector<Joints> tour_seeds(viewpoints.size(), home);
			for (size_t k = 0; k < best_tour.order.size(); ++k)
				tour_seeds[static_cast<size_t>(best_tour.order[k])] = best_tour.poses[k];
			std::vector<std::vector<Joints>> ik_solutions(viewpoints.size());
			std::vector<std::thread> threads;
			for (unsigned int t = 0; t < num_threads; ++t)
				threads.emplace_back([&, t]()
				{
					moveit::core::RobotState thread_state(state);
					random_numbers::RandomNumberGenerator thread_rng(static_cast<uint32_t>(seed * 100003 + 7919 * (restart_index + 1) + static_cast<int>(t)));
					for (size_t v = t; v < viewpoints.size(); v += num_threads)
					{
						const Eigen::Isometry3d target = scene_geometry::Translation(best_position) * viewpoints[v];
						for (int attempt = 0; attempt < restart_ik_solutions * 3 && static_cast<int>(ik_solutions[v].size()) < restart_ik_solutions; ++attempt)
						{
							Joints start = tour_seeds[v];
							if (attempt > 0)
							{
								thread_state.setToRandomPositions(group, thread_rng);
								thread_state.copyJointGroupPositions(group, start);
							}
							const closest_ik::Result result = closest_ik::Solve(scene, thread_state, group, tool, target, start, exact_options);
							if (!result.exact)
								continue;
							const bool duplicate = std::any_of(ik_solutions[v].begin(), ik_solutions[v].end(), [&](const Joints& other) { return joint_travel::JointDistance(other, result.q, travel_joints) < 0.1; });
							if (!duplicate)
								ik_solutions[v].push_back(result.q);
						}
					}
				});
			for (std::thread& thread : threads)
				thread.join();
			gtsp::Solver restart_solver(ik_solutions, home, edge_weights, two_opt_rounds);
			restart_solver.SetTravelJoints(travel_joints);
			const gtsp::Tour tour = restart_solver.Solve(best_tour.order);
			if (static_cast<int>(tour.order.size()) < num_viewpoint)
				continue;
			const double travel = GetPlannedTravel(tour);
			RCLCPP_INFO(this->get_logger(), "GTSP restart %d/%d: planned travel %.2f %s vs %.2f best%s", restart_index + 1, gtsp_restarts, travel, travel_cost_is_time ? "s" : "rad", planned_travel, travel < planned_travel ? "  <-- new best" : "");
			if (travel < planned_travel)
			{
				planned_travel = travel;
				best_tour = tour;
			}
		}
		if (all_reached && gtsp_restarts > 0)
			RCLCPP_INFO(this->get_logger(), "%d GTSP restarts: planned travel %.2f -> %.2f %s", gtsp_restarts, travel_at_reach, planned_travel, travel_cost_is_time ? "s" : "rad");

		const double seconds = GetElapsedSeconds();
		if (all_reached)
			RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned travel %.2f %s (%d restarts, %d steps, %.1f s).", best_position.x(), best_position.y(), best_position.z(), num_viewpoint, planned_travel, travel_cost_is_time ? "s" : "rad", restart, total_steps, seconds);
		else
			RCLCPP_WARN(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%d restarts, %d steps, %.1f s).", best_position.x(), best_position.y(), best_position.z(), std::max(0, best_step.num_exact), num_viewpoint, restart, total_steps, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = std::max(0, best_step.num_exact);
		out["num_total"] = num_viewpoint;
		out["x"] = best_position.x();
		out["y"] = best_position.y();
		out["z"] = best_position.z();
		out["planned_travel"] = planned_travel;
		out["planned_travel_at_reach"] = travel_at_reach;
		out["restarts"] = restart;
		out["steps"] = total_steps;
		out["search_seconds"] = search_seconds;
		out["seconds"] = seconds;
		std::ofstream(output_dir + "/sgd_placement_result.json") << out;
	}

private:
	template <typename T>
	T GetParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SgdPlacementNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
