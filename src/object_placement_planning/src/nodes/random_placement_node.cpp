// Random-sampling baseline for object placement: draws object positions in a box (uniformly, or weighted by the robot-only
// reachability score) until the time / sample budget runs out. At each, the tour is solved like object_placement (IK poses,
// GTSP, OMPL-planned travel); keeps the one reaching the most viewpoints, then the least planned travel, and runs GTSP restarts there.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <limits>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/ik_sampling.hpp"
#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/reachability_map.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// The tour at one sampled position. planned_travel stays infinite unless every viewpoint is reached.
struct PlacementTour
{
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	int num_reachable = -1;
	std::vector<int> viewpoint_order;
	std::vector<Joints> ik_poses;  // parallel to viewpoint_order
	double planned_travel = std::numeric_limits<double>::infinity();
};

// Robot-only reachability score on a grid over the sampling box: per point, the share of approach directions the tool
// reaches, averaged over a ball of ball_radius (about the object's size). No viewpoints involved.
struct ReachabilityGrid
{
	Eigen::Vector3d lo = Eigen::Vector3d::Zero();
	double spacing = 0.04;  // m
	Eigen::Array3i dims = Eigen::Array3i::Zero();
	std::vector<double> scores;  // [x][y][z]
};

ReachabilityGrid ComputeReachabilityGrid(const ReachabilityMap& map, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, double spacing, double ball_radius)
{
	ReachabilityGrid grid;
	grid.lo = lo;
	grid.spacing = spacing;
	grid.dims = (((hi - lo) / spacing).array().floor() + 1.0).cast<int>();
	auto get_index = [&](int x, int y, int z) { return (static_cast<size_t>(x) * grid.dims.y() + static_cast<size_t>(y)) * grid.dims.z() + static_cast<size_t>(z); };
	std::vector<double> dexterity(static_cast<size_t>(grid.dims.prod()), 0.0);
	for (int x = 0; x < grid.dims.x(); ++x)
		for (int y = 0; y < grid.dims.y(); ++y)
			for (int z = 0; z < grid.dims.z(); ++z)
			{
				const Eigen::Vector3d point = lo + spacing * Eigen::Vector3d(x, y, z);
				int num_reached = 0;
				for (size_t bin = 0; bin < map.NumDirections(); ++bin)
					num_reached += map.Value(static_cast<int>(bin), point).value > 0.0 ? 1 : 0;
				dexterity[get_index(x, y, z)] = static_cast<double>(num_reached) / static_cast<double>(map.NumDirections());
			}
	const int ball_cells = static_cast<int>(std::floor(ball_radius / spacing));
	grid.scores.assign(dexterity.size(), 0.0);
	for (int x = 0; x < grid.dims.x(); ++x)
		for (int y = 0; y < grid.dims.y(); ++y)
			for (int z = 0; z < grid.dims.z(); ++z)
			{
				double sum = 0.0;
				int count = 0;
				for (int dx = -ball_cells; dx <= ball_cells; ++dx)
					for (int dy = -ball_cells; dy <= ball_cells; ++dy)
						for (int dz = -ball_cells; dz <= ball_cells; ++dz)
						{
							const Eigen::Array3i cell(x + dx, y + dy, z + dz);
							if (dx * dx + dy * dy + dz * dz > ball_cells * ball_cells || (cell < 0).any() || (cell >= grid.dims).any())
								continue;
							sum += dexterity[get_index(cell.x(), cell.y(), cell.z())];
							++count;
						}
				grid.scores[get_index(x, y, z)] = sum / count;
			}
	return grid;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	const Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

// The surface the robot stands on (top face at the base plane), as in object_placement, so both see the same world.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;  // m
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

void MoveObject(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& frame,
	const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = frame;
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	scene->processCollisionObjectMsg(obj);
}

}  // namespace

class RandomPlacementNode : public rclcpp::Node
{
public:
	RandomPlacementNode()
		: Node("random_placement", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = param<std::string>("group_name", "panda_arm");
		std::string mesh_path = param<std::string>("mesh_path", "");
		const double mesh_scale = param<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = param<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = param<std::string>("output_dir", "/tmp/random_placement_output");
		const std::vector<double> home = param<std::vector<double>>(
			"initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal =
			ToVector3(param<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d rotation =
			RotationFromRpyDeg(param<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		// Same bounds and IK budget names as object_placement's YAML, so one params file serves both.
		const Eigen::Vector3d lo(param<double>("x_min", -0.6), param<double>("y_min", -0.6), param<double>("z_min", -0.2));
		const Eigen::Vector3d hi(param<double>("x_max", 0.6), param<double>("y_max", 0.6), param<double>("z_max", 0.2));
		const int ik_attempts = std::max(1, static_cast<int>(param<int64_t>("ik_attempts", 30)));
		const int seed = static_cast<int>(param<int64_t>("random_seed", 42));
		const double time_budget = param<double>("rp_time_budget_sec", 90.0);
		const int max_samples = static_cast<int>(param<int64_t>("rp_max_samples", 0));  // 0 = no limit
		// true: positions drawn in proportion to the robot-only reachability score (needs the reachability map); false: uniform.
		const bool weighted_by_reachability = param<bool>("rp_weighted_by_reachability", false);
		const double reachability_ball_radius = param<double>("rp_reachability_ball_radius", 0.1);  // m, about the object's size
		const bool stop_when_all_reached = param<bool>("rp_stop_when_all_reached", false);  // true: GTSP restarts at the first position reaching all, then stop

		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs_repaired_cm_binary.stl";
		const std::vector<Eigen::Isometry3d> tour = LoadTourTcpPoses(tour_input_dir);
		const int n = static_cast<int>(tour.size());
		RCLCPP_INFO(get_logger(), "Loaded %d tour poses; sampling in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s",
			n, lo.x(), hi.x(), lo.y(), hi.y(), lo.z(), hi.z(), time_budget);

		robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		moveit::core::RobotState state(model);
		state.setToDefaultValues();
		const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
		auto psm = BuildLocalCollisionScene(shared_from_this(), mesh_path, mesh_scale, nominal, rotation);
		AddMountingSurface(psm);
		const std::string frame = model->getRootLinkName();

		auto valid = [&](moveit::core::RobotState* s, const moveit::core::JointModelGroup* g, const double* q) {
			s->setJointGroupPositions(g, q);
			s->update();
			planning_scene_monitor::LockedPlanningSceneRO scene(psm);
			return scene->isStateValid(*s, g->getName());
		};
		// Tour solve settings: object_placement's parameter names and defaults, so one params file serves both.
		const int max_ik_solutions = std::max(1, static_cast<int>(param<int64_t>("max_solutions_per_candidate", 4)));
		const int ik_retries_per_point = static_cast<int>(param<int64_t>("ik_retries_per_point", 14));
		const int num_tour_solves = std::max(1, static_cast<int>(param<int64_t>("gtsp_num_restart", 2)));
		const int two_opt_rounds = static_cast<int>(param<int64_t>("gtsp_two_opt_rounds", 5));
		const int gtsp_restarts = static_cast<int>(param<int64_t>("gtsp_restarts", 0));
		const int gtsp_restart_ik_solutions = static_cast<int>(param<int64_t>("gtsp_restart_ik_solutions", 8));
		const double planning_time = param<double>("real_cost_planning_time", 0.5);
		const int planning_attempts = static_cast<int>(param<int64_t>("real_cost_attempts", 3));
		const double unreachable_penalty = param<double>("unreachable_penalty", 50.0);
		joint_travel::EdgeWeights edge_weights;
		edge_weights.joint_distance = param<double>("joint_distance_weight", 1.0);
		edge_weights.max_turn = param<double>("max_joint_deviation_weight", 1.0);
		// All joints but the last, which only spins the camera.
		const std::vector<int> travel_joints = joint_travel::AllJoints(static_cast<int>(home.size()) - 1);

		// IK poses per viewpoint, collected like object_placement: one from the seed, the rest from random starts.
		auto collect_ik_poses = [&](const Eigen::Vector3d& position, const std::vector<Joints>& seeds, int max_solutions)
		{
			const Eigen::Isometry3d shift(Eigen::Translation3d(position - nominal));
			Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
			object_pose.translation() = position;
			object_pose.linear() = rotation;
			MoveObject(psm, frame, object_pose);
			std::vector<std::vector<Joints>> ik_poses(tour.size());
			for (size_t v = 0; v < tour.size(); ++v)
			{
				const geometry_msgs::msg::Pose target = ToPoseMsg(shift * tour[v]);
				Joints pose;
				state.setJointGroupPositions(jmg, seeds[v]);
				if (ik_sampling::ExactIk(state, jmg, target, ik_attempts, 0.0, valid))
				{
					state.copyJointGroupPositions(jmg, pose);
					ik_poses[v].push_back(pose);
				}
				for (int attempt = 0; attempt < max_solutions * 3 + ik_retries_per_point &&
					 static_cast<int>(ik_poses[v].size()) < max_solutions;
					 ++attempt)
				{
					if (ik_poses[v].empty() && attempt >= 4)
						break;	// out of reach: every miss costs a full IK call
					state.setToRandomPositions(jmg, ik_sampling::SeededRng());
					if (!ik_sampling::ExactIk(state, jmg, target, ik_attempts, 0.0, valid))
						continue;
					state.copyJointGroupPositions(jmg, pose);
					const bool duplicate = std::any_of(ik_poses[v].begin(), ik_poses[v].end(), [&](const Joints& other) {
						return joint_travel::JointDistance(other, pose, travel_joints) < 0.1;
					});
					if (!duplicate)
						ik_poses[v].push_back(pose);
				}
			}
			return ik_poses;
		};

		// Best of num_solves GTSP tours (most reached, then straight-line travel); planned travel only if all reached.
		auto solve_tour = [&](const Eigen::Vector3d& position, const std::vector<Joints>& seeds, int max_solutions,
							  int num_solves, const std::vector<int>& initial_order)
		{
			PlacementTour best_tour;
			double best_straight_line_travel = std::numeric_limits<double>::infinity();
			for (int s = 0; s < num_solves; ++s)
			{
				gtsp::Solver tour_solver(collect_ik_poses(position, seeds, max_solutions), home, edge_weights, two_opt_rounds);
				tour_solver.SetTravelJoints(travel_joints);
				const gtsp::Tour gtsp_tour = tour_solver.Solve(initial_order);
				const int num_reachable = static_cast<int>(gtsp_tour.order.size());
				if (num_reachable < best_tour.num_reachable ||
					(num_reachable == best_tour.num_reachable && gtsp_tour.cost >= best_straight_line_travel))
					continue;
				best_straight_line_travel = gtsp_tour.cost;
				best_tour.position = position;
				best_tour.num_reachable = num_reachable;
				best_tour.viewpoint_order = gtsp_tour.order;
				best_tour.ik_poses = gtsp_tour.poses;
			}
			if (best_tour.num_reachable < n)
				return best_tour;
			const std::vector<double> edges = PlanTourJointPathLengths(
				shared_from_this(), model, psm, best_tour.ik_poses, home, group_name, planning_time, planning_attempts,
				/*count_last_joint=*/false);
			best_tour.planned_travel = 0.0;
			for (size_t k = 0; k < best_tour.ik_poses.size(); ++k)
				best_tour.planned_travel += (k < edges.size() && edges[k] >= 0.0)
					? edges[k]
					: joint_travel::JointDistance(k == 0 ? home : best_tour.ik_poses[k - 1], best_tour.ik_poses[k], travel_joints) +
						unreachable_penalty;
			return best_tour;
		};
		auto is_better = [](const PlacementTour& a, const PlacementTour& b)
		{
			return a.num_reachable > b.num_reachable ||
				(a.num_reachable == b.num_reachable && a.planned_travel < b.planned_travel);
		};

		// Weighted sampling: score grid from the reachability map (same file and settings as object_placement's), computed before the clock starts.
		ReachabilityGrid reachability_grid;
		std::discrete_distribution<size_t> pick_grid_point;
		if (weighted_by_reachability)
		{
			ReachabilityMap::Options map_options;
			map_options.samples = static_cast<int>(param<int64_t>("reach_map_samples", map_options.samples));
			map_options.self_collision = param<bool>("reach_map_self_collision", map_options.self_collision);
			map_options.voxel = param<double>("reach_map_voxel", map_options.voxel);
			map_options.directions = static_cast<int>(param<int64_t>("reach_map_directions", map_options.directions));
			map_options.limit_sharpness = param<double>("reach_map_limit_sharpness", map_options.limit_sharpness);
			map_options.seed = static_cast<int>(param<int64_t>("reach_map_seed", map_options.seed));
			std::string map_path = param<std::string>("reach_map_file", "");
			if (map_path.empty())
				map_path = ReachabilityMap::DefaultPath(group_name);
			const std::optional<ReachabilityMap> reachability_map = ReachabilityMap::Load(map_path, model, group_name, "tool0", map_options);
			if (!reachability_map)
			{
				RCLCPP_ERROR(get_logger(), "no reachability map built with these settings at %s (ros2 launch object_placement_planning reachability_map.launch.py)", map_path.c_str());
				return;
			}
			const auto grid_start = std::chrono::steady_clock::now();
			reachability_grid = ComputeReachabilityGrid(*reachability_map, lo, hi, map_options.voxel, reachability_ball_radius);
			pick_grid_point = std::discrete_distribution<size_t>(reachability_grid.scores.begin(), reachability_grid.scores.end());
			RCLCPP_INFO(get_logger(), "sampling weighted by robot-only reachability: %d grid points, ball radius %.2f m, grid computed in %.1f s (not counted in the budget)", reachability_grid.dims.prod(), reachability_ball_radius, std::chrono::duration<double>(std::chrono::steady_clock::now() - grid_start).count());
		}

		std::mt19937 rng(static_cast<unsigned int>(seed));
		std::uniform_real_distribution<double> ux(lo.x(), hi.x()), uy(lo.y(), hi.y()), uz(lo.z(), hi.z());
		std::uniform_real_distribution<double> jitter(-0.5 * reachability_grid.spacing, 0.5 * reachability_grid.spacing);
		// Weighted: a grid point by score, then a uniform point in its cell. Uniform: anywhere in the box.
		auto draw_position = [&]() -> Eigen::Vector3d
		{
			if (!weighted_by_reachability)
				return Eigen::Vector3d(ux(rng), uy(rng), uz(rng));
			const size_t index = pick_grid_point(rng);
			const Eigen::Vector3d cell(static_cast<double>(index / static_cast<size_t>(reachability_grid.dims.y() * reachability_grid.dims.z())), static_cast<double>((index / static_cast<size_t>(reachability_grid.dims.z())) % static_cast<size_t>(reachability_grid.dims.y())), static_cast<double>(index % static_cast<size_t>(reachability_grid.dims.z())));
			const Eigen::Vector3d offset(jitter(rng), jitter(rng), jitter(rng));
			return (reachability_grid.lo + reachability_grid.spacing * cell + offset).cwiseMax(lo).cwiseMin(hi);
		};
		const auto t0 = std::chrono::steady_clock::now();
		auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
		const std::vector<Joints> home_seeds(tour.size(), home);
		PlacementTour best;
		best.position = nominal;
		int samples = 0, samples_reaching_all = 0;
		while (rclcpp::ok() && elapsed() < time_budget && (max_samples <= 0 || samples < max_samples))
		{
			const Eigen::Vector3d at = draw_position();
			ik_sampling::ReseedRng(seed + samples);
			const PlacementTour sample_tour = solve_tour(at, home_seeds, max_ik_solutions, num_tour_solves, {});
			++samples;
			samples_reaching_all += sample_tour.num_reachable == n ? 1 : 0;
			if (!is_better(sample_tour, best))
				continue;
			best = sample_tour;
			RCLCPP_INFO(get_logger(), "sample %d (%.1f s): abs (%.4f, %.4f, %.4f) m reaches %d/%d, planned travel %.2f rad  <-- best",
				samples, elapsed(), at.x(), at.y(), at.z(), best.num_reachable, n, best.planned_travel);
			if (stop_when_all_reached && best.num_reachable == n)
				break;
		}
		const double sampling_secs = elapsed();
		RCLCPP_INFO(get_logger(), "%d samples in %.1f s, %d reach all viewpoints", samples, sampling_secs, samples_reaching_all);

		// GTSP restarts at the best, like object_placement: more full solves there, seeded from its poses, with more IK choices.
		const double travel_before_restarts = best.planned_travel;
		for (int i = 0; i < gtsp_restarts && best.num_reachable == n && rclcpp::ok(); ++i)
		{
			std::vector<Joints> seeds = home_seeds;
			for (size_t k = 0; k < best.viewpoint_order.size(); ++k)
				seeds[static_cast<size_t>(best.viewpoint_order[k])] = best.ik_poses[k];
			const PlacementTour restarted = solve_tour(best.position, seeds, gtsp_restart_ik_solutions, 1, best.viewpoint_order);
			if (is_better(restarted, best))
				best = restarted;
		}
		if (gtsp_restarts > 0 && best.num_reachable == n)
			RCLCPP_INFO(get_logger(), "%d GTSP restarts: planned travel %.2f -> %.2f rad", gtsp_restarts,
				travel_before_restarts, best.planned_travel);

		const double secs = elapsed();
		if (best.num_reachable == n)
			RCLCPP_INFO(get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned travel %.2f rad (%d samples, %.1f s).",
				best.position.x(), best.position.y(), best.position.z(), n, best.planned_travel, samples, secs);
		else
			RCLCPP_WARN(get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%d samples, %.1f s).",
				best.position.x(), best.position.y(), best.position.z(), std::max(0, best.num_reachable), n, samples, secs);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = best.num_reachable == n;
		out["num_reachable"] = std::max(0, best.num_reachable);
		out["num_total"] = n;
		out["x"] = best.position.x();
		out["y"] = best.position.y();
		out["z"] = best.position.z();
		out["planned_travel"] = best.num_reachable == n ? best.planned_travel : -1.0;
		out["samples"] = samples;
		out["samples_reaching_all"] = samples_reaching_all;
		out["seconds"] = secs;
		std::ofstream(output_dir + "/random_placement_result.json") << out;
	}

private:
	template <typename T>
	T param(const std::string& name, const T& fallback)
	{
		if (!has_parameter(name))
			declare_parameter(name, fallback);
		return get_parameter(name).get_value<T>();
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RandomPlacementNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
