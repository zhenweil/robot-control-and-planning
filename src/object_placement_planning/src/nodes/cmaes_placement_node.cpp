// CMA-ES object placement baseline: a black-box search over the object position (x, y, z) that scores every candidate
// with object_gtsp_co_optimize's own evaluation (IK, GTSP reorder, refined poses, cost). Ranked by viewpoints reached,
// then missed gap, then the weighted total cost. Writes the best position; the sweep then runs the same frozen-object arm
// stage as for SGD.

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/cmaes.hpp"
#include "object_placement_planning/object_gtsp_co_optimizer.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// The surface the robot stands on, as in object_gtsp_co_optimize, so all methods see the same world.
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
	for (const moveit::core::LinkModel* link : scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == scene->getRobotModel()->getRootLink())
			scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// Better tour solution: more viewpoints reached, then a smaller missed gap, then a lower weighted total cost.
bool IsBetterSolution(const object_gtsp_co_optimize::TourSolutionAtObjectPosition& candidate, const object_gtsp_co_optimize::TourSolutionAtObjectPosition& other)
{
	if (candidate.viewpoint_ik.num_reached != other.viewpoint_ik.num_reached)
		return candidate.viewpoint_ik.num_reached > other.viewpoint_ik.num_reached;
	if (candidate.viewpoint_ik.gap_sum != other.viewpoint_ik.gap_sum)
		return candidate.viewpoint_ik.gap_sum < other.viewpoint_ik.gap_sum;
	return candidate.weighted_total_cost < other.weighted_total_cost;
}

}  // namespace

class CmaesPlacementNode : public rclcpp::Node
{
public:
	CmaesPlacementNode()
		: Node("cmaes_placement", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void run()
	{
		const std::string group_name = this->getParameter<std::string>("group_name", "panda_arm");
		std::string mesh_path = this->getParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->getParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->getParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = this->getParameter<std::string>("output_dir", "/tmp/cmaes_placement_output");
		const Joints home_pose = this->getParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal = ToVector3(this->getParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		const Eigen::Matrix3d object_rotation = RotationFromRpyDeg(this->getParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		const Eigen::Vector3d start(this->getParameter<double>("initial_x", 0.5), this->getParameter<double>("initial_y", 0.0), this->getParameter<double>("initial_z", 0.15));
		const double time_budget = this->getParameter<double>("cmaes_time_budget_sec", 360.0);
		const double initial_step_size = this->getParameter<double>("cmaes_initial_step_size", 0.05);  // m
		const int population_size = static_cast<int>(this->getParameter<int64_t>("cmaes_population_size", 0));  // 0 = the CMA-ES default
		const int random_seed = static_cast<int>(this->getParameter<int64_t>("random_seed", 42));
		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs_repaired_cm_binary.stl";

		std::vector<Eigen::Isometry3d> viewpoints = LoadTourTcpPoses(tour_input_dir);	// relative to the object position below
		for (Eigen::Isometry3d& viewpoint : viewpoints)
			viewpoint.translation() -= nominal;
		const int num_viewpoint = static_cast<int>(viewpoints.size());

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		const moveit::core::RobotModelConstPtr model = loader.getModel();
		const planning_scene_monitor::PlanningSceneMonitorPtr scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, object_rotation);
		AddMountingSurface(scene);
		const object_gtsp_co_optimize::DescentLimits descent_limits = this->readDescentLimits();
		object_gtsp_co_optimize::Optimizer optimizer(this->shared_from_this(), model, scene, group_name, viewpoints, object_rotation, home_pose, descent_limits, this->readMethodSwitches(), this->readCostWeights(), this->readStepSizes(), this->readIkSettings(), this->readTourSettings(), nullptr, mesh_path, mesh_scale);
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; CMA-ES in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s, initial step %.3f m", num_viewpoint, descent_limits.search_box_lower_bound.x(), descent_limits.search_box_upper_bound.x(), descent_limits.search_box_lower_bound.y(), descent_limits.search_box_upper_bound.y(), descent_limits.search_box_lower_bound.z(), descent_limits.search_box_upper_bound.z(), time_budget, initial_step_size);

		const auto start_time = std::chrono::steady_clock::now();
		auto GetElapsedSeconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
		// The start: IK from home and a first GTSP order, as the descent starts.
		object_gtsp_co_optimize::TourSolutionAtObjectPosition best_solution = optimizer.solveIkAndImproveTourAtPosition(start, std::vector<Joints>(viewpoints.size(), home_pose), {}, {});
		int num_evaluations = 1;
		this->logSolution("start", best_solution, num_viewpoint);

		cmaes::Cmaes search(best_solution.viewpoint_ik.position, initial_step_size, static_cast<unsigned int>(random_seed), population_size);
		int num_generations = 0;
		while (rclcpp::ok() && GetElapsedSeconds() < time_budget)
		{
			// Every candidate warm-starts from the best solution so far (its poses and order), as the descent's steps do.
			std::vector<object_gtsp_co_optimize::TourSolutionAtObjectPosition> generation;
			for (const Eigen::VectorXd& point : search.askGeneration())
			{
				if (GetElapsedSeconds() >= time_budget)
					break;
				std::vector<Joints> start_poses(viewpoints.size());
				for (size_t viewpoint_index = 0; viewpoint_index < viewpoints.size(); ++viewpoint_index)
					start_poses[viewpoint_index] = best_solution.viewpoint_ik.chosen_ik_per_viewpoint[viewpoint_index].q;
				generation.push_back(optimizer.solveIkAndImproveTourAtPosition(Eigen::Vector3d(point), start_poses, {}, best_solution.order));
				++num_evaluations;
			}
			if (static_cast<int>(generation.size()) < search.getPopulationSize())
				break;  // the budget ended mid-generation
			std::sort(generation.begin(), generation.end(), IsBetterSolution);
			// Told the clamped positions the candidates were scored at.
			std::vector<Eigen::VectorXd> ranked_points;
			for (const object_gtsp_co_optimize::TourSolutionAtObjectPosition& solution : generation)
				ranked_points.push_back(solution.viewpoint_ik.position);
			search.tellRankedGeneration(ranked_points);
			++num_generations;
			const bool improved = IsBetterSolution(generation.front(), best_solution);
			if (improved)
				best_solution = generation.front();
			const Eigen::VectorXd& mean = search.getMean();
			RCLCPP_INFO(this->get_logger(), "generation %d: best of generation reached %d/%d  gap sum %.4f m  reconfigurations %d  tour length %.2f  cost %.2f%s | mean abs (%.4f, %.4f, %.4f)  step %.4f m  %d evaluations  %.0f s", num_generations, generation.front().viewpoint_ik.num_reached, num_viewpoint, generation.front().viewpoint_ik.gap_sum, generation.front().num_reconfigurations, generation.front().joint_travel_cost, generation.front().weighted_total_cost, improved ? "  <-- new best" : "", mean(0), mean(1), mean(2), search.getStepSize(), num_evaluations, GetElapsedSeconds());
		}
		const double seconds = GetElapsedSeconds();
		const bool all_reached = best_solution.viewpoint_ik.num_reached == num_viewpoint;
		this->logSolution("best", best_solution, num_viewpoint);
		RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches %d/%d poses, %d generations, %d evaluations (%.1f s).", best_solution.viewpoint_ik.position.x(), best_solution.viewpoint_ik.position.y(), best_solution.viewpoint_ik.position.z(), best_solution.viewpoint_ik.num_reached, num_viewpoint, num_generations, num_evaluations, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = best_solution.viewpoint_ik.num_reached;
		out["num_total"] = num_viewpoint;
		out["x"] = best_solution.viewpoint_ik.position.x();
		out["y"] = best_solution.viewpoint_ik.position.y();
		out["z"] = best_solution.viewpoint_ik.position.z();
		out["gap_sum"] = best_solution.viewpoint_ik.gap_sum;
		out["straight_tour_cost"] = best_solution.joint_travel_cost;
		out["reconfigurations"] = best_solution.num_reconfigurations;
		out["weighted_total_cost"] = best_solution.weighted_total_cost;
		out["generations"] = num_generations;
		out["evaluations"] = num_evaluations;
		out["seconds"] = seconds;
		std::ofstream(output_dir + "/cmaes_placement_result.json") << out;
	}

private:
	template <typename T>
	T getParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}

	void logSolution(const std::string& label, const object_gtsp_co_optimize::TourSolutionAtObjectPosition& solution, int num_viewpoint)
	{
		RCLCPP_INFO(this->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%d  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  cost %.2f", label.c_str(), solution.viewpoint_ik.position.x(), solution.viewpoint_ik.position.y(), solution.viewpoint_ik.position.z(), solution.viewpoint_ik.num_reached, num_viewpoint, solution.viewpoint_ik.gap_sum, solution.num_reconfigurations, solution.num_blocked_edges, solution.joint_travel_cost, solution.weighted_total_cost);
	}

	// The settings below read the same parameters as object_gtsp_co_optimize, so one params file serves both.
	object_gtsp_co_optimize::DescentLimits readDescentLimits()
	{
		object_gtsp_co_optimize::DescentLimits descent_limits;
		descent_limits.search_box_lower_bound = Eigen::Vector3d(this->getParameter<double>("x_min", -0.6), this->getParameter<double>("y_min", -0.6), this->getParameter<double>("z_min", -0.2));
		descent_limits.search_box_upper_bound = Eigen::Vector3d(this->getParameter<double>("x_max", 0.6), this->getParameter<double>("y_max", 0.6), this->getParameter<double>("z_max", 0.2));
		return descent_limits;
	}

	object_gtsp_co_optimize::MethodSwitches readMethodSwitches()
	{
		object_gtsp_co_optimize::MethodSwitches method_switches;
		method_switches.use_room = this->getParameter<bool>("co_room", true);
		method_switches.use_slide = this->getParameter<bool>("co_slide", true);
		method_switches.tour_object_move = this->getParameter<bool>("co_tour_object_move", true);
		method_switches.freeze_object_when_reached = false;
		return method_switches;
	}

	object_gtsp_co_optimize::CostWeights readCostWeights()
	{
		object_gtsp_co_optimize::CostWeights cost_weights;
		cost_weights.initial_penalty = this->getParameter<double>("co_initial_penalty", 100.0);
		cost_weights.penalty_growth = this->getParameter<double>("co_penalty_growth", 10.0);
		cost_weights.max_penalty = this->getParameter<double>("co_max_penalty", 1e5);
		cost_weights.allowed_reach_loss = this->getParameter<double>("co_allowed_reach_loss", 0.15);
		cost_weights.miss_cost = this->getParameter<double>("co_miss_cost", 10.0);
		cost_weights.reconfiguration_cost = this->getParameter<double>("co_reconfiguration_cost", 2.0);
		cost_weights.blocked_edge_charge = this->getParameter<double>("co_blocked_edge_charge", 4.0);
		cost_weights.joint_room_band = this->getParameter<double>("co_joint_room_band", 0.2);
		cost_weights.joint_room_weight = this->getParameter<double>("co_joint_room_weight", 50.0);
		return cost_weights;
	}

	object_gtsp_co_optimize::StepSizes readStepSizes()
	{
		object_gtsp_co_optimize::StepSizes step_sizes;
		step_sizes.max_line_search_tries = static_cast<int>(this->getParameter<int64_t>("co_max_line_search_tries", 4));
		step_sizes.initial_step = this->getParameter<double>("co_initial_step", 0.02);
		step_sizes.min_step = this->getParameter<double>("co_min_step", 0.001);
		step_sizes.max_step = this->getParameter<double>("co_max_step", 0.05);
		step_sizes.initial_slide_step = this->getParameter<double>("co_initial_slide_step", 0.3);
		step_sizes.slide_joint_margin = this->getParameter<double>("co_slide_joint_margin", 0.05);
		step_sizes.min_slide_gain = this->getParameter<double>("co_min_slide_gain", 0.05);
		return step_sizes;
	}

	object_gtsp_co_optimize::IkSettings readIkSettings()
	{
		object_gtsp_co_optimize::IkSettings ik_settings;
		closest_ik::Options ik_options;
		ik_options.iters = static_cast<int>(this->getParameter<int64_t>("closest_ik_iters", 60));
		ik_options.margin = this->getParameter<double>("closest_ik_margin", 0.02);
		ik_options.use_weighted_cost = this->getParameter<bool>("closest_ik_weighted_cost", false);
		ik_options.weighted_iters = static_cast<int>(this->getParameter<int64_t>("closest_ik_weighted_iters", 20));
		ik_options.position_weight = this->getParameter<double>("closest_ik_position_weight", 1.0);
		ik_options.angle_weight = this->getParameter<double>("closest_ik_angle_weight", 0.01);
		ik_options.collision_weight = this->getParameter<double>("closest_ik_collision_weight", 1.0);
		ik_options.camera_object_weight = this->getParameter<double>("closest_ik_camera_object_weight", 1.0);
		ik_options.self_distance_weight = this->getParameter<double>("closest_ik_self_distance_weight", 0.0);
		ik_options.self_distance_margin = this->getParameter<double>("closest_ik_self_distance_margin", 0.08);
		ik_options.joint_room_weight = this->getParameter<double>("closest_ik_joint_room_weight", 1e-3);
		ik_options.rot_scale = this->getParameter<double>("rot_metric_scale", 0.1);
		ik_options.limit_sharpness = this->getParameter<double>("manipulability_limit_sharpness", 20.0);
		ik_options.ignore_roll = this->getParameter<bool>("ignore_camera_roll", false);
		ik_options.exact_margin = this->getParameter<double>("exact_ik_margin", 0.0);
		ik_settings.exact_options = ik_options;
		ik_settings.exact_options.exact_only = true;
		ik_settings.closest_options = ik_options;
		ik_settings.closest_options.try_exact_first = false;
		ik_settings.jacobian_damping = this->getParameter<double>("jacobian_damping", 1e-3);
		ik_settings.exact_finish_gap = this->getParameter<double>("exact_ik_finish_gap", 0.05);
		ik_settings.random_ik_seed = static_cast<int>(this->getParameter<int64_t>("random_seed", 42));
		ik_settings.ik_attempts = static_cast<int>(this->getParameter<int64_t>("co_ik_attempts", 10));
		ik_settings.closest_gap_slack = this->getParameter<double>("co_closest_gap_slack", 0.005);
		ik_settings.closest_ik_extra_starts = static_cast<int>(this->getParameter<int64_t>("co_closest_ik_extra_starts", 3));
		return ik_settings;
	}

	object_gtsp_co_optimize::TourSettings readTourSettings()
	{
		object_gtsp_co_optimize::TourSettings tour_settings;
		tour_settings.edge_weights.joint_distance = this->getParameter<double>("joint_distance_weight", 1.0);
		tour_settings.edge_weights.max_turn = this->getParameter<double>("max_joint_deviation_weight", 1.0);
		tour_settings.edge_weights.time = this->getParameter<double>("travel_time_weight", 0.0);
		tour_settings.two_opt_rounds = static_cast<int>(this->getParameter<int64_t>("gtsp_two_opt_rounds", 5));
		tour_settings.path_check_position_step = this->getParameter<double>("path_check_position_step", 0.01);
		tour_settings.path_check_angle_step = this->getParameter<double>("path_check_angle_step", 0.035);
		tour_settings.path_check_max_joint_jump = this->getParameter<double>("path_check_max_joint_jump", 0.2);
		tour_settings.path_check_joint_margin = this->getParameter<double>("path_check_joint_margin", 0.05);
		tour_settings.gtsp_blocked_rounds = static_cast<int>(this->getParameter<int64_t>("co_gtsp_blocked_rounds", 2));
		tour_settings.num_reference_postures = static_cast<int>(this->getParameter<int64_t>("gtsp_reference_postures", 4));
		return tour_settings;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CmaesPlacementNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
