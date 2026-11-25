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
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/cmaes.hpp"
#include "object_placement_planning/object_gtsp_co_optimize_setup.hpp"
#include "object_placement_planning/object_gtsp_co_optimizer.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// Better tour solution: more viewpoints reached, then a smaller missed gap, then a lower weighted total cost.
bool IsBetterSolution(const object_gtsp_co_optimize::TourSolution& candidate, const object_gtsp_co_optimize::TourSolution& other)
{
	if (candidate.placement_ik.num_reached != other.placement_ik.num_reached)
		return candidate.placement_ik.num_reached > other.placement_ik.num_reached;
	if (candidate.placement_ik.gap_sum != other.placement_ik.gap_sum)
		return candidate.placement_ik.gap_sum < other.placement_ik.gap_sum;
	return candidate.total_cost < other.total_cost;
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
		object_gtsp_co_optimize::AddMountingSurface(scene);
		object_gtsp_co_optimize::Settings settings = object_gtsp_co_optimize::ReadSettings(*this);
		settings.method_switches.freeze_object_when_reached = false;
		const object_gtsp_co_optimize::DescentLimits& descent_limits = settings.descent_limits;
		object_gtsp_co_optimize::Optimizer optimizer(this->shared_from_this(), model, scene, group_name, viewpoints, object_rotation, home_pose, settings, object_gtsp_co_optimize::ProgressDisplay{nullptr, mesh_path, mesh_scale, false});
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; CMA-ES in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s, initial step %.3f m", num_viewpoint, descent_limits.search_box_lower_bound.x(), descent_limits.search_box_upper_bound.x(), descent_limits.search_box_lower_bound.y(), descent_limits.search_box_upper_bound.y(), descent_limits.search_box_lower_bound.z(), descent_limits.search_box_upper_bound.z(), time_budget, initial_step_size);

		const auto start_time = std::chrono::steady_clock::now();
		auto GetElapsedSeconds = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count(); };
		// The start: IK from home and a first GTSP order, as the descent starts.
		object_gtsp_co_optimize::TourSolution best_solution = optimizer.solveTourAtPosition(start, std::vector<Joints>(viewpoints.size(), home_pose), {}, {});
		int num_evaluations = 1;
		this->logSolution("start", best_solution, num_viewpoint);

		cmaes::Cmaes search(best_solution.placement_ik.object_position, initial_step_size, static_cast<unsigned int>(random_seed), population_size);
		int num_generations = 0;
		while (rclcpp::ok() && GetElapsedSeconds() < time_budget)
		{
			// Every candidate warm-starts from the best solution so far (its poses and order), as the descent's steps do.
			std::vector<object_gtsp_co_optimize::TourSolution> generation;
			for (const Eigen::VectorXd& point : search.askGeneration())
			{
				if (GetElapsedSeconds() >= time_budget)
					break;
				std::vector<Joints> start_poses(viewpoints.size());
				for (size_t viewpoint_index = 0; viewpoint_index < viewpoints.size(); ++viewpoint_index)
					start_poses[viewpoint_index] = best_solution.placement_ik.ik_solutions[viewpoint_index].joints;
				generation.push_back(optimizer.solveTourAtPosition(Eigen::Vector3d(point), start_poses, {}, best_solution.order));
				++num_evaluations;
			}
			if (static_cast<int>(generation.size()) < search.getPopulationSize())
				break;  // the budget ended mid-generation
			std::sort(generation.begin(), generation.end(), IsBetterSolution);
			// Told the clamped positions the candidates were scored at.
			std::vector<Eigen::VectorXd> ranked_points;
			for (const object_gtsp_co_optimize::TourSolution& solution : generation)
				ranked_points.push_back(solution.placement_ik.object_position);
			search.tellRankedGeneration(ranked_points);
			++num_generations;
			const bool improved = IsBetterSolution(generation.front(), best_solution);
			if (improved)
				best_solution = generation.front();
			const Eigen::VectorXd& mean = search.getMean();
			RCLCPP_INFO(this->get_logger(), "generation %d: best of generation reached %d/%d  gap sum %.4f m  reconfigurations %d  tour length %.2f  cost %.2f%s | mean abs (%.4f, %.4f, %.4f)  step %.4f m  %d evaluations  %.0f s", num_generations, generation.front().placement_ik.num_reached, num_viewpoint, generation.front().placement_ik.gap_sum, generation.front().num_reconfigurations, generation.front().tour_length, generation.front().total_cost, improved ? "  <-- new best" : "", mean(0), mean(1), mean(2), search.getStepSize(), num_evaluations, GetElapsedSeconds());
		}
		const double seconds = GetElapsedSeconds();
		const bool all_reached = best_solution.placement_ik.num_reached == num_viewpoint;
		this->logSolution("best", best_solution, num_viewpoint);
		RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches %d/%d poses, %d generations, %d evaluations (%.1f s).", best_solution.placement_ik.object_position.x(), best_solution.placement_ik.object_position.y(), best_solution.placement_ik.object_position.z(), best_solution.placement_ik.num_reached, num_viewpoint, num_generations, num_evaluations, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = best_solution.placement_ik.num_reached;
		out["num_total"] = num_viewpoint;
		out["x"] = best_solution.placement_ik.object_position.x();
		out["y"] = best_solution.placement_ik.object_position.y();
		out["z"] = best_solution.placement_ik.object_position.z();
		out["gap_sum"] = best_solution.placement_ik.gap_sum;
		out["straight_tour_cost"] = best_solution.tour_length;
		out["reconfigurations"] = best_solution.num_reconfigurations;
		out["weighted_total_cost"] = best_solution.total_cost;
		out["generations"] = num_generations;
		out["evaluations"] = num_evaluations;
		out["seconds"] = seconds;
		std::ofstream(output_dir + "/cmaes_placement_result.json") << out;
	}

private:
	template <typename T>
	T getParameter(const std::string& name, const T& fallback)
	{
		return object_gtsp_co_optimize::GetParameter<T>(*this, name, fallback);
	}

	void logSolution(const std::string& label, const object_gtsp_co_optimize::TourSolution& solution, int num_viewpoint)
	{
		RCLCPP_INFO(this->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%d  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  cost %.2f", label.c_str(), solution.placement_ik.object_position.x(), solution.placement_ik.object_position.y(), solution.placement_ik.object_position.z(), solution.placement_ik.num_reached, num_viewpoint, solution.placement_ik.gap_sum, solution.num_reconfigurations, solution.num_blocked_edges, solution.tour_length, solution.total_cost);
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
