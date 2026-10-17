// Builds the reachability map for a planning group and saves it (settings from config/reachability_map.yaml).
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "panda_arm_control/reachability_map.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>(
		"build_reachability_map", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
	const std::string group = node->get_parameter_or<std::string>("group_name", "panda_arm");
	const std::string tip = node->get_parameter_or<std::string>("tip_link", "tool0");
	ReachabilityMap::Options options;
	options.samples = node->get_parameter_or<int>("reach_map_samples", options.samples);
	options.self_collision = node->get_parameter_or<bool>("reach_map_self_collision", options.self_collision);
	options.voxel = node->get_parameter_or<double>("reach_map_voxel", options.voxel);
	options.directions = node->get_parameter_or<int>("reach_map_directions", options.directions);
	options.limit_sharpness = node->get_parameter_or<double>("reach_map_limit_sharpness", options.limit_sharpness);
	options.seed = node->get_parameter_or<int>("reach_map_seed", options.seed);
	std::string path = node->get_parameter_or<std::string>("reach_map_file", "");
	if (path.empty())
		path = ReachabilityMap::DefaultPath(group);

	robot_model_loader::RobotModelLoader loader(node, "robot_description");
	const moveit::core::RobotModelConstPtr model = loader.getModel();
	if (!model)
	{
		RCLCPP_ERROR(node->get_logger(), "no robot model (robot_description)");
		return 1;
	}
	RCLCPP_INFO(
		node->get_logger(), "building the reachability map of %s -> %s: %d samples (self-collision %s), %.3f m voxels, %d directions",
		group.c_str(), tip.c_str(), options.samples, options.self_collision ? "on" : "off", options.voxel, options.directions);
	const auto t0 = std::chrono::steady_clock::now();
	const ReachabilityMap map = ReachabilityMap::Build(model, group, tip, options);
	const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	if (!map.Save(path))
	{
		RCLCPP_ERROR(node->get_logger(), "could not write %s", path.c_str());
		return 1;
	}
	const Eigen::Array3i dims = map.Dims();
	const double cells = static_cast<double>(dims.prod()) * static_cast<double>(map.NumDirections());
	RCLCPP_INFO(
		node->get_logger(), "built in %.1f s: %d x %d x %d voxels x %zu directions, %.1f%% cells nonzero; saved %s", seconds,
		dims.x(), dims.y(), dims.z(), map.NumDirections(), 100.0 * static_cast<double>(map.NumNonzero()) / cells,
		path.c_str());
	rclcpp::shutdown();
	return 0;
}
