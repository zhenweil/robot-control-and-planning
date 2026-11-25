#pragma once
// Setup shared by object_gtsp_co_optimize and the CMA-ES baseline, so both read one params file.

#include <string>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "object_placement_planning/object_gtsp_co_optimizer.hpp"

namespace object_gtsp_co_optimize
{

// Reads a parameter, declaring it with the fallback if it is missing.
template <typename T>
T GetParameter(rclcpp::Node& node, const std::string& name, const T& fallback)
{
	if (!node.has_parameter(name))
		node.declare_parameter(name, fallback);
	return node.get_parameter(name).get_value<T>();
}

Settings ReadSettings(rclcpp::Node& node);

// The surface the robot stands on, top face at the base plane.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& scene);
visualization_msgs::msg::Marker GetMountingSurfaceMarker(const std::string& base_link, const rclcpp::Time& stamp);

}  // namespace object_gtsp_co_optimize
