#pragma once
// Small builders for RViz markers.

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace marker_utils
{

// An ADD marker in the "world" frame with identity orientation.
visualization_msgs::msg::Marker MakeMarker(const std::string& ns, int type, const rclcpp::Time& stamp, int id = 0);

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a);

}  // namespace marker_utils
