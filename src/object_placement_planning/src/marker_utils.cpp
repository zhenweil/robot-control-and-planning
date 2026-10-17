#include "object_placement_planning/marker_utils.hpp"

namespace marker_utils
{

visualization_msgs::msg::Marker MakeMarker(const std::string& ns, int type, const rclcpp::Time& stamp, int id)
{
	visualization_msgs::msg::Marker m;
	m.header.frame_id = "world";
	m.header.stamp = stamp;
	m.ns = ns;
	m.id = id;
	m.type = type;
	m.action = visualization_msgs::msg::Marker::ADD;
	m.pose.orientation.w = 1.0;
	return m;
}

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a)
{
	std_msgs::msg::ColorRGBA c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
	return c;
}

}  // namespace marker_utils
