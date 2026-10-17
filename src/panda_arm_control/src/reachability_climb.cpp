// Sanity check of the reachability map: from a start object position, climb the map score of all viewpoints (no IK)
// and print every step. Uses the saved map (config/reachability_map.yaml settings) and the selected viewpoints.
// Markers on /object_placement_progress_markers: the object, its trail, and the viewpoints colored by their map value
// (red 0 .. green 1).
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/reachability_map.hpp"
#include "panda_arm_control/viewpoint_io.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>(
		"reachability_climb", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
	auto log = node->get_logger();
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
	const double tau = node->get_parameter_or<double>("reach_map_tau", 0.05);
	const double max_step = node->get_parameter_or<double>("step", 0.05);
	const std::vector<double> nominal =
		node->get_parameter_or<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15});
	const std::vector<double> start = node->get_parameter_or<std::vector<double>>("start", {0.1, 0.0, 0.1});
	const std::string tour_dir = node->get_parameter_or<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
	const std::string mesh_path = node->get_parameter_or<std::string>("mesh_path", "");
	const double mesh_scale = node->get_parameter_or<double>("mesh_scale", 0.01);
	const double step_delay = node->get_parameter_or<double>("step_delay", 0.3);  // s between published steps
	const std::vector<double> rpy = node->get_parameter_or<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0});
	auto markers_pub =
		node->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_progress_markers", rclcpp::QoS(1).transient_local());

	robot_model_loader::RobotModelLoader loader(node, "robot_description");
	const std::optional<ReachabilityMap> map = ReachabilityMap::Load(path, loader.getModel(), group, tip, options);
	if (!map)
	{
		RCLCPP_ERROR(log, "no reachability map with these settings at %s: run reachability_map.launch.py", path.c_str());
		return 1;
	}
	// Viewpoints relative to the object's nominal position, and their approach-direction bins.
	std::vector<Eigen::Isometry3d> viewpoints = LoadTourTcpPoses(tour_dir);
	std::vector<int> bins;
	for (Eigen::Isometry3d& v : viewpoints)
	{
		v.translation() -= Eigen::Vector3d(nominal[0], nominal[1], nominal[2]);
		bins.push_back(map->Bin(v.linear().col(2)));
	}

	Eigen::Vector3d at(start[0], start[1], start[2]);
	// Check: viewpoints the map calls unreachable here, how far each is from its bin's center direction, and the best
	// value among bins within 30 deg. A good neighbor means the value is a binning artifact.
	int negative = 0;
	for (size_t v = 0; v < viewpoints.size(); ++v)
	{
		const Eigen::Vector3d approach = viewpoints[v].linear().col(2);
		const Eigen::Vector3d p = at + viewpoints[v].translation();
		const double value = map->Value(bins[v], p).value;
		if (value > 0.0)
			continue;
		++negative;
		double best = -std::numeric_limits<double>::infinity();
		int best_bin = -1;
		for (int b = 0; b < static_cast<int>(map->NumDirections()); ++b)
		{
			if (b == bins[v] || approach.dot(map->Direction(b)) < std::cos(30.0 * M_PI / 180.0))
				continue;
			const double nb = map->Value(b, p).value;
			if (nb > best)
			{
				best = nb;
				best_bin = b;
			}
		}
		const auto deg = [&](int b) { return std::acos(std::clamp(approach.dot(map->Direction(b)), -1.0, 1.0)) * 180.0 / M_PI; };
		RCLCPP_INFO(
			log, "check vp %2zu: value %.3f in its bin (%.1f deg off center); best bin within 30 deg: %.3f (%.1f deg off)", v,
			value, deg(bins[v]), best, best_bin < 0 ? 0.0 : deg(best_bin));
	}
	RCLCPP_INFO(log, "check: %d/%zu viewpoints unreachable by the map at abs (%.4f, %.4f, %.4f)", negative, viewpoints.size(), at.x(), at.y(), at.z());
	ReachabilityMap::Score score = map->ScoreAt(viewpoints, bins, at, tau);
	auto report = [&](int iter, double step) {
		RCLCPP_INFO(
			log, "climb %3d: abs (%.4f, %.4f, %.4f)  score %.4f  step %.4f m", iter, at.x(), at.y(), at.z(), score.value, step);
	};
	std::vector<Eigen::Vector3d> trail{at};
	auto publish = [&]() {
		using visualization_msgs::msg::Marker;
		visualization_msgs::msg::MarkerArray markers;
		auto make = [&](const std::string& ns, int type) {
			Marker m;
			m.header.frame_id = "world";
			m.header.stamp = node->now();
			m.ns = ns;
			m.type = type;
			m.action = Marker::ADD;
			m.pose.orientation.w = 1.0;
			return m;
		};
		auto point = [](const Eigen::Vector3d& v) {
			geometry_msgs::msg::Point p;
			p.x = v.x();
			p.y = v.y();
			p.z = v.z();
			return p;
		};
		if (!mesh_path.empty())
		{
			Marker mesh = make("reachability_climb_object", Marker::MESH_RESOURCE);
			mesh.mesh_resource = "file://" + mesh_path;
			mesh.pose.position = point(at);
			const Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(rpy[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(rpy[0] * M_PI / 180.0, Eigen::Vector3d::UnitX());
			mesh.pose.orientation.x = q.x();
			mesh.pose.orientation.y = q.y();
			mesh.pose.orientation.z = q.z();
			mesh.pose.orientation.w = q.w();
			mesh.scale.x = mesh.scale.y = mesh.scale.z = mesh_scale;
			mesh.color.r = 1.0f;
			mesh.color.g = 0.85f;
			mesh.color.a = 0.6f;
			markers.markers.push_back(mesh);
		}
		Marker line = make("reachability_climb_trail", Marker::LINE_STRIP);
		line.scale.x = 0.004;
		line.color.r = 1.0f;
		line.color.g = 0.4f;
		line.color.b = 0.1f;
		line.color.a = 0.9f;
		for (const Eigen::Vector3d& p : trail)
			line.points.push_back(point(p));
		if (line.points.size() > 1)
			markers.markers.push_back(line);
		Marker vps = make("reachability_climb_viewpoints", Marker::SPHERE_LIST);
		vps.scale.x = vps.scale.y = vps.scale.z = 0.012;
		for (size_t v = 0; v < viewpoints.size(); ++v)
		{
			const Eigen::Vector3d p = at + viewpoints[v].translation();
			const double value = std::clamp(map->Value(bins[v], p).value, 0.0, 1.0);
			std_msgs::msg::ColorRGBA c;
			c.r = static_cast<float>(1.0 - value);
			c.g = static_cast<float>(value);
			c.a = 1.0f;
			vps.points.push_back(point(p));
			vps.colors.push_back(c);
		}
		markers.markers.push_back(vps);
		Marker text = make("reachability_climb_score", Marker::TEXT_VIEW_FACING);
		text.pose.position = point(at + Eigen::Vector3d(0, 0, 0.25));
		text.scale.z = 0.04;
		text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
		char buf[96];
		std::snprintf(buf, sizeof(buf), "score %.4f", score.value);
		text.text = buf;
		markers.markers.push_back(text);
		markers_pub->publish(markers);
		std::this_thread::sleep_for(std::chrono::duration<double>(step_delay));
	};
	std::this_thread::sleep_for(std::chrono::seconds(2));  // let RViz subscribe
	report(0, 0.0);
	publish();
	int iter = 0;
	for (double step = max_step; step >= 0.005 && score.gradient.norm() > 1e-12 && iter < 500;)
	{
		const Eigen::Vector3d cand = at + step * score.gradient.normalized();
		const ReachabilityMap::Score next = map->ScoreAt(viewpoints, bins, cand, tau);
		if (next.value > score.value)
		{
			at = cand;
			score = next;
			trail.push_back(at);
			report(++iter, step);
			publish();
		}
		else
			step *= 0.5;
	}
	RCLCPP_INFO(
		log, "climb ended after %d steps at abs (%.4f, %.4f, %.4f), moved %.4f m, score %.4f", iter, at.x(), at.y(), at.z(),
		(at - Eigen::Vector3d(start[0], start[1], start[2])).norm(), score.value);
	rclcpp::shutdown();
	return 0;
}
