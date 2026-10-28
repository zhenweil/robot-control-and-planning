#include <filesystem>
#include <fstream>
#include <memory>

#include <json/json.h>

#include "object_placement_planning/object_placement.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/scene_geometry.hpp"

void ExportObjectPlacementResult(const std::string& output_dir, const ObjectPlacementResult& result)
{
	std::filesystem::create_directories(output_dir);
	Json::Value root;
	root["ok"] = result.ok;
	root["num_reachable"] = result.num_reachable;
	root["num_total"] = result.num_total;
	root["x"] = result.x;
	root["y"] = result.y;
	root["z"] = result.z;
	root["total_joint_path_length"] = result.total_joint_path_length;
	root["total_weighted_cost"] = result.total_weighted_cost;
	Json::Value tour(Json::arrayValue);
	for (int v : result.tour_order)
		tour.append(v);
	root["tour_order"] = tour;
	Json::Value joints(Json::arrayValue);
	for (const auto& q : result.joint_solutions)
	{
		Json::Value entry(Json::arrayValue);
		for (double x : q)
			entry.append(x);
		joints.append(entry);
	}
	root["joint_solutions"] = joints;
	Json::Value history(Json::arrayValue);
	for (const auto& h : result.history)
	{
		Json::Value entry(Json::objectValue);
		entry["restart"] = static_cast<int>(h[0]);
		entry["x"] = h[1];
		entry["y"] = h[2];
		entry["z"] = h[3];
		entry["weighted_cost"] = h[4];
		history.append(entry);
	}
	root["history"] = history;

	const std::string path = output_dir + "/object_placement_result.json";
	std::ofstream file(path);
	Json::StreamWriterBuilder builder;
	builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter>(builder.newStreamWriter())->write(root, &file);
	printf("Saved object placement result JSON: %s\n", path.c_str());
}

Eigen::Isometry3d PlacementTransform(const Eigen::Vector3d& object_translation_nominal, double x, double y, double z)
{
	return scene_geometry::Translation(Eigen::Vector3d(x, y, z)) * Eigen::Translation3d(-object_translation_nominal);
}

void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal, double x, double y, double z)
{
	scene_geometry::MoveObject(
		planning_scene_monitor, scene_geometry::MakeIsometry(Eigen::Vector3d(x, y, z), object_rotation_nominal));
}

visualization_msgs::msg::MarkerArray BuildObjectPlacementMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const ObjectPlacementResult& result, bool show_tour_line)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;
	const Eigen::Isometry3d xform = PlacementTransform(object_translation_nominal, result.x, result.y, result.z);

	visualization_msgs::msg::Marker mesh =
		marker_utils::MakeMarker("object_placement_object", visualization_msgs::msg::Marker::MESH_RESOURCE, stamp, id++);
	mesh.mesh_resource = "file://" + resolved_mesh_path;
	mesh.mesh_use_embedded_materials = false;
	mesh.pose = scene_geometry::ToPoseMsg(
		xform * scene_geometry::MakeIsometry(object_translation_nominal, object_rotation_nominal));
	mesh.scale.x = mesh.scale.y = mesh.scale.z = mesh_scale;
	mesh.color = marker_utils::Rgba(0.7f, 0.7f, 0.7f, 0.5f);
	markers.markers.push_back(mesh);

	visualization_msgs::msg::Marker line =
		marker_utils::MakeMarker("object_placement_tour", visualization_msgs::msg::Marker::LINE_STRIP, stamp, id++);
	line.scale.x = 0.002;
	line.color = marker_utils::Rgba(0.1f, 0.9f, 0.1f, 0.9f);
	for (size_t k = 0; k < result.tour_order.size(); ++k)
	{
		visualization_msgs::msg::Marker sphere =
			marker_utils::MakeMarker("object_placement_waypoints", visualization_msgs::msg::Marker::SPHERE, stamp, id++);
		sphere.pose.position = scene_geometry::ToPoint(
			(xform * tour_tcp_poses_nominal[static_cast<size_t>(result.tour_order[k])]).translation());
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		const bool reached = k < result.joint_solutions.size() && !result.joint_solutions[k].empty();
		sphere.color = reached ? marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f) : marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f);
		markers.markers.push_back(sphere);
		line.points.push_back(sphere.pose.position);
	}
	if (show_tour_line)
		markers.markers.push_back(line);
	return markers;
}
