#include "panda_arm_control/viewpoint_io.hpp"

#include <fstream>

#include <json/json.h>

Eigen::Vector3d ToVector3(const std::vector<double>& v, const Eigen::Vector3d& fallback)
{
	if (v.size() != 3)
		return fallback;
	return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Matrix3d RotationFromRpyDeg(const std::vector<double>& rpy_deg)
{
	if (rpy_deg.size() != 3)
		return Eigen::Matrix3d::Identity();

	double roll = rpy_deg[0] * M_PI / 180.0;
	double pitch = rpy_deg[1] * M_PI / 180.0;
	double yaw = rpy_deg[2] * M_PI / 180.0;

	// Matches scipy's R.from_euler("xyz", [roll, pitch, yaw]).as_matrix(): Rz * Ry * Rx.
	Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
						 .toRotationMatrix();
	return R;
}

Eigen::Isometry3d IsometryFromXyzQuat(const std::vector<double>& xyz, const std::vector<double>& quat_xyzw)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	if (xyz.size() == 3)
		T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
	if (quat_xyzw.size() == 4)
		T.linear() = Eigen::Quaterniond(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]).toRotationMatrix();
	return T;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v)
{
	geometry_msgs::msg::Point p;
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	return p;
}

void ExportSelectedViewpoints(const std::string& output_dir, const std::vector<const ViewpointCandidate*>& selected)
{
	Json::Value root(Json::arrayValue);

	std::string csv_path = output_dir + "/selected_viewpoints.csv";
	std::ofstream csv_file(csv_path);
	csv_file << "id,camera_x,camera_y,camera_z,dir_x,dir_y,dir_z,"
				"target_x,target_y,target_z,distance,num_visible_faces\n";

	for (size_t i = 0; i < selected.size(); ++i)
	{
		const auto& c = *selected[i];

		Json::Value item;
		item["id"] = static_cast<int>(i);
		item["camera_pos"]["x"] = c.camera_pos.x();
		item["camera_pos"]["y"] = c.camera_pos.y();
		item["camera_pos"]["z"] = c.camera_pos.z();
		item["view_dir"]["x"] = c.view_dir.x();
		item["view_dir"]["y"] = c.view_dir.y();
		item["view_dir"]["z"] = c.view_dir.z();
		item["target_point"]["x"] = c.target_point.x();
		item["target_point"]["y"] = c.target_point.y();
		item["target_point"]["z"] = c.target_point.z();
		item["distance"] = c.distance;
		item["num_visible_faces"] = c.num_visible_faces;
		root.append(item);

		csv_file << i << "," << c.camera_pos.x() << "," << c.camera_pos.y() << "," << c.camera_pos.z() << ","
				  << c.view_dir.x() << "," << c.view_dir.y() << "," << c.view_dir.z() << "," << c.target_point.x()
				  << "," << c.target_point.y() << "," << c.target_point.z() << "," << c.distance << ","
				  << c.num_visible_faces << "\n";
	}

	std::string json_path = output_dir + "/selected_viewpoints.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved JSON: %s\n", json_path.c_str());
	printf("Saved CSV: %s\n", csv_path.c_str());
}

void ExportSelectedRobotPoses(const std::string& output_dir, const std::vector<const ViewpointCandidate*>& selected)
{
	Json::Value root(Json::arrayValue);

	std::string csv_path = output_dir + "/selected_robot_poses.csv";
	std::ofstream csv_file(csv_path);
	csv_file << "id,tcp_x,tcp_y,tcp_z,qx,qy,qz,qw,camera_x,camera_y,camera_z,"
				"view_dir_x,view_dir_y,view_dir_z,num_visible_faces\n";

	for (size_t i = 0; i < selected.size(); ++i)
	{
		const auto& c = *selected[i];
		const auto& tcp_pos = c.tcp_pose.position;
		const auto& tcp_quat = c.tcp_pose.orientation;

		Json::Value item;
		item["id"] = static_cast<int>(i);
		item["tcp_position"]["x"] = tcp_pos.x;
		item["tcp_position"]["y"] = tcp_pos.y;
		item["tcp_position"]["z"] = tcp_pos.z;
		item["tcp_quaternion_xyzw"]["x"] = tcp_quat.x;
		item["tcp_quaternion_xyzw"]["y"] = tcp_quat.y;
		item["tcp_quaternion_xyzw"]["z"] = tcp_quat.z;
		item["tcp_quaternion_xyzw"]["w"] = tcp_quat.w;
		item["camera_position"]["x"] = c.camera_pos.x();
		item["camera_position"]["y"] = c.camera_pos.y();
		item["camera_position"]["z"] = c.camera_pos.z();
		item["view_dir"]["x"] = c.view_dir.x();
		item["view_dir"]["y"] = c.view_dir.y();
		item["view_dir"]["z"] = c.view_dir.z();
		item["num_visible_faces"] = c.num_visible_faces;
		root.append(item);

		csv_file << i << "," << tcp_pos.x << "," << tcp_pos.y << "," << tcp_pos.z << "," << tcp_quat.x << ","
				  << tcp_quat.y << "," << tcp_quat.z << "," << tcp_quat.w << "," << c.camera_pos.x() << ","
				  << c.camera_pos.y() << "," << c.camera_pos.z() << "," << c.view_dir.x() << "," << c.view_dir.y()
				  << "," << c.view_dir.z() << "," << c.num_visible_faces << "\n";
	}

	std::string json_path = output_dir + "/selected_robot_poses.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved robot poses JSON: %s\n", json_path.c_str());
	printf("Saved robot poses CSV: %s\n", csv_path.c_str());
}

visualization_msgs::msg::MarkerArray BuildViewpointMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<ViewpointCandidate>& unreachable_visible)
{
	visualization_msgs::msg::MarkerArray markers;

	const double arrow_length = 0.02;
	int id = 0;

	// The mesh itself, so it renders alongside the viewpoints (mirrors trimesh's semi-transparent
	// gray preview in visualize_views()).
	Eigen::Quaterniond object_quat(object_rotation_world);
	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "object_mesh";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose.position = ToPoint(object_translation_world);
	mesh_marker.pose.orientation.x = object_quat.x();
	mesh_marker.pose.orientation.y = object_quat.y();
	mesh_marker.pose.orientation.z = object_quat.z();
	mesh_marker.pose.orientation.w = object_quat.w();
	mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = mesh_scale;
	mesh_marker.color.r = 0.7f;
	mesh_marker.color.g = 0.7f;
	mesh_marker.color.b = 0.7f;
	mesh_marker.color.a = 0.5f;
	markers.markers.push_back(mesh_marker);

	auto addCandidateMarkers = [&](const ViewpointCandidate& c, const std::string& ns, float r, float g, float b) {
		Eigen::Vector3d world_pos = object_rotation_world * c.camera_pos + object_translation_world;
		Eigen::Vector3d world_dir = (object_rotation_world * c.view_dir).normalized();

		visualization_msgs::msg::Marker sphere;
		sphere.header.frame_id = "world";
		sphere.header.stamp = stamp;
		sphere.ns = ns + "_camera_pos";
		sphere.id = id++;
		sphere.type = visualization_msgs::msg::Marker::SPHERE;
		sphere.action = visualization_msgs::msg::Marker::ADD;
		sphere.pose.position = ToPoint(world_pos);
		sphere.pose.orientation.w = 1.0;
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.006;
		sphere.color.r = r;
		sphere.color.g = g;
		sphere.color.b = b;
		sphere.color.a = 1.0;
		markers.markers.push_back(sphere);

		visualization_msgs::msg::Marker arrow;
		arrow.header.frame_id = "world";
		arrow.header.stamp = stamp;
		arrow.ns = ns + "_view_dir";
		arrow.id = id++;
		arrow.type = visualization_msgs::msg::Marker::ARROW;
		arrow.action = visualization_msgs::msg::Marker::ADD;
		arrow.points.push_back(ToPoint(world_pos));
		arrow.points.push_back(ToPoint(world_pos + world_dir * arrow_length));
		arrow.scale.x = 0.001; // shaft diameter
		arrow.scale.y = 0.003; // head diameter
		arrow.scale.z = 0.0;	// auto head length
		arrow.color.r = r;
		arrow.color.g = g;
		arrow.color.b = b;
		arrow.color.a = 1.0;
		markers.markers.push_back(arrow);
	};

	for (const auto* c : selected)
		addCandidateMarkers(*c, "selected", 0.0f, 1.0f, 0.0f);

	for (const auto& c : unreachable_visible)
		addCandidateMarkers(c, "unreachable", 1.0f, 0.5f, 0.0f);

	// Visiting order (the tour's final ordering, whichever algorithm produced it).
	if (selected.size() >= 2)
	{
		visualization_msgs::msg::Marker tour;
		tour.header.frame_id = "world";
		tour.header.stamp = stamp;
		tour.ns = "selected_tour";
		tour.id = id++;
		tour.type = visualization_msgs::msg::Marker::LINE_STRIP;
		tour.action = visualization_msgs::msg::Marker::ADD;
		tour.scale.x = 0.0015; // line width
		tour.color.r = 1.0f;
		tour.color.g = 1.0f;
		tour.color.b = 0.0f;
		tour.color.a = 0.8f;

		for (const auto* c : selected)
		{
			Eigen::Vector3d world_pos = object_rotation_world * c->camera_pos + object_translation_world;
			tour.points.push_back(ToPoint(world_pos));
		}
		markers.markers.push_back(tour);
	}

	return markers;
}

visualization_msgs::msg::MarkerArray BuildMeshComparisonMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	const std::string& resolved_simplified_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Vector3d& mesh_comparison_offset)
{
	visualization_msgs::msg::MarkerArray markers;

	Eigen::Quaterniond object_quat(object_rotation_world);
	int id = 0;

	// Original mesh, at the object's real pose (unshifted). resolved_mesh_path points at the raw,
	// unscaled STL, so mesh_scale still needs to be applied here.
	visualization_msgs::msg::Marker original_marker;
	original_marker.header.frame_id = "world";
	original_marker.header.stamp = stamp;
	original_marker.ns = "original_mesh";
	original_marker.id = id++;
	original_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	original_marker.action = visualization_msgs::msg::Marker::ADD;
	original_marker.mesh_resource = "file://" + resolved_mesh_path;
	original_marker.mesh_use_embedded_materials = false;
	original_marker.pose.position = ToPoint(object_translation_world);
	original_marker.pose.orientation.x = object_quat.x();
	original_marker.pose.orientation.y = object_quat.y();
	original_marker.pose.orientation.z = object_quat.z();
	original_marker.pose.orientation.w = object_quat.w();
	original_marker.scale.x = original_marker.scale.y = original_marker.scale.z = mesh_scale;
	original_marker.color.r = 0.7f;
	original_marker.color.g = 0.7f;
	original_marker.color.b = 0.7f;
	original_marker.color.a = 0.8f;
	markers.markers.push_back(original_marker);

	// Simplified (decimated) mesh, shifted by mesh_comparison_offset so it doesn't overlap the
	// original. resolved_simplified_mesh_path was exported from the already-scaled in-memory mesh,
	// so no marker-level scale needed.
	visualization_msgs::msg::Marker simplified_marker;
	simplified_marker.header.frame_id = "world";
	simplified_marker.header.stamp = stamp;
	simplified_marker.ns = "simplified_mesh";
	simplified_marker.id = id++;
	simplified_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	simplified_marker.action = visualization_msgs::msg::Marker::ADD;
	simplified_marker.mesh_resource = "file://" + resolved_simplified_mesh_path;
	simplified_marker.mesh_use_embedded_materials = false;
	simplified_marker.pose.position = ToPoint(object_translation_world + mesh_comparison_offset);
	simplified_marker.pose.orientation.x = object_quat.x();
	simplified_marker.pose.orientation.y = object_quat.y();
	simplified_marker.pose.orientation.z = object_quat.z();
	simplified_marker.pose.orientation.w = object_quat.w();
	simplified_marker.scale.x = simplified_marker.scale.y = simplified_marker.scale.z = 1.0;
	simplified_marker.color.r = 0.2f;
	simplified_marker.color.g = 0.8f;
	simplified_marker.color.b = 0.2f;
	simplified_marker.color.a = 0.8f;
	markers.markers.push_back(simplified_marker);

	return markers;
}

visualization_msgs::msg::MarkerArray BuildCurrentTargetMarkerArray(
	const rclcpp::Time& stamp, const geometry_msgs::msg::Pose& pose)
{
	visualization_msgs::msg::MarkerArray markers;

	visualization_msgs::msg::Marker sphere;
	sphere.header.frame_id = "world";
	sphere.header.stamp = stamp;
	sphere.ns = "current_target_pos";
	sphere.id = 0;
	sphere.type = visualization_msgs::msg::Marker::SPHERE;
	sphere.action = visualization_msgs::msg::Marker::ADD;
	sphere.pose.position = pose.position;
	sphere.pose.orientation.w = 1.0;
	sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
	sphere.color.r = 1.0f;
	sphere.color.a = 1.0f;
	markers.markers.push_back(sphere);

	// Marker::ARROW points along local +X, but the TCP frame's local +Z is the view direction --
	// rotate -90deg about Y to map +X onto +Z.
	Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	Eigen::Quaterniond q_x_to_z(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()));
	Eigen::Quaterniond q_arrow = q * q_x_to_z;

	visualization_msgs::msg::Marker arrow;
	arrow.header.frame_id = "world";
	arrow.header.stamp = stamp;
	arrow.ns = "current_target_dir";
	arrow.id = 0;
	arrow.type = visualization_msgs::msg::Marker::ARROW;
	arrow.action = visualization_msgs::msg::Marker::ADD;
	arrow.pose.position = pose.position;
	arrow.pose.orientation.x = q_arrow.x();
	arrow.pose.orientation.y = q_arrow.y();
	arrow.pose.orientation.z = q_arrow.z();
	arrow.pose.orientation.w = q_arrow.w();
	arrow.scale.x = 0.03;	// arrow length
	arrow.scale.y = 0.006; // shaft diameter
	arrow.scale.z = 0.006; // head diameter
	arrow.color.r = 1.0f;
	arrow.color.a = 1.0f;
	markers.markers.push_back(arrow);

	return markers;
}

geometry_msgs::msg::PoseArray BuildWaypointPoseArray(
	const rclcpp::Time& stamp, const std::vector<const ViewpointCandidate*>& selected)
{
	geometry_msgs::msg::PoseArray msg;
	msg.header.frame_id = "world";
	msg.header.stamp = stamp;
	msg.poses.reserve(selected.size());
	for (const auto* c : selected)
		msg.poses.push_back(c->tcp_pose);
	return msg;
}
