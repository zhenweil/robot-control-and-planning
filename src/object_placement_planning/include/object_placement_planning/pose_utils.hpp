#pragma once

#include <cmath>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

// Port of camera_rotation_from_view_dir() from
// src/path_planning_py/path_planning_py/read_viewpoint_and_publish.py
//
// Camera local +Z axis points along view_dir_world.
inline Eigen::Matrix3d CameraRotationFromViewDir(
	const Eigen::Vector3d& view_dir_world,
	Eigen::Vector3d world_up = Eigen::Vector3d(0.0, 0.0, 1.0))
{
	Eigen::Vector3d z_cam = view_dir_world.normalized();

	// Avoid singularity when z_cam is parallel to world_up.
	if (std::abs(z_cam.dot(world_up)) > 0.95)
		world_up = Eigen::Vector3d(0.0, 1.0, 0.0);

	Eigen::Vector3d x_cam = world_up.cross(z_cam).normalized();
	Eigen::Vector3d y_cam = z_cam.cross(x_cam).normalized();

	Eigen::Matrix3d R;
	R.col(0) = x_cam;
	R.col(1) = y_cam;
	R.col(2) = z_cam;
	return R;
}

// Port of convert_viewpoint_to_tcp_pose() from read_viewpoint_and_publish.py.
//
// camera_pos_object / view_dir_object are expressed in the mesh/object frame.
// object_rotation_world/object_translation_world place the object in the world frame.
// t_tcp_camera is the fixed transform from TCP frame to camera frame
// (p_camera = t_tcp_camera * p_tcp).
inline geometry_msgs::msg::Pose ConvertViewpointToTcpPose(
	const Eigen::Vector3d& camera_pos_object,
	const Eigen::Vector3d& view_dir_object,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera)
{
	Eigen::Vector3d camera_pos_world = object_rotation_world * camera_pos_object + object_translation_world;
	Eigen::Vector3d view_dir_world = object_rotation_world * view_dir_object.normalized();

	Eigen::Isometry3d T_world_camera = Eigen::Isometry3d::Identity();
	T_world_camera.linear() = CameraRotationFromViewDir(view_dir_world);
	T_world_camera.translation() = camera_pos_world;

	Eigen::Isometry3d T_world_tcp = T_world_camera * t_tcp_camera.inverse();
	Eigen::Quaterniond q(T_world_tcp.rotation());

	geometry_msgs::msg::Pose pose;
	pose.position.x = T_world_tcp.translation().x();
	pose.position.y = T_world_tcp.translation().y();
	pose.position.z = T_world_tcp.translation().z();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	return pose;
}
