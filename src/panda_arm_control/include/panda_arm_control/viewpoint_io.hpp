#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/viewpoint_types.hpp"

// Shared geometry helpers used when placing the object / building markers, common to every
// viewpoint_planner_* node.
Eigen::Vector3d ToVector3(const std::vector<double>& v, const Eigen::Vector3d& fallback);
Eigen::Matrix3d RotationFromRpyDeg(const std::vector<double>& rpy_deg);
Eigen::Isometry3d IsometryFromXyzQuat(const std::vector<double>& xyz, const std::vector<double>& quat_xyzw);
geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v);

// Writes selected_viewpoints.{json,csv} (camera_pos/view_dir/target_point/distance/num_visible_faces)
// to output_dir.
void ExportSelectedViewpoints(const std::string& output_dir, const std::vector<const ViewpointCandidate*>& selected);

// Writes selected_robot_poses.{json,csv} (tcp_pose/camera_pos/view_dir/num_visible_faces) to
// output_dir.
void ExportSelectedRobotPoses(const std::string& output_dir, const std::vector<const ViewpointCandidate*>& selected);

// Object mesh + selected (green) + unreachable-visible (orange) candidate markers, plus the
// visiting-order tour line, for /viewpoint_markers.
visualization_msgs::msg::MarkerArray BuildViewpointMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<ViewpointCandidate>& unreachable_visible);

// Original mesh (at the object's real pose) and simplified/decimated mesh (offset by
// mesh_comparison_offset), side by side, for /mesh_comparison_markers.
visualization_msgs::msg::MarkerArray BuildMeshComparisonMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	const std::string& resolved_simplified_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Vector3d& mesh_comparison_offset);

// Red sphere + arrow highlighting the viewpoint the robot is currently driving to during real
// execution, for /viewpoint_markers -- shared by every node's ExecuteTourOnRobot call.
visualization_msgs::msg::MarkerArray BuildCurrentTargetMarkerArray(
	const rclcpp::Time& stamp, const geometry_msgs::msg::Pose& pose);

// selected[i]->tcp_pose for every i, as a world-frame PoseArray, for /cartesian_waypoints.
geometry_msgs::msg::PoseArray BuildWaypointPoseArray(
	const rclcpp::Time& stamp, const std::vector<const ViewpointCandidate*>& selected);
