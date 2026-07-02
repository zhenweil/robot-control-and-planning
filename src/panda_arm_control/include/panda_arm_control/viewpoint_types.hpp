#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

struct MeshData
{
	vtkSmartPointer<vtkPolyData> poly_data;
	std::vector<Eigen::Vector3d> face_centers;
	std::vector<Eigen::Vector3d> face_normals;
	std::vector<double> face_areas;
	double total_area = 0.0;

	size_t NumFaces() const { return face_centers.size(); }
};

struct ViewpointCandidate
{
	Eigen::Vector3d camera_pos = Eigen::Vector3d::Zero();
	Eigen::Vector3d view_dir = Eigen::Vector3d::Zero();
	Eigen::Vector3d target_point = Eigen::Vector3d::Zero();
	int seed_face = -1;
	double distance = 0.0;

	// visible_mask.size() == simplified mesh face count; 1 = face visible from this candidate.
	std::vector<uint8_t> visible_mask;
	int num_visible_faces = 0;

	bool reachable = false;
	geometry_msgs::msg::Pose tcp_pose;
};
