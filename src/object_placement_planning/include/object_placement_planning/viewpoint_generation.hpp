#pragma once
// Viewpoints that cover the object's surface, with no robot reachability check: candidates around the mesh, clearance
// and visibility filters, then greedy set cover. TCP poses are for the object at its given (nominal) pose.

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "object_placement_planning/viewpoint_types.hpp"

namespace viewpoint_generation
{

struct Settings
{
	std::string mesh_path;
	double mesh_scale = 0.01;
	int target_faces = 1000;  // simplified mesh used for candidates and visibility
	std::string candidate_generation_method = "potential_field";  // or "grid"
	int n_surface_samples = 60;
	std::vector<double> standoff_distances = {0.01, 0.02, 0.03};  // m
	std::vector<double> tilt_angles_deg = {0.0, 15.0, -15.0};  // grid method only
	double min_clearance = 0.005;  // m
	double fov_deg = 50.0;
	double max_distance = 30.0;
	double angle_threshold_deg = 70.0;
	int max_rays_per_view = 2000;
	double target_area_visibility = 0.9;
	int random_seed = 42;
	Eigen::Vector3d object_translation_world = Eigen::Vector3d(0.5, 0.0, 0.15);
	Eigen::Matrix3d object_rotation_world = Eigen::Matrix3d::Identity();
	Eigen::Isometry3d t_tcp_camera = Eigen::Isometry3d::Identity();
};

struct GeneratedViewpoints
{
	std::vector<ViewpointCandidate> selected;  // in selection order, tcp_pose filled
	int num_candidates = 0;  // before the clearance and visibility filters
	int num_visible_candidates = 0;
	double area_visibility = 0.0;  // fraction of the simplified mesh's area the selected viewpoints see
};

GeneratedViewpoints GenerateViewpoints(const Settings& settings);

}  // namespace viewpoint_generation
