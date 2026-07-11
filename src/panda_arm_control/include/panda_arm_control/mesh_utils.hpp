#pragma once

#include <random>
#include <string>
#include <vector>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "panda_arm_control/viewpoint_types.hpp"

// Load an STL mesh and uniformly scale it (e.g. 0.01 to convert cm -> m).
// Mirrors trimesh.load(mesh_path).apply_scale(scale).
vtkSmartPointer<vtkPolyData> LoadAndScaleMesh(const std::string& stl_path, double scale);

// Writes poly_data to a binary STL file (RViz's MESH_RESOURCE marker requires binary, not ASCII).
void ExportMeshToStl(vtkSmartPointer<vtkPolyData> poly_data, const std::string& path);

// Decimate a mesh down to approximately target_faces triangles, then clean up
// unreferenced points and fix normal orientation.
// Mirrors simplify_mesh() in sample_viewpoints.py.
vtkSmartPointer<vtkPolyData> SimplifyMesh(vtkSmartPointer<vtkPolyData> input, int target_faces);

// Compute per-face centers/normals/areas for a triangle mesh.
MeshData BuildMeshData(vtkSmartPointer<vtkPolyData> poly_data);

// Boundary/non-manifold edge check. Mirrors mesh.is_watertight (approximately).
bool IsWatertight(vtkSmartPointer<vtkPolyData> poly_data);

// Reject candidates whose camera_pos is inside original_mesh (only checked if
// original_mesh is watertight) or closer than min_clearance to its surface.
// Mirrors filter_candidates_by_clearance().
std::vector<ViewpointCandidate> FilterByClearance(
	vtkSmartPointer<vtkPolyData> original_poly_data,
	std::vector<ViewpointCandidate> candidates,
	double min_clearance);

// Ray-casts against a simplified mesh to determine, for a single candidate view,
// which faces are visible (in FOV, in range, facing the camera, and not occluded).
// Mirrors visible_faces_from_view().
class VisibilityChecker
{
public:
	explicit VisibilityChecker(const MeshData& simplified_mesh);

	// Fills candidate.visible_mask / candidate.num_visible_faces in place.
	void ComputeVisibleFaces(
		ViewpointCandidate& candidate,
		double fov_deg,
		double max_distance,
		double angle_threshold_deg,
		int max_rays_per_view) const;

private:
	const MeshData* mesh;
	vtkSmartPointer<class vtkOBBTree> ray_tree;
};

// Runs VisibilityChecker over every candidate and drops those with zero visible faces.
// Mirrors compute_visibility().
std::vector<ViewpointCandidate> ComputeVisibility(
	const VisibilityChecker& checker,
	std::vector<ViewpointCandidate> candidates,
	double fov_deg,
	double max_distance,
	double angle_threshold_deg,
	int max_rays_per_view);

// Weighted-by-triangle-area random surface sampling. Mirrors trimesh.sample.sample_surface.
class SurfaceSampler
{
public:
	explicit SurfaceSampler(const MeshData& mesh);

	void Sample(
		int n_samples,
		std::mt19937& rng,
		std::vector<Eigen::Vector3d>& points,
		std::vector<Eigen::Vector3d>& normals,
		std::vector<int>& face_ids) const;

private:
	const MeshData* mesh;
	std::vector<double> cumulative_area;
};

// Generates the standoff/tilt candidate grid around sampled surface points.
// Mirrors generate_view_candidates().
std::vector<ViewpointCandidate> GenerateViewCandidates(
	const MeshData& mesh,
	int n_surface_samples,
	const std::vector<double>& standoff_distances,
	const std::vector<double>& tilt_angles_deg,
	std::mt19937& rng);

// Greedy weighted set-cover over candidate visible faces.
// Mirrors greedy_select_viewpoints_target_visibility().
std::vector<const ViewpointCandidate*> GreedySelectViewpoints(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	double target_area_visibility,
	double min_new_area_ratio);

// Cost between two waypoints (or a start reference and a waypoint) used for tour ordering:
// euclidean_distance_m + joint_distance_weight * joint_distance_rad, where joint_distance_rad
// is the L2 norm of the per-joint difference. Combines Cartesian TCP distance with actual
// joint-space distance, since two poses can be close in XYZ but require very different arm
// configurations (or vice versa). Falls back to euclidean-only if either joint vector is empty.
double TourCost(
	const Eigen::Vector3d& pos_a, const std::vector<double>& joints_a, const Eigen::Vector3d& pos_b,
	const std::vector<double>& joints_b, double joint_distance_weight);

// Greedy nearest-neighbor tour over candidates: starts at whichever candidate is closest (by
// TourCost) to the start reference, then repeatedly visits the closest remaining candidate.
// Produces a sensible robot visiting order (not an optimal TSP solution), so consecutive
// viewpoints don't require large arbitrary jumps.
std::vector<const ViewpointCandidate*> NearestNeighborOrder(
	std::vector<const ViewpointCandidate*> candidates,
	const Eigen::Vector3d& start_reference_position,
	const std::vector<double>& start_reference_joints,
	double joint_distance_weight);

// 2-opt local search over an existing tour (e.g. NearestNeighborOrder's output): repeatedly
// reverses whichever segment [i, j] most shortens the total TourCost (start_reference ->
// ordered[0] -> ... -> ordered.back(), no closing edge back to start) until no single reversal
// improves it further. Removes the long crossing/backtracking edges nearest-neighbor
// construction tends to leave behind.
std::vector<const ViewpointCandidate*> TwoOptImprove(
	std::vector<const ViewpointCandidate*> ordered,
	const Eigen::Vector3d& start_reference_position,
	const std::vector<double>& start_reference_joints,
	double joint_distance_weight);
