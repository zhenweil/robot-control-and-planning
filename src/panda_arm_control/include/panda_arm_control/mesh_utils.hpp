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

// Greedy weighted set-cover over candidate visible faces: repeatedly picks whichever remaining
// candidate covers the most previously-uncovered area, stopping once target_area_visibility is
// reached or no candidate contributes any new coverage (gain <= 0). No minimum-gain threshold, so
// it keeps accepting small marginal contributions rather than stopping early.
std::vector<const ViewpointCandidate*> GreedySelectViewpoints(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	double target_area_visibility);
