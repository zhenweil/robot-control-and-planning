#include "object_placement_planning/viewpoint_generation.hpp"

#include <random>

#include "object_placement_planning/mesh_utils.hpp"
#include "object_placement_planning/pose_utils.hpp"
#include "object_placement_planning/potential_field_sampler.hpp"
#include "object_placement_planning/real_cost_planning.hpp"

namespace viewpoint_generation
{

GeneratedViewpoints GenerateViewpoints(const Settings& settings)
{
	const vtkSmartPointer<vtkPolyData> original_mesh = LoadAndScaleMesh(settings.mesh_path, settings.mesh_scale);
	const MeshData simplified_mesh = BuildMeshData(SimplifyMesh(original_mesh, settings.target_faces));

	std::mt19937 rng(static_cast<unsigned int>(settings.random_seed));
	std::vector<ViewpointCandidate> candidates = settings.candidate_generation_method == "grid" ? GenerateViewCandidates(simplified_mesh, settings.n_surface_samples, settings.standoff_distances, settings.tilt_angles_deg, rng) : GeneratePotentialFieldViewCandidates(simplified_mesh, settings.n_surface_samples, settings.standoff_distances, rng);
	GeneratedViewpoints result;
	result.num_candidates = static_cast<int>(candidates.size());

	candidates = FilterByClearance(original_mesh, std::move(candidates), settings.min_clearance);
	const VisibilityChecker visibility_checker(simplified_mesh);
	candidates = ComputeVisibility(visibility_checker, std::move(candidates), settings.fov_deg, settings.max_distance, settings.angle_threshold_deg, settings.max_rays_per_view);
	result.num_visible_candidates = static_cast<int>(candidates.size());

	const std::vector<const ViewpointCandidate*> selected = GreedySelectViewpoints(simplified_mesh, candidates, settings.target_area_visibility);
	result.area_visibility = ComputeAreaVisibility(simplified_mesh, selected);
	for (const ViewpointCandidate* candidate : selected)
	{
		ViewpointCandidate viewpoint = *candidate;
		viewpoint.tcp_pose = ConvertViewpointToTcpPose(viewpoint.camera_pos, viewpoint.view_dir, settings.object_translation_world, settings.object_rotation_world, settings.t_tcp_camera);
		result.selected.push_back(std::move(viewpoint));
	}
	return result;
}

}  // namespace viewpoint_generation
