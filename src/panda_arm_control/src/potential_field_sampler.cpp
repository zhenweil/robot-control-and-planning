#include "panda_arm_control/potential_field_sampler.hpp"

#include "panda_arm_control/mesh_utils.hpp"

namespace
{

// Net attraction toward mesh surface, evaluated at camera_pos: each face pulls in its own
// direction, weighted by its area and falling off as 1/distance^2 (so area * unit_vector /
// distance^2 = area * to_face / distance^3).
Eigen::Vector3d ComputeAttractionDirection(const MeshData& mesh, const Eigen::Vector3d& camera_pos)
{
	Eigen::Vector3d pull = Eigen::Vector3d::Zero();

	for (size_t i = 0; i < mesh.NumFaces(); ++i)
	{
		Eigen::Vector3d to_face = mesh.face_centers[i] - camera_pos;
		double dist = to_face.norm();
		if (dist < 1e-9)
			continue;

		pull += mesh.face_areas[i] * to_face / (dist * dist * dist);
	}

	return pull.normalized();
}

} // namespace

std::vector<ViewpointCandidate> GeneratePotentialFieldViewCandidates(
	const MeshData& mesh,
	int n_surface_samples,
	const std::vector<double>& standoff_distances,
	std::mt19937& rng)
{
	SurfaceSampler sampler(mesh);

	std::vector<Eigen::Vector3d> points, normals;
	std::vector<int> face_ids;
	sampler.Sample(n_surface_samples, rng, points, normals, face_ids);

	std::vector<ViewpointCandidate> candidates;
	candidates.reserve(points.size() * standoff_distances.size());

	for (size_t k = 0; k < points.size(); ++k)
	{
		Eigen::Vector3d n = normals[k].normalized();

		for (double d : standoff_distances)
		{
			Eigen::Vector3d camera_pos = points[k] + d * n;
			Eigen::Vector3d view_dir = ComputeAttractionDirection(mesh, camera_pos);

			ViewpointCandidate c;
			c.camera_pos = camera_pos;
			c.view_dir = view_dir;
			c.target_point = points[k]; // anchor point used to place the camera, not necessarily
										 // where view_dir ends up pointing
			c.seed_face = face_ids[k];
			c.distance = d;
			candidates.push_back(std::move(c));
		}
	}

	return candidates;
}
