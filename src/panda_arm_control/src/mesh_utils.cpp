#include "panda_arm_control/mesh_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <vtkCleanPolyData.h>
#include <vtkFeatureEdges.h>
#include <vtkNew.h>
#include <vtkOBBTree.h>
#include <vtkPolyDataNormals.h>
#include <vtkQuadricDecimation.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkStaticCellLocator.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace
{

Eigen::Vector3d ToEigen(const double p[3])
{
	return Eigen::Vector3d(p[0], p[1], p[2]);
}

void GetTriangleVertices(
	vtkSmartPointer<vtkPolyData> poly_data,
	vtkIdType cell_id,
	Eigen::Vector3d& v0,
	Eigen::Vector3d& v1,
	Eigen::Vector3d& v2)
{
	vtkIdType npts;
	vtkIdType const* pts;
	poly_data->GetCellPoints(cell_id, npts, pts);

	double p0[3], p1[3], p2[3];
	poly_data->GetPoint(pts[0], p0);
	poly_data->GetPoint(pts[1], p1);
	poly_data->GetPoint(pts[2], p2);

	v0 = ToEigen(p0);
	v1 = ToEigen(p1);
	v2 = ToEigen(p2);
}

} // namespace

vtkSmartPointer<vtkPolyData> LoadAndScaleMesh(const std::string& stl_path, double scale)
{
	vtkNew<vtkSTLReader> reader;
	reader->SetFileName(stl_path.c_str());
	reader->Update();

	vtkNew<vtkTransform> transform;
	transform->Scale(scale, scale, scale);

	vtkNew<vtkTransformPolyDataFilter> transform_filter;
	transform_filter->SetInputConnection(reader->GetOutputPort());
	transform_filter->SetTransform(transform);
	transform_filter->Update();

	vtkSmartPointer<vtkPolyData> out = vtkSmartPointer<vtkPolyData>::New();
	out->DeepCopy(transform_filter->GetOutput());
	return out;
}

void ExportMeshToStl(vtkSmartPointer<vtkPolyData> poly_data, const std::string& path)
{
	vtkNew<vtkSTLWriter> writer;
	writer->SetFileName(path.c_str());
	writer->SetInputData(poly_data);
	writer->SetFileTypeToBinary(); // RViz's MESH_RESOURCE marker requires binary STL.
	writer->Write();
}

vtkSmartPointer<vtkPolyData> SimplifyMesh(vtkSmartPointer<vtkPolyData> input, int target_faces)
{
	vtkIdType n_faces = input->GetNumberOfPolys();

	printf("Original faces: %lld\n", static_cast<long long>(n_faces));
	printf("Original vertices: %lld\n", static_cast<long long>(input->GetNumberOfPoints()));

	double reduction = 1.0 - static_cast<double>(target_faces) / static_cast<double>(n_faces);
	reduction = std::clamp(reduction, 0.0, 1.0);

	vtkNew<vtkQuadricDecimation> deci;
	deci->SetInputData(input);
	deci->SetTargetReduction(reduction);
	deci->Update();

	// remove_unreferenced_vertices()
	vtkNew<vtkCleanPolyData> clean;
	clean->SetInputConnection(deci->GetOutputPort());
	clean->Update();

	// fix_normals()
	vtkNew<vtkPolyDataNormals> normals;
	normals->SetInputConnection(clean->GetOutputPort());
	normals->ConsistencyOn();
	normals->AutoOrientNormalsOn();
	normals->ComputeCellNormalsOn();
	normals->SplittingOff();
	normals->Update();

	vtkSmartPointer<vtkPolyData> out = vtkSmartPointer<vtkPolyData>::New();
	out->DeepCopy(normals->GetOutput());

	printf("Simplified faces: %lld\n", static_cast<long long>(out->GetNumberOfPolys()));
	printf("Simplified vertices: %lld\n", static_cast<long long>(out->GetNumberOfPoints()));

	return out;
}

MeshData BuildMeshData(vtkSmartPointer<vtkPolyData> poly_data)
{
	poly_data->BuildCells();
	vtkIdType n_cells = poly_data->GetNumberOfCells();

	MeshData mesh;
	mesh.poly_data = poly_data;
	mesh.face_centers.resize(n_cells);
	mesh.face_normals.resize(n_cells);
	mesh.face_areas.resize(n_cells);
	mesh.total_area = 0.0;

	for (vtkIdType i = 0; i < n_cells; ++i)
	{
		Eigen::Vector3d v0, v1, v2;
		GetTriangleVertices(poly_data, i, v0, v1, v2);

		Eigen::Vector3d cross_v = (v1 - v0).cross(v2 - v0);
		double area = 0.5 * cross_v.norm();

		mesh.face_centers[i] = (v0 + v1 + v2) / 3.0;
		mesh.face_normals[i] = (area > 1e-12) ? Eigen::Vector3d(cross_v / (2.0 * area)) : Eigen::Vector3d(0, 0, 1);
		mesh.face_areas[i] = area;
		mesh.total_area += area;
	}

	return mesh;
}

bool IsWatertight(vtkSmartPointer<vtkPolyData> poly_data)
{
	vtkNew<vtkFeatureEdges> fe;
	fe->SetInputData(poly_data);
	fe->BoundaryEdgesOn();
	fe->NonManifoldEdgesOn();
	fe->FeatureEdgesOff();
	fe->ManifoldEdgesOff();
	fe->Update();

	return fe->GetOutput()->GetNumberOfLines() == 0;
}

std::vector<ViewpointCandidate> FilterByClearance(
	vtkSmartPointer<vtkPolyData> original_poly_data,
	std::vector<ViewpointCandidate> candidates,
	double min_clearance)
{
	size_t n_before = candidates.size();

	bool watertight = IsWatertight(original_poly_data);
	if (!watertight)
		printf("Warning: mesh is not watertight. inside/outside test may be unreliable.\n");

	vtkNew<vtkSelectEnclosedPoints> enclosed;
	if (watertight)
		enclosed->Initialize(original_poly_data);

	vtkNew<vtkStaticCellLocator> locator;
	locator->SetDataSet(original_poly_data);
	locator->BuildLocator();

	std::vector<ViewpointCandidate> filtered;
	filtered.reserve(candidates.size());

	for (auto& c : candidates)
	{
		double x[3] = {c.camera_pos.x(), c.camera_pos.y(), c.camera_pos.z()};

		bool is_inside = watertight && enclosed->IsInsideSurface(x) != 0;
		if (is_inside)
			continue;

		double closest[3];
		vtkIdType cell_id;
		int sub_id;
		double dist2;
		locator->FindClosestPoint(x, closest, cell_id, sub_id, dist2);
		double dist = std::sqrt(dist2);

		if (dist < min_clearance)
			continue;

		filtered.push_back(std::move(c));
	}

	if (watertight)
		enclosed->Complete();

	printf("candidates before clearance filter: %zu\n", n_before);
	printf("candidates after clearance filter: %zu\n", filtered.size());

	return filtered;
}

VisibilityChecker::VisibilityChecker(const MeshData& simplified_mesh) : mesh(&simplified_mesh)
{
	vtkNew<vtkOBBTree> ray_tree;
	ray_tree->SetDataSet(simplified_mesh.poly_data);
	ray_tree->BuildLocator();
	this->ray_tree = ray_tree;
}

void VisibilityChecker::ComputeVisibleFaces(
	ViewpointCandidate& candidate,
	double fov_deg,
	double max_distance,
	double angle_threshold_deg,
	int max_rays_per_view) const
{
	const size_t n = this->mesh->NumFaces();
	candidate.visible_mask.assign(n, 0);
	candidate.num_visible_faces = 0;

	const double cos_fov = std::cos(fov_deg * 0.5 * M_PI / 180.0);
	const double cos_angle_threshold = std::cos(angle_threshold_deg * M_PI / 180.0);

	std::vector<int> valid_faces;
	std::vector<double> valid_dist;
	valid_faces.reserve(n);
	valid_dist.reserve(n);

	for (size_t f = 0; f < n; ++f)
	{
		Eigen::Vector3d to_face = this->mesh->face_centers[f] - candidate.camera_pos;
		double dist = to_face.norm();
		Eigen::Vector3d dir = to_face / (dist + 1e-9);

		bool in_fov = dir.dot(candidate.view_dir) > cos_fov;
		bool in_range = dist < max_distance;
		bool good_angle = (-dir).dot(this->mesh->face_normals[f]) > cos_angle_threshold;

		if (in_fov && in_range && good_angle)
		{
			valid_faces.push_back(static_cast<int>(f));
			valid_dist.push_back(dist);
		}
	}

	if (valid_faces.empty())
		return;

	// Cap to the closest max_rays_per_view faces, mirroring np.argsort(distances)[:max_rays_per_view]
	if (max_rays_per_view > 0 && static_cast<int>(valid_faces.size()) > max_rays_per_view)
	{
		std::vector<int> order(valid_faces.size());
		std::iota(order.begin(), order.end(), 0);
		std::partial_sort(
			order.begin(), order.begin() + max_rays_per_view, order.end(),
			[&](int a, int b) { return valid_dist[a] < valid_dist[b]; });
		order.resize(max_rays_per_view);

		std::vector<int> capped;
		capped.reserve(order.size());
		for (int idx : order)
			capped.push_back(valid_faces[idx]);
		valid_faces.swap(capped);
	}

	int visible_count = 0;
	for (int expected_face : valid_faces)
	{
		Eigen::Vector3d to_face = this->mesh->face_centers[expected_face] - candidate.camera_pos;
		double target_dist = to_face.norm();
		Eigen::Vector3d dir = to_face / (target_dist + 1e-9);
		// Extend slightly past the target face center so the finite-segment query
		// doesn't miss a hit that lands exactly at the segment endpoint.
		Eigen::Vector3d p2 = candidate.camera_pos + dir * (target_dist + 1e-4);

		double p1_arr[3] = {candidate.camera_pos.x(), candidate.camera_pos.y(), candidate.camera_pos.z()};
		double p2_arr[3] = {p2.x(), p2.y(), p2.z()};
		double t, x[3], pcoords[3];
		int sub_id;
		vtkIdType hit_cell_id = -1;

		int hit = this->ray_tree->IntersectWithLine(p1_arr, p2_arr, 1e-6, t, x, pcoords, sub_id, hit_cell_id);

		if (hit && hit_cell_id == static_cast<vtkIdType>(expected_face))
		{
			candidate.visible_mask[expected_face] = 1;
			++visible_count;
		}
	}

	candidate.num_visible_faces = visible_count;
}

std::vector<ViewpointCandidate> ComputeVisibility(
	const VisibilityChecker& checker,
	std::vector<ViewpointCandidate> candidates,
	double fov_deg,
	double max_distance,
	double angle_threshold_deg,
	int max_rays_per_view)
{
	std::vector<ViewpointCandidate> valid_candidates;
	valid_candidates.reserve(candidates.size());

	for (size_t i = 0; i < candidates.size(); ++i)
	{
		auto& c = candidates[i];
		checker.ComputeVisibleFaces(c, fov_deg, max_distance, angle_threshold_deg, max_rays_per_view);

		if (i % 10 == 0)
			printf("candidate %zu/%zu, visible faces = %d\n", i, candidates.size(), c.num_visible_faces);

		if (c.num_visible_faces == 0)
			continue;

		valid_candidates.push_back(std::move(c));
	}

	return valid_candidates;
}

SurfaceSampler::SurfaceSampler(const MeshData& mesh_data) : mesh(&mesh_data)
{
	this->cumulative_area.resize(mesh_data.NumFaces());
	double running = 0.0;
	for (size_t i = 0; i < mesh_data.NumFaces(); ++i)
	{
		running += mesh_data.face_areas[i];
		this->cumulative_area[i] = running;
	}
}

void SurfaceSampler::Sample(
	int n_samples,
	std::mt19937& rng,
	std::vector<Eigen::Vector3d>& points,
	std::vector<Eigen::Vector3d>& normals,
	std::vector<int>& face_ids) const
{
	std::uniform_real_distribution<double> area_dist(0.0, this->cumulative_area.back());
	std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

	points.resize(n_samples);
	normals.resize(n_samples);
	face_ids.resize(n_samples);

	for (int s = 0; s < n_samples; ++s)
	{
		double r = area_dist(rng);
		int f = static_cast<int>(
			std::upper_bound(this->cumulative_area.begin(), this->cumulative_area.end(), r) -
			this->cumulative_area.begin());
		f = std::min(f, static_cast<int>(this->mesh->NumFaces()) - 1);

		Eigen::Vector3d v0, v1, v2;
		GetTriangleVertices(this->mesh->poly_data, f, v0, v1, v2);

		double r1 = unit_dist(rng);
		double r2 = unit_dist(rng);
		double sqrt_r1 = std::sqrt(r1);

		Eigen::Vector3d point = (1.0 - sqrt_r1) * v0 + sqrt_r1 * (1.0 - r2) * v1 + sqrt_r1 * r2 * v2;

		points[s] = point;
		normals[s] = this->mesh->face_normals[f];
		face_ids[s] = f;
	}
}

std::vector<ViewpointCandidate> GenerateViewCandidates(
	const MeshData& mesh,
	int n_surface_samples,
	const std::vector<double>& standoff_distances,
	const std::vector<double>& tilt_angles_deg,
	std::mt19937& rng,
	const std::vector<double>& tip_angles_deg)
{
	SurfaceSampler sampler(mesh);

	std::vector<Eigen::Vector3d> points, normals;
	std::vector<int> face_ids;
	sampler.Sample(n_surface_samples, rng, points, normals, face_ids);

	std::vector<ViewpointCandidate> candidates;
	candidates.reserve(
		points.size() * standoff_distances.size() * tilt_angles_deg.size() * tip_angles_deg.size());

	for (size_t k = 0; k < points.size(); ++k)
	{
		Eigen::Vector3d n = normals[k].normalized();

		// tangent1/tangent2 span the plane perpendicular to n -- tilt swings view_dir toward
		// tangent1, tip swings it toward tangent2, so together they reach any direction around n.
		Eigen::Vector3d tangent1 = n.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
		if (tangent1.norm() < 1e-6)
			tangent1 = n.cross(Eigen::Vector3d(0.0, 1.0, 0.0));
		tangent1.normalize();
		Eigen::Vector3d tangent2 = n.cross(tangent1); // already unit: n and tangent1 are orthonormal

		for (double d : standoff_distances)
		{
			for (double tilt_deg : tilt_angles_deg)
			{
				double tilt = tilt_deg * M_PI / 180.0;

				for (double tip_deg : tip_angles_deg)
				{
					double tip = tip_deg * M_PI / 180.0;

					// Spherical offset from -n: reduces to the original tilt-only formula when
					// tip=0, and stays unit-length without renormalizing (n, tangent1, tangent2
					// orthonormal).
					Eigen::Vector3d view_dir = std::cos(tilt) * std::cos(tip) * (-n) +
						std::sin(tilt) * std::cos(tip) * tangent1 + std::sin(tip) * tangent2;
					Eigen::Vector3d camera_pos = points[k] - d * view_dir;

					ViewpointCandidate c;
					c.camera_pos = camera_pos;
					c.view_dir = view_dir;
					c.target_point = points[k];
					c.seed_face = face_ids[k];
					c.distance = d;
					candidates.push_back(std::move(c));
				}
			}
		}
	}

	return candidates;
}

std::vector<const ViewpointCandidate*> GreedySelectViewpoints(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	double target_area_visibility)
{
	const size_t n_faces = mesh.NumFaces();
	std::vector<uint8_t> uncovered(n_faces, 1);
	double visible_area = 0.0;

	std::vector<const ViewpointCandidate*> selected;

	while (true)
	{
		const ViewpointCandidate* best = nullptr;
		double best_gain_area = 0.0;

		for (const auto& c : candidates)
		{
			double gain = 0.0;
			for (size_t f = 0; f < n_faces; ++f)
				if (uncovered[f] && c.visible_mask[f])
					gain += mesh.face_areas[f];

			if (gain > best_gain_area)
			{
				best_gain_area = gain;
				best = &c;
			}
		}

		if (best == nullptr)
			break;

		double current_visibility = visible_area / mesh.total_area;
		double new_area_ratio = best_gain_area / mesh.total_area;

		if (current_visibility >= target_area_visibility)
			break;

		selected.push_back(best);
		for (size_t f = 0; f < n_faces; ++f)
			if (best->visible_mask[f])
				uncovered[f] = 0;
		visible_area += best_gain_area;

		printf(
			"selected=%zu, new_area=%.2f%%, area_visibility=%.2f%%\n",
			selected.size(), new_area_ratio * 100.0, visible_area / mesh.total_area * 100.0);
	}

	return selected;
}
