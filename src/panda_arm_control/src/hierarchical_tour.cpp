#include "panda_arm_control/hierarchical_tour.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "panda_arm_control/real_cost_planning.hpp"

namespace
{

// Cheap clustering-only proxy distance -- no real motion planning, mirrors the paper's Eq. 1
// (Cartesian + rotational distance) for picking well-separated exemplars and cluster membership.
double ExemplarProxyDistance(const ViewpointCandidate& a, const ViewpointCandidate& b, double rotation_weight)
{
	double cartesian = (a.camera_pos - b.camera_pos).norm();
	double cos_angle = a.view_dir.normalized().dot(b.view_dir.normalized());
	cos_angle = std::clamp(cos_angle, -1.0, 1.0);
	return cartesian + rotation_weight * std::acos(cos_angle);
}

struct ExemplarClustering
{
	std::vector<size_t> exemplar_indices;	// positions into `selected`
	std::vector<size_t> cluster_of;		// per selected[i], index into exemplar_indices
};

// Affinity propagation (Frey & Dueck 2007): picks its own exemplar count via `preference`,
// unlike a fixed-k method. Runs on the cheap proxy metric only, never real planned cost.
ExemplarClustering AffinityPropagationCluster(
	const std::vector<const ViewpointCandidate*>& selected, double rotation_weight, double preference_quantile,
	double damping, int max_iterations, int convergence_iterations)
{
	const size_t n = selected.size();
	ExemplarClustering result;
	result.cluster_of.assign(n, 0);

	if (n == 0)
		return result;
	if (n == 1)
	{
		result.exemplar_indices = {0};
		return result;
	}

	// Similarity = negative proxy distance (standard AP convention: higher = more similar).
	std::vector<std::vector<double>> s(n, std::vector<double>(n, 0.0));
	std::vector<double> off_diagonal;
	off_diagonal.reserve(n * (n - 1));
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = 0; j < n; ++j)
		{
			if (i == j)
				continue;
			s[i][j] = -ExemplarProxyDistance(*selected[i], *selected[j], rotation_weight);
			off_diagonal.push_back(s[i][j]);
		}
	}

	// Preference controls how many exemplars AP settles on -- a lower preference (relative to the
	// spread of similarities) biases toward fewer, larger clusters.
	std::sort(off_diagonal.begin(), off_diagonal.end());
	size_t pref_idx = static_cast<size_t>(
		std::clamp(preference_quantile, 0.0, 1.0) * static_cast<double>(off_diagonal.size() - 1));
	double preference = off_diagonal[pref_idx];
	for (size_t i = 0; i < n; ++i)
		s[i][i] = preference;

	std::vector<std::vector<double>> r(n, std::vector<double>(n, 0.0));
	std::vector<std::vector<double>> a(n, std::vector<double>(n, 0.0));

	std::vector<bool> is_exemplar(n, false);
	std::vector<bool> prev_is_exemplar(n, false);
	int stable_iterations = 0;

	for (int iter = 0; iter < max_iterations; ++iter)
	{
		// r(i,k) = s(i,k) - max_{k'!=k}[a(i,k')+s(i,k')]. Track per-row top-2 so the
		// max-excluding-k lookup is O(1), keeping this O(n^2) total instead of O(n^3).
		for (size_t i = 0; i < n; ++i)
		{
			double best1 = -std::numeric_limits<double>::max();
			double best2 = -std::numeric_limits<double>::max();
			size_t best1_idx = n;
			for (size_t k = 0; k < n; ++k)
			{
				double v = a[i][k] + s[i][k];
				if (v > best1)
				{
					best2 = best1;
					best1 = v;
					best1_idx = k;
				}
				else if (v > best2)
				{
					best2 = v;
				}
			}
			for (size_t k = 0; k < n; ++k)
			{
				double max_other = (k == best1_idx) ? best2 : best1;
				double update = s[i][k] - max_other;
				r[i][k] = damping * r[i][k] + (1.0 - damping) * update;
			}
		}

		// Update availabilities: a(k,k) = sum_{i'!=k} max(0,r(i',k)); a(i,k) = min(0, r(k,k) +
		// sum_{i'!=i,k} max(0,r(i',k))) for i!=k.
		for (size_t k = 0; k < n; ++k)
		{
			double sum_pos = 0.0;
			for (size_t i = 0; i < n; ++i)
				if (i != k)
					sum_pos += std::max(0.0, r[i][k]);

			for (size_t i = 0; i < n; ++i)
			{
				double update;
				if (i == k)
					update = sum_pos;
				else
					update = std::min(0.0, r[k][k] + sum_pos - std::max(0.0, r[i][k]));
				a[i][k] = damping * a[i][k] + (1.0 - damping) * update;
			}
		}

		for (size_t k = 0; k < n; ++k)
			is_exemplar[k] = (r[k][k] + a[k][k]) > 0.0;

		if (is_exemplar == prev_is_exemplar)
		{
			++stable_iterations;
			if (stable_iterations >= convergence_iterations)
				break;
		}
		else
		{
			stable_iterations = 0;
			prev_is_exemplar = is_exemplar;
		}
	}

	for (size_t k = 0; k < n; ++k)
		if (is_exemplar[k])
			result.exemplar_indices.push_back(k);

	if (result.exemplar_indices.empty())
	{
		// Degenerate case -- fall back to the medoid so there's always >=1 exemplar.
		// cluster_of is already zero-initialized, which correctly indexes it.
		size_t medoid = 0;
		double best_total = std::numeric_limits<double>::max();
		for (size_t i = 0; i < n; ++i)
		{
			double total = 0.0;
			for (size_t j = 0; j < n; ++j)
				if (i != j)
					total += -s[i][j];
			if (total < best_total)
			{
				best_total = total;
				medoid = i;
			}
		}
		result.exemplar_indices = {medoid};
		return result;
	}

	// Assign every point to the exemplar maximizing r+a among the discovered exemplar set.
	for (size_t i = 0; i < n; ++i)
	{
		double best = -std::numeric_limits<double>::max();
		size_t best_cluster = 0;
		for (size_t c = 0; c < result.exemplar_indices.size(); ++c)
		{
			size_t k = result.exemplar_indices[c];
			double score = r[i][k] + a[i][k];
			if (score > best)
			{
				best = score;
				best_cluster = c;
			}
		}
		result.cluster_of[i] = best_cluster;
	}

	return result;
}

// A candidate subset copied into its own storage, since NearestNeighborOrderMatrix/TwoOptImproveMatrix
// recover matrix indices via pointer arithmetic into whichever `candidates` vector they're given.
struct ScratchSet
{
	std::vector<ViewpointCandidate> copies;
	std::vector<const ViewpointCandidate*> original_ptrs;	 // original_ptrs[i] mirrors copies[i]
};

ScratchSet MakeScratchSet(const std::vector<const ViewpointCandidate*>& originals)
{
	ScratchSet s;
	s.copies.reserve(originals.size());
	s.original_ptrs.reserve(originals.size());
	for (const ViewpointCandidate* ptr : originals)
	{
		s.copies.push_back(*ptr);
		s.original_ptrs.push_back(ptr);
	}
	return s;
}

std::vector<const ViewpointCandidate*> MapScratchOrderToOriginal(
	const ScratchSet& scratch, const std::vector<const ViewpointCandidate*>& ordered_scratch)
{
	std::vector<const ViewpointCandidate*> result;
	result.reserve(ordered_scratch.size());
	for (const ViewpointCandidate* ptr : ordered_scratch)
	{
		size_t idx = static_cast<size_t>(ptr - scratch.copies.data());
		result.push_back(scratch.original_ptrs[idx]);
	}
	return result;
}

// candidates[idx] maps to matrix index idx+1 (0 is home) -- mirrors real_cost_planning.cpp's own
// (internally-linked) helper of the same name, needed again here since that one isn't exported.
size_t MatrixIndexOf(const std::vector<ViewpointCandidate>& candidates, const ViewpointCandidate* ptr)
{
	return static_cast<size_t>(ptr - candidates.data()) + 1;
}

// Orders `originals` via one real cost matrix + NN + 2-opt, appends it to `final_tour`, and
// accumulates real per-leg costs. Returns the pair count, for logging savings vs. a flat matrix.
int OrderAndAppend(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<const ViewpointCandidate*>& originals, const std::vector<double>& start_reference_joints,
	const std::string& group_name, double planning_time, double joint_distance_weight,
	std::vector<const ViewpointCandidate*>& final_tour, double& total_cartesian, double& total_joint)
{
	ScratchSet scratch = MakeScratchSet(originals);

	TravelCostMatrix matrix = ComputeEndEffectorTravelDistanceMatrix(
		node, robot_model, planning_scene_monitor, scratch.copies, start_reference_joints, group_name,
		planning_time);

	std::vector<const ViewpointCandidate*> scratch_ptrs;
	scratch_ptrs.reserve(scratch.copies.size());
	for (const ViewpointCandidate& c : scratch.copies)
		scratch_ptrs.push_back(&c);

	std::vector<const ViewpointCandidate*> ordered =
		NearestNeighborOrderMatrix(scratch.copies, std::move(scratch_ptrs), matrix, joint_distance_weight);
	ordered = TwoOptImproveMatrix(scratch.copies, std::move(ordered), matrix, joint_distance_weight);

	size_t prev_idx = 0;  // start reference
	for (const ViewpointCandidate* ptr : ordered)
	{
		size_t cur_idx = MatrixIndexOf(scratch.copies, ptr);
		double cartesian = matrix.cartesian_distance[prev_idx][cur_idx];
		double joint = matrix.joint_distance[prev_idx][cur_idx];
		if (cartesian >= 0.0)  // -1.0 sentinel = infeasible; excluded from the running total
		{
			total_cartesian += cartesian;
			total_joint += joint;
		}
		prev_idx = cur_idx;
	}

	std::vector<const ViewpointCandidate*> mapped = MapScratchOrderToOriginal(scratch, ordered);
	final_tour.insert(final_tour.end(), mapped.begin(), mapped.end());

	const int n = static_cast<int>(originals.size());
	return n * (n + 1) / 2;	 // pairs in an (n+1)-node matrix (home + n candidates)
}

} // namespace

std::vector<const ViewpointCandidate*> SolveHierarchicalTour(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	std::vector<const ViewpointCandidate*> selected, const std::vector<double>& start_reference_joints,
	const std::string& group_name, const HierarchicalTourParams& params)
{
	std::vector<const ViewpointCandidate*> final_tour;
	if (selected.empty())
		return final_tour;

	double total_cartesian = 0.0;
	double total_joint = 0.0;
	int real_pairs_used = 0;

	if (static_cast<int>(selected.size()) <= params.min_size_for_hierarchy)
	{
		RCLCPP_INFO(
			node->get_logger(),
			"SolveHierarchicalTour: %zu selected views <= min_size_for_hierarchy (%d), running flat NN+2-opt",
			selected.size(), params.min_size_for_hierarchy);
		real_pairs_used += OrderAndAppend(
			node, robot_model, planning_scene_monitor, selected, start_reference_joints, group_name,
			params.cluster_planning_time, params.joint_distance_weight, final_tour, total_cartesian, total_joint);
	}
	else
	{
		ExemplarClustering clustering = AffinityPropagationCluster(
			selected, params.exemplar_metric_rotation_weight, params.ap_preference_quantile, params.ap_damping,
			params.ap_max_iterations, params.ap_convergence_iterations);

		RCLCPP_INFO(
			node->get_logger(),
			"SolveHierarchicalTour: affinity propagation found %zu exemplars for %zu selected views",
			clustering.exemplar_indices.size(), selected.size());

		std::vector<const ViewpointCandidate*> exemplar_originals;
		exemplar_originals.reserve(clustering.exemplar_indices.size());
		for (size_t idx : clustering.exemplar_indices)
			exemplar_originals.push_back(selected[idx]);

		std::vector<const ViewpointCandidate*> ordered_exemplars;
		double exemplar_cartesian = 0.0;
		double exemplar_joint = 0.0;
		real_pairs_used += OrderAndAppend(
			node, robot_model, planning_scene_monitor, exemplar_originals, start_reference_joints, group_name,
			params.guide_path_planning_time, params.joint_distance_weight, ordered_exemplars, exemplar_cartesian,
			exemplar_joint);

		// Group non-exemplar members by their assigned exemplar's original pointer identity. Every
		// exemplar gets an entry (possibly empty) so the lookup below never misses.
		std::unordered_map<const ViewpointCandidate*, std::vector<const ViewpointCandidate*>> cluster_members;
		for (const ViewpointCandidate* exemplar : exemplar_originals)
			cluster_members[exemplar];
		for (size_t i = 0; i < selected.size(); ++i)
		{
			const ViewpointCandidate* exemplar = exemplar_originals[clustering.cluster_of[i]];
			if (selected[i] != exemplar)
				cluster_members[exemplar].push_back(selected[i]);
		}

		// Chain each cluster's start_reference_joints to the prior cluster's last candidate, so
		// cluster-boundary transitions are real planned edges too, not estimated.
		const std::vector<double>* current_start = &start_reference_joints;
		for (const ViewpointCandidate* exemplar : ordered_exemplars)
		{
			std::vector<const ViewpointCandidate*> cluster_originals = {exemplar};
			const std::vector<const ViewpointCandidate*>& members = cluster_members.at(exemplar);
			cluster_originals.insert(cluster_originals.end(), members.begin(), members.end());

			real_pairs_used += OrderAndAppend(
				node, robot_model, planning_scene_monitor, cluster_originals, *current_start, group_name,
				params.cluster_planning_time, params.joint_distance_weight, final_tour, total_cartesian,
				total_joint);

			current_start = &final_tour.back()->joint_solution;
		}
	}

	const int n = static_cast<int>(selected.size());
	const int flat_pairs = n * (n + 1) / 2;
	const double weighted_total = total_cartesian + params.joint_distance_weight * total_joint;
	RCLCPP_INFO(
		node->get_logger(),
		"SolveHierarchicalTour: used %d real planning pairs (flat matrix over all selected would need "
		"%d) -- cartesian=%.4f m, joint=%.4f rad, weighted_total=%.4f",
		real_pairs_used, flat_pairs, total_cartesian, total_joint, weighted_total);

	return final_tour;
}
