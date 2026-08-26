#include "panda_arm_control/hierarchical_tour.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <std_msgs/msg/color_rgba.hpp>

#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_io.hpp"

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

// Standard HSV->RGB (h in degrees, s/v in [0,1]) -- used to give each cluster a distinct color.
Eigen::Vector3d HsvToRgb(double h, double s, double v)
{
	double c = v * s;
	double x = c * (1.0 - std::abs(std::fmod(h / 60.0, 2.0) - 1.0));
	double m = v - c;
	double r = 0.0, g = 0.0, b = 0.0;
	if (h < 60)
	{
		r = c;
		g = x;
	}
	else if (h < 120)
	{
		r = x;
		g = c;
	}
	else if (h < 180)
	{
		g = c;
		b = x;
	}
	else if (h < 240)
	{
		g = x;
		b = c;
	}
	else if (h < 300)
	{
		r = x;
		b = c;
	}
	else
	{
		r = c;
		b = x;
	}
	return Eigen::Vector3d(r + m, g + m, b + m);
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

// -1.0 means infeasible -- treat as infinitely expensive, matching real_cost_planning.cpp's own
// (unexported) helper of the same name.
double WeightedCost(const TravelCostMatrix& matrix, size_t a, size_t b, double joint_distance_weight)
{
	double cartesian = matrix.cartesian_distance[a][b];
	if (cartesian < 0.0)
		return std::numeric_limits<double>::max();
	return cartesian + joint_distance_weight * matrix.joint_distance[a][b];
}

// One (original candidate, specific IK solution) node per entry -- the "Generalized" in GTSP: the
// solver picks one node per GROUP (original candidate), not one node overall.
struct ExpandedScratchSet
{
	std::vector<ViewpointCandidate> copies;
	std::vector<const ViewpointCandidate*> original_ptrs;	// parallel to copies
	std::vector<size_t> group_of;							// parallel to copies: index into `originals`
	size_t num_groups = 0;
};

// Up to max_solutions_per_candidate copies per original (fewer if it has fewer stored solutions --
// always >=1, since joint_solutions[0] == joint_solution for every reachable candidate).
ExpandedScratchSet MakeExpandedScratchSet(
	const std::vector<const ViewpointCandidate*>& originals, int max_solutions_per_candidate)
{
	ExpandedScratchSet expanded;
	expanded.num_groups = originals.size();

	size_t total = 0;
	for (const ViewpointCandidate* original : originals)
		total += std::max<size_t>(
			1, std::min(original->joint_solutions.size(), static_cast<size_t>(max_solutions_per_candidate)));
	expanded.copies.reserve(total);
	expanded.original_ptrs.reserve(total);
	expanded.group_of.reserve(total);

	for (size_t oi = 0; oi < originals.size(); ++oi)
	{
		const ViewpointCandidate* original = originals[oi];
		size_t num_solutions = std::max<size_t>(
			1, std::min(original->joint_solutions.size(), static_cast<size_t>(max_solutions_per_candidate)));
		for (size_t k = 0; k < num_solutions; ++k)
		{
			ViewpointCandidate copy = *original;
			if (k < original->joint_solutions.size())
				copy.joint_solution = original->joint_solutions[k];
			expanded.copies.push_back(std::move(copy));
			expanded.original_ptrs.push_back(original);
			expanded.group_of.push_back(oi);
		}
	}

	return expanded;
}

// Visits exactly one node per GROUP (unlike NearestNeighborOrderMatrix, which visits every node
// given to it) -- at each step, picks the (group, node) pair with lowest weighted cost among all
// nodes belonging to any unvisited group.
std::vector<const ViewpointCandidate*> GeneralizedNearestNeighborOrderMatrix(
	const ExpandedScratchSet& expanded, const TravelCostMatrix& matrix, double joint_distance_weight)
{
	std::vector<bool> group_visited(expanded.num_groups, false);
	std::vector<const ViewpointCandidate*> ordered;
	ordered.reserve(expanded.num_groups);

	size_t current_matrix_idx = 0;	 // start reference (home)
	for (size_t step = 0; step < expanded.num_groups; ++step)
	{
		double best = std::numeric_limits<double>::max();
		size_t best_idx = expanded.copies.size();	// sentinel: none picked yet

		for (size_t i = 0; i < expanded.copies.size(); ++i)
		{
			if (group_visited[expanded.group_of[i]])
				continue;

			size_t idx = MatrixIndexOf(expanded.copies, &expanded.copies[i]);
			double cost = WeightedCost(matrix, current_matrix_idx, idx, joint_distance_weight);
			if (best_idx == expanded.copies.size() || cost < best)
			{
				best = cost;
				best_idx = i;
			}
		}

		group_visited[expanded.group_of[best_idx]] = true;
		ordered.push_back(&expanded.copies[best_idx]);
		current_matrix_idx = MatrixIndexOf(expanded.copies, &expanded.copies[best_idx]);
	}

	return ordered;
}

// Alternates classic segment-reversal 2-opt with a "solution swap" pass (per position, try every
// other node in that position's group, holding neighbors fixed, keep whichever lowers the two
// adjacent edge costs) until neither improves or max_rounds is hit.
std::vector<const ViewpointCandidate*> GeneralizedTwoOptImprove(
	const ExpandedScratchSet& expanded, std::vector<const ViewpointCandidate*> ordered,
	const TravelCostMatrix& matrix, double joint_distance_weight, int max_rounds = 5)
{
	if (ordered.size() < 2)
		return ordered;

	std::vector<std::vector<size_t>> nodes_by_group(expanded.num_groups);
	for (size_t i = 0; i < expanded.copies.size(); ++i)
		nodes_by_group[expanded.group_of[i]].push_back(i);

	auto matrix_idx = [&](size_t pos) { return MatrixIndexOf(expanded.copies, ordered[pos]); };

	for (int round = 0; round < max_rounds; ++round)
	{
		bool improved = false;

		if (ordered.size() >= 3)
		{
			bool reversal_improved = true;
			while (reversal_improved)
			{
				reversal_improved = false;
				for (size_t i = 0; i + 1 < ordered.size(); ++i)
				{
					size_t prev_idx = (i == 0) ? 0 : matrix_idx(i - 1);
					for (size_t j = i + 1; j < ordered.size(); ++j)
					{
						double old_cost = WeightedCost(matrix, prev_idx, matrix_idx(i), joint_distance_weight);
						double new_cost = WeightedCost(matrix, prev_idx, matrix_idx(j), joint_distance_weight);
						if (j + 1 < ordered.size())
						{
							old_cost +=
								WeightedCost(matrix, matrix_idx(j), matrix_idx(j + 1), joint_distance_weight);
							new_cost +=
								WeightedCost(matrix, matrix_idx(i), matrix_idx(j + 1), joint_distance_weight);
						}
						if (new_cost < old_cost - 1e-9)
						{
							std::reverse(
								ordered.begin() + static_cast<std::ptrdiff_t>(i),
								ordered.begin() + static_cast<std::ptrdiff_t>(j) + 1);
							reversal_improved = true;
							improved = true;
						}
					}
				}
			}
		}

		for (size_t pos = 0; pos < ordered.size(); ++pos)
		{
			size_t cur_expanded_idx = static_cast<size_t>(ordered[pos] - expanded.copies.data());
			size_t group = expanded.group_of[cur_expanded_idx];

			size_t prev_matrix_idx = (pos == 0) ? 0 : matrix_idx(pos - 1);
			bool has_next = pos + 1 < ordered.size();
			size_t next_matrix_idx = has_next ? matrix_idx(pos + 1) : 0;

			double best_cost = std::numeric_limits<double>::max();
			size_t best_expanded_idx = cur_expanded_idx;

			for (size_t candidate_idx : nodes_by_group[group])
			{
				size_t candidate_matrix_idx = MatrixIndexOf(expanded.copies, &expanded.copies[candidate_idx]);
				double cost = WeightedCost(matrix, prev_matrix_idx, candidate_matrix_idx, joint_distance_weight);
				if (has_next)
					cost += WeightedCost(matrix, candidate_matrix_idx, next_matrix_idx, joint_distance_weight);
				if (cost < best_cost)
				{
					best_cost = cost;
					best_expanded_idx = candidate_idx;
				}
			}

			if (best_expanded_idx != cur_expanded_idx)
			{
				ordered[pos] = &expanded.copies[best_expanded_idx];
				improved = true;
			}
		}

		if (!improved)
			break;
	}

	return ordered;
}

// Orders `originals` via one real cost matrix + NN + 2-opt, appends it to `final_tour`, and
// accumulates real per-leg costs. Returns the pair count, for logging savings vs. a flat matrix.
// max_solutions_per_candidate > 1 switches to the generalized (multi-solution) path: each candidate
// contributes multiple IK-solution nodes and the solver picks whichever minimizes reconfiguration,
// writing the chosen solution back onto the original candidate for every downstream consumer to see.
int OrderAndAppend(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<const ViewpointCandidate*>& originals, const std::vector<double>& start_reference_joints,
	const std::string& group_name, double planning_time, double joint_distance_weight,
	int max_solutions_per_candidate, std::vector<const ViewpointCandidate*>& final_tour, double& total_cartesian,
	double& total_joint)
{
	if (max_solutions_per_candidate <= 1)
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

	ExpandedScratchSet expanded = MakeExpandedScratchSet(originals, max_solutions_per_candidate);

	TravelCostMatrix matrix = ComputeEndEffectorTravelDistanceMatrix(
		node, robot_model, planning_scene_monitor, expanded.copies, start_reference_joints, group_name,
		planning_time);

	std::vector<const ViewpointCandidate*> ordered =
		GeneralizedNearestNeighborOrderMatrix(expanded, matrix, joint_distance_weight);
	ordered = GeneralizedTwoOptImprove(expanded, std::move(ordered), matrix, joint_distance_weight);

	size_t prev_idx = 0;
	std::vector<const ViewpointCandidate*> mapped;
	mapped.reserve(ordered.size());
	for (const ViewpointCandidate* ptr : ordered)
	{
		size_t cur_idx = MatrixIndexOf(expanded.copies, ptr);
		double cartesian = matrix.cartesian_distance[prev_idx][cur_idx];
		double joint = matrix.joint_distance[prev_idx][cur_idx];
		if (cartesian >= 0.0)
		{
			total_cartesian += cartesian;
			total_joint += joint;
		}
		prev_idx = cur_idx;

		size_t expanded_idx = static_cast<size_t>(ptr - expanded.copies.data());
		const ViewpointCandidate* original = expanded.original_ptrs[expanded_idx];
		// Write the chosen solution back onto the original -- PlanFinalTourTrajectories and every
		// other downstream consumer reads joint_solution straight off the candidate.
		const_cast<ViewpointCandidate*>(original)->joint_solution = ptr->joint_solution;
		mapped.push_back(original);
	}

	final_tour.insert(final_tour.end(), mapped.begin(), mapped.end());

	const int n = static_cast<int>(expanded.copies.size());
	return n * (n + 1) / 2;
}

} // namespace

std::vector<const ViewpointCandidate*> SolveHierarchicalTour(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	std::vector<const ViewpointCandidate*> selected, const std::vector<double>& start_reference_joints,
	const std::string& group_name, const HierarchicalTourParams& params, HierarchicalTourDebug* debug_out)
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
			params.cluster_planning_time, params.joint_distance_weight, params.max_solutions_per_candidate,
			final_tour, total_cartesian, total_joint);
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
			params.guide_path_planning_time, params.joint_distance_weight, params.max_solutions_per_candidate,
			ordered_exemplars, exemplar_cartesian, exemplar_joint);

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
				params.cluster_planning_time, params.joint_distance_weight, params.max_solutions_per_candidate,
				final_tour, total_cartesian, total_joint);

			current_start = &final_tour.back()->joint_solution;
		}

		if (debug_out != nullptr)
		{
			debug_out->exemplars_in_order = ordered_exemplars;
			debug_out->cluster_members.reserve(ordered_exemplars.size());
			for (const ViewpointCandidate* exemplar : ordered_exemplars)
				debug_out->cluster_members.push_back(cluster_members.at(exemplar));
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

visualization_msgs::msg::MarkerArray BuildExemplarClusterMarkerArray(
	const rclcpp::Time& stamp, const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world, const HierarchicalTourDebug& debug_info)
{
	visualization_msgs::msg::MarkerArray markers;
	const size_t num_clusters = debug_info.exemplars_in_order.size();
	if (num_clusters == 0)
		return markers;

	int id = 0;

	visualization_msgs::msg::Marker spokes;
	spokes.header.frame_id = "world";
	spokes.header.stamp = stamp;
	spokes.ns = "exemplar_cluster_spokes";
	spokes.id = id++;
	spokes.type = visualization_msgs::msg::Marker::LINE_LIST;
	spokes.action = visualization_msgs::msg::Marker::ADD;
	spokes.pose.orientation.w = 1.0;
	spokes.scale.x = 0.001;
	spokes.color.a = 1.0f;	// RViz gates per-vertex color visibility on this top-level alpha

	for (size_t c = 0; c < num_clusters; ++c)
	{
		double hue = 360.0 * static_cast<double>(c) / static_cast<double>(num_clusters);
		Eigen::Vector3d rgb = HsvToRgb(hue, 0.85, 0.95);
		std_msgs::msg::ColorRGBA color;
		color.r = static_cast<float>(rgb.x());
		color.g = static_cast<float>(rgb.y());
		color.b = static_cast<float>(rgb.z());
		color.a = 1.0f;

		const ViewpointCandidate* exemplar = debug_info.exemplars_in_order[c];
		Eigen::Vector3d exemplar_world = object_rotation_world * exemplar->camera_pos + object_translation_world;

		visualization_msgs::msg::Marker exemplar_marker;
		exemplar_marker.header.frame_id = "world";
		exemplar_marker.header.stamp = stamp;
		exemplar_marker.ns = "exemplar";
		exemplar_marker.id = id++;
		exemplar_marker.type = visualization_msgs::msg::Marker::CUBE;
		exemplar_marker.action = visualization_msgs::msg::Marker::ADD;
		exemplar_marker.pose.position = ToPoint(exemplar_world);
		exemplar_marker.pose.orientation.w = 1.0;
		exemplar_marker.scale.x = exemplar_marker.scale.y = exemplar_marker.scale.z = 0.012;
		exemplar_marker.color = color;
		markers.markers.push_back(exemplar_marker);

		for (const ViewpointCandidate* member : debug_info.cluster_members[c])
		{
			Eigen::Vector3d member_world = object_rotation_world * member->camera_pos + object_translation_world;

			visualization_msgs::msg::Marker member_marker;
			member_marker.header.frame_id = "world";
			member_marker.header.stamp = stamp;
			member_marker.ns = "cluster_member";
			member_marker.id = id++;
			member_marker.type = visualization_msgs::msg::Marker::SPHERE;
			member_marker.action = visualization_msgs::msg::Marker::ADD;
			member_marker.pose.position = ToPoint(member_world);
			member_marker.pose.orientation.w = 1.0;
			member_marker.scale.x = member_marker.scale.y = member_marker.scale.z = 0.005;
			member_marker.color = color;
			markers.markers.push_back(member_marker);

			spokes.points.push_back(ToPoint(exemplar_world));
			spokes.points.push_back(ToPoint(member_world));
			spokes.colors.push_back(color);
			spokes.colors.push_back(color);
		}
	}

	markers.markers.push_back(spokes);
	return markers;
}
