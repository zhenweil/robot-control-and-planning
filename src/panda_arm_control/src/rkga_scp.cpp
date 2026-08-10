#include "panda_arm_control/rkga_scp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>

namespace
{

// Sorts candidate indices by chromosome key, then greedily adds each if it still covers new
// area -- same as GreedySelectViewpoints, but order comes from the chromosome, not gain.
std::vector<size_t> DecodeChromosome(
	const std::vector<double>& keys, const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates,
	double target_area_visibility)
{
	std::vector<size_t> order(candidates.size());
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return keys[a] < keys[b]; });

	const size_t n_faces = mesh.NumFaces();
	std::vector<uint8_t> uncovered(n_faces, 1);
	double visible_area = 0.0;

	std::vector<size_t> selected;

	for (size_t idx : order)
	{
		if (visible_area / mesh.total_area >= target_area_visibility)
			break;

		double gain = 0.0;
		const auto& mask = candidates[idx].visible_mask;
		for (size_t f = 0; f < n_faces; ++f)
			if (uncovered[f] && mask[f])
				gain += mesh.face_areas[f];

		if (gain <= 0.0)
			continue; // contributes nothing new -- skip, keep walking the sorted order

		selected.push_back(idx);
		for (size_t f = 0; f < n_faces; ++f)
			if (mask[f])
				uncovered[f] = 0;
		visible_area += gain;
	}

	return selected;
}

// Wraps ComputeAreaVisibility for use during decoding, where selections are index sets.
double ComputeAreaVisibilityByIndex(
	const std::vector<size_t>& selected, const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates)
{
	std::vector<const ViewpointCandidate*> pointers;
	pointers.reserve(selected.size());
	for (size_t idx : selected)
		pointers.push_back(&candidates[idx]);

	return ComputeAreaVisibility(mesh, pointers);
}

// Real travel cost of `selected` in order, via travel_cost_matrix lookups. A -1.0 entry means
// infeasible -- reject the whole sequence with the worst possible fitness.
double EvaluateFitness(
	const std::vector<size_t>& selected, const TravelCostMatrix& travel_cost_matrix, double joint_distance_weight)
{
	if (selected.empty())
		return std::numeric_limits<double>::max();

	double total = 0.0;
	size_t prev_matrix_idx = 0; // start reference

	for (size_t idx : selected)
	{
		size_t cur_matrix_idx = idx + 1;
		double cartesian_distance = travel_cost_matrix.cartesian_distance[prev_matrix_idx][cur_matrix_idx];
		if (cartesian_distance < 0.0)
			return std::numeric_limits<double>::max(); // infeasible transition

		double joint_distance = travel_cost_matrix.joint_distance[prev_matrix_idx][cur_matrix_idx];
		total += cartesian_distance + joint_distance_weight * joint_distance;
		prev_matrix_idx = cur_matrix_idx;
	}

	return total;
}

} // namespace

std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh, const std::vector<ViewpointCandidate>& candidates,
	const TravelCostMatrix& travel_cost_matrix, const RkgaScpParams& params)
{
	if (candidates.empty())
		return {};

	std::mt19937 rng(params.random_seed);
	std::uniform_real_distribution<double> key_dist(0.0, 1.0);

	const size_t n = candidates.size();
	const size_t elite_count =
		std::max<size_t>(1, static_cast<size_t>(params.elite_fraction * params.population_size));
	const size_t mutant_count = static_cast<size_t>(params.mutant_fraction * params.population_size);

	std::vector<std::vector<double>> population(
		static_cast<size_t>(params.population_size), std::vector<double>(n));
	for (auto& individual : population)
		for (double& key : individual)
			key = key_dist(rng);

	std::vector<size_t> best_selected;
	double best_fitness = std::numeric_limits<double>::max();

	for (int gen = 0; gen < params.num_generations; ++gen)
	{
		std::vector<double> fitness(population.size());
		std::vector<std::vector<size_t>> decoded(population.size());
		for (size_t i = 0; i < population.size(); ++i)
		{
			decoded[i] = DecodeChromosome(population[i], mesh, candidates, params.target_area_visibility);
			fitness[i] = EvaluateFitness(decoded[i], travel_cost_matrix, params.joint_distance_weight);
		}

		std::vector<size_t> rank(population.size());
		std::iota(rank.begin(), rank.end(), 0);
		std::sort(rank.begin(), rank.end(), [&](size_t a, size_t b) { return fitness[a] < fitness[b]; });

		bool improved = fitness[rank[0]] < best_fitness;
		if (improved)
		{
			best_fitness = fitness[rank[0]];
			best_selected = decoded[rank[0]];
		}

		double this_gen_visibility = ComputeAreaVisibilityByIndex(decoded[rank[0]], mesh, candidates);
		double best_visibility = ComputeAreaVisibilityByIndex(best_selected, mesh, candidates);

		printf(
			"gen=%d/%d, best_this_gen=%.4f (%zu viewpoints, %.2f%% visibility), "
			"best_overall=%.4f (%zu viewpoints, %.2f%% visibility)%s\n",
			gen + 1, params.num_generations, fitness[rank[0]], decoded[rank[0]].size(), this_gen_visibility * 100.0,
			best_fitness, best_selected.size(), best_visibility * 100.0, improved ? " *" : "");

		std::vector<std::vector<double>> next_population;
		next_population.reserve(population.size());

		for (size_t e = 0; e < elite_count; ++e)
			next_population.push_back(population[rank[e]]);

		for (size_t m = 0; m < mutant_count; ++m)
		{
			std::vector<double> mutant(n);
			for (double& key : mutant)
				key = key_dist(rng);
			next_population.push_back(std::move(mutant));
		}

		std::uniform_int_distribution<size_t> elite_pick(0, elite_count - 1);
		std::uniform_int_distribution<size_t> any_pick(0, population.size() - 1);
		while (next_population.size() < population.size())
		{
			const std::vector<double>& elite_parent = population[rank[elite_pick(rng)]];
			const std::vector<double>& other_parent = population[any_pick(rng)];

			std::vector<double> child(n);
			for (size_t g = 0; g < n; ++g)
				child[g] = (key_dist(rng) < params.elite_bias) ? elite_parent[g] : other_parent[g];

			next_population.push_back(std::move(child));
		}

		population = std::move(next_population);
	}

	std::vector<const ViewpointCandidate*> result;
	result.reserve(best_selected.size());
	for (size_t idx : best_selected)
		result.push_back(&candidates[idx]);

	PrintTourCostBreakdown(candidates, result, travel_cost_matrix, params.joint_distance_weight);

	return result;
}
