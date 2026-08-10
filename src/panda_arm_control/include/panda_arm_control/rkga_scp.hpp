#pragma once

#include <vector>

#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_types.hpp"

struct RkgaScpParams
{
	int population_size = 100;
	int num_generations = 100;
	double elite_fraction = 0.2;  // top fraction of population carried over unchanged each gen
	double mutant_fraction = 0.15;	 // fraction replaced with brand-new random chromosomes each gen
	double elite_bias = 0.7;  // probability an offspring gene is inherited from its elite parent
	double target_area_visibility = 0.95;
	unsigned int random_seed = 42;
	// Added as joint_distance_weight * joint_distance to cartesian_distance when scoring a
	// transition, so the GA also accounts for arm reconfiguration, not just Cartesian travel.
	double joint_distance_weight = 0.0;
};

// BRKGA: chromosome keys decode into a greedy selection+order, stopping at target_area_visibility.
// Fitness is real travel_cost_matrix cost; infeasible transitions get rejected outright.
std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	const TravelCostMatrix& travel_cost_matrix,
	const RkgaScpParams& params);
