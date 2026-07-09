#pragma once

#include <random>
#include <vector>

#include "panda_arm_control/viewpoint_types.hpp"

// Alternative to GenerateViewCandidates (mesh_utils.hpp): instead of a fixed standoff/tilt-angle
// grid, the camera is placed at each standoff distance from a randomly sampled surface point
// (along its normal), and the viewing direction is computed rather than parameterized -- the net
// attraction toward nearby mesh surface, weighted by face area and falling off with inverse-
// square distance (attraction-only potential field, no repulsion term).
std::vector<ViewpointCandidate> GeneratePotentialFieldViewCandidates(
	const MeshData& mesh,
	int n_surface_samples,
	const std::vector<double>& standoff_distances,
	std::mt19937& rng);
