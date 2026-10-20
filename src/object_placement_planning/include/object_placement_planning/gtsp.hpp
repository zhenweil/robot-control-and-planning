#pragma once
// Tour over viewpoints as a GTSP: each viewpoint has several IK poses, and the tour visits every reachable viewpoint
// once using one of its poses. An open path from a start pose (no return).

#include <map>
#include <utility>
#include <vector>

#include "object_placement_planning/joint_travel.hpp"

namespace gtsp
{

struct Tour
{
	double cost = 0.0;
	std::vector<int> order;	 // viewpoint ids in visit order
	std::vector<joint_travel::Joints> poses;  // chosen pose per visit, as given to the solver

};

// One visit: a viewpoint and which of its IK poses.
struct Stop
{
	int viewpoint = -1;
	int pose = -1;
};

class Solver
{
public:
	// viewpoint_ik_poses: IK poses per viewpoint; empty = unreachable, left out of the tour. start_pose: the arm's pose before the
	// first viewpoint. two_opt_rounds: rounds of (segment reversals, then best pose per position). Every joint counts
	// as travel until SetTravelJoints.
	Solver(
		std::vector<std::vector<joint_travel::Joints>> viewpoint_ik_poses, const joint_travel::Joints& start_pose,
		joint_travel::EdgeWeights edge, int two_opt_rounds = 5);

	// Indices of the joints that count as travel; empty = all.
	void SetTravelJoints(const std::vector<int>& indices);

	// Extra cost on the edge between two poses, either direction (e.g. its straight move is blocked). A pose is (viewpoint,
	// index in its IK poses); the start pose is (-1, -1).
	void AddEdgeCost(std::pair<int, int> from, std::pair<int, int> to, double cost);

	// Order and poses: nearest neighbour, then 2-opt. initial_order (e.g. the previous tour's): when it lists exactly
	// the reachable viewpoints, 2-opt also runs from it and the cheaper tour is kept. Deterministic.
	Tour Solve(const std::vector<int>& initial_order = {}) const;

	// Keeps the given viewpoint order and picks the IK pose per viewpoint with the least total travel. Unreachable
	// viewpoints are removed; reachable ones not in the order are added where they add the least joint motion.
	Tour SolveFixedOrder(const std::vector<int>& order) const;

private:
	// Cost of the move into `to`; from = nullptr means from the start pose.
	double EdgeCost(const Stop* from, const Stop& to) const;
	// Extra cost added with AddEdgeCost between two poses (either order), else 0.
	double GetExtraEdgeCost(std::pair<int, int> from, std::pair<int, int> to) const;
	double TourCost(const std::vector<Stop>& stops) const;
	std::vector<Stop> NearestNeighborTour() const;
	// Rounds of segment reversals, then each stop's best pose with its neighbours held fixed.
	std::vector<Stop> TwoOpt(std::vector<Stop> stops) const;
	// `order` with each reachable viewpoint it lacks inserted where its first pose adds the least joint motion.
	std::vector<int> WithMissingInserted(std::vector<int> order) const;

	int two_opt_rounds;
	joint_travel::EdgeWeights edge;
	joint_travel::Joints start_pose;
	std::vector<int> travel_joints;  // indices of the joints that count as travel
	std::vector<std::vector<joint_travel::Joints>> viewpoint_ik_poses;
	std::map<std::pair<std::pair<int, int>, std::pair<int, int>>, double> extra_edge_costs;  // keyed with the smaller end first
};

}  // namespace gtsp
