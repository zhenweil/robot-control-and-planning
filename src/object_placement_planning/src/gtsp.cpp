#include "object_placement_planning/gtsp.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>

namespace gtsp
{

Solver::Solver(
	std::vector<std::vector<joint_travel::Joints>> viewpoint_ik_poses, const joint_travel::Joints& start_pose,
	joint_travel::EdgeWeights edge, int two_opt_rounds)
	: two_opt_rounds(std::max(1, two_opt_rounds)), edge(edge), start_pose(start_pose),
	  travel_joints(joint_travel::AllJoints(static_cast<int>(start_pose.size()))),
	  viewpoint_ik_poses(std::move(viewpoint_ik_poses))
{
}

void Solver::SetTravelJoints(const std::vector<int>& indices)
{
	this->travel_joints = indices.empty() ? joint_travel::AllJoints(static_cast<int>(this->start_pose.size())) : indices;
}

void Solver::AddEdgeCost(std::pair<int, int> from, std::pair<int, int> to, double cost)
{
	this->extra_edge_costs[{std::min(from, to), std::max(from, to)}] += cost;
}

double Solver::GetExtraEdgeCost(std::pair<int, int> from, std::pair<int, int> to) const
{
	if (this->extra_edge_costs.empty())
		return 0.0;
	const auto found = this->extra_edge_costs.find({std::min(from, to), std::max(from, to)});
	return found == this->extra_edge_costs.end() ? 0.0 : found->second;
}

Tour Solver::Solve(const std::vector<int>& initial_order) const
{
	Tour tour;
	std::vector<Stop> best = this->TwoOpt(this->NearestNeighborTour());
	if (best.empty())
		return tour;
	const bool initial_fits = initial_order.size() == best.size() &&
		std::all_of(initial_order.begin(), initial_order.end(), [&](int v) {
			return v >= 0 && v < static_cast<int>(this->viewpoint_ik_poses.size()) &&
				!this->viewpoint_ik_poses[static_cast<size_t>(v)].empty();
		});
	if (initial_fits)
	{
		std::vector<Stop> from_initial;
		for (int v : initial_order)
			from_initial.push_back({v, 0});
		from_initial = this->TwoOpt(std::move(from_initial));
		if (this->TourCost(from_initial) < this->TourCost(best))
			best = std::move(from_initial);
	}
	for (const Stop& stop : best)
	{
		tour.order.push_back(stop.viewpoint);
		tour.poses.push_back(this->viewpoint_ik_poses[static_cast<size_t>(stop.viewpoint)][static_cast<size_t>(stop.pose)]);
	}
	tour.cost = this->TourCost(best);
	return tour;
}

Tour Solver::SolveFixedOrder(const std::vector<int>& order_in) const
{
	std::vector<int> reachable;
	for (int v : order_in)
		if (!this->viewpoint_ik_poses[static_cast<size_t>(v)].empty())
			reachable.push_back(v);
	Tour tour;
	tour.order = this->WithMissingInserted(std::move(reachable));
	const std::vector<int>& order = tour.order;
	if (order.empty())
		return tour;

	// best[k][j]: cheapest order[0..k] ending in pose j of order[k]
	const std::vector<std::vector<joint_travel::Joints>>& poses = this->viewpoint_ik_poses;
	std::vector<std::vector<double>> best(order.size());
	std::vector<std::vector<int>> parent(order.size());
	for (size_t k = 0; k < order.size(); ++k)
	{
		const size_t v = static_cast<size_t>(order[k]);
		best[k].assign(poses[v].size(), std::numeric_limits<double>::max());
		parent[k].assign(poses[v].size(), -1);
		for (size_t j = 0; j < poses[v].size(); ++j)
		{
			if (k == 0)
			{
				best[k][j] = joint_travel::EdgeCost(this->start_pose, poses[v][j], this->edge, this->travel_joints) + this->GetExtraEdgeCost({-1, -1}, {static_cast<int>(v), static_cast<int>(j)});
				continue;
			}
			const size_t u = static_cast<size_t>(order[k - 1]);
			for (size_t i = 0; i < poses[u].size(); ++i)
			{
				const double c =
					best[k - 1][i] + joint_travel::EdgeCost(poses[u][i], poses[v][j], this->edge, this->travel_joints) + this->GetExtraEdgeCost({static_cast<int>(u), static_cast<int>(i)}, {static_cast<int>(v), static_cast<int>(j)});
				if (c < best[k][j])
				{
					best[k][j] = c;
					parent[k][j] = static_cast<int>(i);
				}
			}
		}
	}
	tour.poses.resize(order.size());
	const auto last = std::min_element(best.back().begin(), best.back().end());
	tour.cost = *last;
	int j = static_cast<int>(last - best.back().begin());
	for (size_t k = order.size(); k-- > 0;)
	{
		tour.poses[k] = poses[static_cast<size_t>(order[k])][static_cast<size_t>(j)];
		j = parent[k][static_cast<size_t>(j)];
	}
	return tour;
}

double Solver::EdgeCost(const Stop* from, const Stop& to) const
{
	const joint_travel::Joints& to_pose =
		this->viewpoint_ik_poses[static_cast<size_t>(to.viewpoint)][static_cast<size_t>(to.pose)];
	const joint_travel::Joints& from_pose = from
		? this->viewpoint_ik_poses[static_cast<size_t>(from->viewpoint)][static_cast<size_t>(from->pose)]
		: this->start_pose;
	const std::pair<int, int> from_key = from ? std::make_pair(from->viewpoint, from->pose) : std::make_pair(-1, -1);
	return joint_travel::EdgeCost(from_pose, to_pose, this->edge, this->travel_joints) + this->GetExtraEdgeCost(from_key, {to.viewpoint, to.pose});
}

double Solver::TourCost(const std::vector<Stop>& stops) const
{
	double total = 0.0;
	for (size_t k = 0; k < stops.size(); ++k)
		total += this->EdgeCost(k == 0 ? nullptr : &stops[k - 1], stops[k]);
	return total;
}

std::vector<Stop> Solver::NearestNeighborTour() const
{
	const size_t n = this->viewpoint_ik_poses.size();
	size_t num_reachable = 0;
	for (const auto& poses : this->viewpoint_ik_poses)
		num_reachable += poses.empty() ? 0 : 1;
	std::vector<bool> visited(n, false);
	std::vector<Stop> stops;
	while (stops.size() < num_reachable)
	{
		double best = std::numeric_limits<double>::max();
		Stop pick;
		bool picked = false;
		for (size_t v = 0; v < n; ++v)
			if (!visited[v])
				for (size_t j = 0; j < this->viewpoint_ik_poses[v].size(); ++j)
				{
					const Stop cand{static_cast<int>(v), static_cast<int>(j)};
					const double cost = this->EdgeCost(stops.empty() ? nullptr : &stops.back(), cand);
					if (!picked || cost < best)
					{
						best = cost;
						pick = cand;
						picked = true;
					}
				}
		visited[static_cast<size_t>(pick.viewpoint)] = true;
		stops.push_back(pick);
	}
	return stops;
}

std::vector<Stop> Solver::TwoOpt(std::vector<Stop> stops) const
{
	if (stops.size() < 2)
		return stops;
	auto edge_after = [&](size_t pos, const Stop& stop) {
		return this->EdgeCost(pos == 0 ? nullptr : &stops[pos - 1], stop);
	};
	for (int round = 0; round < this->two_opt_rounds; ++round)
	{
		bool improved = false;
		for (bool reversed = stops.size() >= 3; reversed;)
		{
			reversed = false;
			for (size_t i = 0; i + 1 < stops.size(); ++i)
				for (size_t j = i + 1; j < stops.size(); ++j)
				{
					double old_cost = edge_after(i, stops[i]);
					double new_cost = edge_after(i, stops[j]);
					if (j + 1 < stops.size())
					{
						old_cost += this->EdgeCost(&stops[j], stops[j + 1]);
						new_cost += this->EdgeCost(&stops[i], stops[j + 1]);
					}
					if (new_cost < old_cost - 1e-9)
					{
						std::reverse(
							stops.begin() + static_cast<std::ptrdiff_t>(i), stops.begin() + static_cast<std::ptrdiff_t>(j) + 1);
						reversed = improved = true;
					}
				}
		}
		for (size_t pos = 0; pos < stops.size(); ++pos)
		{
			const int v = stops[pos].viewpoint;
			double best_cost = std::numeric_limits<double>::max();
			int best_pose = stops[pos].pose;
			for (size_t j = 0; j < this->viewpoint_ik_poses[static_cast<size_t>(v)].size(); ++j)
			{
				const Stop cand{v, static_cast<int>(j)};
				double cost = edge_after(pos, cand);
				if (pos + 1 < stops.size())
					cost += this->EdgeCost(&cand, stops[pos + 1]);
				if (cost < best_cost)
				{
					best_cost = cost;
					best_pose = static_cast<int>(j);
				}
			}
			if (best_pose != stops[pos].pose)
			{
				stops[pos].pose = best_pose;
				improved = true;
			}
		}
		if (!improved)
			break;
	}
	return stops;
}

std::vector<int> Solver::WithMissingInserted(std::vector<int> order) const
{
	const std::vector<std::vector<joint_travel::Joints>>& poses = this->viewpoint_ik_poses;
	const std::vector<int>& joints = this->travel_joints;
	for (size_t v = 0; v < poses.size(); ++v)
	{
		if (poses[v].empty() || std::find(order.begin(), order.end(), static_cast<int>(v)) != order.end())
			continue;
		const joint_travel::Joints& qv = poses[v].front();
		size_t best_pos = order.size();
		double best_detour = std::numeric_limits<double>::max();
		for (size_t pos = 0; pos <= order.size(); ++pos)
		{
			const joint_travel::Joints& qa = pos == 0 ? this->start_pose : poses[static_cast<size_t>(order[pos - 1])].front();
			double detour = joint_travel::JointDistance(qa, qv, joints);
			if (pos < order.size())
			{
				const joint_travel::Joints& qb = poses[static_cast<size_t>(order[pos])].front();
				detour += joint_travel::JointDistance(qv, qb, joints) - joint_travel::JointDistance(qa, qb, joints);
			}
			if (detour < best_detour)
			{
				best_detour = detour;
				best_pos = pos;
			}
		}
		order.insert(order.begin() + static_cast<std::ptrdiff_t>(best_pos), static_cast<int>(v));
	}
	return order;
}

}  // namespace gtsp
