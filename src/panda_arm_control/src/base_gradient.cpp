#include "panda_arm_control/base_gradient.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <thread>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <json/json.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <random_numbers/random_numbers.h>

namespace
{

// ---------------------------------------------------------------------------------------------
// Small helpers -- mirror base_placement.cpp's file-local versions of the same name/signature
// (this codebase already duplicates such helpers rather than exporting them; see the
// "Mirrors real_cost_planning.cpp" note in base_placement.cpp).
// ---------------------------------------------------------------------------------------------

// The rigid adjustment applied to the object + its viewpoints, in the robot base frame: translate
// (x, y, z) and tip/tilt (roll about base x, pitch about base y). See BaseGradientBounds for why
// yaw is excluded. Order: object_now = T(x,y,z) * Ry(pitch) * Rx(roll) * object_nominal.
struct BaseOffset
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
};

Eigen::Isometry3d MakeObjectOffset(const BaseOffset& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
	t.linear() = (Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()))
					 .toRotationMatrix();
	return t;
}

Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = translation;
	t.linear() = rotation;
	return t;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

bool IsStateCollisionFree(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState* state,
	const moveit::core::JointModelGroup* group, const double* joint_positions)
{
	state->setJointGroupPositions(group, joint_positions);
	state->update();
	planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
	return locked_scene->isStateValid(*state, group->getName());
}

// MOVE only updates the registered object's pose -- cheap, no mesh reload.
void SetObjectPose(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;

	planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor);
	locked_scene->processCollisionObjectMsg(obj);
}

BaseOffset ProjectToBounds(BaseOffset p, const BaseGradientBounds& b)
{
	p.x = std::clamp(p.x, b.x_min, b.x_max);
	p.y = std::clamp(p.y, b.y_min, b.y_max);
	p.z = std::clamp(p.z, b.z_min, b.z_max);
	p.roll = std::clamp(p.roll, b.roll_min, b.roll_max);
	p.pitch = std::clamp(p.pitch, b.pitch_min, b.pitch_max);
	return p;
}

// The offset gradient / step vector: [d/dx, d/dy, d/dz, d/droll, d/dpitch].
using OffsetVec = Eigen::Matrix<double, 5, 1>;

OffsetVec ToOffsetVec(const BaseOffset& a, const BaseOffset& b)  // a - b, component-wise
{
	OffsetVec v;
	v << a.x - b.x, a.y - b.y, a.z - b.z, a.roll - b.roll, a.pitch - b.pitch;
	return v;
}

// Gaussian kick around `center`: sigma_m meters on x/y/z, sigma_m/rot_scale rad on roll/pitch.
BaseOffset PerturbOffset(const BaseOffset& center, std::mt19937& rng, double sigma_m, double rot_scale)
{
	std::normal_distribution<double> nm(0.0, sigma_m);
	std::normal_distribution<double> nr(0.0, sigma_m / std::max(1e-6, rot_scale));
	return {center.x + nm(rng), center.y + nm(rng), center.z + nm(rng), center.roll + nr(rng),
			center.pitch + nr(rng)};
}

// Mirrors real_cost_planning.cpp's AreJointSolutionsSimilar (combined L2, radians).
bool AreJointSolutionsSimilar(const std::vector<double>& a, const std::vector<double>& b, double threshold_rad)
{
	double dist_sq = 0.0;
	for (size_t i = 0; i < a.size() && i < b.size(); ++i)
	{
		double d = a[i] - b[i];
		dist_sq += d * d;
	}
	return std::sqrt(dist_sq) < threshold_rad;
}

double JointL2Distance(const std::vector<double>& a, const std::vector<double>& b)
{
	double d = 0.0;
	for (size_t k = 0; k < a.size() && k < b.size(); ++k)
	{
		double e = a[k] - b[k];
		d += e * e;
	}
	return std::sqrt(d);
}

double MaxJointDeviation(const std::vector<double>& a, const std::vector<double>& b)
{
	double m = 0.0;
	for (size_t k = 0; k < a.size() && k < b.size(); ++k)
		m = std::max(m, std::abs(a[k] - b[k]));
	return m;
}

// One tour edge's weighted cost -- identical formula for the inner GTSP and the descent objective.
double WeightedEdgeCost(
	const std::vector<double>& qa, const std::vector<double>& qb, const Eigen::Vector3d& pa,
	const Eigen::Vector3d& pb, const BaseGradientParams& params)
{
	return params.cartesian_distance_weight * (pa - pb).norm() +
		params.joint_distance_weight * JointL2Distance(qa, qb) +
		params.max_joint_deviation_weight * MaxJointDeviation(qa, qb);
}

// ---------------------------------------------------------------------------------------------
// IK-branch collection
// ---------------------------------------------------------------------------------------------

// Up to `max_solutions` distinct collision-free IK solutions for every tour pose, at base `base`.
// Object is moved into `base`'s frame once up front. Empty inner vector for a pose that has no
// feasible solution at this base.
//
// seed_per_viewpoint[i] is the warm start for pose i's first IK attempt -- passing the previous
// base's solution for that pose keeps q_i(b) on a continuous IK branch as the base moves, which
// is what makes the objective (and hence the gradient) meaningful across iterations.
std::vector<std::vector<std::vector<double>>> CollectIkSolutions(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const BaseOffset& base, const BaseGradientParams& params,
	int max_solutions, random_numbers::RandomNumberGenerator* rng = nullptr)
{
	Eigen::Isometry3d xform = MakeObjectOffset(base);
	SetObjectPose(planning_scene_monitor, xform * object_pose_original);

	auto validity_callback = [&planning_scene_monitor](
								 moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								 const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	std::vector<std::vector<std::vector<double>>> branches(tour_tcp_poses_original.size());
	const int max_sol = std::max(1, max_solutions);

	for (size_t i = 0; i < tour_tcp_poses_original.size(); ++i)
	{
		geometry_msgs::msg::Pose target_local = ToPoseMsg(xform * tour_tcp_poses_original[i]);
		std::vector<std::vector<double>>& sols = branches[i];

		state.setJointGroupPositions(jmg, seed_per_viewpoint[i]);
		if (state.setFromIK(jmg, target_local, "tool0", params.ik_timeout, validity_callback))
		{
			std::vector<double> s;
			state.copyJointGroupPositions(jmg, s);
			sols.push_back(std::move(s));
		}

		for (int attempt = 0;
			 attempt < max_sol * 3 + params.ik_retries_per_point && static_cast<int>(sols.size()) < max_sol; ++attempt)
		{
			// A pose still with zero solutions after this many random restarts (on top of the
			// warm-started first attempt) is almost certainly out of reach at this offset -- stop
			// before burning the rest of the attempt budget on it, since every miss costs a full
			// ik_timeout. This is the dominant cost at reachability-breaking offsets; the later
			// attempts only ever helped reachable-but-hard poses collect extra branches.
			if (sols.empty() && attempt >= std::max(4, max_sol))
				break;
			if (rng)
				state.setToRandomPositions(jmg, *rng);
			else
				state.setToRandomPositions(jmg);
			if (!state.setFromIK(jmg, target_local, "tool0", params.ik_timeout, validity_callback))
				continue;
			std::vector<double> s;
			state.copyJointGroupPositions(jmg, s);
			bool dup = false;
			for (const auto& e : sols)
				if (AreJointSolutionsSimilar(e, s, 0.1))  // ~5.7 deg, matches real_cost_planning.cpp
				{
					dup = true;
					break;
				}
			if (!dup)
				sols.push_back(std::move(s));
		}
	}
	return branches;
}

// ---------------------------------------------------------------------------------------------
// Inner redundant-IK GTSP (compact reimplementation of the generalized NN + 2-opt / solution-swap
// scheme in hierarchical_tour.cpp, keyed on a plain endpoint cost so it matches the descent
// objective exactly). One node per (tour pose, IK branch); visit exactly one node per pose.
// ---------------------------------------------------------------------------------------------

struct GtspNode
{
	int group = -1;	  // index into the reachable-pose list
	int branch = -1;  // index into that pose's branch list
};

struct GtspContext
{
	std::vector<int> group_pose_index;						 // group -> tour pose index
	std::vector<std::vector<const std::vector<double>*>> joints_by_group;  // group -> branch -> joints
	std::vector<Eigen::Vector3d> tcp_local_by_group;			 // group -> tcp position in base frame
	const std::vector<double>* home_joints = nullptr;
	Eigen::Vector3d home_tcp_local = Eigen::Vector3d::Zero();
	const BaseGradientParams* params = nullptr;

	const std::vector<double>& NodeJoints(const GtspNode& n) const { return *joints_by_group[n.group][n.branch]; }
	const Eigen::Vector3d& NodePos(const GtspNode& n) const { return tcp_local_by_group[n.group]; }

	double EdgeFromHome(const GtspNode& b) const
	{
		return WeightedEdgeCost(*home_joints, NodeJoints(b), home_tcp_local, NodePos(b), *params);
	}
	double Edge(const GtspNode& a, const GtspNode& b) const
	{
		return WeightedEdgeCost(NodeJoints(a), NodeJoints(b), NodePos(a), NodePos(b), *params);
	}
};

std::vector<GtspNode> GeneralizedNearestNeighbor(const GtspContext& ctx)
{
	const size_t num_groups = ctx.joints_by_group.size();
	std::vector<bool> visited(num_groups, false);
	std::vector<GtspNode> order;
	order.reserve(num_groups);

	bool have_current = false;
	GtspNode current;
	for (size_t step = 0; step < num_groups; ++step)
	{
		double best = std::numeric_limits<double>::max();
		GtspNode best_node;
		bool picked = false;
		for (size_t g = 0; g < num_groups; ++g)
		{
			if (visited[g])
				continue;
			for (size_t b = 0; b < ctx.joints_by_group[g].size(); ++b)
			{
				GtspNode cand{static_cast<int>(g), static_cast<int>(b)};
				double cost = have_current ? ctx.Edge(current, cand) : ctx.EdgeFromHome(cand);
				if (!picked || cost < best)
				{
					best = cost;
					best_node = cand;
					picked = true;
				}
			}
		}
		visited[best_node.group] = true;
		order.push_back(best_node);
		current = best_node;
		have_current = true;
	}
	return order;
}

std::vector<GtspNode> GeneralizedTwoOpt(const GtspContext& ctx, std::vector<GtspNode> order, int max_rounds)
{
	if (order.size() < 2)
		return order;

	auto edge_at = [&](int prev_pos, const GtspNode& node) {
		return prev_pos < 0 ? ctx.EdgeFromHome(node) : ctx.Edge(order[prev_pos], node);
	};

	for (int round = 0; round < max_rounds; ++round)
	{
		bool improved = false;

		// Segment-reversal 2-opt.
		if (order.size() >= 3)
		{
			bool reversal_improved = true;
			while (reversal_improved)
			{
				reversal_improved = false;
				for (size_t i = 0; i + 1 < order.size(); ++i)
				{
					for (size_t j = i + 1; j < order.size(); ++j)
					{
						double old_cost = edge_at(static_cast<int>(i) - 1, order[i]);
						double new_cost = edge_at(static_cast<int>(i) - 1, order[j]);
						if (j + 1 < order.size())
						{
							old_cost += ctx.Edge(order[j], order[j + 1]);
							new_cost += ctx.Edge(order[i], order[j + 1]);
						}
						if (new_cost < old_cost - 1e-9)
						{
							std::reverse(
								order.begin() + static_cast<std::ptrdiff_t>(i),
								order.begin() + static_cast<std::ptrdiff_t>(j) + 1);
							reversal_improved = true;
							improved = true;
						}
					}
				}
			}
		}

		// Solution-swap: hold neighbors fixed, try every branch of this position's group.
		for (size_t pos = 0; pos < order.size(); ++pos)
		{
			int group = order[pos].group;
			bool has_next = pos + 1 < order.size();
			double best_cost = std::numeric_limits<double>::max();
			int best_branch = order[pos].branch;
			for (size_t b = 0; b < ctx.joints_by_group[group].size(); ++b)
			{
				GtspNode cand{group, static_cast<int>(b)};
				double cost = edge_at(static_cast<int>(pos) - 1, cand);
				if (has_next)
					cost += ctx.Edge(cand, order[pos + 1]);
				if (cost < best_cost)
				{
					best_cost = cost;
					best_branch = static_cast<int>(b);
				}
			}
			if (best_branch != order[pos].branch)
			{
				order[pos].branch = best_branch;
				improved = true;
			}
		}

		if (!improved)
			break;
	}
	return order;
}

double TourCost(const GtspContext& ctx, const std::vector<GtspNode>& order)
{
	if (order.empty())
		return 0.0;
	double total = ctx.EdgeFromHome(order[0]);
	for (size_t i = 1; i < order.size(); ++i)
		total += ctx.Edge(order[i - 1], order[i]);
	return total;
}

struct GtspSolution
{
	std::vector<int> tour_pose_indices;			   // tour pose index per visit position
	std::vector<std::vector<double>> chosen_joints;  // parallel: the picked IK branch
};

// warm_order (optional): viewpoint indices from a previous solve. When every listed viewpoint is
// still reachable, 2-opt is seeded from that order as well as from a fresh nearest-neighbour pass,
// and the cheaper result is kept -- so a base step can never make the tour worse just because the
// heuristic re-ordered it, which is what caused the cost to bounce between iterations.
GtspSolution RunGtsp(
	const std::vector<std::vector<std::vector<double>>>& branches, const BaseOffset& base,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params,
	const std::vector<int>* warm_order = nullptr)
{
	Eigen::Isometry3d xform = MakeObjectOffset(base);

	GtspContext ctx;
	ctx.home_joints = &home_joints;
	ctx.home_tcp_local = home_tcp_local;
	ctx.params = &params;
	std::vector<int> group_of_viewpoint(branches.size(), -1);
	for (size_t i = 0; i < branches.size(); ++i)
	{
		if (branches[i].empty())
			continue;
		group_of_viewpoint[i] = static_cast<int>(ctx.group_pose_index.size());
		ctx.group_pose_index.push_back(static_cast<int>(i));
		std::vector<const std::vector<double>*> js;
		for (const auto& b : branches[i])
			js.push_back(&b);
		ctx.joints_by_group.push_back(std::move(js));
		ctx.tcp_local_by_group.push_back((xform * tour_tcp_poses_original[i]).translation());
	}

	GtspSolution sol;
	if (ctx.joints_by_group.empty())
		return sol;

	const int rounds = std::max(1, params.gtsp_two_opt_rounds);

	std::vector<GtspNode> best = GeneralizedTwoOpt(ctx, GeneralizedNearestNeighbor(ctx), rounds);
	double best_cost = TourCost(ctx, best);

	if (warm_order && warm_order->size() == ctx.group_pose_index.size())
	{
		std::vector<GtspNode> seeded;
		seeded.reserve(warm_order->size());
		bool usable = true;
		for (int vp : *warm_order)
		{
			if (vp < 0 || vp >= static_cast<int>(group_of_viewpoint.size()) || group_of_viewpoint[vp] < 0)
			{
				usable = false;
				break;
			}
			seeded.push_back({group_of_viewpoint[vp], 0});
		}
		if (usable)
		{
			seeded = GeneralizedTwoOpt(ctx, std::move(seeded), rounds);
			double seeded_cost = TourCost(ctx, seeded);
			if (seeded_cost < best_cost)
			{
				best = std::move(seeded);
				best_cost = seeded_cost;
			}
		}
	}

	for (const GtspNode& n : best)
	{
		sol.tour_pose_indices.push_back(ctx.group_pose_index[n.group]);
		sol.chosen_joints.push_back(ctx.NodeJoints(n));
	}
	return sol;
}

// ---------------------------------------------------------------------------------------------
// Objective evaluation
// ---------------------------------------------------------------------------------------------

double TourWeightedCost(
	const BaseOffset& base, const std::vector<int>& tour, const std::vector<std::vector<double>>& joints,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params)
{
	Eigen::Isometry3d xform = MakeObjectOffset(base);
	double total = 0.0;
	const std::vector<double>* prev_j = &home_joints;
	Eigen::Vector3d prev_p = home_tcp_local;
	for (size_t k = 0; k < tour.size(); ++k)
	{
		Eigen::Vector3d p = (xform * tour_tcp_poses_original[tour[k]]).translation();
		total += WeightedEdgeCost(*prev_j, joints[k], prev_p, p, params);
		prev_j = &joints[k];
		prev_p = p;
	}
	return total;
}

// The full inner problem evaluated at one base pose: collect IK branches, run the GTSP (warm-
// started from `warm_order` if given), and score the resulting tour. This IS Phi(base) -- the
// quantity the outer gradient descent minimizes and the line search must test against.
struct InnerSolution
{
	std::vector<int> tour;					 // viewpoint index per visit position
	std::vector<std::vector<double>> joints;  // parallel: chosen IK branch
	double weighted_cost = 0.0;
	int num_reachable = 0;
	int num_total = 0;
	bool all_reachable = false;
};

// max_solutions: IK branches to collect per pose. Pass 1 for a cheap probe (single warm-started
// IK per pose, GTSP degenerates to plain TSP); pass params.max_solutions_per_candidate to commit.
InnerSolution InnerSolve(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseOffset& base, const BaseGradientParams& params,
	const std::vector<int>* warm_order, int max_solutions)
{
	std::vector<std::vector<std::vector<double>>> branches = CollectIkSolutions(
		state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seed_per_viewpoint, base,
		params, max_solutions);
	GtspSolution gtsp = RunGtsp(
		branches, base, tour_tcp_poses_original, home_joints, home_tcp_local, params, warm_order);

	InnerSolution out;
	out.tour = std::move(gtsp.tour_pose_indices);
	out.joints = std::move(gtsp.chosen_joints);
	out.num_total = static_cast<int>(tour_tcp_poses_original.size());
	out.num_reachable = static_cast<int>(out.tour.size());
	out.all_reachable = (out.num_reachable == out.num_total);
	out.weighted_cost = TourWeightedCost(
							base, out.tour, out.joints, tour_tcp_poses_original, home_joints, home_tcp_local, params) +
		params.unreachable_penalty * (out.num_total - out.num_reachable);
	return out;
}

// InnerSolve run `params.solve_restarts` times (>=1), keeping the lowest-weighted_cost result.
// Consecutive runs in one process advance the shared RNG stream, so they explore different IK
// branch sets -- min-of-N shrinks the seed-driven cost variance at a fixed offset. Use wherever a
// committed cost matters; keep plain InnerSolve for cheap line-search probes.
InnerSolution BestOfNInnerSolve(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseOffset& base, const BaseGradientParams& params,
	const std::vector<int>* warm_order, int max_solutions)
{
	const int n = std::max(1, params.solve_restarts);
	InnerSolution best = InnerSolve(
		state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seed_per_viewpoint,
		home_joints, home_tcp_local, base, params, warm_order, max_solutions);
	for (int i = 1; i < n; ++i)
	{
		InnerSolution cand = InnerSolve(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seed_per_viewpoint,
			home_joints, home_tcp_local, base, params, warm_order, max_solutions);
		if (cand.weighted_cost < best.weighted_cost)
			best = std::move(cand);
	}
	return best;
}

// Score a FIXED visiting order at `base`: walk `order` (viewpoint indices) and pick the cheapest
// IK branch at each stop by exact DP (Viterbi over the branch layers) -- the "which arm config"
// half of the inner problem with the route frozen, no reordering. Viewpoints with no branch at
// this base are skipped and charged params.unreachable_penalty, exactly as RunGtsp scores a
// partial tour. `order` may be any permutation; only its relative order of the reachable stops
// matters.
InnerSolution SolveFixedOrder(
	const std::vector<std::vector<std::vector<double>>>& branches, const BaseOffset& base,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params, const std::vector<int>& order)
{
	const Eigen::Isometry3d xform = MakeObjectOffset(base);
	const int num_total = static_cast<int>(tour_tcp_poses_original.size());

	std::vector<int> vp;					  // viewpoint index per reachable layer, in visit order
	std::vector<Eigen::Vector3d> pos;		  // its tool0 position in the base frame
	for (int i : order)
		if (i >= 0 && i < static_cast<int>(branches.size()) && !branches[i].empty())
		{
			vp.push_back(i);
			pos.push_back((xform * tour_tcp_poses_original[i]).translation());
		}

	InnerSolution out;
	out.num_total = num_total;
	if (vp.empty())
	{
		out.weighted_cost = params.unreachable_penalty * num_total;
		return out;
	}

	const size_t L = vp.size();
	std::vector<std::vector<double>> dp(L);	 // dp[l][k] = min cost to reach branch k of layer l
	std::vector<std::vector<int>> back(L);	 // predecessor branch index

	dp[0].resize(branches[vp[0]].size());
	back[0].assign(branches[vp[0]].size(), -1);
	for (size_t k = 0; k < branches[vp[0]].size(); ++k)
		dp[0][k] = WeightedEdgeCost(home_joints, branches[vp[0]][k], home_tcp_local, pos[0], params);

	for (size_t l = 1; l < L; ++l)
	{
		const auto& cur = branches[vp[l]];
		const auto& prev = branches[vp[l - 1]];
		dp[l].assign(cur.size(), std::numeric_limits<double>::max());
		back[l].assign(cur.size(), -1);
		for (size_t k = 0; k < cur.size(); ++k)
			for (size_t j = 0; j < prev.size(); ++j)
			{
				const double c = dp[l - 1][j] + WeightedEdgeCost(prev[j], cur[k], pos[l - 1], pos[l], params);
				if (c < dp[l][k])
				{
					dp[l][k] = c;
					back[l][k] = static_cast<int>(j);
				}
			}
	}

	size_t best_k = 0;
	for (size_t k = 1; k < dp[L - 1].size(); ++k)
		if (dp[L - 1][k] < dp[L - 1][best_k])
			best_k = k;
	const double tour_cost = dp[L - 1][best_k];

	std::vector<int> chosen(L);
	for (size_t l = L; l-- > 0;)
	{
		chosen[l] = static_cast<int>(best_k);
		if (l > 0)
			best_k = static_cast<size_t>(back[l][best_k]);
	}

	out.tour.resize(L);
	out.joints.resize(L);
	for (size_t l = 0; l < L; ++l)
	{
		out.tour[l] = vp[l];
		out.joints[l] = branches[vp[l]][chosen[l]];
	}
	out.num_reachable = static_cast<int>(L);
	out.all_reachable = (out.num_reachable == num_total);
	out.weighted_cost = tour_cost + params.unreachable_penalty * (num_total - out.num_reachable);
	return out;
}

// seed_per_viewpoint for the next solve: each viewpoint warm-started from its own current joints,
// falling back to `fallback` for any viewpoint not in the current tour.
std::vector<std::vector<double>> SeedsFromSolution(
	const InnerSolution& sol, const std::vector<double>& fallback, size_t num_viewpoints)
{
	std::vector<std::vector<double>> seeds(num_viewpoints, fallback);
	for (size_t k = 0; k < sol.tour.size(); ++k)
		if (!sol.joints[k].empty())
			seeds[sol.tour[k]] = sol.joints[k];
	return seeds;
}

struct TrackResult
{
	bool all_reachable = false;
	int num_reachable = 0;
	std::vector<std::vector<double>> joints;  // per visit position, empty where unreachable
	double weighted_cost = 0.0;				 // over edges whose endpoints are both reachable
	double joint_path_length = 0.0;			 // sum ||dq||_2 over those edges
};

// Re-solve IK for every pose in `tour` (fixed order) at `base`, warm-started from `seeds[k]`.
TrackResult TrackTour(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseOffset& base, const BaseGradientParams& params)
{
	Eigen::Isometry3d xform = MakeObjectOffset(base);
	SetObjectPose(planning_scene_monitor, xform * object_pose_original);

	auto validity_callback = [&planning_scene_monitor](
								 moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								 const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	TrackResult r;
	r.joints.resize(tour.size());

	const std::vector<double>* prev_j = &home_joints;
	Eigen::Vector3d prev_p = home_tcp_local;
	bool prev_reachable = true;

	for (size_t k = 0; k < tour.size(); ++k)
	{
		geometry_msgs::msg::Pose target_local = ToPoseMsg(xform * tour_tcp_poses_original[tour[k]]);
		Eigen::Vector3d p = (xform * tour_tcp_poses_original[tour[k]]).translation();

		state.setJointGroupPositions(jmg, seeds[k]);
		bool ok = state.setFromIK(jmg, target_local, "tool0", params.ik_timeout, validity_callback);
		if (ok)
		{
			state.copyJointGroupPositions(jmg, r.joints[k]);
			r.num_reachable++;
			if (prev_reachable)
			{
				r.weighted_cost += WeightedEdgeCost(*prev_j, r.joints[k], prev_p, p, params);
				r.joint_path_length += JointL2Distance(*prev_j, r.joints[k]);
			}
			prev_j = &r.joints[k];
			prev_p = p;
			prev_reachable = true;
		}
		else
		{
			prev_reachable = false;
		}
	}
	r.all_reachable = (r.num_reachable == static_cast<int>(tour.size()));
	return r;
}

// ---------------------------------------------------------------------------------------------
// Analytic gradient of the weighted tour cost w.r.t. the object offset (x, y, z, roll, pitch).
//
// The object + its viewpoints are moved by M = T(c) Ry(pitch) Rx(roll) in the base frame, so a
// nominal target T_i sits at p_i^b = M T_i. The tracking config q_i(b) satisfies FK(q_i) = M T_i;
// with minimum-norm redundancy resolution
//   dq_i/db = J_i^#  S_i          (J_i^# = damped pinv of the base-frame tool0 Jacobian)
// where S_i (6x5, [v; w] rows to match RobotState::getJacobian; columns x,y,z,roll,pitch) is the
// twist of the moved target per offset component, in the base frame:
//   x/y/z : [ e_axis ; 0 ]
//   roll  : [ (Rp e_x) x (p_i^b - c) ; Rp e_x ]     (Rp = Ry(pitch), c = (x,y,z))
//   pitch : [ e_y x (p_i^b - c) ; e_y ]
// Edge cost d(q_a, q_b) then contributes (dd/dq_a) dq_a/db + (dd/dq_b) dq_b/db, home configs being
// offset-independent (their sensitivities are zero).
// ---------------------------------------------------------------------------------------------

OffsetVec AnalyticGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& joints, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseOffset& base, const BaseGradientParams& params)
{
	const size_t n = tour.size();
	Eigen::Isometry3d xform = MakeObjectOffset(base);
	const Eigen::Vector3d c(base.x, base.y, base.z);
	const Eigen::Vector3d roll_axis =
		Eigen::AngleAxisd(base.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::Vector3d::UnitX();
	const Eigen::Vector3d pitch_axis = Eigen::Vector3d::UnitY();
	const double lambda2 = params.jacobian_damping * params.jacobian_damping;

	// Per visit position: dq/db (dof x 5) and dp/db (3 x 5).
	std::vector<Eigen::MatrixXd> dq_db(n);
	std::vector<Eigen::MatrixXd> dp_db(n);

	for (size_t k = 0; k < n; ++k)
	{
		Eigen::Vector3d r = (xform * tour_tcp_poses_original[tour[k]]).translation() - c;

		Eigen::Matrix<double, 6, 5> S = Eigen::Matrix<double, 6, 5>::Zero();
		S.block<3, 3>(0, 0).setIdentity();
		S.block<3, 1>(0, 3) = roll_axis.cross(r);
		S.block<3, 1>(3, 3) = roll_axis;
		S.block<3, 1>(0, 4) = pitch_axis.cross(r);
		S.block<3, 1>(3, 4) = pitch_axis;

		state.setJointGroupPositions(jmg, joints[k]);
		state.update();
		Eigen::MatrixXd J;  // 6 x dof, [linear; angular] in the model (base) frame
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);

		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += lambda2;
		Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

		dq_db[k] = J_pinv * S;		   // dof x 5
		dp_db[k] = S.topRows<3>();	   // 3 x 5
	}

	const size_t dof = home_joints.size();
	OffsetVec g = OffsetVec::Zero();

	auto add_edge = [&](const std::vector<double>& qa, const std::vector<double>& qb, const Eigen::MatrixXd& dqa,
					   const Eigen::MatrixXd& dqb, const Eigen::MatrixXd& dpa, const Eigen::MatrixXd& dpb,
					   const Eigen::Vector3d& pa, const Eigen::Vector3d& pb) {
		Eigen::VectorXd dq(dof);
		for (size_t i = 0; i < dof; ++i)
			dq(i) = qa[i] - qb[i];
		Eigen::MatrixXd ddiff = dqa - dqb;  // dof x 5

		double nrm = dq.norm();
		if (nrm > 1e-9)
			g += params.joint_distance_weight * (ddiff.transpose() * (dq / nrm));

		int kstar = 0;
		for (size_t i = 1; i < dof; ++i)
			if (std::abs(dq(i)) > std::abs(dq(kstar)))
				kstar = static_cast<int>(i);
		double s = dq(kstar) >= 0.0 ? 1.0 : -1.0;
		g += params.max_joint_deviation_weight * s * ddiff.row(kstar).transpose();

		Eigen::Vector3d du = pa - pb;
		double dn = du.norm();
		if (dn > 1e-9)
			g += params.cartesian_distance_weight * ((dpa - dpb).transpose() * (du / dn));
	};

	Eigen::MatrixXd zero_dq = Eigen::MatrixXd::Zero(dof, 5);
	Eigen::MatrixXd zero_dp = Eigen::MatrixXd::Zero(3, 5);

	if (n > 0)
	{
		Eigen::Vector3d p0 = (xform * tour_tcp_poses_original[tour[0]]).translation();
		add_edge(home_joints, joints[0], zero_dq, dq_db[0], zero_dp, dp_db[0], home_tcp_local, p0);
	}
	for (size_t k = 1; k < n; ++k)
	{
		Eigen::Vector3d pa = (xform * tour_tcp_poses_original[tour[k - 1]]).translation();
		Eigen::Vector3d pb = (xform * tour_tcp_poses_original[tour[k]]).translation();
		add_edge(joints[k - 1], joints[k], dq_db[k - 1], dq_db[k], dp_db[k - 1], dp_db[k], pa, pb);
	}
	return g;
}

// Central-difference gradient of TourWeightedCost via re-tracked IK -- cross-check only.
OffsetVec FiniteDifferenceGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseOffset& base, const BaseGradientParams& params)
{
	const double eps = params.fd_epsilon;
	OffsetVec g = OffsetVec::Constant(std::nan(""));
	for (int axis = 0; axis < 5; ++axis)
	{
		BaseOffset bp = base, bm = base;
		auto component = [](BaseOffset& o, int a) -> double& {
			return a == 0 ? o.x : a == 1 ? o.y : a == 2 ? o.z : a == 3 ? o.roll : o.pitch;
		};
		double* pp = &component(bp, axis);
		double* pm = &component(bm, axis);
		*pp += eps;
		*pm -= eps;
		TrackResult rp = TrackTour(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, tour, seeds, home_joints,
			home_tcp_local, bp, params);
		TrackResult rm = TrackTour(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, tour, seeds, home_joints,
			home_tcp_local, bm, params);
		if (rp.all_reachable && rm.all_reachable)
			g(axis) = (rp.weighted_cost - rm.weighted_cost) / (2.0 * eps);
	}
	return g;
}

// ---------------------------------------------------------------------------------------------
// Live progress markers
// ---------------------------------------------------------------------------------------------

// Offset descent only -- the current offset (cube), the trail it has walked, and the -grad(D)
// arrow. The tour / viewpoints are deliberately NOT drawn here; they appear once, after
// convergence, on /base_gradient_markers (BuildBaseGradientMarkerArray). Every marker below uses a
// fixed id per namespace so each publish overwrites the last instead of stacking ghosts.
void PublishProgress(
	const rclcpp::Node::SharedPtr& node, const BaseGradientParams& params, const std::vector<BaseOffset>& base_history,
	const Eigen::Vector3d& neg_grad_translation, const BaseOffset& base)
{
	if (!params.progress_pub)
		return;

	rclcpp::Time stamp = node->now();
	visualization_msgs::msg::MarkerArray markers;

	// A short cube showing the object offset: position (x,y,z) and tip/tilt (roll,pitch).
	visualization_msgs::msg::Marker base_marker;
	base_marker.header.frame_id = "world";
	base_marker.header.stamp = stamp;
	base_marker.ns = "base_gradient_current";
	base_marker.id = 0;
	base_marker.type = visualization_msgs::msg::Marker::CUBE;
	base_marker.action = visualization_msgs::msg::Marker::ADD;
	base_marker.pose.position.x = base.x;
	base_marker.pose.position.y = base.y;
	base_marker.pose.position.z = base.z;
	Eigen::Quaterniond bq(MakeObjectOffset(base).rotation());
	base_marker.pose.orientation.x = bq.x();
	base_marker.pose.orientation.y = bq.y();
	base_marker.pose.orientation.z = bq.z();
	base_marker.pose.orientation.w = bq.w();
	base_marker.scale.x = base_marker.scale.y = 0.06;
	base_marker.scale.z = 0.015;
	base_marker.color.r = 1.0f;
	base_marker.color.g = 0.85f;
	base_marker.color.a = 1.0f;
	markers.markers.push_back(base_marker);

	if (base_history.size() > 1)
	{
		visualization_msgs::msg::Marker trail;
		trail.header.frame_id = "world";
		trail.header.stamp = stamp;
		trail.ns = "base_gradient_trail";
		trail.id = 0;
		trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
		trail.action = visualization_msgs::msg::Marker::ADD;
		trail.pose.orientation.w = 1.0;
		trail.scale.x = 0.004;
		trail.color.r = 1.0f;
		trail.color.g = 0.4f;
		trail.color.b = 0.1f;
		trail.color.a = 0.8f;

		// All past base poses as one SPHERE_LIST (single marker, fixed id) -- not one marker each,
		// which would leave stale ids behind as the history grows.
		visualization_msgs::msg::Marker crumbs;
		crumbs.header.frame_id = "world";
		crumbs.header.stamp = stamp;
		crumbs.ns = "base_gradient_history";
		crumbs.id = 0;
		crumbs.type = visualization_msgs::msg::Marker::SPHERE_LIST;
		crumbs.action = visualization_msgs::msg::Marker::ADD;
		crumbs.pose.orientation.w = 1.0;
		crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.022;
		crumbs.color.r = 1.0f;
		crumbs.color.g = 0.4f;
		crumbs.color.b = 0.1f;
		crumbs.color.a = 0.7f;

		for (size_t h = 0; h < base_history.size(); ++h)
		{
			geometry_msgs::msg::Point pt;
			pt.x = base_history[h].x;
			pt.y = base_history[h].y;
			pt.z = base_history[h].z;
			trail.points.push_back(pt);
			if (h + 1 < base_history.size())
				crumbs.points.push_back(pt);
		}
		markers.markers.push_back(trail);
		markers.markers.push_back(crumbs);
	}

	// -grad(D) translational part as a 3D arrow from the current offset.
	double gnorm = neg_grad_translation.norm();
	if (gnorm > 1e-9)
	{
		visualization_msgs::msg::Marker arrow;
		arrow.header.frame_id = "world";
		arrow.header.stamp = stamp;
		arrow.ns = "base_gradient_descent_dir";
		arrow.id = 0;
		arrow.type = visualization_msgs::msg::Marker::ARROW;
		arrow.action = visualization_msgs::msg::Marker::ADD;
		arrow.pose.orientation.w = 1.0;
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.scale.z = 0.0;
		arrow.color.r = 0.1f;
		arrow.color.g = 0.5f;
		arrow.color.b = 1.0f;
		arrow.color.a = 0.95f;
		double scale = 0.15 / gnorm;  // fixed on-screen length regardless of magnitude
		geometry_msgs::msg::Point tail, tip;
		tail.x = base.x;
		tail.y = base.y;
		tail.z = base.z;
		tip.x = base.x + neg_grad_translation.x() * scale;
		tip.y = base.y + neg_grad_translation.y() * scale;
		tip.z = base.z + neg_grad_translation.z() * scale;
		arrow.points.push_back(tail);
		arrow.points.push_back(tip);
		markers.markers.push_back(arrow);
	}

	params.progress_pub->publish(markers);
	if (params.visualize_progress_delay_sec > 0.0)
		std::this_thread::sleep_for(std::chrono::duration<double>(params.visualize_progress_delay_sec));
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------------------------

BaseGradientResult SolveBaseGradient(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params)
{
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	const int n = static_cast<int>(tour_tcp_poses_original.size());

	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
	const moveit::core::LinkModel* tool0_link = robot_model->getLinkModel("tool0");

	state.setJointGroupPositions(jmg, start_reference_joints);
	state.update();
	Eigen::Vector3d home_tcp_local = state.getGlobalLinkTransform("tool0").translation();

	BaseGradientResult result;
	result.num_total = n;

	const double rot_scale = std::max(1e-6, params.rot_metric_scale);  // m per rad, for the step metric
	const std::vector<double>& fallback_seed = start_reference_joints;

	auto joint_path_len = [&](const InnerSolution& s) {
		double len = 0.0;
		const std::vector<double>* prev = &start_reference_joints;
		for (const auto& q : s.joints)
		{
			len += JointL2Distance(*prev, q);
			prev = &q;
		}
		return len;
	};

	struct RestartResult
	{
		BaseOffset offset;
		InnerSolution sol;
		double cost = std::numeric_limits<double>::max();
		bool ok = false;  // fully reachable
	};

	// One full gradient descent from `base`, warm-started internally but starting IK/GTSP cold.
	auto run_descent = [&](BaseOffset base, int restart_idx) -> RestartResult {
		base = ProjectToBounds(base, params.bounds);
		std::vector<BaseOffset> base_history{base};
		std::vector<std::vector<double>> seeds(static_cast<size_t>(n), fallback_seed);
		int stall_count = 0;

		RestartResult rr;
		rr.offset = base;

		auto record = [&](const BaseOffset& b, const InnerSolution& s) {
			result.history.push_back(
				{static_cast<double>(restart_idx), b.x, b.y, b.z, b.roll, b.pitch, s.weighted_cost});
			// weighted_cost already includes the unreachable penalty, so the lowest-cost solution
			// is also the one with the best reachability -- no separate all_reachable gate needed.
			if (s.weighted_cost < rr.cost)
			{
				rr.offset = b;
				rr.sol = s;
				rr.cost = s.weighted_cost;
				rr.ok = s.all_reachable;
			}
		};

		InnerSolution cur = BestOfNInnerSolve(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seeds,
			start_reference_joints, home_tcp_local, base, params, nullptr, params.max_solutions_per_candidate);
		result.num_inner_solves += std::max(1, params.solve_restarts);

		if (cur.tour.empty())
		{
			RCLCPP_WARN(node->get_logger(), "restart %d: no tour pose reachable at the start offset", restart_idx + 1);
			return rr;
		}

		record(base, cur);
		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d iter 0: offset (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f deg  D=%.4f  reachable %d/%d",
			restart_idx + 1, std::max(1, params.num_restarts), base.x, base.y, base.z, base.roll * 180.0 / M_PI,
			base.pitch * 180.0 / M_PI, cur.weighted_cost, cur.num_reachable, n);

		for (int outer = 0; outer < params.max_outer_iterations && rclcpp::ok() && !cur.tour.empty(); ++outer)
		{
			OffsetVec g = AnalyticGradient(
				state, jmg, tool0_link, tour_tcp_poses_original, cur.tour, cur.joints, start_reference_joints,
				home_tcp_local, base, params);

			if (params.fd_gradient_check)
			{
				OffsetVec g_fd = FiniteDifferenceGradient(
					state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, cur.tour,
					cur.joints, start_reference_joints, home_tcp_local, base, params);
				RCLCPP_INFO(
					node->get_logger(),
					"    grad check [x y z roll pitch]  analytic (%+.4f %+.4f %+.4f %+.4f %+.4f)  "
					"central-diff (%+.4f %+.4f %+.4f %+.4f %+.4f)",
					g(0), g(1), g(2), g(3), g(4), g_fd(0), g_fd(1), g_fd(2), g_fd(3), g_fd(4));
			}

			// Descend in a metric where 1 rad of tip/tilt equals rot_scale meters.
			OffsetVec g_u = g;
			g_u(3) /= rot_scale;
			g_u(4) /= rot_scale;
			double gnorm = g_u.norm();
			if (gnorm < 1e-6)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: gradient ~ 0 -- converged", restart_idx + 1);
				PublishProgress(node, params, base_history, Eigen::Vector3d(-g.head<3>()), base);
				break;
			}

			// Backtracking line search: every probe re-solves the full inner problem (Phi),
			// warm-started from the current tour; a step is accepted only if Phi itself drops.
			OffsetVec dir_u = -g_u / gnorm;
			seeds = SeedsFromSolution(cur, fallback_seed, static_cast<size_t>(n));
			double step = params.initial_step;
			bool accepted = false;
			BaseOffset b_new = base;
			InnerSolution next;
			for (int ls = 0; ls < params.max_line_search_iters; ++ls)
			{
				BaseOffset cand = ProjectToBounds(
					{base.x + step * dir_u(0), base.y + step * dir_u(1), base.z + step * dir_u(2),
					 base.roll + step * dir_u(3) / rot_scale, base.pitch + step * dir_u(4) / rot_scale},
					params.bounds);
				// Cheap probe: one warm-started IK per pose, GTSP = plain TSP off the current order.
				// weighted_cost carries the unreachable penalty, so a step that drops a viewpoint
				// fails this test and one that regains a viewpoint passes it easily.
				InnerSolution probe = InnerSolve(
					state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seeds,
					start_reference_joints, home_tcp_local, cand, params, &cur.tour, 1);
				if (probe.weighted_cost <= cur.weighted_cost - params.armijo_c * step * gnorm)
				{
					accepted = true;
					b_new = cand;
					next = std::move(probe);
					break;
				}
				step *= params.step_shrink;
				if (step < params.min_step)
					break;
			}

			if (!accepted)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: no step reduces the tour cost -- converged", restart_idx + 1);
				PublishProgress(node, params, base_history, Eigen::Vector3d(-g.head<3>()), base);
				break;
			}

			// Commit with a full multi-branch solve at the accepted offset (cost <= the cheap
			// probe's, so still an improvement); fall back to the probe if that regresses.
			InnerSolution committed = BestOfNInnerSolve(
				state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seeds,
				start_reference_joints, home_tcp_local, b_new, params, &next.tour, params.max_solutions_per_candidate);
			result.num_inner_solves += std::max(1, params.solve_restarts);
			if (committed.weighted_cost > next.weighted_cost)
				committed = std::move(next);

			OffsetVec du = ToOffsetVec(b_new, base);
			du(3) *= rot_scale;
			du(4) *= rot_scale;
			double base_move = du.norm();
			double rel_impr = (cur.weighted_cost - committed.weighted_cost) / std::max(cur.weighted_cost, 1e-9);

			base = b_new;
			cur = std::move(committed);
			base_history.push_back(base);
			record(base, cur);

			RCLCPP_INFO(
				node->get_logger(),
				"restart %d/%d iter %d/%d: offset (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f deg  D=%.4f  "
				"reachable %d/%d  |grad|=%.4f  step=%.4f",
				restart_idx + 1, std::max(1, params.num_restarts), outer + 1, params.max_outer_iterations, base.x,
				base.y, base.z, base.roll * 180.0 / M_PI, base.pitch * 180.0 / M_PI, cur.weighted_cost,
				cur.num_reachable, n, gnorm, step);

			PublishProgress(node, params, base_history, Eigen::Vector3d(-g.head<3>()), base);

			if (rel_impr < params.convergence_tolerance_cost)
			{
				if (++stall_count >= std::max(1, params.patience))
				{
					RCLCPP_INFO(
						node->get_logger(), "  restart %d: %d iterations with <%.1e relative gain -- stopping early",
						restart_idx + 1, stall_count, params.convergence_tolerance_cost);
					break;
				}
			}
			else
			{
				stall_count = 0;
			}

			if (base_move < params.convergence_tolerance_offset && rel_impr < params.convergence_tolerance_cost)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: offset settled -- converged", restart_idx + 1);
				break;
			}
		}

		return rr;  // record() runs at least once, so rr holds this restart's best-cost solution
	};

	std::mt19937 rng(static_cast<unsigned int>(params.random_seed));
	const int num_restarts = std::max(1, params.num_restarts);
	RestartResult overall;
	int since_improved = 0;  // restarts since `overall` last improved (only counted once it is ok)

	for (int r = 0; r < num_restarts && rclcpp::ok(); ++r)
	{
		BaseOffset start =
			(r == 0) ? BaseOffset{params.initial_x, params.initial_y, params.initial_z, params.initial_roll,
								  params.initial_pitch}
					 : PerturbOffset(overall.offset, rng, params.restart_perturbation, rot_scale);
		RestartResult rr = run_descent(start, r);

		// weighted_cost carries the unreachable penalty, so lower cost == better (a fully-
		// reachable result always beats a partial one).
		bool improved = (r == 0) || (rr.cost < overall.cost);

		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d done: D=%.4f  reachable %d/%d  offset (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f deg%s",
			r + 1, num_restarts, rr.cost, rr.sol.num_reachable, n, rr.offset.x, rr.offset.y, rr.offset.z,
			rr.offset.roll * 180.0 / M_PI, rr.offset.pitch * 180.0 / M_PI, (r > 0 && improved) ? "  <-- new best" : "");

		if (improved)
			overall = rr;

		since_improved = improved ? 0 : since_improved + 1;
		if (overall.ok && params.restart_patience > 0 && r + 1 >= params.min_restarts &&
			since_improved >= params.restart_patience)
		{
			RCLCPP_INFO(
				node->get_logger(), "stopping restarts: %d in a row did not beat D=%.4f", since_improved, overall.cost);
			break;
		}
	}

	const InnerSolution& fin = overall.sol;
	result.x = overall.offset.x;
	result.y = overall.offset.y;
	result.z = overall.offset.z;
	result.roll = overall.offset.roll;
	result.pitch = overall.offset.pitch;
	result.tour_order = fin.tour;
	result.joint_solutions = fin.joints;
	// Report the honest tour cost -- strip the unreachable penalty baked in for comparison.
	result.total_weighted_cost =
		fin.weighted_cost - params.unreachable_penalty * (fin.num_total - fin.num_reachable);
	result.total_joint_path_length = joint_path_len(fin);
	result.num_reachable = fin.num_reachable;
	result.ok = overall.ok;

	// Leave the scene as we found it.
	SetObjectPose(planning_scene_monitor, object_pose_original);

	if (result.ok)
		RCLCPP_INFO(
			node->get_logger(),
			"Done. Object offset: translate (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg -- reaches all %d "
			"poses, tour joint path %.4f rad (weighted cost %.4f).",
			result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI, n,
			result.total_joint_path_length, result.total_weighted_cost);
	else
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Object offset: translate (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg -- reaches only "
			"%d/%d poses.",
			result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI,
			result.num_reachable, n);

	return result;
}

void ExportBaseGradientResult(const std::string& output_dir, const BaseGradientResult& result)
{
	std::filesystem::create_directories(output_dir);

	Json::Value root;
	root["ok"] = result.ok;
	root["num_reachable"] = result.num_reachable;
	root["num_total"] = result.num_total;
	root["x"] = result.x;
	root["y"] = result.y;
	root["z"] = result.z;
	root["roll"] = result.roll;
	root["pitch"] = result.pitch;
	root["total_joint_path_length"] = result.total_joint_path_length;
	root["total_weighted_cost"] = result.total_weighted_cost;

	Json::Value tour(Json::arrayValue);
	for (int idx : result.tour_order)
		tour.append(idx);
	root["tour_order"] = tour;

	Json::Value joints(Json::arrayValue);
	for (const auto& sol : result.joint_solutions)
	{
		Json::Value entry(Json::arrayValue);
		for (double v : sol)
			entry.append(v);
		joints.append(entry);
	}
	root["joint_solutions"] = joints;

	Json::Value history(Json::arrayValue);
	for (const auto& h : result.history)
	{
		Json::Value entry(Json::objectValue);
		entry["restart"] = static_cast<int>(h[0]);
		entry["x"] = h[1];
		entry["y"] = h[2];
		entry["z"] = h[3];
		entry["roll"] = h[4];
		entry["pitch"] = h[5];
		entry["weighted_cost"] = h[6];
		history.append(entry);
	}
	root["history"] = history;

	std::string json_path = output_dir + "/base_gradient_result.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved base gradient result JSON: %s\n", json_path.c_str());
}

// ---------------------------------------------------------------------------------------------
// Attribution experiment (see base_gradient.hpp). Reuses the file-local InnerSolve / GTSP so the
// costs here are exactly the descent's objective.
// ---------------------------------------------------------------------------------------------
namespace
{

double MetricNorm(const BaseOffset& o, double rot_scale)
{
	return std::sqrt(
		o.x * o.x + o.y * o.y + o.z * o.z + (o.roll * rot_scale) * (o.roll * rot_scale) +
		(o.pitch * rot_scale) * (o.pitch * rot_scale));
}

BaseGradientPointEval EvalPoint(const InnerSolution& s, const BaseOffset& at, double rot_scale, double unreachable_penalty)
{
	BaseGradientPointEval e;
	e.weighted_cost = s.weighted_cost;
	e.honest_cost = s.weighted_cost - unreachable_penalty * (s.num_total - s.num_reachable);
	e.num_reachable = s.num_reachable;
	e.all_reachable = s.all_reachable;
	e.offset = {at.x, at.y, at.z, at.roll, at.pitch};
	e.d_metric = MetricNorm(at, rot_scale);
	return e;
}

}  // namespace

BaseGradientExperimentResult RunBaseGradientExperiment(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params, int num_random_directions, double min_probe_d_metric, int random_search_budget)
{
	BaseGradientExperimentResult out;
	const int n = static_cast<int>(tour_tcp_poses_original.size());
	out.num_total = n;
	out.seed = params.random_seed;
	const double rot_scale = std::max(1e-6, params.rot_metric_scale);
	out.rot_metric_scale = rot_scale;
	const int sr = std::max(1, params.solve_restarts);

	// b* for this seed -- one full descent.
	RCLCPP_INFO(node->get_logger(), "[experiment] seed %d: descending for b* ...", params.random_seed);
	BaseGradientResult descent = SolveBaseGradient(
		node, robot_model, planning_scene_monitor, group_name, object_translation_original, object_rotation_original,
		tour_tcp_poses_original, start_reference_joints, params);

	out.descent_ok = descent.ok;
	out.descent_offset = {descent.x, descent.y, descent.z, descent.roll, descent.pitch};
	out.descent_reported_cost = descent.total_weighted_cost;
	out.descent_inner_solves = descent.num_inner_solves;
	const BaseOffset bstar{descent.x, descent.y, descent.z, descent.roll, descent.pitch};
	out.descent_d_metric = MetricNorm(bstar, rot_scale);

	double probe_d = out.descent_d_metric;
	if (probe_d < min_probe_d_metric)
	{
		probe_d = min_probe_d_metric;
		out.probe_d_floored = true;
		RCLCPP_WARN(
			node->get_logger(), "[experiment] |b*| = %.5f is ~0; probing exp 3/4 at a floor radius of %.4f",
			out.descent_d_metric, probe_d);
	}
	out.probe_d_metric = probe_d;

	// Solve setup (mirrors SolveBaseGradient).
	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
	state.setJointGroupPositions(jmg, start_reference_joints);
	state.update();
	const Eigen::Vector3d home_tcp_local = state.getGlobalLinkTransform("tool0").translation();
	const Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	const std::vector<double>& home = start_reference_joints;
	const std::vector<std::vector<double>> cold_seeds(static_cast<size_t>(n), home);

	auto cold_solve = [&](const BaseOffset& b) {
		return BestOfNInnerSolve(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, cold_seeds, home,
			home_tcp_local, b, params, nullptr, params.max_solutions_per_candidate);
	};

	// exp 1: cold solve at both endpoints.
	const InnerSolution s0 = cold_solve(BaseOffset{});
	out.c0_cold = EvalPoint(s0, BaseOffset{}, rot_scale, params.unreachable_penalty);
	const InnerSolution sopt = cold_solve(bstar);
	out.copt_cold = EvalPoint(sopt, bstar, rot_scale, params.unreachable_penalty);
	RCLCPP_INFO(
		node->get_logger(), "[experiment] exp1 cold: C0=%.3f (%d/%d)  Copt=%.3f (%d/%d)  dPhi=%+.3f", out.c0_cold.honest_cost,
		out.c0_cold.num_reachable, n, out.copt_cold.honest_cost, out.copt_cold.num_reachable, n,
		out.copt_cold.honest_cost - out.c0_cold.honest_cost);

	// Warm start for exp 4: IK seeds + GTSP order from the offset-0 cold solution, single hop.
	const std::vector<std::vector<double>> seeds0 = SeedsFromSolution(s0, home, static_cast<size_t>(n));
	const std::vector<int> warm_order0 = s0.tour;
	auto warm_solve = [&](const BaseOffset& b) {
		return BestOfNInnerSolve(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, seeds0, home,
			home_tcp_local, b, params, &warm_order0, params.max_solutions_per_candidate);
	};

	out.copt_warm = EvalPoint(warm_solve(bstar), bstar, rot_scale, params.unreachable_penalty);

	// exp 3 / 4: random offsets of the same mixed-metric magnitude as b*.
	std::mt19937 rng(static_cast<unsigned int>(params.random_seed));
	std::normal_distribution<double> gauss(0.0, 1.0);
	const int k = std::max(0, num_random_directions);
	out.rand_cold.reserve(static_cast<size_t>(k));
	out.rand_warm.reserve(static_cast<size_t>(k));
	for (int i = 0; i < k; ++i)
	{
		double v[5];
		double sq = 0.0;
		for (double& c : v)
		{
			c = gauss(rng);
			sq += c * c;
		}
		const double inv = 1.0 / std::sqrt(std::max(sq, 1e-12));
		// Unit vector in the mixed metric, scaled to probe_d, then mapped back to offset units.
		BaseOffset rb = ProjectToBounds(
			{probe_d * v[0] * inv, probe_d * v[1] * inv, probe_d * v[2] * inv, probe_d * v[3] * inv / rot_scale,
			 probe_d * v[4] * inv / rot_scale},
			params.bounds);

		out.rand_cold.push_back(EvalPoint(cold_solve(rb), rb, rot_scale, params.unreachable_penalty));
		out.rand_warm.push_back(EvalPoint(warm_solve(rb), rb, rot_scale, params.unreachable_penalty));
		RCLCPP_INFO(
			node->get_logger(), "[experiment] exp3/4 dir %d/%d: cold=%.3f  warm=%.3f", i + 1, k,
			out.rand_cold.back().honest_cost, out.rand_warm.back().honest_cost);
	}

	// exp 5: random search of matched budget -- offsets drawn uniformly in the bounds, cold
	// best-of-N solve each. Budget <= 0 => same number of committed solves the descent used
	// (descent_inner_solves / solve_restarts, i.e. iterations + 1); else that many points.
	const int rs_points =
		random_search_budget > 0 ? random_search_budget : std::max(1, out.descent_inner_solves / sr);
	std::uniform_real_distribution<double> ux(params.bounds.x_min, params.bounds.x_max);
	std::uniform_real_distribution<double> uy(params.bounds.y_min, params.bounds.y_max);
	std::uniform_real_distribution<double> uz(params.bounds.z_min, params.bounds.z_max);
	std::uniform_real_distribution<double> ur(params.bounds.roll_min, params.bounds.roll_max);
	std::uniform_real_distribution<double> up(params.bounds.pitch_min, params.bounds.pitch_max);
	out.random_search.reserve(static_cast<size_t>(rs_points));
	double rs_best = std::numeric_limits<double>::max();
	for (int i = 0; i < rs_points; ++i)
	{
		BaseOffset rb{ux(rng), uy(rng), uz(rng), ur(rng), up(rng)};
		BaseGradientPointEval e = EvalPoint(cold_solve(rb), rb, rot_scale, params.unreachable_penalty);
		out.random_search.push_back(e);
		rs_best = std::min(rs_best, e.honest_cost);
		RCLCPP_INFO(
			node->get_logger(), "[experiment] exp5 random-search %d/%d: cold=%.3f  (best so far %.3f)", i + 1,
			rs_points, e.honest_cost, rs_best);
	}

	SetObjectPose(planning_scene_monitor, object_pose_original);  // leave the scene as we found it
	return out;
}

void ExportBaseGradientExperimentResult(const std::string& output_dir, const BaseGradientExperimentResult& result)
{
	std::filesystem::create_directories(output_dir);

	auto eval_to_json = [](const BaseGradientPointEval& e) {
		Json::Value j;
		j["weighted_cost"] = e.weighted_cost;
		j["honest_cost"] = e.honest_cost;
		j["num_reachable"] = e.num_reachable;
		j["all_reachable"] = e.all_reachable;
		j["d_metric"] = e.d_metric;
		Json::Value off(Json::arrayValue);
		for (double c : e.offset)
			off.append(c);
		j["offset"] = off;
		return j;
	};

	Json::Value root;
	root["seed"] = result.seed;
	root["num_total"] = result.num_total;
	root["rot_metric_scale"] = result.rot_metric_scale;
	root["probe_d_metric"] = result.probe_d_metric;
	root["probe_d_floored"] = result.probe_d_floored;

	Json::Value descent(Json::objectValue);
	descent["ok"] = result.descent_ok;
	descent["d_metric"] = result.descent_d_metric;
	descent["reported_weighted_cost"] = result.descent_reported_cost;
	descent["inner_solves"] = result.descent_inner_solves;
	Json::Value doff(Json::arrayValue);
	for (double c : result.descent_offset)
		doff.append(c);
	descent["offset"] = doff;
	root["descent"] = descent;

	Json::Value exp1(Json::objectValue);
	exp1["c0_cold"] = eval_to_json(result.c0_cold);
	exp1["copt_cold"] = eval_to_json(result.copt_cold);
	exp1["delta_honest"] = result.copt_cold.honest_cost - result.c0_cold.honest_cost;
	root["exp1_cold_endpoints"] = exp1;

	Json::Value exp3(Json::arrayValue);
	for (const auto& e : result.rand_cold)
		exp3.append(eval_to_json(e));
	root["exp3_cold_placebo"] = exp3;

	Json::Value exp4(Json::objectValue);
	exp4["copt_warm"] = eval_to_json(result.copt_warm);
	Json::Value exp4rand(Json::arrayValue);
	for (const auto& e : result.rand_warm)
		exp4rand.append(eval_to_json(e));
	exp4["random"] = exp4rand;
	root["exp4_warm_placebo"] = exp4;

	Json::Value exp5(Json::arrayValue);
	for (const auto& e : result.random_search)
		exp5.append(eval_to_json(e));
	root["exp5_random_search"] = exp5;

	const std::string json_path =
		output_dir + "/base_gradient_experiment_seed" + std::to_string(result.seed) + ".json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved base gradient experiment JSON: %s\n", json_path.c_str());
}

ObjectOffsetScore ScoreObjectOffset(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params, const std::array<double, 5>& offset, const std::vector<int>& fixed_order)
{
	(void)node;
	ObjectOffsetScore out;
	const int n = static_cast<int>(tour_tcp_poses_original.size());
	out.num_total = n;

	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
	state.setJointGroupPositions(jmg, start_reference_joints);
	state.update();
	const Eigen::Vector3d home_tcp_local = state.getGlobalLinkTransform("tool0").translation();
	const Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	const std::vector<double>& home = start_reference_joints;
	const std::vector<std::vector<double>> cold_seeds(static_cast<size_t>(n), home);

	const BaseOffset b = ProjectToBounds(
		{offset[0], offset[1], offset[2], offset[3], offset[4]}, params.bounds);
	const int R = std::max(1, params.solve_restarts);

	double best_wc = std::numeric_limits<double>::max();
	for (int r = 0; r < R; ++r)
	{
		const auto branches = CollectIkSolutions(
			state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, cold_seeds, b, params,
			params.max_solutions_per_candidate);

		InnerSolution s;
		if (!fixed_order.empty())
		{
			s = SolveFixedOrder(branches, b, tour_tcp_poses_original, home, home_tcp_local, params, fixed_order);
		}
		else
		{
			const GtspSolution g = RunGtsp(
				branches, b, tour_tcp_poses_original, home, home_tcp_local, params, nullptr);
			s.tour = g.tour_pose_indices;
			s.joints = g.chosen_joints;
			s.num_total = n;
			s.num_reachable = static_cast<int>(s.tour.size());
			s.all_reachable = (s.num_reachable == n);
			s.weighted_cost =
				TourWeightedCost(b, s.tour, s.joints, tour_tcp_poses_original, home, home_tcp_local, params) +
				params.unreachable_penalty * (n - s.num_reachable);
		}

		if (s.weighted_cost < best_wc)
		{
			best_wc = s.weighted_cost;
			out.weighted_cost = s.weighted_cost;
			out.honest_cost = s.weighted_cost - params.unreachable_penalty * (n - s.num_reachable);
			out.num_reachable = s.num_reachable;
			out.all_reachable = s.all_reachable;
			out.tour = s.tour;
			out.joints = s.joints;
		}
	}

	SetObjectPose(planning_scene_monitor, object_pose_original);  // leave the scene as we found it
	return out;
}

// ---------------------------------------------------------------------------------------------
// Placement / order separability experiment (see base_gradient.hpp).
// ---------------------------------------------------------------------------------------------
namespace
{

// Grid coordinate i of n points spanning [mn, mx]; the midpoint for n == 1. With odd n and a
// symmetric span the centre index lands exactly on 0, so the nominal offset is on the grid.
double GridCoord(int i, int n, double mn, double mx)
{
	if (n <= 1)
		return 0.5 * (mn + mx);
	return mn + (mx - mn) * (static_cast<double>(i) / (n - 1));
}

// A solved tour (reachable viewpoints only) padded to a full permutation of 0..num_total-1 by
// appending the missing indices in ascending order -- so a reference route always names every
// viewpoint even if the offset it was solved at could not reach them all.
std::vector<int> PadToFullPermutation(const std::vector<int>& tour, int num_total)
{
	std::vector<char> seen(num_total, 0);
	std::vector<int> out;
	out.reserve(num_total);
	for (int v : tour)
		if (v >= 0 && v < num_total && !seen[v])
		{
			seen[v] = 1;
			out.push_back(v);
		}
	for (int v = 0; v < num_total; ++v)
		if (!seen[v])
			out.push_back(v);
	return out;
}

}  // namespace

PlacementOrderExperimentResult RunPlacementOrderExperiment(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params, int grid_n, int grid_start, int grid_count,
	const std::string& reference_orders_file, const std::string& output_dir)
{
	PlacementOrderExperimentResult out;
	const int n = static_cast<int>(tour_tcp_poses_original.size());
	out.seed = params.random_seed;
	out.num_total = n;
	out.unreachable_penalty = params.unreachable_penalty;

	grid_n = std::max(1, grid_n);
	out.grid_shape = {grid_n, grid_n, grid_n};
	out.grid_min = {params.bounds.x_min, params.bounds.y_min, params.bounds.z_min};
	out.grid_max = {params.bounds.x_max, params.bounds.y_max, params.bounds.z_max};

	// Solve setup (mirrors SolveBaseGradient / RunBaseGradientExperiment).
	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
	state.setJointGroupPositions(jmg, start_reference_joints);
	state.update();
	const Eigen::Vector3d home_tcp_local = state.getGlobalLinkTransform("tool0").translation();
	const Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	const std::vector<double>& home = start_reference_joints;
	const std::vector<std::vector<double>> cold_seeds(static_cast<size_t>(n), home);
	const int R = std::max(1, params.solve_restarts);

	// ---- reference routes -----------------------------------------------------------------
	// Load from file when given (so every shard scores against an identical set); otherwise
	// solve them -- full GTSP at the nominal offset + 6 spread offsets -- and write them out.
	if (!reference_orders_file.empty())
	{
		std::ifstream rf(reference_orders_file);
		Json::Value root;
		Json::CharReaderBuilder rb;
		std::string errs;
		if (!rf.is_open() || !Json::parseFromStream(rb, rf, &root, &errs))
		{
			RCLCPP_ERROR(
				node->get_logger(), "[placement] cannot read reference routes from %s (%s) -- aborting",
				reference_orders_file.c_str(), errs.c_str());
			return out;
		}
		for (const auto& l : root["order_labels"])
			out.order_labels.push_back(l.asString());
		for (const auto& ord : root["reference_orders"])
		{
			std::vector<int> v;
			for (const auto& e : ord)
				v.push_back(e.asInt());
			out.reference_orders.push_back(PadToFullPermutation(v, n));
		}
		RCLCPP_INFO(
			node->get_logger(), "[placement] loaded %zu reference routes from %s", out.reference_orders.size(),
			reference_orders_file.c_str());
	}
	else
	{
		const double sx = 0.7 * std::max(std::abs(params.bounds.x_min), std::abs(params.bounds.x_max));
		const double sy = 0.7 * std::max(std::abs(params.bounds.y_min), std::abs(params.bounds.y_max));
		const double sz = 0.7 * std::max(std::abs(params.bounds.z_min), std::abs(params.bounds.z_max));
		const std::vector<std::pair<std::string, BaseOffset>> seed_offsets = {
			{"gtsp@nominal", {0, 0, 0, 0, 0}}, {"gtsp@+x", {sx, 0, 0, 0, 0}},   {"gtsp@-x", {-sx, 0, 0, 0, 0}},
			{"gtsp@+y", {0, sy, 0, 0, 0}},	  {"gtsp@-y", {0, -sy, 0, 0, 0}},  {"gtsp@+z", {0, 0, sz, 0, 0}},
			{"gtsp@-z", {0, 0, -sz, 0, 0}},
		};
		for (const auto& [label, off] : seed_offsets)
		{
			if (!rclcpp::ok())
				break;
			const InnerSolution s = BestOfNInnerSolve(
				state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, cold_seeds, home,
				home_tcp_local, off, params, nullptr, params.max_solutions_per_candidate);
			out.order_labels.push_back(label);
			out.reference_orders.push_back(PadToFullPermutation(s.tour, n));
			RCLCPP_INFO(
				node->get_logger(), "[placement] reference route %s: full GTSP reaches %d/%d, honest cost %.3f",
				label.c_str(), s.num_reachable, n, s.weighted_cost - params.unreachable_penalty * (n - s.num_reachable));
		}

		Json::Value root;
		Json::Value labels(Json::arrayValue), orders(Json::arrayValue);
		for (const auto& l : out.order_labels)
			labels.append(l);
		for (const auto& ord : out.reference_orders)
		{
			Json::Value a(Json::arrayValue);
			for (int v : ord)
				a.append(v);
			orders.append(a);
		}
		root["order_labels"] = labels;
		root["reference_orders"] = orders;
		root["seed"] = params.random_seed;
		std::filesystem::create_directories(output_dir);
		const std::string rpath =
			output_dir + "/placement_reference_orders_seed" + std::to_string(params.random_seed) + ".json";
		std::ofstream rout(rpath);
		Json::StreamWriterBuilder wb;
		wb["indentation"] = "";
		std::unique_ptr<Json::StreamWriter>(wb.newStreamWriter())->write(root, &rout);
		RCLCPP_INFO(node->get_logger(), "[placement] wrote reference routes to %s", rpath.c_str());
	}
	const int K = static_cast<int>(out.reference_orders.size());
	if (K == 0)
	{
		RCLCPP_ERROR(node->get_logger(), "[placement] no reference routes available -- aborting");
		return out;
	}

	// ---- grid sweep (this shard's slice) ---------------------------------------------------
	const int P = grid_n * grid_n * grid_n;
	const int lo = std::clamp(grid_start, 0, P);
	const int hi = grid_count < 0 ? P : std::min(P, lo + grid_count);
	out.grid_start = lo;
	out.grid_count = std::max(0, hi - lo);
	RCLCPP_INFO(
		node->get_logger(), "[placement] seed %d: sweeping grid points [%d, %d) of %d (%d^3), %d routes, min-of-%d",
		params.random_seed, lo, hi, P, grid_n, K, R);

	for (int idx = lo; idx < hi && rclcpp::ok(); ++idx)
	{
		const int ix = idx / (grid_n * grid_n);
		const int iy = (idx / grid_n) % grid_n;
		const int iz = idx % grid_n;
		const BaseOffset b{
			GridCoord(ix, grid_n, params.bounds.x_min, params.bounds.x_max),
			GridCoord(iy, grid_n, params.bounds.y_min, params.bounds.y_max),
			GridCoord(iz, grid_n, params.bounds.z_min, params.bounds.z_max), 0.0, 0.0};

		PlacementGridPoint gp;
		gp.offset = {b.x, b.y, b.z};
		gp.order_weighted_cost.assign(K, std::numeric_limits<double>::max());
		gp.order_honest_cost.assign(K, std::numeric_limits<double>::max());
		gp.order_num_reachable.assign(K, 0);
		double full_wc = std::numeric_limits<double>::max();

		// Per-grid-point seed-pose RNG: keeps the bulk of a point's IK-seed variation independent of
		// the shard layout (MoveIt's own IK RNG still advances globally, so shards are not bit-exact).
		random_numbers::RandomNumberGenerator gp_rng(
			static_cast<unsigned int>(params.random_seed) * 1000003u + static_cast<unsigned int>(idx));

		for (int r = 0; r < R && rclcpp::ok(); ++r)
		{
			const auto branches = CollectIkSolutions(
				state, jmg, planning_scene_monitor, object_pose_original, tour_tcp_poses_original, cold_seeds, b, params,
				params.max_solutions_per_candidate, &gp_rng);

			int ik_reachable = 0;
			for (const auto& br : branches)
				ik_reachable += !br.empty();
			gp.num_ik_reachable = std::max(gp.num_ik_reachable, ik_reachable);

			for (int k = 0; k < K; ++k)
			{
				const InnerSolution s = SolveFixedOrder(
					branches, b, tour_tcp_poses_original, home, home_tcp_local, params, out.reference_orders[k]);
				if (s.weighted_cost < gp.order_weighted_cost[k])
				{
					gp.order_weighted_cost[k] = s.weighted_cost;
					gp.order_honest_cost[k] = s.weighted_cost - params.unreachable_penalty * (n - s.num_reachable);
					gp.order_num_reachable[k] = s.num_reachable;
				}
			}

			const GtspSolution g = RunGtsp(
				branches, b, tour_tcp_poses_original, home, home_tcp_local, params, nullptr);
			const int g_reach = static_cast<int>(g.tour_pose_indices.size());
			const double g_wc =
				TourWeightedCost(
					b, g.tour_pose_indices, g.chosen_joints, tour_tcp_poses_original, home, home_tcp_local, params) +
				params.unreachable_penalty * (n - g_reach);
			if (g_wc < full_wc)
			{
				full_wc = g_wc;
				gp.full_weighted_cost = g_wc;
				gp.full_honest_cost = g_wc - params.unreachable_penalty * (n - g_reach);
				gp.full_num_reachable = g_reach;
				gp.full_tour = g.tour_pose_indices;
			}
		}

		if ((idx - lo) % 10 == 0 || idx + 1 == hi)
			RCLCPP_INFO(
				node->get_logger(),
				"[placement] %d/%d  offset (%.3f, %.3f, %.3f)  ik %d/%d  %s wc %.2f (%d/%d)  full wc %.2f (%d/%d)",
				idx - lo + 1, hi - lo, b.x, b.y, b.z, gp.num_ik_reachable, n, out.order_labels[0].c_str(),
				gp.order_weighted_cost[0], gp.order_num_reachable[0], n, gp.full_weighted_cost, gp.full_num_reachable, n);

		out.grid.push_back(std::move(gp));
	}

	SetObjectPose(planning_scene_monitor, object_pose_original);  // leave the scene as we found it
	return out;
}

void ExportPlacementOrderExperimentResult(const std::string& output_dir, const PlacementOrderExperimentResult& result)
{
	std::filesystem::create_directories(output_dir);

	Json::Value root;
	root["seed"] = result.seed;
	root["num_total"] = result.num_total;
	root["unreachable_penalty"] = result.unreachable_penalty;

	Json::Value shape(Json::arrayValue), gmin(Json::arrayValue), gmax(Json::arrayValue);
	for (int v : result.grid_shape)
		shape.append(v);
	for (double v : result.grid_min)
		gmin.append(v);
	for (double v : result.grid_max)
		gmax.append(v);
	root["grid_shape"] = shape;
	root["grid_min"] = gmin;
	root["grid_max"] = gmax;
	root["grid_start"] = result.grid_start;
	root["grid_count"] = result.grid_count;

	Json::Value labels(Json::arrayValue);
	for (const auto& l : result.order_labels)
		labels.append(l);
	root["order_labels"] = labels;

	Json::Value orders(Json::arrayValue);
	for (const auto& ord : result.reference_orders)
	{
		Json::Value a(Json::arrayValue);
		for (int v : ord)
			a.append(v);
		orders.append(a);
	}
	root["reference_orders"] = orders;

	Json::Value grid(Json::arrayValue);
	for (const auto& gp : result.grid)
	{
		Json::Value j;
		Json::Value off(Json::arrayValue);
		for (double v : gp.offset)
			off.append(v);
		j["offset"] = off;
		j["num_ik_reachable"] = gp.num_ik_reachable;

		Json::Value owc(Json::arrayValue), ohc(Json::arrayValue), onr(Json::arrayValue);
		for (double v : gp.order_weighted_cost)
			owc.append(v);
		for (double v : gp.order_honest_cost)
			ohc.append(v);
		for (int v : gp.order_num_reachable)
			onr.append(v);
		j["order_weighted_cost"] = owc;
		j["order_honest_cost"] = ohc;
		j["order_num_reachable"] = onr;

		j["full_weighted_cost"] = gp.full_weighted_cost;
		j["full_honest_cost"] = gp.full_honest_cost;
		j["full_num_reachable"] = gp.full_num_reachable;
		Json::Value ft(Json::arrayValue);
		for (int v : gp.full_tour)
			ft.append(v);
		j["full_tour"] = ft;

		grid.append(j);
	}
	root["grid"] = grid;

	const std::string json_path = output_dir + "/placement_experiment_seed" + std::to_string(result.seed) + "_g" +
		std::to_string(result.grid_start) + ".json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved placement/order experiment JSON: %s\n", json_path.c_str());
}

void ApplyObjectOffsetToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original, double x,
	double y, double z, double roll, double pitch)
{
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);
	Eigen::Isometry3d xform = MakeObjectOffset(BaseOffset{x, y, z, roll, pitch});
	SetObjectPose(planning_scene_monitor, xform * object_pose_original);
}

visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const BaseGradientResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	Eigen::Isometry3d xform = MakeObjectOffset({result.x, result.y, result.z, result.roll, result.pitch});
	Eigen::Isometry3d object_pose_original = MakeIsometry(object_translation_original, object_rotation_original);

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "base_gradient_object";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose = ToPoseMsg(xform * object_pose_original);
	mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = mesh_scale;
	mesh_marker.color.r = mesh_marker.color.g = mesh_marker.color.b = 0.7f;
	mesh_marker.color.a = 0.5f;
	markers.markers.push_back(mesh_marker);

	visualization_msgs::msg::Marker line;
	line.header.frame_id = "world";
	line.header.stamp = stamp;
	line.ns = "base_gradient_tour";
	line.id = id++;
	line.type = visualization_msgs::msg::Marker::LINE_STRIP;
	line.action = visualization_msgs::msg::Marker::ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.002;
	line.color.r = 0.1f;
	line.color.g = 0.9f;
	line.color.b = 0.1f;
	line.color.a = 0.9f;

	for (size_t k = 0; k < result.tour_order.size(); ++k)
	{
		Eigen::Isometry3d local = xform * tour_tcp_poses_original[result.tour_order[k]];

		visualization_msgs::msg::Marker sphere;
		sphere.header.frame_id = "world";
		sphere.header.stamp = stamp;
		sphere.ns = "base_gradient_waypoints";
		sphere.id = id++;
		sphere.type = visualization_msgs::msg::Marker::SPHERE;
		sphere.action = visualization_msgs::msg::Marker::ADD;
		sphere.pose.position.x = local.translation().x();
		sphere.pose.position.y = local.translation().y();
		sphere.pose.position.z = local.translation().z();
		sphere.pose.orientation.w = 1.0;
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		bool reachable = k < result.joint_solutions.size() && !result.joint_solutions[k].empty();
		sphere.color.r = reachable ? 0.1f : 0.9f;
		sphere.color.g = reachable ? 0.9f : 0.1f;
		sphere.color.b = 0.1f;
		sphere.color.a = 1.0f;
		markers.markers.push_back(sphere);

		line.points.push_back(sphere.pose.position);
	}
	markers.markers.push_back(line);

	return markers;
}
