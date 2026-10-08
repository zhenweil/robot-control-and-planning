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
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <random_numbers/random_numbers.h>
#include <std_msgs/msg/color_rgba.hpp>

namespace
{

// Object placement: absolute position (x, y, z, base frame) plus tilt about that position (roll about
// base x, pitch about base y), relative to the nominal orientation.
struct ObjectPlacement
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
};

// Maps poses given relative to the object's position (base-frame axes, "_obj") to absolute poses.
Eigen::Isometry3d MakePlacement(const ObjectPlacement& p)
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

ObjectPlacement ProjectToBounds(ObjectPlacement p, const BaseGradientBounds& b)
{
	p.x = std::clamp(p.x, b.x_min, b.x_max);
	p.y = std::clamp(p.y, b.y_min, b.y_max);
	p.z = std::clamp(p.z, b.z_min, b.z_max);
	p.roll = std::clamp(p.roll, b.roll_min, b.roll_max);
	p.pitch = std::clamp(p.pitch, b.pitch_min, b.pitch_max);
	return p;
}

Eigen::Matrix<double, 5, 1> ToOffsetVec(const ObjectPlacement& a, const ObjectPlacement& b)  // a - b, component-wise
{
	Eigen::Matrix<double, 5, 1> v;
	v << a.x - b.x, a.y - b.y, a.z - b.z, a.roll - b.roll, a.pitch - b.pitch;
	return v;
}

// Gaussian kick around `center`: sigma_m meters on x/y/z, sigma_m/rot_scale rad on roll/pitch.
ObjectPlacement PerturbOffset(const ObjectPlacement& center, std::mt19937& rng, double sigma_m, double rot_scale)
{
	std::normal_distribution<double> nm(0.0, sigma_m);
	std::normal_distribution<double> nr(0.0, sigma_m / std::max(1e-6, rot_scale));
	return {center.x + nm(rng), center.y + nm(rng), center.z + nm(rng), center.roll + nr(rng),
			center.pitch + nr(rng)};
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

// Yoshikawa manipulability w = sqrt(det(J J^T)) of tool0 at q.
double Manipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<double>& q)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	return std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
}

// Analytic dw/dq: dw/dq_i = w * tr((J J^T)^-1 J dJ_i^T). For revolute joints, column j of dJ/dq_i is
// [z_min(i,j) x Jv_max(i,j); i < j ? z_i x z_j : 0], built from J itself.
Eigen::VectorXd ManipulabilityJointGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<double>& q)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;  // 6 x n, [linear; angular] in the base frame
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	const int n = static_cast<int>(J.cols());
	const Eigen::MatrixXd A = J * J.transpose();
	const double w = std::sqrt(std::max(0.0, A.determinant()));
	Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
	if (w < 1e-12)
		return g;
	const Eigen::MatrixXd M = A.ldlt().solve(J);  // (J J^T)^-1 J
	Eigen::MatrixXd dJ(6, n);
	for (int i = 0; i < n; ++i)
	{
		dJ.setZero();
		for (int j = 0; j < n; ++j)
		{
			const int lo = std::min(i, j), hi = std::max(i, j);
			dJ.col(j).head<3>() = J.col(lo).tail<3>().cross(J.col(hi).head<3>());
			if (i < j)
				dJ.col(j).tail<3>() = J.col(i).tail<3>().cross(J.col(j).tail<3>());
		}
		g(i) = w * M.cwiseProduct(dJ).sum();  // tr(M dJ^T)
	}
	return g;
}

// If two points are too close, use jacobian to push them apart
void ClosePairSeparationGradients(
	const collision_detection::DistanceResult& res, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, double margin, std::vector<Eigen::RowVectorXd>& close_pair_gradients,
	std::vector<double>& dists)
{
	const auto& model = state.getRobotModel();
	for (const auto& pair_entry : res.distances)
		for (const auto& d : pair_entry.second)
		{
			if (d.distance >= margin)
				continue;
			Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(jmg->getVariableCount());
			bool any_robot_side = false;
			for (int side = 0; side < 2; ++side)
			{
				if (d.body_types[side] != collision_detection::BodyType::ROBOT_LINK &&
					d.body_types[side] != collision_detection::BodyType::ROBOT_ATTACHED)
					continue;
				const moveit::core::LinkModel* link = model->getLinkModel(d.link_names[side]);
				if (!link)
					continue;
				const Eigen::Vector3d local_point =
					state.getGlobalLinkTransform(link).inverse() * d.nearest_points[side];
				Eigen::MatrixXd J_link;
				state.getJacobian(jmg, link, local_point, J_link);
				// distance ~ distance0 + normal.(dp1 - dp0); side 0 moves -normal, side 1 moves +normal.
				row += (side == 0 ? -1.0 : 1.0) * (d.normal.transpose() * J_link.topRows<3>());
				any_robot_side = true;
			}
			if (any_robot_side && row.norm() > 1e-9)
			{
				close_pair_gradients.push_back(row);
				dists.push_back(d.distance);
			}
		}
}

// Pose gap in meters: |position|^2 + (rot_scale * |rotation|)^2, square-rooted.
double WeightedPoseGap(const Eigen::Matrix<double, 6, 1>& e, double rot_scale)
{
	return std::sqrt(e.head<3>().squaredNorm() + rot_scale * rot_scale * e.tail<3>().squaredNorm());
}

struct ClosestIkResult
{
	bool found = false;
	Eigen::Matrix<double, 6, 1> pose_error = Eigen::Matrix<double, 6, 1>::Zero();  // [position; axis*angle] error
	double gap = std::numeric_limits<double>::max();
	std::vector<double> joints;  // the best collision-free pose; empty if none found
};

ClosestIkResult CollisionAwareClosestIk(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const moveit::core::LinkModel* tool0_link,
	const Eigen::Isometry3d& target_pose, const std::vector<double>& initial_joints, int iters, double margin, double rot_scale)
{
	const double damping = 0.01; // for jacobian near singular positions
	const double max_joint_step = 0.1;
	// Stop once the smallest gap so far improves by less than 1 mm over 5 steps.
	const int stall_window = 5;
	const double stall_gain = 1e-3;
	const int dof = static_cast<int>(jmg->getVariableCount());
	collision_detection::DistanceRequest req;
	req.enable_nearest_points = true;
	req.enable_signed_distance = true;
	req.type = collision_detection::DistanceRequestType::ALL;
	req.group_name = jmg->getName();
	req.enableGroup(state.getRobotModel());
	req.distance_threshold = margin;
	req.max_contacts_per_body = 2;

	std::vector<double> q = initial_joints;
	ClosestIkResult best;
	std::vector<double> min_gap_history;  // smallest gap seen up to each step
	for (int it = 0; it <= iters; ++it)
	{
		state.setJointGroupPositions(jmg, q);
		state.update();
		const Eigen::Isometry3d& fk = state.getGlobalLinkTransform(tool0_link);
		Eigen::Matrix<double, 6, 1> pose_error;
		pose_error.head<3>() = target_pose.translation() - fk.translation();
		const Eigen::AngleAxisd aa(target_pose.linear() * fk.linear().transpose());
		pose_error.tail<3>() = aa.axis() * aa.angle();
		const double gap = WeightedPoseGap(pose_error, rot_scale);

		// Arm-arm and arm-object pairs within the margin.
		std::vector<Eigen::RowVectorXd> close_pair_gradients;
		std::vector<double> dists;
		double min_pair_distance = std::numeric_limits<double>::max();  // over pairs within the margin
		{
			collision_detection::DistanceResult self_res, world_res;
			planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
			// Skip pairs the scene always allows (adjacent links, hand/fingers), as the validity check does.
			req.acm = &locked_scene->getAllowedCollisionMatrix();
			locked_scene->getCollisionEnv()->distanceSelf(req, self_res, state);
			locked_scene->getCollisionEnv()->distanceRobot(req, world_res, state);
			ClosePairSeparationGradients(self_res, state, jmg, margin, close_pair_gradients, dists);
			ClosePairSeparationGradients(world_res, state, jmg, margin, close_pair_gradients, dists);
			for (const collision_detection::DistanceResult* res : {&self_res, &world_res})
				for (const auto& pair_entry : res->distances)
					for (const auto& d : pair_entry.second)
						min_pair_distance = std::min(min_pair_distance, d.distance);
		}
		// Every pair more than 1 mm apart means no contact; only near-touching poses need the full check.
		const bool collision_free =
			min_pair_distance > 1e-3 || IsStateCollisionFree(planning_scene_monitor, &state, jmg, q.data());
		if (gap < best.gap && collision_free)
		{
			best.found = true;
			best.pose_error = pose_error;
			best.gap = gap;
			best.joints = q;
			if (gap < 1e-6)
				break;
		}
		if (it == iters)
			break;
		min_gap_history.push_back(std::min(gap, min_gap_history.empty() ? gap : min_gap_history.back()));
		if (static_cast<int>(min_gap_history.size()) > stall_window &&
			min_gap_history[min_gap_history.size() - 1 - stall_window] - min_gap_history.back() < stall_gain)
			break;

		Eigen::MatrixXd J;
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += damping * damping;
		Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(pose_error);
		if (!close_pair_gradients.empty())
		{
			// Push pairs out to the margin first; the target step keeps only its null-space part.
			Eigen::MatrixXd Jc(static_cast<Eigen::Index>(close_pair_gradients.size()), dof);
			Eigen::VectorXd r(static_cast<Eigen::Index>(close_pair_gradients.size()));
			for (size_t k = 0; k < close_pair_gradients.size(); ++k)
			{
				Jc.row(static_cast<Eigen::Index>(k)) = close_pair_gradients[k];
				r(static_cast<Eigen::Index>(k)) = margin - dists[k];
			}
			Eigen::MatrixXd JcJct = Jc * Jc.transpose();
			JcJct.diagonal().array() += damping * damping;
			const Eigen::MatrixXd Jc_pinv =
				Jc.transpose() * JcJct.ldlt().solve(Eigen::MatrixXd::Identity(Jc.rows(), Jc.rows()));
			dq = Jc_pinv * r + (Eigen::MatrixXd::Identity(dof, dof) - Jc_pinv * Jc) * dq;
		}
		const double step_max = dq.cwiseAbs().maxCoeff();
		if (step_max > max_joint_step)
			dq *= max_joint_step / step_max;
		for (int k = 0; k < dof; ++k)
			q[static_cast<size_t>(k)] += dq(k);
		state.setJointGroupPositions(jmg, q);
		state.enforceBounds(jmg);
		state.copyJointGroupPositions(jmg, q);
	}
	return best;
}

// Floor on w before taking its log, so a singular configuration gives a large finite barrier.
constexpr double kMinManipulability = 1e-6;

double SumLogManipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<std::vector<double>>& joints)
{
	double total = 0.0;
	for (const auto& q : joints)
		if (!q.empty())
			total += std::log(std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, q)));
	return total;
}

double SumManipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<std::vector<double>>& joints)
{
	double total = 0.0;
	for (const auto& q : joints)
		if (!q.empty())
			total += Manipulability(state, jmg, tool0_link, q);
	return total;
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

// Sample IK candidates for each viewpoint. Sample based on previous solution if not the first run.
std::vector<std::vector<std::vector<double>>> CollectIkSolutions(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const ObjectPlacement& base, const BaseGradientParams& params,
	int max_solutions, random_numbers::RandomNumberGenerator* rng = nullptr)
{
	Eigen::Isometry3d xform = MakePlacement(base);
	SetObjectPose(planning_scene_monitor, xform * object_pose_obj);

	auto validity_callback = [&planning_scene_monitor](
								 moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								 const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	std::vector<std::vector<std::vector<double>>> ik_solutions(tour_poses_obj.size());
	const int max_sol = std::max(1, max_solutions);

	for (size_t i = 0; i < tour_poses_obj.size(); ++i)
	{
		geometry_msgs::msg::Pose target_local = ToPoseMsg(xform * tour_poses_obj[i]);
		std::vector<std::vector<double>>& sols = ik_solutions[i];

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
			// Zero solutions after the warm start + 4 random restarts: treat as out of reach. Every
			// miss costs a full ik_timeout, the dominant cost at offsets that drop viewpoints.
			if (sols.empty() && attempt >= 4)
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
				if (JointL2Distance(e, s) < 0.1)  // same IK solution: ~5.7 deg, matches real_cost_planning.cpp
				{
					dup = true;
					break;
				}
			if (!dup)
				sols.push_back(std::move(s));
		}
	}
	return ik_solutions;
}

// ---------------------------------------------------------------------------------------------
// Inner redundant-IK GTSP (compact reimplementation of the generalized NN + 2-opt / solution-swap
// scheme in hierarchical_tour.cpp. One node per (tour pose, IK branch); visit exactly one node per pose.
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
	// group -> branch -> cost of visiting that branch (-weight * manipulability); empty = none.
	std::vector<std::vector<double>> node_cost_by_group;

	const std::vector<double>& NodeJoints(const GtspNode& n) const { return *joints_by_group[n.group][n.branch]; }
	double NodeCost(const GtspNode& n) const
	{
		return node_cost_by_group.empty() ? 0.0 : node_cost_by_group[n.group][n.branch];
	}
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
				double cost = (have_current ? ctx.Edge(current, cand) : ctx.EdgeFromHome(cand)) + ctx.NodeCost(cand);
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
				double cost = edge_at(static_cast<int>(pos) - 1, cand) + ctx.NodeCost(cand);
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
	for (const GtspNode& n : order)
		total += ctx.NodeCost(n);
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
GtspSolution FindTourOrder(
	const std::vector<std::vector<std::vector<double>>>& ik_solutions, const ObjectPlacement& base,
	const std::vector<Eigen::Isometry3d>& tour_poses_obj, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params,
	const std::vector<int>* warm_order = nullptr, const std::vector<std::vector<double>>* node_cost = nullptr)
{
	Eigen::Isometry3d xform = MakePlacement(base);

	GtspContext ctx;
	ctx.home_joints = &home_joints;
	ctx.home_tcp_local = home_tcp_local;
	ctx.params = &params;
	std::vector<int> group_of_viewpoint(ik_solutions.size(), -1);
	for (size_t i = 0; i < ik_solutions.size(); ++i)
	{
		if (ik_solutions[i].empty())
			continue;
		group_of_viewpoint[i] = static_cast<int>(ctx.group_pose_index.size());
		ctx.group_pose_index.push_back(static_cast<int>(i));
		std::vector<const std::vector<double>*> js;
		for (const auto& b : ik_solutions[i])
			js.push_back(&b);
		ctx.joints_by_group.push_back(std::move(js));
		ctx.tcp_local_by_group.push_back((xform * tour_poses_obj[i]).translation());
		if (node_cost)
			ctx.node_cost_by_group.push_back((*node_cost)[i]);
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

// 1.0 × joint L2 distance  +  1.0 × largest single-joint move  +  0.0 × tool's straight-line distance
// straight-line distance doesn't help with pose optimziation
double TourWeightedCost(
	const ObjectPlacement& base, const std::vector<int>& tour, const std::vector<std::vector<double>>& joints,
	const std::vector<Eigen::Isometry3d>& tour_poses_obj, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params)
{
	Eigen::Isometry3d xform = MakePlacement(base);
	double total = 0.0;
	const std::vector<double>* prev_j = &home_joints;
	Eigen::Vector3d prev_p = home_tcp_local;
	for (size_t k = 0; k < tour.size(); ++k)
	{
		Eigen::Vector3d p = (xform * tour_poses_obj[tour[k]]).translation();
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
	double travel_cost = 0.0;		   // edge part of weighted_cost (home edge included)
	double sum_manipulability = 0.0;  // sum of w over visited viewpoints
	double sum_log_manipulability = 0.0;  // sum of log(w): the reach-margin barrier
	// Missed viewpoints and their closest-IK pose gaps; miss_cost = weight * sum of capped gaps.
	std::vector<int> missed_vp;
	std::vector<Eigen::Matrix<double, 6, 1>> missed_gap;
	std::vector<std::vector<double>> missed_q;  // parallel: closest collision-free pose, empty if none
	double miss_cost = 0.0;
	int num_rescued = 0;  // missed by IK, reached via a collision-free closest-IK hit
	int num_no_free = 0;  // closest IK found no collision-free posture at all (gap taken as the cap)
	int num_reachable = 0;
	int num_total = 0;
	bool all_reachable = false;
};

// max_solutions: IK branches to collect per pose. Pass 1 for a cheap probe (single warm-started
// IK per pose, GTSP degenerates to plain TSP); pass params.max_solutions_per_candidate to commit.
InnerSolution InnerSolve(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>* warm_order, int max_solutions)
{
	std::vector<std::vector<std::vector<double>>> ik_solutions = CollectIkSolutions(
		state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seed_per_viewpoint, base,
		params, max_solutions);
	const moveit::core::LinkModel* tool0_link = state.getRobotModel()->getLinkModel("tool0");

	// Viewpoints IK missed: collision-aware closest IK from the warm seed; random starts only if that finds
	// no collision-free pose under the cap, so the gap changes smoothly with the offset. A hit is added as
	// a branch (IK just missed it); otherwise the collision-free gap is kept, giving the miss a slope.
	const double exact_gap = 1e-4;	// 0.1 mm: tight enough to use the hit as a real IK solution
	std::vector<int> missed_vp;
	std::vector<Eigen::Matrix<double, 6, 1>> missed_gap;
	std::vector<std::vector<double>> missed_q;
	int num_rescued = 0, num_no_free = 0;
	const Eigen::Isometry3d xform = MakePlacement(base);
	for (size_t i = 0; i < ik_solutions.size(); ++i)
	{
		if (!ik_solutions[i].empty())
			continue;
		const Eigen::Isometry3d target = xform * tour_poses_obj[i];
		ClosestIkResult best;
		// Random starts only while no collision-free pose is under the cap (a capped gap has no slope).
		for (int t = 0; t < std::max(1, params.closest_ik_starts) && (!best.found || best.gap >= params.miss_gap_cap);
			 ++t)
		{
			std::vector<double> start_joints = seed_per_viewpoint[i];
			if (t > 0)
			{
				state.setToRandomPositions(jmg);
				state.copyJointGroupPositions(jmg, start_joints);
			}
			const ClosestIkResult r = CollisionAwareClosestIk(
				planning_scene_monitor, state, jmg, tool0_link, target, start_joints, params.closest_ik_iters,
				params.closest_ik_margin, params.rot_metric_scale);
			if (r.found && r.gap < best.gap)
			{
				best = r;
			}
		}
		if (best.found && best.gap < exact_gap)
		{
			ik_solutions[i].push_back(best.joints);
			++num_rescued;
			continue;
		}
		missed_vp.push_back(static_cast<int>(i));
		missed_q.push_back(best.joints);
		if (best.found)
			missed_gap.push_back(best.pose_error);
		else
		{
			++num_no_free;
			// No collision-free posture: a gap at the cap -- flat cost, no slope.
			Eigen::Matrix<double, 6, 1> capped = Eigen::Matrix<double, 6, 1>::Zero();
			capped(0) = params.miss_gap_cap;
			missed_gap.push_back(capped);
		}
	}

	std::vector<std::vector<double>> node_cost(ik_solutions.size());
	for (size_t i = 0; i < ik_solutions.size(); ++i)
		for (const auto& q : ik_solutions[i])
		{
			const double w = Manipulability(state, jmg, tool0_link, q);
			node_cost[i].push_back(
				-params.manipulability_weight * w -
				params.log_manipulability_weight * std::log(std::max(kMinManipulability, w)));
		}
	GtspSolution gtsp = FindTourOrder(
		ik_solutions, base, tour_poses_obj, home_joints, home_tcp_local, params, warm_order, &node_cost);

	InnerSolution out;
	out.tour = std::move(gtsp.tour_pose_indices);
	out.joints = std::move(gtsp.chosen_joints);
	out.num_total = static_cast<int>(tour_poses_obj.size());
	out.num_reachable = static_cast<int>(out.tour.size());
	out.all_reachable = (out.num_reachable == out.num_total);
	out.travel_cost =
		TourWeightedCost(base, out.tour, out.joints, tour_poses_obj, home_joints, home_tcp_local, params);
	out.sum_manipulability = SumManipulability(state, jmg, tool0_link, out.joints);
	out.sum_log_manipulability = SumLogManipulability(state, jmg, tool0_link, out.joints);
	out.num_rescued = num_rescued;
	out.num_no_free = num_no_free;
	out.missed_vp = std::move(missed_vp);
	out.missed_gap = std::move(missed_gap);
	out.missed_q = std::move(missed_q);
	for (const auto& e : out.missed_gap)
		out.miss_cost +=
			params.miss_gap_weight * std::min(params.miss_gap_cap, WeightedPoseGap(e, params.rot_metric_scale));

	out.weighted_cost = out.travel_cost - params.manipulability_weight * out.sum_manipulability -
		params.log_manipulability_weight * out.sum_log_manipulability +
		params.unreachable_penalty * (out.num_total - out.num_reachable) + out.miss_cost;
	return out;
}

// InnerSolve run `params.gtsp_num_restart` times (>=1), keeping the lowest-weighted_cost result.
// Consecutive runs in one process advance the shared RNG stream, so they explore different IK
// branch sets -- min-of-N shrinks the seed-driven cost variance at a fixed offset. Use wherever a
// committed cost matters; keep plain InnerSolve for cheap line-search probes.
InnerSolution BestOfNInnerSolve(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>* warm_order, int max_solutions)
{
	const int n = std::max(1, params.gtsp_num_restart);
	InnerSolution best = InnerSolve(
		state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seed_per_viewpoint,
		home_joints, home_tcp_local, base, params, warm_order, max_solutions);
	for (int i = 1; i < n; ++i)
	{
		InnerSolution cand = InnerSolve(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seed_per_viewpoint,
			home_joints, home_tcp_local, base, params, warm_order, max_solutions);
		if (cand.weighted_cost < best.weighted_cost)
			best = std::move(cand);
	}
	return best;
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
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params)
{
	Eigen::Isometry3d xform = MakePlacement(base);
	SetObjectPose(planning_scene_monitor, xform * object_pose_obj);

	auto validity_callback = [&planning_scene_monitor](
								 moveit::core::RobotState* s, const moveit::core::JointModelGroup* g,
								 const double* jp) { return IsStateCollisionFree(planning_scene_monitor, s, g, jp); };

	TrackResult r;
	r.joints.resize(tour.size());

	const moveit::core::LinkModel* tool0_link = state.getRobotModel()->getLinkModel("tool0");
	const std::vector<double>* prev_j = &home_joints;
	Eigen::Vector3d prev_p = home_tcp_local;
	bool prev_reachable = true;

	for (size_t k = 0; k < tour.size(); ++k)
	{
		geometry_msgs::msg::Pose target_local = ToPoseMsg(xform * tour_poses_obj[tour[k]]);
		Eigen::Vector3d p = (xform * tour_poses_obj[tour[k]]).translation();

		state.setJointGroupPositions(jmg, seeds[k]);
		bool ok = state.setFromIK(jmg, target_local, "tool0", params.ik_timeout, validity_callback);
		if (ok)
		{
			state.copyJointGroupPositions(jmg, r.joints[k]);
			r.num_reachable++;
			const double w = Manipulability(state, jmg, tool0_link, r.joints[k]);
			r.weighted_cost -= params.manipulability_weight * w +
				params.log_manipulability_weight * std::log(std::max(kMinManipulability, w));
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

// Jacobian of a viewpoint's pose w.r.t. the object placement: 6x5, rows [position; rotation],
// columns x, y, z, roll, pitch, in the base frame.
Eigen::Matrix<double, 6, 5> PlacementJacobian(
	const ObjectPlacement& base, const Eigen::Vector3d& viewpoint_position)
{
	const Eigen::Vector3d object_center(base.x, base.y, base.z);
	const Eigen::Vector3d roll_axis =
		Eigen::AngleAxisd(base.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::Vector3d::UnitX();
	const Eigen::Vector3d pitch_axis = Eigen::Vector3d::UnitY();
	const Eigen::Vector3d lever_arm = viewpoint_position - object_center;  // tilt swings the viewpoint around the center
	Eigen::Matrix<double, 6, 5> jacobian = Eigen::Matrix<double, 6, 5>::Zero();
	jacobian.block<3, 3>(0, 0).setIdentity();
	jacobian.block<3, 1>(0, 3) = roll_axis.cross(lever_arm);
	jacobian.block<3, 1>(3, 3) = roll_axis;
	jacobian.block<3, 1>(0, 4) = pitch_axis.cross(lever_arm);
	jacobian.block<3, 1>(3, 4) = pitch_axis;
	return jacobian;
}

// dw/db: how an object placement change moves one viewpoint's manipulability, the arm tracking it from q.
// Its positive side keeps the viewpoint away from the reach edge.
Eigen::Matrix<double, 5, 1> ManipulabilityOffsetGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const ObjectPlacement& base, const Eigen::Vector3d& target_pos,
	const std::vector<double>& q, double damping)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += damping * damping;
	const Eigen::MatrixXd dq_db =
		J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6)) * PlacementJacobian(base, target_pos);
	return dq_db.transpose() * ManipulabilityJointGradient(state, jmg, tool0_link, q);
}

// ---------------------------------------------------------------------------------------------
// Analytic gradient of the cost (travel, manipulability terms, missed-viewpoint gaps) w.r.t. the object
// placement (x, y, z, roll, pitch). Joints follow a moved viewpoint as dq/db = J_pinv * placement_jacobian
// (damped pseudo-inverse of the tool Jacobian); each edge, manipulability term and miss gap is chained
// through that. The home pose doesn't move with the object.
// ---------------------------------------------------------------------------------------------

Eigen::Matrix<double, 5, 1> AnalyticGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& joints, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>& missed_vp, const std::vector<Eigen::Matrix<double, 6, 1>>& missed_gap)
{
	const size_t n = tour.size();
	Eigen::Isometry3d xform = MakePlacement(base);
	const double lambda2 = params.jacobian_damping * params.jacobian_damping;

	// Per visit position: dq/db (dof x 5) and dp/db (3 x 5).
	std::vector<Eigen::MatrixXd> dq_db(n);
	std::vector<Eigen::MatrixXd> dp_db(n);

	for (size_t k = 0; k < n; ++k)
	{
		const Eigen::Matrix<double, 6, 5> placement_jacobian =
			PlacementJacobian(base, (xform * tour_poses_obj[tour[k]]).translation());

		state.setJointGroupPositions(jmg, joints[k]);
		state.update();
		Eigen::MatrixXd J;  // 6 x dof, [linear; angular] in the model (base) frame
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);

		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += lambda2;
		Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

		dq_db[k] = J_pinv * placement_jacobian;		   // dof x 5
		dp_db[k] = placement_jacobian.topRows<3>();	   // 3 x 5
	}

	const size_t dof = home_joints.size();
	Eigen::Matrix<double, 5, 1> g = Eigen::Matrix<double, 5, 1>::Zero();
	for (size_t k = 0; k < n; ++k)
	{
		// d(-lambda w - mu log w)/db = -(lambda + mu / w) (dq/db)^T dw/dq
		const double w = std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, joints[k]));
		g -= (params.manipulability_weight + params.log_manipulability_weight / w) * dq_db[k].transpose() *
			ManipulabilityJointGradient(state, jmg, tool0_link, joints[k]);
	}

	// Missed viewpoints: moving the object moves the viewpoint, which changes its pose error, so
	// d(gap)/db = placement_jacobian^T * weighted_error / gap. No slope once the gap is past the cap.
	for (size_t m = 0; m < missed_vp.size(); ++m)
	{
		const Eigen::Matrix<double, 6, 1>& pose_error = missed_gap[m];
		const double gap = WeightedPoseGap(pose_error, params.rot_metric_scale);
		if (gap < 1e-9 || gap >= params.miss_gap_cap)
			continue;
		Eigen::Matrix<double, 6, 1> weighted_error = pose_error;
		weighted_error.tail<3>() *= params.rot_metric_scale * params.rot_metric_scale;
		const Eigen::Vector3d viewpoint_position =
			(xform * tour_poses_obj[static_cast<size_t>(missed_vp[m])]).translation();
		g += params.miss_gap_weight * PlacementJacobian(base, viewpoint_position).transpose() *
			weighted_error / gap;
	}

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
		Eigen::Vector3d p0 = (xform * tour_poses_obj[tour[0]]).translation();
		add_edge(home_joints, joints[0], zero_dq, dq_db[0], zero_dp, dp_db[0], home_tcp_local, p0);
	}
	for (size_t k = 1; k < n; ++k)
	{
		Eigen::Vector3d pa = (xform * tour_poses_obj[tour[k - 1]]).translation();
		Eigen::Vector3d pb = (xform * tour_poses_obj[tour[k]]).translation();
		add_edge(joints[k - 1], joints[k], dq_db[k - 1], dq_db[k], dp_db[k - 1], dp_db[k], pa, pb);
	}
	return g;
}

// Central-difference gradient of the objective via re-tracked IK -- cross-check only.
Eigen::Matrix<double, 5, 1> FiniteDifferenceGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params)
{
	const double eps = params.fd_epsilon;
	Eigen::Matrix<double, 5, 1> g = Eigen::Matrix<double, 5, 1>::Constant(std::nan(""));
	for (int axis = 0; axis < 5; ++axis)
	{
		ObjectPlacement bp = base, bm = base;
		auto component = [](ObjectPlacement& o, int a) -> double& {
			return a == 0 ? o.x : a == 1 ? o.y : a == 2 ? o.z : a == 3 ? o.roll : o.pitch;
		};
		double* pp = &component(bp, axis);
		double* pm = &component(bm, axis);
		*pp += eps;
		*pm -= eps;
		TrackResult rp = TrackTour(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, tour, seeds, home_joints,
			home_tcp_local, bp, params);
		TrackResult rm = TrackTour(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, tour, seeds, home_joints,
			home_tcp_local, bm, params);
		if (rp.all_reachable && rm.all_reachable)
			g(axis) = (rp.weighted_cost - rm.weighted_cost) / (2.0 * eps);
	}
	return g;
}

// ---------------------------------------------------------------------------------------------
// Live progress markers
// ---------------------------------------------------------------------------------------------

// Object during the descent, in actual (abs, base-frame) position: mesh, trail of past positions,
// -grad(D) arrow, and viewpoints (green reached, red missed). Fixed id per namespace so each
// publish overwrites the last instead of stacking.
void PublishProgress(
	const rclcpp::Node::SharedPtr& node, const BaseGradientParams& params, const Eigen::Isometry3d& object_pose_obj,
	const std::vector<Eigen::Isometry3d>& tour_poses_obj, const std::vector<ObjectPlacement>& base_history,
	const Eigen::Vector3d& neg_grad_translation, const ObjectPlacement& base, const std::vector<int>& missed_vp)
{
	if (!params.progress_pub)
		return;

	const rclcpp::Time stamp = node->now();
	visualization_msgs::msg::MarkerArray markers;
	auto make = [&](const char* ns, int type) {
		visualization_msgs::msg::Marker m;
		m.header.frame_id = "world";
		m.header.stamp = stamp;
		m.ns = ns;
		m.id = 0;
		m.type = type;
		m.action = visualization_msgs::msg::Marker::ADD;
		m.pose.orientation.w = 1.0;
		return m;
	};
	const Eigen::Isometry3d xform = MakePlacement(base);
	const Eigen::Vector3d obj_pos = (xform * object_pose_obj).translation();

	if (!params.progress_mesh_path.empty())
	{
		visualization_msgs::msg::Marker mesh = make("base_gradient_current", visualization_msgs::msg::Marker::MESH_RESOURCE);
		mesh.mesh_resource = "file://" + params.progress_mesh_path;
		mesh.mesh_use_embedded_materials = false;
		mesh.pose = ToPoseMsg(xform * object_pose_obj);
		mesh.scale.x = mesh.scale.y = mesh.scale.z = params.progress_mesh_scale;
		mesh.color.r = 1.0f;
		mesh.color.g = 0.85f;
		mesh.color.a = 0.6f;
		markers.markers.push_back(mesh);
	}

	// Viewpoints at this offset: green reached, red missed.
	visualization_msgs::msg::Marker vps = make("base_gradient_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST);
	vps.scale.x = vps.scale.y = vps.scale.z = 0.01;
	for (size_t i = 0; i < tour_poses_obj.size(); ++i)
	{
		const Eigen::Vector3d p = (xform * tour_poses_obj[i]).translation();
		geometry_msgs::msg::Point pt;
		pt.x = p.x();
		pt.y = p.y();
		pt.z = p.z();
		const bool missed = std::find(missed_vp.begin(), missed_vp.end(), static_cast<int>(i)) != missed_vp.end();
		std_msgs::msg::ColorRGBA c;
		c.r = missed ? 0.9f : 0.1f;
		c.g = missed ? 0.1f : 0.9f;
		c.b = 0.1f;
		c.a = 1.0f;
		vps.points.push_back(pt);
		vps.colors.push_back(c);
	}
	markers.markers.push_back(vps);

	// Trail of past object positions (abs). LINE_STRIP needs >= 2 points.
	if (base_history.size() > 1)
	{
		visualization_msgs::msg::Marker trail = make("base_gradient_trail", visualization_msgs::msg::Marker::LINE_STRIP);
		trail.scale.x = 0.004;
		trail.color.r = 1.0f;
		trail.color.g = 0.4f;
		trail.color.b = 0.1f;
		trail.color.a = 0.8f;
		visualization_msgs::msg::Marker crumbs = make("base_gradient_history", visualization_msgs::msg::Marker::SPHERE_LIST);
		crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.012;
		crumbs.color = trail.color;
		for (const ObjectPlacement& b : base_history)
		{
			const Eigen::Vector3d p = (MakePlacement(b) * object_pose_obj).translation();
			geometry_msgs::msg::Point pt;
			pt.x = p.x();
			pt.y = p.y();
			pt.z = p.z();
			trail.points.push_back(pt);
			crumbs.points.push_back(pt);
		}
		markers.markers.push_back(trail);
		markers.markers.push_back(crumbs);
	}

	// -grad(D) translation from the object's position; a translation offset moves the object 1:1.
	const double gnorm = neg_grad_translation.norm();
	if (gnorm > 1e-9)
	{
		visualization_msgs::msg::Marker arrow = make("base_gradient_descent_dir", visualization_msgs::msg::Marker::ARROW);
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.color.r = 0.1f;
		arrow.color.g = 0.5f;
		arrow.color.b = 1.0f;
		arrow.color.a = 0.95f;
		const Eigen::Vector3d tip = obj_pos + neg_grad_translation * (0.15 / gnorm);  // fixed on-screen length
		geometry_msgs::msg::Point a, b;
		a.x = obj_pos.x();
		a.y = obj_pos.y();
		a.z = obj_pos.z();
		b.x = tip.x();
		b.y = tip.y();
		b.z = tip.z();
		arrow.points.push_back(a);
		arrow.points.push_back(b);
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
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const std::vector<double>& start_reference_joints,
	const BaseGradientParams& params_in)
{
	// Mutable copy: each descent anneals manipulability_weight from its initial value down to
	// params_in.manipulability_weight; results are compared at that final weight.
	BaseGradientParams params = params_in;
	const double lambda_final = params_in.manipulability_weight;
	// Object and viewpoints relative to the object's position; MakePlacement puts them at an absolute one.
	const Eigen::Isometry3d object_pose_obj = MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal);
	std::vector<Eigen::Isometry3d> tour_poses_obj = tour_tcp_poses_nominal;
	for (Eigen::Isometry3d& t : tour_poses_obj)
		t.translation() -= object_translation_nominal;
	const int n = static_cast<int>(tour_poses_obj.size());

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
		ObjectPlacement placement;
		InnerSolution sol;
		double cost = std::numeric_limits<double>::max();
		bool ok = false;  // fully reachable
	};

	// One full gradient descent from `base`, warm-started internally but starting IK/GTSP cold.
	// warm: IK seeds from an earlier solution (restarts), or null to start from the home config.
	// Every descent starts at the initial weight; it decays only while all viewpoints are reached.
	auto run_descent = [&](ObjectPlacement base, int restart_idx, const InnerSolution* warm) -> RestartResult {
		base = ProjectToBounds(base, params.bounds);
		params.manipulability_weight = std::max(lambda_final, params_in.manipulability_weight_initial);
		std::vector<ObjectPlacement> base_history{base};
		std::vector<std::vector<double>> seeds =
			warm ? SeedsFromSolution(*warm, fallback_seed, static_cast<size_t>(n))
				 : std::vector<std::vector<double>>(static_cast<size_t>(n), fallback_seed);
		int stall_count = 0;
		// Each viewpoint's joints from the last time it was reached: seeds its closest-IK gap once missed.
		std::vector<std::vector<double>> last_good(static_cast<size_t>(n));
		// Each missed viewpoint's last closest-IK pose: seeds the next closest IK so the gap stays continuous.
		std::vector<std::vector<double>> last_closest(static_cast<size_t>(n));
		auto remember = [&](const InnerSolution& s) {
			for (size_t k = 0; k < s.tour.size(); ++k)
			{
				last_good[static_cast<size_t>(s.tour[k])] = s.joints[k];
				last_closest[static_cast<size_t>(s.tour[k])].clear();
			}
			for (size_t m = 0; m < s.missed_vp.size(); ++m)
				if (!s.missed_q[m].empty())
					last_closest[static_cast<size_t>(s.missed_vp[m])] = s.missed_q[m];
		};

		RestartResult rr;
		rr.placement = base;

		auto record = [&](const ObjectPlacement& b, const InnerSolution& s) {
			// Re-score at the final weight so points found during annealing compare fairly.
			InnerSolution sf = s;
			sf.weighted_cost = s.travel_cost - lambda_final * s.sum_manipulability -
				params.log_manipulability_weight * s.sum_log_manipulability +
				params.unreachable_penalty * (s.num_total - s.num_reachable) + s.miss_cost;
			result.history.push_back(
				{static_cast<double>(restart_idx), b.x, b.y, b.z, b.roll, b.pitch, sf.weighted_cost});
			// weighted_cost already includes the unreachable penalty, so the lowest-cost solution
			// is also the one with the best reachability -- no separate all_reachable gate needed.
			if (sf.weighted_cost < rr.cost)
			{
				rr.placement = b;
				rr.sol = sf;
				rr.cost = sf.weighted_cost;
				rr.ok = sf.all_reachable;
			}
		};

		InnerSolution cur = BestOfNInnerSolve(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
			start_reference_joints, home_tcp_local, base, params, warm ? &warm->tour : nullptr,
			params.max_solutions_per_candidate);
		result.num_inner_solves += std::max(1, params.gtsp_num_restart);

		if (cur.tour.empty())
		{
			RCLCPP_WARN(node->get_logger(), "restart %d: no tour pose reachable at the start placement", restart_idx + 1);
			return rr;
		}

		record(base, cur);
		remember(cur);
		PublishProgress(
			node, params, object_pose_obj, tour_poses_obj, base_history, Eigen::Vector3d::Zero(), base,
			cur.missed_vp);
		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d iter 0: abs (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f deg  "
			"D=%.4f  lambda=%.1f  travel+miss=%.2f  sum_w=%.3f  sum_logw=%.2f  miss_gap=%.2f  reachable %d/%d  "
			"(rescued %d, no collision-free posture %d)",
			restart_idx + 1, std::max(1, params.descent_num_restart), base.x, base.y, base.z, base.roll * 180.0 / M_PI,
			base.pitch * 180.0 / M_PI, cur.weighted_cost, params.manipulability_weight,
			cur.travel_cost + params.unreachable_penalty * (n - cur.num_reachable), cur.sum_manipulability,
			cur.sum_log_manipulability, cur.miss_cost, cur.num_reachable, n, cur.num_rescued,
			cur.num_no_free);

		for (int outer = 0; outer < params.max_outer_iterations && rclcpp::ok() && !cur.tour.empty(); ++outer)
		{
			if (outer > 0)
			{
				// Hold the weight high (reach recovery) while any viewpoint is missed; decay it otherwise.
				if (!cur.all_reachable)
					params.manipulability_weight = std::max(lambda_final, params_in.manipulability_weight_initial);
				else
					params.manipulability_weight = std::max(
						lambda_final, params.manipulability_weight * params_in.manipulability_weight_decay);
				// Snap to the final weight once within 5% of the starting gap, so a final weight of 0 ends.
				if (params.manipulability_weight - lambda_final <
					0.05 * (params_in.manipulability_weight_initial - lambda_final))
					params.manipulability_weight = lambda_final;
				cur.weighted_cost = cur.travel_cost - params.manipulability_weight * cur.sum_manipulability -
					params.log_manipulability_weight * cur.sum_log_manipulability +
					params.unreachable_penalty * (cur.num_total - cur.num_reachable) + cur.miss_cost;
			}
			// A stop rule hit while still annealing jumps the weight to its final value instead of stopping,
			// so the descent ends at the final objective without waiting out the decay.
			const bool annealing = params.manipulability_weight > lambda_final * (1.0 + 1e-6);

			Eigen::Matrix<double, 5, 1> g = AnalyticGradient(
				state, jmg, tool0_link, tour_poses_obj, cur.tour, cur.joints, start_reference_joints,
				home_tcp_local, base, params, cur.missed_vp, cur.missed_gap);

			if (params.fd_gradient_check)
			{
				Eigen::Matrix<double, 5, 1> g_fd = FiniteDifferenceGradient(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, cur.tour,
					cur.joints, start_reference_joints, home_tcp_local, base, params);
				RCLCPP_INFO(
					node->get_logger(),
					"    grad check [x y z roll pitch]  analytic (%+.4f %+.4f %+.4f %+.4f %+.4f)  "
					"central-diff (%+.4f %+.4f %+.4f %+.4f %+.4f)",
					g(0), g(1), g(2), g(3), g(4), g_fd(0), g_fd(1), g_fd(2), g_fd(3), g_fd(4));
			}

			// Descend in a metric where 1 rad of tip/tilt equals rot_scale meters. Locked axes (min == max)
			// get no share of the step.
			const BaseGradientBounds& bb = params.bounds;
			auto to_u = [&](Eigen::Matrix<double, 5, 1> v) {
				v(3) /= rot_scale;
				v(4) /= rot_scale;
				if (bb.z_min == bb.z_max)
					v(2) = 0.0;
				if (bb.roll_min == bb.roll_max)
					v(3) = 0.0;
				if (bb.pitch_min == bb.pitch_max)
					v(4) = 0.0;
				return v;
			};
			const Eigen::Matrix<double, 5, 1> g_u = to_u(g);
			double gnorm = g_u.norm();
			if (gnorm < 1e-6)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: gradient ~ 0 -- converged", restart_idx + 1);
				PublishProgress(
					node, params, object_pose_obj, tour_poses_obj, base_history,
					Eigen::Vector3d(-g.head<3>()), base, cur.missed_vp);
				break;
			}

			// Backtracking line search. Probes use the quick single-IK solve and are compared with the
			// same quick solve at the current point, so the comparison is fair; only the accepted
			// offset gets a full solve, and it is kept only if it beats the current full solution.
			Eigen::Matrix<double, 5, 1> dir_u = -g_u / gnorm;
			seeds = SeedsFromSolution(cur, fallback_seed, static_cast<size_t>(n));
			for (int v : cur.missed_vp)
			{
				const size_t vi = static_cast<size_t>(v);
				if (!last_closest[vi].empty())
					seeds[vi] = last_closest[vi];
				else if (!last_good[vi].empty())
					seeds[vi] = last_good[vi];
			}
			const InnerSolution here_quick = InnerSolve(
				state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
				start_reference_joints, home_tcp_local, base, params, &cur.tour, 1);
			result.num_inner_solves += 1;
			double step = params.initial_step;
			bool accepted = false;
			ObjectPlacement b_new = base;
			InnerSolution next;
			// Steering: when a probe loses a reached viewpoint, remove the part of the direction that lowers
			// its manipulability (pushes it to the reach edge) and retry the same step.
			std::vector<Eigen::Matrix<double, 5, 1>> blocked;  // orthonormal, step metric
			int num_steers = 0;
			const int kMaxSteers = 3;
			for (int ls = 0; ls < params.max_line_search_iters;)
			{
				ObjectPlacement cand = ProjectToBounds(
					{base.x + step * dir_u(0), base.y + step * dir_u(1), base.z + step * dir_u(2),
					 base.roll + step * dir_u(3) / rot_scale, base.pitch + step * dir_u(4) / rot_scale},
					params.bounds);
				// weighted_cost carries the unreachable penalty, so a probe that drops a viewpoint fails.
				InnerSolution probe = InnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
					start_reference_joints, home_tcp_local, cand, params, &cur.tour, 1);
				result.num_inner_solves += 1;
				// Never trade away a reached viewpoint, whatever the cost says.
				if (probe.num_reachable >= cur.num_reachable &&
					probe.weighted_cost < here_quick.weighted_cost)
				{
					accepted = true;
					b_new = cand;
					next = std::move(probe);
					break;
				}
				if (probe.num_reachable < cur.num_reachable && num_steers < kMaxSteers)
				{
					bool steered = false;
					for (int v : probe.missed_vp)
					{
						const auto it = std::find(cur.tour.begin(), cur.tour.end(), v);
						if (it == cur.tour.end())
							continue;  // already missed at the current offset
						const Eigen::Vector3d p =
							(MakePlacement(base) * tour_poses_obj[static_cast<size_t>(v)]).translation();
						Eigen::Matrix<double, 5, 1> h = to_u(ManipulabilityOffsetGradient(
							state, jmg, tool0_link, base, p, cur.joints[static_cast<size_t>(it - cur.tour.begin())],
							params.jacobian_damping));
						if (dir_u.dot(h) >= 0.0)
							continue;  // the step doesn't lower its manipulability: lost for another reason
						for (const Eigen::Matrix<double, 5, 1>& e : blocked)
							h -= h.dot(e) * e;
						if (h.norm() < 1e-9)
							continue;
						blocked.push_back(h.normalized());
						steered = true;
					}
					if (steered)
					{
						++num_steers;
						Eigen::Matrix<double, 5, 1> d = -g_u / gnorm;
						for (const Eigen::Matrix<double, 5, 1>& e : blocked)
							d -= d.dot(e) * e;
						if (d.norm() < 0.1)
							break;  // under 10% of the descent direction keeps every viewpoint: stuck
						dir_u = d.normalized();
						continue;
					}
				}
				step *= params.step_shrink;
				++ls;
			}

			InnerSolution committed;
			if (accepted)
			{
				committed = BestOfNInnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
					start_reference_joints, home_tcp_local, b_new, params, &next.tour,
					params.max_solutions_per_candidate);
				result.num_inner_solves += std::max(1, params.gtsp_num_restart);
				if (next.weighted_cost < committed.weighted_cost)
					committed = std::move(next);
				if (committed.weighted_cost > cur.weighted_cost || committed.num_reachable < cur.num_reachable)
					accepted = false;  // quick probe looked better but the full solve isn't
			}

			// While viewpoints are missed the weight is held, so retrying the same offset changes nothing.
			if (!accepted && annealing && cur.all_reachable)
			{
				params.manipulability_weight = lambda_final;
				continue;
			}
			if (!accepted)
			{
				if (cur.all_reachable)
					RCLCPP_INFO(node->get_logger(), "  restart %d: no step reduces the tour cost -- converged", restart_idx + 1);
				else
					RCLCPP_WARN(
						node->get_logger(), "  restart %d: no step reduces the tour cost with %d/%d reachable -- stuck",
						restart_idx + 1, cur.num_reachable, n);
				PublishProgress(
					node, params, object_pose_obj, tour_poses_obj, base_history,
					Eigen::Vector3d(-g.head<3>()), base, cur.missed_vp);
				break;
			}

			Eigen::Matrix<double, 5, 1> du = ToOffsetVec(b_new, base);
			du(3) *= rot_scale;
			du(4) *= rot_scale;
			double base_move = du.norm();
			double rel_impr = (cur.weighted_cost - committed.weighted_cost) / std::max(std::abs(cur.weighted_cost), 1e-9);

			base = b_new;
			cur = std::move(committed);
			remember(cur);
			base_history.push_back(base);
			record(base, cur);

			RCLCPP_INFO(
				node->get_logger(),
				"restart %d/%d iter %d/%d: abs (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f deg  "
				"D=%.4f  lambda=%.1f  travel+miss=%.2f  sum_w=%.3f  sum_logw=%.2f  miss_gap=%.2f  reachable %d/%d  "
				"(rescued %d, no collision-free posture %d)  |grad|=%.4f  step=%.4f",
				restart_idx + 1, std::max(1, params.descent_num_restart), outer + 1, params.max_outer_iterations, base.x,
				base.y, base.z, base.roll * 180.0 / M_PI,
				base.pitch * 180.0 / M_PI, cur.weighted_cost, params.manipulability_weight,
				cur.travel_cost + params.unreachable_penalty * (n - cur.num_reachable), cur.sum_manipulability,
				cur.sum_log_manipulability, cur.miss_cost, cur.num_reachable, n, cur.num_rescued,
				cur.num_no_free, gnorm, step);

			PublishProgress(
				node, params, object_pose_obj, tour_poses_obj, base_history,
				Eigen::Vector3d(-g.head<3>()), base, cur.missed_vp);

			if (rel_impr < params.convergence_tolerance_cost)
			{
				if (++stall_count >= std::max(1, params.patience))
				{
					if (annealing)
					{
						params.manipulability_weight = lambda_final;
						stall_count = 0;
						continue;
					}
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
				if (annealing)
				{
					params.manipulability_weight = lambda_final;
					stall_count = 0;
					continue;
				}
				RCLCPP_INFO(node->get_logger(), "  restart %d: placement settled -- converged", restart_idx + 1);
				break;
			}
		}

		return rr;  // record() runs at least once, so rr holds this restart's best-cost solution
	};

	std::mt19937 rng(static_cast<unsigned int>(params.random_seed));
	const int descent_num_restart = std::max(1, params.descent_num_restart);
	RestartResult overall;

	for (int r = 0; r < descent_num_restart && rclcpp::ok(); ++r)
	{
		ObjectPlacement start =
			(r == 0) ? ObjectPlacement{
						   std::isnan(params.initial_x) ? object_translation_nominal.x() : params.initial_x,
						   std::isnan(params.initial_y) ? object_translation_nominal.y() : params.initial_y,
						   std::isnan(params.initial_z) ? object_translation_nominal.z() : params.initial_z,
						   params.initial_roll, params.initial_pitch}
					 : PerturbOffset(overall.placement, rng, params.descent_restart_perturbation, rot_scale);
		RestartResult rr = run_descent(start, r, r == 0 ? nullptr : &overall.sol);

		// weighted_cost carries the unreachable penalty, so lower cost == better (a fully-
		// reachable result always beats a partial one).
		bool improved = (r == 0) || (rr.cost < overall.cost);

		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d done: D=%.4f  travel+miss=%.2f  sum_w=%.3f  reachable %d/%d  abs (%.4f, %.4f, %.4f) m"
			"  tip %.1f tilt %.1f deg%s",
			r + 1, descent_num_restart, rr.cost,
			rr.sol.travel_cost + params.unreachable_penalty * (n - rr.sol.num_reachable), rr.sol.sum_manipulability,
			rr.sol.num_reachable, n,
			rr.placement.x, rr.placement.y, rr.placement.z, rr.placement.roll * 180.0 / M_PI, rr.placement.pitch * 180.0 / M_PI, (r > 0 && improved) ? "  <-- new best" : "");

		if (improved)
			overall = rr;
	}

	const InnerSolution& fin = overall.sol;
	result.x = overall.placement.x;
	result.y = overall.placement.y;
	result.z = overall.placement.z;
	result.roll = overall.placement.roll;
	result.pitch = overall.placement.pitch;
	result.tour_order = fin.tour;
	result.joint_solutions = fin.joints;
	// Report the honest tour cost -- strip the unreachable penalty baked in for comparison.
	result.total_weighted_cost =
		fin.weighted_cost - params.unreachable_penalty * (fin.num_total - fin.num_reachable);
	result.total_joint_path_length = joint_path_len(fin);
	result.num_reachable = fin.num_reachable;
	result.ok = overall.ok;

	// Leave the scene as we found it.
	SetObjectPose(planning_scene_monitor, MakeIsometry(object_translation_nominal, object_rotation_nominal));

	if (result.ok)
		RCLCPP_INFO(
			node->get_logger(),
			"Done. Object abs (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg -- reaches "
			"all %d poses, tour joint path %.4f rad, travel %.2f, sum_w %.3f (weighted cost %.4f).",
			result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI, n,
			result.total_joint_path_length, fin.travel_cost, fin.sum_manipulability, result.total_weighted_cost);
	else
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Object abs (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg -- reaches "
			"only %d/%d poses.",
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

Eigen::Isometry3d PlacementTransform(
	const Eigen::Vector3d& object_translation_nominal, double x, double y, double z, double roll, double pitch)
{
	return MakePlacement(ObjectPlacement{x, y, z, roll, pitch}) * Eigen::Translation3d(-object_translation_nominal);
}

void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal, double x, double y, double z, double roll, double pitch)
{
	SetObjectPose(
		planning_scene_monitor,
		MakePlacement(ObjectPlacement{x, y, z, roll, pitch}) * MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal));
}

visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const BaseGradientResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	const Eigen::Isometry3d xform = PlacementTransform(
		object_translation_nominal, result.x, result.y, result.z, result.roll, result.pitch);
	const Eigen::Isometry3d object_pose_nominal = MakeIsometry(object_translation_nominal, object_rotation_nominal);

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "base_gradient_object";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose = ToPoseMsg(xform * object_pose_nominal);
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
		Eigen::Isometry3d local = xform * tour_tcp_poses_nominal[result.tour_order[k]];

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
