#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <thread>
#include <numeric>
#include <optional>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <json/json.h>
#include <Eigen/Dense>
#include <random_numbers/random_numbers.h>

#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/collision_detection/collision_common.h>

#include "panda_arm_control/base_gradient.hpp"
#include "panda_arm_control/reachability_map.hpp"
#include "panda_arm_control/real_cost_planning.hpp"

namespace
{

using Joints = std::vector<double>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Position = Eigen::Vector3d;  // object position, abs base frame; its orientation stays nominal
using ScenePtr = planning_scene_monitor::PlanningSceneMonitorPtr;

constexpr double kInf = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------------------------------------
// Geometry and scene helpers
// ---------------------------------------------------------------------------------------------

Eigen::Isometry3d Translation(const Position& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = p;
	return t;
}

Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
	Eigen::Isometry3d t = Translation(translation);
	t.linear() = rotation;
	return t;
}

geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Isometry3d& t)
{
	geometry_msgs::msg::Pose p;
	p.position.x = t.translation().x();
	p.position.y = t.translation().y();
	p.position.z = t.translation().z();
	const Eigen::Quaterniond q(t.rotation());
	p.orientation.x = q.x();
	p.orientation.y = q.y();
	p.orientation.z = q.z();
	p.orientation.w = q.w();
	return p;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v)
{
	geometry_msgs::msg::Point p;
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	return p;
}

bool IsStateValid(
	const ScenePtr& scene, moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* q)
{
	state->setJointGroupPositions(group, q);
	state->update();
	planning_scene_monitor::LockedPlanningSceneRO locked(scene);
	return locked->isStateValid(*state, group->getName());
}

// MOVE only updates the registered object's pose: cheap, no mesh reload.
void MoveObject(const ScenePtr& scene, const Eigen::Isometry3d& pose)
{
	moveit_msgs::msg::CollisionObject obj;
	obj.header.frame_id = "panda_link0";
	obj.id = "object";
	obj.pose = ToPoseMsg(pose);
	obj.operation = obj.MOVE;
	planning_scene_monitor::LockedPlanningSceneRW locked(scene);
	locked->processCollisionObjectMsg(obj);
}

Position Clamp(Position p, const BaseGradientBounds& b)
{
	p.x() = std::clamp(p.x(), b.x_min, b.x_max);
	p.y() = std::clamp(p.y(), b.y_min, b.y_max);
	p.z() = std::clamp(p.z(), b.z_min, b.z_max);
	return p;
}

// [position; axis * angle] error from `tool` to `target`.
Vector6d PoseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& tool)
{
	const Eigen::AngleAxisd rot(target.linear() * tool.linear().transpose());
	Vector6d e;
	e.head<3>() = target.translation() - tool.translation();
	e.tail<3>() = rot.axis() * rot.angle();
	return e;
}

// Pose gap in meters, rotation weighed by rot_scale (m per rad).
double PoseGap(const Vector6d& e, double rot_scale)
{
	return std::sqrt(e.head<3>().squaredNorm() + rot_scale * rot_scale * e.tail<3>().squaredNorm());
}

// J^T (J J^T + damping^2 I)^-1
Eigen::MatrixXd DampedPinv(const Eigen::MatrixXd& J, double damping)
{
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += damping * damping;
	return J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(J.rows(), J.rows()));
}

// ---------------------------------------------------------------------------------------------
// Joint travel. Joint 7 (the flange spin about the camera axis) doesn't matter for a viewpoint: not counted.
// ---------------------------------------------------------------------------------------------

size_t CountedJoints(const Joints& a, const Joints& b)
{
	const size_t n = std::min(a.size(), b.size());
	return n > 0 ? n - 1 : 0;
}

double JointDistance(const Joints& a, const Joints& b)
{
	double d = 0.0;
	for (size_t k = 0; k < CountedJoints(a, b); ++k)
		d += (a[k] - b[k]) * (a[k] - b[k]);
	return std::sqrt(d);
}

double MaxJointTurn(const Joints& a, const Joints& b)
{
	double m = 0.0;
	for (size_t k = 0; k < CountedJoints(a, b); ++k)
		m = std::max(m, std::abs(a[k] - b[k]));
	return m;
}

// Sum over joints of (turn past thr)^2: zero unless the move is a reconfiguration.
double ReconfigPenalty(const Joints& a, const Joints& b, double thr)
{
	double p = 0.0;
	for (size_t k = 0; k < CountedJoints(a, b); ++k)
	{
		const double excess = std::abs(a[k] - b[k]) - thr;
		if (excess > 0.0)
			p += excess * excess;
	}
	return p;
}

int CountReconfigs(const Joints& home, const std::vector<Joints>& tour, double thr)
{
	int count = 0;
	for (size_t k = 0; k < tour.size(); ++k)
		count += MaxJointTurn(k == 0 ? home : tour[k - 1], tour[k]) > thr ? 1 : 0;
	return count;
}

// One tour edge's cost, shared by the GTSP and the descent.
double EdgeCost(const Joints& a, const Joints& b, const BaseGradientParams& p)
{
	return p.joint_distance_weight * JointDistance(a, b) + p.max_joint_deviation_weight * MaxJointTurn(a, b) +
		(p.edge_reconfig_weight > 0.0 ? p.edge_reconfig_weight * ReconfigPenalty(a, b, p.reconfig_threshold) : 0.0);
}

double TravelCost(const Joints& home, const std::vector<Joints>& tour, const BaseGradientParams& p)
{
	double total = 0.0;
	for (size_t k = 0; k < tour.size(); ++k)
		total += EdgeCost(k == 0 ? home : tour[k - 1], tour[k], p);
	return total;
}

// ---------------------------------------------------------------------------------------------
// Manipulability
// ---------------------------------------------------------------------------------------------

// Per-joint room score s = 1 - exp(-k (q - lo)(hi - q) / range^2) (~1 mid-range, 0 at a limit) and ds/dq.
// k <= 0 or an unbounded joint: s = 1.
struct JointRoom
{
	Eigen::VectorXd score;
	Eigen::VectorXd slope;
};

JointRoom JointRoomScore(
	const moveit::core::JointModelGroup* group, const moveit::core::RobotModelConstPtr& model, const Joints& q, double k)
{
	const int n = static_cast<int>(q.size());
	JointRoom room{Eigen::VectorXd::Ones(n), Eigen::VectorXd::Zero(n)};
	if (k <= 0.0)
		return room;
	for (int i = 0; i < n; ++i)
	{
		const moveit::core::VariableBounds& b = model->getVariableBounds(group->getVariableNames()[static_cast<size_t>(i)]);
		const double range = b.max_position_ - b.min_position_;
		if (!b.position_bounded_ || range <= 0.0)
			continue;
		const double qi = q[static_cast<size_t>(i)];
		const double u = std::max(0.0, (qi - b.min_position_) * (b.max_position_ - qi)) / (range * range);
		const double e = std::exp(-k * u);
		room.score(i) = 1.0 - e;
		if (u > 0.0)
			room.slope(i) = k * e * (b.min_position_ + b.max_position_ - 2.0 * qi) / (range * range);
	}
	return room;
}

Eigen::MatrixXd ToolJacobian(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const Joints& q)
{
	state.setJointGroupPositions(group, q);
	state.update();
	Eigen::MatrixXd J;  // 6 x dof, [linear; angular] in the base frame
	state.getJacobian(group, tool, Eigen::Vector3d::Zero(), J);
	return J;
}

// w = sqrt(det(Js Js^T)), Js = J with each joint's column scaled by its room score (limit_sharpness 0 = plain).
double Manipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const Joints& q, double limit_sharpness)
{
	const Eigen::MatrixXd J = ToolJacobian(state, group, tool, q);
	const Eigen::MatrixXd Js = J * JointRoomScore(group, state.getRobotModel(), q, limit_sharpness).score.asDiagonal();
	return std::sqrt(std::max(0.0, (Js * Js.transpose()).determinant()));
}

// dw/dq_i = w tr((Js Js^T)^-1 Js dJs_i^T). For revolute joints column j of dJ/dq_i is
// [z_min(i,j) x Jv_max(i,j); i < j ? z_i x z_j : 0], built from J itself.
Eigen::VectorXd ManipulabilityGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const Joints& q, double limit_sharpness)
{
	const Eigen::MatrixXd J = ToolJacobian(state, group, tool, q);
	const int n = static_cast<int>(J.cols());
	const JointRoom room = JointRoomScore(group, state.getRobotModel(), q, limit_sharpness);
	const Eigen::MatrixXd Js = J * room.score.asDiagonal();
	const Eigen::MatrixXd A = Js * Js.transpose();
	const double w = std::sqrt(std::max(0.0, A.determinant()));
	Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
	if (w < 1e-12)
		return g;
	const Eigen::MatrixXd M = A.ldlt().solve(Js);
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
		g(i) = w * (M.cwiseProduct(dJ * room.score.asDiagonal()).sum() + room.slope(i) * M.col(i).dot(J.col(i)));
	}
	return g;
}

// -tau log(sum exp(-w / tau)), ~ the smallest w; shares (optional) gets each w's slope, adding up to 1.
double SoftMin(const std::vector<double>& w, double tau, std::vector<double>* shares = nullptr)
{
	if (w.empty())
		return 0.0;
	const double lo = *std::min_element(w.begin(), w.end());
	double sum = 0.0;
	for (double v : w)
		sum += std::exp(-(v - lo) / tau);
	if (shares)
	{
		shares->clear();
		for (double v : w)
			shares->push_back(std::exp(-(v - lo) / tau) / sum);
	}
	return lo - tau * std::log(sum);
}

// ---------------------------------------------------------------------------------------------
// Close point pairs and the collision-aware closest IK
// ---------------------------------------------------------------------------------------------

// Per pair: d(distance)/dq over the group's joints, the distance, d(distance)/d(object position) (zero unless one
// side is the object) and "link - link".
struct ClosePairs
{
	std::vector<Eigen::RowVectorXd> gradients;
	std::vector<double> dists;
	std::vector<Eigen::Vector3d> object_slopes;
	std::vector<std::string> names;

	void Append(const ClosePairs& o)
	{
		gradients.insert(gradients.end(), o.gradients.begin(), o.gradients.end());
		dists.insert(dists.end(), o.dists.begin(), o.dists.end());
		object_slopes.insert(object_slopes.end(), o.object_slopes.begin(), o.object_slopes.end());
		names.insert(names.end(), o.names.begin(), o.names.end());
	}
};

ClosePairs ClosePairGradients(
	const collision_detection::DistanceResult& res, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* group, double margin)
{
	using collision_detection::BodyType;
	ClosePairs pairs;
	for (const auto& entry : res.distances)
		for (const auto& d : entry.second)
		{
			if (d.distance >= margin)
				continue;
			Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(group->getVariableCount());
			Eigen::Vector3d object_slope = Eigen::Vector3d::Zero();
			bool robot_side = false;
			for (int side = 0; side < 2; ++side)
			{
				// distance ~ distance0 + normal.(dp1 - dp0)
				const double sign = side == 0 ? -1.0 : 1.0;
				if (d.body_types[side] == BodyType::WORLD_OBJECT && d.link_names[side] == "object")
					object_slope += sign * d.normal;
				if (d.body_types[side] != BodyType::ROBOT_LINK && d.body_types[side] != BodyType::ROBOT_ATTACHED)
					continue;
				const moveit::core::LinkModel* link = state.getRobotModel()->getLinkModel(d.link_names[side]);
				if (!link || !group->isLinkUpdated(link->getName()))
					continue;  // the group's joints don't move it; asking for its Jacobian logs an error
				const Eigen::Vector3d local = state.getGlobalLinkTransform(link).inverse() * d.nearest_points[side];
				Eigen::MatrixXd J;
				if (!state.getJacobian(group, link, local, J))
					continue;
				row += sign * (d.normal.transpose() * J.topRows<3>());
				robot_side = true;
			}
			if (robot_side && row.norm() > 1e-9)
			{
				pairs.gradients.push_back(row);
				pairs.dists.push_back(d.distance);
				pairs.object_slopes.push_back(object_slope);
				pairs.names.push_back(d.link_names[0] + " - " + d.link_names[1]);
			}
		}
	return pairs;
}

collision_detection::DistanceRequest CloseRequest(
	const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, double threshold)
{
	collision_detection::DistanceRequest req;
	req.enable_nearest_points = true;
	req.enable_signed_distance = true;
	req.type = collision_detection::DistanceRequestType::ALL;
	req.group_name = group->getName();
	req.enableGroup(state.getRobotModel());
	req.distance_threshold = threshold;
	req.max_contacts_per_body = 2;
	return req;
}

// Self and world distances at the state's pose, skipping pairs the scene always allows (as the validity check does).
std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> Distances(
	const ScenePtr& scene, const moveit::core::RobotState& state, collision_detection::DistanceRequest& req)
{
	std::pair<collision_detection::DistanceResult, collision_detection::DistanceResult> res;
	planning_scene_monitor::LockedPlanningSceneRO locked(scene);
	req.acm = &locked->getAllowedCollisionMatrix();
	req.enableGroup(locked->getRobotModel());  // MoveIt matches active links by pointer, not name
	locked->getCollisionEnv()->distanceSelf(req, res.first, state);
	locked->getCollisionEnv()->distanceRobot(req, res.second, state);
	return res;
}

ClosePairs FindClosePairs(
	const ScenePtr& scene, const moveit::core::RobotState& state, const moveit::core::JointModelGroup* group,
	collision_detection::DistanceRequest& req, double margin)
{
	const auto [self_res, world_res] = Distances(scene, state, req);
	ClosePairs pairs = ClosePairGradients(self_res, state, group, margin);
	pairs.Append(ClosePairGradients(world_res, state, group, margin));
	return pairs;
}

// Links rigid with the tool, plus what hangs off them (hand, fingers).
std::set<std::string> ToolSideLinks(const moveit::core::LinkModel* tool)
{
	std::set<std::string> links;
	const moveit::core::LinkModel* wrist = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tool);
	links.insert(wrist->getName());
	for (const moveit::core::JointModel* joint : wrist->getChildJointModels())
		for (const moveit::core::LinkModel* link : joint->getDescendantLinkModels())
			links.insert(link->getName());
	return links;
}

// True if a 1 cm rod from the tool to the target hits nothing (tool side excepted): a closest pose with something in
// between can't slide onto the viewpoint, so its gap and slope would mislead.
bool ClearPathToTarget(
	const ScenePtr& scene, moveit::core::RobotState& state, const moveit::core::LinkModel* tool,
	const Eigen::Vector3d& target, const std::set<std::string>& tool_side)
{
	const Eigen::Vector3d local = state.getGlobalLinkTransform(tool).inverse() * target;
	const double length = local.norm();
	if (length < 0.01)
		return true;
	Eigen::Isometry3d rod = Eigen::Isometry3d::Identity();
	rod.translation() = 0.5 * local;  // cylinder axis is z, centered on its pose
	rod.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), local / length).toRotationMatrix();
	const std::string id = "path_to_target";
	state.attachBody(
		id, Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Cylinder>(0.005, length)}, {rod}, tool_side,
		tool->getName());
	state.update();
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked(scene);
		locked->checkCollision(req, res, state);
	}
	state.clearAttachedBody(id);
	return !res.collision;
}

struct ClosestIk
{
	bool found = false;
	Vector6d pose_error = Vector6d::Zero();
	double gap = std::numeric_limits<double>::max();
	Joints q;  // the best collision-free pose; empty if none
};

// Damped steps toward the target that keep close pairs out to `margin` (first priority) and climb manipulability in
// the target's null space. Joints at a limit that the step pushes further are frozen and the step re-solved.
ClosestIk ClosestIkPose(
	const ScenePtr& scene, moveit::core::RobotState& state, const moveit::core::JointModelGroup* group,
	const moveit::core::LinkModel* tool, const Eigen::Isometry3d& target, const Joints& start, int iters, double margin,
	double rot_scale, double limit_sharpness)
{
	const double damping = 0.01;
	const double max_joint_step = 0.1;
	const double manip_step = 0.02;  // rad per iteration up the manipulability
	const int stall_window = 5;		 // stop once the best gap improves < 1 mm over this many steps
	const double stall_gain = 1e-3;
	const int dof = static_cast<int>(group->getVariableCount());
	collision_detection::DistanceRequest req = CloseRequest(state, group, margin);

	Joints lower(static_cast<size_t>(dof)), upper(static_cast<size_t>(dof));
	for (size_t k = 0; k < lower.size(); ++k)
	{
		const moveit::core::VariableBounds& b = state.getRobotModel()->getVariableBounds(group->getVariableNames()[k]);
		lower[k] = b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::max();
		upper[k] = b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::max();
	}

	const std::set<std::string> tool_side = ToolSideLinks(tool);
	Joints q = start;
	ClosestIk best;
	std::vector<double> best_gap_history;
	for (int it = 0; it <= iters; ++it)
	{
		state.setJointGroupPositions(group, q);
		state.update();
		const Vector6d pose_error = PoseError(target, state.getGlobalLinkTransform(tool));
		const double gap = PoseGap(pose_error, rot_scale);
		const ClosePairs pairs = FindClosePairs(scene, state, group, req, margin);
		// The full validity check, not just the distance query, which lets self-collisions near the base through.
		if (gap < best.gap && IsStateValid(scene, &state, group, q.data()) &&
			ClearPathToTarget(scene, state, tool, target.translation(), tool_side))
		{
			best = {true, pose_error, gap, q};
			if (gap < 1e-6)
				break;
		}
		if (it == iters)
			break;
		best_gap_history.push_back(std::min(gap, best_gap_history.empty() ? gap : best_gap_history.back()));
		if (static_cast<int>(best_gap_history.size()) > stall_window &&
			best_gap_history[best_gap_history.size() - 1 - stall_window] - best_gap_history.back() < stall_gain)
			break;

		const Eigen::MatrixXd J = ToolJacobian(state, group, tool, q);
		const auto num_pairs = static_cast<Eigen::Index>(pairs.gradients.size());
		Eigen::MatrixXd Jc(num_pairs, dof);
		Eigen::VectorXd push(num_pairs);
		for (Eigen::Index k = 0; k < num_pairs; ++k)
		{
			Jc.row(k) = pairs.gradients[static_cast<size_t>(k)];
			push(k) = margin - pairs.dists[static_cast<size_t>(k)];
		}
		const Eigen::VectorXd manip_grad = ManipulabilityGradient(state, group, tool, q, limit_sharpness);
		const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dof, dof);
		auto masked_step = [&](const Eigen::VectorXd& mask) {
			const Eigen::MatrixXd Jm = J * mask.asDiagonal();
			const Eigen::MatrixXd Jm_pinv = DampedPinv(Jm, damping);
			Eigen::VectorXd step = Jm_pinv * pose_error;
			const Eigen::VectorXd up = (I - Jm_pinv * Jm) * mask.asDiagonal() * manip_grad;
			if (up.norm() > 1e-12)
				step += manip_step * up / up.norm();
			if (num_pairs > 0)
			{
				const Eigen::MatrixXd Jcm = Jc * mask.asDiagonal();
				const Eigen::MatrixXd Jc_pinv = DampedPinv(Jcm, damping);
				step = Jc_pinv * push + (I - Jc_pinv * Jcm) * step;
			}
			return step;
		};
		Eigen::VectorXd mask = Eigen::VectorXd::Ones(dof);
		Eigen::VectorXd dq = masked_step(mask);
		for (int pass = 0; pass < dof; ++pass)
		{
			bool changed = false;
			for (int k = 0; k < dof; ++k)
			{
				const size_t kk = static_cast<size_t>(k);
				if (mask(k) > 0.0 &&
					((q[kk] <= lower[kk] + 1e-4 && dq(k) < 0.0) || (q[kk] >= upper[kk] - 1e-4 && dq(k) > 0.0)))
				{
					mask(k) = 0.0;
					changed = true;
				}
			}
			if (!changed)
				break;
			dq = masked_step(mask);
		}
		const double step_max = dq.cwiseAbs().maxCoeff();
		if (step_max > max_joint_step)
			dq *= max_joint_step / step_max;
		for (int k = 0; k < dof; ++k)
			q[static_cast<size_t>(k)] += dq(k);
		state.setJointGroupPositions(group, q);
		state.enforceBounds(group);
		state.copyJointGroupPositions(group, q);
	}
	return best;
}

// ---------------------------------------------------------------------------------------------
// IK. One RNG for all random starts, reseeded before each solve: the same spot gets the same luck, so runs repeat and
// nearby spots compare fairly (MoveIt's own RNGs seed from the clock).
// ---------------------------------------------------------------------------------------------

std::unique_ptr<random_numbers::RandomNumberGenerator> seeded_rng;

random_numbers::RandomNumberGenerator& SeededRng()
{
	if (!seeded_rng)
		seeded_rng = std::make_unique<random_numbers::RandomNumberGenerator>(42);
	return *seeded_rng;
}

void ReseedRng(int seed)
{
	seeded_rng = std::make_unique<random_numbers::RandomNumberGenerator>(static_cast<boost::uint32_t>(std::max(1, seed)));
}

// attempts > 0: that many single KDL attempts (first from the current state, the rest random), a fixed amount of work
// so a seed repeats regardless of CPU speed. attempts <= 0: one time-limited call.
bool ExactIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity)
{
	if (attempts <= 0)
		return state.setFromIK(group, target, "tool0", timeout, validity);
	const double single_attempt = 1e-9;  // s: KDL stops after its first attempt (0 means the default timeout)
	for (int a = 0; a < attempts; ++a)
	{
		if (a > 0)
			state.setToRandomPositions(group, SeededRng());
		if (state.setFromIK(group, target, "tool0", single_attempt, validity))
			return true;
	}
	return false;
}

// ---------------------------------------------------------------------------------------------
// Tour: a GTSP over (viewpoint, IK solution) nodes, visiting one IK solution per reachable viewpoint.
// Nearest neighbour, then 2-opt segment reversals and per-position IK-solution swaps.
// ---------------------------------------------------------------------------------------------

struct GtspNode
{
	int group = -1;	  // index into the reachable viewpoints
	int branch = -1;  // index into that viewpoint's IK solutions
};

struct Gtsp
{
	std::vector<int> viewpoint;						   // group -> viewpoint
	std::vector<std::vector<const Joints*>> branches;  // group -> IK solutions
	std::vector<std::vector<double>> node_cost;		   // group -> per-solution cost; empty = none
	const Joints* home = nullptr;
	const BaseGradientParams* params = nullptr;

	const Joints& Q(const GtspNode& n) const { return *branches[n.group][n.branch]; }
	double NodeCost(const GtspNode& n) const { return node_cost.empty() ? 0.0 : node_cost[n.group][n.branch]; }
	double Edge(const GtspNode* a, const GtspNode& b) const { return EdgeCost(a ? Q(*a) : *home, Q(b), *params); }
};

std::vector<GtspNode> NearestNeighborTour(const Gtsp& g)
{
	std::vector<bool> visited(g.branches.size(), false);
	std::vector<GtspNode> order;
	for (size_t step = 0; step < g.branches.size(); ++step)
	{
		double best = std::numeric_limits<double>::max();
		GtspNode pick;
		bool picked = false;
		for (size_t grp = 0; grp < g.branches.size(); ++grp)
			if (!visited[grp])
				for (size_t b = 0; b < g.branches[grp].size(); ++b)
				{
					const GtspNode cand{static_cast<int>(grp), static_cast<int>(b)};
					const double cost = g.Edge(order.empty() ? nullptr : &order.back(), cand) + g.NodeCost(cand);
					if (!picked || cost < best)
					{
						best = cost;
						pick = cand;
						picked = true;
					}
				}
		visited[static_cast<size_t>(pick.group)] = true;
		order.push_back(pick);
	}
	return order;
}

std::vector<GtspNode> TwoOpt(const Gtsp& g, std::vector<GtspNode> order, int max_rounds)
{
	if (order.size() < 2)
		return order;
	auto edge_after = [&](size_t pos, const GtspNode& node) { return g.Edge(pos == 0 ? nullptr : &order[pos - 1], node); };
	for (int round = 0; round < max_rounds; ++round)
	{
		bool improved = false;
		for (bool reversed = order.size() >= 3; reversed;)
		{
			reversed = false;
			for (size_t i = 0; i + 1 < order.size(); ++i)
				for (size_t j = i + 1; j < order.size(); ++j)
				{
					double old_cost = edge_after(i, order[i]);
					double new_cost = edge_after(i, order[j]);
					if (j + 1 < order.size())
					{
						old_cost += g.Edge(&order[j], order[j + 1]);
						new_cost += g.Edge(&order[i], order[j + 1]);
					}
					if (new_cost < old_cost - 1e-9)
					{
						std::reverse(
							order.begin() + static_cast<std::ptrdiff_t>(i), order.begin() + static_cast<std::ptrdiff_t>(j) + 1);
						reversed = improved = true;
					}
				}
		}
		// Hold the neighbours fixed and pick each position's best IK solution.
		for (size_t pos = 0; pos < order.size(); ++pos)
		{
			const int grp = order[pos].group;
			double best_cost = std::numeric_limits<double>::max();
			int best_branch = order[pos].branch;
			for (size_t b = 0; b < g.branches[static_cast<size_t>(grp)].size(); ++b)
			{
				const GtspNode cand{grp, static_cast<int>(b)};
				double cost = edge_after(pos, cand) + g.NodeCost(cand);
				if (pos + 1 < order.size())
					cost += g.Edge(&cand, order[pos + 1]);
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

double TourCost(const Gtsp& g, const std::vector<GtspNode>& order)
{
	double total = 0.0;
	for (size_t k = 0; k < order.size(); ++k)
		total += g.Edge(k == 0 ? nullptr : &order[k - 1], order[k]);
	for (const GtspNode& node : order)
		total += g.NodeCost(node);
	return total;
}

// Viewpoint order and chosen IK solution per visit. warm_order (a previous order) also seeds a 2-opt run when every
// listed viewpoint is still reachable; the cheaper tour wins, so a step never loses to a re-ordering accident.
std::pair<std::vector<int>, std::vector<Joints>> FindTour(
	const std::vector<std::vector<Joints>>& ik, const Joints& home, const BaseGradientParams& params,
	const std::vector<int>* warm_order, const std::vector<std::vector<double>>& node_cost)
{
	Gtsp g;
	g.home = &home;
	g.params = &params;
	std::vector<int> group_of(ik.size(), -1);
	for (size_t v = 0; v < ik.size(); ++v)
	{
		if (ik[v].empty())
			continue;
		group_of[v] = static_cast<int>(g.viewpoint.size());
		g.viewpoint.push_back(static_cast<int>(v));
		std::vector<const Joints*> qs;
		for (const Joints& q : ik[v])
			qs.push_back(&q);
		g.branches.push_back(std::move(qs));
		g.node_cost.push_back(node_cost[v]);
	}
	std::pair<std::vector<int>, std::vector<Joints>> tour;
	if (g.branches.empty())
		return tour;
	const int rounds = std::max(1, params.gtsp_two_opt_rounds);
	std::vector<GtspNode> best = TwoOpt(g, NearestNeighborTour(g), rounds);
	if (warm_order && warm_order->size() == g.viewpoint.size())
	{
		std::vector<GtspNode> seeded;
		for (int v : *warm_order)
		{
			if (v < 0 || v >= static_cast<int>(group_of.size()) || group_of[static_cast<size_t>(v)] < 0)
				break;
			seeded.push_back({group_of[static_cast<size_t>(v)], 0});
		}
		if (seeded.size() == warm_order->size())
		{
			seeded = TwoOpt(g, std::move(seeded), rounds);
			if (TourCost(g, seeded) < TourCost(g, best))
				best = std::move(seeded);
		}
	}
	for (const GtspNode& node : best)
	{
		tour.first.push_back(g.viewpoint[static_cast<size_t>(node.group)]);
		tour.second.push_back(g.Q(node));
	}
	return tour;
}

// Cheapest IK solution per viewpoint for a fixed order: dynamic programming over (edge + per-solution) costs.
std::vector<Joints> BestPosesForOrder(
	const std::vector<std::vector<Joints>>& ik, const std::vector<std::vector<double>>& pose_cost,
	const std::vector<int>& order, const Joints& home, const BaseGradientParams& params)
{
	std::vector<std::vector<double>> best(order.size());  // best[k][j]: cheapest order[0..k] ending in solution j
	std::vector<std::vector<int>> parent(order.size());
	for (size_t k = 0; k < order.size(); ++k)
	{
		const size_t v = static_cast<size_t>(order[k]);
		best[k].assign(ik[v].size(), std::numeric_limits<double>::max());
		parent[k].assign(ik[v].size(), -1);
		for (size_t j = 0; j < ik[v].size(); ++j)
		{
			if (k == 0)
			{
				best[k][j] = pose_cost[v][j] + EdgeCost(home, ik[v][j], params);
				continue;
			}
			const size_t u = static_cast<size_t>(order[k - 1]);
			for (size_t i = 0; i < ik[u].size(); ++i)
			{
				const double c = best[k - 1][i] + pose_cost[v][j] + EdgeCost(ik[u][i], ik[v][j], params);
				if (c < best[k][j])
				{
					best[k][j] = c;
					parent[k][j] = static_cast<int>(i);
				}
			}
		}
	}
	std::vector<Joints> joints(order.size());
	if (order.empty())
		return joints;
	int j = static_cast<int>(std::min_element(best.back().begin(), best.back().end()) - best.back().begin());
	for (size_t k = order.size(); k-- > 0;)
	{
		joints[k] = ik[static_cast<size_t>(order[k])][static_cast<size_t>(j)];
		j = parent[k][static_cast<size_t>(j)];
	}
	return joints;
}

// Inserts viewpoints missing from `order` where their joint detour is smallest.
void InsertByDetour(std::vector<int>& order, const std::vector<std::vector<Joints>>& ik, const Joints& home)
{
	for (size_t v = 0; v < ik.size(); ++v)
	{
		if (ik[v].empty() || std::find(order.begin(), order.end(), static_cast<int>(v)) != order.end())
			continue;
		const Joints& qv = ik[v].front();
		size_t best_pos = order.size();
		double best_detour = std::numeric_limits<double>::max();
		for (size_t pos = 0; pos <= order.size(); ++pos)
		{
			const Joints& qa = pos == 0 ? home : ik[static_cast<size_t>(order[pos - 1])].front();
			double detour = JointDistance(qa, qv);
			if (pos < order.size())
			{
				const Joints& qb = ik[static_cast<size_t>(order[pos])].front();
				detour += JointDistance(qv, qb) - JointDistance(qa, qb);
			}
			if (detour < best_detour)
			{
				best_detour = detour;
				best_pos = pos;
			}
		}
		order.insert(order.begin() + static_cast<std::ptrdiff_t>(best_pos), static_cast<int>(v));
	}
}

// ---------------------------------------------------------------------------------------------
// Solutions
// ---------------------------------------------------------------------------------------------

// A missed viewpoint: its closest collision-free pose's error (capped if none) and that pose (empty if none).
struct Miss
{
	int vp;
	Vector6d gap;
	Joints q;
};

// The tour at one object position: what the descent minimizes.
struct TourSolution
{
	std::vector<int> tour;		 // viewpoint per visit
	std::vector<Joints> joints;	 // parallel: chosen IK solution
	std::vector<Miss> missed;
	double cost = 0.0;
	double travel = 0.0;			 // L2 edge cost, home edge included
	double sum_manipulability = 0.0;
	double softmin_manipulability = 0.0;
	double gap_sum = 0.0;			 // sum of capped miss gaps (m)
	double miss_cost = 0.0;			 // miss_gap_weight * gap_sum
	double planned = -1.0;			 // OMPL-planned joint travel; -1 = not planned
	int num_reachable = 0;
	int num_total = 0;
	bool all_reachable = false;
};

Vector6d CappedGap(double cap)
{
	Vector6d e = Vector6d::Zero();
	e(0) = cap;
	return e;
}

// Viewpoints reached in `from` but not in `to`.
std::vector<int> Lost(const TourSolution& from, const TourSolution& to)
{
	std::vector<int> lost;
	for (int v : from.tour)
		if (std::find(to.tour.begin(), to.tour.end(), v) == to.tour.end())
			lost.push_back(v);
	return lost;
}

std::string ListText(const std::vector<int>& v)
{
	std::string out;
	for (int x : v)
		out += " " + std::to_string(x);
	return out.empty() ? std::string(" none") : out;
}

// ---------------------------------------------------------------------------------------------
// Room-then-reach margins
// ---------------------------------------------------------------------------------------------

// One safety margin of a reached viewpoint (joint-limit distance, stretch, reach or collision distance) and its
// slope w.r.t. the object position (the arm following the viewpoint) and w.r.t. the joints.
struct Margin
{
	int vp = -1;
	std::string name;
	double value = 0.0;
	double floor = 0.0;
	Eigen::Vector3d slope = Eigen::Vector3d::Zero();
	Eigen::VectorXd joint_slope;
};

// All reached viewpoints' margins at one spot, and per viewpoint (empty if missed) its joints and dq/d(position).
struct ArmModel
{
	std::vector<Margin> margins;
	std::vector<Joints> q;
	std::vector<Eigen::MatrixXd> dq_dp;

	std::vector<Margin> Of(int v) const
	{
		std::vector<Margin> out;
		for (const Margin& m : margins)
			if (m.vp == v)
				out.push_back(m);
		return out;
	}
};

// A lost viewpoint's margins that ran out on a move, and why.
struct Shortage
{
	std::vector<Margin> margins;
	std::string why;
};

// How far the object can move along unit `dir` before a margin, to first order, reaches min(floor, value / 2).
double RoomAlong(const std::vector<Margin>& margins, const Eigen::Vector3d& dir)
{
	double room = kInf;
	for (const Margin& m : margins)
	{
		const double rate = m.slope.dot(dir);
		if (rate < -1e-9)
			room = std::min(room, std::max(0.0, m.value - std::min(m.floor, 0.5 * m.value)) / -rate);
	}
	return room;
}

// Smallest margin, in floors.
double Tightest(const std::vector<Margin>& margins)
{
	double lo = kInf;
	for (const Margin& m : margins)
		lo = std::min(lo, m.value / m.floor);
	return lo;
}

// Unit direction that most raises these margins' soft-min (in floors), to first order: projected gradient ascent
// within |d| <= radius. Zero if none raises them.
Eigen::Vector3d MostRoomDirection(const std::vector<Margin>& margins, double radius)
{
	const double tau = 1.0;  // floors
	auto softmin_slope = [&](const Eigen::Vector3d& d) {
		std::vector<double> u(margins.size());
		double lo = kInf;
		for (size_t k = 0; k < margins.size(); ++k)
		{
			u[k] = (margins[k].value + margins[k].slope.dot(d)) / margins[k].floor;
			lo = std::min(lo, u[k]);
		}
		Eigen::Vector3d g = Eigen::Vector3d::Zero();
		double sum = 0.0;
		for (size_t k = 0; k < margins.size(); ++k)
		{
			const double w = std::exp(-(u[k] - lo) / tau);
			g += w * margins[k].slope / margins[k].floor;
			sum += w;
		}
		return sum > 0.0 ? Eigen::Vector3d(g / sum) : g;
	};
	Eigen::Vector3d d = Eigen::Vector3d::Zero();
	for (int it = 0; it < 100; ++it)
	{
		const Eigen::Vector3d g = softmin_slope(d);
		if (g.norm() < 1e-9)
			break;
		d += 0.5 * radius / (1.0 + 0.1 * it) * g.normalized();
		if (d.norm() > radius)
			d *= radius / d.norm();
	}
	return d.norm() < 1e-3 * radius ? Eigen::Vector3d::Zero() : Eigen::Vector3d(d.normalized());
}

// ---------------------------------------------------------------------------------------------
// Solver: up to N descents (restarts). Each iteration takes the gradient, line-searches along it, falls back to
// room-then-reach when stuck with misses, and re-solves fully at the accepted spot.
// ---------------------------------------------------------------------------------------------

struct DescentResult
{
	Position position = Position::Zero();
	TourSolution sol;
	double cost = std::numeric_limits<double>::max();
	bool ok = false;  // every viewpoint reached
};

// A move found by the line search or room-then-reach: where, its quick solve, and the step length.
struct Step
{
	Position at;
	TourSolution sol;
	double size = 0.0;
};

class Solver
{
public:
	Solver(
		const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model, const ScenePtr& scene,
		const std::string& group_name, const Eigen::Vector3d& object_translation_nominal,
		const Eigen::Matrix3d& object_rotation_nominal, const std::vector<Eigen::Isometry3d>& viewpoints_nominal,
		const Joints& home, const BaseGradientParams& params);

	// One descent from `start`. warm: IK seeds and viewpoint order from an earlier descent, or null for home.
	DescentResult RunDescent(const Position& start, int restart, const TourSolution* warm);
	// Before a restart: climb the reached viewpoints' manipulability from a stuck spot. Returns where it ends.
	Position ManipulabilityJump(const Position& from, const TourSolution& warm, int restart);
	// Alternative: climb the reachability-map score of all viewpoints (no IK).
	Position ReachabilityJump(const Position& from, int restart);

	BaseGradientResult result;  // history and solve count; SolveBaseGradient fills in the rest

private:
	// --- Descent ---
	bool StartDescent(const Position& start, int restart, const TourSolution* warm);
	int ReachAllViewpoints();
	void RefinePathAndPosition(int iter);
	bool DescentStep(int iter);
	Eigen::Vector3d Gradient();
	void PrepareProbes();
	std::optional<Step> LineSearch(const Eigen::Vector3d& dir);
	std::optional<Step> RoomThenReach();
	std::optional<TourSolution> FullSolveAt(const Step& step);
	bool TakeStep(const Step& step, TourSolution committed, int iter, double gnorm, const Eigen::Vector3d& g);
	void LogRefineAtReach();
	void RefineResult();

	// --- Solving and scoring ---
	std::vector<std::vector<Joints>> CollectIk(
		const Position& at, const BaseGradientParams& p, const std::vector<Joints>& seed, int max_solutions);
	TourSolution Solve(
		const Position& at, const BaseGradientParams& p, const std::vector<Joints>& seed,
		const std::vector<int>* warm_order, int max_solutions, const std::vector<int>* fixed_order);
	TourSolution SolveBestOf(
		const Position& at, const BaseGradientParams& p, const std::vector<int>* warm_order,
		const std::vector<int>* fixed_order);
	TourSolution Probe(const Position& at, int gaps_min_reached = 0);
	TourSolution Probe(const Position& at, BaseGradientParams p);
	TourSolution Refine(const Position& at, TourSolution best, int num_solves);
	double PlannedTravel(const Position& at, const TourSolution& sol);
	TourSolution WithPlannedTravel(const Position& at, TourSolution sol);
	double TravelTerm(const TourSolution& sol) const;
	double Cost(const TourSolution& sol) const;
	bool Better(const TourSolution& a, const TourSolution& b) const;
	void Record(const Position& at, const TourSolution& sol);
	void Remember(const TourSolution& sol);
	Eigen::Vector3d AnalyticGradient(const TourSolution& sol, const BaseGradientParams& p);
	Eigen::Vector3d FiniteDifferenceGradient();

	// --- Room-then-reach ---
	Eigen::Vector3d LockAxes(Eigen::Vector3d v) const;
	Eigen::Vector3d AimAt(int target, const TourSolution& here_at, bool first);
	Eigen::MatrixXd Jacobian(const Joints& q) { return ToolJacobian(state, group, tool, q); }
	double Stretch(const Joints& q);
	double ShoulderToWrist(const Joints& q);
	ArmModel ArmModelAt(const Position& at, const TourSolution& sol);
	std::vector<Margin> ViewpointMargins(int v, const Joints& q);
	Joints PredictedPose(const ArmModel& model, int v, const Eigen::Vector3d& d);
	std::optional<Joints> ExactPose(int v, const Position& at, Joints q);
	Shortage RanOut(int v, const ArmModel& model, const Position& at, const Eigen::Vector3d& d);
	std::optional<Joints> RoomierPose(int v, const Position& at, const ArmModel& model, int tries);

	// --- Logging ---
	void LogCostTerms(const TourSolution& sol);
	void LogMissed(const Position& at, const TourSolution& sol);
	void Publish(const Eigen::Vector3d& neg_grad);
	void PlaceObject(const Position& at) { MoveObject(scene, Translation(at) * object_pose); }

	// --- Fixed for the whole solve ---
	rclcpp::Node::SharedPtr node;
	moveit::core::RobotModelConstPtr robot_model;
	ScenePtr scene;
	std::string group_name;
	const BaseGradientParams params;
	Eigen::Isometry3d object_pose;			  // object relative to its own position
	std::vector<Eigen::Isometry3d> viewpoints;  // viewpoints relative to the object's position
	int num_vp;
	std::vector<int> input_order;
	moveit::core::RobotState state;
	const moveit::core::JointModelGroup* group;
	const moveit::core::LinkModel* tool;
	Joints home;
	moveit::core::GroupStateValidityCallbackFn validity;
	bool use_planning;	   // planned travel can be computed
	bool planned_travel;  // and replaces the L2 travel in the cost
	const moveit::core::LinkModel* shoulder = nullptr;	// reach margin: shoulder-wrist distance vs its max
	const moveit::core::LinkModel* wrist = nullptr;
	double max_reach = 0.0;
	std::optional<ReachabilityMap> reach_map;  // loaded or built on first use
	std::vector<int> reach_bins;				   // each viewpoint's approach-direction bin

	// --- Per descent ---
	int restart = 0;
	int seed = 0;
	Position position;	  // current object position
	TourSolution current;  // full solve at `position`
	DescentResult best;	  // best of this descent
	std::vector<Position> trail;
	std::vector<Joints> seeds;
	int stall_count = 0;
	std::vector<Joints> last_reached;  // each viewpoint's joints when last reached
	std::vector<Joints> last_closest;  // each missed viewpoint's last closest pose: keeps its gap continuous
	bool tracked_probes = true;		   // probes follow the current poses (reach phase) or re-solve fresh (after)
	bool refined_at_reach = false;
	std::chrono::steady_clock::time_point reach_deadline;

	// --- Per iteration ---
	std::vector<int> locked_order;
	const std::vector<int>* order_lock = nullptr;  // null = GTSP reorders
	TourSolution here;							   // quick solve at `position`: what probes compare against
};

Solver::Solver(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model, const ScenePtr& scene,
	const std::string& group_name, const Eigen::Vector3d& object_translation_nominal,
	const Eigen::Matrix3d& object_rotation_nominal, const std::vector<Eigen::Isometry3d>& viewpoints_nominal,
	const Joints& home, const BaseGradientParams& params)
	: node(node), robot_model(robot_model), scene(scene), group_name(group_name), params(params),
	  object_pose(MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal)), viewpoints(viewpoints_nominal),
	  num_vp(static_cast<int>(viewpoints_nominal.size())), input_order(static_cast<size_t>(num_vp)), state(robot_model),
	  home(home)
{
	ReseedRng(params.random_seed);
	for (Eigen::Isometry3d& t : viewpoints)
		t.translation() -= object_translation_nominal;
	std::iota(input_order.begin(), input_order.end(), 0);
	state.setToDefaultValues();
	group = state.getJointModelGroup(group_name);
	tool = robot_model->getLinkModel("tool0");
	validity = [this](moveit::core::RobotState* s, const moveit::core::JointModelGroup* g, const double* q) {
		return IsStateValid(this->scene, s, g, q);
	};
	use_planning = params.real_cost_planning_time > 0.0;
	planned_travel = params.planned_travel_in_cost && use_planning;

	const auto& joints = group->getActiveJointModels();
	if (joints.size() >= 6)
	{
		shoulder = joints[1]->getChildLinkModel();
		wrist = joints[5]->getChildLinkModel();
		// The elbow (joint 4) alone sets the shoulder-wrist distance: sweep it for the max.
		const moveit::core::VariableBounds& elbow = robot_model->getVariableBounds(group->getVariableNames()[3]);
		Joints q = home;
		for (int i = 0; i <= 1000; ++i)
		{
			q[3] = elbow.min_position_ + (elbow.max_position_ - elbow.min_position_) * i / 1000.0;
			max_reach = std::max(max_reach, ShoulderToWrist(q));
		}
	}
	state.setJointGroupPositions(group, home);
	state.update();
	result.num_total = num_vp;
}

// ---------------------------------------------------------------------------------------------
// Descent
// ---------------------------------------------------------------------------------------------

DescentResult Solver::RunDescent(const Position& start, int restart_in, const TourSolution* warm)
{
	if (!StartDescent(start, restart_in, warm))
		return best;
	const int iters = ReachAllViewpoints();
	if (current.all_reachable)
		RefinePathAndPosition(iters);
	return best;
}

std::vector<Joints> SeedsFrom(const TourSolution& sol, const Joints& fallback, int num_vp)
{
	std::vector<Joints> seeds(static_cast<size_t>(num_vp), fallback);
	for (size_t k = 0; k < sol.tour.size(); ++k)
		if (!sol.joints[k].empty())
			seeds[static_cast<size_t>(sol.tour[k])] = sol.joints[k];
	return seeds;
}

// Resets the descent and solves at `start`. False if nothing is reachable there.
bool Solver::StartDescent(const Position& start, int restart_in, const TourSolution* warm)
{
	restart = restart_in;
	seed = params.random_seed + restart;  // each restart draws its own IK luck
	position = Clamp(start, params.bounds);
	trail = {position};
	seeds = warm ? SeedsFrom(*warm, home, num_vp) : std::vector<Joints>(static_cast<size_t>(num_vp), home);
	stall_count = 0;
	last_reached.assign(static_cast<size_t>(num_vp), {});
	last_closest.assign(static_cast<size_t>(num_vp), {});
	tracked_probes = true;
	refined_at_reach = false;
	reach_deadline = params.reach_time_limit > 0.0
		? std::chrono::steady_clock::now() +
			std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				std::chrono::duration<double>(params.reach_time_limit))
		: std::chrono::steady_clock::time_point::max();
	best = DescentResult{};
	best.position = position;

	ReseedRng(seed);
	current = SolveBestOf(position, params, warm ? &warm->tour : nullptr, params.lock_input_order ? &input_order : nullptr);
	current = WithPlannedTravel(position, std::move(current));
	if (current.tour.empty())
	{
		RCLCPP_WARN(node->get_logger(), "restart %d: no tour pose reachable at the start placement", restart + 1);
		return false;
	}
	Record(position, current);
	Remember(current);
	Publish(Eigen::Vector3d::Zero());
	RCLCPP_INFO(
		node->get_logger(), "restart %d/%d iter 0: abs (%.4f, %.4f, %.4f) m  reachable %d/%d", restart + 1,
		std::max(1, params.descent_num_restart), position.x(), position.y(), position.z(), current.num_reachable, num_vp);
	LogCostTerms(current);
	LogMissed(position, current);
	return true;
}

// 1) Descend while any viewpoint is missed. Returns the iterations used.
int Solver::ReachAllViewpoints()
{
	int no_gain = 0;  // iterations since a viewpoint was last gained
	for (int iter = 0; iter < params.max_outer_iterations; ++iter)
	{
		if (current.all_reachable || !rclcpp::ok())
			return iter;
		if (std::chrono::steady_clock::now() > reach_deadline || no_gain >= params.reach_patience)
		{
			RCLCPP_WARN(
				node->get_logger(), "  restart %d: %s with %d/%d reachable -- stuck", restart + 1,
				no_gain >= params.reach_patience ? "no viewpoint gained for reach_patience iterations" : "reach time limit hit",
				current.num_reachable, num_vp);
			return iter;
		}
		const int reached = current.num_reachable;
		if (!DescentStep(iter))
			return iter + 1;
		no_gain = current.num_reachable > reached ? 0 : no_gain + 1;
	}
	return params.max_outer_iterations;
}

// 2) Once all are reached: keep descending on travel and comfort (unless stop_when_all_reached), then refine.
void Solver::RefinePathAndPosition(int iter)
{
	LogRefineAtReach();
	tracked_probes = false;  // a dragged-along tour looks worse than the spot really is
	if (params.stop_when_all_reached)
		RCLCPP_INFO(node->get_logger(), "  restart %d: all viewpoints reached -- stopping", restart + 1);
	else
		for (; iter < params.max_outer_iterations && rclcpp::ok(); ++iter)
			if (!DescentStep(iter))
				break;
	RefineResult();
}

// Gradient, line search, room-then-reach if that fails with misses, full solve at the new spot. False when the
// descent ends (converged or stuck).
bool Solver::DescentStep(int iter)
{
	const Eigen::Vector3d g = Gradient();
	const Eigen::Vector3d g_free = LockAxes(g);
	const double gnorm = g_free.norm();
	if (gnorm < 1e-6)
	{
		RCLCPP_INFO(node->get_logger(), "  restart %d: gradient ~ 0 -- converged", restart + 1);
		Publish(-g);
		return false;
	}
	PrepareProbes();
	std::optional<Step> step = LineSearch(-g_free / gnorm);
	if (!step && params.room_then_reach && !here.missed.empty())
		step = RoomThenReach();
	std::optional<TourSolution> committed;
	if (step)
		committed = FullSolveAt(*step);
	if (!committed)
	{
		if (current.all_reachable)
			RCLCPP_INFO(node->get_logger(), "  restart %d: no step reduces the tour cost -- converged", restart + 1);
		else
			RCLCPP_WARN(
				node->get_logger(), "  restart %d: no step reduces the tour cost with %d/%d reachable -- stuck",
				restart + 1, current.num_reachable, num_vp);
		Publish(-g);
		return false;
	}
	return !TakeStep(*step, std::move(*committed), iter, gnorm, g);
}

Eigen::Vector3d Solver::Gradient()
{
	const Eigen::Vector3d g = AnalyticGradient(current, params);
	if (params.trace_line_search)
	{
		// Each term's own descent direction.
		auto part = [&](bool manip, double gap_weight) {
			BaseGradientParams p = params;
			p.travel_gradient = false;
			p.manipulability_gradient = manip;
			p.miss_gap_weight = gap_weight;
			return AnalyticGradient(current, p);
		};
		auto describe = [](const Eigen::Vector3d& gp) {
			const double nrm = gp.norm();
			const Eigen::Vector3d u = nrm > 1e-9 ? Eigen::Vector3d(-gp / nrm) : Eigen::Vector3d::Zero();
			char buf[96];
			std::snprintf(buf, sizeof(buf), "|%.1f| dir (%+.2f, %+.2f, %+.2f)", nrm, u.x(), u.y(), u.z());
			return std::string(buf);
		};
		RCLCPP_INFO(
			node->get_logger(), "    directions: manip %s  gap %s  total %s",
			describe(part(params.manipulability_gradient, 0.0)).c_str(),
			describe(part(false, params.miss_gap_weight)).c_str(), describe(g).c_str());
	}
	if (params.fd_gradient_check)
	{
		const Eigen::Vector3d g_fd = FiniteDifferenceGradient();
		RCLCPP_INFO(
			node->get_logger(), "    grad check [x y z]  analytic (%+.4f %+.4f %+.4f)  central-diff (%+.4f %+.4f %+.4f)",
			g(0), g(1), g(2), g_fd(0), g_fd(1), g_fd(2));
	}
	return g;
}

// Seeds, order lock, and the quick solve here that this iteration's probes compare against.
void Solver::PrepareProbes()
{
	seeds = SeedsFrom(current, home, num_vp);
	for (const Miss& m : current.missed)
	{
		const size_t v = static_cast<size_t>(m.vp);
		if (!last_closest[v].empty())
			seeds[v] = last_closest[v];
		else if (!last_reached[v].empty())
			seeds[v] = last_reached[v];
	}
	locked_order = current.tour;
	order_lock = params.lock_input_order ? &input_order
		: (params.freeze_order && current.all_reachable) ? &locked_order
														 : nullptr;
	here = WithPlannedTravel(position, Probe(position));
	result.num_inner_solves += 1;
}

// Backtracking along `dir`: shrink the step until a probe beats the quick solve here (same IK effort, so a viewpoint
// the quick solve merely failed to find doesn't block the step).
std::optional<Step> Solver::LineSearch(const Eigen::Vector3d& dir)
{
	double step = params.initial_step;
	for (int ls = 0; ls < params.max_line_search_iters; ++ls)
	{
		const Position cand = Clamp(position + step * dir, params.bounds);
		TourSolution probe = Probe(cand, here.num_reachable);
		result.num_inner_solves += 1;
		if (probe.num_reachable >= here.num_reachable)
			probe = WithPlannedTravel(cand, std::move(probe));	// plan only probes that could be accepted
		if (params.trace_line_search)
			RCLCPP_INFO(
				node->get_logger(),
				"    probe step=%.4f m abs (%.4f, %.4f, %.4f): reached %d vs %d here, cost %.2f vs %.2f here "
				"(sum_w %.3f vs %.3f, planned %.2f vs %.2f)  lost:%s",
				step, cand.x(), cand.y(), cand.z(), probe.num_reachable, here.num_reachable, probe.cost, here.cost,
				probe.sum_manipulability, here.sum_manipulability, probe.planned, here.planned,
				ListText(Lost(here, probe)).c_str());
		if (Better(probe, here))
			return Step{cand, std::move(probe), step};
		step *= params.step_shrink;
	}
	return std::nullopt;
}

// Room then reach. For each missed viewpoint (smallest gap first), aim a full step at it: reached with nothing lost is
// accepted, nothing lost walks on. If reached viewpoints are lost, back up instead along the move that most raises the
// margins that ran out for them (kept off the other viewpoints' limits), then aim again.
std::optional<Step> Solver::RoomThenReach()
{
	const int kMaxRounds = 6;
	const int kMaxReplans = 3;
	const int kLostTries = 10;	   // other-IK candidates for a lost viewpoint
	const int kSqueezedTries = 6;  // and for one the room move would squeeze
	const double step = params.initial_step;
	std::vector<std::optional<Position>> searched_at(static_cast<size_t>(num_vp));
	auto gained = [&](const TourSolution& probe) {
		return Lost(here, probe).empty() && probe.num_reachable > here.num_reachable;
	};
	auto trace = [&](const std::string& what, int target, const Position& cand, const TourSolution& probe,
					 const std::vector<int>& lost, const std::string& verdict) {
		if (params.trace_line_search)
			RCLCPP_INFO(
				node->get_logger(), "    %s for vp %d abs (%.4f, %.4f, %.4f): reached %d vs %d here  lost:%s -> %s",
				what.c_str(), target, cand.x(), cand.y(), cand.z(), probe.num_reachable, here.num_reachable,
				ListText(lost).c_str(), verdict.c_str());
	};
	// Probes start each reached viewpoint's IK from its pose at the spot they move from.
	auto probe_from = [&](const TourSolution& here_at, const Position& cand) {
		for (size_t k = 0; k < here_at.tour.size(); ++k)
			seeds[static_cast<size_t>(here_at.tour[k])] = here_at.joints[k];
		result.num_inner_solves += 1;
		return Probe(cand, here_at.num_reachable);
	};
	std::vector<Miss> targets = here.missed;
	std::sort(targets.begin(), targets.end(), [&](const Miss& a, const Miss& b) {
		return PoseGap(a.gap, params.rot_metric_scale) < PoseGap(b.gap, params.rot_metric_scale);
	});
	if (params.room_max_targets > 0 && static_cast<int>(targets.size()) > params.room_max_targets)
		targets.resize(static_cast<size_t>(params.room_max_targets));  // the far ones rarely succeed
	for (const Miss& miss : targets)
	{
		const int target = miss.vp;
		Position at = position;
		TourSolution here_at = here;
		for (int round = 0; round < kMaxRounds && std::chrono::steady_clock::now() < reach_deadline; ++round)
		{
			// Aim: a full step at the target.
			const Eigen::Vector3d aim = AimAt(target, here_at, round == 0);
			if (aim.norm() < 1e-9)
				break;
			const Position cand = Clamp(at + step * aim, params.bounds);
			TourSolution probe = probe_from(here_at, cand);
			const std::vector<int> lost = Lost(here_at, probe);
			if (gained(probe))
			{
				trace("aim", target, cand, probe, lost, "accepted");
				return Step{cand, WithPlannedTravel(cand, std::move(probe)), step};
			}
			if (lost.empty())
			{
				trace("aim", target, cand, probe, lost, "walk on");
				at = cand;
				here_at = std::move(probe);
				continue;
			}

			// Back up. A viewpoint near a limit first takes its roomiest other IK solution here (once per spot).
			ArmModel model = ArmModelAt(at, here_at);
			auto swap_pose = [&](int v, int tries) {
				std::optional<Position>& last = searched_at[static_cast<size_t>(v)];
				if (last && *last == at)
					return false;
				last = at;
				const std::optional<Joints> roomier = RoomierPose(v, at, model, tries);
				if (!roomier)
					return false;
				const auto it = std::find(here_at.tour.begin(), here_at.tour.end(), v);
				here_at.joints[static_cast<size_t>(it - here_at.tour.begin())] = *roomier;
				return true;
			};
			std::vector<Margin> short_margins;
			std::vector<bool> joined(static_cast<size_t>(num_vp), false);
			// Adds what ran out for viewpoint v on the move to `to`.
			auto add_shortage = [&](int v, const Position& to) {
				const Shortage sh = RanOut(v, model, at, to - at);
				short_margins.insert(short_margins.end(), sh.margins.begin(), sh.margins.end());
				joined[static_cast<size_t>(v)] = true;
				return " vp " + std::to_string(v) + ": " + sh.why + ";";
			};
			bool swapped = false;
			for (int v : lost)
				swapped = swap_pose(v, kLostTries) || swapped;
			if (swapped)
				model = ArmModelAt(at, here_at);
			std::string why;
			for (int v : lost)
				why += add_shortage(v, cand);
			trace("aim", target, cand, probe, lost, "back up (" + why + " )");
			Eigen::Vector3d room = MostRoomDirection(short_margins, step);
			// Viewpoints the move would run out join in, after the same pose swap.
			for (int pass = 0; pass < 3 && room.norm() > 1e-9; ++pass)
			{
				std::vector<int> squeezed;
				for (const Margin& m : model.margins)
					if (!joined[static_cast<size_t>(m.vp)] && RoomAlong({m}, room) < step &&
						std::find(squeezed.begin(), squeezed.end(), m.vp) == squeezed.end())
						squeezed.push_back(m.vp);
				if (squeezed.empty())
					break;
				bool swapped_any = false;
				for (int v : squeezed)
					swapped_any = swap_pose(v, kSqueezedTries) || swapped_any;
				if (swapped_any)
					model = ArmModelAt(at, here_at);
				for (int v : squeezed)
				{
					const std::vector<Margin> mine = model.Of(v);
					if (RoomAlong(mine, room) >= step)
						continue;  // its new pose has room for the move
					for (const Margin& m : mine)
						if (RoomAlong({m}, room) < step)
							short_margins.push_back(m);
					joined[static_cast<size_t>(v)] = true;
				}
				room = MostRoomDirection(short_margins, step);
			}
			if (room.norm() < 1e-9)
				break;

			// Room move: a full step; one that loses a viewpoint adds what ran out for it and is re-planned, last
			// half a step.
			bool moved = false;
			for (int plan = 0; plan <= kMaxReplans && !moved && room.norm() > 1e-9; ++plan)
			{
				const double size = plan < kMaxReplans ? step : 0.5 * step;
				const Position back = Clamp(at + size * room, params.bounds);
				TourSolution rp = probe_from(here_at, back);
				const std::vector<int> back_lost = Lost(here_at, rp);
				char what[64];
				std::snprintf(what, sizeof(what), "room (%+.2f, %+.2f, %+.2f) x %.3f m", room(0), room(1), room(2), size);
				if (gained(rp))
				{
					trace(what, target, back, rp, back_lost, "gained, accepted");
					return Step{back, WithPlannedTravel(back, std::move(rp)), size};
				}
				if (back_lost.empty())
				{
					trace(what, target, back, rp, back_lost, "moved");
					at = back;
					here_at = std::move(rp);
					moved = true;
					break;
				}
				std::string room_why;
				for (int v : back_lost)
				{
					if (swap_pose(v, kLostTries))
						model = ArmModelAt(at, here_at);
					room_why += add_shortage(v, back);
				}
				trace(what, target, back, rp, back_lost, "lost, add what ran out (" + room_why + " ) and replan");
				room = MostRoomDirection(short_margins, step);
			}
			if (!moved)
				break;
		}
	}
	return std::nullopt;
}

// Full solve at the accepted spot (the probe's own solve stands if better); kept only if it beats the current one.
std::optional<TourSolution> Solver::FullSolveAt(const Step& step)
{
	BaseGradientParams p = params;
	if (!step.sol.all_reachable)
		p.gtsp_num_restart = 1;  // the order hardly matters while viewpoints are missed
	ReseedRng(seed);
	TourSolution committed = WithPlannedTravel(step.at, SolveBestOf(step.at, p, &step.sol.tour, order_lock));
	if (Better(step.sol, committed))
		committed = step.sol;
	const bool keep = Better(committed, current);
	if (params.trace_line_search)
		RCLCPP_INFO(
			node->get_logger(), "    full solve: reached %d vs %d current, cost %.2f vs %.2f current -> %s",
			committed.num_reachable, current.num_reachable, committed.cost, current.cost, keep ? "accepted" : "rejected");
	if (!keep)
		return std::nullopt;
	return committed;
}

// Moves to the accepted spot and logs it. True when the descent has converged.
bool Solver::TakeStep(const Step& step, TourSolution committed, int iter, double gnorm, const Eigen::Vector3d& g)
{
	const double moved = (step.at - position).norm();
	const double rel_gain = (current.cost - committed.cost) / std::max(std::abs(current.cost), 1e-9);
	const bool gained = committed.num_reachable > current.num_reachable;  // progress even if the cost rose

	position = step.at;
	current = std::move(committed);
	Remember(current);
	trail.push_back(position);
	Record(position, current);
	RCLCPP_INFO(
		node->get_logger(), "restart %d/%d iter %d/%d: abs (%.4f, %.4f, %.4f) m  reachable %d/%d  |grad|=%.4f  step=%.4f m",
		restart + 1, std::max(1, params.descent_num_restart), iter + 1, params.max_outer_iterations, position.x(),
		position.y(), position.z(), current.num_reachable, num_vp, gnorm, step.size);
	LogCostTerms(current);
	LogMissed(position, current);
	Publish(-g);

	if (gained || rel_gain >= params.convergence_tolerance_cost)
		stall_count = 0;
	else if (++stall_count >= std::max(1, params.patience))
	{
		RCLCPP_INFO(
			node->get_logger(), "  restart %d: %d iterations with <%.1e relative gain -- stopping early", restart + 1,
			stall_count, params.convergence_tolerance_cost);
		return true;
	}
	if (!gained && moved < params.convergence_tolerance_offset && rel_gain < params.convergence_tolerance_cost)
	{
		RCLCPP_INFO(node->get_logger(), "  restart %d: placement settled -- converged", restart + 1);
		return true;
	}
	return false;
}

// Gradient ascent on the reached viewpoints' manipulability (sum + jump_softmin_weight x soft-min), whatever is
// reachable at each spot; misses and travel ignored. Backtracking line search; a step is kept when its full solve
// raises the score. Ends when no step does, or after `patience` steps of tiny gain.
// Gradient ascent on the soft-min, over all viewpoints (reached or not), of their reachability-map values; steps from
// initial_step halving down to 5 mm.
Position Solver::ReachabilityJump(const Position& from, int restart_in)
{
	if (!reach_map)
	{
		ReachabilityMap::Options options;
		options.samples = params.reach_map_samples;
		options.voxel = params.reach_map_voxel;
		options.directions = params.reach_map_directions;
		options.limit_sharpness = params.reach_map_limit_sharpness;
		options.seed = params.reach_map_seed;
		const char* ros_home = std::getenv("HOME");
		const std::string path = !params.reach_map_file.empty()
			? params.reach_map_file
			: std::string(ros_home ? ros_home : "/tmp") + "/.ros/reachability_" + group_name + ".bin";
		reach_map = ReachabilityMap::LoadOrBuild(path, robot_model, group_name, tool->getName(), options);
		for (const Eigen::Isometry3d& v : viewpoints)
			reach_bins.push_back(reach_map->Bin(v.linear().col(2)));
		if (reach_map->built)
			RCLCPP_INFO(node->get_logger(), "reachability map built in %.1f s, saved to %s", reach_map->build_seconds, path.c_str());
		else
			RCLCPP_INFO(node->get_logger(), "reachability map loaded from %s", path.c_str());
	}
	auto score = [&](const Position& at, Eigen::Vector3d* grad) {
		std::vector<double> values(viewpoints.size());
		std::vector<Eigen::Vector3d> grads(viewpoints.size());
		for (size_t v = 0; v < viewpoints.size(); ++v)
			values[v] = reach_map->Value(reach_bins[v], at + viewpoints[v].translation(), &grads[v]);
		std::vector<double> shares;
		const double s = SoftMin(values, params.reach_map_tau, &shares);
		*grad = Eigen::Vector3d::Zero();
		for (size_t v = 0; v < values.size(); ++v)
			*grad += shares[v] * grads[v];
		*grad = LockAxes(*grad);
		return s;
	};
	Position at = from;
	Eigen::Vector3d g;
	const double start_score = score(at, &g);
	double s = start_score;
	for (double step = params.initial_step; step >= 0.005 && g.norm() > 1e-12;)
	{
		const Position cand = Clamp(at + step * g.normalized(), params.bounds);
		Eigen::Vector3d g_cand;
		const double s_cand = score(cand, &g_cand);
		if (s_cand > s)
		{
			at = cand;
			s = s_cand;
			g = g_cand;
		}
		else
			step *= 0.5;
	}
	RCLCPP_INFO(
		node->get_logger(),
		"reachability jump %d: score %.4f -> %.4f, moved %.4f m from abs (%.4f, %.4f, %.4f) to (%.4f, %.4f, %.4f)",
		restart_in + 1, start_score, s, (at - from).norm(), from.x(), from.y(), from.z(), at.x(), at.y(), at.z());
	return at;
}

Position Solver::ManipulabilityJump(const Position& from, const TourSolution& warm, int restart_in)
{
	BaseGradientParams p = params;	// reached viewpoints' comfort only; misses need no closest pose
	p.manipulability_weight = 1.0;
	p.softmin_manipulability_weight = params.jump_softmin_weight;
	p.manipulability_include_missed = false;
	p.travel_gradient = false;
	p.miss_gap_weight = 0.0;
	p.gaps_min_reached = std::numeric_limits<int>::max();
	auto score = [&](const TourSolution& s) { return s.sum_manipulability + p.softmin_manipulability_weight * s.softmin_manipulability; };
	auto log_step = [&](int iter, double step) {
		RCLCPP_INFO(
			node->get_logger(),
			"jump %d iter %d: abs (%.4f, %.4f, %.4f) m  reachable %d/%d  manipulability score %.4f (sum_w %.3f, soft-min "
			"%.4f)  step %.4f m",
			restart_in + 1, iter, position.x(), position.y(), position.z(), current.num_reachable, num_vp, score(current),
			current.sum_manipulability, current.softmin_manipulability, step);
	};
	restart = restart_in;
	seed = params.random_seed + restart;
	position = from;
	tracked_probes = true;
	seeds = SeedsFrom(warm, home, num_vp);
	ReseedRng(seed);
	current = SolveBestOf(position, p, &warm.tour, nullptr);
	log_step(0, 0.0);
	stall_count = 0;
	for (int iter = 0; iter < params.max_outer_iterations && rclcpp::ok(); ++iter)
	{
		const Eigen::Vector3d g = LockAxes(AnalyticGradient(current, p));
		if (g.norm() < 1e-9)
			break;
		seeds = SeedsFrom(current, home, num_vp);
		locked_order = current.tour;
		here = Probe(position, p);
		std::optional<Step> step;
		double size = params.initial_step;
		for (int ls = 0; ls < params.max_line_search_iters && !step; ++ls, size *= params.step_shrink)
		{
			const Position cand = Clamp(position - size * g.normalized(), params.bounds);
			TourSolution probe = Probe(cand, p);
			result.num_inner_solves += 1;
			if (params.trace_line_search)
				RCLCPP_INFO(
					node->get_logger(), "    jump probe step=%.4f m: reached %d, score %.4f vs %.4f here", size,
					probe.num_reachable, score(probe), score(here));
			if (score(probe) > score(here))
				step = Step{cand, std::move(probe), size};
		}
		if (!step)
			break;
		ReseedRng(seed);
		TourSolution committed = SolveBestOf(step->at, p, &step->sol.tour, nullptr);
		if (score(step->sol) > score(committed))
			committed = step->sol;
		if (score(committed) <= score(current))
			break;
		const double gain = (score(committed) - score(current)) / std::max(std::abs(score(current)), 1e-9);
		position = step->at;
		current = std::move(committed);
		log_step(iter + 1, step->size);
		if (gain >= params.convergence_tolerance_cost)
			stall_count = 0;
		else if (++stall_count >= std::max(1, params.patience))
			break;
	}
	RCLCPP_INFO(
		node->get_logger(), "jump %d: moved %.4f m from abs (%.4f, %.4f, %.4f) to (%.4f, %.4f, %.4f)", restart_in + 1,
		(position - from).norm(), from.x(), from.y(), from.z(), position.x(), position.y(), position.z());
	return position;
}

// Debug: log what refining right at the first all-reached spot would give.
void Solver::LogRefineAtReach()
{
	if (params.refine_at_reach <= 0 || !current.all_reachable || refined_at_reach)
		return;
	refined_at_reach = true;
	TourSolution start = current;
	if (use_planning && start.planned < 0.0)
		start.planned = PlannedTravel(position, start);
	const TourSolution refined = Refine(position, start, params.refine_at_reach);
	RCLCPP_INFO(
		node->get_logger(),
		"  restart %d: refined %d solves at first all-reached abs (%.4f, %.4f, %.4f) m: travel %.2f -> %.2f  "
		"real %.2f -> %.2f",
		restart + 1, params.refine_at_reach, position.x(), position.y(), position.z(), start.travel, refined.travel,
		start.planned, refined.planned);
}

// Refine solves at the descent's result, and where the remaining reconfigurations are.
void Solver::RefineResult()
{
	if (params.refine_solves <= 0 || best.sol.tour.empty())
		return;
	TourSolution start = best.sol;
	if (use_planning && start.planned < 0.0)
		start.planned = PlannedTravel(best.position, start);
	best.sol = Refine(best.position, start, params.refine_solves);
	best.cost = Cost(best.sol);
	best.ok = best.sol.all_reachable;
	const TourSolution& s = best.sol;
	std::string where;
	for (size_t k = 0; k < s.joints.size(); ++k)
	{
		const Joints& a = k == 0 ? home : s.joints[k - 1];
		const Joints& b = s.joints[k];
		if (MaxJointTurn(a, b) <= params.reconfig_threshold)
			continue;
		size_t jmax = 0;
		for (size_t j = 0; j < CountedJoints(a, b); ++j)
			if (std::abs(b[j] - a[j]) > std::abs(b[jmax] - a[jmax]))
				jmax = j;
		char buf[64];
		std::snprintf(
			buf, sizeof(buf), " %s->%d j%zu %+.2f", k == 0 ? "home" : std::to_string(s.tour[k - 1]).c_str(), s.tour[k],
			jmax + 1, b[jmax] - a[jmax]);
		where += buf;
	}
	int reordered = 0;
	for (size_t k = 0; k < s.tour.size(); ++k)
		reordered += (k >= start.tour.size() || start.tour[k] != s.tour[k]) ? 1 : 0;
	RCLCPP_INFO(
		node->get_logger(),
		"  restart %d: refined %d solves at final abs (%.4f, %.4f, %.4f) m: travel %.2f -> %.2f  real %.2f -> %.2f  "
		"order changed at %d/%zu positions  reconfigurations %d -> %d:%s",
		restart + 1, params.refine_solves, best.position.x(), best.position.y(), best.position.z(), start.travel,
		s.travel, start.planned, s.planned, reordered, s.tour.size(),
		CountReconfigs(home, start.joints, params.reconfig_threshold),
		CountReconfigs(home, s.joints, params.reconfig_threshold), where.empty() ? " none" : where.c_str());
}

// ---------------------------------------------------------------------------------------------
// Solving and scoring
// ---------------------------------------------------------------------------------------------

// IK solutions per viewpoint: one from its seed (stays near it), the rest from random starts. Tracking: one attempt
// from the seed; random ones only if it fails.
std::vector<std::vector<Joints>> Solver::CollectIk(
	const Position& at, const BaseGradientParams& p, const std::vector<Joints>& seed_q, int max_solutions)
{
	const Eigen::Isometry3d xform = Translation(at);
	MoveObject(scene, xform * object_pose);
	std::vector<std::vector<Joints>> ik(viewpoints.size());
	const int max_sol = std::max(1, max_solutions);
	for (size_t v = 0; v < viewpoints.size(); ++v)
	{
		const geometry_msgs::msg::Pose target = ToPoseMsg(xform * viewpoints[v]);
		std::vector<Joints>& sols = ik[v];
		Joints q;
		state.setJointGroupPositions(group, seed_q[v]);
		if (ExactIk(state, group, target, p.tracking ? 1 : p.ik_attempts, p.ik_timeout, validity))
		{
			state.copyJointGroupPositions(group, q);
			sols.push_back(q);
		}
		for (int attempt = 0; (!p.tracking || sols.empty()) && attempt < max_sol * 3 + p.ik_retries_per_point &&
			 static_cast<int>(sols.size()) < max_sol;
			 ++attempt)
		{
			if (sols.empty() && attempt >= 4)
				break;	// out of reach: every miss costs a full IK call
			state.setToRandomPositions(group, SeededRng());
			if (!ExactIk(state, group, target, p.ik_attempts, p.ik_timeout, validity))
				continue;
			state.copyJointGroupPositions(group, q);
			const bool dup = std::any_of(sols.begin(), sols.end(), [&](const Joints& s) { return JointDistance(s, q) < 0.1; });
			if (!dup)
				sols.push_back(q);
		}
	}
	return ik;
}

// The tour at `at`: IK solutions, closest poses for the misses, then the GTSP (or the fixed order), scored.
TourSolution Solver::Solve(
	const Position& at, const BaseGradientParams& p, const std::vector<Joints>& seed_q,
	const std::vector<int>* warm_order, int max_solutions, const std::vector<int>* fixed_order)
{
	std::vector<std::vector<Joints>> ik = CollectIk(at, p, seed_q, max_solutions);
	const double exact_gap = 1e-4;	// 0.1 mm: a closest pose this near counts as an IK solution
	const Eigen::Isometry3d xform = Translation(at);
	const int num_found = static_cast<int>(std::count_if(ik.begin(), ik.end(), [](const auto& s) { return !s.empty(); }));
	const bool skip_gaps = num_found < p.gaps_min_reached;	// rejected on count anyway
	std::vector<Miss> missed;
	for (size_t v = 0; v < ik.size(); ++v)
	{
		if (!ik[v].empty())
			continue;
		if (skip_gaps)
		{
			missed.push_back({static_cast<int>(v), CappedGap(p.miss_gap_cap), {}});
			continue;
		}
		const Eigen::Isometry3d target = xform * viewpoints[v];
		// Starts: the nearest reached viewpoint's pose (often the right posture family), then its own seed; random
		// starts only while no collision-free pose is under the cap (a capped gap has no slope).
		Joints neighbor;
		double neighbor_dist = std::numeric_limits<double>::max();
		for (size_t u = 0; u < ik.size() && !p.tracking; ++u)
		{
			if (u == v || ik[u].empty())
				continue;
			const Eigen::Isometry3d other = xform * viewpoints[u];
			const double d = (other.translation() - target.translation()).norm() +
				p.rot_metric_scale * Eigen::AngleAxisd(other.linear() * target.linear().transpose()).angle();
			if (d < neighbor_dist)
			{
				neighbor_dist = d;
				neighbor = ik[u].front();
			}
		}
		const int fixed_starts = neighbor.empty() ? 1 : 2;
		ClosestIk closest;
		for (int t = 0; t < std::max(fixed_starts, p.closest_ik_starts) &&
			 (t == 0 || !closest.found || closest.gap >= p.miss_gap_cap);
			 ++t)
		{
			Joints start = seed_q[v];
			if (t == 0 && fixed_starts == 2)
				start = neighbor;
			else if (t > 0 && !(t == 1 && fixed_starts == 2))
			{
				state.setToRandomPositions(group, SeededRng());
				state.copyJointGroupPositions(group, start);
			}
			const ClosestIk r = ClosestIkPose(
				scene, state, group, tool, target, start, p.closest_ik_iters, p.closest_ik_margin, p.rot_metric_scale,
				p.manipulability_limit_sharpness);
			if (r.found && r.gap < closest.gap)
				closest = r;
		}
		if (closest.found && closest.gap < exact_gap)
			ik[v].push_back(closest.q);  // IK just missed it
		else
			missed.push_back({static_cast<int>(v), closest.found ? closest.pose_error : CappedGap(p.miss_gap_cap), closest.q});
	}

	const double k = p.manipulability_limit_sharpness;
	std::vector<std::vector<double>> node_cost(ik.size());
	for (size_t v = 0; v < ik.size(); ++v)
		for (const Joints& q : ik[v])
			node_cost[v].push_back(-p.manipulability_weight * Manipulability(state, group, tool, q, k));
	TourSolution out;
	if (fixed_order)
	{
		for (int v : *fixed_order)
			if (!ik[static_cast<size_t>(v)].empty())
				out.tour.push_back(v);
		InsertByDetour(out.tour, ik, home);
		out.joints = BestPosesForOrder(ik, node_cost, out.tour, home, p);
	}
	else
		std::tie(out.tour, out.joints) = FindTour(ik, home, p, warm_order, node_cost);
	out.num_total = num_vp;
	out.num_reachable = static_cast<int>(out.tour.size());
	out.all_reachable = out.num_reachable == out.num_total;
	out.travel = TravelCost(home, out.joints, p);

	// Comfort: manipulability over the tour poses (and the misses' closest poses, if included).
	std::vector<double> ws;
	double sum_tour = 0.0, sum_missed = 0.0;
	for (const Joints& q : out.joints)
	{
		ws.push_back(Manipulability(state, group, tool, q, k));
		sum_tour += ws.back();
	}
	for (const Miss& m : missed)
		if (p.manipulability_include_missed && !m.q.empty())
		{
			ws.push_back(Manipulability(state, group, tool, m.q, k));
			sum_missed += ws.back();
		}
	out.sum_manipulability = p.manipulability_include_missed ? sum_tour + sum_missed : sum_tour;
	if (p.softmin_manipulability_weight > 0.0)
		out.softmin_manipulability = SoftMin(ws, p.softmin_tau);

	out.missed = std::move(missed);
	for (const Miss& m : out.missed)
		out.gap_sum += std::min(p.miss_gap_cap, PoseGap(m.gap, p.rot_metric_scale));
	out.miss_cost = p.miss_gap_weight * out.gap_sum;
	out.cost = -p.manipulability_weight * out.sum_manipulability - p.softmin_manipulability_weight * out.softmin_manipulability +
		p.unreachable_penalty * (out.num_total - out.num_reachable) + out.miss_cost;
	return out;
}

// The best of gtsp_num_restart solves (each draws new random IK starts).
TourSolution Solver::SolveBestOf(
	const Position& at, const BaseGradientParams& p, const std::vector<int>* warm_order, const std::vector<int>* fixed_order)
{
	const int n = std::max(1, p.gtsp_num_restart);
	result.num_inner_solves += n;
	TourSolution best_sol = Solve(at, p, seeds, warm_order, p.max_solutions_per_candidate, fixed_order);
	for (int i = 1; i < n; ++i)
	{
		TourSolution s = Solve(at, p, seeds, warm_order, p.max_solutions_per_candidate, fixed_order);
		if (s.cost < best_sol.cost)
			best_sol = std::move(s);
	}
	return best_sol;
}

// Quick solve for comparing nearby spots. Tracked (reach phase): each viewpoint follows its current pose with one IK
// attempt in a fixed order. A probe reaching fewer than gaps_min_reached is rejected anyway, so it skips miss gaps.
TourSolution Solver::Probe(const Position& at, int gaps_min_reached)
{
	BaseGradientParams p = params;
	p.gaps_min_reached = gaps_min_reached;
	return Probe(at, p);
}

TourSolution Solver::Probe(const Position& at, BaseGradientParams p)
{
	ReseedRng(seed);
	if (!tracked_probes)
		return Solve(at, p, seeds, &current.tour, params.max_solutions_per_candidate, order_lock);
	p.tracking = true;
	return Solve(at, p, seeds, &current.tour, 1, &locked_order);
}

// More full solves at a fixed spot, each warm-started from the best so far, with more IK choices and GTSP edges that
// penalize reconfigurations. Keeps the lowest cost; never fewer viewpoints reached.
TourSolution Solver::Refine(const Position& at, TourSolution best_sol, int num_solves)
{
	if (use_planning && best_sol.planned < 0.0)
		best_sol.planned = PlannedTravel(at, best_sol);
	best_sol.cost = Cost(best_sol);
	BaseGradientParams p = params;
	p.edge_reconfig_weight = params.reconfig_weight;
	for (int i = 0; i < num_solves && rclcpp::ok(); ++i)
	{
		const std::vector<int>* lock = params.lock_input_order ? &input_order
			: params.freeze_order								 ? &best_sol.tour
																 : nullptr;
		TourSolution s =
			Solve(at, p, SeedsFrom(best_sol, home, num_vp), &best_sol.tour, params.refine_ik_solutions, lock);
		result.num_inner_solves += 1;
		if (s.num_reachable < best_sol.num_reachable)
			continue;
		s.travel = TravelCost(home, s.joints, params);	// without the penalty, comparable with the descent's
		if (use_planning)
			s.planned = PlannedTravel(at, s);
		s.cost = Cost(s);
		if (s.cost < best_sol.cost)
			best_sol = std::move(s);
	}
	return best_sol;
}

// Planned joint travel: every edge planned with OMPL. A failed edge counts its straight-line distance plus
// unreachable_penalty.
double Solver::PlannedTravel(const Position& at, const TourSolution& sol)
{
	PlaceObject(at);
	const std::vector<double> edges = PlanTourJointPathLengths(
		node, robot_model, scene, sol.joints, home, group_name, params.real_cost_planning_time, params.real_cost_attempts,
		/*count_last_joint=*/false);
	double total = 0.0;
	for (size_t k = 0; k < sol.joints.size(); ++k)
		total += (k < edges.size() && edges[k] >= 0.0)
			? edges[k]
			: JointDistance(k == 0 ? home : sol.joints[k - 1], sol.joints[k]) + params.unreachable_penalty;
	return total;
}

// Adds planned travel to the cost, only once every viewpoint is reached (before that comparisons go by count and gap).
TourSolution Solver::WithPlannedTravel(const Position& at, TourSolution sol)
{
	if (!planned_travel || !sol.all_reachable || sol.planned >= 0.0)
		return sol;
	sol.planned = PlannedTravel(at, sol);
	sol.cost += sol.planned;
	return sol;
}

double Solver::TravelTerm(const TourSolution& sol) const
{
	return planned_travel && sol.planned >= 0.0 ? sol.planned : 0.0;
}

double Solver::Cost(const TourSolution& sol) const
{
	return TravelTerm(sol) - params.manipulability_weight * sol.sum_manipulability -
		params.softmin_manipulability_weight * sol.softmin_manipulability +
		params.unreachable_penalty * (sol.num_total - sol.num_reachable) + sol.miss_cost;
}

// More viewpoints reached wins; cost breaks ties. While either side misses one, planned travel is left out: it jumps
// when an arm pose switches family, and the gradient doesn't see it.
bool Solver::Better(const TourSolution& a, const TourSolution& b) const
{
	const bool reach_phase = !a.all_reachable || !b.all_reachable;
	auto cost = [&](const TourSolution& s) { return reach_phase && s.planned >= 0.0 ? s.cost - s.planned : s.cost; };
	return a.num_reachable > b.num_reachable || (a.num_reachable == b.num_reachable && cost(a) < cost(b));
}

// History entry, and the descent's best (most viewpoints reached, then cost).
void Solver::Record(const Position& at, const TourSolution& sol)
{
	TourSolution s = sol;
	s.cost = Cost(sol);
	result.history.push_back({static_cast<double>(restart), at.x(), at.y(), at.z(), s.cost});
	if (s.num_reachable > best.sol.num_reachable || (s.num_reachable == best.sol.num_reachable && s.cost < best.cost))
	{
		best.position = at;
		best.cost = s.cost;
		best.ok = s.all_reachable;
		best.sol = std::move(s);
	}
}

void Solver::Remember(const TourSolution& sol)
{
	for (size_t k = 0; k < sol.tour.size(); ++k)
	{
		last_reached[static_cast<size_t>(sol.tour[k])] = sol.joints[k];
		last_closest[static_cast<size_t>(sol.tour[k])].clear();
	}
	for (const Miss& m : sol.missed)
		if (!m.q.empty())
			last_closest[static_cast<size_t>(m.vp)] = m.q;
}

// Cost gradient w.r.t. the object position. Each pose follows its viewpoint as dq/dp = J^+ (the home pose doesn't
// move); travel edges, comfort terms and miss gaps chain through that.
Eigen::Vector3d Solver::AnalyticGradient(const TourSolution& sol, const BaseGradientParams& p)
{
	const double k = p.manipulability_limit_sharpness;
	const size_t n = sol.tour.size();
	auto dq_dp = [&](const Joints& q) { return Eigen::MatrixXd(DampedPinv(Jacobian(q), p.jacobian_damping).leftCols<3>()); };
	std::vector<Eigen::MatrixXd> dq(n);
	for (size_t i = 0; i < n; ++i)
		dq[i] = dq_dp(sol.joints[i]);

	Eigen::Vector3d g = Eigen::Vector3d::Zero();
	std::vector<double> ws;
	std::vector<Eigen::Vector3d> dw;
	auto add_comfort = [&](const Joints& q, const Eigen::MatrixXd& dq_q) {
		const double w = std::max(1e-6, Manipulability(state, group, tool, q, k));
		const Eigen::Vector3d dw_dp = dq_q.transpose() * ManipulabilityGradient(state, group, tool, q, k);
		g -= p.manipulability_weight * dw_dp;
		ws.push_back(w);
		dw.push_back(dw_dp);
	};
	for (size_t i = 0; i < n && p.manipulability_gradient; ++i)
		add_comfort(sol.joints[i], dq[i]);
	for (const Miss& m : sol.missed)  // the misses' closest poses, as if the arm followed the viewpoint
		if (p.manipulability_include_missed && !m.q.empty())
		{
			const Eigen::MatrixXd dq_m = dq_dp(m.q);
			if (p.manipulability_gradient)
				add_comfort(m.q, dq_m);
		}
	if (p.softmin_manipulability_weight > 0.0 && !ws.empty())
	{
		std::vector<double> shares;
		SoftMin(ws, p.softmin_tau, &shares);
		for (size_t i = 0; i < shares.size(); ++i)
			g -= p.softmin_manipulability_weight * shares[i] * dw[i];
	}
	// Miss gaps: the viewpoint moves with the object; no slope past the cap.
	for (const Miss& m : sol.missed)
	{
		const double gap = PoseGap(m.gap, p.rot_metric_scale);
		if (gap >= 1e-9 && gap < p.miss_gap_cap)
			g += p.miss_gap_weight * Eigen::Vector3d(m.gap.head<3>()) / gap;
	}
	if (!p.travel_gradient)
		return g;
	const size_t dof = home.size();
	const Eigen::MatrixXd fixed = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(dof), 3);
	for (size_t i = 0; i < n; ++i)
	{
		const Joints& qa = i == 0 ? home : sol.joints[i - 1];
		const Joints& qb = sol.joints[i];
		Eigen::VectorXd diff = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof));
		for (size_t j = 0; j < CountedJoints(qa, qb); ++j)
			diff(static_cast<Eigen::Index>(j)) = qa[j] - qb[j];
		const Eigen::MatrixXd d_diff = (i == 0 ? fixed : dq[i - 1]) - dq[i];
		const double nrm = diff.norm();
		if (nrm > 1e-9)
			g += p.joint_distance_weight * (d_diff.transpose() * (diff / nrm));
		Eigen::Index jmax = 0;
		for (Eigen::Index j = 1; j < diff.size(); ++j)
			if (std::abs(diff(j)) > std::abs(diff(jmax)))
				jmax = j;
		g += p.max_joint_deviation_weight * (diff(jmax) >= 0.0 ? 1.0 : -1.0) * d_diff.row(jmax).transpose();
	}
	return g;
}

// Debug: central-difference gradient by re-tracking the tour's poses (one IK per viewpoint) at +-epsilon.
Eigen::Vector3d Solver::FiniteDifferenceGradient()
{
	const double k = params.manipulability_limit_sharpness;
	auto tracked_cost = [&](const Position& at, bool* all) {
		const Eigen::Isometry3d xform = Translation(at);
		MoveObject(scene, xform * object_pose);
		double cost = 0.0;
		const Joints* prev = &home;
		bool prev_ok = true;
		std::vector<Joints> qs(current.tour.size());
		*all = true;
		for (size_t i = 0; i < current.tour.size(); ++i)
		{
			state.setJointGroupPositions(group, current.joints[i]);
			if (!ExactIk(
					state, group, ToPoseMsg(xform * viewpoints[static_cast<size_t>(current.tour[i])]), params.ik_attempts,
					params.ik_timeout, validity))
			{
				prev_ok = *all = false;
				continue;
			}
			state.copyJointGroupPositions(group, qs[i]);
			cost -= params.manipulability_weight * Manipulability(state, group, tool, qs[i], k);
			if (prev_ok)
				cost += EdgeCost(*prev, qs[i], params);
			prev = &qs[i];
			prev_ok = true;
		}
		return cost;
	};
	Eigen::Vector3d g = Eigen::Vector3d::Constant(std::nan(""));
	for (int axis = 0; axis < 3; ++axis)
	{
		const Eigen::Vector3d e = params.fd_epsilon * Eigen::Vector3d::Unit(axis);
		bool all_plus, all_minus;
		const double plus = tracked_cost(position + e, &all_plus);
		const double minus = tracked_cost(position - e, &all_minus);
		if (all_plus && all_minus)
			g(axis) = (plus - minus) / (2.0 * params.fd_epsilon);
	}
	return g;
}

// ---------------------------------------------------------------------------------------------
// Room-then-reach
// ---------------------------------------------------------------------------------------------

// Locked axes (min == max) get no share of a step.
Eigen::Vector3d Solver::LockAxes(Eigen::Vector3d v) const
{
	const BaseGradientBounds& b = params.bounds;
	if (b.x_min == b.x_max)
		v(0) = 0.0;
	if (b.y_min == b.y_max)
		v(1) = 0.0;
	if (b.z_min == b.z_max)
		v(2) = 0.0;
	return v;
}

// Unit direction that shrinks the target's position gap (zero if none). first: the full solve's gap if nearer.
Eigen::Vector3d Solver::AimAt(int target, const TourSolution& here_at, bool first)
{
	auto gap_of = [&](const TourSolution& s) -> const Vector6d* {
		for (const Miss& m : s.missed)
			if (m.vp == target)
				return &m.gap;
		return nullptr;
	};
	const Vector6d* gap = gap_of(here_at);
	if (!gap)
		return Eigen::Vector3d::Zero();
	const Vector6d* full = first ? gap_of(current) : nullptr;
	if (full && PoseGap(*full, params.rot_metric_scale) < PoseGap(*gap, params.rot_metric_scale))
		gap = full;
	const Eigen::Vector3d aim = -LockAxes(gap->head<3>());
	return aim.norm() < 1e-9 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(aim.normalized());
}

// Smallest singular value of the tool Jacobian: 0 when fully stretched or folded.
double Solver::Stretch(const Joints& q)
{
	return Eigen::JacobiSVD<Eigen::MatrixXd>(Jacobian(q)).singularValues().minCoeff();
}

double Solver::ShoulderToWrist(const Joints& q)
{
	state.setJointGroupPositions(group, q);
	state.update();
	return (state.getGlobalLinkTransform(wrist).translation() - state.getGlobalLinkTransform(shoulder).translation())
		.norm();
}

ArmModel Solver::ArmModelAt(const Position& at, const TourSolution& sol)
{
	PlaceObject(at);
	ArmModel model;
	model.q.resize(static_cast<size_t>(num_vp));
	model.dq_dp.resize(static_cast<size_t>(num_vp));
	for (size_t k = 0; k < sol.tour.size(); ++k)
	{
		const Joints& q = sol.joints[k];
		if (q.empty())
			continue;
		const size_t v = static_cast<size_t>(sol.tour[k]);
		model.q[v] = q;
		model.dq_dp[v] = DampedPinv(Jacobian(q), params.jacobian_damping).leftCols<3>();
		const std::vector<Margin> m = ViewpointMargins(sol.tour[k], q);
		model.margins.insert(model.margins.end(), m.begin(), m.end());
	}
	return model;
}

// Viewpoint v's margins at pose q (object already placed): joint limits, stretch, reach, collision pairs within
// 10 cm; slopes with the arm following the viewpoint.
std::vector<Margin> Solver::ViewpointMargins(int v, const Joints& q)
{
	const double kJointFloor = 0.05;	 // rad
	const double kStretchFloor = 0.02;
	const double kReachFloor = 0.01;	 // m short of full stretch
	const double kCollisionFloor = 0.01;  // m
	const double kWindow = 0.1;			 // m
	const double h = 1e-5;				 // rad, finite-difference step
	const int dof = static_cast<int>(q.size());
	const Eigen::MatrixXd dq_dp = DampedPinv(Jacobian(q), params.jacobian_damping).leftCols<3>();
	std::vector<Margin> margins;
	auto add = [&](const std::string& name, double value, double floor, const Eigen::VectorXd& joint_slope,
				   const Eigen::Vector3d& direct) {
		margins.push_back({v, name, value, floor, LockAxes(dq_dp.transpose() * joint_slope + direct), joint_slope});
	};
	for (int i = 0; i < dof; ++i)
	{
		const moveit::core::VariableBounds& b = robot_model->getVariableBounds(group->getVariableNames()[static_cast<size_t>(i)]);
		if (!b.position_bounded_)
			continue;
		const Eigen::VectorXd e = Eigen::VectorXd::Unit(dof, i);
		const std::string joint = "joint " + std::to_string(i + 1);
		add(joint + " low limit", q[static_cast<size_t>(i)] - b.min_position_, kJointFloor, e, Eigen::Vector3d::Zero());
		add(joint + " high limit", b.max_position_ - q[static_cast<size_t>(i)], kJointFloor, -e, Eigen::Vector3d::Zero());
	}
	auto slope_of = [&](auto f) {
		const double f0 = f(q);
		Eigen::VectorXd g(dof);
		for (int i = 0; i < dof; ++i)
		{
			Joints qh = q;
			qh[static_cast<size_t>(i)] += h;
			g(i) = (f(qh) - f0) / h;
		}
		return g;
	};
	add("stretch", Stretch(q), kStretchFloor, slope_of([this](const Joints& x) { return Stretch(x); }),
		Eigen::Vector3d::Zero());
	if (shoulder && wrist)
		add("reach", max_reach - ShoulderToWrist(q), kReachFloor,
			-slope_of([this](const Joints& x) { return ShoulderToWrist(x); }), Eigen::Vector3d::Zero());
	state.setJointGroupPositions(group, q);
	state.update();
	collision_detection::DistanceRequest req = CloseRequest(state, group, kWindow);
	const ClosePairs pairs = FindClosePairs(scene, state, group, req, kWindow);
	for (size_t i = 0; i < pairs.dists.size(); ++i)
		add("collision " + pairs.names[i], pairs.dists[i], kCollisionFloor, pairs.gradients[i].transpose(),
			pairs.object_slopes[i]);
	return margins;
}

Joints Solver::PredictedPose(const ArmModel& model, int v, const Eigen::Vector3d& d)
{
	Joints q = model.q[static_cast<size_t>(v)];
	const Eigen::VectorXd dq = model.dq_dp[static_cast<size_t>(v)] * d;
	for (size_t i = 0; i < q.size(); ++i)
		q[i] += dq(static_cast<Eigen::Index>(i));
	return q;
}

// Newton steps from q onto viewpoint v's exact pose at `at` (limits ignored); empty if it doesn't converge.
std::optional<Joints> Solver::ExactPose(int v, const Position& at, Joints q)
{
	const Eigen::Isometry3d target = Translation(at) * viewpoints[static_cast<size_t>(v)];
	for (int it = 0; it < 50; ++it)
	{
		state.setJointGroupPositions(group, q);
		state.update();
		const Vector6d e = PoseError(target, state.getGlobalLinkTransform(tool));
		if (e.head<3>().norm() < 1e-4 && e.tail<3>().norm() < 1e-3)
			return q;
		const Eigen::VectorXd dq = DampedPinv(Jacobian(q), params.jacobian_damping) * e;
		for (size_t i = 0; i < q.size(); ++i)
			q[i] += dq(static_cast<Eigen::Index>(i));
	}
	return std::nullopt;
}

// What ran out for viewpoint v, lost by moving the object by d from `at`: its exact pose after the move is measured,
// and its margins under their floor there are returned (as modeled at `at`). No exact pose: out of reach (reach and
// stretch). Nothing found: the margins predicted to run out along d.
Shortage Solver::RanOut(int v, const ArmModel& model, const Position& at, const Eigen::Vector3d& d)
{
	const Position cand = Clamp(at + d, params.bounds);
	const Eigen::Vector3d moved = cand - at;
	const std::optional<Joints> q = ExactPose(v, cand, PredictedPose(model, v, moved));
	Shortage out;
	auto take = [&](const std::string& name) {
		for (const Margin& m : model.margins)
			if (m.vp == v && m.name == name)
				out.margins.push_back(m);
	};
	if (!q)
	{
		out.why = "out of reach";
		take("reach");
		take("stretch");
	}
	else
	{
		PlaceObject(cand);
		for (const Margin& after : ViewpointMargins(v, *q))
			if (after.value < after.floor)
			{
				out.why += (out.why.empty() ? "" : ", ") + after.name;
				take(after.name);
			}
		PlaceObject(at);
	}
	if (out.margins.empty())
	{
		out.why += out.why.empty() ? "predicted" : " (predicted)";
		for (const Margin& m : model.margins)
			if (m.vp == v && RoomAlong({m}, moved.normalized()) < moved.norm())
				out.margins.push_back(m);
	}
	return out;
}

// Viewpoint v's roomier IK solution at `at`, if it is near a limit: `tries` candidates, half from random starts, half
// sliding its pose along its self-motion (camera fixed) toward more room on its tightest margin. The best, if its
// tightest margin beats the current one by at least one floor.
std::optional<Joints> Solver::RoomierPose(int v, const Position& at, const ArmModel& model, int tries)
{
	const int resamples = tries / 2;
	const int nudges = tries - resamples;
	const double nudge_step = 0.1;	// rad: nudge i slides (i + 1) times this
	const std::vector<Margin> mine = model.Of(v);
	const double now = Tightest(mine);
	if (now >= 1.0)
		return std::nullopt;
	std::string tightest;
	Eigen::VectorXd raise;	// d(tightest margin)/dq
	for (const Margin& m : mine)
		if (m.value / m.floor == now)
		{
			char buf[96];
			std::snprintf(buf, sizeof(buf), "%s %.4f", m.name.c_str(), m.value);
			tightest = buf;
			raise = m.joint_slope;
		}
	PlaceObject(at);
	const geometry_msgs::msg::Pose target = ToPoseMsg(Translation(at) * viewpoints[static_cast<size_t>(v)]);
	double best_score = now + 1.0;
	std::optional<Joints> roomiest;
	std::string how;
	auto consider = [&](const Joints& q, const char* kind) {
		const double score = Tightest(ViewpointMargins(v, q));
		if (score > best_score)
		{
			best_score = score;
			roomiest = q;
			how = kind;
		}
	};
	for (int i = 0; i < resamples; ++i)
	{
		state.setToRandomPositions(group, SeededRng());
		if (!ExactIk(state, group, target, 1, params.ik_timeout, validity))
			continue;
		Joints q;
		state.copyJointGroupPositions(group, q);
		consider(q, "resampled");
	}
	const Joints& q0 = model.q[static_cast<size_t>(v)];
	const Eigen::MatrixXd J = Jacobian(q0);
	const Eigen::VectorXd slide =
		(Eigen::MatrixXd::Identity(J.cols(), J.cols()) - DampedPinv(J, params.jacobian_damping) * J) * raise;
	for (int i = 0; i < nudges && slide.norm() > 1e-9; ++i)
	{
		Joints q = q0;
		const Eigen::VectorXd dq = (i + 1) * nudge_step * slide.normalized();
		for (size_t j = 0; j < q.size(); ++j)
			q[j] += dq(static_cast<Eigen::Index>(j));
		const std::optional<Joints> exact = ExactPose(v, at, q);
		if (!exact)
			continue;
		state.setJointGroupPositions(group, *exact);
		if (state.satisfiesBounds(group) && IsStateValid(scene, &state, group, exact->data()))
			consider(*exact, "nudged");
	}
	if (params.trace_line_search)
		RCLCPP_INFO(
			node->get_logger(), "    vp %d near a limit (smallest margin %.2f floors, %s): %s", v, now, tightest.c_str(),
			roomiest ? (how + " IK with " + std::to_string(best_score) + " floors swapped in").c_str() : "no roomier IK");
	return roomiest;
}

// ---------------------------------------------------------------------------------------------
// Logging and progress markers
// ---------------------------------------------------------------------------------------------

void Solver::LogCostTerms(const TourSolution& sol)
{
	auto row = [&](const char* name, double raw, double weight) {
		RCLCPP_INFO(node->get_logger(), "    %-16s %12.4f %10.1f %12.2f", name, raw, weight, raw * weight);
	};
	RCLCPP_INFO(node->get_logger(), "    %-16s %12s %10s %12s", "term", "raw", "weight", "weighted");
	if (planned_travel)
		row("planned travel", TravelTerm(sol), 1.0);
	else
		row("L2 travel", sol.travel, 0.0);
	row("manipulability", sol.sum_manipulability, -params.manipulability_weight);
	row("soft-min manip", sol.softmin_manipulability, -params.softmin_manipulability_weight);
	row("missed count", static_cast<double>(sol.num_total - sol.num_reachable), params.unreachable_penalty);
	row("miss gap (m)", sol.gap_sum, params.miss_gap_weight);
	RCLCPP_INFO(node->get_logger(), "    %-16s %12s %10s %12.2f", "total cost", "", "", sol.cost);
}

// Per missed viewpoint, what blocks its closest pose: gap, joints near a limit, closest pairs.
void Solver::LogMissed(const Position& at, const TourSolution& sol)
{
	auto closest = [](const collision_detection::DistanceResult& r) {
		const auto& d = r.minimum_distance;
		if (d.distance > 1e3)
			return std::string("none within 1 m");
		char buf[160];
		std::snprintf(
			buf, sizeof(buf), "%s - %s %.1f mm", d.link_names[0].c_str(), d.link_names[1].c_str(), d.distance * 1000.0);
		return std::string(buf);
	};
	PlaceObject(at);
	for (const Miss& m : sol.missed)
	{
		if (m.q.empty())
		{
			RCLCPP_INFO(node->get_logger(), "    missed vp %2d: no collision-free closest pose", m.vp);
			continue;
		}
		std::string limits;
		for (size_t k = 0; k < m.q.size(); ++k)
		{
			const moveit::core::VariableBounds& b = robot_model->getVariableBounds(group->getVariableNames()[k]);
			const double range = b.max_position_ - b.min_position_;
			if (!b.position_bounded_ || range <= 0.0)
				continue;
			if (m.q[k] - b.min_position_ < 0.02 * range)
				limits += " j" + std::to_string(k + 1) + "@min";
			else if (b.max_position_ - m.q[k] < 0.02 * range)
				limits += " j" + std::to_string(k + 1) + "@max";
		}
		state.setJointGroupPositions(group, m.q);
		state.update();
		collision_detection::DistanceRequest req = CloseRequest(state, group, 1.0);
		const auto [self_res, world_res] = Distances(scene, state, req);
		RCLCPP_INFO(
			node->get_logger(), "    missed vp %2d: gap %.1f mm %.1f deg | near limit:%s | arm-object %s | arm-arm %s", m.vp,
			m.gap.head<3>().norm() * 1000.0, m.gap.tail<3>().norm() * 180.0 / M_PI, limits.empty() ? " none" : limits.c_str(),
			closest(world_res).c_str(), closest(self_res).c_str());
	}
}

visualization_msgs::msg::Marker MakeMarker(const std::string& ns, int type, const rclcpp::Time& stamp, int id = 0)
{
	visualization_msgs::msg::Marker m;
	m.header.frame_id = "world";
	m.header.stamp = stamp;
	m.ns = ns;
	m.id = id;
	m.type = type;
	m.action = visualization_msgs::msg::Marker::ADD;
	m.pose.orientation.w = 1.0;
	return m;
}

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a)
{
	std_msgs::msg::ColorRGBA c;
	c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
	return c;
}

// Live markers: object mesh, trail of past positions, -gradient arrow, viewpoints (green reached, red missed), and
// each miss's closest pose with a line to its target. The previous publish is cleared first.
void Solver::Publish(const Eigen::Vector3d& neg_grad)
{
	using visualization_msgs::msg::Marker;
	if (!params.progress_pub)
		return;
	const rclcpp::Time stamp = node->now();
	const Eigen::Isometry3d xform = Translation(position);
	const Eigen::Vector3d object_at = (xform * object_pose).translation();
	visualization_msgs::msg::MarkerArray markers;
	Marker clear;
	clear.action = Marker::DELETEALL;
	markers.markers.push_back(clear);

	if (!params.progress_mesh_path.empty())
	{
		Marker mesh = MakeMarker("base_gradient_current", Marker::MESH_RESOURCE, stamp);
		mesh.mesh_resource = "file://" + params.progress_mesh_path;
		mesh.mesh_use_embedded_materials = false;
		mesh.pose = ToPoseMsg(xform * object_pose);
		mesh.scale.x = mesh.scale.y = mesh.scale.z = params.progress_mesh_scale;
		mesh.color = Rgba(1.0f, 0.85f, 0.0f, 0.6f);
		markers.markers.push_back(mesh);
	}

	Marker vps = MakeMarker("base_gradient_viewpoints", Marker::SPHERE_LIST, stamp);
	vps.scale.x = vps.scale.y = vps.scale.z = 0.01;
	for (size_t v = 0; v < viewpoints.size(); ++v)
	{
		const bool missed = std::any_of(
			current.missed.begin(), current.missed.end(), [&](const Miss& m) { return m.vp == static_cast<int>(v); });
		vps.points.push_back(ToPoint((xform * viewpoints[v]).translation()));
		vps.colors.push_back(missed ? Rgba(0.9f, 0.1f, 0.1f, 1.0f) : Rgba(0.1f, 0.9f, 0.1f, 1.0f));
	}
	markers.markers.push_back(vps);

	if (trail.size() > 1)
	{
		Marker line = MakeMarker("base_gradient_trail", Marker::LINE_STRIP, stamp);
		line.scale.x = 0.004;
		line.color = Rgba(1.0f, 0.4f, 0.1f, 0.8f);
		Marker crumbs = MakeMarker("base_gradient_history", Marker::SPHERE_LIST, stamp);
		crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.012;
		crumbs.color = line.color;
		for (const Position& p : trail)
		{
			line.points.push_back(ToPoint((Translation(p) * object_pose).translation()));
			crumbs.points.push_back(line.points.back());
		}
		markers.markers.push_back(line);
		markers.markers.push_back(crumbs);
	}

	const double gnorm = neg_grad.norm();
	if (gnorm > 1e-9)
	{
		Marker arrow = MakeMarker("base_gradient_descent_dir", Marker::ARROW, stamp);
		arrow.scale.x = 0.006;
		arrow.scale.y = 0.014;
		arrow.color = Rgba(0.1f, 0.5f, 1.0f, 0.95f);
		arrow.points = {ToPoint(object_at), ToPoint(object_at + neg_grad * (0.15 / gnorm))};
		markers.markers.push_back(arrow);
	}

	Marker gaps = MakeMarker("base_gradient_miss_gap", Marker::LINE_LIST, stamp);
	gaps.scale.x = 0.003;
	gaps.color = Rgba(1.0f, 0.2f, 1.0f, 1.0f);
	visualization_msgs::msg::MarkerArray arms;
	moveit::core::RobotState ghost(state);
	for (const Miss& m : current.missed)
	{
		if (m.q.empty())
			continue;
		ghost.setJointGroupPositions(group, m.q);
		ghost.update();
		ghost.getRobotMarkers(
			arms, ghost.getRobotModel()->getLinkModelNamesWithCollisionGeometry(), Rgba(0.9f, 0.2f, 0.2f, 0.35f),
			"base_gradient_miss_ik", rclcpp::Duration(0, 0));
		gaps.points.push_back(ToPoint(ghost.getGlobalLinkTransform("tool0").translation()));
		gaps.points.push_back(ToPoint((xform * viewpoints[static_cast<size_t>(m.vp)]).translation()));
	}
	if (!gaps.points.empty())
		markers.markers.push_back(gaps);
	for (Marker& m : arms.markers)
	{
		m.header.frame_id = "world";
		m.header.stamp = stamp;
		markers.markers.push_back(m);
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
	const BaseGradientParams& params)
{
	Solver solver(
		node, robot_model, planning_scene_monitor, group_name, object_translation_nominal, object_rotation_nominal,
		tour_tcp_poses_nominal, start_reference_joints, params);
	const int n = static_cast<int>(tour_tcp_poses_nominal.size());
	auto start_or_nominal = [&](double v, int axis) { return std::isnan(v) ? object_translation_nominal(axis) : v; };
	const Position start(
		start_or_nominal(params.initial_x, 0), start_or_nominal(params.initial_y, 1), start_or_nominal(params.initial_z, 2));

	// Restarts: from the best so far, a manipulability jump, then a new descent with a new IK seed. Stop once every
	// viewpoint is reached.
	const int num_restarts = std::max(1, params.descent_num_restart);
	DescentResult overall;
	for (int r = 0; r < num_restarts && rclcpp::ok(); ++r)
	{
		const Position from = r == 0 ? start
			: params.reach_map_jump	 ? solver.ReachabilityJump(overall.position, r)
									 : solver.ManipulabilityJump(overall.position, overall.sol, r);
		const DescentResult d = solver.RunDescent(from, r, r == 0 ? nullptr : &overall.sol);
		const bool improved = r == 0 || d.sol.num_reachable > overall.sol.num_reachable ||
			(d.sol.num_reachable == overall.sol.num_reachable && d.cost < overall.cost);
		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d done: total cost=%.4f  travel+miss=%.2f  sum_w=%.3f  reachable %d/%d  abs (%.4f, %.4f, %.4f) m%s",
			r + 1, num_restarts, d.cost, d.sol.travel + params.unreachable_penalty * (n - d.sol.num_reachable),
			d.sol.sum_manipulability, d.sol.num_reachable, n, d.position.x(), d.position.y(), d.position.z(),
			(r > 0 && improved) ? "  <-- new best" : "");
		if (improved)
			overall = d;
		if (overall.ok)
			break;
	}

	BaseGradientResult result = solver.result;
	const TourSolution& fin = overall.sol;
	result.x = overall.position.x();
	result.y = overall.position.y();
	result.z = overall.position.z();
	result.tour_order = fin.tour;
	result.joint_solutions = fin.joints;
	result.total_weighted_cost = fin.cost - params.unreachable_penalty * (fin.num_total - fin.num_reachable);
	for (size_t k = 0; k < fin.joints.size(); ++k)
		result.total_joint_path_length += JointDistance(k == 0 ? start_reference_joints : fin.joints[k - 1], fin.joints[k]);
	result.num_reachable = fin.num_reachable;
	result.ok = overall.ok;

	MoveObject(planning_scene_monitor, MakeIsometry(object_translation_nominal, object_rotation_nominal));
	if (result.ok)
		RCLCPP_INFO(
			node->get_logger(),
			"Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, tour joint path %.4f rad, travel %.2f, sum_w "
			"%.3f (weighted cost %.4f).",
			result.x, result.y, result.z, n, result.total_joint_path_length, fin.travel, fin.sum_manipulability,
			result.total_weighted_cost);
	else
		RCLCPP_WARN(
			node->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses.", result.x, result.y,
			result.z, result.num_reachable, n);
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
	root["total_joint_path_length"] = result.total_joint_path_length;
	root["total_weighted_cost"] = result.total_weighted_cost;
	Json::Value tour(Json::arrayValue);
	for (int v : result.tour_order)
		tour.append(v);
	root["tour_order"] = tour;
	Json::Value joints(Json::arrayValue);
	for (const auto& q : result.joint_solutions)
	{
		Json::Value entry(Json::arrayValue);
		for (double x : q)
			entry.append(x);
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
		entry["weighted_cost"] = h[4];
		history.append(entry);
	}
	root["history"] = history;

	const std::string path = output_dir + "/base_gradient_result.json";
	std::ofstream file(path);
	Json::StreamWriterBuilder builder;
	builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter>(builder.newStreamWriter())->write(root, &file);
	printf("Saved base gradient result JSON: %s\n", path.c_str());
}

Eigen::Isometry3d PlacementTransform(const Eigen::Vector3d& object_translation_nominal, double x, double y, double z)
{
	return Translation(Position(x, y, z)) * Eigen::Translation3d(-object_translation_nominal);
}

void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal, double x, double y, double z)
{
	MoveObject(planning_scene_monitor, MakeIsometry(Position(x, y, z), object_rotation_nominal));
}

visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const BaseGradientResult& result)
{
	using visualization_msgs::msg::Marker;
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;
	const Eigen::Isometry3d xform = PlacementTransform(object_translation_nominal, result.x, result.y, result.z);

	Marker mesh = MakeMarker("base_gradient_object", Marker::MESH_RESOURCE, stamp, id++);
	mesh.mesh_resource = "file://" + resolved_mesh_path;
	mesh.mesh_use_embedded_materials = false;
	mesh.pose = ToPoseMsg(xform * MakeIsometry(object_translation_nominal, object_rotation_nominal));
	mesh.scale.x = mesh.scale.y = mesh.scale.z = mesh_scale;
	mesh.color = Rgba(0.7f, 0.7f, 0.7f, 0.5f);
	markers.markers.push_back(mesh);

	Marker line = MakeMarker("base_gradient_tour", Marker::LINE_STRIP, stamp, id++);
	line.scale.x = 0.002;
	line.color = Rgba(0.1f, 0.9f, 0.1f, 0.9f);
	for (size_t k = 0; k < result.tour_order.size(); ++k)
	{
		Marker sphere = MakeMarker("base_gradient_waypoints", Marker::SPHERE, stamp, id++);
		sphere.pose.position = ToPoint((xform * tour_tcp_poses_nominal[static_cast<size_t>(result.tour_order[k])]).translation());
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		const bool reached = k < result.joint_solutions.size() && !result.joint_solutions[k].empty();
		sphere.color = reached ? Rgba(0.1f, 0.9f, 0.1f, 1.0f) : Rgba(0.9f, 0.1f, 0.1f, 1.0f);
		markers.markers.push_back(sphere);
		line.points.push_back(sphere.pose.position);
	}
	markers.markers.push_back(line);
	return markers;
}
