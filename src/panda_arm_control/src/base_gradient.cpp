#include "panda_arm_control/base_gradient.hpp"
#include "panda_arm_control/real_cost_planning.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <json/json.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <osqp/osqp.h>
#include <random_numbers/random_numbers.h>
#include <std_msgs/msg/color_rgba.hpp>

namespace
{

// Object placement: absolute position (x, y, z, base frame) plus tilt about that position (roll about
// base x, pitch about base y) and spin (yaw about base z), relative to the nominal orientation.
struct ObjectPlacement
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
};

// Maps poses given relative to the object's position (base-frame axes, "_obj") to absolute poses.
Eigen::Isometry3d MakePlacement(const ObjectPlacement& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
	t.linear() = (Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
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
	p.yaw = std::clamp(p.yaw, b.yaw_min, b.yaw_max);
	return p;
}

Eigen::Matrix<double, 6, 1> ToOffsetVec(const ObjectPlacement& a, const ObjectPlacement& b)  // a - b, component-wise
{
	Eigen::Matrix<double, 6, 1> v;
	v << a.x - b.x, a.y - b.y, a.z - b.z, a.roll - b.roll, a.pitch - b.pitch, a.yaw - b.yaw;
	return v;
}

// Gaussian kick around `center`: sigma_m meters on x/y/z, sigma_m/rot_scale rad on roll/pitch.
ObjectPlacement PerturbOffset(const ObjectPlacement& center, std::mt19937& rng, double sigma_m, double rot_scale)
{
	std::normal_distribution<double> nm(0.0, sigma_m);
	std::normal_distribution<double> nr(0.0, sigma_m / std::max(1e-6, rot_scale));
	return {center.x + nm(rng), center.y + nm(rng), center.z + nm(rng), center.roll + nr(rng),
			center.pitch + nr(rng), center.yaw + nr(rng)};
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
// Per-joint range factor s = 1 - exp(-k (q - lo)(hi - q) / range^2): ~1 mid-range, 0 at a limit; ds = ds/dq.
// k <= 0 or an unbounded joint: s = 1. Scales the joint's Jacobian column (penalized manipulability).
void JointRangeFactors(
	const moveit::core::JointModelGroup* jmg, const moveit::core::RobotModelConstPtr& model,
	const std::vector<double>& q, double k, Eigen::VectorXd& s, Eigen::VectorXd& ds)
{
	const int n = static_cast<int>(q.size());
	s = Eigen::VectorXd::Ones(n);
	ds = Eigen::VectorXd::Zero(n);
	if (k <= 0.0)
		return;
	for (int i = 0; i < n; ++i)
	{
		const moveit::core::VariableBounds& b =
			model->getVariableBounds(jmg->getVariableNames()[static_cast<size_t>(i)]);
		const double range = b.max_position_ - b.min_position_;
		if (!b.position_bounded_ || range <= 0.0)
			continue;
		const double qi = q[static_cast<size_t>(i)];
		const double u = std::max(0.0, (qi - b.min_position_) * (b.max_position_ - qi)) / (range * range);
		const double e = std::exp(-k * u);
		s(i) = 1.0 - e;
		if (u > 0.0)
			ds(i) = k * e * (b.min_position_ + b.max_position_ - 2.0 * qi) / (range * range);
	}
}

// w = sqrt(det(Js Js^T)), Js = J with each joint's column scaled by its range factor (limit_sharpness 0 = plain).
double Manipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<double>& q, double limit_sharpness)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	Eigen::VectorXd s, ds;
	JointRangeFactors(jmg, state.getRobotModel(), q, limit_sharpness, s, ds);
	const Eigen::MatrixXd Js = J * s.asDiagonal();
	return std::sqrt(std::max(0.0, (Js * Js.transpose()).determinant()));
}

// Analytic dw/dq: dw/dq_i = w * tr((Js Js^T)^-1 Js dJs_i^T), Js = J S. For revolute joints, column j of dJ/dq_i
// is [z_min(i,j) x Jv_max(i,j); i < j ? z_i x z_j : 0], built from J itself; dJs_i = dJ_i S + J_i ds_i e_i^T.
Eigen::VectorXd ManipulabilityJointGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<double>& q, double limit_sharpness)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;  // 6 x n, [linear; angular] in the base frame
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	const int n = static_cast<int>(J.cols());
	Eigen::VectorXd s, ds;
	JointRangeFactors(jmg, state.getRobotModel(), q, limit_sharpness, s, ds);
	const Eigen::MatrixXd Js = J * s.asDiagonal();
	const Eigen::MatrixXd A = Js * Js.transpose();
	const double w = std::sqrt(std::max(0.0, A.determinant()));
	Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
	if (w < 1e-12)
		return g;
	const Eigen::MatrixXd M = A.ldlt().solve(Js);  // (Js Js^T)^-1 Js
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
		g(i) = w * (M.cwiseProduct(dJ * s.asDiagonal()).sum() + ds(i) * M.col(i).dot(J.col(i)));
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
				if (!jmg->isLinkUpdated(link->getName()))
					continue;  // the group's joints don't move it (e.g. the base); asking for its Jacobian logs an error
				Eigen::MatrixXd J_link;
				if (!state.getJacobian(jmg, link, local_point, J_link))
					continue;
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

// Distance request for arm-arm and arm-object pairs closer than `threshold`.
collision_detection::DistanceRequest MakeClosePairRequest(
	const moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg, double threshold)
{
	collision_detection::DistanceRequest req;
	req.enable_nearest_points = true;
	req.enable_signed_distance = true;
	req.type = collision_detection::DistanceRequestType::ALL;
	req.group_name = jmg->getName();
	req.enableGroup(state.getRobotModel());
	req.distance_threshold = threshold;
	req.max_contacts_per_body = 2;
	return req;
}

// Pairs within `margin` at the state's current pose, with d(distance)/dq rows.
void FindClosePairs(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, collision_detection::DistanceRequest& req, double margin,
	std::vector<Eigen::RowVectorXd>& close_pair_gradients, std::vector<double>& dists)
{
	collision_detection::DistanceResult self_res, world_res;
	planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
	// Skip pairs the scene always allows (adjacent links, hand/fingers), as the validity check does.
	req.acm = &locked_scene->getAllowedCollisionMatrix();
	// Active links from the scene's own robot model: MoveIt matches them by pointer, not name.
	req.enableGroup(locked_scene->getRobotModel());
	locked_scene->getCollisionEnv()->distanceSelf(req, self_res, state);
	locked_scene->getCollisionEnv()->distanceRobot(req, world_res, state);
	ClosePairSeparationGradients(self_res, state, jmg, margin, close_pair_gradients, dists);
	ClosePairSeparationGradients(world_res, state, jmg, margin, close_pair_gradients, dists);
}

// Joint-limit room at q: each joint closer to a limit than `zone` (share of its range) adds ((zone - f) / zone)^2,
// f = distance to the nearest limit / range: 0 outside the zone, 1 at the limit. grad_q = its joint gradient.
double JointLimitRoomCost(
	const moveit::core::JointModelGroup* jmg, const moveit::core::RobotModelConstPtr& model, const std::vector<double>& q,
	double zone, Eigen::VectorXd& grad_q)
{
	grad_q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(q.size()));
	double cost = 0.0;
	for (size_t i = 0; i < q.size(); ++i)
	{
		const moveit::core::VariableBounds& b = model->getVariableBounds(jmg->getVariableNames()[i]);
		const double range = b.max_position_ - b.min_position_;
		if (!b.position_bounded_ || range <= 0.0 || zone <= 0.0)
			continue;
		const double to_min = (q[i] - b.min_position_) / range;
		const double to_max = (b.max_position_ - q[i]) / range;
		const double f = std::max(0.0, std::min(to_min, to_max));
		if (f >= zone)
			continue;
		const double s = (zone - f) / zone;
		cost += s * s;
		const double df_dq = (to_min < to_max ? 1.0 : -1.0) / range;
		grad_q(static_cast<Eigen::Index>(i)) = -2.0 * s / zone * df_dq;
	}
	return cost;
}

// Self-clearance at q: -sum over all arm-arm pairs of log(distance), and its joint gradient. No cutoff, so
// every pose gets a push to spread the links apart; close pairs push hardest (slope 1/d).
double SelfClearanceCost(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const std::vector<double>& q, Eigen::VectorXd& grad_q)
{
	const double all_pairs = 1e3;	// m: distance threshold large enough to return every pair
	const double min_dist = 1e-3;	// m: floor for touching or overlapping pairs
	grad_q = Eigen::VectorXd::Zero(jmg->getVariableCount());
	if (q.empty())
		return 0.0;
	state.setJointGroupPositions(jmg, q);
	state.update();
	collision_detection::DistanceRequest req = MakeClosePairRequest(state, jmg, all_pairs);
	collision_detection::DistanceResult res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
		req.acm = &locked_scene->getAllowedCollisionMatrix();
		req.enableGroup(locked_scene->getRobotModel());  // scene's model: links are matched by pointer
		locked_scene->getCollisionEnv()->distanceSelf(req, res, state);
	}
	std::vector<Eigen::RowVectorXd> rows;
	std::vector<double> dists;
	ClosePairSeparationGradients(res, state, jmg, all_pairs, rows, dists);
	double cost = 0.0;
	for (size_t i = 0; i < rows.size(); ++i)
	{
		const double d = std::max(min_dist, dists[i]);
		cost -= std::log(d);
		grad_q -= rows[i].transpose() / d;
	}
	return cost;
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
	const Eigen::Isometry3d& target_pose, const std::vector<double>& initial_joints, int iters, double margin, double rot_scale,
	double limit_sharpness)
{
	const double damping = 0.01; // for jacobian near singular positions
	const double max_joint_step = 0.1;
	const double manip_step = 0.02;  // rad: null-space step up the (range-aware) manipulability per iteration
	// Stop once the smallest gap so far improves by less than 1 mm over 5 steps.
	const int stall_window = 5;
	const double stall_gain = 1e-3;
	const int dof = static_cast<int>(jmg->getVariableCount());
	collision_detection::DistanceRequest req = MakeClosePairRequest(state, jmg, margin);

	std::vector<double> lower(static_cast<size_t>(dof)), upper(static_cast<size_t>(dof));
	for (int k = 0; k < dof; ++k)
	{
		const moveit::core::VariableBounds& b =
			state.getRobotModel()->getVariableBounds(jmg->getVariableNames()[static_cast<size_t>(k)]);
		lower[static_cast<size_t>(k)] = b.position_bounded_ ? b.min_position_ : -std::numeric_limits<double>::max();
		upper[static_cast<size_t>(k)] = b.position_bounded_ ? b.max_position_ : std::numeric_limits<double>::max();
	}

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
		FindClosePairs(planning_scene_monitor, state, jmg, req, margin, close_pair_gradients, dists);
		// Full validity check (same as IK and the planner) before keeping a pose: the distance query alone
		// let through self-collisions near the base (hand vs link0, link5 vs link7).
		if (gap < best.gap && IsStateCollisionFree(planning_scene_monitor, &state, jmg, q.data()))
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
		Eigen::MatrixXd Jc(static_cast<Eigen::Index>(close_pair_gradients.size()), dof);
		Eigen::VectorXd r(static_cast<Eigen::Index>(close_pair_gradients.size()));
		for (size_t k = 0; k < close_pair_gradients.size(); ++k)
		{
			Jc.row(static_cast<Eigen::Index>(k)) = close_pair_gradients[k];
			r(static_cast<Eigen::Index>(k)) = margin - dists[k];
		}
		const Eigen::VectorXd manip_grad = ManipulabilityJointGradient(state, jmg, tool0_link, q, limit_sharpness);
		// Step using only the joints in `mask`. Priority: pairs out to the margin, then the target, then
		// manipulability in the target's null space (doesn't change the gap to first order).
		auto masked_step = [&](const Eigen::VectorXd& mask) {
			const Eigen::MatrixXd Jm = J * mask.asDiagonal();
			Eigen::MatrixXd JJt = Jm * Jm.transpose();
			JJt.diagonal().array() += damping * damping;
			const Eigen::MatrixXd Jm_pinv = Jm.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));
			Eigen::VectorXd step = Jm_pinv * pose_error;
			const Eigen::VectorXd up = (Eigen::MatrixXd::Identity(dof, dof) - Jm_pinv * Jm) * mask.asDiagonal() * manip_grad;
			if (up.norm() > 1e-12)
				step += manip_step * up / up.norm();
			if (Jc.rows() > 0)
			{
				const Eigen::MatrixXd Jcm = Jc * mask.asDiagonal();
				Eigen::MatrixXd JcJct = Jcm * Jcm.transpose();
				JcJct.diagonal().array() += damping * damping;
				const Eigen::MatrixXd Jc_pinv =
					Jcm.transpose() * JcJct.ldlt().solve(Eigen::MatrixXd::Identity(Jc.rows(), Jc.rows()));
				step = Jc_pinv * r + (Eigen::MatrixXd::Identity(dof, dof) - Jc_pinv * Jcm) * step;
			}
			return step;
		};
		// Joints at a limit that the step pushes further out are taken out and the step re-solved, so the
		// other joints keep working (a folded arm near the robot sits on its elbow/wrist limits).
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
	const moveit::core::LinkModel* tool0_link, const std::vector<std::vector<double>>& joints, double limit_sharpness)
{
	double total = 0.0;
	for (const auto& q : joints)
		if (!q.empty())
			total += std::log(std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, q, limit_sharpness)));
	return total;
}

double SumManipulability(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<std::vector<double>>& joints, double limit_sharpness)
{
	double total = 0.0;
	for (const auto& q : joints)
		if (!q.empty())
			total += Manipulability(state, jmg, tool0_link, q, limit_sharpness);
	return total;
}

// Soft-min of the w values: -tau * log(sum exp(-w / tau)), ~ the smallest w for small tau. shares (optional)
// gets each value's share exp(-w / tau) / sum, the soft-min's slope w.r.t. that w; shares add up to 1.
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

// One tour edge's weighted cost -- identical formula for the inner GTSP and the descent objective.
double WeightedEdgeCost(
	const std::vector<double>& qa, const std::vector<double>& qb, const Eigen::Vector3d& pa,
	const Eigen::Vector3d& pb, const BaseGradientParams& params)
{
	return params.cartesian_distance_weight * (pa - pb).norm() +
		params.joint_distance_weight * JointL2Distance(qa, qb) +
		params.max_joint_deviation_weight * MaxJointDeviation(qa, qb);
}

// One RNG for all random IK starts, seeded from random_seed in SolveBaseGradient: MoveIt's default RNGs
// seed from the clock, so without this the same seed still gives a different run.
std::unique_ptr<random_numbers::RandomNumberGenerator>& SeededRngSlot()
{
	static std::unique_ptr<random_numbers::RandomNumberGenerator> rng;
	return rng;
}

random_numbers::RandomNumberGenerator& SeededRng()
{
	auto& rng = SeededRngSlot();
	if (!rng)
		rng = std::make_unique<random_numbers::RandomNumberGenerator>(42);
	return *rng;
}

// Restart the IK random starts from `seed`. Done before every solve in the descent, so each placement is
// scored with the same luck: the same spot gives the same answer, and nearby spots compare fairly.
void ReseedRng(int seed)
{
	SeededRngSlot() = std::make_unique<random_numbers::RandomNumberGenerator>(static_cast<boost::uint32_t>(std::max(1, seed)));
}

// IK with a fixed amount of work: attempts > 0 runs that many single KDL attempts (a tiny timeout gives one
// each; the first from the current state, the rest from random states), so a seed gives the same answer
// regardless of CPU speed. attempts <= 0: one time-limited call (timeout).
bool SolveIk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg, const geometry_msgs::msg::Pose& target,
	int attempts, double timeout, const moveit::core::GroupStateValidityCallbackFn& validity,
	random_numbers::RandomNumberGenerator* rng = nullptr)
{
	if (attempts <= 0)
		return state.setFromIK(jmg, target, "tool0", timeout, validity);
	const double single_attempt = 1e-9;  // s: KDL stops after its first attempt (0 would mean the default timeout)
	for (int a = 0; a < attempts; ++a)
	{
		if (a > 0)
			state.setToRandomPositions(jmg, rng ? *rng : SeededRng());
		if (state.setFromIK(jmg, target, "tool0", single_attempt, validity))
			return true;
	}
	return false;
}

// IK solutions per viewpoint: one started from the previous solution (stays near it), the rest from
// random starts.
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
		// Tracking: one attempt from the current pose; the random search below runs only if the arm can't follow.
		if (SolveIk(
				state, jmg, target_local, params.tracking ? 1 : params.ik_attempts, params.ik_timeout, validity_callback,
				rng))
		{
			std::vector<double> s;
			state.copyJointGroupPositions(jmg, s);
			sols.push_back(std::move(s));
		}

		for (int attempt = 0; (!params.tracking || sols.empty()) &&
			 attempt < max_sol * 3 + params.ik_retries_per_point && static_cast<int>(sols.size()) < max_sol; ++attempt)
		{
			// Zero solutions after the warm start + 4 random restarts: treat as out of reach. Every
			// miss costs a full ik_timeout, the dominant cost at offsets that drop viewpoints.
			if (sols.empty() && attempt >= 4)
				break;
			state.setToRandomPositions(jmg, rng ? *rng : SeededRng());
			if (!SolveIk(state, jmg, target_local, params.ik_attempts, params.ik_timeout, validity_callback, rng))
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

// Best arm pose per viewpoint for a fixed visiting order: exact dynamic programming over each viewpoint's
// IK solutions (edge cost + per-pose cost), starting from home. Returns the chosen joints, parallel to order.
std::vector<std::vector<double>> BestPosesForOrder(
	const std::vector<std::vector<std::vector<double>>>& ik_solutions,
	const std::vector<std::vector<double>>& pose_cost, const std::vector<int>& order, const ObjectPlacement& placement,
	const std::vector<Eigen::Isometry3d>& tour_poses_obj, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const BaseGradientParams& params)
{
	const Eigen::Isometry3d xform = MakePlacement(placement);
	// best[k][j]: cheapest cost of visiting order[0..k] ending in IK solution j of order[k].
	std::vector<std::vector<double>> best(order.size());
	std::vector<std::vector<int>> parent(order.size());
	for (size_t k = 0; k < order.size(); ++k)
	{
		const size_t v = static_cast<size_t>(order[k]);
		const Eigen::Vector3d p = (xform * tour_poses_obj[v]).translation();
		best[k].assign(ik_solutions[v].size(), std::numeric_limits<double>::max());
		parent[k].assign(ik_solutions[v].size(), -1);
		for (size_t j = 0; j < ik_solutions[v].size(); ++j)
		{
			if (k == 0)
			{
				best[k][j] = pose_cost[v][j] +
					WeightedEdgeCost(home_joints, ik_solutions[v][j], home_tcp_local, p, params);
				continue;
			}
			const size_t u = static_cast<size_t>(order[k - 1]);
			const Eigen::Vector3d prev_p = (xform * tour_poses_obj[u]).translation();
			for (size_t i = 0; i < ik_solutions[u].size(); ++i)
			{
				const double c = best[k - 1][i] + pose_cost[v][j] +
					WeightedEdgeCost(ik_solutions[u][i], ik_solutions[v][j], prev_p, p, params);
				if (c < best[k][j])
				{
					best[k][j] = c;
					parent[k][j] = static_cast<int>(i);
				}
			}
		}
	}
	std::vector<std::vector<double>> joints(order.size());
	if (order.empty())
		return joints;
	int j = static_cast<int>(std::min_element(best.back().begin(), best.back().end()) - best.back().begin());
	for (size_t k = order.size(); k-- > 0;)
	{
		joints[k] = ik_solutions[static_cast<size_t>(order[k])][static_cast<size_t>(j)];
		j = parent[k][static_cast<size_t>(j)];
	}
	return joints;
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
	double sum_manipulability = 0.0;  // sum of w over visited viewpoints (+ missed closest poses if included)
	double sum_log_manipulability = 0.0;  // sum of log(w): the reach-margin barrier
	// Missed viewpoints and their closest-IK pose gaps; miss_cost = weight * sum of capped gaps.
	std::vector<int> missed_vp;
	std::vector<Eigen::Matrix<double, 6, 1>> missed_gap;
	std::vector<std::vector<double>> missed_q;  // parallel: closest collision-free pose, empty if none
	double miss_cost = 0.0;  // miss_gap_weight * sum of capped gaps
	// self_clearance_weight * penalty, over visited (and, if manipulability includes them, missed) poses.
	double clearance_cost = 0.0;
	double clearance_sum = 0.0;  // unweighted self-clearance: -sum log(distance)
	double softmin_manipulability = 0.0;  // SoftMin of w over visited (and included missed) poses
	double gap_sum = 0.0;		 // unweighted sum of capped miss gaps (m)
	std::vector<Eigen::VectorXd> clearance_grad;		 // parallel to joints: weighted d(penalty)/dq
	std::vector<Eigen::VectorXd> missed_clearance_grad;  // parallel to missed_q
	double limit_cost = 0.0;  // joint_limit_weight * limit_sum
	double limit_sum = 0.0;   // unweighted joint-limit room penalty
	std::vector<Eigen::VectorXd> limit_grad;		 // parallel to joints: weighted d(penalty)/dq
	std::vector<Eigen::VectorXd> missed_limit_grad;  // parallel to missed_q
	int num_rescued = 0;  // missed by IK, reached via a collision-free closest-IK hit
	int num_no_free = 0;  // closest IK found no collision-free posture at all (gap taken as the cap)
	double real_cost = -1.0;  // planned joint travel (sum of ||dq|| along OMPL paths); -1 = not computed
	int num_reachable = 0;
	int num_total = 0;
	bool all_reachable = false;
};

// max_solutions: IK solutions to collect per pose (params.max_solutions_per_candidate everywhere in the descent).
InnerSolution InnerSolve(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<std::vector<double>>& seed_per_viewpoint, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>* warm_order, int max_solutions, const std::vector<int>* fixed_order = nullptr)
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
		// First start: the arm pose of the nearest reached viewpoint, often in the right posture family
		// (e.g. folded) when the warm seed is stuck in the wrong one.
		std::vector<double> neighbor_joints;
		double neighbor_dist = std::numeric_limits<double>::max();
		for (size_t j = 0; j < ik_solutions.size(); ++j)
		{
			if (j == i || ik_solutions[j].empty())
				continue;
			const Eigen::Isometry3d other = xform * tour_poses_obj[j];
			const double d = (other.translation() - target.translation()).norm() +
				params.rot_metric_scale * Eigen::AngleAxisd(other.linear() * target.linear().transpose()).angle();
			if (d < neighbor_dist)
			{
				neighbor_dist = d;
				neighbor_joints = ik_solutions[j].front();
			}
		}
		if (params.tracking)
			neighbor_joints.clear();  // follow its own previous closest pose first; random starts only if that fails
		const int num_fixed_starts = neighbor_joints.empty() ? 1 : 2;
		// Nearest reached viewpoint's pose first; the warm start (previous closest pose) and random starts only
		// while no collision-free pose is under the cap (a capped gap has no slope).
		for (int t = 0; t < std::max(num_fixed_starts, params.closest_ik_starts) &&
			 (t == 0 || !best.found || best.gap >= params.miss_gap_cap);
			 ++t)
		{
			std::vector<double> start_joints = seed_per_viewpoint[i];
			if (t == 0 && num_fixed_starts == 2)
				start_joints = neighbor_joints;
			else if (t == 1 && num_fixed_starts == 2)
				start_joints = seed_per_viewpoint[i];
			else if (t > 0)
			{
				state.setToRandomPositions(jmg, SeededRng());
				state.copyJointGroupPositions(jmg, start_joints);
			}
			const ClosestIkResult r = CollisionAwareClosestIk(
				planning_scene_monitor, state, jmg, tool0_link, target, start_joints, params.closest_ik_iters,
				params.closest_ik_margin, params.rot_metric_scale, params.manipulability_limit_sharpness);
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
			const double w = Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness);
			node_cost[i].push_back(
				-params.manipulability_weight * w -
				params.log_manipulability_weight * std::log(std::max(kMinManipulability, w)));
		}
	InnerSolution out;
	if (fixed_order)
	{
		// Keep the given order (reachable viewpoints only); pick only the arm poses.
		for (int v : *fixed_order)
			if (!ik_solutions[static_cast<size_t>(v)].empty())
				out.tour.push_back(v);
		// Reached viewpoints outside the given order (newly reached): insert where the joint detour is smallest.
		for (size_t v = 0; v < ik_solutions.size(); ++v)
		{
			if (ik_solutions[v].empty() || std::find(out.tour.begin(), out.tour.end(), static_cast<int>(v)) != out.tour.end())
				continue;
			const std::vector<double>& qv = ik_solutions[v].front();
			auto q_at = [&](size_t pos) -> const std::vector<double>& {
				return pos == 0 ? home_joints : ik_solutions[static_cast<size_t>(out.tour[pos - 1])].front();
			};
			size_t best_pos = out.tour.size();
			double best_detour = std::numeric_limits<double>::max();
			for (size_t pos = 0; pos <= out.tour.size(); ++pos)
			{
				const std::vector<double>& qa = q_at(pos);
				double detour = JointL2Distance(qa, qv);
				if (pos < out.tour.size())
				{
					const std::vector<double>& qb = ik_solutions[static_cast<size_t>(out.tour[pos])].front();
					detour += JointL2Distance(qv, qb) - JointL2Distance(qa, qb);
				}
				if (detour < best_detour)
				{
					best_detour = detour;
					best_pos = pos;
				}
			}
			out.tour.insert(out.tour.begin() + static_cast<std::ptrdiff_t>(best_pos), static_cast<int>(v));
		}
		out.joints = BestPosesForOrder(
			ik_solutions, node_cost, out.tour, base, tour_poses_obj, home_joints, home_tcp_local, params);
	}
	else
	{
		GtspSolution gtsp = FindTourOrder(
			ik_solutions, base, tour_poses_obj, home_joints, home_tcp_local, params, warm_order, &node_cost);
		out.tour = std::move(gtsp.tour_pose_indices);
		out.joints = std::move(gtsp.chosen_joints);
	}
	out.num_total = static_cast<int>(tour_poses_obj.size());
	out.num_reachable = static_cast<int>(out.tour.size());
	out.all_reachable = (out.num_reachable == out.num_total);
	out.travel_cost =
		TourWeightedCost(base, out.tour, out.joints, tour_poses_obj, home_joints, home_tcp_local, params);
	out.sum_manipulability = SumManipulability(state, jmg, tool0_link, out.joints, params.manipulability_limit_sharpness);
	out.sum_log_manipulability = SumLogManipulability(state, jmg, tool0_link, out.joints, params.manipulability_limit_sharpness);
	if (params.manipulability_include_missed)
	{
		out.sum_manipulability += SumManipulability(state, jmg, tool0_link, missed_q, params.manipulability_limit_sharpness);
		out.sum_log_manipulability += SumLogManipulability(state, jmg, tool0_link, missed_q, params.manipulability_limit_sharpness);
	}
	if (params.softmin_manipulability_weight > 0.0)
	{
		std::vector<double> ws;
		for (const auto& q : out.joints)
			ws.push_back(Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness));
		if (params.manipulability_include_missed)
			for (const auto& q : missed_q)
				if (!q.empty())
					ws.push_back(Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness));
		out.softmin_manipulability = SoftMin(ws, params.softmin_tau);
	}
	if (params.self_clearance_weight > 0.0)
	{
		auto add_clearance = [&](const std::vector<double>& q, std::vector<Eigen::VectorXd>& grads) {
			Eigen::VectorXd gq;
			const double c =
				SelfClearanceCost(planning_scene_monitor, state, jmg, q, gq);
			out.clearance_sum += c;
			out.clearance_cost += params.self_clearance_weight * c;
			grads.push_back(params.self_clearance_weight * gq);
		};
		for (const auto& q : out.joints)
			add_clearance(q, out.clearance_grad);
		if (params.manipulability_include_missed)
			for (const auto& q : missed_q)
				add_clearance(q, out.missed_clearance_grad);
	}
	if (params.joint_limit_weight > 0.0)
	{
		auto add_limit = [&](const std::vector<double>& q, std::vector<Eigen::VectorXd>& grads) {
			Eigen::VectorXd gq = Eigen::VectorXd::Zero(jmg->getVariableCount());
			if (!q.empty())
			{
				const double c = JointLimitRoomCost(jmg, state.getRobotModel(), q, params.joint_limit_zone, gq);
				out.limit_sum += c;
				out.limit_cost += params.joint_limit_weight * c;
			}
			grads.push_back(params.joint_limit_weight * gq);
		};
		for (const auto& q : out.joints)
			add_limit(q, out.limit_grad);
		if (params.manipulability_include_missed)
			for (const auto& q : missed_q)
				add_limit(q, out.missed_limit_grad);
	}
	out.num_rescued = num_rescued;
	out.num_no_free = num_no_free;
	out.missed_vp = std::move(missed_vp);
	out.missed_gap = std::move(missed_gap);
	out.missed_q = std::move(missed_q);
	for (const auto& e : out.missed_gap)
		out.gap_sum += std::min(params.miss_gap_cap, WeightedPoseGap(e, params.rot_metric_scale));
	out.miss_cost = params.miss_gap_weight * out.gap_sum;

	out.weighted_cost = (params.travel_in_cost ? out.travel_cost : 0.0) - params.manipulability_weight * out.sum_manipulability -
		params.log_manipulability_weight * out.sum_log_manipulability -
		params.softmin_manipulability_weight * out.softmin_manipulability +
		params.unreachable_penalty * (out.num_total - out.num_reachable) + out.miss_cost + out.clearance_cost + out.limit_cost;
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
	const std::vector<int>* warm_order, int max_solutions, const std::vector<int>* fixed_order = nullptr)
{
	const int n = std::max(1, params.gtsp_num_restart);
	InnerSolution best = InnerSolve(
		state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seed_per_viewpoint,
		home_joints, home_tcp_local, base, params, warm_order, max_solutions, fixed_order);
	for (int i = 1; i < n; ++i)
	{
		InnerSolution cand = InnerSolve(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seed_per_viewpoint,
			home_joints, home_tcp_local, base, params, warm_order, max_solutions, fixed_order);
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
		bool ok = SolveIk(state, jmg, target_local, params.ik_attempts, params.ik_timeout, validity_callback);
		if (ok)
		{
			state.copyJointGroupPositions(jmg, r.joints[k]);
			r.num_reachable++;
			const double w = Manipulability(state, jmg, tool0_link, r.joints[k], params.manipulability_limit_sharpness);
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

// Jacobian of a viewpoint's pose w.r.t. the object placement: 6x6, rows [position; rotation],
// columns x, y, z, roll, pitch, yaw, in the base frame.
Eigen::Matrix<double, 6, 6> PlacementJacobian(
	const ObjectPlacement& base, const Eigen::Vector3d& viewpoint_position)
{
	const Eigen::Vector3d object_center(base.x, base.y, base.z);
	// Rotation is yaw(z) * pitch(y) * roll(x): each axis is carried by the rotations applied after it.
	const Eigen::Matrix3d yaw_rot = Eigen::AngleAxisd(base.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	const Eigen::Vector3d roll_axis =
		yaw_rot * Eigen::AngleAxisd(base.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::Vector3d::UnitX();
	const Eigen::Vector3d pitch_axis = yaw_rot * Eigen::Vector3d::UnitY();
	const Eigen::Vector3d yaw_axis = Eigen::Vector3d::UnitZ();
	const Eigen::Vector3d lever_arm = viewpoint_position - object_center;  // tilt swings the viewpoint around the center
	Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
	jacobian.block<3, 3>(0, 0).setIdentity();
	jacobian.block<3, 1>(0, 3) = roll_axis.cross(lever_arm);
	jacobian.block<3, 1>(3, 3) = roll_axis;
	jacobian.block<3, 1>(0, 4) = pitch_axis.cross(lever_arm);
	jacobian.block<3, 1>(3, 4) = pitch_axis;
	jacobian.block<3, 1>(0, 5) = yaw_axis.cross(lever_arm);
	jacobian.block<3, 1>(3, 5) = yaw_axis;
	return jacobian;
}

// dw/db: how an object placement change moves one viewpoint's manipulability, the arm tracking it from q.
// Its positive side keeps the viewpoint away from the reach edge.
Eigen::Matrix<double, 6, 1> ManipulabilityOffsetGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const ObjectPlacement& base, const Eigen::Vector3d& target_pos,
	const std::vector<double>& q, double damping, double limit_sharpness)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += damping * damping;
	const Eigen::MatrixXd dq_db =
		J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6)) * PlacementJacobian(base, target_pos);
	return dq_db.transpose() * ManipulabilityJointGradient(state, jmg, tool0_link, q, limit_sharpness);
}

// ---------------------------------------------------------------------------------------------
// Reach rules: a reached viewpoint stays workable while its arm pose keeps joint room, clearance and
// manipulability. Each rule is linearized in the placement b: value + grad . db >= floor.
// ---------------------------------------------------------------------------------------------

// A wall learned from a probe that lost viewpoint vp: moving against n (step metric, unit) loses it, so steps
// keep n . d >= 0. Moving along n gives vp room. Forgotten once the object is far from where it was learned.
struct Wall
{
	Eigen::Matrix<double, 6, 1> n = Eigen::Matrix<double, 6, 1>::Zero();
	ObjectPlacement at;
	int vp = -1;
};

struct ReachRule
{
	double value = 0.0;
	double floor = 0.0;
	Eigen::Matrix<double, 6, 1> grad = Eigen::Matrix<double, 6, 1>::Zero();  // d(value)/db
	Eigen::VectorXd grad_z;	 // d(value)/dz: z moves the arm in its null space (hand fixed), e.g. an elbow swing
	int z_off = 0;			 // where this pose's z starts among all poses' z
	int vp = -1;
	std::string what;
};

// Rules for reached viewpoint vp, arm pose q, viewpoint at target_pos. The scene's object must sit at `base`.
// The arm follows the viewpoint (dq = J_pinv P db) plus null-space motion N z; returns z's size (N's columns).
int ReachedPoseRules(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const moveit::core::LinkModel* tool0_link, const ObjectPlacement& base,
	const Eigen::Vector3d& target_pos, const std::vector<double>& q, int vp, const BaseGradientParams& params,
	int z_off, std::vector<ReachRule>& rules)
{
	const double kClearZone = 0.03;  // m: pairs closer than this get a rule
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += params.jacobian_damping * params.jacobian_damping;
	const Eigen::MatrixXd dq_db =
		J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6)) * PlacementJacobian(base, target_pos);
	// Null space of J: joint motions that keep the hand still.
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullV);
	const Eigen::VectorXd& sv = svd.singularValues();
	int rank = 0;
	for (Eigen::Index i = 0; i < sv.size(); ++i)
		if (sv(i) > 1e-6 * std::max(1e-12, sv(0)))
			++rank;
	const Eigen::MatrixXd N = svd.matrixV().rightCols(J.cols() - rank);
	const size_t first_rule = rules.size();

	// Joint room: share of range to the nearest limit, for joints inside the joint-limit zone.
	const auto& model = state.getRobotModel();
	for (size_t i = 0; i < q.size(); ++i)
	{
		const moveit::core::VariableBounds& b = model->getVariableBounds(jmg->getVariableNames()[i]);
		const double range = b.max_position_ - b.min_position_;
		if (!b.position_bounded_ || range <= 0.0)
			continue;
		const double to_min = (q[i] - b.min_position_) / range;
		const double to_max = (b.max_position_ - q[i]) / range;
		const double f = std::max(0.0, std::min(to_min, to_max));
		if (f >= params.joint_limit_zone)
			continue;
		ReachRule r;
		r.value = f;
		r.floor = std::min(params.rule_limit_room, f);
		r.grad = ((to_min < to_max ? 1.0 : -1.0) / range) * dq_db.row(static_cast<Eigen::Index>(i)).transpose();
		r.grad_z = ((to_min < to_max ? 1.0 : -1.0) / range) * N.row(static_cast<Eigen::Index>(i)).transpose();
		r.vp = vp;
		r.what = "j" + std::to_string(i + 1) + (to_min < to_max ? "@min" : "@max");
		rules.push_back(r);
	}

	// Clearance: arm-arm and arm-object pairs; the arm side follows dq/db, the object side moves with b.
	collision_detection::DistanceRequest req = MakeClosePairRequest(state, jmg, kClearZone);
	collision_detection::DistanceResult self_res, world_res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
		req.acm = &locked_scene->getAllowedCollisionMatrix();
		req.enableGroup(locked_scene->getRobotModel());  // scene's model: links are matched by pointer
		locked_scene->getCollisionEnv()->distanceSelf(req, self_res, state);
		locked_scene->getCollisionEnv()->distanceRobot(req, world_res, state);
	}
	for (const collision_detection::DistanceResult* res : {&self_res, &world_res})
		for (const auto& pair_entry : res->distances)
			for (const auto& d : pair_entry.second)
			{
				if (d.distance >= kClearZone)
					continue;
				Eigen::RowVectorXd row_q = Eigen::RowVectorXd::Zero(jmg->getVariableCount());
				Eigen::Matrix<double, 6, 1> grad_obj = Eigen::Matrix<double, 6, 1>::Zero();
				for (int side = 0; side < 2; ++side)
				{
					// distance ~ distance0 + normal.(dp1 - dp0)
					const double sign = side == 0 ? -1.0 : 1.0;
					if (d.body_types[side] == collision_detection::BodyType::WORLD_OBJECT)
					{
						if (d.link_names[side] == "object")
							grad_obj += sign *
								(PlacementJacobian(base, d.nearest_points[side]).topRows<3>().transpose() * d.normal);
						continue;
					}
					const moveit::core::LinkModel* link = model->getLinkModel(d.link_names[side]);
					if (!link || !jmg->isLinkUpdated(link->getName()))
						continue;
					const Eigen::Vector3d local_point = state.getGlobalLinkTransform(link).inverse() * d.nearest_points[side];
					Eigen::MatrixXd J_link;
					if (!state.getJacobian(jmg, link, local_point, J_link))
						continue;
					row_q += sign * (d.normal.transpose() * J_link.topRows<3>());
				}
				ReachRule r;
				r.value = d.distance;
				r.floor = std::min(params.rule_clearance, d.distance);
				r.grad = (row_q * dq_db).transpose() + grad_obj;
				r.grad_z = (row_q * N).transpose();
				if (r.grad.norm() < 1e-9 && r.grad_z.norm() < 1e-9)
					continue;
				r.vp = vp;
				r.what = d.link_names[0] + "-" + d.link_names[1];
				rules.push_back(r);
			}

	// Reach edge: manipulability may not halve in one step.
	const double w = Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness);
	if (w > 1e-9)
	{
		ReachRule r;
		r.value = w;
		r.floor = 0.5 * w;
		const Eigen::VectorXd dw_dq = ManipulabilityJointGradient(state, jmg, tool0_link, q, params.manipulability_limit_sharpness);
		r.grad = dq_db.transpose() * dw_dq;
		r.grad_z = N.transpose() * dw_dq;
		r.vp = vp;
		r.what = "manipulability";
		rules.push_back(r);
	}
	for (size_t i = first_rule; i < rules.size(); ++i)
		rules[i].z_off = z_off;
	return static_cast<int>(N.cols());
}

// Step d (step metric) and null-space motions z from a small QP: min g.d + c/2 |d|^2 + c_z/2 |z|^2 + rho sum(s)
// s.t. rule_row.[d; z] >= rule_lo, gap + gap_h.d <= s, 0 <= s <= gap, box_lo <= d <= box_hi, |z_i| <= z_max.
bool SolveReachStep(
	const std::vector<Eigen::VectorXd>& rule_row, const std::vector<double>& rule_lo,
	const std::vector<Eigen::Matrix<double, 6, 1>>& gap_h, const std::vector<double>& gap,
	const Eigen::Matrix<double, 6, 1>& g, double c, double rho, const Eigen::Matrix<double, 6, 1>& box_lo,
	const Eigen::Matrix<double, 6, 1>& box_hi, int nz, double z_max, Eigen::Matrix<double, 6, 1>& d, Eigen::VectorXd& z,
	std::string& status)
{
	const int nr = static_cast<int>(rule_row.size());
	const int nm = static_cast<int>(gap_h.size());
	const int ns = 6 + nz;  // slacks start here
	const int nv = ns + nm;
	const int nc = nr + 2 * nm + 6 + nz;
	std::vector<Eigen::Triplet<double>> trip;
	std::vector<c_float> lo(static_cast<size_t>(nc)), hi(static_cast<size_t>(nc));
	int row = 0;
	for (int i = 0; i < nr; ++i, ++row)
	{
		for (int k = 0; k < ns; ++k)
			if (rule_row[i](k) != 0.0)
				trip.emplace_back(row, k, rule_row[i](k));
		lo[row] = rule_lo[i];
		hi[row] = OSQP_INFTY;
	}
	for (int m = 0; m < nm; ++m, ++row)
	{
		for (int k = 0; k < 6; ++k)
			if (gap_h[m](k) != 0.0)
				trip.emplace_back(row, k, gap_h[m](k));
		trip.emplace_back(row, ns + m, -1.0);
		lo[row] = -OSQP_INFTY;
		hi[row] = -gap[m];
	}
	// 0 <= s <= gap: with s >= gap + gap_h.d, no missed gap may grow.
	for (int m = 0; m < nm; ++m, ++row)
	{
		trip.emplace_back(row, ns + m, 1.0);
		lo[row] = 0.0;
		hi[row] = gap[m];
	}
	for (int k = 0; k < 6; ++k, ++row)
	{
		trip.emplace_back(row, k, 1.0);
		lo[row] = box_lo(k);
		hi[row] = box_hi(k);
	}
	for (int k = 0; k < nz; ++k, ++row)
	{
		trip.emplace_back(row, 6 + k, 1.0);
		lo[row] = -z_max;
		hi[row] = z_max;
	}
	Eigen::SparseMatrix<double> A_mat(nc, nv);
	A_mat.setFromTriplets(trip.begin(), trip.end());
	A_mat.makeCompressed();
	std::vector<c_float> Ax(A_mat.valuePtr(), A_mat.valuePtr() + A_mat.nonZeros());
	std::vector<c_int> Ai(A_mat.innerIndexPtr(), A_mat.innerIndexPtr() + A_mat.nonZeros());
	std::vector<c_int> Ap(A_mat.outerIndexPtr(), A_mat.outerIndexPtr() + nv + 1);
	// P: c on the step, a light c_z on z (null-space motion is cheap but bounded), zero on the slacks.
	std::vector<c_float> Px(static_cast<size_t>(ns), 1e-3 * c);
	std::vector<c_int> Pi(static_cast<size_t>(ns));
	for (int j = 0; j < ns; ++j)
	{
		Pi[static_cast<size_t>(j)] = j;
		if (j < 6)
			Px[static_cast<size_t>(j)] = c;
	}
	std::vector<c_int> Pp(static_cast<size_t>(nv + 1));
	for (int j = 0; j <= nv; ++j)
		Pp[static_cast<size_t>(j)] = std::min(j, ns);
	std::vector<c_float> q_lin(static_cast<size_t>(nv), rho);
	for (int k = 0; k < 6; ++k)
		q_lin[static_cast<size_t>(k)] = g(k);

	csc A_csc{static_cast<c_int>(Ax.size()), nc, nv, Ap.data(), Ai.data(), Ax.data(), -1};
	csc P_csc{ns, nv, nv, Pp.data(), Pi.data(), Px.data(), -1};
	OSQPData data;
	data.n = nv;
	data.m = nc;
	data.P = &P_csc;
	data.A = &A_csc;
	data.q = q_lin.data();
	data.l = lo.data();
	data.u = hi.data();
	OSQPSettings settings;
	osqp_set_default_settings(&settings);
	settings.verbose = 0;
	settings.polish = 1;
	settings.max_iter = 20000;
	settings.eps_abs = 1e-6;
	settings.eps_rel = 1e-4;
	OSQPWorkspace* work = nullptr;
	bool ok = false;
	status = "setup failed";
	if (osqp_setup(&work, &data, &settings) == 0)
	{
		osqp_solve(work);
		status = work->info->status;
		// Max-iter answers are still usable: the real solve checks every step anyway.
		if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE ||
			work->info->status_val == OSQP_MAX_ITER_REACHED)
		{
			for (int k = 0; k < 6; ++k)
				d(k) = std::clamp(static_cast<double>(work->solution->x[k]), box_lo(k), box_hi(k));
			z.resize(nz);
			for (int k = 0; k < nz; ++k)
				z(k) = work->solution->x[6 + k];
			ok = true;
		}
	}
	if (work)
		osqp_cleanup(work);
	return ok;
}

// ---------------------------------------------------------------------------------------------
// Analytic gradient of the cost (travel, manipulability terms, missed-viewpoint gaps) w.r.t. the object
// placement (x, y, z, roll, pitch, yaw). Joints follow a moved viewpoint as dq/db = J_pinv * placement_jacobian
// (damped pseudo-inverse of the tool Jacobian); each edge, manipulability term and miss gap is chained
// through that. The home pose doesn't move with the object.
// ---------------------------------------------------------------------------------------------

Eigen::Matrix<double, 6, 1> AnalyticGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& joints, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>& missed_vp, const std::vector<Eigen::Matrix<double, 6, 1>>& missed_gap,
	const std::vector<std::vector<double>>& missed_q, const std::vector<Eigen::VectorXd>& clearance_grad,
	const std::vector<Eigen::VectorXd>& missed_clearance_grad, const std::vector<Eigen::VectorXd>& limit_grad,
	const std::vector<Eigen::VectorXd>& missed_limit_grad)
{
	const size_t n = tour.size();
	Eigen::Isometry3d xform = MakePlacement(base);
	const double lambda2 = params.jacobian_damping * params.jacobian_damping;

	// Per visit position: dq/db (dof x 5) and dp/db (3 x 5).
	std::vector<Eigen::MatrixXd> dq_db(n);
	std::vector<Eigen::MatrixXd> dp_db(n);

	for (size_t k = 0; k < n; ++k)
	{
		const Eigen::Matrix<double, 6, 6> placement_jacobian =
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
	Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
	// Soft-min term: per pose w and dw/db, combined once all poses are in.
	std::vector<double> softmin_w;
	std::vector<Eigen::Matrix<double, 6, 1>> softmin_dw_db;
	for (size_t k = 0; k < n && params.manipulability_gradient; ++k)
	{
		// d(-lambda w - mu log w)/db = -(lambda + mu / w) (dq/db)^T dw/dq
		const double w = std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, joints[k], params.manipulability_limit_sharpness));
		const Eigen::Matrix<double, 6, 1> dw_db = dq_db[k].transpose() *
			ManipulabilityJointGradient(state, jmg, tool0_link, joints[k], params.manipulability_limit_sharpness);
		g -= (params.manipulability_weight + params.log_manipulability_weight / w) * dw_db;
		softmin_w.push_back(w);
		softmin_dw_db.push_back(dw_db);
	}
	// Self-clearance: d(penalty)/db = (dq/db)^T d(penalty)/dq.
	for (size_t k = 0; k < n && k < clearance_grad.size(); ++k)
		g += dq_db[k].transpose() * clearance_grad[k];
	for (size_t k = 0; k < n && k < limit_grad.size(); ++k)
		g += dq_db[k].transpose() * limit_grad[k];
	// Missed viewpoints: same chain at the closest pose, as if the arm followed the viewpoint.
	for (size_t m = 0; m < missed_vp.size() && m < missed_q.size() && params.manipulability_include_missed; ++m)
	{
		const std::vector<double>& q = missed_q[m];
		if (q.empty())
			continue;
		const Eigen::Vector3d viewpoint_position =
			(xform * tour_poses_obj[static_cast<size_t>(missed_vp[m])]).translation();
		state.setJointGroupPositions(jmg, q);
		state.update();
		Eigen::MatrixXd J;
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += lambda2;
		const Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));
		const Eigen::MatrixXd dq_db_missed = J_pinv * PlacementJacobian(base, viewpoint_position);
		const double w = std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness));
		if (params.manipulability_gradient)
		{
			const Eigen::Matrix<double, 6, 1> dw_db = dq_db_missed.transpose() *
				ManipulabilityJointGradient(state, jmg, tool0_link, q, params.manipulability_limit_sharpness);
			g -= (params.manipulability_weight + params.log_manipulability_weight / w) * dw_db;
			softmin_w.push_back(w);
			softmin_dw_db.push_back(dw_db);
		}
		if (m < missed_clearance_grad.size())
			g += dq_db_missed.transpose() * missed_clearance_grad[m];
		if (m < missed_limit_grad.size())
			g += dq_db_missed.transpose() * missed_limit_grad[m];
	}
	// d(-sigma softmin)/db = -sigma * sum_i share_i * dw_i/db
	if (params.softmin_manipulability_weight > 0.0 && !softmin_w.empty())
	{
		std::vector<double> shares;
		SoftMin(softmin_w, params.softmin_tau, &shares);
		for (size_t i = 0; i < shares.size(); ++i)
			g -= params.softmin_manipulability_weight * shares[i] * softmin_dw_db[i];
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

	Eigen::MatrixXd zero_dq = Eigen::MatrixXd::Zero(dof, 6);
	Eigen::MatrixXd zero_dp = Eigen::MatrixXd::Zero(3, 6);

	if (!params.travel_gradient)
		return g;  // descend on manipulability and miss gaps only; travel still counts in the cost
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
Eigen::Matrix<double, 6, 1> FiniteDifferenceGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params)
{
	const double eps = params.fd_epsilon;
	Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Constant(std::nan(""));
	for (int axis = 0; axis < 6; ++axis)
	{
		ObjectPlacement bp = base, bm = base;
		auto component = [](ObjectPlacement& o, int a) -> double& {
			return a == 0 ? o.x : a == 1 ? o.y : a == 2 ? o.z : a == 3 ? o.roll : a == 4 ? o.pitch : o.yaw;
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
	const Eigen::Vector3d& neg_grad_translation, const ObjectPlacement& base, const moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const InnerSolution& sol)
{
	const std::vector<int>& missed_vp = sol.missed_vp;
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

	// Closest-IK arm pose for each missed viewpoint (translucent red), and a line from its tool0 to the target.
	visualization_msgs::msg::Marker gaps = make("base_gradient_miss_gap", visualization_msgs::msg::Marker::LINE_LIST);
	gaps.scale.x = 0.003;
	gaps.color.r = 1.0f;
	gaps.color.g = 0.2f;
	gaps.color.b = 1.0f;
	gaps.color.a = 1.0f;
	visualization_msgs::msg::MarkerArray arms;
	moveit::core::RobotState ghost(state);
	std_msgs::msg::ColorRGBA arm_color;
	arm_color.r = 0.9f;
	arm_color.g = 0.2f;
	arm_color.b = 0.2f;
	arm_color.a = 0.35f;
	for (size_t k = 0; k < sol.missed_vp.size(); ++k)
	{
		if (sol.missed_q[k].empty())
			continue;
		ghost.setJointGroupPositions(jmg, sol.missed_q[k]);
		ghost.update();
		ghost.getRobotMarkers(
			arms, ghost.getRobotModel()->getLinkModelNamesWithCollisionGeometry(), arm_color,
			"base_gradient_miss_ik", rclcpp::Duration(0, 0));
		const Eigen::Vector3d a = ghost.getGlobalLinkTransform("tool0").translation();
		const Eigen::Vector3d b = (xform * tour_poses_obj[static_cast<size_t>(sol.missed_vp[k])]).translation();
		for (const Eigen::Vector3d& p : {a, b})
		{
			geometry_msgs::msg::Point pt;
			pt.x = p.x();
			pt.y = p.y();
			pt.z = p.z();
			gaps.points.push_back(pt);
		}
	}
	if (!gaps.points.empty())
		markers.markers.push_back(gaps);
	for (auto& m : arms.markers)
	{
		m.header.frame_id = "world";
		m.header.stamp = stamp;
		markers.markers.push_back(m);
	}

	// Clear last publish first, so ghosts and lines of viewpoints now reached disappear.
	visualization_msgs::msg::Marker clear;
	clear.action = visualization_msgs::msg::Marker::DELETEALL;
	markers.markers.insert(markers.markers.begin(), clear);

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
	SeededRngSlot() = std::make_unique<random_numbers::RandomNumberGenerator>(
		static_cast<boost::uint32_t>(std::max(1, params.random_seed)));
	// Object and viewpoints relative to the object's position; MakePlacement puts them at an absolute one.
	const Eigen::Isometry3d object_pose_obj = MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal);
	std::vector<Eigen::Isometry3d> tour_poses_obj = tour_tcp_poses_nominal;
	for (Eigen::Isometry3d& t : tour_poses_obj)
		t.translation() -= object_translation_nominal;
	const int n = static_cast<int>(tour_poses_obj.size());
	std::vector<int> input_order(static_cast<size_t>(n));  // viewpoints in the order given (lock_input_order)
	std::iota(input_order.begin(), input_order.end(), 0);

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
		params.log_manipulability_weight = params_in.log_manipulability_weight;
		// manipulability_after_reach: misses only (lambda = mu = 0) until every viewpoint is reached.
		bool manipulability_on = !params.manipulability_after_reach;
		if (!manipulability_on)
		{
			params.manipulability_weight = 0.0;
			params.log_manipulability_weight = 0.0;
		}
		std::vector<ObjectPlacement> base_history{base};
		std::vector<std::vector<double>> seeds =
			warm ? SeedsFromSolution(*warm, fallback_seed, static_cast<size_t>(n))
				 : std::vector<std::vector<double>>(static_cast<size_t>(n), fallback_seed);
		int stall_count = 0;
		std::vector<Wall> walls;  // room-then-reach steering: remembered across iterations while nearby
		// Yaw joins the descent once translation stops improving (or from the start), unless its bounds lock it.
		bool yaw_active = !params.yaw_after_translation;
		auto unlock_yaw = [&]() {
			if (yaw_active || params.bounds.yaw_min == params.bounds.yaw_max)
				return false;
			yaw_active = true;
			stall_count = 0;
			RCLCPP_INFO(node->get_logger(), "  restart %d: translation stopped improving -- adding spin about z", restart_idx + 1);
			return true;
		};
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

		// Real cost: plan every edge with OMPL and sum the joint motion along the planned paths. A failed edge
		// counts as its straight-line joint distance plus unreachable_penalty, so the cost stays finite.
		const bool use_real_cost = params.real_cost_planning_time > 0.0;
		auto real_cost_of = [&](const ObjectPlacement& at, const InnerSolution& sol) {
			SetObjectPose(planning_scene_monitor, MakePlacement(at) * object_pose_obj);
			const std::vector<double> edges = PlanTourJointPathLengths(
				node, robot_model, planning_scene_monitor, sol.joints, start_reference_joints, group_name,
				params.real_cost_planning_time, params.real_cost_attempts);
			double total = 0.0;
			for (size_t k = 0; k < sol.joints.size(); ++k)
			{
				if (k < edges.size() && edges[k] >= 0.0)
					total += edges[k];
				else
					total += JointL2Distance(k == 0 ? start_reference_joints : sol.joints[k - 1], sol.joints[k]) +
						params.unreachable_penalty;
			}
			return total;
		};
		// planned_travel_in_cost: the travel part of the cost is the planned joint travel instead of the L2 estimate.
		const bool planned_travel = params.planned_travel_in_cost && use_real_cost;
		auto travel_term = [&](const InnerSolution& sol) {
			return planned_travel ? sol.real_cost : params.travel_in_cost ? sol.travel_cost : 0.0;
		};
		// Plans the tour once and swaps the L2 travel in weighted_cost for the planned one.
		auto add_planned_travel = [&](const ObjectPlacement& at, InnerSolution& sol) {
			if (!planned_travel || sol.real_cost >= 0.0)
				return;
			sol.real_cost = real_cost_of(at, sol);
			sol.weighted_cost += sol.real_cost - (params.travel_in_cost ? sol.travel_cost : 0.0);
		};

		RestartResult rr;
		rr.placement = base;

		auto record = [&](const ObjectPlacement& b, const InnerSolution& s) {
			// Re-score at the final weight so points found during annealing compare fairly.
			InnerSolution sf = s;
			sf.weighted_cost = travel_term(s) - lambda_final * s.sum_manipulability -
				params_in.log_manipulability_weight * s.sum_log_manipulability -
				params.softmin_manipulability_weight * s.softmin_manipulability +
				params.unreachable_penalty * (s.num_total - s.num_reachable) + s.miss_cost + s.clearance_cost + s.limit_cost;
			result.history.push_back(
				{static_cast<double>(restart_idx), b.x, b.y, b.z, b.roll, b.pitch, b.yaw, sf.weighted_cost});
			// Best seen: most viewpoints reached, then cost (moves may lose viewpoints under miss_tolerance).
			if (sf.num_reachable > rr.sol.num_reachable ||
				(sf.num_reachable == rr.sol.num_reachable && sf.weighted_cost < rr.cost))
			{
				rr.placement = b;
				rr.sol = sf;
				rr.cost = sf.weighted_cost;
				rr.ok = sf.all_reachable;
			}
		};

		ReseedRng(params.random_seed);
		InnerSolution cur = BestOfNInnerSolve(
			state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
			start_reference_joints, home_tcp_local, base, params, warm ? &warm->tour : nullptr,
			params.max_solutions_per_candidate, params.lock_input_order ? &input_order : nullptr);
		result.num_inner_solves += std::max(1, params.gtsp_num_restart);
		add_planned_travel(base, cur);

		if (cur.tour.empty())
		{
			RCLCPP_WARN(node->get_logger(), "restart %d: no tour pose reachable at the start placement", restart_idx + 1);
			return rr;
		}

		record(base, cur);
		remember(cur);
		PublishProgress(
			node, params, object_pose_obj, tour_poses_obj, base_history, Eigen::Vector3d::Zero(), base,
			state, jmg, cur);
		// Cost terms before and after weighting, one per line, fixed columns.
		auto log_cost_terms = [&](const InnerSolution& sol) {
			const double travel_weight = (planned_travel || params.travel_in_cost) ? 1.0 : 0.0;
			const double travel_raw = planned_travel ? sol.real_cost : sol.travel_cost;
			auto row = [&](const char* name, double raw, double weight) {
				RCLCPP_INFO(node->get_logger(), "    %-16s %12.4f %10.1f %12.2f", name, raw, weight, raw * weight);
			};
			RCLCPP_INFO(node->get_logger(), "    %-16s %12s %10s %12s", "term", "raw", "weight", "weighted");
			row(planned_travel ? "planned travel" : "L2 travel", travel_raw, travel_weight);
			row("manipulability", sol.sum_manipulability, -params.manipulability_weight);
			row("log manip", sol.sum_log_manipulability, -params.log_manipulability_weight);
			row("soft-min manip", sol.softmin_manipulability, -params.softmin_manipulability_weight);
			row("self clearance", sol.clearance_sum, params.self_clearance_weight);
			row("joint-limit room", sol.limit_sum, params.joint_limit_weight);
			row("missed count", static_cast<double>(sol.num_total - sol.num_reachable), params.unreachable_penalty);
			row("miss gap (m)", sol.gap_sum, params.miss_gap_weight);
			RCLCPP_INFO(node->get_logger(), "    %-16s %12s %10s %12.2f", "total cost", "", "", sol.weighted_cost);
		};
		// Per missed viewpoint, what blocks it at its closest pose: gap, joints near a limit, closest pairs.
		auto log_missed = [&](const ObjectPlacement& at, const InnerSolution& sol) {
			SetObjectPose(planning_scene_monitor, MakePlacement(at) * object_pose_obj);
			for (size_t m = 0; m < sol.missed_vp.size(); ++m)
			{
				const int v = sol.missed_vp[m];
				if (m >= sol.missed_q.size() || sol.missed_q[m].empty())
				{
					RCLCPP_INFO(node->get_logger(), "    missed vp %2d: no collision-free closest pose", v);
					continue;
				}
				const std::vector<double>& q = sol.missed_q[m];
				const Eigen::Matrix<double, 6, 1>& e = sol.missed_gap[m];
				std::string limits;
				for (size_t k = 0; k < q.size(); ++k)
				{
					const moveit::core::VariableBounds& b = robot_model->getVariableBounds(jmg->getVariableNames()[k]);
					const double range = b.max_position_ - b.min_position_;
					if (!b.position_bounded_ || range <= 0.0)
						continue;
					if (q[k] - b.min_position_ < 0.02 * range)
						limits += " j" + std::to_string(k + 1) + "@min";
					else if (b.max_position_ - q[k] < 0.02 * range)
						limits += " j" + std::to_string(k + 1) + "@max";
				}
				state.setJointGroupPositions(jmg, q);
				state.update();
				collision_detection::DistanceRequest req = MakeClosePairRequest(state, jmg, 1.0);
				collision_detection::DistanceResult self_res, world_res;
				{
					planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
					req.acm = &locked_scene->getAllowedCollisionMatrix();
					req.enableGroup(locked_scene->getRobotModel());
					locked_scene->getCollisionEnv()->distanceSelf(req, self_res, state);
					locked_scene->getCollisionEnv()->distanceRobot(req, world_res, state);
				}
				auto closest = [](const collision_detection::DistanceResult& r) {
					const auto& d = r.minimum_distance;
					if (d.distance > 1e3)
						return std::string("none within 1 m");
					char buf[160];
					std::snprintf(buf, sizeof(buf), "%s - %s %.1f mm", d.link_names[0].c_str(), d.link_names[1].c_str(),
								  d.distance * 1000.0);
					return std::string(buf);
				};
				RCLCPP_INFO(
					node->get_logger(),
					"    missed vp %2d: gap %.1f mm %.1f deg | near limit:%s | arm-object %s | arm-arm %s", v,
					e.head<3>().norm() * 1000.0, e.tail<3>().norm() * 180.0 / M_PI, limits.empty() ? " none" : limits.c_str(),
					closest(world_res).c_str(), closest(self_res).c_str());
			}
		};
		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d iter 0: abs (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f spin %.1f deg  reachable %d/%d",
			restart_idx + 1, std::max(1, params.descent_num_restart), base.x, base.y, base.z, base.roll * 180.0 / M_PI,
			base.pitch * 180.0 / M_PI, base.yaw * 180.0 / M_PI, cur.num_reachable, n);
		log_cost_terms(cur);
		log_missed(base, cur);

		// Cost at the final weight, so solutions found at different weights compare fairly.
		auto cost_at_final_weight = [&](const InnerSolution& sol) {
			return travel_term(sol) - lambda_final * sol.sum_manipulability -
				params_in.log_manipulability_weight * sol.sum_log_manipulability -
				params.softmin_manipulability_weight * sol.softmin_manipulability +
				params.unreachable_penalty * (sol.num_total - sol.num_reachable) + sol.miss_cost + sol.clearance_cost + sol.limit_cost;
		};
		// Refine at a fixed placement: more full solves, each warm-started from the best so far, scored at the
		// final weight. With freeze_order the viewpoint order is kept and only the arm poses are re-picked.
		// Lowest real cost if enabled, else lowest L2 travel; never fewer viewpoints reached.
		auto refine = [&](const ObjectPlacement& at, InnerSolution best_sol, int num_solves) {
			// Travel-only tours: no manipulability preference when picking arm poses.
			const double saved_weight = params.manipulability_weight;
			const double saved_log_weight = params.log_manipulability_weight;
			params.manipulability_weight = 0.0;
			params.log_manipulability_weight = 0.0;
			best_sol.weighted_cost = cost_at_final_weight(best_sol);
			if (use_real_cost && best_sol.real_cost < 0.0)
				best_sol.real_cost = real_cost_of(at, best_sol);
			for (int i = 0; i < num_solves && rclcpp::ok(); ++i)
			{
				const std::vector<std::vector<double>> refine_seeds =
					SeedsFromSolution(best_sol, fallback_seed, static_cast<size_t>(n));
				const std::vector<int>* order_lock = params.lock_input_order ? &input_order
					: params.freeze_order										   ? &best_sol.tour
																				   : nullptr;
				InnerSolution s = InnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, refine_seeds,
					start_reference_joints, home_tcp_local, at, params, &best_sol.tour,
					params.max_solutions_per_candidate, order_lock);
				result.num_inner_solves += 1;
				if (s.num_reachable < best_sol.num_reachable)
					continue;
				if (use_real_cost)
				{
					s.real_cost = real_cost_of(at, s);
					if (s.real_cost < best_sol.real_cost)
						best_sol = std::move(s);
				}
				else if (s.travel_cost < best_sol.travel_cost)
					best_sol = std::move(s);
			}
			params.manipulability_weight = saved_weight;
			params.log_manipulability_weight = saved_log_weight;
			return best_sol;
		};

		bool refined_at_reach = false;
		for (int outer = 0; outer < params.max_outer_iterations && rclcpp::ok() && !cur.tour.empty(); ++outer)
		{
			// Option "stay at A": the first time everything is reached, log what refining right here would give.
			if (params.refine_at_reach > 0 && cur.all_reachable && !refined_at_reach)
			{
				refined_at_reach = true;
				InnerSolution start = cur;
				if (use_real_cost && start.real_cost < 0.0)
					start.real_cost = real_cost_of(base, start);
				const InnerSolution at_reach = refine(base, start, params.refine_at_reach);
				RCLCPP_INFO(
					node->get_logger(),
					"  restart %d: refined %d solves at first all-reached abs (%.4f, %.4f, %.4f) m: travel %.2f -> %.2f  "
					"real %.2f -> %.2f",
					restart_idx + 1, params.refine_at_reach, base.x, base.y, base.z, start.travel_cost,
					at_reach.travel_cost, start.real_cost, at_reach.real_cost);
			}
			if (params.stop_when_all_reached && cur.all_reachable)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: all viewpoints reached -- stopping", restart_idx + 1);
				break;
			}
			if (outer > 0)
			{
				// Hold the weight high (reach recovery) while any viewpoint is missed; decay it otherwise.
				if (!manipulability_on)
				{
					if (cur.all_reachable)
					{
						manipulability_on = true;
						params.manipulability_weight = std::max(lambda_final, params_in.manipulability_weight_initial);
						params.log_manipulability_weight = params_in.log_manipulability_weight;
						RCLCPP_INFO(node->get_logger(), "  restart %d: all reached -- manipulability on", restart_idx + 1);
					}
				}
				else if (!cur.all_reachable)
					params.manipulability_weight = std::max(lambda_final, params_in.manipulability_weight_initial);
				else
					params.manipulability_weight = std::max(
						lambda_final, params.manipulability_weight * params_in.manipulability_weight_decay);
				// Snap to the final weight once within 5% of the starting gap, so a final weight of 0 ends.
				if (params.manipulability_weight - lambda_final <
					0.05 * (params_in.manipulability_weight_initial - lambda_final))
					params.manipulability_weight = lambda_final;
				cur.weighted_cost = travel_term(cur) - params.manipulability_weight * cur.sum_manipulability -
					params.log_manipulability_weight * cur.sum_log_manipulability -
					params.softmin_manipulability_weight * cur.softmin_manipulability +
					params.unreachable_penalty * (cur.num_total - cur.num_reachable) + cur.miss_cost + cur.clearance_cost + cur.limit_cost;
			}
			// A stop rule hit while still annealing jumps the weight to its final value instead of stopping,
			// so the descent ends at the final objective without waiting out the decay.
			const bool annealing = params.manipulability_weight > lambda_final * (1.0 + 1e-6);

			Eigen::Matrix<double, 6, 1> g = AnalyticGradient(
				state, jmg, tool0_link, tour_poses_obj, cur.tour, cur.joints, start_reference_joints,
				home_tcp_local, base, params, cur.missed_vp, cur.missed_gap, cur.missed_q, cur.clearance_grad,
				cur.missed_clearance_grad, cur.limit_grad, cur.missed_limit_grad);

			if (params.trace_line_search)
			{
				// Each term's own descent direction (translation part of -gradient) and size.
				auto part = [&](bool manip, double gap_weight, bool clear, bool limit) {
					BaseGradientParams p = params;
					p.travel_gradient = false;
					p.manipulability_gradient = manip;
					p.miss_gap_weight = gap_weight;
					return AnalyticGradient(
						state, jmg, tool0_link, tour_poses_obj, cur.tour, cur.joints, start_reference_joints,
						home_tcp_local, base, p, cur.missed_vp, cur.missed_gap, cur.missed_q,
						clear ? cur.clearance_grad : std::vector<Eigen::VectorXd>{},
						clear ? cur.missed_clearance_grad : std::vector<Eigen::VectorXd>{},
						limit ? cur.limit_grad : std::vector<Eigen::VectorXd>{},
						limit ? cur.missed_limit_grad : std::vector<Eigen::VectorXd>{});
				};
				auto describe = [](const Eigen::Matrix<double, 6, 1>& gp) {
					const Eigen::Vector3d d = -gp.head<3>();
					const double nrm = d.norm();
					const Eigen::Vector3d u = nrm > 1e-9 ? Eigen::Vector3d(d / nrm) : Eigen::Vector3d::Zero();
					char buf[96];
					std::snprintf(buf, sizeof(buf), "|%.1f| dir (%+.2f, %+.2f, %+.2f)", nrm, u.x(), u.y(), u.z());
					return std::string(buf);
				};
				RCLCPP_INFO(
					node->get_logger(), "    directions: manip %s  gap %s  clear %s  limit %s  total %s",
					describe(part(params.manipulability_gradient, 0.0, false, false)).c_str(),
					describe(part(false, params.miss_gap_weight, false, false)).c_str(),
					describe(part(false, 0.0, true, false)).c_str(), describe(part(false, 0.0, false, true)).c_str(),
					describe(g).c_str());
			}
			if (params.fd_gradient_check)
			{
				Eigen::Matrix<double, 6, 1> g_fd = FiniteDifferenceGradient(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, cur.tour,
					cur.joints, start_reference_joints, home_tcp_local, base, params);
				RCLCPP_INFO(
					node->get_logger(),
					"    grad check [x y z roll pitch yaw]  analytic (%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f)  "
					"central-diff (%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f)",
					g(0), g(1), g(2), g(3), g(4), g(5), g_fd(0), g_fd(1), g_fd(2), g_fd(3), g_fd(4), g_fd(5));
			}

			// Descend in a metric where 1 rad of tip/tilt/spin equals rot_scale meters. Locked axes (min == max,
			// or yaw before it joins) get no share of the step.
			const BaseGradientBounds& bb = params.bounds;
			auto to_u = [&](Eigen::Matrix<double, 6, 1> v) {
				v(3) /= rot_scale;
				v(4) /= rot_scale;
				v(5) /= rot_scale;
				if (bb.z_min == bb.z_max)
					v(2) = 0.0;
				if (bb.roll_min == bb.roll_max)
					v(3) = 0.0;
				if (bb.pitch_min == bb.pitch_max)
					v(4) = 0.0;
				if (bb.yaw_min == bb.yaw_max || !yaw_active)
					v(5) = 0.0;
				return v;
			};
			const Eigen::Matrix<double, 6, 1> g_u = to_u(g);
			double gnorm = g_u.norm();
			if (gnorm < 1e-6)
			{
				if (unlock_yaw())
					continue;
				RCLCPP_INFO(node->get_logger(), "  restart %d: gradient ~ 0 -- converged", restart_idx + 1);
				PublishProgress(
					node, params, object_pose_obj, tour_poses_obj, base_history,
					Eigen::Vector3d(-g.head<3>()), base, state, jmg, cur);
				break;
			}

			// Backtracking line search. Each probe is one solve, compared with one solve at the current point
			// (same effort); the accepted offset gets a best-of-N solve, kept only if it beats the current one.
			Eigen::Matrix<double, 6, 1> dir_u = -g_u / gnorm;
			seeds = SeedsFromSolution(cur, fallback_seed, static_cast<size_t>(n));
			for (int v : cur.missed_vp)
			{
				const size_t vi = static_cast<size_t>(v);
				if (!last_closest[vi].empty())
					seeds[vi] = last_closest[vi];
				else if (!last_good[vi].empty())
					seeds[vi] = last_good[vi];
			}
			// Locked viewpoint order: the input order from the start (lock_input_order), or the current order
			// once everything is reached (freeze_order). Only arm poses are re-picked; null = GTSP reorders.
			const std::vector<int> locked_order = cur.tour;
			const std::vector<int>* order_lock = params.lock_input_order ? &input_order
				: (params.freeze_order && cur.all_reachable)			   ? &locked_order
																		   : nullptr;
			auto solve_at = [&](const ObjectPlacement& at, int max_solutions) {
				ReseedRng(params.random_seed);
				return InnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds, start_reference_joints,
					home_tcp_local, at, params, &cur.tour, max_solutions, order_lock);
			};
			// Tracked probes: every viewpoint follows its current arm pose (one warm IK attempt, fixed order), so
			// nearby placements compare the same arm poses; the full solve at an accepted spot searches anew.
			BaseGradientParams track_params = params;
			track_params.tracking = true;
			auto quick_solve = [&](const ObjectPlacement& at) {
				if (!params.track_probes)
					return solve_at(at, params.max_solutions_per_candidate);
				ReseedRng(params.random_seed);
				return InnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds, start_reference_joints,
					home_tcp_local, at, track_params, &cur.tour, 1, &locked_order);
			};
			InnerSolution here_quick = quick_solve(base);
			add_planned_travel(base, here_quick);
			result.num_inner_solves += 1;
			double step = params.initial_step;
			bool accepted = false;
			ObjectPlacement b_new = base;
			InnerSolution next;
			// Steering: when a probe loses a reached viewpoint, remove the part of the direction that lowers its
			// manipulability (reach edge) or, failing that, grows its closest-IK gap at the probe (limit, collision).
			std::vector<Eigen::Matrix<double, 6, 1>> blocked;  // orthonormal, step metric
			// More viewpoints reached wins; cost only breaks ties.
			auto better = [](const InnerSolution& a, const InnerSolution& b) {
				return a.num_reachable > b.num_reachable ||
					(a.num_reachable == b.num_reachable && a.weighted_cost < b.weighted_cost);
			};
			// Accepting a move: same rule, but tolerate a cost rise up to cost_slack (planned-travel noise), and
			// losing up to miss_tolerance viewpoints when the cost (which charges each miss) still drops.
			const double slack = std::max(0.0, params.cost_slack);
			const int tol = std::max(0, params.miss_tolerance);
			auto acceptable = [slack, tol](const InnerSolution& a, const InnerSolution& b) {
				return a.num_reachable > b.num_reachable ||
					(a.num_reachable == b.num_reachable && a.weighted_cost < b.weighted_cost + slack) ||
					(a.num_reachable >= b.num_reachable - tol && a.weighted_cost < b.weighted_cost);
			};
			// Viewpoint v is reached here but missed at cand: the direction (step metric) whose opposite loses it --
			// its manipulability slope (reach edge) or, failing that, its gap slope at cand. Zero if neither fits step_dir.
			// `from`/`from_sol`: the spot the step starts at and its solution (where v is reached).
			auto loss_direction = [&](int v, const ObjectPlacement& from, const InnerSolution& from_sol,
									  const ObjectPlacement& cand, const InnerSolution& probe,
									  const Eigen::Matrix<double, 6, 1>& step_dir) {
				const Eigen::Matrix<double, 6, 1> none = Eigen::Matrix<double, 6, 1>::Zero();
				const auto it = std::find(from_sol.tour.begin(), from_sol.tour.end(), v);
				if (it == from_sol.tour.end())
					return none;  // already missed where the step starts
				const Eigen::Vector3d p = (MakePlacement(from) * tour_poses_obj[static_cast<size_t>(v)]).translation();
				Eigen::Matrix<double, 6, 1> h = to_u(ManipulabilityOffsetGradient(
					state, jmg, tool0_link, from, p, from_sol.joints[static_cast<size_t>(it - from_sol.tour.begin())],
					params.jacobian_damping, params.manipulability_limit_sharpness));
				if (step_dir.dot(h) < 0.0)
					return h;
				// Not the reach edge: use the direction that shrinks its gap at the probe instead.
				const size_t m = static_cast<size_t>(
					std::find(probe.missed_vp.begin(), probe.missed_vp.end(), v) - probe.missed_vp.begin());
				if (m >= probe.missed_q.size() || probe.missed_q[m].empty())
					return none;  // no collision-free closest pose: no gap slope
				const Eigen::Matrix<double, 6, 1>& pose_error = probe.missed_gap[m];
				const double gap = WeightedPoseGap(pose_error, params.rot_metric_scale);
				if (gap < 1e-9)
					return none;
				Eigen::Matrix<double, 6, 1> weighted_error = pose_error;
				weighted_error.tail<3>() *= params.rot_metric_scale * params.rot_metric_scale;
				const Eigen::Vector3d pc = (MakePlacement(cand) * tour_poses_obj[static_cast<size_t>(v)]).translation();
				h = -to_u(PlacementJacobian(cand, pc).transpose() * weighted_error / gap);
				return step_dir.dot(h) < 0.0 ? h : none;  // the step doesn't grow its gap either
			};
			// Add v's loss direction to `blk`, orthonormalized. False if it adds nothing new.
			auto block = [&](std::vector<Eigen::Matrix<double, 6, 1>>& blk, int v, const ObjectPlacement& cand,
							 const InnerSolution& probe, const Eigen::Matrix<double, 6, 1>& step_dir) {
				Eigen::Matrix<double, 6, 1> h = loss_direction(v, base, here_quick, cand, probe, step_dir);
				for (const Eigen::Matrix<double, 6, 1>& e : blk)
					h -= h.dot(e) * e;
				if (h.norm() < 1e-9)
					return false;
				blk.push_back(h.normalized());
				return true;
			};
			// Room-then-reach walls: forget walls learned far away; aim_within_walls gives the direction nearest to `a`
			// (unit) that keeps n . d >= 0 for every wall (zero if none is left).
			const double kWallRadius = 0.1;  // step metric (m): walls learned farther away no longer apply
			auto u_dist = [&](const ObjectPlacement& a, const ObjectPlacement& b) {
				Eigen::Matrix<double, 6, 1> v = ToOffsetVec(a, b);
				v.tail<3>() *= rot_scale;
				return v.norm();
			};
			walls.erase(
				std::remove_if(walls.begin(), walls.end(), [&](const Wall& w) { return u_dist(w.at, base) > kWallRadius; }),
				walls.end());
			auto aim_within_walls = [&](const Eigen::Matrix<double, 6, 1>& a) {
				if (walls.empty())
					return a;
				std::vector<Eigen::VectorXd> rows;
				std::vector<double> lo;
				for (const Wall& w : walls)
				{
					rows.push_back(w.n);
					lo.push_back(0.0);
				}
				Eigen::Matrix<double, 6, 1> box_lo, box_hi;
				for (int k = 0; k < 6; ++k)
				{
					const bool locked = to_u(Eigen::Matrix<double, 6, 1>::Unit(k))(k) == 0.0;
					box_lo(k) = locked ? 0.0 : -1.0;
					box_hi(k) = locked ? 0.0 : 1.0;
				}
				Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
				Eigen::VectorXd z;
				std::string status;
				// min -a.d + |d|^2 / 2 within the walls: a projected onto the allowed side.
				if (!SolveReachStep(rows, lo, {}, {}, -a, 1.0, 0.0, box_lo, box_hi, 0, 0.0, d, z, status))
					return Eigen::Matrix<double, 6, 1>(Eigen::Matrix<double, 6, 1>::Zero());
				return d;
			};
			auto add_wall = [&](int v, const Eigen::Matrix<double, 6, 1>& h, const ObjectPlacement& at) {
				if (h.norm() < 1e-9)
					return false;
				// One wall per viewpoint: the newest replaces the old one, so stale walls can't contradict it.
				walls.erase(
					std::remove_if(walls.begin(), walls.end(), [v](const Wall& w) { return w.vp == v; }), walls.end());
				walls.push_back({h.normalized(), at, v});
				if (params.trace_line_search)
					RCLCPP_INFO(
						node->get_logger(), "    wall: vp %d, room side (%+.2f, %+.2f, %+.2f) spin %+.2f, %zu walls", v,
						walls.back().n(0), walls.back().n(1), walls.back().n(2), walls.back().n(5), walls.size());
				return true;
			};
			if (params.room_then_reach && !walls.empty())
			{
				const Eigen::Matrix<double, 6, 1> d = aim_within_walls(dir_u);
				if (d.norm() >= 0.1)
					dir_u = d.normalized();  // stay off known walls
			}
			int num_steers = 0;
			const int kMaxSteers = 3;
			// Gradient line search with steering; skipped when reach rules pick the step.
			for (int ls = 0; !params.reach_rules && ls < params.max_line_search_iters;)
			{
				ObjectPlacement cand = ProjectToBounds(
					{base.x + step * dir_u(0), base.y + step * dir_u(1), base.z + step * dir_u(2),
					 base.roll + step * dir_u(3) / rot_scale, base.pitch + step * dir_u(4) / rot_scale,
					 base.yaw + step * dir_u(5) / rot_scale},
					params.bounds);
				// weighted_cost carries the unreachable penalty, so a probe that drops a viewpoint fails.
				InnerSolution probe = quick_solve(cand);
				result.num_inner_solves += 1;
				if (probe.num_reachable >= here_quick.num_reachable - tol)
					add_planned_travel(cand, probe);  // plan only probes that could be accepted
				if (params.trace_line_search)
				{
					std::string lost;
					for (int v : here_quick.tour)
						if (std::find(probe.tour.begin(), probe.tour.end(), v) == probe.tour.end())
							lost += " " + std::to_string(v);
					RCLCPP_INFO(
						node->get_logger(),
						"    probe step=%.4f m abs (%.4f, %.4f, %.4f): reached %d vs %d here, cost %.2f vs %.2f here "
						"(sum_w %.3f vs %.3f, planned %.2f vs %.2f)  lost:%s",
						step, cand.x, cand.y, cand.z, probe.num_reachable, here_quick.num_reachable, probe.weighted_cost,
						here_quick.weighted_cost, probe.sum_manipulability, here_quick.sum_manipulability, probe.real_cost,
						here_quick.real_cost, lost.empty() ? " none" : lost.c_str());
				}
				// Never trade away a reached viewpoint, whatever the cost says. Compared with the quick solve
				// here (same IK effort), so a viewpoint the quick solve merely failed to find doesn't block the step.
				if (acceptable(probe, here_quick))
				{
					accepted = true;
					b_new = cand;
					next = std::move(probe);
					break;
				}
				// Steer whenever a reached viewpoint is lost, even if the probe gained another.
				bool lost_any = false;
				for (int v : here_quick.tour)
					if (std::find(probe.tour.begin(), probe.tour.end(), v) == probe.tour.end())
						lost_any = true;
				if (params.room_then_reach && params.steering && lost_any && num_steers < kMaxSteers)
				{
					// One-sided walls: block only the side that loses each viewpoint, then re-aim the descent.
					bool steered = false;
					for (int v : probe.missed_vp)
						steered = add_wall(v, loss_direction(v, base, here_quick, cand, probe, dir_u), base) || steered;
					if (steered)
					{
						++num_steers;
						const Eigen::Matrix<double, 6, 1> d = aim_within_walls(-g_u / gnorm);
						if (d.norm() < 0.1)
						{
							if (params.trace_line_search)
								RCLCPP_INFO(node->get_logger(), "    walls block the descent: room-then-reach next");
							break;
						}
						dir_u = d.normalized();
						if (params.trace_line_search)
							RCLCPP_INFO(
								node->get_logger(), "    steered: new dir (%+.2f, %+.2f, %+.2f)", dir_u(0), dir_u(1),
								dir_u(2));
						continue;
					}
				}
				else if (params.steering && lost_any && num_steers < kMaxSteers)
				{
					bool steered = false;
					for (int v : probe.missed_vp)
						steered = block(blocked, v, cand, probe, dir_u) || steered;
					if (steered)
					{
						++num_steers;
						Eigen::Matrix<double, 6, 1> d = -g_u / gnorm;
						for (const Eigen::Matrix<double, 6, 1>& e : blocked)
							d -= d.dot(e) * e;
						if (d.norm() < 0.1)
						{
							// No direction keeps every viewpoint: one half-size try along the descent direction, then stop.
							if (params.trace_line_search)
								RCLCPP_INFO(node->get_logger(), "    steering stuck: one half-size try, then stop");
							dir_u = -g_u / gnorm;
							step *= 0.5;
							num_steers = kMaxSteers;
							ls = params.max_line_search_iters - 1;  // the next probe is the last
							continue;
						}
						dir_u = d.normalized();
						if (params.trace_line_search)
							RCLCPP_INFO(
								node->get_logger(), "    steered: new dir (%+.2f, %+.2f, %+.2f)", dir_u(0), dir_u(1),
								dir_u(2));
						continue;
					}
				}
				step *= params.step_shrink;
				++ls;
			}

			// Reach probe: stuck with misses, step along each missed viewpoint's own gap-shrinking direction
			// (gap size, capped at initial_step, then half). Accepted by the same rule as the main probes.
			for (size_t m = 0; !params.reach_rules && !params.room_then_reach && !accepted && params.reach_probe &&
				 m < here_quick.missed_vp.size();
				 ++m)
			{
				if (m >= here_quick.missed_q.size() || here_quick.missed_q[m].empty())
					continue;  // no collision-free closest pose: no gap direction
				const int v = here_quick.missed_vp[m];
				const Eigen::Matrix<double, 6, 1>& pose_error = here_quick.missed_gap[m];
				const double gap = WeightedPoseGap(pose_error, params.rot_metric_scale);
				if (gap < 1e-9)
					continue;
				Eigen::Matrix<double, 6, 1> weighted_error = pose_error;
				weighted_error.tail<3>() *= params.rot_metric_scale * params.rot_metric_scale;
				const Eigen::Vector3d p = (MakePlacement(base) * tour_poses_obj[static_cast<size_t>(v)]).translation();
				const Eigen::Matrix<double, 6, 1> h = -to_u(PlacementJacobian(base, p).transpose() * weighted_error);
				if (h.norm() < 1e-9)
					continue;
				Eigen::Matrix<double, 6, 1> d = h.normalized();
				std::vector<Eigen::Matrix<double, 6, 1>> reach_blocked;
				int reach_steers = 0;
				double rstep = std::min(gap, params.initial_step);
				for (int k = 0; k < 2 && !accepted;)
				{
					ObjectPlacement cand = ProjectToBounds(
						{base.x + rstep * d(0), base.y + rstep * d(1), base.z + rstep * d(2),
						 base.roll + rstep * d(3) / rot_scale, base.pitch + rstep * d(4) / rot_scale,
						 base.yaw + rstep * d(5) / rot_scale},
						params.bounds);
					InnerSolution probe = quick_solve(cand);
					result.num_inner_solves += 1;
					std::string lost;
					for (int r : here_quick.tour)
						if (std::find(probe.tour.begin(), probe.tour.end(), r) == probe.tour.end())
							lost += " " + std::to_string(r);
					if (probe.num_reachable >= here_quick.num_reachable - tol)
						add_planned_travel(cand, probe);
					const bool ok = acceptable(probe, here_quick);  // same rule as the main probes, miss_tolerance included
					if (params.trace_line_search)
						RCLCPP_INFO(
							node->get_logger(),
							"    reach probe vp %d step=%.4f m dir (%+.2f, %+.2f, %+.2f) abs (%.4f, %.4f, %.4f): reached %d vs "
							"%d here, cost %.2f vs %.2f here  lost:%s -> %s",
							v, rstep, d(0), d(1), d(2), cand.x, cand.y, cand.z, probe.num_reachable,
							here_quick.num_reachable, probe.weighted_cost, here_quick.weighted_cost,
							lost.empty() ? " none" : lost.c_str(), ok ? "accepted" : "rejected");
					if (ok)
					{
						accepted = true;
						b_new = cand;
						next = std::move(probe);
						break;
					}
					// Lost a viewpoint: bend the direction away from its edge, same as the main steering.
					if (params.steering && !lost.empty() && reach_steers < kMaxSteers)
					{
						bool steered = false;
						for (int r : probe.missed_vp)
							steered = block(reach_blocked, r, cand, probe, d) || steered;
						if (steered)
						{
							++reach_steers;
							Eigen::Matrix<double, 6, 1> dd = h.normalized();
							for (const Eigen::Matrix<double, 6, 1>& e : reach_blocked)
								dd -= dd.dot(e) * e;
							if (dd.norm() < 0.1)
								break;  // nothing left of the reach direction
							d = dd.normalized();
							if (params.trace_line_search)
								RCLCPP_INFO(
									node->get_logger(), "    reach probe steered: new dir (%+.2f, %+.2f, %+.2f)", d(0), d(1),
									d(2));
							continue;  // same step size, new direction
						}
					}
					rstep *= 0.5;
					++k;
				}
			}

			// Room then reach: stuck with misses, walk toward each missed viewpoint (closest first). An aim step that
			// reaches it is accepted; one that loses nothing walks on (cost within room_budget); one that loses
			// viewpoints adds walls, then a room move along their room side gives the blockers room. Up to kMaxDetours.
			if (!accepted && params.room_then_reach && !params.reach_rules && !here_quick.missed_vp.empty())
			{
				const int kMaxDetours = 6;  // aim steps + room moves per missed viewpoint
				auto lost_from = [](const InnerSolution& from, const InnerSolution& to) {
					std::vector<int> lost;
					for (int v : from.tour)
						if (std::find(to.tour.begin(), to.tour.end(), v) == to.tour.end())
							lost.push_back(v);
					return lost;
				};
				auto list = [](const std::vector<int>& v) {
					std::string out;
					for (int x : v)
						out += " " + std::to_string(x);
					return out.empty() ? std::string(" none") : out;
				};
				auto shifted = [&](const ObjectPlacement& at, const Eigen::Matrix<double, 6, 1>& d, double size) {
					return ProjectToBounds(
						{at.x + size * d(0), at.y + size * d(1), at.z + size * d(2), at.roll + size * d(3) / rot_scale,
						 at.pitch + size * d(4) / rot_scale, at.yaw + size * d(5) / rot_scale},
						params.bounds);
				};
				// Missed viewpoints, smallest gap first.
				std::vector<size_t> order(here_quick.missed_vp.size());
				std::iota(order.begin(), order.end(), 0);
				std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
					return WeightedPoseGap(here_quick.missed_gap[a], params.rot_metric_scale) <
						WeightedPoseGap(here_quick.missed_gap[b], params.rot_metric_scale);
				});
				for (size_t oi = 0; oi < order.size() && !accepted; ++oi)
				{
					const int target = here_quick.missed_vp[order[oi]];
					ObjectPlacement at = base;
					InnerSolution here_at = here_quick;
					for (int detour = 0; detour <= kMaxDetours && !accepted; ++detour)
					{
						// Aim: the target's gap-shrinking direction at `at`, kept off the walls.
						const size_t mi = static_cast<size_t>(
							std::find(here_at.missed_vp.begin(), here_at.missed_vp.end(), target) - here_at.missed_vp.begin());
						if (mi >= here_at.missed_vp.size() || mi >= here_at.missed_gap.size())
							break;
						Eigen::Matrix<double, 6, 1> pose_error = here_at.missed_gap[mi];
						if (detour == 0)
						{
							// Steadier aim: the full solve's closest pose may be nearer than the tracked one.
							const auto ci = std::find(cur.missed_vp.begin(), cur.missed_vp.end(), target);
							const size_t c = static_cast<size_t>(ci - cur.missed_vp.begin());
							if (ci != cur.missed_vp.end() && c < cur.missed_gap.size() &&
								WeightedPoseGap(cur.missed_gap[c], params.rot_metric_scale) <
									WeightedPoseGap(pose_error, params.rot_metric_scale))
								pose_error = cur.missed_gap[c];
						}
						const double gap = WeightedPoseGap(pose_error, params.rot_metric_scale);
						if (gap < 1e-9)
							break;
						Eigen::Matrix<double, 6, 1> weighted_error = pose_error;
						weighted_error.tail<3>() *= params.rot_metric_scale * params.rot_metric_scale;
						const Eigen::Vector3d tp = (MakePlacement(at) * tour_poses_obj[static_cast<size_t>(target)]).translation();
						const Eigen::Matrix<double, 6, 1> aim_raw = -to_u(PlacementJacobian(at, tp).transpose() * weighted_error);
						if (aim_raw.norm() < 1e-9)
							break;
						const Eigen::Matrix<double, 6, 1> aim_d = aim_within_walls(aim_raw.normalized());
						if (aim_d.norm() < 0.1)
						{
							if (params.trace_line_search)
								RCLCPP_INFO(node->get_logger(), "    aim vp %d: walls leave no way toward it", target);
							break;
						}
						const Eigen::Matrix<double, 6, 1> aim = aim_d.normalized();
						std::vector<int> lost_all;
						bool walked = false;
						for (double size : {params.initial_step, 0.5 * params.initial_step})
						{
							const ObjectPlacement cand = shifted(at, aim, size);
							InnerSolution probe = quick_solve(cand);
							result.num_inner_solves += 1;
							if (probe.num_reachable >= here_quick.num_reachable)
								add_planned_travel(cand, probe);
							const std::vector<int> lost = lost_from(here_at, probe);
							// Counts only if it reaches more and loses nothing; a small cost win alone isn't the goal here.
							const bool ok =
								lost_from(here_quick, probe).empty() && probe.num_reachable > here_quick.num_reachable;
							const bool walk = !ok && lost.empty() &&
								probe.weighted_cost <= here_quick.weighted_cost + params.room_budget;
							if (params.trace_line_search)
								RCLCPP_INFO(
									node->get_logger(),
									"    aim vp %d (detour %d) step=%.4f m dir (%+.2f, %+.2f, %+.2f) abs (%.4f, %.4f, %.4f): reached %d "
									"vs %d here, cost %.2f vs %.2f here  lost:%s -> %s",
									target, detour, size, aim(0), aim(1), aim(2), cand.x, cand.y, cand.z, probe.num_reachable,
									here_quick.num_reachable, probe.weighted_cost, here_quick.weighted_cost, list(lost).c_str(),
									ok ? "accepted" : walk ? "walk on" : "rejected");
							if (ok)
							{
								accepted = true;
								b_new = cand;
								next = std::move(probe);
								step = size;
								break;
							}
							if (walk)
							{
								at = cand;
								here_at = std::move(probe);
								walked = true;
								break;
							}
							for (int v : lost)
								if (add_wall(v, loss_direction(v, at, here_at, cand, probe, aim), at))
									lost_all.push_back(v);
							if (!lost.empty())
								break;  // walls learned: make room rather than shrink
						}
						if (accepted || detour == kMaxDetours)
							break;
						if (walked)
							continue;  // aim again from the new spot
						if (lost_all.empty())
							break;  // no loss, no gain, over budget: nothing to make room for

						// Room: along the new walls' room side (kept within all walls); no viewpoint lost, cost within budget.
						Eigen::Matrix<double, 6, 1> room_raw = Eigen::Matrix<double, 6, 1>::Zero();
						for (size_t w = walls.size() - lost_all.size(); w < walls.size(); ++w)
							room_raw += walls[w].n;
						const Eigen::Matrix<double, 6, 1> room_d =
							room_raw.norm() > 1e-9 ? aim_within_walls(room_raw.normalized()) : room_raw;
						if (room_d.norm() < 0.1)
							break;
						const Eigen::Matrix<double, 6, 1> room = room_d.normalized();
						bool moved = false;
						for (double size : {params.initial_step, 0.5 * params.initial_step})
						{
							const ObjectPlacement cand = shifted(at, room, size);
							InnerSolution rp = quick_solve(cand);
							result.num_inner_solves += 1;
							const std::vector<int> lost = lost_from(here_at, rp);
							if (lost.empty())
								add_planned_travel(cand, rp);
							const bool gained = lost_from(here_quick, rp).empty() && rp.num_reachable > here_quick.num_reachable;
							const bool ok = lost.empty() && rp.weighted_cost <= here_quick.weighted_cost + params.room_budget;
							if (params.trace_line_search)
								RCLCPP_INFO(
									node->get_logger(),
									"    room for%s step=%.4f m dir (%+.2f, %+.2f, %+.2f) abs (%.4f, %.4f, %.4f): reached %d, cost "
									"%.2f vs %.2f here (budget %.1f)  lost:%s -> %s",
									list(lost_all).c_str(), size, room(0), room(1), room(2), cand.x, cand.y, cand.z,
									rp.num_reachable, rp.weighted_cost, here_quick.weighted_cost, params.room_budget,
									list(lost).c_str(), gained ? "gained, accepted" : ok ? "moved" : "rejected");
							if (gained)
							{
								accepted = true;
								b_new = cand;
								next = std::move(rp);
								step = size;
								break;
							}
							if (ok)
							{
								at = cand;
								here_at = std::move(rp);
								moved = true;
								break;
							}
						}
						if (!moved)
							break;
					}
				}
			}

			// Reach rules: a small QP picks the step that keeps every reached viewpoint's joint room, clearance and
			// manipulability (linearized), shrinks missed gaps first, then lowers the cost. A real solve checks it.
			const double kGapProgress = 1e-4;  // m: smallest gap-sum drop that counts as progress
			auto rule_ok = [&](const InnerSolution& a, const InnerSolution& b) {
				if (a.num_reachable != b.num_reachable)
					return a.num_reachable > b.num_reachable;
				for (int v : b.tour)
					if (std::find(a.tour.begin(), a.tour.end(), v) == a.tour.end())
						return false;  // a swap: a reached viewpoint was lost
				if (!b.missed_vp.empty() && a.gap_sum < b.gap_sum - kGapProgress)
					return true;
				return a.gap_sum <= b.gap_sum + kGapProgress && a.weighted_cost < b.weighted_cost + slack;
			};
			if (params.reach_rules)
			{
				const Eigen::Isometry3d xform = MakePlacement(base);
				SetObjectPose(planning_scene_monitor, xform * object_pose_obj);
				std::vector<ReachRule> rules;
				int nz = 0;
				for (size_t k = 0; k < here_quick.tour.size() && k < here_quick.joints.size(); ++k)
				{
					const int v = here_quick.tour[k];
					nz += ReachedPoseRules(
						planning_scene_monitor, state, jmg, tool0_link, base,
						(xform * tour_poses_obj[static_cast<size_t>(v)]).translation(), here_quick.joints[k], v, params,
						nz, rules);
				}
				const double kZMax = 0.3;  // rad: largest null-space joint motion per pose per step
				std::vector<Eigen::VectorXd> rule_row;
				std::vector<Eigen::Matrix<double, 6, 1>> gap_h;
				std::vector<double> rule_lo, gaps;
				std::vector<int> gap_vp;
				for (const ReachRule& r : rules)
				{
					Eigen::VectorXd row = Eigen::VectorXd::Zero(6 + nz);
					row.head<6>() = to_u(r.grad);
					row.segment(6 + r.z_off, r.grad_z.size()) = r.grad_z;
					rule_row.push_back(row);
					rule_lo.push_back(r.floor - r.value);
				}
				for (size_t m = 0; m < here_quick.missed_vp.size() && m < here_quick.missed_gap.size(); ++m)
				{
					const int v = here_quick.missed_vp[m];
					const double gap = WeightedPoseGap(here_quick.missed_gap[m], params.rot_metric_scale);
					if (gap < 1e-9)
						continue;
					Eigen::Matrix<double, 6, 1> weighted_error = here_quick.missed_gap[m];
					weighted_error.tail<3>() *= params.rot_metric_scale * params.rot_metric_scale;
					const Eigen::Vector3d p = (xform * tour_poses_obj[static_cast<size_t>(v)]).translation();
					gap_h.push_back(to_u(PlacementJacobian(base, p).transpose() * weighted_error / gap));
					gaps.push_back(gap);
					gap_vp.push_back(v);
				}
				if (params.trace_line_search)
					RCLCPP_INFO(
						node->get_logger(), "    reach rules: %zu rules on %zu reached viewpoints (%d null-space motions), %zu missed gaps",
						rules.size(), here_quick.tour.size(), nz, gaps.size());

				// Box in the step metric: trust radius, placement bounds, locked axes.
				const double scale[6] = {1.0, 1.0, 1.0, rot_scale, rot_scale, rot_scale};
				const ObjectPlacement lo_b{bb.x_min, bb.y_min, bb.z_min, bb.roll_min, bb.pitch_min, bb.yaw_min};
				const ObjectPlacement hi_b{bb.x_max, bb.y_max, bb.z_max, bb.roll_max, bb.pitch_max, bb.yaw_max};
				const Eigen::Matrix<double, 6, 1> to_lo = ToOffsetVec(lo_b, base), to_hi = ToOffsetVec(hi_b, base);
				const double big = std::max(1.0, gnorm);
				double delta = params.initial_step;
				for (int ls = 0; ls < params.max_line_search_iters && !accepted; ++ls, delta *= params.step_shrink)
				{
					Eigen::Matrix<double, 6, 1> box_lo, box_hi;
					for (int k = 0; k < 6; ++k)
					{
						const bool locked = to_u(Eigen::Matrix<double, 6, 1>::Unit(k))(k) == 0.0;
						box_lo(k) = locked ? 0.0 : std::min(0.0, std::max(-delta, to_lo(k) * scale[k]));
						box_hi(k) = locked ? 0.0 : std::max(0.0, std::min(delta, to_hi(k) * scale[k]));
					}
					Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
					Eigen::VectorXd z;
					std::string qp_status;
					// Objective scaled by 1/big (same answer, better conditioned): gaps weigh 100x the cost slope.
					if (!SolveReachStep(
							rule_row, rule_lo, gap_h, gaps, g_u / big, 1.0 / delta, 100.0, box_lo, box_hi, nz, kZMax, d, z,
							qp_status))
					{
						RCLCPP_WARN(
							node->get_logger(), "    reach rules: QP failed at box %.4f m (%s)", delta, qp_status.c_str());
						continue;
					}
					if (params.trace_line_search && qp_status != "solved")
						RCLCPP_INFO(node->get_logger(), "    reach rules: QP %s", qp_status.c_str());
					if (d.norm() < 1e-4)  // under 0.1 mm: not worth a solve
					{
						if (params.trace_line_search)
							RCLCPP_INFO(node->get_logger(), "    reach rules: no move keeps every rule");
						break;
					}
					ObjectPlacement cand = ProjectToBounds(
						{base.x + d(0), base.y + d(1), base.z + d(2), base.roll + d(3) / rot_scale,
						 base.pitch + d(4) / rot_scale, base.yaw + d(5) / rot_scale},
						params.bounds);
					InnerSolution probe = quick_solve(cand);
					result.num_inner_solves += 1;
					if (probe.num_reachable >= here_quick.num_reachable)
						add_planned_travel(cand, probe);
					const bool ok = rule_ok(probe, here_quick);
					if (params.trace_line_search)
					{
						// Rules the step presses against (predicted value at its floor), and predicted gaps.
						std::string tight, pred;
						int num_tight = 0;
						for (size_t i = 0; i < rules.size(); ++i)
						{
							Eigen::VectorXd x(6 + nz);
							x << d, z;
							const double v = rules[i].value + rule_row[i].dot(x);
							if (v - rules[i].floor <= 0.05 * std::abs(rules[i].value - rules[i].floor) + 1e-6 && num_tight++ < 6)
								tight += " vp" + std::to_string(rules[i].vp) + ":" + rules[i].what;
						}
						for (size_t m = 0; m < gaps.size(); ++m)
						{
							char buf[64];
							std::snprintf(
								buf, sizeof(buf), " vp%d %.1f->%.1f", gap_vp[m], 1e3 * gaps[m],
								1e3 * std::max(0.0, gaps[m] + gap_h[m].dot(d)));
							pred += buf;
						}
						std::string lost;
						for (int v : here_quick.tour)
							if (std::find(probe.tour.begin(), probe.tour.end(), v) == probe.tour.end())
								lost += " " + std::to_string(v);
						RCLCPP_INFO(
							node->get_logger(),
							"    rule step box=%.4f m |d|=%.4f dir (%+.2f, %+.2f, %+.2f) spin %+.2f deg abs (%.4f, %.4f, %.4f): "
							"reached %d vs %d here, gap sum %.1f vs %.1f mm, cost %.2f vs %.2f here, null-space max %.2f rad  lost:%s -> %s",
							delta, d.norm(), d(0) / std::max(1e-12, d.head<3>().norm()),
							d(1) / std::max(1e-12, d.head<3>().norm()), d(2) / std::max(1e-12, d.head<3>().norm()),
							d(5) / rot_scale * 180.0 / M_PI, cand.x, cand.y, cand.z, probe.num_reachable,
							here_quick.num_reachable, 1e3 * probe.gap_sum, 1e3 * here_quick.gap_sum, probe.weighted_cost,
							here_quick.weighted_cost, z.size() ? z.cwiseAbs().maxCoeff() : 0.0, lost.empty() ? " none" : lost.c_str(),
							ok ? "accepted" : "rejected");
						RCLCPP_INFO(
							node->get_logger(), "      predicted gaps (mm):%s | tight rules (%d):%s", pred.empty() ? " none" : pred.c_str(),
							num_tight, tight.empty() ? " none" : tight.c_str());
					}
					if (ok)
					{
						accepted = true;
						b_new = cand;
						next = std::move(probe);
						step = d.norm();
					}
				}
			}

			InnerSolution committed;
			if (accepted)
			{
				ReseedRng(params.random_seed);
				committed = BestOfNInnerSolve(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, seeds,
					start_reference_joints, home_tcp_local, b_new, params, &next.tour,
					params.max_solutions_per_candidate, order_lock);
				result.num_inner_solves += std::max(1, params.gtsp_num_restart);
				add_planned_travel(b_new, committed);
				if (better(next, committed))
					committed = std::move(next);
				const bool keep = params.reach_rules ? rule_ok(committed, cur) : acceptable(committed, cur);
				if (params.trace_line_search)
					RCLCPP_INFO(
						node->get_logger(), "    full solve: reached %d vs %d current, cost %.2f vs %.2f current -> %s",
						committed.num_reachable, cur.num_reachable, committed.weighted_cost, cur.weighted_cost,
						keep ? "accepted" : "rejected");
				if (!keep)
					accepted = false;  // quick probe looked better but the full solve isn't
			}

			// While viewpoints are missed the weight is held, so retrying the same offset changes nothing.
			if (!accepted && annealing && cur.all_reachable)
			{
				params.manipulability_weight = lambda_final;
				continue;
			}
			if (!accepted && unlock_yaw())
				continue;
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
					Eigen::Vector3d(-g.head<3>()), base, state, jmg, cur);
				break;
			}

			Eigen::Matrix<double, 6, 1> du = ToOffsetVec(b_new, base);
			du(3) *= rot_scale;
			du(4) *= rot_scale;
			du(5) *= rot_scale;
			double base_move = du.norm();
			double rel_impr = (cur.weighted_cost - committed.weighted_cost) / std::max(std::abs(cur.weighted_cost), 1e-9);
			// Progress even if the cost rose: a viewpoint gained, or (reach rules) missed gaps shrank.
			const bool gained = committed.num_reachable > cur.num_reachable ||
				(params.reach_rules && committed.num_reachable == cur.num_reachable &&
				 committed.gap_sum < cur.gap_sum - kGapProgress);

			base = b_new;
			cur = std::move(committed);
			remember(cur);
			base_history.push_back(base);
			record(base, cur);

			RCLCPP_INFO(
				node->get_logger(),
				"restart %d/%d iter %d/%d: abs (%.4f, %.4f, %.4f) m  tip %.1f tilt %.1f spin %.1f deg  reachable %d/%d  "
				"|grad|=%.4f  step=%.4f m",
				restart_idx + 1, std::max(1, params.descent_num_restart), outer + 1, params.max_outer_iterations, base.x,
				base.y, base.z, base.roll * 180.0 / M_PI, base.pitch * 180.0 / M_PI, base.yaw * 180.0 / M_PI, cur.num_reachable, n, gnorm, step);
			log_cost_terms(cur);
			log_missed(base, cur);

			PublishProgress(
				node, params, object_pose_obj, tour_poses_obj, base_history,
				Eigen::Vector3d(-g.head<3>()), base, state, jmg, cur);

			if (!gained && rel_impr < params.convergence_tolerance_cost)
			{
				if (++stall_count >= std::max(1, params.patience))
				{
					if (annealing)
					{
						params.manipulability_weight = lambda_final;
						stall_count = 0;
						continue;
					}
					if (unlock_yaw())
						continue;
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

			if (!gained && base_move < params.convergence_tolerance_offset && rel_impr < params.convergence_tolerance_cost)
			{
				if (annealing)
				{
					params.manipulability_weight = lambda_final;
					stall_count = 0;
					continue;
				}
				if (unlock_yaw())
					continue;
				RCLCPP_INFO(node->get_logger(), "  restart %d: placement settled -- converged", restart_idx + 1);
				break;
			}
		}

		// Option "keep descending": refine wherever the descent ended.
		if (params.refine_solves > 0 && !rr.sol.tour.empty())
		{
			InnerSolution start = rr.sol;
			if (use_real_cost && start.real_cost < 0.0)
				start.real_cost = real_cost_of(rr.placement, start);
			rr.sol = refine(rr.placement, start, params.refine_solves);  // best of start and the refine solves
			rr.cost = cost_at_final_weight(rr.sol);
			rr.ok = rr.sol.all_reachable;
			RCLCPP_INFO(
				node->get_logger(),
				"  restart %d: refined %d solves at final abs (%.4f, %.4f, %.4f) m: travel %.2f -> %.2f  real %.2f -> %.2f",
				restart_idx + 1, params.refine_solves, rr.placement.x, rr.placement.y, rr.placement.z, start.travel_cost,
				rr.sol.travel_cost, start.real_cost, rr.sol.real_cost);
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
						   params.initial_roll, params.initial_pitch, params.initial_yaw}
					 : PerturbOffset(overall.placement, rng, params.descent_restart_perturbation, rot_scale);
		RestartResult rr = run_descent(start, r, r == 0 ? nullptr : &overall.sol);

		// weighted_cost carries the unreachable penalty, so lower cost == better (a fully-
		// reachable result always beats a partial one).
		bool improved = (r == 0) || (rr.cost < overall.cost);

		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d done: total cost=%.4f  travel+miss=%.2f  sum_w=%.3f  reachable %d/%d  abs (%.4f, %.4f, %.4f) m"
			"  tip %.1f tilt %.1f spin %.1f deg%s",
			r + 1, descent_num_restart, rr.cost,
			rr.sol.travel_cost + params.unreachable_penalty * (n - rr.sol.num_reachable), rr.sol.sum_manipulability,
			rr.sol.num_reachable, n,
			rr.placement.x, rr.placement.y, rr.placement.z, rr.placement.roll * 180.0 / M_PI, rr.placement.pitch * 180.0 / M_PI, rr.placement.yaw * 180.0 / M_PI, (r > 0 && improved) ? "  <-- new best" : "");

		if (improved)
			overall = rr;
	}

	const InnerSolution& fin = overall.sol;
	result.x = overall.placement.x;
	result.y = overall.placement.y;
	result.z = overall.placement.z;
	result.roll = overall.placement.roll;
	result.pitch = overall.placement.pitch;
	result.yaw = overall.placement.yaw;
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
			"Done. Object abs (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg, spin %.2f deg -- reaches "
			"all %d poses, tour joint path %.4f rad, travel %.2f, sum_w %.3f (weighted cost %.4f).",
			result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI, result.yaw * 180.0 / M_PI, n,
			result.total_joint_path_length, fin.travel_cost, fin.sum_manipulability, result.total_weighted_cost);
	else
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Object abs (%.4f, %.4f, %.4f) m, tip %.2f deg, tilt %.2f deg, spin %.2f deg -- reaches "
			"only %d/%d poses.",
			result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI,
			result.yaw * 180.0 / M_PI, result.num_reachable, n);

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
	root["yaw"] = result.yaw;
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
		entry["yaw"] = h[6];
		entry["weighted_cost"] = h[7];
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
	const Eigen::Vector3d& object_translation_nominal, double x, double y, double z, double roll, double pitch,
	double yaw)
{
	return MakePlacement(ObjectPlacement{x, y, z, roll, pitch, yaw}) * Eigen::Translation3d(-object_translation_nominal);
}

void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal, double x, double y, double z, double roll, double pitch,
	double yaw)
{
	SetObjectPose(
		planning_scene_monitor,
		MakePlacement(ObjectPlacement{x, y, z, roll, pitch, yaw}) * MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal));
}

visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const BaseGradientResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	const Eigen::Isometry3d xform = PlacementTransform(
		object_translation_nominal, result.x, result.y, result.z, result.roll, result.pitch, result.yaw);
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
