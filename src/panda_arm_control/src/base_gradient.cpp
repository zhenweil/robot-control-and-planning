#include <cmath>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <thread>
#include <numeric>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <json/json.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random_numbers/random_numbers.h>

#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/collision_detection/collision_common.h>

#include "panda_arm_control/base_gradient.hpp"
#include "panda_arm_control/real_cost_planning.hpp"

namespace
{

// Object placement: absolute position (x, y, z, base frame); the orientation stays nominal.
struct ObjectPlacement
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

// Maps poses given relative to the object's position (base-frame axes, "_obj") to absolute poses.
Eigen::Isometry3d MakePlacement(const ObjectPlacement& p)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
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
	return p;
}

Eigen::Vector3d ToOffsetVec(const ObjectPlacement& a, const ObjectPlacement& b)  // a - b, component-wise
{
	return Eigen::Vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
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

// Links that move rigidly with the tool, plus anything hanging off them (hand, fingers): the path check ignores them.
std::set<std::string> ToolSideLinks(const moveit::core::LinkModel* tool0_link)
{
	std::set<std::string> links;
	const moveit::core::LinkModel* wrist = moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(tool0_link);
	links.insert(wrist->getName());
	for (const moveit::core::JointModel* joint : wrist->getChildJointModels())
		for (const moveit::core::LinkModel* link : joint->getDescendantLinkModels())
			links.insert(link->getName());
	return links;
}

// True if a thin rod from the tool to the target hits nothing (object or arm; the tool side excepted). A closest
// pose with something in between can't slide onto the viewpoint, so its gap and slope would mislead.
bool ClearPathToTarget(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, moveit::core::RobotState& state,
	const moveit::core::LinkModel* tool0_link, const Eigen::Vector3d& target, const std::set<std::string>& tool_side)
{
	const Eigen::Vector3d local = state.getGlobalLinkTransform(tool0_link).inverse() * target;
	const double length = local.norm();
	if (length < 0.01)
		return true;  // already at the viewpoint
	Eigen::Isometry3d rod_pose = Eigen::Isometry3d::Identity();
	rod_pose.translation() = 0.5 * local;  // cylinder axis is z, centered on the shape pose
	rod_pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), local / length).toRotationMatrix();
	const std::string id = "path_to_target";
	state.attachBody(
		id, Eigen::Isometry3d::Identity(), {std::make_shared<shapes::Cylinder>(0.005, length)}, {rod_pose}, tool_side,
		tool0_link->getName());
	state.update();
	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
		locked_scene->checkCollision(req, res, state);
	}
	state.clearAttachedBody(id);
	return !res.collision;
}

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

	const std::set<std::string> tool_side = ToolSideLinks(tool0_link);
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
		if (gap < best.gap && IsStateCollisionFree(planning_scene_monitor, &state, jmg, q.data()) &&
			ClearPathToTarget(planning_scene_monitor, state, tool0_link, target_pose.translation(), tool_side))
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

// Floor on w, so a singular configuration doesn't divide by zero.
constexpr double kMinManipulability = 1e-6;

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
	// Missed viewpoints and their closest-IK pose gaps; miss_cost = weight * sum of capped gaps.
	std::vector<int> missed_vp;
	std::vector<Eigen::Matrix<double, 6, 1>> missed_gap;
	std::vector<std::vector<double>> missed_q;  // parallel: closest collision-free pose, empty if none
	double miss_cost = 0.0;  // miss_gap_weight * sum of capped gaps
	double softmin_manipulability = 0.0;  // SoftMin of w over visited (and included missed) poses
	double gap_sum = 0.0;		 // unweighted sum of capped miss gaps (m)
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
			node_cost[i].push_back(-params.manipulability_weight * w);
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
	if (params.manipulability_include_missed)
	{
		out.sum_manipulability += SumManipulability(state, jmg, tool0_link, missed_q, params.manipulability_limit_sharpness);
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
	out.num_rescued = num_rescued;
	out.num_no_free = num_no_free;
	out.missed_vp = std::move(missed_vp);
	out.missed_gap = std::move(missed_gap);
	out.missed_q = std::move(missed_q);
	for (const auto& e : out.missed_gap)
		out.gap_sum += std::min(params.miss_gap_cap, WeightedPoseGap(e, params.rot_metric_scale));
	out.miss_cost = params.miss_gap_weight * out.gap_sum;

	out.weighted_cost = (params.travel_in_cost ? out.travel_cost : 0.0) - params.manipulability_weight * out.sum_manipulability -
		params.softmin_manipulability_weight * out.softmin_manipulability +
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
			r.weighted_cost -= params.manipulability_weight * w;
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

// Jacobian of a viewpoint's pose w.r.t. the object position: 6x3, rows [position; rotation]. Moving the object
// moves every viewpoint by the same amount and leaves its orientation alone.
Eigen::Matrix<double, 6, 3> PlacementJacobian()
{
	Eigen::Matrix<double, 6, 3> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
	jacobian.topRows<3>().setIdentity();
	return jacobian;
}

// dw/db: how an object placement change moves one viewpoint's manipulability, the arm tracking it from q.
// Its positive side keeps the viewpoint away from the reach edge.
Eigen::Vector3d ManipulabilityOffsetGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<double>& q, double damping, double limit_sharpness)
{
	state.setJointGroupPositions(jmg, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
	Eigen::MatrixXd JJt = J * J.transpose();
	JJt.diagonal().array() += damping * damping;
	const Eigen::MatrixXd dq_db =
		J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6)) * PlacementJacobian();
	return dq_db.transpose() * ManipulabilityJointGradient(state, jmg, tool0_link, q, limit_sharpness);
}

struct ElbowSwingResult
{
	double travel_before = 0.0;  // joint path length home -> tour (rad)
	double travel_after = 0.0;
	int big_moves_before = 0;  // moves where one joint turns more than elbow_swing_jump_threshold
	int big_moves_after = 0;
	int iters = 0;
	int fail_ik = 0, fail_limits = 0, fail_collision = 0, fail_longer = 0;  // rejected tries, by first failed check
	ObjectPlacement placement;
	std::vector<std::vector<double>> joints;  // parallel to the tour
};

// Elbow-swing smoothing: shortens the tour's joint path (home first) for a fixed viewpoint order. Each step moves
// the object along the summed pull of all poses (each pose follows it through its Jacobian) and slides each pose
// along its self-motion (Jacobian null space), so the tool stays on its viewpoint. Small IK corrections remove
// drift; a step is kept only if all poses stay collision-free and within limits and the goal drops. Goal: path
// length plus jump_weight * sum over moves and joints of (turn - threshold)^2 past the threshold, so big jumps
// (reconfigurations) shrink first.
ElbowSwingResult ElbowSwingSmooth(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& joints, const std::vector<double>& home_joints,
	const ObjectPlacement& base, const BaseGradientParams& params)
{
	const size_t n = tour.size();
	const int dof = static_cast<int>(jmg->getVariableCount());
	auto to_vec = [](const std::vector<double>& q) { return Eigen::Map<const Eigen::VectorXd>(q.data(), q.size()); };
	auto path_len = [&](const std::vector<std::vector<double>>& qs) {
		double len = 0.0;
		const std::vector<double>* prev = &home_joints;
		for (const auto& q : qs)
		{
			len += JointL2Distance(*prev, q);
			prev = &q;
		}
		return len;
	};
	const double thr = params.elbow_swing_jump_threshold, w_jump = params.elbow_swing_jump_weight;
	auto goal = [&](const std::vector<std::vector<double>>& qs) {
		double f = path_len(qs);
		for (size_t k = 0; k < qs.size(); ++k)
			for (size_t j = 0; j < qs[k].size(); ++j)
			{
				const double ex = std::abs(qs[k][j] - (k == 0 ? home_joints : qs[k - 1])[j]) - thr;
				if (ex > 0.0)
					f += w_jump * ex * ex;
			}
		return f;
	};
	auto big_moves = [&](const std::vector<std::vector<double>>& qs) {
		int c = 0;
		for (size_t k = 0; k < qs.size(); ++k)
			c += MaxJointDeviation(k == 0 ? home_joints : qs[k - 1], qs[k]) > thr ? 1 : 0;
		return c;
	};
	auto jacobian = [&](const std::vector<double>& q) {
		state.setJointGroupPositions(jmg, q);
		state.update();
		Eigen::MatrixXd J;  // 6 x dof, [linear; angular] in the base frame
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
		return J;
	};
	auto pinv = [&](const Eigen::MatrixXd& J) {
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += params.jacobian_damping * params.jacobian_damping;
		return Eigen::MatrixXd(J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6)));
	};
	// Pulls q onto `target` with a few damped Newton steps, clamped to the joint limits (a pose at a limit
	// would otherwise fail every step); false if it doesn't get within tolerance.
	auto correct = [&](std::vector<double>& q, const Eigen::Isometry3d& target) {
		for (int it = 0; it < 10; ++it)
		{
			jmg->enforcePositionBounds(q.data());
			state.setJointGroupPositions(jmg, q);
			state.update();
			const Eigen::Isometry3d& tcp = state.getGlobalLinkTransform(tool0_link);
			Eigen::Matrix<double, 6, 1> e;
			e.head<3>() = target.translation() - tcp.translation();
			const Eigen::AngleAxisd aa(target.rotation() * tcp.rotation().transpose());
			e.tail<3>() = aa.angle() * aa.axis();
			if (e.head<3>().norm() < 1e-5 && e.tail<3>().norm() < 1e-4)
				return true;
			Eigen::MatrixXd J;
			state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
			const Eigen::VectorXd dq = pinv(J) * e;
			for (int j = 0; j < dof; ++j)
				q[static_cast<size_t>(j)] += dq(j);
		}
		return false;
	};

	ElbowSwingResult out;
	out.placement = base;
	out.joints = joints;
	out.travel_before = out.travel_after = path_len(joints);
	out.big_moves_before = out.big_moves_after = big_moves(joints);
	double best_goal = goal(joints);
	if (n == 0)
		return out;
	double object_step = params.elbow_swing_object_step;
	double joint_step = params.elbow_swing_joint_step;
	for (int iter = 0; iter < params.elbow_swing_iters && object_step > 1e-5; ++iter)
	{
		// Pull on each pose: the slope of the goal (home edge + both neighbours).
		std::vector<Eigen::VectorXd> g(n, Eigen::VectorXd::Zero(dof));
		for (size_t k = 0; k < n; ++k)
		{
			const Eigen::VectorXd d = to_vec(out.joints[k]) - to_vec(k == 0 ? home_joints : out.joints[k - 1]);
			if (d.norm() < 1e-9)
				continue;
			Eigen::VectorXd gd = d.normalized();
			for (int j = 0; j < dof; ++j)
			{
				const double ex = std::abs(d(j)) - thr;
				if (ex > 0.0)
					gd(j) += 2.0 * w_jump * ex * (d(j) > 0.0 ? 1.0 : -1.0);
			}
			g[k] += gd;
			if (k > 0)
				g[k - 1] -= gd;
		}
		std::vector<Eigen::MatrixXd> Jp(n), N(n);
		Eigen::Vector3d g_obj = Eigen::Vector3d::Zero();
		for (size_t k = 0; k < n; ++k)
		{
			const Eigen::MatrixXd J = jacobian(out.joints[k]);
			Jp[k] = pinv(J);
			N[k] = Eigen::MatrixXd::Identity(dof, dof) - Jp[k] * J;
			g_obj += (Jp[k] * PlacementJacobian()).transpose() * g[k];  // path slope w.r.t. the object position
		}

		bool moved = false;
		while (object_step > 1e-5 && !moved)
		{
			const Eigen::Vector3d dp = g_obj.norm() > 1e-9 ? Eigen::Vector3d(-object_step * g_obj.normalized())
														  : Eigen::Vector3d::Zero();
			const ObjectPlacement cand = ProjectToBounds(
				{out.placement.x + dp(0), out.placement.y + dp(1), out.placement.z + dp(2)}, params.bounds);
			const Eigen::Vector3d dp_real = ToOffsetVec(cand, out.placement);
			const Eigen::Isometry3d xform = MakePlacement(cand);
			SetObjectPose(planning_scene_monitor, xform * object_pose_obj);
			std::vector<std::vector<double>> qs = out.joints;
			bool ok = true;
			for (size_t k = 0; k < n && ok; ++k)
			{
				const Eigen::VectorXd dq =
					Jp[k] * PlacementJacobian() * dp_real - joint_step * N[k] * g[k];
				for (int j = 0; j < dof; ++j)
					qs[k][static_cast<size_t>(j)] += dq(j);
				if (!correct(qs[k], xform * tour_poses_obj[static_cast<size_t>(tour[k])]))
				{
					++out.fail_ik;
					ok = false;
				}
				else if (!jmg->satisfiesPositionBounds(qs[k].data()))
				{
					++out.fail_limits;
					ok = false;
				}
				else if (!IsStateCollisionFree(planning_scene_monitor, &state, jmg, qs[k].data()))
				{
					++out.fail_collision;
					ok = false;
				}
			}
			const double f = ok ? goal(qs) : std::numeric_limits<double>::max();
			if (ok && f >= best_goal - 1e-6)
				++out.fail_longer;
			if (f < best_goal - 1e-6)
			{
				best_goal = f;
				out.placement = cand;
				out.joints = std::move(qs);
				out.travel_after = path_len(out.joints);
				out.big_moves_after = big_moves(out.joints);
				out.iters = iter + 1;
				object_step = std::min(1.5 * object_step, params.elbow_swing_object_step);
				joint_step = std::min(1.5 * joint_step, params.elbow_swing_joint_step);
				moved = true;
			}
			else
			{
				object_step *= 0.5;
				joint_step *= 0.5;
			}
		}
		if (!moved)
			break;
	}
	SetObjectPose(planning_scene_monitor, MakePlacement(out.placement) * object_pose_obj);
	return out;
}

// A wall learned from a probe step that lost viewpoint vp: steps within kWallHalfAngle of `n` (the failed step's
// direction, unit) are blocked. `room` is vp's comfort direction that the step went against (zero if unknown).
// Forgotten once the object is far from where it was learned.
struct Wall
{
	Eigen::Vector3d n = Eigen::Vector3d::Zero();
	Eigen::Vector3d room = Eigen::Vector3d::Zero();
	ObjectPlacement at;
	int vp = -1;
};

// ---------------------------------------------------------------------------------------------
// Analytic gradient of the cost (travel, manipulability terms, missed-viewpoint gaps) w.r.t. the object
// position (x, y, z). Joints follow a moved viewpoint as dq/db = J_pinv * placement_jacobian
// (damped pseudo-inverse of the tool Jacobian); each edge, manipulability term and miss gap is chained
// through that. The home pose doesn't move with the object.
// ---------------------------------------------------------------------------------------------

Eigen::Vector3d AnalyticGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& joints, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params,
	const std::vector<int>& missed_vp, const std::vector<Eigen::Matrix<double, 6, 1>>& missed_gap,
	const std::vector<std::vector<double>>& missed_q)
{
	const size_t n = tour.size();
	Eigen::Isometry3d xform = MakePlacement(base);
	const double lambda2 = params.jacobian_damping * params.jacobian_damping;

	// Per visit position: dq/db (dof x 3) and dp/db (3 x 3).
	std::vector<Eigen::MatrixXd> dq_db(n);
	std::vector<Eigen::MatrixXd> dp_db(n);
	const Eigen::Matrix<double, 6, 3> placement_jacobian = PlacementJacobian();

	for (size_t k = 0; k < n; ++k)
	{
		state.setJointGroupPositions(jmg, joints[k]);
		state.update();
		Eigen::MatrixXd J;  // 6 x dof, [linear; angular] in the model (base) frame
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);

		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += lambda2;
		Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));

		dq_db[k] = J_pinv * placement_jacobian;		   // dof x 3
		dp_db[k] = placement_jacobian.topRows<3>();	   // 3 x 3
	}

	const size_t dof = home_joints.size();
	Eigen::Vector3d g = Eigen::Vector3d::Zero();
	// Soft-min term: per pose w and dw/db, combined once all poses are in.
	std::vector<double> softmin_w;
	std::vector<Eigen::Vector3d> softmin_dw_db;
	for (size_t k = 0; k < n && params.manipulability_gradient; ++k)
	{
		// d(-lambda w)/db = -lambda (dq/db)^T dw/dq
		const double w = std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, joints[k], params.manipulability_limit_sharpness));
		const Eigen::Vector3d dw_db = dq_db[k].transpose() *
			ManipulabilityJointGradient(state, jmg, tool0_link, joints[k], params.manipulability_limit_sharpness);
		g -= params.manipulability_weight * dw_db;
		softmin_w.push_back(w);
		softmin_dw_db.push_back(dw_db);
	}
	// Missed viewpoints: same chain at the closest pose, as if the arm followed the viewpoint.
	for (size_t m = 0; m < missed_vp.size() && m < missed_q.size() && params.manipulability_include_missed; ++m)
	{
		const std::vector<double>& q = missed_q[m];
		if (q.empty())
			continue;
		state.setJointGroupPositions(jmg, q);
		state.update();
		Eigen::MatrixXd J;
		state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), J);
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += lambda2;
		const Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));
		const Eigen::MatrixXd dq_db_missed = J_pinv * placement_jacobian;
		const double w = std::max(kMinManipulability, Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness));
		if (params.manipulability_gradient)
		{
			const Eigen::Vector3d dw_db = dq_db_missed.transpose() *
				ManipulabilityJointGradient(state, jmg, tool0_link, q, params.manipulability_limit_sharpness);
			g -= params.manipulability_weight * dw_db;
			softmin_w.push_back(w);
			softmin_dw_db.push_back(dw_db);
		}
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
		g += params.miss_gap_weight * placement_jacobian.transpose() * weighted_error / gap;
	}

	auto add_edge = [&](const std::vector<double>& qa, const std::vector<double>& qb, const Eigen::MatrixXd& dqa,
					   const Eigen::MatrixXd& dqb, const Eigen::MatrixXd& dpa, const Eigen::MatrixXd& dpb,
					   const Eigen::Vector3d& pa, const Eigen::Vector3d& pb) {
		Eigen::VectorXd dq(dof);
		for (size_t i = 0; i < dof; ++i)
			dq(i) = qa[i] - qb[i];
		Eigen::MatrixXd ddiff = dqa - dqb;  // dof x 3

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

	Eigen::MatrixXd zero_dq = Eigen::MatrixXd::Zero(dof, 3);
	Eigen::MatrixXd zero_dp = Eigen::MatrixXd::Zero(3, 3);

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
Eigen::Vector3d FiniteDifferenceGradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_obj, const std::vector<Eigen::Isometry3d>& tour_poses_obj,
	const std::vector<int>& tour, const std::vector<std::vector<double>>& seeds, const std::vector<double>& home_joints,
	const Eigen::Vector3d& home_tcp_local, const ObjectPlacement& base, const BaseGradientParams& params)
{
	const double eps = params.fd_epsilon;
	Eigen::Vector3d g = Eigen::Vector3d::Constant(std::nan(""));
	for (int axis = 0; axis < 3; ++axis)
	{
		ObjectPlacement bp = base, bm = base;
		auto component = [](ObjectPlacement& o, int a) -> double& {
			return a == 0 ? o.x : a == 1 ? o.y : o.z;
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
	// Mutable copy: manipulability_after_reach switches the comfort weights off while viewpoints are missed.
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
	auto run_descent = [&](ObjectPlacement base, int restart_idx, const InnerSolution* warm) -> RestartResult {
		base = ProjectToBounds(base, params.bounds);
		params.random_seed = params_in.random_seed + restart_idx;  // each restart draws its own IK luck
		params.manipulability_weight = lambda_final;
		// manipulability_after_reach: comfort terms (lambda, soft-min) off while any viewpoint is missed, so the
		// step follows the miss gaps instead of being outvoted; back on once everything is reached.
		bool manipulability_on = !params.manipulability_after_reach;
		auto comfort_off = [&]() {
			params.manipulability_weight = 0.0;
			params.softmin_manipulability_weight = 0.0;
		};
		if (!manipulability_on)
			comfort_off();
		std::vector<ObjectPlacement> base_history{base};
		std::vector<std::vector<double>> seeds =
			warm ? SeedsFromSolution(*warm, fallback_seed, static_cast<size_t>(n))
				 : std::vector<std::vector<double>>(static_cast<size_t>(n), fallback_seed);
		int stall_count = 0;
		std::vector<Wall> walls;  // room-then-reach: remembered across iterations while nearby
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
			// Re-score at the full weight so points found with comfort off compare fairly.
			InnerSolution sf = s;
			sf.weighted_cost = travel_term(s) - lambda_final * s.sum_manipulability -
				params_in.softmin_manipulability_weight * s.softmin_manipulability +
				params.unreachable_penalty * (s.num_total - s.num_reachable) + s.miss_cost;
			result.history.push_back(
				{static_cast<double>(restart_idx), b.x, b.y, b.z, sf.weighted_cost});
			// Best seen: most viewpoints reached, then cost.
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
			row("soft-min manip", sol.softmin_manipulability, -params.softmin_manipulability_weight);
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
			"restart %d/%d iter 0: abs (%.4f, %.4f, %.4f) m  reachable %d/%d",
			restart_idx + 1, std::max(1, params.descent_num_restart), base.x, base.y, base.z, cur.num_reachable, n);
		log_cost_terms(cur);
		log_missed(base, cur);

		// Cost at the final weight, so solutions found at different weights compare fairly.
		auto cost_at_final_weight = [&](const InnerSolution& sol) {
			return travel_term(sol) - lambda_final * sol.sum_manipulability -
				params_in.softmin_manipulability_weight * sol.softmin_manipulability +
				params.unreachable_penalty * (sol.num_total - sol.num_reachable) + sol.miss_cost;
		};
		// Refine at a fixed placement: more full solves, each warm-started from the best so far, scored at the
		// final weight. With freeze_order the viewpoint order is kept and only the arm poses are re-picked.
		// Lowest real cost if enabled, else lowest L2 travel; never fewer viewpoints reached.
		auto refine = [&](const ObjectPlacement& at, InnerSolution best_sol, int num_solves) {
			// Travel-only tours: no manipulability preference when picking arm poses.
			const double saved_weight = params.manipulability_weight;
			params.manipulability_weight = 0.0;
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
				if (params.manipulability_after_reach && manipulability_on && !cur.all_reachable)
				{
					manipulability_on = false;
					comfort_off();
					RCLCPP_INFO(node->get_logger(), "  restart %d: viewpoint missed -- comfort terms off", restart_idx + 1);
				}
				else if (!manipulability_on && cur.all_reachable)
				{
					manipulability_on = true;
					params.manipulability_weight = lambda_final;
					params.softmin_manipulability_weight = params_in.softmin_manipulability_weight;
					RCLCPP_INFO(node->get_logger(), "  restart %d: all reached -- comfort terms on", restart_idx + 1);
				}
				cur.weighted_cost = travel_term(cur) - params.manipulability_weight * cur.sum_manipulability -
					params.softmin_manipulability_weight * cur.softmin_manipulability +
					params.unreachable_penalty * (cur.num_total - cur.num_reachable) + cur.miss_cost;
			}
			Eigen::Vector3d g = AnalyticGradient(
				state, jmg, tool0_link, tour_poses_obj, cur.tour, cur.joints, start_reference_joints,
				home_tcp_local, base, params, cur.missed_vp, cur.missed_gap, cur.missed_q);

			if (params.trace_line_search)
			{
				// Each term's own descent direction (translation part of -gradient) and size.
				auto part = [&](bool manip, double gap_weight) {
					BaseGradientParams p = params;
					p.travel_gradient = false;
					p.manipulability_gradient = manip;
					p.miss_gap_weight = gap_weight;
					return AnalyticGradient(
						state, jmg, tool0_link, tour_poses_obj, cur.tour, cur.joints, start_reference_joints,
						home_tcp_local, base, p, cur.missed_vp, cur.missed_gap, cur.missed_q);
				};
				auto describe = [](const Eigen::Vector3d& gp) {
					const Eigen::Vector3d d = -gp;
					const double nrm = d.norm();
					const Eigen::Vector3d u = nrm > 1e-9 ? Eigen::Vector3d(d / nrm) : Eigen::Vector3d::Zero();
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
				Eigen::Vector3d g_fd = FiniteDifferenceGradient(
					state, jmg, planning_scene_monitor, object_pose_obj, tour_poses_obj, cur.tour,
					cur.joints, start_reference_joints, home_tcp_local, base, params);
				RCLCPP_INFO(
					node->get_logger(),
					"    grad check [x y z]  analytic (%+.4f %+.4f %+.4f)  central-diff (%+.4f %+.4f %+.4f)", g(0), g(1),
					g(2), g_fd(0), g_fd(1), g_fd(2));
			}

			// Locked axes (min == max) get no share of the step.
			const BaseGradientBounds& bb = params.bounds;
			auto to_u = [&](Eigen::Vector3d v) {
				if (bb.x_min == bb.x_max)
					v(0) = 0.0;
				if (bb.y_min == bb.y_max)
					v(1) = 0.0;
				if (bb.z_min == bb.z_max)
					v(2) = 0.0;
				return v;
			};
			const Eigen::Vector3d g_u = to_u(g);
			double gnorm = g_u.norm();
			if (gnorm < 1e-6)
			{
				RCLCPP_INFO(node->get_logger(), "  restart %d: gradient ~ 0 -- converged", restart_idx + 1);
				PublishProgress(
					node, params, object_pose_obj, tour_poses_obj, base_history,
					Eigen::Vector3d(-g), base, state, jmg, cur);
				break;
			}

			// Backtracking line search. Each probe is one solve, compared with one solve at the current point
			// (same effort); the accepted offset gets a best-of-N solve, kept only if it beats the current one.
			Eigen::Vector3d dir_u = -g_u / gnorm;
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
			// Cost for comparisons: while either side misses a viewpoint, without planned travel -- it jumps when an
			// arm pose switches family and the gradient doesn't see it, so it mustn't veto reaching. Reaching first.
			auto cmp_cost = [&](const InnerSolution& s, bool reach_phase) {
				if (!reach_phase || s.real_cost < 0.0)
					return s.weighted_cost;
				return s.weighted_cost - (s.real_cost - (params.travel_in_cost ? s.travel_cost : 0.0));
			};
			auto reaching = [](const InnerSolution& a, const InnerSolution& b) {
				return !a.all_reachable || !b.all_reachable;
			};
			// More viewpoints reached wins; cost only breaks ties. Used for comparing and for accepting moves.
			auto better = [&](const InnerSolution& a, const InnerSolution& b) {
				const bool r = reaching(a, b);
				return a.num_reachable > b.num_reachable ||
					(a.num_reachable == b.num_reachable && cmp_cost(a, r) < cmp_cost(b, r));
			};
			// Viewpoint v is reached here but missed at cand: the direction (step metric) whose opposite loses it --
			// its manipulability slope (reach edge) or, failing that, its gap slope at cand. Zero if neither fits step_dir.
			// `from_sol`: the solution where the step starts (v is reached there).
			auto loss_direction = [&](int v, const InnerSolution& from_sol, const InnerSolution& probe,
									  const Eigen::Vector3d& step_dir) {
				const Eigen::Vector3d none = Eigen::Vector3d::Zero();
				const auto it = std::find(from_sol.tour.begin(), from_sol.tour.end(), v);
				if (it == from_sol.tour.end())
					return none;  // already missed where the step starts
				Eigen::Vector3d h = to_u(ManipulabilityOffsetGradient(
					state, jmg, tool0_link, from_sol.joints[static_cast<size_t>(it - from_sol.tour.begin())],
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
				h = -to_u(PlacementJacobian().transpose() * weighted_error / gap);
				return step_dir.dot(h) < 0.0 ? h : none;  // the step doesn't grow its gap either
			};
			// Room-then-reach walls: forget walls learned far away; aim_within_walls gives the direction nearest to `a`
			// (unit) outside every wall's cone, even one turned far from `a` (zero only if it can't get out).
			const double kWallRadius = 0.1;  // step metric (m): walls learned farther away no longer apply
			auto u_dist = [&](const ObjectPlacement& a, const ObjectPlacement& b) {
				return ToOffsetVec(a, b).norm();
			};
			walls.erase(
				std::remove_if(walls.begin(), walls.end(), [&](const Wall& w) { return u_dist(w.at, base) > kWallRadius; }),
				walls.end());
			const double kWallHalfAngle = M_PI / 4.0;  // a wall blocks steps within this angle of the failed step
			auto aim_within_walls = [&](const Eigen::Vector3d& a) {
				const Eigen::Vector3d none = Eigen::Vector3d::Zero();
				if (walls.empty() || a.norm() < 1e-9)
					return a;
				const Eigen::Vector3d a_u = a.normalized();
				// Push the direction out of each cone it is in, onto the cone's edge (just past it), until it is in none.
				const double edge = kWallHalfAngle + 0.01;
				Eigen::Vector3d d = a_u;
				for (int it = 0; it < 20; ++it)
				{
					bool moved = false;
					for (const Wall& w : walls)
					{
						if (d.dot(w.n) <= std::cos(kWallHalfAngle))
							continue;
						Eigen::Vector3d side = d - d.dot(w.n) * w.n;
						if (side.norm() < 1e-9)
							side = w.n.unitOrthogonal();
						Eigen::Vector3d next = to_u(std::cos(edge) * w.n + std::sin(edge) * side.normalized());
						if (next.norm() < 1e-9)
							return none;
						d = next.normalized();
						moved = true;
					}
					if (!moved)
						return d;
				}
				return none;  // the cones cover every nearby direction
			};
			// The direction farthest from every failed step: maximizes the smallest angle to any wall's axis. Ascent on
			// that smallest angle from each wall's opposite; zero if no walls.
			auto farthest_from_walls = [&]() -> Eigen::Vector3d {
				Eigen::Vector3d best = Eigen::Vector3d::Zero();
				double best_cos = 2.0;  // cosine to the nearest axis; smaller is farther
				auto nearest_cos = [&](const Eigen::Vector3d& d, const Wall** nearest) {
					double c = -2.0;
					for (const Wall& w : walls)
						if (d.dot(w.n) > c)
						{
							c = d.dot(w.n);
							*nearest = &w;
						}
					return c;
				};
				for (const Wall& start : walls)
				{
					Eigen::Vector3d d = to_u(-start.n);
					if (d.norm() < 1e-9)
						continue;
					d.normalize();
					for (int it = 0; it < 100; ++it)
					{
						const Wall* nearest = nullptr;
						nearest_cos(d, &nearest);
						// Turn away from the nearest axis, within the sphere.
						const Eigen::Vector3d away = -(nearest->n - nearest->n.dot(d) * d);
						if (away.norm() < 1e-9)
							break;
						const Eigen::Vector3d next = to_u(d + 0.05 * away.normalized());
						if (next.norm() < 1e-9)
							break;
						d = next.normalized();
					}
					const Wall* nearest = nullptr;
					const double c = nearest_cos(d, &nearest);
					if (c < best_cos)
					{
						best_cos = c;
						best = d;
					}
				}
				return best;
			};
			// Wall for viewpoint v lost by a step along `step_dir` from `at`; `room` is its comfort direction.
			auto add_wall = [&](int v, const Eigen::Vector3d& step_dir, const Eigen::Vector3d& room, const ObjectPlacement& at) {
				if (step_dir.norm() < 1e-9)
					return false;
				const Eigen::Vector3d n = step_dir.normalized();
				for (const Wall& w : walls)
					if (w.n.dot(n) > std::cos(0.1))
						return false;  // this step's cone is already there (one cone per failed step)
				walls.push_back({n, room, at, v});
				if (params.trace_line_search)
					RCLCPP_INFO(
						node->get_logger(), "    wall: vp %d, blocked dir (%+.2f, %+.2f, %+.2f), room (%+.2f, %+.2f, %+.2f), %zu walls",
						v, n(0), n(1), n(2), room(0), room(1), room(2), walls.size());
				return true;
			};
			if (params.room_then_reach && !walls.empty())
			{
				const Eigen::Vector3d d = aim_within_walls(dir_u);
				if (d.norm() >= 0.1)
					dir_u = d.normalized();  // stay off known walls
			}
			// Gradient line search: shrink the step until a probe is better.
			for (int ls = 0; ls < params.max_line_search_iters;)
			{
				ObjectPlacement cand = ProjectToBounds(
					{base.x + step * dir_u(0), base.y + step * dir_u(1), base.z + step * dir_u(2)},
					params.bounds);
				// weighted_cost carries the unreachable penalty, so a probe that drops a viewpoint fails.
				InnerSolution probe = quick_solve(cand);
				result.num_inner_solves += 1;
				if (probe.num_reachable >= here_quick.num_reachable)
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
				if (better(probe, here_quick))
				{
					accepted = true;
					b_new = cand;
					next = std::move(probe);
					break;
				}
				step *= params.step_shrink;
				++ls;
			}

			// Room then reach: stuck with misses, walk toward each missed viewpoint (closest first). An aim step that
			// reaches it is accepted; one that loses nothing walks on (whatever the cost); one that loses
			// viewpoints adds walls, then a room move along their room side gives the blockers room. Up to kMaxDetours.
			if (!accepted && params.room_then_reach && !here_quick.missed_vp.empty())
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
				auto shifted = [&](const ObjectPlacement& at, const Eigen::Vector3d& d, double size) {
					return ProjectToBounds(
						{at.x + size * d(0), at.y + size * d(1), at.z + size * d(2)},
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
						const Eigen::Vector3d aim_raw = -to_u(PlacementJacobian().transpose() * weighted_error);
						if (aim_raw.norm() < 1e-9)
							break;
						const Eigen::Vector3d aim_d = aim_within_walls(aim_raw.normalized());
						if (aim_d.norm() < 0.1)
						{
							if (params.trace_line_search)
								RCLCPP_INFO(node->get_logger(), "    aim vp %d: walls leave no way toward it", target);
							break;
						}
						const Eigen::Vector3d aim = aim_d.normalized();
						std::vector<int> lost_all;
						// Every lost viewpoint outvotes the one target: the room move follows their average comfort direction.
						Eigen::Vector3d room_raw = Eigen::Vector3d::Zero();
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
							// While viewpoints are missed the cost may rise: a walk-on step only has to lose nothing.
							const bool walk = !ok && lost.empty();
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
							{
								const Eigen::Vector3d comfort = loss_direction(v, here_at, probe, aim);
								add_wall(v, aim, comfort, at);
								if (comfort.norm() > 1e-9)
									room_raw += comfort.normalized();
								lost_all.push_back(v);
							}
							if (!lost.empty())
								break;  // walls learned: make room rather than shrink
						}
						if (accepted || detour == kMaxDetours)
							break;
						if (walked)
							continue;  // aim again from the new spot
						if (lost_all.empty())
							break;  // no loss, no gain, over budget: nothing to make room for

						// Room: along the lost viewpoints' comfort directions (outside all walls). While viewpoints are missed
						// the cost may rise here: a room move only has to lose nothing. A room move that loses viewpoints gets
						// its own wall; the second try goes the way still left: the direction farthest from every failed step.
						if (room_raw.norm() < 1e-9)
							break;
						bool moved = false;
						for (int attempt = 0; attempt < 2 && !moved && !accepted; ++attempt)
						{
							Eigen::Vector3d want = room_raw;
							if (attempt > 0)
							{
								want = farthest_from_walls();
								if (want.norm() < 1e-9)
									break;
							}
							const Eigen::Vector3d room_d = aim_within_walls(want.normalized());
							if (room_d.norm() < 0.1)
								break;
							const Eigen::Vector3d room = room_d.normalized();
							std::vector<int> room_lost;
							for (double size : {params.initial_step, 0.5 * params.initial_step})
							{
								const ObjectPlacement cand = shifted(at, room, size);
								InnerSolution rp = quick_solve(cand);
								result.num_inner_solves += 1;
								const std::vector<int> lost = lost_from(here_at, rp);
								if (lost.empty())
									add_planned_travel(cand, rp);
								const bool gained =
									lost_from(here_quick, rp).empty() && rp.num_reachable > here_quick.num_reachable;
								const bool ok = lost.empty();
								if (params.trace_line_search)
									RCLCPP_INFO(
										node->get_logger(),
										"    room for%s (try %d) step=%.4f m dir (%+.2f, %+.2f, %+.2f) abs (%.4f, %.4f, %.4f): "
										"reached %d, cost %.2f vs %.2f here  lost:%s -> %s",
										list(lost_all).c_str(), attempt + 1, size, room(0), room(1), room(2), cand.x, cand.y,
										cand.z, rp.num_reachable, rp.weighted_cost, here_quick.weighted_cost,
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
								if (room_lost.empty())
									room_lost = lost;
							}
							if (!moved && !accepted && !room_lost.empty())
								add_wall(room_lost.front(), room, Eigen::Vector3d::Zero(), at);
						}
						if (!moved)
							break;
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
				const bool keep = better(committed, cur);
				if (params.trace_line_search)
					RCLCPP_INFO(
						node->get_logger(), "    full solve: reached %d vs %d current, cost %.2f vs %.2f current -> %s",
						committed.num_reachable, cur.num_reachable, committed.weighted_cost, cur.weighted_cost,
						keep ? "accepted" : "rejected");
				if (!keep)
					accepted = false;  // quick probe looked better but the full solve isn't
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
					Eigen::Vector3d(-g), base, state, jmg, cur);
				break;
			}

			double base_move = ToOffsetVec(b_new, base).norm();
			double rel_impr = (cur.weighted_cost - committed.weighted_cost) / std::max(std::abs(cur.weighted_cost), 1e-9);
			// Progress even if the cost rose: a viewpoint gained.
			const bool gained = committed.num_reachable > cur.num_reachable;

			base = b_new;
			cur = std::move(committed);
			remember(cur);
			base_history.push_back(base);
			record(base, cur);

			RCLCPP_INFO(
				node->get_logger(),
				"restart %d/%d iter %d/%d: abs (%.4f, %.4f, %.4f) m  reachable %d/%d  |grad|=%.4f  step=%.4f m",
				restart_idx + 1, std::max(1, params.descent_num_restart), outer + 1, params.max_outer_iterations, base.x,
				base.y, base.z, cur.num_reachable, n, gnorm, step);
			log_cost_terms(cur);
			log_missed(base, cur);

			PublishProgress(
				node, params, object_pose_obj, tour_poses_obj, base_history,
				Eigen::Vector3d(-g), base, state, jmg, cur);

			if (!gained && rel_impr < params.convergence_tolerance_cost)
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

			if (!gained && base_move < params.convergence_tolerance_offset && rel_impr < params.convergence_tolerance_cost)
			{
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

		// Elbow-swing smoothing at the result (all reached only): same order, smoothed poses, object nudged with them.
		// Kept only if the planned travel (else the L2 travel) drops, like refine.
		if (params.elbow_swing && rr.ok && !rr.sol.tour.empty())
		{
			InnerSolution before = rr.sol;
			if (use_real_cost && before.real_cost < 0.0)
				before.real_cost = real_cost_of(rr.placement, before);
			const ElbowSwingResult es = ElbowSwingSmooth(
				state, jmg, tool0_link, planning_scene_monitor, object_pose_obj, tour_poses_obj, before.tour,
				before.joints, start_reference_joints, rr.placement, params);
			// Rescore the smoothed tour; every viewpoint is still reached, so there are no missed-pose terms.
			InnerSolution s = before;
			s.joints = es.joints;
			s.travel_cost = TourWeightedCost(
				es.placement, s.tour, s.joints, tour_poses_obj, start_reference_joints, home_tcp_local, params);
			s.sum_manipulability =
				SumManipulability(state, jmg, tool0_link, s.joints, params.manipulability_limit_sharpness);
			if (params_in.softmin_manipulability_weight > 0.0)
			{
				std::vector<double> ws;
				for (const auto& q : s.joints)
					ws.push_back(Manipulability(state, jmg, tool0_link, q, params.manipulability_limit_sharpness));
				s.softmin_manipulability = SoftMin(ws, params.softmin_tau);
			}
			s.real_cost = use_real_cost ? real_cost_of(es.placement, s) : -1.0;
			s.weighted_cost = (params.travel_in_cost ? s.travel_cost : 0.0) -
				params.manipulability_weight * s.sum_manipulability -
				params.softmin_manipulability_weight * s.softmin_manipulability;
			if (planned_travel)
				s.weighted_cost += s.real_cost - (params.travel_in_cost ? s.travel_cost : 0.0);
			const bool keep = use_real_cost ? s.real_cost < before.real_cost : s.travel_cost < before.travel_cost;
			RCLCPP_INFO(
				node->get_logger(),
				"  restart %d: elbow swing %d iters: joint path %.2f -> %.2f rad, big moves %d -> %d, object moved %.4f m to abs (%.4f, %.4f, "
				"%.4f) m, real %.2f -> %.2f%s  (rejected tries: ik %d, limits %d, collision %d, longer %d)",
				restart_idx + 1, es.iters, es.travel_before, es.travel_after, es.big_moves_before, es.big_moves_after, ToOffsetVec(es.placement, rr.placement).norm(),
				es.placement.x, es.placement.y, es.placement.z, before.real_cost, s.real_cost, keep ? "  <-- kept" : "", es.fail_ik,
				es.fail_limits, es.fail_collision, es.fail_longer);
			if (keep)
			{
				rr.placement = es.placement;
				rr.sol = std::move(s);
			}
			else
				rr.sol = std::move(before);
			rr.cost = cost_at_final_weight(rr.sol);
		}

		return rr;  // record() runs at least once, so rr holds this restart's best-cost solution
	};

	const int descent_num_restart = std::max(1, params.descent_num_restart);
	RestartResult overall;

	for (int r = 0; r < descent_num_restart && rclcpp::ok(); ++r)
	{
		ObjectPlacement start =
			(r == 0) ? ObjectPlacement{
						   std::isnan(params.initial_x) ? object_translation_nominal.x() : params.initial_x,
						   std::isnan(params.initial_y) ? object_translation_nominal.y() : params.initial_y,
						   std::isnan(params.initial_z) ? object_translation_nominal.z() : params.initial_z}
					 : overall.placement;  // restarts change only the IK seed
		RestartResult rr = run_descent(start, r, r == 0 ? nullptr : &overall.sol);

		// More viewpoints reached wins, then lower cost.
		const bool improved = (r == 0) || rr.sol.num_reachable > overall.sol.num_reachable ||
			(rr.sol.num_reachable == overall.sol.num_reachable && rr.cost < overall.cost);

		RCLCPP_INFO(
			node->get_logger(),
			"restart %d/%d done: total cost=%.4f  travel+miss=%.2f  sum_w=%.3f  reachable %d/%d  abs (%.4f, %.4f, %.4f) m%s",
			r + 1, descent_num_restart, rr.cost,
			rr.sol.travel_cost + params.unreachable_penalty * (n - rr.sol.num_reachable), rr.sol.sum_manipulability,
			rr.sol.num_reachable, n,
			rr.placement.x, rr.placement.y, rr.placement.z, (r > 0 && improved) ? "  <-- new best" : "");

		if (improved)
			overall = rr;
		if (overall.ok)
			break;  // every viewpoint reached: no more restarts needed
	}

	const InnerSolution& fin = overall.sol;
	result.x = overall.placement.x;
	result.y = overall.placement.y;
	result.z = overall.placement.z;
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
			"Done. Object abs (%.4f, %.4f, %.4f) m -- reaches "
			"all %d poses, tour joint path %.4f rad, travel %.2f, sum_w %.3f (weighted cost %.4f).",
			result.x, result.y, result.z, n,
			result.total_joint_path_length, fin.travel_cost, fin.sum_manipulability, result.total_weighted_cost);
	else
		RCLCPP_WARN(
			node->get_logger(),
			"Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses.", result.x, result.y, result.z,
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
		entry["weighted_cost"] = h[4];
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

Eigen::Isometry3d PlacementTransform(const Eigen::Vector3d& object_translation_nominal, double x, double y, double z)
{
	return MakePlacement(ObjectPlacement{x, y, z}) * Eigen::Translation3d(-object_translation_nominal);
}

void ApplyObjectPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Matrix3d& object_rotation_nominal, double x, double y, double z)
{
	SetObjectPose(
		planning_scene_monitor,
		MakePlacement(ObjectPlacement{x, y, z}) * MakeIsometry(Eigen::Vector3d::Zero(), object_rotation_nominal));
}

visualization_msgs::msg::MarkerArray BuildBaseGradientMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_nominal, const BaseGradientResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	const Eigen::Isometry3d xform = PlacementTransform(object_translation_nominal, result.x, result.y, result.z);
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
