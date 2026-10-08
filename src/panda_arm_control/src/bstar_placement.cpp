#include "panda_arm_control/bstar_placement.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <json/json.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <osqp/osqp.h>
#include <random_numbers/random_numbers.h>

namespace
{

// ---------------------------------------------------------------------------------------------
// Basic helpers
// ---------------------------------------------------------------------------------------------

Eigen::Isometry3d MakeTransform(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
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

double RandomUniform(std::mt19937& rng, double lo, double hi)
{
	std::uniform_real_distribution<double> dist(lo, hi);
	return dist(rng);
}

struct XYOffset
{
	double x = 0.0, y = 0.0;
};

XYOffset RandomInBounds(const BstarPlacementBounds& b, std::mt19937& rng)
{
	return {RandomUniform(rng, b.x_min, b.x_max), RandomUniform(rng, b.y_min, b.y_max)};
}

XYOffset ClampToBounds(XYOffset o, const BstarPlacementBounds& b)
{
	o.x = std::clamp(o.x, b.x_min, b.x_max);
	o.y = std::clamp(o.y, b.y_min, b.y_max);
	return o;
}

double JointL1Distance(const std::vector<double>& a, const std::vector<double>& b)
{
	double d = 0.0;
	for (size_t k = 0; k < a.size() && k < b.size(); ++k)
		d += std::abs(a[k] - b[k]);
	return d;
}

// Small-angle-safe rotation-vector error: the axis-angle of R_target * R_current^T.
Eigen::Vector3d OrientationError(const Eigen::Matrix3d& target, const Eigen::Matrix3d& current)
{
	Eigen::AngleAxisd aa(target * current.transpose());
	return aa.angle() * aa.axis();
}

// ---------------------------------------------------------------------------------------------
// Step-1 relaxation: for one viewpoint, find a collision-free (q, x, y, theta) that reaches it,
// solving the arm joints and the object offset jointly (defined below LinearizeFk, which it uses).
// ---------------------------------------------------------------------------------------------

void PublishRelaxationProgress(
	const rclcpp::Node::SharedPtr& node, const BstarPlacementParams& params, const std::vector<XYOffset>& offsets,
	const std::vector<bool>& placed, const XYOffset& mean)
{
	if (!params.progress_pub)
		return;

	const rclcpp::Time stamp = node->now();
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;
	for (size_t i = 0; i < offsets.size(); ++i)
	{
		visualization_msgs::msg::Marker dot;
		dot.header.frame_id = "world";
		dot.header.stamp = stamp;
		dot.ns = "bstar_placement_relaxation";
		dot.id = id++;
		dot.type = visualization_msgs::msg::Marker::SPHERE;
		dot.action = visualization_msgs::msg::Marker::ADD;
		dot.pose.position.x = offsets[i].x;
		dot.pose.position.y = offsets[i].y;
		dot.pose.orientation.w = 1.0;
		dot.scale.x = dot.scale.y = dot.scale.z = 0.015;
		const bool p = i < placed.size() && placed[i];
		dot.color.r = p ? 0.1f : 0.8f;
		dot.color.g = p ? 0.6f : 0.2f;
		dot.color.b = 0.9f;
		dot.color.a = 0.9f;
		markers.markers.push_back(dot);
	}
	visualization_msgs::msg::Marker mean_marker;
	mean_marker.header.frame_id = "world";
	mean_marker.header.stamp = stamp;
	mean_marker.ns = "bstar_placement_relaxation_mean";
	mean_marker.id = id++;
	mean_marker.type = visualization_msgs::msg::Marker::SPHERE;
	mean_marker.action = visualization_msgs::msg::Marker::ADD;
	mean_marker.pose.position.x = mean.x;
	mean_marker.pose.position.y = mean.y;
	mean_marker.pose.orientation.w = 1.0;
	mean_marker.scale.x = mean_marker.scale.y = mean_marker.scale.z = 0.05;
	mean_marker.color.r = 1.0f;
	mean_marker.color.g = 0.85f;
	mean_marker.color.a = 1.0f;
	markers.markers.push_back(mean_marker);

	params.progress_pub->publish(markers);
	if (params.visualize_progress_delay_sec > 0.0)
		std::this_thread::sleep_for(std::chrono::duration<double>(params.visualize_progress_delay_sec));
}

// ---------------------------------------------------------------------------------------------
// Inner layer: per-point FK linearization (Eq. 5, 11) and collision linearization (Eq. 8, 11).
// ---------------------------------------------------------------------------------------------

// One viewpoint's linearized FK equality: J*dq - S*db = residual (6 rows: [pos; rot_vec]).
struct FkLinearization
{
	Eigen::MatrixXd joint_jacobian;				// 6 x dof, [linear; angular], base_link frame
	Eigen::Matrix<double, 6, 2> xy_offset_jacobian;	// 6 x 2, columns x, y -- pure translation, no rotation search
	Eigen::Matrix<double, 6, 1> residual;
};

// Exact (non-linearized) FK residual at (q0, xy_offset0) -- no Jacobian, so it's cheap for callers
// that only need to know the current error, not which way to step next.
Eigen::Matrix<double, 6, 1> ComputeResidual(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const Eigen::Isometry3d& target_pose,
	const std::vector<double>& q0, const XYOffset& xy_offset0)
{
	const Eigen::Isometry3d target_moved = Eigen::Translation3d(xy_offset0.x, xy_offset0.y, 0.0) * target_pose;

	state.setJointGroupPositions(jmg, q0);
	state.update();
	const Eigen::Isometry3d T_cur = state.getGlobalLinkTransform(tool0_link);

	Eigen::Matrix<double, 6, 1> residual;
	residual.head<3>() = target_moved.translation() - T_cur.translation();
	residual.tail<3>() = OrientationError(target_moved.rotation(), T_cur.rotation());
	return residual;
}

FkLinearization LinearizeFk(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const Eigen::Isometry3d& target_pose,
	const std::vector<double>& q0, const XYOffset& xy_offset0)
{
	FkLinearization out;

	// Pure translation: nudging x or y shifts the target's position directly and leaves its
	// orientation untouched, so S is just the two unit columns with everything else zero.
	out.xy_offset_jacobian.setZero();
	out.xy_offset_jacobian.block<3, 1>(0, 0) = Eigen::Vector3d::UnitX();
	out.xy_offset_jacobian.block<3, 1>(0, 1) = Eigen::Vector3d::UnitY();

	state.setJointGroupPositions(jmg, q0);
	state.update();
	state.getJacobian(jmg, tool0_link, Eigen::Vector3d::Zero(), out.joint_jacobian);

	out.residual = ComputeResidual(state, jmg, tool0_link, target_pose, q0, xy_offset0);
	return out;
}

// Numerically perturbs q0/xy_offset0 and compares against LinearizeFk's analytic J/S; logs a warning
// on disagreement since the whole SLP is only as correct as this derivative.
void CheckFkJacobian(
	const rclcpp::Node::SharedPtr& node, moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
	const moveit::core::LinkModel* tool0_link, const Eigen::Isometry3d& target_pose_original,
	const std::vector<double>& q0, const XYOffset& xy_offset0, double eps)
{
	const FkLinearization at0 = LinearizeFk(state, jmg, tool0_link, target_pose_original, q0, xy_offset0);
	const int dof = static_cast<int>(q0.size());
	Eigen::MatrixXd joint_jacobian_fd(6, dof);
	for (int k = 0; k < dof; ++k)
	{
		std::vector<double> q_plus = q0;
		q_plus[k] += eps;
		const FkLinearization at_plus = LinearizeFk(state, jmg, tool0_link, target_pose_original, q_plus, xy_offset0);
		joint_jacobian_fd.col(k) = (at0.residual - at_plus.residual) / eps;  // d(residual)/dq = -J
	}
	const double j_err = (joint_jacobian_fd - at0.joint_jacobian).norm() / std::max(1e-9, at0.joint_jacobian.norm());

	Eigen::Matrix<double, 6, 2> xy_offset_jacobian_fd;
	for (int c = 0; c < 2; ++c)
	{
		XYOffset b_plus = xy_offset0;
		(c == 0 ? b_plus.x : b_plus.y) += eps;
		const FkLinearization at_plus = LinearizeFk(state, jmg, tool0_link, target_pose_original, q0, b_plus);
		xy_offset_jacobian_fd.col(c) = (at_plus.residual - at0.residual) / eps;  // d(residual)/dbase = +S
	}
	const double s_err = (xy_offset_jacobian_fd - at0.xy_offset_jacobian).norm() / std::max(1e-9, at0.xy_offset_jacobian.norm());

	RCLCPP_INFO(
		node->get_logger(), "[fd check] FK Jacobian relative error: dJ/dq=%.2e  dJ/dbase=%.2e (want << 1)", j_err,
		s_err);
}

// Finds one warm-start (initial) IK solution -- joint angles + xy offset -- for a single
// viewpoint, before the real two-layer optimization runs. Solves joints and offset together via
// Levenberg-Marquardt (damped least squares with adaptive damping), re-linearizing each
// iteration, until the FK residual converges or joint_ik_max_iterations/ik_timeout is hit. The
// caller loops this over every viewpoint.
bool FindInitialIkForViewpoint(
	const moveit::core::RobotModelConstPtr& robot_model, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const moveit::core::LinkModel* tool0_link,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Isometry3d& object_pose_original, const Eigen::Isometry3d& target_pose_original,
	const std::vector<double>& seed_joints, const XYOffset& seed_xy_offset, const BstarPlacementBounds& bounds,
	const BstarPlacementParams& params, std::vector<double>* out_joints, XYOffset* out_xy_offset,
	double* out_best_residual = nullptr, std::string* out_stop_reason = nullptr, int* out_iters_used = nullptr)
{
	const int dof = static_cast<int>(jmg->getVariableCount());
	std::vector<double> q = seed_joints;
	XYOffset xy_offset = ClampToBounds(seed_xy_offset, bounds);
	double damping = params.joint_ik_damping;

	auto clamped_step = [&](const Eigen::VectorXd& delta) {
		std::vector<double> q_try = q;
		for (int k = 0; k < dof; ++k)
		{
			const auto& bnd = robot_model->getVariableBounds(jmg->getVariableNames()[k]);
			q_try[k] = std::clamp(q[k] + delta(k), bnd.min_position_, bnd.max_position_);
		}
		XYOffset xy_offset_try;
		xy_offset_try.x = std::clamp(xy_offset.x + delta(dof), bounds.x_min, bounds.x_max);
		xy_offset_try.y = std::clamp(xy_offset.y + delta(dof + 1), bounds.y_min, bounds.y_max);
		return std::make_pair(q_try, xy_offset_try);
	};

	auto blended_residual = [&](const std::vector<double>& q_try, const XYOffset& xy_offset_try) {
		SetObjectPose(
			planning_scene_monitor, Eigen::Translation3d(xy_offset_try.x, xy_offset_try.y, 0.0) * object_pose_original);
		const auto residual = ComputeResidual(state, jmg, tool0_link, target_pose_original, q_try, xy_offset_try);
		return std::max(residual.head<3>().norm(), params.rot_metric_scale * residual.tail<3>().norm());
	};

	const auto t_start = std::chrono::steady_clock::now();
	int iter = 0;
	for (; iter < params.joint_ik_max_iterations; ++iter)
	{
		if (out_iters_used)
			*out_iters_used = iter;
		if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > params.ik_timeout)
		{
			if (out_stop_reason)
				*out_stop_reason = "timeout";
			break;
		}

		SetObjectPose(
			planning_scene_monitor, Eigen::Translation3d(xy_offset.x, xy_offset.y, 0.0) * object_pose_original);
		const FkLinearization fk = LinearizeFk(state, jmg, tool0_link, target_pose_original, q, xy_offset);

		const double blended =
			std::max(fk.residual.head<3>().norm(), params.rot_metric_scale * fk.residual.tail<3>().norm());
		if (out_best_residual && blended < *out_best_residual)
			*out_best_residual = blended;
		if (blended < params.fk_residual_tolerance)
		{
			if (!IsStateCollisionFree(planning_scene_monitor, &state, jmg, q.data()))
			{
				if (out_stop_reason)
					*out_stop_reason = "collision";
				return false;
			}
			*out_joints = q;
			*out_xy_offset = xy_offset;
			return true;
		}

		Eigen::MatrixXd G(6, dof + 2);
		G.leftCols(dof) = fk.joint_jacobian;
		G.rightCols(2) = -fk.xy_offset_jacobian;

		// Levenberg-Marquardt: shrink damping and take the step when it actually helps; otherwise
		// grow damping (a more conservative, trust-worthy step) and retry.
		bool improved = false;
		for (int lm = 0; lm < params.joint_ik_lm_max_escalations && !improved; ++lm)
		{
			Eigen::MatrixXd GtG = G.transpose() * G;
			GtG.diagonal().array() += damping;
			const Eigen::VectorXd delta = GtG.ldlt().solve(G.transpose() * fk.residual);

			const auto [q_try, xy_offset_try] = clamped_step(delta);
			const double blended_try = blended_residual(q_try, xy_offset_try);
			if (out_best_residual && blended_try < *out_best_residual)
				*out_best_residual = blended_try;

			if (blended_try < blended)
			{
				q = q_try;
				xy_offset = xy_offset_try;
				damping = std::max(damping / 3.0, 1e-8);
				improved = true;
			}
			else
			{
				damping = std::min(damping * 4.0, 1e3);
			}
		}
		if (!improved)
		{
			if (out_stop_reason)
				*out_stop_reason = "stuck";
			break;	// even the most conservative step made things worse -- stuck.
		}
	}
	if (out_stop_reason && out_stop_reason->empty())
		*out_stop_reason = "max_iterations";
	if (out_iters_used)
		*out_iters_used = iter;
	return false;
}

// One linearized collision row: row * dq_i >= rhs. Object re-placed at xy_offset0 for this evaluation,
// so only the robot-link side(s) of a pair contribute a Jacobian.
struct CollisionRow
{
	Eigen::RowVectorXd row;	 // 1 x dof
	double rhs = 0.0;
};

std::vector<CollisionRow> LinearizeCollisionConstraints(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const moveit::core::RobotModelConstPtr& robot_model, moveit::core::RobotState& state,
	const moveit::core::JointModelGroup* jmg, const std::string& group_name, double distance_threshold,
	double min_clearance)
{
	std::vector<CollisionRow> rows;

	collision_detection::DistanceRequest req;
	req.enable_nearest_points = true;
	req.enable_signed_distance = true;
	req.type = collision_detection::DistanceRequestType::ALL;
	req.group_name = group_name;
	req.enableGroup(robot_model);
	req.distance_threshold = distance_threshold;
	req.max_contacts_per_body = 4;

	collision_detection::DistanceResult res;
	{
		planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor);
		locked_scene->getCollisionEnv()->distanceRobot(req, res, state);
	}

	for (const auto& pair_entry : res.distances)
	{
		for (const auto& d : pair_entry.second)
		{
			if (d.distance >= distance_threshold)
				continue;

			Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(jmg->getVariableCount());
			bool any_robot_side = false;
			for (int side = 0; side < 2; ++side)
			{
				if (d.body_types[side] != collision_detection::BodyType::ROBOT_LINK &&
					d.body_types[side] != collision_detection::BodyType::ROBOT_ATTACHED)
					continue;
				const moveit::core::LinkModel* link = robot_model->getLinkModel(d.link_names[side]);
				if (!link)
					continue;
				const Eigen::Vector3d local_point =
					state.getGlobalLinkTransform(link).inverse() * d.nearest_points[side];
				Eigen::MatrixXd J_link;
				state.getJacobian(jmg, link, local_point, J_link);
				// distance ~ distance0 + normal.(dp1-dp0); side 0 moves -normal, side 1 moves +normal.
				const double sign = (side == 0) ? -1.0 : 1.0;
				row += sign * (d.normal.transpose() * J_link.topRows<3>());
				any_robot_side = true;
			}
			if (!any_robot_side)
				continue;

			CollisionRow cr;
			cr.row = row;
			cr.rhs = min_clearance - d.distance;
			rows.push_back(std::move(cr));
		}
	}
	return rows;
}

// ---------------------------------------------------------------------------------------------
// LP assembly (OSQP): variables per point i are dq_i (dof) and db_i (3); L1 terms via slacks.
// ---------------------------------------------------------------------------------------------

struct LpStep
{
	bool ok = false;
	std::vector<std::vector<double>> dq;	// n x dof
	std::vector<XYOffset> db;			// n
	double predicted_cost = 0.0;
	c_int diag_setup_exit_flag = -999;
	c_int diag_status_val = 0;
	std::string diag_status_str;
};

// A tiny local COO builder for the sparse constraint matrix A (l <= A*vars <= u, OSQP-style).
struct CooBuilder
{
	std::vector<Eigen::Triplet<double>> triplets;
	std::vector<double> lo, hi;
	int next_row = 0;

	int add_row(double l, double u)
	{
		lo.push_back(l);
		hi.push_back(u);
		return next_row++;
	}
	void put(int row, int col, double v) { triplets.emplace_back(row, col, v); }
};

LpStep SolveTrustRegionLp(
	const moveit::core::RobotModelConstPtr& robot_model, const moveit::core::JointModelGroup* jmg,
	const std::vector<std::vector<double>>& q0, const std::vector<XYOffset>& xy_offset0,
	const std::vector<FkLinearization>& fk, const std::vector<std::vector<CollisionRow>>& collision,
	const BstarPlacementBounds& bounds, double mu, double trust_region, double trust_region_reg,
	double fk_penalty_weight)
{
	const int n = static_cast<int>(q0.size());
	const int dof = static_cast<int>(jmg->getVariableCount());
	const int n_dq = n * dof;
	const int n_db = n * 2;
	const int n_zedge = std::max(0, n - 1) * dof;
	const int n_zpen = n * 2;
	const int n_zfk = n * 6;
	const int num_vars = n_dq + n_db + n_zedge + n_zpen + n_zfk;

	auto idx_dq = [&](int i, int k) { return i * dof + k; };
	auto idx_db = [&](int i, int c) { return n_dq + i * 2 + c; };
	auto idx_zedge = [&](int i, int k) { return n_dq + n_db + i * dof + k; };
	auto idx_zpen = [&](int i, int c) { return n_dq + n_db + n_zedge + i * 2 + c; };
	auto idx_zfk = [&](int i, int r) { return n_dq + n_db + n_zedge + n_zpen + i * 6 + r; };

	std::vector<double> q_lin(num_vars, 0.0);
	for (int i = 0; i < n; ++i)
		for (int c = 0; c < 2; ++c)
			q_lin[idx_zpen(i, c)] = mu;
	for (int e = 0; e < n - 1; ++e)
		for (int k = 0; k < dof; ++k)
			q_lin[idx_zedge(e, k)] = 1.0;
	for (int i = 0; i < n; ++i)
		for (int r = 0; r < 6; ++r)
			q_lin[idx_zfk(i, r)] = fk_penalty_weight;

	CooBuilder A;
	const double inf = std::numeric_limits<double>::infinity();

	// FK error (paper Eq. 5/11, per point i) as a minimized L1 cost instead of a hard equality:
	// z_fk_i_r >= |J_i*dq_i - S_i*db_i - residual_i(r)|. The outer loop's mu-pulling (Eq. 9) can
	// legitimately drag a point's residual around; a penalized error can never make the combined
	// LP infeasible the way a hard equality band could.
	for (int i = 0; i < n; ++i)
	{
		for (int r = 0; r < 6; ++r)
		{
			int row = A.add_row(-fk[i].residual(r), inf);
			A.put(row, idx_zfk(i, r), 1.0);
			for (int k = 0; k < dof; ++k)
				if (fk[i].joint_jacobian(r, k) != 0.0)
					A.put(row, idx_dq(i, k), -fk[i].joint_jacobian(r, k));
			for (int c = 0; c < 2; ++c)
				if (fk[i].xy_offset_jacobian(r, c) != 0.0)
					A.put(row, idx_db(i, c), fk[i].xy_offset_jacobian(r, c));

			row = A.add_row(fk[i].residual(r), inf);
			A.put(row, idx_zfk(i, r), 1.0);
			for (int k = 0; k < dof; ++k)
				if (fk[i].joint_jacobian(r, k) != 0.0)
					A.put(row, idx_dq(i, k), fk[i].joint_jacobian(r, k));
			for (int c = 0; c < 2; ++c)
				if (fk[i].xy_offset_jacobian(r, c) != 0.0)
					A.put(row, idx_db(i, c), -fk[i].xy_offset_jacobian(r, c));
		}
	}

	// Collision: row.dq_i >= rhs.
	for (int i = 0; i < n; ++i)
		for (const auto& cr : collision[i])
		{
			const int row = A.add_row(cr.rhs, inf);
			for (int k = 0; k < dof; ++k)
				if (cr.row(k) != 0.0)
					A.put(row, idx_dq(i, k), cr.row(k));
		}

	// Base bounds intersected with the trust region.
	for (int i = 0; i < n; ++i)
	{
		const double xlo = std::max(bounds.x_min - xy_offset0[i].x, -trust_region);
		const double xhi = std::min(bounds.x_max - xy_offset0[i].x, trust_region);
		const double ylo = std::max(bounds.y_min - xy_offset0[i].y, -trust_region);
		const double yhi = std::min(bounds.y_max - xy_offset0[i].y, trust_region);
		int row = A.add_row(xlo, xhi);
		A.put(row, idx_db(i, 0), 1.0);
		row = A.add_row(ylo, yhi);
		A.put(row, idx_db(i, 1), 1.0);
	}

	// Joint limits intersected with the trust region.
	for (int i = 0; i < n; ++i)
		for (int k = 0; k < dof; ++k)
		{
			const auto& bnd = robot_model->getVariableBounds(jmg->getVariableNames()[k]);
			const double lo = std::max(bnd.min_position_ - q0[i][k], -trust_region);
			const double hi = std::min(bnd.max_position_ - q0[i][k], trust_region);
			const int row = A.add_row(lo, hi);
			A.put(row, idx_dq(i, k), 1.0);
		}

	// L1 epigraph for consecutive joint-path edges: z_edge_i >= |diff0 + dq_{i+1} - dq_i|.
	for (int e = 0; e < n - 1; ++e)
		for (int k = 0; k < dof; ++k)
		{
			const double diff0 = q0[e + 1][k] - q0[e][k];
			int row = A.add_row(diff0, inf);
			A.put(row, idx_zedge(e, k), 1.0);
			A.put(row, idx_dq(e + 1, k), -1.0);
			A.put(row, idx_dq(e, k), 1.0);
			row = A.add_row(-diff0, inf);
			A.put(row, idx_zedge(e, k), 1.0);
			A.put(row, idx_dq(e + 1, k), 1.0);
			A.put(row, idx_dq(e, k), -1.0);
		}

	// L1 epigraph for the outer-layer penalty: z_pen_i >= |(object_offset0_i - mean0) + db_i - mean(db)|.
	XYOffset mean0{0.0, 0.0};
	for (int i = 0; i < n; ++i)
	{
		mean0.x += xy_offset0[i].x / n;
		mean0.y += xy_offset0[i].y / n;
	}
	for (int i = 0; i < n; ++i)
	{
		const std::array<double, 2> off0 = {xy_offset0[i].x - mean0.x, xy_offset0[i].y - mean0.y};
		for (int c = 0; c < 2; ++c)
		{
			// Variables move to the left side negated, as in the edge rows above.
			int row = A.add_row(off0[c], inf);
			A.put(row, idx_zpen(i, c), 1.0);
			A.put(row, idx_db(i, c), -1.0);
			for (int j = 0; j < n; ++j)
				A.put(row, idx_db(j, c), 1.0 / n);
			row = A.add_row(-off0[c], inf);
			A.put(row, idx_zpen(i, c), 1.0);
			A.put(row, idx_db(i, c), 1.0);
			for (int j = 0; j < n; ++j)
				A.put(row, idx_db(j, c), -1.0 / n);
		}
	}

	// ros-humble-osqp-vendor 0.2.0 ships OSQP's 0.6.x API: plain csc structs, c_float/c_int, and
	// osqp_setup(&work, &data, &settings) writing into an OSQPWorkspace.
	Eigen::SparseMatrix<double> A_mat(A.next_row, num_vars);
	A_mat.setFromTriplets(A.triplets.begin(), A.triplets.end());
	A_mat.makeCompressed();

	std::vector<c_float> Ax(A_mat.valuePtr(), A_mat.valuePtr() + A_mat.nonZeros());
	std::vector<c_int> Ai(A_mat.innerIndexPtr(), A_mat.innerIndexPtr() + A_mat.nonZeros());
	std::vector<c_int> Ap(A_mat.outerIndexPtr(), A_mat.outerIndexPtr() + num_vars + 1);
	// P's diagonal regularizes dq/db only, giving the LP a unique minimum; slacks stay zero-cost
	// in P (they're already penalized linearly via q_lin).
	const int n_reg = n_dq + n_db;
	std::vector<c_float> Px(n_reg, trust_region_reg);
	std::vector<c_int> Pi(n_reg);
	for (int j = 0; j < n_reg; ++j)
		Pi[j] = j;
	std::vector<c_int> Pp(num_vars + 1);
	for (int j = 0; j <= num_vars; ++j)
		Pp[j] = std::min(j, n_reg);

	csc A_csc{static_cast<c_int>(Ax.size()), A.next_row, num_vars, Ap.data(), Ai.data(), Ax.data(), -1};
	csc P_csc{n_reg, num_vars, num_vars, Pp.data(), Pi.data(), Px.data(), -1};

	OSQPData data;
	data.n = num_vars;
	data.m = A.next_row;
	data.P = &P_csc;
	data.A = &A_csc;
	data.q = q_lin.data();
	data.l = A.lo.data();
	data.u = A.hi.data();

	OSQPSettings settings;
	osqp_set_default_settings(&settings);
	settings.verbose = 0;
	settings.polish = 1;
	settings.max_iter = 20000;	// diagnostic: was hitting the 4000 default every call at reg=1e-6
	// Accuracy relative to step size: fixed 1e-3 overshoots small trust regions; fixed 1e-6 hits max_iter.
	settings.eps_abs = 0.01 * trust_region;
	settings.eps_rel = 0.01 * trust_region;

	OSQPWorkspace* work = nullptr;
	const c_int exit_flag = osqp_setup(&work, &data, &settings);

	LpStep step;
	step.diag_setup_exit_flag = exit_flag;
	if (exit_flag == 0)
	{
		osqp_solve(work);
		step.diag_status_val = work->info->status_val;
		step.diag_status_str = work->info->status;
		if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE)
		{
			step.ok = true;
			step.predicted_cost = work->info->obj_val;
			step.dq.assign(n, std::vector<double>(dof, 0.0));
			step.db.assign(n, XYOffset{});
			for (int i = 0; i < n; ++i)
			{
				for (int k = 0; k < dof; ++k)
					step.dq[i][k] = work->solution->x[idx_dq(i, k)];
				step.db[i].x = work->solution->x[idx_db(i, 0)];
				step.db[i].y = work->solution->x[idx_db(i, 1)];
			}
		}
	}

	if (work)
		osqp_cleanup(work);

	return step;
}

// ---------------------------------------------------------------------------------------------
// Inner layer: trust-region SLP. Re-linearizes every iteration; accepts a step only if it makes
// real progress on the true (non-linearized) cost, else shrinks the trust region and retries.
// ---------------------------------------------------------------------------------------------

// Diagnostic breakdown of TrueCost's three components, so a predicted-vs-actual mismatch can be
// traced to which term is responsible instead of guessing from the aggregate number.
struct CostBreakdown
{
	double path_length = 0.0;
	double spread = 0.0;
	double fk = 0.0;
};

double TrueCost(
	const std::vector<std::vector<double>>& q, const std::vector<XYOffset>& xy_offset, double mu,
	const std::vector<Eigen::Matrix<double, 6, 1>>& residuals, double fk_penalty_weight,
	CostBreakdown* out_breakdown = nullptr)
{
	double path_length = 0.0;
	for (size_t i = 0; i + 1 < q.size(); ++i)
		path_length += JointL1Distance(q[i], q[i + 1]);

	XYOffset mean{0.0, 0.0};
	for (const auto& b : xy_offset)
	{
		mean.x += b.x / xy_offset.size();
		mean.y += b.y / xy_offset.size();
	}
	double spread = 0.0;
	for (const auto& b : xy_offset)
		spread += mu * (std::abs(b.x - mean.x) + std::abs(b.y - mean.y));

	double fk_cost = 0.0;
	for (const auto& r : residuals)
		fk_cost += fk_penalty_weight * r.lpNorm<1>();

	if (out_breakdown)
	{
		out_breakdown->path_length = path_length;
		out_breakdown->spread = spread;
		out_breakdown->fk = fk_cost;
	}
	return path_length + spread + fk_cost;
}

// Farthest base offset from the mean, in meters.
double MaxSpread(const std::vector<XYOffset>& xy_offset)
{
	XYOffset m{0.0, 0.0};
	for (const auto& b : xy_offset)
	{
		m.x += b.x / xy_offset.size();
		m.y += b.y / xy_offset.size();
	}
	double spread = 0.0;
	for (const auto& b : xy_offset)
		spread = std::max(spread, std::hypot(b.x - m.x, b.y - m.y));
	return spread;
}

double MaxResidualNorm(const std::vector<Eigen::Matrix<double, 6, 1>>& residuals, double rot_metric_scale)
{
	double worst = 0.0;
	for (const auto& residual : residuals)
	{
		const double pos_norm = residual.head<3>().norm();
		const double rot_norm = residual.tail<3>().norm();
		worst = std::max(worst, std::max(pos_norm, rot_metric_scale * rot_norm));
	}
	return worst;
}

// Fixed-width progress tag so log columns stay aligned; pass -1 for a level that isn't running.
std::string ProgressTag(int restart, int num_restarts, int outer, int max_outer, int inner, int max_inner)
{
	auto count = [](int k) { return k < 0 ? std::string("--") : std::to_string(k); };
	char buf[96];
	std::snprintf(
		buf, sizeof(buf), "[restart %2s/%-2d | outer %2s/%-2d | inner %2s/%-2d]", count(restart).c_str(), num_restarts,
		count(outer).c_str(), max_outer, count(inner).c_str(), max_inner);
	return buf;
}

struct InnerResult
{
	std::vector<std::vector<double>> q;
	std::vector<XYOffset> xy_offset;
	double fk_residual_max = 0.0;
};

InnerResult RunInnerSlp(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const moveit::core::JointModelGroup* jmg, const moveit::core::LinkModel* tool0_link,
	const Eigen::Isometry3d& object_pose_original, const std::vector<Eigen::Isometry3d>& targets,
	std::vector<std::vector<double>> q, std::vector<XYOffset> xy_offset, double mu, const BstarPlacementParams& params,
	int restart_number, int outer_number)
{
	moveit::core::RobotState state(robot_model);
	const int n = static_cast<int>(targets.size());
	double trust_region = params.trust_region_initial;

	for (int iter = 0; iter < params.max_inner_iterations && rclcpp::ok(); ++iter)
	{
		const std::string tag = ProgressTag(
			restart_number, std::max(1, params.num_restarts), outer_number, params.max_outer_iterations, iter + 1,
			params.max_inner_iterations);
		std::vector<FkLinearization> fk(n);
		std::vector<std::vector<CollisionRow>> collision(n);
		for (int i = 0; i < n; ++i)
		{
			SetObjectPose(
				planning_scene_monitor,
				Eigen::Translation3d(xy_offset[i].x, xy_offset[i].y, 0.0) * object_pose_original);
			fk[i] = LinearizeFk(state, jmg, tool0_link, targets[i], q[i], xy_offset[i]);
			collision[i] = LinearizeCollisionConstraints(
				planning_scene_monitor, robot_model, state, jmg, group_name, params.collision_distance_threshold,
				params.min_clearance);
		}

		std::vector<Eigen::Matrix<double, 6, 1>> residuals(n);
		for (int i = 0; i < n; ++i)
			residuals[i] = fk[i].residual;
		CostBreakdown breakdown0;
		const double cost0 = TrueCost(q, xy_offset, mu, residuals, params.fk_penalty_weight, &breakdown0);
		LpStep step = SolveTrustRegionLp(
			robot_model, jmg, q, xy_offset, fk, collision, params.bounds, mu, trust_region, params.trust_region_reg,
			params.fk_penalty_weight);
		if (!step.ok)
		{
			trust_region *= params.trust_region_shrink;
			if (trust_region < params.trust_region_min)
				break;
			continue;
		}

		std::vector<std::vector<double>> q_new = q;
		std::vector<XYOffset> xy_offset_new = xy_offset;
		// Clamp defensively: OSQP's own solver tolerance can let a step overshoot its box by a
		// hair, and that drift compounds over outer iterations until a later, shrunk trust region
		// box becomes invalid (lo > hi), which OSQP's setup then rejects for the whole batch.
		for (int i = 0; i < n; ++i)
		{
			for (size_t k = 0; k < q[i].size(); ++k)
			{
				const auto& bnd = robot_model->getVariableBounds(jmg->getVariableNames()[k]);
				q_new[i][k] = std::clamp(q[i][k] + step.dq[i][k], bnd.min_position_, bnd.max_position_);
			}
			xy_offset_new[i] = ClampToBounds(
				XYOffset{xy_offset[i].x + step.db[i].x, xy_offset[i].y + step.db[i].y}, params.bounds);
		}
		std::vector<Eigen::Matrix<double, 6, 1>> residuals_new(n);
		for (int i = 0; i < n; ++i)
		{
			SetObjectPose(
				planning_scene_monitor,
				Eigen::Translation3d(xy_offset_new[i].x, xy_offset_new[i].y, 0.0) * object_pose_original);
			residuals_new[i] = ComputeResidual(state, jmg, tool0_link, targets[i], q_new[i], xy_offset_new[i]);
		}
		CostBreakdown breakdown_new;
		const double cost_new = TrueCost(q_new, xy_offset_new, mu, residuals_new, params.fk_penalty_weight, &breakdown_new);
		const double predicted_gain = cost0 - step.predicted_cost;
		const double actual_gain = cost0 - cost_new;
		const double ratio = predicted_gain > 1e-12 ? actual_gain / predicted_gain : 0.0;

		if (ratio > params.trust_region_accept_ratio)
		{
			const double spread_m_before = MaxSpread(xy_offset);
			q = q_new;
			xy_offset = xy_offset_new;
			if (ratio > params.trust_region_good_ratio)
				trust_region *= params.trust_region_expand;
			RCLCPP_INFO(
				node->get_logger(),
				"%s %-12s ratio %+8.3f  cost %10.4f -> %10.4f  trust_region %.2e  "
				"path %9.3f -> %9.3f  spread(m) %7.4f -> %7.4f  fk %8.3f -> %8.3f",
				tag.c_str(), "accepted", ratio, cost0, cost_new, trust_region, breakdown0.path_length,
				breakdown_new.path_length,
				spread_m_before, MaxSpread(xy_offset), breakdown0.fk, breakdown_new.fk);
			// Early stop: further accepted steps barely change the cost.
			if (actual_gain < params.inner_stop_rel_improvement * std::abs(cost0))
			{
				RCLCPP_INFO(
					node->get_logger(), "%s %-12s relative gain %.2e < %.2e", tag.c_str(), "early stop",
					actual_gain / std::max(1e-12, std::abs(cost0)), params.inner_stop_rel_improvement);
				break;
			}
		}
		else
		{
			trust_region *= params.trust_region_shrink;
		}
		if (trust_region < params.trust_region_min)
			break;
	}

	InnerResult out;
	out.q = q;
	out.xy_offset = xy_offset;
	std::vector<Eigen::Matrix<double, 6, 1>> final_residuals(n);
	for (int i = 0; i < n; ++i)
	{
		SetObjectPose(
			planning_scene_monitor, Eigen::Translation3d(xy_offset[i].x, xy_offset[i].y, 0.0) * object_pose_original);
		final_residuals[i] = ComputeResidual(state, jmg, tool0_link, targets[i], q[i], xy_offset[i]);
	}
	out.fk_residual_max = MaxResidualNorm(final_residuals, params.rot_metric_scale);
	return out;
}

// ---------------------------------------------------------------------------------------------
// Outer layer: per-point relaxation for the warm start, then progressive constraint tightening.
// ---------------------------------------------------------------------------------------------

struct RestartResult
{
	bool ok = false;
	XYOffset xy_offset;
	std::vector<std::vector<double>> joints;
	double total_joint_path_length = 0.0;
	double fk_residual_max = 0.0;
	int num_reachable = 0;
};

RestartResult RunOuterRelaxation(
	const rclcpp::Node::SharedPtr& node, int restart_number, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& targets, const std::vector<double>& start_reference_joints,
	const BstarPlacementParams& params, std::mt19937& rng)
{
	const int n = static_cast<int>(targets.size());
	const Eigen::Isometry3d object_pose_original =
		MakeTransform(object_translation_original, object_rotation_original);

	moveit::core::RobotState state(robot_model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
	const moveit::core::LinkModel* tool0_link = robot_model->getLinkModel("tool0");

	// Point 1 seeds randomly; later points seed from the preceding point's solution (paper Sec. IV-A).
	std::vector<XYOffset> per_point(n);
	std::vector<bool> placed(n, false);
	std::vector<std::vector<double>> joints(n, start_reference_joints);

	// Seeded from `rng` so a run is fully reproducible from random_seed -- RobotState's own default
	// RNG isn't tied to any of our seeds.
	random_numbers::RandomNumberGenerator joint_rng(rng());
	std::vector<double> seed_q(start_reference_joints.size());
	state.setToRandomPositions(jmg, joint_rng);
	state.copyJointGroupPositions(jmg, seed_q);
	XYOffset seed_xy_offset = RandomInBounds(params.bounds, rng);

	for (int i = 0; i < n; ++i)
	{
		std::vector<double> sol_q;
		XYOffset sol_xy_offset;
		double best_residual = std::numeric_limits<double>::infinity();
		std::string best_stop_reason;
		int best_iters_used = 0;

		// Each attempt's own residual/reason, kept separate so "best" tracks the attempt that
		// actually got closest -- not just whichever ran last.
		auto try_attempt = [&]() {
			double local_residual = std::numeric_limits<double>::infinity();
			std::string local_reason;
			int local_iters = 0;
			const bool ok = FindInitialIkForViewpoint(
				robot_model, state, jmg, tool0_link, planning_scene_monitor, object_pose_original, targets[i],
				seed_q, seed_xy_offset, params.bounds, params, &sol_q, &sol_xy_offset, &local_residual, &local_reason,
				&local_iters);
			if (local_residual < best_residual)
			{
				best_residual = local_residual;
				best_stop_reason = local_reason;
				best_iters_used = local_iters;
			}
			return ok;
		};

		bool ok = try_attempt();
		for (int attempt = 0; !ok && attempt < params.num_init_retries; ++attempt)
		{
			state.setToRandomPositions(jmg, joint_rng);
			state.copyJointGroupPositions(jmg, seed_q);
			seed_xy_offset = RandomInBounds(params.bounds, rng);
			ok = try_attempt();
		}
		if (ok)
		{
			per_point[i] = sol_xy_offset;
			placed[i] = true;
			joints[i] = sol_q;
			seed_q = sol_q;
			seed_xy_offset = sol_xy_offset;
		}
		else
		{
			RCLCPP_WARN(
				node->get_logger(),
				"%s %-12s point %2d failed after %d retries  best residual %.4f  (tolerance %.4f)  "
				"stop_reason '%s'  iters_used %d",
				ProgressTag(
					restart_number, std::max(1, params.num_restarts), -1, params.max_outer_iterations, -1,
					params.max_inner_iterations)
					.c_str(),
				"warm start", i, params.num_init_retries, best_residual, params.fk_residual_tolerance,
				best_stop_reason.c_str(), best_iters_used);
			seed_xy_offset = RandomInBounds(params.bounds, rng);
		}
	}
	const int num_placed = std::count(placed.begin(), placed.end(), true);
	XYOffset mean{0.0, 0.0};
	for (int i = 0; i < n; ++i)
		if (placed[i])
		{
			mean.x += per_point[i].x / std::max(1, num_placed);
			mean.y += per_point[i].y / std::max(1, num_placed);
		}
	for (int i = 0; i < n; ++i)
		if (!placed[i])
			per_point[i] = mean;
	PublishRelaxationProgress(node, params, per_point, placed, mean);
	RCLCPP_INFO(
		node->get_logger(), "%s %-12s %d/%d points found a feasible offset  mean start (%+.4f, %+.4f)",
		ProgressTag(
			restart_number, std::max(1, params.num_restarts), -1, params.max_outer_iterations, -1,
			params.max_inner_iterations)
			.c_str(),
		"relaxation", num_placed, n, mean.x, mean.y);

	if (restart_number == 1 && params.fd_jacobian_check && num_placed > 0)
	{
		const int check_idx = static_cast<int>(std::find(placed.begin(), placed.end(), true) - placed.begin());
		CheckFkJacobian(
			node, state, jmg, tool0_link, targets[check_idx], joints[check_idx], per_point[check_idx],
			params.fd_epsilon);
	}

	std::vector<XYOffset> xy_offset = per_point;
	double mu = params.mu_initial;
	for (int j = 0; j < params.max_outer_iterations && rclcpp::ok(); ++j)
	{
		InnerResult inner = RunInnerSlp(
			node, robot_model, planning_scene_monitor, group_name, jmg, tool0_link, object_pose_original, targets,
			joints, xy_offset, mu, params, restart_number, j + 1);
		joints = inner.q;
		xy_offset = inner.xy_offset;

		XYOffset m{0.0, 0.0};
		for (const auto& b : xy_offset)
		{
			m.x += b.x / n;
			m.y += b.y / n;
		}
		const double spread = MaxSpread(xy_offset);

		RCLCPP_INFO(
			node->get_logger(),
			"%s %-12s mu %9.3g  mean xy_offset (%+.4f, %+.4f)  spread(m) %7.4f  fk_residual %.4f",
			ProgressTag(
				restart_number, std::max(1, params.num_restarts), j + 1, params.max_outer_iterations, -1,
				params.max_inner_iterations)
				.c_str(),
			"outer done", mu, m.x, m.y, spread, inner.fk_residual_max);

		if (spread < params.outer_convergence_tolerance)
		{
			xy_offset.assign(n, m);
			break;
		}
		mu *= params.mu_growth_factor;
	}

	RestartResult result;
	result.xy_offset = xy_offset.empty() ? XYOffset{} : xy_offset.front();
	result.joints = joints;
	for (int i = 0; i + 1 < n; ++i)
		result.total_joint_path_length += JointL1Distance(joints[i], joints[i + 1]);

	// Final reachability check against the actual (non-linearized) FK residual.
	moveit::core::RobotState check_state(robot_model);
	double worst = 0.0;
	int num_reachable = 0;
	for (int i = 0; i < n; ++i)
	{
		SetObjectPose(
			planning_scene_monitor,
			Eigen::Translation3d(result.xy_offset.x, result.xy_offset.y, 0.0) * object_pose_original);
		const auto residual = ComputeResidual(check_state, jmg, tool0_link, targets[i], joints[i], result.xy_offset);
		const double blended = std::max(residual.head<3>().norm(), params.rot_metric_scale * residual.tail<3>().norm());
		worst = std::max(worst, blended);
		if (blended < params.fk_residual_tolerance)
			num_reachable++;
	}
	result.fk_residual_max = worst;
	result.num_reachable = num_reachable;
	result.ok = (num_reachable == n);

	SetObjectPose(planning_scene_monitor, object_pose_original);
	return result;
}

}  // namespace

BstarPlacementResult SolveBstarPlacement(
	const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor, const std::string& group_name,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const std::vector<double>& start_reference_joints,
	const BstarPlacementParams& params)
{
	const Eigen::Isometry3d object_pose_original =
		MakeTransform(object_translation_original, object_rotation_original);
	const int n = static_cast<int>(tour_tcp_poses_original.size());

	RestartResult best;
	bool have_best = false;
	for (int r = 0; r < std::max(1, params.num_restarts) && rclcpp::ok(); ++r)
	{
		std::mt19937 rng(static_cast<unsigned int>(params.random_seed + r));
		RestartResult c = RunOuterRelaxation(
			node, r + 1, robot_model, planning_scene_monitor, group_name, object_translation_original,
			object_rotation_original, tour_tcp_poses_original, start_reference_joints, params, rng);
		// Reach count first, so a 33/38 restart beats a 28/38 one with a shorter path.
		const bool better = !have_best || c.num_reachable > best.num_reachable ||
			(c.num_reachable == best.num_reachable && c.total_joint_path_length < best.total_joint_path_length);
		RCLCPP_INFO(
			node->get_logger(), "%s %-12s xy_offset (%+.4f, %+.4f)  reach %2d/%-2d  path_length %9.3f%s",
			ProgressTag(
				r + 1, std::max(1, params.num_restarts), -1, params.max_outer_iterations, -1,
				params.max_inner_iterations)
				.c_str(),
			"restart done", c.xy_offset.x, c.xy_offset.y, c.num_reachable, n,
			c.total_joint_path_length, better ? "  <-- new best" : "");
		if (better)
		{
			best = c;
			have_best = true;
		}
	}

	SetObjectPose(planning_scene_monitor, object_pose_original);

	BstarPlacementResult result;
	result.num_total = n;
	if (have_best)
	{
		result.x = best.xy_offset.x;
		result.y = best.xy_offset.y;
		result.joint_solutions = best.joints;
		result.num_reachable = best.num_reachable;
		result.ok = best.ok;
		result.total_joint_path_length = best.total_joint_path_length;
		result.fk_residual_max = best.fk_residual_max;
		result.tour_order.resize(n);
		for (int i = 0; i < n; ++i)
			result.tour_order[i] = i;
	}

	if (result.ok)
		RCLCPP_INFO(
			node->get_logger(),
			"Done. Base offset (%.4f, %.4f) -- reaches all %d viewpoints, path length %.3f (fk_residual %.5f).",
			result.x, result.y, n, result.total_joint_path_length, result.fk_residual_max);
	else
		RCLCPP_WARN(
			node->get_logger(), "Done. Base offset (%.4f, %.4f) -- only reaches %d/%d viewpoints.", result.x,
			result.y, result.num_reachable, n);

	return result;
}

void ApplyBstarPlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original, double x,
	double y)
{
	const Eigen::Isometry3d object_pose_original =
		MakeTransform(object_translation_original, object_rotation_original);
	SetObjectPose(planning_scene_monitor, Eigen::Translation3d(x, y, 0.0) * object_pose_original);
}

void ExportBstarPlacementResult(const std::string& output_dir, const BstarPlacementResult& result)
{
	std::filesystem::create_directories(output_dir);

	Json::Value root;
	root["ok"] = result.ok;
	root["num_reachable"] = result.num_reachable;
	root["num_total"] = result.num_total;
	root["x"] = result.x;
	root["y"] = result.y;
	root["total_joint_path_length"] = result.total_joint_path_length;
	root["fk_residual_max"] = result.fk_residual_max;

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

	const std::string json_path = output_dir + "/bstar_placement_result.json";
	std::ofstream json_file(json_path);
	Json::StreamWriterBuilder writer_builder;
	writer_builder["indentation"] = "    ";
	std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
	writer->write(root, &json_file);

	printf("Saved base placement result JSON: %s\n", json_path.c_str());
}

visualization_msgs::msg::MarkerArray BuildBstarPlacementMarkerArray(
	const rclcpp::Time& stamp, const std::string& resolved_mesh_path, double mesh_scale,
	const Eigen::Vector3d& object_translation_original, const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original, const BstarPlacementResult& result)
{
	visualization_msgs::msg::MarkerArray markers;
	int id = 0;

	const Eigen::Isometry3d xform(Eigen::Translation3d(result.x, result.y, 0.0));
	const Eigen::Isometry3d object_pose_original =
		MakeTransform(object_translation_original, object_rotation_original);

	visualization_msgs::msg::Marker mesh_marker;
	mesh_marker.header.frame_id = "world";
	mesh_marker.header.stamp = stamp;
	mesh_marker.ns = "recommended_offset_object";
	mesh_marker.id = id++;
	mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh_marker.action = visualization_msgs::msg::Marker::ADD;
	mesh_marker.mesh_resource = "file://" + resolved_mesh_path;
	mesh_marker.mesh_use_embedded_materials = false;
	mesh_marker.pose = ToPoseMsg(xform * object_pose_original);
	mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = mesh_scale;
	mesh_marker.color.r = 0.7f;
	mesh_marker.color.g = 0.7f;
	mesh_marker.color.b = 0.7f;
	mesh_marker.color.a = 0.5f;
	markers.markers.push_back(mesh_marker);

	std::vector<char> reachable(tour_tcp_poses_original.size(), 0);
	for (int idx : result.tour_order)
		if (idx >= 0 && idx < static_cast<int>(reachable.size()))
			reachable[idx] = 1;

	visualization_msgs::msg::Marker line;
	line.header.frame_id = "world";
	line.header.stamp = stamp;
	line.ns = "recommended_offset_tour";
	line.id = id++;
	line.type = visualization_msgs::msg::Marker::LINE_STRIP;
	line.action = visualization_msgs::msg::Marker::ADD;
	line.pose.orientation.w = 1.0;
	line.scale.x = 0.002;
	line.color.r = 0.1f;
	line.color.g = 0.9f;
	line.color.b = 0.1f;
	line.color.a = 0.9f;

	auto waypoint_marker = [&](int vp) {
		const Eigen::Isometry3d local = xform * tour_tcp_poses_original[vp];
		visualization_msgs::msg::Marker sphere;
		sphere.header.frame_id = "world";
		sphere.header.stamp = stamp;
		sphere.ns = "recommended_offset_waypoints";
		sphere.id = id++;
		sphere.type = visualization_msgs::msg::Marker::SPHERE;
		sphere.action = visualization_msgs::msg::Marker::ADD;
		sphere.pose.position.x = local.translation().x();
		sphere.pose.position.y = local.translation().y();
		sphere.pose.position.z = local.translation().z();
		sphere.pose.orientation.w = 1.0;
		sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.008;
		sphere.color.r = reachable[vp] ? 0.1f : 0.9f;
		sphere.color.g = reachable[vp] ? 0.9f : 0.1f;
		sphere.color.b = 0.1f;
		sphere.color.a = 1.0f;
		return sphere;
	};

	for (int vp : result.tour_order)
	{
		if (vp < 0 || vp >= static_cast<int>(tour_tcp_poses_original.size()))
			continue;
		const visualization_msgs::msg::Marker sphere = waypoint_marker(vp);
		markers.markers.push_back(sphere);
		line.points.push_back(sphere.pose.position);
	}
	for (size_t vp = 0; vp < tour_tcp_poses_original.size(); ++vp)
		if (!reachable[vp])
			markers.markers.push_back(waypoint_marker(static_cast<int>(vp)));

	markers.markers.push_back(line);
	return markers;
}
