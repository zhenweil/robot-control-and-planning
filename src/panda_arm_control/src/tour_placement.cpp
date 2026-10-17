#include "panda_arm_control/tour_placement.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Sparse>
#include <osqp/osqp.h>

namespace tour_placement
{

Eigen::VectorXd GetSelfMotionDirection(const Eigen::MatrixXd& jacobian)
{
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
	return svd.matrixV().col(svd.matrixV().cols() - 1).normalized();
}

Step SolveStep(const Joints& start_pose, const std::vector<LinearizedPose>& poses, const std::vector<int>& counted_joints, const StepLimits& limits)
{
	const int num_poses = static_cast<int>(poses.size());
	const int num_joints = static_cast<int>(start_pose.size());
	const int num_variables = 3 + num_poses;  // object move, then one self-motion amount per pose
	// Joint change of pose k per variable: [dq_dp | self-motion in column 3 + k].
	auto get_motion = [&](int k)
	{
		Eigen::MatrixXd motion = Eigen::MatrixXd::Zero(num_joints, num_variables);
		motion.leftCols(3) = poses[static_cast<size_t>(k)].dq_dp;
		motion.col(3 + k) = poses[static_cast<size_t>(k)].self_motion;
		return motion;
	};

	// Objective: sum_k w_k ||r_k + D_k x||^2 with r_k the current edge and D_k its change per variable (counted joints).
	Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(num_variables, num_variables);
	Eigen::VectorXd linear = Eigen::VectorXd::Zero(num_variables);
	Eigen::MatrixXd previous_motion = Eigen::MatrixXd::Zero(num_joints, num_variables);
	for (int k = 0; k < num_poses; ++k)
	{
		const Joints& previous = k == 0 ? start_pose : poses[static_cast<size_t>(k - 1)].q;
		const Eigen::MatrixXd motion = get_motion(k);
		Eigen::VectorXd edge(static_cast<Eigen::Index>(counted_joints.size()));
		Eigen::MatrixXd edge_change(static_cast<Eigen::Index>(counted_joints.size()), num_variables);
		for (size_t c = 0; c < counted_joints.size(); ++c)
		{
			const int j = counted_joints[c];
			edge(static_cast<Eigen::Index>(c)) = poses[static_cast<size_t>(k)].q[static_cast<size_t>(j)] - previous[static_cast<size_t>(j)];
			edge_change.row(static_cast<Eigen::Index>(c)) = motion.row(j) - previous_motion.row(j);
		}
		const double weight = 1.0 / std::max(edge.norm(), 0.05);  // reweighted: squared distance / current distance ~ distance
		hessian += 2.0 * weight * edge_change.transpose() * edge_change;
		linear += 2.0 * weight * edge_change.transpose() * edge;
		previous_motion = motion;
	}
	hessian.diagonal().array() += 1e-6;

	// Constraints: each pose's joints within their limits (less the margin), then the step limits per variable.
	std::vector<Eigen::Triplet<double>> triplets;
	std::vector<c_float> lower, upper;
	for (int k = 0; k < num_poses; ++k)
	{
		const Eigen::MatrixXd motion = get_motion(k);
		for (int j = 0; j < num_joints; ++j)
		{
			const double q = poses[static_cast<size_t>(k)].q[static_cast<size_t>(j)];
			const int row = static_cast<int>(lower.size());
			for (int v = 0; v < num_variables; ++v)
				if (motion(j, v) != 0.0)
					triplets.emplace_back(row, v, motion(j, v));
			// A joint already inside the margin may not move further in, but needn't jump out.
			lower.push_back(std::min(limits.joint_min[static_cast<size_t>(j)] + limits.joint_margin - q, 0.0));
			upper.push_back(std::max(limits.joint_max[static_cast<size_t>(j)] - limits.joint_margin - q, 0.0));
		}
	}
	for (int v = 0; v < num_variables; ++v)
	{
		const bool frozen = v >= 3 && static_cast<size_t>(v - 3) < limits.frozen_poses.size() && limits.frozen_poses[static_cast<size_t>(v - 3)];
		const double bound = v < 3 ? limits.max_object_step * limits.free_axes(v) : frozen ? 0.0 : limits.max_self_motion;
		triplets.emplace_back(static_cast<int>(lower.size()), v, 1.0);
		lower.push_back(-bound);
		upper.push_back(bound);
	}

	const int num_constraints = static_cast<int>(lower.size());
	Eigen::SparseMatrix<double> constraint_matrix(num_constraints, num_variables);
	constraint_matrix.setFromTriplets(triplets.begin(), triplets.end());
	constraint_matrix.makeCompressed();
	const Eigen::SparseMatrix<double> hessian_upper = Eigen::MatrixXd(hessian.triangularView<Eigen::Upper>()).sparseView();

	// ros-humble-osqp-vendor ships OSQP's 0.6 API (as in bstar_placement.cpp).
	std::vector<c_float> constraint_values(constraint_matrix.valuePtr(), constraint_matrix.valuePtr() + constraint_matrix.nonZeros());
	std::vector<c_int> constraint_rows(constraint_matrix.innerIndexPtr(), constraint_matrix.innerIndexPtr() + constraint_matrix.nonZeros());
	std::vector<c_int> constraint_columns(constraint_matrix.outerIndexPtr(), constraint_matrix.outerIndexPtr() + num_variables + 1);
	std::vector<c_float> hessian_values(hessian_upper.valuePtr(), hessian_upper.valuePtr() + hessian_upper.nonZeros());
	std::vector<c_int> hessian_rows(hessian_upper.innerIndexPtr(), hessian_upper.innerIndexPtr() + hessian_upper.nonZeros());
	std::vector<c_int> hessian_columns(hessian_upper.outerIndexPtr(), hessian_upper.outerIndexPtr() + num_variables + 1);
	std::vector<c_float> linear_values(linear.data(), linear.data() + num_variables);
	csc constraint_csc{static_cast<c_int>(constraint_values.size()), num_constraints, num_variables, constraint_columns.data(), constraint_rows.data(), constraint_values.data(), -1};
	csc hessian_csc{static_cast<c_int>(hessian_values.size()), num_variables, num_variables, hessian_columns.data(), hessian_rows.data(), hessian_values.data(), -1};

	OSQPData data;
	data.n = num_variables;
	data.m = num_constraints;
	data.P = &hessian_csc;
	data.A = &constraint_csc;
	data.q = linear_values.data();
	data.l = lower.data();
	data.u = upper.data();
	OSQPSettings settings;
	osqp_set_default_settings(&settings);
	settings.verbose = 0;
	settings.polish = 1;
	settings.max_iter = 20000;
	OSQPWorkspace* work = nullptr;
	Step step;
	if (osqp_setup(&work, &data, &settings) == 0)
	{
		osqp_solve(work);
		if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE)
		{
			step.ok = true;
			step.object_move = Eigen::Vector3d(work->solution->x[0], work->solution->x[1], work->solution->x[2]);
			step.self_motion_amounts.assign(work->solution->x + 3, work->solution->x + num_variables);
		}
	}
	if (work)
		osqp_cleanup(work);
	return step;
}

}  // namespace tour_placement
