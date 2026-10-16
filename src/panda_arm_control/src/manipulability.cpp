#include "panda_arm_control/manipulability.hpp"

#include <algorithm>
#include <cmath>

namespace manipulability
{

JointRoom JointRoomScore(
	const moveit::core::JointModelGroup* group, const moveit::core::RobotModelConstPtr& model,
	const std::vector<double>& q, double k)
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
	const std::vector<double>& q)
{
	state.setJointGroupPositions(group, q);
	state.update();
	Eigen::MatrixXd J;
	state.getJacobian(group, tool, Eigen::Vector3d::Zero(), J);
	return J;
}

double Value(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const std::vector<double>& q, double limit_sharpness)
{
	const Eigen::MatrixXd J = ToolJacobian(state, group, tool, q);
	const Eigen::MatrixXd Js = J * JointRoomScore(group, state.getRobotModel(), q, limit_sharpness).score.asDiagonal();
	return std::sqrt(std::max(0.0, (Js * Js.transpose()).determinant()));
}

// dw/dq_i = w tr((Js Js^T)^-1 Js dJs_i^T). For revolute joints column j of dJ/dq_i is
// [z_min(i,j) x Jv_max(i,j); i < j ? z_i x z_j : 0], built from J itself.
Eigen::VectorXd Gradient(
	moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* tool,
	const std::vector<double>& q, double limit_sharpness)
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

double SoftMin(const std::vector<double>& w, double tau)
{
	if (w.empty())
		return 0.0;
	const double lo = *std::min_element(w.begin(), w.end());
	double sum = 0.0;
	for (double v : w)
		sum += std::exp(-(v - lo) / tau);
	return lo - tau * std::log(sum);
}

std::vector<double> SoftMinShares(const std::vector<double>& w, double tau)
{
	std::vector<double> shares;
	if (w.empty())
		return shares;
	const double lo = *std::min_element(w.begin(), w.end());
	double sum = 0.0;
	for (double v : w)
		sum += std::exp(-(v - lo) / tau);
	for (double v : w)
		shares.push_back(std::exp(-(v - lo) / tau) / sum);
	return shares;
}

}  // namespace manipulability
