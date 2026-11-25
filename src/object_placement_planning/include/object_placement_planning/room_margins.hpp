#pragma once
// Kinematic margins of arm poses and the object moves that raise them (KME).

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace room_margins
{

// One margin of a pose, with slopes over object position and joints.
struct Margin
{
	int viewpoint_index = -1;
	std::string name;
	double value = 0.0;
	double floor = 0.0;  // the margin's unit (its safe distance)
	Eigen::Vector3d slope = Eigen::Vector3d::Zero();
	Eigen::VectorXd joint_slope;
};

// Reached viewpoints' margins at one spot, plus per viewpoint its joints and dq/d(position).
struct ArmModel
{
	std::vector<Margin> margins;
	std::vector<std::vector<double>> q;
	std::vector<Eigen::MatrixXd> dq_dp;
};

// Margins that ran out on a move, and why.
struct Shortage
{
	std::vector<Margin> margins;
	std::string why;
};

std::vector<Margin> MarginsOf(const ArmModel& model, int viewpoint_index);

// How far the object can move along a direction before a margin runs low.
double RoomAlong(const std::vector<Margin>& margins, const Eigen::Vector3d& direction);

// Smallest margin, in floors.
double Tightest(const std::vector<Margin>& margins);

// Soft-min width in floors: a margin this far above the smallest weighs e^-1.
constexpr double soft_min_width = 1.0;

// Soft-min of the margins in floors; infinite if empty.
double SoftMin(const std::vector<Margin>& margins);

// Unit direction that most raises the soft-min to first order; zero if none.
Eigen::Vector3d MostRoomDirection(const std::vector<Margin>& margins, double radius);

}  // namespace room_margins
