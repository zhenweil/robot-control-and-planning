#pragma once
// Safety margins of reached viewpoints (joint-limit distance, stretch, collision distance) and the object moves
// that keep or raise them. Used by room-then-reach.

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace room_margins
{

// One margin of a reached viewpoint and its slope w.r.t. the object position (the arm following the viewpoint) and
// w.r.t. the joints.
struct Margin
{
	int viewpoint_index = -1;
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
	std::vector<std::vector<double>> q;
	std::vector<Eigen::MatrixXd> dq_dp;
};

// A lost viewpoint's margins that ran out on a move, and why.
struct Shortage
{
	std::vector<Margin> margins;
	std::string why;
};

// Viewpoint v's margins in the model.
std::vector<Margin> MarginsOf(const ArmModel& model, int v);

// How far the object can move along unit `dir` before a margin, to first order, reaches min(floor, value / 2).
double RoomAlong(const std::vector<Margin>& margins, const Eigen::Vector3d& dir);

// Smallest margin, in floors.
double Tightest(const std::vector<Margin>& margins);

// Unit direction that most raises these margins' soft-min (in floors), to first order: projected gradient ascent
// within |d| <= radius. Zero if none raises them.
Eigen::Vector3d MostRoomDirection(const std::vector<Margin>& margins, double radius);

}  // namespace room_margins
