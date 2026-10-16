#include "panda_arm_control/room_margins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace room_margins
{

std::vector<Margin> MarginsOf(const ArmModel& model, int v)
{
	std::vector<Margin> out;
	for (const Margin& m : model.margins)
		if (m.viewpoint_index == v)
			out.push_back(m);
	return out;
}

double RoomAlong(const std::vector<Margin>& margins, const Eigen::Vector3d& dir)
{
	double room = std::numeric_limits<double>::infinity();
	for (const Margin& m : margins)
	{
		const double rate = m.slope.dot(dir);
		if (rate < -1e-9)
			room = std::min(room, std::max(0.0, m.value - std::min(m.floor, 0.5 * m.value)) / -rate);
	}
	return room;
}

double Tightest(const std::vector<Margin>& margins)
{
	double lo = std::numeric_limits<double>::infinity();
	for (const Margin& m : margins)
		lo = std::min(lo, m.value / m.floor);
	return lo;
}

Eigen::Vector3d MostRoomDirection(const std::vector<Margin>& margins, double radius)
{
	const double tau = 1.0;  // floors
	auto softmin_slope = [&](const Eigen::Vector3d& d) {
		std::vector<double> u(margins.size());
		double lo = std::numeric_limits<double>::infinity();
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

}  // namespace room_margins
