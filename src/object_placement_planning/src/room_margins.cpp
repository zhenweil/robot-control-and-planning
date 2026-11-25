#include "object_placement_planning/room_margins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace room_margins
{

std::vector<Margin> MarginsOf(const ArmModel& model, int viewpoint_index)
{
	std::vector<Margin> viewpoint_margins;
	for (const Margin& margin : model.margins)
		if (margin.viewpoint_index == viewpoint_index)
			viewpoint_margins.push_back(margin);
	return viewpoint_margins;
}

double RoomAlong(const std::vector<Margin>& margins, const Eigen::Vector3d& direction)
{
	double room = std::numeric_limits<double>::infinity();
	for (const Margin& margin : margins)
	{
		const double rate = margin.slope.dot(direction);
		if (rate < -1e-9)
			room = std::min(room, std::max(0.0, margin.value - std::min(margin.floor, 0.5 * margin.value)) / -rate);
	}
	return room;
}

double Tightest(const std::vector<Margin>& margins)
{
	double tightest = std::numeric_limits<double>::infinity();
	for (const Margin& margin : margins)
		tightest = std::min(tightest, margin.value / margin.floor);
	return tightest;
}

double SoftMin(const std::vector<Margin>& margins)
{
	const double tightest = Tightest(margins);
	if (margins.empty())
		return tightest;
	double weight_sum = 0.0;
	for (const Margin& margin : margins)
		weight_sum += std::exp(-(margin.value / margin.floor - tightest) / soft_min_width);
	return tightest - soft_min_width * std::log(weight_sum);
}

Eigen::Vector3d MostRoomDirection(const std::vector<Margin>& margins, double radius)
{
	// Soft-min gradient at object move `move`, margins in floors.
	auto get_soft_min_gradient = [&](const Eigen::Vector3d& move)
	{
		std::vector<double> floors_left(margins.size());
		double tightest = std::numeric_limits<double>::infinity();
		for (size_t margin_index = 0; margin_index < margins.size(); ++margin_index)
		{
			floors_left[margin_index] = (margins[margin_index].value + margins[margin_index].slope.dot(move)) / margins[margin_index].floor;
			tightest = std::min(tightest, floors_left[margin_index]);
		}
		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
		double weight_sum = 0.0;
		for (size_t margin_index = 0; margin_index < margins.size(); ++margin_index)
		{
			const double weight = std::exp(-(floors_left[margin_index] - tightest) / soft_min_width);
			gradient += weight * margins[margin_index].slope / margins[margin_index].floor;
			weight_sum += weight;
		}
		return weight_sum > 0.0 ? Eigen::Vector3d(gradient / weight_sum) : gradient;
	};
	// Projected gradient ascent inside the ball of the given radius.
	Eigen::Vector3d move = Eigen::Vector3d::Zero();
	for (int iteration = 0; iteration < 100; ++iteration)
	{
		const Eigen::Vector3d gradient = get_soft_min_gradient(move);
		if (gradient.norm() < 1e-9)
			break;
		move += 0.5 * radius / (1.0 + 0.1 * iteration) * gradient.normalized();
		if (move.norm() > radius)
			move *= radius / move.norm();
	}
	return move.norm() < 1e-3 * radius ? Eigen::Vector3d::Zero() : Eigen::Vector3d(move.normalized());
}

}  // namespace room_margins
