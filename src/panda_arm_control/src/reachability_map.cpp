#include "panda_arm_control/reachability_map.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <algorithm>
#include <chrono>
#include <filesystem>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

namespace
{

// Even directions on the sphere (Fibonacci).
std::vector<Eigen::Vector3d> SphereDirections(int n)
{
	std::vector<Eigen::Vector3d> dirs;
	for (int i = 0; i < n; ++i)
	{
		const double phi = std::acos(1.0 - 2.0 * (i + 0.5) / n);
		const double theta = M_PI * (1.0 + std::sqrt(5.0)) * (i + 0.5);
		dirs.emplace_back(std::cos(theta) * std::sin(phi), std::sin(theta) * std::sin(phi), std::cos(phi));
	}
	return dirs;
}

// Product over the group's joints of their room scores (1 at mid-range, 0 at a limit).
double JointRoom(
	const moveit::core::RobotModel& model, const moveit::core::JointModelGroup* group, const std::vector<double>& q,
	double k)
{
	double room = 1.0;
	for (size_t i = 0; i < q.size() && k > 0.0; ++i)
	{
		const moveit::core::VariableBounds& b = model.getVariableBounds(group->getVariableNames()[i]);
		const double range = b.max_position_ - b.min_position_;
		if (b.position_bounded_ && range > 0.0)
			room *= 1.0 - std::exp(-k * std::max(0.0, (q[i] - b.min_position_) * (b.max_position_ - q[i])) / (range * range));
	}
	return room;
}

}  // namespace

std::string ReachabilityMap::Key(
	const moveit::core::RobotModel& model, const std::string& group, const std::string& tip, const Options& options)
{
	return model.getName() + "/" + group + "/" + tip + "/" + std::to_string(options.samples) +
		(options.self_collision ? "/self-collision/" : "/") + std::to_string(options.voxel) + "/" + std::to_string(options.directions) + "/" +
		std::to_string(options.limit_sharpness) + "/" + std::to_string(options.seed);
}

std::string ReachabilityMap::DefaultPath(const std::string& group)
{
	const char* home = std::getenv("HOME");
	return std::string(home ? home : "/tmp") + "/.ros/reachability_" + group + ".bin";
}

size_t ReachabilityMap::NumNonzero() const
{
	return static_cast<size_t>(std::count_if(values.begin(), values.end(), [](float v) { return v > 0.0f; }));
}

ReachabilityMap ReachabilityMap::Build(
	const moveit::core::RobotModelConstPtr& model, const std::string& group_name, const std::string& tip,
	const Options& options)
{
	moveit::core::RobotState state(model);
	state.setToDefaultValues();
	const moveit::core::JointModelGroup* group = state.getJointModelGroup(group_name);
	const moveit::core::LinkModel* tool = model->getLinkModel(tip);
	random_numbers::RandomNumberGenerator rng(static_cast<boost::uint32_t>(std::max(1, options.seed)));

	// Self-collision only (no world), with the allowed pairs from the robot's SRDF.
	planning_scene::PlanningScene scene(model);
	collision_detection::CollisionRequest request;
	request.group_name = group_name;
	const size_t n = static_cast<size_t>(std::max(1, options.samples));
	std::vector<Eigen::Vector3d> position, approach;
	std::vector<double> weight;
	std::vector<double> q;
	const rclcpp::Logger log = rclcpp::get_logger("reachability_map");
	const auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < n; ++i)
	{
		if (i > 0 && i % std::max<size_t>(1, n / 20) == 0)
		{
			const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
			RCLCPP_INFO(
				log, "%3.0f%% of samples (%zu kept)  %.0f s elapsed, ~%.0f s left", 100.0 * i / n, position.size(), s,
				s * (n - i) / i);
		}
		state.setToRandomPositions(group, rng);
		state.update();
		if (options.self_collision)
		{
			collision_detection::CollisionResult result;
			scene.checkSelfCollision(request, result, state);
			if (result.collision)
				continue;
		}
		state.copyJointGroupPositions(group, q);
		const Eigen::Isometry3d& t = state.getGlobalLinkTransform(tool);
		position.push_back(t.translation());
		approach.push_back(t.linear().col(2));
		weight.push_back(JointRoom(*model, group, q, options.limit_sharpness));
	}

	ReachabilityMap map;
	map.key = Key(*model, group_name, tip, options);
	map.voxel = options.voxel;
	map.directions = SphereDirections(options.directions);
	Eigen::Vector3d hi = position.front();
	map.lo = position.front();
	for (const Eigen::Vector3d& p : position)
	{
		map.lo = map.lo.cwiseMin(p);
		hi = hi.cwiseMax(p);
	}
	map.lo.array() -= map.voxel;
	map.dims = ((hi - map.lo).array() / map.voxel + 1.0).ceil().cast<int>();
	map.values.assign(static_cast<size_t>(options.directions) * static_cast<size_t>(map.dims.prod()), 0.0f);
	for (size_t i = 0; i < position.size(); ++i)
	{
		const Eigen::Array3i cell = ((position[i] - map.lo).array() / map.voxel).floor().cast<int>().max(0).min(map.dims - 1);
		map.values[map.Index(map.Bin(approach[i]), cell)] += static_cast<float>(weight[i]);
	}
	const float top = *std::max_element(map.values.begin(), map.values.end());
	if (top > 0.0f)
		for (float& v : map.values)
			v /= top;
	map.ComputeField();
	return map;
}

int ReachabilityMap::Bin(const Eigen::Vector3d& approach) const
{
	int best = 0;
	for (size_t d = 1; d < directions.size(); ++d)
		if (directions[d].dot(approach) > directions[static_cast<size_t>(best)].dot(approach))
			best = static_cast<int>(d);
	return best;
}

size_t ReachabilityMap::Index(int bin, const Eigen::Array3i& c) const
{
	return ((static_cast<size_t>(bin) * dims.x() + c.x()) * dims.y() + c.y()) * dims.z() + c.z();
}

namespace
{

// Exact 1D squared distance transform (Felzenszwalb & Huttenlocher): out[q] = min_p (q - p)^2 + f[p].
std::vector<double> SquaredDistance1D(const std::vector<double>& f)
{
	const int n = static_cast<int>(f.size());
	std::vector<double> out(f.size());
	std::vector<int> v(f.size());
	std::vector<double> z(f.size() + 1);
	int k = 0;
	v[0] = 0;
	z[0] = -std::numeric_limits<double>::infinity();
	z[1] = std::numeric_limits<double>::infinity();
	for (int q = 1; q < n; ++q)
	{
		double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
		while (s <= z[k])
		{
			--k;
			s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]));
		}
		++k;
		v[k] = q;
		z[k] = s;
		z[k + 1] = std::numeric_limits<double>::infinity();
	}
	k = 0;
	for (int q = 0; q < n; ++q)
	{
		while (z[k + 1] < q)
			++k;
		out[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
	}
	return out;
}

}  // namespace

void ReachabilityMap::ComputeField()
{
	const double far = 1e12;  // squared cells: "no reached cell yet" (finite, so the transform stays exact)
	field.assign(values.size(), 0.0f);
	empty_bin.assign(directions.size(), true);
	const size_t per_bin = static_cast<size_t>(dims.prod());
	std::vector<double> d2(per_bin);
	for (int bin = 0; bin < static_cast<int>(directions.size()); ++bin)
	{
		const size_t base = static_cast<size_t>(bin) * per_bin;
		for (size_t i = 0; i < per_bin; ++i)
		{
			d2[i] = values[base + i] > 0.0f ? 0.0 : far;
			empty_bin[static_cast<size_t>(bin)] = empty_bin[static_cast<size_t>(bin)] && values[base + i] <= 0.0f;
		}
		if (empty_bin[static_cast<size_t>(bin)])
			continue;
		// Squared distance in cells, one axis at a time (x, then y, then z).
		for (int axis = 0; axis < 3; ++axis)
		{
			const int len = dims(axis);
			const size_t stride = axis == 0 ? static_cast<size_t>(dims.y() * dims.z()) : axis == 1 ? static_cast<size_t>(dims.z()) : 1;
			for (size_t start = 0; start < per_bin; ++start)
			{
				if ((start / stride) % static_cast<size_t>(len) != 0)
					continue;  // not the first cell of a line along this axis
				std::vector<double> line(static_cast<size_t>(len));
				for (int i = 0; i < len; ++i)
					line[static_cast<size_t>(i)] = d2[start + static_cast<size_t>(i) * stride];
				const std::vector<double> out = SquaredDistance1D(line);
				for (int i = 0; i < len; ++i)
					d2[start + static_cast<size_t>(i) * stride] = out[static_cast<size_t>(i)];
			}
		}
		for (size_t i = 0; i < per_bin; ++i)
			field[base + i] = values[base + i] > 0.0f ? values[base + i] : static_cast<float>(-std::sqrt(d2[i]) * voxel);
	}
}

ReachabilityMap::Sample ReachabilityMap::Value(int bin, const Eigen::Vector3d& point) const
{
	const Eigen::Array3d f = (point - lo).array() / voxel - 0.5;  // in voxel-center units
	const Eigen::Array3i i0 = f.floor().cast<int>();
	const Eigen::Array3d t = f - i0.cast<double>();
	Sample s;
	for (int corner = 0; corner < 8; ++corner)
	{
		const Eigen::Array3i bit(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
		const Eigen::Array3d b = bit.cast<double>();
		const Eigen::Array3d w = b * t + (1.0 - b) * (1.0 - t);  // trilinear weight per axis
		const Eigen::Array3d dw = (2.0 * b - 1.0) / voxel;		 // its slope
		const double g = field[Index(bin, (i0 + bit).max(0).min(dims - 1))];
		s.value += g * w.prod();
		s.gradient += g * Eigen::Vector3d(dw.x() * w.y() * w.z(), w.x() * dw.y() * w.z(), w.x() * w.y() * dw.z());
	}
	return s;
}

ReachabilityMap::Score ReachabilityMap::ScoreAt(
	const std::vector<Eigen::Isometry3d>& viewpoints, const std::vector<int>& bins, const Eigen::Vector3d& position,
	double tau) const
{
	std::vector<Sample> samples;
	double lowest = std::numeric_limits<double>::infinity();
	for (size_t v = 0; v < viewpoints.size(); ++v)
	{
		if (empty_bin[static_cast<size_t>(bins[v])])
			continue;  // a direction the arm never reaches: no information
		samples.push_back(Value(bins[v], position + viewpoints[v].translation()));
		lowest = std::min(lowest, samples.back().value);
	}
	Score score;
	if (samples.empty())
		return score;
	// Soft-min -tau log(sum exp(-v / tau)); each value's share of the slope is exp(-v / tau) / sum.
	double sum = 0.0;
	for (const Sample& s : samples)
		sum += std::exp(-(s.value - lowest) / tau);
	score.value = lowest - tau * std::log(sum);
	for (const Sample& s : samples)
		score.gradient += std::exp(-(s.value - lowest) / tau) / sum * s.gradient;
	return score;
}

// File: key length + key, voxel, lo, dims, direction count + directions, values.
bool ReachabilityMap::Save(const std::string& path) const
{
	std::filesystem::create_directories(std::filesystem::path(path).parent_path());
	std::ofstream out(path, std::ios::binary);
	if (!out)
		return false;
	auto put = [&](const void* data, size_t bytes) { out.write(static_cast<const char*>(data), static_cast<std::streamsize>(bytes)); };
	const size_t key_size = key.size(), num_dirs = directions.size();
	put(&key_size, sizeof key_size);
	put(key.data(), key_size);
	put(&voxel, sizeof voxel);
	put(lo.data(), 3 * sizeof(double));
	put(dims.data(), 3 * sizeof(int));
	put(&num_dirs, sizeof num_dirs);
	for (const Eigen::Vector3d& d : directions)
		put(d.data(), 3 * sizeof(double));
	put(values.data(), values.size() * sizeof(float));
	return static_cast<bool>(out);
}

std::optional<ReachabilityMap> ReachabilityMap::Load(
	const std::string& path, const moveit::core::RobotModelConstPtr& model, const std::string& group,
	const std::string& tip, const Options& options)
{
	const std::string expected_key = Key(*model, group, tip, options);
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return std::nullopt;
	ReachabilityMap loaded;
	ReachabilityMap* map = &loaded;
	auto get = [&](void* data, size_t bytes) { return static_cast<bool>(in.read(static_cast<char*>(data), static_cast<std::streamsize>(bytes))); };
	size_t key_size = 0, num_dirs = 0;
	if (!get(&key_size, sizeof key_size) || key_size > 4096)
		return std::nullopt;
	std::string key(key_size, '\0');
	if (!get(key.data(), key_size) || key != expected_key)
		return std::nullopt;
	map->key = key;
	if (!get(&map->voxel, sizeof map->voxel) || !get(map->lo.data(), 3 * sizeof(double)) ||
		!get(map->dims.data(), 3 * sizeof(int)) || !get(&num_dirs, sizeof num_dirs) || num_dirs > 100000)
		return std::nullopt;
	map->directions.resize(num_dirs);
	for (Eigen::Vector3d& d : map->directions)
		if (!get(d.data(), 3 * sizeof(double)))
			return std::nullopt;
	map->values.resize(num_dirs * static_cast<size_t>(map->dims.prod()));
	if (!get(map->values.data(), map->values.size() * sizeof(float)))
		return std::nullopt;
	map->ComputeField();
	return loaded;
}
