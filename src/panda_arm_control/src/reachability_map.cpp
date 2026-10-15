#include "panda_arm_control/reachability_map.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <filesystem>

#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>

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

ReachabilityMap ReachabilityMap::LoadOrBuild(
	const std::string& path, const moveit::core::RobotModelConstPtr& model, const std::string& group,
	const std::string& tip, const Options& options)
{
	const std::string key = model->getName() + "/" + group + "/" + tip + "/" + std::to_string(options.samples) + "/" +
		std::to_string(options.voxel) + "/" + std::to_string(options.directions) + "/" +
		std::to_string(options.limit_sharpness) + "/" + std::to_string(options.seed);
	ReachabilityMap map;
	if (Load(path, key, &map))
		return map;
	const auto t0 = std::chrono::steady_clock::now();
	map = Build(model, group, tip, options);
	map.key = key;
	map.built = true;
	map.build_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	map.Save(path);
	return map;
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

	const size_t n = static_cast<size_t>(std::max(1, options.samples));
	std::vector<Eigen::Vector3d> position(n), approach(n);
	std::vector<double> weight(n);
	std::vector<double> q;
	for (size_t i = 0; i < n; ++i)
	{
		state.setToRandomPositions(group, rng);
		state.update();
		state.copyJointGroupPositions(group, q);
		const Eigen::Isometry3d& t = state.getGlobalLinkTransform(tool);
		position[i] = t.translation();
		approach[i] = t.linear().col(2);
		weight[i] = JointRoom(*model, group, q, options.limit_sharpness);
	}

	ReachabilityMap map;
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
	for (size_t i = 0; i < n; ++i)
	{
		const Eigen::Array3i cell = ((position[i] - map.lo).array() / map.voxel).floor().cast<int>().max(0).min(map.dims - 1);
		map.At(map.Bin(approach[i]), cell) += static_cast<float>(weight[i]);
	}
	const float top = *std::max_element(map.values.begin(), map.values.end());
	if (top > 0.0f)
		for (float& v : map.values)
			v /= top;
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

float ReachabilityMap::At(int bin, const Eigen::Array3i& c) const
{
	return values[((static_cast<size_t>(bin) * dims.x() + c.x()) * dims.y() + c.y()) * dims.z() + c.z()];
}

float& ReachabilityMap::At(int bin, const Eigen::Array3i& c)
{
	return values[((static_cast<size_t>(bin) * dims.x() + c.x()) * dims.y() + c.y()) * dims.z() + c.z()];
}

double ReachabilityMap::Value(int bin, const Eigen::Vector3d& point, Eigen::Vector3d* gradient) const
{
	const Eigen::Array3d f = (point - lo).array() / voxel - 0.5;  // in voxel-center units
	const Eigen::Array3i i0 = f.floor().cast<int>();
	const Eigen::Array3d t = f - i0.cast<double>();
	double value = 0.0;
	*gradient = Eigen::Vector3d::Zero();
	for (int corner = 0; corner < 8; ++corner)
	{
		const Eigen::Array3i bit(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1);
		const Eigen::Array3d b = bit.cast<double>();
		const Eigen::Array3d w = b * t + (1.0 - b) * (1.0 - t);  // trilinear weight per axis
		const Eigen::Array3d dw = (2.0 * b - 1.0) / voxel;		 // its slope
		const double g = At(bin, (i0 + bit).max(0).min(dims - 1));
		value += g * w.prod();
		*gradient += g * Eigen::Vector3d(dw.x() * w.y() * w.z(), w.x() * dw.y() * w.z(), w.x() * w.y() * dw.z());
	}
	return value;
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

bool ReachabilityMap::Load(const std::string& path, const std::string& expected_key, ReachabilityMap* map)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return false;
	auto get = [&](void* data, size_t bytes) { return static_cast<bool>(in.read(static_cast<char*>(data), static_cast<std::streamsize>(bytes))); };
	size_t key_size = 0, num_dirs = 0;
	if (!get(&key_size, sizeof key_size) || key_size > 4096)
		return false;
	std::string key(key_size, '\0');
	if (!get(key.data(), key_size) || key != expected_key)
		return false;
	map->key = key;
	if (!get(&map->voxel, sizeof map->voxel) || !get(map->lo.data(), 3 * sizeof(double)) ||
		!get(map->dims.data(), 3 * sizeof(int)) || !get(&num_dirs, sizeof num_dirs) || num_dirs > 100000)
		return false;
	map->directions.resize(num_dirs);
	for (Eigen::Vector3d& d : map->directions)
		if (!get(d.data(), 3 * sizeof(double)))
			return false;
	map->values.resize(num_dirs * static_cast<size_t>(map->dims.prod()));
	return get(map->values.data(), map->values.size() * sizeof(float));
}
