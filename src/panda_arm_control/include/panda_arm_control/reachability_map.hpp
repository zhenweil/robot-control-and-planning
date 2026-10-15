#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>

// How well an arm can put its tool at a position with a given approach (tool z) direction. Built once from forward
// kinematics of random joint samples, so any robot model works; saved to disk and reused.
// Each sample counts by its joint-limit room (product over joints of 1 - exp(-k (q-lo)(hi-q)/range^2)), so poses
// reachable only at the joint limits count little. Values per (direction bin, voxel), normalized to [0, 1], and
// trilinear between voxel centers so they have a slope.
class ReachabilityMap
{
public:
	struct Options
	{
		int samples = 1000000;
		double voxel = 0.04;		   // m
		int directions = 100;		   // approach-direction bins
		double limit_sharpness = 20.0;  // k above; 0 = every sample counts 1
		int seed = 42;
	};

	// Loads `path` if it was built with the same robot, group, tip and options; otherwise builds and saves it.
	static ReachabilityMap LoadOrBuild(
		const std::string& path, const moveit::core::RobotModelConstPtr& model, const std::string& group,
		const std::string& tip, const Options& options);

	// Direction bin nearest to a unit approach direction.
	int Bin(const Eigen::Vector3d& approach) const;
	// Value at `point` in bin `bin`, and its gradient w.r.t. the point.
	double Value(int bin, const Eigen::Vector3d& point, Eigen::Vector3d* gradient) const;

	bool built = false;  // true if LoadOrBuild had to build it (not loaded)
	double build_seconds = 0.0;

private:
	static ReachabilityMap Build(
		const moveit::core::RobotModelConstPtr& model, const std::string& group, const std::string& tip,
		const Options& options);
	bool Save(const std::string& path) const;
	static bool Load(const std::string& path, const std::string& key, ReachabilityMap* map);
	float At(int bin, const Eigen::Array3i& cell) const;
	float& At(int bin, const Eigen::Array3i& cell);

	std::string key;  // robot, group, tip and options it was built with
	double voxel = 0.04;
	Eigen::Vector3d lo = Eigen::Vector3d::Zero();
	Eigen::Array3i dims = Eigen::Array3i::Zero();
	std::vector<Eigen::Vector3d> directions;
	std::vector<float> values;	// [bin][x][y][z]
};
