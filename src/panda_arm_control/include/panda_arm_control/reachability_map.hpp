#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>

// How well an arm can put its tool at a position with a given approach (tool z) direction. Built once, by the
// build_reachability_map program, from forward kinematics of random joint samples (any robot model works), saved to
// disk and loaded by the planner.
// Self-colliding samples are dropped. Each sample counts by its joint-limit room (product over joints of 1 - exp(-k (q-lo)(hi-q)/range^2)), so poses
// reachable only at the joint limits count little. Values per (direction bin, voxel), normalized to [0, 1].
// Cells no sample reached hold minus the distance (m) to the nearest reached cell of the same direction bin, so an
// unreachable viewpoint still has a slope toward where it becomes reachable. Trilinear between voxel centers.
class ReachabilityMap
{
public:
	struct Options
	{
		int samples = 1000000;
		bool self_collision = true;  // drop samples where the arm hits itself
		double voxel = 0.04;		   // m
		int directions = 100;		   // approach-direction bins
		double limit_sharpness = 20.0;  // k above; 0 = every sample counts 1
		int seed = 42;
	};

	static ReachabilityMap Build(
		const moveit::core::RobotModelConstPtr& model, const std::string& group, const std::string& tip,
		const Options& options);
	bool Save(const std::string& path) const;
	// The map at `path`, if it was built with the same robot, group, tip and options.
	static std::optional<ReachabilityMap> Load(
		const std::string& path, const moveit::core::RobotModelConstPtr& model, const std::string& group,
		const std::string& tip, const Options& options);
	// ~/.ros/reachability_<group>.bin
	static std::string DefaultPath(const std::string& group);

	struct Sample
	{
		double value = 0.0;
		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();  // w.r.t. the point
	};
	struct Score
	{
		double value = 0.0;							 // soft-min over the viewpoints
		Eigen::Vector3d gradient = Eigen::Vector3d::Zero();  // w.r.t. the object position
	};

	// Direction bin nearest to a unit approach direction.
	int Bin(const Eigen::Vector3d& approach) const;
	// Value at `point` in bin `bin`: reachability in [0, 1], or minus the distance (m) to a reachable cell.
	Sample Value(int bin, const Eigen::Vector3d& point) const;
	// Soft-min (temperature tau) over viewpoints of their values with the object at `position`. Viewpoints are given
	// relative to the object position; bins are their approach-direction bins. Viewpoints whose direction bin the arm
	// never reaches are left out.
	Score ScoreAt(
		const std::vector<Eigen::Isometry3d>& viewpoints, const std::vector<int>& bins, const Eigen::Vector3d& position,
		double tau) const;

	Eigen::Array3i Dims() const { return dims; }
	size_t NumDirections() const { return directions.size(); }
	size_t NumNonzero() const;

private:
	static std::string Key(
		const moveit::core::RobotModel& model, const std::string& group, const std::string& tip, const Options& options);
	size_t Index(int bin, const Eigen::Array3i& cell) const;
	// Fills `field` from `values` (reachability, or minus the distance to the nearest reached cell).
	void ComputeField();

	std::string key;  // robot, group, tip and options it was built with
	double voxel = 0.04;
	Eigen::Vector3d lo = Eigen::Vector3d::Zero();
	Eigen::Array3i dims = Eigen::Array3i::Zero();
	std::vector<Eigen::Vector3d> directions;
	std::vector<float> values;	// [bin][x][y][z]: sample weight, normalized
	std::vector<float> field;	// same layout: what Value interpolates
	std::vector<bool> empty_bin;  // direction bins no sample reached
};
