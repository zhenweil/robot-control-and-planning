#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#include "panda_arm_control/viewpoint_types.hpp"

// Builds a local, self-contained PlanningSceneMonitor (not synced with any external move_group --
// no startSceneMonitor()/requestPlanningSceneState() calls) and registers the object mesh as a
// collision object directly in it, so reachability can be checked in isolation without depending
// on panda_arm.launch.py/move_group being up.
planning_scene_monitor::PlanningSceneMonitorPtr BuildLocalCollisionScene(
	const rclcpp::Node::SharedPtr& node,
	const std::string& mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world);

// Computes each candidate's tcp_pose and solves IK for it, seeded from robot_state's current
// joint values, rejecting/retrying solutions that are in collision (self-collision or with the
// object registered in planning_scene_monitor) via a validity callback -- mirrors
// WaypointFollower::isStateCollisionFree (panda_arm_control.cpp). Fills in reachable/joint_solution.
std::vector<ViewpointCandidate> ComputeReachabilityWithCollisionCheck(
	std::vector<ViewpointCandidate> candidates,
	const moveit::core::RobotStatePtr& robot_state,
	const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera,
	double ik_timeout);

// Keeps only candidates with reachable == true. Since ComputeReachabilityWithCollisionCheck's
// validity callback rejects colliding IK solutions, this removes both unreachable and
// in-collision candidates in one step -- reachable == true already implies collision-free.
std::vector<ViewpointCandidate> FilterByReachability(std::vector<ViewpointCandidate> candidates);

// Fraction of total mesh area covered by the union of the given candidates' visible_mask.
double ComputeAreaVisibility(const MeshData& mesh, const std::vector<const ViewpointCandidate*>& selected);

struct TravelCostBreakdown
{
	double cartesian_distance_m = 0.0;	 // sum of euclidean_distance_m across the whole sequence
	double joint_distance_rad = 0.0;	 // sum of (unweighted) joint_distance_rad across the sequence
};

// Same sequence TourCost sums over (start_reference -> selected[0] -> selected[1] -> ...), but
// reports the Cartesian and joint-space components separately instead of combining them via
// joint_distance_weight, for diagnostics.
TravelCostBreakdown ComputeTravelCostBreakdown(
	const std::vector<const ViewpointCandidate*>& selected, const Eigen::Vector3d& start_reference_position,
	const std::vector<double>& start_reference_joints);

struct RkgaScpParams
{
	int population_size = 100;
	int num_generations = 100;
	double elite_fraction = 0.2;  // top fraction of population carried over unchanged each gen
	double mutant_fraction = 0.15;	 // fraction replaced with brand-new random chromosomes each gen
	double elite_bias = 0.7;  // probability an offspring gene is inherited from its elite parent
	double target_area_visibility = 0.95;
	unsigned int random_seed = 42;
};

// Random-key genetic algorithm (BRKGA) jointly solving viewpoint selection and visiting order:
// a chromosome is one random key in [0,1] per candidate; decoding sorts candidates by key and
// greedily adds each (in that order) as long as it contributes any new coverage, stopping once
// target_area_visibility is reached (order comes from the chromosome instead of greedy gain, and
// there's no minimum-gain threshold -- any positive contribution is accepted). Fitness is travel
// cost alone (TourCost of the decoded selection in that order, starting from
// start_reference_position/joints, via ComputeReachabilityWithCollisionCheck-style multi-solution
// dynamic programming) -- no per-viewpoint penalty, so evolution optimizes purely for cheaper
// travel, independent of how many viewpoints that ends up using.
std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	const Eigen::Vector3d& start_reference_position,
	const std::vector<double>& start_reference_joints,
	double joint_distance_weight,
	const RkgaScpParams& params);
