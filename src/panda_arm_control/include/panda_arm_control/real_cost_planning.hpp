#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/viewpoint_types.hpp"

// Self-contained PlanningSceneMonitor, not synced with any external move_group, with the object
// mesh registered as a collision object -- lets reachability be checked without panda_arm.launch.py.
planning_scene_monitor::PlanningSceneMonitorPtr BuildLocalCollisionScene(
	const rclcpp::Node::SharedPtr& node,
	const std::string& mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world);

// IK per candidate with a collision-validity callback (self-collision or the registered object),
// retried on failure -- mirrors WaypointFollower::isStateCollisionFree. Fills reachable/joint_solution.
std::vector<ViewpointCandidate> ComputeReachabilityWithCollisionCheck(
	std::vector<ViewpointCandidate> candidates,
	const moveit::core::RobotStatePtr& robot_state,
	const moveit::core::JointModelGroup* jmg,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_world,
	const Eigen::Matrix3d& object_rotation_world,
	const Eigen::Isometry3d& t_tcp_camera,
	double ik_timeout);

// Keeps only reachable candidates -- reachable already implies collision-free, since the validity
// callback rejected colliding solutions.
std::vector<ViewpointCandidate> FilterByReachability(std::vector<ViewpointCandidate> candidates);

// Fraction of total mesh area covered by the union of the given candidates' visible_mask.
double ComputeAreaVisibility(const MeshData& mesh, const std::vector<const ViewpointCandidate*>& selected);

// Real travel cost matrix; index 0 is home, i+1 is candidates[i]. -1.0 means infeasible. Kept
// unweighted so joint_distance_weight can change without recomputing.
struct TravelCostMatrix
{
	std::vector<std::vector<double>> cartesian_distance;  // meters, summed tool0 position deltas
	std::vector<std::vector<double>> joint_distance;		// radians, summed joint-space deltas
};

// Real OMPL-planned travel cost for every candidate pair, both Cartesian and joint-space, computed
// once and reused as lookups by whichever ordering algorithm runs. O(n^2), parallelized across
// threads (see .cpp).
TravelCostMatrix ComputeEndEffectorTravelDistanceMatrix(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<ViewpointCandidate>& candidates,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double planning_time);

// Orders selected viewpoints via nearest-neighbor + 2-opt using real travel_cost_matrix costs.
// candidates must be the same vector the matrix was built from.
std::vector<const ViewpointCandidate*> NearestNeighborOrderMatrix(
	const std::vector<ViewpointCandidate>& candidates,
	std::vector<const ViewpointCandidate*> selected,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// 2-opt local search over an ordered tour, reversing segments that shorten total weighted
// travel_cost_matrix cost, until no reversal improves it further.
std::vector<const ViewpointCandidate*> TwoOptImproveMatrix(
	const std::vector<ViewpointCandidate>& candidates,
	std::vector<const ViewpointCandidate*> ordered,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// Prints per-leg cartesian/joint cost breakdown and totals -- lets joint_distance_weight be tuned
// against real magnitudes, regardless of which algorithm produced the order.
void PrintTourCostBreakdown(
	const std::vector<ViewpointCandidate>& candidates,
	const std::vector<const ViewpointCandidate*>& selected,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// One planned tour leg. Re-plans fresh (travel_cost_matrix's own trajectories were discarded) so
// execution runs an actually-planned trajectory. ok is false if re-planning failed.
struct TourTrajectory
{
	bool ok = false;
	moveit_msgs::msg::RobotTrajectory trajectory;
};

// Plans each consecutive leg of selected from start_reference_joints, retrying num_planning_attempts
// times per leg and keeping the shortest -- affordable since it's only selected.size() calls.
std::vector<TourTrajectory> PlanFinalTourTrajectories(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double planning_time,
	int num_planning_attempts);

// Drives the real robot through selected: re-plans exact legs, moves to start_reference_joints,
// executes each leg (publishing a current-target marker first). Aborts on first failure.
void ExecuteTourOnRobot(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const moveit::planning_interface::MoveGroupInterfacePtr& move_group,
	const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double execution_planning_time,
	int execution_planning_attempts);
