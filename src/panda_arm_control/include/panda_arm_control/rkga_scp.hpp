#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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

// (N+1)x(N+1) matrices of real travel cost: index 0 is the start reference (robot home), indices
// 1..N correspond to candidates[0..N-1] in order. cartesian_distance[a][b] == -1.0 means no valid
// plan was found between those two configurations (infeasible transition) -- in that case
// joint_distance[a][b] is also -1.0. Kept as two separate raw components (not pre-combined via a
// weight) so joint_distance_weight can be retuned and the GA rerun without recomputing this
// expensive matrix.
struct TravelCostMatrix
{
	std::vector<std::vector<double>> cartesian_distance;  // meters, summed tool0 position deltas
	std::vector<std::vector<double>> joint_distance;		// radians, summed joint-space deltas
};

// Precomputes real travel cost between every pair of candidates' joint_solution (plus the start
// reference), by actually running OMPL motion planning (self-contained -- its own PlanningPipeline
// against planning_scene_monitor's local scene, no external move_group needed) and measuring the
// resulting trajectory's actual path length, in both Cartesian terms (summed tool0 position deltas
// between consecutive waypoints, via forward kinematics) and joint-space terms (summed per-joint
// L2 deltas between consecutive waypoints) -- both from the same planned trajectory, instead of
// assuming a straight line for either. Meant to be computed once, up front, then reused as cheap
// lookups during the GA's many fitness evaluations, not recomputed per generation.
TravelCostMatrix ComputeEndEffectorTravelDistanceMatrix(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<ViewpointCandidate>& candidates,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double planning_time);

struct RkgaScpParams
{
	int population_size = 100;
	int num_generations = 100;
	double elite_fraction = 0.2;  // top fraction of population carried over unchanged each gen
	double mutant_fraction = 0.15;	 // fraction replaced with brand-new random chromosomes each gen
	double elite_bias = 0.7;  // probability an offspring gene is inherited from its elite parent
	double target_area_visibility = 0.95;
	unsigned int random_seed = 42;
	// Added to cartesian_distance (meters) as joint_distance_weight * joint_distance (radians) when
	// scoring a transition -- lets the GA also account for how much the arm reconfigures, not just
	// how far the TCP travels in Cartesian space, without recomputing the travel_cost_matrix.
	double joint_distance_weight = 0.0;
};

// Random-key genetic algorithm (BRKGA) jointly solving viewpoint selection and visiting order:
// a chromosome is one random key in [0,1] per candidate; decoding sorts candidates by key and
// greedily adds each (in that order) as long as it contributes any new coverage, stopping once
// target_area_visibility is reached (order comes from the chromosome instead of greedy gain, and
// there's no minimum-gain threshold -- any positive contribution is accepted). Fitness is the real
// end-effector travel distance plus params.joint_distance_weight * joint-space distance of the
// decoded selection in that order, looked up from travel_cost_matrix (see
// ComputeEndEffectorTravelDistanceMatrix) -- a sequence that uses an infeasible transition (-1.0 in
// the matrix) is rejected outright. No per-viewpoint penalty, so evolution optimizes purely for
// cheaper real travel, independent of viewpoint count.
// travel_cost_matrix must have been computed from exactly this same `candidates` vector, in the
// same order, or the index correspondence (matrix index i+1 <-> candidates[i]) breaks.
std::vector<const ViewpointCandidate*> SolveRkgaScp(
	const MeshData& mesh,
	const std::vector<ViewpointCandidate>& candidates,
	const TravelCostMatrix& travel_cost_matrix,
	const RkgaScpParams& params);

// Alternative to SolveRkgaScp's GA-based ordering: orders an already-selected set of viewpoints
// (e.g. from GreedySelectViewpoints) via nearest-neighbor + 2-opt local search, using the same real
// motion-planned costs (cartesian_distance + joint_distance_weight * joint_distance) from
// travel_cost_matrix that the GA's fitness function uses -- so the two ordering strategies can be
// compared under the exact same cost model, with selection left to whichever method the caller
// used. `candidates` must be the exact same vector travel_cost_matrix was built from (pointer
// identity is used to look up each selected candidate's matrix index), and every pointer in
// `selected` must point into it.
std::vector<const ViewpointCandidate*> NearestNeighborOrderMatrix(
	const std::vector<ViewpointCandidate>& candidates,
	std::vector<const ViewpointCandidate*> selected,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// 2-opt local search over an already-ordered tour (e.g. NearestNeighborOrderMatrix's output),
// repeatedly reversing whichever segment [i, j] most shortens total weighted travel_cost_matrix
// cost (see NearestNeighborOrderMatrix), starting from the home reference (matrix index 0), until
// no reversal improves it further.
std::vector<const ViewpointCandidate*> TwoOptImproveMatrix(
	const std::vector<ViewpointCandidate>& candidates,
	std::vector<const ViewpointCandidate*> ordered,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// Prints the raw per-component cost breakdown (cartesian, joint) of `selected` in its current
// order, plus totals and the weighted total -- lets joint_distance_weight be tuned by comparing
// real magnitudes, regardless of which algorithm (GA or NN+2-opt) produced the order. `candidates`
// must be the exact same vector travel_cost_matrix was built from, same as
// NearestNeighborOrderMatrix.
void PrintTourCostBreakdown(
	const std::vector<ViewpointCandidate>& candidates,
	const std::vector<const ViewpointCandidate*>& selected,
	const TravelCostMatrix& travel_cost_matrix,
	double joint_distance_weight);

// A single planned, ready-to-execute leg of the final tour. travel_cost_matrix only ever kept a
// scalar cost per pair (the trajectories used to measure it were discarded) and its pairs don't
// correspond to the final tour's consecutive legs anyway -- this is a focused re-plan of just the
// n-1 (or n, including the trailing return-to-home leg) legs actually used, so what gets executed
// is an exact trajectory that was actually planned, not re-derived from cost alone. ok is false if
// this particular re-plan failed (RRTConnect is randomized, so a re-plan isn't guaranteed to
// succeed even though the original cost-matrix pass found this transition feasible).
struct TourTrajectory
{
	bool ok = false;
	moveit_msgs::msg::RobotTrajectory trajectory;
};

// Plans the exact consecutive legs of `selected`, in order, starting from start_reference_joints
// (e.g. robot home): leg 0 is start_reference_joints -> selected[0]->joint_solution, leg i is
// selected[i-1]->joint_solution -> selected[i]->joint_solution. Since this is only
// selected.size() calls (not the O(n^2) of ComputeEndEffectorTravelDistanceMatrix), each leg can
// afford num_planning_attempts tries, keeping whichever attempt has the shortest measured
// end-effector travel distance -- mirrors WaypointFollower's rationale for re-running RRTConnect a
// few times (panda_arm_control.cpp).
std::vector<TourTrajectory> PlanFinalTourTrajectories(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::vector<const ViewpointCandidate*>& selected,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	double planning_time,
	int num_planning_attempts);
