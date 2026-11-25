#pragma once
// Object placement and coverage tour, optimized together in one descent.
// Each iteration: reach step and KME, pose slides, then CPD.

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/room_margins.hpp"

namespace object_gtsp_co_optimize
{

using Joints = std::vector<double>;

// One IK solution for a viewpoint; the error fields are zero when exact.
struct IkSolution
{
	Joints joints;
	double gap = 0.0;                                          // m, position + scaled angle + clearance shortfall
	Eigen::Vector3d position_error = Eigen::Vector3d::Zero();  // m, viewpoint minus camera
	double angle_error = 0.0;                                  // rad
	double clearance_shortfall = 0.0;                          // m
};

// Every viewpoint's IK solution at one object placement.
struct PlacementIkSolutions
{
	int num_reached = 0;
	double gap_sum = 0.0;                                        // m, over the missed viewpoints
	std::vector<char> is_reached;                                // per viewpoint; char, not bool, for thread safety
	std::vector<IkSolution> ik_solutions;                        // per viewpoint, the solution in use
	Eigen::Vector3d object_position = Eigen::Vector3d::Zero();   // abs, m
	std::vector<std::vector<IkSolution>> closest_ik_candidates;  // per missed viewpoint, within the gap slack of the best
};

// A missed viewpoint's IK solved from one neighbour's pose.
struct NeighborIkResult
{
	bool found = false;
	bool exact = false;
	IkSolution solution;
};

// A placement, its viewpoint order and their cost.
struct TourSolution
{
	double total_cost = 0.0;             // tour length + gap and reconfiguration penalties
	double tour_length = 0.0;            // rad from home, blocked-edge charges included
	int num_blocked_edges = 0;           // edges whose straight joint move collides
	int num_reconfigurations = 0;        // edges the camera can't follow on a straight path
	double smallest_joint_margin = 0.0;  // rad, over the reached poses
	std::vector<int> order;
	PlacementIkSolutions placement_ik;
};

// Search box and run limits.
struct DescentLimits
{
	Eigen::Vector3d search_box_lower_bound = Eigen::Vector3d::Zero();  // abs, m
	Eigen::Vector3d search_box_upper_bound = Eigen::Vector3d::Zero();  // abs, m
	double time_budget = 300.0;  // s
	int max_iterations = 200;
};

// Method parts that can be switched off for ablations.
struct MethodSwitches
{
	bool use_kme = true;  // KME on misses, then on reconfigurations
	bool use_cpd = true;  // object moves that lower the tour cost
	bool freeze_object_when_reached = false;
};

// Cost terms and the KME keep rule.
struct CostWeights
{
	double gap_penalty = 100.0;  // per m of missed gap
	double reconfiguration_cost = 2.0;  // per reconfiguration
	double blocked_edge_charge = 4.0;  // x edge length when home gives no free detour; 0 = off
	double allowed_reach_loss = 0.15;  // percentage of reached viewpoints a KME move may lose
};

// Object step and pose slide sizes.
struct StepSizes
{
	double initial_step = 0.02;  // m
	double min_step = 0.001;  // m
	double max_step = 0.05;  // m
	int max_line_search_tries = 4;  // halvings per step
	double initial_slide_scale = 0.3;  // rad; first-try scale on the normalized pose slides
	double slide_joint_margin = 0.05;  // rad kept from each joint limit
	double min_slide_gain = 0.05;  // cost drop a slide must beat
};

struct IkSettings
{
	closest_ik::Options exact_options;
	closest_ik::Options closest_options;
	double jacobian_damping = 1e-3;
	int random_ik_seed = 42;
	int random_ik_attempts = 10;  // random starts when the previous pose fails
	int closest_ik_extra_starts = 3;  // random starts also tried for closest IK
	double closest_gap_slack = 0.005;  // m; closest poses this near the best stay GTSP options
	double exact_finish_gap = 0.05;  // m; closer closest poses get an exact IK try
};

// Edge cost, GTSP and the straight camera path check.
struct TourSettings
{
	joint_travel::EdgeWeights edge_weights;  // joint speed limits are filled in by the optimizer
	int two_opt_rounds = 5;
	double path_check_position_step = 0.01;  // m of camera travel per step
	double path_check_angle_step = 0.035;  // rad of camera turn per step
	double path_check_max_joint_jump = 0.2;  // rad; larger is a flip
	double path_check_joint_margin = 0.05;  // rad; closer to a limit fails
	int gtsp_penalty_rounds = 2;  // GTSP re-solves with blocked and reconfiguration edges charged
	int num_reference_postures = 4;  // random postures whose IK adds GTSP options
};

struct Settings
{
	DescentLimits descent_limits;
	MethodSwitches method_switches;
	CostWeights cost_weights;
	StepSizes step_sizes;
	IkSettings ik_settings;
	TourSettings tour_settings;
};

// Live RViz markers of the descent; a null publisher draws nothing.
struct ProgressDisplay
{
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
	std::string mesh_path;
	double mesh_scale = 0.01;
	bool show_tool_path = false;
};

class Optimizer
{
public:
	// viewpoints are relative to the object position.
	Optimizer(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model, planning_scene_monitor::PlanningSceneMonitorPtr scene, const std::string& group_name, std::vector<Eigen::Isometry3d> viewpoints, const Eigen::Matrix3d& object_rotation, Joints home_pose, Settings settings, ProgressDisplay progress_display);

	// The one descent from start (abs, clamped to the search box).
	TourSolution runDescent(const Eigen::Vector3d& start);
	// IK for every viewpoint, then the order and poses improved and scored.
	TourSolution solveTourAtPosition(const Eigen::Vector3d& object_position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const std::vector<int>& order);

	std::vector<Joints> getPosesInTourOrder(const PlacementIkSolutions& placement_ik, const std::vector<int>& order) const;
	double getSmallestJointMargin(const std::vector<Joints>& joint_poses) const;
	const joint_travel::EdgeWeights& getEdgeWeights() const;
	const std::vector<int>& getCountedJoints() const;
	void publishProgressMarkers(const PlacementIkSolutions& placement_ik, const std::vector<int>& order, const Eigen::Vector3d& object_direction);

private:
	// Descent steps
	bool takeReachStep(TourSolution& solution, double& reconfiguration_step, int iteration);
	bool moveTowardMisses(TourSolution& solution, Eigen::Vector3d& kept_direction, int iteration);
	bool expandMarginsOfMisses(TourSolution& solution, double object_step, const Eigen::Vector3d& kept_reach_direction, int iteration);
	bool expandMarginsOfReconfigurations(TourSolution& solution, double& object_step, int iteration);
	bool slideArmPoses(TourSolution& solution, int iteration);
	bool moveObjectDownCost(TourSolution& solution, double& object_step, int iteration);
	void recordObjectMove(const TourSolution& solution, const Eigen::Vector3d& direction);

	// Tour scoring at a fixed object position
	TourSolution improveTourForPlacement(const PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	void improveOrderAndPoses(TourSolution& solution);
	void refinePosesAndScore(TourSolution& solution);

	// Step directions
	Eigen::Vector3d getMoveTowardMisses(const PlacementIkSolutions& placement_ik) const;
	Eigen::Vector3d getDirectionDownTourLength(const TourSolution& solution);
	std::vector<Eigen::VectorXd> getPoseSlidesDownTourLength(const TourSolution& solution);
	std::vector<Eigen::VectorXd> getTourLengthGradientPerViewpoint(const PlacementIkSolutions& placement_ik, const std::vector<int>& order) const;
	std::vector<Joints> getPosesAfterSlides(const PlacementIkSolutions& placement_ik, const std::vector<Eigen::VectorXd>& slides, double slide_scale) const;
	Eigen::Vector3d getKmeDirection(const PlacementIkSolutions& placement_ik, const std::vector<room_margins::Margin>& margins, double object_step, bool& keeps_reached_clear);
	Eigen::Vector3d removePartsTighteningMargins(const Eigen::Vector3d& move, const std::vector<room_margins::Margin>& margins) const;

	// KME margins
	std::vector<room_margins::Margin> getMarginsOfPose(size_t viewpoint_index, const Joints& joints, double blocking_multiple);
	std::vector<room_margins::Margin> getMarginsOfViewpoints(const PlacementIkSolutions& placement_ik, const std::vector<size_t>& viewpoint_indices, double blocking_multiple);
	std::vector<room_margins::Margin> getMarginsOfCarriedPoses(const PlacementIkSolutions& placement_ik, const std::vector<size_t>& viewpoint_indices, const Eigen::Vector3d& object_move, double blocking_multiple);
	std::vector<room_margins::Margin> getMarginsOfReconfigurations(const TourSolution& solution, double blocking_multiple);
	Eigen::MatrixXd getJointRatePerObjectMove(const Joints& joints) const;
	double getSmallestSingularValue(const Joints& joints) const;

	// IK
	PlacementIkSolutions solvePlacement(const Eigen::Vector3d& object_position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const PlacementIkSolutions* kept_misses = nullptr);
	std::vector<closest_ik::Result> solveClosestIkFromStarts(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts, bool push_for_clearance);
	closest_ik::Result solveSmallestGapIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const Joints& start);
	closest_ik::Result pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest_result) const;
	std::vector<IkSolution> keepSolutionsNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest_results) const;
	IkSolution makeIkSolution(moveit::core::RobotState& state, const closest_ik::Result& result) const;
	double getClearanceShortfall(moveit::core::RobotState& state, const Joints& joints) const;
	bool isCameraSidePair(const std::string& pair_name) const;
	uint32_t getRandomIkSeed(const Eigen::Vector3d& object_position, size_t viewpoint_index) const;

	// Viewpoint order and edges
	std::vector<int> reorderTourWithGtsp(PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	std::vector<std::vector<Joints>> collectGtspOptions(PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	void bringMissesCloserFromNeighbors(PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	int followTourAlongStraightCameraPaths(PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	std::vector<size_t> getReconfigurationEdges(const PlacementIkSolutions& placement_ik, const std::vector<int>& order, std::vector<Joints>* arrival_poses = nullptr);
	bool followStraightCameraPath(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose, Joints& end_pose);
	std::vector<std::pair<size_t, double>> getBlockedEdgeCharges(const PlacementIkSolutions& placement_ik, const std::vector<int>& order);
	bool isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose) const;
	void unwrapFullTurnsAlongTour(PlacementIkSolutions& placement_ik, const std::vector<int>& order) const;
	void unwrapFullTurnsToward(Joints& pose, const Joints& reference) const;

	// Small helpers
	const Joints& getPoseBefore(const PlacementIkSolutions& placement_ik, const std::vector<int>& order, size_t tour_position) const;
	std::vector<Joints> getChosenJoints(const PlacementIkSolutions& placement_ik) const;
	const moveit::core::VariableBounds& getJointBounds(size_t joint_index) const;
	std::vector<size_t> getViewpointsWithReach(const PlacementIkSolutions& placement_ik, bool reached) const;
	Eigen::Isometry3d getViewpointTarget(const Eigen::Vector3d& object_position, size_t viewpoint_index) const;
	void placeObjectAt(const Eigen::Vector3d& object_position);
	Eigen::Vector3d clampToSearchBox(const Eigen::Vector3d& position) const;
	bool isAllReached(const PlacementIkSolutions& placement_ik) const;
	double getElapsedSeconds() const;
	void runInParallel(size_t num_items, const std::function<void(moveit::core::RobotState& thread_state, size_t item_index)>& work) const;

	// Logging
	void logTourSolution(const std::string& label, const TourSolution& solution) const;
	void logMissedViewpoints(const PlacementIkSolutions& placement_ik);

	rclcpp::Node::SharedPtr node;
	moveit::core::RobotModelConstPtr model;
	planning_scene_monitor::PlanningSceneMonitorPtr scene;
	std::unique_ptr<moveit::core::RobotState> state;
	const moveit::core::JointModelGroup* group = nullptr;
	const moveit::core::LinkModel* tool = nullptr;
	std::set<std::string> camera_side_links;  // excluded from the arm clearance

	std::vector<Eigen::Isometry3d> viewpoints;  // relative to the object position
	Eigen::Matrix3d object_rotation = Eigen::Matrix3d::Identity();
	Joints home_pose;
	Settings settings;
	std::vector<int> counted_joints;  // joints in the tour length; the camera spin is left out
	std::vector<Joints> reference_postures;
	unsigned int num_threads = 1;
	std::chrono::steady_clock::time_point start_time;

	// Caches valid at one object position; cleared when it moves.
	std::map<std::pair<Joints, Joints>, std::pair<bool, Joints>> straight_path_cache;  // (from, to) -> (followed, end pose)
	Eigen::Vector3d straight_path_cache_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	std::mutex straight_path_cache_mutex;
	std::map<std::pair<size_t, Joints>, NeighborIkResult> neighbor_ik_cache;  // (viewpoint, start pose) -> result
	Eigen::Vector3d neighbor_ik_cache_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	std::mutex neighbor_ik_cache_mutex;

	ProgressDisplay progress_display;
	std::vector<Eigen::Vector3d> object_trail;  // object positions so far
	std::set<std::pair<std::string, int>> published_marker_keys;
};

}  // namespace object_gtsp_co_optimize
