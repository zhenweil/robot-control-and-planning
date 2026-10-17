#pragma once
// Object placement: finds the object position (abs x, y, z; orientation stays nominal) from which the arm reaches
// every viewpoint with the least tour travel. Kinematic moves (gradient steps) inside reach attempts; a manipulability
// jump before each new reach attempt; GTSP restarts and tour-placement once every viewpoint is reached.

#include <chrono>
#include <limits>
#include <string>
#include <vector>
#include <optional>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "panda_arm_control/joint_travel.hpp"
#include "panda_arm_control/room_margins.hpp"
#include "panda_arm_control/scene_geometry.hpp"
#include "panda_arm_control/object_placement.hpp"

namespace object_placement
{

using Joints = std::vector<double>;
using Position = Eigen::Vector3d;  // object position, abs base frame; its orientation stays nominal

// A missed viewpoint: its closest collision-free pose's error (capped if none) and that pose (empty if none).
struct MissedViewpoint
{
	int viewpoint_index;
	Joints closest_pose;  // empty if none
	scene_geometry::Vector6d pose_error;  // from closest_pose to the viewpoint; capped if none
};

// The tour at one object position: what the optimizer minimizes.
struct TourSolution
{
	std::vector<Joints> ik_poses;  // parallel to viewpoint_order: chosen IK pose
	std::vector<int> viewpoint_order;  // viewpoint indices in visiting order
	std::vector<MissedViewpoint> missed_viewpoints;

	double miss_cost = 0.0; // miss_gap_weight * miss_gap_sum
	double miss_gap_sum = 0.0; // sum of missed viewpoints' capped gaps (m)
	double planned_cost = -1.0; // OMPL-planned joint travel; -1 = not planned
	double weighted_cost_sum = 0.0;
	double manipulability_sum = 0.0;
	double softmin_manipulability = 0.0;
	double joint_distance_plus_turn = 0.0;  // GTSP edge cost sum: joint distance + largest single-joint turn per edge	

	int num_reachable = 0;
	bool all_reachable = false;
};

// The best spot of one reach attempt.
struct ReachAttemptResult
{
	TourSolution tour_solution;
	bool all_reachable = false;
	Position position = Position::Zero();
	double weighted_cost_sum = std::numeric_limits<double>::max();
};

// A move found by the line search or room-then-reach: where, its quick solve, and the step length.
struct MoveCandidate
{
	Position next_position;
	double step_length = 0.0;  // m
	TourSolution tour_solution;  // quick solve at next_position
};

// Every move of the object, kept for RViz.
enum class MoveType
{
	KinematicRoomMove,
	ManipulabilityJump
};

struct MoveSegment
{
	MoveType type;
	Position from, to;
};


Position ClampToBounds(Position p, const ObjectPlacementBounds& b); // The position moved inside the bounds, axis by axis.
std::vector<int> GetLostViewpoints(const TourSolution& from, const TourSolution& to); // Viewpoints reached initially but lost after move.
std::vector<Joints> GetIkStartPoses(const TourSolution& sol, const Joints& fallback, int num_viewpoint);
std::vector<Joints> PrependHomePose(const Joints& home, const std::vector<Joints>& tour);

class ObjectPlacementOptimizer
{
public:
	// viewpoints_nominal: tour poses (abs) with the object at its nominal pose. home: the arm's pose before the tour.
	ObjectPlacementOptimizer(
		const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
		const planning_scene_monitor::PlanningSceneMonitorPtr& scene, const std::string& group_name,
		const Eigen::Vector3d& object_translation_nominal, const Eigen::Matrix3d& object_rotation_nominal,
		const std::vector<Eigen::Isometry3d>& viewpoints_nominal, const Joints& home, const ObjectPlacementParams& params);
	ObjectPlacementOptimizer(const ObjectPlacementOptimizer&) = delete;
	ObjectPlacementOptimizer& operator=(const ObjectPlacementOptimizer&) = delete;
	ObjectPlacementResult Optimize(); // Run object placement optimization

private:
	// Top level functions
	bool ResetAndSolveAtStart(const Position& start, int restart_in, const TourSolution* warm); // Initialize first solve
	ReachAttemptResult ReachAllViewpoints(const Position& from); // Try to reach all viewpoints. Closest IK + kinematic room then reach + manipulability jump
	void RefineGtspAndPlacement(int iter); // Refine tour and object placement after all viewpoints reached

	// Lower level functions
	void PrepareReachViewpoints();
	bool TryReachViewpoints(int iter); // Try to reach all viewpoints one attempt
	std::optional<MoveCandidate> TryKinematicRoomMove();
	std::optional<MoveCandidate> LineSearch(const Eigen::Vector3d& dir);
	std::optional<TourSolution> ConfirmImprovementWithFullSolve(const MoveCandidate& step);
	bool CommitMoveAndCheckConvergence(const MoveCandidate& step, TourSolution committed, int iter, const Eigen::Vector3d& g); // Actually move the object
	Position RunManipulabilityJump(const Position& from, const TourSolution& warm, int restart_in);
	double GetJumpScore(const TourSolution& tour) const;
	void OptimizeTourAndPlacement();
	void RestartGtsp();

	// --- Solving and scoring ---
	TourSolution QuickSolveTour(const Position& at, ObjectPlacementParams p);
	TourSolution TestCandidateMove(const TourSolution& from_tour, const Position& at);
	TourSolution FindMostManipulablePoses(const Position& at, const ObjectPlacementParams& p);
	TourSolution RunGtspRestartsAndEvaluate(const Position& at, TourSolution best_sol, int num_restarts, bool warm_start, const std::vector<int>* fixed_order);
	std::vector<std::vector<Joints>> CollectIkPoses(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed, int max_solutions);
	std::vector<MissedViewpoint> FindClosestPosesForMissedViewpoints(const Position& at, const ObjectPlacementParams& p, const TourSolution& tour, const std::vector<Joints>& seed_q);
	TourSolution EvaluateObjectPosition(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed, const std::vector<int>* initial_order, int max_solutions, const std::vector<int>* fixed_order);
	TourSolution SampleIkAndSolveGtsp(const Position& at, const ObjectPlacementParams& p, const std::vector<Joints>& seed, const std::vector<int>* initial_order, int max_solutions, const std::vector<int>* fixed_order);

	// Compute gradient
	Eigen::Vector3d ComputeCostGradient(const TourSolution& sol, const ObjectPlacementParams& p); // Cost=closest_IK_gap + manipulability. Used for closest IK move
	Eigen::Vector3d ComputeManipulabilityGradient(const std::vector<Joints>& poses, double sum_weight, double softmin_weight);

	double ComputeWeightedCost(const TourSolution& sol) const;
	void RecordHistoryAndBest(const Position& at, const TourSolution& sol);
	TourSolution AddPlannedTravelToCost(const Position& at, TourSolution sol);
	double ComputeRealPlannedTravelCost(const Position& at, const TourSolution& sol);
	bool HasMoreReachedOrLowerCost(const TourSolution& a, const TourSolution& b) const;

	// --- Room-then-reach ---
	void TraceRoomMove(const std::string& what, int target, const Position& cand, const TourSolution& candidate_tour, const std::string& verdict);
	double GetStretch(const Joints& q);
	std::vector<room_margins::Margin> GetIkPoseMargins(int v, const Joints& q);
	room_margins::ArmModel BuildRoomMarginArmModel(const Position& at, const TourSolution& sol);
	Eigen::Vector3d GetMoveDirectionToCloseGap(int target, const TourSolution& here_at, bool first);
	std::optional<Joints> FindRoomierPose(int v, const Position& at, const room_margins::ArmModel& model, int tries);
	room_margins::Shortage GetMarginsThatRunOut(int v, const room_margins::ArmModel& model, const Position& at, const Eigen::Vector3d& d);

	// --- Fixed for the whole run ---
	std::string group_name;
	rclcpp::Node::SharedPtr node;
	const ObjectPlacementParams params;
	moveit::core::RobotModelConstPtr robot_model;
	planning_scene_monitor::PlanningSceneMonitorPtr scene;

	Eigen::Isometry3d object_pose;			  // object relative to its own position
	Eigen::Matrix3d object_rotation_nominal;
	Eigen::Vector3d object_translation_nominal;

	int num_viewpoint;
	std::vector<int> input_order;
	moveit::core::RobotState robot_state;
	const moveit::core::LinkModel* tool_link;
	std::vector<Eigen::Isometry3d> viewpoints;  // viewpoints relative to the object's position
	const moveit::core::JointModelGroup* joint_group;

	bool can_plan;	   // planned travel can be computed
	Joints home_pose;
	bool use_planned_travel_in_cost;  // and replaces the L2 travel in the cost
	std::vector<int> counted_joint_indices;  // joints that count as travel: all but joint 7, which only spins the camera
	moveit::core::GroupStateValidityCallbackFn state_validity;

	// --- Whole run ---
	ObjectPlacementResult run_result;		 // history and solve count; Optimize fills in the rest
	std::vector<MoveSegment> move_history;	 // every move, kept for RViz

	// --- Per reach attempt ---
	int stall_count = 0;
	int attempt_seed = 0;
	int attempt_index = 0;
	double ik_seconds = 0.0;  // exact IK time since the move began
	double closest_ik_seconds = 0.0;  // closest-IK time since the move began
	std::chrono::steady_clock::time_point move_start;  // for the move log

	Position current_position;	  // current object position
	TourSolution current_tour;  // full solve at current_position
	TourSolution quick_solve_tour;  // quick solve at current_position: what line-search tries compare against
	std::vector<int> locked_order;
	ReachAttemptResult best_in_attempt;	  // best of this reach attempt
	std::vector<Joints> ik_start_poses;
	std::vector<Joints> last_reached_poses;  // each viewpoint's joints when last reached
	std::vector<Joints> last_closest_poses;  // each missed viewpoint's last closest pose: keeps its gap continuous
	const std::vector<int>* fixed_order = nullptr;  // order the GTSP must keep; null = it reorders
	bool quick_solve_start_from_current_ik = true;  // quick solves follow the current poses (reach phase) or re-solve fresh (after)
	bool logged_gtsp_restarts_at_first_reach = false;
	
	// Log and visualization
	void LogJumpStep(int restart_in, int iter, double step);
	void LogGtspRestartsAtFirstReach();
	void PublishProgressMarkers(const Eigen::Vector3d& neg_grad);
};

}  // namespace object_placement
