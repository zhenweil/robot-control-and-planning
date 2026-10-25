#pragma once
// Object placement in one descent (object_gtsp_co_optimize). Each iteration: a reach step moves the object to reach more
// viewpoints (room-then-reach frees what blocks the misses; once all are reached, what stops the reconfiguration edges'
// camera paths); then, for the tour (length + reconfigurations, joints kept off
// their limits), IK is re-solved with the object held still (self-motion slides, GTSP reorder, straight camera paths) and the
// object is moved. Each pose follows the object by IK from its last pose, so it keeps its posture. Repeats until nothing
// changes.

#include <map>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/room_margins.hpp"

namespace object_gtsp_co_optimize
{

using Joints = std::vector<double>;

// One IK pose of a viewpoint and its gap parts (all zero if exact).
struct ClosestIkCandidate
{
	Joints q;
	double gap = 0.0;  // m
	double angle_error = 0.0;  // rad
	double clearance_shortfall = 0.0;  // m
	Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
};

// Every viewpoint's arm pose at one object position.
struct ViewpointIkAtPosition
{
	int num_reached = 0;
	double gap_sum = 0.0;  // m, over the missed viewpoints
	std::vector<char> reached;  // one byte per viewpoint: written from parallel threads (std::vector<bool> packs bits and races)
	Eigen::Vector3d position = Eigen::Vector3d::Zero();	 // abs, m
	std::vector<ClosestIkCandidate> chosen_ik_per_viewpoint;  // per viewpoint: the pose in use (exact IK, else a closest pose) and its gap parts
	std::vector<std::vector<ClosestIkCandidate>> closest_ik_candidates;  // per missed viewpoint: closest poses GTSP may choose from
};

// A missed viewpoint's IK from one neighbour pose (exact, else closest): found, exact, and the pose with its gap parts.
struct NeighborIkResult
{
	bool found = false;
	bool exact = false;
	ClosestIkCandidate candidate;
};

// One candidate solution at one object position: every viewpoint's arm pose, the tour order and their cost.
struct TourSolutionAtObjectPosition
{
	ViewpointIkAtPosition viewpoint_ik;
	std::vector<int> order;	 // viewpoint visit order

	int num_blocked_edges = 0;  // tour edges whose straight move collides
	int num_reconfigurations = 0;  // tour edges the arm can't follow along the straight camera path
	double room_shortfall = 0.0;  // rad^2: summed squared shortfall of joint room below room_band, over all poses
	double smallest_margin = 0.0;  // rad beyond the hard limit, over the reached poses
	double joint_travel_cost = 0.0;	 // GTSP edge cost summed from home (tour length in rad with joint_distance_weight 1)
	double weighted_total_cost = 0.0; // Joint_travel_cost+miss_cost+gap_cost+reconfiguration_cost+joint_room
};

// Where the object may go and how long the descent may run.
struct DescentLimits
{
	Eigen::Vector3d search_box_lower_bound = Eigen::Vector3d::Zero();  // abs, m
	Eigen::Vector3d search_box_upper_bound = Eigen::Vector3d::Zero();
	double time_budget = 300.0;  // s for the descent
	int max_steps = 200;  // iterations of the descent
};

// Parts of the method that can be turned off (ablations).
struct MethodSwitches
{
	bool use_room = true;  // room-then-reach: moves that free what blocks the misses, then (once all are reached) the reconfiguration edges, treated as misses
	bool use_slide = true;  // poses slide along their self-motion
	bool tour_object_move = true;  // false = the object moves only for reach
	bool freeze_object_when_reached = false;  // true = the object stays put once every viewpoint is reached
};

// The weighted total cost's weights.
struct CostWeights
{
	double miss_cost = 10.0;  // cost per missed viewpoint
	double initial_penalty = 100.0;  // cost per m of missed gap
	double penalty_growth = 10.0;
	double max_penalty = 1e5;
	double allowed_reach_loss = 0.15;  // share of the reached viewpoints a room-then-reach move may lose
	double reconfiguration_cost = 2.0;  // cost per reconfiguration
	double joint_room_band = 0.2;  // rad beyond the hard limit a joint should keep
	double joint_room_weight = 50.0;  // cost per rad^2 of room shortfall
	double blocked_edge_charge = 4.0;  // x edge length extra for a blocked edge with no free detour through home; 0 = off
};

// Object step and pose slide sizes.
struct StepSizes
{
	double initial_step = 0.02;  // m
	double min_step = 0.001;  // m
	double max_step = 0.05;  // m
	int max_line_search_tries = 4;  // tries per sub-step
	double initial_slide_step = 0.3;  // rad, largest pose slide
	double slide_joint_margin = 0.05;  // rad a slide stays from each (hard) joint limit
	double min_slide_gain = 0.05;  // a pose slide is kept only if the cost drops by more than this
};

struct IkSettings
{
	closest_ik::Options exact_options;  // exact IK (exact_only)
	closest_ik::Options closest_options;  // closest IK (try_exact_first off)
	double jacobian_damping = 1e-3;
	int random_ik_seed = 42;  // seeds the random IK starts
	int ik_attempts = 10;  // random start poses when a missed viewpoint's own pose fails
	int closest_ik_extra_starts = 3;  // random starts also tried for a miss's closest pose
	double closest_gap_slack = 0.005;  // m: a miss's closest poses this near its best gap stay GTSP candidates
	double exact_finish_gap = 0.05;  // m: a closest pose this near its target gets exact IK started from it
};

// Tour edge cost and the GTSP solve.
struct TourSettings
{
	joint_travel::EdgeWeights edge_weights;  // the joint speed limits are filled in by the optimizer
	int two_opt_rounds = 5;
	// Reconfiguration: the arm can't follow the camera on a straight line (position) with an even turn (angle) from one
	// viewpoint's pose to the next, one Jacobian update per step and exact IK at the end, without a joint jump, a joint near
	// its limit or a collision.
	double path_check_position_step = 0.01;  // m of camera travel per step
	double path_check_angle_step = 0.035;  // rad of camera turn per step
	double path_check_max_joint_jump = 0.2;  // rad: a larger change of any joint in one step is a flip
	double path_check_joint_margin = 0.05;  // rad: a step this near a (hard) joint limit fails
	int gtsp_blocked_rounds = 2;  // GTSP re-solves charging blocked and reconfiguration edges
	int num_reference_postures = 4;  // fixed random arm postures; each reached viewpoint's IK from them is a GTSP candidate (0 = off)
};

class Optimizer
{
public:
	// viewpoints: relative to the object position. progress_pub: live markers, or null for none; mesh_path and mesh_scale draw the object in them.
	Optimizer(
		rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model, planning_scene_monitor::PlanningSceneMonitorPtr scene,
		const std::string& group_name, std::vector<Eigen::Isometry3d> viewpoints, const Eigen::Matrix3d& object_rotation, Joints home,
		DescentLimits descent_limits, MethodSwitches method_switches, CostWeights cost_weights, StepSizes step_sizes, IkSettings ik_settings,
		TourSettings tour_settings, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub, std::string mesh_path,
		double mesh_scale);

	// The one descent from `requested_start` (abs, clamped to the search box).
	TourSolutionAtObjectPosition runDescent(const Eigen::Vector3d& requested_start);
	std::vector<Joints> getPosesInTourOrder(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const;
	double getSmallestJointMargin(const std::vector<Joints>& poses) const;
	const joint_travel::EdgeWeights& getEdgeWeights() const;
	const std::vector<int>& getCountedJoints() const;

	void publishProgressMarkers(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order, const Eigen::Vector3d& object_direction);

private:
	// Descent loop and its steps
	Eigen::Vector3d clampToSearchBox(const Eigen::Vector3d& position) const;
	void refinePosesAndComputeCost(TourSolutionAtObjectPosition& solution);
	void improveOrderAndPosturesWithObjectFixed(TourSolutionAtObjectPosition& solution_to_complete);
	bool improveArmPosesWithObjectFixed(TourSolutionAtObjectPosition& current_solution, int iteration);
	bool moveObjectToReachMore(TourSolutionAtObjectPosition& current_solution, double& reach_step, double& reconfiguration_step, int iteration);
	bool moveObjectTowardMissedViewpoints(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration);
	bool moveObjectToLowerCost(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration);
	bool moveObjectToUnblockMisses(TourSolutionAtObjectPosition& current_solution, double object_step, int iteration);
	bool moveObjectToUnblockReconfigurations(TourSolutionAtObjectPosition& current_solution, double& object_step, int iteration);
	TourSolutionAtObjectPosition solveIkAndImproveTourAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses, const std::vector<int>& order);

	// Step directions
	Eigen::Vector3d getDirectionTowardMissedViewpoints(const ViewpointIkAtPosition& viewpoint_ik);
	Eigen::Vector3d getDirectionToLowerTourCost(const TourSolutionAtObjectPosition& solution);
	std::vector<Eigen::VectorXd> getSelfMotionSlidesToLowerTourCost(const TourSolutionAtObjectPosition& solution);
	Eigen::MatrixXd getSelfMotionProjector(const Eigen::MatrixXd& jacobian) const;
	Eigen::MatrixXd getToolJacobian(moveit::core::RobotState& state, const Joints& joint_values) const;
	Eigen::Vector3d combineMovesWithoutConflict(const std::vector<Eigen::Vector3d>& moves) const;
	std::vector<Eigen::VectorXd> getDTourLengthDJointsPerViewpoint(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const;
	std::vector<Joints> getPosesAfterJointSlides(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<Eigen::VectorXd>& joint_slides, double slide_step) const;
	Eigen::Vector3d dropPartsTighteningMargins(const Eigen::Vector3d& move, const std::vector<room_margins::Margin>& margins) const;
	Eigen::Vector3d getRoomMoveDirection(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<room_margins::Margin>& margins, double object_step, bool& keeps_reached_clear);

	// Joint room and blocking margins
	double getSmallestJacobianSingularValue(const Joints& joint_values);
	double getJointRoomShortfall(const Joints& joint_values, Eigen::VectorXd* slope) const;
	std::vector<room_margins::Margin> getBlockingMarginsOfPose(size_t viewpoint_index, const Joints& joint_values, double blocking_distance_multiple = 2.0);
	std::vector<room_margins::Margin> getBlockingMarginsOfViewpoints(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<size_t>& viewpoint_indices, double blocking_distance_multiple);
	std::vector<room_margins::Margin> getBlockingMarginsOfReconfigurations(const TourSolutionAtObjectPosition& solution, double blocking_distance_multiple);

	// IK
	uint32_t getRandomIkSeed(const Eigen::Vector3d& position, size_t viewpoint_index) const;
	std::vector<Joints> getChosenPoses(const ViewpointIkAtPosition& viewpoint_ik) const;
	double getClearanceShortfall(moveit::core::RobotState& state, const Joints& joint_values) const;
	std::vector<ClosestIkCandidate> keepCandidatesNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest_results);
	closest_ik::Result solveSmallestGapIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts);
	closest_ik::Result pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest_result);
	ViewpointIkAtPosition solveViewpointIkAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& start_poses, const std::vector<Joints>& fallback_start_poses = {});
	std::vector<closest_ik::Result> solveClosestIkFromStarts(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts);

	// Tour order and reconfigurations
	bool followStraightCameraPath(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose, Joints& end_pose);
	int followTourAlongStraightCameraPaths(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order);
	void bringMissesCloserFromNeighbors(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order);
	std::vector<size_t> getReconfigurationEdges(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order, std::vector<Joints>* arrival_poses = nullptr);
	void unwrapFullTurnJointsToward(Joints& pose, const Joints& reference) const;
	int findCandidateIndex(const std::vector<Joints>& candidates, const Joints& pose) const;
	std::vector<int> reorderTourWithGtsp(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order);
	void unwrapFullTurnJointsAlongTour(ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order) const;
	bool isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& from_pose, const Joints& to_pose) const;
	std::vector<std::pair<size_t, double>> getBlockedEdgeDetourCharges(const ViewpointIkAtPosition& viewpoint_ik, const std::vector<int>& order);

	// Logging
	void logMissedViewpoints(const ViewpointIkAtPosition& viewpoint_ik);
	void logTourSolution(const std::string& label, const TourSolutionAtObjectPosition& solution);

	rclcpp::Node::SharedPtr node;
	moveit::core::RobotModelConstPtr model;
	std::unique_ptr<moveit::core::RobotState> state;
	const moveit::core::LinkModel* tool = nullptr;
	const moveit::core::JointModelGroup* group = nullptr;
	planning_scene_monitor::PlanningSceneMonitorPtr scene;

	// Settings
	Joints home_pose;
	StepSizes step_sizes;
	IkSettings ik_settings;
	CostWeights cost_weights;
	TourSettings tour_settings;
	DescentLimits descent_limits;
	MethodSwitches method_switches;
	std::vector<int> counted_joints;
	std::vector<Joints> reference_postures;  // fixed for the run (from the IK seed): GTSP candidates in posture families the tour may not use yet
	std::vector<Eigen::Isometry3d> viewpoints;	// relative to the object position
	Eigen::Matrix3d object_rotation = Eigen::Matrix3d::Identity();

	// 
	unsigned int num_threads = 1;
	std::set<std::string> camera_side_links;
	std::chrono::steady_clock::time_point start_time;
	// Straight camera path results at one object position, keyed by (from pose, to pose): (followed, arrival pose); cleared when the object moves.
	std::map<std::pair<Joints, Joints>, std::pair<bool, Joints>> straight_path_cache;
	Eigen::Vector3d straight_path_cache_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	std::mutex straight_path_cache_mutex;
	double miss_gap_penalty = 0.0;  // cost per m of missed gap: initial_penalty at the descent start, grows by penalty_growth when a step changes nothing
	// Missed viewpoints' IK from neighbour poses at one object position, keyed by (viewpoint, start pose); cleared when the object moves.
	std::map<std::pair<size_t, Joints>, NeighborIkResult> neighbor_ik_cache;
	Eigen::Vector3d neighbor_ik_cache_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	std::mutex neighbor_ik_cache_mutex;
	double neighbor_ik_seconds = 0.0;  // wall time of bringMissesCloserFromNeighbors, for the log (reset per solution)

	// Visualization
	std::string mesh_path;  // object mesh for the progress markers
	double mesh_scale = 0.01;
	std::vector<Eigen::Vector3d> object_trail;	 // object positions this descent
	std::set<std::pair<std::string, int>> progress_marker_keys;	 // markers in the last progress publish
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
};

}  // namespace object_gtsp_co_optimize
