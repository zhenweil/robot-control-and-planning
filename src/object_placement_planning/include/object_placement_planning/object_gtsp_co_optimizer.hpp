#pragma once
// Object placement in one descent (object_gtsp_co_optimize). Each iteration: a reach step moves the object to reach more
// viewpoints (room-then-reach when it loses reached ones); then, for the tour (length + reconfigurations, joints kept off
// their limits), IK is re-solved with the object held still (self-motion slides, GTSP reorder, posture swaps) and the
// object is moved. Each pose follows the object by IK from its last pose, so it keeps its posture. Repeats until nothing
// changes.

#include <chrono>
#include <cstdint>
#include <memory>
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

// One reconfiguration edge's posture gap: `viewpoint_index`'s closest pose started from `neighbor_pose` (its tour neighbour
// across the edge), i.e. the viewpoint in the neighbour's arm posture. Exact = the reconfiguration can be removed by swapping.
struct ReconfigurationPostureMatch
{
	size_t viewpoint_index = 0;
	Joints neighbor_pose;
	Joints pose_in_neighbor_posture;
	Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
	double gap = 0.0;  // m, as the reach gaps (position, angle, clearance shortfall)
	double angle_error = 0.0;  // rad
	double clearance_shortfall = 0.0;  // m, its clearance part
	bool exact = false;
};

// One evaluated spot: poses, tour order, and the reconfiguration edges still left with their posture gaps.
struct ObjectPositionEvaluation
{
	ViewpointIkAtPosition placement;
	std::vector<int> order;	 // viewpoint visit order
	std::vector<ReconfigurationPostureMatch> matches;  // per remaining reconfiguration edge

	int num_blocked_edges = 0;  // tour edges whose straight move collides
	int num_reconfigurations = 0;
	double room_shortfall = 0.0;  // rad^2: summed squared shortfall of joint room below room_band, over all poses
	double posture_gap_sum = 0.0;  // m	
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
	bool use_room = true;  // room tries for blocked misses, and room-then-reach for viewpoints a reach step loses
	bool room_in_reach_direction = false;  // also blend the room pull into every reach move
	bool use_neighbor_ik = true;  // reorders also offer IK from each pose's tour neighbours
	bool use_posture_match = true;  // reconfiguration edges pull the object and swap poses
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
	double reconfiguration_cost = 2.0;  // cost per reconfiguration
	double reconfiguration_weight = 10.0;  // cost per m of posture gap
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
	double reconfig_threshold = 1.0;  // rad: a move turning any joint (the last aside) more than this is a reconfiguration
	int gtsp_blocked_rounds = 2;  // GTSP re-solves charging blocked edges
};

// Downhill move of one step: the object's direction and each pose's joint slide along its self-motion.
struct DescentDirection
{
	Eigen::Vector3d object_direction = Eigen::Vector3d::Zero();  // unit, or zero
	std::vector<Eigen::VectorXd> joint_slides;  // per viewpoint, largest norm 1 over all poses; empty vector = no slide
	bool has_slide = false;
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

	// The one descent from `start` (abs, clamped to the search box).
	ObjectPositionEvaluation runDescent(const Eigen::Vector3d& start);
	std::vector<Joints> getPosesInTourOrder(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const;
	double getSmallestJointMargin(const std::vector<Joints>& poses) const;
	const joint_travel::EdgeWeights& getEdgeWeights() const;
	const std::vector<int>& getCountedJoints() const;

	void publishProgressMarkers(const ViewpointIkAtPosition& placement, const std::vector<int>& order, const Eigen::Vector3d& object_direction);

private:
	// Descent loop and its steps
	Eigen::Vector3d clampToSearchBox(const Eigen::Vector3d& position) const;
	ObjectPositionEvaluation runDescentFromStart(const Eigen::Vector3d& start);
	void updateWeightedTotalCost(ObjectPositionEvaluation& descent, double penalty);
	void evaluateCurrentAndReorderedTour(ObjectPositionEvaluation& descent, double penalty);
	bool improveArmPosesWithObjectFixed(ObjectPositionEvaluation& current, double penalty, int iteration);
	bool moveObjectToReachMore(ObjectPositionEvaluation& current, double& step, double penalty, int iteration);
	bool moveObjectToLowerCost(ObjectPositionEvaluation& current, double& step, double penalty, int iteration);
	bool moveObjectToUnblockMisses(ObjectPositionEvaluation& current, double step, double penalty, int iteration);
	ObjectPositionEvaluation evaluateObjectPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds, const std::vector<int>& order, double penalty);

	// Step directions
	Eigen::Vector3d getReachDirection(const ViewpointIkAtPosition& placement);
	DescentDirection getTourDirection(const ObjectPositionEvaluation& descent);
	Eigen::MatrixXd getSelfMotionProjector(const Eigen::MatrixXd& jacobian) const;
	Eigen::MatrixXd getToolJacobian(moveit::core::RobotState& state, const Joints& q) const;
	Eigen::Vector3d combineMovesWithoutConflict(const std::vector<Eigen::Vector3d>& moves) const;
	Eigen::Vector3d getRoomDirectionOfMissedViewpoint(const ViewpointIkAtPosition& placement, size_t v);
	std::vector<Eigen::VectorXd> getTourCostSlopePerPose(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const;
	std::vector<Joints> getPosesAfterJointSlides(const ViewpointIkAtPosition& placement, const DescentDirection& direction, double slide_step) const;
	Eigen::Vector3d getRoomThenReachDirection(const ViewpointIkAtPosition& placement, const std::vector<size_t>& lost, const Eigen::Vector3d& aim, double step);

	// Joint room and blocking margins
	double getSmallestJacobianSingularValue(const Joints& q);
	double getJointRoomShortfall(const Joints& q, Eigen::VectorXd* slope) const;
	std::vector<room_margins::Margin> getBlockingMarginsOfPose(size_t v, const Joints& q, double near_floors = 2.0);
	std::vector<room_margins::Margin> getBlockingMarginsOfViewpoints(const ViewpointIkAtPosition& placement, const std::vector<size_t>& misses, double near_floors);

	// IK
	uint32_t getRandomIkSeed(const Eigen::Vector3d& position, size_t v) const;
	std::vector<Joints> getChosenPoses(const ViewpointIkAtPosition& placement) const;
	double getClearanceShortfall(moveit::core::RobotState& state, const Joints& q) const;
	std::vector<ClosestIkCandidate> keepCandidatesNearBestGap(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest);
	closest_ik::Result solveSmallestGapIk(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts);
	closest_ik::Result pushArmToExactClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest);
	ViewpointIkAtPosition solveViewpointIkAtPosition(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds = {});
	std::vector<closest_ik::Result> solveClosestIkFromStarts(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts);

	// Tour order and reconfigurations
	bool isReconfiguration(const Joints& from, const Joints& to) const;
	void swapInReconfigurationPostureMatches(ObjectPositionEvaluation& descent);
	void unwrapFullTurnJointsToward(Joints& pose, const Joints& reference) const;
	int findCandidateIndex(const std::vector<Joints>& candidates, const Joints& pose) const;
	std::vector<int> reorderTourWithGtsp(ViewpointIkAtPosition& placement, const std::vector<int>& order);
	int countReconfigurations(const ViewpointIkAtPosition& placement, const std::vector<int>& order) const;
	void unwrapFullTurnJointsAlongTour(ViewpointIkAtPosition& placement, const std::vector<int>& order) const;
	bool isStraightJointMoveCollisionFree(moveit::core::RobotState& state, const Joints& a, const Joints& b) const;
	int countReconfigurationsAround(const ViewpointIkAtPosition& placement, const std::vector<int>& order, size_t k, const Joints& pose) const;
	std::vector<std::pair<size_t, double>> getBlockedEdgeDetourCharges(const ViewpointIkAtPosition& placement, const std::vector<int>& order);
	std::vector<ReconfigurationPostureMatch> findReconfigurationPostureMatches(const ViewpointIkAtPosition& placement, const std::vector<int>& order);

	// Logging
	void logMissedViewpoints(const ViewpointIkAtPosition& placement);
	void logObjectPositionEvaluation(const std::string& label, const ObjectPositionEvaluation& descent);

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
	std::vector<Eigen::Isometry3d> viewpoints;	// relative to the object position
	Eigen::Matrix3d object_rotation = Eigen::Matrix3d::Identity();

	// 
	unsigned int num_threads = 1;
	std::set<std::string> camera_side_links;
	std::chrono::steady_clock::time_point start_time;

	// Visualization
	std::string mesh_path;  // object mesh for the progress markers
	double mesh_scale = 0.01;
	std::vector<Eigen::Vector3d> object_trail;	 // object positions this descent
	std::set<std::pair<std::string, int>> progress_marker_keys;	 // markers in the last progress publish
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
};

}  // namespace object_gtsp_co_optimize
