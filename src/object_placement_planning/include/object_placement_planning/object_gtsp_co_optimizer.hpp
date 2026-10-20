#pragma once
// Object placement in one descent (object_gtsp_co_optimize). Each iteration: a reach step moves the object to reach more
// viewpoints (room-then-reach when it loses reached ones); then, for the tour (length + reconfigurations, joints kept off
// their limits), IK is re-solved with the object held still (self-motion slides, GTSP reorder, posture swaps) and the
// object is moved. Each pose follows the object by IK from its last pose, so it keeps its posture. Repeats until nothing
// changes.

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/joint_travel.hpp"

namespace object_gtsp_co_optimize
{

using Joints = std::vector<double>;

// One closest pose of a missed viewpoint and its gap parts.
struct MissCandidate
{
	Joints q;
	Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
	double angle_error = 0.0;  // rad
	double clearance_shortfall = 0.0;  // m
	double gap = 0.0;  // m
};

// Every viewpoint's arm pose at one object position.
struct Placement
{
	Eigen::Vector3d position = Eigen::Vector3d::Zero();	 // abs, m
	std::vector<Joints> poses;	// per viewpoint: exact IK, else the closest pose
	std::vector<char> reached;  // one byte per viewpoint: written from parallel threads (std::vector<bool> packs bits and races)
	std::vector<Eigen::Vector3d> position_errors;  // per viewpoint, tool to target; zero if reached
	std::vector<double> gaps;  // per viewpoint, m: position and angle (at the closest IK's rot_scale) + clearance shortfall; zero if reached
	std::vector<double> clearance_shortfalls;  // per viewpoint, m: the part of its gap from arm clearance
	std::vector<double> angle_errors;  // per viewpoint, rad, zero if reached
	std::vector<std::vector<MissCandidate>> miss_candidates;  // per missed viewpoint: closest poses GTSP may choose from
	int num_reached = 0;
	double gap_sum = 0.0;  // m, over the missed viewpoints
};

// One reconfiguration edge's posture gap: `viewpoint`'s closest pose started from `neighbor_pose` (its tour neighbour across
// the edge), i.e. the viewpoint in the neighbour's arm posture. Exact = the reconfiguration can be removed by swapping.
struct PostureMatch
{
	size_t viewpoint = 0;
	Joints neighbor_pose;
	Joints pose;
	Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
	double gap = 0.0;  // m, as the reach gaps (position, angle, clearance shortfall)
	double clearance_shortfall = 0.0;  // m, its clearance part
	double angle_error = 0.0;  // rad
	bool exact = false;
};

// One evaluated spot: poses, tour order, and the reconfiguration edges still left with their posture gaps.
struct Descent
{
	Placement placement;
	std::vector<int> order;	 // viewpoint visit order
	std::vector<PostureMatch> matches;  // per remaining reconfiguration edge
	int num_reconfigurations = 0;
	int num_blocked_edges = 0;  // tour edges whose straight move collides
	double posture_gap_sum = 0.0;  // m
	double tour_cost = 0.0;	 // GTSP edge cost summed from home (tour length in rad with joint_distance_weight 1)
	double room_shortfall = 0.0;  // rad^2: summed squared shortfall of joint room below room_band, over all poses
	double smallest_margin = 0.0;  // rad beyond the hard limit, over the reached poses
	double cost = 0.0;
};

// The tunable parameters (the node reads them from the co_* and shared ROS parameters).
struct Settings
{
	Eigen::Vector3d lower_bound = Eigen::Vector3d::Zero();  // abs, m: search box
	Eigen::Vector3d upper_bound = Eigen::Vector3d::Zero();
	joint_travel::EdgeWeights edge_weights;  // tour edge cost; the joint speed limits are filled in by the optimizer
	closest_ik::Options exact_options;  // exact IK (exact_only)
	closest_ik::Options closest_options;  // closest IK (try_exact_first off)
	double jacobian_damping = 1e-3;
	int two_opt_rounds = 5;
	int seed = 42;  // seeds the random IK starts
	double reconfig_threshold = 1.0;  // rad: a move turning any joint (the last aside) more than this is a reconfiguration
	double time_budget = 300.0;  // s for the descent
	int max_steps = 200;  // iterations of the descent
	int max_line_search_tries = 4;  // tries per sub-step
	int ik_attempts = 10;  // random start poses when a missed viewpoint's own pose fails
	int closest_ik_extra_starts = 3;  // random starts also tried for a miss's closest pose
	double closest_gap_slack = 0.005;  // m: a miss's closest poses this near its best gap stay GTSP candidates
	double exact_finish_gap = 0.05;  // m: a closest pose this near its target gets exact IK started from it
	double initial_step = 0.02;  // m
	double min_step = 0.001;  // m
	double max_step = 0.05;  // m
	double initial_penalty = 100.0;  // cost per m of missed gap
	double penalty_growth = 10.0;
	double max_penalty = 1e5;
	double miss_cost = 10.0;  // cost per missed viewpoint
	bool use_room = true;  // room tries for blocked misses, and room-then-reach for viewpoints a reach step loses
	bool room_in_reach_direction = false;  // also blend the room pull into every reach move
	bool use_neighbor_ik = true;  // reorders also offer IK from each pose's tour neighbours
	bool use_posture_match = true;  // reconfiguration edges pull the object and swap poses
	double reconfiguration_cost = 2.0;  // cost per reconfiguration
	double reconfiguration_weight = 10.0;  // cost per m of posture gap
	double blocked_edge_charge = 4.0;  // x edge length extra for a blocked edge with no free detour through home; 0 = off
	int gtsp_blocked_rounds = 2;  // GTSP re-solves charging blocked edges
	double joint_room_band = 0.2;  // rad beyond the hard limit a joint should keep
	double joint_room_weight = 50.0;  // cost per rad^2 of room shortfall
	bool use_slide = true;  // poses slide along their self-motion
	double initial_slide_step = 0.3;  // rad, largest pose slide
	double slide_joint_margin = 0.05;  // rad a slide stays from each (hard) joint limit
	bool tour_object_move = true;  // ablation: false = the object moves only for reach
	bool freeze_object_when_reached = false;  // ablation: true = the object stays put once every viewpoint is reached
	std::string mesh_path;  // object mesh for the progress markers
	double mesh_scale = 0.01;
	double progress_delay = 0.0;  // s pause after each progress publish
};

class Optimizer
{
public:
	// viewpoints: relative to the object position. progress_pub: live markers, or null for none.
	Optimizer(
		rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model, planning_scene_monitor::PlanningSceneMonitorPtr scene,
		const std::string& group_name, std::vector<Eigen::Isometry3d> viewpoints, const Eigen::Matrix3d& rotation, Joints home,
		Settings settings, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub);
	~Optimizer();

	// The one descent from `start` (abs, clamped to the search box).
	Descent Descend(const Eigen::Vector3d& start);

	std::vector<Joints> GetTourPoses(const Placement& placement, const std::vector<int>& order) const;
	// Smallest distance of any pose joint (camera spin aside) from its (hard) limit, rad.
	double GetSmallestJointMargin(const std::vector<Joints>& poses) const;
	void PublishProgress(const Placement& placement, const std::vector<int>& order, const Eigen::Vector3d& object_direction);
	const std::vector<int>& GetTravelJoints() const;
	const joint_travel::EdgeWeights& GetEdgeWeights() const;

private:
	class Impl;
	std::unique_ptr<Impl> impl;
};

}  // namespace object_gtsp_co_optimize
