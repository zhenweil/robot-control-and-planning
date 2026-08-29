#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Search bounds for the candidate fixed-base offset (x, y, yaw), relative to the robot's actual
// current mount (panda_link0). Kept small by default since a real mount usually can't move far.
struct BasePlacementBounds
{
	double x_min = -0.3, x_max = 0.3;
	double y_min = -0.3, y_max = 0.3;
	double yaw_min = -M_PI, yaw_max = M_PI;
};

struct BasePlacementParams
{
	BasePlacementBounds bounds;
	// Independent restarts from different random initializations -- keeps whichever converges
	// with the most reachable points, then lowest joint-space path length. Mirrors B*'s
	// (arXiv:2504.12719) up-to-10 retries with different random initializations.
	int num_restarts = 6;
	// Outer-loop iterations of progressive tightening (see SolveBasePlacement doc).
	int max_outer_iterations = 8;
	// Random candidate offsets tried per point per outer iteration.
	int candidates_per_point = 16;
	// IK random-restart retries per point when the seeded attempt fails.
	int ik_retries_per_point = 4;
	double radius_shrink_factor = 0.6;
	double convergence_tolerance_xy = 0.002;  // meters
	double convergence_tolerance_yaw = 0.01;  // radians
	double ik_timeout = 0.1;
	int random_seed = 42;

	// Optional: if set, publishes live convergence markers -- each tour point's current
	// individual base-pose guess, the running mean, and the current search radius -- after the
	// initial relaxation and every outer iteration, so the refinement loop can be watched in
	// RViz on this topic. nullptr (default) disables progress publishing entirely.
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	// Sleep this long after each progress publish so RViz has time to render before the next
	// iteration overwrites it. 0.0 (default) adds no artificial delay.
	double visualize_progress_delay_sec = 0.0;
};

struct BasePlacementResult
{
	bool ok = false;  // true iff every input pose is reachable at (x, y, yaw)
	int num_reachable = 0;
	int num_total = 0;
	double x = 0.0, y = 0.0, yaw = 0.0;  // offset relative to today's actual mount
	// One entry per input pose, in order; empty vector where unreachable at the final placement.
	std::vector<std::vector<double>> joint_solutions;
	// Sum of |q_{i+1} - q_i| across all reachable legs (Eq. 4 of arXiv:2504.12719).
	double total_joint_path_length = 0.0;
};

// Finds the best fixed base placement for a whole ordered end-effector trajectory (e.g. hgtsp's
// exported tour), inspired by B* (Zhao et al., "B*: Efficient and Optimal Base Placement for
// Fixed-Base Manipulators", arXiv:2504.12719).
//
// The paper's core idea: relax the fixed-base constraint by letting the base move per-timestep
// (trivially feasible, like a mobile manipulator), then progressively tighten an L1 penalty
// pulling every timestep's base pose toward their mean until they converge to one fixed pose.
// The paper solves each fixed-penalty subproblem via sequential linearization + an LP; here we
// swap that inner solve for direct IK-feasibility search instead (per the user's request), since
// the outer relaxation/tightening scheme -- not the LP -- is what actually finds a good
// placement:
//   1. Per point, independently search for any collision-free IK solution (mobile-base
//      relaxation), seeded from the previous point's own placement for continuity.
//   2. Repeat: recompute the mean placement across points, then re-search each point within a
//      shrinking radius around that mean (falling back to the full bounds for a point that has no
//      feasible pose nearby), until every point's placement is within tolerance of the mean.
//   3. Fix the base at the converged mean and run one final full-sequence IK pass (seeded
//      point-to-point, matching how the tour will actually be executed) to get the reported
//      joint_solutions and path length.
//   4. Repeat from a few random initializations and keep the best (most reachable, then shortest
//      path).
//
// object_translation_original/object_rotation_original are the object's pose relative to
// panda_link0 as currently mounted (mirrors real_cost_planning.cpp's BuildLocalCollisionScene
// convention). tour_tcp_poses_original are the tool0 target poses in that same (current-mount)
// frame -- e.g. straight from selected_robot_poses.json.
//
// planning_scene_monitor must already have the object registered at object_translation_original/
// object_rotation_original with id "object" (see BuildLocalCollisionScene); this function moves
// that collision object's pose during the search and leaves it at the *original* pose again
// before returning.
BasePlacementResult SolveBasePlacement(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const std::string& group_name,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const std::vector<double>& start_reference_joints,
	const BasePlacementParams& params);

// Writes base_placement_result.json to output_dir.
void ExportBasePlacementResult(const std::string& output_dir, const BasePlacementResult& result);

// Updates the registered "object" collision object's pose (relative to panda_link0, id "object",
// as registered by BuildLocalCollisionScene) to where it would appear if the robot's base sat at
// (x, y, yaw) instead of its real mount, given the object's pose at the real mount
// (object_translation_original/object_rotation_original). Used internally by SolveBasePlacement's
// search, and by callers (see base_placement_node.cpp) that want to drive the real robot through
// the recommended placement's tour: since the real base can't move, this reproduces the same
// relative geometry by moving the object instead. Only valid if that matches reality -- e.g. the
// object has actually been physically moved to match, or the base has actually been remounted --
// otherwise the real robot will move relative to where it *thinks* the object is, not where the
// real object actually is.
void ApplyBasePlacementToScene(
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	double x,
	double y,
	double yaw);

// Object mesh + tour waypoints re-expressed in the *recommended* base's frame (i.e. as they'd
// appear relative to the robot's real, unmoved base if the base had actually been placed at
// result.x/y/yaw) -- since panda_link0 itself can't move in this visualization, this shows the
// equivalent relative geometry instead. frame_id is "world" (coincides with panda_link0 by
// convention -- see BuildLocalCollisionScene -- and needs no TF lookup to render).
visualization_msgs::msg::MarkerArray BuildBasePlacementMarkerArray(
	const rclcpp::Time& stamp,
	const std::string& resolved_mesh_path,
	double mesh_scale,
	const Eigen::Vector3d& object_translation_original,
	const Eigen::Matrix3d& object_rotation_original,
	const std::vector<Eigen::Isometry3d>& tour_tcp_poses_original,
	const BasePlacementResult& result);
