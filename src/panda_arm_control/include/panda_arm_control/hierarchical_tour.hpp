#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

#include "panda_arm_control/viewpoint_types.hpp"

struct HierarchicalTourParams
{
	// Below this, skip clustering and run flat NN+2-opt (matches nn2opt) -- not worth the overhead.
	int min_size_for_hierarchy = 8;

	// "alpha": weight on rotation vs. Cartesian distance in the cheap clustering-only proxy metric.
	// Not a substitute for real planned cost -- only builds the affinity propagation similarity matrix.
	double exemplar_metric_rotation_weight = 0.1;

	// Affinity propagation (Frey & Dueck 2007) hyperparameters -- picks exemplar count from data
	// instead of a fixed k, mirroring arXiv:2502.19591's own exemplar step.
	// Quantile of pairwise similarities. Median (0.5, the typical default) collapses to one
	// exemplar for tightly-clustered viewpoints; 0.9-0.95 avoids that empirically.
	double ap_preference_quantile = 0.95;
	double ap_damping = 0.9;
	int ap_max_iterations = 200;
	int ap_convergence_iterations = 15;  // stop early once the exemplar set is stable this long

	double joint_distance_weight = 1.0;
	double guide_path_planning_time = 2.0;  // per-pair planning_time for the exemplar-only matrix
	double cluster_planning_time = 2.0;	 // per-pair planning_time for each local cluster matrix
};

// Orders `selected` with far fewer real planning calls than a flat matrix: clusters via affinity
// propagation, orders exemplars into a small "guide path", then locally refines each cluster,
// chaining real edges across cluster boundaries. Returns pointers into `selected`'s own storage.
std::vector<const ViewpointCandidate*> SolveHierarchicalTour(
	const rclcpp::Node::SharedPtr& node,
	const moveit::core::RobotModelConstPtr& robot_model,
	const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
	std::vector<const ViewpointCandidate*> selected,
	const std::vector<double>& start_reference_joints,
	const std::string& group_name,
	const HierarchicalTourParams& params);
