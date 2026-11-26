#include "object_placement_planning/object_gtsp_co_optimize_setup.hpp"

#include <cstdint>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "object_placement_planning/marker_utils.hpp"

namespace object_gtsp_co_optimize
{

namespace
{

constexpr double mounting_surface_size = 4.0;  // m
constexpr double mounting_surface_thickness = 0.05;  // m

int GetIntParameter(rclcpp::Node& node, const std::string& name, int64_t fallback)
{
	return static_cast<int>(GetParameter<int64_t>(node, name, fallback));
}

DescentLimits ReadDescentLimits(rclcpp::Node& node)
{
	DescentLimits limits;
	limits.search_box_lower_bound = Eigen::Vector3d(GetParameter<double>(node, "x_min", -0.6), GetParameter<double>(node, "y_min", -0.6), GetParameter<double>(node, "z_min", -0.2));
	limits.search_box_upper_bound = Eigen::Vector3d(GetParameter<double>(node, "x_max", 0.6), GetParameter<double>(node, "y_max", 0.6), GetParameter<double>(node, "z_max", 0.2));
	limits.time_budget = GetParameter<double>(node, "co_time_budget_sec", 300.0);
	limits.max_iterations = GetIntParameter(node, "co_max_steps", 200);
	return limits;
}

MethodSwitches ReadMethodSwitches(rclcpp::Node& node)
{
	MethodSwitches switches;
	switches.use_kme = GetParameter<bool>(node, "co_room", true);
	switches.use_cpd = GetParameter<bool>(node, "co_tour_object_move", true);
	switches.freeze_object_when_reached = GetParameter<bool>(node, "co_freeze_object_when_reached", false);
	return switches;
}

CostWeights ReadCostWeights(rclcpp::Node& node)
{
	CostWeights weights;
	weights.gap_penalty = GetParameter<double>(node, "co_gap_penalty", 100.0);
	weights.allowed_reach_loss = GetParameter<double>(node, "co_allowed_reach_loss", 0.15);
	weights.reconfiguration_cost = GetParameter<double>(node, "co_reconfiguration_cost", 2.0);
	weights.blocked_edge_charge = GetParameter<double>(node, "co_blocked_edge_charge", 4.0);
	return weights;
}

StepSizes ReadStepSizes(rclcpp::Node& node)
{
	StepSizes steps;
	steps.max_line_search_tries = GetIntParameter(node, "co_max_line_search_tries", 4);
	steps.initial_step = GetParameter<double>(node, "co_initial_step", 0.02);
	steps.min_step = GetParameter<double>(node, "co_min_step", 0.001);
	steps.max_step = GetParameter<double>(node, "co_max_step", 0.05);
	steps.initial_slide_scale_deg = GetParameter<double>(node, "co_initial_slide_scale_deg", 17.18873);
	steps.slide_joint_margin_deg = GetParameter<double>(node, "co_slide_joint_margin_deg", 2.86479);
	steps.min_slide_gain = GetParameter<double>(node, "co_min_slide_gain", 0.05);
	return steps;
}

// Names shared with object_placement, so one params file serves all methods.
IkSettings ReadIkSettings(rclcpp::Node& node)
{
	closest_ik::SolveSettings options;
	options.iters = GetIntParameter(node, "closest_ik_iters", 60);
	options.margin = GetParameter<double>(node, "closest_ik_margin", 0.02);
	options.use_weighted_cost = GetParameter<bool>(node, "closest_ik_weighted_cost", false);
	options.weighted_iters = GetIntParameter(node, "closest_ik_weighted_iters", 20);
	options.position_weight = GetParameter<double>(node, "closest_ik_position_weight", 1.0);
	options.angle_weight = GetParameter<double>(node, "closest_ik_angle_weight", 0.01);
	options.collision_weight = GetParameter<double>(node, "closest_ik_collision_weight", 1.0);
	options.camera_object_weight = GetParameter<double>(node, "closest_ik_camera_object_weight", 1.0);
	options.self_distance_weight = GetParameter<double>(node, "closest_ik_self_distance_weight", 0.0);
	options.self_distance_margin = GetParameter<double>(node, "closest_ik_self_distance_margin", 0.08);
	options.joint_room_weight = GetParameter<double>(node, "closest_ik_joint_room_weight", 1e-3);
	options.rot_scale = GetParameter<double>(node, "rot_metric_scale", 0.1);
	options.limit_sharpness = GetParameter<double>(node, "manipulability_limit_sharpness", 20.0);
	options.ignore_roll = GetParameter<bool>(node, "ignore_camera_roll", false);
	options.exact_margin = GetParameter<double>(node, "exact_ik_margin", 0.0);
	IkSettings ik_settings;
	ik_settings.exact_solve_settings = options;
	ik_settings.exact_solve_settings.exact_only = true;
	ik_settings.closest_solve_settings = options;
	ik_settings.closest_solve_settings.try_exact_first = false;
	ik_settings.jacobian_damping = GetParameter<double>(node, "jacobian_damping", 1e-3);
	ik_settings.max_closest_ik_gap_to_retry_exact_ik = GetParameter<double>(node, "co_max_closest_ik_gap_to_retry_exact_ik", 0.05);
	ik_settings.random_ik_seed = GetIntParameter(node, "random_seed", 42);
	ik_settings.exact_ik_random_attempts = GetIntParameter(node, "co_exact_ik_random_attempts", 10);
	ik_settings.max_gap_above_best_closest_ik_pose = GetParameter<double>(node, "co_max_gap_above_best_closest_ik_pose", 0.005);
	ik_settings.closest_ik_random_attempts = GetIntParameter(node, "co_closest_ik_random_attempts", 3);
	return ik_settings;
}

TourSettings ReadTourSettings(rclcpp::Node& node)
{
	TourSettings tour_settings;
	tour_settings.edge_weights.joint_distance = GetParameter<double>(node, "joint_distance_weight", 1.0);
	tour_settings.edge_weights.max_turn = GetParameter<double>(node, "max_joint_deviation_weight", 1.0);
	tour_settings.edge_weights.time = GetParameter<double>(node, "travel_time_weight", 0.0);
	tour_settings.two_opt_rounds = GetIntParameter(node, "gtsp_two_opt_rounds", 5);
	tour_settings.reconfiguration_check_camera_move_per_step = GetParameter<double>(node, "reconfiguration_check_camera_move_per_step", 0.01);
	tour_settings.reconfiguration_check_camera_turn_per_step_deg = GetParameter<double>(node, "reconfiguration_check_camera_turn_per_step_deg", 2.00535);
	tour_settings.path_check_max_joint_jump_deg = GetParameter<double>(node, "path_check_max_joint_jump_deg", 11.45916);
	tour_settings.gtsp_penalty_rounds = GetIntParameter(node, "co_gtsp_blocked_rounds", 2);
	tour_settings.num_reference_postures = GetIntParameter(node, "gtsp_reference_postures", 4);
	return tour_settings;
}

}  // namespace

Settings ReadSettings(rclcpp::Node& node)
{
	Settings settings;
	settings.descent_limits = ReadDescentLimits(node);
	settings.method_switches = ReadMethodSwitches(node);
	settings.cost_weights = ReadCostWeights(node);
	settings.step_sizes = ReadStepSizes(node);
	settings.ik_settings = ReadIkSettings(node);
	settings.tour_settings = ReadTourSettings(node);
	return settings;
}

void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& scene)
{
	planning_scene_monitor::LockedPlanningSceneRW locked_scene(scene);
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = locked_scene->getRobotModel()->getRootLinkName();
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {mounting_surface_size, mounting_surface_size, mounting_surface_thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * mounting_surface_thickness;
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	locked_scene->processCollisionObjectMsg(surface);
	// The base rests on it: allow every link fixed to the root.
	const moveit::core::RobotModelConstPtr& model = locked_scene->getRobotModel();
	for (const moveit::core::LinkModel* link : model->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == model->getRootLink())
			locked_scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

visualization_msgs::msg::Marker GetMountingSurfaceMarker(const std::string& base_link, const rclcpp::Time& stamp)
{
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = base_link;
	marker.header.stamp = stamp;
	marker.ns = "mounting_surface";
	marker.type = visualization_msgs::msg::Marker::CUBE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.z = -0.5 * mounting_surface_thickness;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = mounting_surface_size;
	marker.scale.y = mounting_surface_size;
	marker.scale.z = mounting_surface_thickness;
	marker.color = marker_utils::Rgba(0.55f, 0.45f, 0.35f, 0.6f);
	return marker;
}

}  // namespace object_gtsp_co_optimize
