// Object placement in one descent. Each iteration: a reach step moves the object to reach more viewpoints (room-then-reach
// when it loses reached ones); then, for the tour (length + reconfigurations, joints kept off their limits), IK is re-solved
// with the object held still (self-motion slides, GTSP reorder, posture swaps) and the object is moved. Each pose follows
// the object by IK from its last pose, so it keeps its posture. Repeats until nothing changes.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <random_numbers/random_numbers.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "object_placement_planning/closest_ik.hpp"
#include "object_placement_planning/gtsp.hpp"
#include "object_placement_planning/joint_travel.hpp"
#include "object_placement_planning/manipulability.hpp"
#include "object_placement_planning/marker_utils.hpp"
#include "object_placement_planning/object_placement.hpp"
#include "object_placement_planning/real_cost_planning.hpp"
#include "object_placement_planning/room_margins.hpp"
#include "object_placement_planning/scene_geometry.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

namespace
{

using Joints = std::vector<double>;

// The surface the robot stands on (top face at the base plane), as in object_placement, so all methods see the same world.
void AddMountingSurface(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
	const double size = 4.0, thickness = 0.05;	// m
	planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
	const std::string base_link = scene->getRobotModel()->getRootLinkName();
	moveit_msgs::msg::CollisionObject surface;
	surface.header.frame_id = base_link;
	surface.id = "mounting_surface";
	shape_msgs::msg::SolidPrimitive box;
	box.type = box.BOX;
	box.dimensions = {size, size, thickness};
	geometry_msgs::msg::Pose pose;
	pose.position.z = -0.5 * thickness;
	pose.orientation.w = 1.0;
	surface.primitives.push_back(box);
	surface.primitive_poses.push_back(pose);
	surface.operation = surface.ADD;
	scene->processCollisionObjectMsg(surface);
	// The base rests on it: allow every link fixed to the root.
	for (const moveit::core::LinkModel* link : scene->getRobotModel()->getLinkModels())
		if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(link) == scene->getRobotModel()->getRootLink())
			scene->getAllowedCollisionMatrixNonConst().setEntry(surface.id, link->getName(), true);
}

// RViz marker for the mounting surface, as object_placement draws it.
visualization_msgs::msg::Marker GetMountingSurfaceMarker(const std::string& base_link, const rclcpp::Time& stamp)
{
	const double size = 4.0, thickness = 0.05;	// m, as AddMountingSurface
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = base_link;
	marker.header.stamp = stamp;
	marker.ns = "mounting_surface";
	marker.type = visualization_msgs::msg::Marker::CUBE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.z = -0.5 * thickness;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = thickness;
	marker.color = marker_utils::Rgba(0.55f, 0.45f, 0.35f, 0.6f);
	return marker;
}

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
	std::vector<bool> reached;
	std::vector<Eigen::Vector3d> position_errors;  // per viewpoint, tool to target; zero if reached
	std::vector<double> gaps;  // per viewpoint, m: position and angle (at the closest IK's rot_scale) + clearance shortfall; zero if reached
	std::vector<double> clearance_shortfalls;  // per viewpoint, m: the part of its gap from arm clearance
	std::vector<double> angle_errors;  // per viewpoint, rad, zero if reached
	std::vector<std::vector<MissCandidate>> miss_candidates;  // per missed viewpoint: closest poses GTSP may choose from
	int num_reached = 0;
	double gap_sum = 0.0;  // m, over the missed viewpoints
};

// Downhill move of one step: the object's direction and each pose's joint slide along its self-motion.
struct DescentDirection
{
	Eigen::Vector3d object_direction = Eigen::Vector3d::Zero();  // unit, or zero
	std::vector<Eigen::VectorXd> slides;  // per viewpoint, largest norm 1 over all poses; empty vector = no slide
	bool has_slide = false;
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

}  // namespace

class ObjectGtspCoOptimizeNode : public rclcpp::Node
{
public:
	ObjectGtspCoOptimizeNode()
		: Node("object_gtsp_co_optimize", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
	}

	void Run()
	{
		const std::string group_name = this->GetParameter<std::string>("group_name", "panda_arm");
		std::string mesh_path = this->GetParameter<std::string>("mesh_path", "");
		const double mesh_scale = this->GetParameter<double>("mesh_scale", 0.01);
		const std::string tour_input_dir = this->GetParameter<std::string>("tour_input_dir", "/tmp/viewpoint_planner_output");
		const std::string output_dir = this->GetParameter<std::string>("output_dir", "/tmp/object_gtsp_co_optimize_output");
		this->home = this->GetParameter<std::vector<double>>("initial_joints", std::vector<double>{0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785});
		const Eigen::Vector3d nominal = ToVector3(this->GetParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), Eigen::Vector3d(0.5, 0.0, 0.15));
		this->rotation = RotationFromRpyDeg(this->GetParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		// object_placement's parameter names, so one params file serves all methods.
		this->lower_bound = Eigen::Vector3d(this->GetParameter<double>("x_min", -0.6), this->GetParameter<double>("y_min", -0.6), this->GetParameter<double>("z_min", -0.2));
		this->upper_bound = Eigen::Vector3d(this->GetParameter<double>("x_max", 0.6), this->GetParameter<double>("y_max", 0.6), this->GetParameter<double>("z_max", 0.2));
		const Eigen::Vector3d start(this->GetParameter<double>("initial_x", 0.5), this->GetParameter<double>("initial_y", 0.0), this->GetParameter<double>("initial_z", 0.15));
		closest_ik::Options ik_options;
		this->seed = static_cast<int>(this->GetParameter<int64_t>("random_seed", 42));
		ik_options.iters = static_cast<int>(this->GetParameter<int64_t>("closest_ik_iters", 60));
		ik_options.margin = this->GetParameter<double>("closest_ik_margin", 0.02);
		ik_options.rot_scale = this->GetParameter<double>("rot_metric_scale", 0.1);
		ik_options.limit_sharpness = this->GetParameter<double>("manipulability_limit_sharpness", 20.0);
		ik_options.ignore_roll = this->GetParameter<bool>("ignore_camera_roll", false);
		ik_options.exact_margin = this->GetParameter<double>("exact_ik_margin", 0.0);
		this->exact_options = ik_options;
		this->exact_options.exact_only = true;
		this->closest_options = ik_options;
		this->closest_options.try_exact_first = false;
		this->jacobian_damping = this->GetParameter<double>("jacobian_damping", 1e-3);
		this->edge_weights.joint_distance = this->GetParameter<double>("joint_distance_weight", 1.0);
		this->edge_weights.max_turn = this->GetParameter<double>("max_joint_deviation_weight", 1.0);
		this->edge_weights.time = this->GetParameter<double>("travel_time_weight", 0.0);
		this->two_opt_rounds = static_cast<int>(this->GetParameter<int64_t>("gtsp_two_opt_rounds", 5));
		const double reconfig_threshold = this->GetParameter<double>("reconfig_threshold", 1.0);
		const double planning_time = this->GetParameter<double>("real_cost_planning_time", 0.5);
		const int planning_attempts = static_cast<int>(this->GetParameter<int64_t>("real_cost_attempts", 3));
		const double unreachable_penalty = this->GetParameter<double>("unreachable_penalty", 50.0);
		// The co-optimization's own tunable parameters.
		const double time_budget = this->GetParameter<double>("co_time_budget_sec", 300.0);
		this->max_steps = static_cast<int>(this->GetParameter<int64_t>("co_max_steps", 200));  // iterations of the descent
		this->max_line_search_tries = static_cast<int>(this->GetParameter<int64_t>("co_max_line_search_tries", 4));  // tries per sub-step
		this->ik_attempts = static_cast<int>(this->GetParameter<int64_t>("co_ik_attempts", 10));  // random start poses when a missed viewpoint's own pose fails
		this->closest_gap_slack = this->GetParameter<double>("co_closest_gap_slack", 0.005);  // m: a miss's closest poses this near its best gap stay candidates
		this->closest_ik_extra_starts = static_cast<int>(this->GetParameter<int64_t>("co_closest_ik_extra_starts", 3));  // random starts also tried for a miss's closest pose
		this->initial_step = this->GetParameter<double>("co_initial_step", 0.02);	 // m
		this->min_step = this->GetParameter<double>("co_min_step", 0.001);	// m: below this the descent has settled at this penalty
		this->max_step = this->GetParameter<double>("co_max_step", 0.05);	 // m
		this->initial_penalty = this->GetParameter<double>("co_initial_penalty", 100.0);	 // cost per m of missed gap
		this->penalty_growth = this->GetParameter<double>("co_penalty_growth", 10.0);
		this->max_penalty = this->GetParameter<double>("co_max_penalty", 1e5);
		this->use_neighbor_ik = this->GetParameter<bool>("co_neighbor_ik", true);	// reorders also offer IK from each pose's tour neighbours
		this->use_slide = this->GetParameter<bool>("co_slide", true);	// each step also slides reached poses along their self-motion
		this->initial_slide_step = this->GetParameter<double>("co_initial_slide_step", 0.3);	// rad, largest pose slide at co_initial_step
		this->slide_joint_margin = this->GetParameter<double>("co_slide_joint_margin", 0.05);	// rad a slide stays from each (hard) joint limit
		this->reconfig_threshold = reconfig_threshold;
		this->use_posture_match = this->GetParameter<bool>("co_posture_match", true);	// reconfiguration edges pull the object and swap poses
		this->reconfiguration_weight = this->GetParameter<double>("co_reconfiguration_weight", 10.0);	// cost per m of posture gap
		this->blocked_edge_charge = this->GetParameter<double>("co_blocked_edge_charge", 4.0);	// x edge length extra for a blocked edge with no free detour through home; 0 = off
		this->gtsp_blocked_rounds = static_cast<int>(this->GetParameter<int64_t>("co_gtsp_blocked_rounds", 2));	// GTSP re-solves charging blocked edges
		this->miss_cost = this->GetParameter<double>("co_miss_cost", 10.0);	// cost per missed viewpoint (a step may lose one if it gains more)
		this->reconfiguration_cost = this->GetParameter<double>("co_reconfiguration_cost", 2.0);	// cost per reconfiguration
		this->joint_room_band = this->GetParameter<double>("co_joint_room_band", 0.2);	// rad beyond the hard limit a joint should keep
		this->joint_room_weight = this->GetParameter<double>("co_joint_room_weight", 50.0);	// cost per rad^2 of room shortfall
		this->use_room = this->GetParameter<bool>("co_room", true);
		// Ablation switches: no object move for the tour (reach still moves it); object frozen once every viewpoint is reached.
		this->tour_object_move = this->GetParameter<bool>("co_tour_object_move", true);
		this->freeze_object_when_reached = this->GetParameter<bool>("co_freeze_object_when_reached", false);	// a blocked missed viewpoint also pulls the object to free what blocks it
		this->progress_delay = this->GetParameter<double>("visualize_progress_delay_sec", 0.0);  // s pause after each progress publish
		const bool keep_alive = this->GetParameter<bool>("keep_alive", false);  // keep republishing the final markers until Ctrl-C (for RViz)

		if (mesh_path.empty())
			mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs.stl";
		this->viewpoints = LoadTourTcpPoses(tour_input_dir);	// relative to the object position below
		const std::vector<Eigen::Isometry3d> viewpoints_nominal = this->viewpoints;	// abs, object at nominal
		for (Eigen::Isometry3d& viewpoint : this->viewpoints)
			viewpoint.translation() -= nominal;
		const int num_viewpoint = static_cast<int>(this->viewpoints.size());

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		this->model = loader.getModel();
		this->state = std::make_unique<moveit::core::RobotState>(this->model);
		this->state->setToDefaultValues();
		this->group = this->state->getJointModelGroup(group_name);
		this->tool = this->model->getLinkModel("tool0");
		// The camera side, as exact IK's clearance check: links rigid with the tool and below, the wrist link itself aside.
		this->camera_side_links = closest_ik::ToolSideLinks(this->tool);
		this->camera_side_links.erase(moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(this->tool)->getName());
		SetJointSpeedLimits(this->model, group_name, this->edge_weights);
		this->travel_joints = joint_travel::AllJoints(static_cast<int>(this->home.size()) - 1);  // the last joint only spins the camera
		this->scene = BuildLocalCollisionScene(this->shared_from_this(), mesh_path, mesh_scale, nominal, this->rotation);
		AddMountingSurface(this->scene);
		this->mesh_path = mesh_path;
		this->mesh_scale = mesh_scale;
		this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_markers", rclcpp::QoS(1).transient_local());
		this->progress_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_placement_progress_markers", rclcpp::QoS(1).transient_local());
		visualization_msgs::msg::MarkerArray surface_markers;
		surface_markers.markers.push_back(GetMountingSurfaceMarker(this->model->getRootLinkName(), this->now()));
		this->marker_pub->publish(surface_markers);
		this->num_threads = std::max(1u, std::min(std::thread::hardware_concurrency(), static_cast<unsigned int>(num_viewpoint)));
		RCLCPP_INFO(this->get_logger(), "Loaded %d tour poses; co-optimizing in box (%.2f..%.2f, %.2f..%.2f, %.2f..%.2f) m, budget %.0f s", num_viewpoint, this->lower_bound.x(), this->upper_bound.x(), this->lower_bound.y(), this->upper_bound.y(), this->lower_bound.z(), this->upper_bound.z(), time_budget);

		this->start_time = std::chrono::steady_clock::now();
		this->time_budget_end = time_budget;
		// One descent from the given start, no restarts.
		const Descent best = this->Descend(this->ClampToBox(start));
		const double search_seconds = this->GetElapsedSeconds();
		const bool all_reached = best.placement.num_reached == num_viewpoint;

		// Planned tour length and, for reference, execution time: straight moves where collision-free, else OMPL (camera spin
		// aside in the length; timed with TOTG for the time).
		std::vector<Joints> tour_poses = this->GetTourPoses(best.placement, best.order);
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(best.placement.position, this->rotation));
		const std::vector<double> edge_lengths = PlanTourEdgeLengths(this->shared_from_this(), this->model, this->scene, tour_poses, this->home, group_name, planning_time, planning_attempts, /*count_last_joint=*/false);
		double planned_length = 0.0;
		int num_failed_length_edges = 0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_lengths.size() && edge_lengths[k] >= 0.0;
			num_failed_length_edges += edge_ok ? 0 : 1;
			planned_length += edge_ok ? edge_lengths[k] : joint_travel::JointDistance(k == 0 ? this->home : tour_poses[k - 1], tour_poses[k], this->travel_joints) + unreachable_penalty;
		}
		const std::vector<double> edge_times = PlanTourEdgeTimes(this->shared_from_this(), this->model, this->scene, tour_poses, this->home, group_name, planning_time, planning_attempts);
		double planned_time = 0.0;
		int num_failed_edges = 0;
		for (size_t k = 0; k < tour_poses.size(); ++k)
		{
			const bool edge_ok = k < edge_times.size() && edge_times[k] >= 0.0;
			num_failed_edges += edge_ok ? 0 : 1;
			planned_time += edge_ok ? edge_times[k] : joint_travel::StraightMoveTime(k == 0 ? this->home : tour_poses[k - 1], tour_poses[k], this->edge_weights.max_velocity, this->edge_weights.max_acceleration) + unreachable_penalty;
		}
		std::vector<Joints> poses_from_home = tour_poses;
		poses_from_home.insert(poses_from_home.begin(), this->home);
		const int num_reconfigurations = joint_travel::CountReconfigs(poses_from_home, reconfig_threshold, this->travel_joints);
		const double smallest_margin = this->GetSmallestJointMargin(tour_poses);
		const double seconds = this->GetElapsedSeconds();
		if (all_reached)
			RCLCPP_INFO(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches all %d poses, planned length %.2f rad (%d failed edges), %d reconfigurations, smallest joint margin %.3f rad beyond the hard limit; planned time %.2f s for reference (%.1f s).", best.placement.position.x(), best.placement.position.y(), best.placement.position.z(), num_viewpoint, planned_length, num_failed_length_edges, num_reconfigurations, smallest_margin, planned_time, seconds);
		else
			RCLCPP_WARN(this->get_logger(), "Done. Object abs (%.4f, %.4f, %.4f) m -- reaches only %d/%d poses (%.1f s).", best.placement.position.x(), best.placement.position.y(), best.placement.position.z(), std::max(0, best.placement.num_reached), num_viewpoint, seconds);

		std::filesystem::create_directories(output_dir);
		Json::Value out;
		out["ok"] = all_reached;
		out["num_reachable"] = std::max(0, best.placement.num_reached);
		out["num_total"] = num_viewpoint;
		out["x"] = best.placement.position.x();
		out["y"] = best.placement.position.y();
		out["z"] = best.placement.position.z();
		out["planned_travel"] = all_reached ? planned_length : -1.0;  // rad
		out["planned_time"] = all_reached ? planned_time : -1.0;  // s, reference only
		out["straight_tour_cost"] = best.tour_cost;
		out["failed_edges"] = num_failed_length_edges;
		out["reconfigurations"] = num_reconfigurations;
		out["smallest_joint_margin"] = smallest_margin;
		out["search_seconds"] = search_seconds;
		out["seconds"] = seconds;
		for (int viewpoint : best.order)
			out["tour_order"].append(viewpoint);
		for (const Joints& pose : tour_poses)
		{
			Json::Value joints(Json::arrayValue);
			for (double value : pose)
				joints.append(value);
			out["joint_solutions"].append(joints);
		}
		std::ofstream(output_dir + "/object_gtsp_co_optimize_result.json") << out;

		// Final markers on /object_placement_markers, as object_placement draws them: the object and the tour's viewpoints.
		ObjectPlacementResult result;
		result.ok = all_reached;
		result.x = best.placement.position.x();
		result.y = best.placement.position.y();
		result.z = best.placement.position.z();
		result.tour_order = best.order;
		for (int viewpoint : best.order)
			result.joint_solutions.push_back(best.placement.reached[static_cast<size_t>(viewpoint)] ? best.placement.poses[static_cast<size_t>(viewpoint)] : Joints());
		visualization_msgs::msg::MarkerArray final_markers = BuildObjectPlacementMarkerArray(this->now(), this->mesh_path, this->mesh_scale, nominal, this->rotation, viewpoints_nominal, result);
		final_markers.markers.push_back(GetMountingSurfaceMarker(this->model->getRootLinkName(), this->now()));
		this->marker_pub->publish(final_markers);
		this->PublishProgress(best.placement, best.order, Eigen::Vector3d::Zero());
		while (keep_alive && rclcpp::ok())
		{
			std::this_thread::sleep_for(std::chrono::seconds(2));
			this->marker_pub->publish(final_markers);
		}
	}

private:
	template <typename T>
	T GetParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}

	double GetElapsedSeconds() const
	{
		return std::chrono::duration<double>(std::chrono::steady_clock::now() - this->start_time).count();
	}

	Eigen::Vector3d ClampToBox(const Eigen::Vector3d& position) const
	{
		return position.cwiseMax(this->lower_bound).cwiseMin(this->upper_bound);
	}

	// One descent from `start`. Each iteration: (reach) if viewpoints are missed, move the object to reach more, with
	// room-then-reach when a move loses reached ones; (tour, poses) re-solve IK with the object held still (slides, GTSP
	// reorder, posture swaps) for a lower cost; (tour, object) move the object for a lower cost, which may lose viewpoints
	// if the gain beats their gaps. Repeats until an iteration changes nothing; if viewpoints are still missed then, the reach
	// penalty grows (up to max_penalty) and it goes on.
	Descent Descend(const Eigen::Vector3d& start)
	{
		double penalty = this->initial_penalty;
		Descent current;
		current.placement = this->SolvePoses(start, std::vector<Joints>(this->viewpoints.size(), this->home));
		current.order = this->ReorderTour(current.placement, {});
		this->EvaluateWithBestOrder(current, penalty);
		const int num_viewpoint = static_cast<int>(this->viewpoints.size());
		double reach_step = this->initial_step;
		double tour_step = this->initial_step;
		this->trail.assign(1, start);
		this->PublishProgress(current.placement, current.order, Eigen::Vector3d::Zero());
		this->LogDescent("start", current);
		bool object_frozen = false;	 // freeze_object_when_reached: set once every viewpoint is reached
		for (int iteration = 1; iteration <= this->max_steps && rclcpp::ok() && this->GetElapsedSeconds() < this->time_budget_end; ++iteration)
		{
			if (this->freeze_object_when_reached && !object_frozen && current.placement.num_reached == num_viewpoint)
			{
				object_frozen = true;
				RCLCPP_INFO(this->get_logger(), "iteration %d: all reached; object frozen at abs (%.4f, %.4f, %.4f)", iteration, current.placement.position.x(), current.placement.position.y(), current.placement.position.z());
			}
			bool changed = false;
			if (current.placement.num_reached < num_viewpoint && !object_frozen)
				changed = this->TakeReachStep(current, reach_step, penalty, iteration) || changed;
			changed = this->ImprovePosesInPlace(current, penalty, iteration) || changed;
			if (this->tour_object_move && !object_frozen)
				changed = this->TakeTourStep(current, tour_step, penalty, iteration) || changed;
			if (changed)
				continue;
			if (current.placement.num_reached == num_viewpoint || penalty >= this->max_penalty)
				break;
			penalty = std::min(this->max_penalty, penalty * this->penalty_growth);
			this->UpdateCost(current, penalty);
			reach_step = this->initial_step;
			tour_step = this->initial_step;
			RCLCPP_INFO(this->get_logger(), "iteration %d: nothing changed with %d missed; penalty -> %.0f per m", iteration, num_viewpoint - current.placement.num_reached, penalty);
			this->LogMissedViewpoints(current.placement);
		}
		this->LogDescent("end", current);
		this->LogMissedViewpoints(current.placement);
		return current;
	}

	void LogDescent(const std::string& label, const Descent& descent)
	{
		RCLCPP_INFO(this->get_logger(), "%s: abs (%.4f, %.4f, %.4f) m  reached %d/%zu  gap sum %.4f m  reconfigurations %d (posture gap sum %.3f m)  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f", label.c_str(), descent.placement.position.x(), descent.placement.position.y(), descent.placement.position.z(), descent.placement.num_reached, this->viewpoints.size(), descent.placement.gap_sum, descent.num_reconfigurations, descent.posture_gap_sum, descent.num_blocked_edges, descent.tour_cost, descent.smallest_margin, descent.cost);
	}

	// The object at `position`, IK from `seeds` (then `fallback_seeds`), evaluated like every spot.
	Descent EvaluateSpot(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds, const std::vector<int>& order, double penalty)
	{
		Descent candidate;
		candidate.placement = this->SolvePoses(this->ClampToBox(position), seeds, fallback_seeds);
		candidate.order = order;
		this->EvaluateWithBestOrder(candidate, penalty);
		return candidate;
	}

	// Reach step: along the missed viewpoints' pulls; kept if it reaches more, or as many with smaller gaps. A try that loses
	// reached viewpoints turns to room-then-reach for them: the aim, cut so it lowers none of their margins (joint limits,
	// stretch, collisions), or a pure back-off that most raises them. Up to max_line_search_tries tries; the step halves on
	// a plain failure and grows on success.
	bool TakeReachStep(Descent& current, double& step, double penalty, int iteration)
	{
		const Eigen::Vector3d aim = this->GetReachDirection(current.placement);
		if (aim.norm() < 0.5)
			return false;
		Eigen::Vector3d direction = aim;
		bool tried_miss_room = false;
		for (int attempt = 0; attempt < this->max_line_search_tries && step >= this->min_step; ++attempt)
		{
			const Descent candidate = this->EvaluateSpot(current.placement.position + step * direction, current.placement.poses, {}, current.order, penalty);
			std::vector<size_t> lost;
			for (size_t v = 0; v < this->viewpoints.size(); ++v)
				if (current.placement.reached[v] && !candidate.placement.reached[v])
					lost.push_back(v);
			const bool kept = candidate.placement.num_reached > current.placement.num_reached || (candidate.placement.num_reached == current.placement.num_reached && candidate.placement.gap_sum < current.placement.gap_sum);
			RCLCPP_INFO(this->get_logger(), "iteration %d reach: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d (lost %zu)  gap sum %.4f m vs %.4f%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, lost.size(), candidate.placement.gap_sum, current.placement.gap_sum, kept ? "  <-- kept" : "");
			if (kept)
			{
				current = candidate;
				this->trail.push_back(current.placement.position);
				this->PublishProgress(current.placement, current.order, direction);
				step = std::min(this->max_step, step * 1.5);
				// Misses nearly on target: say why they still miss.
				bool has_tiny_miss = false;
				for (size_t v = 0; v < this->viewpoints.size(); ++v)
					has_tiny_miss = has_tiny_miss || (!current.placement.reached[v] && current.placement.gaps[v] < 1e-3);
				if (has_tiny_miss)
					this->LogMissedViewpoints(current.placement);
				return true;
			}
			if (!lost.empty() && this->use_room)
			{
				const Eigen::Vector3d room_direction = this->GetRoomThenReachDirection(current.placement, lost, aim, step);
				if (room_direction.norm() > 0.5 && (room_direction - direction).norm() > 1e-3)
				{
					direction = room_direction;
					continue;
				}
			}
			// Room-then-reach for the misses: once per step, a move that frees what blocks them (joint limits, stretch,
			// collisions), kept if it raises their tightest margin and loses nothing; the next iteration aims again.
			if (this->use_room && !tried_miss_room)
			{
				tried_miss_room = true;
				if (this->TakeMissRoomStep(current, step, penalty, iteration))
					return true;
			}
			step *= 0.5;
		}
		// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
		step = this->initial_step;
		return false;
	}

	// Tour step, poses: the object held still, IK re-solved from each pose slid along its self-motion (downhill in tour
	// length and joint room, or toward its neighbour across a reconfiguration edge), then GTSP and posture matching; kept if
	// the cost drops. The slide halves on failure.
	bool ImprovePosesInPlace(Descent& current, double penalty, int iteration)
	{
		if (!this->use_slide)
		{
			Descent reordered = current;
			this->EvaluateWithBestOrder(reordered, penalty);
			const bool kept = reordered.cost < current.cost - 1e-9;
			if (kept)
				current = reordered;
			return kept;
		}
		const DescentDirection direction = this->GetTourDirection(current, penalty);
		double slide_step = this->initial_slide_step;
		for (int attempt = 0; attempt < this->max_line_search_tries && direction.has_slide; ++attempt)
		{
			const Descent candidate = this->EvaluateSpot(current.placement.position, this->GetSlidPoses(current.placement, direction, slide_step), current.placement.poses, current.order, penalty);
			const bool kept = candidate.cost < current.cost - 1e-9;
			RCLCPP_INFO(this->get_logger(), "iteration %d poses: slide %.3f rad  reached %d  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, slide_step, candidate.placement.num_reached, candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.tour_cost, candidate.smallest_margin, candidate.cost, current.cost, kept ? "  <-- kept" : "");
			if (kept)
			{
				current = candidate;
				return true;
			}
			slide_step *= 0.5;
		}
		return false;
	}

	// Tour step, object: along the combined tour pulls; kept if the cost drops (it may lose viewpoints if the gain beats
	// their gaps). The step halves on failure and grows on success.
	bool TakeTourStep(Descent& current, double& step, double penalty, int iteration)
	{
		const DescentDirection direction = this->GetTourDirection(current, penalty);
		if (direction.object_direction.norm() < 0.5)
			return false;
		for (int attempt = 0; attempt < this->max_line_search_tries && step >= this->min_step; ++attempt)
		{
			const Descent candidate = this->EvaluateSpot(current.placement.position + step * direction.object_direction, current.placement.poses, {}, current.order, penalty);
			const bool kept = candidate.cost < current.cost - 1e-9;
			RCLCPP_INFO(this->get_logger(), "iteration %d tour: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  gap sum %.4f m  reconfigurations %d  blocked edges %d  tour length %.2f  smallest margin %.3f rad  cost %.2f vs %.2f%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, candidate.placement.gap_sum, candidate.num_reconfigurations, candidate.num_blocked_edges, candidate.tour_cost, candidate.smallest_margin, candidate.cost, current.cost, kept ? "  <-- kept" : "");
			if (kept)
			{
				current = candidate;
				this->trail.push_back(current.placement.position);
				this->PublishProgress(current.placement, current.order, direction.object_direction);
				step = std::min(this->max_step, step * 1.5);
				return true;
			}
			step *= 0.5;
		}
		// Failed at this spot: the next try (after other sub-steps may have changed things) starts from the full step.
		step = this->initial_step;
		return false;
	}

	// Room move for blocked misses (part of their gap the object can't close directly, or on target but blocked): along the
	// move that most raises their blocking margins; kept if it raises the tightest of them, reaches as many and loses none.
	bool TakeMissRoomStep(Descent& current, double step, double penalty, int iteration)
	{
		const double near_floors = 6.0;
		std::vector<size_t> blocked;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			if (current.placement.reached[v])
				continue;
			const double gap = current.placement.gaps[v];
			const double blocked_share = gap < 1e-3 ? 1.0 : 1.0 - current.placement.position_errors[v].norm() / gap;
			if (blocked_share > 0.3)
				blocked.push_back(v);
		}
		if (blocked.empty())
			return false;
		const std::vector<room_margins::Margin> before = this->GetMissMargins(current.placement, blocked, near_floors);
		if (before.empty())
			return false;
		const Eigen::Vector3d room_direction = room_margins::MostRoomDirection(before, step);
		if (room_direction.norm() < 0.5)
			return false;
		const Descent candidate = this->EvaluateSpot(current.placement.position + step * room_direction, current.placement.poses, {}, current.order, penalty);
		bool lost_any = false;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
			lost_any = lost_any || (current.placement.reached[v] && !candidate.placement.reached[v]);
		const std::vector<room_margins::Margin> after = this->GetMissMargins(candidate.placement, blocked, near_floors);
		const double tightest_before = room_margins::Tightest(before);
		const double tightest_after = after.empty() ? std::numeric_limits<double>::infinity() : room_margins::Tightest(after);
		const bool kept = !lost_any && candidate.placement.num_reached >= current.placement.num_reached && tightest_after > tightest_before + 1e-3;
		RCLCPP_INFO(this->get_logger(), "iteration %d reach room: step %.4f m -> abs (%.4f, %.4f, %.4f)  reached %d  blocked misses %zu  tightest margin %.2f -> %.2f floors%s", iteration, step, candidate.placement.position.x(), candidate.placement.position.y(), candidate.placement.position.z(), candidate.placement.num_reached, blocked.size(), tightest_before, tightest_after, kept ? "  <-- kept" : "");
		if (!kept)
			return false;
		current = candidate;
		this->trail.push_back(current.placement.position);
		this->PublishProgress(current.placement, current.order, room_direction);
		return true;
	}

	// Margins of the given missed viewpoints' closest poses at their spot, up to near_floors times each floor.
	std::vector<room_margins::Margin> GetMissMargins(const Placement& placement, const std::vector<size_t>& misses, double near_floors)
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		std::vector<room_margins::Margin> margins;
		for (size_t v : misses)
		{
			const std::vector<room_margins::Margin> mine = this->GetBlockingMargins(v, placement.poses[v], near_floors);
			margins.insert(margins.end(), mine.begin(), mine.end());
		}
		return margins;
	}

	// Room-then-reach for viewpoints `lost` on a reach try: their margins at the current spot (joint limits, stretch,
	// collisions); the aim with every part that lowers one of them removed, or, if little of it is left, the move that most
	// raises them.
	Eigen::Vector3d GetRoomThenReachDirection(const Placement& placement, const std::vector<size_t>& lost, const Eigen::Vector3d& aim, double step)
	{
		const double lost_margin_floors = 6.0;  // margins up to this many floors count as at risk
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		std::vector<room_margins::Margin> margins;
		for (size_t v : lost)
		{
			const std::vector<room_margins::Margin> mine = this->GetBlockingMargins(v, placement.poses[v], lost_margin_floors);
			margins.insert(margins.end(), mine.begin(), mine.end());
		}
		if (margins.empty())
			return Eigen::Vector3d::Zero();
		Eigen::Vector3d direction = aim;
		for (int round = 0; round < 20; ++round)
		{
			bool changed = false;
			for (const room_margins::Margin& margin : margins)
			{
				const double rate = margin.slope.dot(direction);
				if (rate >= -1e-12 || margin.slope.squaredNorm() < 1e-24)
					continue;
				direction -= rate / margin.slope.squaredNorm() * margin.slope;
				changed = true;
			}
			if (!changed)
				break;
		}
		if (direction.norm() < 0.3 * aim.norm())
			return room_margins::MostRoomDirection(margins, step);
		return direction.normalized();
	}

	// Reach first: more viewpoints reached wins, fewer loses; at equal reach the lower total cost (gaps + tour) decides.
	bool IsBetter(const Descent& candidate, const Descent& current) const
	{
		if (candidate.placement.num_reached != current.placement.num_reached)
			return candidate.placement.num_reached > current.placement.num_reached;
		return candidate.cost < current.cost;
	}

	// Tour cost, reconfigurations, joint room and the total: tour cost + miss_cost x missed viewpoints + penalty x their gaps
	// + reconfiguration_cost x reconfigurations + reconfiguration_weight x posture gaps + room weight x room shortfall (reached
	// poses only).
	void UpdateCost(Descent& descent, double penalty)
	{
		// Tour length, with each edge whose straight move collides charged its estimated detour (see GetBlockedEdgeCharges).
		double detour_charge = 0.0;
		const std::vector<std::pair<size_t, double>> blocked = this->GetBlockedEdgeCharges(descent.placement, descent.order);
		for (const std::pair<size_t, double>& edge : blocked)
			detour_charge += edge.second;
		descent.num_blocked_edges = static_cast<int>(blocked.size());
		descent.tour_cost = this->GetTourCost(descent.placement, descent.order) + detour_charge;
		descent.num_reconfigurations = this->CountReconfigurations(descent.placement, descent.order);
		descent.posture_gap_sum = 0.0;
		for (const PostureMatch& match : descent.matches)
			descent.posture_gap_sum += match.gap;
		descent.room_shortfall = 0.0;
		std::vector<Joints> reached_poses;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			// Reached poses only: a missed viewpoint's closest pose presses on limits by design.
			if (!descent.placement.reached[v])
				continue;
			descent.room_shortfall += this->GetRoomShortfall(descent.placement.poses[v], nullptr);
			reached_poses.push_back(descent.placement.poses[v]);
		}
		descent.smallest_margin = this->GetSmallestJointMargin(reached_poses);
		const int num_missed = static_cast<int>(this->viewpoints.size()) - descent.placement.num_reached;
		descent.cost = descent.tour_cost + this->miss_cost * num_missed + penalty * descent.placement.gap_sum + this->reconfiguration_cost * descent.num_reconfigurations + this->reconfiguration_weight * descent.posture_gap_sum + this->GetRoomWeight(penalty) * descent.room_shortfall;
	}

	// Joint room weight: keeps joints off their limits (penalty unused; the weight is fixed).
	double GetRoomWeight(double /*penalty*/) const
	{
		return this->joint_room_weight;
	}

	// Joint room of one pose beyond the hard limit: sum over its joints (camera spin aside) of max(0, room_band - margin)^2,
	// rad^2; `slope` (if given) gets its gradient in the joints.
	double GetRoomShortfall(const Joints& q, Eigen::VectorXd* slope) const
	{
		const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
		if (slope != nullptr)
			*slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(q.size()));
		double shortfall = 0.0;
		for (int j : this->travel_joints)
		{
			const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(j)]->getVariableBounds()[0];
			if (!bounds.position_bounded_)
				continue;
			const double value = q[static_cast<size_t>(j)];
			const double low = value - bounds.min_position_;
			const double high = bounds.max_position_ - value;
			if (low < this->joint_room_band)
			{
				shortfall += (this->joint_room_band - low) * (this->joint_room_band - low);
				if (slope != nullptr)
					(*slope)[j] -= 2.0 * (this->joint_room_band - low);
			}
			if (high < this->joint_room_band)
			{
				shortfall += (this->joint_room_band - high) * (this->joint_room_band - high);
				if (slope != nullptr)
					(*slope)[j] += 2.0 * (this->joint_room_band - high);
			}
		}
		return shortfall;
	}

	// Every evaluated spot, current or candidate, the same way: posture matching in the given order, and a GTSP reorder (with
	// neighbour IK) then posture matching; the better of the two by IsBetter. GTSP orders by tour cost alone, hence both.
	void EvaluateWithBestOrder(Descent& descent, double penalty)
	{
		Descent reordered = descent;
		this->MatchPostures(descent);
		this->UpdateCost(descent, penalty);
		this->ReorderAndMatch(reordered, penalty);
		if (this->IsBetter(reordered, descent))
			descent = reordered;
	}

	void ReorderAndMatch(Descent& descent, double penalty)
	{
		descent.order = this->ReorderTour(descent.placement, descent.order);
		this->MatchPostures(descent);
		this->UpdateCost(descent, penalty);
	}

	bool IsReconfiguration(const Joints& from, const Joints& to) const
	{
		return joint_travel::MaxJointTurn(from, to, this->travel_joints) > this->reconfig_threshold;
	}

	// Reconfigurations along home -> tour.
	int CountReconfigurations(const Placement& placement, const std::vector<int>& order) const
	{
		int count = 0;
		for (size_t k = 0; k < order.size(); ++k)
			count += this->IsReconfiguration(k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])], placement.poses[static_cast<size_t>(order[k])]) ? 1 : 0;
		return count;
	}

	// Reconfigurations on the two edges touching tour position k if its viewpoint had `pose`.
	int CountReconfigurationsAround(const Placement& placement, const std::vector<int>& order, size_t k, const Joints& pose) const
	{
		const Joints& before = k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])];
		int count = this->IsReconfiguration(before, pose) ? 1 : 0;
		if (k + 1 < order.size())
			count += this->IsReconfiguration(pose, placement.poses[static_cast<size_t>(order[k + 1])]) ? 1 : 0;
		return count;
	}

	// For every reconfiguration edge A -> B: B's pose started from A's, and A's started from B's (exact IK first, else the
	// closest pose), each kept only if it no longer reconfigures against that neighbour; the smaller gap wins. Then swaps:
	// an exact match that removes reconfigurations around its viewpoint replaces the pose, as does a missed viewpoint's
	// closer match that adds none. Two rounds (a swap can expose or remove others); the edges left keep their matches.
	void MatchPostures(Descent& descent)
	{
		descent.matches.clear();
		if (!this->use_posture_match)
			return;
		Placement& placement = descent.placement;
		const std::vector<int>& order = descent.order;
		std::vector<size_t> position_of(this->viewpoints.size(), 0);
		for (size_t k = 0; k < order.size(); ++k)
			position_of[static_cast<size_t>(order[k])] = k;
		for (int round = 0; round < 2; ++round)
		{
			descent.matches = this->FindPostureMatches(placement, order);
			bool swapped = false;
			for (const PostureMatch& match : descent.matches)
			{
				const size_t k = position_of[match.viewpoint];
				const int before = this->CountReconfigurationsAround(placement, order, k, placement.poses[match.viewpoint]);
				const int after = this->CountReconfigurationsAround(placement, order, k, match.pose);
				const bool removes_reconfiguration = match.exact && after < before;
				const bool closer_miss = !placement.reached[match.viewpoint] && !match.exact && match.gap < placement.gaps[match.viewpoint] && after <= before;
				if (!removes_reconfiguration && !closer_miss)
					continue;
				placement.poses[match.viewpoint] = match.pose;
				if (match.exact && !placement.reached[match.viewpoint])
				{
					placement.reached[match.viewpoint] = true;
					placement.num_reached += 1;
				}
				const double new_gap = match.exact ? 0.0 : match.gap;
				placement.gap_sum += new_gap - placement.gaps[match.viewpoint];
				placement.gaps[match.viewpoint] = new_gap;
				placement.clearance_shortfalls[match.viewpoint] = match.clearance_shortfall;
				placement.angle_errors[match.viewpoint] = match.angle_error;
				placement.position_errors[match.viewpoint] = match.exact ? Eigen::Vector3d::Zero() : match.position_error;
				swapped = true;
			}
			if (!swapped)
				return;
		}
		descent.matches = this->FindPostureMatches(placement, order);
	}

	// The posture match of every reconfiguration edge, in parallel (see MatchPostures).
	std::vector<PostureMatch> FindPostureMatches(const Placement& placement, const std::vector<int>& order)
	{
		std::vector<size_t> edges;  // tour position k of each reconfiguration edge order[k-1] (or home) -> order[k]
		for (size_t k = 0; k < order.size(); ++k)
			if (this->IsReconfiguration(k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])], placement.poses[static_cast<size_t>(order[k])]))
				edges.push_back(k);
		std::vector<PostureMatch> matches(edges.size());
		std::vector<bool> found(edges.size(), false);
		if (edges.empty())
			return {};
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(edges.size()));
		std::vector<std::thread> threads;
		for (unsigned int t = 0; t < num_workers; ++t)
			threads.emplace_back([&, t]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t e = t; e < edges.size(); e += num_workers)
				{
					const size_t k = edges[e];
					const size_t to = static_cast<size_t>(order[k]);
					const Joints& from_pose = k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])];
					// (viewpoint, neighbour pose): B in A's posture, and A in B's (home has no viewpoint to move).
					std::vector<std::pair<size_t, const Joints*>> options = {{to, &from_pose}};
					if (k > 0)
						options.push_back({static_cast<size_t>(order[k - 1]), &placement.poses[to]});
					for (const std::pair<size_t, const Joints*>& option : options)
					{
						const Eigen::Isometry3d target = scene_geometry::Translation(placement.position) * this->viewpoints[option.first];
						closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *option.second, this->exact_options);
						if (!result.exact)
							result = this->SolveClosestWithRoom(thread_state, target, {*option.second});
						if (!result.found || this->IsReconfiguration(*option.second, result.q))
							continue;
						const double clearance_shortfall = result.exact ? 0.0 : this->GetClearanceShortfall(thread_state, result.q);
						const double gap = result.exact ? 0.0 : scene_geometry::PoseGap(result.pose_error, this->closest_options.rot_scale) + clearance_shortfall;
						if (found[e] && gap >= matches[e].gap)
							continue;
						found[e] = true;
						matches[e].viewpoint = option.first;
						matches[e].neighbor_pose = *option.second;
						matches[e].pose = result.q;
						matches[e].position_error = result.exact ? Eigen::Vector3d::Zero() : Eigen::Vector3d(result.pose_error.head<3>());
						matches[e].gap = gap;
						matches[e].clearance_shortfall = clearance_shortfall;
						matches[e].angle_error = result.exact ? 0.0 : result.pose_error.tail<3>().norm();
						matches[e].exact = result.exact;
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
		std::vector<PostureMatch> kept;
		for (size_t e = 0; e < edges.size(); ++e)
			if (found[e])
				kept.push_back(matches[e]);
		return kept;
	}

	// Smallest singular value of the tool Jacobian: 0 when fully stretched or folded.
	double GetStretch(const Joints& q)
	{
		return Eigen::JacobiSVD<Eigen::MatrixXd>(manipulability::ToolJacobian(*this->state, this->group, this->tool, q)).singularValues().minCoeff();
	}

	// What blocks a pose, as room_margins does it: joints near a limit, stretch, and close collision pairs, each with its
	// slope w.r.t. the object position (the pose following its viewpoint: dq/dp = J+ [I; 0]). Only margins under
	// `near_floors` times their floor count.
	std::vector<room_margins::Margin> GetBlockingMargins(size_t v, const Joints& q, double near_floors = 2.0)
	{
		const double joint_floor = 0.05;  // rad
		const double stretch_floor = 0.02;
		const double collision_floor = 0.01;  // m
		const double window = this->closest_options.margin + near_floors * collision_floor;  // m, collision pairs looked at
		const int dof = static_cast<int>(q.size());
		const Eigen::MatrixXd dq_dp = scene_geometry::DampedPinv(manipulability::ToolJacobian(*this->state, this->group, this->tool, q), this->jacobian_damping).leftCols<3>();
		const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
		std::vector<room_margins::Margin> margins;
		for (int i = 0; i < dof; ++i)
		{
			const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(i)]->getVariableBounds()[0];
			if (!bounds.position_bounded_)
				continue;
			const Eigen::VectorXd unit = Eigen::VectorXd::Unit(dof, i);
			const std::string joint_name = joints[static_cast<size_t>(i)]->getName();
			const double low = q[static_cast<size_t>(i)] - bounds.min_position_;
			const double high = bounds.max_position_ - q[static_cast<size_t>(i)];
			if (low < near_floors * joint_floor)
				margins.push_back({static_cast<int>(v), joint_name + " low limit", low, joint_floor, dq_dp.transpose() * unit, unit});
			if (high < near_floors * joint_floor)
				margins.push_back({static_cast<int>(v), joint_name + " high limit", high, joint_floor, dq_dp.transpose() * -unit, -unit});
		}
		const double stretch = this->GetStretch(q);
		if (stretch < near_floors * stretch_floor)
		{
			const double delta = 1e-5;  // rad
			Eigen::VectorXd stretch_slope(dof);
			for (int i = 0; i < dof; ++i)
			{
				Joints shifted = q;
				shifted[static_cast<size_t>(i)] += delta;
				stretch_slope(i) = (this->GetStretch(shifted) - stretch) / delta;
			}
			margins.push_back({static_cast<int>(v), "stretch", stretch, stretch_floor, dq_dp.transpose() * stretch_slope, stretch_slope});
		}
		this->state->setJointGroupPositions(this->group, q);
		this->state->update();
		const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, window), window);
		for (size_t k = 0; k < pairs.dists.size(); ++k)
		{
			const Eigen::VectorXd joint_slope = pairs.gradients[k].transpose();
			margins.push_back({static_cast<int>(v), "collision " + pairs.names[k], pairs.dists[k], collision_floor, dq_dp.transpose() * joint_slope + pairs.object_slopes[k], joint_slope});
		}
		return margins;
	}

	// Unit object move that most raises what blocks missed viewpoint v's closest pose (zero if nothing blocks it).
	Eigen::Vector3d GetRoomDirection(const Placement& placement, size_t v)
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		const std::vector<room_margins::Margin> margins = this->GetBlockingMargins(v, placement.poses[v]);
		if (margins.empty())
			return Eigen::Vector3d::Zero();
		return room_margins::MostRoomDirection(margins, this->initial_step);
	}

	// Per missed viewpoint: its gap and what its closest pose is up against (joints near a limit, close collision pairs).
	void LogMissedViewpoints(const Placement& placement)
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
		const double limit_band = 0.02;  // rad: a joint this close to its (hard) limit counts as at it
		const double contact_band = this->closest_options.margin + 0.002;  // m
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			if (placement.reached[v])
				continue;
			std::string blockers;
			for (size_t j = 0; j < joints.size() && j < placement.poses[v].size(); ++j)
			{
				const moveit::core::VariableBounds& bounds = joints[j]->getVariableBounds()[0];
				const double value = placement.poses[v][j];
				if (bounds.position_bounded_ && (value - bounds.min_position_ < limit_band || bounds.max_position_ - value < limit_band))
					blockers += " " + joints[j]->getName() + " at limit;";
			}
			this->state->setJointGroupPositions(this->group, placement.poses[v]);
			this->state->update();
			const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, *this->state, this->group, closest_ik::CloseRequest(*this->state, this->group, contact_band), contact_band);
			for (size_t k = 0; k < pairs.names.size(); ++k)
				blockers += " " + pairs.names[k] + " " + std::to_string(static_cast<int>(std::round(pairs.dists[k] * 1000.0))) + " mm;";
			const bool valid = scene_geometry::IsStateValid(this->scene, this->state.get(), this->group, placement.poses[v].data());
			RCLCPP_INFO(this->get_logger(), "  missed viewpoint %zu: gap %.2f mm (position %.2f mm, angle %.3f deg, clearance short %.2f mm, closest pose %s), blocked by:%s", v, 1000.0 * placement.gaps[v], 1000.0 * placement.position_errors[v].norm(), placement.angle_errors[v] * 180.0 / M_PI, 1000.0 * placement.clearance_shortfalls[v], valid ? "valid" : "INVALID", blockers.empty() ? " nothing (reach)" : blockers.c_str());
		}
	}

	// Closest IK from each start, each finished for clearance (FinishClearance) when its arm sits closer than the exact poses'
	// clearance. Returns the found poses in start order; an exact one (every exact check passed) is flagged exact.
	std::vector<closest_ik::Result> SolveClosestCandidates(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts)
	{
		std::vector<closest_ik::Result> results;
		for (const Joints& start : starts)
		{
			closest_ik::Result result = closest_ik::Solve(this->scene, state, this->group, this->tool, target, start, this->closest_options);
			if (!result.found)
				continue;
			if (this->GetClearanceShortfall(state, result.q) > 0.0)
			{
				// The closest stage calls a pose on target exact without the exact poses' arm clearance check.
				result.exact = false;
				result = this->FinishClearance(state, target, result);
			}
			results.push_back(result);
			if (result.exact)
				break;
		}
		return results;
	}

	// The closest poses within closest_gap_slack of the best gap; of those, the ones with room (no clearance shortfall, every
	// joint at least joint_room_band inside the hard limit) if any. Start order kept, so the one from the viewpoint's own pose
	// (if kept) comes first and is the default; GTSP may pick another by tour.
	std::vector<MissCandidate> KeepNearAndRoomy(moveit::core::RobotState& state, const std::vector<closest_ik::Result>& closest)
	{
		std::vector<MissCandidate> all;
		std::vector<bool> roomy;
		double best_gap = std::numeric_limits<double>::max();
		for (const closest_ik::Result& result : closest)
		{
			MissCandidate candidate;
			candidate.q = result.q;
			candidate.position_error = result.pose_error.head<3>();
			candidate.angle_error = result.pose_error.tail<3>().norm();
			candidate.clearance_shortfall = this->GetClearanceShortfall(state, result.q);
			candidate.gap = scene_geometry::PoseGap(result.pose_error, this->closest_options.rot_scale) + candidate.clearance_shortfall;
			best_gap = std::min(best_gap, candidate.gap);
			roomy.push_back(candidate.clearance_shortfall <= 0.0 && this->GetSmallestJointMargin({result.q}) >= this->joint_room_band);
			all.push_back(candidate);
		}
		std::vector<MissCandidate> near, near_roomy;
		for (size_t k = 0; k < all.size(); ++k)
		{
			if (all[k].gap > best_gap + this->closest_gap_slack)
				continue;
			near.push_back(all[k]);
			if (roomy[k])
				near_roomy.push_back(all[k]);
		}
		return near_roomy.empty() ? near : near_roomy;
	}

	// The single closest pose for a posture match: exact if any start gives one, else the smallest gap.
	closest_ik::Result SolveClosestWithRoom(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const std::vector<Joints>& starts)
	{
		closest_ik::Result best;
		double best_gap = std::numeric_limits<double>::max();
		for (const closest_ik::Result& result : this->SolveClosestCandidates(state, target, starts))
		{
			if (result.exact)
				return result;
			const double gap = scene_geometry::PoseGap(result.pose_error, this->closest_options.rot_scale) + this->GetClearanceShortfall(state, result.q);
			if (gap < best_gap)
			{
				best = result;
				best_gap = gap;
			}
		}
		return best;
	}

	// A closest pose whose arm is within the exact poses' clearance: keep the tool on target (first priority) and use the
	// arm's self-motion to push the too-close arm pairs out past the clearance, then try exact IK from there. Returns the
	// exact result, or the finished pose if its gap (with shortfall) is smaller, else `closest` unchanged. Needs spare
	// freedom: a 6-joint arm on a full 6D target can't move without moving the tool.
	closest_ik::Result FinishClearance(moveit::core::RobotState& state, const Eigen::Isometry3d& target, const closest_ik::Result& closest)
	{
		const double margin = this->exact_options.exact_margin;
		if (margin <= 0.0)
			return closest;
		const double goal_clearance = margin + 0.002;  // m: a little past the exact poses' clearance
		const double max_joint_step = 0.05;	 // rad per iteration
		const int max_iterations = 40;
		Joints q = closest.q;
		for (int iteration = 0; iteration < max_iterations; ++iteration)
		{
			state.setJointGroupPositions(this->group, q);
			state.update();
			const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, goal_clearance), goal_clearance);
			Eigen::VectorXd push = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(q.size()));
			for (size_t k = 0; k < pairs.names.size(); ++k)
			{
				const size_t split = pairs.names[k].find(" - ");
				if (this->camera_side_links.count(pairs.names[k].substr(0, split)) || this->camera_side_links.count(pairs.names[k].substr(split + 3)))
					continue;
				const Eigen::VectorXd gradient = pairs.gradients[k].transpose();
				push += (goal_clearance - pairs.dists[k]) * gradient / (gradient.squaredNorm() + 1e-9);
			}
			if (push.norm() < 1e-9)
				break;
			const scene_geometry::Vector6d pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->exact_options.ignore_roll);
			Eigen::MatrixXd jacobian = manipulability::ToolJacobian(state, this->group, this->tool, q);
			if (this->exact_options.ignore_roll)
			{
				const Eigen::Vector3d axis = state.getGlobalLinkTransform(this->tool).linear().col(2);
				jacobian.bottomRows<3>() = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * jacobian.bottomRows<3>();
			}
			const Eigen::MatrixXd pseudo_inverse = scene_geometry::DampedPinv(jacobian, this->jacobian_damping);
			const Eigen::MatrixXd self_motion = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) - pseudo_inverse * jacobian;
			Eigen::VectorXd step = pseudo_inverse * pose_error + self_motion * push;
			if (step.cwiseAbs().maxCoeff() > max_joint_step)
				step *= max_joint_step / step.cwiseAbs().maxCoeff();
			for (size_t j = 0; j < q.size(); ++j)
				q[j] += step[static_cast<Eigen::Index>(j)];
			state.setJointGroupPositions(this->group, q);
			state.enforceBounds(this->group);
			state.copyJointGroupPositions(this->group, q);
		}
		const closest_ik::Result exact = closest_ik::Solve(this->scene, state, this->group, this->tool, target, q, this->exact_options);
		if (exact.exact)
			return exact;
		if (!scene_geometry::IsStateValid(this->scene, &state, this->group, q.data()))
			return closest;
		state.setJointGroupPositions(this->group, q);
		state.update();
		closest_ik::Result finished;
		finished.found = true;
		finished.exact = false;
		finished.pose_error = closest_ik::GetPoseError(target, state.getGlobalLinkTransform(this->tool), this->exact_options.ignore_roll);
		finished.gap = scene_geometry::PoseGap(finished.pose_error, this->closest_options.rot_scale);
		finished.q = q;
		const double finished_gap = finished.gap + this->GetClearanceShortfall(state, q);
		const double closest_gap = scene_geometry::PoseGap(closest.pose_error, this->closest_options.rot_scale) + this->GetClearanceShortfall(state, closest.q);
		return finished_gap < closest_gap ? finished : closest;
	}

	// Smallest distance from an arm link (camera side aside) to anything, m, looked for up to `window` (returned if none).
	double GetArmClearance(moveit::core::RobotState& state, const Joints& q, double window) const
	{
		state.setJointGroupPositions(this->group, q);
		state.update();
		const closest_ik::ClosePairs pairs = closest_ik::FindClosePairs(this->scene, state, this->group, closest_ik::CloseRequest(state, this->group, window), window);
		double clearance = window;
		for (size_t k = 0; k < pairs.names.size(); ++k)
		{
			const size_t split = pairs.names[k].find(" - ");
			if (this->camera_side_links.count(pairs.names[k].substr(0, split)) || this->camera_side_links.count(pairs.names[k].substr(split + 3)))
				continue;
			clearance = std::min(clearance, pairs.dists[k]);
		}
		return clearance;
	}

	// How far a pose's arm clearance falls short of the exact poses' (exact_ik_margin), m, with the same arm-only check
	// (camera side left out). Added to a missed viewpoint's gap, so a miss blocked only by clearance isn't free to lose.
	double GetClearanceShortfall(moveit::core::RobotState& state, const Joints& q) const
	{
		const double margin = this->exact_options.exact_margin;
		if (margin <= 0.0)
			return 0.0;
		return std::max(0.0, margin - this->GetArmClearance(state, q, margin));
	}

	// Seed of viewpoint v's random IK starts at `position`, from random_seed, v and the position on a 0.1 mm grid.
	uint32_t GetSpotSeed(const Eigen::Vector3d& position, size_t v) const
	{
		const Eigen::Vector3d cell = (position * 1e4).array().round();
		uint64_t hash = static_cast<uint64_t>(this->seed) * 1000003ULL + static_cast<uint64_t>(v) * 7919ULL;
		for (int axis = 0; axis < 3; ++axis)
			hash = hash * 6364136223846793005ULL + static_cast<uint64_t>(static_cast<int64_t>(cell[axis]));
		return static_cast<uint32_t>(hash ^ (hash >> 32));
	}

	// Every viewpoint's pose at `position`, in parallel: exact IK from seeds[v], then from fallback_seeds[v] (its unslid pose,
	// if given), then from random starts; if none is exact, the closest pose from seeds[v].
	Placement SolvePoses(const Eigen::Vector3d& position, const std::vector<Joints>& seeds, const std::vector<Joints>& fallback_seeds = {})
	{
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(position, this->rotation));
		Placement placement;
		placement.position = position;
		placement.poses = seeds;
		placement.reached.assign(this->viewpoints.size(), false);
		placement.position_errors.assign(this->viewpoints.size(), Eigen::Vector3d::Zero());
		placement.clearance_shortfalls.assign(this->viewpoints.size(), 0.0);
		placement.angle_errors.assign(this->viewpoints.size(), 0.0);
		placement.miss_candidates.assign(this->viewpoints.size(), {});
		std::vector<double> gaps(this->viewpoints.size(), 0.0);
		std::vector<std::thread> threads;
		for (unsigned int t = 0; t < this->num_threads; ++t)
			threads.emplace_back([&, t]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t v = t; v < this->viewpoints.size(); v += this->num_threads)
				{
					// Random starts seeded by viewpoint and spot (0.1 mm grid): the same spot always gives the same poses.
					random_numbers::RandomNumberGenerator viewpoint_rng(this->GetSpotSeed(position, v));
					const Eigen::Isometry3d target = scene_geometry::Translation(position) * this->viewpoints[v];
					closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, seeds[v], this->exact_options);
					if (!result.exact && v < fallback_seeds.size())
						result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, fallback_seeds[v], this->exact_options);
					std::vector<Joints> random_starts;
					for (int attempt = 0; attempt < this->ik_attempts && !result.exact; ++attempt)
					{
						Joints random_start;
						thread_state.setToRandomPositions(this->group, viewpoint_rng);
						thread_state.copyJointGroupPositions(this->group, random_start);
						random_starts.push_back(random_start);
						result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, random_start, this->exact_options);
					}
					if (!result.exact)
					{
						// Closest poses from its own pose and a few of the random starts (other postures). Those near the best gap
						// and with room are kept for GTSP to choose by tour; the one from its own pose is the default.
						std::vector<Joints> closest_starts = {seeds[v]};
						for (size_t k = 0; k < random_starts.size() && static_cast<int>(closest_starts.size()) <= this->closest_ik_extra_starts; ++k)
							closest_starts.push_back(random_starts[k]);
						const std::vector<closest_ik::Result> closest = this->SolveClosestCandidates(thread_state, target, closest_starts);
						result = closest_ik::Result();
						for (const closest_ik::Result& candidate : closest)
							if (candidate.exact)
								result = candidate;
						if (!result.exact && !closest.empty())
						{
							placement.miss_candidates[v] = this->KeepNearAndRoomy(thread_state, closest);
							result = closest_ik::Result();
							result.found = true;
							result.q = placement.miss_candidates[v].front().q;
						}
					}
					if (!result.found)
					{
						// No collision-free pose near: keep the last one and a fixed large gap, no pull.
						gaps[v] = this->closest_options.margin + 1.0;
						continue;
					}
					placement.poses[v] = result.q;
					placement.reached[v] = result.exact;
					if (!result.exact)
					{
						const MissCandidate& chosen = placement.miss_candidates[v].front();
						placement.position_errors[v] = chosen.position_error;
						placement.angle_errors[v] = chosen.angle_error;
						placement.clearance_shortfalls[v] = chosen.clearance_shortfall;
						gaps[v] = chosen.gap;
					}
				}
			});
		for (std::thread& thread : threads)
			thread.join();
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			placement.num_reached += placement.reached[v] ? 1 : 0;
			placement.gap_sum += gaps[v];
		}
		placement.gaps = gaps;
		return placement;
	}

	std::vector<Joints> GetTourPoses(const Placement& placement, const std::vector<int>& order) const
	{
		std::vector<Joints> poses;
		for (int viewpoint : order)
			poses.push_back(placement.poses[static_cast<size_t>(viewpoint)]);
		return poses;
	}

	// GTSP edge cost summed from home (tour length with joint_distance_weight 1), missed viewpoints at their closest poses.
	double GetTourCost(const Placement& placement, const std::vector<int>& order) const
	{
		std::vector<Joints> poses = this->GetTourPoses(placement, order);
		poses.insert(poses.begin(), this->home);
		return joint_travel::TravelCost(poses, this->edge_weights, this->travel_joints);
	}

	// d(edge cost)/d(to) by central differences. The cost depends on to - from only, so d/d(from) is its negative.
	Eigen::VectorXd GetEdgeCostSlope(const Joints& from, const Joints& to) const
	{
		const double delta = 1e-5;
		Eigen::VectorXd slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(to.size()));
		Joints shifted = to;
		for (size_t j = 0; j < to.size(); ++j)
		{
			shifted[j] = to[j] + delta;
			const double cost_up = joint_travel::EdgeCost(from, shifted, this->edge_weights, this->travel_joints);
			shifted[j] = to[j] - delta;
			const double cost_down = joint_travel::EdgeCost(from, shifted, this->edge_weights, this->travel_joints);
			shifted[j] = to[j];
			slope[static_cast<Eigen::Index>(j)] = (cost_up - cost_down) / (2.0 * delta);
		}
		return slope;
	}

	// Tool Jacobian at q; with ignore_roll the spin about the camera axis is dropped (as closest IK does).
	Eigen::MatrixXd GetToolJacobian(const Joints& q)
	{
		Eigen::MatrixXd jacobian = manipulability::ToolJacobian(*this->state, this->group, this->tool, q);
		if (this->exact_options.ignore_roll)
		{
			const Eigen::Vector3d axis = this->state->getGlobalLinkTransform(this->tool).linear().col(2);
			jacobian.bottomRows<3>() = (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * jacobian.bottomRows<3>();
		}
		return jacobian;
	}

	// Projector onto the self-motion: joint moves that leave the tool still. Zero if the arm has none (e.g. 6 joints).
	Eigen::MatrixXd GetSelfMotionProjector(const Eigen::MatrixXd& jacobian) const
	{
		const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullV);
		const Eigen::VectorXd& singular_values = svd.singularValues();
		const double tolerance = 1e-6 * std::max(1.0, singular_values.size() > 0 ? singular_values[0] : 0.0);
		const Eigen::Index rank = (singular_values.array() > tolerance).count();
		const Eigen::MatrixXd null_basis = svd.matrixV().rightCols(jacobian.cols() - rank);
		return null_basis * null_basis.transpose();
	}

	// d(tour cost)/d(pose joints) per viewpoint: each pose sits at the end of one edge and the start of the next.
	std::vector<Eigen::VectorXd> GetPoseSlopes(const Placement& placement, const std::vector<int>& order) const
	{
		std::vector<Eigen::VectorXd> pose_slopes(this->viewpoints.size(), Eigen::VectorXd::Zero(static_cast<Eigen::Index>(this->home.size())));
		for (size_t k = 0; k < order.size(); ++k)
		{
			const size_t to = static_cast<size_t>(order[k]);
			const Joints& from_pose = k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])];
			const Eigen::VectorXd slope = this->GetEdgeCostSlope(from_pose, placement.poses[to]);
			pose_slopes[to] += slope;
			if (k > 0)
				pose_slopes[static_cast<size_t>(order[k - 1])] -= slope;
		}
		return pose_slopes;
	}

	// Reach direction: each missed viewpoint closes its gap's position part (as in the SGD baseline) and frees what blocks
	// the rest, angle or clearance (room); unit, or zero if nothing is missed.
	Eigen::Vector3d GetReachDirection(const Placement& placement)
	{
		Eigen::Vector3d reach_move = Eigen::Vector3d::Zero();
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			if (placement.reached[v])
				continue;
			const double gap = placement.gaps[v];
			// The closest pose is a minimum over the joints, so only the gap's position part moves with the object.
			if (gap > 1e-9)
				reach_move -= placement.position_errors[v] / gap;
			// A miss on target (gap under 1 mm) is all blocked.
			const double blocked_share = gap < 1e-3 ? 1.0 : 1.0 - placement.position_errors[v].norm() / gap;
			if (this->use_room && blocked_share > 1e-3)
				reach_move += blocked_share * this->GetRoomDirection(placement, v);
		}
		return reach_move.norm() > 1e-12 ? Eigen::Vector3d(reach_move.normalized()) : Eigen::Vector3d::Zero();
	}

	// Tour direction from three pulls: reconfigurations (each posture match's gap, weighted by reconfiguration_weight), tour
	// length and joint room (their slopes in each pose's joints, through how the pose follows the object, J+ [I; 0]); where
	// two conflict each drops its part against the other (PCGrad). Slides (use_slide): each reached pose's length + room
	// slope projected onto its self-motion, downhill; a pose with a posture match slides toward its neighbour instead.
	DescentDirection GetTourDirection(const Descent& descent, double penalty)
	{
		const Placement& placement = descent.placement;
		const std::vector<Eigen::VectorXd> pose_slopes = this->GetPoseSlopes(placement, descent.order);
		Eigen::MatrixXd object_twist = Eigen::MatrixXd::Zero(6, 3);
		object_twist.topRows<3>() = Eigen::Matrix3d::Identity();
		Eigen::Vector3d reconfiguration_move = Eigen::Vector3d::Zero();
		Eigen::Vector3d length_move = Eigen::Vector3d::Zero();
		Eigen::Vector3d room_move = Eigen::Vector3d::Zero();
		DescentDirection direction;
		direction.slides.resize(this->viewpoints.size());
		double largest_slide = 0.0;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			const Eigen::MatrixXd jacobian = this->GetToolJacobian(placement.poses[v]);
			const Eigen::MatrixXd follow_rate = scene_geometry::DampedPinv(jacobian, this->jacobian_damping) * object_twist;
			Eigen::VectorXd room_slope = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(placement.poses[v].size()));
			if (placement.reached[v])
				this->GetRoomShortfall(placement.poses[v], &room_slope);
			length_move -= follow_rate.transpose() * pose_slopes[v];
			room_move -= this->GetRoomWeight(penalty) * follow_rate.transpose() * room_slope;
			if (!this->use_slide || !placement.reached[v])
				continue;
			direction.slides[v] = -this->GetSelfMotionProjector(jacobian) * (pose_slopes[v] + this->GetRoomWeight(penalty) * room_slope);
			largest_slide = std::max(largest_slide, direction.slides[v].norm());
		}
		for (Eigen::VectorXd& slide : direction.slides)
			if (slide.size() > 0)
				slide = largest_slide > 1e-9 ? Eigen::VectorXd(slide / largest_slide) : Eigen::VectorXd();
		for (const PostureMatch& match : descent.matches)
		{
			if (match.gap > 1e-9)
				reconfiguration_move -= this->reconfiguration_weight * match.position_error / match.gap;
			if (!this->use_slide || !placement.reached[match.viewpoint])
				continue;
			// Toward the neighbour's joints along the self-motion, full slide step.
			const Eigen::VectorXd toward = Eigen::Map<const Eigen::VectorXd>(match.neighbor_pose.data(), static_cast<Eigen::Index>(match.neighbor_pose.size())) - Eigen::Map<const Eigen::VectorXd>(placement.poses[match.viewpoint].data(), static_cast<Eigen::Index>(match.neighbor_pose.size()));
			const Eigen::VectorXd slide = this->GetSelfMotionProjector(this->GetToolJacobian(placement.poses[match.viewpoint])) * toward;
			if (slide.norm() > 1e-9)
				direction.slides[match.viewpoint] = slide.normalized();
		}
		const Eigen::Vector3d object_move = this->CombineWithoutConflict({reconfiguration_move, length_move, room_move});
		if (object_move.norm() > 1e-12)
			direction.object_direction = object_move.normalized();
		direction.has_slide = std::any_of(direction.slides.begin(), direction.slides.end(), [](const Eigen::VectorXd& slide) { return slide.size() > 0; });
		return direction;
	}

	// PCGrad (Yu et al. 2020): each move drops its component against every other move it conflicts with (negative dot
	// product, using the others' original directions), then all are summed. No move is ranked above another.
	Eigen::Vector3d CombineWithoutConflict(const std::vector<Eigen::Vector3d>& moves) const
	{
		Eigen::Vector3d total = Eigen::Vector3d::Zero();
		for (size_t i = 0; i < moves.size(); ++i)
		{
			Eigen::Vector3d move = moves[i];
			for (size_t j = 0; j < moves.size(); ++j)
			{
				const double other_size = moves[j].squaredNorm();
				if (j == i || other_size < 1e-24)
					continue;
				const double against = move.dot(moves[j]);
				if (against < 0.0)
					move -= against / other_size * moves[j];
			}
			total += move;
		}
		return total;
	}

	// Each pose slid by slide_step x its slide, stopping slide_joint_margin short of each joint limit (a joint already closer
	// may only move away); poses without a slide unchanged.
	std::vector<Joints> GetSlidPoses(const Placement& placement, const DescentDirection& direction, double slide_step) const
	{
		std::vector<Joints> slid_poses = placement.poses;
		const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
		for (size_t v = 0; v < slid_poses.size(); ++v)
		{
			if (direction.slides[v].size() == 0)
				continue;
			for (size_t j = 0; j < slid_poses[v].size(); ++j)
			{
				const moveit::core::VariableBounds& bounds = joints[j]->getVariableBounds()[0];
				const double value = slid_poses[v][j];
				slid_poses[v][j] += slide_step * direction.slides[v][static_cast<Eigen::Index>(j)];
				if (bounds.position_bounded_)
					slid_poses[v][j] = std::clamp(slid_poses[v][j], std::min(value, bounds.min_position_ + this->slide_joint_margin), std::max(value, bounds.max_position_ - this->slide_joint_margin));
			}
		}
		return slid_poses;
	}

	// GTSP over each viewpoint's pose, plus (use_neighbor_ik) exact IK started from its tour neighbours' poses; the chosen
	// poses replace the placement's. Never worse than `order` with the current poses.
	std::vector<int> ReorderTour(Placement& placement, const std::vector<int>& order)
	{
		std::vector<std::vector<Joints>> candidates(this->viewpoints.size());
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			candidates[v].push_back(placement.poses[v]);
			// A missed viewpoint's other closest poses (near its best gap, with room): GTSP picks by tour.
			if (!placement.reached[v] && v < placement.miss_candidates.size())
				for (const MissCandidate& candidate : placement.miss_candidates[v])
					if (candidate.q != placement.poses[v])
						candidates[v].push_back(candidate.q);
		}
		if (this->use_neighbor_ik && order.size() == this->viewpoints.size())
		{
			scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
			std::vector<std::thread> threads;
			for (unsigned int t = 0; t < this->num_threads; ++t)
				threads.emplace_back([&, t]()
				{
					moveit::core::RobotState thread_state(*this->state);
					for (size_t k = t; k < order.size(); k += this->num_threads)
					{
						const size_t v = static_cast<size_t>(order[k]);
						if (!placement.reached[v])
							continue;
						const Eigen::Isometry3d target = scene_geometry::Translation(placement.position) * this->viewpoints[v];
						std::vector<const Joints*> neighbors = {k == 0 ? &this->home : &placement.poses[static_cast<size_t>(order[k - 1])]};
						if (k + 1 < order.size())
							neighbors.push_back(&placement.poses[static_cast<size_t>(order[k + 1])]);
						for (const Joints* neighbor : neighbors)
						{
							const closest_ik::Result result = closest_ik::Solve(this->scene, thread_state, this->group, this->tool, target, *neighbor, this->exact_options);
							if (result.exact)
								candidates[v].push_back(result.q);
						}
					}
				});
			for (std::thread& thread : threads)
				thread.join();
		}
		gtsp::Solver solver(candidates, this->home, this->edge_weights, this->two_opt_rounds);
		solver.SetTravelJoints(this->travel_joints);
		gtsp::Tour tour = solver.Solve(order);
		// GTSP sees straight-line lengths only: charge the chosen edges whose straight move collides and solve again, so the
		// order can route around them (checking every pair up front would be too slow).
		for (int round = 0; round < this->gtsp_blocked_rounds && this->blocked_edge_charge > 0.0; ++round)
		{
			Placement chosen = placement;
			for (size_t k = 0; k < tour.order.size(); ++k)
				chosen.poses[static_cast<size_t>(tour.order[k])] = tour.poses[k];
			const std::vector<std::pair<size_t, double>> blocked = this->GetBlockedEdgeCharges(chosen, tour.order);
			if (blocked.empty())
				break;
			for (const std::pair<size_t, double>& edge : blocked)
			{
				const size_t k = edge.first;
				const size_t to = static_cast<size_t>(tour.order[k]);
				const std::pair<int, int> to_key = {static_cast<int>(to), this->FindPoseIndex(candidates[to], tour.poses[k])};
				const std::pair<int, int> from_key = k == 0 ? std::make_pair(-1, -1) : std::make_pair(tour.order[k - 1], this->FindPoseIndex(candidates[static_cast<size_t>(tour.order[k - 1])], tour.poses[k - 1]));
				solver.AddEdgeCost(from_key, to_key, edge.second);
			}
			tour = solver.Solve(tour.order);
		}
		for (size_t k = 0; k < tour.order.size(); ++k)
		{
			const size_t v = static_cast<size_t>(tour.order[k]);
			placement.poses[v] = tour.poses[k];
			if (placement.reached[v] || v >= placement.miss_candidates.size())
				continue;
			// A missed viewpoint's chosen closest pose brings its own gap.
			for (const MissCandidate& candidate : placement.miss_candidates[v])
			{
				if (candidate.q != tour.poses[k])
					continue;
				placement.gap_sum += candidate.gap - placement.gaps[v];
				placement.gaps[v] = candidate.gap;
				placement.position_errors[v] = candidate.position_error;
				placement.angle_errors[v] = candidate.angle_error;
				placement.clearance_shortfalls[v] = candidate.clearance_shortfall;
			}
		}
		return tour.order;
	}

	// Index of `pose` among a viewpoint's GTSP candidates (they are copied, so equal exactly); 0 if not found.
	int FindPoseIndex(const std::vector<Joints>& candidates, const Joints& pose) const
	{
		for (size_t i = 0; i < candidates.size(); ++i)
			if (candidates[i] == pose)
				return static_cast<int>(i);
		return 0;
	}

	// True if the straight joint move a -> b hits nothing, checked every 0.02 rad as the final planned length does.
	bool IsStraightMoveFree(moveit::core::RobotState& state, const Joints& a, const Joints& b) const
	{
		const double check_step = 0.02;	 // rad
		double largest = 0.0;
		for (size_t j = 0; j < a.size(); ++j)
			largest = std::max(largest, std::abs(b[j] - a[j]));
		const int steps = std::max(1, static_cast<int>(std::ceil(largest / check_step)));
		Joints q(a.size());
		for (int i = 1; i < steps; ++i)
		{
			const double fraction = static_cast<double>(i) / steps;
			for (size_t j = 0; j < a.size(); ++j)
				q[j] = a[j] + fraction * (b[j] - a[j]);
			if (!scene_geometry::IsStateValid(this->scene, &state, this->group, q.data()))
				return false;
		}
		return true;
	}

	// Tour edges order[k-1] (or home) -> order[k] whose straight move collides, with the extra length their detour likely
	// takes: through home if both straight halves are free, else blocked_edge_charge x the edge length. In parallel.
	std::vector<std::pair<size_t, double>> GetBlockedEdgeCharges(const Placement& placement, const std::vector<int>& order)
	{
		if (this->blocked_edge_charge <= 0.0 || order.empty())
			return {};
		scene_geometry::MoveObject(this->scene, scene_geometry::MakeIsometry(placement.position, this->rotation));
		std::vector<double> charges(order.size(), -1.0);  // -1: straight move free
		const unsigned int num_workers = std::min(this->num_threads, static_cast<unsigned int>(order.size()));
		std::vector<std::thread> threads;
		for (unsigned int t = 0; t < num_workers; ++t)
			threads.emplace_back([&, t]()
			{
				moveit::core::RobotState thread_state(*this->state);
				for (size_t k = t; k < order.size(); k += num_workers)
				{
					const Joints& from_pose = k == 0 ? this->home : placement.poses[static_cast<size_t>(order[k - 1])];
					const Joints& to_pose = placement.poses[static_cast<size_t>(order[k])];
					if (this->IsStraightMoveFree(thread_state, from_pose, to_pose))
						continue;
					const double straight = joint_travel::EdgeCost(from_pose, to_pose, this->edge_weights, this->travel_joints);
					const bool via_home = k > 0 && this->IsStraightMoveFree(thread_state, from_pose, this->home) && this->IsStraightMoveFree(thread_state, this->home, to_pose);
					const double detour = joint_travel::EdgeCost(from_pose, this->home, this->edge_weights, this->travel_joints) + joint_travel::EdgeCost(this->home, to_pose, this->edge_weights, this->travel_joints);
					charges[k] = via_home ? std::max(0.0, detour - straight) : this->blocked_edge_charge * straight;
				}
			});
		for (std::thread& thread : threads)
			thread.join();
		std::vector<std::pair<size_t, double>> blocked;
		for (size_t k = 0; k < order.size(); ++k)
			if (charges[k] >= 0.0)
				blocked.push_back({k, charges[k]});
		return blocked;
	}

	// Live markers on /object_placement_progress_markers: the object, its trail this descent, the descent direction, the
	// viewpoints (green reached, red missed), the tour's tool path, and each miss's closest arm pose with a line to its target.
	void PublishProgress(const Placement& placement, const std::vector<int>& order, const Eigen::Vector3d& object_direction)
	{
		const rclcpp::Time stamp = this->now();
		const Eigen::Isometry3d object_transform = scene_geometry::Translation(placement.position);
		visualization_msgs::msg::MarkerArray markers;
		// Markers update in place (same ns and id); the first publish clears what an earlier run left.
		if (this->progress_marker_keys.empty())
		{
			visualization_msgs::msg::Marker clear;
			clear.action = visualization_msgs::msg::Marker::DELETEALL;
			markers.markers.push_back(clear);
		}

		visualization_msgs::msg::Marker mesh = marker_utils::MakeMarker("object_placement_current", visualization_msgs::msg::Marker::MESH_RESOURCE, stamp);
		mesh.mesh_resource = "file://" + this->mesh_path;
		mesh.mesh_use_embedded_materials = false;
		mesh.pose = scene_geometry::ToPoseMsg(scene_geometry::MakeIsometry(placement.position, this->rotation));
		mesh.scale.x = mesh.scale.y = mesh.scale.z = this->mesh_scale;
		mesh.color = marker_utils::Rgba(1.0f, 0.85f, 0.0f, 0.6f);
		markers.markers.push_back(mesh);

		visualization_msgs::msg::Marker viewpoint_markers = marker_utils::MakeMarker("object_placement_viewpoints", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
		viewpoint_markers.scale.x = viewpoint_markers.scale.y = viewpoint_markers.scale.z = 0.01;
		viewpoint_markers.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			viewpoint_markers.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[v]).translation()));
			viewpoint_markers.colors.push_back(placement.reached[v] ? marker_utils::Rgba(0.1f, 0.9f, 0.1f, 1.0f) : marker_utils::Rgba(0.9f, 0.1f, 0.1f, 1.0f));
		}
		markers.markers.push_back(viewpoint_markers);

		// Trail as one cylinder per move: line markers vanish from some viewing angles.
		visualization_msgs::msg::Marker crumbs = marker_utils::MakeMarker("object_placement_history", visualization_msgs::msg::Marker::SPHERE_LIST, stamp);
		crumbs.scale.x = crumbs.scale.y = crumbs.scale.z = 0.012;
		crumbs.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
		for (size_t k = 0; k < this->trail.size(); ++k)
		{
			crumbs.points.push_back(scene_geometry::ToPoint(this->trail[k]));
			const double length = k == 0 ? 0.0 : (this->trail[k] - this->trail[k - 1]).norm();
			if (length < 1e-6)
				continue;
			visualization_msgs::msg::Marker segment = marker_utils::MakeMarker("object_placement_trail", visualization_msgs::msg::Marker::CYLINDER, stamp, static_cast<int>(k));
			Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
			pose.translation() = 0.5 * (this->trail[k] + this->trail[k - 1]);
			pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), this->trail[k] - this->trail[k - 1]).toRotationMatrix();
			segment.pose = scene_geometry::ToPoseMsg(pose);
			segment.scale.x = segment.scale.y = 0.005;
			segment.scale.z = length;
			segment.color = marker_utils::Rgba(1.0f, 1.0f, 1.0f, 1.0f);
			markers.markers.push_back(segment);
		}
		markers.markers.push_back(crumbs);

		if (object_direction.norm() > 1e-9)
		{
			visualization_msgs::msg::Marker arrow = marker_utils::MakeMarker("object_placement_descent_dir", visualization_msgs::msg::Marker::ARROW, stamp);
			arrow.scale.x = 0.006;
			arrow.scale.y = 0.014;
			arrow.color = marker_utils::Rgba(0.1f, 0.5f, 1.0f, 0.95f);
			arrow.points = {scene_geometry::ToPoint(placement.position), scene_geometry::ToPoint(placement.position + 0.15 * object_direction.normalized())};
			markers.markers.push_back(arrow);
		}

		// The tour's tool path from home, through each pose (missed ones at their closest pose).
		visualization_msgs::msg::Marker tool_path = marker_utils::MakeMarker("object_placement_tool_path", visualization_msgs::msg::Marker::LINE_STRIP, stamp);
		tool_path.scale.x = 0.003;
		tool_path.color = marker_utils::Rgba(0.2f, 0.6f, 1.0f, 0.9f);
		moveit::core::RobotState ghost(*this->state);
		std::vector<Joints> path_poses = this->GetTourPoses(placement, order);
		path_poses.insert(path_poses.begin(), this->home);
		for (const Joints& pose : path_poses)
		{
			ghost.setJointGroupPositions(this->group, pose);
			ghost.update();
			tool_path.points.push_back(scene_geometry::ToPoint(ghost.getGlobalLinkTransform(this->tool).translation()));
		}
		if (tool_path.points.size() > 1)
			markers.markers.push_back(tool_path);

		visualization_msgs::msg::Marker gaps = marker_utils::MakeMarker("object_placement_miss_gap", visualization_msgs::msg::Marker::LINE_LIST, stamp);
		gaps.scale.x = 0.003;
		gaps.color = marker_utils::Rgba(1.0f, 0.2f, 1.0f, 1.0f);
		visualization_msgs::msg::MarkerArray arms;
		for (size_t v = 0; v < this->viewpoints.size(); ++v)
		{
			if (placement.reached[v])
				continue;
			ghost.setJointGroupPositions(this->group, placement.poses[v]);
			ghost.update();
			ghost.getRobotMarkers(arms, this->model->getLinkModelNamesWithCollisionGeometry(), marker_utils::Rgba(0.9f, 0.2f, 0.2f, 0.35f), "object_placement_miss_ik", rclcpp::Duration(0, 0));
			gaps.points.push_back(scene_geometry::ToPoint(ghost.getGlobalLinkTransform(this->tool).translation()));
			gaps.points.push_back(scene_geometry::ToPoint((object_transform * this->viewpoints[v]).translation()));
		}
		if (!gaps.points.empty())
			markers.markers.push_back(gaps);
		for (visualization_msgs::msg::Marker& arm : arms.markers)
		{
			arm.header.frame_id = "world";
			arm.header.stamp = stamp;
			markers.markers.push_back(arm);
		}

		// Delete markers the last publish had and this one doesn't (e.g. a miss now reached).
		std::set<std::pair<std::string, int>> keys;
		for (const visualization_msgs::msg::Marker& marker : markers.markers)
			if (marker.action == visualization_msgs::msg::Marker::ADD)
				keys.insert({marker.ns, marker.id});
		for (const std::pair<std::string, int>& key : this->progress_marker_keys)
			if (keys.count(key) == 0)
			{
				visualization_msgs::msg::Marker stale = marker_utils::MakeMarker(key.first, visualization_msgs::msg::Marker::SPHERE, stamp, key.second);
				stale.action = visualization_msgs::msg::Marker::DELETE;
				markers.markers.push_back(stale);
			}
		this->progress_marker_keys = std::move(keys);
		this->progress_pub->publish(markers);
		if (this->progress_delay > 0.0)
			std::this_thread::sleep_for(std::chrono::duration<double>(this->progress_delay));
	}

	// Smallest distance of any tour joint (camera spin aside) from its limit, beyond the hard margin the launch file applies.
	double GetSmallestJointMargin(const std::vector<Joints>& poses) const
	{
		double smallest = std::numeric_limits<double>::infinity();
		const std::vector<const moveit::core::JointModel*>& joints = this->group->getActiveJointModels();
		for (const Joints& pose : poses)
			for (int j : this->travel_joints)
			{
				const moveit::core::VariableBounds& bounds = joints[static_cast<size_t>(j)]->getVariableBounds()[0];
				if (!bounds.position_bounded_)
					continue;
				const double value = pose[static_cast<size_t>(j)];
				smallest = std::min(smallest, std::min(value - bounds.min_position_, bounds.max_position_ - value));
			}
		return smallest;
	}

	moveit::core::RobotModelConstPtr model;
	std::unique_ptr<moveit::core::RobotState> state;
	const moveit::core::JointModelGroup* group = nullptr;
	const moveit::core::LinkModel* tool = nullptr;
	planning_scene_monitor::PlanningSceneMonitorPtr scene;
	std::vector<Eigen::Isometry3d> viewpoints;	// relative to the object position
	Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d lower_bound = Eigen::Vector3d::Zero();
	Eigen::Vector3d upper_bound = Eigen::Vector3d::Zero();
	Joints home;
	std::vector<int> travel_joints;
	joint_travel::EdgeWeights edge_weights;
	closest_ik::Options exact_options;
	closest_ik::Options closest_options;
	double jacobian_damping = 1e-3;
	int two_opt_rounds = 5;
	int seed = 42;
	unsigned int num_threads = 1;
	int max_steps = 200;
	int max_line_search_tries = 4;
	int ik_attempts = 10;
	int closest_ik_extra_starts = 3;
	double closest_gap_slack = 0.005;
	double initial_step = 0.02;
	double min_step = 0.001;
	double max_step = 0.05;
	double initial_penalty = 100.0;
	double penalty_growth = 10.0;
	double max_penalty = 1e5;
	bool use_neighbor_ik = true;
	bool use_slide = true;
	double initial_slide_step = 0.3;
	double slide_joint_margin = 0.05;
	double reconfig_threshold = 1.0;
	bool use_posture_match = true;
	double reconfiguration_weight = 10.0;
	bool use_room = true;
	bool tour_object_move = true;
	bool freeze_object_when_reached = false;
	double blocked_edge_charge = 4.0;
	int gtsp_blocked_rounds = 2;
	double miss_cost = 10.0;
	double reconfiguration_cost = 2.0;
	double joint_room_band = 0.2;
	double joint_room_weight = 50.0;
	std::chrono::steady_clock::time_point start_time;
	double time_budget_end = 0.0;  // s since start_time
	std::string mesh_path;
	double mesh_scale = 0.01;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_pub;
	std::set<std::string> camera_side_links;
	std::set<std::pair<std::string, int>> progress_marker_keys;	 // markers in the last progress publish
	std::vector<Eigen::Vector3d> trail;	 // object positions this descent
	double progress_delay = 0.0;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ObjectGtspCoOptimizeNode>();
	std::thread spin([node]() { rclcpp::spin(node); });
	node->Run();
	rclcpp::shutdown();
	spin.join();
	return 0;
}
