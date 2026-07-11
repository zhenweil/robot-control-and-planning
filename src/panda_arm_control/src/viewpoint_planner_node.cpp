#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/mesh_utils.hpp"
#include "panda_arm_control/pose_utils.hpp"
#include "panda_arm_control/potential_field_sampler.hpp"
#include "panda_arm_control/rkga_scp.hpp"
#include "panda_arm_control/viewpoint_types.hpp"

namespace
{

struct Params
{
	std::string mesh_path;
	double mesh_scale = 0.01;
	int target_faces = 1000;
	int n_surface_samples = 500;
	std::vector<double> standoff_distances = {0.01, 0.02, 0.03};
	std::vector<double> tilt_angles_deg = {0.0, 15.0, -15.0};
	double min_clearance = 0.005;
	double fov_deg = 30.0;
	double max_distance = 20.0;
	double angle_threshold_deg = 70.0;
	int max_rays_per_view = 2000;
	double target_area_visibility = 0.95;
	double min_new_area_ratio = 0.001;
	double ik_timeout = 0.1;
	int random_seed = 42;
	std::string group_name = "panda_arm";
	std::vector<double> object_translation_world = {0.2, 0.2, 0.38};
	std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
	// Shared with WaypointFollower::initial_joints (panda_arm_control.cpp) via
	// config/object_pose.yaml -- the proven-safe "ready" configuration the real robot actually
	// returns to. Used as the tour's home reference point instead of the URDF's raw (uncollision-
	// checked) joint defaults, since executeTourOnRobot() actually drives the real robot here.
	std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};
	std::vector<double> t_tcp_camera_xyz = {0.0, 0.0, 0.0};
	std::vector<double> t_tcp_camera_quat_xyzw = {0.0, 0.0, 0.0, 1.0};
	// Tour ordering cost is euclidean_distance_m + joint_distance_weight * joint_distance_rad,
	// so this trades off "prefer short Cartesian hops" against "prefer small joint motion"
	// (1.0 rad of joint motion counts as much as joint_distance_weight meters of TCP travel).
	double joint_distance_weight = 0.1;
	// Applied only to the original mesh's comparison-marker position (not the object's actual
	// pose), so the original and simplified meshes render side by side instead of overlapping.
	std::vector<double> mesh_comparison_offset = {0.0, 0.3, 0.0};
	// Swaps candidate generation/reachability/selection for the potential-field sampler +
	// collision-aware IK + RKGA-SCP genetic algorithm (rkga_scp.hpp) instead of the standoff/tilt
	// grid + plain IK + greedy set-cover + NN/2-opt pipeline.
	bool use_rkga = false;
	// Only used when use_rkga is true. Instead of SolveRkgaScp's GA jointly picking selection +
	// order, select via the same GreedySelectViewpoints used by the non-RKGA pipeline, then order
	// via NearestNeighborOrderMatrix + TwoOptImproveMatrix -- using the real travel_cost_matrix
	// costs, not TourCost's straight-line estimate. Lets GA-ordering vs NN/2-opt-ordering be
	// compared under the exact same real-cost model.
	bool use_matrix_2opt = false;
	// Per-pair timeout for ComputeEndEffectorTravelDistanceMatrix's real motion-planning calls
	// (used only when use_rkga is true). Runs once, up front, for every pair of reachable
	// candidates -- keep this well under panda_arm_control's 20s execution-time budget, since the
	// total cost is this value times roughly N^2/2 pairs.
	double rkga_planning_time = 2.0;
	// Only used when use_rkga is true. When true, after the final tour is selected/ordered, this
	// node re-plans just that tour's legs at higher fidelity (PlanFinalTourTrajectories) and drives
	// the real robot through them directly via MoveGroupInterface -- instead of (or as well as)
	// just publishing poses on /cartesian_waypoints for a separate node to re-plan and execute.
	// Defaults to false so nothing physically moves unless explicitly requested.
	bool execute_on_robot = false;
	// Planning time/attempts for PlanFinalTourTrajectories -- affordable to be more generous than
	// rkga_planning_time since this only runs once per tour leg (selected.size() calls), not
	// O(n^2).
	double execution_planning_time = 5.0;
	int execution_planning_attempts = 5;
	std::string output_dir = "/tmp/viewpoint_planner_output";
};

Eigen::Vector3d ToVector3(const std::vector<double>& v, const Eigen::Vector3d& fallback)
{
	if (v.size() != 3)
		return fallback;
	return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Matrix3d RotationFromRpyDeg(const std::vector<double>& rpy_deg)
{
	if (rpy_deg.size() != 3)
		return Eigen::Matrix3d::Identity();

	double roll = rpy_deg[0] * M_PI / 180.0;
	double pitch = rpy_deg[1] * M_PI / 180.0;
	double yaw = rpy_deg[2] * M_PI / 180.0;

	// Matches scipy's R.from_euler("xyz", [roll, pitch, yaw]).as_matrix(): Rz * Ry * Rx.
	Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
						 .toRotationMatrix();
	return R;
}

Eigen::Isometry3d IsometryFromXyzQuat(const std::vector<double>& xyz, const std::vector<double>& quat_xyzw)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	if (xyz.size() == 3)
		T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
	if (quat_xyzw.size() == 4)
		T.linear() = Eigen::Quaterniond(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]).toRotationMatrix();
	return T;
}

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& v)
{
	geometry_msgs::msg::Point p;
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	return p;
}

} // namespace

class ViewpointPlannerNode : public rclcpp::Node
{
public:
	explicit ViewpointPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("viewpoint_planner", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
	{
	}

	void init()
	{
		this->declareToolParameters();
		this->loadParameters();

		robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
		this->robot_model = loader.getModel();
		this->robot_state = std::make_shared<moveit::core::RobotState>(this->robot_model);
		this->jmg = this->robot_state->getJointModelGroup(this->params.group_name);

		// this->params.initial_joints (shared with WaypointFollower via config/object_pose.yaml) is
		// the proven-safe "ready" configuration the real robot actually returns to before/after
		// executing waypoints, not the URDF's raw joint defaults. Those can be in collision with the
		// registered object (setToDefaultValues() is never collision-checked), which only matters
		// now that executeTourOnRobot() actually drives the real robot to this pose instead of just
		// using it as an abstract cost-reference point.
		this->robot_state->setToDefaultValues();
		this->robot_state->setJointGroupPositions(this->jmg, this->params.initial_joints);

		// Capture the home tool0 position/joint values now, before computeReachability() mutates
		// robot_state with IK solutions -- this is the reference point the nearest-neighbor
		// tour starts from.
		this->robot_state->update();
		this->home_tcp_position = this->robot_state->getGlobalLinkTransform("tool0").translation();
		this->robot_state->copyJointGroupPositions(this->jmg, this->home_joint_values);

		this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/viewpoint_markers", rclcpp::QoS(1).transient_local());

		this->mesh_comparison_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/mesh_comparison_markers", rclcpp::QoS(1).transient_local());

		this->waypoint_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
			"/cartesian_waypoints", rclcpp::QoS(1).transient_local());

		if (this->params.use_rkga && this->params.execute_on_robot)
		{
			this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
				this->shared_from_this(), this->params.group_name);
			this->move_group->startStateMonitor();
			this->move_group->setPlanningTime(this->params.execution_planning_time);
			this->move_group->setNumPlanningAttempts(this->params.execution_planning_attempts);
			this->move_group->setMaxVelocityScalingFactor(1.0);
			this->move_group->setMaxAccelerationScalingFactor(1.0);
			this->move_group->setEndEffectorLink("tool0");
		}

		this->runPipeline();

		this->marker_timer = this->create_wall_timer(
			std::chrono::seconds(2), [this]() { this->marker_pub->publish(this->marker_array); });
	}

private:
	Params params;

	moveit::core::RobotModelPtr robot_model;
	moveit::core::RobotStatePtr robot_state;
	const moveit::core::JointModelGroup* jmg = nullptr;
	Eigen::Vector3d home_tcp_position = Eigen::Vector3d::Zero();
	std::vector<double> home_joint_values;

	std::vector<ViewpointCandidate> reachable_candidates;
	std::vector<ViewpointCandidate> unreachable_visible;
	std::vector<const ViewpointCandidate*> selected;
	std::string resolved_mesh_path;
	std::string resolved_simplified_mesh_path;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_comparison_pub;
	rclcpp::TimerBase::SharedPtr marker_timer;
	visualization_msgs::msg::MarkerArray marker_array;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_pub;

	// Only constructed when params.execute_on_robot is true (see init()) -- talks to the real
	// move_group action server, unlike robot_state/jmg above which are a local, disconnected
	// simulated state used only for reachability/planning-time-only checks.
	moveit::planning_interface::MoveGroupInterfacePtr move_group;

	// automatically_declare_parameters_from_overrides(true) (needed for robot_description*)
	// already auto-declares any of these that the launch file passed as overrides, so guard
	// against re-declaring them here.
	template <typename T>
	void declareIfNeeded(const std::string& name, const T& default_value)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, default_value);
	}

	void declareToolParameters()
	{
		this->declareIfNeeded("mesh_path", this->params.mesh_path);
		this->declareIfNeeded("mesh_scale", this->params.mesh_scale);
		this->declareIfNeeded("target_faces", this->params.target_faces);
		this->declareIfNeeded("n_surface_samples", this->params.n_surface_samples);
		this->declareIfNeeded("standoff_distances", this->params.standoff_distances);
		this->declareIfNeeded("tilt_angles_deg", this->params.tilt_angles_deg);
		this->declareIfNeeded("min_clearance", this->params.min_clearance);
		this->declareIfNeeded("fov_deg", this->params.fov_deg);
		this->declareIfNeeded("max_distance", this->params.max_distance);
		this->declareIfNeeded("angle_threshold_deg", this->params.angle_threshold_deg);
		this->declareIfNeeded("max_rays_per_view", this->params.max_rays_per_view);
		this->declareIfNeeded("target_area_visibility", this->params.target_area_visibility);
		this->declareIfNeeded("min_new_area_ratio", this->params.min_new_area_ratio);
		this->declareIfNeeded("ik_timeout", this->params.ik_timeout);
		this->declareIfNeeded("random_seed", this->params.random_seed);
		this->declareIfNeeded("group_name", this->params.group_name);
		this->declareIfNeeded("object_translation_world", this->params.object_translation_world);
		this->declareIfNeeded("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->declareIfNeeded("initial_joints", this->params.initial_joints);
		this->declareIfNeeded("t_tcp_camera_xyz", this->params.t_tcp_camera_xyz);
		this->declareIfNeeded("t_tcp_camera_quat_xyzw", this->params.t_tcp_camera_quat_xyzw);
		this->declareIfNeeded("joint_distance_weight", this->params.joint_distance_weight);
		this->declareIfNeeded("mesh_comparison_offset", this->params.mesh_comparison_offset);
		this->declareIfNeeded("use_rkga", this->params.use_rkga);
		this->declareIfNeeded("use_matrix_2opt", this->params.use_matrix_2opt);
		this->declareIfNeeded("rkga_planning_time", this->params.rkga_planning_time);
		this->declareIfNeeded("execute_on_robot", this->params.execute_on_robot);
		this->declareIfNeeded("execution_planning_time", this->params.execution_planning_time);
		this->declareIfNeeded("execution_planning_attempts", this->params.execution_planning_attempts);
		this->declareIfNeeded("output_dir", this->params.output_dir);
	}

	void loadParameters()
	{
		this->get_parameter("mesh_path", this->params.mesh_path);
		this->get_parameter("mesh_scale", this->params.mesh_scale);
		this->get_parameter("target_faces", this->params.target_faces);
		this->get_parameter("n_surface_samples", this->params.n_surface_samples);
		this->get_parameter("standoff_distances", this->params.standoff_distances);
		this->get_parameter("tilt_angles_deg", this->params.tilt_angles_deg);
		this->get_parameter("min_clearance", this->params.min_clearance);
		this->get_parameter("fov_deg", this->params.fov_deg);
		this->get_parameter("max_distance", this->params.max_distance);
		this->get_parameter("angle_threshold_deg", this->params.angle_threshold_deg);
		this->get_parameter("max_rays_per_view", this->params.max_rays_per_view);
		this->get_parameter("target_area_visibility", this->params.target_area_visibility);
		this->get_parameter("min_new_area_ratio", this->params.min_new_area_ratio);
		this->get_parameter("ik_timeout", this->params.ik_timeout);
		this->get_parameter("random_seed", this->params.random_seed);
		this->get_parameter("group_name", this->params.group_name);
		this->get_parameter("object_translation_world", this->params.object_translation_world);
		this->get_parameter("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->get_parameter("initial_joints", this->params.initial_joints);
		this->get_parameter("t_tcp_camera_xyz", this->params.t_tcp_camera_xyz);
		this->get_parameter("t_tcp_camera_quat_xyzw", this->params.t_tcp_camera_quat_xyzw);
		this->get_parameter("joint_distance_weight", this->params.joint_distance_weight);
		this->get_parameter("mesh_comparison_offset", this->params.mesh_comparison_offset);
		this->get_parameter("use_rkga", this->params.use_rkga);
		this->get_parameter("use_matrix_2opt", this->params.use_matrix_2opt);
		this->get_parameter("rkga_planning_time", this->params.rkga_planning_time);
		this->get_parameter("execute_on_robot", this->params.execute_on_robot);
		this->get_parameter("execution_planning_time", this->params.execution_planning_time);
		this->get_parameter("execution_planning_attempts", this->params.execution_planning_attempts);
		this->get_parameter("output_dir", this->params.output_dir);
	}

	void runPipeline()
	{
		std::string mesh_path = this->params.mesh_path;
		if (mesh_path.empty())
		{
			mesh_path =
				ament_index_cpp::get_package_share_directory("panda_arm_control") + "/meshes/bunny_holding_eggs.stl";
		}

		this->resolved_mesh_path = mesh_path;
		std::filesystem::create_directories(this->params.output_dir);

		auto original_poly = LoadAndScaleMesh(mesh_path, this->params.mesh_scale);
		MeshData original_mesh = BuildMeshData(original_poly);

		auto simplified_poly = SimplifyMesh(original_poly, this->params.target_faces);
		MeshData simplified_mesh = BuildMeshData(simplified_poly);

		this->resolved_simplified_mesh_path = this->params.output_dir + "/simplified_mesh.stl";
		ExportMeshToStl(simplified_poly, this->resolved_simplified_mesh_path);

		std::mt19937 rng(this->params.random_seed);
		std::vector<ViewpointCandidate> candidates;
		if (this->params.use_rkga)
		{
			candidates = GeneratePotentialFieldViewCandidates(
				simplified_mesh, this->params.n_surface_samples, this->params.standoff_distances, rng);
		}
		else
		{
			candidates = GenerateViewCandidates(
				simplified_mesh, this->params.n_surface_samples, this->params.standoff_distances,
				this->params.tilt_angles_deg, rng);
		}

		RCLCPP_INFO(this->get_logger(), "candidate views: %zu", candidates.size());

		candidates = FilterByClearance(original_poly, std::move(candidates), this->params.min_clearance);

		VisibilityChecker vis_checker(simplified_mesh);
		candidates = ComputeVisibility(
			vis_checker, std::move(candidates), this->params.fov_deg, this->params.max_distance,
			this->params.angle_threshold_deg, this->params.max_rays_per_view);

		RCLCPP_INFO(this->get_logger(), "visible candidate views: %zu", candidates.size());

		planning_scene_monitor::PlanningSceneMonitorPtr rkga_local_scene;
		if (this->params.use_rkga)
		{
			Eigen::Vector3d object_translation_world =
				ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
			Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);
			Eigen::Isometry3d t_tcp_camera =
				IsometryFromXyzQuat(this->params.t_tcp_camera_xyz, this->params.t_tcp_camera_quat_xyzw);

			rkga_local_scene = BuildLocalCollisionScene(
				this->shared_from_this(), this->resolved_mesh_path, this->params.mesh_scale,
				object_translation_world, object_rotation_world);

			candidates = ComputeReachabilityWithCollisionCheck(
				std::move(candidates), this->robot_state, this->jmg, rkga_local_scene, object_translation_world,
				object_rotation_world, t_tcp_camera, this->params.ik_timeout);
		}
		else
		{
			candidates = this->computeReachability(std::move(candidates));
		}

		for (auto& c : candidates)
		{
			if (c.reachable)
				this->reachable_candidates.push_back(std::move(c));
			else
				this->unreachable_visible.push_back(std::move(c));
		}

		RCLCPP_INFO(
			this->get_logger(), "reachable candidate views: %zu / %zu", this->reachable_candidates.size(),
			this->reachable_candidates.size() + this->unreachable_visible.size());

		if (this->params.use_rkga)
		{
			RCLCPP_INFO(
				this->get_logger(), "Computing real end-effector travel distance matrix for %zu candidates (this "
									 "runs real motion planning for every pair -- can take a while)...",
				this->reachable_candidates.size());

			TravelCostMatrix travel_cost_matrix = ComputeEndEffectorTravelDistanceMatrix(
				this->shared_from_this(), this->robot_model, rkga_local_scene, this->reachable_candidates,
				this->home_joint_values, this->params.group_name, this->params.rkga_planning_time);

			if (this->params.use_matrix_2opt)
			{
				this->selected = GreedySelectViewpoints(
					simplified_mesh, this->reachable_candidates, this->params.target_area_visibility,
					this->params.min_new_area_ratio);

				RCLCPP_INFO(this->get_logger(), "selected views: %zu", this->selected.size());

				this->selected = NearestNeighborOrderMatrix(
					this->reachable_candidates, std::move(this->selected), travel_cost_matrix,
					this->params.joint_distance_weight);
				this->selected = TwoOptImproveMatrix(
					this->reachable_candidates, std::move(this->selected), travel_cost_matrix,
					this->params.joint_distance_weight);

				PrintTourCostBreakdown(
					this->reachable_candidates, this->selected, travel_cost_matrix,
					this->params.joint_distance_weight);

				double achieved_visibility = ComputeAreaVisibility(simplified_mesh, this->selected);
				RCLCPP_INFO(
					this->get_logger(),
					"NN+2-opt (real travel_cost_matrix) reordered %zu selected views, %.2f%% area visibility",
					this->selected.size(), achieved_visibility * 100.0);
			}
			else
			{
				RkgaScpParams rkga_params;
				rkga_params.target_area_visibility = this->params.target_area_visibility;
				rkga_params.random_seed = static_cast<unsigned int>(this->params.random_seed);
				rkga_params.joint_distance_weight = this->params.joint_distance_weight;

				this->selected =
					SolveRkgaScp(simplified_mesh, this->reachable_candidates, travel_cost_matrix, rkga_params);

				double achieved_visibility = ComputeAreaVisibility(simplified_mesh, this->selected);
				RCLCPP_INFO(
					this->get_logger(),
					"RKGA-SCP selected %zu views (jointly selected + ordered), %.2f%% area visibility",
					this->selected.size(), achieved_visibility * 100.0);
			}
		}
		else
		{
			this->selected = GreedySelectViewpoints(
				simplified_mesh, this->reachable_candidates, this->params.target_area_visibility,
				this->params.min_new_area_ratio);

			RCLCPP_INFO(this->get_logger(), "selected views: %zu", this->selected.size());

			this->selected = NearestNeighborOrder(
				std::move(this->selected), this->home_tcp_position, this->home_joint_values,
				this->params.joint_distance_weight);
			this->selected = TwoOptImprove(
				std::move(this->selected), this->home_tcp_position, this->home_joint_values,
				this->params.joint_distance_weight);

			RCLCPP_INFO(
				this->get_logger(),
				"reordered %zu selected views via nearest-neighbor + 2-opt from robot home position",
				this->selected.size());
		}

		this->exportSelectedViewpoints();
		this->exportSelectedRobotPoses();
		this->marker_array = this->buildMarkerArray();
		this->marker_pub->publish(this->marker_array);

		this->mesh_comparison_pub->publish(this->buildMeshComparisonMarkers());

		this->publishWaypoints();

		// Runs after publishing markers/waypoints (not before) so RViz shows the planned mesh/tour
		// before the real robot is committed to executing it, rather than only appearing once
		// execution -- which blocks for the whole tour -- has already finished.
		if (this->params.use_rkga && this->params.execute_on_robot)
			this->executeTourOnRobot(rkga_local_scene);
	}

	// Plans+executes a move from the robot's current state to home_joint_values. Used both before
	// the tour (the cached legs were planned starting from home_joint_values, so the real robot has
	// to actually be there first) and after it (to leave the robot in the same resting
	// configuration WaypointFollower's return_to_initial_pose() does).
	bool moveToHome()
	{
		this->move_group->setStartStateToCurrentState();
		this->move_group->setJointValueTarget(this->home_joint_values);
		moveit::planning_interface::MoveGroupInterface::Plan home_plan;
		return static_cast<bool>(this->move_group->plan(home_plan)) &&
			this->move_group->execute(home_plan) == moveit::core::MoveItErrorCode::SUCCESS;
	}

	// Re-plans the final tour's legs at higher fidelity (PlanFinalTourTrajectories) and drives the
	// real robot through them directly via move_group, instead of just publishing poses for a
	// separate node to re-plan -- so what gets executed is the exact trajectory that was planned,
	// not re-derived by a second, independent IK+plan pass with different seeds/randomness (see
	// PlanFinalTourTrajectories in rkga_scp.hpp for why that matters).
	void executeTourOnRobot(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
	{
		if (this->selected.empty())
		{
			RCLCPP_WARN(this->get_logger(), "No reachable/selected viewpoints -- nothing to execute on robot");
			return;
		}

		RCLCPP_INFO(
			this->get_logger(), "Planning %zu final tour legs at higher fidelity for execution...",
			this->selected.size());

		std::vector<TourTrajectory> legs = PlanFinalTourTrajectories(
			this->shared_from_this(), this->robot_model, planning_scene_monitor, this->selected,
			this->home_joint_values, this->params.group_name, this->params.execution_planning_time,
			this->params.execution_planning_attempts);

		for (size_t i = 0; i < legs.size(); ++i)
		{
			if (!legs[i].ok)
			{
				RCLCPP_ERROR(this->get_logger(), "Aborting execution: failed to plan leg %zu/%zu", i, legs.size());
				return;
			}
		}

		// The cached legs were planned starting from home_joint_values -- the real robot has to
		// actually be there before executing them, or the first leg's trajectory won't match the
		// robot's current state.
		RCLCPP_INFO(this->get_logger(), "Moving to tour start (robot home) before execution...");
		if (!this->moveToHome())
		{
			RCLCPP_ERROR(this->get_logger(), "Aborting execution: failed to move to tour start (robot home)");
			return;
		}

		for (size_t i = 0; i < legs.size(); ++i)
		{
			RCLCPP_INFO(this->get_logger(), "Executing tour leg %zu/%zu", i + 1, legs.size());
			moveit::core::MoveItErrorCode result = this->move_group->execute(legs[i].trajectory);
			if (result != moveit::core::MoveItErrorCode::SUCCESS)
			{
				RCLCPP_ERROR(
					this->get_logger(), "Aborting execution: leg %zu/%zu failed to execute (error code %d)", i + 1,
					legs.size(), result.val);
				return;
			}
		}

		RCLCPP_INFO(this->get_logger(), "Finished executing %zu-leg tour on robot, returning to home...", legs.size());
		if (!this->moveToHome())
			RCLCPP_ERROR(this->get_logger(), "Failed to return to home after executing tour");
	}

	void publishWaypoints()
	{
		if (this->selected.empty())
		{
			RCLCPP_WARN(
				this->get_logger(), "No reachable/selected viewpoints -- nothing to publish on /cartesian_waypoints");
			return;
		}

		geometry_msgs::msg::PoseArray msg;
		msg.header.frame_id = "world";
		msg.header.stamp = this->now();
		msg.poses.reserve(this->selected.size());
		for (const auto* c : this->selected)
			msg.poses.push_back(c->tcp_pose);

		this->waypoint_pub->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Published %zu TCP waypoints to /cartesian_waypoints", msg.poses.size());
	}

	std::vector<ViewpointCandidate> computeReachability(std::vector<ViewpointCandidate> candidates)
	{
		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);
		Eigen::Isometry3d t_tcp_camera =
			IsometryFromXyzQuat(this->params.t_tcp_camera_xyz, this->params.t_tcp_camera_quat_xyzw);

		for (size_t i = 0; i < candidates.size(); ++i)
		{
			auto& c = candidates[i];
			c.tcp_pose = ConvertViewpointToTcpPose(
				c.camera_pos, c.view_dir, object_translation_world, object_rotation_world, t_tcp_camera);

			c.reachable = this->robot_state->setFromIK(this->jmg, c.tcp_pose, "tool0", this->params.ik_timeout);
			if (c.reachable)
				this->robot_state->copyJointGroupPositions(this->jmg, c.joint_solution);

			if (i % 10 == 0)
				RCLCPP_INFO(
					this->get_logger(), "IK check %zu/%zu, reachable=%s", i, candidates.size(),
					c.reachable ? "yes" : "no");
		}

		return candidates;
	}

	void exportSelectedViewpoints()
	{
		Json::Value root(Json::arrayValue);

		std::string csv_path = this->params.output_dir + "/selected_viewpoints.csv";
		std::ofstream csv_file(csv_path);
		csv_file << "id,camera_x,camera_y,camera_z,dir_x,dir_y,dir_z,"
					"target_x,target_y,target_z,distance,num_visible_faces\n";

		for (size_t i = 0; i < this->selected.size(); ++i)
		{
			const auto& c = *this->selected[i];

			Json::Value item;
			item["id"] = static_cast<int>(i);
			item["camera_pos"]["x"] = c.camera_pos.x();
			item["camera_pos"]["y"] = c.camera_pos.y();
			item["camera_pos"]["z"] = c.camera_pos.z();
			item["view_dir"]["x"] = c.view_dir.x();
			item["view_dir"]["y"] = c.view_dir.y();
			item["view_dir"]["z"] = c.view_dir.z();
			item["target_point"]["x"] = c.target_point.x();
			item["target_point"]["y"] = c.target_point.y();
			item["target_point"]["z"] = c.target_point.z();
			item["distance"] = c.distance;
			item["num_visible_faces"] = c.num_visible_faces;
			root.append(item);

			csv_file << i << "," << c.camera_pos.x() << "," << c.camera_pos.y() << "," << c.camera_pos.z() << ","
					  << c.view_dir.x() << "," << c.view_dir.y() << "," << c.view_dir.z() << "," << c.target_point.x()
					  << "," << c.target_point.y() << "," << c.target_point.z() << "," << c.distance << ","
					  << c.num_visible_faces << "\n";
		}

		std::string json_path = this->params.output_dir + "/selected_viewpoints.json";
		std::ofstream json_file(json_path);
		Json::StreamWriterBuilder writer_builder;
		writer_builder["indentation"] = "    ";
		std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
		writer->write(root, &json_file);

		RCLCPP_INFO(this->get_logger(), "Saved JSON: %s", json_path.c_str());
		RCLCPP_INFO(this->get_logger(), "Saved CSV: %s", csv_path.c_str());
	}

	void exportSelectedRobotPoses()
	{
		Json::Value root(Json::arrayValue);

		std::string csv_path = this->params.output_dir + "/selected_robot_poses.csv";
		std::ofstream csv_file(csv_path);
		csv_file << "id,tcp_x,tcp_y,tcp_z,qx,qy,qz,qw,camera_x,camera_y,camera_z,"
					"view_dir_x,view_dir_y,view_dir_z,num_visible_faces\n";

		for (size_t i = 0; i < this->selected.size(); ++i)
		{
			const auto& c = *this->selected[i];
			const auto& tcp_pos = c.tcp_pose.position;
			const auto& tcp_quat = c.tcp_pose.orientation;

			Json::Value item;
			item["id"] = static_cast<int>(i);
			item["tcp_position"]["x"] = tcp_pos.x;
			item["tcp_position"]["y"] = tcp_pos.y;
			item["tcp_position"]["z"] = tcp_pos.z;
			item["tcp_quaternion_xyzw"]["x"] = tcp_quat.x;
			item["tcp_quaternion_xyzw"]["y"] = tcp_quat.y;
			item["tcp_quaternion_xyzw"]["z"] = tcp_quat.z;
			item["tcp_quaternion_xyzw"]["w"] = tcp_quat.w;
			item["camera_position"]["x"] = c.camera_pos.x();
			item["camera_position"]["y"] = c.camera_pos.y();
			item["camera_position"]["z"] = c.camera_pos.z();
			item["view_dir"]["x"] = c.view_dir.x();
			item["view_dir"]["y"] = c.view_dir.y();
			item["view_dir"]["z"] = c.view_dir.z();
			item["num_visible_faces"] = c.num_visible_faces;
			root.append(item);

			csv_file << i << "," << tcp_pos.x << "," << tcp_pos.y << "," << tcp_pos.z << "," << tcp_quat.x << ","
					  << tcp_quat.y << "," << tcp_quat.z << "," << tcp_quat.w << "," << c.camera_pos.x() << ","
					  << c.camera_pos.y() << "," << c.camera_pos.z() << "," << c.view_dir.x() << "," << c.view_dir.y()
					  << "," << c.view_dir.z() << "," << c.num_visible_faces << "\n";
		}

		std::string json_path = this->params.output_dir + "/selected_robot_poses.json";
		std::ofstream json_file(json_path);
		Json::StreamWriterBuilder writer_builder;
		writer_builder["indentation"] = "    ";
		std::unique_ptr<Json::StreamWriter> writer(writer_builder.newStreamWriter());
		writer->write(root, &json_file);

		RCLCPP_INFO(this->get_logger(), "Saved robot poses JSON: %s", json_path.c_str());
		RCLCPP_INFO(this->get_logger(), "Saved robot poses CSV: %s", csv_path.c_str());
	}

	visualization_msgs::msg::MarkerArray buildMarkerArray() const
	{
		visualization_msgs::msg::MarkerArray markers;

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);

		const double arrow_length = 0.02;
		int id = 0;
		auto stamp = this->now();

		// The mesh itself, so it renders alongside the viewpoints (mirrors trimesh's
		// semi-transparent gray preview in visualize_views()).
		Eigen::Quaterniond object_quat(object_rotation_world);
		visualization_msgs::msg::Marker mesh_marker;
		mesh_marker.header.frame_id = "world";
		mesh_marker.header.stamp = stamp;
		mesh_marker.ns = "object_mesh";
		mesh_marker.id = id++;
		mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		mesh_marker.action = visualization_msgs::msg::Marker::ADD;
		mesh_marker.mesh_resource = "file://" + this->resolved_mesh_path;
		mesh_marker.mesh_use_embedded_materials = false;
		mesh_marker.pose.position = ToPoint(object_translation_world);
		mesh_marker.pose.orientation.x = object_quat.x();
		mesh_marker.pose.orientation.y = object_quat.y();
		mesh_marker.pose.orientation.z = object_quat.z();
		mesh_marker.pose.orientation.w = object_quat.w();
		mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = this->params.mesh_scale;
		mesh_marker.color.r = 0.7f;
		mesh_marker.color.g = 0.7f;
		mesh_marker.color.b = 0.7f;
		mesh_marker.color.a = 0.5f;
		markers.markers.push_back(mesh_marker);

		auto addCandidateMarkers = [&](const ViewpointCandidate& c, const std::string& ns, float r, float g, float b) {
			Eigen::Vector3d world_pos = object_rotation_world * c.camera_pos + object_translation_world;
			Eigen::Vector3d world_dir = (object_rotation_world * c.view_dir).normalized();

			visualization_msgs::msg::Marker sphere;
			sphere.header.frame_id = "world";
			sphere.header.stamp = stamp;
			sphere.ns = ns + "_camera_pos";
			sphere.id = id++;
			sphere.type = visualization_msgs::msg::Marker::SPHERE;
			sphere.action = visualization_msgs::msg::Marker::ADD;
			sphere.pose.position = ToPoint(world_pos);
			sphere.pose.orientation.w = 1.0;
			sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.006;
			sphere.color.r = r;
			sphere.color.g = g;
			sphere.color.b = b;
			sphere.color.a = 1.0;
			markers.markers.push_back(sphere);

			visualization_msgs::msg::Marker arrow;
			arrow.header.frame_id = "world";
			arrow.header.stamp = stamp;
			arrow.ns = ns + "_view_dir";
			arrow.id = id++;
			arrow.type = visualization_msgs::msg::Marker::ARROW;
			arrow.action = visualization_msgs::msg::Marker::ADD;
			arrow.points.push_back(ToPoint(world_pos));
			arrow.points.push_back(ToPoint(world_pos + world_dir * arrow_length));
			arrow.scale.x = 0.001; // shaft diameter
			arrow.scale.y = 0.003; // head diameter
			arrow.scale.z = 0.0;	// auto head length
			arrow.color.r = r;
			arrow.color.g = g;
			arrow.color.b = b;
			arrow.color.a = 1.0;
			markers.markers.push_back(arrow);
		};

		for (const auto* c : this->selected)
			addCandidateMarkers(*c, "selected", 0.0f, 1.0f, 0.0f);

		for (const auto& c : this->unreachable_visible)
			addCandidateMarkers(c, "unreachable", 1.0f, 0.5f, 0.0f);

		// Visiting order (nearest-neighbor + 2-opt tour from the robot's home position).
		if (this->selected.size() >= 2)
		{
			visualization_msgs::msg::Marker tour;
			tour.header.frame_id = "world";
			tour.header.stamp = stamp;
			tour.ns = "selected_tour";
			tour.id = id++;
			tour.type = visualization_msgs::msg::Marker::LINE_STRIP;
			tour.action = visualization_msgs::msg::Marker::ADD;
			tour.scale.x = 0.0015; // line width
			tour.color.r = 1.0f;
			tour.color.g = 1.0f;
			tour.color.b = 0.0f;
			tour.color.a = 0.8f;

			for (const auto* c : this->selected)
			{
				Eigen::Vector3d world_pos = object_rotation_world * c->camera_pos + object_translation_world;
				tour.points.push_back(ToPoint(world_pos));
			}
			markers.markers.push_back(tour);
		}

		return markers;
	}

	// Original mesh (offset by mesh_comparison_offset) and simplified/decimated mesh (at the
	// object's actual pose) side by side, on their own topic separate from /viewpoint_markers.
	visualization_msgs::msg::MarkerArray buildMeshComparisonMarkers() const
	{
		visualization_msgs::msg::MarkerArray markers;

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);
		Eigen::Vector3d comparison_offset =
			ToVector3(this->params.mesh_comparison_offset, Eigen::Vector3d(0.0, 0.3, 0.0));

		Eigen::Quaterniond object_quat(object_rotation_world);
		auto stamp = this->now();
		int id = 0;

		// Original mesh, at the object's real pose (unshifted). resolved_mesh_path points at the
		// raw, unscaled STL, so mesh_scale still needs to be applied here.
		visualization_msgs::msg::Marker original_marker;
		original_marker.header.frame_id = "world";
		original_marker.header.stamp = stamp;
		original_marker.ns = "original_mesh";
		original_marker.id = id++;
		original_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		original_marker.action = visualization_msgs::msg::Marker::ADD;
		original_marker.mesh_resource = "file://" + this->resolved_mesh_path;
		original_marker.mesh_use_embedded_materials = false;
		original_marker.pose.position = ToPoint(object_translation_world);
		original_marker.pose.orientation.x = object_quat.x();
		original_marker.pose.orientation.y = object_quat.y();
		original_marker.pose.orientation.z = object_quat.z();
		original_marker.pose.orientation.w = object_quat.w();
		original_marker.scale.x = original_marker.scale.y = original_marker.scale.z = this->params.mesh_scale;
		original_marker.color.r = 0.7f;
		original_marker.color.g = 0.7f;
		original_marker.color.b = 0.7f;
		original_marker.color.a = 0.8f;
		markers.markers.push_back(original_marker);

		// Simplified (decimated) mesh, shifted by mesh_comparison_offset so it doesn't overlap
		// the original. resolved_simplified_mesh_path was exported from the already-scaled
		// in-memory mesh, so no marker-level scale needed.
		visualization_msgs::msg::Marker simplified_marker;
		simplified_marker.header.frame_id = "world";
		simplified_marker.header.stamp = stamp;
		simplified_marker.ns = "simplified_mesh";
		simplified_marker.id = id++;
		simplified_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
		simplified_marker.action = visualization_msgs::msg::Marker::ADD;
		simplified_marker.mesh_resource = "file://" + this->resolved_simplified_mesh_path;
		simplified_marker.mesh_use_embedded_materials = false;
		simplified_marker.pose.position = ToPoint(object_translation_world + comparison_offset);
		simplified_marker.pose.orientation.x = object_quat.x();
		simplified_marker.pose.orientation.y = object_quat.y();
		simplified_marker.pose.orientation.z = object_quat.z();
		simplified_marker.pose.orientation.w = object_quat.w();
		simplified_marker.scale.x = simplified_marker.scale.y = simplified_marker.scale.z = 1.0;
		simplified_marker.color.r = 0.2f;
		simplified_marker.color.g = 0.8f;
		simplified_marker.color.b = 0.2f;
		simplified_marker.color.a = 0.8f;
		markers.markers.push_back(simplified_marker);

		return markers;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ViewpointPlannerNode>();

	// executeTourOnRobot() (when execute_on_robot is true) makes blocking MoveGroupInterface calls
	// (plan()/execute()) from inside init(), which need this node's own callbacks -- action-client
	// goal/result responses, joint state updates for startStateMonitor -- serviced concurrently, or
	// they hang forever. Spin on a background thread from the start (rather than after init()
	// returns, as before) so init() can safely make those calls on the main thread.
	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
