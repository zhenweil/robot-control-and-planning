#include <cmath>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/hierarchical_tour.hpp"
#include "panda_arm_control/mesh_utils.hpp"
#include "panda_arm_control/potential_field_sampler.hpp"
#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_io.hpp"
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
	// Only used when candidate_generation_method is "grid". tilt/tip are two orthogonal swing
	// angles away from straight-on the surface normal -- see GenerateViewCandidates.
	std::vector<double> tilt_angles_deg = {0.0, 15.0, -15.0};
	std::vector<double> tip_angles_deg = {0.0, 15.0, -15.0};
	// "grid" (GenerateViewCandidates: standoff/tilt grid per surface point) or "potential_field"
	// (GeneratePotentialFieldViewCandidates: view direction from a mesh-attraction field).
	std::string candidate_generation_method = "grid";
	double min_clearance = 0.005;
	double fov_deg = 30.0;
	double max_distance = 20.0;
	double angle_threshold_deg = 70.0;
	int max_rays_per_view = 2000;
	double target_area_visibility = 0.95;
	double ik_timeout = 0.1;
	int random_seed = 42;
	std::string group_name = "panda_arm";
	std::vector<double> object_translation_world = {0.2, 0.2, 0.38};
	std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
	// Shared with WaypointFollower via config/object_pose.yaml -- proven-safe "ready" config.
	// Used as the tour's home reference, and (if execute_on_robot) where the robot returns to.
	std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};
	std::vector<double> t_tcp_camera_xyz = {0.0, 0.0, 0.0};
	std::vector<double> t_tcp_camera_quat_xyzw = {0.0, 0.0, 0.0, 1.0};
	// Tour cost is cartesian_distance_m + joint_distance_weight * joint_distance_rad, both from
	// real planned trajectories -- trades off Cartesian travel against joint motion.
	double joint_distance_weight = 0.1;
	// Applied only to the original mesh's comparison-marker position (not the object's actual
	// pose), so the original and simplified meshes render side by side instead of overlapping.
	std::vector<double> mesh_comparison_offset = {0.0, 0.3, 0.0};
	// Below this many selected viewpoints, SolveHierarchicalTour skips clustering and runs flat
	// NN+2-opt over all of them (matches viewpoint_planner_nn2opt).
	int hgtsp_min_size_for_hierarchy = 8;
	// "alpha" in the cheap clustering-only proxy metric used to pick exemplars/cluster membership.
	double hgtsp_exemplar_metric_rotation_weight = 0.1;
	// Affinity propagation hyperparameters -- see hierarchical_tour.hpp for what each controls.
	// See hierarchical_tour.hpp: 0.9-0.95 (not the sklearn-typical median) is needed for
	// tightly-clustered viewpoints to avoid affinity propagation collapsing to one exemplar.
	double hgtsp_ap_preference_quantile = 0.95;
	double hgtsp_ap_damping = 0.9;
	int hgtsp_ap_max_iterations = 200;
	int hgtsp_ap_convergence_iterations = 15;
	// Per-pair timeout for the small exemplar-only real cost matrix (upper-level guide path).
	double hgtsp_guide_path_planning_time = 2.0;
	// Per-pair timeout for each small per-cluster real cost matrix (lower-level local refinement).
	double hgtsp_cluster_planning_time = 2.0;
	// Redundant-IK tour ordering: each candidate can offer up to this many joint_solutions, letting
	// the solver pick whichever minimizes reconfiguration. 1 = today's single-solution behavior.
	int hgtsp_max_solutions_per_candidate = 3;
	// Extra penalty on the single largest per-joint swing on a leg, beyond joint_distance_weight's
	// combined L2 norm -- targets "one joint does a big twist" specifically.
	double hgtsp_max_joint_deviation_weight = 1.0;
	// When true, re-plans the tour's legs and drives the robot directly via move_group, instead
	// of publishing waypoints. Don't also run waypoint_follower -- uncoordinated with this.
	bool execute_on_robot = true;
	// Planning time/attempts for PlanFinalTourTrajectories -- affordable to be more generous than
	// hgtsp_cluster_planning_time since this only runs once per tour leg (selected.size() calls),
	// not O(n^2).
	double execution_planning_time = 5.0;
	int execution_planning_attempts = 5;
	std::string output_dir = "/tmp/viewpoint_planner_output";
};

} // namespace

class ViewpointPlannerHgtspNode : public rclcpp::Node
{
public:
	explicit ViewpointPlannerHgtspNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node(
			  "viewpoint_planner_hgtsp",
			  rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
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

		// initial_joints (shared via object_pose.yaml) is the proven-safe home config, not the
		// URDF's raw defaults -- those can collide with the registered object.
		this->robot_state->setToDefaultValues();
		this->robot_state->setJointGroupPositions(this->jmg, this->params.initial_joints);

		// Capture the home joint values now, before reachability computation mutates robot_state
		// with IK solutions -- this is the reference point (matrix index 0) every cost/tour
		// computation starts from.
		this->robot_state->update();
		this->robot_state->copyJointGroupPositions(this->jmg, this->home_joint_values);

		this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/viewpoint_markers", rclcpp::QoS(1).transient_local());

		this->mesh_comparison_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/mesh_comparison_markers", rclcpp::QoS(1).transient_local());

		this->coverage_gap_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/coverage_gap_markers", rclcpp::QoS(1).transient_local());

		this->exemplar_cluster_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/exemplar_cluster_markers", rclcpp::QoS(1).transient_local());

		this->waypoint_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
			"/cartesian_waypoints", rclcpp::QoS(1).transient_local());

		if (this->params.execute_on_robot)
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
	std::vector<double> home_joint_values;

	std::vector<ViewpointCandidate> reachable_candidates;
	std::vector<ViewpointCandidate> unreachable_visible;
	std::vector<const ViewpointCandidate*> selected;
	std::string resolved_mesh_path;
	std::string resolved_simplified_mesh_path;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_comparison_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_gap_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr exemplar_cluster_pub;
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
		this->declareIfNeeded("tip_angles_deg", this->params.tip_angles_deg);
		this->declareIfNeeded("candidate_generation_method", this->params.candidate_generation_method);
		this->declareIfNeeded("min_clearance", this->params.min_clearance);
		this->declareIfNeeded("fov_deg", this->params.fov_deg);
		this->declareIfNeeded("max_distance", this->params.max_distance);
		this->declareIfNeeded("angle_threshold_deg", this->params.angle_threshold_deg);
		this->declareIfNeeded("max_rays_per_view", this->params.max_rays_per_view);
		this->declareIfNeeded("target_area_visibility", this->params.target_area_visibility);
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
		this->declareIfNeeded("hgtsp_min_size_for_hierarchy", this->params.hgtsp_min_size_for_hierarchy);
		this->declareIfNeeded(
			"hgtsp_exemplar_metric_rotation_weight", this->params.hgtsp_exemplar_metric_rotation_weight);
		this->declareIfNeeded("hgtsp_ap_preference_quantile", this->params.hgtsp_ap_preference_quantile);
		this->declareIfNeeded("hgtsp_ap_damping", this->params.hgtsp_ap_damping);
		this->declareIfNeeded("hgtsp_ap_max_iterations", this->params.hgtsp_ap_max_iterations);
		this->declareIfNeeded("hgtsp_ap_convergence_iterations", this->params.hgtsp_ap_convergence_iterations);
		this->declareIfNeeded("hgtsp_guide_path_planning_time", this->params.hgtsp_guide_path_planning_time);
		this->declareIfNeeded("hgtsp_cluster_planning_time", this->params.hgtsp_cluster_planning_time);
		this->declareIfNeeded(
			"hgtsp_max_solutions_per_candidate", this->params.hgtsp_max_solutions_per_candidate);
		this->declareIfNeeded(
			"hgtsp_max_joint_deviation_weight", this->params.hgtsp_max_joint_deviation_weight);
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
		this->get_parameter("tip_angles_deg", this->params.tip_angles_deg);
		this->get_parameter("candidate_generation_method", this->params.candidate_generation_method);
		this->get_parameter("min_clearance", this->params.min_clearance);
		this->get_parameter("fov_deg", this->params.fov_deg);
		this->get_parameter("max_distance", this->params.max_distance);
		this->get_parameter("angle_threshold_deg", this->params.angle_threshold_deg);
		this->get_parameter("max_rays_per_view", this->params.max_rays_per_view);
		this->get_parameter("target_area_visibility", this->params.target_area_visibility);
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
		this->get_parameter("hgtsp_min_size_for_hierarchy", this->params.hgtsp_min_size_for_hierarchy);
		this->get_parameter(
			"hgtsp_exemplar_metric_rotation_weight", this->params.hgtsp_exemplar_metric_rotation_weight);
		this->get_parameter("hgtsp_ap_preference_quantile", this->params.hgtsp_ap_preference_quantile);
		this->get_parameter("hgtsp_ap_damping", this->params.hgtsp_ap_damping);
		this->get_parameter("hgtsp_ap_max_iterations", this->params.hgtsp_ap_max_iterations);
		this->get_parameter("hgtsp_ap_convergence_iterations", this->params.hgtsp_ap_convergence_iterations);
		this->get_parameter("hgtsp_guide_path_planning_time", this->params.hgtsp_guide_path_planning_time);
		this->get_parameter("hgtsp_cluster_planning_time", this->params.hgtsp_cluster_planning_time);
		this->get_parameter("hgtsp_max_solutions_per_candidate", this->params.hgtsp_max_solutions_per_candidate);
		this->get_parameter("hgtsp_max_joint_deviation_weight", this->params.hgtsp_max_joint_deviation_weight);
		this->get_parameter("execute_on_robot", this->params.execute_on_robot);
		this->get_parameter("execution_planning_time", this->params.execution_planning_time);
		this->get_parameter("execution_planning_attempts", this->params.execution_planning_attempts);
		this->get_parameter("output_dir", this->params.output_dir);

		if (this->params.candidate_generation_method != "grid" &&
			this->params.candidate_generation_method != "potential_field")
		{
			RCLCPP_WARN(
				this->get_logger(), "Unknown candidate_generation_method '%s', falling back to 'grid'",
				this->params.candidate_generation_method.c_str());
			this->params.candidate_generation_method = "grid";
		}
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
		if (this->params.candidate_generation_method == "potential_field")
		{
			candidates = GeneratePotentialFieldViewCandidates(
				simplified_mesh, this->params.n_surface_samples, this->params.standoff_distances, rng);
		}
		else
		{
			candidates = GenerateViewCandidates(
				simplified_mesh, this->params.n_surface_samples, this->params.standoff_distances,
				this->params.tilt_angles_deg, rng, this->params.tip_angles_deg);
		}

		RCLCPP_INFO(this->get_logger(), "candidate views: %zu", candidates.size());

		candidates = FilterByClearance(original_poly, std::move(candidates), this->params.min_clearance);

		VisibilityChecker vis_checker(simplified_mesh);
		candidates = ComputeVisibility(
			vis_checker, std::move(candidates), this->params.fov_deg, this->params.max_distance,
			this->params.angle_threshold_deg, this->params.max_rays_per_view);

		RCLCPP_INFO(this->get_logger(), "visible candidate views: %zu", candidates.size());

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);
		Eigen::Isometry3d t_tcp_camera =
			IsometryFromXyzQuat(this->params.t_tcp_camera_xyz, this->params.t_tcp_camera_quat_xyzw);

		planning_scene_monitor::PlanningSceneMonitorPtr local_scene = BuildLocalCollisionScene(
			this->shared_from_this(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world);

		candidates = ComputeReachabilityWithCollisionCheck(
			std::move(candidates), this->robot_model, this->params.group_name, local_scene, object_translation_world,
			object_rotation_world, t_tcp_camera, this->params.ik_timeout);

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

		this->selected = GreedySelectViewpoints(
			simplified_mesh, this->reachable_candidates, this->params.target_area_visibility);

		RCLCPP_INFO(this->get_logger(), "selected views: %zu", this->selected.size());

		// Only collect alternates for the (small) selected set, not every reachable candidate --
		// most reachable candidates never get used, so retries there would be wasted work.
		CollectAlternateJointSolutions(
			this->selected, this->robot_model, this->params.group_name, local_scene, object_translation_world,
			object_rotation_world, t_tcp_camera, this->params.ik_timeout,
			this->params.hgtsp_max_solutions_per_candidate);

		HierarchicalTourParams hgtsp_params;
		hgtsp_params.min_size_for_hierarchy = this->params.hgtsp_min_size_for_hierarchy;
		hgtsp_params.exemplar_metric_rotation_weight = this->params.hgtsp_exemplar_metric_rotation_weight;
		hgtsp_params.ap_preference_quantile = this->params.hgtsp_ap_preference_quantile;
		hgtsp_params.ap_damping = this->params.hgtsp_ap_damping;
		hgtsp_params.ap_max_iterations = this->params.hgtsp_ap_max_iterations;
		hgtsp_params.ap_convergence_iterations = this->params.hgtsp_ap_convergence_iterations;
		hgtsp_params.joint_distance_weight = this->params.joint_distance_weight;
		hgtsp_params.max_joint_deviation_weight = this->params.hgtsp_max_joint_deviation_weight;
		hgtsp_params.guide_path_planning_time = this->params.hgtsp_guide_path_planning_time;
		hgtsp_params.cluster_planning_time = this->params.hgtsp_cluster_planning_time;
		hgtsp_params.max_solutions_per_candidate = this->params.hgtsp_max_solutions_per_candidate;

		RCLCPP_INFO(
			this->get_logger(), "Ordering %zu selected views via hierarchical guide-path tour (affinity "
								 "propagation + real per-cluster cost matrices, inspired by H-Joint-GTSP, "
								 "arXiv:2502.19591)...",
			this->selected.size());

		HierarchicalTourDebug hierarchical_debug;
		this->selected = SolveHierarchicalTour(
			this->shared_from_this(), this->robot_model, local_scene, std::move(this->selected),
			this->home_joint_values, this->params.group_name, hgtsp_params, &hierarchical_debug);

		double achieved_visibility = ComputeAreaVisibility(simplified_mesh, this->selected);
		RCLCPP_INFO(
			this->get_logger(), "hierarchical guide-path tour ordered %zu selected views, %.2f%% area visibility",
			this->selected.size(), achieved_visibility * 100.0);

		ExportSelectedViewpoints(this->params.output_dir, this->selected);
		ExportSelectedRobotPoses(this->params.output_dir, this->selected);

		this->marker_array = BuildViewpointMarkerArray(
			this->now(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world, this->selected, this->unreachable_visible);
		this->marker_pub->publish(this->marker_array);

		this->mesh_comparison_pub->publish(BuildMeshComparisonMarkerArray(
			this->now(), this->resolved_mesh_path, this->resolved_simplified_mesh_path, this->params.mesh_scale,
			object_translation_world, object_rotation_world,
			ToVector3(this->params.mesh_comparison_offset, Eigen::Vector3d(0.0, 0.3, 0.0))));

		// Red/green mesh overlay showing exactly which faces GreedySelectViewpoints did/didn't
		// cover -- diagnostic for why achieved_visibility falls short of target_area_visibility.
		this->coverage_gap_pub->publish(BuildCoverageGapMarkerArray(
			this->now(), simplified_mesh, object_translation_world, object_rotation_world, this->selected));

		// Exemplar (cube) + its cluster members (spheres), spoke-connected and color-coded per cluster.
		this->exemplar_cluster_pub->publish(BuildExemplarClusterMarkerArray(
			this->now(), object_translation_world, object_rotation_world, hierarchical_debug));

		if (this->selected.empty())
		{
			RCLCPP_WARN(
				this->get_logger(), "No reachable/selected viewpoints -- nothing to publish on /cartesian_waypoints");
		}
		else
		{
			geometry_msgs::msg::PoseArray waypoint_msg = BuildWaypointPoseArray(this->now(), this->selected);
			this->waypoint_pub->publish(waypoint_msg);
			RCLCPP_INFO(
				this->get_logger(), "Published %zu TCP waypoints to /cartesian_waypoints", waypoint_msg.poses.size());
		}

		// Runs after publishing markers/waypoints (not before) so RViz shows the planned mesh/tour
		// before the real robot is committed to executing it, rather than only appearing once
		// execution -- which blocks for the whole tour -- has already finished.
		if (this->params.execute_on_robot)
		{
			ExecuteTourOnRobot(
				this->shared_from_this(), this->robot_model, local_scene, this->move_group, this->marker_pub,
				this->selected, this->home_joint_values, this->params.group_name,
				this->params.execution_planning_time, this->params.execution_planning_attempts);
		}
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ViewpointPlannerHgtspNode>();

	// ExecuteTourOnRobot's blocking MoveGroupInterface calls need this node's own callbacks
	// serviced concurrently or they hang. Spin on a background thread from the start.
	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
