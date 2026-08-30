#include <algorithm>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "panda_arm_control/base_placement.hpp"
#include "panda_arm_control/real_cost_planning.hpp"
#include "panda_arm_control/viewpoint_io.hpp"
#include "panda_arm_control/viewpoint_types.hpp"

namespace
{

struct Params
{
	std::string mesh_path;
	double mesh_scale = 0.01;
	std::string group_name = "panda_arm";
	std::vector<double> object_translation_world = {0.2, 0.2, 0.38};
	std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
	std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};
	// Directory a prior viewpoint_planner_* run exported selected_robot_poses.json into --
	// that ordered tour is this node's input.
	std::string tour_input_dir = "/tmp/viewpoint_planner_output";
	std::string output_dir = "/tmp/base_placement_output";

	// Object-offset search box (m), relative to the object's nominal pose. Rotation held at 0.
	double bp_x_min = -0.15, bp_x_max = 0.15;
	double bp_y_min = -0.15, bp_y_max = 0.15;
	double bp_z_min = -0.15, bp_z_max = 0.15;
	int bp_num_restarts = 3;
	int bp_max_outer_iterations = 10;
	int bp_candidates_per_point = 16;
	int bp_ik_retries_per_point = 4;
	double bp_initial_step = 0.06;
	double bp_step_shrink = 0.5;
	double bp_min_step = 0.005;
	double bp_convergence_tolerance_cost = 1e-2;
	// solve_restarts for the ScoreObjectOffset cost -- 1 (default) trades some noise for speed;
	// 2 matches the placement experiment's careful runs.
	int bp_score_solve_restarts = 1;
	double ik_timeout = 0.15;
	int random_seed = 42;
	// Seconds to pause after each refinement-loop iteration's progress publish, so the
	// convergence can actually be watched in RViz on /base_placement_progress_markers. 0.0
	// (default) adds no delay -- iterations still publish, just at full search speed.
	double visualize_progress_delay_sec = 0.0;

	// Drives the real robot through the recommended placement's tour by moving the *object*
	// (in this local scene, not physically) into the frame the base would see if it had actually
	// been placed at the recommended offset. Only turn this on once you've verified that matches
	// reality -- see ApplyBasePlacementToScene's doc comment. Defaults to false since driving the
	// real robot relative to a hypothetical, unverified object placement is unsafe.
	bool execute_on_robot = false;
	double execution_planning_time = 5.0;
	int execution_planning_attempts = 5;
};

std::vector<Eigen::Isometry3d> LoadTourTcpPoses(const std::string& tour_input_dir)
{
	std::string json_path = tour_input_dir + "/selected_robot_poses.json";
	std::ifstream file(json_path);
	if (!file.is_open())
		throw std::runtime_error("Could not open tour input file: " + json_path);

	Json::Value root;
	Json::CharReaderBuilder reader_builder;
	std::string errs;
	if (!Json::parseFromStream(reader_builder, file, &root, &errs))
		throw std::runtime_error("Failed to parse " + json_path + ": " + errs);

	std::vector<std::pair<int, Eigen::Isometry3d>> entries;
	for (const auto& item : root)
	{
		int id = item["id"].asInt();
		const auto& pos = item["tcp_position"];
		const auto& quat = item["tcp_quaternion_xyzw"];

		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		pose.translation() = Eigen::Vector3d(pos["x"].asDouble(), pos["y"].asDouble(), pos["z"].asDouble());
		pose.linear() = Eigen::Quaterniond(
							 quat["w"].asDouble(), quat["x"].asDouble(), quat["y"].asDouble(), quat["z"].asDouble())
							 .toRotationMatrix();
		entries.emplace_back(id, pose);
	}

	std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	std::vector<Eigen::Isometry3d> poses;
	poses.reserve(entries.size());
	for (const auto& e : entries)
		poses.push_back(e.second);
	return poses;
}

} // namespace

class BasePlacementNode : public rclcpp::Node
{
public:
	explicit BasePlacementNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("base_placement", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
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

		this->robot_state->setToDefaultValues();
		this->robot_state->setJointGroupPositions(this->jmg, this->params.initial_joints);
		this->robot_state->update();
		this->robot_state->copyJointGroupPositions(this->jmg, this->home_joint_values);

		this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/base_placement_markers", rclcpp::QoS(1).transient_local());

		this->progress_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/base_placement_progress_markers", rclcpp::QoS(1).transient_local());

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
	std::string resolved_mesh_path;

	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr progress_marker_pub;
	rclcpp::TimerBase::SharedPtr marker_timer;
	visualization_msgs::msg::MarkerArray marker_array;

	moveit::planning_interface::MoveGroupInterfacePtr move_group;

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
		this->declareIfNeeded("group_name", this->params.group_name);
		this->declareIfNeeded("object_translation_world", this->params.object_translation_world);
		this->declareIfNeeded("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->declareIfNeeded("initial_joints", this->params.initial_joints);
		this->declareIfNeeded("tour_input_dir", this->params.tour_input_dir);
		this->declareIfNeeded("output_dir", this->params.output_dir);
		this->declareIfNeeded("bp_x_min", this->params.bp_x_min);
		this->declareIfNeeded("bp_x_max", this->params.bp_x_max);
		this->declareIfNeeded("bp_y_min", this->params.bp_y_min);
		this->declareIfNeeded("bp_y_max", this->params.bp_y_max);
		this->declareIfNeeded("bp_z_min", this->params.bp_z_min);
		this->declareIfNeeded("bp_z_max", this->params.bp_z_max);
		this->declareIfNeeded("bp_num_restarts", this->params.bp_num_restarts);
		this->declareIfNeeded("bp_max_outer_iterations", this->params.bp_max_outer_iterations);
		this->declareIfNeeded("bp_candidates_per_point", this->params.bp_candidates_per_point);
		this->declareIfNeeded("bp_ik_retries_per_point", this->params.bp_ik_retries_per_point);
		this->declareIfNeeded("bp_initial_step", this->params.bp_initial_step);
		this->declareIfNeeded("bp_step_shrink", this->params.bp_step_shrink);
		this->declareIfNeeded("bp_min_step", this->params.bp_min_step);
		this->declareIfNeeded("bp_convergence_tolerance_cost", this->params.bp_convergence_tolerance_cost);
		this->declareIfNeeded("bp_score_solve_restarts", this->params.bp_score_solve_restarts);
		this->declareIfNeeded("ik_timeout", this->params.ik_timeout);
		this->declareIfNeeded("random_seed", this->params.random_seed);
		this->declareIfNeeded("visualize_progress_delay_sec", this->params.visualize_progress_delay_sec);
		this->declareIfNeeded("execute_on_robot", this->params.execute_on_robot);
		this->declareIfNeeded("execution_planning_time", this->params.execution_planning_time);
		this->declareIfNeeded("execution_planning_attempts", this->params.execution_planning_attempts);
	}

	void loadParameters()
	{
		this->get_parameter("mesh_path", this->params.mesh_path);
		this->get_parameter("mesh_scale", this->params.mesh_scale);
		this->get_parameter("group_name", this->params.group_name);
		this->get_parameter("object_translation_world", this->params.object_translation_world);
		this->get_parameter("object_rotation_rpy_deg", this->params.object_rotation_rpy_deg);
		this->get_parameter("initial_joints", this->params.initial_joints);
		this->get_parameter("tour_input_dir", this->params.tour_input_dir);
		this->get_parameter("output_dir", this->params.output_dir);
		this->get_parameter("bp_x_min", this->params.bp_x_min);
		this->get_parameter("bp_x_max", this->params.bp_x_max);
		this->get_parameter("bp_y_min", this->params.bp_y_min);
		this->get_parameter("bp_y_max", this->params.bp_y_max);
		this->get_parameter("bp_z_min", this->params.bp_z_min);
		this->get_parameter("bp_z_max", this->params.bp_z_max);
		this->get_parameter("bp_num_restarts", this->params.bp_num_restarts);
		this->get_parameter("bp_max_outer_iterations", this->params.bp_max_outer_iterations);
		this->get_parameter("bp_candidates_per_point", this->params.bp_candidates_per_point);
		this->get_parameter("bp_ik_retries_per_point", this->params.bp_ik_retries_per_point);
		this->get_parameter("bp_initial_step", this->params.bp_initial_step);
		this->get_parameter("bp_step_shrink", this->params.bp_step_shrink);
		this->get_parameter("bp_min_step", this->params.bp_min_step);
		this->get_parameter("bp_convergence_tolerance_cost", this->params.bp_convergence_tolerance_cost);
		this->get_parameter("bp_score_solve_restarts", this->params.bp_score_solve_restarts);
		this->get_parameter("ik_timeout", this->params.ik_timeout);
		this->get_parameter("random_seed", this->params.random_seed);
		this->get_parameter("visualize_progress_delay_sec", this->params.visualize_progress_delay_sec);
		this->get_parameter("execute_on_robot", this->params.execute_on_robot);
		this->get_parameter("execution_planning_time", this->params.execution_planning_time);
		this->get_parameter("execution_planning_attempts", this->params.execution_planning_attempts);
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

		std::vector<Eigen::Isometry3d> tour_tcp_poses = LoadTourTcpPoses(this->params.tour_input_dir);
		RCLCPP_INFO(
			this->get_logger(), "Loaded %zu tour poses from %s/selected_robot_poses.json", tour_tcp_poses.size(),
			this->params.tour_input_dir.c_str());

		if (tour_tcp_poses.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No tour poses loaded -- nothing to place a base for");
			return;
		}

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);

		planning_scene_monitor::PlanningSceneMonitorPtr local_scene = BuildLocalCollisionScene(
			this->shared_from_this(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world);

		BasePlacementParams bp_params;
		bp_params.bounds.x_min = this->params.bp_x_min;
		bp_params.bounds.x_max = this->params.bp_x_max;
		bp_params.bounds.y_min = this->params.bp_y_min;
		bp_params.bounds.y_max = this->params.bp_y_max;
		bp_params.bounds.z_min = this->params.bp_z_min;
		bp_params.bounds.z_max = this->params.bp_z_max;
		bp_params.num_restarts = this->params.bp_num_restarts;
		bp_params.max_outer_iterations = this->params.bp_max_outer_iterations;
		bp_params.candidates_per_point = this->params.bp_candidates_per_point;
		bp_params.ik_retries_per_point = this->params.bp_ik_retries_per_point;
		bp_params.ik_timeout = this->params.ik_timeout;
		bp_params.initial_step = this->params.bp_initial_step;
		bp_params.step_shrink = this->params.bp_step_shrink;
		bp_params.min_step = this->params.bp_min_step;
		bp_params.convergence_tolerance_cost = this->params.bp_convergence_tolerance_cost;
		bp_params.random_seed = this->params.random_seed;
		bp_params.progress_pub = this->progress_marker_pub;
		bp_params.visualize_progress_delay_sec = this->params.visualize_progress_delay_sec;

		// The cost every candidate offset is scored with -- identical to the base_gradient
		// placement experiment: struct defaults are the retuned IK / weight / penalty values;
		// only the offset bounds and (optionally) solve_restarts / ik_timeout differ.
		BaseGradientParams score_params;
		score_params.bounds.x_min = this->params.bp_x_min;
		score_params.bounds.x_max = this->params.bp_x_max;
		score_params.bounds.y_min = this->params.bp_y_min;
		score_params.bounds.y_max = this->params.bp_y_max;
		score_params.bounds.z_min = this->params.bp_z_min;
		score_params.bounds.z_max = this->params.bp_z_max;
		score_params.bounds.roll_min = score_params.bounds.roll_max = 0.0;
		score_params.bounds.pitch_min = score_params.bounds.pitch_max = 0.0;
		score_params.solve_restarts = this->params.bp_score_solve_restarts;
		score_params.ik_timeout = this->params.ik_timeout;
		score_params.random_seed = this->params.random_seed;

		RCLCPP_INFO(
			this->get_logger(), "B* baseline: optimizing the object offset (x,y,z) over %zu viewpoints...",
			tour_tcp_poses.size());

		BasePlacementResult result = SolveBasePlacement(
			this->shared_from_this(), this->robot_model, local_scene, this->params.group_name,
			object_translation_world, object_rotation_world, tour_tcp_poses, this->home_joint_values, bp_params,
			score_params);

		ExportBasePlacementResult(this->params.output_dir, result);

		this->marker_array = BuildBasePlacementMarkerArray(
			this->now(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world, tour_tcp_poses, result);
		this->marker_pub->publish(this->marker_array);

		if (!result.ok)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"Best base placement found only reaches %d/%d tour poses -- not executing even if "
				"execute_on_robot is set",
				result.num_reachable, result.num_total);
			return;
		}

		if (this->params.execute_on_robot)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"execute_on_robot: driving the REAL robot against the object moved by (%.4f, %.4f, %.4f) m in "
				"software -- only correct if the physical object has actually been re-fixtured to match.",
				result.x, result.y, result.z);

			ApplyBasePlacementToScene(
				local_scene, object_translation_world, object_rotation_world, result.x, result.y, result.z);

			Eigen::Isometry3d object_offset = Eigen::Isometry3d::Identity();
			object_offset.translation() = Eigen::Vector3d(result.x, result.y, result.z);

			std::vector<ViewpointCandidate> owned(result.tour_order.size());
			std::vector<const ViewpointCandidate*> selected;
			selected.reserve(result.tour_order.size());
			for (size_t k = 0; k < result.tour_order.size(); ++k)
			{
				Eigen::Isometry3d local_pose = object_offset * tour_tcp_poses[result.tour_order[k]];
				owned[k].tcp_pose.position.x = local_pose.translation().x();
				owned[k].tcp_pose.position.y = local_pose.translation().y();
				owned[k].tcp_pose.position.z = local_pose.translation().z();
				Eigen::Quaterniond q(local_pose.rotation());
				owned[k].tcp_pose.orientation.x = q.x();
				owned[k].tcp_pose.orientation.y = q.y();
				owned[k].tcp_pose.orientation.z = q.z();
				owned[k].tcp_pose.orientation.w = q.w();
				owned[k].joint_solution = result.joint_solutions[k];
				selected.push_back(&owned[k]);
			}

			ExecuteTourOnRobot(
				this->shared_from_this(), this->robot_model, local_scene, this->move_group, this->marker_pub,
				selected, this->home_joint_values, this->params.group_name, this->params.execution_planning_time,
				this->params.execution_planning_attempts);
		}
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BasePlacementNode>();

	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
