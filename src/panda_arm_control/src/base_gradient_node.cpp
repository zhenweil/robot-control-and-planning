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

#include "panda_arm_control/base_gradient.hpp"
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
	// Directory a prior viewpoint_planner_* run exported selected_robot_poses.json into.
	std::string tour_input_dir = "/tmp/viewpoint_planner_output";
	std::string output_dir = "/tmp/base_gradient_output";

	double bg_x_min = -0.3, bg_x_max = 0.3;
	double bg_y_min = -0.3, bg_y_max = 0.3;
	double bg_z_min = -0.2, bg_z_max = 0.2;
	double bg_roll_min = -0.35, bg_roll_max = 0.35;	  // rad (~20 deg) -- object tip
	double bg_pitch_min = -0.35, bg_pitch_max = 0.35;  // rad -- object tilt
	double bg_initial_x = 0.0, bg_initial_y = 0.0, bg_initial_z = 0.0;
	double bg_initial_roll = 0.0, bg_initial_pitch = 0.0;
	double bg_rot_metric_scale = 0.3;  // m per rad, blends translation & tip/tilt in the step

	double bg_joint_distance_weight = 1.0;
	double bg_cartesian_distance_weight = 0.0;
	double bg_max_joint_deviation_weight = 1.0;
	double bg_unreachable_penalty = 50.0;

	int bg_max_solutions_per_candidate = 2;
	int bg_ik_retries_per_point = 8;
	int bg_gtsp_two_opt_rounds = 5;

	int bg_num_restarts = 4;
	int bg_restart_patience = 2;
	int bg_max_outer_iterations = 15;
	double bg_initial_step = 0.05;
	double bg_step_shrink = 0.5;
	double bg_armijo_c = 1e-4;
	double bg_min_step = 1e-4;
	int bg_max_line_search_iters = 8;
	double bg_jacobian_damping = 1e-3;

	double bg_convergence_tolerance_offset = 0.002;
	double bg_convergence_tolerance_cost = 1e-3;
	int bg_patience = 3;

	bool bg_fd_gradient_check = false;
	double bg_fd_epsilon = 1e-4;

	double ik_timeout = 0.1;
	int random_seed = 42;
	double visualize_progress_delay_sec = 0.0;

	// Drives the real robot through the recommended placement's tour by moving the (software-only)
	// collision object into the frame the base would see at the recommended offset. Only correct
	// if that matches reality -- see ApplyObjectOffsetToScene's doc comment. Defaults to false.
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

}  // namespace

class BaseGradientNode : public rclcpp::Node
{
public:
	explicit BaseGradientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
		: Node("base_gradient", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
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
			"/base_gradient_markers", rclcpp::QoS(1).transient_local());
		this->progress_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"/base_gradient_progress_markers", rclcpp::QoS(1).transient_local());

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
		this->declareIfNeeded("bg_x_min", this->params.bg_x_min);
		this->declareIfNeeded("bg_x_max", this->params.bg_x_max);
		this->declareIfNeeded("bg_y_min", this->params.bg_y_min);
		this->declareIfNeeded("bg_y_max", this->params.bg_y_max);
		this->declareIfNeeded("bg_z_min", this->params.bg_z_min);
		this->declareIfNeeded("bg_z_max", this->params.bg_z_max);
		this->declareIfNeeded("bg_roll_min", this->params.bg_roll_min);
		this->declareIfNeeded("bg_roll_max", this->params.bg_roll_max);
		this->declareIfNeeded("bg_pitch_min", this->params.bg_pitch_min);
		this->declareIfNeeded("bg_pitch_max", this->params.bg_pitch_max);
		this->declareIfNeeded("bg_initial_x", this->params.bg_initial_x);
		this->declareIfNeeded("bg_initial_y", this->params.bg_initial_y);
		this->declareIfNeeded("bg_initial_z", this->params.bg_initial_z);
		this->declareIfNeeded("bg_initial_roll", this->params.bg_initial_roll);
		this->declareIfNeeded("bg_initial_pitch", this->params.bg_initial_pitch);
		this->declareIfNeeded("bg_rot_metric_scale", this->params.bg_rot_metric_scale);
		this->declareIfNeeded("bg_joint_distance_weight", this->params.bg_joint_distance_weight);
		this->declareIfNeeded("bg_cartesian_distance_weight", this->params.bg_cartesian_distance_weight);
		this->declareIfNeeded("bg_max_joint_deviation_weight", this->params.bg_max_joint_deviation_weight);
		this->declareIfNeeded("bg_unreachable_penalty", this->params.bg_unreachable_penalty);
		this->declareIfNeeded("bg_max_solutions_per_candidate", this->params.bg_max_solutions_per_candidate);
		this->declareIfNeeded("bg_ik_retries_per_point", this->params.bg_ik_retries_per_point);
		this->declareIfNeeded("bg_gtsp_two_opt_rounds", this->params.bg_gtsp_two_opt_rounds);
		this->declareIfNeeded("bg_num_restarts", this->params.bg_num_restarts);
		this->declareIfNeeded("bg_restart_patience", this->params.bg_restart_patience);
		this->declareIfNeeded("bg_max_outer_iterations", this->params.bg_max_outer_iterations);
		this->declareIfNeeded("bg_initial_step", this->params.bg_initial_step);
		this->declareIfNeeded("bg_step_shrink", this->params.bg_step_shrink);
		this->declareIfNeeded("bg_armijo_c", this->params.bg_armijo_c);
		this->declareIfNeeded("bg_min_step", this->params.bg_min_step);
		this->declareIfNeeded("bg_max_line_search_iters", this->params.bg_max_line_search_iters);
		this->declareIfNeeded("bg_jacobian_damping", this->params.bg_jacobian_damping);
		this->declareIfNeeded("bg_convergence_tolerance_offset", this->params.bg_convergence_tolerance_offset);
		this->declareIfNeeded("bg_convergence_tolerance_cost", this->params.bg_convergence_tolerance_cost);
		this->declareIfNeeded("bg_patience", this->params.bg_patience);
		this->declareIfNeeded("bg_fd_gradient_check", this->params.bg_fd_gradient_check);
		this->declareIfNeeded("bg_fd_epsilon", this->params.bg_fd_epsilon);
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
		this->get_parameter("bg_x_min", this->params.bg_x_min);
		this->get_parameter("bg_x_max", this->params.bg_x_max);
		this->get_parameter("bg_y_min", this->params.bg_y_min);
		this->get_parameter("bg_y_max", this->params.bg_y_max);
		this->get_parameter("bg_z_min", this->params.bg_z_min);
		this->get_parameter("bg_z_max", this->params.bg_z_max);
		this->get_parameter("bg_roll_min", this->params.bg_roll_min);
		this->get_parameter("bg_roll_max", this->params.bg_roll_max);
		this->get_parameter("bg_pitch_min", this->params.bg_pitch_min);
		this->get_parameter("bg_pitch_max", this->params.bg_pitch_max);
		this->get_parameter("bg_initial_x", this->params.bg_initial_x);
		this->get_parameter("bg_initial_y", this->params.bg_initial_y);
		this->get_parameter("bg_initial_z", this->params.bg_initial_z);
		this->get_parameter("bg_initial_roll", this->params.bg_initial_roll);
		this->get_parameter("bg_initial_pitch", this->params.bg_initial_pitch);
		this->get_parameter("bg_rot_metric_scale", this->params.bg_rot_metric_scale);
		this->get_parameter("bg_joint_distance_weight", this->params.bg_joint_distance_weight);
		this->get_parameter("bg_cartesian_distance_weight", this->params.bg_cartesian_distance_weight);
		this->get_parameter("bg_max_joint_deviation_weight", this->params.bg_max_joint_deviation_weight);
		this->get_parameter("bg_unreachable_penalty", this->params.bg_unreachable_penalty);
		this->get_parameter("bg_max_solutions_per_candidate", this->params.bg_max_solutions_per_candidate);
		this->get_parameter("bg_ik_retries_per_point", this->params.bg_ik_retries_per_point);
		this->get_parameter("bg_gtsp_two_opt_rounds", this->params.bg_gtsp_two_opt_rounds);
		this->get_parameter("bg_num_restarts", this->params.bg_num_restarts);
		this->get_parameter("bg_restart_patience", this->params.bg_restart_patience);
		this->get_parameter("bg_max_outer_iterations", this->params.bg_max_outer_iterations);
		this->get_parameter("bg_initial_step", this->params.bg_initial_step);
		this->get_parameter("bg_step_shrink", this->params.bg_step_shrink);
		this->get_parameter("bg_armijo_c", this->params.bg_armijo_c);
		this->get_parameter("bg_min_step", this->params.bg_min_step);
		this->get_parameter("bg_max_line_search_iters", this->params.bg_max_line_search_iters);
		this->get_parameter("bg_jacobian_damping", this->params.bg_jacobian_damping);
		this->get_parameter("bg_convergence_tolerance_offset", this->params.bg_convergence_tolerance_offset);
		this->get_parameter("bg_convergence_tolerance_cost", this->params.bg_convergence_tolerance_cost);
		this->get_parameter("bg_patience", this->params.bg_patience);
		this->get_parameter("bg_fd_gradient_check", this->params.bg_fd_gradient_check);
		this->get_parameter("bg_fd_epsilon", this->params.bg_fd_epsilon);
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
			mesh_path =
				ament_index_cpp::get_package_share_directory("panda_arm_control") + "/meshes/bunny_holding_eggs.stl";
		this->resolved_mesh_path = mesh_path;

		std::vector<Eigen::Isometry3d> tour_tcp_poses = LoadTourTcpPoses(this->params.tour_input_dir);
		RCLCPP_INFO(
			this->get_logger(), "Loaded %zu tour poses from %s/selected_robot_poses.json", tour_tcp_poses.size(),
			this->params.tour_input_dir.c_str());

		if (tour_tcp_poses.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "No tour poses loaded -- nothing to optimize a base for");
			return;
		}

		Eigen::Vector3d object_translation_world =
			ToVector3(this->params.object_translation_world, Eigen::Vector3d(0.2, 0.2, 0.38));
		Eigen::Matrix3d object_rotation_world = RotationFromRpyDeg(this->params.object_rotation_rpy_deg);

		planning_scene_monitor::PlanningSceneMonitorPtr local_scene = BuildLocalCollisionScene(
			this->shared_from_this(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world);

		BaseGradientParams bg;
		bg.bounds.x_min = this->params.bg_x_min;
		bg.bounds.x_max = this->params.bg_x_max;
		bg.bounds.y_min = this->params.bg_y_min;
		bg.bounds.y_max = this->params.bg_y_max;
		bg.bounds.z_min = this->params.bg_z_min;
		bg.bounds.z_max = this->params.bg_z_max;
		bg.bounds.roll_min = this->params.bg_roll_min;
		bg.bounds.roll_max = this->params.bg_roll_max;
		bg.bounds.pitch_min = this->params.bg_pitch_min;
		bg.bounds.pitch_max = this->params.bg_pitch_max;
		bg.initial_x = this->params.bg_initial_x;
		bg.initial_y = this->params.bg_initial_y;
		bg.initial_z = this->params.bg_initial_z;
		bg.initial_roll = this->params.bg_initial_roll;
		bg.initial_pitch = this->params.bg_initial_pitch;
		bg.rot_metric_scale = this->params.bg_rot_metric_scale;
		bg.joint_distance_weight = this->params.bg_joint_distance_weight;
		bg.cartesian_distance_weight = this->params.bg_cartesian_distance_weight;
		bg.max_joint_deviation_weight = this->params.bg_max_joint_deviation_weight;
		bg.unreachable_penalty = this->params.bg_unreachable_penalty;
		bg.max_solutions_per_candidate = this->params.bg_max_solutions_per_candidate;
		bg.ik_timeout = this->params.ik_timeout;
		bg.ik_retries_per_point = this->params.bg_ik_retries_per_point;
		bg.gtsp_two_opt_rounds = this->params.bg_gtsp_two_opt_rounds;
		bg.num_restarts = this->params.bg_num_restarts;
		bg.restart_patience = this->params.bg_restart_patience;
		bg.max_outer_iterations = this->params.bg_max_outer_iterations;
		bg.initial_step = this->params.bg_initial_step;
		bg.step_shrink = this->params.bg_step_shrink;
		bg.armijo_c = this->params.bg_armijo_c;
		bg.min_step = this->params.bg_min_step;
		bg.max_line_search_iters = this->params.bg_max_line_search_iters;
		bg.jacobian_damping = this->params.bg_jacobian_damping;
		bg.convergence_tolerance_offset = this->params.bg_convergence_tolerance_offset;
		bg.convergence_tolerance_cost = this->params.bg_convergence_tolerance_cost;
		bg.patience = this->params.bg_patience;
		bg.random_seed = this->params.random_seed;
		bg.fd_gradient_check = this->params.bg_fd_gradient_check;
		bg.fd_epsilon = this->params.bg_fd_epsilon;
		bg.progress_pub = this->progress_marker_pub;
		bg.visualize_progress_delay_sec = this->params.visualize_progress_delay_sec;

		RCLCPP_INFO(
			this->get_logger(), "Descending the object offset (x, y, z, tip, tilt) for a %zu-pose tour...",
			tour_tcp_poses.size());

		BaseGradientResult result = SolveBaseGradient(
			this->shared_from_this(), this->robot_model, local_scene, this->params.group_name, object_translation_world,
			object_rotation_world, tour_tcp_poses, this->home_joint_values, bg);

		ExportBaseGradientResult(this->params.output_dir, result);

		this->marker_array = BuildBaseGradientMarkerArray(
			this->now(), this->resolved_mesh_path, this->params.mesh_scale, object_translation_world,
			object_rotation_world, tour_tcp_poses, result);
		this->marker_pub->publish(this->marker_array);

		if (!result.ok)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"Best base offset reaches only %d/%d tour poses -- not executing even if execute_on_robot is set",
				result.num_reachable, result.num_total);
			return;
		}

		if (this->params.execute_on_robot)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"execute_on_robot: driving the REAL robot against the object moved by (%.4f, %.4f, %.4f) m + tip "
				"%.2f deg / tilt %.2f deg in software -- only correct if the physical object is actually fixtured "
				"to match.",
				result.x, result.y, result.z, result.roll * 180.0 / M_PI, result.pitch * 180.0 / M_PI);

			ApplyObjectOffsetToScene(
				local_scene, object_translation_world, object_rotation_world, result.x, result.y, result.z,
				result.roll, result.pitch);

			Eigen::Isometry3d object_offset = Eigen::Isometry3d::Identity();
			object_offset.translation() = Eigen::Vector3d(result.x, result.y, result.z);
			object_offset.linear() = (Eigen::AngleAxisd(result.pitch, Eigen::Vector3d::UnitY()) *
									  Eigen::AngleAxisd(result.roll, Eigen::Vector3d::UnitX()))
										 .toRotationMatrix();

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
				this->shared_from_this(), this->robot_model, local_scene, this->move_group, this->marker_pub, selected,
				this->home_joint_values, this->params.group_name, this->params.execution_planning_time,
				this->params.execution_planning_attempts);
		}
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BaseGradientNode>();

	std::thread spin_thread([node]() { rclcpp::spin(node); });

	node->init();

	spin_thread.join();
	rclcpp::shutdown();
	return 0;
}
