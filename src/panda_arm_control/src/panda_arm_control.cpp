#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <visualization_msgs/msg/marker_array.hpp>

class WaypointFollower : public rclcpp::Node
{
	public:
		WaypointFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: Node("waypoint_follower", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
		{}

		void init()
		{
			// Shared with viewpoint_planner via config/object_pose.yaml, so both nodes agree on the
			// robot's actual home configuration -- see registerCollisionObject() below for the same
			// pattern applied to object_translation_world/object_rotation_rpy_deg.
			if (!this->has_parameter("initial_joints"))
				this->declare_parameter("initial_joints", this->initial_joints);
			this->get_parameter("initial_joints", this->initial_joints);

			this->registerCollisionObject();

			this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
				shared_from_this(), "panda_arm");
			this->move_group->startStateMonitor();

			this->planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
				shared_from_this(), "robot_description");
			this->planning_scene_monitor->startSceneMonitor();
			this->planning_scene_monitor->startWorldGeometryMonitor();
			this->planning_scene_monitor->startStateMonitor();
			this->planning_scene_monitor->requestPlanningSceneState();

			this->move_group->setPlanningTime(20.0);
			// RRTConnect is randomized and stops at the first valid path it finds, which can be
			// meandering. Re-running it a few times and keeping the shortest result is much
			// cheaper than switching to an optimizing planner (e.g. RRT*), which would use its
			// full time budget on every single waypoint instead of just a few fast reruns.
			this->move_group->setNumPlanningAttempts(5);
			this->move_group->setMaxVelocityScalingFactor(1.0);
			this->move_group->setMaxAccelerationScalingFactor(1.0);
			this->move_group->setEndEffectorLink("tool0");
			this->sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"/cartesian_waypoints",
				rclcpp::QoS(1).transient_local(),
				std::bind(&WaypointFollower::waypointCallback, this, std::placeholders::_1));
			this->marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
				"/viewpoint_markers", rclcpp::QoS(1).transient_local());
			RCLCPP_INFO(this->get_logger(), "Subscriber created");
			this->worker = std::thread(&WaypointFollower::workerLoop, this);
		}

	private:
		std::thread worker;
		std::mutex waypoint_mutex;
		bool has_new_waypoints = false;
		std::condition_variable waypoint_cv;	
		geometry_msgs::msg::PoseArray latest_waypoints;
		// Fallback default if the initial_joints parameter isn't set (see init()) -- normally
		// overridden by config/object_pose.yaml, shared with viewpoint_planner.
		std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};

		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

		// object_translation_world / object_rotation_rpy_deg are shared with viewpoint_planner
		// via config/object_pose.yaml -- keep them as parameters here rather than hardcoding a
		// second copy, since viewpoint_planner's viewpoint selection and this node's collision
		// object must describe the exact same physical object pose.
		void registerCollisionObject()
		{
			std::vector<double> object_translation_world = {0.0, 0.0, 0.3};
			std::vector<double> object_rotation_rpy_deg = {0.0, 0.0, 0.0};
			if (!this->has_parameter("object_translation_world"))
				this->declare_parameter("object_translation_world", object_translation_world);
			if (!this->has_parameter("object_rotation_rpy_deg"))
				this->declare_parameter("object_rotation_rpy_deg", object_rotation_rpy_deg);
			this->get_parameter("object_translation_world", object_translation_world);
			this->get_parameter("object_rotation_rpy_deg", object_rotation_rpy_deg);

			std::string object_mesh_path = "package://panda_arm_control/meshes/bunny_holding_eggs.stl";
			Eigen::Vector3d scale(0.01, 0.01, 0.01);
			shapes::Mesh* m = shapes::createMeshFromResource(object_mesh_path, scale);

			shape_msgs::msg::Mesh mesh_msg;
			shapes::ShapeMsg mesh_tmp;
			shapes::constructMsgFromShape(m, mesh_tmp);
			mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_tmp);

			// Matches scipy's R.from_euler("xyz", [roll, pitch, yaw]).as_matrix() convention used
			// by viewpoint_planner_node's RotationFromRpyDeg, for consistency.
			double roll = object_rotation_rpy_deg[0] * M_PI / 180.0;
			double pitch = object_rotation_rpy_deg[1] * M_PI / 180.0;
			double yaw = object_rotation_rpy_deg[2] * M_PI / 180.0;
			Eigen::Quaterniond q(
				Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

			geometry_msgs::msg::Pose mesh_pose;
			mesh_pose.position.x = object_translation_world[0];
			mesh_pose.position.y = object_translation_world[1];
			mesh_pose.position.z = object_translation_world[2];
			mesh_pose.orientation.x = q.x();
			mesh_pose.orientation.y = q.y();
			mesh_pose.orientation.z = q.z();
			mesh_pose.orientation.w = q.w();

			moveit_msgs::msg::CollisionObject obj;
			obj.header.frame_id = "world";
			obj.id = "bunny_holding_eggs";
			obj.meshes.push_back(mesh_msg);
			obj.mesh_poses.push_back(mesh_pose);
			obj.operation = obj.ADD;

			moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
			planning_scene_interface.applyCollisionObject(obj);
		}

		bool isStateCollisionFree(
			moveit::core::RobotState* state, const moveit::core::JointModelGroup* group, const double* joint_positions)
		{
			state->setJointGroupPositions(group, joint_positions);
			state->update();

			// Solutions right at a joint limit can pass this check individually but still get
			// rejected as INVALID_MOTION_PLAN later -- OMPL's path simplification/time
			// parameterization can nudge an intermediate trajectory point slightly past the
			// limit even when the endpoint itself is technically in bounds. Requiring some
			// margin here makes setFromIK's retry mechanism keep looking for a different
			// (redundant-DOF) solution with more clearance instead.
			constexpr double kJointLimitMargin = 0.05; // radians
			if (!state->satisfiesBounds(group, kJointLimitMargin))
				return false;

			planning_scene_monitor::LockedPlanningSceneRO locked_scene(this->planning_scene_monitor);
			return locked_scene->isStateValid(*state, group->getName());
		}

		void publishCurrentTargetMarker(const geometry_msgs::msg::Pose& pose)
		{
			visualization_msgs::msg::MarkerArray markers;
			auto stamp = this->now();

			visualization_msgs::msg::Marker sphere;
			sphere.header.frame_id = "world";
			sphere.header.stamp = stamp;
			sphere.ns = "current_target_pos";
			sphere.id = 0;
			sphere.type = visualization_msgs::msg::Marker::SPHERE;
			sphere.action = visualization_msgs::msg::Marker::ADD;
			sphere.pose.position = pose.position;
			sphere.pose.orientation.w = 1.0;
			sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
			sphere.color.r = 1.0f;
			sphere.color.a = 1.0f;
			markers.markers.push_back(sphere);

			// Marker::ARROW in pose-mode (no `points`) points along its local +X axis, but
			// CameraRotationFromViewDir (pose_utils.hpp) makes the TCP frame's local +Z axis the
			// approach/view direction. Compose the pose with a fixed -90deg rotation about Y,
			// which maps local +X onto local +Z, so the rendered arrow follows the real view
			// direction instead of the arbitrary in-plane +X axis.
			Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
			Eigen::Quaterniond q_x_to_z(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()));
			Eigen::Quaterniond q_arrow = q * q_x_to_z;

			visualization_msgs::msg::Marker arrow;
			arrow.header.frame_id = "world";
			arrow.header.stamp = stamp;
			arrow.ns = "current_target_dir";
			arrow.id = 0;
			arrow.type = visualization_msgs::msg::Marker::ARROW;
			arrow.action = visualization_msgs::msg::Marker::ADD;
			arrow.pose.position = pose.position;
			arrow.pose.orientation.x = q_arrow.x();
			arrow.pose.orientation.y = q_arrow.y();
			arrow.pose.orientation.z = q_arrow.z();
			arrow.pose.orientation.w = q_arrow.w();
			arrow.scale.x = 0.03;	// arrow length
			arrow.scale.y = 0.006; // shaft diameter
			arrow.scale.z = 0.006; // head diameter
			arrow.color.r = 1.0f;
			arrow.color.a = 1.0f;
			markers.markers.push_back(arrow);

			this->marker_pub->publish(markers);
		}

		void return_to_initial_pose()
		{
			moveit::planning_interface::MoveGroupInterface::Plan return_plan;
			this->move_group->setStartStateToCurrentState();
			this->move_group->setJointValueTarget(this->initial_joints);
			bool ok = static_cast<bool>(this->move_group->plan(return_plan));

			if (ok)
			{
				RCLCPP_INFO(this->get_logger(), "Returning to initial state");
				this->move_group->execute(return_plan);
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to plan back to initial joint state");
			}
		}

		void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
		{
			{
				std::lock_guard<std::mutex> lock(this->waypoint_mutex);
				this->latest_waypoints = *msg;   // keep only latest
				this->has_new_waypoints = true;
			}
			this->waypoint_cv.notify_one();
		}

		void workerLoop()
		{
			while (rclcpp::ok())
			{
				geometry_msgs::msg::PoseArray msg;
				{
					std::unique_lock<std::mutex> lock(this->waypoint_mutex);
					this->waypoint_cv.wait(lock, [this]() {
						return this->has_new_waypoints || !rclcpp::ok();
					});

					if (!rclcpp::ok())
					{
						return;
					}

					this->has_new_waypoints = false;
				}
				
				auto waypoints = this->convert_msg_to_waypoints(this->latest_waypoints);
				auto current_state = this->move_group->getCurrentState(2.0);
				if (!current_state)
				{
					RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
					continue;
				}

				RCLCPP_INFO(this->get_logger(), "Got current state, received %zu waypoints", waypoints.size());
				
				this->execute_waypoints(waypoints);
			}
		}

		std::vector<geometry_msgs::msg::Pose> convert_msg_to_waypoints(const geometry_msgs::msg::PoseArray& msg)
		{
			std::vector<geometry_msgs::msg::Pose> waypoints;
			waypoints.reserve(msg.poses.size());
			for (const auto& pose : msg.poses)
				waypoints.push_back(pose);

			return waypoints;
		}

		void execute_waypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints)
		{
			this->return_to_initial_pose();

			// Waypoints are already IK-filtered upstream by viewpoint_planner. Plan each one
			// individually via OMPL rather than stringing them into one Cartesian path --
			// consecutive viewpoints can differ enough in position/orientation that straight-line
			// interpolation hits collisions or joint limits, driving computeCartesianPath's
			// fraction too low to ever execute.
			for (size_t i = 0; i < waypoints.size(); ++i)
			{
				this->publishCurrentTargetMarker(waypoints[i]);

				auto current_state = this->move_group->getCurrentState(2.0);
				if (!current_state)
				{
					RCLCPP_WARN(this->get_logger(), "Failed to get current state for waypoint %zu/%zu, skipping", i, waypoints.size());
					continue;
				}

				// Seed IK from the current joint state (rather than letting setPoseTarget's goal
				// sampler pick any solution) so consecutive waypoints stay in the same arm
				// configuration -- otherwise nearby Cartesian poses can resolve to opposite
				// elbow-up/elbow-down or wrist-flip solutions, producing large joint swings.
				// The validity callback makes setFromIK reject/retry solutions that are in
				// self-collision or collide with the object -- plain setFromIK only checks
				// kinematic reachability and joint limits, so without this, plan() would silently
				// be handed an invalid goal state and abort almost instantly.
				const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(this->move_group->getName());
				bool ik_ok = current_state->setFromIK(
					jmg, waypoints[i], "tool0", 0.3,
					std::bind(
						&WaypointFollower::isStateCollisionFree, this, std::placeholders::_1, std::placeholders::_2,
						std::placeholders::_3));

				if (!ik_ok)
				{
					RCLCPP_WARN(this->get_logger(), "Waypoint %zu/%zu not reachable, skipping", i, waypoints.size());
					continue;
				}

				std::vector<double> joint_target;
				current_state->copyJointGroupPositions(jmg, joint_target);

				this->move_group->setStartStateToCurrentState();
				this->move_group->setJointValueTarget(joint_target);

				moveit::planning_interface::MoveGroupInterface::Plan plan;
				moveit::core::MoveItErrorCode plan_result = this->move_group->plan(plan);

				if (!static_cast<bool>(plan_result))
				{
					std::ostringstream joints_str;
					for (size_t j = 0; j < joint_target.size(); ++j)
						joints_str << (j == 0 ? "" : ", ") << joint_target[j];
					RCLCPP_WARN(
						this->get_logger(),
						"Plan to waypoint %zu/%zu failed (%s), skipping. IK-approved joint target: [%s]", i,
						waypoints.size(), moveit::core::error_code_to_string(plan_result).c_str(), joints_str.str().c_str());
					continue;
				}

				auto result = this->move_group->execute(plan);
				if (result != moveit::core::MoveItErrorCode::SUCCESS)
					RCLCPP_ERROR(this->get_logger(), "Execution failed for waypoint %zu/%zu", i, waypoints.size());
			}

			this->return_to_initial_pose();
		}
};

int main(int argc, char* argv[])
{
		rclcpp::init(argc, argv);

		auto node = std::make_shared<WaypointFollower>(rclcpp::NodeOptions());
		node->init();
		rclcpp::spin(node);

		rclcpp::shutdown();
		return 0;
}
