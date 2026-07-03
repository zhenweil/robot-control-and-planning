#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class WaypointFollower : public rclcpp::Node
{
	public:
		WaypointFollower(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
			: Node("waypoint_follower", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
		{}

		void init()
		{
			this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
				shared_from_this(), "panda_arm");
			this->move_group->startStateMonitor();

			this->move_group->setPlanningTime(20.0);
			this->move_group->setMaxVelocityScalingFactor(0.2);
			this->move_group->setMaxAccelerationScalingFactor(0.2);
			this->move_group->setEndEffectorLink("tool0");
			this->sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"/cartesian_waypoints",
				rclcpp::QoS(1).transient_local(),
				std::bind(&WaypointFollower::waypointCallback, this, std::placeholders::_1));
			RCLCPP_INFO(this->get_logger(), "Subscriber created");
			this->worker = std::thread(&WaypointFollower::workerLoop, this);
		}

	private:
		std::thread worker;
		std::mutex waypoint_mutex;
		bool has_new_waypoints = false;
		std::condition_variable waypoint_cv;	
		geometry_msgs::msg::PoseArray latest_waypoints;
		std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};

		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
		
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
				auto current_state = move_group->getCurrentState(2.0);
				if (!current_state)
				{
					RCLCPP_WARN(this->get_logger(), "Failed to get current state for waypoint %zu/%zu, skipping", i, waypoints.size());
					continue;
				}

				// Seed IK from the current joint state (rather than letting setPoseTarget's goal
				// sampler pick any solution) so consecutive waypoints stay in the same arm
				// configuration -- otherwise nearby Cartesian poses can resolve to opposite
				// elbow-up/elbow-down or wrist-flip solutions, producing large joint swings.
				const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup(move_group->getName());
				bool ik_ok = current_state->setFromIK(jmg, waypoints[i], "tool0", 0.1);

				if (!ik_ok)
				{
					RCLCPP_WARN(this->get_logger(), "Waypoint %zu/%zu not reachable, skipping", i, waypoints.size());
					continue;
				}

				std::vector<double> joint_target;
				current_state->copyJointGroupPositions(jmg, joint_target);

				move_group->setStartStateToCurrentState();
				move_group->setJointValueTarget(joint_target);

				moveit::planning_interface::MoveGroupInterface::Plan plan;
				bool ok = static_cast<bool>(move_group->plan(plan));

				if (!ok)
				{
					RCLCPP_WARN(this->get_logger(), "Plan to waypoint %zu/%zu failed, skipping", i, waypoints.size());
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

		std::string object_mesh_path = "package://panda_arm_control/meshes/bunny_holding_eggs.stl";
		Eigen::Vector3d scale(0.01, 0.01, 0.01);
		shapes::Mesh* m = shapes::createMeshFromResource(object_mesh_path, scale);
		std::cout << "loaded object mesh." << std::endl;

		shape_msgs::msg::Mesh mesh_msg;
		shapes::ShapeMsg mesh_tmp;
		shapes::constructMsgFromShape(m, mesh_tmp);
		mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_tmp);	

		geometry_msgs::msg::Pose mesh_pose;
		mesh_pose.orientation.w = 1.0;
		mesh_pose.position.x = 0.2;
		mesh_pose.position.y = 0.2;
		mesh_pose.position.z = 0.38;
		
		moveit_msgs::msg::CollisionObject obj;
		obj.header.frame_id = "world";
		obj.id = "bunny_holding_eggs";
		obj.meshes.push_back(mesh_msg);
		obj.mesh_poses.push_back(mesh_pose);
		obj.operation = obj.ADD;

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		planning_scene_interface.applyCollisionObject(obj);

		// auto node = rclcpp::Node::make_shared("moveit_node");
		// rclcpp::executors::SingleThreadedExecutor executor;
		// executor.add_node(node);
		// std::thread spinner([&executor]() { executor.spin(); });
		// moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
		// move_group.startStateMonitor();
		// rclcpp::sleep_for(std::chrono::seconds(1));
		// auto state = move_group.getCurrentState(2.0);

		auto node = std::make_shared<WaypointFollower>(rclcpp::NodeOptions());
		node->init();
		rclcpp::spin(node);
		
		rclcpp::shutdown();
		return 0;
}
