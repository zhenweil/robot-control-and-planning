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
			: Node("waypoint_follower", options)
		{}

		void init()
		{
			this->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
				shared_from_this(), "panda_arm");

			this->move_group->setPlanningTime(20.0);
			this->move_group->setMaxVelocityScalingFactor(0.2);
			this->move_group->setMaxAccelerationScalingFactor(0.2);

			sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"/cartesian_waypoints",
				10,
				std::bind(&WaypointFollower::waypointCallback, this, std::placeholders::_1));
			
			RCLCPP_INFO(this->get_logger(), "Subscriber created");
		}
	private:
		std::vector<double> initial_joints = {0.0, -0.8745, 0.0, -2.356, 0.0, 1.571, 0.785};

		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub;

		void return_to_initial_pose()
		{
			moveit::planning_interface::MoveGroupInterface::Plan return_plan;
			// this->move_group->setStartStateToCurrentState();
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
			this->return_to_initial_pose();

			RCLCPP_INFO(this->get_logger(), "callback fired");
			if (msg->poses.empty())
			{
				RCLCPP_WARN(this->get_logger(), "Received empty PoseArray");
				return;
			}
			
			RCLCPP_INFO(this->get_logger(), "Received %zu waypoints", msg->poses.size());

			std::vector<geometry_msgs::msg::Pose> waypoints;
			waypoints.reserve(msg->poses.size());

			for (const auto& pose : msg->poses)
			{
				waypoints.push_back(pose);
			}

			move_group->setStartStateToCurrentState();
			move_group->setPoseTarget(msg->poses[0]);
			moveit::planning_interface::MoveGroupInterface::Plan first_plan;
			bool ok = static_cast<bool>(move_group->plan(first_plan));
			
			if(!ok)
			{
				return;
			} 
			this->move_group->execute(first_plan);
			rclcpp::sleep_for(std::chrono::seconds(1));

			RCLCPP_INFO(this->get_logger(), "first pose reachable: %s", ok ? "yes" : "no");
			moveit_msgs::msg::RobotTrajectory trajectory;

			// Tune these if needed
			const double eef_step = 0.005;      // meters
			const double jump_threshold = 0.0;  // disable jump check for now

			const double fraction = this->move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

			RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.3f", fraction);

			if (fraction < 0.9)
			{
				RCLCPP_WARN(this->get_logger(), "Path planning incomplete, not executing");
				this->return_to_initial_pose();
				return;
			}

			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			auto result = this->move_group->execute(plan);
			if (result != moveit::core::MoveItErrorCode::SUCCESS)
				RCLCPP_ERROR(this->get_logger(), "Execution failed");

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
		mesh_pose.position.x = 0.3;
		mesh_pose.position.y = 0.1;
		mesh_pose.position.z = 0.65;
		
		moveit_msgs::msg::CollisionObject obj;
		obj.header.frame_id = "world";
		obj.id = "bunny_holding_eggs";
		obj.meshes.push_back(mesh_msg);
		obj.mesh_poses.push_back(mesh_pose);
		obj.operation = obj.ADD;

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		// planning_scene_interface.applyCollisionObject(obj);

		// auto node = std::make_shared<rclcpp::Node>(
		// 				"panda_arm_control",
		// 				rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

		// auto logger = rclcpp::get_logger("panda_arm_control");

		// rclcpp::executors::SingleThreadedExecutor executor;
		// executor.add_node(node);
		// std::thread spinner([&executor]() { executor.spin(); });

		// const std::string planning_group = "panda_arm";
		// moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

		// std::vector<double> initial_joints = move_group.getCurrentJointValues();

		// move_group.setStartStateToCurrentState();
		// move_group.setPlanningTime(100.0);
		// move_group.setNumPlanningAttempts(50);

		// // Make it obviously slow
		// move_group.setMaxVelocityScalingFactor(0.2);
		// move_group.setMaxAccelerationScalingFactor(0.1);

		// // Wait for state updates to settle
		// rclcpp::sleep_for(std::chrono::seconds(2));

		// auto current_state = move_group.getCurrentState(5.0);
		// if (!current_state)
		// {
		// 		RCLCPP_ERROR(logger, "Failed to get current state");
		// 		rclcpp::shutdown();
		// 		spinner.join();
		// 		return 1;
		// }

		// const moveit::core::JointModelGroup* joint_model_group =
		// 		current_state->getJointModelGroup(planning_group);

		// if (!joint_model_group)
		// {
		// 		RCLCPP_ERROR(logger, "Could not find joint model group %s", planning_group.c_str());
		// 		rclcpp::shutdown();
		// 		spinner.join();
		// 		return 1;
		// }

		// std::vector<double> joint_values;
		// current_state->copyJointGroupPositions(joint_model_group, joint_values);

		// RCLCPP_INFO(logger, "Current joints:");
		// for (size_t i = 0; i < joint_values.size(); ++i)
		// {
		// 		RCLCPP_INFO(logger, "  joint[%zu] = %.3f", i, joint_values[i]);
		// }

		// //move_group.setJointValueTarget(joint_goal);
		// geometry_msgs::msg::Pose target_pose;
		// target_pose.position.x = 0.4;
		// target_pose.position.y = 0.0;
		// target_pose.position.z = 0.9;

		// target_pose.orientation.x = 0.0;
		// target_pose.orientation.y = 0.0;
		// target_pose.orientation.z = 0.0;
		// target_pose.orientation.w = 1.0;
		
		// move_group.setPoseTarget(target_pose);

		// moveit::planning_interface::MoveGroupInterface::Plan plan;
		// bool success = static_cast<bool>(move_group.plan(plan));

		// if (!success)
		// {
		// 		RCLCPP_ERROR(logger, "Planning failed");
		// 		rclcpp::shutdown();
		// 		spinner.join();
		// 		return 1;
		// }

		// RCLCPP_INFO(logger, "Plan succeeded.");
		// RCLCPP_INFO(logger, "Look at RViz now. Executing in 5 seconds...");
		// rclcpp::sleep_for(std::chrono::seconds(5));

		// auto result = move_group.execute(plan);
		// if (!static_cast<bool>(result))
		// {
		// 		RCLCPP_ERROR(logger, "Execution failed");
		// 		rclcpp::shutdown();
		// 		spinner.join();
		// 		return 1;
		// }

		// RCLCPP_INFO(logger, "Execution finished. Holding final pose for 5 seconds...");
		// rclcpp::sleep_for(std::chrono::seconds(5));

		// RCLCPP_INFO(logger, "Goint back to initial state...");
		// move_group.setStartStateToCurrentState();
		// move_group.setJointValueTarget(initial_joints);
		// moveit::planning_interface::MoveGroupInterface::Plan return_plan;
		// bool return_ok = static_cast<bool>(move_group.plan(return_plan));
		// if(return_ok)
		// {
		// 		move_group.execute(return_plan);
		// }
		// else
		// {
		// 		RCLCPP_ERROR(logger, "Return failed");
		// 		return 1;
		// }
		auto node = std::make_shared<WaypointFollower>();
		node->init();

		rclcpp::spin(node);
		rclcpp::shutdown();
		// spinner.join();
		return 0;
}
