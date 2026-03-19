#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
		mesh_pose.position.z = 0.7;
		
		moveit_msgs::msg::CollisionObject obj;
		obj.header.frame_id = "world";
		obj.id = "bunny_holding_eggs";
		obj.meshes.push_back(mesh_msg);
		obj.mesh_poses.push_back(mesh_pose);
		obj.operation = obj.ADD;

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		planning_scene_interface.applyCollisionObject(obj);

		auto node = std::make_shared<rclcpp::Node>(
						"panda_arm_control",
						rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

		auto logger = rclcpp::get_logger("panda_arm_control");

		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		std::thread spinner([&executor]() { executor.spin(); });

		const std::string planning_group = "panda_arm";
		moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

		std::vector<double> initial_joints = move_group.getCurrentJointValues();

		move_group.setStartStateToCurrentState();
		move_group.setPlanningTime(100.0);
		move_group.setNumPlanningAttempts(50);

		// Make it obviously slow
		move_group.setMaxVelocityScalingFactor(0.05);
		move_group.setMaxAccelerationScalingFactor(0.05);

		// Wait for state updates to settle
		rclcpp::sleep_for(std::chrono::seconds(2));

		auto current_state = move_group.getCurrentState(5.0);
		if (!current_state)
		{
				RCLCPP_ERROR(logger, "Failed to get current state");
				rclcpp::shutdown();
				spinner.join();
				return 1;
		}

		const moveit::core::JointModelGroup* joint_model_group =
				current_state->getJointModelGroup(planning_group);

		if (!joint_model_group)
		{
				RCLCPP_ERROR(logger, "Could not find joint model group %s", planning_group.c_str());
				rclcpp::shutdown();
				spinner.join();
				return 1;
		}

		std::vector<double> joint_values;
		current_state->copyJointGroupPositions(joint_model_group, joint_values);

		RCLCPP_INFO(logger, "Current joints:");
		for (size_t i = 0; i < joint_values.size(); ++i)
		{
				RCLCPP_INFO(logger, "  joint[%zu] = %.3f", i, joint_values[i]);
		}

		// Pick a clearly different target
		std::vector<double> joint_goal = joint_values;
		joint_goal[0] = 0.0;
		joint_goal[1] = -0.8;
		joint_goal[2] = 0.0;
		joint_goal[3] = -2.0;
		joint_goal[4] = 0.0;
		joint_goal[5] = 1.6;
		joint_goal[6] = 0.8;

		move_group.setJointValueTarget(joint_goal);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = static_cast<bool>(move_group.plan(plan));

		if (!success)
		{
				RCLCPP_ERROR(logger, "Planning failed");
				rclcpp::shutdown();
				spinner.join();
				return 1;
		}

		RCLCPP_INFO(logger, "Plan succeeded.");
		RCLCPP_INFO(logger, "Look at RViz now. Executing in 5 seconds...");
		rclcpp::sleep_for(std::chrono::seconds(5));

		auto result = move_group.execute(plan);
		if (!static_cast<bool>(result))
		{
				RCLCPP_ERROR(logger, "Execution failed");
				rclcpp::shutdown();
				spinner.join();
				return 1;
		}

		RCLCPP_INFO(logger, "Execution finished. Holding final pose for 5 seconds...");
		rclcpp::sleep_for(std::chrono::seconds(5));

		RCLCPP_INFO(logger, "Goint back to initial state...");
		move_group.setStartStateToCurrentState();
		move_group.setJointValueTarget(initial_joints);
		moveit::planning_interface::MoveGroupInterface::Plan return_plan;
		bool return_ok = static_cast<bool>(move_group.plan(return_plan));
		if(return_ok)
		{
				move_group.execute(return_plan);
		}
		else
		{
				RCLCPP_ERROR(logger, "Return failed");
				return 1;
		}


		rclcpp::shutdown();
		spinner.join();
		return 0;
}
