# launch/display.launch.py
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = FindPackageShare("panda_arm_moveit")

    rviz_config = PathJoinSubstitution([
        pkg_share, "config", "panda.rviz"
    ])

    moveit_config = (
            MoveItConfigsBuilder("panda", package_name="panda_arm_moveit")
            .robot_description(file_path="config/panda.urdf.xacro")
            .robot_description_semantic(file_path="config/panda.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("panda_arm_moveit"),
        "config",
        "ros2_controllers.yaml",
    )
    
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
    )

    run_move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()]
    )

    static_transform_publisher = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description, 
            ros2_controllers_path,
        ],
        output="both",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )
    
    return LaunchDescription([
        static_transform_publisher,
        robot_state_publisher,
        ros2_control_node,
        arm_controller_spawner,
        joint_state_broadcaster_spawner,
        run_move_group,
        rviz2_node,
    ])
