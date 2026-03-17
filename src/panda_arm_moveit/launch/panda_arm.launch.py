# launch/display.launch.py
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = FindPackageShare("panda_arm_moveit")

    xacro_file = PathJoinSubstitution([
        pkg_share, "config", "panda.urdf.xacro"
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share, "config", "panda.rviz"
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    moveit_config = (
            MoveItConfigsBuilder("panda", package_name="panda_arm_moveit")
            .robot_description(file_path="config/panda.urdf.xacro")
            .robot_description_semantic(file_path="config/panda.srdf")
            .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
            )
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
