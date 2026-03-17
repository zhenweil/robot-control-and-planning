# launch/display.launch.py
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution

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
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
