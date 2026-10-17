"""Builds the reachability map used by random_placement's weighted sampling; settings in config/reachability_map.yaml."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


def generate_launch_description():
    reachability_map_config = PathJoinSubstitution(
        [FindPackageShare("panda_arm_control"), "config", "reachability_map.yaml"]
    )

    def make_node(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        moveit_config = robots.GetMoveItConfig(robot)
        return [Node(
            package="panda_arm_control",
            executable="build_reachability_map",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.joint_limits,
                {"group_name": robot["group"], "tip_link": "tool0"},
                reachability_map_config,
            ],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda, kinova or xarm7 (see robots.py)."),
        OpaqueFunction(function=make_node),
    ])
