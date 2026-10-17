"""Builds the reachability map used by random_placement's weighted sampling; settings in config/reachability_map.yaml."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_arm_moveit")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    reachability_map_config = PathJoinSubstitution(
        [FindPackageShare("panda_arm_control"), "config", "reachability_map.yaml"]
    )
    return LaunchDescription([
        Node(
            package="panda_arm_control",
            executable="build_reachability_map",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.joint_limits,
                {"group_name": "panda_arm", "tip_link": "tool0"},
                reachability_map_config,
            ],
        ),
    ])
