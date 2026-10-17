"""Sanity check: climb the reachability-map score from a start object position (no IK); prints each step and shows it in
RViz (/object_placement_progress_markers: object, trail, viewpoints colored by map value, the worst one larger).

ros2 launch panda_arm_control reachability_climb.launch.py start:="[0.1, 0.0, 0.1]"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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
    config = lambda name: PathJoinSubstitution([FindPackageShare("panda_arm_control"), "config", name])
    use_rviz = LaunchConfiguration("use_rviz")
    climb = Node(
        package="panda_arm_control",
        executable="reachability_climb",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            config("object_pose.yaml"),
            config("reachability_map.yaml"),
            {
                "group_name": "panda_arm",
                "tip_link": "tool0",
                "start": ParameterValue(LaunchConfiguration("start"), value_type=None),
                "step": ParameterValue(LaunchConfiguration("step"), value_type=float),
                "step_delay": ParameterValue(LaunchConfiguration("step_delay"), value_type=float),
                "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
                "mesh_scale": 0.01,
            },
        ],
    )
    # RViz with the robot shown at its home pose.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", config("viewpoint_planner.rviz")],
        output="screen",
        condition=IfCondition(use_rviz),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(use_rviz),
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[{"zeros": {
            "panda_joint1": 0.0, "panda_joint2": -0.8745, "panda_joint3": 0.0, "panda_joint4": -2.356,
            "panda_joint5": 0.0, "panda_joint6": 1.571, "panda_joint7": 0.785,
        }}],
        condition=IfCondition(use_rviz),
    )
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        condition=IfCondition(use_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument("start", default_value="[0.1, 0.0, 0.1]", description="abs x, y, z (m), with decimals"),
        DeclareLaunchArgument("step", default_value="0.05", description="largest step (m)"),
        DeclareLaunchArgument("step_delay", default_value="0.3", description="s between shown steps"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        rviz,
        robot_state_publisher,
        joint_state_publisher,
        world_tf,
        climb,
    ])
