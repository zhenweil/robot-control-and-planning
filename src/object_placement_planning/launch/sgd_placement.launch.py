import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


# SGD placement baseline (Mayer and Althoff 2026). Pass the same params_file as object_placement so both use one
# search box (x/y/z_min/max), start (initial_x/y/z), closest-IK settings and random_seed.
def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda, kinova, xarm7, ur5e or ur3e (see robots.py)."),
        DeclareLaunchArgument("random_seed", default_value="42", description="Seeds the random restarts."),
        DeclareLaunchArgument("time_budget_sec", default_value="300.0", description="Stop after this many seconds."),
        DeclareLaunchArgument("output_dir", default_value="/tmp/sgd_placement_output"),
        DeclareLaunchArgument("tour_input_dir", default_value="/tmp/viewpoint_planner_output"),
        DeclareLaunchArgument("params_file", default_value="", description="Optional YAML loaded last (e.g. object_placement_run.yaml)."),
    ]

    object_pose_config = PathJoinSubstitution([FindPackageShare("object_placement_planning"), "config", "object_pose.yaml"])

    def make_node(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        params_file = LaunchConfiguration("params_file").perform(context)
        moveit_config = robots.ShrinkJointRanges(robots.GetMoveItConfig(robot), robots.GetJointMargin(os.path.expanduser(params_file)))
        extra = [os.path.expanduser(params_file)] if params_file else []
        seed = LaunchConfiguration("random_seed").perform(context)
        node_params = {
            "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
            "mesh_scale": 0.01,
            "tour_input_dir": LaunchConfiguration("tour_input_dir").perform(context),
            "output_dir": LaunchConfiguration("output_dir").perform(context),
            "random_seed": int(seed),
            "sgd_time_budget_sec": float(LaunchConfiguration("time_budget_sec").perform(context)),
        }
        return [Node(
            package="object_placement_planning",
            executable="sgd_placement",
            name="sgd_placement",
            output="screen",
            # params_file comes before the node params so random_seed / budget launch args win.
            parameters=[moveit_config.to_dict(), object_pose_config, robots.GetRobotParameters(robot), *extra, node_params],
            additional_env={"RANDOM_SEED": seed},
        )]

    return LaunchDescription([*args, OpaqueFunction(function=make_node)])
