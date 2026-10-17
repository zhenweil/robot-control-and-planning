import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


# Random-sampling placement baseline. Pass the same params_file as base_gradient so both use one
# search box (bg_x/y/z_min/max), IK budget (ik_attempts) and random_seed.
def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda or kinova (see robots.py)."),
        DeclareLaunchArgument("random_seed", default_value="42", description="Seeds the sampler and the IK restarts."),
        DeclareLaunchArgument("time_budget_sec", default_value="90.0", description="Stop after this many seconds."),
        DeclareLaunchArgument("max_samples", default_value="0", description="Stop after this many samples (0 = no limit)."),
        DeclareLaunchArgument("stop_when_all_reached", default_value="false", description="Refine the first position reaching every viewpoint, then stop."),
        DeclareLaunchArgument("weighted_by_reachability", default_value="false", description="Sample in proportion to the robot-only reachability score (needs the reachability map)."),
        DeclareLaunchArgument("output_dir", default_value="/tmp/random_placement_output"),
        DeclareLaunchArgument("tour_input_dir", default_value="/tmp/viewpoint_planner_output"),
        DeclareLaunchArgument("params_file", default_value="", description="Optional YAML loaded last (e.g. base_gradient_run.yaml)."),
    ]

    object_pose_config = PathJoinSubstitution([FindPackageShare("panda_arm_control"), "config", "object_pose.yaml"])
    reachability_map_config = PathJoinSubstitution([FindPackageShare("panda_arm_control"), "config", "reachability_map.yaml"])

    def make_node(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        moveit_config = robots.GetMoveItConfig(robot)
        params_file = LaunchConfiguration("params_file").perform(context)
        extra = [os.path.expanduser(params_file)] if params_file else []
        seed = LaunchConfiguration("random_seed").perform(context)
        node_params = {
            "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
            "mesh_scale": 0.01,
            "tour_input_dir": LaunchConfiguration("tour_input_dir").perform(context),
            "output_dir": LaunchConfiguration("output_dir").perform(context),
            "random_seed": int(seed),
            "rp_time_budget_sec": float(LaunchConfiguration("time_budget_sec").perform(context)),
            "rp_max_samples": int(LaunchConfiguration("max_samples").perform(context)),
            "rp_stop_when_all_reached": LaunchConfiguration("stop_when_all_reached").perform(context).lower() == "true",
            "rp_weighted_by_reachability": LaunchConfiguration("weighted_by_reachability").perform(context).lower() == "true",
        }
        return [Node(
            package="panda_arm_control",
            executable="random_placement",
            name="random_placement",
            output="screen",
            # params_file comes before the node params so random_seed / budget launch args win.
            parameters=[moveit_config.to_dict(), object_pose_config, robots.GetRobotParameters(robot), reachability_map_config, *extra, node_params],
            additional_env={"RANDOM_SEED": seed},
        )]

    return LaunchDescription([*args, OpaqueFunction(function=make_node)])
