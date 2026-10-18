import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


# One-off check: heavy IK search for missed viewpoints at given object positions (check_cases in cases_file).
# Pass the same params_file as object_placement so both use the same IK settings and camera roll.
def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda, kinova, xarm7, ur5e or ur3e (see robots.py)."),
        DeclareLaunchArgument("tour_input_dir", default_value="/tmp/viewpoint_planner_output"),
        DeclareLaunchArgument("params_file", default_value="", description="Optional YAML (e.g. object_placement_run.yaml)."),
        DeclareLaunchArgument("cases_file", description="YAML with check_cases: [x, y, z, viewpoint, ...] (abs m)."),
    ]

    object_pose_config = PathJoinSubstitution([FindPackageShare("object_placement_planning"), "config", "object_pose.yaml"])

    def make_node(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        moveit_config = robots.GetMoveItConfig(robot)
        params_file = LaunchConfiguration("params_file").perform(context)
        extra = [os.path.expanduser(params_file)] if params_file else []
        node_params = {
            "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
            "mesh_scale": 0.01,
            "tour_input_dir": LaunchConfiguration("tour_input_dir").perform(context),
        }
        return [Node(
            package="object_placement_planning",
            executable="ik_reachability_check",
            name="ik_reachability_check",
            output="screen",
            parameters=[moveit_config.to_dict(), object_pose_config, robots.GetRobotParameters(robot), *extra, os.path.expanduser(LaunchConfiguration("cases_file").perform(context)), node_params],
        )]

    return LaunchDescription([*args, OpaqueFunction(function=make_node)])
