import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


# Object placement co-optimizing reach and tour time. Pass the same params_file as object_placement so all use one
# search box (x/y/z_min/max), start (initial_x/y/z), closest-IK settings and random_seed.
def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda, kinova, xarm7, ur5e or ur3e (see robots.py)."),
        DeclareLaunchArgument("random_seed", default_value="42", description="Seeds the random IK starts."),
        DeclareLaunchArgument("time_budget_sec", default_value="300.0", description="Stop after this many seconds."),
        DeclareLaunchArgument("output_dir", default_value="/tmp/object_gtsp_co_optimize_output"),
        DeclareLaunchArgument("tour_input_dir", default_value="/tmp/viewpoint_planner_output"),
        DeclareLaunchArgument("mesh_path", default_value="/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl", description="Object mesh (binary STL, cm: mesh_scale 0.01)."),
        DeclareLaunchArgument("params_file", default_value="", description="Optional YAML loaded last (e.g. object_placement_run.yaml)."),
        DeclareLaunchArgument("use_rviz", default_value="false", description="Launch RViz with the robot; the node then keeps its final markers up until Ctrl-C."),
        DeclareLaunchArgument("execute_on_robot", default_value="false", description="Drive the robot through the final tour; needs move_group running (robot_sim.launch.py, or panda_arm.launch.py). Leave use_rviz false then."),
    ]
    use_rviz = LaunchConfiguration("use_rviz")

    object_pose_config = PathJoinSubstitution([FindPackageShare("object_placement_planning"), "config", "object_pose.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("object_placement_planning"), "config", "viewpoint_planner.rviz"])

    def make_nodes(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        params_file = LaunchConfiguration("params_file").perform(context)
        moveit_config = robots.ShrinkJointRanges(robots.GetMoveItConfig(robot), robots.GetJointMargin(os.path.expanduser(params_file)))
        extra = [os.path.expanduser(params_file)] if params_file else []
        seed = LaunchConfiguration("random_seed").perform(context)
        node_params = {
            "mesh_path": LaunchConfiguration("mesh_path").perform(context),
            "mesh_scale": 0.01,
            "tour_input_dir": LaunchConfiguration("tour_input_dir").perform(context),
            "output_dir": LaunchConfiguration("output_dir").perform(context),
            "random_seed": int(seed),
            "co_time_budget_sec": float(LaunchConfiguration("time_budget_sec").perform(context)),
            "keep_alive": use_rviz.perform(context).lower() == "true",
            "execute_on_robot": LaunchConfiguration("execute_on_robot").perform(context).lower() == "true",
        }
        nodes = [Node(
            package="object_placement_planning",
            executable="object_gtsp_co_optimize",
            name="object_gtsp_co_optimize",
            output="screen",
            # params_file comes before the node params so random_seed / budget launch args win.
            parameters=[moveit_config.to_dict(), object_pose_config, robots.GetRobotParameters(robot), *extra, node_params],
            additional_env={"RANDOM_SEED": seed},
        )]
        # With use_rviz: the robot itself, held at its home pose, so RViz shows it without the MoveIt launch.
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            parameters=[moveit_config.robot_description],
            condition=IfCondition(use_rviz),
        ))
        nodes.append(Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="log",
            parameters=[{"zeros": robot["home"]}],
            condition=IfCondition(use_rviz),
        ))
        if robot["base_link"]:  # a URDF without its own world link
            nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=["0", "0", "0", "0", "0", "0", "world", robot["base_link"]],
                condition=IfCondition(use_rviz),
            ))
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(use_rviz),
        ))
        return nodes

    return LaunchDescription([*args, OpaqueFunction(function=make_nodes)])
