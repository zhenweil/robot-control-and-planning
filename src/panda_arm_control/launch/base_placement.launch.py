from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz. Set to true if no other launch file is already "
        "providing it (panda.rviz shows /base_placement_markers too).",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    execute_on_robot_arg = DeclareLaunchArgument(
        "execute_on_robot",
        default_value="false",
        description="Drive the REAL robot through the recommended placement's tour by moving "
        "the (software-only) collision object into the frame the base would see at the "
        "recommended offset. Only correct if that matches reality -- the object has actually "
        "been physically moved to match, or the base has actually been remounted. Requires "
        "move_group (e.g. panda_arm.launch.py) already running. Defaults to false.",
    )
    execute_on_robot = ParameterValue(LaunchConfiguration("execute_on_robot"), value_type=bool)

    visualize_progress_delay_sec_arg = DeclareLaunchArgument(
        "visualize_progress_delay_sec",
        default_value="0.0",
        description="Seconds to pause after each refinement-loop iteration's progress publish "
        "(topic /base_placement_progress_markers), so the search's convergence can actually be "
        "watched in RViz instead of flashing by. 0.0 (default) adds no delay; try e.g. 0.5.",
    )
    visualize_progress_delay_sec = ParameterValue(
        LaunchConfiguration("visualize_progress_delay_sec"), value_type=float
    )

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

    base_placement_params = {
        "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
        "mesh_scale": 0.01,
        "group_name": "panda_arm",
        # Where a prior viewpoint_planner_* run (e.g. viewpoint_planner_hgtsp) exported the
        # ordered tour's selected_robot_poses.json -- this node's input.
        "tour_input_dir": "/tmp/viewpoint_planner_output",
        "output_dir": "/tmp/base_placement_output",
        # Candidate base offset search bounds, relative to today's actual mount -- see
        # base_placement.hpp for what each controls.
        "bp_x_min": -0.3,
        "bp_x_max": 0.3,
        "bp_y_min": -0.3,
        "bp_y_max": 0.3,
        "bp_yaw_min": -3.14159265,
        "bp_yaw_max": 3.14159265,
        "bp_num_restarts": 6,
        "bp_max_outer_iterations": 8,
        "bp_candidates_per_point": 16,
        "bp_ik_retries_per_point": 4,
        "bp_radius_shrink_factor": 0.6,
        "bp_convergence_tolerance_xy": 0.002,
        "bp_convergence_tolerance_yaw": 0.01,
        "ik_timeout": 0.1,
        "random_seed": 42,
        "visualize_progress_delay_sec": visualize_progress_delay_sec,
        "execute_on_robot": execute_on_robot,
        "execution_planning_time": 5.0,
        "execution_planning_attempts": 5,
    }

    # Shared with viewpoint_planner_hgtsp.launch.py so both agree on where the object actually
    # is -- see object_pose.yaml.
    object_pose_config = PathJoinSubstitution(
        [FindPackageShare("panda_arm_control"), "config", "object_pose.yaml"]
    )

    base_placement_node = Node(
        package="panda_arm_control",
        executable="base_placement",
        name="base_placement",
        output="screen",
        parameters=[moveit_config.to_dict(), base_placement_params, object_pose_config],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("panda_arm_control"), "config", "viewpoint_planner.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            execute_on_robot_arg,
            visualize_progress_delay_sec_arg,
            base_placement_node,
            rviz_node,
        ]
    )
