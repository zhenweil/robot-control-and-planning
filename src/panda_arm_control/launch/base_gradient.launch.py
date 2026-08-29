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
        "providing it (panda.rviz shows /base_gradient_markers too).",
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
        description="Seconds to pause after each descent iteration's progress publish "
        "(topic /base_gradient_progress_markers), so the base's descent can actually be "
        "watched in RViz instead of flashing by. 0.0 (default) adds no delay; try e.g. 0.5.",
    )
    visualize_progress_delay_sec = ParameterValue(
        LaunchConfiguration("visualize_progress_delay_sec"), value_type=float
    )

    fd_gradient_check_arg = DeclareLaunchArgument(
        "fd_gradient_check",
        default_value="false",
        description="Log the analytic base-pose gradient next to a central-difference estimate "
        "each outer iteration -- they should agree while no tour reorder / IK-branch switch "
        "happens. Defaults to false.",
    )
    fd_gradient_check = ParameterValue(LaunchConfiguration("fd_gradient_check"), value_type=bool)

    random_seed_arg = DeclareLaunchArgument(
        "random_seed",
        default_value="42",
        description="Seeds both the basin-hop RNG (bg param) and, via the RANDOM_SEED env var, "
        "MoveIt's IK plugin -- so a full run is reproducible. Change it to sample a different "
        "run; the KDL IK re-seeds nondeterministically without this.",
    )
    random_seed = LaunchConfiguration("random_seed")

    experiment_mode_arg = DeclareLaunchArgument(
        "experiment_mode",
        default_value="false",
        description="Instead of the normal run: descend once for b*, then cold/warm re-solve the "
        "tour at b*, offset 0, and random offsets of the same size, to attribute the descent's "
        "cost drop to the object move vs inner-solve (IK/GTSP) variance. Writes "
        "base_gradient_experiment_seed<random_seed>.json to output_dir and exits. Sweep "
        "random_seed across launches (scripts/run_base_gradient_experiments.py does this).",
    )
    experiment_mode = ParameterValue(LaunchConfiguration("experiment_mode"), value_type=bool)

    experiment_num_random_dirs_arg = DeclareLaunchArgument(
        "experiment_num_random_dirs",
        default_value="10",
        description="experiment_mode: how many random offsets (of magnitude |b*|) to probe for "
        "experiments 3 and 4.",
    )
    experiment_num_random_dirs = ParameterValue(
        LaunchConfiguration("experiment_num_random_dirs"), value_type=int
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/tmp/base_gradient_output",
        description="Where base_gradient_result.json (or, in experiment_mode, "
        "base_gradient_experiment_seed<seed>.json) is written.",
    )
    output_dir = LaunchConfiguration("output_dir")

    tour_input_dir_arg = DeclareLaunchArgument(
        "tour_input_dir",
        default_value="/tmp/viewpoint_planner_output",
        description="Directory a prior viewpoint_planner_* run exported selected_robot_poses.json "
        "into -- the tour this node optimizes a base offset for.",
    )
    tour_input_dir = LaunchConfiguration("tour_input_dir")

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

    base_gradient_params = {
        "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
        "mesh_scale": 0.01,
        "group_name": "panda_arm",
        # Where a prior viewpoint_planner_* run exported the ordered tour's
        # selected_robot_poses.json -- this node's input.
        "tour_input_dir": tour_input_dir,
        "output_dir": output_dir,
        # Object offset search bounds in the robot base frame: translation (m) + tip roll / tilt
        # pitch (rad). Yaw is excluded (redundant with joint 1). Realized by re-fixturing the
        # object, not moving the arm.
        "bg_x_min": -0.3,
        "bg_x_max": 0.3,
        "bg_y_min": -0.3,
        "bg_y_max": 0.3,
        "bg_z_min": -0.2,
        "bg_z_max": 0.2,
        "bg_roll_min": -0.35,
        "bg_roll_max": 0.35,
        "bg_pitch_min": -0.35,
        "bg_pitch_max": 0.35,
        # Where the descent starts (0 = object's nominal pose).
        "bg_initial_x": 0.0,
        "bg_initial_y": 0.0,
        "bg_initial_z": 0.0,
        "bg_initial_roll": 0.0,
        "bg_initial_pitch": 0.0,
        # Meters per radian: how tip/tilt trades off against translation in the descent step.
        "bg_rot_metric_scale": 0.3,
        # Objective weights (see base_gradient.hpp). Cartesian term has no gradient; kept for
        # inner-GTSP ordering parity with the rest of the pipeline.
        "bg_joint_distance_weight": 1.0,
        "bg_cartesian_distance_weight": 0.0,
        "bg_max_joint_deviation_weight": 1.0,
        # Cost added per viewpoint left unreachable -- keeps a partial solution from looking cheap.
        "bg_unreachable_penalty": 50.0,
        "bg_max_solutions_per_candidate": 2,
        "bg_ik_retries_per_point": 8,
        "bg_gtsp_two_opt_rounds": 5,
        # Optional basin hopping (1 = single descent, the default). Raise only for a problem that
        # looks multi-basin; each restart after the first descends from the best offset so far
        # kicked by a Gaussian of std-dev restart_perturbation (m).
        "bg_num_restarts": 1,
        "bg_restart_perturbation": 0.08,
        "bg_restart_patience": 2,
        "bg_min_restarts": 3,
        "bg_max_outer_iterations": 15,
        "bg_initial_step": 0.05,
        "bg_step_shrink": 0.5,
        "bg_armijo_c": 1e-4,
        "bg_min_step": 1e-4,
        "bg_max_line_search_iters": 8,
        "bg_jacobian_damping": 1e-3,
        "bg_convergence_tolerance_offset": 0.002,
        "bg_convergence_tolerance_cost": 1e-3,
        # Stop after this many consecutive iterations that each gain < bg_convergence_tolerance_cost.
        "bg_patience": 3,
        "bg_fd_gradient_check": fd_gradient_check,
        "bg_fd_epsilon": 1e-4,
        "bg_experiment_mode": experiment_mode,
        "bg_experiment_num_random_dirs": experiment_num_random_dirs,
        "bg_experiment_min_probe_metric": 0.01,
        "ik_timeout": 0.1,
        "random_seed": ParameterValue(random_seed, value_type=int),
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

    base_gradient_node = Node(
        package="panda_arm_control",
        executable="base_gradient",
        name="base_gradient",
        output="screen",
        parameters=[moveit_config.to_dict(), base_gradient_params, object_pose_config],
        # Makes MoveIt's IK (KDL) deterministic -- it otherwise re-seeds from /dev/urandom on
        # retries, which is the main run-to-run inconsistency in the result.
        additional_env={"RANDOM_SEED": random_seed},
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
            fd_gradient_check_arg,
            random_seed_arg,
            experiment_mode_arg,
            experiment_num_random_dirs_arg,
            output_dir_arg,
            tour_input_dir_arg,
            base_gradient_node,
            rviz_node,
        ]
    )
