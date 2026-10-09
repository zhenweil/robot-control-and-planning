import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        description="Launch RViz with the robot (also starts robot_state_publisher). Leave false "
        "when panda_arm.launch.py is running -- its panda.rviz shows the same markers.",
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

    # Inner-solve tuning -- exposed so speed vs. IK-seed noise can be traded without a rebuild.
    gtsp_num_restart_arg = DeclareLaunchArgument(
        "gtsp_num_restart",
        default_value="2",
        description="Min-of-N inner solves wherever a committed cost matters (descent iter-0 + each "
        "accepted step). 1 = fastest.",
    )
    gtsp_num_restart = ParameterValue(LaunchConfiguration("gtsp_num_restart"), value_type=int)

    max_solutions_per_candidate_arg = DeclareLaunchArgument(
        "max_solutions_per_candidate", default_value="4",
        description="IK branches collected per viewpoint for the committed GTSP solve.",
    )
    max_solutions_per_candidate = ParameterValue(
        LaunchConfiguration("max_solutions_per_candidate"), value_type=int
    )

    ik_retries_per_point_arg = DeclareLaunchArgument(
        "ik_retries_per_point", default_value="14",
        description="Extra random IK attempts per viewpoint on top of max_solutions_per_candidate*3.",
    )
    ik_retries_per_point = ParameterValue(
        LaunchConfiguration("ik_retries_per_point"), value_type=int
    )

    ik_timeout_arg = DeclareLaunchArgument(
        "ik_timeout", default_value="0.15", description="Per-attempt IK time budget (s)."
    )
    ik_timeout = ParameterValue(LaunchConfiguration("ik_timeout"), value_type=float)

    # Descent length; descent_num_restart:=1 max_outer_iterations:=0 just solves the tour at the start position.
    descent_num_restart_arg = DeclareLaunchArgument(
        "descent_num_restart", default_value="6", description="Number of descents (restarts)."
    )
    descent_num_restart = ParameterValue(LaunchConfiguration("descent_num_restart"), value_type=int)
    max_outer_iterations_arg = DeclareLaunchArgument(
        "max_outer_iterations", default_value="50", description="Max descent iterations per restart."
    )
    max_outer_iterations = ParameterValue(LaunchConfiguration("max_outer_iterations"), value_type=int)
    stop_when_all_reached_arg = DeclareLaunchArgument(
        "stop_when_all_reached", default_value="false", description="End each descent once every viewpoint is reached."
    )
    stop_when_all_reached = ParameterValue(LaunchConfiguration("stop_when_all_reached"), value_type=bool)
    refine_solves_arg = DeclareLaunchArgument(
        "refine_solves", default_value="0",
        description="Extra full solves at each descent's result, warm-started from its best solution.",
    )
    refine_solves = ParameterValue(LaunchConfiguration("refine_solves"), value_type=int)
    travel_gradient_arg = DeclareLaunchArgument(
        "travel_gradient", default_value="true",
        description="false: the descent direction ignores tour travel (the cost still includes it).",
    )
    travel_gradient = ParameterValue(LaunchConfiguration("travel_gradient"), value_type=bool)
    manipulability_gradient_arg = DeclareLaunchArgument(
        "manipulability_gradient", default_value="true",
        description="false: the descent direction ignores the manipulability terms (the cost still includes them).",
    )
    manipulability_gradient = ParameterValue(LaunchConfiguration("manipulability_gradient"), value_type=bool)
    travel_in_cost_arg = DeclareLaunchArgument(
        "travel_in_cost", default_value="true",
        description="false: the placement cost leaves out travel (GTSP still orders viewpoints by travel).",
    )
    travel_in_cost = ParameterValue(LaunchConfiguration("travel_in_cost"), value_type=bool)
    manipulability_weight_initial_arg = DeclareLaunchArgument(
        "manipulability_weight_initial", default_value="40.0", description="Starting lambda (manipulability weight)."
    )
    manipulability_weight_initial = ParameterValue(
        LaunchConfiguration("manipulability_weight_initial"), value_type=float)
    log_manipulability_weight_arg = DeclareLaunchArgument(
        "log_manipulability_weight", default_value="2.0", description="mu (log-manipulability barrier weight)."
    )
    log_manipulability_weight = ParameterValue(LaunchConfiguration("log_manipulability_weight"), value_type=float)
    miss_gap_weight_arg = DeclareLaunchArgument(
        "miss_gap_weight", default_value="5000.0", description="Weight on missed viewpoints' closest-IK gap (0 = no miss gradient)."
    )
    miss_gap_weight = ParameterValue(LaunchConfiguration("miss_gap_weight"), value_type=float)
    freeze_order_arg = DeclareLaunchArgument(
        "freeze_order", default_value="false",
        description="Once all viewpoints are reached, keep the viewpoint order fixed (arm poses still re-picked).",
    )
    freeze_order = ParameterValue(LaunchConfiguration("freeze_order"), value_type=bool)
    lock_input_order_arg = DeclareLaunchArgument(
        "lock_input_order", default_value="false",
        description="Visit viewpoints in the input order throughout (no GTSP reordering; arm poses still re-picked).",
    )
    lock_input_order = ParameterValue(LaunchConfiguration("lock_input_order"), value_type=bool)
    manipulability_after_reach_arg = DeclareLaunchArgument(
        "manipulability_after_reach", default_value="false",
        description="Misses only (lambda = mu = 0) until all viewpoints are reached, then manipulability on.",
    )
    manipulability_after_reach = ParameterValue(LaunchConfiguration("manipulability_after_reach"), value_type=bool)
    real_cost_planning_time_arg = DeclareLaunchArgument(
        "real_cost_planning_time", default_value="0.0",
        description="> 0: refine picks tours by real cost (OMPL-planned joint travel); seconds per planning attempt.",
    )
    real_cost_planning_time = ParameterValue(LaunchConfiguration("real_cost_planning_time"), value_type=float)
    real_cost_attempts_arg = DeclareLaunchArgument(
        "real_cost_attempts", default_value="3", description="Planning attempts per leg for the real cost."
    )
    real_cost_attempts = ParameterValue(LaunchConfiguration("real_cost_attempts"), value_type=int)
    planned_travel_in_cost_arg = DeclareLaunchArgument(
        "planned_travel_in_cost", default_value="false",
        description="Travel in the cost = OMPL-planned joint travel; overrides travel_in_cost (needs real_cost_planning_time > 0).",
    )
    planned_travel_in_cost = ParameterValue(LaunchConfiguration("planned_travel_in_cost"), value_type=bool)
    steering_arg = DeclareLaunchArgument(
        "steering", default_value="true", description="Steer the step around viewpoints it would lose."
    )
    steering = ParameterValue(LaunchConfiguration("steering"), value_type=bool)
    trace_line_search_arg = DeclareLaunchArgument(
        "trace_line_search", default_value="false",
        description="Log every line-search probe and full solve against the current point.",
    )
    trace_line_search = ParameterValue(LaunchConfiguration("trace_line_search"), value_type=bool)
    refine_at_reach_arg = DeclareLaunchArgument(
        "refine_at_reach", default_value="0",
        description="Log N refine solves at the first all-reached placement; the descent then continues.",
    )
    refine_at_reach = ParameterValue(LaunchConfiguration("refine_at_reach"), value_type=int)

    # Object position bounds, abs (m, base frame). Defaults: nominal (0.5, 0, 0.15) +-0.6 in x/y, z locked.
    position_bound_args = []
    position_bounds = {}
    for _name, _default in (("x_min", -0.5), ("x_max", 0.5), ("y_min", -0.5), ("y_max", 0.5),
                            ("z_min", 0.15), ("z_max", 0.15)):
        position_bound_args.append(DeclareLaunchArgument(
            _name, default_value=str(_default), description=f"Object position {_name} bound (m, abs).",
        ))
        position_bounds[f"bg_{_name}"] = ParameterValue(LaunchConfiguration(_name), value_type=float)

    # Descent start position, abs (m, base frame). Defaults to the nominal object position.
    initial_position_args = []
    initial_position = {}
    for _axis, _default in (("x", 0.5), ("y", 0.0), ("z", 0.15)):
        _name = f"initial_{_axis}"
        initial_position_args.append(DeclareLaunchArgument(
            _name, default_value=str(_default), description=f"Descent start position {_axis} (m, abs).",
        ))
        initial_position[f"bg_{_name}"] = ParameterValue(LaunchConfiguration(_name), value_type=float)

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/tmp/base_gradient_output",
        description="Where base_gradient_result.json is written.",
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
        # Object position bounds (abs, m) and tilt bounds (rad). Yaw excluded (redundant with joint 1).
        **position_bounds,
        # Tilt locked at 0: position-only search for now.
        "bg_roll_min": 0.0,
        "bg_roll_max": 0.0,
        "bg_pitch_min": 0.0,
        "bg_pitch_max": 0.0,
        # Where the descent starts (abs position) and its starting tilt.
        **initial_position,
        "bg_initial_roll": 0.0,
        "bg_initial_pitch": 0.0,
        # Meters per radian: how tip/tilt trades off against translation in the descent step.
        "bg_rot_metric_scale": 0.3,
        # Objective weights (see base_gradient.hpp). Cartesian term has no gradient; kept for
        # inner-GTSP ordering parity with the rest of the pipeline.
        "bg_joint_distance_weight": 1.0,
        "bg_cartesian_distance_weight": 0.0,
        "bg_max_joint_deviation_weight": 1.0,
        # Objective = travel cost - this * sum of manipulability (sum is ~2.8 for 38 viewpoints).
        "bg_manipulability_weight": 0.0,
        # Start each descent at this weight and multiply by the decay per outer iteration down to the
        # value above (40 * 0.8^k, snapped to it once within 5%: ~14 iterations to reach 0).
        "bg_manipulability_weight_initial": manipulability_weight_initial,
        "bg_manipulability_weight_decay": 0.8,
        # Reach-margin barrier: objective also subtracts this * sum of log(manipulability).
        "bg_log_manipulability_weight": log_manipulability_weight,
        # A missed viewpoint costs the penalty + this * its closest-IK pose gap (m), capped at the cap.
        "bg_miss_gap_weight": miss_gap_weight,
        "bg_miss_gap_cap": 0.15,
        # Collision-aware closest IK for missed viewpoints: clearance margin (m), iterations, starts.
        "bg_closest_ik_margin": 0.02,
        "bg_closest_ik_iters": 60,
        "bg_closest_ik_starts": 5,
        # Cost added per viewpoint left unreachable -- keeps a partial solution from looking cheap.
        "bg_unreachable_penalty": 50.0,
        # Raised from 2/8/0.1 + min-of-N committed solves: thin IK branch coverage made the tour
        # cost at a fixed offset swing ~5% on the seed alone. All four are launch args.
        "bg_max_solutions_per_candidate": max_solutions_per_candidate,
        "bg_ik_retries_per_point": ik_retries_per_point,
        "bg_gtsp_num_restart": gtsp_num_restart,
        "bg_gtsp_two_opt_rounds": 5,
        # Basin hopping (1 = single descent). Each restart after the first descends from the best
        # offset so far kicked by a Gaussian of std-dev descent_restart_perturbation (m).
        "bg_descent_num_restart": descent_num_restart,
        "bg_descent_restart_perturbation": 0.05,
        "bg_max_outer_iterations": max_outer_iterations,
        "bg_initial_step": 0.02,
        "bg_step_shrink": 0.25,
        "bg_max_line_search_iters": 4,
        "bg_jacobian_damping": 1e-3,
        "bg_convergence_tolerance_offset": 0.002,
        "bg_convergence_tolerance_cost": 1e-3,
        # Stop after this many consecutive iterations that each gain < bg_convergence_tolerance_cost.
        "bg_patience": 3,
        "bg_stop_when_all_reached": stop_when_all_reached,
        "bg_refine_solves": refine_solves,
        "bg_travel_gradient": travel_gradient,
        "bg_manipulability_gradient": manipulability_gradient,
        "bg_travel_in_cost": travel_in_cost,
        "bg_freeze_order": freeze_order,
        "bg_lock_input_order": lock_input_order,
        "bg_manipulability_after_reach": manipulability_after_reach,
        "bg_real_cost_planning_time": real_cost_planning_time,
        "bg_real_cost_attempts": real_cost_attempts,
        "bg_planned_travel_in_cost": planned_travel_in_cost,
        "bg_steering": steering,
        "bg_trace_line_search": trace_line_search,
        "bg_refine_at_reach": refine_at_reach,
        "bg_fd_gradient_check": fd_gradient_check,
        "bg_fd_epsilon": 1e-4,
        "ik_timeout": ik_timeout,
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

    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value="",
        description="Optional YAML of bg_* node parameters; loaded last, so it overrides the launch arguments.",
    )

    def make_base_gradient_node(context):
        params_file = LaunchConfiguration("params_file").perform(context)
        extra = [os.path.expanduser(params_file)] if params_file else []
        return [Node(
            package="panda_arm_control",
            executable="base_gradient",
            name="base_gradient",
            output="screen",
            parameters=[moveit_config.to_dict(), base_gradient_params, object_pose_config, *extra],
            # Makes MoveIt's IK (KDL) deterministic -- it otherwise re-seeds from /dev/urandom on
            # retries, which is the main run-to-run inconsistency in the result.
            additional_env={"RANDOM_SEED": random_seed},
        )]

    base_gradient_node = OpaqueFunction(function=make_base_gradient_node)

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

    # With use_rviz, publish the robot itself so RViz shows it without the MoveIt launch. Don't
    # combine with panda_arm.launch.py, which already publishes these.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
        condition=IfCondition(use_rviz),
    )
    # Holds the arm at initial_joints (object_pose.yaml), the tour's start pose.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[
            {
                "zeros": {
                    "panda_joint1": 0.0,
                    "panda_joint2": -0.8745,
                    "panda_joint3": 0.0,
                    "panda_joint4": -2.356,
                    "panda_joint5": 0.0,
                    "panda_joint6": 1.571,
                    "panda_joint7": 0.785,
                }
            }
        ],
        condition=IfCondition(use_rviz),
    )
    world_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            execute_on_robot_arg,
            visualize_progress_delay_sec_arg,
            fd_gradient_check_arg,
            random_seed_arg,
            gtsp_num_restart_arg,
            max_solutions_per_candidate_arg,
            ik_retries_per_point_arg,
            ik_timeout_arg,
            descent_num_restart_arg,
            max_outer_iterations_arg,
            stop_when_all_reached_arg,
            refine_solves_arg,
            travel_gradient_arg,
            manipulability_gradient_arg,
            travel_in_cost_arg,
            manipulability_weight_initial_arg,
            log_manipulability_weight_arg,
            miss_gap_weight_arg,
            freeze_order_arg,
            lock_input_order_arg,
            manipulability_after_reach_arg,
            real_cost_planning_time_arg,
            real_cost_attempts_arg,
            planned_travel_in_cost_arg,
            steering_arg,
            trace_line_search_arg,
            refine_at_reach_arg,
            *position_bound_args,
            *initial_position_args,
            output_dir_arg,
            tour_input_dir_arg,
            params_file_arg,
            base_gradient_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
            world_tf_node,
            rviz_node,
        ]
    )
