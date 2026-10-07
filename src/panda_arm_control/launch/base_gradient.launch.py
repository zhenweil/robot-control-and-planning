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
        "accepted step, and every experiment cold/warm solve). 1 = fastest.",
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

    experiment_random_search_budget_arg = DeclareLaunchArgument(
        "experiment_random_search_budget",
        default_value="0",
        description="experiment_mode, exp 5: uniform-in-bounds offsets to cold-solve for the "
        "equal-budget random-search baseline. 0 = match the descent's committed-solve count.",
    )
    experiment_random_search_budget = ParameterValue(
        LaunchConfiguration("experiment_random_search_budget"), value_type=int
    )

    placement_experiment_arg = DeclareLaunchArgument(
        "placement_experiment",
        default_value="false",
        description="Instead of the normal run: sweep a placement_grid_n^3 grid of (x,y,z) object "
        "offsets (rotation held at 0), scoring each of several reference routes with a fixed-order "
        "branch DP plus one free-routing GTSP per grid point. Answers whether position affects "
        "cost, whether re-routing helps at the best position, and whether the best position moves "
        "with the route. Writes placement_experiment_seed<seed>_g<grid_start>.json and exits. "
        "scripts/run_placement_order_experiment.py drives + reports this.",
    )
    placement_experiment = ParameterValue(LaunchConfiguration("placement_experiment"), value_type=bool)

    placement_grid_n_arg = DeclareLaunchArgument(
        "placement_grid_n",
        default_value="5",
        description="placement_experiment: grid points per axis (grid_n^3 total). Odd values put "
        "the nominal offset exactly on the grid.",
    )
    placement_grid_n = ParameterValue(LaunchConfiguration("placement_grid_n"), value_type=int)

    placement_grid_start_arg = DeclareLaunchArgument(
        "placement_grid_start",
        default_value="0",
        description="placement_experiment: flat index of the first grid point to evaluate -- for "
        "sharding the sweep across processes.",
    )
    placement_grid_start = ParameterValue(LaunchConfiguration("placement_grid_start"), value_type=int)

    placement_grid_count_arg = DeclareLaunchArgument(
        "placement_grid_count",
        default_value="-1",
        description="placement_experiment: number of grid points to evaluate from placement_grid_start; "
        "-1 = to the end of the grid, 0 = a routes-only prep run (write the reference-routes file, "
        "no grid, no result JSON).",
    )
    placement_grid_count = ParameterValue(LaunchConfiguration("placement_grid_count"), value_type=int)

    placement_reference_orders_file_arg = DeclareLaunchArgument(
        "placement_reference_orders_file",
        default_value="",
        description="placement_experiment: path to a reference-routes JSON to score against. Empty "
        "= solve the routes and write them to output_dir/placement_reference_orders_seed<seed>.json. "
        "Set it on grid shards so every shard uses one identical route set.",
    )
    placement_reference_orders_file = LaunchConfiguration("placement_reference_orders_file")

    # Object-offset translation bounds -- exposed so a run (esp. the placement sweep) can zoom in
    # without a rebuild. Defaults are the full search box.
    offset_bound_args = []
    offset_bounds = {}
    for _axis, _default in (("x", 0.6), ("y", 0.6), ("z", 0.0)):
        for _side, _sign in (("min", -1.0), ("max", 1.0)):
            _name = f"{_axis}_{_side}"
            offset_bound_args.append(DeclareLaunchArgument(
                _name, default_value=str(_sign * _default),
                description=f"Object offset {_axis} {_side} bound (m).",
            ))
            offset_bounds[f"bg_{_name}"] = ParameterValue(
                LaunchConfiguration(_name), value_type=float)

    # Descent start offset (m, relative to the nominal object pose), settable without a rebuild.
    initial_offset_args = []
    initial_offset = {}
    for _axis in ("x", "y", "z"):
        _name = f"initial_{_axis}"
        initial_offset_args.append(DeclareLaunchArgument(
            _name, default_value="0.0", description=f"Descent start offset {_axis} (m, rel to nominal).",
        ))
        initial_offset[f"bg_{_name}"] = ParameterValue(LaunchConfiguration(_name), value_type=float)

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
        **offset_bounds,
        # z/roll/pitch locked at 0: x-y only search for now.
        "bg_roll_min": 0.0,
        "bg_roll_max": 0.0,
        "bg_pitch_min": 0.0,
        "bg_pitch_max": 0.0,
        # Where the descent starts (0 = object's nominal pose).
        **initial_offset,
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
        "bg_manipulability_weight_initial": 40.0,
        "bg_manipulability_weight_decay": 0.8,
        # Reach-margin barrier: objective also subtracts this * sum of log(manipulability).
        "bg_log_manipulability_weight": 2.0,
        # A missed viewpoint costs the penalty + this * its closest-IK pose gap (m), capped at the cap.
        "bg_miss_gap_weight": 5000.0,
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
        "bg_descent_num_restart": 6,
        "bg_descent_restart_perturbation": 0.05,
        "bg_max_outer_iterations": 50,
        "bg_initial_step": 0.02,
        "bg_step_shrink": 0.25,
        "bg_max_line_search_iters": 4,
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
        "bg_experiment_random_search_budget": experiment_random_search_budget,
        "bg_placement_experiment": placement_experiment,
        "bg_placement_grid_n": placement_grid_n,
        "bg_placement_grid_start": placement_grid_start,
        "bg_placement_grid_count": placement_grid_count,
        "bg_placement_reference_orders_file": placement_reference_orders_file,
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
            experiment_mode_arg,
            experiment_num_random_dirs_arg,
            experiment_random_search_budget_arg,
            placement_experiment_arg,
            placement_grid_n_arg,
            placement_grid_start_arg,
            placement_grid_count_arg,
            placement_reference_orders_file_arg,
            *offset_bound_args,
            *initial_offset_args,
            output_dir_arg,
            tour_input_dir_arg,
            base_gradient_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
            world_tf_node,
            rviz_node,
        ]
    )
