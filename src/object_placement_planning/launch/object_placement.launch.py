import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robots  # noqa: E402


def generate_launch_description():
    robot_arg = DeclareLaunchArgument("robot", default_value="panda", description="Robot: panda, kinova, xarm7, ur5e or ur3e (see robots.py).")

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
        "(topic /object_placement_progress_markers), so the base's descent can actually be "
        "watched in RViz instead of flashing by. 0.0 (default) adds no delay; try e.g. 0.5.",
    )
    visualize_progress_delay_sec = ParameterValue(
        LaunchConfiguration("visualize_progress_delay_sec"), value_type=float
    )

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

    # Descent length; descent_num_restart:=1 max_kinematic_moves:=0 just solves the tour at the start position.
    descent_num_restart_arg = DeclareLaunchArgument(
        "descent_num_restart", default_value="6", description="Number of descents (restarts)."
    )
    descent_num_restart = ParameterValue(LaunchConfiguration("descent_num_restart"), value_type=int)
    max_kinematic_moves_arg = DeclareLaunchArgument(
        "max_kinematic_moves", default_value="20", description="Max kinematic moves per reach attempt."
    )
    max_kinematic_moves = ParameterValue(LaunchConfiguration("max_kinematic_moves"), value_type=int)
    stop_when_all_reached_arg = DeclareLaunchArgument(
        "stop_when_all_reached", default_value="false", description="End each descent once every viewpoint is reached."
    )
    stop_when_all_reached = ParameterValue(LaunchConfiguration("stop_when_all_reached"), value_type=bool)
    gtsp_restarts_arg = DeclareLaunchArgument(
        "gtsp_restarts", default_value="0",
        description="Extra full solves at each descent's result, warm-started from its best solution.",
    )
    gtsp_restarts = ParameterValue(LaunchConfiguration("gtsp_restarts"), value_type=int)
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
    real_cost_planning_time_arg = DeclareLaunchArgument(
        "real_cost_planning_time", default_value="0.0",
        description="> 0: GTSP restarts pick tours by real cost (OMPL-planned joint travel); seconds per planning attempt.",
    )
    real_cost_planning_time = ParameterValue(LaunchConfiguration("real_cost_planning_time"), value_type=float)
    real_cost_attempts_arg = DeclareLaunchArgument(
        "real_cost_attempts", default_value="3", description="Planning attempts per leg for the real cost."
    )
    real_cost_attempts = ParameterValue(LaunchConfiguration("real_cost_attempts"), value_type=int)
    planned_travel_in_cost_arg = DeclareLaunchArgument(
        "planned_travel_in_cost", default_value="false",
        description="Travel in the cost = OMPL-planned joint travel once all viewpoints are reached (needs real_cost_planning_time > 0).",
    )
    planned_travel_in_cost = ParameterValue(LaunchConfiguration("planned_travel_in_cost"), value_type=bool)
    trace_line_search_arg = DeclareLaunchArgument(
        "trace_line_search", default_value="false",
        description="Log every line-search probe and full solve against the current point.",
    )
    trace_line_search = ParameterValue(LaunchConfiguration("trace_line_search"), value_type=bool)
    gtsp_restarts_at_reach_arg = DeclareLaunchArgument(
        "gtsp_restarts_at_reach", default_value="0",
        description="Log N GTSP restarts at the first all-reached placement; the descent then continues.",
    )
    gtsp_restarts_at_reach = ParameterValue(LaunchConfiguration("gtsp_restarts_at_reach"), value_type=int)

    # Object position bounds, abs (m, base frame). Defaults: nominal (0.5, 0, 0.15) +-0.6 in x/y, z locked.
    position_bound_args = []
    position_bounds = {}
    for _name, _default in (("x_min", -0.5), ("x_max", 0.5), ("y_min", -0.5), ("y_max", 0.5),
                            ("z_min", 0.15), ("z_max", 0.15)):
        position_bound_args.append(DeclareLaunchArgument(
            _name, default_value=str(_default), description=f"Object position {_name} bound (m, abs).",
        ))
        position_bounds[_name] = ParameterValue(LaunchConfiguration(_name), value_type=float)

    # Descent start position, abs (m, base frame). Defaults to the nominal object position.
    initial_position_args = []
    initial_position = {}
    for _axis, _default in (("x", 0.5), ("y", 0.0), ("z", 0.15)):
        _name = f"initial_{_axis}"
        initial_position_args.append(DeclareLaunchArgument(
            _name, default_value=str(_default), description=f"Descent start position {_axis} (m, abs).",
        ))
        initial_position[_name] = ParameterValue(LaunchConfiguration(_name), value_type=float)

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/tmp/object_placement_output",
        description="Where object_placement_result.json is written.",
    )
    output_dir = LaunchConfiguration("output_dir")

    tour_input_dir_arg = DeclareLaunchArgument(
        "tour_input_dir",
        default_value="/tmp/viewpoint_planner_output",
        description="Directory a prior viewpoint_planner_* run exported selected_robot_poses.json "
        "into -- the tour this node optimizes a base offset for.",
    )
    tour_input_dir = LaunchConfiguration("tour_input_dir")

    object_placement_params = {
        "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",
        "mesh_scale": 0.01,
        # Where a prior viewpoint_planner_* run exported the ordered tour's
        # selected_robot_poses.json -- this node's input.
        "tour_input_dir": tour_input_dir,
        "output_dir": output_dir,
        # Object position bounds (abs, m); the object keeps its nominal orientation.
        **position_bounds,
        # Where the descent starts (abs position).
        **initial_position,
        # Meters per radian: weighs a missed viewpoint's angle gap against its distance gap.
        "rot_metric_scale": 0.3,
        # Objective weights (see object_placement.hpp). Cartesian term has no gradient; kept for
        # inner-GTSP ordering parity with the rest of the pipeline.
        "joint_distance_weight": 1.0,
        "max_joint_deviation_weight": 1.0,
        # A missed viewpoint costs the penalty + this * its closest-IK pose gap (m), capped at the cap.
        "miss_gap_weight": miss_gap_weight,
        "miss_gap_cap": 0.15,
        # Collision-aware closest IK for missed viewpoints: clearance margin (m), iterations, starts.
        "closest_ik_margin": 0.02,
        "closest_ik_iters": 60,
        "closest_ik_starts": 5,
        # Cost added per viewpoint left unreachable -- keeps a partial solution from looking cheap.
        "unreachable_penalty": 50.0,
        # Raised from 2/8/0.1 + min-of-N committed solves: thin IK branch coverage made the tour
        # cost at a fixed offset swing ~5% on the seed alone. All four are launch args.
        "max_solutions_per_candidate": max_solutions_per_candidate,
        "ik_retries_per_point": ik_retries_per_point,
        "gtsp_num_restart": gtsp_num_restart,
        "gtsp_two_opt_rounds": 5,
        # Descents (1 = single). Each restart starts from the best position so far with a new IK seed.
        "descent_num_restart": descent_num_restart,
        "max_kinematic_moves": max_kinematic_moves,
        "initial_step": 0.02,
        "step_shrink": 0.25,
        "max_line_search_iters": 4,
        "jacobian_damping": 1e-3,
        "convergence_tolerance_offset": 0.002,
        "convergence_tolerance_cost": 1e-3,
        # Stop after this many consecutive iterations that each gain < convergence_tolerance_cost.
        "patience": 3,
        "stop_when_all_reached": stop_when_all_reached,
        "gtsp_restarts": gtsp_restarts,
        "freeze_order": freeze_order,
        "lock_input_order": lock_input_order,
        "real_cost_planning_time": real_cost_planning_time,
        "real_cost_attempts": real_cost_attempts,
        "planned_travel_in_cost": planned_travel_in_cost,
        "trace_line_search": trace_line_search,
        "gtsp_restarts_at_reach": gtsp_restarts_at_reach,
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
        [FindPackageShare("object_placement_planning"), "config", "object_pose.yaml"]
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value="",
        description="Optional YAML of node parameters; loaded last, so it overrides the launch arguments.",
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("object_placement_planning"), "config", "viewpoint_planner.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # The robot-dependent nodes: object_placement, and with use_rviz the robot itself so RViz shows it without the MoveIt
    # launch (don't combine with panda_arm.launch.py, which already publishes these).
    def make_robot_nodes(context):
        robot = robots.GetRobot(LaunchConfiguration("robot").perform(context))
        moveit_config = robots.GetMoveItConfig(robot)
        params_file = LaunchConfiguration("params_file").perform(context)
        extra = [os.path.expanduser(params_file)] if params_file else []
        object_placement_node = Node(
            package="object_placement_planning",
            executable="object_placement",
            name="object_placement",
            output="screen",
            parameters=[moveit_config.to_dict(), object_placement_params, object_pose_config, robots.GetRobotParameters(robot), *extra],
            # Makes MoveIt's IK (KDL) deterministic -- it otherwise re-seeds from /dev/urandom on
            # retries, which is the main run-to-run inconsistency in the result.
            additional_env={"RANDOM_SEED": random_seed},
        )
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            parameters=[moveit_config.robot_description],
            condition=IfCondition(use_rviz),
        )
        # Holds the arm at its home pose, the tour's start pose.
        joint_state_publisher_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="log",
            parameters=[{"zeros": robot["home"]}],
            condition=IfCondition(use_rviz),
        )
        nodes = [object_placement_node, robot_state_publisher_node, joint_state_publisher_node]
        if robot["base_link"]:  # a URDF without its own world link
            nodes.append(Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=["0", "0", "0", "0", "0", "0", "world", robot["base_link"]],
                condition=IfCondition(use_rviz),
            ))
        return nodes

    return LaunchDescription(
        [
            use_rviz_arg,
            execute_on_robot_arg,
            visualize_progress_delay_sec_arg,
            random_seed_arg,
            gtsp_num_restart_arg,
            max_solutions_per_candidate_arg,
            ik_retries_per_point_arg,
            ik_timeout_arg,
            descent_num_restart_arg,
            max_kinematic_moves_arg,
            stop_when_all_reached_arg,
            gtsp_restarts_arg,
            miss_gap_weight_arg,
            freeze_order_arg,
            lock_input_order_arg,
            real_cost_planning_time_arg,
            real_cost_attempts_arg,
            planned_travel_in_cost_arg,
            trace_line_search_arg,
            gtsp_restarts_at_reach_arg,
            *position_bound_args,
            *initial_position_args,
            output_dir_arg,
            tour_input_dir_arg,
            params_file_arg,
            robot_arg,
            OpaqueFunction(function=make_robot_nodes),
            rviz_node,
        ]
    )
