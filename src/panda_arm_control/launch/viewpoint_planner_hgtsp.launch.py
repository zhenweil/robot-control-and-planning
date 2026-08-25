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
        description="Launch RViz. Set to true if no other launch file "
        "(e.g. panda_arm.launch.py, whose panda.rviz already shows "
        "/viewpoint_markers) is already providing it.",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    candidate_generation_method_arg = DeclareLaunchArgument(
        "candidate_generation_method",
        default_value="grid",
        description="'grid': standoff/tilt grid per surface point (GenerateViewCandidates). "
        "'potential_field': view direction from a mesh-attraction field instead of an explicit "
        "tilt grid (GeneratePotentialFieldViewCandidates).",
    )
    candidate_generation_method = LaunchConfiguration("candidate_generation_method")

    execute_on_robot_arg = DeclareLaunchArgument(
        "execute_on_robot",
        default_value="true",
        description="After the final tour is selected/ordered, re-plan its legs at higher "
        "fidelity and drive the real robot through them directly via MoveGroupInterface, instead "
        "of just publishing poses on /cartesian_waypoints for a separate node (waypoint_follower) "
        "to re-plan. Requires move_group (e.g. panda_arm.launch.py) to already be running. Do not "
        "also run panda_arm_control.launch.py (waypoint_follower) at the same time -- both would "
        "independently try to drive the same robot with no coordination between them. Defaults to "
        "true; set to false to only publish waypoints instead of moving the robot.",
    )
    execute_on_robot = ParameterValue(LaunchConfiguration("execute_on_robot"), value_type=bool)

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

    viewpoint_planner_params = {
        "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm_binary.stl",  # empty -> resolved via ament_index_cpp in code; rviz mesh markers require binary STL
        "mesh_scale": 0.01,
        "target_faces": 1000,
        "n_surface_samples": 1000,
        "standoff_distances": [0.03],
        "tilt_angles_deg": [0.0],
        "tip_angles_deg": [0.0],
        "candidate_generation_method": candidate_generation_method,
        "min_clearance": 0.005,
        "fov_deg": 30.0,
        "max_distance": 30.0,
        "angle_threshold_deg": 60.0,
        "max_rays_per_view": 2000,
        "target_area_visibility": 0.95,
        "ik_timeout": 0.5,
        "random_seed": 42,
        "group_name": "panda_arm",
        # Tour ordering cost is cartesian_distance_m + joint_distance_weight * joint_distance_rad,
        # both measured from real planned trajectories.
        "joint_distance_weight": 1.0,
        "t_tcp_camera_xyz": [0.0, 0.0, 0.0],
        "t_tcp_camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
        # Hierarchical guide-path tour ordering (SolveHierarchicalTour) -- see
        # hierarchical_tour.hpp for what each of these controls.
        "hgtsp_min_size_for_hierarchy": 8,
        "hgtsp_exemplar_metric_rotation_weight": 0.1,
        "hgtsp_ap_preference_quantile": 0.95,
        "hgtsp_ap_damping": 0.9,
        "hgtsp_ap_max_iterations": 200,
        "hgtsp_ap_convergence_iterations": 15,
        "hgtsp_guide_path_planning_time": 2.0,
        "hgtsp_cluster_planning_time": 2.0,
        "execute_on_robot": execute_on_robot,
        "execution_planning_time": 5.0,
        "execution_planning_attempts": 5,
        "output_dir": "/tmp/viewpoint_planner_output",
    }

    # Shared with panda_arm_control.launch.py so both nodes agree on where the object actually
    # is -- see object_pose.yaml.
    object_pose_config = PathJoinSubstitution(
        [FindPackageShare("panda_arm_control"), "config", "object_pose.yaml"]
    )

    viewpoint_planner_node = Node(
        package="panda_arm_control",
        executable="viewpoint_planner_hgtsp",
        name="viewpoint_planner_hgtsp",
        output="screen",
        parameters=[moveit_config.to_dict(), viewpoint_planner_params, object_pose_config],
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
            candidate_generation_method_arg,
            execute_on_robot_arg,
            viewpoint_planner_node,
            rviz_node,
        ]
    )
