from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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
        "mesh_path": "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired_cm.stl",  # empty -> resolved via ament_index_cpp in code
        "mesh_scale": 0.01,
        "target_faces": 1000,
        "n_surface_samples": 500,
        "standoff_distances": [0.01, 0.02, 0.03],
        "tilt_angles_deg": [0.0, 15.0, -15.0],
        "min_clearance": 0.005,
        "fov_deg": 30.0,
        "max_distance": 20.0,
        "angle_threshold_deg": 70.0,
        "max_rays_per_view": 2000,
        "target_area_visibility": 0.95,
        "min_new_area_ratio": 0.001,
        "ik_timeout": 0.1,
        "random_seed": 42,
        "group_name": "panda_arm",
        "object_translation_world": [0.2, 0.2, 0.38],
        "object_rotation_rpy_deg": [0.0, 0.0, 0.0],
        "t_tcp_camera_xyz": [0.0, 0.0, 0.0],
        "t_tcp_camera_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
        "output_dir": "/tmp/viewpoint_planner_output",
    }

    viewpoint_planner_node = Node(
        package="panda_arm_control",
        executable="viewpoint_planner",
        name="viewpoint_planner",
        output="screen",
        parameters=[moveit_config.to_dict(), viewpoint_planner_params],
    )

    return LaunchDescription([viewpoint_planner_node])
