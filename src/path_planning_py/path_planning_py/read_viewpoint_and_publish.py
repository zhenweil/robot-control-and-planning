#!/usr/bin/env python3

import json
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R


# ------------------------------------------------------------
# User settings
# ------------------------------------------------------------

VIEWPOINT_FILE = "/home/zhenweil/mesh-processing/output/selected_viewpoints.json"

# If your viewpoint file is in mesh/object frame, convert it to robot world frame.
# This is similar to your old:
# points / 100 + np.array([0.2, 0.2, 0.38])
#
# Since the new viewpoints are already in meters, only add translation.
OBJECT_TRANSLATION_WORLD = np.array([0.2, 0.2, 0.38])

# If object frame and world frame have same orientation, keep identity.
OBJECT_ROTATION_WORLD = R.from_euler("xyz", [0.0, 0.0, 0.0], degrees=True).as_matrix()

# Fixed transform from TCP frame to camera frame.
# If your camera frame is exactly the same as TCP frame, keep identity.
#
# T_tcp_camera means:
#     p_camera = T_tcp_camera * p_tcp
#
# Then:
#     T_world_tcp = T_world_camera * inverse(T_tcp_camera)
#
T_TCP_CAMERA = np.eye(4)

# Convention:
# This script assumes camera local +Z axis points along view_dir.
# If your robot tool looks along -Z instead, see note below.
CAMERA_FORWARD_AXIS = "+Z"


# ------------------------------------------------------------
# Geometry helpers
# ------------------------------------------------------------

def normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    return v / (np.linalg.norm(v) + eps)


def make_transform(position, rotation_matrix):
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = position
    return T


def transform_to_pose(T):
    pos = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()  # x, y, z, w
    return pos, quat


def camera_rotation_from_view_dir(view_dir, world_up=np.array([0.0, 0.0, 1.0])):
    """
    Build camera orientation from viewing direction.

    Assumption:
        camera local +Z axis points along view_dir.

    Returns:
        R_world_camera
    """

    z_cam = normalize(view_dir)

    # Avoid singularity when z_cam is parallel to world_up
    if abs(np.dot(z_cam, world_up)) > 0.95:
        world_up = np.array([0.0, 1.0, 0.0])

    x_cam = normalize(np.cross(world_up, z_cam))
    y_cam = normalize(np.cross(z_cam, x_cam))

    R_world_camera = np.column_stack([x_cam, y_cam, z_cam])

    return R_world_camera


def load_viewpoints_json(path):
    with open(path, "r") as f:
        data = json.load(f)

    camera_positions = []
    view_dirs = []

    for item in data:
        p = item["camera_pos"]
        d = item["view_dir"]

        camera_positions.append([
            float(p["x"]),
            float(p["y"]),
            float(p["z"]),
        ])

        view_dirs.append([
            float(d["x"]),
            float(d["y"]),
            float(d["z"]),
        ])

    return np.asarray(camera_positions), np.asarray(view_dirs)


def convert_viewpoint_to_tcp_pose(camera_pos_object, view_dir_object):
    """
    Convert one viewpoint from object frame to TCP pose in world frame.
    """

    # Object frame -> world frame
    camera_pos_world = OBJECT_ROTATION_WORLD @ camera_pos_object + OBJECT_TRANSLATION_WORLD
    view_dir_world = OBJECT_ROTATION_WORLD @ normalize(view_dir_object)

    # Camera orientation in world
    R_world_camera = camera_rotation_from_view_dir(view_dir_world)

    T_world_camera = make_transform(camera_pos_world, R_world_camera)

    # Camera pose -> TCP pose
    T_world_tcp = T_world_camera @ np.linalg.inv(T_TCP_CAMERA)

    tcp_pos, tcp_quat = transform_to_pose(T_world_tcp)

    return tcp_pos, tcp_quat


# ------------------------------------------------------------
# ROS publisher
# ------------------------------------------------------------

class ViewpointPosePub(Node):
    def __init__(self, viewpoint_file):
        super().__init__("viewpoint_pose_pub")

        self.pub = self.create_publisher(PoseArray, "/cartesian_waypoints", 10)

        self.camera_positions, self.view_dirs = load_viewpoints_json(viewpoint_file)

        assert len(self.camera_positions) == len(self.view_dirs)

        self.get_logger().info(f"Loaded {len(self.camera_positions)} viewpoints")
        self.get_logger().info(f"Publishing to /cartesian_waypoints")

    def build_pose_array(self):
        msg = PoseArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        for camera_pos, view_dir in zip(self.camera_positions, self.view_dirs):
            tcp_pos, tcp_quat = convert_viewpoint_to_tcp_pose(camera_pos, view_dir)

            pose = Pose()

            pose.position.x = float(tcp_pos[0])
            pose.position.y = float(tcp_pos[1])
            pose.position.z = float(tcp_pos[2])

            pose.orientation.x = float(tcp_quat[0])
            pose.orientation.y = float(tcp_quat[1])
            pose.orientation.z = float(tcp_quat[2])
            pose.orientation.w = float(tcp_quat[3])

            msg.poses.append(pose)

        return msg

    def send(self):
        msg = self.build_pose_array()

        while self.pub.get_subscription_count() == 0:
            self.get_logger().info("waiting for subscriber...")
            time.sleep(0.1)

        self.pub.publish(msg)

        self.get_logger().info(f"Published {len(msg.poses)} TCP poses")


def main():
    rclpy.init()

    node = ViewpointPosePub(VIEWPOINT_FILE)
    node.send()

    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()