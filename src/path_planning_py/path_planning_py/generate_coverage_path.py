#!/usr/bin/env python3
import sys
sys.path.append("/home/zhenweil/mesh-processing/")
sys.path.append("/home/zhenweil/traveling-salesman-problem/")

import rclpy
from rclpy.node import Node
import trimesh
import numpy as np
from three_opt_tsp import three_opt  
from nn_tsp import nearest_neighbor_tour 
from mesh_segmentation import EZMesh
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header

def normals_to_quaternions(normals):
    normals = np.asarray(normals, dtype=float)

    # normalize
    normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)

    z_axis = np.array([0, 0, 1])

    # cross product (N,3)
    axes = np.cross(np.tile(z_axis, (normals.shape[0], 1)), normals)
    axis_norms = np.linalg.norm(axes, axis=1, keepdims=True)

    # dot product (N,)
    dots = np.sum(normals * z_axis, axis=1)
    angles = np.arccos(np.clip(dots, -1.0, 1.0))

    # avoid divide by zero
    axes = np.where(axis_norms < 1e-6, np.array([1, 0, 0]), axes)
    axes = axes / np.linalg.norm(axes, axis=1, keepdims=True)

    rotvecs = axes * angles[:, None]

    quats = R.from_rotvec(rotvecs).as_quat()  # (N,4)

    # fix opposite case (dot ≈ -1)
    opposite = dots < -0.9999
    quats[opposite] = np.array([1, 0, 0, 0])

    return quats  # shape (N,4)

class WaypointPub(Node):
    def __init__(self, points, normals, mesh_path):
        super().__init__("waypoint_pub")
        assert len(points) == len(normals)
        self.pub = self.create_publisher(PoseArray, "/cartesian_waypoints", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/mesh_points", 10)
        self.points = points/100 + np.array([0.2, 0.2, 0.38])
        # self.points = self.points[:2,:]
        self.normals = normals  
        print(self.points)
        mesh = trimesh.load(mesh_path)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(
                tuple(g for g in mesh.geometry.values())
            )

        sampled_points, _ = trimesh.sample.sample_surface(mesh, 5000)
        sampled_points /= 100
        sampled_points += np.array([0.2, 0.2, 0.38])
        self.sampled_points = sampled_points.astype(np.float32)
    
    def make_cloud_msg(self, points: np.ndarray) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def send(self):
        msg = PoseArray()
        msg.header.frame_id = "world"

        quats = normals_to_quaternions(self.normals)
        for i in range(len(self.points)):
            pose = Pose()
            x,y,z = self.points[i,:]
            qx,qy,qz,qw = quats[i,:]
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)

            # pose.orientation.x = 1.0 
            # pose.orientation.y = 0.0 
            # pose.orientation.z = 0.0 
            # pose.orientation.w = 0.0 

            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            msg.poses.append(pose)

        self.pub.publish(msg)
        cloud_msg = self.make_cloud_msg(self.sampled_points)
        self.cloud_pub.publish(cloud_msg)
        print("Published waypoints")


def main():
    fname = "/home/zhenweil/mesh-processing/data/bunny_holding_eggs_repaired.stl"
    my_mesh = EZMesh(fname)
    segmentation, centroids, normals = my_mesh.segment_based_on_normal(90)
    num_groups = len(segmentation)
    colors = np.random.rand(num_groups, 3)
    face_colors = np.zeros((my_mesh.num_faces, 3), dtype=float)

    for seg_idx, face_indices in enumerate(segmentation):
        face_colors[face_indices] = colors[seg_idx]
    new_mesh = trimesh.Trimesh(vertices=my_mesh.vertices, faces=my_mesh.faces, process=False)
    new_mesh.visual.face_colors = face_colors
    #new_mesh.show()
    
    clearance = 10
    view_points = centroids + clearance*normals
    segments = np.stack([view_points, view_points-normals], axis=1)
    path = trimesh.load_path(segments)
    points = trimesh.points.PointCloud(view_points)

    num_view_points = len(view_points)
    dist_matrix = np.linalg.norm(view_points[:,None,:]-points[None,:,:], axis=2)
    initial_tour, _ = nearest_neighbor_tour(dist_matrix)
    # best_tour, best_dist = three_opt(initial_tour, dist_matrix)
    # print("best tour is: ", best_tour)
    view_points_sorted = view_points[initial_tour, :]
    normals_sorted = -normals[initial_tour, :]

    rclpy.init()
    waypoint_pub = WaypointPub(view_points_sorted, normals_sorted, fname)
    waypoint_pub.send()

    rclpy.spin_once(waypoint_pub, timeout_sec=0.5)
    print("Successfully published waypoints")
    waypoint_pub.destroy_node()
    rclpy.shutdown()
    # scene = trimesh.Scene()
    # scene.add_geometry(new_mesh)
    # scene.add_geometry(points)
    # scene.add_geometry(path)
    # scene.show()
