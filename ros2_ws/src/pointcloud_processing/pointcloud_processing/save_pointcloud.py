#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import os

class SavePointCloud(Node):
    def __init__(self):
        super().__init__('save_pointcloud')

        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/cropped_pointcloud')
        self.declare_parameter('parent_frame_id', 'radar_global_map')  # Parent frame ID
        self.declare_parameter('child_frame_id', 'radar')  # Child frame ID
        self.declare_parameter('save_directory', '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds')

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.save_directory = self.get_parameter('save_directory').value

        # Ensure save directory exists
        os.makedirs(self.save_directory, exist_ok=True)
        self.file_counter = 0  # File counter for unique filenames

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.save_pointcloud,
            10
        )
        self.get_logger().info(f"Listening to topic: {self.pointcloud_topic}")
        self.get_logger().info(f"Saving point clouds to directory: {self.save_directory}")
        self.get_logger().info(f"Parent frame: {self.parent_frame_id}, Child frame: {self.child_frame_id}")

    def save_pointcloud(self, msg):
        # Convert PointCloud2 to a numpy array
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = np.array(list(points), dtype=np.float32)

        # Generate dynamic filename
        filename = os.path.join(self.save_directory, f'pointcloud_{self.file_counter:06d}.pcd')

        # Save to PCD file using numpy
        self.save_pcd_file(filename, points_list)
        self.file_counter += 1

        self.get_logger().info(f"Saved point cloud to {filename} (Frame: {self.parent_frame_id} -> {self.child_frame_id})")

    @staticmethod
    def save_pcd_file(filename, points):
        """ Save point cloud data to a PCD file """
        with open(filename, 'w') as pcd_file:
            pcd_file.write('# .PCD v0.7 - Point Cloud Data file format\n')
            pcd_file.write('VERSION 0.7\n')
            pcd_file.write('FIELDS x y z\n')
            pcd_file.write('SIZE 4 4 4\n')
            pcd_file.write('TYPE F F F\n')
            pcd_file.write('COUNT 1 1 1\n')
            pcd_file.write(f'WIDTH {points.shape[0]}\n')
            pcd_file.write('HEIGHT 1\n')
            pcd_file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            pcd_file.write(f'POINTS {points.shape[0]}\n')
            pcd_file.write('DATA ascii\n')
            np.savetxt(pcd_file, points, fmt='%f %f %f')

def main(args=None):
    rclpy.init(args=args)
    node = SavePointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)