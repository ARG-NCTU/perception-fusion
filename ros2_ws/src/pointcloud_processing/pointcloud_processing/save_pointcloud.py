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
        self.declare_parameter('pointcloud_topic', '/halo_radar/transformed_pointcloud')
        self.declare_parameter('save_directory', '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_pcds')

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
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
        # self.get_logger().info(f"Listening to topic: {self.pointcloud_topic}")
        # self.get_logger().info(f"Saving point clouds to directory: {self.save_directory}")

    def save_pointcloud(self, msg):
        # Read points from PointCloud2
        field_names = [field.name for field in msg.fields]
        points = list(pc2.read_points(msg, field_names=tuple(field_names), skip_nans=True))
        # self.get_logger().info(f"Read {len(points)} points from PointCloud2.")

        if len(points) == 0:
            # self.get_logger().error("PointCloud2 message contains no valid points.")
            return

        # Convert points into a 2D numpy array
        points_array = np.array([list(point) for point in points], dtype=float)

        # Separate the 3D points and intensity if available
        xyz = points_array[:, :3]  # Extract (x, y, z)
        intensity = points_array[:, 3] if points_array.shape[1] > 3 else None

        # Generate dynamic filename
        filename = os.path.join(self.save_directory, f'pointcloud_{self.file_counter:06d}.pcd')

        # Save to PCD file using numpy
        self.save_pcd_file(filename, xyz, intensity)
        self.file_counter += 1

        # self.get_logger().info(f"Saved point cloud to {filename}")

    @staticmethod
    def save_pcd_file(filename, xyz, intensity):
        """ Save point cloud data to a PCD file """
        with open(filename, 'w') as pcd_file:
            pcd_file.write('# .PCD v0.7 - Point Cloud Data file format\n')
            pcd_file.write('VERSION 0.7\n')
            pcd_file.write(f'FIELDS x y z{" intensity" if intensity is not None else ""}\n')
            pcd_file.write(f'SIZE 4 4 4{" 4" if intensity is not None else ""}\n')
            pcd_file.write(f'TYPE F F F{" F" if intensity is not None else ""}\n')
            pcd_file.write(f'COUNT 1 1 1{" 1" if intensity is not None else ""}\n')
            pcd_file.write(f'WIDTH {xyz.shape[0]}\n')
            pcd_file.write('HEIGHT 1\n')
            pcd_file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            pcd_file.write(f'POINTS {xyz.shape[0]}\n')
            pcd_file.write('DATA ascii\n')

            # Combine xyz and intensity if available
            if intensity is not None:
                data = np.hstack((xyz, intensity[:, np.newaxis]))
            else:
                data = xyz

            np.savetxt(pcd_file, data, fmt='%f')

def main(args=None):
    rclpy.init(args=args)
    node = SavePointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
