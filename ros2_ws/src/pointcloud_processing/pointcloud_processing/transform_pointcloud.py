#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud
from tf2_ros import Buffer, TransformListener
import numpy as np


class TransformPointcloud(Node):
    def __init__(self):
        super().__init__('transform_pointcloud') # Initialize the node

        # Declare parameters
        self.declare_parameter('sub_radar_topic', '/halo_radar/cropped_pointcloud')  # Input topic
        self.declare_parameter('pub_radar_topic', '/halo_radar/transformed_pointcloud')  # Output topic
        self.declare_parameter('parent_frame_id', 'map')  # Parent frame
        self.declare_parameter('child_frame_id', 'base_link')  # Target child frame

        # Get parameters
        self.sub_radar_topic = self.get_parameter('sub_radar_topic').get_parameter_value().string_value
        self.pub_radar_topic = self.get_parameter('pub_radar_topic').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers and Publishers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.sub_radar_topic,
            self.pointcloud_callback,
            10
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.pub_radar_topic,
            10
        )

        self.get_logger().info(f"Subscribed to {self.sub_radar_topic}, publishing to {self.pub_radar_topic}")

    def pointcloud_callback(self, msg):
        """
        Callback function to transform and republish PointCloud2 data.
        """
        try:
            # Log fields in the incoming PointCloud2 message
            field_names = [field.name for field in msg.fields]
            self.get_logger().info(f"Fields in incoming PointCloud2 message: {field_names}")

            # Determine if intensity is available
            has_intensity = 'intensity' in field_names

            # Get the latest transform between parent_frame_id and child_frame_id
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,  # Use child_frame_id from the message
                rclpy.time.Time()  # Get the latest available transform
            )

            # Extract translation and rotation from the transform
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            rotation_matrix = self.quaternion_to_rotation_matrix(transform.transform.rotation)

            # Read points from PointCloud2
            points = list(read_points(msg, field_names=tuple(field_names), skip_nans=True))
            self.get_logger().info(f"Read {len(points)} points from PointCloud2.")

            if len(points) == 0:
                self.get_logger().error("PointCloud2 message contains no valid points.")
                return

            # Convert points into a 2D numpy array
            points_array = np.array([list(point) for point in points], dtype=float)

            # Separate the 3D points and intensity if available
            xyz = points_array[:, :3]  # Extract (x, y, z)
            intensity = points_array[:, 3] if has_intensity else None

            # Apply transformation: rotate + translate points
            transformed_xyz = np.dot(xyz, rotation_matrix.T) + translation

            # Combine transformed points with intensity if available
            if has_intensity:
                transformed_points = np.hstack((transformed_xyz, intensity.reshape(-1, 1)))
            else:
                transformed_points = transformed_xyz

            # Re-create a new PointCloud2 message
            transformed_cloud = create_cloud(msg.header, msg.fields, transformed_points)

            # Update the header frame_id to the parent_frame_id
            transformed_cloud.header.frame_id = self.parent_frame_id

            # Publish the transformed pointcloud
            self.pointcloud_pub.publish(transformed_cloud)
            self.get_logger().info("Transformed and published PointCloud2 message.")

        except Exception as e:
            self.get_logger().error(f"Failed to transform PointCloud2: {e}")

    @staticmethod
    def filter_valid_points(points, num_fields):
        """
        Filter out invalid points that don't have the expected number of fields.
        """
        valid_points = []
        for point in points:
            if len(point) == num_fields:
                valid_points.append(point)
        return valid_points

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """
        Convert a quaternion to a 3x3 rotation matrix.
        """
        # Extract quaternion components
        x, y, z, w = q.x, q.y, q.z, q.w

        # Compute the rotation matrix
        rotation_matrix = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ])

        return rotation_matrix

def main(args=None):
    rclpy.init(args=args)
    node = TransformPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)