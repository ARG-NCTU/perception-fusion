#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from math import atan2, cos, sin, pi
import numpy as np


class TransformLaserScan(Node):
    def __init__(self):
        super().__init__('transform_laserscan')

        # Declare parameters
        self.declare_parameter('sub_laserscan_topic', '/halo_radar/cropped_scan')
        self.declare_parameter('pub_laserscan_topic', '/halo_radar/transformed_scan')
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        # Get parameters
        self.sub_laserscan_topic = self.get_parameter('sub_laserscan_topic').get_parameter_value().string_value
        self.pub_laserscan_topic = self.get_parameter('pub_laserscan_topic').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # QoS profile
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers and Publishers
        self.laserscan_sub = self.create_subscription(
            LaserScan,
            self.sub_laserscan_topic,
            self.laserscan_callback,
            self.qos
        )
        self.laserscan_pub = self.create_publisher(
            LaserScan,
            self.pub_laserscan_topic,
            self.qos_pub
        )

        self.get_logger().info(f"Subscribed to LaserScan topic: {self.sub_laserscan_topic}")
        self.get_logger().info(f"Publishing transformed LaserScan to: {self.pub_laserscan_topic}")

    def laserscan_callback(self, msg):
        """
        Callback function to transform and rotate LaserScan data.
        """
        try:
            # Lookup the transform from the child frame to the parent frame
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,
                rclpy.time.Time()
            )

            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Convert quaternion to yaw (rotation around z-axis)
            yaw = 2.0 * atan2(rotation.z, rotation.w)

            # Transform LaserScan data
            transformed_scan = self.transform_laserscan(msg, yaw, translation)

            # Publish the transformed LaserScan
            self.laserscan_pub.publish(transformed_scan)

        except Exception as e:
            self.get_logger().warn(f"Could not transform LaserScan: {str(e)}")

    def transform_laserscan(self, scan: LaserScan, yaw: float, translation) -> LaserScan:
        """
        Apply rotation and translation to a LaserScan message.

        Args:
            scan (LaserScan): Input LaserScan message.
            yaw (float): Rotation angle in radians.
            translation: Translation (x, y, z).

        Returns:
            LaserScan: Transformed LaserScan message.
        """
        # Convert LaserScan ranges to Cartesian coordinates
        cartesian_points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max:
                x = r * cos(angle)
                y = r * sin(angle)
                cartesian_points.append((x, y))
            angle += scan.angle_increment

        # Apply rotation and translation
        transformed_points = []
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        for (x, y) in cartesian_points:
            # Rotate
            x_rot = cos_yaw * x - sin_yaw * y
            y_rot = sin_yaw * x + cos_yaw * y

            # Translate
            x_trans = x_rot + translation.x
            y_trans = y_rot + translation.y

            transformed_points.append((x_trans, y_trans))

        # Convert transformed points back to polar coordinates
        transformed_ranges = []
        transformed_angles = []
        for (x, y) in transformed_points:
            r = (x**2 + y**2)**0.5
            theta = atan2(y, x)  # Angle relative to the origin
            transformed_ranges.append(r)
            transformed_angles.append(theta)

        # Dynamically compute new angle_min and angle_max
        new_angle_min = min(transformed_angles)
        new_angle_max = max(transformed_angles)

        # Update the LaserScan message
        transformed_scan = LaserScan()
        transformed_scan.header = scan.header
        transformed_scan.header.frame_id = self.parent_frame_id
        transformed_scan.angle_min = new_angle_min
        transformed_scan.angle_max = new_angle_max
        transformed_scan.angle_increment = scan.angle_increment  # Increment stays the same
        transformed_scan.time_increment = scan.time_increment
        transformed_scan.scan_time = scan.scan_time
        transformed_scan.range_min = scan.range_min
        transformed_scan.range_max = scan.range_max
        transformed_scan.ranges = transformed_ranges

        # Update intensities (optional, no change needed)
        if len(scan.intensities) > 0:
            transformed_scan.intensities = scan.intensities
        else:
            transformed_scan.intensities = []

        return transformed_scan



def main(args=None):
    rclpy.init(args=args)
    node = TransformLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
