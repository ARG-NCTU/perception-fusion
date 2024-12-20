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
        self.declare_parameter('sub_laserscan_topic', '/halo_radar/republished_scan')
        self.declare_parameter('pub_laserscan_topic', '/halo_radar/transformed_scan')
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        # Get parameters
        self.sub_laserscan_topic = self.get_parameter('sub_laserscan_topic').value
        self.pub_laserscan_topic = self.get_parameter('pub_laserscan_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers and Publishers
        self.laserscan_sub = self.create_subscription(
            LaserScan,
            self.sub_laserscan_topic,
            self.laserscan_callback,
            10
        )
        self.laserscan_pub = self.create_publisher(
            LaserScan,
            self.pub_laserscan_topic,
            10
        )

        self.get_logger().info(f"Subscribed to LaserScan topic: {self.sub_laserscan_topic}")
        self.get_logger().info(f"Publishing transformed LaserScan to: {self.pub_laserscan_topic}")

    def laserscan_callback(self, msg):
        """
        Callback function to transform and rotate LaserScan data.
        """
        try:
            # Look up transform
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,
                rclpy.time.Time()
            )
            rotation = transform.transform.rotation

            # Convert quaternion to yaw and normalize it
            yaw = 2.0 * atan2(rotation.z, rotation.w)
            yaw = self.normalize_angle_rad(yaw)  # Normalize yaw to [-pi, pi)

            # Process LaserScan
            transformed_scan = self.transform_laserscan(msg, yaw)
            self.laserscan_pub.publish(transformed_scan)
        
        except Exception as e:
            self.get_logger().warn(f"Could not transform LaserScan: {str(e)}")


    def transform_laserscan(self, scan: LaserScan, yaw: float) -> LaserScan:
        """
        Apply rotation and translation to a LaserScan message.

        Args:
            scan (LaserScan): Input LaserScan message.
            yaw (float): Rotation angle in radians.
            translation: Translation (x, y, z).

        Returns:
            LaserScan: Transformed LaserScan message.
        """
        # Adjust angles based on yaw and normalize them
        new_angle_min = self.normalize_angle_rad(scan.angle_min + yaw)
        new_angle_max = self.normalize_angle_rad(scan.angle_max + yaw)

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
        transformed_scan.ranges = scan.ranges

        # Update intensities (optional, no change needed)
        if len(scan.intensities) > 0:
            transformed_scan.intensities = scan.intensities
        else:
            transformed_scan.intensities = []

        return transformed_scan

    def normalize_angle_rad(self, angle):
        """
        Normalize an angle to the range [-pi, pi).
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi





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
