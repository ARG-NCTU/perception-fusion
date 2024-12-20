#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class SimpleLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('simple_laserscan_publisher')

        # Declare parameters
        self.declare_parameter('sub_laserscan_topic', '/halo_radar/cropped_scan')
        self.declare_parameter('pub_laserscan_topic', '/halo_radar/republished_scan')

        # Get parameters
        self.sub_laserscan_topic = self.get_parameter('sub_laserscan_topic').value
        self.pub_laserscan_topic = self.get_parameter('pub_laserscan_topic').value

        # QoS profile for subscription
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # QoS profile for publishing
        self.qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Subscriber and Publisher
        self.laserscan_sub = self.create_subscription(
            LaserScan,
            self.sub_laserscan_topic,
            self.laserscan_qos_callback,
            self.qos
        )
        self.laserscan_pub = self.create_publisher(
            LaserScan,
            self.pub_laserscan_topic,
            self.qos_pub
        )

        self.get_logger().info(f"Subscribed to LaserScan topic: {self.sub_laserscan_topic}")
        self.get_logger().info(f"Republishing LaserScan to topic: {self.pub_laserscan_topic}")

    def laserscan_qos_callback(self, msg):
        """
        Callback function to simply republish the LaserScan data.
        """
        self.get_logger().debug(f"Republishing LaserScan with {len(msg.ranges)} ranges.")
        self.laserscan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaserScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(args=sys.argv)

