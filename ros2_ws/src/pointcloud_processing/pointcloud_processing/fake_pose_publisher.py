#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class FakePosePublisher(Node):
    def __init__(self):
        super().__init__('fake_pose_publisher')
        # Declare and get parameters
        self.declare_parameter('pose_pub_topic', '/wamv/localization/pose')
        self.pose_pub_topic = self.get_parameter('pose_pub_topic').value
        self.publisher = self.create_publisher(PoseStamped, self.pose_pub_topic, 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # Publish at 10 Hz
        self.angle = 0.0

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'  # Global frame

        # Set fake position (you can customize this)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        # Set fake orientation (rotating around Z-axis)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(self.angle / 2)
        pose.pose.orientation.w = math.cos(self.angle / 2)

        self.angle = (self.angle + 0.01) % (2 * math.pi) # Increment angle
        self.publisher.publish(pose)
        # self.get_logger().info(f"Published fake pose with angle: {self.angle}")

def main(args=None):
    rclpy.init(args=args)
    node = FakePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
