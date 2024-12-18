#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster

class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_broadcaster')

        # Declare and get parameters
        self.declare_parameter('dynamic_pose', '/wamv/localization/pose')
        self.sub_pose_topic = self.get_parameter('dynamic_pose').value

        self.declare_parameter('parent_frame_id', 'map')
        self.parent_frame_id = self.get_parameter('parent_frame_id').value

        self.declare_parameter('child_frame_id', 'base_link')
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # Log configuration
        self.get_logger().info(f"Broadcasting pose from topic: {self.sub_pose_topic}")
        self.get_logger().info(f"Parent frame: {self.parent_frame_id}")
        self.get_logger().info(f"Child frame: {self.child_frame_id}")

        # Transform Broadcaster
        self.tfb_ = TransformBroadcaster(self)

        # Subscriber to the PoseStamped topic
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            self.sub_pose_topic,
            self.handle_pose,
            10
        )

    def handle_pose(self, msg):
        # Create a TransformStamped message
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = self.parent_frame_id  # Global frame
        tfs.child_frame_id = self.child_frame_id  # Dynamic frame

        # Extract position from PoseStamped message
        tfs.transform.translation.x = msg.pose.position.x
        tfs.transform.translation.y = msg.pose.position.y
        tfs.transform.translation.z = msg.pose.position.z  # Use z = 0.0 if flat

        # Extract orientation (quaternion) from PoseStamped message
        tfs.transform.rotation.x = msg.pose.orientation.x
        tfs.transform.rotation.y = msg.pose.orientation.y
        tfs.transform.rotation.z = msg.pose.orientation.z
        tfs.transform.rotation.w = msg.pose.orientation.w

        # Broadcast the transform
        self.tfb_.sendTransform(tfs)
        self.get_logger().info(f"Broadcasted transform from {self.parent_frame_id} to {self.child_frame_id}")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
