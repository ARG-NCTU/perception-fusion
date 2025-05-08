import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class CameraMergeNode(Node):
    def __init__(self):
        super().__init__('camera_merge_node')

        self.declare_parameter('left_topic', '/camera1_fix/pcd_projection/compressed')
        self.declare_parameter('mid_topic', '/camera2_fix/pcd_projection/compressed')
        self.declare_parameter('right_topic', '/camera3_fix/pcd_projection/compressed')
        self.declare_parameter('back_topic', '/camera4_fix/pcd_projection/compressed')
        self.declare_parameter('output_topic', '/camera_merged/compressed')

        self.left_topic = self.get_parameter('left_topic').value
        self.mid_topic = self.get_parameter('mid_topic').value
        self.right_topic = self.get_parameter('right_topic').value
        self.back_topic = self.get_parameter('back_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.bridge = CvBridge()
        self.queues = {
            'left': deque(maxlen=1),
            'mid': deque(maxlen=1),
            'right': deque(maxlen=1),
            'back': deque(maxlen=1)
        }

        self.sub_left = self.create_subscription(CompressedImage, self.left_topic, self.cb_left, 10)
        self.sub_mid = self.create_subscription(CompressedImage, self.mid_topic, self.cb_mid, 10)
        self.sub_right = self.create_subscription(CompressedImage, self.right_topic, self.cb_right, 10)
        self.sub_back = self.create_subscription(CompressedImage, self.back_topic, self.cb_back, 10)

        self.pub = self.create_publisher(CompressedImage, self.output_topic, 10)
        self.get_logger().info("Camera Merge Node started.")

    def cb_left(self, msg):
        self.queues['left'].append(msg)
        self.try_publish()

    def cb_mid(self, msg):
        self.queues['mid'].append(msg)
        self.try_publish()

    def cb_right(self, msg):
        self.queues['right'].append(msg)
        self.try_publish()

    def cb_back(self, msg):
        self.queues['back'].append(msg)
        self.try_publish()

    def try_publish(self):
        if all(len(q) > 0 for q in self.queues.values()):
            left_img = self.bridge.compressed_imgmsg_to_cv2(self.queues['left'].popleft())
            mid_img = self.bridge.compressed_imgmsg_to_cv2(self.queues['mid'].popleft())
            right_img = self.bridge.compressed_imgmsg_to_cv2(self.queues['right'].popleft())
            back_img = self.bridge.compressed_imgmsg_to_cv2(self.queues['back'].popleft())

            top_row = np.hstack([left_img, mid_img, right_img])
            bottom_row = np.hstack([np.zeros_like(left_img), back_img, np.zeros_like(right_img)])
            merged = np.vstack([top_row, bottom_row])

            msg = self.bridge.cv2_to_compressed_imgmsg(merged)
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()