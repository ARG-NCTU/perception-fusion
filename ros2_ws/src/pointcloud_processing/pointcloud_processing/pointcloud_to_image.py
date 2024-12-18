#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import os

class PointCloud2Image(Node):
    def __init__(self):
        super().__init__('pointcloud_to_image')

        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/cropped_pointcloud')
        self.declare_parameter('image_topic', '/radar_image/compressed')
        self.declare_parameter('parent_frame_id', 'radar_global_map')  # Parent frame ID
        self.declare_parameter('child_frame_id', 'radar')  # Child frame ID
        self.declare_parameter('save_images', False)
        self.declare_parameter('save_directory', '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images')

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.save_images = self.get_parameter('save_images').value
        self.save_directory = self.get_parameter('save_directory').value

        # Create the save directory if needed
        if self.save_images and not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.process_pointcloud,
            10
        )
        self.publisher = self.create_publisher(CompressedImage, self.image_topic, 10)

        self.bridge = CvBridge()
        self.image_counter = 0  # Counter for saved images

    def process_pointcloud(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        img_width = 640
        img_height = 480
        intensity_image = np.zeros((img_height, img_width), dtype=np.uint8)

        # Generate a grayscale intensity image
        for point in points:
            x, y, z, intensity = point
            u = int((x + 1) * (img_width / 2))   # Map x to pixel
            v = int((y + 1) * (img_height / 2)) # Map y to pixel
            if 0 <= u < img_width and 0 <= v < img_height:
                intensity_image[v, u] = min(255, int(intensity * 255))

        # Compress the image
        _, compressed_image = cv2.imencode('.jpg', intensity_image)
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = msg.header.stamp
        compressed_msg.header.frame_id = self.parent_frame_id  # Parent frame ID
        compressed_msg.child_frame_id = self.child_frame_id  # Set child frame ID (non-standard but for context)
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()

        # Publish the compressed image
        self.publisher.publish(compressed_msg)
        self.get_logger().info(f"Published compressed image with parent_frame: {self.parent_frame_id}, child_frame: {self.child_frame_id}")

        # Save the image to a file if enabled
        if self.save_images:
            filename = os.path.join(self.save_directory, f'image_{self.image_counter:06d}.jpg')
            cv2.imwrite(filename, intensity_image)
            self.image_counter += 1
            self.get_logger().info(f"Saved image to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2Image()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
