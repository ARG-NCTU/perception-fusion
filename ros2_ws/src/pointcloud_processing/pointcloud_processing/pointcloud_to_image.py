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
        self.declare_parameter('pointcloud_topic', '/halo_radar/transformed_pointcloud')
        self.declare_parameter('image_topic', '/halo_radar/radar_image/compressed')
        self.declare_parameter('save_images', False)
        self.declare_parameter('save_directory', '/home/arg/perception-fusion/ros2_ws/src/pointcloud_processing/data/radar_images')
        self.declare_parameter('range', 120.0)  # Define range in meters for normalization
        self.declare_parameter('use_grayscale', False)  # Use grayscale visualization if True
        self.declare_parameter('circle_radius', 3)  # Circle radius for visualization

        # Get parameters
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.save_images = self.get_parameter('save_images').value
        self.save_directory = self.get_parameter('save_directory').value
        self.range = self.get_parameter('range').value
        self.use_grayscale = self.get_parameter('use_grayscale').value
        self.circle_radius = self.get_parameter('circle_radius').value

        # Create the save directory if needed
        if self.save_images:
            os.makedirs(self.save_directory, exist_ok=True)

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

        # Generate an intensity image with a rainbow colormap or grayscale
        img_width = 480
        img_height = 480
        intensity_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        # Set up the normalization range
        range_min, range_max = -self.range, self.range

        if intensity is not None:
            intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
            for i, (x, y, z) in enumerate(xyz):
                # Normalize x and y to image pixels, with the center of the image as 0, 0
                u = int((y - range_min) / (range_max - range_min) * img_width - img_width / 2)
                v = int((x - range_min) / (range_max - range_min) * img_height - img_height / 2)
                u = img_height // 2 - u
                v = img_width // 2 - v
                # self.get_logger().info(f"u: {u}, v: {v}")
                if 0 <= u < img_width and 0 <= v < img_height:
                    if self.use_grayscale:
                        gray_value = int(intensity_normalized[i] * 255)
                        color = (gray_value, gray_value, gray_value)
                    else:
                        color = cv2.applyColorMap(
                            np.uint8(intensity_normalized[i] * 255).reshape((1, 1)),
                            cv2.COLORMAP_JET
                        )[0, 0]  # Apply rainbow colormap
                    cv2.circle(intensity_image, (u, v), self.circle_radius, tuple(map(int, color)), -1)
        else:
            for (x, y, z) in xyz:
                u = int((y - range_min) / (range_max - range_min) * img_width - img_width / 2)
                v = int((x - range_min) / (range_max - range_min) * img_height - img_height / 2)
                u = img_height // 2 - u
                v = img_width // 2 - v
                # self.get_logger().info(f"u: {u}, v: {v}")
                if 0 <= u < img_width and 0 <= v < img_height:
                    cv2.circle(intensity_image, (u, v), self.circle_radius, (255, 255, 255), -1)  # White color for points

        # Compress the image
        _, compressed_image = cv2.imencode('.jpg', intensity_image)
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = msg.header.stamp
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()

        # Publish the compressed image
        self.publisher.publish(compressed_msg)
        # self.get_logger().info(f"Published compressed image to {self.image_topic}")

        # Save the image to a file if enabled
        if self.save_images:
            filename = os.path.join(self.save_directory, f'image_{self.image_counter:06d}.jpg')
            cv2.imwrite(filename, intensity_image)
            self.image_counter += 1
            # self.get_logger().info(f"Saved image to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2Image()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(args=sys.argv)
