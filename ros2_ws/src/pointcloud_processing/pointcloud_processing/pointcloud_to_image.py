#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import rcl_interfaces.msg
from datetime import datetime


class PointCloud2Image(Node):
    def __init__(self):
        super().__init__('pointcloud_to_image')

        # Declare parameters
        self.declare_parameter('pointcloud_topic', '/halo_radar/cropped_pointcloud')
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
            now = datetime.now()
            timestamp_str = now.strftime("%Y%m%d_%H%M%S")
            self.current_save_directory = os.path.join(self.save_directory, timestamp_str)
            os.makedirs(self.current_save_directory, exist_ok=True)
            self.get_logger().info(f"Created new directory: {self.current_save_directory}")

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.process_pointcloud,
            10
        )
        self.publisher = self.create_publisher(CompressedImage, self.image_topic, 10)

        self.bridge = CvBridge()
        self.current_save_directory = self.save_directory  # Used for dynamically updating save location

        # Set parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'pointcloud_topic':
                self.pointcloud_topic = param.value
                self.get_logger().info(f"Updated pointcloud_topic to {self.pointcloud_topic}")
                self.subscription = self.create_subscription(
                    PointCloud2,
                    self.pointcloud_topic,
                    self.process_pointcloud,
                    10
                )
            elif param.name == 'image_topic':
                self.image_topic = param.value
                self.get_logger().info(f"Updated image_topic to {self.image_topic}")
                self.publisher = self.create_publisher(CompressedImage, self.image_topic, 10)
            elif param.name == 'save_images':
                self.save_images = param.value
                self.get_logger().info(f"Updated save_images to {self.save_images}")
                if self.save_images:
                    # Create a new directory based on current time
                    now = datetime.now()
                    timestamp_str = now.strftime("%Y%m%d_%H%M%S")
                    self.current_save_directory = os.path.join(self.save_directory, timestamp_str)
                    os.makedirs(self.current_save_directory, exist_ok=True)
                    self.get_logger().info(f"Created new directory: {self.current_save_directory}")
            elif param.name == 'save_directory':
                self.save_directory = param.value
                self.get_logger().info(f"Updated save_directory to {self.save_directory}")
                if self.save_images:
                    # Update current save directory with new save_directory
                    now = datetime.now()
                    timestamp_str = now.strftime("%Y%m%d_%H%M%S")
                    self.current_save_directory = os.path.join(self.save_directory, timestamp_str)
                    os.makedirs(self.current_save_directory, exist_ok=True)
            elif param.name == 'range':
                self.range = param.value
                self.get_logger().info(f"Updated range to {self.range}")
            elif param.name == 'use_grayscale':
                self.use_grayscale = param.value
                self.get_logger().info(f"Updated use_grayscale to {self.use_grayscale}")
            elif param.name == 'circle_radius':
                self.circle_radius = param.value
                self.get_logger().info(f"Updated circle_radius to {self.circle_radius}")
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def process_pointcloud(self, msg):
        # Read points from PointCloud2
        field_names = [field.name for field in msg.fields]
        points = list(pc2.read_points(msg, field_names=tuple(field_names), skip_nans=True))
        if len(points) == 0:
            return

        # Convert points into a 2D numpy array
        points_array = np.array([list(point) for point in points], dtype=float)

        # Separate the 3D points and intensity if available
        xyz = points_array[:, :3]  # Extract (x, y, z)
        intensity = points_array[:, 3] if points_array.shape[1] > 3 else None

        # Generate an intensity image
        img_width = 480
        img_height = 480
        intensity_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)

        range_min, range_max = -self.range, self.range

        if intensity is not None:
            intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
            for i, (x, y, z) in enumerate(xyz):
                u = int((y - range_min) / (range_max - range_min) * img_width - img_width / 2)
                v = int((x - range_min) / (range_max - range_min) * img_height - img_height / 2)
                u = img_height // 2 - u
                v = img_width // 2 - v
                if 0 <= u < img_width and 0 <= v < img_height:
                    if self.use_grayscale:
                        gray_value = int(intensity_normalized[i] * 255)
                        color = (gray_value, gray_value, gray_value)
                    else:
                        color = cv2.applyColorMap(
                            np.uint8(intensity_normalized[i] * 255).reshape((1, 1)),
                            cv2.COLORMAP_JET
                        )[0, 0]
                    cv2.circle(intensity_image, (u, v), self.circle_radius, tuple(map(int, color)), -1)
        else:
            for (x, y, z) in xyz:
                u = int((y - range_min) / (range_max - range_min) * img_width - img_width / 2)
                v = int((x - range_min) / (range_max - range_min) * img_height - img_height / 2)
                u = img_height // 2 - u
                v = img_width // 2 - v
                if 0 <= u < img_width and 0 <= v < img_height:
                    cv2.circle(intensity_image, (u, v), self.circle_radius, (255, 255, 255), -1)

        # Compress the image
        _, compressed_image = cv2.imencode('.jpg', intensity_image)
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = msg.header.stamp
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()

        # Publish the compressed image
        self.publisher.publish(compressed_msg)

        # Save the image to a file if enabled
        if self.save_images:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamp_str = f"{timestamp:.6f}".replace(".", "_")
            filename = os.path.join(self.current_save_directory, f'image_{timestamp_str}.jpg')
            cv2.imwrite(filename, intensity_image)
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
