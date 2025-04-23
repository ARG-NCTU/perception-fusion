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

        # Timer to enforce FPS >= 1
        self.timer = self.create_timer(0.5, self.timer_callback)  # Check every 0.5s

        # Store last message and timestamp
        self.last_compressed_msg = None
        self.last_publish_time = None

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
        points_structured = pc2.read_points(msg, field_names=tuple(field_names), skip_nans=True)
        
        # Convert structured data to a flat NumPy array
        points = np.array([tuple(point) for point in points_structured], dtype=np.float32)
        if len(points) == 0:
            return

        # Extract xyz and intensity
        xyz = points[:, :3]
        intensity = points[:, 3] if points.shape[1] > 3 else None

        # Image setup
        img_width, img_height = 480, 480
        intensity_image = np.zeros((img_height, img_width, 3), dtype=np.uint8)
        range_min, range_max = -self.range, self.range

        # Compute pixel coordinates in bulk
        u = ((xyz[:, 1] - range_min) / (range_max - range_min) * img_width - img_width / 2).astype(int)
        v = ((xyz[:, 0] - range_min) / (range_max - range_min) * img_height - img_height / 2).astype(int)
        u = img_height // 2 + u
        v = img_width // 2 - v

        # Filter valid coordinates
        mask = (0 <= u) & (u < img_width) & (0 <= v) & (v < img_height)
        u, v = u[mask], v[mask]

        if intensity is not None:
            intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
            intensity_norm = intensity_normalized[mask]

            if self.use_grayscale:
                gray_values = (intensity_norm * 255).astype(np.uint8)
                colors = np.stack([gray_values, gray_values, gray_values], axis=-1)
            else:
                color_vals = np.uint8(intensity_norm * 255)
                if color_vals.size == 0:
                    return  # No valid intensity points, skip processing

                # Apply colormap safely
                color_mapped = cv2.applyColorMap(color_vals.reshape(-1, 1), cv2.COLORMAP_JET)
                colors = color_mapped.reshape(-1, 3)
                # colors = cv2.applyColorMap(np.uint8(intensity_norm * 255), cv2.COLORMAP_JET)[:, 0, :]
            
            # Assign colors directly (this is still per-pixel, see next optimization for further speedup)
            intensity_image[u, v] = colors
        else:
            intensity_image[u, v] = [255, 255, 255]

        intensity_image = cv2.rotate(intensity_image, cv2.ROTATE_90_CLOCKWISE)

        # ===== Add grid lines every 10 meters (i.e., every 40 pixels) =====
        grid_spacing = int(img_width / (self.range / 10))  # 10m per cell → 40px if 480/12
        for i in range(0, img_width, grid_spacing):
            cv2.line(intensity_image, (i, 0), (i, img_height), color=(100, 100, 100), thickness=1)
        for j in range(0, img_height, grid_spacing):
            cv2.line(intensity_image, (0, j), (img_width, j), color=(100, 100, 100), thickness=1)

        # Draw X and Y axes with arrows
        center_x, center_y = img_width // 2, img_height // 2
        axis_length_px = int((10 / self.range) * img_width)  # 10m → 40 pixels

        # X-axis → Red arrow upward (positive X)
        cv2.arrowedLine(intensity_image,
                        (center_x, center_y),
                        (center_x, center_y - axis_length_px),
                        color=(0, 0, 255),  # Red
                        thickness=2, tipLength=0.15)

        # Y-axis → Green arrow to the left (positive Y)
        cv2.arrowedLine(intensity_image,
                        (center_x, center_y),
                        (center_x - axis_length_px, center_y),
                        color=(0, 255, 0),  # Green
                        thickness=2, tipLength=0.15)


        # Compress and publish
        _, compressed_image = cv2.imencode('.jpg', intensity_image)
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = msg.header.stamp
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()
        self.publisher.publish(compressed_msg)

        # Update last message and timestamp
        self.last_compressed_msg = compressed_msg
        self.last_publish_time = self.get_clock().now().nanoseconds * 1e-9

        # Save if enabled
        if self.save_images:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamp_str = f"{timestamp:.6f}".replace(".", "_")
            filename = os.path.join(self.current_save_directory, f'image_{timestamp_str}.jpg')
            cv2.imwrite(filename, intensity_image)
            self.get_logger().info(f"Saved image to {filename}")

    def timer_callback(self):
        """Republish last message if FPS < 1."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_compressed_msg is not None and self.last_publish_time is not None:
            time_since_last_publish = current_time - self.last_publish_time
            if time_since_last_publish >= 0.5:  # More than 1 second since last publish (FPS < 1)
                # Update timestamp to current time
                self.last_compressed_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(self.last_compressed_msg)
                # self.last_publish_time = current_time
                self.get_logger().info("Republished last image to maintain FPS >= 1")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2Image()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(args=sys.argv)
