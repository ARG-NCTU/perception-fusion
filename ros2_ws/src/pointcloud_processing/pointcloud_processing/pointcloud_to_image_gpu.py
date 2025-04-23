#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cupy as cp
import cv2
import os
import rcl_interfaces.msg
from datetime import datetime


class PointCloud2ImageGPU(Node):
    def __init__(self):
        super().__init__('pointcloud_to_image_gpu')

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

        # Create save directory if needed
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
        # Unchanged from original
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

    def jet_colormap(self, intensity, max_val=255):
        """GPU-based reversed JET colormap: high intensity = red, low intensity = blue."""
        # Normalize intensity to [0, 1]
        intensity_normalized = intensity / max_val
        # Reverse the intensity for colormap (high intensity -> red, low -> blue)
        intensity_reversed = 1.0 - intensity_normalized
        
        # Define JET colormap transitions
        r = cp.zeros_like(intensity_reversed)
        g = cp.zeros_like(intensity_reversed)
        b = cp.zeros_like(intensity_reversed)

        # Blue to Cyan (0 to 0.25 in reversed scale, originally high intensity)
        mask = (intensity_reversed <= 0.25)
        b[mask] = 255 * (1 - 4 * intensity_reversed[mask])
        g[mask] = 255 * (4 * intensity_reversed[mask])

        # Cyan to Green (0.25 to 0.5)
        mask = (intensity_reversed > 0.25) & (intensity_reversed <= 0.5)
        g[mask] = 255
        b[mask] = 255 * (2 - 4 * intensity_reversed[mask])

        # Green to Yellow (0.5 to 0.75)
        mask = (intensity_reversed > 0.5) & (intensity_reversed <= 0.75)
        r[mask] = 255 * (4 * intensity_reversed[mask] - 2)
        g[mask] = 255

        # Yellow to Red (0.75 to 1.0, originally low intensity)
        mask = (intensity_reversed > 0.75)
        r[mask] = 255
        g[mask] = 255 * (4 - 4 * intensity_reversed[mask])

        return cp.stack([r, g, b], axis=-1).astype(cp.uint8)

    def process_pointcloud(self, msg):
        # Read points from PointCloud2 (CPU)
        field_names = [field.name for field in msg.fields]
        points_structured = pc2.read_points(msg, field_names=tuple(field_names), skip_nans=True)
        
        # Convert to flat array and move to GPU (single transfer)
        points_np = np.array([tuple(point) for point in points_structured], dtype=np.float32)
        if len(points_np) == 0:
            return
        points = cp.array(points_np)  # Transfer to GPU once

        # Extract xyz and intensity on GPU
        xyz = points[:, :3]
        intensity = points[:, 3] if points.shape[1] > 3 else None

        # Image setup on GPU
        img_width, img_height = 480, 480
        intensity_image = cp.zeros((img_height, img_width, 3), dtype=cp.uint8)
        range_min, range_max = -self.range, self.range

        # Compute pixel coordinates on GPU
        u = ((xyz[:, 1] - range_min) / (range_max - range_min) * img_width - img_width / 2).astype(cp.int32)
        v = ((xyz[:, 0] - range_min) / (range_max - range_min) * img_height - img_height / 2).astype(cp.int32)
        u = img_height // 2 + u
        v = img_width // 2 - v

        # Filter valid coordinates on GPU
        mask = (0 <= u) & (u < img_width) & (0 <= v) & (v < img_height)
        u, v = u[mask], v[mask]

        if intensity is not None:
            intensity_normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
            intensity_norm = intensity_normalized[mask]

            if self.use_grayscale:
                gray_values = (intensity_norm * 255).astype(cp.uint8)
                colors = cp.stack([gray_values, gray_values, gray_values], axis=-1)
            else:
                # Use reversed GPU-based JET colormap
                colors = self.jet_colormap(intensity_norm * 255)
            
            # Assign colors on GPU
            intensity_image[u, v] = colors
        else:
            intensity_image[u, v] = cp.array([255, 255, 255])

        # Single transfer back to CPU for publishing and saving
        intensity_image = cp.asnumpy(intensity_image)
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

        # Compress and publish (CPU)
        _, compressed_image = cv2.imencode('.jpg', intensity_image)
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()  # Use current time
        compressed_msg.format = "jpeg"
        compressed_msg.data = compressed_image.tobytes()
        self.publisher.publish(compressed_msg)

        # Update last message and timestamp
        self.last_compressed_msg = compressed_msg
        self.last_publish_time = self.get_clock().now().nanoseconds * 1e-9

        # Save if enabled (CPU)
        if self.save_images:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamp_str = f"{timestamp:.6f}".replace(".", "_")
            filename = os.path.join(self.current_save_directory, f'image_{timestamp_str}.jpg')
            cv2.imwrite(filename, intensity_image_np)
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
    node = PointCloud2ImageGPU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)