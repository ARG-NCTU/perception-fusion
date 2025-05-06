#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
import datetime
import threading
import json

class SavePerceptions(Node):
    def __init__(self):
        super().__init__('save_perceptions')

        # Declare parameters
        self.declare_parameter('save_base_directory', '/home/arg/perception-fusion/data/argnctu-perception/samples')
        self.declare_parameter('topics', '{}')

        self.base_dir = self.get_parameter('save_base_directory').value
        topics_str = self.get_parameter('topics').get_parameter_value().string_value
        self.topics = json.loads(topics_str)

        # Create sub-directories
        self.dirs = {
            'CAMERA_LEFT': os.path.join(self.base_dir, 'CAMERA_LEFT'),
            'CAMERA_MID': os.path.join(self.base_dir, 'CAMERA_MID'),
            'CAMERA_RIGHT': os.path.join(self.base_dir, 'CAMERA_RIGHT'),
            'CAMERA_STITCHED': os.path.join(self.base_dir, 'CAMERA_STITCHED'),
            'CAMERA_BACK': os.path.join(self.base_dir, 'CAMERA_BACK'),
            'RADAR_IMAGE': os.path.join(self.base_dir, 'RADAR_IMAGE'),
            'RADAR_PCD': os.path.join(self.base_dir, 'RADAR_PCD'),
            'LIDAR_IMAGE': os.path.join(self.base_dir, 'LIDAR_IMAGE'),
            'LIDAR_PCD': os.path.join(self.base_dir, 'LIDAR_PCD')
        }
        for path in self.dirs.values():
            os.makedirs(path, exist_ok=True)

        self.bridge = CvBridge()

        # Latest timestamps
        self.latest_radar_stamp = "00000000-00000000"
        self.latest_lidar_stamp = "00000000-00000000"
        self.latest_camera_stitched_stamp = "00000000-00000000"

        # Save control
        self.last_saved_time = {
            'CAMERA_LEFT': 0.0,
            'CAMERA_MID': 0.0,
            'CAMERA_RIGHT': 0.0,
            'CAMERA_STITCHED': 0.0,
            'CAMERA_BACK': 0.0
        }
        self.save_interval = 1  # seconds

        # Locks
        self.lock_radar = threading.Lock()
        self.lock_lidar = threading.Lock()

        # Subscriptions
        self.create_subscription(CompressedImage, self.topics['camera_left'], lambda msg: self.image_callback(msg, 'CAMERA_LEFT'), 10)
        self.create_subscription(CompressedImage, self.topics['camera_mid'], lambda msg: self.image_callback(msg, 'CAMERA_MID'), 10)
        self.create_subscription(CompressedImage, self.topics['camera_right'], lambda msg: self.image_callback(msg, 'CAMERA_RIGHT'), 10)
        self.create_subscription(CompressedImage, self.topics['camera_stitched'], lambda msg: self.image_callback(msg, 'CAMERA_STITCHED'), 10)
        self.create_subscription(CompressedImage, self.topics['camera_back'], lambda msg: self.image_callback(msg, 'CAMERA_BACK'), 10)
        self.create_subscription(CompressedImage, self.topics['radar_image'], self.radar_image_callback, 10)
        self.create_subscription(PointCloud2, self.topics['radar_pcd'], self.radar_pcd_callback, 10)
        self.create_subscription(CompressedImage, self.topics['lidar_image'], self.lidar_image_callback, 10)
        self.create_subscription(PointCloud2, self.topics['lidar_pcd'], self.lidar_pcd_callback, 10)

    def get_current_timestamp_str(self):
        dt = datetime.datetime.now()
        return dt.strftime("%Y%m%d-%H%M%S")

    def radar_image_callback(self, msg):
        radar_stamp = self.get_current_timestamp_str()

        # with self.lock_radar:
        filename_base = f"r_{radar_stamp}_l_{self.latest_lidar_stamp}_c_{self.latest_camera_stitched_stamp}"
        self.latest_radar_stamp = radar_stamp

        save_path = os.path.join(self.dirs['RADAR_IMAGE'], filename_base + '.png')
        self.save_image(msg, save_path)

    def radar_pcd_callback(self, msg):
        radar_stamp = self.get_current_timestamp_str()

        # with self.lock_radar:
        filename_base = f"r_{radar_stamp}_l_{self.latest_lidar_stamp}_c_{self.latest_camera_stitched_stamp}"
        self.latest_radar_stamp = radar_stamp

        save_path = os.path.join(self.dirs['RADAR_PCD'], filename_base + '.pcd')
        self.save_pcd(msg, save_path)

    def lidar_image_callback(self, msg):
        if self.lock_radar.locked():
            return

        lidar_stamp = self.get_current_timestamp_str()

        # with self.lock_lidar:
        self.latest_lidar_stamp = lidar_stamp
        filename_base = f"r_{self.latest_radar_stamp}_l_{lidar_stamp}_c_{self.latest_camera_stitched_stamp}"

        save_path = os.path.join(self.dirs['LIDAR_IMAGE'], filename_base + '.png')
        self.save_image(msg, save_path)

    def lidar_pcd_callback(self, msg):
        if self.lock_radar.locked():
            return

        lidar_stamp = self.get_current_timestamp_str()

        # with self.lock_lidar:
        self.latest_lidar_stamp = lidar_stamp
        filename_base = f"r_{self.latest_radar_stamp}_l_{lidar_stamp}_c_{self.latest_camera_stitched_stamp}"

        save_path = os.path.join(self.dirs['LIDAR_PCD'], filename_base + '.pcd')
        self.save_pcd(msg, save_path)

    def image_callback(self, msg, camera_position):
        now = time.time()

        # CAMERA_MID is special, it should not be saved if radar or lidar is locked
        if camera_position == 'CAMERA_STITCHED':
            # if self.lock_radar.locked() or self.lock_lidar.locked():
            #     return
            self.latest_camera_stitched_stamp = self.get_current_timestamp_str()

        # Check if enough time has passed since the last save
        if now - self.last_saved_time[camera_position] >= self.save_interval:
            self.last_saved_time[camera_position] = now
            filename_base = f"r_{self.latest_radar_stamp}_l_{self.latest_lidar_stamp}_c_{self.latest_camera_stitched_stamp}"
            img_save_path = os.path.join(self.dirs[camera_position], filename_base + '.png')
            self.save_image(msg, img_save_path)

    def save_pcd(self, msg, filename):
        field_names = [field.name for field in msg.fields]
        points = list(pc2.read_points(msg, field_names=tuple(field_names), skip_nans=True))

        if len(points) == 0:
            self.get_logger().warn("No points to save for PCD.")
            return

        points_array = np.array([list(p) for p in points], dtype=float)
        xyz = points_array[:, :3]
        intensity = points_array[:, 3] if points_array.shape[1] > 3 else None

        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write(f'FIELDS x y z{" intensity" if intensity is not None else ""}\n')
            f.write(f'SIZE 4 4 4{" 4" if intensity is not None else ""}\n')
            f.write(f'TYPE F F F{" F" if intensity is not None else ""}\n')
            f.write(f'COUNT 1 1 1{" 1" if intensity is not None else ""}\n')
            f.write(f'WIDTH {xyz.shape[0]}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {xyz.shape[0]}\n')
            f.write('DATA ascii\n')
            if intensity is not None:
                data = np.hstack((xyz, intensity[:, np.newaxis]))
            else:
                data = xyz
            np.savetxt(f, data, fmt='%f')

    def save_image(self, msg, filename):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(filename, cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SavePerceptions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
