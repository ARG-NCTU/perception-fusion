#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospkg
import os
import json

rospack = rospkg.RosPack()

class ROSImageScale:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_scale', anonymous=True)

        # Load parameters
        self.sub_topic = rospy.get_param('~sub_camera_topic', '/camera2/color/image_raw/compressed')
        self.camera_orientation = rospy.get_param('~camera_orientation', 'mid')
        self.sub_camera_info_topic = rospy.get_param('~sub_camera_info_topic', '/camera2/color/camera_info')
        self.pub_topic = rospy.get_param('~pub_camera_topic', '/camera2/scaled/compressed')

        self.json_path = rospy.get_param('~json_path')

        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Publisher
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)
        rospy.Subscriber(self.sub_camera_info_topic, CameraInfo, self.camera_info_callback)

        rospy.loginfo("Image scaler initialized. Waiting for images...")

        self.K = None

    def callback(self, msg):
        # Convert the compressed image to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        scaled_image = self.image_scale(image, self.camera_orientation)

        # Convert the scaled image back to ROS format
        scaled_msg = self.bridge.cv2_to_compressed_imgmsg(scaled_image)

        # Publish the scaled image
        self.publisher.publish(scaled_msg)

        rospy.loginfo("Image scaled and published.")
    
    def camera_info_callback(self, msg):
        # Get camera intrinsic parameters
        self.K = np.array(msg.K).reshape(3, 3)

        rospy.loginfo("Camera info received.")

    def image_scale(self, image, camera_orientation):

        # Check if the image loaded correctly
        if image is None:
            raise ValueError("Error: Could not load the image.")

        # Get image dimensions
        height, width = image.shape[:2]

        rospy.loginfo(f"Image dimensions: {width}x{height}")

        if self.K is None:
            # Camera intrinsic parameters (provided earlier)
            if height == 480 and width == 640:
                # 640x480 camera
                # K = np.array([
                #     [219.11268079, 0.0, 335.02464347],
                #     [0.0, 291.76661082, 237.94837061],
                #     [0.0, 0.0, 1.0]
                # ])
                with open(self.json_path, 'r') as f:
                    data = json.load(f)
                K = np.array(data['K']).reshape((3, 3))
            else:
                raise ValueError("Error: Camera intrinsic parameters not provided.")
        else:
            K = self.K
        
        fx, fy = float(K[0, 0]), float(K[1, 1])
        u0, v0 = float(K[0, 2]), height - float(K[1, 2])

        # Camera parameters
        cx, cy, cz = 0, 3, 0 # Camera position (x, y, z in meters)
        fov_angles = np.arange(-60, 65, 15) # -60° to +60° in 15° steps
        ranges = [50, 100, 150]  # Fan-like markers in meters
        range_colors = [(0, 0, 255), (0, 255, 255), (0, 255, 0)]  # Red, Yellow, Green

        def project_to_image(X, Y, Z):
            u = int(u0 + (fx * (X - cx)) / (Z - cz))
            v = int(v0 - (fy * (Y - cy)) / (Z - cz))
            
            return u, v

        # # Draw FOV radial lines (-60° to +60°)
        # for angle in fov_angles:
        #     rad = np.radians(angle)
        #     X = np.sin(rad) * 200  # X-position based on angle
        #     Y = 0  # Camera height
        #     Z = np.cos(rad) * 200  # Extend lines to 200m ahead
        #     u, v = project_to_image(X, Y, Z)
        #     origin_u = int(u0)
        #     origin_v = int(v0 + fy)
        #     if u < 0 or u >= width or v < 0 or v >= height:
        #         continue
        #     cv2.line(image, (origin_u, origin_v), (u, v), (255, 255, 255), 2)

        # Draw perpendicular lines every 15 degrees for 50m, 100m, 150m distances
        for r, color in zip(ranges, range_colors):
            for angle in fov_angles:  # Every 15 degrees
                rad = np.radians(angle)
                X = np.sin(rad) * r  # Compute X position based on angle
                Y = 0  # Camera height
                Z = np.cos(rad) * r  # Distance from the camera

                # Get the perpendicular line points
                # (u1, v1), (u2, v2) = project_perpendicular_to_fov(X, Y, Z)

                u1, v1 = project_to_image(X, Y, Z)
                u2, v2 = u1 + 20, v1

                # Draw the perpendicular line
                if u1 < 0 or u1 >= width or v1 < 0 or v1 >= height or u2 < 0 or u2 >= width or v2 < 0 or v2 >= height:
                    continue
                cv2.line(image, (u1, v1), (u2, v2), color, 2)

                # add distance text and right of the line
                text = f"{r}m"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                text_x = u2 + 5
                text_y = v2 + text_size[1] // 2
                cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)

        # Draw vertical lines at every 15 degrees for 200m distances
        for angle in fov_angles:
            rad = np.radians(angle)
            X = np.sin(rad) * 200  # X-position based on angle
            Y = 3
            Z = np.cos(rad) * 200  # Extend lines to 200m ahead
            u, v = project_to_image(X, Y, Z)
            u1 = u
            v1 = height // 3
            u2 = u
            v2 = height // 3 + 20
            if u1 < 0 or u1 >= width or v1 < 0 or v1 >= height or u2 < 0 or u2 >= width or v2 < 0 or v2 >= height:
                continue
            cv2.line(image, (u1, v1), (u2, v2), (0, 0, 0), 4)
            cv2.line(image, (u1, v1), (u2, v2), (255, 255, 255), 2)
            # add degree text and put center of the line
            if self.camera_orientation == 'left':
                text = f"{angle-30}"
            elif self.camera_orientation == 'mid':
                text = f"{angle}"
            elif self.camera_orientation == 'right':
                text = f"{angle+30}"
            else:
                raise ValueError("Error: Invalid camera orientation.")
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            text_x = u - text_size[0] // 2
            text_y = v2 + text_size[1] + 5
            cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # add "Degree" text
        # text = "Degree"
        # text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        # text_x = width // 2 - text_size[0] // 2
        # text_y = height // 3 - 10
        # cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        # cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return image
    
if __name__ == '__main__':
    try:
        ROSImageScale()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


