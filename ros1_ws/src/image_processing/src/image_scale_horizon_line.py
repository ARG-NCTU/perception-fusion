#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospkg
import os
import math

rospack = rospkg.RosPack()

class ROSImageScaleHorizonLine:
    def __init__(self):
        rospy.init_node('image_scale_horizon_line', anonymous=True)

        self.sub_topic = rospy.get_param('~sub_camera_topic', '/camera2_fix/color/image_raw/compressed')
        self.pub_topic = rospy.get_param('~pub_camera_topic', '/camera2_fix/scaled_exp/compressed')
        self.camera_height = rospy.get_param('~camera_height', 5)  # in meters
        self.image_save_dir = rospy.get_param('~image_save_dir', '')  # Optional save path

        if self.image_save_dir:
            os.makedirs(self.image_save_dir, exist_ok=True)

        self.image_count = 0  # <- Add counter

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)
        rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)

        rospy.loginfo("Image scaler with horizon overlay initialized. Waiting for images...")

    def callback(self, msg):
        # Convert compressed image to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # Save to image file if enabled
        if self.image_save_dir:
            filename = os.path.join(self.image_save_dir, f"{self.image_count}_in.png")
            cv2.imwrite(filename, image)
            rospy.loginfo(f"Saved image to {filename}")
        
        processed_image = self.image_scale(image)

        # Convert processed image back to ROS message and publish
        scaled_msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
        self.publisher.publish(scaled_msg)

        # Save to image file if enabled
        if self.image_save_dir:
            filename = os.path.join(self.image_save_dir, f"{self.image_count}_out.png")
            cv2.imwrite(filename, processed_image)
            rospy.loginfo(f"Saved image to {filename}")
            self.image_count += 1

        rospy.loginfo("Processed image published.")


    def image_scale(self, image):
        if image is None:
            raise ValueError("Error: Could not load the image.")

        height, width = image.shape[:2]
        rospy.loginfo(f"Image dimensions: {width}x{height}")

        # Detect the horizon
        horizon_y, horizon_lines = self.detect_horizon_line(image)

        # Draw detected horizon lines
        for x1, y1, x2, y2 in horizon_lines:
            cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 1)

        # Draw distance arcs based on camera height
        image = self.draw_distance_arcs(image, horizon_y, self.camera_height)

        return image

    def detect_horizon_line(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100,
                                minLineLength=100, maxLineGap=50)

        horizon_y = frame.shape[0]
        horizon_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y2 - y1) < 10:
                    avg_y = (y1 + y2) // 2
                    if avg_y < horizon_y:
                        horizon_y = avg_y
                        horizon_lines.append((x1, y1, x2, y2))
        return horizon_y, horizon_lines

    def draw_distance_arcs(self, frame, horizon_y, camera_height_m):
        height, width = frame.shape[:2]
        Ax, Ay = width // 2, height - 1  # A: bottom center

        R = 6371000  # Earth radius (m)
        d_C = math.sqrt(2 * camera_height_m * R)  # real distance to horizon in meters

        marks_m = [1000, 2000, 3000, 4000, 5000]
        arcs = {m: [] for m in marks_m}

        for angle_deg in range(-89, 90):  # avoid tan(90) issues
            theta = np.deg2rad(angle_deg)
            sin_theta = np.sin(theta)
            cos_theta = np.cos(theta)

            if abs(sin_theta) < 1e-6:  # too flat, skip
                continue

            # Step 1: Compute r_C: how far along the ray to hit horizon line
            r_C = (Ay - horizon_y) / sin_theta

            # Step 2: for each distance d, compute position along the ray
            for d in marks_m:
                scale = d / d_C
                r_B = r_C * scale

                bx = int(Ax + r_B * cos_theta)
                by = int(Ay - r_B * sin_theta)

                if 0 <= bx < width and 0 <= by < height:
                    arcs[d].append((bx, by))

        # Step 3: Draw arcs
        for d, pts in arcs.items():
            for pt in pts:
                cv2.circle(frame, pt, 1, (0, 255, 0), -1)
            if pts:
                label = f"{int(d/1000)}km"
                cv2.putText(frame, label, pts[0], cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        cv2.putText(frame, f"Horizon {round(d_C/1000, 2)} km", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return frame





if __name__ == '__main__':
    try:
        ROSImageScaleHorizonLine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
