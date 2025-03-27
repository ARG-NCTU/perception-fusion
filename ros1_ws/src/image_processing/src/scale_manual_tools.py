#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospkg
import os
import json
from threading import Lock

rospack = rospkg.RosPack()

class ScaleManualTools:
    def __init__(self):
        rospy.init_node('scale_tools', anonymous=True)

        self.sub_topic = rospy.get_param('~sub_camera_topic', '/camera_stitched/color/image_raw/compressed')
        self.pub_topic = rospy.get_param('~pub_camera_topic', '/camera_stitched/manual_scaled/compressed')
        self.json_path = rospy.get_param('~json_path', os.path.join(rospack.get_path('image_processing'), 'config', 'points.json'))

        if not os.path.exists(self.json_path):
            os.makedirs(os.path.dirname(self.json_path), exist_ok=True)
            default_points = {str(deg): None for deg in range(-90, 91, 5)}  # Changed to 5-degree steps
            with open(self.json_path, 'w') as f:
                json.dump(default_points, f, indent=4)
                rospy.loginfo(f"Created JSON file at {self.json_path} with default points")
        else:
            rospy.loginfo(f"Loaded JSON file from {self.json_path}")

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)
        rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)

        self.image_width = None
        self.image_height = None
        self.current_angle = 0
        self.add_mode = False
        self.drag_point = None
        self.points_lock = Lock()
        self.load_points()

        self.scaled_image = None
        self.image_lock = Lock()

        cv2.namedWindow('Scale Tools')
        cv2.setMouseCallback('Scale Tools', self.mouse_callback)
        # Trackbar: -90 to 90 with 5-degree steps = 37 positions (0 to 36), start at 18 (-90 + 18*5 = 0)
        cv2.createTrackbar('Angle', 'Scale Tools', 18, 36, self.angle_callback)
        rospy.loginfo("GUI initialized. Use 'a' to toggle Add, 'c' to Confirm, 'r' to Clear")

    def load_points(self):
        default_points = {str(deg): None for deg in range(-90, 91, 5)}  # 5-degree steps
        try:
            with open(self.json_path, 'r') as f:
                loaded_points = json.load(f)
                self.points = default_points.copy()
                self.points.update({k: v for k, v in loaded_points.items() if k in default_points})
                rospy.loginfo(f"Loaded points: {self.points}")
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
            self.points = default_points.copy()
            self.save_points()
            rospy.logwarn(f"Reset points due to error: {str(e)}")

    def save_points(self):
        with self.points_lock:
            with open(self.json_path, 'w') as f:
                json.dump(self.points, f, indent=4)
            rospy.loginfo(f"Saved points: {self.points}")

    def angle_callback(self, pos):
        self.current_angle = (pos - 18) * 5  # Maps 0-36 to -90 to 90 in 5-degree steps
        self.add_mode = False
        self.drag_point = None
        with self.points_lock:
            point = self.points.get(str(self.current_angle))
            rospy.loginfo(f"Angle set to {self.current_angle}, point: {point}")

    def add_button_callback(self):
        self.add_mode = not self.add_mode
        self.drag_point = None if not self.add_mode else [self.image_width // 2 if self.image_width else 320, self.image_height // 2 if self.image_height else 240]
        rospy.loginfo(f"Add mode: {self.add_mode}")

    def confirm_button_callback(self):
        if self.add_mode and self.drag_point:
            with self.points_lock:
                self.points[str(self.current_angle)] = self.drag_point.copy()
            self.save_points()
            self.add_mode = False
            self.drag_point = None
            rospy.loginfo(f"Point confirmed at angle {self.current_angle}")

    def clear_button_callback(self):
        with self.points_lock:
            self.points[str(self.current_angle)] = None
        self.save_points()
        rospy.loginfo(f"Point cleared at angle {self.current_angle}")

    def mouse_callback(self, event, x, y, flags, param):
        if self.add_mode and self.drag_point:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag_point = [x, y]
            elif event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
                self.drag_point = [x, y]

    def image_scale_manual(self, image):
        height, width = image.shape[:2]
        ref_points = {}
        for angle in range(-90, 91, 5):  # Changed to 5-degree steps
            x = int(width/2 + (width/2) * np.tan(np.radians(angle)))
            if 0 <= x < width:
                ref_points[angle] = [x, height//2]

        for angle, pos in ref_points.items():
            cv2.circle(image, (pos[0], pos[1]), 3, (0, 255, 0), -1)  # Smaller green points (radius 3)

        with self.points_lock:
            current_point = self.points.get(str(self.current_angle))
            if current_point and not self.add_mode:  # Blue point in blank mode
                x, y = current_point
                cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # Smaller blue point (radius 5)

        if self.add_mode and self.drag_point:
            cv2.circle(image, (self.drag_point[0], self.drag_point[1]), 5, (255, 0, 0), -1)  # Smaller red point (radius 5)

        cv2.putText(image, f"Angle: {self.current_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, "a: Add, c: Confirm, r: Clear", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return image

    def callback(self, msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg)
            if image is None or image.size == 0:
                rospy.logwarn("Received empty image")
                return
            rospy.loginfo(f"Received image of size: {image.shape}")

            self.image_width, self.image_height = image.shape[1], image.shape[0]
            scaled_image = self.image_scale_manual(image.copy())

            with self.image_lock:
                self.scaled_image = scaled_image

            output_dir = os.path.join(rospack.get_path('image_processing'), 'output')
            os.makedirs(output_dir, exist_ok=True)
            cv2.imwrite(os.path.join(output_dir, 'scaled_image.png'), scaled_image)

            scaled_msg = self.bridge.cv2_to_compressed_imgmsg(scaled_image)
            scaled_msg.header = msg.header
            self.publisher.publish(scaled_msg)
        except Exception as e:
            rospy.logerr(f"Callback error: {str(e)}")

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.scaled_image is not None:
                    cv2.imshow('Scale Tools', self.scaled_image)
            key = cv2.waitKey(33)
            if key == 27:  # ESC to exit
                rospy.signal_shutdown("User requested shutdown")
            elif key == ord('a'):
                self.add_button_callback()
            elif key == ord('c'):
                self.confirm_button_callback()
            elif key == ord('r'):
                self.clear_button_callback()
            elif key != -1:
                rospy.loginfo(f"Key pressed: {key}")
            rate.sleep()

if __name__ == '__main__':
    try:
        scale_tools = ScaleManualTools()
        scale_tools.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()