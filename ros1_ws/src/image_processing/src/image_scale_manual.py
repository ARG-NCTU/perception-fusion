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

class ROSImageScaleManual:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_scale_manual', anonymous=True)

        # Load parameters
        self.sub_topic = rospy.get_param('~sub_camera_topic', '/camera_stitched/scaled/compressed')
        self.pub_topic = rospy.get_param('~pub_camera_topic', '/camera_stitched/manual_scaled/compressed')

        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Publisher
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)

        rospy.loginfo("Image manual scaler initialized. Waiting for images...")

        # Initialize points
        self.points = {}
        self.json_path = rospy.get_param('~json_path', os.path.join(rospack.get_path('image_processing'), 'config', 'points.json'))
        self.load_points()


    def callback(self, msg):
        # Convert the compressed image to OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        scaled_image = self.image_scale_manual(image)

        # Convert the scaled image back to ROS format
        scaled_msg = self.bridge.cv2_to_compressed_imgmsg(scaled_image)

        # Publish the manually scaled image
        self.publisher.publish(scaled_msg)

        rospy.loginfo("Image manually scaled and published.")

    def load_points(self):
        with open(self.json_path, 'r') as f:
            self.points = json.load(f)
            rospy.loginfo(f"Loaded points: {self.points}")

    def image_scale_manual(self, image):

        # Check if the image loaded correctly
        if image is None:
            raise ValueError("Error: Could not load the image.")

        # Get image dimensions
        height, width = image.shape[:2]

        rospy.loginfo(f"Image dimensions: {width}x{height}")

        # Add all points to the image
        for angle, point in self.points.items():
            if point is not None:
                x, y = point
                cv2.circle(image, tuple(point), 5, (0, 255, 0), -1)
                cv2.putText(image, str(angle), (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image
        
if __name__ == '__main__':
    try:
        ROSImageScaleManual()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


