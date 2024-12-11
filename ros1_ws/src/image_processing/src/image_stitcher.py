#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospkg
import os
from Stitcher import Stitcher

rospack = rospkg.RosPack()


class ROSImageStitcher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_stitcher', anonymous=True)

        # Load parameters
        self.left_topic = rospy.get_param('~sub_camera_topic_left', '/camera_left/color/image_raw/compressed')
        self.mid_topic = rospy.get_param('~sub_camera_topic_mid', '/camera_middle/color/image_raw/compressed')
        self.right_topic = rospy.get_param('~sub_camera_topic_right', '/camera_right/color/image_raw/compressed')
        self.output_topic = rospy.get_param('~pub_camera_topic', '/camera_real_stitched/color/image_raw/compressed')
        self.output_dir = rospy.get_param('~output_dir', 'stitched_results')
        self.h1_path = rospy.get_param('~h1_path', "stitched_results/homography/H1_34.npy")
        self.h2_path = rospy.get_param('~h2_path', "stitched_results/homography/H2_34.npy")

        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Initialize the image index
        self.image_index = 1

        # Publisher
        self.publisher = rospy.Publisher(self.output_topic, CompressedImage, queue_size=1)

        # Start a timer to periodically process images
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo("Image stitcher initialized. Waiting for images...")

    def get_image(self, topic_name):
        try:
            msg = rospy.wait_for_message(topic_name, CompressedImage, timeout=0.1)
            return self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except rospy.ROSException:
            rospy.loginfo(f"No new image received from {topic_name}")
            return None

    def stitch_images(self):
        # Get images from topics
        self.left_image = self.get_image(self.left_topic)
        self.mid_image = self.get_image(self.mid_topic)
        self.right_image = self.get_image(self.right_topic)

        if self.left_image is None or self.mid_image is None or self.right_image is None:
            rospy.loginfo("Skipping stitching due to missing images.")
            return None

        output_dir = os.path.join(rospack.get_path('image_processing'), self.output_dir)
        os.makedirs(output_dir, exist_ok=True)

        intermediate_dir = os.path.join(output_dir, 'intermediate')
        homography_dir = os.path.join(output_dir, 'homography')
        os.makedirs(intermediate_dir, exist_ok=True)
        os.makedirs(homography_dir, exist_ok=True)

        # Initialize the stitcher
        stitcher = Stitcher()

        h1_path = os.path.join(rospack.get_path('image_processing'), self.h1_path) if self.h1_path else None
        h2_path = os.path.join(rospack.get_path('image_processing'), self.h2_path) if self.h2_path else None
        H1 = np.load(h1_path) if h1_path and os.path.exists(h1_path) else None
        H2 = np.load(h2_path) if h2_path and os.path.exists(h2_path) else None

        img_left = cv2.flip(self.mid_image, 1)
        img_right = cv2.flip(self.left_image, 1)

        # First stitching step with H1
        LM_img = stitcher.stitching(
            img_left, img_right, flip=True, H=H1, save_H_path=(None if H1 is not None else os.path.join(homography_dir, f"H1_{self.image_index}.npy"))
        )

        if LM_img is None:
            rospy.loginfo(f"Skipping stitching for image set {self.image_index} due to missing or invalid stitching.")
            return None

        if not self.h1_path and not self.h2_path:
            intermediate_path = os.path.join(intermediate_dir, f"{self.image_index}.png")
            cv2.imwrite(intermediate_path, LM_img)

        img_left = LM_img
        img_right = self.right_image

        # Final stitching step with H2
        final_image = stitcher.stitching(
            img_left, img_right, flip=False, H=H2, save_H_path=(None if H2 is not None else os.path.join(homography_dir, f"H2_{self.image_index}.npy"))
        )

        if final_image is None:
            rospy.loginfo(f"Skipping final stitching for image set {self.image_index} due to missing or invalid stitching.")
            return None

        # Increment the image index
        self.image_index += 1

        # Save the stitched image
        if not self.h1_path and not self.h2_path:
            images_dir = os.path.join(output_dir, 'stitched_images')
            os.makedirs(images_dir, exist_ok=True)
            cv2.imwrite(os.path.join(images_dir, f"{self.image_index}.png"), final_image)

        return final_image

    def timer_callback(self, event):
        stitched_image = self.stitch_images()
        if stitched_image is not None:
            # Convert OpenCV image to ROS CompressedImage message
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(stitched_image, dst_format='jpeg')
            self.publisher.publish(compressed_msg)
            rospy.loginfo(f"Published stitched image {self.image_index - 1}.")


if __name__ == '__main__':
    try:
        stitcher = ROSImageStitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image stitcher node terminated.")
