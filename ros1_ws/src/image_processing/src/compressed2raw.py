#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CompressedToImage:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('compressed_to_raw', anonymous=True)

        # Create a CvBridge object for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the Image topic
        sub_topic_name = rospy.get_param('~sub_camera_topic', "/camera1/image_raw/compressed")  # Change to your desired input topic name
        self.image_sub = rospy.Subscriber(sub_topic_name, CompressedImage, self.image_callback)

        # Publisher to publish the flipped image as CompressedImage
        pub_topic_name = rospy.get_param('~pub_camera_topic', "/camera1/image_raw")
        self.image_pub = rospy.Publisher(pub_topic_name, Image, queue_size=1)

        # Get the flip parameter (default is True, meaning the image will be flipped by default)
        self.flip_image = rospy.get_param('~flip', False)

    def image_callback(self, data):
        try:
            # Convert the ROS CompressedImage message to an OpenCV image
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Check if flipping is enabled
        if self.flip_image:
            # rospy.loginfo("Flipping image")
            # Flip the image vertically (upside down)
            flipped_image = cv2.flip(cv_image, 0)
        else:
            # rospy.loginfo("No flipping")
            # Use the original image without flipping
            flipped_image = cv_image

        try:
            # Encode the OpenCV image back to a image format
            raw_msg = self.bridge.cv2_to_imgmsg(flipped_image, "bgr8")
            raw_msg.header = data.header
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Publish the flipped compressed image
        self.image_pub.publish(raw_msg)

if __name__ == '__main__':
    try:
        # Create an instance of the ImageToCompressedFlipper and start the node
        CompressedToImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
