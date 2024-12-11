#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import rospkg
import os

def publish_video(video_path, pub_topic):

    # Create a publisher object
    pub = rospy.Publisher(pub_topic, CompressedImage, queue_size=10)

    # Initialize the bridge between ROS and OpenCV
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Error opening video file at %s", video_path)
        return

    # Set frame rate of the video
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    rate = rospy.Rate(fps)  # Set the sleep rate based on video fps

    try:
        # while not rospy.is_shutdown():
        # Reset the video to the first frame
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.loginfo("End of video file reached, restarting...")
                break  # Exit the inner loop to restart the video

            # Convert the image to a ROS message
            ros_msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            # ros_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # ros_msg.header.stamp = rospy.Time.now()
            
            # Publish the ROS message
            pub.publish(ros_msg)

            rate.sleep()
    finally:
        cap.release()

if __name__ == '__main__':
    # Get video path from ROS parameter server
    rospy.init_node('video_publisher', anonymous=True)
    video_path = rospy.get_param('~video_path', 'source_video/2024-10-23-20-57-55_mid.mp4')  # Default path if not provided
    camera_topic = rospy.get_param('~pub_camera_topic', '/camera/image_raw/compressed')  # Default topic if not provided
    rospack = rospkg.RosPack()
    video_path = os.path.join(rospack.get_path('object_detection'), video_path)
    publish_video(video_path, camera_topic)
