#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import json
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CameraUndistorter:
    def __init__(self, input_topic, output_topic, dim, K, D, balance=1.0, save_dir=None, save_new_K=False):
        self.dim = dim
        self.K = K
        self.D = D
        self.bridge = CvBridge()
        self.save_dir = save_dir
        self.save_new_K = save_new_K

        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, self.dim, np.eye(3), balance=balance)

        self.sub = rospy.Subscriber(input_topic, CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher(output_topic, CompressedImage, queue_size=1)

        rospy.loginfo(f"[{input_topic}] Undistorter initialized.")
        rospy.loginfo(f"[{input_topic}] New camera matrix:\n{self.new_K}")

        if self.save_new_K and self.save_dir:
            self.save_intrinsics(input_topic)

    def save_intrinsics(self, input_topic):
        try:
            camera_name = input_topic.split('/')[1]
            save_path = os.path.join(self.save_dir, f"{camera_name}.json")
            os.makedirs(os.path.dirname(save_path), exist_ok=True)

            output_data = {
                "new_K": self.new_K.flatten().tolist(),
                "D": self.D.flatten().tolist()
            }

            with open(save_path, 'w') as f:
                json.dump(output_data, f, indent=4)

            rospy.loginfo(f"[{camera_name}] Saved new_K and D to {save_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save intrinsic params for {input_topic}: {e}")

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                rospy.logerr("Failed to decode image.")
                return

            img_undistorted = cv2.fisheye.undistortImage(img, self.K, self.D, Knew=self.new_K)
            msg_out = self.bridge.cv2_to_compressed_imgmsg(img_undistorted, dst_format="jpeg")
            self.pub.publish(msg_out)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

def load_camera_params(json_path):
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        K = np.array(data['K']).reshape((3, 3))
        D = np.array(data['D']).reshape((4, 1))
        return K, D
    except Exception as e:
        rospy.logerr(f"Failed to load camera parameters from {json_path}: {e}")
        raise

if __name__ == '__main__':
    try:
        rospy.init_node('camera_undistorter', anonymous=False)

        input_topic = rospy.get_param('~input_topic')
        output_topic = rospy.get_param('~output_topic')
        json_path = rospy.get_param('~json_path')
        balance = rospy.get_param('~balance', 0.33)
        save_dir = rospy.get_param('~save_dir', None)
        save_new_K = rospy.get_param('~save_new_K', False)

        DIM = (640, 480)
        K, D = load_camera_params(json_path)

        undistorter = CameraUndistorter(
            input_topic, output_topic, DIM, K, D,
            balance=balance,
            save_dir=save_dir,
            save_new_K=save_new_K
        )

        rospy.loginfo(f"✅ Node for {input_topic} started.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        rospy.loginfo("Node terminated.")
