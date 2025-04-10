import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from threading import Thread
import time

class RTSPToCompressedImage(Node):
    def __init__(self):
        self.declare_parameter('node_name', 'camera1_rtsp_to_compressed_image')
        self.node_name = self.get_parameter('node_name').value
        super().__init__(self.node_name)
        # Declare and get parameters
        self.declare_parameter('rtsp_url', 'rtsp://admin:admin@192.168.0.101:554/video')
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.declare_parameter('topic_name', '/camera1/color/image_raw/compressed')
        self.topic_name = self.get_parameter('topic_name').value
        self.publisher = self.create_publisher(CompressedImage, self.topic_name, 1)

        self.pipeline = (
            f"rtspsrc location={self.rtsp_url} latency=0 protocols=udp drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        )

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ 無法開啟 RTSP 串流：{self.rtsp_url}")
            self.cap = None

        self.running = True
        self.thread = Thread(target=self.stream_loop, daemon=True)
        self.thread.start()

    def stream_loop(self):
        while rclpy.ok() and self.running:
            if self.cap is None:
                time.sleep(1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn(f"⚠️ 讀取 RTSP 影格失敗")
                time.sleep(0.1)
                continue

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"

            success, encoded_img = cv2.imencode(".jpg", frame)
            if not success:
                self.get_logger().warn(f"❌ OpenCV 影像壓縮失敗")
                time.sleep(0.05)
                continue

            msg.data = encoded_img.tobytes()
            self.publisher.publish(msg)

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = RTSPToCompressedImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)
