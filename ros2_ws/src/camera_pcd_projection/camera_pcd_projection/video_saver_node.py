import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class VideoSaverNode(Node):
    def __init__(self):
        super().__init__('video_saver_node')

        # Declare and get parameters
        self.declare_parameter('camera_topic', '/camera_merged/compressed')
        self.declare_parameter('save_dir', '/home/arg/perception-fusion/data/videos')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.save_dir = self.get_parameter('save_dir').value

        # Create directory if not exist
        os.makedirs(self.save_dir, exist_ok=True)

        # Generate video filename with current timestamp
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = os.path.join(self.save_dir, f"video_{now}.mp4")

        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_size = None
        self.fps = 10

        self.sub = self.create_subscription(CompressedImage, self.camera_topic, self.image_callback, 10)
        self.get_logger().info(f"Saving video to: {self.video_path}")

    def image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.video_writer is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, self.frame_size)

        self.video_writer.write(frame)

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VideoSaverNode stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
