import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import pandas as pd
from scipy.spatial.transform import Rotation as R
from camera_pcd_projection.utils import project_pcl_to_image, pcd_to_camera_frame
from sensor_msgs_py import point_cloud2

class CameraPCDProjectionNode(Node):
    def __init__(self):
        super().__init__('camera_pcd_projection')

        # Declare parameters
        self.declare_parameter('camera_position', 'CAMERA_LEFT')
        self.declare_parameter('pcd_sensor', 'LIDAR_PCD')
        self.declare_parameter('camera_topic', '/camera1_fix/color/image_raw/compressed')
        self.declare_parameter('pointcloud_topic', '/js/velodyne_points')
        self.declare_parameter('calibrated_path', '')
        self.declare_parameter('output_image_topic', '/camera1_fix/pcd_projection/compressed')

        self.camera_position = self.get_parameter('camera_position').value
        self.pcd_sensor = self.get_parameter('pcd_sensor').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.calibrated_path = self.get_parameter('calibrated_path').value
        self.output_image_topic = self.get_parameter('output_image_topic').value

        # Load calibration
        self._load_calibration()

        # Subscribers
        self.bridge = CvBridge()
        self.latest_pcd = None
        self.sub_pcd = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pcd_callback, 10)
        self.sub_image = self.create_subscription(CompressedImage, self.camera_topic, self.image_callback, 10)
        self.pub_image = self.create_publisher(CompressedImage, self.output_image_topic, 10)

        self.get_logger().info("camera_pcd_projection node initialized.")

    def _load_calibration(self):
        with open(self.calibrated_path, 'r') as f:
            data = json.load(f)

        sensor_map = {item['sensor']: item for item in data}

        if self.camera_position not in sensor_map or self.pcd_sensor not in sensor_map:
            raise ValueError(f"Missing calibration for {self.camera_position} or {self.pcd_sensor}")

        def to_matrix(item):
            quat = item['rotation']
            r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            rot_mat = self.ros_to_cam_rotation(r.as_matrix())
            trans = np.array(item['translation']).reshape(3, 1)
            mat = np.eye(4)
            mat[:3, :3] = rot_mat
            mat[:3, 3] = trans.flatten()
            return mat

        cam_ext = to_matrix(sensor_map[self.camera_position])
        pcd_ext = to_matrix(sensor_map[self.pcd_sensor])
        self.extrinsic = cam_ext @ np.linalg.inv(pcd_ext)
        self.intrinsic = np.array(sensor_map[self.camera_position]['camera_intrinsic']).reshape(3, 3)

        self.get_logger().info(f"Loaded calibration: camera={self.camera_position}, pcd={self.pcd_sensor}")

    def ros_to_cam_rotation(self, R_ros):
        T = np.array([
            [0,  1,  0],
            [0,  0, -1],
            [1,  0,  0]
        ])
        return T @ R_ros

    def get_custom_colormap(self):
        colors = [
            (1.0, 0.0, 0.0),   # Red
            (1.0, 0.5, 0.0),   # Orange
            (1.0, 1.0, 0.0),   # Yellow
            (0.0, 1.0, 0.0),   # Green
            (0.0, 0.0, 1.0),   # Blue
            (0.5, 0.0, 1.0)    # Purple
        ]
        colors_bgr = [(int(c[2]*255), int(c[1]*255), int(c[0]*255)) for c in colors]
        cmap = np.zeros((256, 3), dtype=np.uint8)
        for i in range(5):
            for j in range(3):
                cmap[i*51:(i+1)*51, j] = np.linspace(colors_bgr[i][j], colors_bgr[i+1][j], 51)
        return cmap

    def pcd_callback(self, msg):
        self.latest_pcd = msg

    def image_callback(self, img_msg):
        if self.latest_pcd is None:
            self.get_logger().warn("No point cloud received yet.")
            return

        image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        points = np.array([list(p)[:3] for p in point_cloud2.read_points(
            self.latest_pcd, field_names=["x", "y", "z"], skip_nans=True)])

        pcl_df = pd.DataFrame(points, columns=['x', 'y', 'z'])
        pcl_df['rcs'] = 1.0
        pcl_df[['x', 'y', 'z']] = pcd_to_camera_frame(pcl_df[['x', 'y', 'z']].values)

        uvs, depth, _ = project_pcl_to_image(pcl_df, self.extrinsic, self.intrinsic, image.shape)
        cmap = self.get_custom_colormap()

        # Normalize depth to 0–255 range (for 0–120m)
        depth_clipped = np.clip(depth, 0, 120)
        depth_norm = ((depth_clipped / 120.0) * 255).astype(np.uint8)

        # Draw projected points with color
        for pt, d in zip(uvs, depth_norm):
            color = tuple(int(c) for c in cmap[d])
            cv2.circle(image, tuple(pt), 2, color, -1)

        # Draw color legend in the top-right corner
        legend_h, legend_w = 20, 256
        legend_img = np.zeros((legend_h, legend_w, 3), dtype=np.uint8)
        legend_img[:] = cmap[np.newaxis, :]
        legend_x = image.shape[1] - legend_w - 40
        legend_y = 10
        image[legend_y:legend_y + legend_h, legend_x:legend_x + legend_w] = legend_img

        # Add distance labels with black border
        def draw_text_with_outline(img, text, org, font, scale, color, thickness=1):
            cv2.putText(img, text, org, font, scale, (0, 0, 0), thickness + 2, lineType=cv2.LINE_AA)
            cv2.putText(img, text, org, font, scale, color, thickness, lineType=cv2.LINE_AA)

        draw_text_with_outline(image, "0m",     (legend_x, legend_y + legend_h + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        draw_text_with_outline(image, "40m",    (legend_x + 85, legend_y + legend_h + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        draw_text_with_outline(image, "80m",    (legend_x + 170, legend_y + legend_h + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        draw_text_with_outline(image, "120m",   (legend_x + 240, legend_y + legend_h + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        out_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        out_msg.header = img_msg.header
        self.pub_image.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPCDProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
