import os
import numpy as np
import cv2
from glob import glob
from PerceptionFusion.dataloader import DataLoader
from PerceptionFusion.utils import project_pcl_to_image, pcd_to_camera_frame
import pandas as pd
import argparse

class PerceptionFusion:
    def __init__(self, data_root):
        self.loader = DataLoader(data_root)

    def get_filename_from_frame(self, frame_index, sensor_dir_name):
        pcd_dir = os.path.join(self.loader.samples_dir, sensor_dir_name)
        files = sorted(glob(os.path.join(pcd_dir, "*.pcd")))
        if frame_index >= len(files):
            raise IndexError(f"Frame index {frame_index} exceeds available frames in {sensor_dir_name}")
        return os.path.basename(files[frame_index])

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
    
    def visualize(self, filename, image_sensor, pcd_sensor, save_path=None):
        # Extract timestamp from filename (e.g. r_..._l_..._c_TIMESTAMP.pcd)
        parts = filename.split("_c_")
        if len(parts) != 2:
            print(f"[Error] Invalid filename format: {filename}")
            return
        timestamp = parts[1].replace(".pcd", "")

        image = self.loader.load_camera_image(image_sensor, timestamp)
        pcd = self.loader.load_pointcloud_by_filename(pcd_sensor, filename)

        if image is None or pcd is None:
            print(f"[Error] Missing data for {filename} in {image_sensor} or {pcd_sensor}")
            return

        intrinsic = self.loader.get_intrinsic(image_sensor)
        cam_ext = self.loader.get_extrinsic(image_sensor)
        pcd_ext = self.loader.get_extrinsic(pcd_sensor)
        extrinsic = cam_ext @ np.linalg.inv(pcd_ext)

        pcd_np = np.asarray(pcd.points)
        pcl_df = pd.DataFrame(pcd_np, columns=['x','y','z'])
        pcl_df['rcs'] = 1.0

        # Coordinate frame conversion: from PCD to camera frame
        pcl_df[['x','y','z']] = pcd_to_camera_frame(pcl_df[['x','y','z']].values)

        uvs, depth, _ = project_pcl_to_image(pcl_df, extrinsic, intrinsic, image.shape)
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

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            cv2.imwrite(save_path, image)
            print(f"[Saved] {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Perception Fusion Visualization")
    parser.add_argument("--frame", type=int, default=10, help="Frame index")
    parser.add_argument("--proj", choices=["lidar", "radar", "both"], default="lidar", help="Which pointcloud to project")
    parser.add_argument("--data", type=str, default="/home/arg/perception-fusion/data/south-tw-maritime-multi-modal-dataset", help="Path to the dataset")
    parser.add_argument("--save_dir", type=str, default="visualization", help="Base directory to save images")
    args = parser.parse_args()

    dataroot = args.data
    if not os.path.exists(dataroot):
        raise FileNotFoundError(f"Data root {dataroot} does not exist")

    fusion = PerceptionFusion(dataroot)

    sensor_type = "LIDAR_PCD" if args.proj == "lidar" else "RADAR_PCD"
    filename = fusion.get_filename_from_frame(args.frame, sensor_type)

    camera_list = ["CAMERA_LEFT", "CAMERA_MID", "CAMERA_RIGHT", "CAMERA_BACK"]
    frame_dir = os.path.join(args.save_dir, str(args.frame))

    for cam in camera_list:
        if args.proj in ["radar", "both"]:
            fusion.visualize(filename, cam, "RADAR_PCD", save_path=os.path.join(frame_dir, f"{cam}_radar.png"))
        if args.proj in ["lidar", "both"]:
            fusion.visualize(filename, cam, "LIDAR_PCD", save_path=os.path.join(frame_dir, f"{cam}_lidar.png"))


# Usage:
# cd ~/perception-fusion
# python3 -m PerceptionFusion.PerceptionFusion --frame 200 --proj lidar --data /home/arg/perception-fusion/data/south-tw-maritime-multi-modal-dataset --save_dir visualization
