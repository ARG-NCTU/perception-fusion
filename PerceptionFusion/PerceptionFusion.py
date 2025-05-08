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

        uvs, _, _ = project_pcl_to_image(pcl_df, extrinsic, intrinsic, image.shape)
        for pt in uvs:
            cv2.circle(image, pt, 2, (0, 255, 0), -1)

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
