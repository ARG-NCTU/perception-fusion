### dataloader.py
import os
import json
import cv2
import open3d as o3d
import numpy as np
from glob import glob
from scipy.spatial.transform import Rotation as R
from datetime import datetime, timedelta

class DataLoader:
    def __init__(self, root_dir):
        self.root_dir = root_dir
        self.calib_path = os.path.join(root_dir, "v1.0-trainval", "calibrated_sensor.json")
        self.samples_dir = os.path.join(root_dir, "samples")
        self.calibration = self._load_calibration()

    def _load_calibration(self):
        with open(self.calib_path, 'r') as f:
            data = json.load(f)

        calib = {}
        for item in data:
            sensor = item['sensor']
            quat = item['rotation']
            r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            rot_mat = r.as_matrix()
            trans = np.array(item['translation']).reshape(3, 1)

            extrinsic = np.eye(4)
            extrinsic[:3, :3] = rot_mat
            extrinsic[:3, 3] = trans.flatten()

            intrinsic = None
            if item['camera_intrinsic']:
                intrinsic = np.array(item['camera_intrinsic']).reshape(3, 3)

            calib[sensor] = {
                'extrinsic': extrinsic,
                'intrinsic': intrinsic
            }

        return calib

    def load_camera_image(self, sensor, target_timestamp):
        img_dir = os.path.join(self.samples_dir, sensor)
        files = sorted(glob(os.path.join(img_dir, "*.png")))
        if not files:
            return None
        
        target_timestamp = target_timestamp.split("-")[1]  

        timestamps = []
        for f in files:
            basename = os.path.basename(f)
            if "_c_" in basename:
                ts = basename.split("_c_")[1].replace(".png", "").split("-")[1]
                if int(ts) > int(target_timestamp):
                    timestamps.append((abs(int(ts) - int(target_timestamp)), f)) 

        if not timestamps:
            return None

        timestamps.sort()
        closest_file = timestamps[0][1]
        return cv2.imread(closest_file)

    def load_pointcloud_by_filename(self, sensor, filename):
        path = os.path.join(self.samples_dir, sensor, filename)
        if not os.path.exists(path):
            print(f"[Warning] Point cloud not found: {path}")
            return None
        return o3d.io.read_point_cloud(path)


    def get_extrinsic(self, sensor):
        return self.calibration[sensor]['extrinsic']

    def get_intrinsic(self, sensor):
        intr = self.calibration[sensor]['intrinsic']
        if intr is None:
            raise ValueError(f"Sensor {sensor} has no intrinsic matrix.")
        return intr
