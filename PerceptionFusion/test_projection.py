import numpy as np
import cv2
import os
from PerceptionFusion.utils import view_points

def test_projection_on_image(image_shape=(480, 640, 3)):
    # 建立一張全黑圖片模擬相機畫面
    image = np.zeros(image_shape, dtype=np.uint8)

    # 設定 CAMERA_MID 的 intrinsic matrix（根據你提供的 calibrated_sensor.json）
    intrinsic = np.array([
        [231.808179, 0.0954715798, 341.577012],
        [0.0,        308.082067,   233.755295],
        [0.0,        0.0,           1.0]
    ])

    # 假設的 3D 點（z > 0 表示在相機前方）
    test_points = np.array([
        [0.0, 0.0, 5.0],    # 中央
        [1.0, 0.0, 5.0],    # 右側
        [-1.0, 0.0, 5.0],   # 左側
        [0.0, 1.0, 5.0],    # 上方
        [0.0, -1.0, 5.0]    # 下方
    ])  # shape (5, 3)

    # 投影
    uv = view_points(test_points.T, intrinsic).T[:, :2].astype(np.int32)

    # 畫上紅點
    for pt in uv:
        if 0 <= pt[0] < image_shape[1] and 0 <= pt[1] < image_shape[0]:
            cv2.circle(image, tuple(pt), 5, (0, 0, 255), -1)
        print(f"3D → 2D: {pt}")

    # 儲存或顯示測試結果
    os.makedirs("visualization", exist_ok=True)
    cv2.imwrite("visualization/test_projection.png", image)
    print("Projection image saved to test_projection.png")

if __name__ == "__main__":
    test_projection_on_image()

# Usage:
# cd ~/perception-fusion
# python3 -m PerceptionFusion.test_projection