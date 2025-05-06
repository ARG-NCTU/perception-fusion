ros2 launch localization2 localization_gps_imu.launch.py

# 設定當前GPS為局部座標原點
ros2 service call /localization_gps_imu/set_origin std_srvs/srv/Empty

# 設定當前IMU方向為0度
ros2 service call /localization_gps_imu/set_imu_zero std_srvs/srv/Empty

ros2 service call /localization_gps_imu/imu_offset duckiepond_interfaces/srv/SetValue "{data: 0.3}" 
