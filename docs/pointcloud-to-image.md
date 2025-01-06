# Perception Fusion

PointCloud Processing Guide

## Clone repo 

```
git clone --recursive git@github.com:ARG-NCTU/perception-fusion.git
``` 

## Update repo and submodules

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

## Enter the repo

```bash
cd perception-fusion
```

## Set up Environment

1. Enter Docker Enviroment

1.1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros2-cpu/run.sh
```

1.2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros2-cpu/run.sh
```

2. Building ROS Point Cloud Processing Package

```bash
source build_ros2_pcd.sh
```

## Usage

### Terminal 1: Launch Point Cloud Processing Node

1. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Visualize Radar

```bash
ros2 launch pointcloud_processing demo.launch.py
```

### Terminal 2: Play Radar Ros2 Bag (or Run Radar)

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Play ROS2 Bag for example

Ros2 Bag NAS [Link](http://gofile.me/773h8/rf6BFgCfG)

```bash
ros2 bag play bags/20241129_cropped/processed_rosbag_halo_20241205-145227/processed_rosbag_halo_20241205-145227_0.db3
```

### Terminal 3: Rviz2 to Visualize Original Radar Message

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run Combined Launch File

```bash
rviz2 -d rviz2/radar-cropped-example.rviz 
```

### Terminal 4: rqt_image_view to Visualize Radar Compressed Image

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view
```

### Terminal 5: rqt_reconfigure to Reconfigure ROS2 parameters

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run rqt_reconfigure

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Click "Refresh" button, and click "point_to_image", you can reconfigure all paramters you want.

### Terminal 6: ROS2 Bag Recording

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Record ROS2 Bag

```bash
source ros2bag/record_all_sensor.sh
```