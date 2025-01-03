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
cd ros-humble-ros1-bridge-builder
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

3. Run Combined Launch File

```bash
ros2 launch pointcloud_processing combined_pointcloud_processing.launch.py
```

### Terminal 2: Play Radar Ros2 Bag

1. Docker Run

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

### Terminal 3: Rviz2 to Visualize Transformed Radar Message

1. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Rviz2

```bash
rviz2 -d rviz2/radar-transformed-with-pose.rviz 
```

### Terminal 4: Rviz2 to Visualize Original Radar Message

1. Docker Run

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

### Terminal 5: rqt_image_view to Visualize Radar Compressed Image

1. Docker Run

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