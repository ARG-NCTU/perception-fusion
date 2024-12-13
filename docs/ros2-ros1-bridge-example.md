# Perception Fusion

ROS2-ROS1-Bridge Guide

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

## Set up Environment

1. Enter Docker Enviroment

1.1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros1-ros2/run.sh
```

1.2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros1-ros2/run.sh
```

2. Building ROS2 ROS1 bridge

```bash
source build_ros2_ros1_bridge.sh
```

3. Building All ROS1 package

```bash
source build_ros1_all.sh
```

## Usage

### Terminal 1: ROSCORE

1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Roscore

```bash
roscore
```
### Terminal 2: ROS1 & ROS2 BRIDGE

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 & ROS2 Environment

```bash
source environment_ros2_ros1_bridge.sh
```

3. Run Ros1_bridge

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Download bags

Please download bags folder and put in the root path of the repo: [Link](http://gofile.me/773h8/WtbSM5xSh) 

### Terminal 3: ROS1

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Run ROS1 package

Image Bag
```bash
cd bags/images-2024-11-23-15-28-41/
rosbag play 2024-11-23-15-28-41.bag 
```

Pose & Layser Bag
```bash
cd bags/1123_1528/
rosbag play 2024-11-23-15-28-44_0.bag 
```

### Terminal 4: ROS2

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run ROS2 Bag Example

Radar Bag
```bash
cd bags/recorded_rosbag_halo_20241123-141728/
ros2 bag play recorded_rosbag_halo_20241123-141728_0.db3
```

### Terminal 5
1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Rviz

```bash
rviz -d rviz/radar-example.rviz
```

### Terminal 6
1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Rviz2

```bash
rviz2 -d rviz2/lidar-camera-example.rviz
```

