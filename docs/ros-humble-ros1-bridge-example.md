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

## Enter the repo

```bash
cd perception-fusion
```

## Set up Environment

1. Build ROS-HUMBLE-ROS1-Bridge package

1.1. Pull the Builder Docker Image

```bash
source Docker/ros-humble-ros1-bridge-builder/pull.sh
```

1.2. Build ROS-HUMBLE-ROS1-Bridge package

```bash
source build_bridge_package.sh
```

2. Enter Docker Enviroment

2.1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros-humble-minimal/run.sh
```

2.2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros-humble-minimal/run.sh
```

## Usage

### Terminal 1: ROSCORE

1. Docker Run

```bash
source Docker/ros1-cpu/run.sh
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
source Docker/ros-humble-minimal/run.sh
```

2. Source ROS1 & ROS2 Environment

```bash
source environment_ros_humble_ros1_bridge.sh
```

3. Run Ros1_bridge

3.1. Dynamic Bridge

For all ROS1 & ROS2 topics
```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

For all ROS1 topics
```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
```

For all ROS2 topics
```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

3.2. Parameter Bridge

3.2.1. Docker Run In ROS1 Docker

Run this command in a new terminal

```bash
source Docker/ros1-cpu/run.sh
```

3.2.2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3.2.3. Load parameters

Write your own yaml file (ros1 type) or use

```bash
rosparam load config/bridge_1to2_params.yaml
```

```bash
rosparam load config/bridge_2to1_params.yaml
```

If you are not sure the ros1 type, run step 3.3. in bridging terminal

3.2.4. Run parameter bridge

Back to bridging terminal, for all specific topics bridges

```bash
ros2 run ros1_bridge parameter_bridge
```

3.3. Search all ROS1 ROS2 pair msg types available for bridge 
```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

### Download bags

Please download bags folder and put in the root path of the repo: [Link](http://gofile.me/773h8/WtbSM5xSh) 

### Terminal 3: ROS1

1. Docker Run

```bash
source Docker/ros1-cpu/run.sh
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
source Docker/ros2-cpu/run.sh
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
source Docker/ros1-cpu/run.sh
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
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Rviz2

```bash
rviz2 -d rviz2/lidar-camera-example.rviz
```

