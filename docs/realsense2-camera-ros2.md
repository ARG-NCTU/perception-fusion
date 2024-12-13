# Perception Fusion

Realsesnse2-Camera-ROS2 Guide

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
source Docker/ros2-cpu/run.sh
```

1.2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros2-cpu/run.sh
```

2. Building Librealsense package

```bash
source build_librealsense.sh
```

## Usage

### Usage 1: Single D435 camera

1. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

2. Search Serial Number

```bash
rs-enumerate-devices -s
```

3. Edit Serial Number in yaml file

```bash
/home/arg/perception-fusion/ros2_ws/src/realsense-ros/realsense2_camera/yaml/rs_launch_1.yaml
```

4. Building Realsense ROS2 package

```bash
source build_ros2_camera.sh
```

5. Source ROS2 Environment

```bash
source environment_ros2.sh
```

6. Run Realsense Node

```bash
ros2 launch realsense2_camera side_camera_launch.py config_file:=yaml/rs_launch_1.yaml
```

### Usage 2: Triple D435 camera

1. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

2. Search Serial Number

```bash
rs-enumerate-devices -s
```

3. Edit Serial Number in yaml file

```bash
/home/arg/perception-fusion/ros2_ws/src/realsense-ros/realsense2_camera/yaml/rs_launch_3.yaml
```

4. Building Realsense ROS2 package

```bash
source build_ros2_camera.sh
```

5. Source ROS2 Environment

```bash
source environment_ros2.sh
```

6. Run Realsense Node

```bash
ros2 launch realsense2_camera side_camera_launch.py config_file:=yaml/rs_launch_3.yaml
```

### ROS2 Bag Recording

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
source ros2bag/record_images.sh
```