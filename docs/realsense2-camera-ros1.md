# Perception Fusion

Realsesnse2-Camera-ROS1 Guide

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
source Docker/ros1-cpu/run.sh
```

1.2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros1-cpu/join.sh
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

3. ROSCORE

```bash
roscore
```

### Terminal 2 (Option 1): Single D435 camera

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Search Serial Number

```bash
rs-enumerate-devices -s
```

3. Edit Serial Number in Launch file

```bash
~/perception-fusion/ros1_ws/src/image_processing/launch/side_camera_1.launch
```

4. Source ROS1 Environment

```bash
source environment_ros1.sh
```

6. Run Realsense Node

```bash
roslaunch image_processing side_camera_1.launch
```

### Terminal 2 (Option 2): Triple D435 camera

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Search Serial Number

```bash
rs-enumerate-devices -s
```

3. Edit Serial Number in Launch file

```bash
~/perception-fusion/ros1_ws/src/image_processing/launch/side_camera_3.launch
```

4. Source ROS1 Environment

```bash
source environment_ros1.sh
```

5. Run Realsense Node

```bash
roslaunch image_processing side_camera_3.launch
```

6. (Optional) Image Sticher

```bash
roslaunch image_processing image_stitcher.launch
```

If 3 camera image msg transmission rates are different

```bash
roslaunch image_processing image_stitcher_queue.launch
```

### Terminal 3: Visualize Camera Topics

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Visualization

3.1. Visualize raw image msg in Rviz

```bash
rviz -d rviz/camera-example.rviz
```

3.2. Visualize compressed image msg in rqt_image_view

```bash
ros run rqt_image_view rqt_image_view 
```

### Terminal 4: ROSBag Recording

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Record ROSBag

```bash
source rosbag/record_images.sh
```

### ROSBag Playing Example

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Play ROSBag for example

```bash
rosbag play bags/images-2025_0106_1714/2025-01-06-17-14-51.bag
```