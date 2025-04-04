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

## Enter the repo

```bash
cd perception-fusion
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
~/perception-fusion/ros2_ws/src/realsense2_camera_example/config/rs_launch_1.yaml
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
ros2 launch realsense2_camera_example side_camera.launch.py config_file:=config/rs_launch_1.yaml
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
~/perception-fusion/ros2_ws/src/realsense2_camera_example/config/rs_launch_3.yaml
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
ros2 launch realsense2_camera_example side_camera.launch.py config_file:=config/rs_launch_3.yaml
```

### Visualize Camera Topics

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Visualization

3.1. Visualize raw image msg in Rviz2

```bash
rviz2 -d rviz2/camera-example.rviz
```

3.2. Visualize compressed image msg in rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view 
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

### ROS2 Bag Playing Example

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```

2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Enter Directory of bag for example

```bash
cd bags/images-2024_1213_1533
```

3. Play ROS2 Bag for example

```bash
ros2 bag play images-2024_1213_1533_0.db3 
```

### Record Camera Intrinsics Example

Refer "Usage 1: Single D435 camera" or "Usage 2: Triple D435 camera" step 1-5

6. Run Camera Info Saver Node

For Single camera:

```bash
ros2 launch realsense2_camera_example camera_info_saver.launch.py save_dir:=/home/arg/perception-fusion/data/camera_info config_file:=config/rs_launch_1.yaml
```

For Triple camera:

```bash
ros2 launch realsense2_camera_example camera_info_saver.launch.py save_dir:=/home/arg/perception-fusion/data/camera_info config_file:=config/rs_launch_3.yaml
```