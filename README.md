# Perception Fusion

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

1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros1-ros2/run.sh
```

2. Docker Join

If want to enter same docker image, type below command.

```bash
source Docker/ros1-ros2/run.sh
```

3. Building ROS1 bridge

```bash
source build_ros1_bridge.sh
```

## Usage

### Terminal 1

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
### Terminal 2

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 & ROS2 Environment

```bash
source environment_ros1.sh
source environment_ros2.sh
```

3. Run Ros1_bridge

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Terminal 3

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Run ROS1 Bag Example

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

### Terminal 4

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
rviz
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
rviz2
```

