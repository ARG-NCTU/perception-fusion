# Perception Fusion

Localization-ROS2 Guide

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

2. Building All ROS2 package

```bash
source build_ros2_all.sh
```

## Usage

### Terminal 1: Velodyne Lidar

1. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

2. Run Localization 

```bash
ros2 launch localization2 localization_gps_imu.launch.py
```

### Terminal 2: Set Origin with GPS and Coordinate with IMU

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```
2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run ROS2 Service Call

```bash
ros2 service call /localization_gps_imu/set_origin std_srvs/srv/Empty
```