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

1. Setup your computer IP

Do not set the same IP as the Velodyne Lidar.

For example, if the Velodyne Lidar IP address is 192.168.131.201, you can set your computer IP address to 192.168.131.xxx with Netmask 255.255.255.0 and unsetting Gateway.

If you want to change the Velodyne Lidar settings, you can type the IP address in your browser. e.g. 192.168.131.201

2. Docker Run

```bash
source Docker/ros2-cpu/run.sh
```

3. Edit device_ip and port in yaml file

```bash
vim /opt/ros/humble/share/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml
```

Edit device_ip and port in yaml file with your Velodyne Lidar IP address and port.
For example: 
    device_ip:=192.168.131.201 
    port:=2369

4. Run Velodyne Node

```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py 
```

### Terminal 2: Rviz2

1. Docker Join

```bash
source Docker/ros2-cpu/run.sh
```
2. Source ROS2 Environment

```bash
source environment_ros2.sh
```

3. Run Rviz2

```bash
rviz2 -d rviz2/lidar-example.rviz
```