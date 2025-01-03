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

## Enter the repo

```bash
cd ros-humble-ros1-bridge-builder
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

2. Building All ROS1 package

```bash
source build_ros1_all.sh
```

## Usage

### Terminal 1: ROSCORE

1. Docker Run

Run this script to pull docker image to your workstation.

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
### Terminal 2: Nuscene Dataset Visualization

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Run Nuscene Dataset Visualization

```bash
roslaunch nuscene_visualize visualize_launch.launch
```