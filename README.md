# Perception Fusion

## Clone repo 

```
git clone --recursive git@github.com:ARG-NCTU/moos-dawg-2024.git
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

## Example 1: run the bridge and the example talker and listener

### Example 1a: ROS 1 talker and ROS 2 listener

#### Terminal 1

1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS 1 Environment

```bash
source environment_ros1.sh
```

3. Roscore

```bash
roscore
```
#### Terminal 2

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS 2 Environment

```bash
source environment_ros2.sh
```

3. Run Ros1_bridge

```bash
ros2 run ros1_bridge dynamic_bridge
```

#### Terminal 3

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Source ROS 1 Environment

```bash
source environment_ros1.sh
```

3. Run ROS1 Talker

```bash
rosrun rospy_tutorials talker
```

#### Terminal 4

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Run ROS2 Listener

```bash
ros2 run demo_nodes_cpp listener
```

### Example 1b: ROS 2 talker and ROS 1 listener

Terminal 1 & 2 are same with Example 1a.

#### Terminal 3

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Run ROS2 Talker

```bash
ros2 run demo_nodes_py talker
```

#### Terminal 4

1. Docker Run

```bash
source Docker/ros1-ros2/run.sh
```

2. Run ROS1 Listener

```bash
rosrun roscpp_tutorials listener
```