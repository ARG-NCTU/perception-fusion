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
cd ~/perception-fusion
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

### Terminal 1: Pixhawk Mavros

1. Docker Run

```bash
source Docker/ros1-cpu/run.sh
```

2. Source ROS1 Environment

Use Default IP (127.0.0.1):
```bash
source environment_ros1.sh
```

Use another IP:
```bash
source environment_ros1.sh MASTER_IP ROS_IP
```

3. Edit Pixhawk Mavros launch file 

For 1 pair:
```bash
rosed pixhawk_mavros gps_imu.launch
```

For 2 pair:
```bash
rosed pixhawk_mavros gps_imu_2.launch
```

Modify the USB port (ttyACM0) and IP (Substitude 127.0.0.1 with your ROS IP)

4. Launch Pixhawk Mavros

For 1 pair:
```bash
roslaunch pixhawk_mavros gps_imu.launch
```

For 2 pair:
```bash
roslaunch pixhawk_mavros gps_imu_2.launch
```

### Terminal 2: Echo Pixhawk Mavros topics

1. Docker Join

```bash
source Docker/ros1-cpu/join.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Echo topics

Echo GPS topic:
```bash
rostopic echo /pixhawk1/mavros/global_position/raw/fix
```

Echo Imu topic:
```bash
rostopic echo /pixhawk1/mavros/imu/data
```

