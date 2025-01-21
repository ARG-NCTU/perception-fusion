# Perception Fusion

Plot Trajectory Guide

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

2. Build Catkin Workspace

```bash
source build_ros1_all.sh
```

## Usage

### Terminal 1

1. Docker Run

```bash
source Docker/ros1-cpu/run.sh
```

2. Source ROS1 Environment

```bash
source environment_ros1.sh
```

3. Edit Input bag_file and Output csv_file ROS Parameters in Launch file

```bash
~/perception-fusion/ros1_ws/src/bag_to_tools/launch/bag_to_csv.launch
```

4. Launch bag tools

```bash
roslaunch bag_to_tools bag_to_csv.launch 
```

### Terminal 2

1. Clone the uav-usv-traj Repo

```bash
cd ~/ && git clone git@github.com:ARG-NCTU/uav-usv-traj.git && cd uav-usv-traj
```

2. Docker Run

```bash
source docker_run.sh
```

3. Launch Jupyter Notebook

```bash
source jupyter.sh
```

Copy and Paste the url to your web browser

4. Run notebook

Open and run every cell of this notebook: 010-csv-plot-trajectory-on-map.ipynb
