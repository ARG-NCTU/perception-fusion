# Perception Fusion

Record-ROS2-Bag Guide

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

1. Docker Run (or Join)

```bash
source Docker/ros2-cpu/run.sh
```

2. Record ROS2 Bag 

```bash
source ros2bag/record_all_sensor.sh
```
