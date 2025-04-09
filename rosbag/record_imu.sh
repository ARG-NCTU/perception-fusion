#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S)

BAGS=$HOME/perception-fusion/bags/imu-$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -p $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /pixhawk1/mavros_1/imu/data \
    /pixhawk1/mavros_1/imu/data_raw \
    /pixhawk1/mavros_1/imu/diff_pressure \
    /pixhawk1/mavros_1/imu/mag \
    /pixhawk1/mavros_1/imu/static_pressure \
    /pixhawk1/mavros_1/imu/temperature_baro \
    /pixhawk1/mavros_1/imu/temperature_imu \
    /pixhawk2/mavros_2/imu/data \
    /pixhawk2/mavros_2/imu/data_raw \
    /pixhawk2/mavros_2/imu/diff_pressure \
    /pixhawk2/mavros_2/imu/mag \
    /pixhawk2/mavros_2/imu/static_pressure \
    /pixhawk2/mavros_2/imu/temperature_baro \
    /pixhawk2/mavros_2/imu/temperature_imu
