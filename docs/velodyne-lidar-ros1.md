cd perception-fusion
source Docker/ros1-cpu/run.sh
source environment_ros1.sh
roslaunch velodyne_example velodyne_example.launch 

cd robotx-2022/
source ipc_run.sh 
source environment.sh 
roslaunch js_lidar_crop lidar_crop_real.launch 



