ssh arg@192.168.0.20
byobu

Add New Page: fn + f2
change: fn + f3 / f4

cd moos-dawg-2024
source Docker/ipc/cpu/cpu_run.sh
source environment_demo.sh
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.131.201 port:=2369

cd robotx-2022/
source nuc_run.sh 
source environment.sh 
roslaunch lidar_crop lidar_crop.launch 



cd js-perceptions/
source minimal_docker_nctu_run.sh 
ros2 launch halo_radar_visualize halo_radar_bringup.launch.py


