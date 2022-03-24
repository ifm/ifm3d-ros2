#!/bin/bash
echo "Preparing the environment"
galactic
source install/setup.bash

echo "Starting the ros nodes for four cameras"
echo "Starting rviz"
(trap 'kill 0' SIGINT; ros2 launch ifm3d_ros2 multi_cameras.launch.py namespace:='ifm3d_ros2' params:='/home/shareduser/ros2_ws/src/ifm3d-ros2/config/params_4cams.yaml' & rviz2 -d "/home/shareduser/ros2_ws/src/ifm3d-ros2/config/modex.rviz")




