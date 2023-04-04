#!/bin/sh

print_help()
{
  echo Convenience script for launching a ifm3d-ros2 launchfile from a docker container.
}

test $# -lt 1 && print_help

IMAGE="ifm3d-ros:humble-x86_64_latest"

docker run -it $IMAGE \
    sh -c ". /opt/ros/humble/setup.sh; \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
    . /home/ifm/colcon_ws/ifm3d-ros2/install/setup.sh; \
    ros2 launch ifm3d_ros2 camera.launch.py $@"
