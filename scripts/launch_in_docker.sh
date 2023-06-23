#!/bin/sh

print_help()
{
  echo Convenience script for launching a ifm3d-ros2 launchfile from a docker container.
  echo
  echo \$1: Docker image name
  echo "\$2: Launchfile (Optional; Default: 'camera_managed.launch.py')"
  exit 22
}

test $# -lt 1 && print_help

image=${1}
shift
launchfile=${1:-camera_managed.launch.py}
test $# -ge 1 && shift

# Including "-it" so that CTRL-C works
docker run -it -p 11311:11311 $image \
  sh -c ". /opt/ros/foxy/setup.sh; \
  . /home/ifm/colcon_ws/ifm3d-ros2/install/setup.sh; \
  ros2 launch ifm3d_ros2 $launchfile $@"

