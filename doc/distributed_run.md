# How to RUN this ROS2 wrapper from inside a Docker container image

## Run it natively
Follow these steps to get the ROS 2 node to run on your system.  

1. Build from source: please don't forget to build the ifm3d C++ API form source as well. See the installation documentation [here](/doc/building.md)
2. This step is only required for ROS 2 foxy and older distributions:  
(Install alternative DDS settings configuration: This needs to be done once: `$ sudo apt-get install ros-foxy-rmw-cyclonedds-cpp`)
3. Source the `ìfm3d_ros2` installation: `$ source ~/colcon_ws/install/setup.bash`
4. This step is only required for ROS 2 foxy and older distributions:  
(Change DDS settings: This needs to be done every time (changes the DDS setup) and needs to be done after the source of ROS and the ifm3d_ros2 node: `$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp `)
6. Run the ros2 node: `$ ros2 launch ifm3d_ros2 camera_managed.launch.py `

You should now be able to see these topics (via `$ ros2 topic list`):
```
/ifm3d/camera/amplitude
/ifm3d/camera/cloud
/ifm3d/camera/confidence
/ifm3d/camera/distance
/ifm3d/camera/extrinsics
/ifm3d/camera/raw_amplitude
/ifm3d/camera/transition_event
/ifm3d/camera/xyz_image
/parameter_events
/rosout
/tf_static
```

Please keep in mind that the launch file `camera_managed.launch.py` uses the default TCP port 50010 (for the data communication), so the camera head's 3D imager needs to be connected to the phyical port 0 (marked `PORT0` on the VPU).

## RUN ROS node in a docker container
Follow these steps to get our supplied Docker container to run on your system.  

### Docker container (master)
The instructions below apply to ROS2 galactic. Please change the commands to suit your ROS 2 distribution.  

1. Download and save the Docker container image to your deployment system.
2. Run the Docker container image with container networking settings: `docker run -ti amd64/ifm3d-ros2:dev_galactic bash`
3. Source ROS2 and the `ìfm3d_ros2` installation: `$ source /opt/ros/galactic/setup.bash && source /home/ifm/colcon_ws/ifm3d-ros2/install/setup.bash`
4. (All ROS nodes should see each other as long as they are on the same ROS domain ID. The default `ROS_DOMAIN_ID` is 0 and doesn't get changed here)
5. Run the ros2 node: `$ ros2 launch ifm3d_ros2 camera_managed.launch.py `

### Second ROS2 galactic installation (slave)
The instructions below apply to ROS2 galactic. Please change the commands to suit your ROS 2 distribution.  

1. (Source ROS2 galactic on slave): `source /opt/ros/galactic/setup.bash`
2. Check that ROS topics are available (on `ROS_DOMAIN_ID=0`): `ros2 topic list`

You should now be able to see these topics (via `$ ros2 topic list`) on the slave machine as well:
```
/ifm3d/camera/amplitude
/ifm3d/camera/cloud
/ifm3d/camera/confidence
/ifm3d/camera/distance
/ifm3d/camera/extrinsics
/ifm3d/camera/raw_amplitude
/ifm3d/camera/transition_event
/ifm3d/camera/xyz_image
/parameter_events
/rosout
/tf_static
```