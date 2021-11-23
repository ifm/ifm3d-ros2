# ifm3d-ros2

**This release is intended to be used with the O3R camera platform ONLY. For other ifm cameras please see the main branch.**  

**NOTE: The ifm3d-ros2 package has had major changes recently. Please be aware that this might cause problems on your system for building pipelines based on our old build instructions.**  
We tried to ensure backward compatibility where ever possible. If you find any major breaks, please let us know.

`ifm3d-ros2` is a wrapper around [ifm3d](https://github.com/ifm/ifm3d) enabling the usage of ifm O3R ToF camera platform from within [ROS 2](https://index.ros.org/doc/ros2/) software systems.

![rviz](doc/figures/O3R_merged_point_cloud.png)

## Software Compatibility Matrix

| ifm3d_ros2 version | ifm3d version | ROS 2 distribution |
| ----------- | ----------- | ----------- |
| 1.0.0 | 0.92.0 | Galactic |
| 0.3.0 DEPRECATED | 0.17.0 | Dashing, Eloquent |
| 0.2.0 DEPRECATED | 0.12.0 | Dashing |
| 0.1.1 DEPRECATED | 0.12.0 | Dashing |
| 0.1.0 DEPRECATED | 0.12.0 | Dashing |

> Note: `ifm3d_ros2` version 1.0.0 is released as an early developer release for the O3R camera.


## Building and Installing the Software

### Pre-requisites

1. [ROS2](https://docs.ros.org/en/galactic/Installation.html)
2. [ifm3d](https://ifm.github.io/ifm3d-docs/content/source_build.html) - be sure to build the IMAGE module (using PCL and OPENCV).

In addition to the base packages found in `ros-*-desktop-full` you will need the following ROS packages:
- cv_bridge
- vision_opencv
- pcl-conversions

These two packages are only required for testing but not at runtime:  
- launch_testing
- launch_testing_ament_cmake


On debian based systems they may be installed as follows (replacing `galactic`with your target ROS2 distribution).
```
$ sudo apt install ros-galactic-cv-bridge ros-galactic-vision-opencv ros-galactic-pcl-conversions
```
```
$ sudo apt install ros-galactic-launch-testing ros-galactic-launch-testing-ament-cmake
```

### Building from source

Please see the separate building instruction for building from source: [here](doc/building.md)

### Launch the node
Launch the camera node (assuming you are in `~/colcon_ws/`):
```
$ . install/setup.bash
$ ros2 launch ifm3d_ros2 camera_managed.launch.py
```

Open another shell and start the RVIZ node to visualize the data coming from the camera:
```
$ ros2 launch ifm3d_ros2 rviz.launch.py
```
> Note: `rviz.launch.py` does not include the camera node itself, but subscribes to published topics (distance, amplitude, etc). A camera node need to be running in parallel to rviz (you can use `camera_managed.launch`).
> Note also that the `rviz.launch.py` launchfile assumes one data stream publishes at `/ifm3d/camera/<topic_name>`.

At this point, you should see an rviz window that looks something like the image below (note that this is the view from 3 camera heads):
![rviz1](doc/figures/O3R_merged_point_cloud.png)

Congratulations! You can now have complete control over the O3R perception platform from inside ROS.



## ROS Interface

### Parameters

| Name | Data Type | Default Value | Description |
| --------- | --------- | --------- | --------- |
| ~/frame_latency_thresh | float | 1.0 | Time (seconds) used to determine that timestamps from the camera cannot be trusted. When this threshold is exceeded, when compared to system time, we use the reception time of the frame and not the capture time of the frame. |
| ~/ip | string | 192.168.0.69 | The ip address of the camera. |
| ~/password | string | | The password required to establish an edit session with the camera. |
| ~/schema_mask | int16 |0xf | The schema mask to apply to the active session with the frame grabber. This determines which images are available for publication from the camera. More about schemas can be gleaned from the ifm3d project |
| ~/timeout_millis | int | 500 | The number of milliseconds to wait for the framegrabber to return new frame data before declaring a "timeout" and to stop blocking on new data. |
| ~/timeout_tolerance_secs | float | 5.0 | The wall time to wait with no new data from the camera before trying to establish a new connection to the camera. This helps to provide robustness against camera cables becoming unplugged or other in-field pathologies which would cause the connection between the ROS node and the camera to be broken. |
| ~/sync_clocks DEPRECATED | bool | false | Attempt to sync the camera clock to the system clock at start-up. The side-effect is that timestamps on the image should reflect the capture time as opposed to the receipt time. Please note: resolution of this sync is only granular to 1 second. If fine-grained image acquisition times are needed, consider using the on-camera NTP server (available on select camera models). |
| ~/xmlrpc_port | uint16 | 80 | The TCP port the camera's xmlrpc server is listening on for requests. |
| ~/pcic_port | uint16 | 50010 | The TCP (data) port the camera's pcic server is listening on for requests. |

### Published Topics

| Name | Data Type | Quality of Service (QoS) | Description |
| -------- | -------- | -------- | -------- |
| amplitude | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The normalized amplitude image |
| cloud | sensor_msgs/msg/PointCloud2 | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The point cloud data |
| confidence | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The confidence image |
| distance | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The radial distance image |
| raw_amplitude | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The raw amplitude image |
| xyz_image | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | A 3-channel image encoding of the point cloud. Each of the three image channels respesent a spatial data plane encoding the x, y, z Cartesian values respectively. |
| rgb | sensor_msgs/msg/Image | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The RGB 2D image of the 2D imager |

### Subscribed Topics

None.

### Advertised Services
| Name | Service Definition | Description |
| ------ |  ------ |  ------ | 
| Dump | <a href="srv/Dump.srv">ifm3d/Dump</a> | Dumps the state of the camera parameters to JSON |
| Config | <a href="srv/Config.srv">ifm3d/Config</a> | Provides a means to configure the camera and imager settings, declaratively from a JSON encoding of the desired settings. |


## Additional Documentation

* [Inspecting and configuring the camera/imager settings](doc/dump_and_config.md)
* [Building the ROS node from source](doc/building.md)
* [Visualization](doc/visualization.md)
* [Running the ROS node on a distributed system](doc/distributed_run.md)
* [`ifm3d` API RPC error codes](doc/rpc_error_codes.md)

## ToDo

We are currently working on rounding out the feature set of our ROS2 interface. Our current objectives are to get the feature set to an equivalent
level to that of our ROS1 interface and to tune the ROS2/DDS performance to optimize the usage of our cameras from within ROS2 system (for different DDS implementations).  
Thanks for your patience as we continue to ensure our ROS2 interface is feature-rich, robust, and performant. Your feedback is greatly appreciated.

## Known limitations
This ROS 2 node build on top of the `ifm3d` API which handles the data communication between the camera platform and the outside world. This is based on ASIO which conflicts with the DDS middleware fastRTPS implementation in ROS (until ROS foxy). We are currently working on a solution. Until then we suggest to use cyclone DDS for older ROS 2 distributions.

## LICENSE
Please see the file called [LICENSE](LICENSE).
