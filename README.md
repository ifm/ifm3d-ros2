# ifm3d-ros2

**NOTE: The ifm3d-ros2 package has had major changes recently. Please be aware that this might cause problems on your system for building pipelines based on our old build instructions.**

------

**This release is intended to be used with the O3R camera platform ONLY. For other ifm cameras (e.g. O3D3xx and O3X2xx) please see the tagged release 0.3.0.**


`ifm3d-ros2` is a wrapper around [ifm3d](https://github.com/ifm/ifm3d) enabling the usage of ifm O3R ToF camera platform from within [ROS 2](https://index.ros.org/doc/ros2/) software systems.

![rviz](doc/figures/O3R_merged_point_cloud.png)

## Software Compatibility Matrix

### Prerelease versions

| ifm3d_ros2 version | ifm3d version | (O3R) embedded FW versions | ROS 2 distribution |
| ------------------ | ------------- | -------------------------- | ------------------ |
| 1.1.0 (unreleased) | 1.2.4         | 1.0.14                     | Humble             |
### Release versions

| ifm3d_ros2 version | ifm3d version | (O3R) embedded FW versions | ROS 2 distribution |
| ------------------ | ------------- | -------------------------- | ------------------ |
| 1.0.1              | 0.93.0        | 0.14.23                    | Foxy               |

### Deprecated ifm3d-ros2 Versions
The following versions are deprecated and no longer supported.
| ifm3d_ros2 version | ifm3d version | ROS 2 distribution |
| ------------------ | ------------- | ------------------ |
| 1.0.1 DEPRECATED   | 0.93.0        | Galactic           |
| 1.0.0 DEPRECATED   | 0.92.0        | Galactic           |
| 0.3.0 DEPRECATED   | 0.17.0        | Dashing, Eloquent  |
| 0.2.0 DEPRECATED   | 0.12.0        | Dashing            |
| 0.1.1 DEPRECATED   | 0.12.0        | Dashing            |
| 0.1.0 DEPRECATED   | 0.12.0        | Dashing            |




## Building and Installing the Software

### Pre-requisites

1. [ROS2](https://docs.ros.org/en/galactic/Installation.html)
2. [ifm3d](ifm3d/doc/sphinx/content/README:ifm3d%20Overview)

These two packages are only required for testing but not at runtime:
- launch_testing
- launch_testing_ament_cmake


On debian based systems they may be installed as follows (replacing `galactic`with your target ROS2 distribution).
```
$ sudo apt install ros-galactic-launch-testing ros-galactic-launch-testing-ament-cmake
```


### Building from source

Please see the separate building instruction for building from source: [here](doc/building.md).
For running the ROS node on an embedded system such as the O3R VPU please build the software inside a [Docker container](Dockerfile).

### Launch the node
Launch the camera node (assuming you are in `~/colcon_ws/`):
```
$ . install/setup.bash
$ ros2 launch ifm3d_ros2 camera.launch.py
```

> Note: we also provide a helper launch file to start multiple camera nodes. See the documentation [here](doc/multi_head.md).

To visualize the data with RViz, set the `visualization` argument of the launch script to `true`:
```
$ ros2 launch ifm3d_ros2 camera.launch.py visualization:=true
```


![rviz1](doc/figures/O3R_merged_point_cloud.png)
Congratulations! You can now have complete control over the O3R perception platform from inside ROS2.



## ROS Interface

### Parameters

| Name                     | Data Type | Default Value | Description                                                                                                                                                                                                                                                                                                                                                                           |
| ------------------------ | --------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ~/frame_latency_thresh   | float     | 1.0           | Time (seconds) used to determine that timestamps from the camera cannot be trusted. When this threshold is exceeded, when compared to system time, we use the reception time of the frame and not the capture time of the frame.                                                                                                                                                      |
| ~/ip                     | string    | 192.168.0.69  | The ip address of the camera.                                                                                                                                                                                                                                                                                                                                                         |
| ~/password               | string    |               | The password required to establish an edit session with the camera.                                                                                                                                                                                                                                                                                                                   |
| ~/schema_mask            | int16     | 0xf           | The schema mask to apply to the active session with the frame grabber. This determines which images are available for publication from the camera. More about schemas can be gleaned from the ifm3d project                                                                                                                                                                           |
| ~/timeout_millis         | int       | 500           | The number of milliseconds to wait for the framegrabber to return new frame data before declaring a "timeout" and to stop blocking on new data.                                                                                                                                                                                                                                       |
| ~/timeout_tolerance_secs | float     | 5.0           | The wall time to wait with no new data from the camera before trying to establish a new connection to the camera. This helps to provide robustness against camera cables becoming unplugged or other in-field pathologies which would cause the connection between the ROS node and the camera to be broken.                                                                          |
| ~/sync_clocks DEPRECATED | bool      | false         | Attempt to sync the camera clock to the system clock at start-up. The side-effect is that timestamps on the image should reflect the capture time as opposed to the receipt time. Please note: resolution of this sync is only granular to 1 second. If fine-grained image acquisition times are needed, consider using the on-camera NTP server (available on select camera models). |
| ~/xmlrpc_port            | uint16    | 80            | The TCP port the camera's xmlrpc server is listening on for requests.                                                                                                                                                                                                                                                                                                                 |
| ~/pcic_port              | uint16    | 50010         | The TCP (data) port the camera's pcic server is listening on for requests.                                                                                                                                                                                                                                                                                                            |
| ~/log_level| string | warning| ifm3d-ros2 node logging level.  |

### Published Topics

| Name            | Data Type                   | Quality of Service (QoS)                                          | Description                                                   |
| --------------- | --------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------- |
| amplitude       | sensor_msgs/msg/Image       | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The normalized amplitude image                                |
| cloud           | sensor_msgs/msg/PointCloud2 | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The point cloud data                                          |
| confidence      | sensor_msgs/msg/Image       | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The confidence image                                          |
| distance        | sensor_msgs/msg/Image       | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The radial distance image                                     |
| raw_amplitude | sensor_msgs/msg/Image     | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The raw amplitude image (currently not available for the O3R) |
| rgb             | sensor_msgs/msg/Image       | <a href="include/ifm3d_ros2/qos.hpp">ifm3d_ros::LowLatencyQoS</a> | The RGB 2D image of the 2D imager                             |

### Subscribed Topics

None.

### Advertised Services
| Name    | Service Definition                          | Description                                                                                                               |
| ------- | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Dump    | <a href="srv/Dump.srv">ifm3d/Dump</a>       | Dumps the state of the camera parameters to JSON                                                                          |
| Config  | <a href="srv/Config.srv">ifm3d/Config</a>   | Provides a means to configure the camera and imager settings, declaratively from a JSON encoding of the desired settings. |
| Softon  | <a href="srv/Softon.srv">ifm3d/Softon</a>   | Provides a means to quickly change the camera state from IDLE to RUN.                                                     |
| Softoff | <a href="srv/Softoff.srv">ifm3d/Softoff</a> | Provides a means to quickly change the camera state from RUN to IDLE.                                                     |



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

## Known Issues
+ This ROS 2 node build on top of the `ifm3d` API which handles the data communication between the camera platform and the outside world. This is based on ASIO which conflicts with the DDS middleware fastRTPS implementation in ROS. We are currently working on a solution. Until then we suggest to use cyclone DDS for all ROS 2 distributions.
+ Installing ifm3d API with it's default runtime libs may result in multiple versions of glog on the system: this results in a compilable but non-functional ROS node.
Please either use the glog version as included in your Ubuntu release (if compatible) or uninstall any incompatible lib version before installing the ifm3d required version.

## LICENSE
Please see the file called [LICENSE](LICENSE).
