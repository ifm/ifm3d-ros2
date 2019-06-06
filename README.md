ifm3d-ros2
==========
`ifm3d-ros2` is a wrapper around [ifm3d](https://github.com/lovepark/ifm3d)
enabling the usage of ifm pmd-based ToF cameras from within
[ROS 2](https://index.ros.org/doc/ros2/) software systems.

![rviz](doc/figures/rviz_sample.png)

Software Compatibility Matrix
=============================
<table>
  <tr>
    <th>ifm3d-ros2 version</th>
    <th>ifm3d version</th>
    <th>ROS 2 distribution(s)</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>0.12.0</td>
    <td>Dashing</td>
  </tr>
</table>

Building and Installing the Software
====================================

### Pre-requisites

1. [ROS2](https://index.ros.org/doc/ros2/Installation/)
2. [ifm3d](https://github.com/ifm/ifm3d)

### Building from source

`ifm3d-ros2` is intended to be built with
[colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html#build-ros-2-packages.)
To that end, the shell commands below assume a single colcon workspace in which
`ifm3d-ros2` will be built.

Create the colcon workspace:
```
$ mkdir ~/colcon/ifm3d_ros2/src
```

Clone the `ifm3d-ros2` github repo into this workspace and build it:
```
$ cd ~/colcon/ifm3d_ros2/src
$ git clone https://github.com/ifm/ifm3d-ros2.git ifm3d_ros2
$ cd ..
$ colcon build
Starting >>> ifm3d_ros2
Finished <<< ifm3d_ros2 [17.6s]

Summary: 1 package finished [17.8s]
```

Launch the camera node (assuming you are in `~/colcon/ifm3d_ros2`):
```
$ . install/setup.bash
$ ros2 launch ifm3d_ros2 camera_managed.launch.py
```

In a new shell, to visualize the data from the camera in rviz (assuming you are in
`~/colcon/ifm3d_ros2`):
```
$ . install/setup.bash
$ ros2 launch ifm3d_ros2 rviz.launch.py
```

ROS Interface
=============

### Parameters

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>~/frame_latency_thresh</td>
    <td>float</td>
    <td>1.0</td>
    <td>
      Time (seconds) used to determine that timestamps from the camera cannot
      be trusted. When this threshold is exceeded, when compared to system
      time, we use the reception time of the frame and not the capture time of
      the frame.
    </td>
  </tr>
  <tr>
    <td>~/ip</td>
    <td>string</td>
    <td>192.168.0.69</td>
    <td>
      The ip address of the camera.
    </td>
  </tr>
  <tr>
    <td>~/password</td>
    <td>string</td>
    <td></td>
    <td>
      The password required to establish an edit session with the camera.
    </td>
  </tr>
  <tr>
    <td>~/schema_mask</td>
    <td>uint16</td>
    <td>0xf</td>
    <td>
      The schema mask to apply to the active session with the frame
      grabber. This determines which images are available for publication from
      the camera. More about schemas can be gleaned from the
      <a href="https://github.com/ifm/ifm3d">ifm3d</a> project.
    </td>
  </tr>
  <tr>
    <td>~/timeout_millis</td>
    <td>int</td>
    <td>500</td>
    <td>
      The number of milliseconds to wait for the framegrabber to return new
      frame data before declaring a "timeout" and to stop blocking on new
      data.
    </td>
  </tr>
  <tr>
    <td>~/timeout_tolerance_secs</td>
    <td>float</td>
    <td>5.0</td>
    <td>
      The wall time to wait with no new data from the camera before trying to
      establish a new connection to the camera. This helps to provide
      robustness against camera cables becoming unplugged or other in-field
      pathologies which would cause the connection between the ROS node and the
      camera to be broken.
    </td>
  </tr>
  <tr>
    <td>~/sync_clocks</td>
    <td>bool</td>
    <td>false</td>
    <td>
      Attempt to sync the camera clock to the system clock at start-up. The
      side-effect is that timestamps on the image should reflect the capture
      time as opposed to the receipt time. Please note: resolution of this
      synch is only granular to 1 second. If fine-grained image acquisition
      times are needed, consider using the on-camera NTP server (available on
      select camera models).
    </td>
  </tr>
  <tr>
    <td>~/xmlrpc_port</td>
    <td>uint16</td>
    <td>80</td>
    <td>
      The TCP port the camera's xmlrpc server is listening on for requests.
    </td>
  </tr>
</table>

### Published Topics

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>amplitude</td>
    <td>sensor_msgs/Image</td>
    <td>The normalized amplitude image</td>
  </tr>
  <tr>
    <td>cloud</td>
    <td>sensor_msgs/PointCloud2</td>
    <td>The point cloud data</td>
  </tr>
  <tr>
    <td>confidence</td>
    <td>sensor_msgs/Image</td>
    <td>The confidence image</td>
  </tr>
  <tr>
    <td>distance</td>
    <td>sensor_msgs/Image</td>
    <td>The radial distance image</td>
  </tr>
  <tr>
    <td>raw_amplitude</td>
    <td>sensor_msgs/Image</td>
    <td>The raw amplitude image</td>
  </tr>
  <tr>
    <td>unit_vectors</td>
    <td>sensor_msgs/Image</td>
    <td>The rotated unit vectors</td>
  </tr>
  <tr>
    <td>xyz_image</td>
    <td>sensor_msgs/Image</td>
    <td>
      A 3-channel image encoding of the point cloud. Each of the three image
      channels respesent a spatial data plane encoding the x, y, z Cartesian
      values respectively.
    </td>
  </tr>
</table>

### Subscribed Topics

None.

### Advertised Services

Currently none. However, we are actively working to round out the features of
our ROS2 interface to be, at least, equivalent to that of our ROS1 node. Check
back soon for things like `Dump`, `Config`, etc. In the interim, to configure
imager parameters, you should use the underlying (non-ROS) utilities available
in [ifm3d](https://github.com/ifm/ifm3d).

Additional Documentation
========================
Coming soon.

TODO
====
We are currently working on rounding out the feature set of our ROS2
interface. Our current objectives are to get the feature set to an equivalent
level to that of our ROS1 interface and to tune the ROS2/DDS performance to
optimize the usage of our cameras from within ROS2 system. Thanks for your
patience as we continue to ensure our ROS2 interface is feature-rich, robust,
and performant. Your feedback on our
[issue tracker](https://github.com/ifm/ifm3d-ros2/issues) is greatly appreciated.

Please see the file called [TODO](TODO.md) for more information of what we are
currently working on.


LICENSE
=======
Please see the file called [LICENSE](LICENSE).
