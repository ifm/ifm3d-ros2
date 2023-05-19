
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

## ToDo

We are currently working on rounding out the feature set of our ROS2 interface. Our current objectives are to get the feature set to an equivalent
level to that of our ROS1 interface and to tune the ROS2/DDS performance to optimize the usage of our cameras from within ROS2 system (for different DDS implementations).
Thanks for your patience as we continue to ensure our ROS2 interface is feature-rich, robust, and performant. Your feedback is greatly appreciated.

## Known Issues
+ This ROS 2 node build on top of the `ifm3d` API which handles the data communication between the camera platform and the outside world. This is based on ASIO which conflicts with the DDS middleware fastRTPS implementation in ROS. We are currently working on a solution. Until then we suggest to use cyclone DDS for all ROS 2 distributions.
+ Installing ifm3d API with it's default runtime libs may result in multiple versions of glog on the system: this results in a compilable but non-functional ROS node.
Please either use the glog version as included in your Ubuntu release (if compatible) or uninstall any incompatible lib version before installing the ifm3d required version.
+ A couple publishers are missing, for the radial distance noise image, the reflectivity image and the asynchronous diagnostic in particular.
## LICENSE
Please see the file called [LICENSE](LICENSE).
