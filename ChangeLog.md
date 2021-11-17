## 1.0.0 (2021-11) unreleased
* Update the ROS node for the O3R camera
* add a 2D RGB data publisher
* add support for specifying the PCIC data communication TCP port
* add example yaml file for multi PCIC TCP port settings
* add boost to the list of dependencies

### known limitations
* DDS settings need to be set to cyclonedds
* custom services can result in seg fault errors for ROS2 foxy default dds settings


## 0.3.0 (2020-01-31)

* Publish camera temperature (Thanks @dustingooding of Houston Mechatronics)
* Tested on Eloquent

## 0.2.0 (2019-06-26)

* Provided an implementation of the `Dump` service
* Provided an implementation of the `dump` command-line tool
* Provided an implementation of the `Config` service
* Provided an implementation of the `config` command-line tool

## 0.1.1 (2019-06-25)

* Unit vectors are published on a "latched topic". Please see `qos.hpp` for
  the specifics of the QoS profile `ifm3d_ros2::LatchedQoS`.
* On-camera extrinsics in support of off-line point cloud computation are now
  exposed.
* Bootstrapped some unit tests
* Cleaned up some documentation.

## 0.1.0 (2019-06-06)

* Initial (alpha) release
