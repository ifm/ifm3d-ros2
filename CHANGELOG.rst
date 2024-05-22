^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ifm3d-ros2
^^^^^^^^^^^^^^^^^^^^^^^^^

1.1
===

1.1.0 (unreleased preparations for FW 1.4 release)
------------------
* Update for compatibility with O3R FW 1.4.x and ifm3d >= 1.4.3
  * Tested with FW 1.4.22 and ifm3d API 1.4.3
  * Tested with FW 1.4.22 and ifm3d API 1.5.3

1.1.0
------------------
* Update for compatibility with O3R FW >= 1.0.14 and ifm3d >= 1.2.6
  * Using ifm3d::O3R instead of ifm3d::Device to access camera
  * Added `buffer_id_list` param to define which data to get from the device and consequently which topics to publish. Maps to ifm3d `buffer_id`.

* Remove the need to use cyclonedds: TODO: verification

* Remove dependency to Boost library

* Include Dockerfile and helper scripts:
  * Dockerfile for building the ROS2 node inside a Docker container
  * Helper build and launch scripts for ROS Docker images
  * Adds docker-compose example files for launching the ROS node on a O3R edge device: VPU

* Moved Publishers into node namespace: This matches the ifm3d-ros2 node version's naming (<= 1.0.x).

* Switch to a new camera.launch.py launch file (instead of camera_managed and camera_standalone): launches and activates a single camera node
  * focusses on configurability
  * camera_default_parameters.yaml contains only default params
  * Add option for visualization to launch file: If visualization is true, RViz2 with the config file from etc is opened
  * Formatting of node names improved
* old launch files are marked as deprecated and will be removed with next release: please migrate your launch files to the new structure

* Switching json handling to ifm3d::json

* Reduce logging level for continuous outputs.

* Added topics:
  * diagnostics (published to global /diagnostic topic),
  * camera_info,
  * TOF_INFO,
  * RGB_INFO,
  * extrinsics

* Added parameters:
  * log_level,
  * tf related parameters (see parameters doc for more details),
  * buffer_id_list,
  * diag_mode



1.0
===
1.0.4 (unreleased)
------------------
* Updating to underlying ifm3d API version 1.1.1
* Name change for TOF_INFO and RGB_INFO buffer.
* Added Error, AsynError and AsyncNotification callbacks.

1.0.3 (unreleased)
------------------
* Updating to underlying ifm3d API version 1.0.1
* Switching to using buffer_ids instead of schemas to determine data types.
* Introducing buffer_id_utils.hpp for buffer_id handling.

1.0.2
------------------
* Fixed tf chain in launchfiles

1.0.1
-----
* Removed dependencies to OpenCV and PCL and the Image module of the ifm3d library. Now using the StlImage module of ifm3d.
* The RGB image is published as a compressed JPEG image. To view it with RViz, you can use the image_transport republish node (see documentation).
* Update the launchfiles for compatibility with pyyaml 6.0 (backward compatibility maintained down to pyyaml 5.1).


1.0.0
--------

* Update the ROS node for the O3R camera platform
* add a 2D RGB data publisher
* add support for specifying the PCIC data communication TCP port
* add example yaml files and launch file for multi PCIC TCP port settings
* add service functions for setting each camera head to RUN / IDLE state
* update the dump and config service to O3R JSON structure
* removed unit vector publishing: please use the intrinsic camera calibration parameters instead
* removed the temperature publisher until diagnose chunk is available
* removed axis permutation for ROS node specific axis order
* removed xyz image publisher

known limitations
------------------
* DDS settings need to be set to cyclonedds - ROS 2 Galactic is the preferred ROS 2 distro atm.

0.3.0 (2020-01-31)
--------

* Publish camera temperature (Thanks @dustingooding of Houston Mechatronics)
* Tested on Eloquent

0.2.0 (2019-06-26)
--------

* Provided an implementation of the `Dump` service
* Provided an implementation of the `dump` command-line tool
* Provided an implementation of the `Config` service
* Provided an implementation of the `config` command-line tool

0.1.1 (2019-06-25)
--------

* Unit vectors are published on a "latched topic". Please see `qos.hpp` for
  the specifics of the QoS profile `ifm3d_ros2::LatchedQoS`.
* On-camera extrinsics in support of off-line point cloud computation are now
  exposed.
* Bootstrapped some unit tests
* Cleaned up some documentation.


0.1.0 (2019-06-06)
--------

* Initial (alpha) release
