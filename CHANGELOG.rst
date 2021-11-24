^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ifm3d-ros2
^^^^^^^^^^^^^^^^^^^^^^^^^

1.0
===

1.0.0 (unreleased)
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
