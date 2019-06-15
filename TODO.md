TODO
====

* Expose the intrinsic camera calibration via CameraInfo
* Expose the on-camera extrinsic calibration
* Investigate performance through the ROS middleware, tune QoS for our camera
* Unit tests (investigate `launch_test` for node integration tests)
* Write an inproc demo node
* Port ROS Services from our ROS 1 Node (e.g., dump, config, etc.). For now,
  users should use the underlying `ifm3d dump ...` and `ifm3d config ...`
  command line utilities.
* Investigate using/need for ImageTransport (compressed image topics?) like our
  ROS 1 node.
* Create an installable `snap` for `snapd` supported systems
