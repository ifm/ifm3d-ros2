TODO
====

* Expose the intrinsic camera calibration via CameraInfo
* Expose the on-camera extrinsic calibration
* Investigate performance through the ROS middleware, tune QoS for our camera
* Write an inproc demo node
  * Enable intra-process comms
  * pub/sub w/ std::unique_ptr
* Investigate using/need for ImageTransport (compressed image topics?) like our
  ROS 1 node.
* Create an installable `snap` for `snapd` supported systems
