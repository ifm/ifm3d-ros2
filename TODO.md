TODO
====

* It is not clear how to mimic ROS 1 "latching" behavior. To that end, we are
  currently publishing the Unit Vectors on each loop iteration. To be clear, we
  are not streaming the unit vectors continuously from the camera, we fetch
  them once and then cache them on the ROS node. However, we currently SPAM the
  ROS/DDS network with Unit Vector data. We need to investigate how to mimic a
  latched topic in ROS2 or perhaps move the Unit Vectors into a service
  call. The service call approach could be made backward compatibile once we
  have a latched topic -- i.e., maintain both the service call and latched
  topic. In either case, the same cached data are returned.
* Expose the intrinsic camera calibration via CameraInfo
* Expose the on-camera extrinsic calibration
* Investigate performance through the ROS middleware, tune QoS for our camera
* Unit tests
* Write an inproc demo node
* Port ROS Services from our ROS 1 Node (e.g., dump, config, etc.). For now,
  users should use the underlying `ifm3d dump ...` and `ifm3d config ...`
  command line utilities.
* Investigate using/need for ImageTransport (compressed image topics?) like our
  ROS 1 node.
* Create an installable `snap` for `snapd` supported systems
