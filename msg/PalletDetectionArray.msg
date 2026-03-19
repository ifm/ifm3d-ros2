# PalletDetectionArray.msg

std_msgs/Header header

ifm3d_ros2/PalletDetection[] detections

# Volume used in the algorithm to approximate the position of the front face of the pallet.
ifm3d_ros2/BoundingBoxStamped depth_estimation_voi

# Volume where the detection takes place. The pixels outside this area are discarded.
ifm3d_ros2/BoundingBoxStamped projection_voi