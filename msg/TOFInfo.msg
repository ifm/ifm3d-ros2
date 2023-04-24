std_msgs/Header header

uint32 measurement_block_index
float32 measurement_range_min
float32 measurement_range_max
uint32 version
float32 distance_resolution
float32 amplitude_resolution
float32[3] amp_normalization_factors

ifm3d_ros2/Extrinsics extrinsics
ifm3d_ros2/Intrinsics intrinsics
ifm3d_ros2/InverseIntrinsics inverse_intrinsics

uint64[3] exposure_timestamps_ns
uint32[3] exposure_times_s
float32 illu_temperature
string<=32 mode
string<=32 imager