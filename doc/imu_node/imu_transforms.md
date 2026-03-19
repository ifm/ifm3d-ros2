# IMU transforms

IMU data is published with reference to the `imu_frame_name` (defaults to `imu_link`), which lies inside the VPU. 
Additional used frames are the `base_frame_name` (defaults to `imu_base_link`) which should be named identical for all nodes related to the same VPU, and the `mounting_frame_name` (defaults to `vpu_mounting_link`) which represents the mounting position of the VPU.

Publication of base to mounting transforms and mounting to sensor transforms as well as the three frame names can be configured, see [IMU Parameters](./imu_params.md) for details.

Publishing the tf from the robot's `"base_link"` to the `base_frame_name` (defaults to `imu_base_link`) should be done by the user.

## Additional information

For more information about the IMU built into the VPU, refer to the [ifm IMU documentation](https://ifm3d.com/latest/Technology/VPU/IMU/imu.html).

For more information about the IMU reference point and its location inside the VPU, refer to the [OVP calibration documentation](https://ifm3d.com/latest/CalibrationRoutines/OVPCalibration/README.html).
