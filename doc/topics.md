# Topics

## Published Topics

| Name            | Data Type                    | Description                                                         |
| --------------- | ---------------------------- | ------------------------------------------------------------------- | 
| amplitude       | sensor_msgs/msg/Image        | The normalized amplitude image                                      |
| cloud           | sensor_msgs/msg/PointCloud2  | The point cloud data                                                |
| confidence      | sensor_msgs/msg/Image        | The confidence image                                                |
| distance        | sensor_msgs/msg/Image        | The radial distance image                                           |
| raw_amplitude   | sensor_msgs/msg/Image        | The raw amplitude image (currently not available for the O3R)       |
| rgb             | sensor_msgs/msg/Image        | The RGB 2D image of the 2D imager                                   |
| extrinsics      | ifm3d_ros2::msg::Extrinsics  | The extrinsic calibration of the camera (camera to world)           |
| INTRINSIC_CALIB | ifm3d_ros2::msg::Intrinsics  | The intrinsic calibration of the camera (optical system parameters) |
| INVERSE_INTRINSIC_CALIBRATION | ifm3d_ros2::msg::InverseIntrinsics | The inverse intrinsic calibration of the camera |
| camera_info     | sensor_msgs::msg::CameraInfo | The camera info topic containing the distortion model. This topic is published if the INTRINSIC_CALIB is part of the buffer list               |
| TOF_INFO        | ifm3d_ros2::msg::TOFInfo     | A topic gathering various information from the tof camera (see [TOFinfo.msg](../msg/TOFInfo.msg)) |
| RGB_INFO        | ifm3d_ros2::msg::RGBInfo     | A topic gathering various information from the rgb camera (see [RGBInfo.msg](../msg/RGBInfo.msg)) |
| diagnostics     | diagnostic_msgs::msg::DiagnosticArray | Diagnostic messages pulled from the device every second    |

:::{note}
All the topics are published with QoS `ifm3d_ros2::LowLatencyQoS`.
:::

## Subscribed Topics

None.
