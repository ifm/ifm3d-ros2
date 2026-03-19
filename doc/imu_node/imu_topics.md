# IMU topics

Topic names shown below are the resolved names when launching with the default namespace (`/ifm3d`) and node name (`imu`).
If you override the namespace and/or node name in your launch setup, the resolved topic names will change accordingly.

## Published topics
| Name                     | Data type                             | Description |
| ------------------------ | ------------------------------------- | ----------- |
| `/ifm3d/imu/average_imu` | `sensor_msgs/msg/Imu`                 | Averages all IMU readings from a received buffer and publishes the result here. |
| `/ifm3d/imu/imu_burst`   | `ifm3d_ros2/msg/ImuBurst`             | All IMU readings retrieved from a buffer are published here as an array for further processing. |
| `/diagnostics`           | `diagnostic_msgs/msg/DiagnosticArray` | Check out [the diagnostic documentation](../diagnostic.md) for more details. |

:::{note}
`/ifm3d/imu/average_imu` topic is turned off by default. It can be activated by setting the `imu.publish_averaged_data` parameter to `true`

:::
:::{note} 
All the topics are published with QoS `ifm3d_ros2::LowLatencyQoS`. 
:::

## Subscribed topics
None