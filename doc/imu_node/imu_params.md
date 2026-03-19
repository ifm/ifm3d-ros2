# IMU parameters

| Name                           | Data Type | Default Value                | Description |
| ------------------------------ | --------- | ---------------------------- | ----------- |
| `~/imu.publish_averaged_data`  | bool      | false                        | Collected IMU samples are averaged and published as sensor_msgs/Imu message. |
| `~/imu.publish_bulk_data`      | bool      | false                        | Collected IMU samples are published in bulk for further processing by the user as ifm3d_ros2/ImuBurst message. |
| `~/ip`                         | String    | "192.168.0.69"               | The IP address of the OVP8xx platform. |
| `~/pcic_port`                  | Integer   | 50016                        | The TCP port the PCIC server for the active application is listening to. Can be read out in the JSON configuration at the `"/applications/instances/appX/data/PcicTCPPort"` key, or retrieved using the ifm3d API with `O3R->Port("appX").pcic_port`, where `"appX"` is the active application. |
| `~/tf.base_frame_name`         | String    | "ifm_base_link"              | Name for ifm reference frame. |
| `~/tf.imu_frame_name`          | String    | "imu_link"                   | Name for the sensor frame, used for tf publication and as frame_id in messages. |
| `~/tf.mounting_frame_name`     | String    | "vpu_mounting_link"          | Name for the mounting point frame. |
| `~/tf.publish_base_to_mounting`| bool      | true                         | Whether the transform from the ifm base link to the VPU mounting point should be published. |
| `~/tf.publish_mounting_to_imu` | bool      | true                         | Whether the transform from the VPU mounting point link to the imu sensor frame should be published. |
| `~/xmlrpc_port`                | Integer   | `ifm3d::DEFAULT_XMLRPC_PORT` | The TCP port the XMLRPC server for the active application is listening to. Typically, the default value can be used. |