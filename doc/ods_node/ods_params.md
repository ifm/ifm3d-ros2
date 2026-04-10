# ODS parameters

| Name                                 | Data Type  | Default Value   s  | Description |
| ------------------------------------ | ---------- | ----------------- | ----------- |
| `~/config_file`                      | string     | `""`              | Path to a JSON configuration file to be used when configuring the node. Absolute path to a file, or relative path from a package's share directory possible, if prefixed by `package://<my_package>/path/to/config.json`. Relative path without `package://` prefix is assumed to be relative to `ifm3d_ros2` share directory.  Parameter is ignored if `""` is provided. |
| `~/ip`                               | String     | `"192.168.0.69"`  | The IP address of the OVP8xx platform. |
| `~/ods.frame_id`                     | String     | `"ifm_base_link"` | The name of the `frame_id` used for the occupancy grid and the zones topics headers. |
| `~/ods.publish_occupancy_grid`       | bool       | true              | Set module to publish `nav_msgs/OccupancyGrid`. |
| `~/ods.publish_polar_occupancy_grid` | bool       | true              | Set module to publish `sensor_msgs/LaserScan` from polar distance data. This provides LiDAR-like visualization of the 360° polar distance measurements from the ODS system. |
| `~/ods.publish_costmap`              | bool       | false             | Set module to publish `nav2_msgs/Costmap`. This format is used by Navigation2 for path planning and is not directly visualizable in RViz (use the occupancy grid for visualization). |
| `~/pcic_port`                        | Integer    | 51010             | The TCP port the PCIC server for the active application is listening to. Can be read out in the JSON configuration at the `"/applications/instances/appX/data/PcicTCPPort"` key, or retrieved using the ifm3d API with `O3R->Port("appX").pcic_port`, where `"appX"` is the active application. |
| `~/publish_uncompressed`             | bool       | false             | If true, most sensor messages will be published best_effort instead of reliable. |
| `~/use_timestamp_from_device`        | bool       | true              | Uses timestamp from VPU for messages if true; uses ROS time if false. |
| `~/xmlrpc_port`                      | Integer    | 80                | The TCP port the XMLRPC server for the active application is listening to. Typically, the default value can be used. |
