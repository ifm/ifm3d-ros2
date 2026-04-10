# PDS node parameters

| Name                          | Data Type | Default Value    | Description                                                                          |
| ----------------------------- | --------- | ---------------- | ------------------------------------------------------------------------------------ |
| `~/app_instance`              | String    | `"app0"`         | Name of the used PDS application instance.                                           |
| `~/config_file`               | String    | `""`             | Path to a JSON configuration file to be used when configuring the node. Path to a JSON configuration file to be used when configuring the node. Absolute path to a file, or relative path from a package's share directory possible, if prefixed by `package://<my_package>/path/to/config.json`. Relative path without `package://` prefix is assumed to be relative to `ifm3d_ros2` share directory.  Parameter is ignored if `""` is provided. |
| `~/ip`                        | String    | `192.168.0.69`   | The IP address of the OVP8xx platform.                                               |
| `~/pds.frame_id`              | String    | `"ifm_pds_link"` | The name of the `frame_id` used for the published results and visualization markers. |
| `~/pcic_port`                 | Integer   | 51011            | The TCP port the PCIC server for the active application is listening to. Can be read out in the JSON configuration at the `"/applications/instances/appX/data/PcicTCPPort"` key, or retrieved using the ifm3d API with `O3R->Port("appX").pcic_port`, where `"appX"` is the active application. |
| `~/publish_uncompressed`      | bool      | false            | If true, most sensor messages will be published best_effort instead of reliable. |
| `~/use_timestamp_from_device` | bool      | true             | Uses timestamp from VPU for messages if true; uses ROS time if false. |
| `~/xmlrpc_port`               | Integer   | 80               | The TCP port the XMLRPC server for the active application is listening to. Typically, the default value can be used. |