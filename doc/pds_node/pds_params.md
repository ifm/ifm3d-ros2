# PDS node parameters

| Name             | Data Type | Default Value  | Description                                                                          |
| ---------------- | --------- | -------------- | ------------------------------------------------------------------------------------ |
| `~/app_instance` | String    | `"app0"`       | Name of the used PDS application instance.                                           |
| `~/config_file`  | String    | `""`           | Path to a JSON configuration file to be used when configuring the node.              |
| `~/ip`           | String    | "192.168.0.69" | The IP address of the OVP8xx platform.                                               |
| `~/pds.frame_id` | String    | "ifm_pds_link" | The name of the `frame_id` used for the published results and visualization markers. |
| `~/pcic_port`    | Integer   | 51011          | The TCP port the PCIC server for the active application is listening to. Can be read out in the JSON configuration at the `"/applications/instances/appX/data/PcicTCPPort"` key, or retrieved using the ifm3d API with `O3R->Port("appX").pcic_port`, where `"appX"` is the active application. |
