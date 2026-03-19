# PDS topics

Topic names shown below are the resolved names when launching with the default namespace (`/ifm3d`) and node names (`pds`, `pds_vis`).
If you override the namespace and/or node names in your launch setup, the resolved topic names will change accordingly.

The PDS node publishes the full combined result on `~/pds_full_result` (resolved to `/ifm3d/pds/pds_full_result` with defaults).

## Published topics
| Name                             | Data type                           | Description |
| ------------------------------- | ------------------------------------- | ---------- |
| `/ifm3d/pds/pallet_detection`   | `ifm3d_ros2/msg/PalletDetectionArray` | Result of the `getPallet` PDS function. |
| `/ifm3d/pds/pds_full_result`    | `ifm3d_ros2/msg/PdsFullResult`        | Combined result of the PDS functions and the raw J SON; only one of the three result submessages is filled with valid data, indicated by the `last_command` string. |
| `/ifm3d/pds/rack_detection`     | `ifm3d_ros2/msg/RackDetection`        | Result of the `getRack` PDS function. |
| `/ifm3d/pds/volume_check`       | `ifm3d_ros2/msg/VolumeCheck`          | Result of the `volCheck` PDS function. |   
| `/ifm3d/pds_vis/pds_vis_marker` | `visualization_msgs/msg/MarkerArray`  | Markers to visualize results in RViz2. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Check out [the diagnostic documentation](../diagnostic.md) for more details. |

:::{note}
All the topics are published with QoS `ifm3d_ros2::LowLatencyQoS`.
:::

## Subscribed topics

The PDS Visualization node subscribes to `/ifm3d/pds/pds_full_result` from the PDS node.
