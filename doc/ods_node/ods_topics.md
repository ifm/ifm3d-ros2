# ODS topics

## Published topics
| Name                               | Data type                    | Description                                                                                                                      |
| ---------------------------------- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `/ifm3d/ods/ods_info`              | `ifm3d_ros2/msg/Zones`       | Provides the id of the currently active zone set, as well as whether each zone is occupied (`=1`) or not (`=0`).                 |
| `/ifm3d/ods/ods_occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | The occupancy grid where each cell represent the probability of an obstacle being there. |
| `/ifm3d/ods/ods_polar_occupancy_grid` | `sensor_msgs/msg/LaserScan` | 360° polar distance measurements from the ODS system in LaserScan format. Provides LiDAR-like visualization with 675 angular measurements starting from 0° and increasing counter-clockwise. Distance values are in meters, with `infinity` indicating no detection. |
| `/ifm3d/ods/ods_occupancy_grid_costmap` | `nav2_msgs/msg/Costmap` | The occupancy grid formatted as a costmap for Navigation2 integration. Publication of this topic is optional and is not directly visualizable in RViz (use the occupancy grid topic for visualization). |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Check out [the diagnostic documentation](../diagnostic.md) for more details. |

:::{note} 
All the topics are published with QoS `ifm3d_ros2::LowLatencyQoS`. 
:::

## Subscribed topics
None