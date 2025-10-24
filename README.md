# ifm3d-ros2 overview

*This documentation is formatted to be read on [www.ros2.ifm3d.com](https://ros2.ifm3d.com/latest/).*

:::{note}
This release is intended to be used with the O3R camera platform ONLY. For other ifm cameras (e.g. O3D3xx and O3X1xx) please see the tagged releases 0.3.0 and 0.7.0 respectively.
:::

`ifm3d-ros2` is a wrapper around [ifm3d](https://github.com/ifm/ifm3d) enabling the usage of ifm O3R camera platform from within [ROS 2](https://index.ros.org/doc/ros2/) software systems.

As of version 1.2.0, `ifm3d_ros2` also supports the Obstacle Detection Solution (ODS). Refer to [the ODS documentation](https://ifm3d.com/latest/ODS/index_ods.html) for more details on this application.

![rviz](doc/camera_node/figures/O3R_merged_point_cloud.png)

## Software Compatibility Matrix


| `ifm3d_ros2` version | ifm3d version   | O3R firmware version | ROS 2 distribution | Comment                                                                      |
| -------------------- | --------------- | -------------------- | ------------------ | ---------------------------------------------------------------------------- |
| 1.3.0                | 1.6.12          | 1.20.29, 1.21.6      | Jazzy, Humble      | Enhanced diagnostics, ODS polar grid, uncompressed images, additional image types |
| 1.2.0                | 1.4.3 and 1.5.3 | 1.4.30 and 1.10.13   | Jazzy, Humble      | Added support for ODS applications                                           |

> Note: The version numbers listed above for the ifm3d API or the O3R firmware versions are the ones explicitly tested. Any other version might work but is not officially supported.

## Changelog

For a complete list of changes, refer to [the changelog](./CHANGELOG.rst).

### Known Issues

- **Cross-compilation limitations**: The `build_container.sh` script may encounter issues when cross-compiling from AMD64 to ARM architectures. For ARM-based deployments, we recommend building directly on the target ARM platform or using ARM-compatible build environments. *A fix for this issue is being developed and will be included in the next patch release.*

### Deprecated ifm3d-ros2 versions

The following versions are deprecated and no longer supported.

| `ifm3d_ros2` version | ifm3d version | ROS 2 distribution |
| -------------------- | ------------- | ------------------ |
| 1.1.0 DEPRECATED     | 1.0.14        | Humble             |
| 1.0.1 DEPRECATED     | 0.93.0        | Galactic, Foxy     |
| 1.0.0 DEPRECATED     | 0.92.0        | Galactic           |
| 0.3.0 DEPRECATED     | 0.17.0        | Dashing, Eloquent  |
| 0.2.0 DEPRECATED     | 0.12.0        | Dashing            |
| 0.1.1 DEPRECATED     | 0.12.0        | Dashing            |
| 0.1.0 DEPRECATED     | 0.12.0        | Dashing            |

## LICENSE

Please see the file called [LICENSE](LICENSE).
