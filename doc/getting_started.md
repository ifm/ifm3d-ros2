# Getting Started

This section describes the basic hardware setup and configuration needed to test different functionalities of this driver.

## Hardware setup and installation

You can follow the guide provided at [ifm3d.com](https://ifm3d.com/latest/GettingStarted/Unboxing/hw_unboxing.html) on how to wire and setup the O3R system.

To use the `camera.launch.py` or `pds.launch.py` examples, at least one camera head is needed.
To use the `ods.launch.py`, two camera heads are needed.
`imu.launch.py` does not require a connected camera.

Follow the instructions provided in [the building section](building.md) to install and/or build the required ifm3d SDK and ROS 2 packages.

## Default port configuration

The provided launch files and configuration files are designed to work out of the box with the following port assignment:

| Node / Launch file | Port(s) | PCIC port(s) | Description |
|--------------------|---------|--------------|-------------|
| `camera.launch.py` (3D) | port4 | 50014 | 3D camera head |
| `camera.launch.py` (2D) | port0 | 50010 | 2D camera head (requires changing `pcic_port` in config) |
| `ods.launch.py` | port2, port3 | 51010 (app0) | Two 3D camera heads for ODS |
| `pds.launch.py` | port5 | 51011 (app1) | One 3D camera head for PDS |
| `imu.launch.py` | port6 | 50016 | IMU (fixed, cannot be changed) |

> **Note:** If your physical camera heads are not connected to the ports listed above, you must update the corresponding configuration files so that the launch files work correctly.
> In particular:
> - For `camera.launch.py`: update `pcic_port` in `config/camera_default_parameters.yaml`.
> - For `ods.launch.py`: update the port references in `config/o3r_configs/o3r_ods.json` and, if needed, the `pcic_port` in `config/ods_default_parameters.yaml`.
> - For `pds.launch.py`: update the port references in `config/o3r_configs/o3r_pds.json` and, if needed, the `pcic_port` in `config/pds_default_parameters.yaml`.
>
> The `pcic_port` values map to camera ports as follows: `50010`–`50015` correspond to port0–port5, `50016` is the IMU port. Application ports start at `51010`.

## Package structure

The `doc` directory contains more in-depth documentation on nodes,deployment, visualization, and diagnostics.

In the `launch` directory, exemplary launch scripts are provided.
They rely on the example configurations provided in `config` and setup to be usable with the hardware setup described above.

`action`, `msg`, and `srv` contain interface definitions for custom communication.

`include` contains header files, `src` contains the source code, `test` contains the integration tests. 

## Running nodes

**Connection configuration:**
All config files contain the `ip` and `pcic_port` parameter to configure connection to the O3R system.
For these tutorials, the IP is set to `192.168.0.69` which is the factory default IP used for all OVP devices.
Make sure your system is connected to the `ETH0` port of the OVP and can ping the device.
See the [Default port configuration](#default-port-configuration) section above for the expected port-to-`pcic_port` mapping.

**Lifecycle:**
All nodes in the repository are ROS 2 lifecycle nodes.
Some node parameters can be changed while the node is active, others require re-transitioning through to configure state.

### camera.launch.py

Running `ros2 launch ifm3d_ros2 camera.launch.py` uses the `config/camera_default_parameters.yaml` to start a `camera_node` which publishes data received on port4 (3D data).
To stream 2D data instead, change `pcic_port` in the configuration file to `50010` (port0).

The default configuration does **not** include camera head calibration. To include extrinsic calibration in the config, see `config/o3r_configs/o3r_one_head_calibration_only.json` for an example. Alternatively, calibrate the camera beforehand using the [ifmVisionAssistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/iVA_with_linux.html).

You may run `ros2 launch ifm3d_ros2 camera.launch.py visualization:=true` to open a RViz2 window to visualize the published data.

For detailed information, see [here](doc/camera_node/index_camera_node.md).

### imu.launch.py

Running `ros2 launch ifm3d_ros2 imu.launch.py` uses the `config/imu_default_parameters.yaml` to start an `imu_node` which publishes IMU data collected in the OVP on port6.
This port is fixed and cannot be changed by the user.

IMU readings are measured at a higher rate than they can be published.
By default, they are published in bulk on the `~/imu_burst` topic.
To publish singular messages containing averaged data, set the `imu.publish_averaged_data` parameter to true.

For detailed information, see [here](doc/imu_node/index_imu_node.md).

### ods.launch.py

Running ODS requires the cameras to be extrinsically calibrated.
The default configuration file (`config/o3r_configs/o3r_ods.json`) does **not** include camera head calibration.
Calibration can be performed beforehand using the [ifmVisionAssistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/iVA_with_linux.html), or included directly in the JSON configuration file.
See `config/o3r_configs/o3r_ods_presets_calibration.json` for an example that combines ODS application setup with camera head calibration.

Running `ros2 launch ifm3d_ros2 ods.launch.py` uses the `config/ods_default_parameters.yaml` to start an `ods_node` which publishes ODS application data.
By default, ODS is configured to use two 3D camera heads on port2 and port3.
You may run `ros2 launch ifm3d_ros2 ods.launch.py visualization:=true` to open a RViz2 window to visualize the published occupancy grid.

For detailed information, see [here](doc/ods_node/index_ods_node.md).

### pds.launch.py

Running PDS requires the camera to be extrinsically calibrated.
The default configuration file (`config/o3r_configs/o3r_pds.json`) does **not** include camera head calibration.
Calibration can be performed beforehand using the [ifmVisionAssistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/iVA_with_linux.html), or included directly in the JSON configuration file (similar to the ODS example in `config/o3r_configs/o3r_ods_presets_calibration.json`).
More information on PDS calibration can be found [here](https://ifm3d.com/latest/PDS/Calibration/pds_calibration.html).

Running `ros2 launch ifm3d_ros2 pds.launch.py` uses the `config/pds_default_parameters.yaml` to start a `pds_node` which uses a 3D camera head on port5.

For detailed information, see [here](doc/pds_node/index_pds_node.md).