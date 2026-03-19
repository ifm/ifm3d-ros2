# Configuring PDS 

:::{note}
ifm's Pick and Drop System (PDS) is supported in `ifm3d-ros2` version 1.3.0 and above.
:::

PDS (Pick and Drop System) by ifm is a software solution built on the O3R ecosystem that detects the pose of pallets and racks and verifies volume occupancy, enabling AGVs and AMRs to autonomously perform  pick-and-drop operations from point A to point B.
For more details on PDS, refer to [the PDS documentation on ifm3d.com](https://ifm3d.com/latest/PDS/index_pds.html). 

## Camera head calibration

PDS requires the camera head to be extrinsically calibrated before use.
The default configuration file (`config/o3r_configs/o3r_pds.json`) does **not** include camera calibration — it only sets up the PDS application and its port.

Calibration can be provided in one of the following ways:
- **Using the ifm Vision Assistant**: calibrate the camera head before launching the node. See [the iVA documentation](https://ifm3d.com/latest/SoftwareInterfaces/iVA/iVA_with_linux.html) and [PDS calibration guide](https://ifm3d.com/latest/PDS/Calibration/pds_calibration.html).
- **Including calibration in the config file**: add the `extrinsicHeadToUser` parameters for the relevant port in the JSON configuration (similar to the ODS example in `config/o3r_configs/o3r_ods_presets_calibration.json`).

## Using the Vision Assistant

There are multiple ways to create and configure a PDS application.
We typically recommend to use the ifm Vision Assistant, which is ifm's official GUI for interfacing with all the vision products.

To get started with PDS in the Vision Assistant, refer to [the PDS getting started documentation](https://ifm3d.com/latest/PDS/GettingStarted/index_getting_started.html).
After following these instructions, you will have set up and properly configured an PDS application, and you will be ready to launch the PDS node.

## Using the `Config` service

It is also possible to configure an application directly using the `ifm3d-ros2` service `Config`.
We recommend to use this method when re-configuring the application at runtime.

## Using the `config_file` parameter

Additionally, it is possible to configure an application (or any other aspect of the system) using a configuration file, provided through the `config_file` parameter.
This configuration file will be used to set the configuration in the `on_configuration` transition of the node.
To re-configure a node using a new configuration file, the `on_configuration` transition has to be triggered again.