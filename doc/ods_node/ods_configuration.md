# Configuring ODS 

:::{note}
ifm's Obstacle Detection Solution (ODS) is supported in `ifm3d-ros2` version 1.2.0 and above.
:::

ODS is an application that runs on the O3R platform, that provides the ability to detect the position of obstacles within the camera field of view.
For more details on ODS, refer to [the ODS documentation on ifm3d.com](https://ifm3d.com/latest/ODS/index_ods.html).

There are multiple ways to configure ODS parameters, which are described in the following subsections.

## Using the Vision Assistant

We typically recommend to use the ifm Vision Assistant, which is ifm's official GUI for interfacing with all the vision products.

To get started with ODS in the Vision Assistant, refer to [the ODS getting started documentation](https://ifm3d.com/latest/ODS/getting_started.html).
After following these instructions, you will have set up and properly configured an ODS application, and you will be ready to launch the ODS node.

## Using the `Config` service

It is also possible to configure an application directly using the `ifm3d-ros2` service `Config`.
We recommend to use this method when re-configuring the application at runtime, for example to change the active ports.

In this case, you can use the following command.
This will switch the `activePorts` parameter of the application `app0` to `["port3"]`.

```bash
$ ros2 service call /ifm3d/ods/Config ifm3d_ros2/srv/Config "{json: '{\"applications\":{\"instances\":{\"app0\":{\"configuration\":{\"activePorts\":[\"port3\"]}}}}}'}"
requester: making request: ifm3d_ros2.srv.Config_Request(json='{"applications":{"instances":{"app0":{"configuration":{"activePorts":["port3"]}}}}}')

response:
ifm3d_ros2.srv.Config_Response(status=0, msg='OK')
```

## Using the `config_file` parameter

Additionally, it is possible to configure an application (or any other aspect of the system) using a configuration file, provided through the `config_file` parameter in the `ods_default_parameters.yaml` file.
This configuration file will be used to set the configuration in the `on_configure` transition of the node.
To re-configure a node using a new configuration file, the `on_configure` transition has to be triggered again.

### Example Configuration

The default ODS parameters include a sample configuration:

```yaml
/ifm3d/ods:
  ros__parameters:
    config_file: "config/o3r_configs/o3r_ods.json"
    # ... other parameters
```

The referenced JSON file (`config/o3r_configs/o3r_ods.json`) contains:

```json
{
  "ports": {
    "port2": {
      "processing": {
        "extrinsicHeadToUser": {
          "rotX": 0,
          "rotY": 1.57,
          "rotZ": -1.57,
          "transX": 0,
          "transY": 0,
          "transZ": 0.35
        }
      }
    }
  },
  "applications": {
    "instances": {
      "app0": {
        "class": "ods",
        "name": "Standard ODS app",
        "ports": ["port2", "port6"],
        "state": "CONF",
        "configuration": {
          "vo": {
            "voPorts": ["port2"]
          },
          "activePorts": ["port2"],
          "zones": {
            "zoneCoordinates": [
              [
                [0, 1],
                [1, 1],
                [1, -1],
                [0, -1]
              ],
              [
                [1, 1],
                [2, 1],
                [2, -1],
                [1, -1]
              ],
              [
                [2, 1],
                [3, 1],
                [3, -1],
                [2, -1]
              ]
            ]
          }
        }
      }
    }
  }
}
```

### Path Resolution

- **Relative paths**: If the `config_file` parameter contains a relative path (doesn't start with `/`), it will be resolved relative to the `ifm3d_ros2` package share directory.
- **Absolute paths**: If the path starts with `/`, it will be used as-is.
- **Empty parameter**: If `config_file` is empty (`""`), no configuration file will be loaded.

### Re-configuration

To apply a new configuration file:

1. Update the `config_file` parameter:

   ```bash
   ros2 param set /ifm3d/ods config_file "path/to/new/config.json"
   ```

2. Trigger reconfiguration:

   ```bash
   ros2 lifecycle set /ifm3d/ods configure
   ros2 lifecycle set /ifm3d/ods activate
   ```

:::{note}
The configuration file is only loaded during the `configure` transition. Changing the `config_file` parameter at runtime requires re-triggering the `configure` transition to take effect.
:::