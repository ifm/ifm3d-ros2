# Configuring ODS 

:::{note}
ifm's Obstacle Detection Solution (ODS) is supported in `ifm3d-ros2` version 1.2.0 and above.
:::

ODS is an application that runs on the O3R platform, that provides the ability to detect the position of obstacles within the camera field of view.
For more details on ODS, refer to [the ODS documentation on ifm3d.com](https://ifm3d.com/latest/ODS/index_ods.html). 

## Using the Vision Assistant

There are multiple ways to create and configure an ODS application.
We typically recommend to use the ifm Vision Assistant, which is ifm's official GUI for interfacing with all the vision products.

To get started with ODS in the Vision Assistant, refer to [the ODS getting started documentation](https://ifm3d.com/latest/ODS/getting_started.html).
After following these instructions, you will have set up and properly configured an ODS application, and you will be ready to launch the ODS node.

## Using the `Config` service

It is also possible to configure an application directly using the `ifm3d-ros2` service `Config`.
We recommend to use this method when re-configuring the application at runtime, for example to change the active ports. 

In this case, you can use the following command. 
This will switch the `activePort` parameter of the application `app0` to `"port3"`.
```bash
$ ros2 service call /ifm3d/ods/Config ifm3d_ros2/srv/Config "{json: '{\"applications\":{\"instances\":{\"app0\":{\"configuration\":{\"activePorts\":[\"port3\"]}}}}}'}"
requester: making request: ifm3d_ros2.srv.Config_Request(json='{"applications":{"instances":{"app0":{"configuration":{"activePorts":["port3"]}}}}}')

response:
ifm3d_ros2.srv.Config_Response(status=0, msg='OK')

```

## Using the `config_file` parameter

Additionally, it is possible to configure an application (or any other aspect of the system) using a configuration file, provided through the `config_file` parameter in the `ods_default_parameters.yaml` file.
This configuration file will be used to set the configuration in the `on_configuration` transition of the node.
To re-configure a node using a new configuration file, the `on_configuration` transition has to be triggered again.

:::{note}
Ensure to provide the absolute path for the `config_file` parameter in the `ods_default_parameters.yaml` file. If a relative path is used, make sure to run the launch file from the directory corresponding to the relative path.
:::