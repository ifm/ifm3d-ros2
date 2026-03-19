# Configuring the IMU Node

:::{note}
The IMU node is supported in `ifm3d-ros2` version 1.3.0 and above.
:::

## Using the `config_file` parameter

The IMU node should be configured using a configuration file, provided through the `config_file` parameter.
This configuration file will be used to set the configuration in the `on_configuration` transition of the node.
To re-configure a node using a new configuration file, the `on_configuration` transition has to be triggered again.