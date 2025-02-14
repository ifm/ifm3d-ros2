# Launch the node

Launch the camera node (assuming you are in `~/colcon_ws/`):
```
$ source install/setup.bash
$ ros2 launch ifm3d_ros2 camera.launch.py
```

This will launch the `/ifm3d/camera/` node with default arguments, using the default YAML configuration file `camera_default_parameters.yaml`. Depending on the specified `pcic_port`, the node will initialize either a 3D camera (if the port corresponds to a 3D camera) or a 2D camera.

The respective node information should look like this: 

```
$ ros2 node info /ifm3d/camera

/ifm3d/camera
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /ifm3d/camera/INTRINSIC_CALIB: ifm3d_ros2/msg/Intrinsics
    /ifm3d/camera/INVERSE_INTRINSIC_CALIBRATION: ifm3d_ros2/msg/InverseIntrinsics
    /ifm3d/camera/RGB_INFO: ifm3d_ros2/msg/RGBInfo
    /ifm3d/camera/TOF_INFO: ifm3d_ros2/msg/TOFInfo
    /ifm3d/camera/amplitude: sensor_msgs/msg/Image
    /ifm3d/camera/camera_info: sensor_msgs/msg/CameraInfo
    /ifm3d/camera/cloud: sensor_msgs/msg/PointCloud2
    /ifm3d/camera/confidence: sensor_msgs/msg/Image
    /ifm3d/camera/distance: sensor_msgs/msg/Image
    /ifm3d/camera/extrinsics: ifm3d_ros2/msg/Extrinsics
    /ifm3d/camera/rgb: sensor_msgs/msg/CompressedImage
    /ifm3d/camera/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /tf_static: tf2_msgs/msg/TFMessage
  Service Servers:
    /ifm3d/camera/Config: ifm3d_ros2/srv/Config
    /ifm3d/camera/Dump: ifm3d_ros2/srv/Dump
    /ifm3d/camera/Softoff: ifm3d_ros2/srv/Softoff
    /ifm3d/camera/Softon: ifm3d_ros2/srv/Softon
    /ifm3d/camera/change_state: lifecycle_msgs/srv/ChangeState
    /ifm3d/camera/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ifm3d/camera/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /ifm3d/camera/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/camera/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ifm3d/camera/get_parameters: rcl_interfaces/srv/GetParameters
    /ifm3d/camera/get_state: lifecycle_msgs/srv/GetState
    /ifm3d/camera/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/camera/list_parameters: rcl_interfaces/srv/ListParameters
    /ifm3d/camera/set_parameters: rcl_interfaces/srv/SetParameters
    /ifm3d/camera/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```

:::{note}
We also provide a helper launch file to start multiple camera nodes. See the documentation [here](multi_head.md).
:::

To visualize the data with RViz, set the `visualization` argument of the launch script to `true`:
```
$ ros2 launch ifm3d_ros2 camera.launch.py visualization:=true
```

# Configuration

The camera parameters can be configured, wheter extrinsic calibration or acquisition parameters to filtering parameters.
There are multiple ways to do that that we are describin in the following undersections

## Using the Vision Assistant

We typically recommend to use the ifm Vision Assistant, which is ifm's official GUI for interfacing with all the vision products.

To get started with Vision Assistant, refer to [the Introduction to the ifmVisionAssistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/introduction_and_installation.html).
And then to change parameters, refer to [changin parameters using ifm Vision Assistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/changing_parameters.html) After following these instructions, you will have set up and properly configured camera ports parameters.

## Using the `Config` service

It is also possible to configure an application directly using the `ifm3d-ros2` service `Config`.
We recommend to use this method when re-configuring the application at runtime, for example to change the active ports.

In this case, you can use the following command.
This will switch the `activePort` parameter of the application `app0` to `"port3"`.
```bash
$ ros2 service call /ifm3d/camera/Config ifm3d_ros2/srv/Config "{json: '{\"ports\":{\"port2\":{\"processing\":{\"extrinsicHeadToUser\":{\""transZ"\":1}}}}}'}"
requester: making request: ifm3d_ros2.srv.Config_Request(json='{"ports":{"port2":{"processing":{"extrinsicHeadToUser":{"transZ":1}}}}}')

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

![rviz1](./figures/O3R_merged_point_cloud.png)

Congratulations! You can now have complete control over the O3R perception platform from inside ROS2.
