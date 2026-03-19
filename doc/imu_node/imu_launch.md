# Launch the IMU node

To launch the IMU node, you can use the provided launch file `imu.launch.py`:
```bash
$ ros2 launch ifm3d_ros2 imu.launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2025-11-25-18-02-48-851732-ids-ackerman-890653
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [imu_standalone-1]: process details: cmd='/home/user/checkout/ifm/colcon_ws/install/ifm3d_ros2/lib/ifm3d_ros2/imu_standalone --ros-args --log-level info --ros-args -r __node:=imu -r __ns:=/ifm3d --params-file /home/user/checkout/ifm/colcon_ws/install/ifm3d_ros2/share/ifm3d_ros2/config/imu_default_parameters.yaml', cwd='None', custom_env?=True
[INFO] [imu_standalone-1]: process started with pid [890664]
[imu_standalone-1] [INFO] namespace: /ifm3d  (ifm3d.imu ImuNode:45)
[imu_standalone-1] [INFO] node name: imu  (ifm3d.imu ImuNode:46)
[imu_standalone-1] [INFO] middleware: rmw_fastrtps_cpp  (ifm3d.imu ImuNode:47)
[imu_standalone-1] [INFO] Declaring parameters...  (ifm3d.imu ImuNode:52)
[imu_standalone-1] [INFO] After the parameters declaration  (ifm3d.imu ImuNode:54)
[imu_standalone-1] [INFO] node created, waiting for `configure()`...  (ifm3d.imu ImuNode:58)
[imu_standalone-1] [INFO] on_configure(): unconfigured -> configuring  (ifm3d.imu on_configure:68)
[imu_standalone-1] [INFO] Parsing parameters...  (ifm3d.imu on_configure:74)
[imu_standalone-1] [INFO] Config file:   (ifm3d.imu parse_params:356)
[imu_standalone-1] [INFO] ip: 192.168.0.69  (ifm3d.imu parse_params:359)
[imu_standalone-1] [INFO] pcic_port: 50016  (ifm3d.imu parse_params:362)
[imu_standalone-1] [INFO] xmlrpc_port: 80  (ifm3d.imu parse_params:365)
[imu_standalone-1] [INFO] Parameters parsed.  (ifm3d.imu on_configure:76)
[imu_standalone-1] [INFO] Adding callbacks to handle parameter changes at runtime...  (ifm3d.imu on_configure:81)
[imu_standalone-1] [INFO] Callbacks set.  (ifm3d.imu on_configure:83)
[imu_standalone-1] [INFO] Initializing Device  (ifm3d.imu on_configure:94)
[imu_standalone-1] [INFO] Initializing FrameGrabber for data  (ifm3d.imu on_configure:96)
[imu_standalone-1] [INFO] Creating ImuModule...  (ifm3d.imu on_configure:127)
[imu_standalone-1] [INFO] FunctionModule contructor called.  (ifm3d.imu FunctionModule:13)
[imu_standalone-1] [INFO] ImuModule contructor called.  (ifm3d.imu ImuModule:21)
[imu_standalone-1] [INFO] ImuModule created.  (ifm3d.imu on_configure:129)
[imu_standalone-1] [INFO] Creating DiagModule...  (ifm3d.imu on_configure:133)
[imu_standalone-1] [INFO] FunctionModule contructor called.  (ifm3d.imu FunctionModule:13)
[imu_standalone-1] [INFO] hardware_id: /ifm3d/diag_module  (ifm3d.imu DiagModule:25)
[imu_standalone-1] [INFO] DiagModule created.  (ifm3d.imu on_configure:135)
[imu_standalone-1] [INFO] ImuModule: on_configure called  (ifm3d.imu on_configure:253)
[imu_standalone-1] [INFO] tf.base_frame_name: ifm_base_link  (ifm3d.imu parse_parameters:176)
[imu_standalone-1] [INFO] tf.mounting_frame_name: vpu_mounting_link  (ifm3d.imu parse_parameters:180)
[imu_standalone-1] [INFO] tf.imu_frame_name: imu_link  (ifm3d.imu parse_parameters:184)
[imu_standalone-1] [INFO] tf.publish_base_to_mounting: true  (ifm3d.imu parse_parameters:188)
[imu_standalone-1] [INFO] tf.publish_mounting_to_imu: true  (ifm3d.imu parse_parameters:192)
[imu_standalone-1] [INFO] Parameter imu.publish_averaged_data set to 'false'  (ifm3d.imu on_configure:259)
[imu_standalone-1] [INFO] Parameter imu.publish_bulk_data set to 'true'  (ifm3d.imu on_configure:263)
[imu_standalone-1] [INFO] Creating BaseServices...  (ifm3d.imu on_configure:156)
[imu_standalone-1] [INFO] BaseServices constructor called.  (ifm3d.imu BaseServices:19)
[imu_standalone-1] [INFO] Services created;  (ifm3d.imu BaseServices:44)
[imu_standalone-1] [INFO] BaseServices created.  (ifm3d.imu on_configure:159)
[imu_standalone-1] [INFO] Configuration complete.  (ifm3d.imu on_configure:161)
[imu_standalone-1] [INFO] on_activate(): inactive -> activating  (ifm3d.imu on_activate:167)
[imu_standalone-1] [INFO] Starting the Framegrabbers...  (ifm3d.imu on_activate:182)
[imu_standalone-1] [INFO] Diagnostic monitoring active.  (ifm3d.imu on_activate:187)
[imu_standalone-1] [INFO] ImuModule: on_activate called  (ifm3d.imu on_activate:302)

```

This will launch the `/ifm3d/imu` node, using the default parameters defined in `config/imu_default_parameters.yaml`.

This node provides the following interfaces:
```bash
$ ros2 node info /ifm3d/imu 
/ifm3d/imu
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /ifm3d/imu/imu_burst: ifm3d_ros2/msg/ImuBurst
    /ifm3d/imu/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /tf_static: tf2_msgs/msg/TFMessage
  Service Servers:
    /ifm3d/imu/Config: ifm3d_ros2/srv/Config
    /ifm3d/imu/Dump: ifm3d_ros2/srv/Dump
    /ifm3d/imu/GetDiag: ifm3d_ros2/srv/GetDiag
    /ifm3d/imu/Softoff: ifm3d_ros2/srv/Softoff
    /ifm3d/imu/Softon: ifm3d_ros2/srv/Softon
    /ifm3d/imu/change_state: lifecycle_msgs/srv/ChangeState
    /ifm3d/imu/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ifm3d/imu/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /ifm3d/imu/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/imu/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ifm3d/imu/get_parameters: rcl_interfaces/srv/GetParameters
    /ifm3d/imu/get_state: lifecycle_msgs/srv/GetState
    /ifm3d/imu/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/imu/list_parameters: rcl_interfaces/srv/ListParameters
    /ifm3d/imu/set_parameters: rcl_interfaces/srv/SetParameters
    /ifm3d/imu/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```