# Launch the ODS node

To launch the ODS node, you can use the provided launchfile `ods.launch.py`:
```bash
$ ros2 launch ifm3d_ros2 ods.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO                                                                                                                                                          
[INFO] [ods_standalone-1]: process details: cmd='/home/usmasslo/ROS/ROS2/colcon_ws/install/ifm3d_ros2/lib/ifm3d_ros2/ods_standalone --ros-args --log-level info --ros-args -r __node:=ods -r __ns:=/ifm3d --params-
file ~/colcon_ws/install/ifm3d_ros2/share/ifm3d_ros2/config/ods_default_parameters.yaml', cwd='None', custom_env?=True                                                                       
[INFO] [ods_standalone-1]: process started with pid [486398]                                                                                       
[ods_standalone-1] [INFO] [1728931867.359101479] [ifm3d.ods]: namespace: /ifm3d                                                                                          
[ods_standalone-1] [INFO] [1728931867.359179144] [ifm3d.ods]: node name: ods                                                                                            
[ods_standalone-1] [INFO] [1728931867.359186657] [ifm3d.ods]: middleware: rmw_fastrtps_cpp                                                                               
[ods_standalone-1] [INFO] [1728931867.359190490] [ifm3d.ods]: Declaring parameters...                                                                                  
[ods_standalone-1] [INFO] [1728931867.359323428] [ifm3d.ods]: After the parameters declaration                                                                                    
[ods_standalone-1] [INFO] [1728931867.359406942] [ifm3d.ods]: node created, waiting for `configure()`...                                                                                           
[ods_standalone-1] [INFO] [1728931867.521751135] [ifm3d.ods]: on_configure(): unconfigured -> configuring                                                                                   
[ods_standalone-1] [INFO] [1728931867.521862570] [ifm3d.ods]: Parsing parameters...             [ods_standalone-1] [INFO] [1728931867.521944451] [ifm3d.ods]: ip: 192.168.0.69
[ods_standalone-1] [INFO] [1728931867.521980451] [ifm3d.ods]: pcic_port: 51010
[ods_standalone-1] [INFO] [1728931867.521997881] [ifm3d.ods]: xmlrpc_port: 80
[ods_standalone-1] [INFO] [1728931867.522012697] [ifm3d.ods]: Parameters parsed.
[ods_standalone-1] [INFO] [1728931867.522028686] [ifm3d.ods]: Adding callbacks to handle parameter changes at runtime...
[ods_standalone-1] [INFO] [1728931867.525109429] [ifm3d.ods]: Callbacks set.
[ods_standalone-1] [INFO] [1728931867.525177112] [ifm3d.ods]: Initializing Device
[ods_standalone-1] [INFO] [1728931867.525325450] [ifm3d.ods]: Initializing FrameGrabber for data
[ods_standalone-1] [INFO] [1728931868.040227637] [ifm3d.ods]: Creating OdsModule...
[ods_standalone-1] [INFO] [1728931868.040519950] [ifm3d.ods]: FunctionModule contructor called.
[ods_standalone-1] [INFO] [1728931868.040545387] [ifm3d.ods]: OdsModule contructor called.
[ods_standalone-1] [INFO] [1728931868.040638930] [ifm3d.ods]: OdsModule created.
[ods_standalone-1] [INFO] [1728931868.040644930] [ifm3d.ods]: Creating DiagModule...
[ods_standalone-1] [INFO] [1728931868.040675690] [ifm3d.ods]: FunctionModule contructor called.
[ods_standalone-1] [INFO] [1728931868.040736808] [ifm3d.ods]: hardware_id: /ifm3d/diag_module
[ods_standalone-1] [INFO] [1728931868.040756934] [ifm3d.ods]: DiagModule created.
[ods_standalone-1] [INFO] [1728931868.040803035] [ifm3d.ods]: OdsModule: on_configure called
[ods_standalone-1] [INFO] [1728931868.055527902] [ifm3d.ods]: Parameter ods.frame_id set to 'ods_frame'
[ods_standalone-1] [INFO] [1728931868.055542337] [ifm3d.ods]: DiagModule: on_configure called
[ods_standalone-1] [INFO] [1728931868.057683191] [ifm3d.ods]: Creating BaseServices...
[2]
[ods_standalone-1] [INFO] [1728931868.057733035] [ifm3d.ods]: BaseServices constructor called.
[ods_standalone-1] [INFO] [1728931868.060201097] [ifm3d.ods]: Services created;
[ods_standalone-1] [INFO] [1728931868.060224508] [ifm3d.ods]: BaseServices created.
[ods_standalone-1] [INFO] [1728931868.060237899] [ifm3d.ods]: Configuration complete.
[ods_standalone-1] [INFO] [1728931868.061711296] [ifm3d.ods]: on_activate(): inactive -> activating                                                                                                        [89/139]
[ods_standalone-1] [INFO] [1728931868.061807873] [ifm3d.ods]: Starting the Framegrabbers...
[ods_standalone-1] [INFO] [1728931868.187758863] [ifm3d.ods]: Diagnostic monitoring active.
[ods_standalone-1] [INFO] [1728931868.187814195] [ifm3d.ods]: OdsModule: on_activate called
[ods_standalone-1] [INFO] [1728931868.187832369] [ifm3d.ods]: DiagModule: on_activate called
```

This will launch the `/ifm3d/ods` node, using the default parameters defined in `config/ods_default_parameters.yaml`.

This node provides the following interfaces:
```bash
$ ros2 node info /ifm3d/ods                                                
/ifm3d/ods           
  Subscribers:           
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /ifm3d/ods/ods_info: ifm3d_ros2/msg/Zones
    /ifm3d/ods/ods_occupancy_map_ifm: ifm3d_ros2/msg/OccGrid
    /ifm3d/ods/ods_occupancy_map_ros: nav_msgs/msg/OccupancyGrid
    /ifm3d/ods/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /ifm3d/ods/Config: ifm3d_ros2/srv/Config
    /ifm3d/ods/Dump: ifm3d_ros2/srv/Dump
    /ifm3d/ods/GetDiag: ifm3d_ros2/srv/GetDiag
    /ifm3d/ods/Softoff: ifm3d_ros2/srv/Softoff
    /ifm3d/ods/Softon: ifm3d_ros2/srv/Softon
    /ifm3d/ods/change_state: lifecycle_msgs/srv/ChangeState
    /ifm3d/ods/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ifm3d/ods/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /ifm3d/ods/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/ods/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ifm3d/ods/get_parameters: rcl_interfaces/srv/GetParameters
    /ifm3d/ods/get_state: lifecycle_msgs/srv/GetState
    /ifm3d/ods/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/ods/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /ifm3d/ods/list_parameters: rcl_interfaces/srv/ListParameters
    /ifm3d/ods/set_parameters: rcl_interfaces/srv/SetParameters
    /ifm3d/ods/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```