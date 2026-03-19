# Launch the PDS nodes

To launch the PDS nodes, you can use the provided launchfile `pds.launch.py`:
```bash
$ ros2 launch ifm3d_ros2 pds.launch.py
```

This will launch the `/ifm3d/pds` and `/ifm3d/pds_vis` nodes, using the default parameters defined in `config/pds_default_parameters.yaml`.

This nodes provide the following interfaces:
```bash
$ ros2 node info /ifm3d/pds
/ifm3d/pds
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /ifm3d/pds/pallet_detection: ifm3d_ros2/msg/PalletDetectionArray
    /ifm3d/pds/pds_full_result: ifm3d_ros2/msg/PdsFullResult
    /ifm3d/pds/rack_detection: ifm3d_ros2/msg/RackDetection
    /ifm3d/pds/transition_event: lifecycle_msgs/msg/TransitionEvent
    /ifm3d/pds/volume_check: ifm3d_ros2/msg/VolumeCheck
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /ifm3d/pds/Config: ifm3d_ros2/srv/Config
    /ifm3d/pds/Dump: ifm3d_ros2/srv/Dump
    /ifm3d/pds/GetDiag: ifm3d_ros2/srv/GetDiag
    /ifm3d/pds/Softoff: ifm3d_ros2/srv/Softoff
    /ifm3d/pds/Softon: ifm3d_ros2/srv/Softon
    /ifm3d/pds/change_state: lifecycle_msgs/srv/ChangeState
    /ifm3d/pds/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ifm3d/pds/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /ifm3d/pds/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/pds/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ifm3d/pds/get_parameters: rcl_interfaces/srv/GetParameters
    /ifm3d/pds/get_state: lifecycle_msgs/srv/GetState
    /ifm3d/pds/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/pds/list_parameters: rcl_interfaces/srv/ListParameters
    /ifm3d/pds/set_parameters: rcl_interfaces/srv/SetParameters
    /ifm3d/pds/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /ifm3d/pds/set_pds_mode: ifm3d_ros2/srv/SetPdsMode
  Service Clients:

  Action Servers:
    /ifm3d/pds/get_pallet: ifm3d_ros2/action/GetPallet
    /ifm3d/pds/get_rack: ifm3d_ros2/action/GetRack
    /ifm3d/pds/volume_check: ifm3d_ros2/action/VolumeCheck
  Action Clients:

$ ros2 node info /ifm3d/pds_vis 
/ifm3d/pds_vis
  Subscribers:
    /ifm3d/pds/pds_full_result: ifm3d_ros2/msg/PdsFullResult
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /ifm3d/pds_vis/pds_vis_marker: visualization_msgs/msg/MarkerArray
    /ifm3d/pds_vis/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /ifm3d/pds_vis/change_state: lifecycle_msgs/srv/ChangeState
    /ifm3d/pds_vis/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ifm3d/pds_vis/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /ifm3d/pds_vis/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/pds_vis/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ifm3d/pds_vis/get_parameters: rcl_interfaces/srv/GetParameters
    /ifm3d/pds_vis/get_state: lifecycle_msgs/srv/GetState
    /ifm3d/pds_vis/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /ifm3d/pds_vis/list_parameters: rcl_interfaces/srv/ListParameters
    /ifm3d/pds_vis/set_parameters: rcl_interfaces/srv/SetParameters
    /ifm3d/pds_vis/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```