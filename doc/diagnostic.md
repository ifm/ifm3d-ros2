# Diagnostic 

Both the camera node and the ODS node publish diagnostic information to the `/diagnostic` topic. 
The diagnostic message contains an error code and a message, referring directly to an error from ifm3d or from the embedded software. 

Below is an example of a diagnostic message:
```bash
$ ros2 topic echo /diagnostics 
header:
  stamp:
    sec: 1652509745
    nanosec: 818232736
  frame_id: ''
status:
- level: "\x02"
  name: ''
  message: ''
  hardware_id: /ifm3d/diag_module
  values:
  - key: bootid
    value: '"71fd8e7d-385a-4f86-8a88-bb59f5112c73"'
  - key: events
    value: '[{"description":"Unable to determine velocity","id":105007,"name":"ERROR_ODSAPP_VELOCITY_UNAVAILABLE","source":"/applications/in...'
  - key: timestamp
    value: '1652509745818232736'
  - key: version
    value: '{"diagnostics":"0.0.11","euphrates":"1.34.226","firmware":"1.1.41.4507"}'
---
```

For more details on the error codes and potential troubleshooting strategies, refer to the [O3R diagnostic documentation](../../../documentation/SoftwareInterfaces/ifmDiagnostic/index_diagnostic.md).

