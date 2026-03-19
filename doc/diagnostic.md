# Diagnostic

Both the camera node and the ODS node publish diagnostic information to the `/diagnostics` topic.
The diagnostic message contains an error code and a message, referring directly to an error from ifm3d or from the embedded software.

## Severity mapping and operator handling

ROS diagnostics supports the levels `OK`, `WARN`, `ERROR`, and `STALE`.
ifm severities are mapped as follows:

| ifm severity | ROS `status.level` | Operational meaning |
| --- | --- | --- |
| `info` | `OK` (`0`) | Informational |
| `minor` | `WARN` (`1`) | Degraded, continue with caution |
| `major` | `ERROR` (`2`) | Stop robot; can self-heal |
| `critical` | `ERROR` (`2`) | Stop robot; manual intervention required |

`major` and `critical` are both published as `ERROR` to keep safety behavior conservative and consistent.
To distinguish them, additional key/value metadata is published in each diagnostic status:

- `severity_normalized`
- `safety_action`
- `recovery_expectation`
- `operator_action`

This allows dashboards and integrators to trigger the same stop behavior for all `ERROR`s, while still handling `major` and `critical` differently in SOPs.

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
  hardware_id: /ifm3d/camera
  values:
  - key: bootid
    value: '"71fd8e7d-385a-4f86-8a88-bb59f5112c73"'
  - key: events
    value: '[{"description":"Unable to determine velocity","id":105007,"name":"ERROR_ODSAPP_VELOCITY_UNAVAILABLE","source":"/applications/in...'
  - key: timestamp
    value: '1652509745818232736'
  - key: version
    value: '{"diagnostics":"0.0.11","euphrates":"1.34.226","firmware":"1.1.41.4507"}'
  - key: severity_normalized
    value: major
  - key: safety_action
    value: stop_robot
  - key: recovery_expectation
    value: self_healing_possible
  - key: operator_action
    value: stop_then_monitor_for_auto_recovery
---
```

For more details on the error codes and potential troubleshooting strategies, refer to the [O3R diagnostic documentation](https://ifm3d.com/latest/SoftwareInterfaces/ifmDiagnostic/diagnostic.html).

