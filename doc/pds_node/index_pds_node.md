# PDS node

The `ifm3d-ros2` package provides a Pick and Drop System (PDS) node, which has the capability to detect pallets, open rack space, and check volumes for occupation.
Results are visualized using `visualization_markers`.

:::{toctree}
    :maxdepth: 2
    PDS configuration <pds_configuration>
    Launch <pds_launch>
    Topics <pds_topics>
    Parameters <pds_params>
:::

## Operating Modes

The PDS node has different operating modes, which can be set via the `~/set_pds_mode` service.
The default mode is `0` which is the `ACTION_ONLY` mode.

In `ACTION_ONLY` mode, the node does provide three action servers `~/get_pallet`, `~/get_rack`, and `~/volume_check`.
Using these actions, the PDS functions `getPallet`, `getRack`, and `volCheck` can be used to get a singular detection result.

The following modes allow for continuous detections, which are published to their respective topic:
`GET_PALLET_CONTINUOUS` (1), `GET_RACK_CONTINUOUS` (2), and `VOLUME_CHECK_CONTINUOUS` (3).
To activate these modes, select the correct mode and provide the corresponding request via the `~/set_pds_mode` service.

Calling one of the actions mentioned above while in continuous mode switches the mode to `ACTION_ONLY` (0).
If the goal field `switch_to_continuous_mode` is set to `true`, the action switches the node to continuous mode and keeps publishing new detections to the corresponding topic.

## Usage Examples

### Using Actions

Actions use a nested `request` field for their parameters. Here are examples for each action type:

#### Get Pallet (single detection)
```bash
ros2 action send_goal /ifm3d/pds/get_pallet ifm3d_ros2/action/GetPallet "{request: {depth_hint: 1.5, pallet_index: 0, pallet_order: \"zDescending\"}}" --feedback
```

#### Get Rack (single detection)
```bash
ros2 action send_goal /ifm3d/pds/get_rack ifm3d_ros2/action/GetRack "{request: {horizontal_drop_position: \"left\", vertical_drop_position: \"interior\", depth_hint: 1.8, z_hint: -0.4}}" --feedback
```

With a custom clearing volume:
```bash
ros2 action send_goal /ifm3d/pds/get_rack ifm3d_ros2/action/GetRack "{request: {horizontal_drop_position: \"left\", vertical_drop_position: \"interior\", depth_hint: 1.8, z_hint: 0.4, clearing_volume: {x_min: -0.1, x_max: 1.2, y_min: 0.1, y_max: 1.3, z_min: 0.1, z_max: 0.4}}}" --feedback
```

#### Volume Check (single detection)
```bash
ros2 action send_goal /ifm3d/pds/volume_check ifm3d_ros2/action/VolumeCheck "{request: {x_min: 1.0, x_max: 3.0, y_min: -0.5, y_max: 0.5, z_min: -0.2, z_max: 0.5}}" --feedback
```

### Switching to Continuous Mode

There are two ways to enable continuous mode:

#### Option 1: Using the `set_pds_mode` service

Switch to continuous pallet detection:
```bash
ros2 service call /ifm3d/pds/set_pds_mode ifm3d_ros2/srv/SetPdsMode "{mode: 1, get_pallet_request: {depth_hint: 1.5, pallet_index: 0}}"
```

Switch to continuous rack detection:
```bash
ros2 service call /ifm3d/pds/set_pds_mode ifm3d_ros2/srv/SetPdsMode "{mode: 2, get_rack_request: {horizontal_drop_position: \"right\", vertical_drop_position: \"interior\", depth_hint: 1.6, z_hint: 0.4}}"
```

Switch to continuous volume check:
```bash
ros2 service call /ifm3d/pds/set_pds_mode ifm3d_ros2/srv/SetPdsMode "{mode: 3, volume_check_request: {x_min: 1.0, x_max: 3.0, y_min: -0.5, y_max: 0.5, z_min: -0.2, z_max: 0.5}}"
```

#### Option 2: Using action goals with `switch_to_continuous_mode: true`

```bash
ros2 action send_goal /ifm3d/pds/get_rack ifm3d_ros2/action/GetRack "{request: {depth_hint: 1.4, z_hint: 0.4}, switch_to_continuous_mode: true}" --feedback
```

This sends the action goal, returns the result, and then switches to continuous mode, continuously publishing detections to the `~/rack_detection` topic.

### Switching Back to ACTION_ONLY Mode

To stop continuous detections and return to action-only mode, use the `set_pds_mode` service:
```bash
ros2 service call /ifm3d/pds/set_pds_mode ifm3d_ros2/srv/SetPdsMode "{mode: 0}"
```

:::{note}
While in continuous mode, action goals will be rejected. You must first switch back to `ACTION_ONLY` mode using the service above before actions can be accepted again.
:::

## Actions

The actions return their detection as result to the action client.
Additionally, they publish a message to the corresponding topic.

## Topics

Whenever a PDS function is used, the full detection result is published on the `~/pds_full_result` topic.
With the default launch namespace (`/ifm3d`) and node name (`pds`), this resolves to `/ifm3d/pds/pds_full_result`.
Additionally, messages are sent to specialized topics dependent on the used function:

| Mode                          | Default Topic           |
| ----------------------------- | ----------------------- |
| `ACTION_ONLY` (0)             | Action-dependent      |
| `GET_PALLET_CONTINUOUS` (1)   | `~/pallet_detection`    |
| `GET_RACK_CONTINUOUS` (2)     | `~/rack_detection`      |
| `VOLUME_CHECK_CONTINUOUS` (3) | `~/volume_check`        |
