# Camera Transforms and Calibration

This document explains how the O3R camera transform mechanism works, the calibration process, and how transforms are published in ROS 2.

## Overview

The O3R camera system uses a multi-level transform hierarchy to accurately position sensor data in 3D space. Understanding this hierarchy is crucial for proper integration with robotic systems and point cloud processing.

## Transform Hierarchy

The O3R system defines three main coordinate frames:

1. **`ifm_base_link`** - The reference frame after calibration
2. **`mounting_link`** - The physical mounting point of the camera 
3. **`optical_link`** - The optical center of the camera sensor

![Transform hierarchy overview](./figures/transforms-1.png)

The complete transform chain is: `ifm_base_link` → `mounting_link` → `optical_link`

## Calibration Reference Point

:::{important}
The O3R calibration reference point, `mounting_link`, is located at the **center back of the camera housing**, not at the optical center. This is a key difference from some other camera systems.
:::

![Detailed view of mounting and optical frames](./figures/transforms-2.png)

When performing calibration:
- The **mounting frame** reference is at the back of the O3R camera head housing
- Scale drawings for exact mounting dimensions can be found on [ifm.com](https://www.ifm.com) in the download section for your specific camera model
- The **optical frame** is automatically calculated from factory intrinsic parameters

### Frame Reference Behavior

**ifm_base_link positioning:**
- The `ifm_base_link` is defined by the extrinsic parameters set in the JSON configuration (`extrinsicHeadToUser`)
- Generally, the ifm base frame is configured to have its origin at the center of the robot coordinate system, but this is **not required**
- **When no extrinsic parameters are set**, the `ifm_base_link` and the `mounting_link` are the same

**Intrinsic parameters:**
- Each set of intrinsic parameters is **unique to a specific camera head** and set in production
- These parameters are **not expected to change over time**
- They define the relationship between `mounting_link` and `optical_link`

### Additional Calibration Resources

For comprehensive information about O3R calibration routines and best practices:
- **General calibration guide**: [O3R Calibration Routines](https://ifm3d.com/latest/CalibrationRoutines/index_calibrations.html)
- **Introduction to calibrations**: [Calibration Concepts](https://ifm3d.com/latest/CalibrationRoutines/IntroToCalibrations/README.html)
## Calibration Process

### Where to Calibrate

Calibration should be performed **in the ifm coordinate system**, not in "ROS world" coordinates. There are two primary methods:

#### Method 1: Using ifm Vision Assistant

The ifm Vision Assistant provides a graphical interface for camera calibration:

1. Connect to your O3R device via Vision Assistant
2. Navigate to the camera configuration section
3. Use the built-in calibration tools to set extrinsic parameters
4. The calibration defines the transform from `mounting_link` to `ifm_base_link`

For detailed instructions, refer to [manual calibration using ifm Vision Assistant](https://ifm3d.com/latest/SoftwareInterfaces/iVA/manual_calibration.html).

#### Method 2: JSON Configuration File

The calibration values can be applied via JSON configuration files. See the configuration documentation:

- For camera nodes: [Camera Parameters](parameters.md) (see `config_file` parameter)
- For ODS applications: [ODS Configuration](../ods_node/ods_configuration.md)

A ready-to-use example for a single camera head is provided in `config/o3r_configs/o3r_one_head_calibration_only.json`:
```json
{
  "ports": {
    "port4": {
      "processing": {
        "extrinsicHeadToUser": {
          "rotX": 0,
          "rotY": 1.57,
          "rotZ": -1.57,
          "transX": 0,
          "transY": 0,
          "transZ": 0.35
        }
      },
      "state": "RUN"
    }
  }
}
```

For ODS, see `config/o3r_configs/o3r_ods_presets_calibration.json` for an example combining calibration with application setup and presets.

:::{note}
**Independent Calibration Values**: The rotation and translation values in the JSON configuration are applied **independently**, not sequentially. Each parameter contributes directly to the final transform matrix.
:::

## Published Transforms

The `ifm3d-ros2` node can publish two transforms:

### 1. Base to Mounting Transform
- **From**: `ifm_base_link` 
- **To**: `mounting_link`
- **Source**: Extrinsic calibration parameters (`extrinsicHeadToUser`)
- **Purpose**: Positions the camera relative to the calibrated reference frame

### 2. Mounting to Optical Transform  
- **From**: `mounting_link`
- **To**: `optical_link` 
- **Source**: Factory intrinsic calibration parameters
- **Purpose**: Provides the optical center for image and point cloud data

**Transform Control:**
- Both transforms can be enabled/disabled via ROS parameters
- The names for all three frames can be changed via node parameters  
- See [Camera Parameters](parameters.md) for detailed parameter descriptions

## Point Cloud Reference Frame

After calibration, all point cloud data is published with `ifm_base_link` as the reference frame. This means:

- Point cloud coordinates are relative to your calibrated reference system
- The calibration establishes the relationship between camera data and your robot/world coordinate system
- Visualization tools like RViz will display the point cloud in the `ifm_base_link` frame

## Integration with ROS Navigation Stack

For integration with ROS navigation and other robotics applications:

1. **Robot Base Link**: Your robot should publish a transform from its `base_link` to `ifm_base_link`
2. **Static Transform**: This can typically be a static transform since the camera position relative to the robot is fixed
3. **Multiple Cameras**: Each camera can have its own `ifm_base_link` or share a common reference frame

Example static transform publisher:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link ifm_base_link
```