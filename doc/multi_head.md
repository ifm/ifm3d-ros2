# Multi head launch file configuration

It is possible to stream data from multiple ports, while connecting both
2D and 3D data from your camera with your VPU or even with multiple cameras
connected to one VPU.

This can be achieved by including the provided 'camera.launch.py` multiple times
into one launch script and setting the camera-specific parameters (like PCIC-port) correctly.

There are two examples provided doing exactly that.

### example_o3r_2d_and_3d.launch.py
This example script connects two CameraNodes to the 2D and 3D stream of one O3R.

The nodes are named camera_2d and camera_3d.

The Ports on the VPU should be connected as follows:
* Camera 2D: Port 0
* Camera 3D: Port 2

The configuration for the nodes is read from a two different files:
o3r_2d.yaml and o3r_3d.yaml. To change the port configuration, edit the YAML file directly.

To launch this example, use the following command:
```
ros2 launch ifm3d_ros2 example_o3r_2d_and_3d.launch.py
```

### example_two_o3r_heads.launch.py
This example script connects four CameraNodes to the 2D and 3D streams of two O3R.

The nodes are named left_camera_2d, left_camera_3d, right_camera_2d and
right_camera_3d. The differentiation in left and right is arbitrary and only acts
as illustration for a fictitious use case.

The Ports on the VPU should be connected as follows:
* Left Camera 2D: Port 0
* Left Camera 3D: Port 2
* Right Camera 2D: Port 1
* Right Camera 3D: Port 3

The configuration for all four nodes is read from a single file: two_o3r_heads.yaml

To launch this example, use the following command:
```
ros2 launch ifm3d_ros2 example_two_o3r_heads.launch.py
```

### Adapting the camera configuration to your needs
There are multiple example configurations provided in `config/example`.
They can act as inspiration when configuring your own multi-camera setup.
