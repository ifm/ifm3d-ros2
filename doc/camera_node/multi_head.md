# Multi head launch file configuration

It is possible to stream data from multiple cameras or from the two ports of one camera at once.
This can be achieved by setting the camera-specific parameters in the launch configuration files, namely `pcic-port` and `buffer_id_lists`.
We provide examples of default parameters for getting the 2D or 3D data from one camera, or all the data from two full O3R camera (two 2D data streams and two 3D data streams).

## `o3r_2d.yaml`
This example configuration connects to the 2D (RGB) data stream of one O3R camera head connected to port 0:

The ports on the VPU should be connected as follows:
* Camera 2D: physical port 0 - corresponds to `pcic_port=50010`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 camera.launch.py camera_name:=camera_2d parameter_file_name:=examples/o3r_2d.yaml
```
> **Note:** Don’t forget to specify the `camera_name` parameter if it differs from the default value, "camera", and the `camera_namespace` if it differs from the default, "ifm3d".

## `o3r_3d.yaml`
This example configuration connects to the 3D data stream of one O3R camera head connected to port 2:

The ports on the VPU should be connected as follows:
* Camera 3D: physical port 2 - corresponds to `pcic_port=50012`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 camera.launch.py camera_name:=camera_3d parameter_file_name:=examples/o3r_3d.yaml
```
> **Note:** Don’t forget to specify the `camera_name` parameter if it differs from the default value, "camera", and the `camera_namespace` if it differs from the default, "ifm3d".

## Launching a 2D and a 3D node simultaneously
This example configuration connects to one 3D data stream **AND** one 2D (RGB) data stream connected to ports 0 and 2:

- Camera 2D: physical port 0 - corresponds to `pcic_port=50010`
- Camera 3D: physical port 2 - corresponds to `pcic_port=50012`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 example_o3r_2d_and_3d.launch.py 
```


## `two_o3r_heads.yaml`
This example configuration connects to two 3D data stream **AND** two 2D (RGB) data stream of **two** O3R camera head connected to ports 0 - 3:

The ports on the VPU should be connected as follows:
Camera head 1:
* Camera 2D: physical port 0 - corresponds to `pcic_port=50010`
* Camera 3D: physical port 2 - corresponds to `pcic_port=50012`

Camera head 2:
* Camera 2D: physical port 1 - corresponds to `pcic_port=50011`
* Camera 3D: physical port 3 - corresponds to `pcic_port=50013`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 example_two_o3r_heads.launch.py parameter_file_name:=two_o3r_heads.yaml
```

## Adapting the camera configuration to your needs
The example configurations are provided in the directory `config/examples`.
They can act as inspiration when configuring your own multi-camera setup.

To adjust the configuration to your own setup, use these configuration files as a base to add or remove cameras, change the port number or edit the list of requested buffers.