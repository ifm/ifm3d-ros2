# Multi head launch file configuration

It is possible to stream data from multiple imagers and camera heads (i.e. ports): while connecting both 2D and 3D data from your camera with your VPU or even with multiple cameras connected to one VPU.

This can be achieved by altering your launch configuration yaml file and setting the camera-specific parameters, namely `pcic-port` and `buff_id_lists`, correctly.
There are examples provided doing exactly that.

## `o3r_2d.yaml`
This example configuration connects to the 2D (RGB) data stream of one O3R camera head connected to port 0:

The Ports on the VPU should be connected as follows:
* Camera 2D: physical port 0 - corresponds to `pcic_port=50010`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 camera.launch.py parameter_file_name:=o3r_2d.yaml
```

## `o3r_3d.yaml`
This example configuration connects to the 3D (TOF) data stream of one O3R camera head connected to port 2:

The Ports on the VPU should be connected as follows:
* Camera 3D: physical port 2 - corresponds to `pcic_port=50012`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 camera.launch.py parameter_file_name:=o3r_3d.yaml
```

## `two_o3r_heads.yaml`
This example configuration connects to two 3D (TOF) data stream **AND** two 2D (RGB) data stream of **two** O3R camera head connected to ports 0 - 3:

The Ports on the VPU should be connected as follows:
Camera head 1:
* Camera 2D: physical port 0 - corresponds to `pcic_port=50010`
* Camera 3D: physical port 2 - corresponds to `pcic_port=50012`

Camera head 2:
* Camera 2D: physical port 1 - corresponds to `pcic_port=50011`
* Camera 3D: physical port 3 - corresponds to `pcic_port=50013`

To launch this example, use the following command:
```bash
ros2 launch ifm3d_ros2 two_o3r_heads.launch.py parameter_file_name:=two_o3r_heads.yaml
```

### Adapting the camera configuration to your needs
The example configurations are provided in the directory `config/examples`.
They can act as inspiration when configuring your own multi-camera setup.

Please edit / add your own specific hardware and software configuration via these configuration yml files.
For a specific number of ros node camera streams other than 1 or 4 (see examples) please create a updated launch helper yourself.