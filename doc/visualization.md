# How to visualize data with RVIZ

The included launch file `camera.launch.py` will publish and remap all topics and services to `/ifm3d/camera`, for example the point cloud topic will be remapped to `/ifm3d/camera/cloud`.
Both the namespace (default: `ifm3d`) and the node name (default: `camera`) can be changed via the launch description files.

A pre-configured RViz2 will be launched when setting the `visualization` argument of the launch file to `true`. Be aware that the RViz configuration assumes the default namespace and node name.
You have to change the topic subscriptions yourself when using non-default values.
```bash
ros2 launch ifm3d_ros2 camera.launch.py visualization:=true
```

## Viewing the RGB image

The RGB image is published on the `/ifm3d/camera*/rgb` topic in a compressed JPEG format.
This is useful to save bandwidth, but cannot be visualized as-is with RViz.

The image can be uncompressed using the `image_transport republish` node:
```bash
$ sudo apt install ros-humble-compressed-image-transport
$ ros2 run image_transport republish compressed raw --ros-args --remap /in/compressed:=/ifm3d/camera/rgb --remap out:=/uncompressed_rgb
```

In RViz, you can now subscribe to the `/uncompressed_rgb` topic to visualize the RGB image.
