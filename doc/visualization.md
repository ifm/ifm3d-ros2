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

### Automatic Uncompressed Image Publishing

The ifm3d_ros2 package now automatically handles uncompressed image publishing through the `publish_uncompressed` parameter. When this parameter is set to `true` in your camera configuration, the launch file will automatically spawn a republish node to provide the uncompressed RGB image.

**Configuration options:**

1. **YAML configuration** (set under the camera's `ros__parameters`):
```yaml
/ifm3d/camera:
  ros__parameters:
    publish_uncompressed: true  # Automatically provides rgb_uncompressed topic
```

2. **Launch argument override** (takes precedence over YAML):
```bash
ros2 launch ifm3d_ros2 camera.launch.py publish_uncompressed:=true
```

**Precedence and defaults:**
- Launch arg empty (default "") → defer to YAML setting
- Launch arg explicit true/false → overrides YAML setting  
- Neither provided → feature disabled

**Available RGB topics:**
- `/ifm3d/camera*/rgb` - Compressed JPEG image (bandwidth efficient)
- `/ifm3d/camera*/rgb_uncompressed` - Uncompressed image (RViz compatible)

When active, an `image_transport republish compressed raw` helper process is started. The core camera node still only publishes compressed images, keeping CPU usage low when the uncompressed topic is not needed.

In RViz, you can directly subscribe to the `/ifm3d/camera*/rgb_uncompressed` topic to visualize the RGB image without any additional setup.

### Manual Method (if needed)

If the automatic method is not available, you can still manually uncompress images:
```bash
$ sudo apt install ros-humble-compressed-image-transport
$ ros2 run image_transport republish compressed raw --ros-args --remap /in/compressed:=/ifm3d/camera/rgb --remap out:=/uncompressed_rgb
```
