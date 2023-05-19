# How to visualize the point cloud with RVIZ
The included launch file `camera.launch.py` will publish and remap all topics and services to `/ifm3d/camera`, 
for example the point cloud topic will be remapped to `/ifm3d/camera/cloud`.
Both the namespace (default: `ifm3d`) and the node name (default: `camera`)
can be changed via launch arguments.

A pre-configured RViz2 will be launched when setting the `visualization` argument of the launch file to `true`.
Be aware that the RViz configuration assumes the default namespace and node name.
You have to change the topic subscribtions yourself when using non-default values.
```bash
ros2 launch ifm3d_ros2 camera.launch.py visualization:=true
```

## QoS reliability - best effort
When you open RVIZ for the first time and subscribe to the point cloud topic, it will not be displayed as we need to change the default quality of service (QoS) settings per topic.

To change these settings:
1. subscribe to a topic in RVIZ by adding it (ADD button)
2. Expand the topic settings
3. Select `reliability` and set it to `Best Effort`
