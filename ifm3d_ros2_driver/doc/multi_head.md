## Multi head launch file configuration
How to start streams from multiple heads at the same time:  

Use as following command: (relative location starting from your `colcon_ws`)
```
ros2 launch ifm3d_ros2 multi_cameras.launch.py namespace:='ifm3d_ros2' params:='src/config/params_2cams.yaml'
```

This will launch two instances of the lifecycle node and publish their data to independent topics and coordinate reference frames.  

> Note: Please be aware that the `namespace:=ifm3d_ros2` argument has to be the same as the first field in the yaml configuration file. Otherwise, the namespace and dependent topic name declarations will not match.

### up to 6 heads
We ship this ROS node with two example yaml files:
1. For 2 imagers, i.e both the 2D RGB imager and 3D TOF imager of a O3R camera head are connected.
2. For 6 imager: no specific match for 2D RGB or 3D TOF imagers

Please build your own yaml starting from this. A node process requires exactly one image stream. If the node gets started while the image stream is missing the node's process will reply with timeout warnings displayed on your main process shell. 
