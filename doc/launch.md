# Launch the node

Launch the camera node (assuming you are in `~/colcon_ws/`):
```
$ . install/setup.bash
$ ros2 launch ifm3d_ros2 camera.launch.py
```

:::{note}
We also provide a helper launch file to start multiple camera nodes. See the documentation [here](multi_head.md).
:::

To visualize the data with RViz, set the `visualization` argument of the launch script to `true`:
```
$ ros2 launch ifm3d_ros2 camera.launch.py visualization:=true
```


![rviz1](figures/O3R_merged_point_cloud.png)

Congratulations! You can now have complete control over the O3R perception platform from inside ROS2.
