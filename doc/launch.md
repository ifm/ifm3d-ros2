
# Launch the node

Due to an incompatibility between FastDDS (the default DDS implementation for ROS Humble) and the ifm3d API, the DDS settings need to be changed before running ifm3d-ros2:
- Install the alternative DDS settings configuration (this needs to be done once): `$ sudo apt-get install ros-foxy-rmw-cyclonedds-cpp`
- Source the `ìfm3d_ros2` installation: `$ source ~/colcon_ws/install/setup.bash`
- Change the DDS settings (this needs to be done every time and needs to be done after the source of ROS and the ifm3d_ros2 node): `$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp `

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
