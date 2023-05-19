# Building and Installing the ifm3d-ros2 package

## Prerequisites

### Ubuntu and ROS
We suggest building the `ifm3d-ros2` node on top of [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/) and [ROS Humble](https://docs.ros.org/en/humble/index.html).

>  This ROS node only supports cycloneDDS as the DDS implementation. To use it, install cycloneDDS with `sudo apt-get install ros-foxy-rmw-cyclonedds-cpp` (do this once), and export the configuration with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (do this every time).


### ifm3d C++ API
The ROS node `ifm3d_ros2` requires the C++ API ifm3d to be installed locally for your system before compiling and running the ROS node.
Refer to [the compatibility matrix](../README.md) to find out the correct ifm3d API version.

Follow these instructions on how to install `ifm3d` (we recommend using the pre-built package): [install ifm3d](https://api.ifm3d.com/stable/content/installation_instructions/install_linux_binary.html).

### Testing prerequisite

These two packages are only required for testing but not at runtime:
- launch_testing
- launch_testing_ament_cmake

On debian based systems they may be installed as follows (replacing `galactic`with your target ROS2 distribution).
```
$ sudo apt install ros-galactic-launch-testing ros-galactic-launch-testing-ament-cmake
```
:::{note}
The tests are currently a work in progress. 
:::
## Build and install `ifm3d-ros2`

Building and installing ifm3d-ros2 is accomplished by utilizing the ROS2 [colcon](https://colcon.readthedocs.io/en/released/) tool.

### Installation directory
First, we need to decide where we want our software to be installed. For purposes of this document, we will assume that we will install our ROS packages at `~/colcon_ws/src`.

:::{note}
Below we assume `humble`. Adapting to other ROS distributions is left as an exercise for the reader.
:::
### Colcon workspace
Next, we want to create a _colcon workspace_ that we can use to build and install that code from.

```
$ mkdir -p ~/colcon_ws/src
```

### Get the `ifm3d-ros2` code from GitHub
Next, we need to get the code from GitHub. Please adapt the commands when not following the suggested directory structure: `~/colcon_ws/src/`

```bash
$ cd ~/colcon_ws/src
$ git clone https://github.com/ifm/ifm3d-ros2.git
$ git checkout <version> # Replace the targetted version
```

:::{note}
The master branch is generally a work in progress.

We recommend picking a {{ '[tagged released version]({})'.format(ifm3d_ros2_latest_tag_url) }} for your builds, to ensure stability between builds.
:::

### Build `ifm3d-ros2`
Build your workspace:

```bash
$ cd ~/colcon_ws/
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ colcon build --cmake-args -DBUILD_TESTING=OFF
Starting >>> ifm3d_ros2
Finished <<< ifm3d_ros2 [17.6s]

Summary: 1 package finished [17.8s]
```

:::{note}
The tests are not functional at the moment. 
:::

To confirm that the node is functional, try [launching it](../doc/launch.md) and inspecting [the published topics](../doc/topics.md).

