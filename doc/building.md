# Building and Installing the ifm3d-ros2 package

## Prerequisites

### Ubuntu and ROS
We suggest building the `ifm3d-ros2` node on top of [Ubuntu 22.04 Jammy Jellyfish](https://releases.ubuntu.com/jammy/) and [ROS Humble](https://docs.ros.org/en/humble/index.html) or
[Ubuntu 24.04 Noble Numbat](https://releases.ubuntu.com/noble/) and [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)

### ifm3d C++ API
The ROS node `ifm3d_ros2` requires the C++ API ifm3d to be installed locally for your system before compiling and running the ROS node.
Refer to [the compatibility matrix](../README.md) to find out the correct ifm3d API version.

Follow these instructions on how to install `ifm3d` (we recommend using the pre-built package): [install ifm3d](https://api.ifm3d.com/stable/content/installation_instructions/install_linux_binary.html).

## Build and install `ifm3d-ros2`

Building and installing `ifm3d-ros2` is accomplished by utilizing the ROS 2 [colcon](https://colcon.readthedocs.io/en/released/) tool.

### Installation directory
First, we need to decide where we want our software to be installed. 
For purposes of this document, we will assume that we will install our ROS packages at `~/colcon_ws/`.

:::{note}
Below we assume `humble`. Adapting to other ROS distributions is left as an exercise for the reader.
:::

### Colcon workspace
Next, we want to create a _colcon workspace_ that we can use to build and install that code from.

```
$ mkdir -p ~/colcon_ws/src
```

### Get the `ifm3d-ros2` code from GitHub

Next, we need to get the code from GitHub. Please adapt the commands when not following the suggested directory structure.

```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/colcon_ws/src
$ git clone https://github.com/ifm/ifm3d-ros2.git
$ git checkout <version> # Replace the targeted version
```

:::{note}
The master branch is generally a work in progress.

We recommend picking a {{ '[tagged released version]({})'.format(ifm3d_ros2_latest_tag_url) }} for your builds, to ensure stability between builds.
:::

### Build `ifm3d-ros2`
Build your workspace:

```bash
$ cd ~/colcon_ws/
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
Starting >>> ifm3d_ros2
Finished <<< ifm3d_ros2 [17.6s]

Summary: 1 package finished [17.8s]
```

:::{note}
The tests are not functional at the moment.
:::

To confirm that the node is functional, try [launching it](camera_node/launch.md) and inspecting [the published topics](camera_node/topics.md).


## Docker containers
We provide a Dockerfile build instruction for building the ROS2 node inside a Docker container.
For reference and for deployment of the ROS node via a Docker container see the [Dockerfile](../Dockerfile).