# Building and Installing the ifm3d-ros2 package

## Table of contents
- [Building and Installing the ifm3d-ros2 package](#building-and-installing-the-ifm3d-ros2-package)
  - [Table of contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
    - [ifm3d C++ API](#ifm3d-c-api)
  - [Step-by-Step build instructions for the ROS node `ifm3d-ros2`](#step-by-step-build-instructions-for-the-ros-node-ifm3d-ros2)
    - [1. Installation directory of ROS node](#1-installation-directory-of-ros-node)
    - [2. create and initialize your colcon workspace](#2-create-and-initialize-your-colcon-workspace)
    - [3. Get the `ifm3d-ros2` wrapper code from GitHub](#3-get-the-ifm3d-ros2-wrapper-code-from-github)
    - [4. build the ROS node code](#4-build-the-ros-node-code)


## Prerequisites

We suggest building the `ifm3d-ros2` node on top of Ubuntu 22.04 Jammy Jellyfish and ROS Humble.

>  This ROS node only supports cycloneDDS as the DDS implementation. To use it, install cycloneDDS with `sudo apt-get install ros-foxy-rmw-cyclonedds-cpp` (do this once), and export the configuration with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (do this every time).


### ifm3d C++ API
The ROS node `ifm3d_ros2` requires the C++ API ifm3d to be installed locally for your system before compiling and running the ROS node.
Please see the compatibility matrix to find out the correct ifm3d API version.

Please follow these instructions on how to build `ifm3d` from source: [build ifm3d from source instructions](https://api.ifm3d.com/html/content/installation_instructions/install_from_source_linux.html)


## Step-by-Step build instructions for the ROS node `ifm3d-ros2`

Building and installing ifm3d-ros2 is accomplished by utilizing the ROS2 [colcon](https://colcon.readthedocs.io/en/released/) tool.
There are many tutorials and other pieces of advice available online advising how to most effectively utilize it.

### 1. Installation directory of ROS node
First, we need to decide where we want our software to be installed. For purposes of this document, we will assume that we will install our ROS packages at `~/colcon_ws/src`.

>NOTE: Below we assume `humble`. Adapting to other ROS distributions is left as an exercise for the reader.

### 2. create and initialize your colcon workspace
Next, we want to create a _colcon workspace_ that we can use to build and install that code from.

```
$ mkdir -p ~/colcon_ws/src
```

### 3. Get the `ifm3d-ros2` wrapper code from GitHub
Next, we need to get the code from GitHub. Please adapt the commands when not following the suggested directory structure: `~/colcon_ws/src/`

```
$ cd ~/colcon_ws/src
$ git clone https://github.com/ifm/ifm3d-ros2.git
$ git checkout dev1.1
```
> Note: the master branch is generally a work in progress.
> We recommend picking a {{ '[tagged released version]({})'.format(ifm3d_ros2_latest_tag_url) }} for your builds, to ensure stability between builds.

### 4. build the ROS node code
Build your workspace:

>NOTE: the `--cmake-args -DBUILD_TESTING=ON` part of the `colcon` command below is not strictly necessary (tests are `ON` by default), however, it is explicit (see: `python3 -mthis`)).
```
$ cd ~/colcon_ws/
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ colcon build --cmake-args -DBUILD_TESTING=OFF
Starting >>> ifm3d_ros2
Finished <<< ifm3d_ros2 [17.6s]

Summary: 1 package finished [17.8s]
```

Tests: They are not functional at the moment. Please check the launch files to see if the ROS node works after building it for now.
```
$ colcon test
$ colcon test-result --all
[ ... output omitted ... ]
```

Now that the package is build, continue following our [README](../README.md) instructions for launching the node(s).
