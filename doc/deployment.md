# How to run ifm3d-ros2 from inside a Docker container

Follow these steps to get our supplied Docker container to run on your system.
:::{note}
The instructions below apply to ROS2 Humble. Please change the commands to suit your ROS 2 distribution.
:::

## Build the docker container

We provide a Dockerfile and a build script to help you build a docker container with `ifm3d` and `ifm3d-ros2`. To get started, check out the [`build_container.sh`](../build_container.sh) script. Open it up and adjust the arguments to suit your target architecture (arm64v8 or amd64) and the targeted `ifm3d` and `ifm3d-ros2` version. Then you can build the container:

```bash
$ ./build_container.sh
[+] Building 675.3s (23/23) FINISHED
 => [internal] load build definition from Dockerfile                                                                                                                    0.0s
 => => transferring dockerfile: 2.92kB                                                                                                                                  0.0s
 => [internal] load .dockerignore                                                                                                                                       0.0s
 => => transferring context: 2B                                                                                                                                         0.0s
 => [internal] load metadata for docker.io/arm64v8/ros:humble-ros-core                                                                                                  1.1s
 => [internal] load metadata for docker.io/arm64v8/ros:humble                                                                                                           0.9s
 => [auth] arm64v8/ros:pull token for registry-1.docker.io                                                                                                              0.0s
 => CACHED [build 1/9] FROM docker.io/arm64v8/ros:humble@sha256:13eed2b61402a7be4dcfb1463398966f27fe807e81e447456d167627ce9ee8ee                                        0.0s
 => CACHED https://github.com/ifm/ifm3d/releases/download/v1.2.6/ifm3d-ubuntu-22.04-arm64-debs_1.2.6.tar                                                                0.6s
 => CACHED [stage-1 1/7] FROM docker.io/arm64v8/ros:humble-ros-core@sha256:b58d5d27371fac49e2d50649cb37effa390a3f854a473689a333c44e64a66f81                             0.0s
 => [build 2/9] RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm                                                                         0.5s
 => [build 3/9] WORKDIR /home/ifm                                                                                                                                       0.0s
 => [build 4/9] RUN apt-get update && apt-get install -y     git     jq     libxmlrpc-c++8-dev     libproj-dev     build-essential     coreutils     cmake     wget   172.1s
 => [build 5/9] RUN mkdir /home/ifm/ifm3d                                                                                                                               0.5s
 => [build 6/9] ADD https://github.com/ifm/ifm3d/releases/download/v1.2.6/ifm3d-ubuntu-22.04-arm64-debs_1.2.6.tar /home/ifm/ifm3d                                       0.0s
 => [build 7/9] RUN cd /home/ifm/ifm3d &&    tar -xf ifm3d-ubuntu-22.04-arm64-debs_1.2.6.tar &&      dpkg -i *.deb                                                      2.1s
 => [build 8/9] RUN mkdir -p /home/ifm/colcon_ws/src &&     cd /home/ifm/colcon_ws/src &&     git clone https://github.com/ifm/ifm3d-ros2.git -b lm_humble_tests --sin  4.2s
 => [build 9/9] RUN cd /home/ifm/colcon_ws &&     source /opt/ros/humble/setup.bash &&     colcon build --cmake-  242.8s
 => [stage-1 2/7] COPY --from=build /home/ifm/colcon_ws /home/ifm/colcon_ws                                                                                             0.1s
 => [stage-1 3/7] COPY --from=build /home/ifm/ifm3d/*.deb /home/ifm/ifm3d/                                                                                              0.0s
 => [stage-1 4/7] WORKDIR /home/ifm                                                                                                                                     0.0s
 => [stage-1 5/7] RUN apt-get update     && apt-get install -y --no-install-recommends     libxmlrpc-c++8v5     locales     sudo     libssl-dev     libgoogle-glog0v  238.2s
 => [stage-1 6/7] RUN cd /home/ifm/ifm3d &&    dpkg -i *.deb                                                                                                            2.1s
 => [stage-1 7/7] RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen &&     locale-gen en_US.UTF-8 &&     /usr/sbin/update-locale LANG=en_US.UTF-8                         7.1s
 => exporting to image                                                                                                                                                  3.3s
 => => exporting layers                                                                                                                                                 3.3s
 => => writing image sha256:c1d72f3316c084ee0bb759c9aa9bdfdd64bcdcff33a3011efeadcf6f38815160                                                                            0.0s
 => => naming to docker.io/library/ifm3d-ros:humble-arm64_v8
```
## Deploy the container
To deploy the container on to the VPU, or use the container locally to interact with the O3R platform, please refer to our [docker documentation](https://ifm3d.com/latest/SoftwareInterfaces/Docker/index_docker.html).

## Resource management
Resources on the OVP8xx are limited and shared between all the running processes. We recommend assigning the Docker process to specific cores so as not to interfere with other applications. Refer to [the resource management documentation on ifm3d.com](https://ifm3d.com/latest/SoftwareInterfaces/Docker/cpu.html).

## Distributed setup

It is possible to run a complete ROS system in a distributed way. In this section we provide instructions to run ifm3d-ros2 in a container deployed on the VPU (primary container), and the visualization locally on a laptop (secondary system).
These instructions can be adapted to suit other architectural designs.

### Primary docker container: ifm3d-ros2 node

1. Build and deploy the docker container image to the VPU (see [the documentation on how to deploy a container](https://ifm3d.com/latest/SoftwareInterfaces/Docker/deployVPU.html)):
```bash
$ docker save docker.io/library/ifm3d-ros:humble-arm64_v8 | ssh -C oem@192.168.0.69 docker load
```

- SSH to the VPU and run the container:
- 
```bash
$ ssh oem@192.168.0.69 #Adapt to the IP address of your VPU
o3r-vpu-c0$ docker run -ti --net=host ifm3d-ros:humble-arm64_v8
root@952330b98eac:/home/ifm#
```
-  Source ROS2 and the `ìfm3d_ros2` installation:
```bash
root@952330b98eac:/home/ifm# source /opt/ros/humble/setup.bash && source colcon_ws/install/setup.bash
```
- All ROS nodes should see each other as long as they are on the same ROS domain ID. The default `ROS_DOMAIN_ID` is 0 and doesn't get changed here.
- Run the ros2 node:
```bash
$ ros2 launch ifm3d_ros2 camera.launch.py
```

:::{note}
The command above uses the default configuration defined in the `/config/camera_default_parameters.yaml` file. You can adapt the parameters by using a different config file:
```bash
root@62b0c2e120bb:/home/ifm/$ ros2 launch ifm3d_ros2 camera.launch.py  parameter_file_directory:=config/examples parameter_file_name:=o3r_3d.yaml camera_name:=camera_3d
```
:::

## Second ROS2 client: plain ros2

1. Source ROS2 on the secondary machine:
```bash
$ source /opt/ros/humble/setup.bash
```
- Check that ROS topics are available (on `ROS_DOMAIN_ID=0`):
```bash
$ ros2 topic list
/ifm3d/camera/amplitude
/ifm3d/camera/cloud
/ifm3d/camera/confidence
/ifm3d/camera/distance
/ifm3d/camera/extrinsics
/ifm3d/camera/raw_amplitude
/ifm3d/camera/transition_event
/parameter_events
/rosout
/tf_static
```
- Start the visualization and subscribe to the relevant topic:
```bash
$ rviz2
```

:::{tip}
To use Rviz2 from a docker container, the container has to be started with a specific environment:
```bash
$ docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY --env=QT_X11_NO_MITSHM=1 --net=host --privileged ros:humble-ros-core
```
:::