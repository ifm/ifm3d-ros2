ARG ARCH="amd64"
ARG BASE_IMAGE="arm64v8/ros"
ARG BUILD_IMAGE_TAG="humble"
ARG FINAL_IMAGE_TAG="humble-ros-core"
ARG IFM3D_ROS2_BRANCH="master"
ARG IFM3D_ROS2_REPO="https://github.com/ifm/ifm3d-ros2.git"
ARG IFM3D_VERSION="1.2.6"
ARG UBUNTU_VERSION="22.04"

FROM ${BASE_IMAGE}:${BUILD_IMAGE_TAG} AS build
ARG ARCH
ARG IFM3D_ROS2_BRANCH
ARG IFM3D_ROS2_REPO
ARG IFM3D_VERSION
ARG UBUNTU_VERSION

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
WORKDIR /home/ifm

# Dependencies for both ifm3d and ifm3d-ros2
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    coreutils \
    git \
    jq \
    libboost-all-dev \    
    libgoogle-glog-dev \
    libgoogle-glog0v5 \
    libproj-dev \
    libssl-dev \
    libxmlrpc-c++8-dev \
    wget 

# Install ifm3d using the deb files
RUN mkdir /home/ifm/ifm3d
ADD https://github.com/ifm/ifm3d/releases/download/v${IFM3D_VERSION}/ifm3d-ubuntu-${UBUNTU_VERSION}-${ARCH}-debs_${IFM3D_VERSION}.tar /home/ifm/ifm3d
RUN cd /home/ifm/ifm3d &&\
    tar -xf ifm3d-ubuntu-${UBUNTU_VERSION}-${ARCH}-debs_${IFM3D_VERSION}.tar &&  \
    dpkg -i *.deb

ADD . /home/ifm/colcon_ws/src/ifm3d-ros2
RUN cd /home/ifm/colcon_ws && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-path src -y --ignore-src -t build

# # Clone and build ifm3d-ros2 repo
# RUN mkdir -p /home/ifm/colcon_ws/src && \
#     cd /home/ifm/colcon_ws/src && \
#     git clone ${IFM3D_ROS2_REPO} -b ${IFM3D_ROS2_BRANCH} --single-branch 

SHELL ["/bin/bash", "-c"]
RUN cd /home/ifm/colcon_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DBUILD_TESTING=OFF

# Multistage build to reduce image size
ARG BASE_IMAGE
FROM ${BASE_IMAGE}:${FINAL_IMAGE_TAG}
# Copy files built in previous stage
COPY --from=build /home/ifm/colcon_ws /home/ifm/colcon_ws
COPY --from=build /home/ifm/ifm3d/*.deb /home/ifm/ifm3d/
WORKDIR /home/ifm

# Install ifm3d and ifm3d-ros2 runtime dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libgoogle-glog0v5 \    
    libssl-dev \
    libxmlrpc-c++8v5 \
    locales \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install ifm3d
RUN cd /home/ifm/ifm3d &&\
    dpkg -i *.deb

RUN cd /home/ifm/colcon_ws && \
    apt-get update &&\
    rosdep init && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-path src -y --ignore-src -t exec


# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8