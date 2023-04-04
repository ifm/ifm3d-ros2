ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
ARG FINAL_IMAGE_TAG
ARG IFM3D_TAG

ARG ROS2_COMMIT

FROM ${BASE_IMAGE}:${BASE_IMAGE_TAG} AS base


# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
WORKDIR /home/ifm

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    jq \
    libcurl4-openssl-dev \
    libgtest-dev \
    libxmlrpc-c++8-dev \
    libproj-dev \
    build-essential \
    coreutils \
    cmake \
    ninja-build \
    wget \
    libssl-dev\
    libboost-all-dev

RUN apt-get clean


# # Install cmake
# RUN wget -O - "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -i).tar.gz" \
#     | tar -xz --strip-components=1 -C /usr


# clone and install ifm3d frmom tag
ARG IFM3D_CLONE_REPO

# | Flag name | Description | Default value |
# | --------- | ----------- | ------------- |
# | BUILD_MODULE_FRAMEGRABBER | Build the framegrabber module | ON |
# | BUILD_MODULE_STLIMAGE | Build the stl image module (Only relies on standard c++ libraries) | OFF |
# | BUILD_MODULE_IMAGE **DEPRECATED**| Build the image module (Depends on OpenCV and PCL) | OFF |
# | BUILD_MODULE_OPENCV **DEPRECATED**| Build the OpenCV-only image container | OFF |
# | BUILD_MODULE_TOOLS | Build the command-line utility | ON |
# | BUILD_IN_DEPS | Download and build dependencies | ON |
# | BUILD_MODULE_PYBIND11 | Build the ifm3dpy python package (it can also be installed directly through `pip`) | OFF |
# | USE_LEGACY_COORDINATES | Use the legacy coordinates (ifm3d <= 0.92.x) with swapped axis | OFF |
# | BUILD_MODULE_SWUPDATER | Build the swupdater module | ON |
# | BUILD_SDK_PKG | Build install packages for development purposes | ON |
# | FORCE_OPENCV3 | Force the build to require OpenCV 3 | OFF |
# | FORCE_OPENCV2 | Force the build to require OpenCV 2.4 | OFF |
# | BUILD_SHARED_LIBS | Build modules as shared libraries | ON |
# | BUILD_EXAMPLES | Build the examples | OFF |
# | BUILD_DOC | Build documentation | OFF |
# | BUILD_TESTS | Build unit tests | ON
# | BUILD_MODULE_PCICCLIENT | Build the pcicclient module | OFF |

RUN cd /home/ifm/ \
    && git clone ${IFM3D_CLONE_REPO} ifm3d \
    && cd ifm3d git checkout ${IFM3D_TAG} \
    && mkdir -p /home/ifm/ifm3d/build \
    && cd /home/ifm/ifm3d/build \
    && cmake -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/install \
    -DBUILD_MODULE_OPENCV=OFF \
    -DBUILD_MODULE_PCICCLIENT=ON \
    -DBUILD_MODULE_PYBIND11=OFF \
    -DBUILD_MODULE_TOOLS=ON\
    -DBUILD_MODULE_SWUPDATER=OFF\
    -DBUILD_SDK_PKG=ON\
    -DBUILD_EXAMPLES=OFF\
    -DBUILD_TESTS=OFF\
    .. \
    && cmake --build . \
    && cmake --build . --target install

RUN cp -r /install/* /usr

# Install ROS2 dds settings and runtime dependencies
RUN apt-get update && apt-get install -y  \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-numpy


RUN apt-get install nlohmann-json3-dev/jammy

# Initialize colcon workspace
WORKDIR /home/ifm
RUN mkdir -p /home/ifm/colcon_ws/ifm3d-ros2/src \
    && /bin/bash -c 'cd /home/ifm/colcon_ws/ifm3d-ros2; . /opt/ros/${ROS_DISTRO}/setup.bash'

# Clone and build ifm3d-ros2 repo
ARG IFM3D_ROS_CLONE_REPO
ADD . /home/ifm/colcon_ws/ifm3d-ros2/src

# ARG ROS2_COMMIT
# RUN cd /home/ifm/colcon_ws/ifm3d-ros2/src && git checkout ${ROS2_COMMIT}
# RUN sed -i  --expression  's@contrib/nlohmann@device@' /home/ifm/colcon_ws/ifm3d-ros2/src/src/lib/camera_node.cpp


RUN /bin/bash -c 'cd /home/ifm/colcon_ws/ifm3d-ros2/; . /opt/ros/${ROS_DISTRO}/setup.bash; ifm3d_DIR=/usr/lib/cmake; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; colcon build --cmake-args -DBUILD_TESTING=OFF'


# multistage
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
ARG FINAL_IMAGE_TAG

FROM ${BASE_IMAGE}:${BASE_IMAGE_TAG}

ARG DEBIAN_FRONTEND=noninteractive
COPY --from=base /install /usr
COPY --from=base /home/ifm/colcon_ws/ifm3d-ros2/ /home/ifm/colcon_ws/ifm3d-ros2/

WORKDIR /home/ifm

# install ifm3d dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libxmlrpc-c++8v5 \
    locales \
    sudo \
    libssl-dev \
    libboost-all-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Create the rosuser user
RUN id rosuser 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U rosuser
RUN echo "rosuser ALL=(ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/rosuser

USER rosuser

# # NOTE: when NOT using the supplied entrypoint.sh script please manually: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ADD ros-entrypoint.sh /ros-entrypoint.sh
# ENTRYPOINT [ "/ros-entrypoint.sh" ]