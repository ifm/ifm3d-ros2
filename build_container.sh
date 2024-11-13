#!/bin/bash
set -euo pipefail

##############
# For AMD64:
##############
ARCH="amd64"
BASE_IMAGE="ros"
TAG=ifm3d-ros:jazzy-amd64

##############
# For ARM64V8:
##############
#ARCH="arm64"
#BASE_IMAGE="arm64v8/ros"
#TAG=ifm3d-ros:jazzy-arm64_v8

##############
# Arguments common for both architecture
##############
BUILD_IMAGE_TAG="jazzy"
FINAL_IMAGE_TAG="jazzy-ros-core"
IFM3D_VERSION="1.5.3"
IFM3D_ROS2_REPO="https://github.com/ifm/ifm3d-ros2.git"
IFM3D_ROS2_BRANCH="v1.2.0"
UBUNTU_VERSION="22.04"

docker build -t $TAG \
    --build-arg ARCH=${ARCH} \
    --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg BUILD_IMAGE_TAG=${BUILD_IMAGE_TAG} \
    --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} \
    --build-arg IFM3D_VERSION=${IFM3D_VERSION} \
    --build-arg IFM3D_ROS2_REPO=${IFM3D_ROS2_REPO} \
    --build-arg IFM3D_ROS2_BRANCH=${IFM3D_ROS2_BRANCH} \
    -f Dockerfile .
