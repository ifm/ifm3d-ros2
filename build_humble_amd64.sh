#!/bin/bash
set -euo pipefail

# AMD64
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
IFM3D_TAG="v1.2.4"
BASE_IMAGE_TAG="humble"
BASE_IMAGE="ros"
FINAL_IMAGE_TAG="humble-ros-core"
TAG=ifm3d-ros:humble-x86_64_latest
ROS2_COMMIT="HEAD"

docker build --no-cache -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_TAG=${IFM3D_TAG} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BASE_IMAGE_TAG=${BASE_IMAGE_TAG} --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} -f Dockerfile .


# ARM64V8
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
IFM3D_TAG="v1.2.4"
BASE_IMAGE_TAG="humble"
BASE_IMAGE="arm64v8/ros"
FINAL_IMAGE_TAG="humble-ros-core"
TAG=ifm3d-ros:humbke-arm64_v8_latest
ROS2_COMMIT="HEAD"

docker build --no-cache -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_TAG=${IFM3D_TAG} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BASE_IMAGE_TAG=${BASE_IMAGE_TAG} --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} -f Dockerfile .
