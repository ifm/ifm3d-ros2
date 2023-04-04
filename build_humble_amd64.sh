#!/bin/bash
set -euo pipefail

# HEAD
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
BASE_IMAGE_TAG="humble"
BASE_IMAGE="ros"
FINAL_IMAGE_TAG="humble-ros-core"
TAG=ifm3d-ros:humble-x86_64_latest
ROS2_COMMIT="HEAD"

docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg ROS2_COMMIT=${ROS2_COMMIT}\
    --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BASE_IMAGE_TAG=${BASE_IMAGE_TAG} --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} -f Dockerfile .



# # v1.0.1
# IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
# IFM3D_TAG="v1.0.1"
# BASE_IMAGE_TAG="humble"
# BASE_IMAGE="ros"
# FINAL_IMAGE_TAG="humble-ros-core"
# TAG=ifm3d-ros:humble-x86_64_v101
# ROS2_COMMIT="5569964ee707efe48843bedf52dd632d568f8359"

# docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg ROS2_COMMIT=${ROS2_COMMIT} --build-arg IFM3D_TAG=${IFM3D_TAG}\
#     --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BASE_IMAGE_TAG=${BASE_IMAGE_TAG} --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} -f Dockerfile .


# # v1.1.1
# IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
# IFM3D_TAG="v1.1.1"
# BASE_IMAGE_TAG="humble"
# BASE_IMAGE="ros"
# FINAL_IMAGE_TAG="humble-ros-core"
# TAG=ifm3d-ros:humble-x86_64_v111
# ROS2_COMMIT="e315ec84ce6a7161ab0f2e0355f035f90ebb8ddb"

# docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg ROS2_COMMIT=${ROS2_COMMIT} --build-arg IFM3D_TAG=${IFM3D_TAG}\
#     --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg BASE_IMAGE_TAG=${BASE_IMAGE_TAG} --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} -f Dockerfile .
