#!/bin/bash
set -euo pipefail

##############
# Toggle target architecture by (un)commenting a block
##############

##############
# For AMD64 (default)
##############
#ARCH="amd64"
#BASE_IMAGE="ros"            # Official multi-arch ROS image
#TAG=ifm3d-ros2:humble-amd64

##############
# For ARM64 (native arm64 or cross from amd64 with CROSS=1)
##############
ARCH="arm64"
BASE_IMAGE="ros"              # Use multi-arch image
TAG=ifm3d-ros2:humble-arm64

##############
# Common arguments
##############
BUILD_IMAGE_TAG="humble"
FINAL_IMAGE_TAG="humble-ros-core"
IFM3D_VERSION="1.6.12"
IFM3D_ROS2_REPO="https://github.com/ifm/ifm3d-ros2.git"
IFM3D_ROS2_BRANCH="v1.3.0"
UBUNTU_VERSION="22.04"

# Optional: cross-build. Use only if host != target.
# Examples:
#  Native amd64 build: (uncomment amd64 block) ./build_container.sh
#  Native arm64 build: (uncomment arm64 block) ./build_container.sh
#  Cross amd64 -> arm64: (arm64 block active) CROSS=1 ./build_container.sh
#  Cross arm64 -> amd64: (amd64 block active) CROSS=1 ./build_container.sh
if [[ "${CROSS:-0}" == "1" ]]; then
  HOST_ARCH_RAW=$(uname -m)
  case "$HOST_ARCH_RAW" in
    x86_64) HOST_ARCH="amd64" ;;
    aarch64|arm64) HOST_ARCH="arm64" ;;
    *) HOST_ARCH="$HOST_ARCH_RAW" ;;
  esac
  if [[ "$ARCH" == "arm64" ]]; then PLATFORM="linux/arm64"; else PLATFORM="linux/amd64"; fi
  if [[ "$HOST_ARCH" == "$ARCH" ]]; then
    echo "[INFO] Host arch already matches target ($ARCH); CROSS=1 not required but proceeding with buildx." >&2
  else
    echo "[INFO] Cross-building $HOST_ARCH -> $ARCH using buildx." >&2
  fi
  docker buildx create --name ifm_simple --use >/dev/null 2>&1 || docker buildx use ifm_simple
  docker run --rm --privileged tonistiigi/binfmt --install all >/dev/null 2>&1 || true
  docker buildx build --platform ${PLATFORM} -t $TAG \
    --build-arg ARCH=${ARCH} \
    --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg BUILD_IMAGE_TAG=${BUILD_IMAGE_TAG} \
    --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} \
    --build-arg IFM3D_VERSION=${IFM3D_VERSION} \
    --build-arg IFM3D_ROS2_REPO=${IFM3D_ROS2_REPO} \
    --build-arg IFM3D_ROS2_BRANCH=${IFM3D_ROS2_BRANCH} \
    -f Dockerfile --load .
  echo "[DONE] Image built (buildx) $TAG"
  exit 0
fi

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

echo "[DONE] Image built $TAG"
