#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="${1:-all}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

echo "=== Building for ROS2 ${ROS_DISTRO} ==="

echo "=== Step 1: Building base image ==="
docker compose build --build-arg ROS_DISTRO="${ROS_DISTRO}" trackdlo-base

if [[ "$TARGET" == "all" || "$TARGET" == "core" ]]; then
  echo "=== Building core image ==="
  docker compose build --build-arg ROS_DISTRO="${ROS_DISTRO}" trackdlo-core
fi

if [[ "$TARGET" == "all" || "$TARGET" == "sam2" ]]; then
  echo "=== Building SAM2 image (CPU) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cpu trackdlo-sam2
fi

if [[ "$TARGET" == "sam2-cuda" ]]; then
  echo "=== Building SAM2 image (CUDA) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cuda trackdlo-sam2
fi

echo "=== Build complete ==="
