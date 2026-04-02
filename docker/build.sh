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

echo "=== Build complete ==="
