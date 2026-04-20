#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ur5-control-lab"
EXPORTS_DIR="$ROOT_DIR/exports"
CONTAINER_EXPORTS_DIR="/root/ur5_gazebo/exports"

cleanup() {
    xhost -local:root >/dev/null 2>&1 || true
}

trap cleanup EXIT

mkdir -p "$EXPORTS_DIR/experiments"

echo "[1/3] Building Docker image..."
docker build -t "$IMAGE_NAME" .

echo "[2/3] Configuring X11..."
xhost +local:root >/dev/null

echo "[3/3] Starting container with the default entrypoint..."
# The container entrypoint launches rosrun ur5_control_lab main.py after Gazebo is ready.
docker run -it --rm \
    --net=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$EXPORTS_DIR:$CONTAINER_EXPORTS_DIR" \
    --name ur5_lab_container \
    "$IMAGE_NAME"
