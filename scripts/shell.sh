#!/usr/bin/env bash

# Written with Claude

# Drops you into the micro-ROS container with the project mounted at /workspace
# and the ESP32 forwarded over USB/IP, so idf.py build/flash/monitor all work
# from one shell. Builds the image on first use.
#
#   scripts/shell.sh            interactive shell in the container
#   scripts/shell.sh nodevice   same, but skip USB/IP forwarding entirely
#                               (host unit tests / builds with no board plugged in)
#   scripts/shell.sh build      (re)build the image from scratch, then exit

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="swarm-idf:latest"
DOCKERFILE="$REPO/.devcontainer/Dockerfile"
DEVICE="/dev/ttyUSB0"

die() { echo "shell.sh: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"
docker info >/dev/null 2>&1 || die "Docker daemon is not running"

build_image() {
  docker build "$@" -t "$IMAGE" -f "$DOCKERFILE" "$(dirname "$DOCKERFILE")"
}

skip_device=0
case "${1:-}" in
  build)
    build_image --no-cache
    exit 0
    ;;
  nodevice | host)
    skip_device=1
    ;;
esac

docker image inspect "$IMAGE" >/dev/null 2>&1 || build_image

device_args=()
if [ "$skip_device" -eq 1 ]; then
  echo "shell.sh: nodevice -- skipping board forwarding (build/host-test only)" >&2
elif bash "$REPO/scripts/usbip-host.sh"; then
  device_args=(--device "$DEVICE")
else
  echo "shell.sh: board not forwarded -- continuing without $DEVICE (build only)" >&2
fi

exec docker run --rm -it "${device_args[@]+"${device_args[@]}"}" \
  -v "$REPO:/workspace" \
  -w /workspace \
  "$IMAGE" bash
