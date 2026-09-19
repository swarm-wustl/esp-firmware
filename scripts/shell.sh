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
#
# Seeds micro-ROS from microros.lock first, so every peer builds the same
# sources -- see scripts/microros-pin.sh.

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

# an image predating the empy pin still builds, but micro-ROS codegen dies on
# em.BUFFERED_OPT much later and the error says nothing about the image
check_image_deps() {
  local ver
  ver="$(docker run --rm "$IMAGE" bash -lc \
    'python -c "import em; print(em.__version__)"' 2>/dev/null | tail -1 | tr -d "\r")" || return 0
  case "$ver" in
    3.*) return 0 ;;
    "")  return 0 ;;
    *)   echo "shell.sh: $IMAGE has empy $ver -- micro-ROS needs 3.x." >&2
         echo "shell.sh: rebuild it with: scripts/shell.sh build" >&2
         return 1 ;;
  esac
}

# colcon lives in the container's IDF python env, so the dev workspace has to be
# built in there rather than on the host
seed_microros() {
  [ -f "$REPO/microros.lock" ] || {
    echo "shell.sh: no microros.lock -- skipping micro-ROS seed" >&2
    return 0
  }
  docker run --rm -v "$REPO:/workspace" -w /workspace "$IMAGE" \
    bash -lc '. $IDF_PATH/export.sh >/dev/null && scripts/microros-pin.sh seed'
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

check_image_deps || die "stale image -- see above"

seed_microros

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
