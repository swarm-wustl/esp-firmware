#!/usr/bin/env bash


# Drives /cmd_vel from the keyboard, through the micro-ROS agent the board
# connects to over UDP.
#
#   scripts/teleop.sh           start the agent if needed, then teleop
#   scripts/teleop.sh agent     agent only, in the foreground (logs)
#   scripts/teleop.sh stop      stop the agent container
#   scripts/teleop.sh build     (re)build the teleop image
#
# The topic and port come out of sdkconfig, so the board and the keyboard can't
# drift apart.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="swarm-teleop:latest"
DOCKERFILE="$REPO/docker/teleop/Dockerfile"
AGENT_IMAGE="microros/micro-ros-agent:humble"
AGENT="swarm-agent"

die() { echo "teleop.sh: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"
docker info >/dev/null 2>&1 || die "Docker daemon is not running"

sdkconfig() {
  sed -n "s/^$1=\"\{0,1\}\([^\"]*\)\"\{0,1\}$/\1/p" "$REPO/sdkconfig" | tail -1
}

PORT="$(sdkconfig CONFIG_MICRO_ROS_AGENT_PORT)"
NAMESPACE="$(sdkconfig CONFIG_SWARM_ROS_NAMESPACE)"
[ -n "$PORT" ] || die "no CONFIG_MICRO_ROS_AGENT_PORT in sdkconfig -- run idf.py menuconfig"
[ -n "$NAMESPACE" ] || die "no CONFIG_SWARM_ROS_NAMESPACE in sdkconfig"
TOPIC="/$NAMESPACE/cmd_vel"

build_image() {
  docker build "$@" -t "$IMAGE" -f "$DOCKERFILE" "$(dirname "$DOCKERFILE")"
}

agent_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$AGENT" 2>/dev/null)" = "true" ]
}

start_agent() {
  docker rm -f "$AGENT" >/dev/null 2>&1 || true
  docker run -d --name "$AGENT" -p "$PORT:$PORT/udp" \
    "$AGENT_IMAGE" udp4 --port "$PORT" -v4 >/dev/null
  echo "teleop.sh: agent listening on udp/$PORT (docker logs -f $AGENT)" >&2
}

case "${1:-}" in
  build)
    build_image --no-cache
    exit 0
    ;;
  stop)
    docker rm -f "$AGENT" >/dev/null 2>&1 || true
    exit 0
    ;;
  agent)
    docker rm -f "$AGENT" >/dev/null 2>&1 || true
    exec docker run --rm -it --name "$AGENT" -p "$PORT:$PORT/udp" \
      "$AGENT_IMAGE" udp4 --port "$PORT" -v6
    ;;
esac

docker image inspect "$IMAGE" >/dev/null 2>&1 || build_image

agent_running || start_agent

# the agent's network namespace, not a bridge: XRCE-mapped topics are only
# discoverable from inside it
exec docker run --rm -it --network "container:$AGENT" "$IMAGE" bash -lc \
  ". /opt/ros/humble/setup.bash && exec ros2 run teleop_twist_keyboard \
   teleop_twist_keyboard --ros-args -r /cmd_vel:=$TOPIC"
