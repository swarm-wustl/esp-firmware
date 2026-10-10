#!/usr/bin/env bash


# Runs the micro-ROS agent the board's XRCE session dials, with the port read
# out of sdkconfig so firmware and agent can't disagree.
#
#   scripts/agent.sh            agent in the foreground (ctrl-c to stop)
#   scripts/agent.sh detach     same, in the background as `swarm-agent`
#   scripts/agent.sh stop       stop the background agent
#   scripts/agent.sh status     whether it is up, and on which port
#   scripts/agent.sh port       print the configured port
#
# Anything else is passed through to micro_ros_agent, e.g. `scripts/agent.sh -v6`.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="microros/micro-ros-agent:humble"
NAME="swarm-agent"

die() { echo "agent.sh: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"
docker info >/dev/null 2>&1 || die "Docker daemon is not running"

PORT="$(sed -n 's/^CONFIG_MICRO_ROS_AGENT_PORT="\{0,1\}\([^"]*\)"\{0,1\}$/\1/p' \
  "$REPO/sdkconfig" | tail -1)"
[ -n "$PORT" ] ||
  die "no CONFIG_MICRO_ROS_AGENT_PORT in sdkconfig -- run idf.py menuconfig"

running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]
}

detach=0
case "${1:-}" in
  port)   echo "$PORT"; exit 0 ;;
  stop)   docker rm -f "$NAME" >/dev/null 2>&1 || true; exit 0 ;;
  status) running && echo "running on udp/$PORT" || echo "not running"; exit 0 ;;
  detach) detach=1; shift ;;
esac

if [ "$detach" -eq 1 ] && running; then
  exit 0
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true

# host-gateway lets anything inside the container reach this machine; the board
# itself arrives from the LAN on the published UDP port
mode=(--rm -it)
if [ "$detach" -eq 1 ]; then
  mode=(-d)
fi

cmd=(docker run "${mode[@]}"
     --name "$NAME"
     -p "$PORT:$PORT/udp"
     --add-host=host.docker.internal:host-gateway
     "$IMAGE" udp4 --port "$PORT" "$@")

if [ "$detach" -eq 1 ]; then
  "${cmd[@]}" >/dev/null
  echo "agent.sh: listening on udp/$PORT (docker logs -f $NAME)" >&2
  exit 0
fi

exec "${cmd[@]}"
