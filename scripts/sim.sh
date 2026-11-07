#!/usr/bin/env bash


# Runs simulated robots (sim/sim_base.cpp) that speak the firmware's topics,
# plus a Foxglove bridge on ws://localhost:8765 to watch them. The node runs the
# firmware's own control loop and odometry against a simulated plant, built
# fresh into build_sim/ on every start.
#
#   scripts/sim.sh                  one robot, namespaced from sdkconfig
#   scripts/sim.sh swarm_1 swarm_2  one robot per namespace
#   scripts/sim.sh teleop [ns]      drive a robot from the keyboard (2nd terminal)
#   scripts/sim.sh shell            bash in the running sim, for ros2 topic etc.
#   scripts/sim.sh build            (re)build the sim image
#
# Wheel geometry and the default namespace come out of sdkconfig, through the
# same headers the firmware compiles, so the sim can't drift from the board.
#
# The sim runs on its own ROS_DOMAIN_ID (SIM_DOMAIN, default 42): DDS discovery
# crosses containers, so a fake swarm_1 on domain 0 would publish over the
# real board's topics whenever swarm-agent is up. SIM_DOMAIN=0 mixes them.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="swarm-sim:humble"
DOCKERFILE="$REPO/docker/sim/Dockerfile"
NAME="swarm-sim"
DOMAIN="${SIM_DOMAIN:-42}"

die() { echo "sim.sh: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"
docker info >/dev/null 2>&1 || die "Docker daemon is not running"

config() {
  sed -n "s/^CONFIG_$1=\"\{0,1\}\([^\"]*\)\"\{0,1\}\$/\1/p" "$REPO/sdkconfig" |
    tail -1
}

NAMESPACE="$(config SWARM_ROS_NAMESPACE)"
[ -n "$NAMESPACE" ] || die "no CONFIG_SWARM_ROS_NAMESPACE in sdkconfig"

build_image() {
  docker build "$@" -t "$IMAGE" -f "$DOCKERFILE" "$(dirname "$DOCKERFILE")"
}

in_sim() {
  docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null | grep -q true ||
    die "sim is not running -- start it with scripts/sim.sh"
  exec docker exec -it "$NAME" bash -lc ". /opt/ros/humble/setup.bash && $1"
}

case "${1:-}" in
  build)
    build_image --no-cache
    exit 0 ;;
  teleop)
    in_sim "exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r cmd_vel:=/${2:-$NAMESPACE}/cmd_vel" ;;
  shell)
    in_sim "exec bash" ;;
esac

docker image inspect "$IMAGE" >/dev/null 2>&1 || build_image

namespaces=("$@")
[ "${#namespaces[@]}" -gt 0 ] || namespaces=("$NAMESPACE")

robots=""
for ns in "${namespaces[@]}"; do
  robots+="build_sim/sim_base --ros-args -r __ns:=/$ns & "
done

# the 3D panel only offers frames it has seen in TF, and the firmware publishes
# none, so odom would be undisplayable without a tree to hang it from
# TODO: drop once an EKF publishes map -> odom
robots+="ros2 run tf2_ros static_transform_publisher \
  --frame-id map --child-frame-id odom >/dev/null 2>&1 & "

echo "sim.sh: ${namespaces[*]} up; open Foxglove on ws://localhost:8765" >&2

exec docker run --rm -it --init \
  --name "$NAME" \
  -p 8765:8765 \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -v "$REPO:/workspace" -w /workspace \
  "$IMAGE" bash -lc ". /opt/ros/humble/setup.bash && \
    cmake -S sim -B build_sim >/dev/null && cmake --build build_sim -j && \
    $robots \
    exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
