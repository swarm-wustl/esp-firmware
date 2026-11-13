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

# robots are put down in a row along map's y, so they don't start on top of
# each other
SPACING=0.5

# UWB anchors as name:x,y in map. This is the only place they are placed:
# each becomes a static TF frame, the sim ranges whatever frames it is named,
# and host/uwb_fix.py looks an anchor up by the frame its range carries
ANCHORS=(anchor_0:-1.5,-1.5 anchor_1:2.5,-1.5 anchor_2:2.5,2.5 anchor_3:-1.5,2.5)
# what the sim adds to each range and what the fix node assumes it adds
RANGE_NOISE=0.05

anchors=""
anchor_names=""
for anchor in "${ANCHORS[@]}"; do
  name="${anchor%%:*}"
  IFS=, read -r ax ay <<<"${anchor#*:}"
  anchors+="ros2 run tf2_ros static_transform_publisher --x $ax --y $ay \
    --frame-id map --child-frame-id $name >/dev/null 2>&1 & "
  anchor_names+="${anchor_names:+,}$name"
done

# each robot gets its own local EKF (sim/ekf.yaml), the filter that will run
# against the real board too. Frames are <ns>/odom and <ns>/base_link, as
# Robot::frame_in builds them on the board, so the TF trees stay apart
robots=""
i=0
for ns in "${namespaces[@]}"; do
  # always with a decimal point: ROS rejects [0.0,0,0.0] as a mixed list
  y="$(awk "BEGIN{printf \"%.3f\", $i * $SPACING}")"

  robots+="build_sim/sim_base --ros-args -r __ns:=/$ns \
    -p start:=[0.0,$y,0.0] -p anchors:=[$anchor_names] \
    -p range_noise:=$RANGE_NOISE & "
  robots+="ros2 run robot_localization ekf_node --ros-args -r __ns:=/$ns \
    --params-file sim/ekf.yaml -p odom_frame:=$ns/odom \
    -p world_frame:=$ns/odom -p base_link_frame:=$ns/base_link & "
  robots+="python3 host/uwb_fix.py --ros-args -r __ns:=/$ns \
    -p range_sigma:=$RANGE_NOISE & "
  # the global filter starts where the robot was put down, which a real robot
  # would have to estimate rather than know. Its own node name and output
  # topic keep it apart from the local filter in the same namespace
  robots+="ros2 run robot_localization ekf_node --ros-args -r __ns:=/$ns \
    -r __node:=ekf_global -r odometry/filtered:=odometry/global \
    --params-file sim/ekf.yaml --params-file sim/ekf_global.yaml \
    -p map_frame:=map -p world_frame:=map -p odom_frame:=$ns/odom \
    -p base_link_frame:=$ns/base_link \
    -p initial_state:=[0.0,$y,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] & "
  # the 3D panel's pose tool publishes on /move_base_simple/goal, so a click
  # sends every robot there
  robots+="python3 host/go_to_goal.py --ros-args -r __ns:=/$ns \
    -r goal:=/move_base_simple/goal & "

  i=$((i + 1))
done

echo "sim.sh: ${namespaces[*]} up; open Foxglove on ws://localhost:8765" >&2

exec docker run --rm -it --init \
  --name "$NAME" \
  -p 8765:8765 \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -v "$REPO:/workspace" -w /workspace \
  "$IMAGE" bash -lc ". /opt/ros/humble/setup.bash && \
    cmake -S sim -B build_sim >/dev/null && cmake --build build_sim -j || exit 1
    $anchors $robots \
    exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765"
