#!/usr/bin/env bash

# See https://www.reddit.com/r/vim/comments/1b2zer7/lsp_and_clangd_what_if_the_software_is_built_in_a/
# Written with Claude

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="swarm-idf:latest"
NAME="swarm-clangd"
INDEX_VOLUME="swarm-clangd-index"
QUERY_DRIVER='/opt/esp/tools/**/xtensa-esp32-elf*'

die() {
  echo "clangd.sh: $*" >&2
  exit 1
}
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"

is_running() {
  docker ps --filter "name=^/${NAME}$" --filter "status=running" --format '{{.Names}}' |
    grep -qx "$NAME"
}

up() {
  if is_running; then return 0; fi
  docker image inspect "$IMAGE" >/dev/null 2>&1 ||
    die "image $IMAGE missing -- run: scripts/shell.sh build"
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  docker run -d --name "$NAME" \
    -v "$REPO:/workspace" \
    -v "$INDEX_VOLUME:/root/.cache/clangd" \
    -w /workspace \
    --entrypoint sleep \
    "$IMAGE" infinity >/dev/null
}

down() {
  docker rm -f "$NAME" >/dev/null 2>&1 && echo "$NAME removed" || echo "$NAME not present"
}

case "${1:-}" in
up | ensure | start)
  up
  echo "$NAME running"
  exit 0
  ;;
down | stop)
  down
  exit 0
  ;;
restart)
  down
  up
  echo "$NAME running"
  exit 0
  ;;
status)
  is_running && echo "$NAME: running" || echo "$NAME: stopped"
  exit 0
  ;;
logs) exec docker logs "${@:2}" "$NAME" ;;
shell)
  up
  exec docker exec -it "$NAME" bash
  ;;
esac

up
exec docker exec -i "$NAME" \
  clangd \
  --query-driver="$QUERY_DRIVER" \
  --path-mappings="${REPO}=/workspace" \
  --background-index \
  --pch-storage=memory \
  --header-insertion=never \
  "$@"
