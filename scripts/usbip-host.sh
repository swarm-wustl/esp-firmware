#!/usr/bin/env bash

# Written with Claude

# Forwards the ESP32's USB serial port into Docker over USB/IP so idf.py can
# flash and monitor from inside the container. Run once per host boot; it's
# idempotent, so re-running while already forwarded is a no-op.
#
#   pyusbip (host, exports the CP210x on :3240)
#     -> devmgr container (nsenter into the Docker VM, runs `usbip attach`)
#       -> /dev/ttyUSB0 in the shared VM kernel, usable by any container

set -euo pipefail

PYUSBIP_DIR="${PYUSBIP_DIR:-$HOME/pyusbip}"
CP210X="10c4:ea60"
REMOTE="host.docker.internal"
DEVMGR_IMAGE="jonathanberi/devmgr"

vm() { docker exec devmgr nsenter -t 1 -m "$@"; }

# Set when pyusbip is (re)started this run. A restart silently kills the TCP
# link behind any existing VM attach, leaving a dead /dev/ttyUSB0 that hangs
# esptool -- so a fresh pyusbip means we must detach and re-attach.
pyusbip_fresh=0

start_pyusbip() {
  if lsof -nP -iTCP:3240 -sTCP:LISTEN >/dev/null 2>&1; then
    return
  fi
  echo "[usbip] starting pyusbip"
  ( cd "$PYUSBIP_DIR" && source .venv/bin/activate \
      && nohup python pyusbip.py >/tmp/pyusbip.log 2>&1 & )
  sleep 2
  lsof -nP -iTCP:3240 -sTCP:LISTEN >/dev/null 2>&1 \
    || { echo "[usbip] pyusbip failed to start; see /tmp/pyusbip.log" >&2; exit 1; }
  pyusbip_fresh=1
}

start_devmgr() {
  if [ "$(docker inspect -f '{{.State.Running}}' devmgr 2>/dev/null)" = "true" ]; then
    return
  fi
  docker rm -f devmgr >/dev/null 2>&1 || true
  echo "[usbip] starting devmgr"
  docker run -d --privileged --pid=host --name devmgr \
    --restart unless-stopped "$DEVMGR_IMAGE" \
    nsenter -t 1 -m sleep infinity >/dev/null
}

detach_stale() {
  local port
  port=$(vm usbip port 2>/dev/null | grep -B1 "$CP210X" \
    | grep -oE "Port [0-9]+" | grep -oE "[0-9]+" | head -1 || true)
  if [ -n "$port" ]; then
    echo "[usbip] detaching stale port $port"
    vm usbip detach -p "$port" || true
  fi
}

attach() {
  vm modprobe vhci_hcd
  if [ "$pyusbip_fresh" -eq 0 ] && vm usbip port 2>/dev/null | grep -q "$CP210X"; then
    echo "[usbip] already attached"
    return
  fi
  detach_stale
  local busid
  busid=$(vm usbip list -r "$REMOTE" 2>/dev/null \
    | grep "$CP210X" | head -1 | awk -F: '{gsub(/ /,"",$1); print $1}' || true)
  if [ -z "$busid" ]; then
    echo "[usbip] CP210x ($CP210X) not exported -- is the board plugged in?" >&2
    exit 1
  fi
  echo "[usbip] attaching bus $busid"
  vm usbip attach -r "$REMOTE" -b "$busid"
  sleep 1
}

start_pyusbip
start_devmgr
attach

if vm ls /dev/ttyUSB0 >/dev/null 2>&1; then
  echo "[usbip] ready: /dev/ttyUSB0"
else
  echo "[usbip] attached but /dev/ttyUSB0 missing -- check 'docker exec devmgr nsenter -t 1 -m dmesg'" >&2
  exit 1
fi
