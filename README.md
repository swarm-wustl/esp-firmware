# SWARM ESP32 Firmware
Codebase for the ESP32 microcontrollers that power the individual swarm robots.

# Flashing
Run `./flash.sh [USB device]`. The script provides specific instructions on how to properly flash the devices.

# Teleop

`./scripts/agent.sh` runs the micro-ROS agent (`microros/micro-ros-agent:humble`)
on `CONFIG_MICRO_ROS_AGENT_PORT`, read out of `sdkconfig`. The board has to reach
`CONFIG_MICRO_ROS_AGENT_IP` on the LAN, so that must be this machine's address.

- `scripts/agent.sh` -- foreground, ctrl-c to stop; extra args reach
  `micro_ros_agent`, e.g. `scripts/agent.sh -v6`
- `scripts/agent.sh detach` / `stop` / `status` / `port`

`./scripts/teleop.sh` starts the agent if it isn't up, then drops you into
`teleop_twist_keyboard` publishing to `/<CONFIG_SWARM_ROS_NAMESPACE>/cmd_vel`. It
runs in the agent's network namespace -- XRCE-mapped topics aren't discoverable
across a docker bridge. `scripts/teleop.sh build` rebuilds `swarm-teleop:latest`
from `docker/teleop/Dockerfile`.

# Editor / clangd LSP
Always start Neovim from the swarm repo root (`nvim` from this directory), not from a
subfolder or via a file path elsewhere. The clangd LSP is wired up in the project-local
`.nvim.lua`, which Neovim only sources (via `exrc`) when you launch it from the root.
Start it anywhere else and clangd won't run inside the dev container (`scripts/clangd.sh`),
so you'll get broken or missing completions.

# Better Documentation
For better documentation that does not rely on installing [random shit](https://docs.google.com/document/d/17GTTClKVn9E28jOMqUGvaovNeYVKzXzsIPRU075Y8_k/edit?tab=t.0) on your computer and running vibe-coded flashing scripts, see [Documentation.org](Documentation.org).

