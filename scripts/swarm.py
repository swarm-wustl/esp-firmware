#!/usr/bin/env python3
"""
Wrapper over idf.py that runs build commands inside the micro-ROS Docker container
and runs flash/monitor commands natively on the host (USB passthrough limitation on macOS).

Usage:
    ./scripts/swarm.py build
    ./scripts/swarm.py build flash -p /dev/tty.usbserial-0001
    ./scripts/swarm.py flash -p /dev/tty.usbserial-0001
    ./scripts/swarm.py monitor -p /dev/tty.usbserial-0001
    ./scripts/swarm.py menuconfig
    ./scripts/swarm.py --test build          # builds the test project instead
    ./scripts/swarm.py --test flash -p /dev/tty.usbserial-0001
    ./scripts/swarm.py build-image           # (re)build the Docker image
    ./scripts/swarm.py build-image --no-cache

Docker runs build/menuconfig. Host runs flash/monitor (requires idf.py in PATH or
ESP-IDF sourced). All other idf.py subcommands are forwarded to Docker by default.

The Docker image (swarm-idf:latest) is built from .devcontainer/Dockerfile on
first use. Bump the ESP-IDF version via the IDF_VERSION arg in that Dockerfile.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Locally-built image (see .devcontainer/Dockerfile). The official
# microros/esp-idf-microros image is stale (IDF v5.0), so we build our own off
# espressif/idf with the micro-ROS Python deps added.
DOCKER_IMAGE = "swarm-idf:latest"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
TEST_DIR = PROJECT_ROOT / "test"
DOCKERFILE = PROJECT_ROOT / ".devcontainer" / "Dockerfile"

# Subcommands that must run on the host (need USB access)
HOST_COMMANDS = {"flash", "monitor", "erase_flash", "erase_otadata"}


def find_serial_port() -> str | None:
    """Best-effort guess at the ESP32 serial port on macOS."""
    import glob
    candidates = glob.glob("/dev/tty.usbserial-*") + glob.glob("/dev/tty.SLAB_USBtoUART*") + glob.glob("/dev/tty.usbmodem*")
    return candidates[0] if candidates else None


def ensure_docker() -> None:
    if not shutil.which("docker"):
        sys.exit("Error: docker not found in PATH")
    result = subprocess.run(["docker", "info"], capture_output=True)
    if result.returncode != 0:
        sys.exit("Error: Docker daemon is not running")


def ensure_idf() -> None:
    if shutil.which("idf.py"):
        return
    export = Path.home() / "esp" / "esp-idf" / "export.sh"
    if export.exists():
        sys.exit(
            f"Error: idf.py not in PATH. Source ESP-IDF first:\n  source {export}"
        )
    sys.exit("Error: idf.py not found. Install ESP-IDF or source export.sh.")


def image_exists() -> bool:
    result = subprocess.run(
        ["docker", "image", "inspect", DOCKER_IMAGE],
        capture_output=True,
    )
    return result.returncode == 0


def build_image(force: bool = False) -> int:
    """Build the local Docker image from .devcontainer/Dockerfile."""
    ensure_docker()
    cmd = [
        "docker", "build",
        "-t", DOCKER_IMAGE,
        "-f", str(DOCKERFILE),
        str(DOCKERFILE.parent),
    ]
    if force:
        cmd.insert(2, "--no-cache")
    print(f"[docker] building {DOCKER_IMAGE} from {DOCKERFILE}")
    return subprocess.run(cmd).returncode


def docker_build(idf_args: list[str], project_dir: Path) -> int:
    """Run idf.py inside the Docker container."""
    ensure_docker()

    # Build the image on first use so the user doesn't have to remember to.
    if not image_exists():
        rc = build_image()
        if rc != 0:
            return rc

    workspace = "/workspace"
    # Mount project root always; if building test, set working dir to test subdir
    workdir = workspace if project_dir == PROJECT_ROOT else f"{workspace}/test"

    cmd = [
        "docker", "run", "--rm", "-it",
        "-v", f"{PROJECT_ROOT}:{workspace}",
        "-w", workdir,
        "-e", "LC_ALL=C.UTF-8",
        "-e", "LANG=C.UTF-8",
        DOCKER_IMAGE,
        "idf.py", *idf_args,
    ]

    print(f"[docker] idf.py {' '.join(idf_args)}")
    return subprocess.run(cmd).returncode


def host_run(idf_args: list[str], project_dir: Path) -> int:
    """Run idf.py natively on the host."""
    ensure_idf()

    cmd = ["idf.py", *idf_args]
    print(f"[host]   idf.py {' '.join(idf_args)}")
    return subprocess.run(cmd, cwd=project_dir).returncode


def split_commands(idf_args: list[str]) -> tuple[list[str], list[str]]:
    """
    Split a mixed command list like ['build', 'flash', '-p', '/dev/...'] into
    docker_args=['build'] and host_args=['flash', '-p', '/dev/...'].

    Shared flags (like -p PORT) are included in host_args only.
    """
    docker_args: list[str] = []
    host_args: list[str] = []
    target = docker_args
    i = 0
    while i < len(idf_args):
        arg = idf_args[i]
        if arg in HOST_COMMANDS:
            target = host_args
        # Flags with values (e.g. -p PORT, -b BAUD) stay with their section
        target.append(arg)
        i += 1
    return docker_args, host_args


def main() -> None:
    # Pull out our own --test flag before forwarding the rest to idf.py
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--test", action="store_true")
    known, idf_args = pre.parse_known_args()

    project_dir = TEST_DIR if known.test else PROJECT_ROOT

    if not idf_args:
        print(__doc__)
        sys.exit(0)

    # Our own meta-command to (re)build the Docker image.
    if idf_args[0] == "build-image":
        force = "--no-cache" in idf_args
        sys.exit(build_image(force=force))

    docker_args, host_args = split_commands(idf_args)

    # If there are shared flags (e.g. -p, -b) before any subcommand, they
    # belong to both sections or neither — just pass them to docker if no
    # host commands are present, otherwise to host.
    rc = 0

    if docker_args:
        rc = docker_build(docker_args, project_dir)
        if rc != 0:
            sys.exit(rc)

    if host_args:
        rc = host_run(host_args, project_dir)

    sys.exit(rc)


if __name__ == "__main__":
    main()
