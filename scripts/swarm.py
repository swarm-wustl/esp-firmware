#!/usr/bin/env python3

# Written with Claude

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

Docker runs build/menuconfig. Host runs flash/monitor/erase via esptool directly
(requires esptool in PATH, i.e. ESP-IDF sourced) -- the Docker build dir can't be
reused by idf.py on the host because CMake bakes in /workspace paths. All other
idf.py subcommands are forwarded to Docker by default.

The Docker image (swarm-idf:latest) is built from .devcontainer/Dockerfile on
first use. Bump the ESP-IDF version via the IDF_VERSION arg in that Dockerfile.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Locally-built image (see .devcontainer/Dockerfile). The official
# microros/esp-idf-microros image is stale (IDF v5.0), so we build our own off
# espressif/idf with the micro-ROS Python deps added.
DOCKER_IMAGE = "swarm-idf:latest"

# Container mount point for the project. clangd runs inside the container (see
# scripts/clangd.sh) and translates host paths to this with --path-mappings, so it
# just needs to match the mount in scripts/clangd.sh and .devcontainer.
WORKSPACE = "/workspace"
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


def find_export_script() -> Path | None:
    """Locate ESP-IDF's export.sh on the host."""
    candidates = []
    if os.environ.get("IDF_PATH"):
        candidates.append(Path(os.environ["IDF_PATH"]) / "export.sh")
    candidates.append(Path.home() / "esp" / "esp-idf" / "export.sh")
    for c in candidates:
        if c.exists():
            return c
    return None


# Cache of the environment after sourcing ESP-IDF, so we only pay the cost once.
_idf_env: dict[str, str] | None = None


def idf_env() -> dict[str, str] | None:
    """Return an environment with ESP-IDF sourced.

    If esptool is already on PATH, the current env is fine (returns None). Otherwise
    we source export.sh in a subshell and capture the resulting environment so host
    commands work without the user having to source it themselves.
    """
    global _idf_env
    if shutil.which("esptool.py") or shutil.which("esptool"):
        return None
    if _idf_env is not None:
        return _idf_env

    export = find_export_script()
    if export is None:
        sys.exit(
            "Error: esptool not in PATH and ESP-IDF export.sh not found.\n"
            "Set IDF_PATH or install ESP-IDF under ~/esp/esp-idf."
        )

    print(f"[host]   sourcing ESP-IDF: {export}")
    # Source in a quiet subshell and dump the environment as NUL-separated pairs.
    result = subprocess.run(
        ["bash", "-c", f'source "{export}" >/dev/null 2>&1 && env -0'],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.exit(f"Error: failed to source {export}\n{result.stderr}")

    env: dict[str, str] = {}
    for entry in result.stdout.split("\0"):
        if "=" in entry:
            key, _, value = entry.partition("=")
            env[key] = value
    _idf_env = env
    return env


def find_esptool() -> list[str]:
    """Locate esptool on the host (installed alongside idf.py in the IDF env)."""
    env = idf_env()
    path = shutil.which("esptool.py", path=(env or os.environ).get("PATH")) or \
        shutil.which("esptool", path=(env or os.environ).get("PATH"))
    if path:
        return [path]
    # esptool ships as a Python module in the IDF env even if the script isn't
    # exposed on PATH.
    return [sys.executable, "-m", "esptool"]


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

    workspace = WORKSPACE
    # Mount project root always; if building test, set working dir to test subdir
    workdir = workspace if project_dir == PROJECT_ROOT else f"{workspace}/test"

    # -it only when attached to a real terminal (menuconfig needs it); without a
    # TTY (CI, background runs) it would error out.
    interactive = ["-it"] if sys.stdin.isatty() and sys.stdout.isatty() else []

    cmd = [
        "docker", "run", "--rm", *interactive,
        "-v", f"{PROJECT_ROOT}:{workspace}",
        "-w", workdir,
        "-e", "LC_ALL=C.UTF-8",
        "-e", "LANG=C.UTF-8",
        DOCKER_IMAGE,
        "idf.py", *idf_args,
    ]

    print(f"[docker] idf.py {' '.join(idf_args)}")
    return subprocess.run(cmd).returncode


def read_flasher_args(build_dir: Path) -> dict:
    """Load the flasher_args.json produced by the (Docker) build."""
    fa = build_dir / "flasher_args.json"
    if not fa.exists():
        sys.exit(
            f"Error: {fa} not found.\n"
            "Build first, e.g. ./scripts/swarm.py build"
        )
    return json.loads(fa.read_text())


def parse_port_baud(host_args: list[str]) -> tuple[str | None, str | None, list[str]]:
    """Pull -p/--port and -b/--baud out of the host args, return the rest."""
    port = baud = None
    rest: list[str] = []
    i = 0
    while i < len(host_args):
        arg = host_args[i]
        if arg in ("-p", "--port") and i + 1 < len(host_args):
            port = host_args[i + 1]
            i += 2
            continue
        if arg in ("-b", "--baud") and i + 1 < len(host_args):
            baud = host_args[i + 1]
            i += 2
            continue
        rest.append(arg)
        i += 1
    return port, baud, rest


def resolve_port(port: str | None) -> str:
    port = port or find_serial_port()
    if not port:
        sys.exit("Error: no serial port found; pass -p /dev/tty.usbserial-XXXX")
    return port


def host_flash(project_dir: Path, port: str | None, baud: str | None) -> int:
    """Flash the already-built binaries with esptool (no CMake reconfigure).

    idf.py would re-run CMake against the build dir, but that dir was configured
    inside Docker with /workspace paths, so it can't be reused on the host. The
    binaries and flash layout are portable, so we drive esptool directly.
    """
    build_dir = project_dir / "build"
    fa = read_flasher_args(build_dir)
    extra = fa.get("extra_esptool_args", {})
    chip = extra.get("chip", "esp32")
    before = extra.get("before", "default_reset")
    after = extra.get("after", "hard_reset")
    port = resolve_port(port)

    cmd = [
        *find_esptool(),
        "--chip", chip,
        "-p", port,
        "-b", baud or "460800",
        "--before", before,
        "--after", after,
        "write_flash", "@flash_args",
    ]
    print(f"[host]   esptool write_flash on {port} (chip {chip})")
    # @flash_args paths are relative to the build dir.
    return subprocess.run(cmd, cwd=build_dir, env=idf_env()).returncode


def host_erase(project_dir: Path, port: str | None) -> int:
    build_dir = project_dir / "build"
    chip = read_flasher_args(build_dir).get("extra_esptool_args", {}).get("chip", "esp32")
    port = resolve_port(port)
    cmd = [*find_esptool(), "--chip", chip, "-p", port, "erase_flash"]
    print(f"[host]   esptool erase_flash on {port}")
    return subprocess.run(cmd, env=idf_env()).returncode


def host_monitor(project_dir: Path, port: str | None) -> int:
    """Open the serial monitor without reconfiguring via idf.py."""
    build_dir = project_dir / "build"
    fa = read_flasher_args(build_dir)
    port = resolve_port(port)

    elf = None
    app_bin = fa.get("app", {}).get("file")
    if app_bin:
        candidate = build_dir / (Path(app_bin).stem + ".elf")
        if candidate.exists():
            elf = str(candidate)

    env = idf_env()
    monitor = shutil.which("esp_idf_monitor", path=(env or os.environ).get("PATH"))
    base = [monitor] if monitor else [sys.executable, "-m", "esp_idf_monitor"]
    cmd = [*base, "-p", port]
    if elf:
        cmd.append(elf)
    print(f"[host]   monitor on {port}")
    return subprocess.run(cmd, cwd=build_dir, env=env).returncode


def run_host_commands(host_args: list[str], project_dir: Path) -> int:
    """Run the host-side (USB) commands via esptool, in the order given."""
    port, baud, rest = parse_port_baud(host_args)
    rc = 0
    for cmd in [a for a in rest if a in HOST_COMMANDS]:
        if cmd == "flash":
            rc = host_flash(project_dir, port, baud)
        elif cmd == "monitor":
            rc = host_monitor(project_dir, port)
        elif cmd in ("erase_flash", "erase_otadata"):
            rc = host_erase(project_dir, port)
        if rc != 0:
            return rc
    return rc


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
        rc = run_host_commands(host_args, project_dir)

    sys.exit(rc)


if __name__ == "__main__":
    main()
