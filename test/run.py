#!/usr/bin/env python3

# Written with Claude

# Single entry point for the DWM test suite. Builds the test app for the chosen
# target, then hands off to pytest-embedded (which drives the unity menu). Run
# from inside the container (scripts/shell.sh). Extra args pass through to pytest.
#
#   ./run.py host           pure + mock-SPI unit tests, linux target, native
#   ./run.py device         on-device integration tests, esp32 (needs a board)
#   ./run.py host -s        forward -s (or any flag) straight to pytest

import argparse
import pathlib
import subprocess
import sys

TARGETS = {
    "host": dict(
        idf_target="linux",
        build_dir="build_host",
        sdkconfig="sdkconfig.host",
        services="idf",
        set_target=["--preview", "set-target", "linux"],
    ),
    "device": dict(
        idf_target="esp32",
        build_dir="build_device",
        sdkconfig="sdkconfig.device",
        services="idf,esp",
        set_target=["set-target", "esp32"],
    ),
}


def run(cmd, cwd):
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode:
        sys.exit(result.returncode)


def main():
    ap = argparse.ArgumentParser(description="build + run the DWM tests")
    ap.add_argument("target", choices=TARGETS, help="host = linux native, device = esp32")
    ap.add_argument("--no-build", action="store_true", help="skip the firmware build")
    args, pytest_args = ap.parse_known_args()

    cfg = TARGETS[args.target]
    here = pathlib.Path(__file__).resolve().parent

    if not args.no_build:
        run(["idf.py", "-B", cfg["build_dir"], f"-DSDKCONFIG={cfg['sdkconfig']}",
             *cfg["set_target"], "build"], here)

    # -v so each unity case prints as its own subtest line (pass -q to quiet it)
    run(["pytest", "-v", "--target", cfg["idf_target"], "--build-dir", cfg["build_dir"],
         "--embedded-services", cfg["services"], *pytest_args], here)


if __name__ == "__main__":
    main()
