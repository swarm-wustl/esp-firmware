#!/usr/bin/env bash

# Written with Claude

# Runs the DWM test suite. Two tiers, selected by target:
#   ./run.sh host     pure + mock-SPI unit tests, built for the linux target and
#                     run natively (no board). [dwm_data] + [dwm_mock]
#   ./run.sh device   the on-device integration app (esp32). [dwm_reg] + [dwm]
#                     builds only; append `flash` to flash+monitor a board
#
# Each target keeps its own sdkconfig + build dir so the two never clobber each
# other. Run from inside the micro-ROS container (scripts/shell.sh).

set -euo pipefail
cd "$(dirname "$0")"

usage() {
  cat <<EOF
usage: $0 <host|device [flash]>

  host           build the pure + mock-SPI unit tests for the linux target and
                 run them natively (no board)
  device         build the on-device integration test app (esp32)
  device flash   also flash it to a connected board and open the serial monitor
                 to run the tests on real hardware
EOF
}

case "${1:-}" in
host)
  idf.py -B build_host -DSDKCONFIG=sdkconfig.host --preview set-target linux build
  ./build_host/unit_test.elf
  ;;
device)
  idf.py -B build_device -DSDKCONFIG=sdkconfig.device set-target esp32 build
  if [ "${2:-}" = "flash" ]; then
    idf.py -B build_device -DSDKCONFIG=sdkconfig.device flash monitor
  fi
  ;;
"")
  usage
  ;;
*)
  usage >&2
  exit 1
  ;;
esac
