#!/bin/bash

# Check if USB device argument is provided
if [ -z "$1" ]; then
  echo "Error: No USB device specified"
  echo "Usage: $0 <usb_device>"
  echo "Example: $0 /dev/ttyUSB0"
  exit 1
fi

USB_DEVICE="$1"

# Check if the device exists
if [ ! -e "$USB_DEVICE" ]; then
  echo "Error: Device $USB_DEVICE does not exist"
  exit 1
fi

# Check if idf.py is available
if ! command -v idf.py &>/dev/null; then
  echo "idf.py not found in PATH. Attempting to source ESP-IDF..."

  # Common ESP-IDF installation locations
  IDF_PATHS=(
    "$HOME/esp/esp-idf/export.sh"
    "$HOME/.espressif/esp-idf/export.sh"
    "/opt/esp-idf/export.sh"
  )

  IDF_FOUND=false
  for path in "${IDF_PATHS[@]}"; do
    if [ -f "$path" ]; then
      echo "Found ESP-IDF at: $path"
      echo "Sourcing ESP-IDF environment..."
      source "$path"
      IDF_FOUND=true
      break
    fi
  done

  if [ "$IDF_FOUND" = false ]; then
    echo "Error: ESP-IDF not found!"
    echo ""
    echo "Please install ESP-IDF by following the instructions at:"
    echo "https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/"
    echo ""
    echo "Or manually source your ESP-IDF installation with:"
    echo "source /path/to/esp-idf/export.sh"
    exit 1
  fi

  # Verify idf.py is now available
  if ! command -v idf.py &>/dev/null; then
    echo "Error: Failed to load ESP-IDF environment"
    exit 1
  fi
fi

echo "ESP-IDF environment loaded successfully"
echo "Flashing swarm.bin to $USB_DEVICE..."

# Flash the binary
esptool.py --chip esp32 --port "$USB_DEVICE" write_flash 0x10000 build/swarm.bin

if [ $? -eq 0 ]; then
  echo "Flashing completed successfully!"
else
  echo "Flashing failed!"
  exit 1
fi
