#pragma once

#include <optional>

namespace ESP32 {

// One-shot reads on ADC1. Any ADC1 GPIO works: the channel is looked up from
// the pin, so there is no channel table to keep in step with the pins. Each
// read is 64 samples averaged and converted through the chip's calibration,
// and returns volts at the pin -- before any divider
//
// Not thread-safe: every read goes through one shared unit, so keep all callers
// in a single task
class Adc1 {
public:
  // empty if the pin is not an ADC1 pin or the driver reports an error
  std::optional<double> read_volts(int gpio);
};

} // namespace ESP32
