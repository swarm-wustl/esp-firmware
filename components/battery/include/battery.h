#pragma once

#include <algorithm>
#include <concepts>
#include <optional>

namespace Battery {

// volts at each divider output, i.e. what the ADC pin sees
struct Taps {
  double t1, t2, t3;
};

struct Reading {
  double cell[3];
  double total;
  int percent;
};

// (R1 + R2) / R2 for each tap. Change these if the resistors change
constexpr double DIV1 = 2.0, DIV2 = 3.0, DIV3 = 4.0;

constexpr Reading estimate(const Taps &adc) {
  const double v1 = adc.t1 * DIV1;
  const double v2 = adc.t2 * DIV2;
  const double v3 = adc.t3 * DIV3;

  Reading r{{v1, v2 - v1, v3 - v2}, v3, 0};
  const double lowest = std::min({r.cell[0], r.cell[1], r.cell[2]});
  // +0.5 rounds to nearest: truncating turns a full pack's 99.99999 into 99
  r.percent =
      std::clamp(static_cast<int>((lowest - 3.2) * 100.0 / 1.0 + 0.5), 0, 100);

  return r;
}

// the one thing the real ADC and the simulation have to agree on
template <typename T>
concept Source = requires(T &s) {
  { s.read_taps() } -> std::convertible_to<std::optional<Taps>>;
};

// anything that can report the volts at a pin, such as ESP32::Adc1
template <typename T>
concept VoltageReader = requires(T &a, int pin) {
  { a.read_volts(pin) } -> std::convertible_to<std::optional<double>>;
};

// the three divider outputs read through an ADC. Pins are plain GPIO numbers
// so this header stays free of ESP-IDF. A failed read on any tap drops the
// whole sample rather than mixing a fresh tap with a missing one
template <VoltageReader Adc, int Pin1, int Pin2, int Pin3>
class AdcSource {
public:
  std::optional<Taps> read_taps() {
    const auto t1 = adc_.read_volts(Pin1);
    const auto t2 = adc_.read_volts(Pin2);
    const auto t3 = adc_.read_volts(Pin3);

    if (!t1 || !t2 || !t3) {
      return std::nullopt;
    }

    return Taps{*t1, *t2, *t3};
  }

private:
  Adc adc_;
};

// a drain from 4.2 V to 3.2 V per cell, then back to full, so every
// percentage shows up on the topic without a pack on the bench
class SimulatedSource {
public:
  std::optional<Taps> read_taps() {
    const double cell = 4.2 - 1.0 * (step_++ % STEPS) / (STEPS - 1);

    return Taps{cell * 1 / DIV1, cell * 2 / DIV2, cell * 3 / DIV3};
  }

private:
  static constexpr unsigned STEPS = 300; // 10 minutes at the 2 s period
  unsigned step_ = 0;
};

static_assert(Source<SimulatedSource>);

// full pack, empty pack, and a weak cell dragging the percentage down
static_assert(estimate({2.1, 2.8, 3.15}).percent == 100);
static_assert(estimate({1.6, 2.4, 3.2}).percent == 0);
static_assert(estimate({1.85, 2.8, 3.15}).percent < 100);

} // namespace Battery
