#ifndef ENCODER_H
#define ENCODER_H

#include <concepts>
#include <cstdint>
#include <numbers>
#include <utility>

#include "motor.h"
#include "pins.h"

namespace Encoder {
// how many counts one channel-pair period yields: X1 counts one edge of one
// channel, X2 both edges of one, X4 every edge of both. It is a property of the
// counting hardware, not of the motor, so a Counter publishes it and nothing
// else restates it
enum class Decoding : uint8_t { X1 = 1, X2 = 2, X4 = 4 };

// what the motor itself contributes: a hall sensor with this many pole pairs on
// the motor shaft, behind this gearbox
struct Geometry {
  uint32_t pole_pairs;
  uint32_t gear_ratio;
};

struct Spec {
  Geometry geometry;
  Decoding decoding;

  constexpr uint32_t counts_per_rev() const {
    return geometry.pole_pairs * static_cast<uint32_t>(decoding) *
           geometry.gear_ratio;
  }
};

// a zero on either side would make counts_per_rev() zero and every derived
// angle a division by it, so such a geometry is not a constant expression
consteval Geometry geometry(uint32_t pole_pairs, uint32_t gear_ratio) {
  if (pole_pairs == 0 || gear_ratio == 0) {
    std::unreachable();
  }

  return Geometry{pole_pairs, gear_ratio};
}

constexpr bool usable(const Geometry &geometry) {
  return geometry.pole_pairs > 0 && geometry.gear_ratio > 0;
}

// DFRobot FIT0485: 7 pole pairs, 210:1 gearbox. Their "Hall Feedback
// Resolution: 2940" is 7 * 2 * 210, i.e. the X2 figure -- an X4 counter sees
// 5880 per output revolution
inline constexpr Geometry fit0485 = geometry(7, 210);

template <typename C>
concept Counter =
    requires(C counter, int unit, HAL::Pin a, HAL::Pin b) {
      { C::decoding } -> std::convertible_to<Decoding>;
      { counter.configure_unit(unit, a, b) } -> std::same_as<void>;
      { counter.count(unit) } -> std::same_as<int32_t>;
      { counter.clear(unit) } -> std::same_as<void>;
    };

struct Reading {
  Motor::Name name;
  int32_t counts;
};

constexpr double revolutions(const Spec &spec, int32_t counts) {
  return static_cast<double>(counts) /
         static_cast<double>(spec.counts_per_rev());
}

constexpr double radians(const Spec &spec, int32_t counts) {
  return revolutions(spec, counts) * 2.0 * std::numbers::pi;
}

constexpr double rev_per_second(const Spec &spec, int32_t counts,
                               double seconds) {
  return seconds > 0.0 ? revolutions(spec, counts) / seconds : 0.0;
}

constexpr double rpm(const Spec &spec, int32_t counts, double seconds) {
  return rev_per_second(spec, counts, seconds) * 60.0;
}

constexpr double rad_per_second(const Spec &spec, int32_t counts,
                                double seconds) {
  return seconds > 0.0 ? radians(spec, counts) / seconds : 0.0;
}

// a count is cumulative, so velocity needs the difference between two of them.
// The subtraction is on purpose unsigned-wrapped: a counter that rolls over
// still yields the right delta as long as it moved less than 2^31 counts
constexpr int32_t difference(int32_t now, int32_t before) {
  return static_cast<int32_t>(static_cast<uint32_t>(now) -
                              static_cast<uint32_t>(before));
}

class Tracker {
public:
  constexpr int32_t advance(int32_t count) {
    const int32_t delta = difference(count, last_);
    last_ = count;

    return delta;
  }

  [[nodiscard]] constexpr int32_t last() const { return last_; }

private:
  int32_t last_ = 0;
};
} // namespace Encoder

#endif
