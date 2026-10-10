#ifndef ENCODERS_H
#define ENCODERS_H

#include <array>
#include <concepts>
#include <cstddef>
#include <utility>

#include "assembly.h"
#include "drive.h"
#include "encoder.h"
#include "motor.h"
#include "swarm_hal.h"

namespace Encoders {
struct Row {
  Motor::Name name;
  int channel_a;
  int channel_b;
  int unit;

  consteval auto pins() const {
    return HAL::pins(HAL::NamedPin{"a", channel_a},
                     HAL::NamedPin{"b", channel_b});
  }
};

template <auto Table, HAL::fixed_string L> consteval auto pin_column() {
  return [&]<size_t... I>(std::index_sequence<I...>) {
    return std::array<HAL::Pin, sizeof...(I)>{Table[I].pins()[L]...};
  }(std::make_index_sequence<Table.size()>{});
}

template <size_t N>
consteval auto roster_of(const std::array<Row, N> &table) {
  std::array<Motor::Name, N> names{};

  for (size_t i = 0; i < N; ++i) {
    names[i] = table[i].name;
  }

  return names;
}

template <size_t N>
consteval auto claims_of(const std::array<Row, N> &table) {
  std::array<HAL::Claim, N * 3> claims{};
  size_t next = 0;

  for (const Row &row : table) {
    for (const HAL::Claim &claim : row.pins().claims()) {
      claims[next++] = claim;
    }

    claims[next++] = {HAL::Resource::PcntUnit, row.unit, HAL::Use::Exclusive};
  }

  return claims;
}

// the style decides which motors exist, so a table that names a motor the
// geometry does not command -- or misses one it does -- never becomes a value
template <Drive::Style S, std::same_as<Row>... Rows>
consteval auto for_style(Rows... rows) {
  const std::array table{rows...};

  if (HAL::unique_names(roster_of(table)) &&
      Drive::covers_roster<S>(roster_of(table))) {
    return table;
  }

  std::unreachable();
}

template <Encoder::Counter Counter, auto Table, Encoder::Geometry Motor>
  requires HAL::Claiming<Counter>
class Bank {
public:
  static_assert(Encoder::usable(Motor),
                "an encoder needs a nonzero pole-pair count and gear ratio");

  static constexpr auto motors = roster_of(Table);

  // the counter decides how many counts a revolution is worth, so the spec is
  // derived from it rather than declared twice
  static constexpr Encoder::Spec spec{Motor, Counter::decoding};

  static constexpr auto claims = HAL::concat(claims_of(Table), Counter::claims);

  explicit Bank(Swarm::Assembly assembly) : counter_{assembly} {
    for (size_t i = 0; i < Table.size(); ++i) {
      counter_.configure_unit(Table[i].unit, kChannelA[i], kChannelB[i]);
    }
  }

  Bank(const Bank &) = delete;
  Bank &operator=(const Bank &) = delete;

  Bank(Bank &&) = default;
  Bank &operator=(Bank &&) = default;

  [[nodiscard]] std::array<Encoder::Reading, Table.size()> read() {
    std::array<Encoder::Reading, Table.size()> out{};

    for (size_t i = 0; i < Table.size(); ++i) {
      out[i] = {Table[i].name, counter_.count(Table[i].unit)};
    }

    return out;
  }

  // counts since the previous call, which is what a velocity estimate needs
  [[nodiscard]] std::array<Encoder::Reading, Table.size()> advance() {
    std::array<Encoder::Reading, Table.size()> out = read();

    for (size_t i = 0; i < out.size(); ++i) {
      out[i].counts = trackers_[i].advance(out[i].counts);
    }

    return out;
  }

  void reset() {
    for (size_t i = 0; i < Table.size(); ++i) {
      counter_.clear(Table[i].unit);
      trackers_[i] = Encoder::Tracker{};
    }
  }

private:
  static constexpr auto kChannelA = pin_column<Table, "a">();
  static constexpr auto kChannelB = pin_column<Table, "b">();

  Counter counter_;
  std::array<Encoder::Tracker, Table.size()> trackers_{};
};
} // namespace Encoders

#endif
