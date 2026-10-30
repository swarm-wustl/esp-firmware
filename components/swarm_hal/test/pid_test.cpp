#include "pid.h"
#include "unity.h"

namespace {
constexpr auto kGains = Pid::gains(0.5, 2.0, 0.1);
constexpr double kDt = 0.02;

constexpr Pid::State kRest{.integral = 0.0, .measured = 0.0};

static_assert(Pid::step(Pid::gains(0.0, 0.0, 0.0), kRest, {1.0, 0.0, 0.3}, kDt)
                  .output == 0.3);
} // namespace

TEST_CASE("pid: no error leaves only the feedforward", "[pid]") {
  const auto step = Pid::step(kGains, {.integral = 0.0, .measured = 2.0},
                              {.setpoint = 2.0, .measured = 2.0,
                               .feedforward = 0.4},
                              kDt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.4, step.output);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, step.state.integral);
}

TEST_CASE("pid: proportional and integral push toward the setpoint", "[pid]") {
  const auto step = Pid::step(
      kGains, kRest, {.setpoint = 0.2, .measured = 0.0, .feedforward = 0.0},
      kDt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.2 * kDt, step.state.integral);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.5 * 0.2 + 2.0 * 0.2 * kDt, step.output);
}

TEST_CASE("pid: a setpoint step does not kick the derivative", "[pid]") {
  const auto gains = Pid::gains(0.0, 0.0, 1.0);
  const auto step = Pid::step(
      gains, kRest, {.setpoint = 5.0, .measured = 0.0, .feedforward = 0.0},
      kDt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, step.output);
}

TEST_CASE("pid: derivative opposes a rising measurement", "[pid]") {
  const auto gains = Pid::gains(0.0, 0.0, 1.0);
  const auto step = Pid::step(
      gains, kRest, {.setpoint = 0.0, .measured = 0.01, .feedforward = 0.0},
      kDt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, -0.01 / kDt, step.output);
}

TEST_CASE("pid: output saturates and the integral stops winding", "[pid]") {
  const Pid::State wound{.integral = 1.0, .measured = 0.0};
  const auto step = Pid::step(
      kGains, wound, {.setpoint = 10.0, .measured = 0.0, .feedforward = 1.0},
      kDt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, step.output);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, step.state.integral);
}

TEST_CASE("pid: a saturated integral still unwinds once the error flips",
          "[pid]") {
  const Pid::State wound{.integral = 1.0, .measured = 1.0};
  const auto step = Pid::step(
      Pid::gains(0.5, 2.0, 0.0), wound,
      {.setpoint = 0.0, .measured = 1.0, .feedforward = 0.0},
      kDt);

  TEST_ASSERT_TRUE(step.state.integral < 1.0);
}
