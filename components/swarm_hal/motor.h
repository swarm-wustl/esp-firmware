#ifndef MOTOR_H
#define MOTOR_H

#include <array>
#include <cstddef>
#include <utility>

/*
Motor Namespace
Defines the abstract types used for a motor.
This does not represent any physical hardware.
Related abstract types would be for IMU, Control (PID), Robot State (IDLE, ACTIVE), Battery, DistanceSensor, Pose, etc.
*/
namespace Motor {
    enum class Direction {
        FORWARD,
        REVERSE,
        STOP
    };

    enum class Name {
        LEFT, RIGHT,
        UPPER_LEFT, UPPER_RIGHT,
        LOWER_LEFT, LOWER_RIGHT,
        FRONT_UMC_A, FRONT_UMC_B
    };

    struct Command {
        Name name;
        Direction dir;
        double pwm_ratio;
    };

    template <size_t N> struct Frame {
        std::array<Command, N> commands;
    };

    constexpr double magnitude(double v) { return v < 0.0 ? -v : v; }

    constexpr Command command_for(Name name, double velocity) {
        return {name,
                velocity > 0.0   ? Direction::FORWARD
                : velocity < 0.0 ? Direction::REVERSE
                                 : Direction::STOP,
                magnitude(velocity)};
    }

    // motors wired to move as one: no geometry to invert, just an effort in
    // [-1, 1] whose sign is the direction. A motor mounted facing the other
    // way has its in_a/in_b swapped in the pin table, not here
    template <auto Names>
    constexpr Frame<Names.size()> ganged(double effort) {
        return [effort]<size_t... I>(std::index_sequence<I...>) {
            const double clamped = effort < -1.0 ? -1.0 : effort > 1.0 ? 1.0 : effort;

            return Frame<Names.size()>{{command_for(Names[I], clamped)...}};
        }(std::make_index_sequence<Names.size()>{});
    }
}

#endif
