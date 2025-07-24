#ifndef MOTOR_H
#define MOTOR_H

#include <cstddef>
#include <type_traits>
#include <list>

/*
Motor Namespace
Defines the abstract types used for a motor.
This does not represent any physical hardware.
*/
namespace Motor {
    enum class Direction {
        FORWARD,
        REVERSE,
        STOP
    };

    using ID = size_t;

    enum class DifferentialDrive : ID {
        LEFT,
        RIGHT
    };

    struct Command {
        ID name;
        Direction dir;
        double pwm_ratio;
    };
}

#endif