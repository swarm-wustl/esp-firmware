#ifndef MOTOR_H
#define MOTOR_H

#include <cstddef>

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
        LOWER_LEFT, LOWER_RIGHT
    };

    struct Command {
        Name name;
        Direction dir;
        double pwm_ratio;
    };
}

#endif
