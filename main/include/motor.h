#ifndef MOTOR_H
#define MOTOR_H

#include "error.h"

namespace Motor {
    enum class Direction {
        FORWARD,
        REVERSE,
        STOP
    };

    struct Data {
        Direction dir;
        double pwm_ratio;
    };

    struct Command {
        Data left;
        Data right;
    };
}

#endif