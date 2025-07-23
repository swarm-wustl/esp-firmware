#ifndef MOTOR_H
#define MOTOR_H

#include <stddef.h>

namespace Motor {
    enum class Direction {
        FORWARD,
        REVERSE,
        STOP
    };

    // TODO: make a table
    enum class ID : size_t {
        LEFT,
        RIGHT,
        SIZE    // Used for sizing table. TODO: see if needed
    };

    struct Command {
        ID id;
        Direction dir;
        double pwm_ratio;
    };

    class Instance {
    public:
        // TODO: constructor that takes GPIO
        ~Instance();

        void run(Command cmd);
    };
}

#endif