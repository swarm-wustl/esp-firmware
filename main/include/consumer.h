#ifndef CONSUMER_H
#define CONSUMER_H

#include "error.h"
#include "motor.h"

namespace Consumer {
    enum class MessageTag {
        MOTOR_COMMAND
        // TODO: IMU read, send data to ROS server, etc.
    };

    union MessageBody {
        Motor::Command motor_cmd;
    };

    struct Message {
        MessageTag tag;
        MessageBody body;
    };

    Result push_to_queue(MessageTag tag, MessageBody body);
    Result push_motor_command(Motor::Command cmd);

    void spin();
}

#endif