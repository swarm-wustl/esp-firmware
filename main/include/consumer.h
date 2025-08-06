#ifndef CONSUMER_H
#define CONSUMER_H

#include <type_traits>

#include "error.h"
#include "motor.h"
#include "hal.h"
#include "hardware.h"

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
    Result pop_from_queue(MessageTag& tag, MessageBody& body);
    Result push_motor_command(Motor::Command cmd);

    // The default values for the templated types come from the specific hardware being used
    template <
        size_t MotorCount = HW::MOTOR_COUNT,
        HAL::MotorDriverTrait MotorDriver = HW::MotorDriver,
        HAL::DriveStyleTrait<MotorCount> DriveStyle = HW::DriveStyle
    >
    void spin(MotorDriver driver) {
        while (1) {
            MessageTag tag;
            MessageBody body;

            if (pop_from_queue(tag, body) != Result::SUCCESS) {
                continue;
            }

            switch (tag) {
                case MessageTag::MOTOR_COMMAND:
                    // TODO: handle motor command message
                    break;

                default:
                    fatal("Unhandled message type: tag=%d", tag);
                    break;
            }
        }
    }
}

#endif