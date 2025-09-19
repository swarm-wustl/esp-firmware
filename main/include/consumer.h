#ifndef CONSUMER_H
#define CONSUMER_H

#include <type_traits>

#include "error.h"
#include "motor.h"
#include "hal.h"
#include "hardware.h"
#include "queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

    // The default values for the templated types come from the specific hardware being used
    template <
        size_t MotorCount = HW::MOTOR_COUNT,
        HAL::MotorDriverTrait MotorDriver = HW::MotorDriver,
        HAL::DriveStyleTrait<MotorCount> DriveStyle = HW::DriveStyle
    >
    void spin(MotorDriver driver, Queue<MessageTag, MessageBody> queue) {
        // TODO: delete
        while (1) {
            static Motor::Direction dir = Motor::Direction::FORWARD;
            static Motor::Name motor = Motor::Name::LEFT;

            driver.run({
                .name = motor,
                .dir = dir,
                .pwm_ratio = 0.5
            });

            motor = dir == Motor::Direction::REVERSE ? (motor == Motor::Name::LEFT ? Motor::Name::RIGHT : Motor::Name::LEFT) : motor;
            dir = dir == Motor::Direction::FORWARD ? Motor::Direction::REVERSE : Motor::Direction::FORWARD;

            log("driving motor...");

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // TODO: set some sort of frequency for this to be called
        while (1) {
            MessageTag tag;
            MessageBody body;

            if (queue.popFromQueue(tag, body) != Result::SUCCESS) {
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