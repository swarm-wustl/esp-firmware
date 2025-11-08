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
    constexpr size_t CONSUMER_QUEUE_SIZE = 25;

    enum class MessageTag {
        MOTOR_COMMAND,
        // TODO: IMU read, send data to ROS server, etc.
    };

    union MessageBody {
        Motor::Command motor_cmd;
    };

    using QueueType = Queue<MessageTag, MessageBody, CONSUMER_QUEUE_SIZE>;

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
    void spin(MotorDriver& driver, Queue<MessageTag, MessageBody, CONSUMER_QUEUE_SIZE>& queue) {
        // TODO: set some sort of frequency for this to be called
        while (1) {
            MessageTag tag;
            MessageBody body;

            if (queue.popFromQueue(tag, body) != Result::SUCCESS) {
                continue;
            }

            switch (tag) {
                case MessageTag::MOTOR_COMMAND: {
                    Motor::Command cmd = body.motor_cmd;
                    driver.run(cmd); 
                    break;
                }

                default: {
                    fatal("Unhandled message type: tag=%d", tag);
                    break;
                }
            }
        }
    }
}

#endif