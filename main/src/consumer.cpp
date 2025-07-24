#include "consumer.h"

#include <utility>

#include "freertos/FreeRTOS.h"

#define MAX_QUEUE_LENGTH 25

static QueueHandle_t xQueue = xQueueCreate(MAX_QUEUE_LENGTH, sizeof(Consumer::Message));

// TODO: change to use rvalue refs
// we want to enforce moving here.

Result Consumer::push_to_queue(MessageTag tag, MessageBody body) {
    // Move the tag and body args into the struct to avoid unnecessary copies
    Message msg {
        .tag = std::move(tag),
        .body = std::move(body)
    };

    // Copy the msg instance into the queue
    if (xQueueSend(xQueue, &msg, portMAX_DELAY) != pdPASS) {
        // TODO: error handle
    }

    return Result::SUCCESS;
}

Result Consumer::push_motor_command(Motor::Command cmd) {
    MessageBody body;
    body.motor_cmd = std::move(cmd);

    return push_to_queue(MessageTag::MOTOR_COMMAND, body);
}

void Consumer::spin() {
    while (1) {
        Message msg;
        
        if (xQueueReceive(xQueue, &msg, portMAX_DELAY) != pdPASS) {
            fatal("Unable to read from consumer queue");
        }

        switch (msg.tag) {
            case MessageTag::MOTOR_COMMAND:
                // TODO: handle motor command message
                break;

            default:
                fatal("Unhandled message type: tag=%d", msg.tag);
                break;
        }
    }
}