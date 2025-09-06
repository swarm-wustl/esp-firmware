#include "consumer.h"

#include <utility>

#define MAX_QUEUE_LENGTH 25

static QueueHandle_t xQueue = xQueueCreate(MAX_QUEUE_LENGTH, sizeof(Consumer::Message));

// TODO: change to use rvalue refs
// we want to enforce moving here.

Result Consumer::push_to_queue(MessageTag tag, MessageBody body) {
    // TODO: is move needed?
    Message msg {
        .tag = std::move(tag),
        .body = std::move(body)
    };

    // Copy the msg instance into the queue
    if (xQueueSend(xQueue, &msg, portMAX_DELAY) != pdPASS) {
        // TODO: maybe return error instead?
        fatal("Unable to send to consumer queue");
    }

    return Result::SUCCESS;
}

Result Consumer::pop_from_queue(MessageTag& tag, MessageBody& body) {
    Message msg;

    if (xQueueReceive(xQueue, &msg, portMAX_DELAY) != pdPASS) {
        // TODO: maybe return error instead?
        fatal("Unable to read from consumer queue");
    }

    // Move the tag and body args out of the struct to avoid unnecessary copies
    tag = msg.tag;
    body = msg.body;

    return Result::SUCCESS;
}

Result Consumer::push_motor_command(Motor::Command cmd) {
    MessageBody body;
    body.motor_cmd = std::move(cmd);

    return push_to_queue(MessageTag::MOTOR_COMMAND, body);
}