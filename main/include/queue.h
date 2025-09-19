#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include <cstdint>
#include "error.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

template <typename Tag, typename Body>
class Queue {
public:
    Queue(uint32_t maxQueueSize) : queueHandle(xQueueCreate(maxQueueSize, sizeof(Message))) {}
    
    Result pushToQueue(Tag tag, Body body) {
        // TODO: is move needed?
        Message msg {
            .tag = std::move(tag),
            .body = std::move(body)
        };

        // Copy the msg instance into the queue
        if (xQueueSend(queueHandle, &msg, portMAX_DELAY) != pdPASS) {
            // TODO: maybe return error instead?
            fatal("Unable to send to queue");
        }

        return Result::SUCCESS;
    }

    Result popFromQueue(Tag& tag, Body& body) {
        Message msg;

        if (xQueueReceive(queueHandle, &msg, portMAX_DELAY) != pdPASS) {
            // TODO: maybe return error instead?
            fatal("Unable to read from queue");
        }

        // Move the tag and body args out of the struct to avoid unnecessary copies
        tag = msg.tag;
        body = msg.body;

        return Result::SUCCESS;
    }

private:
    struct Message {
        Tag tag;
        Body body;
    };

    QueueHandle_t queueHandle;
};

#endif