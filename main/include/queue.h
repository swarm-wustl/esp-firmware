#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include <cstdint>
#include <utility>
#include <array>
#include "error.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// TODO: make queue capacity a templated type, change vector to std::array
template <typename Tag, typename Body, size_t Capacity>
class Queue {
private:
    struct Message {
        Tag tag;
        Body body;
    };

    std::array<Message, Capacity> messages;
    size_t index;

    QueueHandle_t queueHandle;

public:
    Queue() : index(0), queueHandle(xQueueCreate(Capacity, sizeof(Message))) {}

    // Non-copyable
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    Queue(Queue&&) = default;
    Queue& operator=(Queue&&) = default;
    
    Result pushToQueue(Tag tag, Body body) {
        // Do NOT use std::move! It breaks with FreeRTOS queues
        messages[index] = {
            .tag = tag,
            .body = body
        };    

        // Copy the msg instance into the queue
        if (xQueueSend(queueHandle, &messages[index], portMAX_DELAY) != pdPASS) {
            // TODO: maybe return error instead?
            fatal("Unable to send to queue");
        }

        // Only increment if message was successfully sent to queue
        index = (index + 1) % Capacity;

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
};

#endif