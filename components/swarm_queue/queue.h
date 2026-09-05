#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdlib>

template <typename Tag, typename Body, size_t Capacity> class Queue {
private:
  struct Message {
    Tag tag;
    Body body;
  };

  QueueHandle_t queueHandle;

public:
  Queue() : queueHandle(xQueueCreate(Capacity, sizeof(Message))) {
    if (queueHandle == nullptr) {
      ESP_LOGE("queue", "Unable to create queue");
      abort();
    }
  }

  ~Queue() {
    if (queueHandle != nullptr) {
      vQueueDelete(queueHandle);
    }
  }

  Queue(const Queue &) = delete;
  Queue &operator=(const Queue &) = delete;

  Queue(Queue &&other) noexcept : queueHandle(other.queueHandle) {
    other.queueHandle = nullptr;
  }

  Queue &operator=(Queue &&) = delete;

  void push(Tag tag, Body body) {
    // xQueueSend copies the message into the queue's own storage, so a local
    // is enough -- and std::move would leave the copied-from bytes behind
    const Message msg{.tag = tag, .body = body};

    if (xQueueSend(queueHandle, &msg, portMAX_DELAY) != pdPASS) {
      ESP_LOGE("queue", "Unable to send to queue");
      abort();
    }
  }

  void pop(Tag &tag, Body &body) {
    Message msg;

    if (xQueueReceive(queueHandle, &msg, portMAX_DELAY) != pdPASS) {
      ESP_LOGE("queue", "Unable to read from queue");
      abort();
    }

    tag = msg.tag;
    body = msg.body;
  }
};

#endif
