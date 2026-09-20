// Written with Claude
#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <concepts>
#include <expected>
#include <optional>
#include <type_traits>
#include <utility>

enum class QueueError : uint8_t { Full, Closed };

// xQueueSend/xQueueReceive memcpy the item in and out, so anything with a
// non-trivial copy, move or destructor would be silently torn apart
template <typename T>
concept Payload = std::is_trivially_copyable_v<T>;

template <typename Tag, typename Body, size_t Capacity>
  requires Payload<Tag> && Payload<Body>
class Queue {
public:
  struct Message {
    Tag tag;
    Body body;
  };

private:
  QueueHandle_t handle;

  explicit Queue(QueueHandle_t h) : handle(h) {}

public:
  [[nodiscard]] static std::optional<Queue> create() {
    QueueHandle_t h = xQueueCreate(Capacity, sizeof(Message));

    if (h == nullptr) {
      return std::nullopt;
    }

    return Queue{h};
  }

  ~Queue() {
    if (handle != nullptr) {
      vQueueDelete(handle);
    }
  }

  Queue(const Queue &) = delete;
  Queue &operator=(const Queue &) = delete;

  Queue(Queue &&other) noexcept : handle(std::exchange(other.handle, nullptr)) {}

  Queue &operator=(Queue &&other) noexcept {
    if (this != &other) {
      if (handle != nullptr) {
        vQueueDelete(handle);
      }
      handle = std::exchange(other.handle, nullptr);
    }
    return *this;
  }

  [[nodiscard]] std::expected<void, QueueError>
  push(Tag tag, Body body, TickType_t timeout = portMAX_DELAY) {
    if (handle == nullptr) {
      return std::unexpected{QueueError::Closed};
    }

    // xQueueSend copies the message into the queue's own storage, so a local
    // is enough -- and std::move would leave the copied-from bytes behind
    const Message msg{.tag = tag, .body = body};

    if (xQueueSend(handle, &msg, timeout) != pdPASS) {
      return std::unexpected{QueueError::Full};
    }

    return {};
  }

  [[nodiscard]] std::optional<Message> pop(TickType_t timeout = portMAX_DELAY) {
    Message msg;

    if (handle == nullptr || xQueueReceive(handle, &msg, timeout) != pdPASS) {
      return std::nullopt;
    }

    return msg;
  }
};

#endif