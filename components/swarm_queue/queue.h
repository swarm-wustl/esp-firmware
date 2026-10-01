#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <concepts>
#include <expected>
#include <optional>
#include <type_traits>

enum class QueueError : uint8_t { Full };

// xQueueSend/xQueueReceive memcpy the item in and out, so anything with a
// non-trivial copy, move or destructor would be silently torn apart
template <typename T>
concept Payload = std::is_trivially_copyable_v<T>;

template <typename Body, size_t Capacity>
  requires Payload<Body>
class Queue {
public:
  using value_type = Body;

  // static storage, so creation cannot fail and there is no such thing as a
  // channel that does not exist yet
  Queue()
      : handle(xQueueCreateStatic(Capacity, sizeof(Body), storage, &control)) {}

  ~Queue() { vQueueDelete(handle); }

  Queue(const Queue &) = delete;
  Queue &operator=(const Queue &) = delete;

  // the queue's storage lives in this object, so the handle cannot outlive its
  // address
  Queue(Queue &&) = delete;
  Queue &operator=(Queue &&) = delete;

  [[nodiscard]] std::expected<void, QueueError>
  push(const Body &body, TickType_t timeout = portMAX_DELAY) {
    if (xQueueSend(handle, &body, timeout) != pdPASS) {
      return std::unexpected{QueueError::Full};
    }

    return {};
  }

  [[nodiscard]] std::optional<Body> pop(TickType_t timeout = portMAX_DELAY) {
    Body body;

    if (xQueueReceive(handle, &body, timeout) != pdPASS) {
      return std::nullopt;
    }

    return body;
  }

private:
  StaticQueue_t control{};
  alignas(Body) uint8_t storage[Capacity * sizeof(Body)]{};
  QueueHandle_t handle;
};

#endif