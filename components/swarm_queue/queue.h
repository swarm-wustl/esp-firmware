#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
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

  // xQueueOverwrite is only defined for a queue of length one
  void overwrite(const Body &body)
    requires(Capacity == 1)
  {
    xQueueOverwrite(handle, &body);
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

// a wake-up with no payload, raised from an ISR. Binary, so edges that land
// while the waiter is busy collapse into one -- the waiter has to read whatever
// status the hardware latched, not count wake-ups
class Signal {
public:
  Signal() : handle(xSemaphoreCreateBinaryStatic(&control)) {}

  ~Signal() { vSemaphoreDelete(handle); }

  Signal(const Signal &) = delete;
  Signal &operator=(const Signal &) = delete;
  Signal(Signal &&) = delete;
  Signal &operator=(Signal &&) = delete;

  void raise() { xSemaphoreGive(handle); }

  void raise_from_isr() {
    BaseType_t woken = pdFALSE;

    xSemaphoreGiveFromISR(handle, &woken);
    portYIELD_FROM_ISR(woken);
  }

  [[nodiscard]] bool wait(TickType_t timeout = portMAX_DELAY) {
    return xSemaphoreTake(handle, timeout) == pdTRUE;
  }

private:
  StaticSemaphore_t control{};
  SemaphoreHandle_t handle;
};

#endif