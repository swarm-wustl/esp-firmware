#ifndef CONSUMER_H
#define CONSUMER_H

#include <type_traits>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor.h"
#include "queue.h"
#include "swarm_hal.h"

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

template <HAL::MotorDriverTrait MotorDriver>
void spin(MotorDriver &driver, QueueType &queue) {
  // TODO: set some sort of frequency for this to be called
  while (1) {
    MessageTag tag;
    MessageBody body;

    queue.pop(tag, body);

    switch (tag) {
    case MessageTag::MOTOR_COMMAND: {
      Motor::Command cmd = body.motor_cmd;
      driver.run(cmd);
      break;
    }

    default: {
      ESP_LOGE("consumer", "Unhandled message type: tag=%d",
               static_cast<int>(tag));
      abort();
    }
    }
  }
}
} // namespace Consumer

#endif
