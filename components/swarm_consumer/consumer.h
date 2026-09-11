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
#include <optional>

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
  while (true) {
    std::optional<QueueType::Message> msg = queue.pop();

    if (!msg) {
      continue;
    }

    switch (msg->tag) {
    case MessageTag::MOTOR_COMMAND: {
      if (!driver.run(msg->body.motor_cmd)) {
        ESP_LOGE("consumer", "Unable to run command for motor %d",
                 static_cast<int>(msg->body.motor_cmd.name));
      }
      break;
    }

    default: {
      ESP_LOGE("consumer", "Unhandled message type: tag=%d",
               static_cast<int>(msg->tag));
      break;
    }
    }
  }
}
} // namespace Consumer

#endif
