#ifndef CONSUMER_H
#define CONSUMER_H

#include <optional>

#include "drive.h"
#include "queue.h"
#include "swarm_hal.h"

namespace Consumer {
constexpr size_t CONSUMER_QUEUE_SIZE = 25;

enum class MessageTag {
  MOTOR_FRAME,
  // TODO: IMU read, send data to ROS server, etc.
};

template <auto Names> union MessageBody {
  Drive::Frame<Names> motor_frame;
};

template <auto Names>
using QueueType = Queue<MessageTag, MessageBody<Names>, CONSUMER_QUEUE_SIZE>;

template <auto Names, HAL::MotorDriverTrait<Names> MotorDriver>
void spin(MotorDriver &driver, QueueType<Names> &queue) {
  // TODO: set some sort of frequency for this to be called
  while (1) {
    std::optional<typename QueueType<Names>::Message> msg = queue.pop();

    if (!msg) {
      continue;
    }

    switch (msg->tag) {
    case MessageTag::MOTOR_FRAME:
      driver.run(msg->body.motor_frame);
      break;
    }
  }
}
} // namespace Consumer

#endif
