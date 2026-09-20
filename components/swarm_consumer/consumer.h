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

template <Drive::Style S> union MessageBody {
  Drive::Frame<S> motor_frame;
};

template <Drive::Style S>
using QueueType = Queue<MessageTag, MessageBody<S>, CONSUMER_QUEUE_SIZE>;

template <Drive::Style S, HAL::MotorDriverTrait<S> MotorDriver>
void spin(MotorDriver &driver, QueueType<S> &queue) {
  // TODO: set some sort of frequency for this to be called
  while (1) {
    std::optional<typename QueueType<S>::Message> msg = queue.pop();

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
