#ifndef ROS_SESSION_H
#define ROS_SESSION_H

#include "queue.h"
#include "ros.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <optional>
#include <tuple>
#include <utility>

namespace ROS {
bool check(rcl_ret_t status, const char *what);

namespace detail {
template <Declaration Decl> struct entity;

template <Subscribes Decl> struct entity<Decl> {
  using declaration = Decl;

  rcl_subscription_t handle{};
  typename Decl::message_type storage{};
};

template <Publishes Decl> struct entity<Decl> {
  using declaration = Decl;

  rcl_publisher_t handle{};
};

template <Streams Decl> struct entity<Decl> {
  using declaration = Decl;
  using channel_type = Queue<typename Decl::payload_type, Decl::depth>;

  rcl_publisher_t handle{};
  typename Decl::message_type message{};
  channel_type channel{};
};
} // namespace detail

template <typename Node> class session;

template <name Name, name Namespace, Declaration... Decls>
class session<node<Name, Namespace, Decls...>> {
public:
  session() = default;
  session(const session &) = delete;
  session &operator=(const session &) = delete;

  template <Declaration Decl>
  static constexpr bool declared = (std::same_as<Decl, Decls> || ...);

  [[nodiscard]] bool init() {
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();

    if (!check(rcl_init_options_init(&options, allocator), "init_options")) {
      return false;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    if (!check(rmw_uros_options_set_udp_address(
                   CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT,
                   rcl_init_options_get_rmw_init_options(&options)),
               "agent address")) {
      return false;
    }
#endif

    if (!check(rclc_support_init_with_options(&support, 0, nullptr, &options,
                                              &allocator),
               "support")) {
      return false;
    }

    if (!check(rclc_node_init_default(&handle, Name, Namespace, &support),
               "node")) {
      return false;
    }

    if (!check(rclc_executor_init(&executor, &support.context, executor_handles,
                                  &allocator),
               "executor")) {
      return false;
    }

    bool ok = true;

    std::apply([&](auto &...e) { ((ok = ok && create(e)), ...); }, entities);

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    // the ESP32 boots at epoch 0, so a header stamp is meaningless until the
    // agent's clock is borrowed. Losing it costs stamps, not the session
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
      synchronised = false;
    }
#endif

    up = ok;

    return ok;
  }

  template <Subscribes Decl, typename Handler>
    requires declared<Decl> &&
             std::invocable<Handler, const typename Decl::message_type &>
  [[nodiscard]] bool on(Handler &handler) {
    auto &e = std::get<detail::entity<Decl>>(entities);

    return check(
        rclc_executor_add_subscription_with_context(
            &executor, &e.handle, &e.storage,
            [](const void *msg, void *context) {
              (*static_cast<Handler *>(context))(
                  *static_cast<const typename Decl::message_type *>(msg));
            },
            &handler, ON_NEW_DATA),
        "add subscription");
  }

  // callable from any task: a queue push, no rcl contact. Timeout 0, so a
  // stalled network drops the sample rather than blocking the producer
  template <Streams Decl>
    requires declared<Decl>
  [[nodiscard]] bool send(const typename Decl::payload_type &payload) {
    if (!up) {
      return false;
    }

    auto &e = std::get<detail::entity<Decl>>(entities);

    return e.channel.push(payload, 0).has_value();
  }

  template <Publishes Decl>
    requires declared<Decl>
  [[nodiscard]] bool publish(const typename Decl::message_type &msg) {
    auto &e = std::get<detail::entity<Decl>>(entities);

    return check(rcl_publish(&e.handle, &msg, nullptr), "publish");
  }

  // one pass: service the executor, then publish whatever the producers queued.
  // Non-blocking -- the caller's period paces it
  void poll() {
    rclc_executor_spin_some(&executor, 0);

    std::apply([&](auto &...e) { (flush(e), ...); }, entities);
  }

  [[nodiscard]] bool synced() const { return synchronised; }
  [[nodiscard]] uint32_t dropped_publishes() const { return dropped; }

private:
  using description = node<Name, Namespace, Decls...>;

  // rclc rejects a zero-handle executor, and a node may legitimately be
  // publish-only
  static constexpr size_t executor_handles =
      description::subscriptions == 0 ? 1 : description::subscriptions;

  template <typename Entity> bool create(Entity &e) {
    using Decl = typename Entity::declaration;

    const rosidl_message_type_support_t *type_support =
        message<typename Decl::message_type>::type_support();

    if constexpr (Subscribes<Decl>) {
      return check(rclc_subscription_init(&e.handle, &handle, type_support,
                                          Decl::topic, profile_for(Decl::qos)),
                   Decl::topic);
    } else {
      if (!check(rclc_publisher_init(&e.handle, &handle, type_support,
                                     Decl::topic, profile_for(Decl::qos)),
                 Decl::topic)) {
        return false;
      }

      if constexpr (Streams<Decl>) {
        if constexpr (Stamped<typename Decl::message_type>) {
          rosidl_runtime_c__String__assign(&e.message.header.frame_id,
                                           Decl::frame_id);
        }
      }

      return true;
    }
  }

  template <typename Entity> void flush(Entity &e) {
    using Decl = typename Entity::declaration;

    if constexpr (Streams<Decl>) {
      while (auto sample = e.channel.pop(0)) {
        Decl::fill(e.message, *sample);

        if constexpr (Stamped<typename Decl::message_type>) {
          stamp(e.message);
        }

        if (rcl_publish(&e.handle, &e.message, nullptr) != RCL_RET_OK) {
          ++dropped;
        }
      }
    }
  }

  template <Stamped Msg> void stamp(Msg &msg) {
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    if (!synchronised) {
      return;
    }

    const int64_t now = rmw_uros_epoch_nanos();

    msg.header.stamp.sec = static_cast<int32_t>(now / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(now % 1000000000LL);
#else
    (void)msg;
#endif
  }

  rclc_support_t support{};
  rcl_node_t handle{};
  rclc_executor_t executor{};
  std::tuple<detail::entity<Decls>...> entities{};
  uint32_t dropped = 0;
  bool synchronised = true;

  // producers on other tasks may call send() before the uros task has brought
  // the session up; their samples have no publisher to go to yet
  bool up = false;
};
} // namespace ROS

#endif
