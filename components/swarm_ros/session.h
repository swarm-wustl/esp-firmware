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
  bool created = false;
};

template <Publishes Decl> struct entity<Decl> {
  using declaration = Decl;

  rcl_publisher_t handle{};
  bool created = false;
};

template <Streams Decl> struct entity<Decl> {
  using declaration = Decl;
  using channel_type = Queue<typename Decl::payload_type, Decl::depth>;

  rcl_publisher_t handle{};
  typename Decl::message_type message{};
  channel_type channel{};
  bool created = false;
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
    allocator_ = rcl_get_default_allocator();

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();

    if (!check(rcl_init_options_init(&options, allocator_), "init_options")) {
      return false;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    if (!check(rmw_uros_options_set_udp_address(
                   CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT,
                   rcl_init_options_get_rmw_init_options(&options)),
               "agent address")) {
      rcl_init_options_fini(&options);

      return false;
    }
#endif

    const bool supported = check(
        rclc_support_init_with_options(&support, 0, nullptr, &options,
                                       &allocator_),
        "support");

    // rcl_init copies the options into the context, so the local set is spent
    // either way -- and leaking it costs a retry its memory
    rcl_init_options_fini(&options);

    if (!supported) {
      return false;
    }

    stage_ = stage::support;

    if (!check(rclc_node_init_default(&handle, Name, Namespace, &support),
               "node")) {
      fini();

      return false;
    }

    stage_ = stage::node;

    if (!check(rclc_executor_init(&executor, &support.context, executor_handles,
                                  &allocator_),
               "executor")) {
      fini();

      return false;
    }

    stage_ = stage::executor;

    bool ok = true;

    std::apply([&](auto &...e) { ((ok = ok && create(e)), ...); }, entities);

    if (!ok) {
      fini();

      return false;
    }

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    // the ESP32 boots at epoch 0, so a header stamp is meaningless until the
    // agent's clock is borrowed. Losing it costs stamps, not the session
    if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
      synchronised = false;
    }
#endif

    stage_ = stage::running;

    return true;
  }

  // unwinds whatever init() managed to build, in reverse, so a failed attempt
  // can be retried instead of costing the caller its task
  void fini() {
    switch (stage_) {
    case stage::running:
    case stage::executor:
      std::apply([&](auto &...e) { (destroy(e), ...); }, entities);
      rclc_executor_fini(&executor);
      [[fallthrough]];

    case stage::node:
      rcl_node_fini(&handle);
      [[fallthrough]];

    case stage::support:
      rclc_support_fini(&support);
      [[fallthrough]];

    case stage::none:
      break;
    }

    stage_ = stage::none;
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
    if (stage_ != stage::running) {
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
    const rcl_ret_t rc = rclc_executor_spin_some(&executor, 0);

    // an executor that fails every cycle looks exactly like a topic nobody
    // publishes to, so the one return value that can tell them apart is worth
    // reading. RCL_RET_TIMEOUT just means nothing was ready
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
      check(rc, "spin");
    }

    std::apply([&](auto &...e) { (flush(e), ...); }, entities);
  }

  // XRCE pushes subscription data unsolicited, so a client whose outbound
  // traffic stops -- a best-effort reader sends nothing on its own -- loses
  // any NAT mapping between it and the agent, and the push goes nowhere
  [[nodiscard]] bool ping(int timeout_ms = 100) {
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    return rmw_uros_ping_agent(timeout_ms, 1) == RMW_RET_OK;
#else
    return true;
#endif
  }

  [[nodiscard]] bool synced() const { return synchronised; }
  [[nodiscard]] uint32_t dropped_publishes() const { return dropped; }

private:
  using description = node<Name, Namespace, Decls...>;

  // construction is strictly ordered, so how far init() got is also exactly
  // what fini() has to undo. A producer on another task may reach send()
  // before any of it exists, which is what anything short of running means
  enum class stage : uint8_t { none, support, node, executor, running };

  // rclc rejects a zero-handle executor, and a node may legitimately be
  // publish-only
  static constexpr size_t executor_handles =
      description::subscriptions == 0 ? 1 : description::subscriptions;

  template <typename Entity> bool create(Entity &e) {
    using Decl = typename Entity::declaration;

    const rosidl_message_type_support_t *type_support =
        message<typename Decl::message_type>::type_support();

    if constexpr (Subscribes<Decl>) {
      e.created =
          check(rclc_subscription_init(&e.handle, &handle, type_support,
                                       Decl::topic, profile_for(Decl::qos)),
                Decl::topic);

      return e.created;
    } else {
      if (!check(rclc_publisher_init(&e.handle, &handle, type_support,
                                     Decl::topic, profile_for(Decl::qos)),
                 Decl::topic)) {
        return false;
      }

      e.created = true;

      if constexpr (Streams<Decl>) {
        if constexpr (Stamped<typename Decl::message_type>) {
          rosidl_runtime_c__String__assign(&e.message.header.frame_id,
                                           Decl::frame_id);
        }
      }

      return true;
    }
  }

  template <typename Entity> void destroy(Entity &e) {
    using Decl = typename Entity::declaration;

    if (!e.created) {
      return;
    }

    if constexpr (Subscribes<Decl>) {
      rcl_subscription_fini(&e.handle, &handle);
    } else {
      rcl_publisher_fini(&e.handle, &handle);
    }

    e.created = false;
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

  // rclc_support_t and rclc_executor_t both store the allocator by POINTER and
  // dereference it long after init() returns, so it cannot be an automatic
  rcl_allocator_t allocator_{};
  rclc_support_t support{};
  rcl_node_t handle{};
  rclc_executor_t executor{};
  std::tuple<detail::entity<Decls>...> entities{};
  uint32_t dropped = 0;
  bool synchronised = true;
  stage stage_ = stage::none;
};
} // namespace ROS

#endif
