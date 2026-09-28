#ifndef ROS_H
#define ROS_H

#include <rmw/qos_profiles.h>
#include <rmw_microxrcedds_c/config.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <concepts>
#include <cstddef>

namespace ROS {
template <size_t N> struct name {
  char value[N]{};

  consteval name(const char (&literal)[N]) {
    for (size_t i = 0; i < N; ++i) {
      value[i] = literal[i];
    }
  }

  constexpr operator const char *() const { return value; }
  static constexpr size_t size = N - 1;
};

template <typename Msg> struct message;

template <typename Msg>
concept Described = requires {
  { message<Msg>::type_support() } -> std::same_as<
      const rosidl_message_type_support_t *>;
};

// RELIABLE is declarable but has not worked against this libmicroros build --
// the agent refuses the datareader and rcl reports a bare RMW_RET_ERROR. The
// middleware is compiled with RMW_UXRCE_MAX_HISTORY 1 while the reliable
// profile asks for depth 10, which is the likeliest cause but unconfirmed
enum class QoS { SENSOR_DATA, RELIABLE };

constexpr const rmw_qos_profile_t *profile_for(QoS qos) {
  return qos == QoS::RELIABLE ? &rmw_qos_profile_default
                              : &rmw_qos_profile_sensor_data;
}

struct subscription_kind {};
struct publisher_kind {};

template <Described Msg, name Topic, QoS Q = QoS::SENSOR_DATA>
struct subscribes {
  using kind = subscription_kind;
  using message_type = Msg;

  static constexpr auto topic = Topic;
  static constexpr QoS qos = Q;
};

template <Described Msg, name Topic, QoS Q = QoS::SENSOR_DATA>
struct publishes {
  using kind = publisher_kind;
  using message_type = Msg;

  static constexpr auto topic = Topic;
  static constexpr QoS qos = Q;
};

template <typename T>
concept Subscribes = std::same_as<typename T::kind, subscription_kind>;

template <typename T>
concept Publishes = std::same_as<typename T::kind, publisher_kind>;

template <typename T>
concept Declaration = Subscribes<T> || Publishes<T>;

namespace detail {
template <Declaration... Decls> consteval bool topics_unique() {
  if constexpr (sizeof...(Decls) == 0) {
    return true;
  } else {
    const char *topics[]{Decls::topic...};

    for (size_t i = 0; i < sizeof...(Decls); ++i) {
      for (size_t j = i + 1; j < sizeof...(Decls); ++j) {
        const char *a = topics[i];
        const char *b = topics[j];

        while (*a != '\0' && *a == *b) {
          ++a;
          ++b;
        }

        if (*a == *b) {
          return false;
        }
      }
    }

    return true;
  }
}
} // namespace detail

template <name Name, name Namespace, Declaration... Decls> struct node {
  static constexpr auto node_name = Name;
  static constexpr auto node_namespace = Namespace;

  static constexpr size_t subscriptions = (0 + ... + (Subscribes<Decls> ? 1 : 0));
  static constexpr size_t publishers = (0 + ... + (Publishes<Decls> ? 1 : 0));

  static_assert(subscriptions <= RMW_UXRCE_MAX_SUBSCRIPTIONS,
                "more subscriptions than libmicroros was built for -- raise "
                "RMW_UXRCE_MAX_SUBSCRIPTIONS in colcon.meta and rebuild");
  static_assert(publishers <= RMW_UXRCE_MAX_PUBLISHERS,
                "more publishers than libmicroros was built for -- raise "
                "RMW_UXRCE_MAX_PUBLISHERS in colcon.meta and rebuild");
  static_assert(detail::topics_unique<Decls...>(),
                "two declarations share a topic name");
};
} // namespace ROS

#define SWARM_ROS_MESSAGE(Pkg, Subfolder, Msg)                                 \
  namespace ROS {                                                              \
  template <> struct message<Pkg##__##Subfolder##__##Msg> {                    \
    static const rosidl_message_type_support_t *type_support() {               \
      return ROSIDL_GET_MSG_TYPE_SUPPORT(Pkg, Subfolder, Msg);                 \
    }                                                                          \
  };                                                                           \
  }

#endif
