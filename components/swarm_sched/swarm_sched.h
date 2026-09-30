#ifndef SWARM_SCHED_H
#define SWARM_SCHED_H

#include "queue.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace Sched {
template <typename Body, size_t Capacity> using channel = Queue<Body, Capacity>;

namespace detail {
template <typename T> struct is_expected : std::false_type {};

template <typename T, typename E>
struct is_expected<std::expected<T, E>> : std::true_type {};

template <typename T>
inline constexpr bool is_expected_v = is_expected<std::decay_t<T>>::value;

template <typename T> struct yields_nothing : std::is_void<T> {};

template <typename E>
struct yields_nothing<std::expected<void, E>> : std::true_type {};

template <typename T>
inline constexpr bool yields_nothing_v = yields_nothing<std::decay_t<T>>::value;

template <typename F, typename V>
constexpr decltype(auto) invoke_stage(F &fn, V &&value) {
  if constexpr (std::same_as<std::decay_t<V>, std::monostate>) {
    return fn();
  } else {
    return fn(std::forward<V>(value));
  }
}

consteval uint32_t gcd(uint32_t a, uint32_t b) {
  return b == 0 ? a : gcd(b, a % b);
}

consteval uint32_t min_of(uint32_t a, uint32_t b) { return a < b ? a : b; }
} // namespace detail

template <typename S>
concept Source = requires { S::period_ms; };

template <uint32_t PeriodMs> struct timer_source {
  static_assert(PeriodMs > 0, "a period of zero would never yield");

  static constexpr uint32_t period_ms = PeriodMs;

  template <typename F> void poll(F &&emit) { emit(std::monostate{}); }
};

template <uint32_t PeriodMs, typename Channel> struct drain_source {
  static constexpr uint32_t period_ms = PeriodMs;

  Channel *channel;

  template <typename F> void poll(F &&emit) {
    while (auto msg = channel->pop(0)) {
      emit(*std::move(msg));
    }
  }
};

// a velocity command supersedes the one before it, so the queue is drained and
// only the newest survives -- acting on a backlog would replay stale motion
template <uint32_t PeriodMs, typename Channel> struct latest_source {
  static constexpr uint32_t period_ms = PeriodMs;

  Channel *channel;

  template <typename F> void poll(F &&emit) {
    std::optional<typename Channel::value_type> newest;

    while (auto msg = channel->pop(0)) {
      newest = msg;
    }

    if (newest) {
      emit(*std::move(newest));
    }
  }
};

template <uint32_t PeriodMs> inline constexpr timer_source<PeriodMs> every{};

template <uint32_t PeriodMs, typename Channel>
constexpr auto from(Channel &channel) {
  return drain_source<PeriodMs, Channel>{&channel};
}

template <uint32_t PeriodMs, typename Channel>
constexpr auto latest(Channel &channel) {
  return latest_source<PeriodMs, Channel>{&channel};
}

template <typename F, bool Terminal> struct stage {
  static constexpr bool terminal = Terminal;

  F fn;

  template <typename V> decltype(auto) operator()(V &&value) {
    return detail::invoke_stage(fn, std::forward<V>(value));
  }
};

template <typename S>
concept Stage = requires { S::terminal; };

template <typename F> constexpr auto then(F fn) {
  return stage<F, false>{std::move(fn)};
}

template <typename F> constexpr auto to(F fn) {
  return stage<F, true>{std::move(fn)};
}

template <Source Src, Stage... Stages> class pipeline {
public:
  static constexpr uint32_t period_ms = Src::period_ms;

  constexpr pipeline(Src source, std::tuple<Stages...> stages)
      : source_(std::move(source)), stages_(std::move(stages)) {}

  void run() {
    source_.poll([this](auto &&value) { step<0>(std::forward<decltype(value)>(value)); });
  }

  [[nodiscard]] uint32_t drops() const { return drops_; }

  template <Stage Next> constexpr auto append(Next next) && {
    static_assert(not ends_in_sink,
                  "a `to` stage is the end of the pipeline -- nothing can "
                  "follow it. Use `then` for a stage that feeds another");

    return pipeline<Src, Stages..., Next>{
        std::move(source_),
        std::tuple_cat(std::move(stages_), std::tuple{std::move(next)})};
  }

private:
  static constexpr bool ends_in_sink =
      std::tuple_element_t<sizeof...(Stages) - 1,
                           std::tuple<Stages...>>::terminal;

  template <size_t I, typename V> void step(V &&value) {
    if constexpr (I == sizeof...(Stages)) {
      return;
    } else {
      using StageAt = std::tuple_element_t<I, std::tuple<Stages...>>;
      using Result = decltype(std::get<I>(stages_)(std::forward<V>(value)));

      static_assert(not StageAt::terminal || detail::yields_nothing_v<Result>,
                    "a `to` stage has nowhere to send a value, so it must "
                    "return void or expected<void, E>. Use `then` instead");

      static_assert(not std::is_void_v<Result> || I + 1 == sizeof...(Stages),
                    "a stage returning void ends the pipeline, so the stages "
                    "after it would never run. Return a value, or move it last "
                    "and write it as `to`");

      if constexpr (std::is_void_v<Result>) {
        std::get<I>(stages_)(std::forward<V>(value));
      } else if constexpr (detail::is_expected_v<Result>) {
        auto result = std::get<I>(stages_)(std::forward<V>(value));

        if (!result) {
          ++drops_;
          return;
        }

        if constexpr (std::is_void_v<typename std::decay_t<Result>::value_type>) {
          step<I + 1>(std::monostate{});
        } else {
          step<I + 1>(*std::move(result));
        }
      } else {
        step<I + 1>(std::get<I>(stages_)(std::forward<V>(value)));
      }
    }
  }

  Src source_;
  std::tuple<Stages...> stages_;
  uint32_t drops_ = 0;
};

template <Source Src, Stage Next> constexpr auto operator|(Src source, Next next) {
  return pipeline<Src, Next>{std::move(source), std::tuple{std::move(next)}};
}

template <Source Src, Stage... Stages, Stage Next>
constexpr auto operator|(pipeline<Src, Stages...> pipe, Next next) {
  return std::move(pipe).append(std::move(next));
}

template <uint32_t StackBytes, UBaseType_t Priority, typename... Pipes>
class task {
public:
  static constexpr uint32_t base_ms =
      (Pipes::period_ms | ... | 0) == 0
          ? 1
          : [] {
              uint32_t g = 0;
              ((g = detail::gcd(g, Pipes::period_ms)), ...);
              return g;
            }();

  static constexpr uint32_t shortest_ms = [] {
    uint32_t m = UINT32_MAX;
    ((m = detail::min_of(m, Pipes::period_ms)), ...);
    return m;
  }();

  static_assert(sizeof...(Pipes) > 0, "a task with no pipelines does nothing");
  static_assert(base_ms == shortest_ms,
                "every period in a task must be a multiple of the shortest one "
                "-- otherwise the derived wake period collapses toward 1ms. "
                "Adjust the period or give the pipeline its own task");

  using setup_fn = bool (*)();

  constexpr explicit task(Pipes... pipes) : pipes_(std::move(pipes)...) {}

  // run once on the task itself, before the loop -- for work that must happen
  // where the pipelines run rather than on the caller's task. Returning false
  // abandons the task
  void on_start(setup_fn setup) { setup_ = setup; }

  // the task body dereferences `this`, so the object has to outlive the task --
  // static storage, not a local
  void spawn(const char *name) {
    xTaskCreate(
        [](void *self) {
          static_cast<task *>(self)->run();
          vTaskDelete(nullptr);
        },
        name, StackBytes, this, Priority, nullptr);
  }

  void run() {
    if (setup_ != nullptr && !setup_()) {
      return;
    }

    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0;

    while (true) {
      vTaskDelayUntil(&last, pdMS_TO_TICKS(base_ms));

      ++tick;

      std::apply([&](auto &...pipe) { (dispatch(pipe, tick), ...); }, pipes_);
    }
  }

  [[nodiscard]] uint32_t overruns() const { return overruns_; }

private:
  template <typename Pipe> void dispatch(Pipe &pipe, uint32_t tick) {
    constexpr uint32_t divisor = Pipe::period_ms / base_ms;

    if (tick % divisor == 0) {
      pipe.run();
    }
  }

  std::tuple<Pipes...> pipes_;
  setup_fn setup_ = nullptr;
  uint32_t overruns_ = 0;
};

template <uint32_t StackBytes, UBaseType_t Priority, typename... Pipes>
[[nodiscard]] constexpr auto make_task(Pipes... pipes) {
  return task<StackBytes, Priority, Pipes...>{std::move(pipes)...};
}
} // namespace Sched

#endif
