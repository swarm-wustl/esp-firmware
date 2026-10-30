#ifndef SWARM_SCHED_H
#define SWARM_SCHED_H

#include "queue.h"
#include "watchdog.h"

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

using signal = Signal;

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

constexpr uint32_t gcd(uint32_t a, uint32_t b) {
  return b == 0 ? a : gcd(b, a % b);
}

constexpr uint32_t min_of(uint32_t a, uint32_t b) { return a < b ? a : b; }

// the tick counter wraps, so a deadline already behind `now` shows up as a huge
// unsigned distance
constexpr TickType_t ticks_until(TickType_t now, TickType_t deadline) {
  const TickType_t distance = deadline - now;

  return distance > portMAX_DELAY / 2 ? 0 : distance;
}
} // namespace detail

// a period alone is not a source -- a pipeline carries one too, and nothing
// should be able to stand in for the thing that emits the values
template <typename S>
concept Source = requires(S source) {
  S::period_ms;
  source.poll([](auto &&) {});
};

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

// a setpoint stands until the next one replaces it, and a closed loop has to
// act on it every period, not only on the period a new one arrives
template <uint32_t PeriodMs, typename Channel> struct held_source {
  static constexpr uint32_t period_ms = PeriodMs;

  Channel *channel;
  std::optional<typename Channel::value_type> held;

  template <typename F> void poll(F &&emit) {
    while (auto msg = channel->pop(0)) {
      held = msg;
    }

    if (held) {
      emit(*held);
    }
  }
};

// a held setpoint is only safe while whoever set it is still there: if the
// channel goes quiet for TimeoutMs, the source falls back to `safe` until the
// next message
template <uint32_t PeriodMs, uint32_t TimeoutMs, typename Channel>
struct watched_source {
  static_assert(TimeoutMs > PeriodMs,
                "a timeout within one period would expire between messages");

  using value_type = typename Channel::value_type;

  static constexpr uint32_t period_ms = PeriodMs;

  Channel *channel;
  value_type safe;
  Watchdog::Feed<value_type> feed;

  template <typename F> void poll(F &&emit) {
    std::optional<value_type> newest;

    while (auto msg = channel->pop(0)) {
      newest = msg;
    }

    feed = Watchdog::observe(
        feed, newest,
        {.safe = safe, .timeout_ms = TimeoutMs, .period_ms = PeriodMs});

    emit(feed.value);
  }
};

// no period: the task blocks on the signal and runs the pipeline as soon as it
// is raised, which is the whole reason to prefer this over polling a queue
struct signal_source {
  static constexpr uint32_t period_ms = 0;

  signal *raised;

  [[nodiscard]] bool wait(TickType_t timeout) { return raised->wait(timeout); }

  template <typename F> void poll(F &&emit) { emit(std::monostate{}); }
};

template <typename S>
concept Awaited = Source<S> && requires(S source, TickType_t timeout) {
  { source.wait(timeout) } -> std::same_as<bool>;
};

template <Source Src, typename F> struct started_source {
  static constexpr uint32_t period_ms = Src::period_ms;

  F fn;
  Src source;

  [[nodiscard]] bool start() { return static_cast<bool>(fn()); }

  [[nodiscard]] bool wait(TickType_t timeout)
    requires Awaited<Src>
  {
    return source.wait(timeout);
  }

  template <typename E> void poll(E &&emit) {
    source.poll(std::forward<E>(emit));
  }
};

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

template <typename F> struct startup {
  F fn;
};

// Either a boolean or a expected<void, E>
template <typename R>
concept StartResult = std::same_as<std::decay_t<R>, bool> ||
                      (detail::yields_nothing_v<R> && not std::is_void_v<R>);

// run once on the task itself, before its loop -- for work that must happen
// where the pipelines run rather than on the caller's task. A falsy result
// abandons the task
template <typename F>
  requires StartResult<std::invoke_result_t<F &>>
constexpr auto on_start(F fn) {
  return startup<F>{std::move(fn)};
}

template <Source Src, Stage... Stages> class pipeline {
public:
  static constexpr uint32_t period_ms = Src::period_ms;
  static constexpr size_t stages = sizeof...(Stages);
  static constexpr bool awaited = Awaited<Src>;

  constexpr pipeline(Src source, std::tuple<Stages...> stages)
      : source_(std::move(source)), stages_(std::move(stages)) {}

  void run() {
    source_.poll([this](auto &&value) {
      step<0>(std::forward<decltype(value)>(value));
    });
  }

  [[nodiscard]] bool wait(TickType_t timeout)
    requires awaited
  {
    return source_.wait(timeout);
  }

  [[nodiscard]] bool start() {
    if constexpr (requires { source_.start(); }) {
      return source_.start();
    } else {
      return true;
    }
  }

  [[nodiscard]] uint32_t drops() const { return drops_; }

  template <typename F> constexpr auto started_by(F fn) && {
    return pipeline<started_source<Src, F>>{{std::move(fn), std::move(source_)},
                                            {}};
  }

  template <Stage Next> constexpr auto append(Next next) && {
    static_assert(not ends_in_sink,
                  "a `to` stage is the end of the pipeline -- nothing can "
                  "follow it. Use `then` for a stage that feeds another");

    return pipeline<Src, Stages..., Next>{
        std::move(source_),
        std::tuple_cat(std::move(stages_), std::tuple{std::move(next)})};
  }

private:
  static constexpr bool ends_in_sink = [] {
    if constexpr (sizeof...(Stages) == 0) {
      return false;
    } else {
      return std::tuple_element_t<sizeof...(Stages) - 1,
                                  std::tuple<Stages...>>::terminal;
    }
  }();

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

        // TODO: some way to properly handle drops?
        if (!result) {
          ++drops_;
          return;
        }

        if constexpr (std::is_void_v<
                          typename std::decay_t<Result>::value_type>) {
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

// a source enters the chain already wrapped, so every link of the fold has a
// pipeline on the left and one overload covers all of them
template <Source Src, Stage... Stages, Stage Next>
constexpr auto operator|(pipeline<Src, Stages...> pipe, Next next) {
  return std::move(pipe).append(std::move(next));
}

// the bare pipeline on the right is what puts this before the source: stages
// attach to the left, so once one exists this overload no longer matches
template <typename F, Source Src>
constexpr auto operator|(startup<F> setup, pipeline<Src> pipe) {
  return std::move(pipe).started_by(std::move(setup.fn));
}

template <uint32_t PeriodMs>
inline constexpr auto every =
    pipeline<timer_source<PeriodMs>>{timer_source<PeriodMs>{}, {}};

template <uint32_t PeriodMs, typename Channel>
constexpr auto from(Channel &channel) {
  return pipeline<drain_source<PeriodMs, Channel>>{{&channel}, {}};
}

template <uint32_t PeriodMs, typename Channel>
constexpr auto latest(Channel &channel) {
  return pipeline<latest_source<PeriodMs, Channel>>{{&channel}, {}};
}

template <uint32_t PeriodMs, typename Channel>
constexpr auto
held(Channel &channel,
     std::optional<typename Channel::value_type> initial = std::nullopt) {
  return pipeline<held_source<PeriodMs, Channel>>{{&channel, initial}, {}};
}

template <uint32_t PeriodMs, uint32_t TimeoutMs, typename Channel>
constexpr auto watched(Channel &channel, typename Channel::value_type safe) {
  return pipeline<watched_source<PeriodMs, TimeoutMs, Channel>>{
      {&channel, safe, {safe, 0}}, {}};
}

constexpr auto on(signal &raised) {
  return pipeline<signal_source>{signal_source{&raised}, {}};
}

template <uint32_t StackBytes, UBaseType_t Priority, typename... Pipes>
class task {
public:
  static constexpr uint32_t base_ms =
      (Pipes::period_ms | ... | 0) == 0 ? 1 : [] {
        uint32_t g = 0;
        ((g = detail::gcd(g, Pipes::period_ms)), ...);
        return g;
      }();

  static constexpr uint32_t shortest_ms = [] {
    uint32_t m = UINT32_MAX;
    ((m = Pipes::period_ms > 0 ? detail::min_of(m, Pipes::period_ms) : m), ...);
    return m;
  }();

  static constexpr bool periodic = ((Pipes::period_ms > 0) || ...);
  static constexpr size_t awaited = (size_t{Pipes::awaited} + ... + 0);

  static_assert(sizeof...(Pipes) > 0, "a task with no pipelines does nothing");
  static_assert(((Pipes::stages > 0) && ...),
                "a pipeline with no stages polls its source and throws every "
                "value away -- give it a `to`");
  static_assert(awaited <= 1,
                "a task blocks on one thing at a time -- share one signal "
                "between the interrupts, or give each pipeline its own task");
  static_assert(not periodic || base_ms == shortest_ms,
                "every period in a task must be a multiple of the shortest one "
                "-- otherwise the derived wake period collapses toward 1ms. "
                "Adjust the period or give the pipeline its own task");

  constexpr explicit task(Pipes... pipes) : pipes_(std::move(pipes)...) {}

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
    if (!std::apply([](auto &...pipe) { return (pipe.start() && ...); },
                    pipes_)) {
      return;
    }

    if constexpr (awaited > 0) {
      serve();
    } else {
      keep_time();
    }
  }

  [[nodiscard]] uint32_t overruns() const { return overruns_; }

private:
  void keep_time() {
    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0;

    while (true) {
      vTaskDelayUntil(&last, pdMS_TO_TICKS(base_ms));

      ++tick;

      std::apply([&](auto &...pipe) { (dispatch(pipe, tick), ...); }, pipes_);
    }
  }

  // the wait is bounded by the next period, so a periodic pipeline sharing this
  // task keeps its rate with no interrupt arriving at all
  void serve() {
    const TickType_t period = pdMS_TO_TICKS(base_ms);

    TickType_t deadline = xTaskGetTickCount() + period;
    uint32_t tick = 0;

    while (true) {
      const TickType_t timeout =
          periodic ? detail::ticks_until(xTaskGetTickCount(), deadline)
                   : portMAX_DELAY;

      std::apply([&](auto &...pipe) { (await(pipe, timeout), ...); }, pipes_);

      if (periodic && detail::ticks_until(xTaskGetTickCount(), deadline) == 0) {
        deadline += period;
        ++tick;

        std::apply([&](auto &...pipe) { (dispatch(pipe, tick), ...); }, pipes_);
      }
    }
  }

  template <typename Pipe> void await(Pipe &pipe, TickType_t timeout) {
    if constexpr (Pipe::awaited) {
      if (pipe.wait(timeout)) {
        pipe.run();
      }
    }
  }

  template <typename Pipe> void dispatch(Pipe &pipe, uint32_t tick) {
    if constexpr (not Pipe::awaited) {
      constexpr uint32_t divisor = Pipe::period_ms / base_ms;

      if (tick % divisor == 0) {
        pipe.run();
      }
    }
  }

  std::tuple<Pipes...> pipes_;
  uint32_t overruns_ = 0;
};

template <uint32_t StackBytes, UBaseType_t Priority, typename... Pipes>
[[nodiscard]] constexpr auto make_task(Pipes... pipes) {
  return task<StackBytes, Priority, Pipes...>{std::move(pipes)...};
}
} // namespace Sched

#endif
