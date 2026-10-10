# `Sched::init` — setup as a pipeline stage

Design note, not yet implemented. Kills the two TODOs in `main/src/main.cpp`
(`// TODO: encapsulate this`, `// TODO: maybe make this part of construction so
we don't have two-phase?`) and gives micro-ROS agent reconnection as a side
effect.

## Problem

`task::on_start` is one `bool (*)()` per **task**, bolted on after construction:

```cpp
static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
    Sched::every<HW::EXECUTOR_PERIOD_MS> |
    Sched::to([] { return session.poll(); }));

ros.on_start([] {
  while (!session.init() || !session.on<HW::CmdVel>(onTwist)) { ... }
  return true;
});
```

Three things wrong with it:

- Setup lives apart from the pipeline that needs it, so the declaration is a
  task plus a lambda rather than one thing.
- One hook per task means two pipelines in a task cannot each have their own
  setup.
- Returning `false` deletes the task permanently (`swarm_sched.h`, `run()`), so
  the retry loop has to live inside the hook.

It cannot simply move into construction: init has to run **on the uros task**,
not on whoever calls `spawn()`, so a dead agent does not stop `motion` from
spawning. The two-phase split is real; the fix is to make it invisible, not to
remove it.

## Design

A third stage kind alongside `then` and `to`. `stage<F, Terminal>` already tags
stages by kind -- `to` is `then` with `Terminal = true` -- so this is the same
move.

```cpp
template <typename F> struct setup_stage { F fn; };
template <typename F> constexpr auto init(F fn) { return setup_stage<F>{fn}; }
```

### The name decides, not the position

`init` is a setup stage wherever it appears in the chain. Both of these compile
and mean the same thing:

```cpp
Sched::init([] { return session.init(); })
  | Sched::every<HW::EXECUTOR_PERIOD_MS>
  | Sched::init([] { return session.on<HW::CmdVel>(onTwist); })
  | Sched::to([] { return session.poll(); })
```

Letting position alone decide (left of the source = once, right = per tick)
would read better but is positional correspondence, which CLAUDE.md says to
design out. Order *among* setup stages still matters -- `session.init()` must
precede `session.on<...>()` -- and tuple order preserves it on either side.

### Run until it succeeds, not run once

A failed setup means the pipe skips its data path this tick and retries next
tick. This subsumes the retry loop and yields reconnection: if `poll()` ever
reports failure, clear `ready_` and the next tick re-establishes the session.

```cpp
bool ready_ = false;

void run() {
  if (!ready_) {
    ready_ = start();
    if (!ready_) { return; }
  }

  source_.poll(...);
}
```

## Implementation

Keep one `stages_` tuple and filter both traversals rather than carrying a
second parameter pack.

- `step<I>` skips setup stages: `if constexpr (detail::is_setup_v<StageAt>)
  step<I + 1>(std::forward<V>(value));` -- the value passes through untouched,
  and `Result` is only computed for non-setup stages.
- `start()` folds over only the setup stages, in tuple order, short-circuiting
  on failure. `is_expected_v` / `yields_nothing_v` already handle the return
  side.
- One extra `operator|` for the pre-source form; a setup at index 0 of
  `stages_` is harmless because `step` skips it:

```cpp
template <typename F, typename Src, typename... Stages>
constexpr auto operator|(setup_stage<F> setup, pipeline<Src, Stages...> pipe);
```

New `static_assert`s, in the spirit of the existing ones: a setup stage must be
invocable with no arguments, must return `void`, `bool` or
`expected<void, E>`, and cannot be terminal.

Zero overhead holds -- all `if constexpr`, one `bool` test per pipe per tick.

## Open points

- **A blocking `init()` overruns the period.** `session.init()` takes seconds
  against a dead agent (XRCE's own retry timeouts), so the tick that attempts it
  blows a 10ms period. Survivable on the uros task since it runs alone and
  `vTaskDelayUntil` catches up, but the advertised period is fiction while
  disconnected. A backoff counter fixes it at the cost of state in `pipeline`.
- **`init(a) | init(b) | every<10>` does not parse** -- `|` is left-associative
  and neither operand is a pipeline. Either accept a single pre-source `init`
  (the rest go after the source; the name still carries the meaning) or add a
  `setup_group<F...>` with its own `|`. Not worth the two extra types until
  something needs it.
- Worth a comment at the declaration: a pre-source `init` still runs **on the
  task**, not at construction. Putting it visually first makes that easier to
  misread.

## Files

- `components/swarm_sched/swarm_sched.h` -- `setup_stage`, `init`, both `|`
  overloads, filtered `step`/`start`, `ready_`; delete `on_start`, `setup_fn`,
  `setup_` and the setup call in `run()`.
- `main/src/main.cpp` -- fold the `ros.on_start` block into the pipeline; drop
  both TODOs.
