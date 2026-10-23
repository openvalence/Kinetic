# Kinetic

Kinetic is a jerk-limited motion planner for linear machines. It is a
header-only C++ library with no hardware dependencies, so it drops into any
project that can compile C++20. It plans every stroke on the OSSM Flagship
(through the Nucleus firmware), and you can use it on its own.

You give it knots: "be at this position at this time." It gives back a smooth
trajectory you can sample at any moment, and it never exceeds the velocity,
acceleration and jerk limits you set. So a script can ask for anything, and the
motor still gets a curve it can follow.

**[Try it in your browser](https://openvalence.org/Kinetic/)** | [Project page](https://openvalence.org/kinetic/) | [Docs](https://openvalence.org/Valence/)

## How it works

- **Knots in, a trajectory out.** Every source (a script segment, a live
  sample, a stroke generator) becomes knots: the curve passes position `p` at
  time `t_us`, optionally with an end velocity. The engine never sees a wire
  format, so any source feeds the same planner.
- **A composite cubic Bézier in the time-position plane.** Each knot has one
  angle and two handle lengths, and the limits bound those lengths. The curve is
  continuous in position and velocity by construction, so the motor never sees a
  step in speed.
- **Event-driven.** The pending knots are solved when they change, lazily at the
  next sample. There is no fixed control clock inside the planner, so you sample
  on whatever clock you have.
- **Time never gives, amplitude does.** A knot the limits cannot reach in time
  moves toward the knot before it. It is never dropped and never placed late. So
  a script keeps its rhythm and gives up stroke length.
- **Limits are ceilings.** The planner stays under them and does not plan to
  reach them. Every trim, clamp and refusal is recorded as an anomaly you can
  read, so you can see exactly where a pattern asked for too much.
- **Two models on top.** The stream model handles live input one sample behind,
  and the handle model renders authored scripts. Both end up as knots in the
  same timeline.

## Example

```cpp
#include <kinetic2/engine.hpp>

kinetic2::Config cfg;
cfg.limits = {10.0f, 400.0f, 50000.0f};      // vmax, amax, jmax in window units
kinetic2::Engine<> engine(cfg, 0.0f);        // at rest at the window's low end
engine.resetAt(0.0f, now_us);

kinetic2::Knot k;
k.t_us = now_us + 250000;                    // pass 0.75 in 250 ms
k.p    = 0.75f;                              // no end velocity: the solver picks the angle
engine.submit(k, now_us);

for (uint64_t t = now_us; t <= now_us + 300000; t += 1000)
    drive(engine.positionAt(t));             // sample on your own clock, non-decreasing

kinetic2::Anomaly an;
while (engine.popAnomaly(an))                // anomalies the planner recorded
    report(an.kind, an.detail);
```

The rest of the API:

- `truncateAfter()` replaces what is queued.
- `brake()` stops from the current state.
- `reframe()` restates the plan when your window moves.
- `solved()` and `peek()` expose the plan for telemetry or a renderer.
- `setConfig()` and `setLimits()` apply at the next submit or reset.
- `expect(axis, until_us)` tells the engine more knots are coming, so a stream
  passes same-direction knots near chord speed instead of slowing toward each
  one.

`include/kinetic2/engine.hpp` documents each call.

## Planner options

`kinetic2::Config` holds the limits and the options the renderer reads.
`include/kinetic2/types.hpp` documents each.

| Member | Default | What it sets |
|---|---|---|
| `limits` | 3, 30, 500 | vmax, amax, jmax: ceilings, never targets |
| `smoothness` | 0 | free knots: 0 is crisp (no overshoot between monotone knots), 1 is smooth (continuous acceleration, may overshoot), values between blend the two |
| `handle_floor` | 0.15 | the shortest handle a ceiling fit may leave, as a share of its span |
| `trim_max` | 1 | the farthest a knot moves toward its predecessor to fit the ceilings, as a share of the window |
| `react_us` | 4000 | how far ahead of now a knot arriving mid-motion re-plans from |
| `solve_budget` | 96 | the machine's bound on one planner run; not read yet |

## Units

| Quantity | Unit |
|---|---|
| position | normalized 0..1 across your stroke window |
| velocity, acceleration, jerk | window units per second, per second², per second³ |
| limits | the same; derive them as mm limits divided by the window span |
| time | microseconds, `uint64_t`, supplied by you (never read from a clock) |

Turning the window into millimeters, steps or encoder counts is your code's job,
so the same plan works for any machine.

## Deterministic

Every quantity is `float` and time is integer microseconds. The same calls give
bit-identical results on native, ESP32-P4 and WebAssembly builds, provided every
translation unit that compiles the planner uses `-ffp-contract=off` and not
`-ffast-math`. Without the flag, GCC fuses multiply-adds on targets such as the
ESP32-P4 and results drift by a few ULPs.

So a plan previewed from the same knots, submitted and sampled at the same
times under the same `Config` (`react_us` included), is the plan the firmware
runs: a script rendered ahead previews exactly. A live re-plan (a stream, a
jog or a supersede arriving while the axis moves) can differ in timing and
origin between a preview and a machine: the host chooses `react_us` per submit
from its own clock (Nucleus adapts it to its measured solve time and the
stream's lead), and its fallbacks for a late plan, an exhausted plan buffer or
a late tick run on the firmware clock. The CMake
target carries the flag for you, and `tests/test_kinetic2.cpp` pins a
fingerprint of the rendered motion. The engine is single-threaded: every call on
one `Engine` comes from one task.

## Use it

Kinetic needs C++20. Pick one:

**CMake**

```cmake
add_subdirectory(Kinetic)
target_link_libraries(app PRIVATE kinetic::kinetic2)
```

**PlatformIO**

```ini
build_flags = -std=gnu++2b -ffp-contract=off
lib_deps =
    symlink://../Kinetic
```

**Vendor it.** Copy `include/`, `LICENSE` and `NOTICE.md` into your tree and
never edit the copy.

Nucleus clones Kinetic beside it and records the commit in a pin file, so its
lint fails if the checkout drifts. Changes land in Kinetic first, then the pin
moves.

## Build and test

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Every kinematic assertion samples the produced trajectory on a 1 ms grid and
checks those samples against the limits. `bench_kinetic2` prints the solver's
cost per submit; it is not a test.

## Playground and WebAssembly

The [playground](https://openvalence.org/Kinetic/) runs the planner in your
browser. Place knots on the position trace, change the planner options, and read
the result: the curve, velocity, acceleration and jerk against the limits, the
solved knots and every anomaly. So you can see what the planner does with a
pattern before it reaches a motor. Show planner steps plays through what the
planner computed for those knots, one step at a time. Its source is `playground/`.

`wasm/` builds the same planner behind a small C interface
(`wasm/kinetic2_wasm.h`), standalone with no imports and no JS glue:

```
emcmake cmake -S . -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm
```

The output is `build-wasm/wasm/kinetic2.wasm`. Sample with non-decreasing times
between resets, because sampling retires knots the clock has passed.
`build-wasm/wasm/kinetic2-trace.wasm` is the same planner with the trace hooks
in `include/kinetic2/trace.hpp` recording what it computes, for the planner
steps. Undefined, every hook compiles to nothing.

## What Kinetic does not do

Kinetic plans motion inside a window you give it. Homing, rail limits, e-stop,
arbitration, the wire protocol and step generation belong to the machine around
it. In OpenValence that machine is Nucleus.

## Versions

`kinetic2::kVersion` (in `include/kinetic2/types.hpp`) is the planner's version
string, and the wasm module reports it from `kinetic2_version()`. `library.json`
carries the repository release, tagged `vX.Y.Z`. Kinetic 1, the first planner,
lives on the `kinetic1` branch as a test oracle and is not on `main`.

## License

Apache-2.0. See `LICENSE` and `NOTICE.md`.
