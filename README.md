# Kinetic

Kinetic is a header-only C++ library that plans jerk-limited trajectories for
one linear axis. It has no hardware dependencies and its output is
deterministic. It is the motion planner of the OpenValence Nucleus firmware
(OSSM Flagship) and can be used outside Nucleus.

The repository holds two planners.

- **Kinetic²** (`include/kinetic2/`) is the planner Nucleus ships: one knot
  timeline per axis, a lookahead solver over the pending knots, three
  junction kinds, no Ruckig. `wasm/kinetic2_wasm.h` is its C ABI and
  `playground/` runs it in a browser. The design is RFC-105 in the Valence
  RFC queue.
- **Kinetic 1** (`include/kinetic/`, with the vendored Ruckig in
  `third_party/ruckig/`) is the previous planner. It stays in this repository
  as the test oracle that grades Kinetic² (`tests/test_kinetic2_oracle.cpp`)
  and is not built into Nucleus.

The sections below describe Kinetic 1. Kinetic²'s API is documented in its
headers; `playground/README.md` and `wasm/kinetic2_wasm.h` show it in use.

## The model

- **One trajectory per command**, planned from the engine's actual
  position, velocity and acceleration at that instant. The caller samples it
  on its own clock.
- **Event-driven planning.** A plan is computed when a command arrives
  (or once, when a moving plan ends with nothing after it). Sampling is
  polynomial or profile evaluation.
- **Waveform segments** (a command with a duration) are Hermite curves in the
  sender's declared family (C1 cubic or C2 quintic) over exactly the commanded
  duration, scanned against the velocity, acceleration and jerk ceilings and
  the window before adoption.
- **Infeasible segments follow a declared policy.** `Blend` (the default)
  keeps the deadline and reduces shape and amplitude together, only as far as
  the ceilings require. `Stretch` keeps the whole stroke and overruns the
  deadline.
- **Amplitude floor.** `infeasible_amplitude_budget` bounds how much of a
  commanded stroke Blend may give up; the search never crosses it.
- **The Ruckig guard.** A shape still illegal at the floor is handed to Ruckig
  and planned time-optimally under the ceilings; it arrives after the deadline
  and records an anomaly. Bare
  points (no duration) are chased by Ruckig, replanned per point; a moving
  plan that starves is braked to rest by Ruckig's velocity interface.
- **Limits.** Limits are upper bounds; the planner does not plan to reach
  them. Every infeasible path records a motion anomaly that describes what
  the planner gave up.

## Example

```cpp
#include <kinetic/kinetic.hpp>

kinetic::Config cfg;
cfg.limits = {10.0f, 400.0f, 50000.0f};   // vmax, amax, jmax in window units
cfg.infeasible_policy = kinetic::InfeasiblePolicy::Blend;
kinetic::Engine engine(cfg, 0.0f);        // at rest at the window's low end

kinetic::Command seg;
seg.target       = 0.75f;                 // window units, 0..1
seg.duration_us  = 250000;                // reach it in 250 ms
seg.has_duration = true;                  // a waveform segment
seg.end_vel      = 0.0f;
seg.has_end_vel  = true;                  // arrive at rest
engine.commit(seg, now_us);               // plans once, from the actual p, v, a

for (uint64_t t = now_us; t <= now_us + 300000; t += 1000)
    drive(engine.positionAt(t));          // sample on your own clock

kinetic::Anomaly an;
while (engine.popAnomaly(an))             // anomalies the planner recorded
    report(an.kind, an.detail);
```

The rest of the API: `Command::anchor_us` schedules a segment on the
engine's clock (a future anchor is planned now and parked), `brake()` stops
from the current state, `snapshot()` and `planView()` expose the plan for
telemetry or for a renderer on another core, `setConfig()` / `setLimits()`
apply at the next plan. The header documents each.

## Units and frames

| Quantity | Unit |
|---|---|
| position | normalized 0..1 across the caller's stroke window |
| velocity, acceleration, jerk | window units per second, per second², per second³ |
| limits | the same; derive them as mm limits divided by the window span |
| time | microseconds, `uint64_t`, supplied by the caller (never read from a clock) |

Mapping the window to millimeters, steps or encoder counts is the caller's.
`wasm/kinetic_wasm.cpp` is a complete example of that mapping.

## Determinism

Planning math is `double`; the public API is `float`. The same sequence of calls
gives bit-identical results on every IEEE-754 target, provided every translation unit that
compiles the planner is built with `-ffp-contract=off` and without
`-ffast-math`. The CMake target carries the flag as an INTERFACE option; any
other build must set it.

GCC's default FP contraction emits fused multiply-add on targets such as the
ESP32-P4, and results then differ from native and wasm builds by a few ULPs.

The engine is single-threaded: every call on one `Engine` comes from one
task. `commit()` nests KB-scale Ruckig temporaries, so the task that calls it
needs a deep stack; measure its high-water mark before shrinking it.

## Integration

**Sibling checkout** (what Nucleus does): clone Kinetic beside your project,
point the build at it with one of the forms below, and record the Kinetic
commit sha in a pin file (Nucleus: `kinetic.pin`; its lint fails when the
checkout's HEAD is not the pin). Changes are made in Kinetic first; the pin is then updated.
Kinetic² needs only `include/`; the Ruckig library below is Kinetic 1's.

**Vendor it**: copy `include/`, `third_party/ruckig/`, `LICENSE` and
`NOTICE.md` into your tree. Never edit the copy.

**PlatformIO**: both directories are libraries.

```ini
build_flags = -std=gnu++2b -ffp-contract=off
lib_deps =
    symlink://../Kinetic
    symlink://../Kinetic/third_party/ruckig
```

**CMake**:

```cmake
add_subdirectory(Kinetic)
target_link_libraries(app PRIVATE kinetic::kinetic)
```

Requires C++20 (`library.json` builds as gnu++2b).

## Tests

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Every kinematic assertion samples the produced trajectory on a 1 ms grid
and checks those samples against the ceilings.

## WebAssembly

`wasm/` builds the engine alone behind a small C ABI (`wasm/kinetic_wasm.h`):
create with mm limits over a rail, set the window, submit a segment in mm,
step the handle's clock, read a 64-byte sample.

```
emcmake cmake -S . -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm
```

Output `build-wasm/wasm/kinetic.wasm`: standalone, zero imports, no JS glue.

```js
const { instance } = await WebAssembly.instantiate(bytes, {});
const k = instance.exports;
k._initialize();
const h = k.kinetic_create(1000, 40000, 5e6, 100);   // mm/s, mm/s², mm/s³, rail mm
k.kinetic_set_window(h, 20, 80);
k.kinetic_submit_segment(h, 50, 500, NaN, 0, 2);     // 50 mm in 500 ms, quintic
const out = k.malloc(64);
k.kinetic_step(h, 0.001, out);                        // layout: kinetic_wasm.h
```

Nucleus builds a separate module for its offline renderer: this engine wrapped
in the firmware's motion arbiter (Nucleus `tools/kinetic-wasm/`). That module
stays in Nucleus.

The same build also produces `build-wasm/wasm/kinetic2.wasm`: Kinetic² behind
`wasm/kinetic2_wasm.h`. Create a handle, configure the ceilings and planner
options, reset at a position, submit knots in window units on your own clock,
sample the state at a time, read the solved knots and the anomalies. Sampling
retires knots the clock has passed, so sample with non-decreasing times
between resets. `playground/` is a browser page over this module.

## Scope

Kinetic plans one axis inside a window it is given. These belong to the
machine around it, and in OpenValence they live in Nucleus:

- window ownership: homing, the stroke window's physical limits, the clamp
  that is the hard backstop downstream of the planner;
- arbitration: which source owns motion, ESTOP, pause, power gates;
- the wire protocol: decoding Valence segments into `Command`s, pacing,
  schedule horizons;
- step generation and the emitter.

## Versioning

`kinetic::kVersion` (in `kinetic.hpp`) is the version string, matching
`library.json` and the `vX.Y.Z` tag. The wasm module reports it from
`kinetic_version()` as `"kinetic X.Y.Z"`.

## License

Apache-2.0, see `LICENSE` and `NOTICE.md`.

Built on [Ruckig](https://github.com/pantor/ruckig) Community Version by Lars
Berscheid (MIT), vendored unmodified.
