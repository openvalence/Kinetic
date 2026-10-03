# Kinetic

A jerk-limited trajectory engine for one linear axis. Header-only C++, no
hardware dependencies, deterministic. It is the motion planner of the
OpenValence Nucleus firmware (OSSM Flagship), published on its own so the
motion work can be reused in other projects.

## The model

- **One trajectory per command**, planned from the engine's ACTUAL
  position, velocity and acceleration at that instant. The caller samples it
  on its own clock.
- **Event-driven, never clocked.** A plan is computed when a command arrives
  (or once, when a moving plan ends with nothing after it). Sampling is
  polynomial or profile evaluation, nothing more.
- **Waveform segments** (a command with a duration) are Hermite curves in the
  sender's declared family (C1 cubic or C2 quintic) over exactly the commanded
  duration, scanned against the velocity, acceleration and jerk ceilings and
  the window before adoption.
- **Infeasible segments follow a declared policy.** `Blend` (the default)
  keeps the deadline and spends shape and amplitude together, only as far as
  the ceilings demand. `Stretch` keeps the whole stroke and overruns the
  deadline.
- **Amplitude is a floor.** `infeasible_amplitude_budget` bounds how much of a
  commanded stroke Blend may give up; the search never crosses it.
- **The Ruckig guard.** A shape still illegal at the floor is handed to Ruckig
  and planned time-optimally under the ceilings, late, and reported. Bare
  points (no duration) are chased by Ruckig, replanned per point; a moving
  plan that starves is braked to rest by Ruckig's velocity interface.
- **Limits are ceilings, never targets.** Every infeasible path records an
  `Anomaly` that names the axis it spent.

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
while (engine.popAnomaly(an))             // what the planner had to spend
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
| position | normalized 0..1 across the caller's travel window |
| velocity, acceleration, jerk | window units per second, per second², per second³ |
| limits | the same; derive them as mm limits divided by the window span |
| time | microseconds, `uint64_t`, supplied by the caller (never read from a clock) |

Mapping the window to millimeters, steps or encoder counts is the caller's.
`wasm/kinetic_wasm.cpp` is a complete example of that mapping.

## Determinism

Planning math is `double`; the public API is `float`. Same calls in give the
same bits out on every IEEE-754 target, provided every translation unit that
compiles the planner is built with `-ffp-contract=off` and without
`-ffast-math`. The CMake target carries the flag as an INTERFACE option; any
other build must set it.

The ESP32-P4 is why this is a rule: GCC's default contraction emitted fused
multiply-add instructions in the firmware's float paths there, and the machine
then disagreed with the native and wasm builds by a few ULPs.

The engine is single-threaded: every call on one `Engine` comes from one
task. `commit()` nests KB-scale Ruckig temporaries, so the task that calls it
needs a deep stack; measure its high-water mark before shrinking it.

## Using it

**Vendor it** (what Nucleus does): copy `include/`, `third_party/ruckig/`,
`LICENSE` and `NOTICE.md` into your tree and record the Kinetic commit sha in
a pin file beside them (Nucleus: `kinetic.pin`, checked by its lint). Never
edit the copy; changes land here first.

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

Every kinematic assertion samples the produced trajectory on a 1 ms grid, so
the ceilings are verified as sampled reality rather than trusted from the
planner.

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

Nucleus builds a different module, the whole machine (its motion arbiter
around this engine) for its offline renderer; that one stays in Nucleus.

## Not included

Kinetic plans one axis inside a window it is given. These belong to the
machine around it, and in OpenValence they live in Nucleus:

- window ownership: homing, the stroke window's physical limits, the clamp
  that is the hard backstop downstream of the planner;
- arbitration: which source owns motion, e-stop, pause, power gates;
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
