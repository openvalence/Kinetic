# Kinetic

Kinetic is a header-only C++ library that plans jerk-limited trajectories. It
has no hardware dependencies and its output is deterministic. It is the
motion planner of the OpenValence Nucleus firmware (OSSM Flagship) and can be
used outside Nucleus.

The planner is Kinetic² (`include/kinetic2/`): one knot timeline per axis, a
lookahead solver over the pending knots, three junction kinds. Its API is
documented in its headers; `wasm/kinetic2_wasm.h` is its C ABI and
`playground/` runs it in a browser. The design is RFC-105 in the Valence RFC
queue.

Kinetic 1 and the K2-vs-K1 oracle comparison live on branch `kinetic1` (tag `kinetic1-final`), off main since 2026-10-08 (operator ruling, kin-xfq).

## The model

- **Knots in, a sampled trajectory out.** Every source (a segment, a sample,
  a stroke generator) becomes knots: the curve passes `p` at `t_us`, with an
  optional authored end velocity. The engine never sees a wire format.
- **Event-driven, never clocked.** The pending window is solved when it
  changes, lazily at the next sample; a piece is built when the sampler first
  needs it, from the state the previous piece ends in, so the curve is
  continuous in position and velocity by construction.
- **Time never gives, amplitude does.** A knot the ceilings cannot reach in
  time is trimmed toward its predecessor and records an anomaly; it is never
  dropped or placed late.
- **Limits are ceilings.** The planner never exceeds them and does not plan
  to reach them. Every trim, clamp and refusal records an anomaly.

## Example

```cpp
#include <kinetic2/engine.hpp>

kinetic2::Config cfg;
cfg.limits = {10.0f, 400.0f, 50000.0f};      // vmax, amax, jmax in window units
kinetic2::Engine<> engine(cfg, 0.0f);        // at rest at the window's low end
engine.resetAt(0.0f, now_us);

kinetic2::Knot k;
k.t_us   = now_us + 250000;                  // pass 0.75 in 250 ms
k.p      = 0.75f;
k.family = kinetic2::Family::C2;
engine.submit(k, now_us);

for (uint64_t t = now_us; t <= now_us + 300000; t += 1000)
    drive(engine.positionAt(t));             // sample on your own clock, non-decreasing

kinetic2::Anomaly an;
while (engine.popAnomaly(an))                // anomalies the planner recorded
    report(an.kind, an.detail);
```

The rest of the API: `truncateAfter()` replaces what is queued, `brake()`
stops from the current state, `reseedAt()` restates the state when the
caller's frame moves, `solved()` and `peek()` expose the plan for telemetry or
a renderer, `setConfig()` / `setLimits()` apply at the next submit or reset.
`engine.hpp` documents each.

## Units and frames

| Quantity | Unit |
|---|---|
| position | normalized 0..1 across the caller's stroke window |
| velocity, acceleration, jerk | window units per second, per second², per second³ |
| limits | the same; derive them as mm limits divided by the window span |
| time | microseconds, `uint64_t`, supplied by the caller (never read from a clock) |

Mapping the window to millimeters, steps or encoder counts is the caller's.

## Determinism

Every quantity is `float`; time is `uint64_t` microseconds. The same sequence
of calls gives bit-identical results on every IEEE-754 target, provided every
translation unit that compiles the planner is built with `-ffp-contract=off`
and without `-ffast-math`. The CMake target carries the flag as an INTERFACE
option; any other build must set it. `tests/test_kinetic2.cpp` pins a
fingerprint of the rendered motion.

GCC's default FP contraction emits fused multiply-add on targets such as the
ESP32-P4, and results then differ from native and wasm builds by a few ULPs.

The engine is single-threaded: every call on one `Engine` comes from one task.

## Integration

**Sibling checkout** (what Nucleus does): clone Kinetic beside your project,
point the build at it with one of the forms below, and record the Kinetic
commit sha in a pin file (Nucleus: `kinetic.pin`; its lint fails when the
checkout's HEAD is not the pin). Changes are made in Kinetic first; the pin is then updated.

**Vendor it**: copy `include/`, `LICENSE` and `NOTICE.md` into your tree.
Never edit the copy.

**PlatformIO**:

```ini
build_flags = -std=gnu++2b -ffp-contract=off
lib_deps =
    symlink://../Kinetic
```

**CMake**:

```cmake
add_subdirectory(Kinetic)
target_link_libraries(app PRIVATE kinetic::kinetic2)
```

Requires C++20 (`library.json` builds as gnu++2b).

## Tests

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Every kinematic assertion samples the produced trajectory on a 1 ms grid
and checks those samples against the ceilings. `bench_kinetic2` prints the
solver's cost per submit and is not a test.

## WebAssembly

`wasm/` builds Kinetic² behind a small C ABI (`wasm/kinetic2_wasm.h`).

```
emcmake cmake -S . -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm
```

Output `build-wasm/wasm/kinetic2.wasm`: standalone, zero imports, no JS glue.
Create a handle, configure the ceilings and planner options, reset at a
position, submit knots in window units on your own clock, sample the state at
a time, read the solved knots and the anomalies. Sampling retires knots the
clock has passed, so sample with non-decreasing times between resets.
`playground/` is a browser page over this module.

Nucleus builds a separate module for its offline renderer: this engine wrapped
in the firmware's motion arbiter (Nucleus `tools/kinetic-wasm/`). That module
stays in Nucleus.

## Scope

Kinetic plans motion inside a window it is given. These belong to the
machine around it, and in OpenValence they live in Nucleus:

- window ownership: homing, the stroke window's physical limits, the clamp
  that is the hard backstop downstream of the planner;
- arbitration: which source owns motion, ESTOP, pause, power gates;
- the wire protocol: decoding Valence segments into knots, pacing,
  schedule horizons;
- step generation and the emitter.

## Versioning

`kinetic2::kVersion` (in `include/kinetic2/types.hpp`) is the planner's
version string; the wasm module reports it from `kinetic2_version()` as
`"kinetic2 X.Y.Z"`. `library.json` carries the repository release, tagged
`vX.Y.Z`.

## License

Apache-2.0, see `LICENSE` and `NOTICE.md`.
