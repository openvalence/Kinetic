# Vendored: Ruckig Community Version

Upstream:  https://github.com/pantor/ruckig
Tag:       v0.19.4
Commit:    a8db97a4e9c55e5160a3855f739fa3b270df8e4c
License:   MIT (see LICENSE, copied verbatim from upstream)

## What is included

- `include/ruckig/*.hpp` — all upstream headers EXCEPT `calculator_cloud.hpp`
  (Pro/cloud waypoint calculator; its include site in `calculator.hpp` is
  fenced by `#ifdef WITH_CLOUD_CLIENT`, which is never defined here).
- `src/ruckig/*.cpp` — the community polynomial-root solvers
  (`brake`, `position_*_step1/2`, `velocity_*_step1/2`). `cloud_client.cpp`
  is deliberately NOT vendored.

## Local modifications

NONE. The vendored files are byte-identical to upstream. Keep it that way:
any behavior Kinetic needs lives in `include/kinetic/kinetic.hpp`, the
wrapper. If a genuine upstream patch is ever unavoidable, record the diff here.

## Why vendored

Ruckig is not in the PlatformIO registry, and pinning by git URL would make
every clean build a network build. The subset is small, MIT, and frozen.

## Update procedure

1. Clone upstream, checkout the new tag.
2. Re-copy `include/ruckig/` (minus `calculator_cloud.hpp`) and the community
   `src/ruckig/*.cpp` set (minus `cloud_client.cpp`).
3. Update Tag/Commit above and `library.json` version.
4. The native suite (`ctest`) and the wasm build stay green before commit.

## Notes

- Ruckig computes in `double` internally. Planning is event-driven (per
  command, never per sample), so a target without hardware double pays that
  cost at plan time only.
- Exceptions: all `throw` sites are behind `template<bool throw_error>`
  paths that Kinetic never instantiates with `true`; the non-throwing
  `calculate()`/`update()` overloads return `Result` error codes instead.
