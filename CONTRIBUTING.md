# Contributing

Three rules a change must keep. A pull request that breaks one is not merged.

1. **Wrap Ruckig, never patch it.** `third_party/ruckig/` is byte-identical to
   upstream v0.19.4 (`third_party/ruckig/VENDORED.md`). Behavior Kinetic needs
   lives in `include/kinetic/kinetic.hpp`. An upstream bump follows the update
   procedure in `VENDORED.md`, as its own commit.
2. **`-ffp-contract=off`, and never `-ffast-math`.** Same calls in, same bits
   out, on every target. The CMake targets carry the flag; every other build
   that compiles the planner (a PlatformIO env, a firmware component, the wasm
   build) sets it too.
3. **The native suite is green before a tag.**

   ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ctest --test-dir build --output-on-failure
   ```

   The wasm build (README.md "WebAssembly") compiles too.

## Releasing

`kinetic::kVersion` in `include/kinetic/kinetic.hpp` and `version` in
`library.json` move together, in the commit that is tagged `vX.Y.Z`.

## Style

American English. A comment states a constraint, an invariant the code cannot
show, or a pointer; history goes in the commit message.
