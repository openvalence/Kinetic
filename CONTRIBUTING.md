# Contributing

Two rules a change must keep. A pull request that breaks one is not merged.

1. **`-ffp-contract=off`, and never `-ffast-math`.** Same calls in, same bits
   out, on every target. The CMake targets carry the flag; every other build
   that compiles the planner (a PlatformIO env, a firmware component, the wasm
   build) sets it too.
2. **The native suites are green before a tag.**

   ```
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   ctest --test-dir build --output-on-failure
   ```

   The wasm build (README.md "WebAssembly") compiles too.

## Releasing

`version` in `library.json` moves in the commit that is tagged `vX.Y.Z`.

## Style

American English. A comment states a constraint, an invariant the code cannot
show, or a pointer; history goes in the commit message.
