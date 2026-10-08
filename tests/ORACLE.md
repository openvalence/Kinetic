# The Kinetic 1 oracle against a newer Kinetic²

Branch `kinetic1` (tag `kinetic1-final`) holds Kinetic 1 and
`tests/test_kinetic2_oracle.cpp`, which grades Kinetic²'s brake and park
against Kinetic 1's time-optimal profile. Main no longer carries Kinetic 1.
To grade main's Kinetic², check this branch out beside main and point the
oracle at main's headers:

```
git -C ../Kinetic worktree add ../Kinetic-k1 kinetic1
cd ../Kinetic-k1
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DKINETIC2_INCLUDE=../Kinetic/include
cmake --build build --target test_kinetic2_oracle test_kinetic
./build/tests/test_kinetic2_oracle
./build/tests/test_kinetic
```

`KINETIC2_INCLUDE` defaults to this branch's own `include/`. It changes only
`test_kinetic2_oracle`; build just the two targets above, since this branch's
`test_kinetic2` cases need not compile against a newer Kinetic². WinLibs
MinGW-w64 first on PATH on Windows. In PowerShell, quote the define
(`"-DKINETIC2_INCLUDE=../Kinetic/include"`): unquoted, PowerShell splits it
and the cache variable comes out empty, so the oracle silently grades this
branch's own Kinetic². `ninja -C build -t deps
tests/CMakeFiles/test_kinetic2_oracle.dir/test_kinetic2_oracle.cpp.obj` shows
which `kinetic2/engine.hpp` was compiled.
