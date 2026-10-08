# The native suite: the engine cases plus a smoke test of the wasm C ABI,
# compiled natively. Green before every tag (CONTRIBUTING.md).
add_executable(test_kinetic test_kinetic.cpp test_wasm_abi.cpp ../wasm/kinetic_wasm.cpp)
target_link_libraries(test_kinetic PRIVATE kinetic::kinetic)
target_include_directories(test_kinetic PRIVATE third_party ../wasm)
target_compile_options(test_kinetic PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall>)
add_test(NAME test_kinetic COMMAND test_kinetic)

# Kinetic²: its own binary, no Ruckig in the link.
add_executable(test_kinetic2 test_kinetic2.cpp)
target_link_libraries(test_kinetic2 PRIVATE kinetic::kinetic2)
target_include_directories(test_kinetic2 PRIVATE third_party)
target_compile_options(test_kinetic2 PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall -Wextra>)
add_test(NAME test_kinetic2 COMMAND test_kinetic2)

# Kinetic² against its spec, the handle model. handles_fixture.hpp is
# generated: `node tests/gen_handles_fixture.mjs` from the Kinetic root.
add_executable(test_kinetic2_handles test_kinetic2_handles.cpp)
target_link_libraries(test_kinetic2_handles PRIVATE kinetic::kinetic2)
target_include_directories(test_kinetic2_handles PRIVATE third_party)
target_compile_options(test_kinetic2_handles PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall -Wextra>)
add_test(NAME test_kinetic2_handles COMMAND test_kinetic2_handles)

# Kinetic 1 as the oracle for Kinetic²: the one binary that links both.
add_executable(test_kinetic2_oracle test_kinetic2_oracle.cpp)
target_link_libraries(test_kinetic2_oracle PRIVATE kinetic::kinetic kinetic::kinetic2)
target_include_directories(test_kinetic2_oracle PRIVATE third_party)
# KINETIC2_INCLUDE: the Kinetic² the oracle grades. Point it at a newer main
# checkout's include/ (tests/ORACLE.md); it is searched before this branch's own.
set(KINETIC2_INCLUDE "${PROJECT_SOURCE_DIR}/include" CACHE PATH "include/ holding the Kinetic² the oracle grades")
target_include_directories(test_kinetic2_oracle BEFORE PRIVATE "${KINETIC2_INCLUDE}")
target_compile_options(test_kinetic2_oracle PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall>)
add_test(NAME test_kinetic2_oracle COMMAND test_kinetic2_oracle)

# The solver's cost per submit (kin-ys0): counts under KINETIC2_STATS, never a
# test (it prints a table and always exits 0).
add_executable(bench_kinetic2 bench_kinetic2.cpp)
target_link_libraries(bench_kinetic2 PRIVATE kinetic::kinetic2)
target_compile_definitions(bench_kinetic2 PRIVATE KINETIC2_STATS)
target_compile_options(bench_kinetic2 PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall -Wextra>)
