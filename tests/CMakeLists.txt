# The native suites. Green before every tag (CONTRIBUTING.md).
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

# The solver's cost per submit (kin-ys0): counts under KINETIC2_STATS, never a
# test (it prints a table and always exits 0).
add_executable(bench_kinetic2 bench_kinetic2.cpp)
target_link_libraries(bench_kinetic2 PRIVATE kinetic::kinetic2)
target_compile_definitions(bench_kinetic2 PRIVATE KINETIC2_STATS)
target_compile_options(bench_kinetic2 PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall -Wextra>)
