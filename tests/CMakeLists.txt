# The native suite: the engine cases plus a smoke test of the wasm C ABI,
# compiled natively. Green before every tag (CONTRIBUTING.md).
add_executable(test_kinetic test_kinetic.cpp test_wasm_abi.cpp ../wasm/kinetic_wasm.cpp)
target_link_libraries(test_kinetic PRIVATE kinetic::kinetic)
target_include_directories(test_kinetic PRIVATE third_party ../wasm)
target_compile_options(test_kinetic PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O2 -Wall>)
add_test(NAME test_kinetic COMMAND test_kinetic)
