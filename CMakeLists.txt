# Kinetic -- jerk-limited trajectory planner (Kinetic², header-only).
#
# Consumers: add_subdirectory(<path to Kinetic>) and link kinetic::kinetic2.
# Top level: the doctest suites under ctest; under emcmake, wasm/ instead.
#
# -ffp-contract=off is the determinism rule (README.md "Determinism"): it is
# an INTERFACE option so every consumer TU that compiles the planner gets it.
# Never add -ffast-math to anything that links this target.

cmake_minimum_required(VERSION 3.21)
project(kinetic LANGUAGES CXX)

set(KINETIC_FP_FLAGS $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)

add_library(kinetic2 INTERFACE)
add_library(kinetic::kinetic2 ALIAS kinetic2)
target_include_directories(kinetic2 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(kinetic2 INTERFACE cxx_std_20)
target_compile_options(kinetic2 INTERFACE ${KINETIC_FP_FLAGS})

if(PROJECT_IS_TOP_LEVEL)
    if(EMSCRIPTEN)
        add_subdirectory(wasm)
    else()
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
