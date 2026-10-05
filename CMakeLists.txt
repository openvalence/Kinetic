# Kinetic -- jerk-limited trajectory engine (header-only) over vendored Ruckig.
#
# Consumers: add_subdirectory(<path to Kinetic>) and link kinetic::kinetic.
# Top level: the doctest suite under ctest; under emcmake, wasm/ instead.
#
# -ffp-contract=off is the determinism rule (README.md "Determinism"): it is
# an INTERFACE option so every consumer TU that compiles the planner gets it.
# Never add -ffast-math to anything that links this target.

cmake_minimum_required(VERSION 3.21)
project(kinetic LANGUAGES CXX)

set(KINETIC_FP_FLAGS $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off>)

file(GLOB KINETIC_RUCKIG_SRC CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ruckig/src/ruckig/*.cpp")
add_library(kinetic_ruckig STATIC ${KINETIC_RUCKIG_SRC})
target_include_directories(kinetic_ruckig PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ruckig/include")
target_compile_features(kinetic_ruckig PUBLIC cxx_std_20)
target_compile_options(kinetic_ruckig PRIVATE ${KINETIC_FP_FLAGS})

add_library(kinetic INTERFACE)
add_library(kinetic::kinetic ALIAS kinetic)
target_include_directories(kinetic INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(kinetic INTERFACE kinetic_ruckig)
target_compile_options(kinetic INTERFACE ${KINETIC_FP_FLAGS})

# Kinetic² (include/kinetic2): the rewrite. Header-only, float-first, no Ruckig.
# Kinetic 1 above stays as the native test oracle (Nucleus val-7p2).
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
