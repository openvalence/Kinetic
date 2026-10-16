# kinetic2.wasm -- Kinetic² behind the C ABI in kinetic2_wasm.h. Built from the
# repo root under emcmake (README.md "Playground and WebAssembly"); output is gitignored.
#
# Standalone wasm, no JS glue, no filesystem: instantiate with
# WebAssembly.instantiate and call the exports.

add_executable(kinetic2_wasm kinetic2_wasm.cpp)
set_target_properties(kinetic2_wasm PROPERTIES OUTPUT_NAME kinetic2 SUFFIX ".wasm")
target_link_libraries(kinetic2_wasm PRIVATE kinetic::kinetic2)
target_compile_options(kinetic2_wasm PRIVATE -O2 -fno-fast-math -Wall)
target_link_options(kinetic2_wasm PRIVATE
    -O2
    --no-entry
    -sSTANDALONE_WASM=1
    -sFILESYSTEM=0
    -sALLOW_MEMORY_GROWTH=0
    -sINITIAL_MEMORY=2MB
    -sSTACK_SIZE=128KB
    -sMALLOC=emmalloc
    -sEXPORTED_FUNCTIONS=_malloc,_free)

# kinetic2-trace.wasm: the same ABI with the trace hooks recording (the
# playground's planner steps, loaded only when they are shown). Its 12 MB
# record buffer is zeroed memory, not file size.
add_executable(kinetic2_trace_wasm kinetic2_trace.cpp)
set_target_properties(kinetic2_trace_wasm PROPERTIES OUTPUT_NAME kinetic2-trace SUFFIX ".wasm")
target_link_libraries(kinetic2_trace_wasm PRIVATE kinetic::kinetic2)
target_compile_options(kinetic2_trace_wasm PRIVATE -O2 -fno-fast-math -Wall)
target_link_options(kinetic2_trace_wasm PRIVATE
    -O2
    --no-entry
    -sSTANDALONE_WASM=1
    -sFILESYSTEM=0
    -sALLOW_MEMORY_GROWTH=0
    -sINITIAL_MEMORY=32MB
    -sSTACK_SIZE=128KB
    -sMALLOC=emmalloc
    -sEXPORTED_FUNCTIONS=_malloc,_free)
