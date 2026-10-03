# kinetic.wasm -- the engine behind the C ABI in kinetic_wasm.h. Built from the
# repo root under emcmake (README.md "WebAssembly"); output is gitignored.
#
# Standalone wasm, no JS glue, no filesystem: instantiate with
# WebAssembly.instantiate and call the exports. The stack is sized for
# commit(), which nests KB-scale Ruckig temporaries.

add_executable(kinetic_wasm kinetic_wasm.cpp)
set_target_properties(kinetic_wasm PROPERTIES OUTPUT_NAME kinetic SUFFIX ".wasm")
target_link_libraries(kinetic_wasm PRIVATE kinetic::kinetic)
target_compile_options(kinetic_wasm PRIVATE -O2 -fno-fast-math -Wall)
target_link_options(kinetic_wasm PRIVATE
    -O2
    --no-entry
    -sSTANDALONE_WASM=1
    -sFILESYSTEM=0
    -sALLOW_MEMORY_GROWTH=0
    -sINITIAL_MEMORY=4MB
    -sSTACK_SIZE=256KB
    -sMALLOC=emmalloc
    -sEXPORTED_FUNCTIONS=_malloc,_free)
