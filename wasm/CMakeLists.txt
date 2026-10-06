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

# kinetic2.wasm -- Kinetic² behind kinetic2_wasm.h. No Ruckig, a smaller stack.
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
