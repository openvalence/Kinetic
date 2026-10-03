// test_wasm_abi -- the engine-only C ABI (wasm/kinetic_wasm.h), compiled
// natively: frame conversion, refusals and one segment rendered to rest
// Constraints:
// - The wasm build compiles the same kinetic_wasm.cpp; a case here is the
//   native proof of the export behavior, not of the Emscripten toolchain.

#include <doctest/doctest.h>

#include "kinetic_wasm.h"

#include <cmath>
#include <cstring>

TEST_CASE("wasm ABI: one segment renders to its target in mm and comes to rest") {
    kinetic_handle* h = kinetic_create(1000.0f, 40000.0f, 5e6f, 100.0f);
    REQUIRE(h != nullptr);
    // Created at rest at 0 mm, so the seed sits outside this window: the
    // entry plan starts from the honest seed, never from the window edge.
    REQUIRE(kinetic_set_window(h, 20.0f, 80.0f) == 1);

    REQUIRE(kinetic_submit_segment(h, 50.0f, 500, NAN, 0.0, 2) == 1);
    kinetic_sample s{};
    bool moved = false;
    for (int i = 0; i < 800; ++i) {
        kinetic_step(h, 0.001, &s);
        moved = moved || (s.flags & KINETIC_FLAG_BUSY) != 0;
        CHECK(s.position_mm >= 0.0f);
        CHECK(s.position_mm <= 80.0f);
    }
    CHECK(moved);
    CHECK(s.t_us == doctest::Approx(800000.0));
    CHECK(s.position_mm == doctest::Approx(50.0f).epsilon(1e-4));
    CHECK(std::fabs(s.velocity_mm_s) < 1e-3f);
    CHECK((s.flags & KINETIC_FLAG_BUSY) == 0);
    CHECK(s.plans >= 1);
    kinetic_destroy(h);
}

TEST_CASE("wasm ABI: refusals") {
    CHECK(kinetic_create(0.0f, 1.0f, 1.0f, 100.0f) == nullptr);
    CHECK(kinetic_create(NAN, 1.0f, 1.0f, 100.0f) == nullptr);

    kinetic_handle* h = kinetic_create(1000.0f, 40000.0f, 5e6f, 100.0f);
    REQUIRE(h != nullptr);
    CHECK(kinetic_set_window(h, 60.0f, 40.0f) == 0);
    CHECK(kinetic_set_window(h, 0.0f, 120.0f) == 0);
    CHECK(kinetic_submit_segment(h, 50.0f, 0, NAN, 0.0, 0) == -1);

    kinetic_sample s{};
    CHECK(kinetic_submit_segment(h, NAN, 100, NAN, 0.0, 0) == 0);
    kinetic_step(h, 0.001, &s);
    CHECK((s.flags & KINETIC_FLAG_REFUSED) != 0);
    kinetic_step(h, 0.001, &s);
    CHECK((s.flags & KINETIC_FLAG_REFUSED) == 0);
    CHECK(kinetic_now_us(h) == doctest::Approx(2000.0));
    kinetic_destroy(h);

    CHECK(std::strncmp(kinetic_version(), "kinetic ", 8) == 0);
}
