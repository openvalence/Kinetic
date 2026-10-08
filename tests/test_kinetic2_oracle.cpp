// test_kinetic2_oracle.cpp -- Kinetic 1 as the oracle for Kinetic² (Nucleus
// val-7p2): the only place the two engines meet. Kinetic 1 (with its vendored
// Ruckig) is time-optimal by construction; these cases measure how far the
// Kinetic² brake and park sit from that, and pin the gap so it cannot grow
// unnoticed. Never built into firmware.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <cmath>
#include <cstdio>

#include "kinetic/kinetic.hpp"
#include "kinetic2/engine.hpp"

namespace {
constexpr uint64_t kMs = 1000;

// Kinetic 1 with every softener off: the oracle must be the time-optimal
// profile under the ceilings, nothing gentler.
kinetic::Config k1Config(float v, float a, float j) {
    kinetic::Config c; c.limits = {v, a, j};
    c.chase_jerk_scale = false; c.recovery_vmax = 0.0f;
    return c;
}
}  // namespace

TEST_CASE("brake: Kinetic² stops in the same time Ruckig does, from the same state") {
    const float V = 5.0f, A = 40.0f, J = 800.0f;
    kinetic::Engine k1(k1Config(V, A, J), 0.1f);
    kinetic::Command seg; seg.target = 0.9f; seg.duration_us = 600000; seg.has_duration = true; seg.end_vel = 0.0f; seg.has_end_vel = true;
    REQUIRE(k1.commit(seg, 0));
    // Sample to a moving instant, brake there, and read Ruckig's stop duration.
    for (uint64_t t = 0; t <= 200 * kMs; t += kMs) (void)k1.positionAt(t);
    double p, v, a;
    k1.rawSampleAt(200 * kMs, p, v, a);
    REQUIRE(std::fabs(v) > 0.5);
    REQUIRE(k1.brake(200 * kMs));
    const float t_ruckig = k1.snapshot(200 * kMs).duration_s;
    REQUIRE(t_ruckig > 0.0f);

    const kinetic2::Limits L{V, A, J};
    const kinetic2::Profile ours = kinetic2::Profile::brake(kinetic2::State{float(p), float(v), float(a)}, 200 * kMs, L);
    const float t_ours = ours.duration();
    MESSAGE("brake from v=" << v << " a=" << a << ": ruckig " << t_ruckig << " s, kinetic2 " << t_ours << " s, ratio " << t_ours / t_ruckig);
    // Both are the bang-bang stop under the same ceilings: within 5 %.
    CHECK(t_ours <= t_ruckig * 1.05f);
    CHECK(t_ours >= t_ruckig * 0.95f);
}

// Kinetic²'s park renders a rest-to-rest piece (kin-y6e): it is whole when
// the knot is reached untrimmed at its time with no anomaly, every 1 ms sample
// inside the ceilings. The oracle compares sampled reality, not the renderer's
// internals: the least whole time is found by rendering.
static bool parkWhole(float V, float A, float J, float d, uint64_t t_us) {
    kinetic2::Config cfg; cfg.limits = {V, A, J};
    kinetic2::Engine<> k2(cfg, 0.05f);
    kinetic2::Knot k; k.t_us = t_us; k.p = 0.05f + d; k.has_v = true; k.v = 0.0f; k.family = kinetic2::Family::C2;
    if (!k2.submit(k, 0)) return false;
    float vpk = 0, apk = 0, jpk = 0, a_prev = 0, p_end = 0;
    // From rest (a = 0 before the first sample) through the hold after the knot.
    for (uint64_t t = 0; t <= k.t_us + 50 * kMs; t += kMs) {
        const auto s = k2.stateAt(0, t);
        vpk = std::fmax(vpk, std::fabs(s.v)); apk = std::fmax(apk, std::fabs(s.a));
        jpk = std::fmax(jpk, std::fabs(s.a - a_prev) / 1e-3f);
        a_prev = s.a;
        if (t == k.t_us) p_end = s.p;
    }
    kinetic2::Anomaly an; int spent = 0;
    while (k2.popAnomaly(an)) ++spent;
    return spent == 0 && vpk <= V * 1.001f && apk <= A * 1.001f && jpk <= J * 1.001f && std::fabs(p_end - k.p) <= 1e-4f;
}

TEST_CASE("park: the least time a rest-to-rest stroke renders whole against Ruckig's optimum") {
    const float V = 3.0f, A = 30.0f, J = 500.0f;
    const float strokes[] = {0.05f, 0.2f, 0.5f, 0.9f};
    for (const float d : strokes) {
        kinetic::Engine k1(k1Config(V, A, J), 0.05f);
        kinetic::Command pt; pt.target = 0.05f + d; pt.has_duration = false; pt.has_end_vel = true; pt.end_vel = 0.0f;
        REQUIRE(k1.commit(pt, 0));
        const float t_ruckig = k1.snapshot(0).duration_s;
        REQUIRE(t_ruckig > 0.0f);
        // Kinetic²'s park: from half the third-length cubic's analytic
        // minimum, up in 1 percent steps to the first whole time.
        const float t_cubic = std::fmax(std::fmax(1.5f * d / V, std::sqrt(6.0f * d / A)), std::cbrt(12.0f * d / J));
        float t_k2 = 0.5f * t_cubic;
        while (!parkWhole(V, A, J, d, uint64_t(t_k2 * 1e3f) * kMs) && t_k2 < 4.0f * t_cubic) t_k2 *= 1.01f;
        REQUIRE(t_k2 < 4.0f * t_cubic);
        const float ratio = t_k2 / t_ruckig;
        MESSAGE("park d=" << d << ": ruckig " << t_ruckig << " s, cubic " << t_cubic << " s, whole at " << t_k2 << " s, ratio " << ratio);
        // The pinned gap. Nothing legal beats the time-optimal profile; if the
        // gap grows past the pin, a park profile earns its place (RFC-105).
        CHECK(ratio >= 0.99f);
        CHECK(ratio <= 1.15f);   // measured 1.025 to 1.084 (2026-10-07)
    }
}
