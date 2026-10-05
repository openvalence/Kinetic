// test_kinetic2.cpp -- Kinetic² native suite. Every kinematic assertion samples
// the rendered trajectory on a 1 ms grid: ceilings and knots are verified as
// sampled reality, never trusted from the planner.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "kinetic2/engine.hpp"

using namespace kinetic2;

namespace {

constexpr uint64_t kMs = 1000;

Knot knotAt(uint64_t t_us, float p, bool has_v = false, float v = 0.0f, Family f = Family::C2) {
    Knot k; k.t_us = t_us; k.p = p; k.has_v = has_v; k.v = v; k.family = f;
    return k;
}

// Samples every 1 ms from t0 to t1 inclusive.
std::vector<State> sweep(Engine<>& e, uint64_t t0, uint64_t t1) {
    std::vector<State> out;
    for (uint64_t t = t0; t <= t1; t += kMs) out.push_back(e.stateAt(0, t));
    return out;
}

}  // namespace

TEST_CASE("timeline orders knots and refuses a step backwards") {
    Timeline<4> tl;
    CHECK(tl.push(knotAt(100, 0.1f)));
    CHECK(tl.push(knotAt(200, 0.2f)));
    CHECK_FALSE(tl.push(knotAt(200, 0.3f)));   // not strictly after
    CHECK_FALSE(tl.push(knotAt(150, 0.3f)));
    CHECK(tl.push(knotAt(300, 0.3f)));
    CHECK(tl.push(knotAt(400, 0.4f)));
    CHECK(tl.full());
    CHECK_FALSE(tl.push(knotAt(500, 0.5f)));
    CHECK(tl.firstAfter(250) == 2);
    CHECK(tl.firstAfter(400) == 4);
    tl.popFront();
    CHECK(tl.at(0).t_us == 200);
}

TEST_CASE("a fresh engine holds its seed") {
    Engine<> e(Config{}, 0.3f);
    for (uint64_t t = 0; t < 50 * kMs; t += kMs) {
        const State s = e.stateAt(0, t);
        CHECK(s.p == doctest::Approx(0.3f));
        CHECK(s.v == 0.0f);
    }
    CHECK_FALSE(e.isBusy(10 * kMs));
}

TEST_CASE("knots are hit at their times and the curve is continuous") {
    Engine<> e(Config{}, 0.2f);
    const uint64_t now = 1000 * kMs;
    REQUIRE(e.submit(knotAt(now + 200 * kMs, 0.8f), now));
    REQUIRE(e.submit(knotAt(now + 400 * kMs, 0.3f), now));
    REQUIRE(e.submit(knotAt(now + 600 * kMs, 0.6f, true, 0.0f, Family::C1), now));
    CHECK(e.pending() == 3);
    CHECK(e.isBusy(now));

    const auto s = sweep(e, now, now + 700 * kMs);
    CHECK(s[200].p == doctest::Approx(0.8f).epsilon(1e-4));
    CHECK(s[400].p == doctest::Approx(0.3f).epsilon(1e-4));
    CHECK(s[600].p == doctest::Approx(0.6f).epsilon(1e-4));
    CHECK(s[600].v == doctest::Approx(0.0f).epsilon(1e-3));
    // Past the last knot: a hold at its position.
    CHECK(s[700].p == doctest::Approx(0.6f));
    CHECK(s[700].v == 0.0f);
    CHECK_FALSE(e.isBusy(now + 700 * kMs));
    // Continuity: no sample moves farther than its own velocity could carry
    // it in 1 ms plus a small tolerance, and velocity never jumps.
    for (size_t i = 1; i < s.size(); ++i) {
        const float dp = std::fabs(s[i].p - s[i - 1].p);
        const float vmax_local = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v));
        CHECK(dp <= vmax_local * 1e-3f + 1e-4f);
        CHECK(std::fabs(s[i].v - s[i - 1].v) < 0.5f);
    }
}

TEST_CASE("a knot in the past or behind the newest is refused and counted") {
    Engine<> e(Config{}, 0.5f);
    const uint64_t now = 10 * kMs;
    CHECK_FALSE(e.submit(knotAt(now, 0.6f), now));
    REQUIRE(e.submit(knotAt(now + 100 * kMs, 0.6f), now));
    CHECK_FALSE(e.submit(knotAt(now + 50 * kMs, 0.7f), now));
    CHECK_FALSE(e.submit(knotAt(now + 200 * kMs, NAN), now));
    Anomaly an;
    int refused = 0;
    while (e.popAnomaly(an)) if (an.kind == uint8_t(AnomalyKind::KnotRefused)) ++refused;
    CHECK(refused == 3);
}

TEST_CASE("brake drops the future and comes to rest") {
    Config cfg; cfg.limits.amax = 20.0f;
    Engine<> e(cfg, 0.1f);
    const uint64_t now = 0;
    REQUIRE(e.submit(knotAt(500 * kMs, 0.9f), now));
    REQUIRE(e.submit(knotAt(1000 * kMs, 0.1f), now));
    const State mid = e.stateAt(0, 250 * kMs);
    CHECK(mid.v > 0.0f);
    e.brake(250 * kMs);
    CHECK(e.pending() == 1);
    const auto s = sweep(e, 250 * kMs, 1000 * kMs);
    CHECK(s.back().v == 0.0f);
    CHECK(s.back().p > mid.p);        // it stopped ahead of where it was
    CHECK(s.back().p < 0.9f);         // and short of the dropped knot
    Anomaly an; bool settled = false;
    while (e.popAnomaly(an)) settled |= an.kind == uint8_t(AnomalyKind::SettleEngaged);
    CHECK(settled);
}

TEST_CASE("reset forgets everything and holds the new position") {
    Engine<> e(Config{}, 0.5f);
    REQUIRE(e.submit(knotAt(300 * kMs, 0.9f), 0));
    e.resetAt(0.2f, 100 * kMs);
    CHECK(e.pending() == 0);
    CHECK(e.stateAt(0, 150 * kMs).p == doctest::Approx(0.2f));
    CHECK_FALSE(e.isBusy(150 * kMs));
}

TEST_CASE("same calls in, same bits out") {
    auto run = [] {
        Engine<> e(Config{}, 0.25f);
        e.submit(knotAt(150 * kMs, 0.75f), 0);
        e.submit(knotAt(333 * kMs, 0.4f, true, -0.3f), 0);
        e.submit(knotAt(777 * kMs, 0.6f), 0);
        std::vector<State> s;
        for (uint64_t t = 0; t <= 800 * kMs; t += 7 * kMs) s.push_back(e.stateAt(0, t));
        return s;
    };
    const auto a = run(), b = run();
    REQUIRE(a.size() == b.size());
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(State)) == 0);
}

TEST_CASE("two axes are independent") {
    Engine<2> e(Config{}, 0.5f);
    REQUIRE(e.submit(0, knotAt(200 * kMs, 0.9f), 0));
    REQUIRE(e.submit(1, knotAt(300 * kMs, 0.1f), 0));
    CHECK(e.stateAt(0, 200 * kMs).p == doctest::Approx(0.9f).epsilon(1e-4));
    CHECK(e.stateAt(1, 200 * kMs).p != doctest::Approx(0.9f));
    CHECK(e.stateAt(1, 300 * kMs).p == doctest::Approx(0.1f).epsilon(1e-4));
    CHECK(e.stateAt(0, 300 * kMs).p == doctest::Approx(0.9f));
}
