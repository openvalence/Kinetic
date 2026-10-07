// test_kinetic2.cpp -- Kinetic² native suite. Every kinematic assertion samples
// the rendered trajectory on a 1 ms grid: ceilings and knots are verified as
// sampled reality, never trusted from the planner.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "kinetic2/engine.hpp"

#ifndef KINETIC2_FINGERPRINT
#define KINETIC2_FINGERPRINT 0xeeb05ecb9e775781ull   // accepted 2026-10-07 (kin-7jd): the C1 segment at rest is an authored knot, no longer Hard
#endif

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
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
    Engine<> e(cfg, 0.2f);
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
    // Past the last knot: a hold at its position (a C1 knot keeps the
    // author's acceleration into it, and the brake from there settles within
    // a hundred-thousandth of the window).
    CHECK(s[700].p == doctest::Approx(0.6f).epsilon(1e-4));
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

TEST_CASE("brake drops the future and stops as fast as the ceilings allow") {
    Config cfg; cfg.limits = {10.0f, 20.0f, 500.0f};
    Engine<> e(cfg, 0.1f);
    const uint64_t now = 0;
    REQUIRE(e.submit(knotAt(500 * kMs, 0.9f), now));
    REQUIRE(e.submit(knotAt(1000 * kMs, 0.1f), now));
    const State mid = e.stateAt(0, 250 * kMs);
    CHECK(mid.v > 0.0f);
    e.brake(250 * kMs);
    CHECK(e.pending() == 0);
    CHECK(e.isBusy(260 * kMs));
    const auto s = sweep(e, 250 * kMs, 1000 * kMs);
    CHECK(std::fabs(s.back().v) <= 1e-5f);
    CHECK(s.back().p > mid.p);        // it stopped ahead of where it was
    CHECK(s.back().p < 0.9f);         // and short of the dropped knot
    // Peaks: the decel ceiling is reached (a real brake, not a glide) and
    // nothing is exceeded. Jerk by finite difference over 1 ms.
    float a_min = 0.0f, j_max = 0.0f;
    for (size_t i = 1; i < s.size(); ++i) { a_min = std::min(a_min, s[i].a); j_max = std::max(j_max, std::fabs(s[i].a - s[i - 1].a) / 1e-3f); }
    CHECK(a_min <= -cfg.limits.amax * 0.99f);
    CHECK(a_min >= -cfg.limits.amax * 1.001f);
    CHECK(j_max <= cfg.limits.jmax * 1.05f);
    // Continuous into the brake: no jump at 250 ms.
    CHECK(s[0].p == doctest::Approx(mid.p));
    CHECK(s[0].v == doctest::Approx(mid.v));
    Anomaly an; bool settled = false;
    while (e.popAnomaly(an)) settled |= an.kind == uint8_t(AnomalyKind::SettleEngaged);
    CHECK(settled);
    // A knot before the brake's end is past; one after it chains from rest.
    CHECK_FALSE(e.submit(knotAt(300 * kMs, 0.5f), 260 * kMs));
    CHECK(e.submit(knotAt(1500 * kMs, 0.5f), 260 * kMs));
}

TEST_CASE("the brake profile stops exactly, from any entry state") {
    const Limits L{5.0f, 40.0f, 800.0f};
    const State entries[] = {{0.5f, 2.0f, 0.0f}, {0.5f, -3.0f, 10.0f}, {0.5f, 1.0f, -40.0f}, {0.5f, 0.1f, -30.0f}, {0.5f, 0.0f, 20.0f}, {0.5f, 4.0f, 35.0f}};
    for (const State& s0 : entries) {
        const Profile pr = Profile::brake(s0, 0, L);
        const State end = pr.atSeconds(pr.duration());
        CHECK(end.v == doctest::Approx(0.0f).epsilon(1e-3).scale(1.0));
        CHECK(end.a == doctest::Approx(0.0f).epsilon(1e-3).scale(1.0));
        CHECK(pr.worstRatio(L, -10.0f, 10.0f) <= 1.001f);
        for (int i = 0; i < pr.n; ++i) CHECK(pr.dt[i] >= 0.0f);
    }
}

TEST_CASE("a brake under a lower amax than the deceleration in flight never reverses (Nucleus val-9z5)") {
    // The arbiter rig on a 268 mm rail: a jog at 200 mm/s decelerating at
    // 200 mm/s^2 is paused, and the pause brake is planned under the input
    // amax of 20 mm/s^2 and the input jerk of 5e6 mm/s^3.
    const float rail = 268.0f;
    const Limits L{1000.0f / rail, 20.0f / rail, 5.0e6f / rail};
    const State s0{0.5f, 0.75f, -0.75f};
    const Profile pr = Profile::brake(s0, 0, L);
    for (int i = 0; i < pr.n; ++i) CHECK(pr.dt[i] >= 0.0f);
    float min_v = s0.v;
    for (float t = 0.0f; t <= pr.duration(); t += 1e-4f) min_v = std::fmin(min_v, pr.atSeconds(t).v);
    CHECK(min_v >= -1e-6f);
    // The end by the phases in closed form: a ten second brake read through
    // atSeconds() resolves its 4 us ramp-out only to float time at 10 s.
    State end = s0;
    for (int i = 0; i < pr.n; ++i) end = Profile::step(end, pr.jerk[i], pr.dt[i]);
    CHECK(end.v == doctest::Approx(0.0f).epsilon(1e-3).scale(1.0));
    CHECK(end.a == doctest::Approx(0.0f).epsilon(1e-3).scale(1.0));
    // Ahead of the entry by no more than the stop from v under the ceilings
    // plus the travel while the deceleration ramps down to amax.
    const float v = s0.v, A = L.amax, J = L.jmax;
    const float ramp_in = v * (-s0.a - A) / J;
    CHECK(end.p > s0.p);
    CHECK(end.p - s0.p <= v * v / (2.0f * A) + v * A / (2.0f * J) + ramp_in + 1e-5f);
    // After the ramp the deceleration never exceeds amax.
    float peak_a = 0.0f;
    for (float t = (-s0.a - A) / J; t <= pr.duration(); t += 1e-4f) peak_a = std::fmax(peak_a, std::fabs(pr.atSeconds(t).a));
    CHECK(peak_a <= A * 1.001f);
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
        Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
        Engine<> e(cfg, 0.25f);
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
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
    Engine<2> e(cfg, 0.5f);
    // Authored rests: a free last knot continues at its secant and brakes past it.
    REQUIRE(e.submit(0, knotAt(200 * kMs, 0.9f, true, 0.0f), 0));
    REQUIRE(e.submit(1, knotAt(300 * kMs, 0.1f, true, 0.0f), 0));
    CHECK(e.stateAt(0, 200 * kMs).p == doctest::Approx(0.9f).epsilon(1e-4));
    CHECK(e.stateAt(1, 200 * kMs).p != doctest::Approx(0.9f));
    CHECK(e.stateAt(1, 300 * kMs).p == doctest::Approx(0.1f).epsilon(1e-4));
    CHECK(e.stateAt(0, 300 * kMs).p == doctest::Approx(0.9f));
}

// ---- the solver (kin-ahl) ---------------------------------------------------

namespace {

struct Peaks { float v = 0, a = 0, j = 0, lo = 1e9f, hi = -1e9f; };

// Peaks of the sampled trajectory on a 1 ms grid; jerk by finite difference of a.
Peaks peaksOf(const std::vector<State>& s) {
    Peaks pk;
    for (size_t i = 0; i < s.size(); ++i) {
        pk.v = std::max(pk.v, std::fabs(s[i].v));
        pk.a = std::max(pk.a, std::fabs(s[i].a));
        pk.lo = std::min(pk.lo, s[i].p); pk.hi = std::max(pk.hi, s[i].p);
        if (i) pk.j = std::max(pk.j, std::fabs(s[i].a - s[i - 1].a) / 1e-3f);
    }
    return pk;
}

std::vector<Anomaly> drain(Engine<>& e) {
    std::vector<Anomaly> out; Anomaly an;
    while (e.popAnomaly(an)) out.push_back(an);
    return out;
}
int countKind(const std::vector<Anomaly>& v, AnomalyKind k, float* last_detail = nullptr) {
    int n = 0;
    for (const Anomaly& an : v) if (an.kind == uint8_t(k)) { ++n; if (last_detail) *last_detail = an.detail; }
    return n;
}

}  // namespace

TEST_CASE("a legal script keeps every ceiling in sampled reality") {
    Config cfg; cfg.limits = {3.0f, 30.0f, 500.0f};
    Engine<> e(cfg, 0.2f);
    // A gentle stroke chain: 0.6 of travel per 500 ms (rest-to-rest peaks:
    // v 2.25, a 13.9, j 288), under every ceiling.
    for (int i = 1; i <= 6; ++i) REQUIRE(e.submit(knotAt(uint64_t(i) * 500 * kMs, (i % 2) ? 0.8f : 0.2f), 0));
    const auto s = sweep(e, 0, 3200 * kMs);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
    CHECK(pk.j <= cfg.limits.jmax * 1.05f);    // finite difference over 1 ms
    CHECK(pk.lo >= -1e-4f); CHECK(pk.hi <= 1.0f + 1e-4f);
    for (int i = 1; i <= 6; ++i) CHECK(s[size_t(i) * 500].p == doctest::Approx((i % 2) ? 0.8f : 0.2f).epsilon(1e-4));
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::WaveformScaled) == 0);
    CHECK(countKind(an, AnomalyKind::DeadlineStretched) == 0);
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
}

TEST_CASE("free knots ring by less than 0.2 percent of the window at a kink") {
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
    Engine<> e(cfg, 0.1f);
    // A staircase: monotone rising steps then a plateau. The minimum-jerk
    // junctions may dip a hair at the kink; never more than this, never a bulge.
    const float ps[] = {0.3f, 0.5f, 0.52f, 0.9f, 0.9f, 0.9f};
    for (int i = 0; i < 6; ++i) REQUIRE(e.submit(knotAt(uint64_t(i + 1) * 100 * kMs, ps[i]), 0));
    const auto s = sweep(e, 0, 600 * kMs);
    float worst_dip = 0.0f, run_max = s[0].p;
    for (size_t i = 1; i < s.size(); ++i) { run_max = std::max(run_max, s[i].p); worst_dip = std::max(worst_dip, run_max - s[i].p); }
    CHECK(worst_dip <= 2e-3f);
    CHECK(peaksOf(s).hi <= 0.9f + 2e-3f);
    for (int i = 0; i < 6; ++i) CHECK(s[size_t(i + 1) * 100].p == doctest::Approx(ps[i]).epsilon(2e-3));
}

TEST_CASE("Blend trims an impossible stroke to the ceilings and reports the share") {
    Config cfg; cfg.limits = {2.0f, 50.0f, 5000.0f}; cfg.policy = Policy::Blend; cfg.amplitude_floor = 0.05f;
    Engine<> e(cfg, 0.0f);
    // Full travel in 120 ms: the accel ceiling allows about 0.125 of it.
    REQUIRE(e.submit(knotAt(120 * kMs, 1.0f, true, 0.0f), 0));
    const auto s = sweep(e, 0, 200 * kMs);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
    CHECK(pk.j <= cfg.limits.jmax * 1.05f);
    CHECK(s[120].p < 1.0f);                 // amplitude spent
    CHECK(s[120].p > cfg.amplitude_floor);  // but not below the floor
    CHECK(s[120].v == doctest::Approx(0.0f).epsilon(1e-3));   // deadline and the stop kept
    float share = 0.0f;
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::WaveformScaled, &share) == 1);
    CHECK(share == doctest::Approx(s[120].p).epsilon(1e-3));
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
}

TEST_CASE("Stretch keeps the stroke, moves the knot and everything after it") {
    Config cfg; cfg.limits = {2.0f, 50.0f, 5000.0f}; cfg.policy = Policy::Stretch;
    Engine<> e(cfg, 0.0f);
    REQUIRE(e.submit(knotAt(120 * kMs, 1.0f, true, 0.0f), 0));
    REQUIRE(e.submit(knotAt(620 * kMs, 0.5f, true, 0.0f), 0));
    CHECK(e.isBusy(700 * kMs));             // the second knot moved past 700 ms
    const auto s = sweep(e, 0, 1500 * kMs);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
    CHECK(pk.hi >= 1.0f - 1e-3f);           // the full stroke happened
    float added = 0.0f;
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::DeadlineStretched, &added) == 1);
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
    CHECK(added > 0.1f);
    // The second knot moved by the same amount: it lands at 0.5 at 620 ms + added.
    const size_t t2 = 620 + size_t(added * 1000.0f + 0.5f);
    CHECK(s[t2].p == doctest::Approx(0.5f).epsilon(2e-3));
}

TEST_CASE("the rail is a wall: a reversal that would bulge past it is spent") {
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f}; cfg.policy = Policy::Blend; cfg.amplitude_floor = 0.1f;
    Engine<> e(cfg, 0.5f);
    // Arrive at the top rail fast and leave fast: the authored velocities would
    // carry the curve past 1.0 between the knots.
    REQUIRE(e.submit(knotAt(100 * kMs, 1.0f, true, 4.0f), 0));
    REQUIRE(e.submit(knotAt(200 * kMs, 0.5f, true, 0.0f), 0));
    const auto s = sweep(e, 0, 250 * kMs);
    CHECK(peaksOf(s).hi <= 1.0f + 1e-4f);
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::EndVelClamped) >= 1);
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
}

TEST_CASE("a hard stop keeps its speed longer than a smooth stop and lands at rest") {
    Config cfg; cfg.limits = {4.0f, 40.0f, 1000.0f};
    auto run = [&](Family f) {
        Engine<> e(cfg, 0.0f);
        Knot k = knotAt(500 * kMs, 0.8f, true, 0.0f, f);
        k.sample = f == Family::C1;   // Hard is a live jog: a C1 sample at rest
        REQUIRE(e.submit(k, 0));
        return sweep(e, 0, 600 * kMs);
    };
    const auto hard = run(Family::C1), smooth = run(Family::C2);
    for (const auto* s : {&hard, &smooth}) {
        CHECK((*s)[500].p == doctest::Approx(0.8f).epsilon(1e-3));
        CHECK(std::fabs((*s)[500].v) < 1e-2f);
        const Peaks pk = peaksOf(*s);
        CHECK(pk.v <= cfg.limits.vmax * 1.001f);
        CHECK(pk.a <= cfg.limits.amax * 1.001f);
        CHECK(pk.j <= cfg.limits.jmax * 1.05f);
    }
    // Time at which each has fallen to half its own peak speed, on the way in.
    auto halfTime = [](const std::vector<State>& s) {
        float vpk = 0.0f; size_t ipk = 0;
        for (size_t i = 0; i < 500; ++i) if (s[i].v > vpk) { vpk = s[i].v; ipk = i; }
        for (size_t i = ipk; i < 500; ++i) if (s[i].v < 0.5f * vpk) return i;
        return size_t(500);
    };
    CHECK(halfTime(hard) > halfTime(smooth));
    // The hard stop reaches the decel ceiling; the smooth one need not.
    float a_hard = 0.0f;
    for (const State& x : hard) a_hard = std::min(a_hard, x.a);
    CHECK(a_hard <= -cfg.limits.amax * 0.98f);
}

// ---- sources: one sample behind (kin-ob3) ------------------------------------
#include "kinetic2/sources.hpp"

TEST_CASE("a sample stream renders one behind and never overshoots a dead stop") {
    // The scrub that overshot on the bench (Phosphor ph-ffsk, Nucleus val-1bf):
    // 60 Hz samples racing across the window, then the pointer stops dead.
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
    const uint32_t latency = 40 * kMs;   // the grant's schedule_latency_us
    Engine<> e(cfg, 0.1f);
    const uint64_t dt = 16667;           // 60 Hz
    uint64_t t = 0;
    float p = 0.1f;
    std::vector<State> s;
    auto sampleUntil = [&](uint64_t until) { for (; t < until; t += kMs) s.push_back(e.stateAt(0, t)); };
    // 20 samples moving fast (0.03 per sample = 1.8 units/s), then 30 samples parked at the stop.
    for (int i = 0; i < 50; ++i) {
        if (i < 20) p += 0.03f;
        REQUIRE(e.submit(knotFromSample(p, t, latency), t));
        sampleUntil(t + dt);
    }
    sampleUntil(t + 200 * kMs);
    const float stop = p;
    float hi = -1.0f;
    for (const State& x : s) hi = std::max(hi, x.p);
    CHECK(hi <= stop + 1e-4f);                    // no overshoot past the last sample
    CHECK(s.back().p == doctest::Approx(stop).epsilon(1e-4));
    CHECK(std::fabs(s.back().v) <= 1e-5f);
    // One behind: the curve passes each moving sample `latency` after it arrived.
    const State at_k10 = e.stateAt(0, 10 * dt + latency);
    (void)at_k10;   // the engine's clock has moved on; the sweep is the record
    const size_t idx = size_t((10 * dt + latency) / kMs);
    CHECK(s[idx].p == doctest::Approx(0.1f + 11 * 0.03f).epsilon(2e-2));
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
}

TEST_CASE("segments become knots at anchor plus duration with the authored end velocity") {
    const Knot k = knotFromSegment(0.7f, 250 * kMs, true, -1.5f, 1000 * kMs, Family::C1);
    CHECK(k.t_us == 1250 * kMs);
    CHECK(k.p == 0.7f);
    CHECK(k.has_v); CHECK(k.v == -1.5f);
    CHECK(junctionOf(k) == Junction::Authored);
    // An authored C1 segment at rest is a reversal or a hold, never Hard;
    // a C1 sample at rest (a live jog) is.
    CHECK(junctionOf(knotFromSegment(0.7f, 250 * kMs, true, 0.0f, 0, Family::C1)) == Junction::Authored);
    Knot jog = knotFromSample(0.7f, 0, 250 * kMs); jog.has_v = true; jog.family = Family::C1;
    CHECK(junctionOf(jog) == Junction::Hard);
    CHECK(junctionOf(knotFromSegment(0.7f, 250 * kMs, false, 0.0f, 0, Family::C2)) == Junction::Smooth);
}

// ---- the property suite (kin-vcr) --------------------------------------------

namespace {

// A small deterministic PRNG (xorshift32): the same sequences on every host.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    float uni(float lo, float hi) { return lo + (hi - lo) * float(next() % 10000u) / 9999.0f; }
    int pick(int n) { return int(next() % uint32_t(n)); }
};

// FNV-1a over the sampled states: a run's fingerprint.
uint64_t fingerprint(const std::vector<State>& s) {
    uint64_t h = 1469598103934665603ull;
    for (const State& x : s) {
        const unsigned char* b = reinterpret_cast<const unsigned char*>(&x);
        for (size_t i = 0; i < sizeof(State); ++i) { h ^= b[i]; h *= 1099511628211ull; }
    }
    return h;
}

struct Score { int knots = 0, hit = 0, spent = 0, violations = 0, overshoots = 0, failed = 0; float amplitude = 1.0f; int why[5] = {}; };

// Random knot sequences: free, authored and hard knots, legal and not, over a
// random policy and ceilings. Returns the sampled score.
Score randomRun(uint32_t seed, Policy policy) {
    Rng r(seed);
    Config cfg;
    cfg.limits = {r.uni(1.0f, 8.0f), r.uni(10.0f, 200.0f), r.uni(200.0f, 20000.0f)};
    cfg.policy = policy; cfg.amplitude_floor = r.uni(0.05f, 0.4f);
    Engine<> e(cfg, r.uni(0.0f, 1.0f));
    const int n = 3 + r.pick(20);
    std::vector<Knot> ks;
    uint64_t t = 0;
    for (int i = 0; i < n; ++i) {
        t += uint64_t(r.uni(20.0f, 600.0f) * 1000.0f);
        Knot k; k.t_us = t; k.p = r.uni(0.0f, 1.0f);
        const int kind = r.pick(3);
        if (kind == 1) { k.has_v = true; k.v = r.uni(-cfg.limits.vmax, cfg.limits.vmax); k.family = Family::C2; }
        if (kind == 2) { k.has_v = true; k.v = 0.0f; k.family = Family::C1; }
        if (kind == 0) k.family = Family::C2;
        ks.push_back(k);
    }
    // Submit everything up front (a scheduled bundle), then sample through.
    for (const Knot& k : ks) REQUIRE(e.submit(k, 0));
    std::vector<State> s;
    const uint64_t end = t + 300 * kMs;
    // Sample past the sender's end until the engine is idle: Stretch may have
    // moved the last knot (a 30 s cap guards the loop).
    for (uint64_t x = 0;; x += kMs) {
        s.push_back(e.stateAt(0, x));
        if ((x >= end && !e.isBusy(x)) || x > 30000 * kMs) break;
    }
    Score sc; sc.knots = n;
    const Peaks pk = peaksOf(s);
    if (pk.v > cfg.limits.vmax * 1.01f) { ++sc.violations; ++sc.why[0]; }
    if (pk.a > cfg.limits.amax * 1.01f) { ++sc.violations; ++sc.why[1]; }
    if (pk.j > cfg.limits.jmax * 1.10f) { ++sc.violations; ++sc.why[2]; }   // finite difference over 1 ms
    if (pk.lo < -1e-3f || pk.hi > 1.0f + 1e-3f) { ++sc.violations; ++sc.why[3]; }
    const auto an = drain(e);
    sc.failed = countKind(an, AnomalyKind::PlanFailed);
    sc.spent = countKind(an, AnomalyKind::WaveformScaled) + countKind(an, AnomalyKind::DeadlineStretched);
    // Every knot not spent is hit at its time, at rest if hard.
    for (const Knot& k : ks) {
        const size_t idx = size_t(k.t_us / kMs);
        bool spentHere = false;
        for (const Anomaly& a : an) if ((a.kind == uint8_t(AnomalyKind::WaveformScaled) || a.kind == uint8_t(AnomalyKind::DeadlineStretched) || a.kind == uint8_t(AnomalyKind::PlanFailed)) && std::fabs(a.target - k.p) < 1e-3f) spentHere = true;
        if (policy == Policy::Stretch && sc.spent) break;   // times moved: the index no longer applies after the first stretch
        if (!spentHere && idx < s.size()) {
            if (std::fabs(s[idx].p - k.p) < 2e-3f) ++sc.hit;
        }
    }
    // The curve ends at rest on the last knot's solved position.
    if (std::fabs(s.back().v) > 1e-3f) { ++sc.violations; ++sc.why[4]; }
    return sc;
}

}  // namespace

TEST_CASE("property: random knot sequences never exceed a ceiling or the window, under either policy") {
    int runs = 0, violations = 0, spent = 0, hits = 0, knots = 0, failed = 0, why[5] = {}, withFail = 0, withoutFail = 0;
    for (uint32_t seed = 1; seed <= 400; ++seed) {
        for (const Policy pol : {Policy::Blend, Policy::Stretch}) {
            const Score sc = randomRun(seed, pol);
            ++runs; violations += sc.violations; spent += sc.spent; hits += sc.hit; knots += sc.knots; failed += sc.failed;
            for (int w = 0; w < 5; ++w) why[w] += sc.why[w];
            if (sc.violations) { if (sc.failed) ++withFail; else ++withoutFail; }
            if (sc.violations && (withFail + withoutFail) <= 6) MESSAGE("seed " << seed << " policy " << int(pol) << ": v" << sc.why[0] << " a" << sc.why[1] << " j" << sc.why[2] << " win" << sc.why[3] << " rest" << sc.why[4] << " failed " << sc.failed);
        }
    }
    MESSAGE(runs << " runs, " << knots << " knots, " << hits << " hit exactly, " << spent << " spent, " << failed << " PlanFailed, " << violations
            << " violations (v " << why[0] << ", a " << why[1] << ", j " << why[2] << ", window " << why[3] << ", rest " << why[4] << "); runs with violations: " << withFail << " with PlanFailed, " << withoutFail << " without");
    CHECK(violations == 0);
    CHECK(hits > 0);
}

TEST_CASE("fingerprint: the canonical run has not changed bits") {
    // A fixed scenario covering every junction kind and both spends. The
    // constant below is the hash of the accepted output; a change here is a
    // change in rendered motion and must be deliberate (update the constant
    // in the same commit, say why).
    Config cfg; cfg.limits = {4.0f, 60.0f, 3000.0f}; cfg.policy = Policy::Blend; cfg.amplitude_floor = 0.2f;
    Engine<> e(cfg, 0.3f);
    REQUIRE(e.submit(knotAt(150 * kMs, 0.9f), 0));
    REQUIRE(e.submit(knotAt(260 * kMs, 0.1f, true, -1.0f), 0));
    REQUIRE(e.submit(knotAt(500 * kMs, 0.7f, true, 0.0f, Family::C1), 0));
    REQUIRE(e.submit(knotAt(520 * kMs, 0.95f), 0));
    REQUIRE(e.submit(knotAt(900 * kMs, 0.4f), 0));
    std::vector<State> s;
    for (uint64_t t = 0; t <= 1000 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
    e.brake(1000 * kMs);
    for (uint64_t t = 1000 * kMs; t <= 1200 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
    const uint64_t fp = fingerprint(s);
    MESSAGE("fingerprint 0x" << std::hex << fp);
    CHECK(fp == KINETIC2_FINGERPRINT);
}

// ---- the corner option (kin-jub, RFC-105 planner option `corner`) -------------

TEST_CASE("corner: cubic keeps the author's acceleration step as a jerk-limited ramp, continuous smooths it") {
    // A rising corner: fast in, slow out, authored C1 with nonzero velocity.
    auto run = [](Corner c) {
        Config cfg; cfg.limits = {6.0f, 80.0f, 2000.0f}; cfg.corner = c;
        Engine<> e(cfg, 0.1f);
        REQUIRE(e.submit(knotAt(300 * kMs, 0.6f, true, 2.5f, Family::C1), 0));
        REQUIRE(e.submit(knotAt(700 * kMs, 0.9f, true, 0.3f, Family::C1), 0));
        REQUIRE(e.submit(knotAt(1000 * kMs, 0.9f, true, 0.0f, Family::C1), 0));
        return sweep(e, 0, 1100 * kMs);
    };
    const auto cubic = run(Corner::Cubic), cont = run(Corner::Continuous);
    for (const auto* s : {&cubic, &cont}) {
        // The knot is hit at its time with its velocity, under every ceiling.
        CHECK((*s)[300].p == doctest::Approx(0.6f).epsilon(2e-3));
        CHECK((*s)[300].v == doctest::Approx(2.5f).epsilon(5e-2));
        CHECK((*s)[700].p == doctest::Approx(0.9f).epsilon(2e-3));
        const Peaks pk = peaksOf(*s);
        CHECK(pk.v <= 6.0f * 1.001f);
        CHECK(pk.a <= 80.0f * 1.001f);
        CHECK(pk.j <= 2000.0f * 1.05f);
    }
    // Cubic: each side of the knot carries the author's cubic acceleration
    // (left: the span from rest at 0.1 over 300 ms; right: the span to 0.9 at
    // 0.3 over 400 ms), joined by a ramp at the jerk ceiling. The ramp is
    // |a_r - a_l| / jmax long, centered on the knot.
    const float a_l = (6.0f * (0.1f - 0.6f) + 0.3f * (2.0f * 0.0f + 4.0f * 2.5f)) / (0.3f * 0.3f);
    const float a_r = (6.0f * (0.9f - 0.6f) - 0.4f * (4.0f * 2.5f + 2.0f * 0.3f)) / (0.4f * 0.4f);
    const float h_ms = 0.5f * std::fabs(a_r - a_l) / 2000.0f * 1000.0f;
    const size_t before = 300 - size_t(h_ms + 2.0f), after = 300 + size_t(h_ms + 2.0f);
    CHECK(cubic[before].a == doctest::Approx(a_l).epsilon(0.2).scale(10.0));
    CHECK(cubic[after].a == doctest::Approx(a_r).epsilon(0.2).scale(10.0));
    // Continuous has no such step: its acceleration moves smoothly through.
    CHECK(std::fabs(cont[after].a - cont[before].a) < 0.5f * std::fabs(a_r - a_l));
    float jpk = 0.0f;
    for (size_t i = 281; i <= 320; ++i) jpk = std::max(jpk, std::fabs(cubic[i].a - cubic[i - 1].a) / 1e-3f);
    CHECK(jpk == doctest::Approx(2000.0f).epsilon(0.05));
    // The default is Cubic (operator ruling 2026-10-07).
    Config d; CHECK(d.corner == Corner::Cubic);
}

// ---- the oscillation modulator (kin-b5g, RFC-103) -----------------------------
#include "kinetic2/oscillator.hpp"

namespace {
// Sample the sum of a planned state stream and the oscillator at 1 ms.
std::vector<State> oscSweep(Oscillator& o, const Limits& L, const std::vector<State>& planned, uint64_t t0 = 0) {
    std::vector<State> s;
    for (size_t i = 0; i < planned.size(); ++i) s.push_back(o.apply(planned[i], t0 + i * kMs, L, 0.0f, 1.0f));
    return s;
}
std::vector<State> rest(float p, size_t n) { return std::vector<State>(n, State{p, 0.0f, 0.0f}); }
}  // namespace

TEST_CASE("oscillator: a sine at rest swings the asked amplitude at the asked period, inside the ceilings") {
    const Limits L{5.0f, 200.0f, 20000.0f};
    Oscillator o; OscParams p; p.enabled = true; p.frequency = 10.0f; p.amplitude = 0.02f; p.shape = OscShape::Sine; o.set(p);
    const auto s = oscSweep(o, L, rest(0.5f, 2000));
    float lo = 1, hi = 0; for (const State& x : s) { lo = std::min(lo, x.p); hi = std::max(hi, x.p); }
    CHECK(hi == doctest::Approx(0.52f).epsilon(2e-3));
    CHECK(lo == doctest::Approx(0.48f).epsilon(2e-3));
    CHECK(o.amplitudeEffective() == doctest::Approx(0.02f));
    // Period: the crest recurs every 100 ms.
    size_t first = 0; for (size_t i = 1; i + 1 < 300; ++i) if (s[i].p > s[i - 1].p && s[i].p >= s[i + 1].p) { first = i; break; }
    size_t second = 0; for (size_t i = first + 50; i + 1 < 400; ++i) if (s[i].p > s[i - 1].p && s[i].p >= s[i + 1].p) { second = i; break; }
    CHECK(second - first == doctest::Approx(100).epsilon(0.03));
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= L.vmax * 1.001f); CHECK(pk.a <= L.amax * 1.001f); CHECK(pk.j <= L.jmax * 1.05f);
    // 10 Hz at 0.02: v peak 1.26, a peak 79, j peak 4960: all under, so nothing was shed.
    CHECK(pk.a > 70.0f);
}

TEST_CASE("oscillator: yields first; a full-speed stroke sheds it to nothing, half speed keeps part") {
    const Limits L{4.0f, 60.0f, 2000.0f};
    Oscillator o; OscParams p; p.enabled = true; p.frequency = 20.0f; p.amplitude = 0.05f; o.set(p);
    std::vector<State> full(500, State{0.5f, 4.0f, 0.0f}), half(500, State{0.5f, 2.0f, 0.0f});
    oscSweep(o, L, full);
    CHECK(o.amplitudeEffective() == 0.0f);
    CHECK_FALSE(o.active());
    o.set(p);
    const auto s = oscSweep(o, L, half);
    CHECK(o.amplitudeEffective() > 0.0f);
    CHECK(o.amplitudeEffective() < 0.05f);   // the jerk ceiling binds at 20 Hz
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= L.vmax * 1.001f); CHECK(pk.a <= L.amax * 1.001f); CHECK(pk.j <= L.jmax * 1.05f);
}

TEST_CASE("oscillator: every shape stays under the ceilings and inside its amplitude; dwells hold the extremes") {
    const Limits L{5.0f, 300.0f, 30000.0f};
    for (const OscShape sh : {OscShape::Sine, OscShape::Square, OscShape::Saw, OscShape::SawReverse}) {
        Oscillator o; OscParams p; p.enabled = true; p.frequency = 4.0f; p.amplitude = 0.05f; p.shape = sh; p.dwell_crest = 0.3f; o.set(p);
        const auto s = oscSweep(o, L, rest(0.5f, 2600));
        const Peaks pk = peaksOf(s);
        CHECK(pk.v <= L.vmax * 1.001f); CHECK(pk.a <= L.amax * 1.001f); CHECK(pk.j <= L.jmax * 1.05f);
        CHECK(pk.hi <= 0.55f + 1e-3f); CHECK(pk.lo >= 0.45f - 1e-3f);
        // Period with a 0.3 crest dwell: 1.3 / 4 Hz = 325 ms; the crest holds 75 ms of it.
        const float crest = 0.5f + o.amplitudeEffective();   // the ramped shapes shed amplitude to the jerk ceiling
        size_t atCrest = 0; for (size_t i = 325; i < 325 * 5; ++i) if (s[i].p > crest - 2e-3f) ++atCrest;
        const float share = float(atCrest) / (325.0f * 4.0f);
        // A saw arrives at its crest moving, so the saw shapes ignore dwells by design.
        if (sh == OscShape::Sine || sh == OscShape::Square) CHECK(share >= 0.3f / 1.3f * 0.9f);
        CHECK(o.amplitudeEffective() > 0.0f);
    }
}

TEST_CASE("oscillator: never leaves the window and is bit-exact") {
    const Limits L{5.0f, 300.0f, 30000.0f};
    auto run = [&] {
        Oscillator o; OscParams p; p.enabled = true; p.frequency = 3.0f; p.amplitude = 0.2f; o.set(p);
        return oscSweep(o, L, rest(0.05f, 700));   // 0.05 from the low rail
    };
    const auto a = run(), b = run();
    const Peaks pk = peaksOf(a);
    CHECK(pk.lo >= -1e-6f);
    CHECK(pk.hi <= 0.1f + 1e-3f);   // amplitude limited to the gap, 0.05
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(State)) == 0);
}

TEST_CASE("oscillator over a planned stroke: the sum keeps every ceiling") {
    Config cfg; cfg.limits = {4.0f, 60.0f, 2000.0f};
    Engine<> e(cfg, 0.2f);
    REQUIRE(e.submit(knotAt(600 * kMs, 0.8f, true, 0.0f), 0));
    REQUIRE(e.submit(knotAt(1200 * kMs, 0.2f, true, 0.0f), 0));
    Oscillator o; OscParams p; p.enabled = true; p.frequency = 8.0f; p.amplitude = 0.03f; o.set(p);
    std::vector<State> sum;
    for (uint64_t t = 0; t <= 1400 * kMs; t += kMs) sum.push_back(o.apply(e.stateAt(0, t), t, cfg.limits, 0.0f, 1.0f));
    const Peaks pk = peaksOf(sum);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
    CHECK(pk.j <= cfg.limits.jmax * 1.10f);
    CHECK(pk.lo >= -1e-3f); CHECK(pk.hi <= 1.0f + 1e-3f);
}

// ---- continuity under submits in flight (the arbiter's measured 67 mm jump) ----

TEST_CASE("a knot submitted mid-flight never moves the curve under the carriage") {
    Config cfg; cfg.limits = {4.0f, 60.0f, 2000.0f};
    for (const Policy pol : {Policy::Blend, Policy::Stretch}) {
        cfg.policy = pol;
        Engine<> e(cfg, 0.2f);
        std::vector<State> s;
        uint64_t t = 0;
        // 250 ms C2 segments arriving every 250 ms, 125 ms ahead of their start, while sampling.
        auto sampleUntil = [&](uint64_t until) { for (; t < until; t += kMs) s.push_back(e.stateAt(0, t)); };
        float target = 0.8f;
        for (int i = 0; i < 12; ++i) {
            const uint64_t start = (i + 1) * 250 * kMs;
            sampleUntil(start - 125 * kMs);
            REQUIRE(e.submit(knotFromSegment(target, 250 * kMs, true, 0.0f, start, Family::C2), t));
            target = target > 0.5f ? 0.2f : 0.8f;
        }
        sampleUntil(t + 600 * kMs);
        float worst_jump = 0.0f;
        for (size_t i = 1; i < s.size(); ++i) {
            const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
            worst_jump = std::max(worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
        }
        CHECK(worst_jump <= 0.0f);
        const Peaks pk = peaksOf(s);
        CHECK(pk.v <= cfg.limits.vmax * 1.001f);
        CHECK(pk.a <= cfg.limits.amax * 1.001f);
        CHECK(pk.j <= cfg.limits.jmax * 1.10f);
    }
}

TEST_CASE("a 60 Hz scrub submitted while sampling stays continuous and one behind") {
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f}; cfg.policy = Policy::Stretch;
    Engine<> e(cfg, 0.1f);
    std::vector<State> s;
    uint64_t t = 0; float p = 0.1f;
    auto sampleUntil = [&](uint64_t until) { for (; t < until; t += kMs) s.push_back(e.stateAt(0, t)); };
    int refused = 0;
    for (int i = 0; i < 90; ++i) {
        if (i < 40) p += 0.015f;
        if (!e.submit(knotFromSample(p, t, 40 * kMs), t)) ++refused;
        sampleUntil(t + 16667);
    }
    sampleUntil(t + 300 * kMs);
    float worst_jump = 0.0f;
    for (size_t i = 1; i < s.size(); ++i) {
        const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
        worst_jump = std::max(worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
    }
    CHECK(worst_jump <= 0.0f);
    CHECK(refused == 0);
    CHECK(s.back().p == doctest::Approx(p).epsilon(1e-3));
}

// ---- spent streams (val-log) --------------------------------------------------
// The arbiter's scrub on the OSSM window: a 150 mm/s ramp, a hold, a 1.5 Hz
// sweep of 60 mm on 400 mm, under its default ceilings (1000 mm/s, 50000
// mm/s^2, 2e6 mm/s^3 normalized). Every knot here spends something.

namespace {

struct StreamRun {
    float worst_jump = 0.0f;   // beyond what the velocity carries per tick
    int   refused = 0;
    int   dropped = 0;
    float worst_lag_ms = 0.0f; // the newest knot's solved time behind its authored time, at any submit
    int   worst_lag_at = -1;
    float end_p = 0.0f;
};

StreamRun runArbiterScrub(Policy policy, float* out_last = nullptr) {
    Config cfg; cfg.limits = {2.5f, 125.0f, 5000.0f}; cfg.policy = policy;
    Engine<> e(cfg, 0.5f);
    StreamRun r;
    std::vector<State> s;
    uint64_t t = 0;
    auto sampleUntil = [&](uint64_t until) { for (; t < until; t += kMs) s.push_back(e.stateAt(0, t)); };
    float p = 0.5f;
    for (int i = 0; i < 240; ++i) {
        p = i < 40 ? (200.0f + 2.5f * float(i + 1)) / 400.0f
          : i < 60 ? 0.75f
                   : 0.75f + 0.15f * std::sin(9.424778f * float(i - 60) / 60.0f);
        const Knot k = knotFromSample(p, t, 16667);
        if (!e.submit(k, t)) ++r.refused;
        if (const size_t n = e.pending(0)) {
            const uint64_t tail = e.solved(0, n - 1).t_us;
            if (tail > k.t_us && float(tail - k.t_us) * 1e-3f > r.worst_lag_ms) { r.worst_lag_ms = float(tail - k.t_us) * 1e-3f; r.worst_lag_at = i; }
        }
        sampleUntil(t + (i % 3 == 2 ? 16 : 17) * kMs);
    }
    sampleUntil(t + 500 * kMs);
    for (size_t i = 1; i < s.size(); ++i) {
        const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
        r.worst_jump = std::max(r.worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
    }
    Anomaly a;
    while (e.popAnomaly(a)) if (a.kind == uint8_t(AnomalyKind::PlanFailed)) ++r.dropped;
    r.end_p = s.back().p;
    if (out_last) *out_last = p;
    return r;
}

}  // namespace

TEST_CASE("a spent 60 Hz stream stays continuous, refuses nothing, drops nothing and tracks") {
    for (Policy policy : {Policy::Stretch, Policy::Blend}) {
        CAPTURE(int(policy));
        float last = 0.0f;
        const StreamRun r = runArbiterScrub(policy, &last);
        MESSAGE("policy ", int(policy), " jump ", r.worst_jump * 400.0f, " mm, refused ", r.refused, ", dropped ", r.dropped,
                ", worst lag ", r.worst_lag_ms, " ms at sample ", r.worst_lag_at, ", end ", r.end_p * 400.0f, " mm vs ", last * 400.0f);
        CHECK(r.worst_jump <= 0.0f);
        CHECK(r.refused == 0);
        CHECK(r.dropped == 0);
        // The stream's corners are infeasible on purpose (instant starts and
        // stops): each one costs a stretch, and the replay closes it. Eight
        // cadences is the bound; a lag that grew would fill the timeline.
        CHECK(r.worst_lag_ms <= 135.0f);
        // The stream ends moving (a sweep cut mid-swing): the engine brakes
        // past its newest sample by at most the stop distance from the
        // stream's speed there (RFC-105 (dd)).
        const float v_end = (0.15f * std::sin(9.424778f * 179.0f / 60.0f) - 0.15f * std::sin(9.424778f * 178.0f / 60.0f)) / 0.016667f;
        const float stop = v_end * v_end / (2.0f * 125.0f) + std::fabs(v_end) * 125.0f / (2.0f * 5000.0f);
        CHECK(std::fabs(r.end_p - last) <= stop + 0.003f);
    }
}

TEST_CASE("a lone far sample from rest is reached exactly, at rest, never dropped or trimmed") {
    for (Policy policy : {Policy::Stretch, Policy::Blend}) {
        CAPTURE(int(policy));
        Config cfg; cfg.limits = {2.5f, 125.0f, 5000.0f}; cfg.policy = policy;
        Engine<> e(cfg, 0.5f);
        REQUIRE(e.submit(knotFromSample(0.75f, 0, 16667), 0));
        const auto s = sweep(e, 0, 2000 * kMs);
        float worst_jump = 0.0f, peak = 0.0f;
        for (size_t i = 1; i < s.size(); ++i) {
            const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
            worst_jump = std::max(worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
            peak = std::max(peak, s[i].p);
        }
        int dropped = 0; Anomaly a;
        while (e.popAnomaly(a)) if (a.kind == uint8_t(AnomalyKind::PlanFailed)) ++dropped;
        MESSAGE("policy ", int(policy), " reached ", peak * 400.0f, " mm, end ", s.back().p * 400.0f, " mm, jump ", worst_jump * 400.0f);
        CHECK(dropped == 0);
        CHECK(worst_jump <= 0.0f);
        // A sample is never trimmed (Blend is for segments): under either
        // policy the lone sample is reached exactly, and at rest.
        CHECK(peak == doctest::Approx(0.75f).epsilon(2e-3));
        CHECK(s.back().p == doctest::Approx(0.75f).epsilon(2e-3));
    }
}

TEST_CASE("a staircase of segments without end velocities, authored ahead, stays continuous and rests at the top") {
    Config cfg;   // default ceilings
    Engine<> e(cfg, 0.1f);
    for (int k = 1; k <= 5; ++k)
        REQUIRE(e.submit(knotFromSegment(0.1f + 0.15f * float(k), 300 * kMs, false, 0.0f, uint64_t(k - 1) * 300 * kMs, Family::C2), 0));
    const auto s = sweep(e, 0, 2200 * kMs);
    float worst_jump = 0.0f, lo = 1.0f, hi = 0.0f;
    for (size_t i = 1; i < s.size(); ++i) {
        const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
        worst_jump = std::max(worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
        lo = std::min(lo, s[i].p); hi = std::max(hi, s[i].p);
    }
    CHECK(worst_jump <= 0.0f);
    CHECK(lo >= 0.1f - 1e-4f);
    CHECK(hi <= 0.85f + 1e-4f);
    CHECK(s.back().p == doctest::Approx(0.85f).epsilon(1e-4));
    CHECK(std::fabs(s.back().v) < 1e-4f);
}

TEST_CASE("a lone hard stop from rest holds, launches and lands: it never winds up backward") {
    Config cfg;
    Engine<> e(cfg, 0.1f);
    Knot k = knotAt(1500 * kMs, 0.5f, true, 0.0f, Family::C1);
    k.sample = true;   // a live jog
    REQUIRE(e.submit(k, 0));
    const auto s = sweep(e, 0, 1600 * kMs);
    float worst_jump = 0.0f, lo = 1.0f, hi = 0.0f;
    for (size_t i = 1; i < s.size(); ++i) {
        const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
        worst_jump = std::max(worst_jump, std::fabs(s[i].p - s[i - 1].p) - allowed);
        lo = std::min(lo, s[i].p); hi = std::max(hi, s[i].p);
    }
    CHECK(worst_jump <= 0.0f);
    CHECK(lo >= 0.1f - 1e-4f);
    CHECK(hi <= 0.5f + 1e-4f);
    CHECK(s[1500].p == doctest::Approx(0.5f).epsilon(1e-4));
    CHECK(std::fabs(s[1500].v) < 1e-3f);
    CHECK(s[200].p == doctest::Approx(0.1f).epsilon(1e-4));   // holding, not crawling
}

TEST_CASE("a segment with no end velocity and no successor rests at its target; a successor frees it (SPEC 9.6)") {
    Config cfg; cfg.limits = {2.5f, 125.0f, 5000.0f};
    Engine<> e(cfg, 0.0f);
    // 50 mm of 400 in 500 ms, unspecified end velocity, nothing after it.
    REQUIRE(e.submit(knotFromSegment(0.125f, 500 * kMs, false, 0.0f, 0, Family::C2), 0));
    const auto s = sweep(e, 0, 1200 * kMs);
    float peak = 0.0f;
    for (const State& st : s) peak = std::max(peak, st.p);
    CHECK(peak == doctest::Approx(0.125f).epsilon(1e-4));
    CHECK(s.back().p == doctest::Approx(0.125f).epsilon(1e-4));
    CHECK(std::fabs(s[500].v) < 1e-4f);
    CHECK_FALSE(e.isBusy(600 * kMs));
    // The same segment with a successor queued behind it passes through moving.
    Engine<> e2(cfg, 0.0f);
    REQUIRE(e2.submit(knotFromSegment(0.125f, 500 * kMs, false, 0.0f, 0, Family::C2), 0));
    REQUIRE(e2.submit(knotFromSegment(0.25f, 500 * kMs, false, 0.0f, 500 * kMs, Family::C2), 0));
    CHECK(e2.stateAt(0, 500 * kMs).v > 0.05f);
    CHECK(e2.stateAt(0, 1000 * kMs).p == doctest::Approx(0.25f).epsilon(1e-4));
}

// ---- the segments flush (RFC-087, Nucleus val-dz9) ------------------------------

namespace {

// 250 ms of 50 ms C2 segments climbing 0.2 -> 0.3 from t = 0, the last resting.
void queueRamp(Engine<>& e) {
    for (int k = 0; k < 5; ++k)
        REQUIRE(e.submit(knotFromSegment(0.2f + 0.02f * float(k + 1), 50 * kMs, k < 4, 0.4f, uint64_t(k) * 50 * kMs,
                                         Family::C2), 0));
}

float worstJump(const std::vector<State>& s) {
    float worst = 0.0f;
    for (size_t i = 1; i < s.size(); ++i) {
        const float allowed = std::max(std::fabs(s[i].v), std::fabs(s[i - 1].v)) * 1e-3f + 2e-4f;
        worst = std::max(worst, std::fabs(s[i].p - s[i - 1].p) - allowed);
    }
    return worst;
}

}  // namespace

TEST_CASE("truncateAfter: a flush 40 ms out drops the queue from there and hands off continuously") {
    Config cfg; cfg.limits = {4.0f, 60.0f, 2000.0f};
    for (const Policy pol : {Policy::Blend, Policy::Stretch}) {
        cfg.policy = pol;
        Engine<> e(cfg, 0.2f), twin(cfg, 0.2f);
        queueRamp(e); queueRamp(twin);
        std::vector<State> s = sweep(e, 0, 110 * kMs);
        (void)sweep(twin, 0, 110 * kMs);
        // The seek at 110 ms: every segment starting at or after 150 ms is replaced
        // by one segment back down; the knot AT 150 ms ends the segment that
        // started at 100 ms and stays (RFC-087): it is the hand-off.
        const uint64_t now = 110 * kMs, t_base = 150 * kMs;
        CHECK(e.truncateAfter(t_base, now) == 2);                   // the knots at 200 and 250 ms
        CHECK(e.newest().t_us == t_base);                           // the author's knot at 150 ms
        CHECK(e.newest().family == Family::C2);
        const float v_base = e.solved(0, e.pending() - 1).v;        // its solved junction velocity
        const size_t kept = e.pending();
        REQUIRE(e.submit(knotFromSegment(0.15f, 300 * kMs, false, 0.0f, t_base, Family::C2), now));
        REQUIRE(e.pending() == kept + 1);
        for (size_t i = 0; i < e.pending(); ++i) {
            const uint64_t t = e.solved(0, i).t_us;
            CHECK(t != 200 * kMs); CHECK(t != 250 * kMs);
        }
        for (uint64_t t = now + kMs; t <= 900 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
        // The curve in flight runs to the new first start: there it is where
        // the queued plan would have been.
        const State old_at_base = twin.stateAt(0, t_base);
        MESSAGE("policy " << int(pol) << " hand-off p " << s[150].p << " vs queued " << old_at_base.p << ", v " << s[150].v
                          << " vs " << old_at_base.v << ", worst jump " << worstJump(s) << ", end " << s.back().p);
        CHECK(s[150].p == doctest::Approx(old_at_base.p).epsilon(1e-4));
        CHECK(s[150].v == doctest::Approx(v_base).epsilon(1e-3));
        CHECK(s[150].v == doctest::Approx(old_at_base.v).epsilon(2e-2));
        CHECK(worstJump(s) <= 0.0f);
        const Peaks pk = peaksOf(s);
        CHECK(pk.v <= cfg.limits.vmax * 1.001f);
        CHECK(pk.a <= cfg.limits.amax * 1.001f);
        CHECK(pk.j <= cfg.limits.jmax * 1.10f);
        CHECK(s.back().p == doctest::Approx(0.15f).epsilon(1e-4));
        CHECK(std::fabs(s.back().v) < 1e-4f);
        const auto an = drain(e);
        CHECK(countKind(an, AnomalyKind::KnotRefused) == 0);
        CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
    }
}

TEST_CASE("truncateAfter: a flush at now drops the whole window and hands off at the reaction horizon") {
    Config cfg; cfg.limits = {4.0f, 60.0f, 2000.0f};
    Engine<> e(cfg, 0.2f), twin(cfg, 0.2f);
    queueRamp(e); queueRamp(twin);
    std::vector<State> s = sweep(e, 0, 110 * kMs);
    (void)sweep(twin, 0, 110 * kMs);
    const uint64_t now = 110 * kMs;
    CHECK(e.truncateAfter(now, now) == 3);
    CHECK(e.pending() == 0);
    CHECK(e.newest().t_us == now + cfg.react_us);
    REQUIRE(e.submit(knotFromSegment(0.15f, 300 * kMs, false, 0.0f, now, Family::C2), now));
    for (uint64_t t = now + kMs; t <= 900 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
    // Through the horizon the curve is the one that was rendering.
    for (uint64_t t = now; t <= now + cfg.react_us; t += kMs)
        CHECK(s[t / kMs].p == twin.stateAt(0, t).p);
    CHECK(worstJump(s) <= 0.0f);
    CHECK(s.back().p == doctest::Approx(0.15f).epsilon(1e-4));
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::KnotRefused) == 0);
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
    // Nothing at or after the time: nothing changes.
    Engine<> idle(cfg, 0.2f);
    queueRamp(idle);
    CHECK(idle.truncateAfter(300 * kMs, 0) == 0);
    CHECK(idle.pending() == 5);
}

// ---- the solve budget (kin-ys0) -------------------------------------------------

TEST_CASE("solve budget: a bundle past it is solved over later ticks, never dropped, and renders as unbounded") {
    // An RFC-087 bundle: 32 segments in one submit burst, most past the
    // ceilings, solved at the next sample. The default budget cannot solve
    // them in one tick; the knots it does not reach wait, and the motion is
    // the unbounded engine's, bit for bit.
    Config cfg; cfg.limits = {2.0f, 100.0f, 10000.0f}; cfg.policy = Policy::Blend;
    REQUIRE(cfg.solve_budget > 0);
    Config unb = cfg; unb.solve_budget = 0;
    Engine<> e(cfg, 0.5f), ref(unb, 0.5f);
    uint64_t at = 0;
    for (int i = 0; i < 32; ++i) {
        at += (i % 3 == 0 ? 90 : 140) * kMs;
        const float p = 0.5f + (i % 2 ? -1.0f : 1.0f) * (0.1f + 0.012f * float(i));
        const Knot k = knotFromSegment(p, uint32_t(at), true, 0.0f, 0, Family::C2);
        REQUIRE(e.submit(k, 0));
        REQUIRE(ref.submit(k, 0));
    }
    std::vector<State> s, r;
    std::vector<Anomaly> an;
    for (uint64_t t = 0; t <= at + 500 * kMs; t += kMs) {
        s.push_back(e.stateAt(0, t));
        r.push_back(ref.stateAt(0, t));
        Anomaly a;
        while (e.popAnomaly(a)) an.push_back(a);
    }
    float last = 0.0f;
    const int failed = countKind(an, AnomalyKind::PlanFailed, &last);
    CHECK(failed == 0);
    CHECK(countKind(an, AnomalyKind::WaveformScaled) > 0);
    size_t differ = 0;
    for (size_t i = 0; i < s.size(); ++i) if (std::memcmp(&s[i], &r[i], sizeof(State)) != 0) ++differ;
    CHECK(differ == 0);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= cfg.limits.vmax * 1.001f);
    CHECK(pk.a <= cfg.limits.amax * 1.001f);
    CHECK(pk.j <= cfg.limits.jmax * 1.10f);
}

// ---- the HARD knot as a point-to-point profile (kin-hnp) -------------------------

namespace {

// A live jog on the 84 mm jog window: 200 mm/s, 200 mm/s^2, 5e6 mm/s^3.
const Limits kJog{200.0f / 84.0f, 200.0f / 84.0f, 5.0e6f / 84.0f};

// The profile sampled on the 1 ms grid, its end included exactly.
std::vector<State> sampled(const Profile& pr) {
    std::vector<State> s;
    for (float t = 0.0f; t < pr.duration(); t += 1e-3f) s.push_back(pr.atSeconds(t));
    s.push_back(pr.atSeconds(pr.duration()));
    return s;
}

}  // namespace

TEST_CASE("a HARD knot from a carriage moving away turns at once and lands at rest on it (the live jog)") {
    // Nucleus "jog is live": the carriage runs away at 0.705 and the jog
    // redirects 0.323 behind it.
    const State s0{0.3946f, 0.705f, -1.13f};
    const float target = 0.3946f - 0.323f;
    float fastest = 0.0f;
    const Profile pr = Profile::point(s0, target, 0, kJog, 0.0f, &fastest);
    REQUIRE(pr.n > 0);
    for (int i = 0; i < pr.n; ++i) CHECK(pr.dt[i] >= 0.0f);
    CHECK(pr.duration() == doctest::Approx(fastest).epsilon(1e-4));
    const auto s = sampled(pr);
    // It reverses within the stop from 0.705 at amax plus the jerk ramp, and
    // never turns back away once it has.
    size_t rev = s.size();
    for (size_t i = 0; i < s.size(); ++i) if (s[i].v < 0.0f) { rev = i; break; }
    REQUIRE(rev < s.size());
    CHECK(float(rev) * 1e-3f <= s0.v / kJog.amax + kJog.amax / kJog.jmax + 1e-3f);
    for (size_t i = rev; i < s.size(); ++i) CHECK(s[i].v <= 1e-6f);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= kJog.vmax * 1.001f);
    CHECK(pk.a <= kJog.amax * 1.001f);
    CHECK(pk.j <= kJog.jmax * 1.10f);
    CHECK(s.back().p == doctest::Approx(target).epsilon(1e-5));
    CHECK(std::fabs(s.back().v) <= 1e-5f);
    CHECK(std::fabs(s.back().a) <= 1e-4f);

    // Through the solver: a jog sample whose estimated deadline is too early
    // takes the profile's end as its time, unreported, rendered as that
    // profile.
    Knot k = knotFromSample(target, 1000000, 400000);
    k.has_v = true; k.family = Family::C1;
    Config cfg; cfg.limits = kJog; cfg.policy = Policy::Stretch;
    Solved out[1];
    jerk::Workspace ws{};
    int reports = 0;
    solveWindow(s0, 1000000, &k, 1, cfg, out, [&](AnomalyKind, size_t, uint64_t, float, float) { ++reports; }, ws);
    CHECK(out[0].hard);
    CHECK_FALSE(out[0].dropped);
    CHECK(out[0].t_us == Profile::point(s0, target, 1000000, kJog).end_us());
    CHECK(out[0].t_us > k.t_us);
    CHECK(reports == 0);
}

TEST_CASE("a HARD knot ahead of a carriage moving toward it never slows before its brake") {
    const State s0{0.2f, 0.786f, 0.0f};
    const float target = 0.2f + 0.56f;
    const Profile pr = Profile::point(s0, target, 0, kJog);
    REQUIRE(pr.n > 0);
    const auto s = sampled(pr);
    // One deceleration, at the end: from the first sample that decelerates,
    // the velocity only falls, and what is left is the stop from the peak.
    size_t dec = s.size();
    for (size_t i = 0; i < s.size(); ++i) if (s[i].a < -1e-3f * kJog.amax) { dec = i; break; }
    REQUIRE(dec < s.size());
    float vpk = 0.0f;
    for (size_t i = 0; i < dec; ++i) { CHECK(s[i].a >= -1e-3f * kJog.amax); vpk = std::fmax(vpk, s[i].v); }
    for (size_t i = dec + 1; i < s.size(); ++i) CHECK(s[i].v <= s[i - 1].v + 1e-6f);
    const float stop = Profile::brake(State{0.0f, vpk, 0.0f}, 0, kJog).duration();
    CHECK(pr.duration() - float(dec) * 1e-3f <= stop + 2e-3f);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= kJog.vmax * 1.001f);
    CHECK(pk.a <= kJog.amax * 1.001f);
    CHECK(s.back().p == doctest::Approx(target).epsilon(1e-5));
    CHECK(std::fabs(s.back().v) <= 1e-5f);
}

TEST_CASE("a HARD knot with time to spare: from rest it holds then launches, moving it cruises slower, landing on time") {
    const float target = 0.7f;
    // From rest: the hold comes first and the move lands exactly at 1.2 s.
    const Profile still = Profile::point(State{0.3f, 0.0f, 0.0f}, target, 0, kJog, 1.2f);
    REQUIRE(still.n > 1);
    CHECK(still.jerk[0] == 0.0f);
    CHECK(still.duration() == doctest::Approx(1.2f).epsilon(1e-4));
    CHECK(still.atSeconds(still.dt[0]).p == 0.3f);
    // Moving toward it: never faster than it is going, on time, at rest.
    const State s0{0.3f, 0.4f, 0.0f};
    const Profile moving = Profile::point(s0, target, 0, kJog, 1.2f);
    REQUIRE(moving.n > 0);
    CHECK(moving.duration() == doctest::Approx(1.2f).epsilon(1e-3));
    const auto s = sampled(moving);
    for (const State& x : s) CHECK(x.v <= s0.v + 1e-5f);
    CHECK(s.back().p == doctest::Approx(target).epsilon(1e-5));
    // Past it: it stops beyond it, then comes back.
    const Profile past = Profile::point(State{0.69f, 2.0f, 0.0f}, target, 0, kJog);
    REQUIRE(past.n > 0);
    const auto sp = sampled(past);
    float pmax = 0.0f;
    for (const State& x : sp) pmax = std::fmax(pmax, x.p);
    CHECK(pmax > target);
    CHECK(sp.back().p == doctest::Approx(target).epsilon(1e-5));
    CHECK(peaksOf(sp).a <= kJog.amax * 1.001f);
}

TEST_CASE("a live jog redirected mid-move replaces the move in flight and turns at once") {
    // As Nucleus sends a jog: flush at now (the hand-off is the reaction
    // horizon), then a HARD sample due as soon as possible.
    Config cfg; cfg.limits = kJog; cfg.policy = Policy::Stretch;
    Engine<> e(cfg, 0.5f);
    auto jog = [&](float target, uint64_t now) {
        (void)e.truncateAfter(now, now);
        const Knot h = e.newest();
        Knot k = knotFromSample(target, h.t_us > now ? h.t_us : now, 1000);
        k.has_v = true; k.family = Family::C1;
        REQUIRE(e.submit(k, now));
    };
    std::vector<State> s;
    jog(0.95f, 0);
    for (uint64_t t = 0; t < 300 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
    const float v_turn = s.back().v;
    REQUIRE(v_turn > 0.25f * kJog.vmax);   // still accelerating away
    jog(0.3f, 300 * kMs);
    CHECK(e.pending() == 1);   // the move in flight was replaced, never queued behind
    for (uint64_t t = 300 * kMs; t <= 2000 * kMs; t += kMs) s.push_back(e.stateAt(0, t));
    size_t rev = s.size();
    for (size_t i = 300; i < s.size(); ++i) if (s[i].v < 0.0f) { rev = i; break; }
    REQUIRE(rev < s.size());
    // The curve runs on, still accelerating, through the reaction horizon;
    // from there the stop at amax after the jerk ramp from +amax to -amax,
    // to the 1 ms grid.
    const float react = float(cfg.react_us) * 1e-6f, v_h = v_turn + kJog.amax * react;
    const float bound = react + v_h / kJog.amax + 2.0f * kJog.amax / kJog.jmax + 3e-3f;
    CHECK(float(rev - 300) * 1e-3f <= bound);
    CHECK(worstJump(s) <= 0.0f);
    const Peaks pk = peaksOf(s);
    CHECK(pk.v <= kJog.vmax * 1.001f);
    CHECK(pk.a <= kJog.amax * 1.001f);
    CHECK(s.back().p == doctest::Approx(0.3f).epsilon(1e-4));
    CHECK(std::fabs(s.back().v) < 1e-4f);
    const auto an = drain(e);
    CHECK(countKind(an, AnomalyKind::KnotRefused) == 0);
    CHECK(countKind(an, AnomalyKind::PlanFailed) == 0);
    CHECK(countKind(an, AnomalyKind::DeadlineStretched) == 0);
}

// ---- a C1 script renders as its author's cubics (kin-7jd) --------------------------

namespace {

struct ScriptRun {
    double v_err_max = 0.0, v_err_first = 0.0, v_peak = 0.0;
    int dips = 0, refused = 0, stretched = 0, trimmed = 0, failed = 0;
};

// A 1.3 s sine of 0.3 of the window as Phosphor sends a funscript: a segment
// per 100 ms knot with the PCHIP slope as its end velocity, each submitted
// 250 ms before its start, under the 84 mm rig's ceilings. The rendered
// velocity is compared on the 1 ms grid with the derivative of the script's
// own PCHIP curve; v_err_max is taken after the first span (the carriage
// starts at rest with no acceleration, the first cubic does not).
ScriptRun runPchipSine(Family fam, Corner corner) {
    Config cfg; cfg.limits = {1000.0f / 84.0f, 50000.0f / 84.0f, 1.0e7f / 84.0f}; cfg.corner = corner;
    Engine<> e(cfg, 0.5f);
    const double period = 1300.0, step = 100.0, amp = 0.3;
    const int n = int(5 * period / step);
    std::vector<double> t(n), p(n), m(n, 0.0);
    for (int i = 0; i < n; ++i) { t[i] = i * step; p[i] = 0.5 + amp * std::sin(6.283185307 * t[i] / period); }
    for (int i = 1; i + 1 < n; ++i) {
        const double a = (p[i] - p[i - 1]) / step, b = (p[i + 1] - p[i]) / step;
        if (a * b <= 0.0) continue;
        m[i] = 2.0 / (1.0 / a + 1.0 / b);   // Fritsch-Carlson on equal spacing (per ms)
    }
    auto pchipV = [&](double ms) {   // window units per second
        for (int i = 1; i < n; ++i) {
            if (ms > t[i]) continue;
            const double h = t[i] - t[i - 1], s = (ms - t[i - 1]) / h;
            const double d00 = 6 * s * s - 6 * s, d10 = 3 * s * s - 4 * s + 1, d01 = -6 * s * s + 6 * s, d11 = 3 * s * s - 2 * s;
            return (d00 * p[i - 1] + d10 * h * m[i - 1] + d01 * p[i] + d11 * h * m[i]) / h * 1000.0;
        }
        return 0.0;
    };
    ScriptRun r;
    std::vector<float> v;
    int next = 1;
    const uint64_t end = uint64_t(t[n - 1]) * kMs;
    for (uint64_t now = 0; now <= end; now += kMs) {
        while (next < n && uint64_t(t[next - 1]) * kMs <= now + 250 * kMs) {
            const Knot k = knotFromSegment(float(p[next]), uint32_t(step) * kMs, true, float(m[next] * 1000.0),
                                           uint64_t(t[next - 1]) * kMs, fam);
            if (!e.submit(k, now)) ++r.refused;
            ++next;
        }
        v.push_back(e.stateAt(0, now).v);
        Anomaly a;
        while (e.popAnomaly(a)) {
            if (a.kind == uint8_t(AnomalyKind::DeadlineStretched)) ++r.stretched;
            if (a.kind == uint8_t(AnomalyKind::WaveformScaled)) ++r.trimmed;
            if (a.kind == uint8_t(AnomalyKind::PlanFailed)) ++r.failed;
        }
    }
    for (size_t i = 0; i < v.size(); ++i) {
        const double err = std::fabs(v[i] - pchipV(double(i)));
        r.v_peak = std::fmax(r.v_peak, std::fabs(pchipV(double(i))));
        if (i < size_t(step)) r.v_err_first = std::fmax(r.v_err_first, err);
        else r.v_err_max = std::fmax(r.v_err_max, err);
    }
    // A dip: inside a run of one sign, |v| falls and rises again by more
    // than 2 percent of where it was.
    for (size_t i = 2; i + 2 < v.size(); ++i) {
        const float a = v[i - 2], b = v[i], c = v[i + 2];
        if (a * c <= 0.0f || a * b <= 0.0f) continue;
        if (std::fabs(b) < 0.98f * std::fabs(a) && std::fabs(c) > std::fabs(b) + 0.02f * std::fabs(a)) ++r.dips;
    }
    return r;
}

}  // namespace

TEST_CASE("a C1 PCHIP script with the cubic corner renders as its author's curve; C2 compromises at every knot") {
    const ScriptRun c1 = runPchipSine(Family::C1, Corner::Cubic);
    const ScriptRun c2 = runPchipSine(Family::C2, Corner::Continuous);
    MESSAGE("C1 cubic: v error " << c1.v_err_max << " (" << 100.0 * c1.v_err_max / c1.v_peak << " % of peak " << c1.v_peak
            << "), first span " << c1.v_err_first << "; C2 continuous: v error " << c2.v_err_max << " ("
            << 100.0 * c2.v_err_max / c2.v_peak << " %), first span " << c2.v_err_first);
    CHECK(c1.v_err_max <= 0.02 * c1.v_peak);
    CHECK(c1.dips == 0);
    CHECK(c1.refused == 0);
    CHECK(c1.stretched == 0);
    CHECK(c1.trimmed == 0);
    CHECK(c1.failed == 0);
    // The sender asking for C2 gets the chain's compromise: allowed to wobble,
    // and an order of magnitude off the author's curve.
    CHECK(c2.v_err_max > 10.0 * c1.v_err_max);
}

// RFC-087: a bundle that begins exactly where the queue ends replaces nothing;
// the knot at its first start is the end of a segment that started before it.
// Dropping it stood a C2 hand-off knot in its place on every bundle, and a C1
// script lost the author's corner at every knot (bench, 2026-10-06).
TEST_CASE("truncateAfter keeps the knot at its time: a bundle starting where the queue ends changes nothing") {
    Config cfg; cfg.limits = {3.0f, 30.0f, 2000.0f};
    Engine<> e(cfg, 0.2f);
    REQUIRE(e.submit(knotFromSegment(0.4f, 200000, true, 0.5f, 0, Family::C1), 0));
    REQUIRE(e.submit(knotFromSegment(0.6f, 200000, true, 0.5f, 200000, Family::C1), 0));
    (void)e.stateAt(0, 1000);
    CHECK(e.pending(0) == 2);
    CHECK(e.truncateAfter(400000, 1000) == 0);   // the queue ends at 400 ms: nothing after it
    CHECK(e.pending(0) == 2);
    CHECK(e.newest().t_us == 400000);
    CHECK(e.newest().family == Family::C1);
    CHECK(e.truncateAfter(200000, 1000) == 1);   // after 200 ms: the second span goes, the first knot stays
    CHECK(e.pending(0) == 1);
    CHECK(e.newest().t_us == 200000);
    CHECK(e.newest().family == Family::C1);
    REQUIRE(e.submit(knotFromSegment(0.7f, 200000, true, 0.0f, 200000, Family::C1), 1000));
    CHECK(e.truncateAfter(300000, 1000) == 1);   // inside the span: the hand-off knot stands at 300 ms
    CHECK(e.newest().t_us == 300000);
    CHECK(e.newest().has_v);
}
