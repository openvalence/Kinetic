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
#define KINETIC2_FINGERPRINT 0xaf187335dc85dfa1ull   // accepted 2026-10-05: solver with drops, relaxation, end rest
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
    CHECK(s.back().v == 0.0f);
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
    REQUIRE(e.submit(0, knotAt(200 * kMs, 0.9f), 0));
    REQUIRE(e.submit(1, knotAt(300 * kMs, 0.1f), 0));
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

TEST_CASE("free knots never overshoot between two knots") {
    Config cfg; cfg.limits = {10.0f, 400.0f, 50000.0f};
    Engine<> e(cfg, 0.1f);
    // A staircase: monotone rising steps then a plateau. No dip, no bulge.
    const float ps[] = {0.3f, 0.5f, 0.52f, 0.9f, 0.9f, 0.9f};
    for (int i = 0; i < 6; ++i) REQUIRE(e.submit(knotAt(uint64_t(i + 1) * 100 * kMs, ps[i]), 0));
    const auto s = sweep(e, 0, 600 * kMs);
    for (size_t i = 1; i < s.size(); ++i) CHECK(s[i].p >= s[i - 1].p - 1e-4f);   // monotone rise
    CHECK(peaksOf(s).hi <= 0.9f + 1e-4f);
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
        REQUIRE(e.submit(knotAt(500 * kMs, 0.8f, true, 0.0f, f), 0));
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
    CHECK(s.back().v == 0.0f);
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
    CHECK(junctionOf(knotFromSegment(0.7f, 250 * kMs, true, 0.0f, 0, Family::C1)) == Junction::Hard);
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
