// test_kinetic — doctest suite for the Kinetic engine (ctest target).
//
// Hardware-free, fully deterministic: time is a synthetic uint64 microsecond
// counter, no clocks, no randomness. Every kinematic assertion is checked by
// SAMPLING the produced trajectory on a 1 ms grid (the same cadence the
// firmware's stream sampler uses) — limits are verified as sampled reality,
// not trusted from Ruckig's promises.
//
// Finite-difference tolerances: velocity/accel come from Ruckig analytically,
// but jerk is checked as Δa/Δt on the 1 ms grid, which averages across the
// bang-bang jerk switching instants — so the jerk bound uses a small margin.
//
// THE MACHINE IS THE FIXTURE: liveTuning() below is the default for every
// scenario case. The three softer fixtures are alternatives, each used only by
// cases that are ABOUT the alternative and each carrying the one-line reason.
//
// Field figures are GENERATED, never transcribed. Nucleus's trace tool:
//   python tools/segtrace_to_case.py --dir artifacts/segtrace-<run>
//          --from <s> --to <s> --name <id>
// prints the fieldreplay::S[] table and the Config that produced it.
// ============================================================================

// Debug scaffolding for a bench session; every printf in this file is behind it
// and the suite is silent at 0. Build with -DKINETIC_TEST_VERBOSE=1.
#ifndef KINETIC_TEST_VERBOSE
#define KINETIC_TEST_VERBOSE 0
#endif

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "kinetic/kinetic.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using kinetic::AnomalyType;
using kinetic::Command;
using kinetic::Config;
using kinetic::Engine;
using kinetic::InfeasiblePolicy;
using kinetic::Mode;

namespace {

constexpr uint64_t kMs = 1000ULL;        // µs per ms
constexpr uint64_t kS  = 1000000ULL;     // µs per s

// The Valence registry's `segment_dwell_span` (RFC-058), the value Nucleus
// injects. This library stays protocol-free, so the number is pinned here.
constexpr float kSegmentDwellSpan = 0.02f;

// ---- THE MACHINE IS THE FIXTURE ---------------------------------------------
// The tuning the machine actually runs, read off channel 0x1122 on 2026-09-02:
// a 100 mm window (87-187 mm), 1000 mm/s / 40000 mm/s^2 / 5e6 mm/s^3 of input
// ceiling, a 200 mm/s user ceiling, Blend at 0.5 with both budgets at 0.5 over
// 6 ray steps, c1_cubic segments, 200 ms of settle grace, chase gain 0.1 with
// no lookahead, and the overshoot guard left at the engine default.
//
// PINNED AGAINST THE NUCLEUS DEVICE CATALOG. The fields here that are catalog
// DEFAULTS (infeasible_blend, both budgets, blend steps, and the untouched
// handoff_chord_factor and overshoot_guard) have their one home in the Nucleus
// firmware's motion tuning defaults. A default that moves there and not here
// silently re-targets this whole suite, so a change touches BOTH.
// The rest are the operator's session values, not defaults: Blend against
// the catalog's Stretch, 200 ms against 30, gain 0.1 against 0.9, lookahead 0
// against 3.
//
// curve_policy stays FollowClient because the client declares its family per
// segment (fam=1, c1_cubic, on the wire); the replay rows carry that.
Config liveTuning() {
    Config cfg;
    cfg.limits.vmax = 10.0f;       // 1000 mm/s   / 100 mm
    cfg.limits.amax = 400.0f;      // 40000 mm/s2 / 100 mm
    cfg.limits.jmax = 50000.0f;    // 5e6 mm/s3   / 100 mm
    cfg.recovery_vmax   = 2.0f;    // user 200 mm/s / 100 mm
    cfg.infeasible_policy           = InfeasiblePolicy::Blend;
    cfg.infeasible_blend            = 0.5f;   // sm_tune_infeas_blend
    cfg.infeasible_smooth_budget    = 0.5f;   // sm_tune_smooth_budget
    cfg.infeasible_amplitude_budget = 0.5f;   // sm_tune_amp_budget
    cfg.infeasible_blend_steps      = 6;      // sm_tune_blend_steps
    cfg.settle_grace_us = 200000;  // sm_tune_settle_grace_us, operator-set
    cfg.chase_ff_gain   = 0.1f;    // sm_tune_chase_gain, operator-set
    cfg.chase_lookahead = 0.0f;    // sm_tune_chase_look, operator-set
    return cfg;
}

// ALTERNATIVE FIXTURE, soft ceilings (2/20/300). Only for cases whose assertion
// needs a ceiling the command can actually break, or a jerk bound a 1 ms finite
// difference can resolve; at the fixture's 50000 those assertions are vacuous.
Config testConfig() {
    Config cfg;
    cfg.limits.vmax = 2.0f;
    cfg.limits.amax = 20.0f;
    cfg.limits.jmax = 300.0f;
    // PINNED, not inherited: a fixture must not depend on a default it is not
    // the test of, or it re-targets itself the day that default moves. The
    // policy comparisons below set their own policy explicitly.
    cfg.infeasible_policy = InfeasiblePolicy::Blend;
    // Same rule, second default: every number below predates the overshoot
    // guard, and the guard changes which shapes are legal at all. Tests that ARE
    // about the guard arm it themselves.
    cfg.overshoot_guard = 0.0f;
    return cfg;
}

// ALTERNATIVE FIXTURE. The limit set of the virtual machine both defects were
// measured against, kept because the M7a and referee cases are ABOUT a jerk
// ceiling too low to turn the boundary state around:
// 500 mm stroke window, 550 mm/s input speed, 8000 mm/s² input accel, fixed
// jerk — normalized by the window span exactly as the firmware glue does.
Config machineConfig() {
    Config cfg;
    cfg.limits.vmax = 1.1f;    // 550 mm/s  / 500 mm
    cfg.limits.amax = 16.0f;   // 8000 mm/s² / 500 mm
    cfg.limits.jmax = 500.0f;  // firmware default (SystemState::sm_tune_jmax_ovr)
    return cfg;
}

// ALTERNATIVE FIXTURE. An older operator machine; kSpanMm is the mm scale the
// amplitude complaints were made in and the shape/overshoot cases still quote.
// The OPERATOR's machine, measured: stroke window [150, 350] mm (span 200),
// input speed 1000 mm/s, input accel 50000 mm/s², jerk 2e6 mm/s³ — normalized
// by the span exactly as the firmware glue does. Amplitude numbers below are
// quoted in millimeters against this window, because that is the domain the
// amplitude complaint was made in.
constexpr double kSpanMm = 200.0;
Config operatorConfig() {
    Config cfg;
    cfg.limits.vmax = 5.0f;       // 1000 mm/s  / 200 mm
    cfg.limits.amax = 250.0f;     // 50000 mm/s² / 200 mm
    cfg.limits.jmax = 10000.0f;   // 2e6 mm/s³   / 200 mm
    cfg.overshoot_guard = 0.0f;   // pinned — see testConfig()
    return cfg;
}

// Drain the ring, reporting whether a given anomaly kind appeared and the
// detail/target of its LAST occurrence.
struct AnomalyHit {
    bool  seen   = false;
    float detail = 0.0f;
    float target = 0.0f;
    int   count  = 0;
};

AnomalyHit drainFor(Engine& e, AnomalyType kind) {
    AnomalyHit h;
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)kind) {
            h.seen = true;
            h.detail = ev.detail;
            h.target = ev.target;
            h.count++;
        }
    }
    return h;
}

struct SweepStats {
    double max_abs_v = 0.0;
    double max_abs_a = 0.0;
    double max_abs_j = 0.0;   // finite-difference of accel on the grid
    double min_p = 1e9, max_p = -1e9;
    // The same excursion on the UNCLAMPED state. Asserting a window on
    // positionAt asserts on the clamp itself and can never fail; these are
    // what a plan leaving [0,1] actually shows up in.
    double min_raw = 1e9, max_raw = -1e9;
    double max_dv = 0.0;      // largest velocity step between adjacent samples
};

// Sample [t0, t1] on a 1 ms grid and accumulate kinematic extremes.
SweepStats sweep(Engine& e, uint64_t t0_us, uint64_t t1_us) {
    SweepStats s;
    double prev_v = 0.0, prev_a = 0.0;
    bool first = true;
    for (uint64_t t = t0_us; t <= t1_us; t += kMs) {
        const double p = e.positionAt(t);
        const double v = e.velocityAt(t);
        const double a = e.accelerationAt(t);
        REQUIRE(std::isfinite(p));
        REQUIRE(std::isfinite(v));
        REQUIRE(std::isfinite(a));
        s.max_abs_v = std::max(s.max_abs_v, std::fabs(v));
        s.max_abs_a = std::max(s.max_abs_a, std::fabs(a));
        s.min_p = std::min(s.min_p, p);
        s.max_p = std::max(s.max_p, p);
        double rp, rv, ra;
        e.rawSampleAt(t, rp, rv, ra);
        s.min_raw = std::min(s.min_raw, rp);
        s.max_raw = std::max(s.max_raw, rp);
        if (!first) {
            s.max_abs_j = std::max(s.max_abs_j, std::fabs(a - prev_a) / 1e-3);
            s.max_dv    = std::max(s.max_dv, std::fabs(v - prev_v));
        }
        prev_v = v; prev_a = a; first = false;
    }
    return s;
}

// ---- Single-segment SHAPE measurement (the 0.6.0 sharpness work) ------------
// Everything the operator asked to see for one timed segment: what it actually
// delivered, and how straight the line was while it delivered it. Sampled on a
// 0.2 ms grid strictly INSIDE the plan (positionAt runs maybeSettle, and a
// sample past the deadline would measure the brake, not the segment).
struct Shape {
    double travel_mm  = 0.0;
    double sharp      = 1.0;    // Snapshot::sharpness = peak jerk / jmax
    double vpk        = 0.0;    // normalized units/s
    double apk        = 0.0;
    double jpk        = 0.0;    // finite-difference on the sample grid
    double pk_over_mean = 0.0;  // 1.0 = square (flat), 1.875 = min-jerk quintic
    double flat_pct   = 0.0;    // % of the segment within 2 % of its own vpk
    double duration_s = 0.0;
    uint8_t kind      = 0;      // PlanKind
    uint32_t failures = 0;      // PlanFailed count
};

Shape segShape(Config cfg, double start, double target, uint32_t ms,
               bool has_ev = false, float ev = 0.0f) {
    Engine e(cfg, (float)start);
    Command c;
    c.target       = (float)target;
    c.duration_us  = ms * (uint32_t)kMs;
    c.has_duration = true;
    c.end_vel      = ev;
    c.has_end_vel  = has_ev;
    REQUIRE(e.commit(c, 0));

    Shape s;
    s.sharp      = e.snapshot(0).sharpness;
    s.duration_s = e.snapshot(0).duration_s;
    s.failures   = e.snapshot(0).failures;
    s.kind       = (uint8_t)e.planKind();

    const uint64_t end = (uint64_t)ms * kMs;
    std::vector<double> vs;
    double prev_a = 0.0; bool first = true;
    for (uint64_t t = 0; t < end; t += 200) {
        const double v = std::fabs((double)e.velocityAt(t));
        const double a = (double)e.accelerationAt(t);
        s.vpk = std::max(s.vpk, v);
        s.apk = std::max(s.apk, std::fabs(a));
        if (!first) s.jpk = std::max(s.jpk, std::fabs(a - prev_a) / 2e-4);
        prev_a = a; first = false;
        vs.push_back(v);
    }
    s.travel_mm = std::fabs((double)e.positionAt(end) - start) * kSpanMm;
    const double mean = (s.travel_mm / kSpanMm) / ((double)ms * 1e-3);
    s.pk_over_mean = mean > 1e-9 ? s.vpk / mean : 0.0;
    int flat = 0;
    for (double v : vs) if (v >= 0.98 * s.vpk) flat++;
    s.flat_pct = 100.0 * flat / (double)vs.size();
    return s;
}

} // namespace

// ============================================================================
TEST_CASE("Idle: fresh engine holds its seed position, not busy") {
    Engine e(liveTuning(), 0.3f);
    CHECK(e.positionAt(0) == doctest::Approx(0.3f));
    CHECK(e.velocityAt(5 * kS) == doctest::Approx(0.0));
    CHECK_FALSE(e.isBusy(0));
    CHECK(e.mode() == Mode::Idle);
}

TEST_CASE("Waveform rest-to-rest: lands on target, at rest, on the deadline") {
    Engine e(liveTuning(), 0.2f);

    Command c;
    c.target       = 0.8f;
    c.duration_us  = 600 * (uint32_t)kMs;
    c.has_duration = true;
    const uint64_t t0 = 1 * kS;
    REQUIRE(e.commit(c, t0));
    CHECK(e.mode() == Mode::Waveform);
    CHECK(e.planKind() == kinetic::PlanKind::Quintic);
    CHECK(e.isBusy(t0 + 10 * kMs));

    auto snap = e.snapshot(t0);
    // A quintic segment spans exactly the commanded duration.
    CHECK(snap.duration_s == doctest::Approx(0.6).epsilon(0.01));

    // Landed: position, rest, not busy.
    const uint64_t tEnd = t0 + 700 * kMs;
    CHECK(e.positionAt(tEnd) == doctest::Approx(0.8).epsilon(1e-4));
    CHECK(e.velocityAt(tEnd) == doctest::Approx(0.0).epsilon(1e-4));
    CHECK_FALSE(e.isBusy(tEnd));

    // No anomalies for a clean feasible move.
    kinetic::Anomaly ev;
    CHECK_FALSE(e.popAnomaly(ev));
}

TEST_CASE("Waveform quintic is the min-jerk curve (shape fidelity)") {
    // The whole point of the quintic path: reproduce the sender's spline, not
    // a cruise-and-burst chord. Rest-to-rest min-jerk has exact analytic
    // midpoint values: p(T/2) = midpoint, v(T/2) = 1.875 * dist / T.
    // NOT the machine fixture: this is about the QUINTIC family, which the
    // machine's clients never ask for (they declare c1_cubic).
    Engine e(testConfig(), 0.2f);
    Command c;
    c.target = 0.8f; c.duration_us = 600 * (uint32_t)kMs; c.has_duration = true;
    REQUIRE(e.commit(c, 0));
    REQUIRE(e.planKind() == kinetic::PlanKind::Quintic);

    CHECK(e.positionAt(300 * kMs) == doctest::Approx(0.5).epsilon(1e-4));
    CHECK(e.velocityAt(300 * kMs) ==
          doctest::Approx(1.875 * 0.6 / 0.6).epsilon(1e-3));
    // Acceleration at both ends is zero (C2 into a rest hold).
    CHECK(e.accelerationAt(0) == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(e.accelerationAt(600 * kMs) == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Consecutive G-slope segments join C2 (no accel jump at boundaries)") {
    // The legacy cubic was C1: acceleration JUMPED at every v4 boundary
    // (bench-measured ~8 u/s^2 discontinuities). The quintic chain replans
    // from its own sampled (p,v,a), so accel is continuous at joints — the
    // largest 1 ms accel step anywhere must be jerk-limited, not a jump.
    Config cfg;
    cfg.limits.vmax = 3.0f; cfg.limits.amax = 30.0f; cfg.limits.jmax = 500.0f;
    // NOT the machine fixture: a jerk-continuity bound is only a bound where a
    // 1 ms finite difference can resolve jmax. At 50000 it admits any jump.
    const double amp = 0.35, f = 0.5, base = 0.5;
    Engine e(cfg, (float)base);

    auto srcP = [&](double t){ return base + amp * std::sin(2*M_PI*f*t); };
    auto srcV = [&](double t){ return amp*2*M_PI*f * std::cos(2*M_PI*f*t); };

    const uint64_t seg = 500 * kMs;
    uint64_t next_cmd = 0;
    double prev_a = 0.0; bool have_prev = false;
    double max_da = 0.0;
    for (uint64_t t = 0; t <= 4 * kS; t += kMs) {
        if (t >= next_cmd) {
            const double te = (double)(next_cmd + seg) * 1e-6;
            Command c;
            c.target       = (float)srcP(te);
            c.duration_us  = (uint32_t)seg;
            c.has_duration = true;
            c.end_vel      = (float)srcV(te);
            c.has_end_vel  = true;
            REQUIRE(e.commit(c, next_cmd));
            REQUIRE(e.planKind() == kinetic::PlanKind::Quintic);
            next_cmd += seg;
        }
        const double a = e.accelerationAt(t);
        // skip the first (catch-up) segment: entry-state mismatch is real work
        if (have_prev && t > seg) max_da = std::max(max_da, std::fabs(a - prev_a));
        prev_a = a; have_prev = true;
    }
    // Jerk-limited step, never a discontinuity: da <= jmax * 1ms with margin.
    // (The cubic's boundary jumps were ~8.0 here — an order of magnitude out.)
    CHECK(max_da <= 500.0 * 1e-3 * 1.5 + 0.05);
}

TEST_CASE("Over-demanding waveform falls back to the Ruckig guard, ceilings hold") {
    // 0->1 in 900 ms wants a min-jerk peak velocity of 1.875/0.9 = 2.08 —
    // just over the 2.0 ceiling. The quintic must NOT be executed; the guard
    // takes the segment and every sampled ceiling still holds.
    //
    // Pinned to Stretch AND to a soft ceiling: this is the GUARD's test, and
    // the guard only takes a segment the command can actually break. Under the
    // machine's Blend the same command is shortened toward the amplitude floor.
    auto cfg = testConfig();
    cfg.infeasible_policy = InfeasiblePolicy::Stretch;
    Engine e(cfg, 0.0f);

    Command c;
    c.target       = 1.0f;
    c.duration_us  = 900 * (uint32_t)kMs;
    c.has_duration = true;
    REQUIRE(e.commit(c, 0));
    CHECK(e.planKind() == kinetic::PlanKind::Ruckig);

    kinetic::Anomaly ev;
    bool saw_fallback = false;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::WaveformFallback) saw_fallback = true;
    }
    CHECK(saw_fallback);

    auto s = sweep(e, 0, 1 * kS);
    CHECK(s.max_abs_v <= cfg.limits.vmax * 1.001);
    CHECK(s.max_abs_a <= cfg.limits.amax * 1.001);
    // Finite-difference jerk averages across switching instants — margin.
    CHECK(s.max_abs_j <= cfg.limits.jmax * 1.05 + 1.0);
    CHECK(s.min_p >= -1e-9);
    CHECK(s.max_p <= 1.0 + 1e-9);
}

TEST_CASE("Cold-start governor: opening plan out of rest is vmax-clamped, "
          "warm plans are not") {
    // velocityAt is a LIVE evaluator (maybeSettle advances state), so each
    // phase gets its own engine instance and one monotonic scan.
    auto cfg = liveTuning();    // recovery 2.0, well under limits.vmax (10)
    Command c;
    c.target = 0.9f; c.duration_us = 100 * (uint32_t)kMs; c.has_duration = true;

    // COLD: first commit from Idle at rest. Big infeasible ask; the governor
    // clamps vmax; the feasibility machinery absorbs the deadline.
    {
        Engine e(cfg, 0.1f);
        REQUIRE(e.commit(c, 0));
        double vpk = 0.0;
        for (uint64_t t = 0; t < 5 * kS; t += kMs)
            vpk = std::max(vpk, std::fabs((double)e.velocityAt(t)));
        CHECK(vpk <= 2.0 * 1.02);
        CHECK(vpk > 0.1);   // it does actually move
    }

    // WARM: identical engine, but the second commit lands MID-FLIGHT
    // (moving, so not cold) and plans at FULL limits.
    {
        Engine e(cfg, 0.1f);
        REQUIRE(e.commit(c, 0));
        uint64_t tMid = 0;
        for (uint64_t t = kMs; t < 5 * kS; t += kMs)
            if (std::fabs((double)e.velocityAt(t)) > 0.05) { tMid = t; break; }
        REQUIRE(tMid > 0);
        Command c2;
        c2.target = 0.9f; c2.duration_us = 300 * (uint32_t)kMs;
        c2.has_duration = true;
        REQUIRE(e.commit(c2, tMid));
        double vpk2 = 0.0;
        for (uint64_t t = tMid; t < tMid + 2 * kS; t += kMs)
            vpk2 = std::max(vpk2, std::fabs((double)e.velocityAt(t)));
        CHECK(vpk2 > 2.0 * 1.05);   // exceeded the recovery clamp = unclamped
    }
}

TEST_CASE("Cold-start governor: from-rest strokes inside a live stream are "
          "NOT clamped (cold needs a command gap)") {
    Engine e(liveTuning(), 0.2f);
    // Prime: one commit establishes the stream (cold, clamped -- fine).
    Command c;
    c.target = 0.4f; c.duration_us = 200 * (uint32_t)kMs; c.has_duration = true;
    REQUIRE(e.commit(c, 0));
    // A stroke 500 ms later, arriving from rest (explicit-rest content):
    // WARM by the gap rule, so it may exceed the recovery clamp.
    // 0.6 norm over 200 ms: min-jerk peak v = 5.6 -- inside every ceiling
    // (v 10, a 400, j 50000), well above the 2.0 recovery clamp.
    Command c2;
    c2.target = 1.0f; c2.duration_us = 200 * (uint32_t)kMs; c2.has_duration = true;
    const uint64_t t2 = 500 * kMs;
    REQUIRE(e.commit(c2, t2));
    double vpk = 0.0;
    for (uint64_t t = t2; t < t2 + 1500 * kMs; t += kMs)
        vpk = std::max(vpk, std::fabs((double)e.velocityAt(t)));
    CHECK(vpk > 2.0 * 1.05);
}

TEST_CASE("Chase jerk scales with move demand: slow streams plan soft, fast "
          "streams keep authority") {
    // NOT the machine fixture: the scale is a FRACTION of jmax against demand,
    // so it needs a ceiling the demand can approach.
    auto run = [](double f, double amp) {
        Config cfg;
        cfg.limits.vmax = 3.0f;
        cfg.limits.amax = 30.0f;
        cfg.limits.jmax = 500.0f;
        cfg.chase_jerk_scale = true;    // default; forced so the pin outlives it
        Engine e(cfg, 0.5f);
        float sharp = 1.0f;
        for (uint64_t t = 0; t <= 800 * kMs; t += 20 * kMs) {
            Command c;
            c.target = (float)(0.5 + amp * std::sin(2.0 * 3.14159265 * f *
                                                    (double(t) * 1e-6)));
            e.commit(c, t);
            if (t >= 400 * kMs) sharp = std::min(sharp, e.snapshot(t).sharpness);
        }
        return sharp;
    };
    // ~10% of vmax demand vs ~85%: the soft plan must be well under the
    // sharp one, and the sharp one keeps most of the ceiling.
    const float soft = run(0.3, 0.15);
    const float sharp = run(1.3, 0.31);
    // CURRENT-BEHAVIOR pin (experimental): the scale is MONOTONE in demand
    // and floored; absolute authority on fast content is the open tuning
    // item (sd-d77.1).
    CHECK(soft < 0.35f);
    CHECK(sharp > soft + 0.1f);
    CHECK(soft >= 0.15f - 1e-3f);   // floor holds
}

TEST_CASE("Infeasible deadline stretches to physical minimum + anomaly") {
    // NOT the machine fixture: Stretch is the policy under test, and a full
    // stroke has to be physically impossible in its deadline for it to fire.
    auto cfg = testConfig();          // vmax = 2 → 0→1 takes ≥ 0.5 s
    cfg.infeasible_policy = InfeasiblePolicy::Stretch;   // the guard's test
    Engine e(cfg, 0.0f);

    Command c;
    c.target       = 1.0f;
    c.duration_us  = 50 * (uint32_t)kMs;   // ludicrous 50 ms demand
    c.has_duration = true;
    REQUIRE(e.commit(c, 0));

    auto snap = e.snapshot(0);
    CHECK(snap.duration_s > 0.5f);    // stretched to ≥ distance / vmax

    kinetic::Anomaly ev;
    bool saw_fallback = false, saw_stretch = false;
    float stretch_detail = 0.0f;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::WaveformFallback) saw_fallback = true;
        if (ev.kind == (uint8_t)AnomalyType::DeadlineStretched) {
            saw_stretch = true;
            stretch_detail = ev.detail;
        }
    }
    CHECK(saw_fallback);              // the quintic could never do this move
    REQUIRE(saw_stretch);
    CHECK(stretch_detail == doctest::Approx(snap.duration_s).epsilon(0.01));
}

TEST_CASE("Retarget mid-move is C2-continuous at the commit instant") {
    Engine e(liveTuning(), 0.2f);

    Command a;
    a.target = 0.9f; a.duration_us = 800 * (uint32_t)kMs; a.has_duration = true;
    REQUIRE(e.commit(a, 0));

    const uint64_t tSwitch = 300 * kMs;
    const double p1 = e.positionAt(tSwitch);
    const double v1 = e.velocityAt(tSwitch);
    const double a1 = e.accelerationAt(tSwitch);
    REQUIRE(std::fabs(v1) > 0.1);     // genuinely mid-flight

    Command b;                        // bare chase point, hard reversal
    b.target = 0.1f;
    REQUIRE(e.commit(b, tSwitch));
    CHECK(e.mode() == Mode::Chase);

    // The new plan starts EXACTLY from the sampled state — no kink in p/v/a.
    CHECK(e.positionAt(tSwitch)     == doctest::Approx(p1).epsilon(1e-6));
    CHECK(e.velocityAt(tSwitch)     == doctest::Approx(v1).epsilon(1e-6));
    CHECK(e.accelerationAt(tSwitch) == doctest::Approx(a1).epsilon(1e-6));

    // And it still lands on the new target at rest.
    const uint64_t tEnd = tSwitch + 3 * kS;
    CHECK(e.positionAt(tEnd) == doctest::Approx(0.1).epsilon(1e-4));
    CHECK(e.velocityAt(tEnd) == doctest::Approx(0.0).epsilon(1e-4));
}

TEST_CASE("Chase: 60 Hz sine stream tracks smoothly within limits") {
    // NOT the machine fixture: tracking lag is only measurable where the
    // ceilings bind. At 10 u/s the tracker hugs this sine and the figure is 0.
    Config cfg;
    cfg.limits.vmax = 3.0f;
    cfg.limits.amax = 30.0f;
    cfg.limits.jmax = 500.0f;
    Engine e(cfg, 0.5f);

    // 0.75 Hz sine, amplitude 0.4 → peak vel ≈ 1.88, peak acc ≈ 8.9: well
    // inside the ceilings, so a good tracker should hug it.
    const double f = 0.75;
    auto target = [&](double t) {
        return 0.5 + 0.4 * std::sin(2.0 * M_PI * f * t);
    };

    double worst_err = 0.0;
    double prev_v = 0.0;
    bool have_prev = false;
    uint64_t next_cmd = 0;
    for (uint64_t t = 0; t <= 3 * kS; t += kMs) {
        if (t >= next_cmd) {
            Command c;
            c.target = (float)target((double)t * 1e-6);
            REQUIRE(e.commit(c, t));
            next_cmd += 16667;        // ~60 Hz point stream
        }
        const double p = e.positionAt(t);
        const double v = e.velocityAt(t);
        REQUIRE(std::isfinite(p));
        REQUIRE(p >= -1e-9);
        REQUIRE(p <= 1.0 + 1e-9);
        // Velocity must stay ceiling-bounded and step-continuous ACROSS
        // replans — this is the C2 no-microstutter claim, sampled.
        CHECK(std::fabs(v) <= cfg.limits.vmax * 1.001);
        if (have_prev) {
            CHECK(std::fabs(v - prev_v) <=
                  (double)cfg.limits.amax * 1e-3 * 1.05 + 1e-6);
        }
        prev_v = v; have_prev = true;
        if (t > 500 * kMs) {          // after initial catch-up
            worst_err = std::max(worst_err,
                                 std::fabs(p - target((double)t * 1e-6)));
        }
    }
    // DERIVED, not baselined: the newest point is already ONE interval stale
    // and the aim extrapolates chase_lookahead intervals past it, so the
    // tracker cannot sit further off the source than the source itself travels
    // across that window. Source peak |v| = amp*2*pi*f.
    // The "+1" is load-bearing and was not always here: a bound of the horizon
    // ALONE shrinks as the lead shrinks, which is backwards, because less lead
    // means more residual lag. Swept on this fixture (worst_err by lookahead):
    // 3.0 -> 0.071, 2.0 -> 0.026, 1.5 -> 0.042, 1.3 -> 0.048, 1.0 -> 0.059,
    // 0.5 -> 0.078. The optimum is a property of the MACHINE's headroom, not
    // of the engine, so this asserts sanity on both sides and the tuning is
    // done on the bench (bd val-091.14).
    const double src_vpk   = 0.4 * 2.0 * M_PI * f;
    const double horizon_s = ((double)cfg.chase_lookahead + 1.0) * 16667e-6;
    CHECK(worst_err < src_vpk * horizon_s);
}

TEST_CASE("Starve-settle: dead stream brakes to rest and holds") {
    Engine e(liveTuning(), 0.5f);

    // Feed an ascending ramp with feedforward so the trajectory is mid-glide
    // with real velocity when the stream dies.
    uint64_t t = 0;
    for (int i = 0; i < 30; i++) {
        Command c;
        c.target = 0.2f + 0.02f * (float)i;   // steady 1.2 units/s ramp
        REQUIRE(e.commit(c, t));
        t += 16667;
    }
    const uint64_t t_dead = t;

    // Keep sampling — no more commands. The engine must brake to rest.
    double final_p = -1.0;
    for (uint64_t ts = t_dead; ts <= t_dead + 2 * kS; ts += kMs) {
        final_p = e.positionAt(ts);
    }
    CHECK(e.velocityAt(t_dead + 2 * kS) == doctest::Approx(0.0).epsilon(1e-6));
    CHECK_FALSE(e.isBusy(t_dead + 2 * kS));
    CHECK(e.mode() == Mode::Idle);

    // Position frozen after settle (hold, no drift).
    CHECK(e.positionAt(t_dead + 3 * kS) == doctest::Approx(final_p).epsilon(1e-9));

    // A SettleEngaged anomaly was recorded.
    kinetic::Anomaly ev;
    bool saw_settle = false;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::SettleEngaged) saw_settle = true;
    }
    CHECK(saw_settle);
}

TEST_CASE("End velocity near a wall is clamped bound-safe") {
    auto cfg = liveTuning();
    // The wall guard is what this measures; the cold-start governor would clamp
    // the same vf first and hide which guard fired.
    cfg.recovery_vmax = 0.0f;
    Engine e(cfg, 0.5f);

    Command c;                        // v4 point: land at 0.98 STILL MOVING fast
    c.target       = 0.98f;
    c.duration_us  = 400 * (uint32_t)kMs;
    c.has_duration = true;
    c.end_vel      = 6.0f;
    c.has_end_vel  = true;
    REQUIRE(e.commit(c, 0));

    kinetic::Anomaly ev;
    bool saw_clamp = false;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::EndVelClamped) {
            saw_clamp = true;
            // clamped to √(amax·dist) = √(400·0.02) ≈ 2.83, not the asked 6.0
            CHECK(std::fabs(ev.detail) <= std::sqrt(400.0 * 0.02) + 1e-6);
        }
    }
    CHECK(saw_clamp);

    // Sampled trajectory (then starve-settle) never leaves the window.
    auto s = sweep(e, 0, 2 * kS);
    CHECK(s.max_p <= 1.0 + 1e-9);
    CHECK(s.min_p >= -1e-9);
    CHECK(e.velocityAt(2 * kS) == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Non-finite input is rejected; previous plan keeps executing") {
    Engine e(liveTuning(), 0.2f);

    Command good;
    good.target = 0.7f; good.duration_us = 500 * (uint32_t)kMs;
    good.has_duration = true;
    REQUIRE(e.commit(good, 0));

    Command evil;
    evil.target = std::nanf("");
    CHECK_FALSE(e.commit(evil, 100 * kMs));

    kinetic::Anomaly ev;
    REQUIRE(e.popAnomaly(ev));
    CHECK(ev.kind == (uint8_t)AnomalyType::PlanFailed);
    CHECK(ev.detail == doctest::Approx(-99.0f));

    // The good plan is untouched and still lands.
    CHECK(e.positionAt(600 * kMs) == doctest::Approx(0.7).epsilon(1e-4));
    CHECK(e.snapshot(600 * kMs).failures == 1);
}

TEST_CASE("Determinism: identical command/time sequences → identical samples") {
    auto run = [](std::vector<float>& out) {
        Config cfg;
        cfg.limits.vmax = 3.0f; cfg.limits.amax = 25.0f; cfg.limits.jmax = 400.0f;
        Engine e(cfg, 0.5f);
        uint64_t next_cmd = 0;
        int i = 0;
        for (uint64_t t = 0; t <= 2 * kS; t += kMs) {
            if (t >= next_cmd) {
                Command c;
                c.target = 0.5f + 0.35f * std::sin(0.3 * (double)i);
                if (i % 3 == 0) {      // mix modes
                    c.duration_us = 120 * (uint32_t)kMs;
                    c.has_duration = true;
                }
                e.commit(c, t);
                next_cmd += 40 * kMs;
                i++;
            }
            out.push_back(e.positionAt(t));
            out.push_back(e.velocityAt(t));
        }
    };
    std::vector<float> a, b;
    run(a); run(b);
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); i++) REQUIRE(a[i] == b[i]);
}

// ---- InfeasiblePolicy -------------------------------------------------------
// which fidelity gets sacrificed when the wire lies

TEST_CASE("Stretch policy: same command keeps the stroke and overruns the deadline") {
    // NOT the machine fixture: Stretch is the alternative policy, and its whole
    // contract (keep the stroke, overrun the clock) needs an impossible ask.
    auto cfg = machineConfig();
    cfg.infeasible_policy = InfeasiblePolicy::Stretch;
    Engine e(cfg, 0.0f);

    Command c;
    c.target       = 1.0f;
    c.duration_us  = 100 * (uint32_t)kMs;
    c.has_duration = true;
    REQUIRE(e.commit(c, 0));

    // Pre-0.3 behavior, byte for byte: the Ruckig guard owns the segment.
    CHECK(e.planKind() == kinetic::PlanKind::Ruckig);
    const double dur = e.snapshot(0).duration_s;
    MESSAGE("Stretch 0->1 in 100ms: plan runs " << dur << " s");
    CHECK(dur > 0.9);                   // ≥ distance / vmax = 1/1.1
    CHECK(e.isBusy(400 * kMs));         // still going long after the deadline

    // Full stroke delivered — late.
    CHECK(e.positionAt(2 * kS) == doctest::Approx(1.0).epsilon(1e-3));

    kinetic::Anomaly ev;
    bool saw_scaled = false, saw_fallback = false, saw_stretch = false;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::WaveformScaled)    saw_scaled = true;
        if (ev.kind == (uint8_t)AnomalyType::WaveformFallback)  saw_fallback = true;
        if (ev.kind == (uint8_t)AnomalyType::DeadlineStretched) saw_stretch = true;
    }
    CHECK_FALSE(saw_scaled);            // Stretch never scales
    CHECK(saw_fallback);
    CHECK(saw_stretch);
}

TEST_CASE("A feasible segment is bit-identical under BOTH policies") {
    // The policy is a fallback branch, not a filter: a segment the quintic can
    // legally execute must be untouched, sample for sample, whichever policy
    // is armed. Nothing outside the infeasible path may touch a legal shape.
    // NOT the machine fixture: a POLICY comparison, so both policies must be
    // reachable, which means a ceiling the commands can straddle.
    auto sample = [](InfeasiblePolicy pol, std::vector<float>& out) {
        auto cfg = machineConfig();
        cfg.infeasible_policy = pol;
        Engine e(cfg, 0.2f);
        Command c;
        c.target       = 0.5f;          // comfortably inside every ceiling
        c.duration_us  = 600 * (uint32_t)kMs;
        c.has_duration = true;
        REQUIRE(e.commit(c, 0));
        REQUIRE(e.planKind() == kinetic::PlanKind::Quintic);
        for (uint64_t t = 0; t <= 700 * kMs; t += kMs) {
            out.push_back(e.positionAt(t));
            out.push_back(e.velocityAt(t));
            out.push_back(e.accelerationAt(t));
        }
        kinetic::Anomaly ev;
        while (e.popAnomaly(ev)) {
            CHECK(ev.kind != (uint8_t)AnomalyType::WaveformScaled);
            CHECK(ev.kind != (uint8_t)AnomalyType::WaveformFallback);
        }
    };
    std::vector<float> bl, st;
    sample(InfeasiblePolicy::Blend, bl);
    sample(InfeasiblePolicy::Stretch, st);
    REQUIRE(bl.size() == st.size());
    for (size_t i = 0; i < bl.size(); i++) REQUIRE(bl[i] == st[i]);
    // And it actually reached the commanded target on the deadline.
    CHECK(bl[(600 * 3)] == doctest::Approx(0.5f).epsilon(1e-4));
}

TEST_CASE("Both policies keep the sampled window invariant [0,1]") {
    // The search builds a NEW curve that never went through commit()'s target
    // clamp, and the guard hands Ruckig — which has NO position limits — the
    // commanded endpoint, so prove the window still holds under a full-stroke
    // segment chain that is infeasible in both directions.
    // NOT the machine fixture: same reason as above, both policies must fire.
    for (auto pol : {InfeasiblePolicy::Blend, InfeasiblePolicy::Stretch}) {
        auto cfg = machineConfig();
        cfg.infeasible_policy = pol;
        Engine e(cfg, 0.5f);

        const uint64_t seg = 250 * kMs;
        uint64_t next_cmd = 0;
        int i = 0;
        double min_p = 1e9, max_p = -1e9;
        double min_raw = 1e9, max_raw = -1e9;
        for (uint64_t t = 0; t <= 6 * kS; t += kMs) {
            if (t >= next_cmd) {
                Command c;
                c.target       = (i % 2) ? 1.0f : 0.0f;   // rail to rail
                c.duration_us  = (uint32_t)seg;
                c.has_duration = true;
                c.end_vel      = (i % 2) ? 0.8f : -0.8f;  // and still moving
                c.has_end_vel  = true;
                e.commit(c, next_cmd);
                next_cmd += seg;
                i++;
            }
            const double p = e.positionAt(t);
            const double v = e.velocityAt(t);
            REQUIRE(std::isfinite(p));
            REQUIRE(p >= -1e-9);
            REQUIRE(p <= 1.0 + 1e-9);
            CHECK(std::fabs(v) <= cfg.limits.vmax * 1.001);
            min_p = std::min(min_p, p); max_p = std::max(max_p, p);
            // THE ASSERTION THAT CAN ACTUALLY FAIL. The two above ride the
            // sampler clamp, so they hold whatever the planner does; these
            // are the raw polynomial state, and the bound is what the two
            // referees actually permit -- kRuckigLegalEps of ratio slack on
            // an adopted profile, plus the coast's own kCoastMaxNorm.
            double rp, rv, ra;
            e.rawSampleAt(t, rp, rv, ra);
            min_raw = std::min(min_raw, rp); max_raw = std::max(max_raw, rp);
        }
        MESSAGE("policy " << (int)pol << ": excursion [" << min_p << ", "
                          << max_p << "]  RAW [" << min_raw << ", " << max_raw
                          << "]");
        CHECK(min_raw >= -0.10);
        CHECK(max_raw <= 1.10);
    }
}

// ---- Second-order predictive aim --------------------------------------------

TEST_CASE("Second-order chase aim stops overshooting a crest near the rail") {
    // A sine cresting just under the top rail is the shape that exposed the
    // linear aim: at the crest the velocity EMA still reads the pre-crest
    // climb, so aim = target + v_est·look is thrown PAST the crest, clamps at
    // the wall, and the end-vel guard then parks the carriage there. The
    // ½·a_est·look² term is negative through a crest and should pull it back.
    const double base = 0.55, amp = 0.40, f = 0.75;   // crest at 0.95
    const double crest = base + amp;

    // NOT the machine fixture: the aim A/B is only visible where the tracker
    // cannot simply out-accelerate its own lag.
    struct Result { double peak; double rail_ms; };
    auto run = [&](bool extrap) {
        Config cfg;
        cfg.limits.vmax = 3.0f;
        cfg.limits.amax = 30.0f;
        cfg.limits.jmax = 500.0f;
        cfg.chase_aim_accel_extrap = extrap;
        Engine e(cfg, (float)base);
        auto src = [&](double t) { return base + amp * std::sin(2*M_PI*f*t); };

        double peak = -1e9;
        int near_crest_ms = 0;
        uint64_t next_cmd = 0;
        for (uint64_t t = 0; t <= 4 * kS; t += kMs) {
            if (t >= next_cmd) {
                Command c;
                c.target = (float)src((double)t * 1e-6);
                e.commit(c, t);
                next_cmd += 16667;      // ~60 Hz bare point stream
            }
            const double p = e.positionAt(t);
            REQUIRE(p >= -1e-9);
            REQUIRE(p <= 1.0 + 1e-9);
            if (t > 700 * kMs) {        // past the catch-up transient
                peak = std::max(peak, p);
                if (p > crest) near_crest_ms++;   // time spent ABOVE the crest
            }
        }
        return Result{peak, (double)near_crest_ms};
    };

    const Result on  = run(true);
    const Result off = run(false);
    MESSAGE("crest " << crest << "  2nd-order peak " << on.peak
            << " (over-crest " << on.rail_ms << " ms)   linear peak "
            << off.peak << " (over-crest " << off.rail_ms << " ms)");

    // The absolute peak used to be pinned here at crest + 0.030, which was a
    // baseline drawn around the day's 0.0256 rather than a bound anything
    // derives; it is DELETED. The residual is chase LAG, not aim error, so the
    // claim this case can actually make is the A/B below: the accel term
    // removes overshoot, and no drift in both arms can satisfy it.
    // Turning the term off measurably re-introduces aim overshoot on top.
    CHECK(off.peak > on.peak + 0.005);
    CHECK(off.rail_ms > on.rail_ms);
}

TEST_CASE("Second-order aim shortens the dead-stop park at the rail") {
    // The costly form of the same defect: when the source crest sits ON the
    // top rail the overshooting aim CLAMPS to 1.0, applyEndVelGuard then sees
    // dist-to-wall = 0 and forces vf = 0, and the carriage sits at the rail at
    // a dead stop (measured 127–138 ms per stroke against the source sine's
    // own 0.35 % tangency dwell). Second-order aim should cut the park time.
    const double base = 0.60, amp = 0.40, f = 0.75;   // crest exactly at 1.0

    // NOT the machine fixture: same reason as the crest case above.
    auto park_ms = [&](bool extrap) {
        Config cfg;
        cfg.limits.vmax = 3.0f;
        cfg.limits.amax = 30.0f;
        cfg.limits.jmax = 500.0f;
        cfg.chase_aim_accel_extrap = extrap;
        Engine e(cfg, (float)base);
        auto src = [&](double t) { return base + amp * std::sin(2*M_PI*f*t); };

        int parked = 0;
        uint64_t next_cmd = 0;
        for (uint64_t t = 0; t <= 4 * kS; t += kMs) {
            if (t >= next_cmd) {
                Command c;
                c.target = (float)src((double)t * 1e-6);
                e.commit(c, t);
                next_cmd += 16667;
            }
            const double p = e.positionAt(t);
            const double v = e.velocityAt(t);
            REQUIRE(p >= -1e-9);
            REQUIRE(p <= 1.0 + 1e-9);
            // "Parked": pinned at the wall with essentially no motion left.
            if (t > 700 * kMs && p > 0.999 && std::fabs(v) < 0.05) parked++;
        }
        return parked;
    };
    const int on  = park_ms(true);
    const int off = park_ms(false);
    MESSAGE("rail park: 2nd-order " << on << " ms   linear " << off << " ms");
    CHECK(on < off);
}

TEST_CASE("Predictive aim v2 arrives at the velocity the stream will HAVE") {
    // The aim/arrival-velocity consistency fix. v1 aimed `look` seconds ahead
    // but requested v_est(NOW) as the arrival velocity — so through a crest,
    // where the stream is decelerating, every plan was told to arrive at the
    // pre-crest climb speed and duly overshot. v2 extrapolates the arrival
    // velocity over the same horizon (v_est + a_est·look).
    //
    // Measured as tracking error against the source sine, which is where the
    // inconsistency actually shows up: a plan that arrives too fast is a plan
    // that is in the wrong place one interval later.
    const double base = 0.55, amp = 0.40, f = 0.75;
    // NOT the machine fixture: same reason as the crest case above.
    struct Track { double rms; double peak_err; double max_pos; };
    auto run = [&](bool v2) {
        Config cfg;
        cfg.limits.vmax = 3.0f;
        cfg.limits.amax = 30.0f;
        cfg.limits.jmax = 500.0f;
        cfg.chase_aim_accel_extrap = v2;
        Engine e(cfg, (float)base);
        auto src = [&](double t) { return base + amp * std::sin(2*M_PI*f*t); };

        double sq = 0.0, peak = 0.0, maxp = -1e9;
        int n = 0;
        uint64_t next_cmd = 0;
        for (uint64_t t = 0; t <= 4 * kS; t += kMs) {
            if (t >= next_cmd) {
                Command c;
                c.target = (float)src((double)t * 1e-6);
                e.commit(c, t);
                next_cmd += 16667;      // ~60 Hz bare point stream
            }
            const double p = e.positionAt(t);
            REQUIRE(p >= -1e-9);
            REQUIRE(p <= 1.0 + 1e-9);
            if (t > 700 * kMs) {
                const double err = std::fabs(p - src((double)t * 1e-6));
                sq += err * err; peak = std::max(peak, err); n++;
                maxp = std::max(maxp, p);
            }
        }
        return Track{std::sqrt(sq / n), peak, maxp};
    };
    const Track v2 = run(true);
    const Track v1 = run(false);
    MESSAGE("sine track  v2 rms " << v2.rms << " peak " << v2.peak_err
            << " max-pos " << v2.max_pos
            << "   |  v1 rms " << v1.rms << " peak " << v1.peak_err
            << " max-pos " << v1.max_pos);
    // v2 must not be worse on either figure, and must strictly cut the
    // overshoot past the source crest (base+amp = 0.95).
    CHECK(v2.rms <= v1.rms);
    CHECK(v2.max_pos < v1.max_pos);
}

// ---- Snapshot telemetry -----------------------------------------------------

TEST_CASE("Snapshot::sharpness reports the plan's real peak jerk") {
    // The field has to mean the same thing on both plan kinds, or it is a
    // policy artifact rather than telemetry. Cross-check it against the SAMPLED
    // jerk on an easy quintic (the shape is entirely the sender's) and on a
    // guard profile (where it is Ruckig's planning ceiling).
    // NOT the machine fixture: the second half needs a GUARD profile, so the
    // command has to be infeasible under Stretch's contract.
    auto cfg = operatorConfig();
    cfg.infeasible_policy = InfeasiblePolicy::Stretch;
    cfg.limits.vmax = 2.5f;
    const double jmax = cfg.limits.jmax;

    const Shape q = segShape(cfg, 0.30, 0.55, 600);     // comfortably feasible
    REQUIRE(q.kind == (uint8_t)kinetic::PlanKind::Quintic);
    CHECK(q.sharp < 0.05);                              // an easy stroke IS soft
    CHECK(q.sharp * jmax == doctest::Approx(q.jpk).epsilon(0.05));

    const Shape r = segShape(cfg, 0.15, 0.85, 400);     // the guard's profile
    REQUIRE(r.kind == (uint8_t)kinetic::PlanKind::Ruckig);
    // Ruckig is bang-bang in jerk, so the sampled peak IS the planning ceiling
    // (the finite difference smears the switching instants, hence the margin).
    CHECK(r.sharp * jmax == doctest::Approx(r.jpk).epsilon(0.10));
}

// ---- Settle grace -----------------------------------------------------------
// transport jitter is not starvation

TEST_CASE("Settle grace coasts at the end velocity, then brakes when the stream is really gone") {
    // Two paced segments establish a cadence estimate, the second ends MOVING,
    // then the stream stops. Inside the grace the engine must COAST at the
    // end velocity (a freeze stamps a flat spot into every late-successor
    // chord join, the sd-ar3 notch); past it, the brake engages as always.
    auto run = [](uint32_t grace_us) {
        auto cfg = liveTuning();
        cfg.settle_grace_us = grace_us;
        Engine e(cfg, 0.30f);

        Command c1;
        c1.target = 0.45f; c1.duration_us = 100 * (uint32_t)kMs;
        c1.has_duration = true; c1.end_vel = 1.5f; c1.has_end_vel = true;
        REQUIRE(e.commit(c1, 0));
        Command c2;
        c2.target = 0.60f; c2.duration_us = 100 * (uint32_t)kMs;
        c2.has_duration = true; c2.end_vel = 1.5f; c2.has_end_vel = true;
        REQUIRE(e.commit(c2, 100 * kMs));
        return e;
    };

    SUBCASE("grace on: the grace window coasts at the end velocity") {
        Engine e = run(30000);
        const double p_end = e.positionAt(200 * kMs);
        const double v_end = e.velocityAt(200 * kMs);
        REQUIRE(std::fabs(v_end) > 0.5);          // genuinely ends moving
        // Sample forward through the grace on the 1 ms grid: motion continues
        // at v_end, each step bounded by one ms of it.
        double max_jump = 0.0, prev = p_end;
        for (uint64_t t = 200 * kMs; t <= 229 * kMs; t += kMs) {
            const double p = e.positionAt(t);
            max_jump = std::max(max_jump, std::fabs(p - prev));
            prev = p;
        }
        CHECK(e.positionAt(229 * kMs) ==
              doctest::Approx(p_end + v_end * 0.029).epsilon(1e-6));
        CHECK(max_jump <= std::fabs(v_end) * 1e-3 * 1.05);
        CHECK(e.mode() == Mode::Waveform);        // NOT Settle
        CHECK(e.planKind() == kinetic::PlanKind::Quintic);
        CHECK(drainFor(e, AnomalyType::SettleEngaged).seen == false);

        // Past the grace the brake engages — and does so CONTINUOUSLY (the
        // settle is anchored at the end of the hold, not at plan expiry: the
        // wrong anchor would enter the brake profile 30 ms deep and jump).
        double jump = 0.0; prev = e.positionAt(229 * kMs);
        for (uint64_t t = 230 * kMs; t <= 500 * kMs; t += kMs) {
            const double p = e.positionAt(t);
            jump = std::max(jump, std::fabs(p - prev));
            prev = p;
        }
        CHECK(e.mode() != Mode::Waveform);
        CHECK(jump <= 10.0 * 1e-3 * 1.05);        // ≤ vmax·1 ms: no jump
        CHECK(drainFor(e, AnomalyType::SettleEngaged).seen);
        CHECK(e.velocityAt(600 * kMs) == doctest::Approx(0.0).epsilon(1e-6));
    }

    SUBCASE("grace off (0): pre-0.4 behavior, brakes the instant it expires") {
        Engine e = run(0);
        e.positionAt(201 * kMs);
        CHECK(e.mode() == Mode::Settle);
        CHECK(drainFor(e, AnomalyType::SettleEngaged).seen);
    }

    SUBCASE("isolated point move still settles promptly (no cadence estimate)") {
        auto cfg = liveTuning();
        Engine e(cfg, 0.30f);
        Command c;
        c.target = 0.45f; c.duration_us = 100 * (uint32_t)kMs;
        c.has_duration = true; c.end_vel = 1.5f; c.has_end_vel = true;
        REQUIRE(e.commit(c, 0));                  // ONE command: no estimate
        e.positionAt(101 * kMs);
        CHECK(e.mode() == Mode::Settle);
    }
}

TEST_CASE("Dwell rule: a re-commanded hold's declared arrival velocity is ignored") {
    // The measured pathology: a client re-sends its hold point ~1 Hz with a
    // stale spline tangent. Honoring vf whips through the hold at 3.4 norm/s.
    // The span is the protocol's, injected exactly as the arbiter does.
    auto cfg = liveTuning();
    cfg.dwell_span_norm = kSegmentDwellSpan;
    Engine e(cfg, 0.60f);
    Command c;
    c.target = 0.60f; c.duration_us = 132 * (uint32_t)kMs;
    c.has_duration = true; c.end_vel = 0.0f; c.has_end_vel = true;
    REQUIRE(e.commit(c, 0));                // arms the previous-target latch
    c.end_vel = -3.4f;
    REQUIRE(e.commit(c, 1000 * kMs));       // the poisoned re-send
    double vpk = 0.0;
    for (uint64_t t = 1000 * kMs; t <= 1132 * kMs; t += kMs)
        vpk = std::max(vpk, (double)std::fabs(e.velocityAt(t)));
    CHECK(vpk < 0.2);                       // a hold stays held
    // Its OWN kind, not the RFC-008 knot bound: two different referees, and
    // sharing one kind made the anomaly census unreadable (sd-6b2.1). ONE
    // drain -- drainFor empties the ring, so a second call always reads clean.
    bool  dwell_seen = false, handoff_seen = false;
    float dropped = 0.0f;
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::DwellZeroed) {
            dwell_seen = true;
            dropped = ev.detail;
        }
        if (ev.kind == (uint8_t)AnomalyType::HandoffBounded) handoff_seen = true;
    }
    CHECK(dwell_seen);
    CHECK(dropped == doctest::Approx(-3.4f));   // the vf that was dropped
    CHECK_FALSE(handoff_seen);
}

TEST_CASE("Anchored commit: a late-released segment renders the wire timeline") {
    // Same chain twice: reference committed exactly on time, candidate's
    // second segment released 4 ms late but anchored at its due time. The
    // rendered curves must be identical -- release jitter never becomes
    // geometry.
    auto cfg = liveTuning();
    Engine ref(cfg, 0.30f);
    Engine late(cfg, 0.30f);

    Command c1;
    c1.target = 0.45f; c1.duration_us = 100 * (uint32_t)kMs;
    c1.has_duration = true; c1.end_vel = 1.5f; c1.has_end_vel = true;
    REQUIRE(ref.commit(c1, 0));
    REQUIRE(late.commit(c1, 0));

    Command c2;
    c2.target = 0.60f; c2.duration_us = 100 * (uint32_t)kMs;
    c2.has_duration = true; c2.end_vel = 0.0f; c2.has_end_vel = true;
    REQUIRE(ref.commit(c2, 100 * kMs));
    c2.anchor_us = 100 * kMs; c2.has_anchor = true;
    REQUIRE(late.commit(c2, 104 * kMs));

    for (uint64_t t = 105 * kMs; t <= 200 * kMs; t += kMs)
        CHECK(late.positionAt(t) ==
              doctest::Approx(ref.positionAt(t)).epsilon(1e-9));
}

// ---- Scenario harness -------------------------------------------------------
// A field figure is a TABLE, and the table is GENERATED (Nucleus tools/):
//   python tools/segtrace_to_case.py --dir artifacts/segtrace-jitter-06
//          --from 158.40 --to 166.13 --name HOLD
// reads segin.txt/plan.txt from a tools/segtrace.py pull and prints the S[]
// literal plus the Config it ran under. Never hand-transcribe a trace:
// these figures were hand-typed once, and regenerating them from the same trace
// found a swapped row pair in one and three wrong warm-up rows in another.
// late_ms is each row's own plan.txt `late=`, not a constant.
//
// play() asserts the DEFAULT INVARIANTS on every replay, so a scenario case
// carries no kinematics code of its own: it plays a table and reads the census.
namespace fieldreplay {

// mode_expect: kAny = don't care, else the Mode the commit must leave behind.
constexpr int8_t kAny   = -1;
constexpr int8_t kWave  = (int8_t)Mode::Waveform;
constexpr int8_t kChase = (int8_t)Mode::Chase;

// kCoastMaxNorm: sampleRaw coasts past a plan end so a late chord join stays
// continuous, and that coast is BOUNDED rather than zero. The raw window
// invariant has to admit exactly that much and no more.
constexpr double kCoastSlack = 0.05;

struct S {
    float tgt; uint32_t dur_ms; float vf; bool has_vf; float next; bool has_next;
    uint32_t late_ms = 3;          // this row's own queue-drain lag
    int8_t mode_expect = kAny;
};

struct Log {
    int settles = 0, scaled = 0, smoothed = 0, endvel = 0, fallback = 0;
    int failed = 0, flips = 0, commits = 0;
    double vpk = 0.0;              // peak |v| over the whole replay
    double achieved = 1.0;         // detail of the LAST WaveformScaled
    int8_t kind_last = 0;          // PlanKind adopted by the LAST commit
    // Set ONLY by a control arm that is MEANT to misbehave (the grace-off A/B
    // is the one). Everything else takes the invariants.
    bool no_invariants = false;
    // The engine is already mid-stream when this replay starts, so its row 0 is
    // CONTENT rather than an opening positioning move: the cold-start and
    // isolated-point exemptions below do not apply to it.
    bool warm_entry = false;
    // ARRIVAL MODE (sd-4k1.19). 0 = each row commits late_ms AFTER its anchor,
    // the drain-on-due timeline. Non-zero = each row commits that many ms
    // BEFORE its anchor, the forward-on-arrival timeline, so the rows pile up
    // in the engine's schedule queue. The cold-start demand census is off in
    // lead mode: it measures a row against the position at COMMIT, which in
    // lead mode is a whole lookahead before the row starts.
    uint32_t lead_ms = 0;
};

inline void drain(Engine& e, Log& lg, bool print) {
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) {
        if (KINETIC_TEST_VERBOSE && print)
            printf("      anomaly kind=%u target=%.3f detail=%.3f t=%.3f\n",
                   unsigned(ev.kind), (double)ev.target, (double)ev.detail,
                   ev.t_us / 1e6);
        switch ((AnomalyType)ev.kind) {
            case AnomalyType::SettleEngaged:    lg.settles++;  break;
            case AnomalyType::WaveformScaled:   lg.scaled++;
                                                lg.achieved = ev.detail; break;
            case AnomalyType::WaveformSmoothed: lg.smoothed++; break;
            case AnomalyType::EndVelClamped:    lg.endvel++;   break;
            case AnomalyType::WaveformFallback: lg.fallback++; break;
            case AnomalyType::PlanFailed:       lg.failed++;   break;
            default: break;
        }
    }
}

// Play a script against a live engine, advancing `now` on the firmware's own
// 1 ms grid. due0 is the first row's anchor; each row is committed late_ms
// after its anchor, exactly as the queue drain releases it.
inline void play(Engine& e, uint64_t& now, const S* seq, size_t n, uint64_t due0,
                 Log& lg, bool print = false) {
    const double vmax = (double)e.config().limits.vmax;
    const double amax = (double)e.config().limits.amax;
    const double recovery = (double)e.config().recovery_vmax;
    const double seed = e.positionAt(now);
    const double win_lo = std::min(0.0, seed) - kCoastSlack;
    const double win_hi = std::max(1.0, seed) + kCoastSlack;

    uint64_t due = due0;
    uint8_t kind = (uint8_t)e.planKind();
    double prev_rp = 0.0, prev_v = 0.0, row_vpk = 0.0, p_cmd = seed;
    bool first = true, have_row = false;
    // A row anchored late_ms in the past is already that deep in its own
    // timeline when the first sample lands on it, so exactly one sample after a
    // commit legitimately advances 1 + late_ms of PLAN time. Everywhere else
    // the step is one millisecond.
    uint32_t skew_ms = 0;
    float row_tgt = 0.0f; uint32_t row_dur = 0;

    // A row's execution is measured over the run_to that follows its commit.
    auto close_row = [&]() {
        if (!have_row || lg.no_invariants || recovery <= 0.0 || row_dur == 0)
            return;
        const double demand =
            std::fabs((double)row_tgt - p_cmd) / ((double)row_dur * 1e-3);
        // A cold cap mid-stream is the governor firing on content: the row
        // demanded well over the recovery ceiling and never got above it.
        if (demand > recovery * 1.2) CHECK(row_vpk > recovery * 1.02);
    };

    auto run_to = [&](uint64_t t_end, bool successor_pending) {
        for (; now < t_end; now += kMs) {
            (void)e.positionAt(now);
            double rp, rv, ra;
            e.rawSampleAt(now, rp, rv, ra);
            const double v  = (double)e.velocityAt(now);
            const double av = std::fabs(v);
            lg.vpk  = std::max(lg.vpk, av);
            row_vpk = std::max(row_vpk, av);
            const int settles_before = lg.settles;
            drain(e, lg, print);
            if (!lg.no_invariants) {
                // The stream is not starved while its successor is already
                // scheduled or is only a grace away from being due. ONE command
                // is not a stream: with no cadence measured the grace is zero
                // and the isolated-point rule brakes at expiry by design, so
                // the bound starts at the second commit.
                if (successor_pending && (lg.commits >= 2 || lg.warm_entry))
                    CHECK(lg.settles == settles_before);
                CHECK(rp >= win_lo);
                CHECK(rp <= win_hi);
                CHECK(av <= vmax * 1.001);
                const double step_s = (1.0 + (double)skew_ms) * 1e-3;
                if (!first) {
                    CHECK(std::fabs(rp - prev_rp) <= vmax * step_s * 1.05);
                    CHECK(std::fabs(v - prev_v)   <= amax * step_s * 1.5);
                }
            }
            // PlanKind::None is the gap between a plan expiring and its late
            // successor landing, not a planner changing its mind: only
            // transitions between REAL kinds are flips.
            const uint8_t k = (uint8_t)e.planKind();
            if (k != 0) {
                if (kind != 0 && k != kind) lg.flips++;
                kind = k;
            }
            prev_rp = rp; prev_v = v; first = false;
            skew_ms = 0;
        }
    };

    for (size_t i = 0; i < n; ++i) {
        row_vpk = 0.0;
        const uint64_t commit_at =
            lg.lead_ms != 0
                ? (due > (uint64_t)lg.lead_ms * kMs ? due - (uint64_t)lg.lead_ms * kMs : 0)
                : due + (uint64_t)seq[i].late_ms * kMs;
        run_to(commit_at, true);
        close_row();                       // closes row i-1
        Command c;
        c.target = seq[i].tgt; c.duration_us = seq[i].dur_ms * 1000u;
        c.has_duration = true;
        c.end_vel = seq[i].vf; c.has_end_vel = seq[i].has_vf;
        c.next_chord = seq[i].next; c.has_next_chord = seq[i].has_next;
        c.anchor_us = due; c.has_anchor = true;
        c.client_curve_family = 1;              // RFC-030 c1_cubic, fam=1
        const bool ok = e.commit(c, now);
        lg.commits++;
        if (KINETIC_TEST_VERBOSE && print)
            printf("  commit tgt=%.3f dur=%u vf=%s%.3f late=%u -> ok=%d kind=%u "
                   "mode=%u p=%.3f v=%.3f\n",
                   (double)c.target, unsigned(seq[i].dur_ms),
                   c.has_end_vel ? "" : "S", (double)c.end_vel,
                   unsigned(seq[i].late_ms), int(ok), unsigned(e.planKind()),
                   unsigned(e.mode()), (double)e.positionAt(now),
                   (double)e.velocityAt(now));
        if (!lg.no_invariants) {
            CHECK(ok);
            if (seq[i].mode_expect != kAny)
                CHECK((int8_t)e.mode() == seq[i].mode_expect);
        }
        drain(e, lg, print);
        lg.kind_last = (int8_t)e.planKind();
        skew_ms = lg.lead_ms != 0 ? 0 : seq[i].late_ms;
        // Row 0 out of a fresh engine is a POSITIONING move and is capped on
        // purpose (the cold-start governor); only content is checked.
        have_row = (i > 0 || lg.warm_entry) && lg.lead_ms == 0;
        p_cmd = (double)e.positionAt(now);
        row_tgt = seq[i].tgt; row_dur = seq[i].dur_ms;
        due += seq[i].dur_ms * 1000u;
    }
    row_vpk = 0.0;
    run_to(due + 3 * kMs, false);
    close_row();
    if (!lg.no_invariants) {
        CHECK(lg.failed == 0);
        // The flip-flop metric the field could only see on the machine: a
        // planner may change kind no more often than it is commanded to.
        CHECK(lg.flips <= lg.commits);
    }
}
}  // namespace fieldreplay

TEST_CASE("Field replay: a long hold segment at the rail is content, not a cold start") {
    // segtrace-jitter-06 158.412-166.122 s: the approach, a 6.875 s hold at
    // 1.000, then the exit. On the device the exit was capped at the USER speed
    // (endvel_clamped to -2.0 = the recovery vmax), shrunk to 70 %, and settled
    // at its end. Every loop.
    //   tools/segtrace_to_case.py --dir artifacts/segtrace-jitter-06
    //       --from 158.40 --to 166.13 --name HOLD
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.45f);
    uint64_t now = 1000 * kMs; Log lg;
    const S HOLD[] = {
        {0.550f,   41,   2.817f,  true,  3.600f,  true,  3},
        {1.000f,  125,   0.000f,  true,  0.000f, false,  1},
        {1.000f, 6875,   0.000f,  true,  0.000f, false,  3},
        {0.650f,  167,  -2.245f,  true,  0.000f, false,  1, kWave},
        {0.350f,  125,   0.000f,  true,  0.000f, false,  4, kWave},
        {1.000f,  208,   0.000f,  true,  0.000f, false,  4, kWave},
        {0.900f,  169,  -0.839f,  true,  0.000f, false,  1, kWave},
        {0.500f,  250,  -1.134f,  true,  0.000f, false,  4, kWave},
    };
    // The approach and the hold first (row 0 IS a cold start and is capped on
    // purpose, and its declared 2.817 handoff is clamped to the recovery
    // ceiling with it); the census is the EXIT from the hold, which enters
    // mid-stream and therefore takes every invariant including the cold-cap one.
    Log warm;
    play(e, now, HOLD, 3, now + 200 * kMs, warm);
    lg.warm_entry = true;
    play(e, now, HOLD + 3, sizeof(HOLD) / sizeof(HOLD[0]) - 3, now - 3 * kMs, lg);
    CHECK(lg.endvel == 0);      // the exit is not capped at the recovery limit
    CHECK(lg.settles == 0);     // and nothing brakes at a plan end mid-stream
    CHECK(lg.vpk > 2.05);       // the exit really ran above the recovery limit

    // sd-4k1.19: the SAME figure with every row committed 110 ms BEFORE its
    // anchor, which is what the hub forwards now. Same census.
    Engine el(liveTuning(), 0.45f);
    uint64_t nowl = 1000 * kMs;
    Log warm_l; warm_l.lead_ms = 110;
    play(el, nowl, HOLD, 3, nowl + 200 * kMs, warm_l);
    Log lead; lead.lead_ms = 110; lead.warm_entry = true;
    play(el, nowl, HOLD + 3, sizeof(HOLD) / sizeof(HOLD[0]) - 3, nowl - 3 * kMs,
         lead);
    CHECK(lead.endvel == 0);
    CHECK(lead.settles == 0);
    CHECK(lead.vpk > 2.05);
}

TEST_CASE("Field replay: the reversal figure (0.5 knot with vf -1.134) does not settle") {
    // segtrace-jitter-06 168.117-170.042 s, two full strokes: settle fired at
    // the exact plan end of the 250 ms segment into the 0.5 reversal knot,
    // successor 2.7 ms late.
    //   tools/segtrace_to_case.py --dir artifacts/segtrace-jitter-06
    //       --from 168.11 --to 170.05 --name REVERSAL
    // The hand-typed version of this table had the 207/-0.730 and 208/-0.727
    // rows swapped; the generated one is the trace.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.35f);
    uint64_t now = 1000 * kMs; Log lg;
    const S REVERSAL[] = {
        {0.350f,  166,   0.000f,  true,  0.000f, false,  3},
        {0.400f,   85,   0.859f,  true,  2.400f,  true,  1, kWave},
        {1.000f,  250,   0.000f,  true,  0.000f, false,  3, kWave},
        {0.900f,  207,  -0.730f,  true,  0.000f, false,  4, kWave},
        {0.500f,  250,  -1.134f,  true,  0.000f, false,  2, kWave},
        {0.350f,  166,   0.000f,  true,  0.000f, false,  5, kWave},
        {0.400f,   85,   0.859f,  true,  2.400f,  true,  1, kWave},
        {1.000f,  250,   0.000f,  true,  0.000f, false,  2, kWave},
        {0.900f,  208,  -0.727f,  true,  0.000f, false,  1, kWave},
        {0.500f,  250,  -1.134f,  true,  0.000f, false,  3, kWave},
        {0.350f,  166,   0.000f,  true,  0.000f, false,  3, kWave},
    };
    play(e, now, REVERSAL, sizeof(REVERSAL) / sizeof(REVERSAL[0]),
         now + 200 * kMs, lg);
    CHECK(lg.settles == 0);

    // sd-4k1.19: committed 110 ms early, the whole figure sits in the schedule
    // queue and the census must not move.
    Engine el(liveTuning(), 0.35f);
    uint64_t nowl = 1000 * kMs; Log lead; lead.lead_ms = 110;
    play(el, nowl, REVERSAL, sizeof(REVERSAL) / sizeof(REVERSAL[0]),
         nowl + 200 * kMs, lead);
    CHECK(lead.settles == 0);
}

TEST_CASE("Field replay: a re-seed is a cold start -- the next segment runs at the recovery limit") {
    // segtrace-jitter-06 147.657-147.987 s: the driver re-seeded the engine at
    // the window edge mid-script and the next segment (a 125 ms slam to 1.000)
    // was planned at the full input limit: 84 mm at a 960 mm/s peak. Doctrine
    // (sd-d77): the opening plan out of a re-seed traverses at the USER limit.
    //   tools/segtrace_to_case.py --dir artifacts/segtrace-jitter-06
    //       --from 147.65 --to 148.00 --name WARM
    // The hand-typed warm-up was three rows from elsewhere in the script; these
    // are the slam's actual predecessors.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.50f);
    uint64_t now = 1000 * kMs; Log lg;
    const S WARM[] = {
        {0.640f,  122,  -1.992f,  true,  0.000f, false,  3},
        {0.500f,   91,   0.000f, false,  0.000f, false,  3},
        {0.590f,  117,   1.238f,  true,  3.280f,  true,  4},
    };
    play(e, now, WARM, sizeof(WARM) / sizeof(WARM[0]), now + 200 * kMs, lg);
    e.resetAt(0.0f, now);
    // The slam is the FIRST row of its own replay, so the harness exempts it
    // from the cold-cap invariant -- which is the point: after a re-seed it is
    // supposed to be cold, and this case measures that it is.
    Log cold;
    const S SLAM[] = { {1.000f, 125, 0.000f, true, 0.000f, false, 5} };
    play(e, now, SLAM, 1, now + 3 * kMs, cold);
    CHECK(cold.vpk <= 2.05);

    // sd-4k1.19: the same re-seed with the rows committed 110 ms early. A
    // re-seed is still a cold start.
    Engine el(liveTuning(), 0.50f);
    uint64_t nowl = 1000 * kMs;
    Log warm_l; warm_l.lead_ms = 110;
    play(el, nowl, WARM, sizeof(WARM) / sizeof(WARM[0]), nowl + 200 * kMs,
         warm_l);
    el.resetAt(0.0f, nowl);
    Log cold_l; cold_l.lead_ms = 110;
    play(el, nowl, SLAM, 1, nowl + 3 * kMs, cold_l);
    CHECK(cold_l.vpk <= 2.05);
}

// ---- The one activity clock (sd-6b2.6) --------------------------------------

TEST_CASE("A re-seed voids the plan, never the stream: cold start, cadence kept") {
    // The driver's re-seed door fires BECAUSE the stream is alive (sd-wve), so
    // the opening plan is cold AND the grace stays cadence-sized. Zeroing the
    // estimator gave the first post-seed segment zero grace, which braked it at
    // its own expiry (the sd-wve chain re-entering through the reset door).
    auto cfg = liveTuning();
    // A shorter grace than the machine's 200 ms, so the coast-then-brake
    // boundary lands inside the coast cap and is observable at all.
    cfg.settle_grace_us = 30000;
    Engine e(cfg, 0.30f);
    auto seg = [&](float tgt, uint64_t at, uint32_t dur_ms, float vf) {
        Command c;
        c.target = tgt; c.duration_us = dur_ms * (uint32_t)kMs;
        c.has_duration = true; c.end_vel = vf; c.has_end_vel = true;
        c.anchor_us = at; c.has_anchor = true;
        REQUIRE(e.commit(c, at));
    };
    seg(0.40f,          0, 100, 1.0f);
    seg(0.50f, 100 * kMs, 100, 1.0f);
    seg(0.60f, 200 * kMs, 100, 1.0f);
    e.resetAt(0.60f, 250 * kMs);
    // 4.0 norm/s of declared handoff: warm it survives, cold it is cut to the
    // recovery ceiling -- which is the clamp, observed.
    seg(0.64f, 250 * kMs, 200, 4.0f);
    const AnomalyHit clamped = drainFor(e, AnomalyType::EndVelClamped);
    CHECK(clamped.seen);
    CHECK(clamped.detail == doctest::Approx(2.0).epsilon(1e-3));
    double vpk = 0.0;
    for (uint64_t t = 250 * kMs; t <= 450 * kMs; t += kMs)
        vpk = std::max(vpk, std::fabs((double)e.velocityAt(t)));
    CHECK(vpk <= 2.0 * 1.02);
    // dt_ema survived the seed, so the grace is 30 ms of coast, not zero.
    const auto snap = e.snapshot(300 * kMs);
    const uint64_t plan_end =
        e.lastPlanUs() + (uint64_t)(snap.duration_s * 1e6 + 0.5);
    (void)e.positionAt(plan_end + 20 * kMs);
    CHECK(e.mode() != Mode::Settle);
    (void)e.positionAt(plan_end + 45 * kMs);
    CHECK(e.mode() == Mode::Settle);
}

TEST_CASE("A hold longer than the cold-start gap is content: its exit is warm") {
    // The activity clock is stamped by plan ENDS as well as commits, so a 3 s
    // hold segment is not silence (field trace 2026-09-02, the 6.9 s rail hold
    // whose every exit ran at the recovery limit).
    Config cfg = liveTuning();
    Engine e(cfg, 0.50f);
    auto seg = [&](float tgt, uint64_t at, uint32_t dur_ms) {
        Command c;
        c.target = tgt; c.duration_us = dur_ms * (uint32_t)kMs;
        c.has_duration = true; c.end_vel = 0.0f; c.has_end_vel = true;
        c.anchor_us = at; c.has_anchor = true;
        REQUIRE(e.commit(c, at));
    };
    seg(0.50f, 0, 3000);                  // longer than kColdStartGapUs (2 s)
    (void)e.positionAt(3000 * kMs);       // the hold collapses: plan end stamped
    seg(0.90f, 3000 * kMs, 150);          // 0.4 over 150 ms: peak ~5, over 2.0
    double vpk = 0.0;
    for (uint64_t t = 3000 * kMs; t <= 3400 * kMs; t += kMs)
        vpk = std::max(vpk, std::fabs((double)e.velocityAt(t)));
    CHECK(vpk > 2.0 * 1.05);              // the exit is not clamped
}

TEST_CASE("Coast cap: past it the state is frozen, and the next plan inherits that") {
    // Reporting a velocity the position does not have is a ground-truth defect:
    // the successor would be planned from motion the machine stopped having.
    auto cfg = liveTuning();
    auto ends_moving = [&](Engine& e) {
        Command c;
        c.target = 0.45f; c.duration_us = 100 * (uint32_t)kMs;
        c.has_duration = true; c.end_vel = 1.5f; c.has_end_vel = true;
        REQUIRE(e.commit(c, 0));
        CHECK(std::fabs((double)e.velocityAt(99 * kMs)) > 1.0);
    };
    auto successor = [&](Engine& e, uint64_t at) {
        Command c;
        c.target = 0.70f; c.duration_us = 200 * (uint32_t)kMs;
        c.has_duration = true;
        REQUIRE(e.commit(c, at));
        return std::fabs((double)e.velocityAt(at));
    };
    // Inside the cap (20 ms past expiry, 0.03 of the 0.05 cap) the coast is
    // real motion.
    Engine inside(cfg, 0.30f);
    ends_moving(inside);
    CHECK(successor(inside, 120 * kMs) > 1.0);
    // Past it (200 ms) the position has been frozen for 140 ms: v reads zero.
    Engine outside(cfg, 0.30f);
    ends_moving(outside);
    CHECK(successor(outside, 300 * kMs) == doctest::Approx(0.0).epsilon(1e-9));
}

// ---- One planner per channel (sd-6b2.7) -------------------------------------

TEST_CASE("A 10 ms segment is a 10 ms span with its authored tangent") {
    // Ruling 2026-09-02 (Nucleus .claude/rules/motion-control.md, "Division of
    // labor"): WAVEFORM carries every duration-carrying segment, at any
    // duration. A short knot arrives with a real duration and a real end_vel,
    // and a routing floor threw both away.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.50f);
    Command c;
    c.target       = 0.502f;
    c.duration_us  = 10 * (uint32_t)kMs;
    c.has_duration = true;
    c.end_vel      = 0.30f;
    c.has_end_vel  = true;
    c.client_curve_family = 1;   // RFC-030 c1_cubic
    REQUIRE(e.commit(c, 0));
    // Read the plan BEFORE sampling its far end: the first sample past expiry
    // is what engages the settle brake, and that is a different plan.
    const auto   kind = e.planKind();
    const double dur  = (double)e.snapshot(0).duration_s;
    const double vT   = (double)e.velocityAt(10 * kMs);
    const double pT   = (double)e.positionAt(10 * kMs);
    MESSAGE("10 ms knot: kind " << unsigned(kind) << ", dur " << dur
            << ", v(T) " << vT << ", p(T) " << pT);
    CHECK((kind == kinetic::PlanKind::Cubic ||
           kind == kinetic::PlanKind::Quintic));
    CHECK(dur == doctest::Approx(0.010).epsilon(1e-6));
    // The authored tangent is the point of the ruling: it is rendered, not
    // replaced by a chase arrival estimate.
    CHECK(vT == doctest::Approx(0.30).epsilon(0.02));
    CHECK(pT == doctest::Approx(0.502).epsilon(0.01));
}

TEST_CASE("The field's mixed script plans ONE kind end to end -- no planner flips") {
    // 2026-09-02 field trace: a 10 ms knot between 85-250 ms segments routed
    // to chase, so the plan kind flipped cubic/ruckig/cubic twice per stroke.
    // Every command here carries a duration, so every commit is waveform.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.35f);
    const S F[] = {
        {0.400f,  85, 0.859f,  true, 2.400f, true},
        {1.000f, 250, 0.0f,    true, 0.0f,  false},
        {1.000f,  10, 0.0f,    true, 0.0f,  false},
        {0.900f, 248, -0.644f, true, 0.0f,  false},
        {0.500f, 250, -1.134f, true, 0.0f,  false},
    };
    uint64_t now = 1000 * kMs, due = now + 200 * kMs;
    int flips = 0;
    unsigned prev = 0;
    for (size_t i = 0; i < sizeof(F) / sizeof(F[0]); ++i) {
        for (; now < due + 3 * kMs; now += kMs) (void)e.positionAt(now);
        Command c;
        c.target = F[i].tgt; c.duration_us = F[i].dur_ms * 1000u;
        c.has_duration = true;
        c.end_vel = F[i].vf; c.has_end_vel = F[i].has_vf;
        c.next_chord = F[i].next; c.has_next_chord = F[i].has_next;
        c.anchor_us = due; c.has_anchor = true;
        c.client_curve_family = 1;
        REQUIRE(e.commit(c, now));
        const unsigned k = (unsigned)e.planKind();
        if (i > 0 && k != prev) flips++;
        prev = k;
        // The knot's successor lands 1 ms later on the wire, not 10 ms.
        due += (F[i].dur_ms == 10) ? 1 * kMs : F[i].dur_ms * 1000u;
    }
    CHECK(flips == 0);
}

TEST_CASE("Estimator cadence is the segment SPAN, not the anchor spacing") {
    // A segment's target is where the machine will be at anchor + T, so anchor
    // spacing is not that segment's rate. Burst-released anchors 1 ms apart
    // used to teach the estimator a 1 ms cadence (and a chord rate two orders
    // of magnitude over vmax), which pinned the chase jerk scale at the
    // ceiling and shrank the settle grace to nothing.
    using namespace fieldreplay;
    const uint32_t dur_ms[] = {166, 208, 250, 248, 166, 250, 208, 250};
    const size_t n = sizeof(dur_ms) / sizeof(dur_ms[0]);
    double mean = 0.0;
    for (size_t i = 0; i < n; ++i) mean += dur_ms[i] * 1e-3;
    mean /= (double)n;

    Engine e(liveTuning(), 0.40f);
    for (size_t i = 0; i < n; ++i) {
        Command c;
        c.target = (i % 2) ? 0.80f : 0.40f;
        c.duration_us  = dur_ms[i] * (uint32_t)kMs;
        c.has_duration = true;
        c.anchor_us = 1000 * kMs + (uint64_t)i * kMs;   // 1 ms apart
        c.has_anchor = true;
        REQUIRE(e.commit(c, 1000 * kMs + (uint64_t)i * kMs));
    }
    MESSAGE("dt_ema " << e.streamIntervalS() << " s against a " << mean
            << " s content cadence");
    CHECK(std::fabs(e.streamIntervalS() - mean) / mean < 0.20);
}

TEST_CASE("A bare point with no duration is still the chase planner's") {
    Engine e(liveTuning(), 0.30f);
    Command c;
    c.target = 0.70f;
    REQUIRE(e.commit(c, 0));
    CHECK(e.mode() == Mode::Chase);
    CHECK(e.planKind() == kinetic::PlanKind::Ruckig);
}

TEST_CASE("A segment longer than chase_stale_us must not starve its own settle grace") {
    // Field trace 2026-09-02: 587 ms segments in a slow section settled (braked
    // at amax) at plan expiry, 3 ms before their successor landed, because
    // staleness was measured from the last COMMIT. A stream whose plan is still
    // executing is not stale. chase_stale_us is the engine's 400 ms default, so
    // the long row outlives it while its own plan is still running.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.50f);
    uint64_t now = 1000 * kMs; Log lg;
    const S LONG[] = {
        {0.70f, 167,  0.0f, true, 0.0f, false, 3},
        {0.50f, 167,  0.0f, true, 0.0f, false, 3, kWave},
        {0.70f, 167,  0.0f, true, 0.0f, false, 3, kWave},
        {0.50f, 587, -0.3f, true, 0.0f, false, 3, kWave},   // arrives moving, outlives the stale window
        {0.35f, 208,  0.0f, true, 0.0f, false, 3, kWave},   // the successor, 3 ms late
    };
    play(e, now, LONG, sizeof(LONG) / sizeof(LONG[0]), now + 200 * kMs, lg);
    CHECK(lg.settles == 0);
    CHECK(lg.flips == 0);       // one planner across the long row and its neighbors
}

TEST_CASE("Segment chain with 5 ms arrival jitter: no settle storm, no mode flap") {
    // The measured defect: the firmware's 5 ms Valence pacing drain makes
    // segment arrivals jitter around their scheduled instant, so plans expire a
    // few ms before their successor lands. Pre-0.4 that fired a full Ruckig
    // brake plan every time. Every other row here is released ON its anchor and
    // every other one 5 ms after it, which is the shape the drain produces.
    using namespace fieldreplay;
    S J[14];
    for (size_t i = 0; i < sizeof(J) / sizeof(J[0]); ++i)
        // The first two rows land ON their anchor: ONE command is not a stream,
        // and with no cadence measured the isolated-point rule brakes at expiry
        // by design. The jitter starts once the stream exists.
        J[i] = S{(i % 2) ? 1.00f : 0.30f, 167, (i % 2) ? 2.6f : -2.6f, true,
                 0.0f, false, (uint32_t)((i >= 3 && (i % 2)) ? 5 : 0), kWave};

    Engine e(liveTuning(), 0.30f);
    uint64_t now = 1000 * kMs; Log lg;
    play(e, now, J, sizeof(J) / sizeof(J[0]), now + 200 * kMs, lg);
    CHECK(lg.settles == 0);
    // The flip-flop metric this case used to COMPUTE and throw away: a jittered
    // chain of identically shaped segments never changes planner at all.
    CHECK(lg.flips == 0);

    // The grace is what removes them. This arm is MEANT to misbehave, so it
    // waives the default invariants and is read only for its settle census.
    auto off = liveTuning();
    off.settle_grace_us = 0;
    Engine ctl(off, 0.30f);
    uint64_t ctl_now = 1000 * kMs;
    Log lg_off; lg_off.no_invariants = true;
    play(ctl, ctl_now, J, sizeof(J) / sizeof(J[0]), ctl_now + 200 * kMs, lg_off);
    MESSAGE("jitter chain: grace on " << lg.settles << " settles, grace off "
            << lg_off.settles);
    // Six of the fourteen rows arrive 5 ms late; with no grace each of those
    // expiries brakes, so the control must show most of them.
    CHECK(lg_off.settles >= 5);
}

TEST_CASE("Reset drops everything back to a hold") {
    Engine e(liveTuning(), 0.5f);
    Command c;
    c.target = 0.9f; c.duration_us = 500 * (uint32_t)kMs; c.has_duration = true;
    REQUIRE(e.commit(c, 0));
    REQUIRE(e.isBusy(100 * kMs));

    e.resetAt(0.42f, 200 * kMs);
    CHECK_FALSE(e.isBusy(200 * kMs));
    CHECK(e.positionAt(250 * kMs) == doctest::Approx(0.42f));
    CHECK(e.velocityAt(250 * kMs) == doctest::Approx(0.0));
    CHECK(e.mode() == Mode::Idle);
}

// ---- RFC-008 ----------------------------------------------------------------
// hub-side handoff sanity guard (one-segment lookahead)
//
// "The machine plans for the worst so clients don't have to."
//
// The guard is a SAFETY/QUALITY bound over a CONTINUOUS domain, so a handful of
// hand-picked cases would prove almost nothing about it. These tests sweep the
// whole (chord_in, chord_out, end_vel, k) grid and assert the invariants
// everywhere, in the same spirit as the other exhaustive sweeps in this file:
// proof by construction, not by sampling three points and hoping.
//
// The MEASURED pathology that motivated RFC-008 is a NAMED POINT inside that
// swept domain (kPathoEndVel below), so the real-world failure is a labeled
// regression case rather than folklore.
// ============================================================================

namespace {

using kinetic::boundHandoffVelocity;

// The measured failure, MFP plugin v0.2.1 against Valence Sim, 2026-07-25: a Makima
// tangent of 1.816 norm/s handed into a span whose own mean velocity is 0.050
// norm/s -- 36x over. Sane relative to its OWN span (whose chord was steep) and
// absurd only relative to the NEXT one, which is exactly why a current-chord-
// only bound cannot catch it.
constexpr float kPathoEndVel  = 1.816f;   // norm/s, as measured on the wire
constexpr float kPathoNext    = 0.050f;   // norm/s, the FOLLOWING span's mean
constexpr float kPathoCurrent = 3.000f;   // norm/s, its own span's mean
constexpr float kK            = 1.5f;     // the shape-preserving chord factor

// The swept axes. Chords run from a dead plateau (0 -- "arriving at a hold")
// through everything a normalized stroke window can physically mean, and are
// deliberately DENSE at the small end, because that is where the bound bites.
// The measured pathology's numbers are members of their own axes.
const std::vector<float> kChordAxis = {
    0.0f, 0.001f, 0.01f, 0.025f, 0.050f, 0.075f, 0.1f, 0.15f, 0.2f, 0.3f,
    0.4f, 0.5f, 0.7f, 0.9f, 1.0f, 1.2f, 1.5f, 1.816f, 2.0f, 2.5f,
    3.0f, 4.0f, 5.0f, 8.0f, 12.0f, 32.767f,
};
// Signed handoffs: reversals matter as much as runs, 0 is a legitimate "ends at
// rest", and the extremes are the 0x0085 wire's own +-32.767 norm/s rails.
const std::vector<float> kEndVelAxis = {
    -32.767f, -12.0f, -5.0f, -3.0f, -2.0f, -1.816f, -1.5f, -1.0f, -0.7f,
    -0.5f, -0.3f, -0.2f, -0.15f, -0.1f, -0.075f, -0.050f, -0.025f, -0.01f,
    -0.001f, -0.0f,
    0.0f, 0.001f, 0.01f, 0.025f, 0.050f, 0.075f, 0.1f, 0.15f, 0.2f, 0.3f,
    0.5f, 0.7f, 1.0f, 1.5f, 1.816f, 2.0f, 3.0f, 5.0f, 12.0f, 32.767f,
};
// k is the ONE aggressiveness knob: 0 = guard off, 1.5 = the Fritsch-Carlson
// shape-preserving value, 3.0 = the looser per-tangent box, 8.0 = the clamp
// ceiling. 12.0 is swept to prove the clamp actually clamps.
const std::vector<float> kFactorAxis = { 0.0f, 1.5f, 3.0f, 8.0f, 12.0f };

inline float sgnf(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

// One guarded WAVEFORM segment, sampled on the firmware's own 1 ms grid.
// Returns how many HandoffBounded anomalies the engine recorded.
// NOT the machine fixture: the RFC-008 cases below are a bound over chords in
// normalized units, and a soft ceiling keeps the ADOPTED plan out of the
// infeasible search so the guard is the only variable between the two arms.
int runGuardedSegment(bool lookahead, float end_vel, float next_chord, float k,
                      std::vector<double>& out) {
    Config cfg = testConfig();
    cfg.handoff_chord_factor = k;
    Engine e(cfg, 0.2f);
    Command c;
    c.target         = 0.6f;
    c.duration_us    = 400 * (uint32_t)kMs;
    c.has_duration   = true;
    c.end_vel        = end_vel;
    c.has_end_vel    = true;
    c.next_chord     = next_chord;
    c.has_next_chord = lookahead;
    REQUIRE(e.commit(c, 0));
    // POSITION AND VELOCITY, interleaved. Position alone is not enough to tell
    // two handoffs apart: a quintic lands on its commanded target either way --
    // the whole difference between a bounded and an unbounded handoff is the
    // VELOCITY it arrives with. Comparing the pair is what makes "identical
    // trajectory" mean identical motion rather than identical endpoints.
    out.clear();
    for (uint64_t t = 0; t <= 400 * kMs; t += kMs) {
        out.push_back(e.positionAt(t));
        out.push_back(e.velocityAt(t));
    }
    return drainFor(e, AnomalyType::HandoffBounded).count;
}

// True when two sampled trajectories differ ANYWHERE.
bool trajectoriesDiffer(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) if (a[i] != b[i]) return true;
    return false;
}

}  // namespace

TEST_CASE("RFC-008 handoff bound: property sweep over the whole (in, out, vf, k) grid") {
    // Every assertion below is a real doctest CHECK, so the reported assertion
    // count IS the coverage. Grid: 26 chords x 26 chords x 40 handoffs x 5 k.
    size_t points = 0;

    for (float k_raw : kFactorAxis) {
        // The engine clamps an untrusted config push to [0, 8]; the invariants
        // are stated against the EFFECTIVE k, which is what actually applied.
        const float k = k_raw > 8.0f ? 8.0f : k_raw;
        for (float cin : kChordAxis) {
            for (float cout : kChordAxis) {
                const float limit = k * std::min(cin, cout);
                for (float vf : kEndVelAxis) {
                    const float got = boundHandoffVelocity(vf, cin, cout, k_raw);
                    ++points;

                    if (k_raw <= 0.0f) {
                        // OFF SWITCH: byte-for-byte passthrough, no exceptions.
                        // This is what makes the M5d A/B honest.
                        CHECK(got == vf);
                        continue;
                    }

                    // (1) THE POINT OF THE WHOLE EXERCISE: never faster than
                    //     the FOLLOWING chord's Fritsch-Carlson limit.
                    CHECK(std::fabs(got) <= k * cout + 1e-5f);
                    // (2) ...nor the knot limit, which is the min of both sides.
                    CHECK(std::fabs(got) <= limit + 1e-5f);
                    // (3) The guard only ever REMOVES speed.
                    CHECK(std::fabs(got) <= std::fabs(vf) + 1e-5f);
                    // (4) Sign is preserved: a wrong-signed handoff stays
                    //     wrong-signed but bounded. The only thing the guard
                    //     changes is pathological SIZE.
                    if (got != 0.0f) CHECK(sgnf(got) == sgnf(vf));
                    // (5) IN-BOUNDS INPUT IS UNTOUCHED, bit for bit. A guard
                    //     that perturbs a well-behaved client is a feel
                    //     regression for every well-behaved client.
                    if (std::fabs(vf) <= limit) CHECK(got == vf);
                }
            }
        }
    }

    MESSAGE("handoff bound swept " << points
            << " (chord_in, chord_out, end_vel, k) points");
    CHECK(points == kChordAxis.size() * kChordAxis.size() *
                    kEndVelAxis.size() * kFactorAxis.size());
}

TEST_CASE("RFC-008 handoff bound: monotone in both chords") {
    // Bounding must be MONOTONE in the next segment's slope -- a roomier
    // successor can never buy LESS handoff speed than a tighter one. Without
    // this the guard would be a source of discontinuous feel changes as a
    // script's chord lengths drift. (kChordAxis is ascending by construction.)
    size_t pairs = 0;
    for (float cin : kChordAxis) {
        for (float vf : kEndVelAxis) {
            float prev = 0.0f;
            bool  first = true;
            for (float cout : kChordAxis) {
                const float got = std::fabs(boundHandoffVelocity(vf, cin, cout, kK));
                if (!first) { CHECK(got >= prev - 1e-5f); ++pairs; }
                prev = got; first = false;
            }
        }
    }
    // ...and monotone in the CURRENT chord for the same reason, by symmetry of
    // the min(). Asserted rather than assumed: min() is symmetric, the code
    // that calls it need not be.
    for (float cout : kChordAxis) {
        for (float vf : kEndVelAxis) {
            float prev = 0.0f;
            bool  first = true;
            for (float cin : kChordAxis) {
                const float got = std::fabs(boundHandoffVelocity(vf, cin, cout, kK));
                if (!first) { CHECK(got >= prev - 1e-5f); ++pairs; }
                prev = got; first = false;
            }
        }
    }
    MESSAGE("handoff bound monotonicity: " << pairs << " adjacent-pair checks");
}

TEST_CASE("RFC-008 handoff bound: zero, plateau, reversal and unknown-chord cases") {
    // end_vel = 0 is a REAL slope ("ends at rest"), never the absent sentinel
    // (that is INT16_MIN on the 0x0085 wire, decoded at ingress into
    // has_end_vel = false, so the guard never sees it at all). It must survive
    // every chord combination untouched.
    for (float cin : kChordAxis)
        for (float cout : kChordAxis)
            CHECK(boundHandoffVelocity(0.0f, cin, cout, kK) == 0.0f);

    // A plateau on EITHER side forces a dead stop at the knot: arriving at a
    // hold still moving is exactly the lie the Ground Truth doctrine forbids.
    CHECK(boundHandoffVelocity(2.0f, 0.0f, 5.0f, kK) == 0.0f);
    CHECK(boundHandoffVelocity(2.0f, 5.0f, 0.0f, kK) == 0.0f);
    CHECK(boundHandoffVelocity(-2.0f, 5.0f, 0.0f, kK) == 0.0f);

    // Reversal: a handoff pointing the OPPOSITE way to both chords keeps its
    // sign and only loses magnitude.
    CHECK(boundHandoffVelocity(-4.0f, 1.0f, 1.0f, kK) == doctest::Approx(-1.5f));
    CHECK(boundHandoffVelocity(4.0f, 1.0f, 1.0f, kK) == doctest::Approx(1.5f));

    // "Unknown" chord (negative) disables the bound on that side entirely --
    // the guard never bounds on a guess.
    CHECK(boundHandoffVelocity(9.0f, -1.0f, 0.01f, kK) == 9.0f);
    CHECK(boundHandoffVelocity(9.0f, 0.01f, -1.0f, kK) == 9.0f);

    // Non-finite passthrough: Engine::commit() rejects non-finite command
    // inputs upstream, and a numeric guard is not the place to relitigate it.
    CHECK(std::isnan(boundHandoffVelocity(std::nanf(""), 1.0f, 1.0f, kK)));
    CHECK(std::isinf(boundHandoffVelocity(INFINITY, 1.0f, 1.0f, kK)));
    // A non-finite CHORD is passed through as the caller's own end_vel, NOT as
    // a NaN: a garbage lookahead must never contaminate a sane handoff.
    CHECK(boundHandoffVelocity(1.0f, std::nanf(""), 1.0f, kK) == 1.0f);
    CHECK(boundHandoffVelocity(1.0f, 1.0f, std::nanf(""), kK) == 1.0f);
}

TEST_CASE("RFC-008 handoff bound: the MEASURED pathology (1.816 into a 0.050 span)") {
    // The regression case, named. Bounding against the CURRENT segment's own
    // chord alone leaves the pathological value completely untouched...
    CHECK(boundHandoffVelocity(kPathoEndVel, kPathoCurrent, kPathoCurrent, kK)
          == kPathoEndVel);
    // ...which is the whole reason the guard has to look ONE SEGMENT FORWARD.
    const float bounded =
        boundHandoffVelocity(kPathoEndVel, kPathoCurrent, kPathoNext, kK);
    CHECK(bounded == doctest::Approx(0.075f));
    // 36x over becomes exactly 1.5x the following span's mean speed -- the same
    // 1.816 -> 0.075 the plugin's own client-side limiter measured.
    MESSAGE("measured pathology: end_vel " << kPathoEndVel << " -> " << bounded
            << " norm/s (" << (kPathoEndVel / kPathoNext)
            << "x the next span's mean -> " << (bounded / kPathoNext) << "x)");
    CHECK(bounded / kPathoNext == doctest::Approx(kK));
}

TEST_CASE("RFC-008 guard in the engine: lookahead arms it, absence changes nothing") {
    // The ENGINE-level contract, sampled as reality on the firmware's own 1 ms
    // grid. A guard that changes motion for good input is a defect, and this is
    // the assertion that says so about the real planner rather than about the
    // arithmetic.
    SUBCASE("well-behaved handoff: untouched, no anomaly, identical trajectory") {
        // 0.4 units over 0.4 s -> chord_in 1.0; a 0.8 norm/s successor chord
        // permits up to 1.2 norm/s, so 0.5 is comfortably inside the bound.
        std::vector<double> without, with;
        CHECK(runGuardedSegment(false, 0.5f, 0.8f, kK, without) == 0);
        CHECK(runGuardedSegment(true,  0.5f, 0.8f, kK, with)    == 0);
        REQUIRE(without.size() == with.size());
        for (size_t i = 0; i < without.size(); ++i) CHECK(without[i] == with[i]);
    }

    SUBCASE("the measured pathology: bounded once, counted, motion changes") {
        std::vector<double> without, with;
        CHECK(runGuardedSegment(false, kPathoEndVel, kPathoNext, kK, without) == 0);
        CHECK(runGuardedSegment(true,  kPathoEndVel, kPathoNext, kK, with)    == 1);
        // The machine no longer tries to arrive 36x too fast, so the motion is
        // genuinely different -- and specifically the ARRIVAL VELOCITY is, which
        // is where a handoff lives. (The quintic lands on 0.6 either way.)
        CHECK(trajectoriesDiffer(without, with));
        CHECK(std::fabs(without.back()) > std::fabs(with.back()));
    }

    SUBCASE("TAIL CASE: no known successor is accepted exactly as sent") {
        // Deliberate: guessing a chord we do not have would trim well-behaved senders for free. RFC-049c evaluated an
        // own-chord fallback for exactly this case and REJECTED it (see
        // commitWaveform's comment) as an unvalidated motion-quality change --
        // this stays the honest tail case.
        std::vector<double> tail;
        CHECK(runGuardedSegment(false, kPathoEndVel, kPathoNext, kK, tail) == 0);
    }

    SUBCASE("k = 0 is a true off switch (the M5d A/B control)") {
        std::vector<double> off_with, off_without, on_with;
        CHECK(runGuardedSegment(true,  kPathoEndVel, kPathoNext, 0.0f, off_with)    == 0);
        CHECK(runGuardedSegment(false, kPathoEndVel, kPathoNext, 0.0f, off_without) == 0);
        CHECK(runGuardedSegment(true,  kPathoEndVel, kPathoNext, kK,   on_with)     == 1);
        REQUIRE(off_with.size() == off_without.size());
        for (size_t i = 0; i < off_with.size(); ++i) CHECK(off_with[i] == off_without[i]);
        CHECK(trajectoriesDiffer(on_with, off_with));
    }
}

TEST_CASE("RFC-008 guard: a bounded handoff does not poison the NEXT segment's af") {
    // The second half of the measured failure: the engine estimates a segment's
    // end ACCELERATION as a backward difference of consecutive end velocities,
    // so before the guard existed one oversized tangent corrupted the FOLLOWING
    // segment's boundary conditions too. The guard feeds the ACCEPTED value
    // forward, so the af series is built from velocities the machine actually
    // intends to reach.
    // 600 ms, so the FIRST segment is comfortably feasible and the guard is the
    // only thing that differs between the two arms. At 200 ms the first segment
    // is itself infeasible, the policy's own search picks the endpoint, and the
    // successor saturates amax in both arms -- a real answer to a different
    // question.
    // NOT the machine fixture: the poisoned af is only visible as a successor
    // that saturates amax, which needs an amax the crawl can reach.
    auto chain = [](bool lookahead) {
        Engine e(testConfig(), 0.2f);
        Command a;
        a.target = 0.6f; a.duration_us = 600 * (uint32_t)kMs; a.has_duration = true;
        a.end_vel = kPathoEndVel; a.has_end_vel = true;
        a.next_chord = kPathoNext; a.has_next_chord = lookahead;
        REQUIRE(e.commit(a, 0));
        Command b;
        b.target = 0.61f; b.duration_us = 200 * (uint32_t)kMs; b.has_duration = true;
        b.end_vel = 0.05f; b.has_end_vel = true;   // a sane, gentle successor
        REQUIRE(e.commit(b, 600 * kMs));
        return sweep(e, 600 * kMs, 800 * kMs);
    };
    const SweepStats poisoned = chain(false);
    const SweepStats guarded  = chain(true);
    MESSAGE("successor segment peak |a|: unguarded " << poisoned.max_abs_a
            << " vs guarded " << guarded.max_abs_a);
    // The successor is a 0.01-unit crawl; with the guard armed it stays a crawl
    // instead of inheriting a wild af from a handoff nobody could honor.
    CHECK(guarded.max_abs_a < poisoned.max_abs_a);
}

// ---- M7a --------------------------------------------------------------------
// NOT the machine fixture, and deliberately: every case in this section is ABOUT
// a jerk ceiling too low to turn the boundary state around, which is the
// captured machine's regime and not this one's.
//
// THE SPEED CEILING HOLDS IN BOTH DIRECTIONS
//
// Found by Valence Trace on its first real capture against Valence Sim: the plan-strip
// channel reported cur_vel = -896 mm/s on a machine whose input speed ceiling
// was 550 mm/s, while the POSITIVE peaks sat at exactly +550. The asymmetry was
// a red herring — every ceiling test in this engine is on |v|, and always was.
// What the capture had actually caught was this:
//
//   RUCKIG IS NOT A LEGALITY ORACLE. `max_velocity` is an input to its profile
//   SEARCH, not a postcondition of its output. Handed a jerk ceiling too low to
//   turn the boundary state around, Ruckig Community returns a profile that
//   sails through the velocity ceiling — and, lower still, clean out of the
//   stroke window — instead of reporting the move infeasible.
//
// A caller that asks for a SOFTER jerk ceiling than the mechanical one walks
// straight into that region if "arrives in time" is its whole feasibility
// predicate. On the captured chase→segment handoff a ceiling of j = 115 against
// a 4000 limit had Ruckig planning 1.43x vmax and swinging 0.34 units past its
// own start position.
//
// These cases pin the fix at the planner: it must not ADOPT an illegal profile,
// whatever ceiling it was handed. Only shapes that were already illegal change
// — a legal plan scans identically before and after.
namespace {

// Tolerance on the sampled ceiling, mirroring the engine's kRuckigLegalEps. A
// plan that STARTS at the ceiling with adverse acceleration must overshoot it
// slightly on the way back inside; the jerk limit says so and no planner can
// avoid it. The pathology under test is 1.4x and up, nowhere near this band.
constexpr double kCeilTol = 1.05;

// One chase→segment handoff. `pts` dense stream points along a sine (bare
// points, so they plan as CHASE), then ONE timed segment carrying no end
// velocity — the 0x0085 INT16_MIN sentinel — which is precisely what makes the
// engine fill vf from its own stream estimate rather than from the wire.
SweepStats handoffCase(const Config& cfg, double amp, double freq, double phase,
                       int pts, float seg_target, uint32_t seg_ms) {
    Engine e(cfg, 0.5f);
    const uint64_t dt = 20 * kMs;                       // 50 Hz, dense
    uint64_t clock = 0;
    for (int i = 0; i < pts; i++) {
        const uint64_t at = (uint64_t)i * dt;
        // Advance the engine the way the firmware actually does — the 1 ms
        // sampler runs BETWEEN commands, and it is what drives maybeSettle.
        // Committing on a frozen clock is a different machine.
        for (; clock < at; clock += kMs) (void)e.positionAt(clock);
        const double t = (double)at * 1e-6;
        Command c;
        c.target      = (float)(0.5 + amp * std::sin(2*M_PI*freq*t + phase));
        c.end_vel     = (float)(amp * 2*M_PI*freq * std::cos(2*M_PI*freq*t + phase));
        c.has_end_vel = true;
        e.commit(c, at);                                // bare point -> chase
    }
    const uint64_t seg_at = (uint64_t)pts * dt;
    Command s;
    s.target       = seg_target;
    s.duration_us  = seg_ms * (uint32_t)kMs;
    s.has_duration = true;
    s.has_end_vel  = false;                             // the wire sentinel
    e.commit(s, seg_at);
    return sweep(e, seg_at, seg_at + (uint64_t)seg_ms * kMs);
}

}  // namespace

TEST_CASE("M7a: the captured Valence Trace excursion — chase handoff into a segment") {
    // The capture, reconstructed: Valence Sim's limit set, the probe's own sine
    // (0.5 + 0.35·sin 2π·0.8t at 50 Hz), then the probe's own segment (target
    // 0.7, 900 ms, end-velocity sentinel). Before the fix this planned 1.43x
    // vmax and dived 0.34 units the wrong way; the plan-strip reported it
    // faithfully, which is how Valence Trace caught it.
    // Valence Sim derives the engine's normalized jerk ceiling from the mm-domain
    // input limit set (2e6 mm/s³ / 500 mm span = 4000), not from the firmware's
    // sm_tune_jmax default that machineConfig() carries — and the capture ran
    // under the derived one. The distinction matters here and only here: the
    // sharpness search hunts DOWNWARD from this ceiling, so where it starts is
    // what decides whether it can reach the illegal region at all.
    Config cfg = machineConfig();
    cfg.limits.jmax = 4000.0f;
    const SweepStats s = handoffCase(cfg, 0.35, 0.8, 0.0, 400, 0.7f, 900);
    MESSAGE("handoff peak |v| = " << s.max_abs_v << " (vmax " << cfg.limits.vmax
            << ", ratio " << s.max_abs_v / cfg.limits.vmax << "), position span ["
            << s.min_p << ", " << s.max_p << "]");
    CHECK(s.max_abs_v <= cfg.limits.vmax * kCeilTol);
    CHECK(s.max_abs_a <= cfg.limits.amax * kCeilTol);
}

TEST_CASE("M7a: |v| <= vmax across the whole handoff domain, both directions") {
    // The property sweep. If the ceiling can be broken at all, one capture is a
    // poor way to find out — so plan the handoff across a domain of stream
    // shapes (which set the inherited p, v, a AND the vf the engine estimates),
    // segment targets on BOTH sides of the handoff position (reversals), and
    // segment durations spanning the stretch factor that drives the sharpness
    // search. Upward and downward targets are tracked separately: the original
    // report was an asymmetry claim, and this is what settles it.
    for (int cfg_idx = 0; cfg_idx < 2; cfg_idx++) {
        const Config cfg = cfg_idx == 0 ? machineConfig() : operatorConfig();
        const std::string name = cfg_idx == 0 ? "virtual machine" : "operator machine";
        double worst = 0.0, worst_up = 0.0, worst_down = 0.0;
        int cases = 0;

        for (double amp : {0.20, 0.35})
        for (double freq : {0.5, 0.8, 1.2})
        for (double phase : {0.0, 0.5*M_PI, M_PI, 1.5*M_PI})
        for (float tgt : {0.3f, 0.7f})
        for (uint32_t ms : {200u, 500u, 900u}) {
            const SweepStats s = handoffCase(cfg, amp, freq, phase, 200, tgt, ms);
            cases++;
            const double ratio = s.max_abs_v / (double)cfg.limits.vmax;
            worst = std::max(worst, ratio);
            if (tgt > 0.5f) worst_up   = std::max(worst_up, ratio);
            else            worst_down = std::max(worst_down, ratio);
            CHECK(s.max_abs_v <= cfg.limits.vmax * kCeilTol);
        }
        MESSAGE(name << ": " << cases << " handoffs, worst |v|/vmax = " << worst
                << " (upward targets " << worst_up << ", downward " << worst_down
                << ")");
        // The asymmetry the report suspected does not exist. Both directions
        // are bounded alike, and the bound is the same number.
        CHECK(worst_up   <= kCeilTol);
        CHECK(worst_down <= kCeilTol);
    }
}

TEST_CASE("M7a: adopted plans stay legal under BOTH policies") {
    // The mechanism in isolation, with no stream involved: a moving start into
    // a short reversal on a long deadline is the shape whose adopted plan is
    // most easily illegal. Run under both policies, because the two reach the
    // ceiling by different routes -- Blend through the search, Stretch through
    // the guard -- and neither is allowed to exceed it.
    // NOT the machine fixture: M7a is ABOUT a jerk ceiling too low to turn the
    // boundary state around, which is the captured machine's, not this one's.
    auto worstFor = [](InfeasiblePolicy pol, float end_vel, uint32_t ms) {
        Config cfg = machineConfig();
        cfg.infeasible_policy = pol;
        Engine e(cfg, 0.8f);
        // Seed a moving state heading DOWN, then command a short reversal that
        // has to be delivered on a long clock.
        Command a;
        a.target = 0.72f; a.duration_us = 120 * (uint32_t)kMs; a.has_duration = true;
        a.end_vel = -1.0f; a.has_end_vel = true;
        e.commit(a, 0);
        Command b;
        b.target = 0.70f; b.duration_us = ms * (uint32_t)kMs; b.has_duration = true;
        b.end_vel = end_vel; b.has_end_vel = true;
        e.commit(b, 120 * kMs);
        const SweepStats s = sweep(e, 120 * kMs, 120 * kMs + (uint64_t)ms * kMs);
        return s.max_abs_v / (double)cfg.limits.vmax;
    };

    for (uint32_t ms : {300u, 600u, 900u, 1400u})
    for (float ev : {-1.05f, -0.8f, -0.4f, 0.0f, 0.4f, 0.8f, 1.05f}) {
        CHECK(worstFor(InfeasiblePolicy::Blend, ev, ms) <= kCeilTol);
        CHECK(worstFor(InfeasiblePolicy::Stretch, ev, ms) <= kCeilTol);
    }
}

// ---- OVERSHOOT GUARD (Config::overshoot_guard) -------------------------------
// The defect: a segment whose commanded duration is long relative to what the
// move needs, entered at speed. A fixed duration plus fixed endpoints uniquely
// determines a Hermite curve, so the excess time is spent as EXCURSION — the
// carriage sails far past the target and comes back. Nothing in the ceiling scan
// sees it: the plan is inside vmax, amax, jmax and the stroke window the whole
// way.

// Excursion outside [p_start, target] over the segment's own life, in window
// fractions. The bench's seg_over_* metric, in the one place a unit test can
// hold the whole thing still.
double bandExcursion(Engine& e, double p_start, double target, uint64_t t0_us,
                     uint64_t t1_us) {
    const double lo = std::min(p_start, target), hi = std::max(p_start, target);
    double ex = 0.0;
    for (uint64_t t = t0_us; t <= t1_us; t += kMs) {
        const double p = e.positionAt(t);
        ex = std::max(ex, std::max(lo - p, p - hi));
    }
    return ex;
}

TEST_CASE("Overshoot guard: a long deadline on a short move must not arc") {
    // 0.30 -> 0.72 at speed, then "be at 0.70 in 900 ms" — 0.02 of travel with
    // the carriage still moving. Unguarded, the only curve satisfying those
    // boundary conditions over 900 ms leaves the band by a wide margin.
    // NOT the machine fixture: the excursion figures below are quoted in mm
    // against this 200 mm window, which is the domain the defect was reported in.
    auto runOne = [](float guard, double& ex_out, double& reach_out) {
        auto cfg = operatorConfig();
        cfg.overshoot_guard = guard;
        Engine e(cfg, 0.30f);
        Command a;
        a.target = 0.72f; a.duration_us = 120 * (uint32_t)kMs; a.has_duration = true;
        a.end_vel = 1.0f; a.has_end_vel = true;
        REQUIRE(e.commit(a, 0));
        const double p0 = e.positionAt(120 * kMs);
        Command b;
        b.target = 0.70f; b.duration_us = 900 * (uint32_t)kMs; b.has_duration = true;
        REQUIRE(e.commit(b, 120 * kMs));
        ex_out = bandExcursion(e, p0, 0.70, 120 * kMs, 1020 * kMs);
        reach_out = e.positionAt(1020 * kMs);
    };

    double ex_off = 0, ex_on = 0, reach_off = 0, reach_on = 0;
    runOne(0.0f, ex_off, reach_off);
    runOne(1.0f, ex_on, reach_on);
    MESSAGE("overshoot guard: off " << ex_off * kSpanMm << " mm excursion, on "
            << ex_on * kSpanMm << " mm");

    // The defect is present and large without the guard.
    CHECK(ex_off > 0.05);                     // > 10 mm on the 200 mm window
    // ...and the guard cuts it by more than half, without losing the endpoint.
    CHECK(ex_on < ex_off * 0.5);
    CHECK(reach_on == doctest::Approx(0.70).epsilon(0.02));
}

TEST_CASE("Overshoot guard: MONOTONE in its own value, and inert at 0") {
    // The 0.9.0 guard was not: it sized its allowance with v0^2/(2*amax), a
    // closed form that ignores the jerk ceiling, so it was ~2.3x too strict and
    // scored WORSE at 1 than at 2. The allowance is measured now
    // (physicalBandExcess), and a looser slack factor must never buy a tighter
    // excursion.
    // NOT the machine fixture: same window as the case above, so the two read
    // as one measurement.
    auto excursionAt = [](float guard) {
        auto cfg = operatorConfig();
        cfg.overshoot_guard = guard;
        Engine e(cfg, 0.30f);
        Command a;
        a.target = 0.72f; a.duration_us = 120 * (uint32_t)kMs; a.has_duration = true;
        a.end_vel = 1.0f; a.has_end_vel = true;
        REQUIRE(e.commit(a, 0));
        const double p0 = e.positionAt(120 * kMs);
        Command b;
        b.target = 0.70f; b.duration_us = 900 * (uint32_t)kMs; b.has_duration = true;
        REQUIRE(e.commit(b, 120 * kMs));
        return bandExcursion(e, p0, 0.70, 120 * kMs, 1020 * kMs);
    };
    double prev = -1.0;
    for (float g : {0.5f, 1.0f, 2.0f, 4.0f, 8.0f}) {
        const double ex = excursionAt(g);
        CHECK(ex >= prev - 1e-9);             // never tightens as slack grows
        prev = ex;
    }
    // 0 is off, byte for byte: the same plan the pre-guard engine adopted.
    auto cfg = operatorConfig();   // same window as the excursion above
    Engine a(cfg, 0.30f), b(cfg, 0.30f);
    cfg.overshoot_guard = 0.0f;
    Command c;
    c.target = 0.90f; c.duration_us = 200 * (uint32_t)kMs; c.has_duration = true;
    REQUIRE(a.commit(c, 0));
    REQUIRE(b.commit(c, 0));
    for (uint64_t t = 0; t <= 200 * kMs; t += kMs)
        CHECK(a.positionAt(t) == doctest::Approx(b.positionAt(t)).epsilon(1e-12));
}

// ---- The two legality referees agree (sd-tki.8) ------------------------------

TEST_CASE("Both referees call the SAME curve legal (coincident-curve sweep)") {
    // The premise that makes this comparable: a pure COAST is drawn identically
    // by both planners. Boundary conditions (p, v, 0) -> (p + v*T, v, 0) make
    // the min-jerk quintic exactly the straight line c = {p, v*T, 0, 0, 0, 0},
    // and Ruckig handed max_velocity = |v| cannot arrive any sooner, so its
    // time-optimal profile is that same line over the same duration. Same
    // samples, same grid: any difference in the verdict is a difference in the
    // DEFINITION of legal, which is the bug this pins (a window grace on one
    // side only).
    // NOT the machine fixture: the sweep needs coast slopes that straddle vmax,
    // so the verdict boundary is inside the swept domain.
    auto cfg = testConfig();
    Engine e(cfg, 0.5f);
    ruckig::Ruckig<1> calc;
    int compared = 0, illegal = 0;
    for (double p0 : {-0.03, -0.01, 0.0, 0.2, 0.5, 0.9, 0.99, 1.01, 1.04}) {
        for (double vf : {-2.4, -1.0, -0.4, 0.4, 1.0, 2.4}) {
            for (double T : {0.05, 0.2, 0.6}) {
                const double p1 = p0 + vf * T;
                ruckig::InputParameter<1> in;
                in.current_position[0]     = p0;
                in.current_velocity[0]     = vf;
                in.current_acceleration[0] = 0.0;
                in.target_position[0]      = p1;
                in.target_velocity[0]      = vf;
                in.target_acceleration[0]  = 0.0;
                in.max_velocity[0]         = std::fabs(vf);
                in.max_acceleration[0]     = cfg.limits.amax;
                in.max_jerk[0]             = cfg.limits.jmax;
                ruckig::Trajectory<1> traj;
                if ((int)calc.calculate(in, traj) < 0) continue;
                // Coincidence check, not decoration: a stretched or re-shaped
                // Ruckig answer would make the comparison meaningless.
                REQUIRE(traj.get_duration() == doctest::Approx(T).epsilon(1e-9));
                const double c[6] = {p0, vf * T, 0.0, 0.0, 0.0, 0.0};
                for (double allow : {-1.0, 0.0, 0.02, 0.2}) {
                    const double wq = e.quinticWorstRatio(c, T, allow);
                    const double wr = e.ruckigWorstRatio(traj, allow);
                    // 1e-6, not 1e-9: both referees scan in float now, and one
                    // rounds coefficients while the other rounds Ruckig's
                    // double samples. The VERDICT below is the invariant.
                    CHECK(wq == doctest::Approx(wr).epsilon(1e-6));
                    CHECK((wq > 1.0) == (wr > 1.0));
                    compared++;
                    if (wq > 1.0) illegal++;
                }
            }
        }
    }
    // The sweep has to contain both verdicts, or "they never disagree" is
    // satisfied by never asking a hard question.
    MESSAGE("referee sweep: " << compared << " comparisons, " << illegal
            << " illegal");
    CHECK(compared > 300);
    CHECK(illegal > 0);
    CHECK(illegal < compared);
}

TEST_CASE("The window grace is gone from BOTH referees (rail-grazing coast)") {
    // The exact disagreement that was live: a plan grazing 0.01 outside the
    // rail scored 1.01 (illegal) as a quintic and 0.0 (legal) as a Ruckig
    // profile, because only the Ruckig side still carried the +-0.02 grace.
    // NOT the machine fixture: the graze is constructed against this vmax.
    auto cfg = testConfig();
    Engine e(cfg, 0.5f);
    ruckig::Ruckig<1> calc;
    const double v = 0.4, T = 0.1, p0 = -0.01;
    ruckig::InputParameter<1> in;
    in.current_position[0]     = p0;
    in.current_velocity[0]     = v;
    in.current_acceleration[0] = 0.0;
    in.target_position[0]      = p0 + v * T;
    in.target_velocity[0]      = v;
    in.target_acceleration[0]  = 0.0;
    in.max_velocity[0]         = v;
    in.max_acceleration[0]     = cfg.limits.amax;
    in.max_jerk[0]             = cfg.limits.jmax;
    ruckig::Trajectory<1> traj;
    REQUIRE((int)calc.calculate(in, traj) >= 0);
    const double c[6] = {p0, v * T, 0.0, 0.0, 0.0, 0.0};
    CHECK(e.quinticWorstRatio(c, T, -1.0) > 1.0);
    CHECK(e.ruckigWorstRatio(traj, -1.0) > 1.0);
}

// ---- commit() clamps every commanded target to [0,1] (sd-tki.13) ------------
// NOT the machine fixture: the clamp is proven by driving every command kind off
// BOTH rails, which needs a ceiling slow enough that a whole overshooting stroke
// fits inside the sampled window.

TEST_CASE("Out-of-window targets are clamped on EVERY command kind") {
    const double kOut[] = {1.7, 2.5, -0.9, -0.05, 1.05};
    SUBCASE("waveform (has_duration)") {
        // 1200 ms is a deadline the full clamped stroke MEETS, so the plan's
        // endpoint is the clamped target itself. On a demanding deadline the
        // endpoint is legitimately the reshaped one (measured 0.961 at 400 ms),
        // which is a policy answer, not a clamp answer -- the window sweep
        // below covers that case instead.
        for (double t : kOut) {
            auto cfg = testConfig();
            Engine e(cfg, 0.5f);
            Command c;
            c.target = (float)t;
            c.duration_us = 1200 * (uint32_t)kMs;
            c.has_duration = true;
            REQUIRE(e.commit(c, 0));
            const auto s = e.snapshot(0);
            CHECK(s.target == doctest::Approx(t > 1.0 ? 1.0 : (t < 0.0 ? 0.0 : t))
                                  .epsilon(1e-6));
            const auto sw = sweep(e, 0, 1400 * kMs);
            CHECK(sw.min_p >= -1e-9);
            CHECK(sw.max_p <= 1.0 + 1e-9);
            // Not the clamp's own output: a plan leaving the window fails here.
            CHECK(sw.min_raw >= -1e-6);
            CHECK(sw.max_raw <= 1.0 + 1e-6);
        }
    }
    SUBCASE("waveform on a demanding deadline stays in the window") {
        for (double t : kOut) {
            auto cfg = testConfig();
            Engine e(cfg, 0.5f);
            Command c;
            c.target = (float)t;
            c.duration_us = 400 * (uint32_t)kMs;
            c.has_duration = true;
            REQUIRE(e.commit(c, 0));
            const auto s = e.snapshot(0);
            CHECK(s.target >= -1e-6);
            CHECK(s.target <= 1.0 + 1e-6);
            const auto sw = sweep(e, 0, 600 * kMs);
            CHECK(sw.min_p >= -1e-9);
            CHECK(sw.max_p <= 1.0 + 1e-9);
            CHECK(sw.min_raw >= -1e-6);
            CHECK(sw.max_raw <= 1.0 + 1e-6);
        }
    }
    SUBCASE("bare point (chase)") {
        for (double t : kOut) {
            auto cfg = testConfig();
            Engine e(cfg, 0.5f);
            Command c;
            c.target = (float)t;
            REQUIRE(e.commit(c, 0));
            const auto s = e.snapshot(0);
            CHECK(s.target == doctest::Approx(t > 1.0 ? 1.0 : (t < 0.0 ? 0.0 : t))
                                  .epsilon(1e-6));
            const auto sw = sweep(e, 0, 2 * kS);
            CHECK(sw.min_p >= -1e-9);
            CHECK(sw.max_p <= 1.0 + 1e-9);
            CHECK(sw.min_raw >= -1e-6);
            CHECK(sw.max_raw <= 1.0 + 1e-6);
        }
    }
    SUBCASE("dense bare-point stream that runs off both rails (chase)") {
        // 50 Hz bare points on a sine of amplitude 0.8 about the midpoint:
        // roughly a third of every cycle is commanded outside the window.
        auto cfg = testConfig();
        Engine e(cfg, 0.5f);
        const uint64_t dt = 20 * kMs;
        double pmin = 1e9, pmax = -1e9;
        double rmin = 1e9, rmax = -1e9;
        for (uint64_t t = 0; t <= 2 * kS; t += dt) {
            Command c;
            c.target = (float)(0.5 + 0.8 * std::sin(2.0 * 3.14159265358979 *
                                                    0.8 * (double(t) * 1e-6)));
            c.has_anchor = true;
            c.anchor_us  = t;
            e.commit(c, t);   // PlanFailed is tolerated (engine contract)
            for (uint64_t q = t; q < t + dt; q += kMs) {
                const double p = e.positionAt(q);
                pmin = std::min(pmin, p);
                pmax = std::max(pmax, p);
                double rp, rv, ra;
                e.rawSampleAt(q, rp, rv, ra);
                rmin = std::min(rmin, rp);
                rmax = std::max(rmax, rp);
            }
        }
        MESSAGE("clamped chase band [" << pmin << ", " << pmax << "]  RAW ["
                << rmin << ", " << rmax << "]");
        CHECK(pmin >= -1e-9);
        CHECK(pmax <= 1.0 + 1e-9);
        // Every chase plan is refereed now (sd-6b2.2), so the RAW band is
        // bounded too: kRuckigLegalEps of slack plus the coast's own cap.
        CHECK(rmin >= -0.10);
        CHECK(rmax <= 1.10);
    }
}

// ---- The Blend ray (sd-6b2.1) -----------------------------------------------
// The regime the field defect was measured in is liveTuning()'s: 1000 mm/s over
// a 100 mm window with a stiff drive, i.e. where a rail slam lands 1 % over the
// ceiling and everything hangs on what the search does with it.

TEST_CASE("Blend: a rail slam just over the ceiling is shortened, not surrendered") {
    // 0.85 -> 1.000 in 64 ms, entering at the fastest the wall guard allows
    // (7.746 units/s: |vf|^2 = amax * 0.15). Worst ratio 1.00037, i.e. barely
    // over -- and before the floor probe it took the flat Ruckig guard, which is
    // the field's waveform_fallback on every rail end.
    using namespace fieldreplay;
    Engine e(liveTuning(), 0.50f);
    uint64_t now = 1000 * kMs;
    const S RAIL[] = {
        {0.700f, 100, 0.0f, true, 0.0f, false, 3},   // row 0 is the cold opener
        {0.850f,  60, 8.0f, true, 0.0f, false, 3},   // the run-up into the rail
        {1.000f,  64, 0.0f, true, 0.0f, false, 3},   // the slam
    };
    Log warm;
    play(e, now, RAIL, 2, now + 200 * kMs, warm);
    Log lg; lg.warm_entry = true;
    play(e, now, RAIL + 2, 1, now - 3 * kMs, lg);

    // A Hermite plan in the declared family, not the guard's bang-bang profile.
    CHECK(lg.kind_last == (int8_t)kinetic::PlanKind::Cubic);
    CHECK(lg.fallback == 0);
    CHECK(lg.scaled == 1);
    // The amplitude budget is a FLOOR: at most `budget` of the stroke may be
    // surrendered, so the achieved fraction can never fall below 1 - budget.
    CHECK(lg.achieved >= 1.0 - (double)liveTuning().infeasible_amplitude_budget - 1e-4);
    // The RAY GRID IS WALKED, NOT BISECTED (sd-6b2.9), so the adopted step is
    // the smallest legal point on a grid of infeasible_blend_steps, here
    // s = 1/6 and achieved 0.833. The bisection resolved to 2^-steps and adopted
    // 0.984 -- finer on the cases it could solve, and it could only solve the
    // ones whose ray END was legal. Resolution is bought back with
    // infeasible_blend_steps, which now costs a fraction of what it did.
    CHECK(lg.achieved == doctest::Approx(0.8333).epsilon(0.01));
}

TEST_CASE("Blend: smoothing never raises |vf| past the RFC-008 bound") {
    // A long own-chord into a short successor chord is the geometry RFC-008
    // exists for. The bound runs BEFORE the search; lerping the end handle
    // toward this span's own (large) chord afterwards used to hand the velocity
    // straight back, which is the exact failure the guard was written to stop.
    Config cfg = liveTuning();
    cfg.handoff_chord_factor = 1.5f;
    // A single-segment probe has no stream to be warm in, and the cold-start
    // governor is not what this measures.
    cfg.recovery_vmax = 0.0f;

    int  checked = 0, smoothed = 0;
    for (float chord_out : {0.05f, 0.25f, 0.75f, 1.5f}) {
        for (uint32_t ms : {60u, 80u, 110u}) {
            Engine e(cfg, 0.10f);
            Command c;
            c.target         = 0.90f;               // a long stroke...
            c.duration_us    = ms * (uint32_t)kMs;  // ...in far too little time
            c.has_duration   = true;
            c.end_vel        = 12.0f;               // and a wire handoff to match
            c.has_end_vel    = true;
            c.has_next_chord = true;
            c.next_chord     = chord_out;
            REQUIRE(e.commit(c, 0));

            const auto snap = e.snapshot(0);
            bool is_hermite = snap.plan_kind == (uint8_t)kinetic::PlanKind::Cubic ||
                              snap.plan_kind == (uint8_t)kinetic::PlanKind::Quintic;
            kinetic::Anomaly ev;
            while (e.popAnomaly(ev))
                if (ev.kind == (uint8_t)AnomalyType::WaveformSmoothed) smoothed++;
            if (!is_hermite) continue;   // the guard's plan is not this test's

            const double v_end = e.velocityAt((uint64_t)ms * kMs);
            const double bound = (double)cfg.handoff_chord_factor * (double)chord_out;
            INFO("chord_out ", chord_out, " T ", ms, " ms  v_end ", v_end);
            CHECK(std::fabs(v_end) <= bound + 1e-3);
            checked++;
        }
    }
    MESSAGE("RFC-008 after smoothing: " << checked << " adopted plans, "
            << smoothed << " smoothed");
    CHECK(checked > 0);      // never vacuous
    CHECK(smoothed > 0);     // and the smoothness axis really was spent
}

// ---- The referee itself (sd-6b2.9, sd-6b2.2, sd-6b2.3) ----------------------
// A fixed grid answers "the worst of 65 samples"; a referee has to answer "the
// worst of the curve". These pin the difference.
//
// NOT the machine fixture: these sweep CURVES past a ceiling rather than replay a
// script, so the ceiling has to sit inside the swept domain. At 10/400/50000
// every shape below is legal and the sweeps stop asking anything. The two cases
// that DO replay (the settle brake and the coast) run the machine fixture.

// The 65-point grid the referee used to BE, kept here as the cross-check. The
// closed-form peaks must never report LESS than it, and where a peak falls
// between two samples they report more.
double gridWorstRatio(const Engine& e, const double* c, double T, double allow) {
    double lo = 0.0, hi = 0.0, band = allow;
    if (allow >= 0.0) {
        double p_end = 0.0;
        for (int k = 0; k < 6; k++) p_end += c[k];
        lo = std::min(c[0], p_end);
        hi = std::max(c[0], p_end);
        band += (double)e.config().overshoot_chord_slack * (hi - lo);
    }
    const double jc = e.config().limits.jmax;
    double worst = 0.0;
    for (int i = 0; i <= 64; i++) {
        const double t = (double)i / 64.0;
        const double pp = ((((c[5]*t + c[4])*t + c[3])*t + c[2])*t + c[1])*t + c[0];
        const double vv = ((((5*c[5]*t + 4*c[4])*t + 3*c[3])*t + 2*c[2])*t + c[1]) / T;
        const double aa = (((20*c[5]*t + 12*c[4])*t + 6*c[3])*t + 2*c[2]) / (T*T);
        const double jj = ((60*c[5]*t + 24*c[4])*t + 6*c[3]) / (T*T*T);
        if (jc > 0.0) worst = std::max(worst, std::fabs(jj) / jc);
        worst = std::max(worst, e.pointWorst(pp, vv, aa, c[0], lo, hi, band));
    }
    return worst;
}

TEST_CASE("Closed-form peaks never report less than the 65-point grid") {
    auto cfg = machineConfig();
    Engine e(cfg, 0.5f);
    int shapes = 0, strictly_more = 0;
    double worst_under = 0.0, most_over = 0.0;
    // The boundary-condition domain the waveform path can hand the referee:
    // both directions, feasible to absurd, spans from 12 ms to 400 ms.
    for (double p0 : {0.05, 0.3, 0.5, 0.72, 0.95}) {
        for (double v0 : {-2.0, -0.5, 0.0, 0.9, 2.4}) {
            for (double a0 : {-20.0, 0.0, 14.0, 40.0}) {
                for (double tgt : {0.0, 0.25, 0.6, 1.0}) {
                    for (double vf : {-1.6, 0.0, 1.6}) {
                        for (double T : {0.012, 0.08, 0.4}) {
                            double c[6];
                            Engine::senderCurve(false, p0, v0, a0, tgt, vf,
                                                a0 * 0.5, T, c);
                            for (double allow : {-1.0, 0.05}) {
                                const double closed =
                                    e.quinticWorstRatio(c, T, allow);
                                const double grid =
                                    gridWorstRatio(e, c, T, allow);
                                // RELATIVE, because the ratio itself spans
                                // five decades on this domain and the scan is
                                // float: 1e-7 of relative rounding on a ratio
                                // of 1e5 is 0.01 in absolute terms and means
                                // nothing. The grid SAMPLES; the closed form
                                // finds the extremum, so anything materially
                                // BELOW the grid is the safety hole.
                                const double rel = grid > 1e-9
                                                     ? (grid - closed) / grid
                                                     : grid - closed;
                                CHECK(rel <= 1e-5);
                                worst_under = std::max(worst_under, rel);
                                if (closed > grid * (1.0 + 1e-4)) {
                                    strictly_more++;
                                    most_over = std::max(
                                        most_over, (closed - grid) / grid);
                                }
                                shapes++;
                            }
                        }
                    }
                }
            }
        }
    }
    MESSAGE("peak sweep: " << shapes << " shapes, " << strictly_more
            << " the grid understated (worst by " << most_over * 100.0
            << " %), worst relative shortfall " << worst_under);
    CHECK(shapes >= 2000);
    CHECK(worst_under <= 1e-5);
    CHECK(strictly_more > 0);
}

TEST_CASE("A peak BETWEEN grid points flips the verdict (sd-6b2.9)") {
    // Constructed, because the hole is precise. It is not a narrow spike -- a
    // quintic cannot draw one -- it is a MISALIGNED crest: the acceleration
    // peaks midway between samples 32/64 and 33/64, the grid reads both
    // shoulders, and the shoulders are lower than the peak. The shape is the
    // degree-3 Chebyshev polynomial in tau, which is the cubic with the most
    // curvature per unit of its own height, mapped so its interior extremum
    // lands at that midpoint.
    const double T  = 0.1;
    const double M  = 0.01;                 // peak of a(tau) * T^2
    const double ts = 0.5078125;            // exactly (32/64 + 33/64) / 2
    const double A  = 0.95;                 // x = A*tau + B, and T3 peaks at
    const double B  = -0.5 - A * ts;        // x = -0.5, i.e. tau = ts
    // a(tau) * T^2 = M * T3(A tau + B) = M * (4x^3 - 3x), matched term by term
    // against 2c2 + 6c3 tau + 12c4 tau^2 + 20c5 tau^3.
    double c[6];
    c[0] = 0.5;
    c[1] = 0.0;
    c[2] = M * (4.0 * B * B * B - 3.0 * B) / 2.0;
    c[3] = M * A * (4.0 * B * B - 1.0) / 2.0;
    c[4] = M * A * A * B;
    c[5] = M * A * A * A / 5.0;

    Config cfg = machineConfig();           // v, j and the window stay far
    cfg.limits.amax = (float)(M / (T * T) / 1.0002);   // inside; a peaks 1.0002
    Engine e(cfg, 0.5f);

    const double closed = e.quinticWorstRatio(c, T, -1.0);
    const double grid   = gridWorstRatio(e, c, T, -1.0);
    MESSAGE("misaligned crest: closed " << closed << "  grid " << grid);
    CHECK(closed > 1.0);        // the curve breaks the ceiling...
    CHECK(grid   < 1.0);        // ...and the grid it used to be scored on says
                                //    it does not. That is the safety hole.
}

TEST_CASE("jmax = 0 reports ILLEGAL, never legal (sd-6b2.3)") {
    // 0/0 is NaN and fmax drops a NaN, so the scan used to report the curve
    // LEGAL and the whole quintic referee was off. A zero jerk ceiling reaches
    // the engine from NVS through the host's input_max_jerk / span.
    auto cfg = machineConfig();
    cfg.limits.jmax = 0.0f;
    Engine e(cfg, 0.5f);
    // A dead-straight coast: zero jerk, zero accel, well inside every other
    // ceiling, which is the curve most likely to sail through.
    const double c[6] = {0.5, 0.05, 0.0, 0.0, 0.0, 0.0};
    CHECK(e.quinticWorstRatio(c, 0.1, -1.0) > 1.0);
    // ...and no waveform command is adopted as a Hermite plan under it.
    Command cmd;
    cmd.target = 0.7f; cmd.duration_us = 200 * (uint32_t)kMs;
    cmd.has_duration = true;
    e.commit(cmd, 0);
    const auto s = e.snapshot(0);
    CHECK(s.plan_kind != (uint8_t)kinetic::PlanKind::Quintic);
    CHECK(s.plan_kind != (uint8_t)kinetic::PlanKind::Cubic);
}

TEST_CASE("A 1 ms due gap does not blow up the af estimate (sd-6b2.3)") {
    // af = (vf - prev_vf)/gap, and the field's 10 ms segments can land anchors
    // 1 ms apart. Unfloored, a 1 ms gap against a 20 ms span scaled the
    // arrival acceleration by 1000x and sent the curve to the guard.
    auto run = [](uint64_t gap_us) {
        auto cfg = machineConfig();
        Engine e(cfg, 0.5f);
        Command a;
        a.target = 0.55f; a.duration_us = 20 * (uint32_t)kMs;
        a.has_duration = true; a.end_vel = 0.2f; a.has_end_vel = true;
        REQUIRE(e.commit(a, 0));
        Command b;
        b.target = 0.60f; b.duration_us = 20 * (uint32_t)kMs;
        b.has_duration = true; b.end_vel = 0.9f; b.has_end_vel = true;
        REQUIRE(e.commit(b, gap_us));
        return sweep(e, gap_us, gap_us + 20 * kMs);
    };
    const auto tight = run(1 * kMs);      // the pathological gap
    const auto sane  = run(20 * kMs);     // the same knots, honestly spaced
    MESSAGE("af gap: 1 ms peak |a| " << tight.max_abs_a << "   20 ms peak |a| "
            << sane.max_abs_a);
    // Same order, not 1000x apart, and inside the ceiling either way.
    CHECK(tight.max_abs_a <= sane.max_abs_a * 4.0 + 1.0);
    CHECK(tight.max_abs_a <= machineConfig().limits.amax * 1.05);
}

TEST_CASE("The settle brake is windowed: it ENDS inside the rail (sd-6b2.2)") {
    // A velocity-interface brake has no position target, so it lands wherever
    // v^2/2a puts it and _hold_pos = clamp01() then erases the difference,
    // leaving the engine's belief and the machine's position apart by exactly
    // the overshoot. Re-planned to the rail, the plan ends where it says.
    auto cfg = liveTuning();              // 10 u/s, 400 u/s^2: 12.5 mm of brake
    cfg.settle_grace_us = 0;
    // One segment, deliberately ending AT vmax into the rail: the cold-start
    // governor would cap the very thing under test.
    cfg.recovery_vmax = 0.0f;
    Engine e(cfg, 0.90f);
    Command c;
    c.target = 0.98f; c.duration_us = 40 * (uint32_t)kMs;
    c.has_duration = true;
    c.end_vel = 10.0f; c.has_end_vel = true;   // ends at vmax, into the rail
    REQUIRE(e.commit(c, 0));
    double last_p = 0.0, max_raw = -1e9;
    for (uint64_t t = 0; t <= 2 * kS; t += kMs) {
        double rp, rv, ra;
        e.rawSampleAt(t, rp, rv, ra);
        max_raw = std::max(max_raw, rp);
        last_p  = rp;
    }
    MESSAGE("settle: raw peak " << max_raw << "  rest at " << last_p);
    // It comes to REST, and it rests INSIDE the window -- the plan's own end,
    // not the clamp's opinion of where the plan ended.
    CHECK(last_p <= 1.0 + 1e-6);
    CHECK(last_p >= -1e-6);
    CHECK(std::fabs((double)e.velocityAt(2 * kS)) < 1e-3);
    // The transient IS physics (you cannot stop in less than v^2/2a), but it
    // is now bounded by that rather than by nothing.
    CHECK(max_raw <= 0.98 + 0.5 * 10.0 * 10.0 / 400.0 + 0.02);
}

TEST_CASE("Every chase plan is refereed; the window holds on RAW state "
          "(sd-6b2.2)") {
    // Chase at full authority never reached ruckigWorstRatio, and Ruckig is
    // not a legality oracle: the header's own table has it spanning
    // [-0.605, 0.853] at a low jerk ceiling. A deliberately weak softened
    // ceiling makes that reachable here. Either the softened plan is refused
    // and the mechanical retry lands, or a terminal plan is adopted WITH a
    // WaveformFallback naming the ratio. Never silently.
    // NOT the machine fixture: a deliberately weak jerk ceiling is the whole
    // premise -- it is what makes the illegal region reachable at all.
    auto cfg = machineConfig();
    cfg.limits.jmax = 30.0f;              // the regime of the header's table:
    cfg.chase_jerk_scale = false;         // too little jerk to turn the state
    Engine e(cfg, 0.5f);                  // around inside the ceilings
    int fallbacks = 0;
    double max_raw = -1e9, min_raw = 1e9;
    const uint64_t dt = 20 * kMs;
    for (uint64_t t = 0; t <= 1500 * kMs; t += dt) {
        Command c;
        c.target = (float)(0.5 + 0.85 * std::sin(2.0 * 3.14159265358979 * 1.5
                                                 * (double)t * 1e-6));
        c.has_anchor = true; c.anchor_us = t;
        e.commit(c, t);
        kinetic::Anomaly ev;
        while (e.popAnomaly(ev))
            if (ev.kind == (uint8_t)AnomalyType::WaveformFallback) fallbacks++;
        for (uint64_t q = t; q < t + dt; q += kMs) {
            double rp, rv, ra;
            e.rawSampleAt(q, rp, rv, ra);
            max_raw = std::max(max_raw, rp);
            min_raw = std::min(min_raw, rp);
        }
    }
    MESSAGE("refereed chase: " << fallbacks << " fallbacks, RAW band ["
            << min_raw << ", " << max_raw << "]");
    CHECK(fallbacks > 0);                 // never vacuous: the referee bit
    // Bounded by what the referee itself permits (kRuckigLegalEps of ratio
    // slack) plus the coast's own cap -- not by the sampler clamp.
    CHECK(max_raw <= 1.10);
    CHECK(min_raw >= -0.10);
    // ...and the clamped channel never reports outward velocity at a wall it
    // says the machine is parked against (F4).
    for (uint64_t q = 0; q <= 1500 * kMs; q += kMs) {
        const double p = e.positionAt(q);
        const double v = e.velocityAt(q);
        REQUIRE(p >= -1e-9);
        REQUIRE(p <= 1.0 + 1e-9);
        if (p >= 1.0 - 1e-12) CHECK(v <= 1e-9);
        if (p <= 1e-12)       CHECK(v >= -1e-9);
    }
}

TEST_CASE("The coast is bounded OUTSIDE the window and reports no velocity "
          "there (sd-6b2.2)") {
    // sampleRaw coasts past plan expiry so a chord join stays continuous, and
    // commit() SEEDS the next plan from it. Unbounded, at 1000 mm/s over a
    // 100 mm window, 60 ms of coast is 60 mm outside a window the machine can
    // never leave, so the successor plans from a position that never existed.
    auto cfg = liveTuning();
    cfg.settle_grace_us = 60000;          // the full grace, so the coast runs
    // The coast is what is under test, and a cold opening plan would cap the
    // velocity that produces it.
    cfg.recovery_vmax = 0.0f;
    Engine e(cfg, 0.10f);
    // A cadence first: with none measured the settle brakes at expiry and
    // there is no coast to bound (settleGraceS).
    uint64_t at = 0;
    for (int i = 1; i <= 3; i++) {
        Command w;
        w.target = (float)(0.10 + 0.20 * i);
        w.duration_us = 100 * (uint32_t)kMs;
        w.has_duration = true;
        REQUIRE(e.commit(w, at));
        at += 100 * kMs;
    }
    Command c;
    c.target = 0.80f; c.duration_us = 100 * (uint32_t)kMs;
    c.has_duration = true;
    c.end_vel = 5.0f; c.has_end_vel = true;   // ends fast, aimed at the rail
    REQUIRE(e.commit(c, at));
    double max_raw = -1e9;
    bool   moving_while_capped = false;
    for (uint64_t t = at; t <= at + 200 * kMs; t += kMs) {
        double rp, rv, ra;
        e.rawSampleAt(t, rp, rv, ra);
        max_raw = std::max(max_raw, rp);
        // At the cap the position stops advancing, so the velocity it reports
        // must stop too: one state, one story.
        if (rp >= 1.05 - 1e-6 && std::fabs(rv) > 1e-9) moving_while_capped = true;
    }
    // The PLAN itself is legal and stays in the window; everything past 1.0
    // here is the coast, which is the thing under test.
    {
        double pp, vv, aa;
        e.rawSampleAt(at + 100 * kMs, pp, vv, aa);
        CHECK(pp <= 1.0 + 1e-6);
    }
    MESSAGE("coast: raw peak " << max_raw);
    // 60 ms of coast at 5 u/s is 0.30, which unbounded lands at 1.10.
    CHECK(max_raw <= 1.05 + 1e-6);     // 1.0 + kCoastMaxNorm, and no further
    CHECK(max_raw > 1.0);              // and it really did coast out
    CHECK_FALSE(moving_while_capped);
}

// ---- Scheduled plans: a future anchor is a plan, not a demotion (sd-6b2.5) --

TEST_CASE("A future anchor is planned now and promoted AT its anchor") {
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    Command c1;
    c1.target = 0.45f; c1.duration_us = 100 * (uint32_t)kMs;
    c1.has_duration = true; c1.end_vel = 1.5f; c1.has_end_vel = true;
    REQUIRE(e.commit(c1, 0));

    const uint64_t anchor = 120 * kMs;
    Command c2;
    c2.target = 0.60f; c2.duration_us = 100 * (uint32_t)kMs;
    c2.has_duration = true; c2.end_vel = 0.0f; c2.has_end_vel = true;
    c2.anchor_us = anchor; c2.has_anchor = true;
    REQUIRE(e.commit(c2, 20 * kMs));

    // Not promoted before its anchor: the plan in flight is still the first
    // one, so its own end state is what the sampler renders up to the anchor.
    CHECK(e.lastPlanUs() == 0);
    for (uint64_t t = 20 * kMs; t < anchor; t += kMs) CHECK(e.isBusy(t));

    double pb, vb, ab, pa, va, aa;
    e.rawSampleAt(anchor - 1, pb, vb, ab);
    CHECK(e.lastPlanUs() == 0);            // still the incumbent one sample out
    e.rawSampleAt(anchor, pa, va, aa);
    CHECK(e.lastPlanUs() == anchor);       // promoted exactly at the anchor
    CHECK(e.isBusy(anchor));

    // Continuous by construction: the successor was planned FROM the state the
    // incumbent has at the anchor, and starts there.
    // The two samples are ONE MICROSECOND apart, so the whole permitted gap
    // is a microsecond of motion at the joining velocity.
    CHECK(std::fabs(pa - pb) < 1e-5);
    CHECK(std::fabs(va - vb) < 1e-3);
}

TEST_CASE("Two future anchors: both are kept, in anchor order") {
    // sd-4k1.19 CHANGED THIS EXPECTATION. It read "LAST WINS, one slot deep":
    // the later anchor replaced the parked plan and only it ever ran. With the
    // schedule queue a LATER anchor is a successor, not a replacement, so both
    // promote at their own instants. Last-wins survives only where it is still
    // true, in the earlier-anchor case below.
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    Command c1;
    c1.target = 0.45f; c1.duration_us = 100 * (uint32_t)kMs;
    c1.has_duration = true; c1.end_vel = 1.5f; c1.has_end_vel = true;
    REQUIRE(e.commit(c1, 0));

    Command c2;
    c2.target = 0.60f; c2.duration_us = 100 * (uint32_t)kMs;
    c2.has_duration = true; c2.anchor_us = 120 * kMs; c2.has_anchor = true;
    REQUIRE(e.commit(c2, 20 * kMs));

    Command c3 = c2;
    c3.target = 0.80f; c3.anchor_us = 140 * kMs;
    REQUIRE(e.commit(c3, 30 * kMs));

    e.positionAt(130 * kMs);
    CHECK(e.lastPlanUs() == 120 * kMs);      // the second command promoted
    e.positionAt(140 * kMs);
    CHECK(e.lastPlanUs() == 140 * kMs);      // ...and then the third
    CHECK(e.snapshot(140 * kMs).target > 0.70);
}

TEST_CASE("An EARLIER anchor drops the plans queued behind it") {
    // Last wins where it still means something (sd-4k1.19): a command anchored
    // before a queued plan invalidates that plan's start state, so it and
    // everything behind it go and the new one is planned from the survivor.
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    Command c1;
    c1.target = 0.45f; c1.duration_us = 100 * (uint32_t)kMs;
    c1.has_duration = true; c1.end_vel = 1.5f; c1.has_end_vel = true;
    REQUIRE(e.commit(c1, 0));

    Command c2;
    c2.target = 0.60f; c2.duration_us = 60 * (uint32_t)kMs;
    c2.has_duration = true; c2.anchor_us = 120 * kMs; c2.has_anchor = true;
    REQUIRE(e.commit(c2, 20 * kMs));
    Command c3 = c2;
    c3.target = 0.75f; c3.anchor_us = 180 * kMs;
    REQUIRE(e.commit(c3, 25 * kMs));
    Command c4 = c2;
    c4.target = 0.90f; c4.anchor_us = 240 * kMs;
    REQUIRE(e.commit(c4, 30 * kMs));

    // A revision anchored at 150 ms: c3 (180) and c4 (240) never ran, so they
    // are dropped and this plan starts from c2's end state.
    Command c5 = c2;
    c5.target = 0.20f; c5.anchor_us = 150 * kMs; c5.duration_us = 90 * (uint32_t)kMs;
    REQUIRE(e.commit(c5, 40 * kMs));

    // Continuous across BOTH promotions, and the geometry after 150 ms is the
    // revision's, not the dropped chain's.
    double prev = e.positionAt(50 * kMs);
    for (uint64_t t = 51 * kMs; t <= 300 * kMs; t += kMs) {
        const double p = e.positionAt(t);
        CHECK(std::fabs(p - prev) < 0.02);   // vmax * 1 ms, with margin
        prev = p;
        if (t == 130 * kMs) CHECK(e.lastPlanUs() == 120 * kMs);
        // Nothing promotes at 180 or 240: those plans were dropped.
        if (t >= 155 * kMs && t <= 239 * kMs)
            CHECK(e.lastPlanUs() == 150 * kMs);
        if (t == 239 * kMs) CHECK(p < 0.45);
    }
}

TEST_CASE("An anchor beyond the lead bound is refused; the plan in flight is untouched") {
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    // Comfortably inside the cold-start governor's ceiling, so the incumbent
    // plan is the command byte for byte and "untouched" means untouched.
    Command c1;
    c1.target = 0.40f; c1.duration_us = 200 * (uint32_t)kMs;
    c1.has_duration = true;
    REQUIRE(e.commit(c1, 0));
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) {}
    const double ref = e.positionAt(50 * kMs);

    Command c2 = c1;
    c2.target = 0.90f;
    c2.anchor_us = 1100 * kMs;  // past kAnchorMaxLeadUs (1000 ms)
    c2.has_anchor = true;
    CHECK_FALSE(e.commit(c2, 10 * kMs));

    bool saw = false;
    while (e.popAnomaly(ev)) {
        if (ev.kind == (uint8_t)AnomalyType::PlanFailed &&
            ev.detail == doctest::Approx(-97.0f)) saw = true;
    }
    CHECK(saw);
    CHECK(e.positionAt(50 * kMs) == doctest::Approx(ref).epsilon(1e-12));
    CHECK(e.snapshot(50 * kMs).target == doctest::Approx(0.40).epsilon(1e-6));
    CHECK_FALSE(e.isBusy(900 * kMs));   // no slot was parked
}

TEST_CASE("No settle while a scheduled successor exists") {
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    // A 60 ms segment that ends MOVING: left alone it settles at expiry.
    Command c1;
    c1.target = 0.50f; c1.duration_us = 60 * (uint32_t)kMs;
    c1.has_duration = true; c1.end_vel = 1.2f; c1.has_end_vel = true;
    REQUIRE(e.commit(c1, 0));

    Command c2;
    c2.target = 0.70f; c2.duration_us = 100 * (uint32_t)kMs;
    c2.has_duration = true;
    c2.anchor_us = 300 * kMs; c2.has_anchor = true;
    REQUIRE(e.commit(c2, 5 * kMs));

    kinetic::Anomaly ev;
    int settles = 0;
    for (uint64_t t = 5 * kMs; t < 300 * kMs; t += kMs) {
        e.positionAt(t);
        CHECK(e.isBusy(t));
        while (e.popAnomaly(ev))
            if (ev.kind == (uint8_t)AnomalyType::SettleEngaged) settles++;
    }
    CHECK(settles == 0);
    CHECK(e.mode() != Mode::Settle);
    e.positionAt(300 * kMs);
    CHECK(e.lastPlanUs() == 300 * kMs);
}

TEST_CASE("A 110 ms lookahead of segments renders every anchor, continuously") {
    // sd-4k1.19: the hub forwards on arrival, so the whole lookahead is parked
    // in the engine at once. Twenty segments of 41 to 250 ms, each committed
    // 110 ms before its anchor, must render at their own anchors with no
    // settle and no discontinuity.
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    constexpr uint64_t kLead = 110 * kMs;
    const uint32_t durs[4] = {41, 60, 41, 250};
    struct Row { uint64_t at, anchor; Command c; };
    std::vector<Row> rows;
    uint64_t due = 200 * kMs;
    for (int i = 0; i < 20; ++i) {
        Command c;
        c.target = (i % 2 == 0) ? 0.70f : 0.30f;
        c.duration_us = durs[i % 4] * (uint32_t)kMs;
        c.has_duration = true;
        c.end_vel = 0.0f; c.has_end_vel = true;
        c.anchor_us = due; c.has_anchor = true;
        rows.push_back({due - kLead, due, c});
        due += durs[i % 4] * kMs;
    }

    const double vmax = cfg.limits.vmax, amax = cfg.limits.amax;
    size_t next_row = 0, next_anchor = 0;
    int settles = 0, failed = 0;
    double prev_p = e.positionAt(0), prev_v = 0.0;
    kinetic::Anomaly ev;
    for (uint64_t t = kMs; t <= due + 50 * kMs; t += kMs) {
        while (next_row < rows.size() && rows[next_row].at <= t) {
            CHECK(e.commit(rows[next_row].c, t));
            ++next_row;
        }
        const double p = e.positionAt(t), v = e.velocityAt(t);
        CHECK(std::fabs(p - prev_p) <= vmax * 1e-3 * 1.05);
        CHECK(std::fabs(v - prev_v) <= amax * 1e-3 * 1.5);
        CHECK(p >= -0.05);
        CHECK(p <= 1.05);
        prev_p = p; prev_v = v;
        if (next_anchor < rows.size() && t >= rows[next_anchor].anchor) {
            CHECK(e.lastPlanUs() == rows[next_anchor].anchor);
            ++next_anchor;
        }
        while (e.popAnomaly(ev)) {
            if (ev.kind == (uint8_t)AnomalyType::SettleEngaged) settles++;
            if (ev.kind == (uint8_t)AnomalyType::PlanFailed) failed++;
        }
    }
    CHECK(next_anchor == rows.size());   // every anchor promoted, in order
    CHECK(settles == 0);
    CHECK(failed == 0);
}

TEST_CASE("A full schedule queue refuses with -96 and keeps what it holds") {
    auto cfg = liveTuning();
    Engine e(cfg, 0.30f);

    Command c;
    c.duration_us = 40 * (uint32_t)kMs;
    c.has_duration = true; c.end_vel = 0.0f; c.has_end_vel = true;
    c.has_anchor = true;
    // kScheduleDepth (32) segments of 30 ms, the whole 1000 ms horizon at
    // 32/s: every one parks.
    constexpr int kDepth = 32;
    c.duration_us = 30 * (uint32_t)kMs;
    for (int i = 0; i < kDepth; ++i) {
        c.target = (i % 2 == 0) ? 0.60f : 0.40f;
        c.anchor_us = uint32_t((40 + 30 * i) * kMs);
        REQUIRE(e.commit(c, 10 * kMs));
    }
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) {}

    Command over = c;
    over.target = 0.95f;
    over.anchor_us = uint32_t((40 + 30 * kDepth) * kMs);    // behind all of them
    CHECK_FALSE(e.commit(over, 10 * kMs));
    bool saw = false;
    while (e.popAnomaly(ev))
        if (ev.kind == (uint8_t)AnomalyType::PlanFailed &&
            ev.detail == doctest::Approx(-96.0f)) saw = true;
    CHECK(saw);

    // Every one it holds still runs, at its own anchor, and the refused one
    // never appears.
    for (int i = 0; i < kDepth; ++i) {
        const uint64_t at = (40 + 30 * i) * kMs;
        e.positionAt(at);
        CHECK(e.lastPlanUs() == at);
    }
    for (uint64_t t = (40 + 30 * kDepth) * kMs; t <= (120 + 30 * kDepth) * kMs; t += kMs)
        CHECK(e.positionAt(t) < 0.80);
}

// ---- The honest seed and the entry-relative window (sd-6b2.10) --------------

TEST_CASE("An out-of-window seed plans a monotone inward entry (the 87 mm field case)") {
    // Homed: carriage at 0 mm, window 87-187 mm, so the seed normalizes to
    // -0.87. The engine must keep that as real state and plan the entry.
    auto cfg = liveTuning();
    Engine e(cfg, -0.87f);
    CHECK(e.positionAt(0) == doctest::Approx(-0.87).epsilon(1e-6));

    Command c;
    c.target = 0.40f; c.duration_us = 168 * (uint32_t)kMs;
    c.has_duration = true;
    REQUIRE(e.commit(c, 0));

    double prev = -1e9, min_raw = 1e9, max_raw = -1e9;
    double min_p = 1e9, max_p = -1e9;
    bool monotone = true;
    for (uint64_t t = 0; t <= 400 * kMs; t += kMs) {
        double rp, rv, ra;
        e.rawSampleAt(t, rp, rv, ra);
        min_raw = std::min(min_raw, rp);
        max_raw = std::max(max_raw, rp);
        const double p = e.positionAt(t);
        min_p = std::min(min_p, p);
        max_p = std::max(max_p, p);
        if (rp < prev - 1e-9) monotone = false;
        prev = rp;
    }
    MESSAGE("entry from -0.87: raw [" << min_raw << ", " << max_raw
            << "]  clamped [" << min_p << ", " << max_p << "]");
    CHECK(monotone);                     // inward, never further out
    CHECK(min_raw >= -0.87 - 1e-6);
    CHECK(min_p >= -0.87 - 1e-6);
    CHECK(max_p <= 1.0 + 1e-9);
}

TEST_CASE("A plan leaving the window further out than it entered is ILLEGAL") {
    auto cfg = liveTuning();
    Engine e(cfg, -0.50f);
    // Same entry, two curves: one heading inward, one dipping further out.
    const double T = 0.2;
    const double inward[6]  = {-0.5,  0.15, 0.0, 0.0, 0.0, 0.0};
    const double outward[6] = {-0.5, -0.15, 0.0, 0.0, 0.0, 0.0};
    CHECK(e.quinticWorstRatio(inward, T, -1.0) <= 1.0);
    CHECK(e.quinticWorstRatio(outward, T, -1.0) > 1.0);
}

TEST_CASE("In-window plans are refereed EXACTLY as before (entry sweep over [0,1])") {
    // The window term is untouched wherever the seed and the entry are inside:
    // [min(0,p0), max(1,p0)] is [0,1] there by construction. Pinned against an
    // independent [0,1] scorer rather than against itself.
    // NOT the machine fixture: a referee sweep, so the swept slopes have to
    // straddle vmax for the comparison to have both verdicts in it.
    auto cfg = testConfig();
    Engine e(cfg, 0.5f);
    int compared = 0, illegal = 0;
    for (double p0 = 0.0; p0 <= 1.0001; p0 += 0.05) {
        for (double dp : {-0.9, -0.3, 0.0, 0.3, 0.9}) {
            for (double T : {0.05, 0.2, 0.6}) {
                const double c[6] = {p0, dp, 0.0, 0.0, 0.0, 0.0};
                const double got = e.quinticWorstRatio(c, T, -1.0);
                double want = std::fabs((float)(dp / T)) /
                              (double)cfg.limits.vmax;
                for (int i = 0; i <= 64; i++) {
                    const double pp = p0 + dp * ((double)i / 64.0);
                    if (pp < -1e-6) want = std::max(want, 1.0 + (-pp));
                    if (pp > 1.0 + 1e-6) want = std::max(want, 1.0 + (pp - 1.0));
                }
                CHECK(got == doctest::Approx(want).epsilon(1e-5));
                compared++;
                if (got > 1.0) illegal++;
            }
        }
    }
    MESSAGE("in-window referee sweep: " << compared << " curves, " << illegal
            << " illegal");
    CHECK(compared > 300);
    CHECK(illegal > 0);
    CHECK(illegal < compared);
}

TEST_CASE("planView hands out the plan without touching it") {
    // The accessor is what a renderer on another core reads instead of holding
    // its own engine, so its whole contract is two claims: calling it changes
    // nothing, and the piece it returns evaluates to what the engine's own raw
    // sampler evaluates.
    Engine e(liveTuning(), 0.2f);
    e.resetAt(0.2f, 0);
    Command c;
    c.target = 0.8f;
    c.has_duration = true;
    c.duration_us = 300000;
    c.has_end_vel = true;
    c.end_vel = 0.0f;
    REQUIRE(e.commit(c, 1000));

    // NO STATE CHANGE ACROSS A CALL, and none from asking about instants past
    // expiry either, where every mutating sampler here would settle.
    const uint64_t t_mid = 1000 + 150000;
    const auto kind0 = e.planKind();
    const uint64_t start0 = e.lastPlanUs();
    const auto mode0 = e.mode();
    double p0, v0, a0;
    e.rawSampleAt(t_mid, p0, v0, a0);

    kinetic::PlanView pv = e.planView();
    for (int k = 0; k < 3; ++k) pv = e.planView();

    CHECK(e.planKind() == kind0);
    CHECK(e.lastPlanUs() == start0);
    CHECK(e.mode() == mode0);
    double p1, v1, a1;
    e.rawSampleAt(t_mid, p1, v1, a1);
    CHECK(p1 == doctest::Approx(p0));
    CHECK(v1 == doctest::Approx(v0));

    CHECK(pv.active.kind == kind0);
    CHECK(pv.active.start_us == start0);
    CHECK(pv.next_ok == false);
    CHECK(pv.lo <= 0.0);
    CHECK(pv.hi >= 1.0);
    CHECK(pv.coast_cap_s > 0.0);
    CHECK(pv.coast_max_norm > 0.0);

    // The piece IS the plan: three instants inside the span, where rawSampleAt
    // is a pure read of the same polynomial.
    for (uint64_t t : {uint64_t(1000 + 20000), uint64_t(1000 + 150000),
                       uint64_t(1000 + 290000)}) {
        double ep, ev, ea, rp, rv, ra;
        kinetic::Engine::evalPiece(pv.active, t, ep, ev, ea);
        e.rawSampleAt(t, rp, rv, ra);
        CHECK(ep == doctest::Approx(rp).epsilon(1e-9));
        CHECK(ev == doctest::Approx(rv).epsilon(1e-9));
        CHECK(ea == doctest::Approx(ra).epsilon(1e-9));
    }

    // A scheduled successor shows up as its own piece, at its own anchor.
    Command later;
    later.target = 0.4f;
    later.has_duration = true;
    later.duration_us = 200000;
    later.has_anchor = true;
    later.anchor_us = 1000 + 250000;
    REQUIRE(e.commit(later, t_mid));
    const kinetic::PlanView sched = e.planView();
    CHECK(sched.next_ok == true);
    CHECK(sched.next.start_us == later.anchor_us);
    CHECK(sched.next.kind != kinetic::PlanKind::None);
    CHECK(sched.active.start_us == start0);
}

TEST_CASE("brake(): a STOP brakes a moving plan to rest without reversing, drops the schedule") {
    Engine e(liveTuning(), 0.2f);
    CHECK_FALSE(e.brake(1 * kS));   // nothing moving

    Command c;
    c.target       = 0.8f;
    c.duration_us  = 600 * (uint32_t)kMs;
    c.has_duration = true;
    const uint64_t t0 = 1 * kS;
    REQUIRE(e.commit(c, t0));
    Command next = c;
    next.target     = 0.3f;
    next.has_anchor = true;
    next.anchor_us  = t0 + 600 * kMs;
    const uint64_t ts = t0 + 200 * kMs;       // mid-stroke, moving up
    REQUIRE(e.commit(next, ts));              // parked in the schedule
    const double p0 = e.positionAt(ts);
    const double v0 = e.velocityAt(ts);
    REQUIRE(v0 > 0.5);
    REQUIRE(e.brake(ts));
    CHECK(e.mode() == Mode::Settle);

    double prev = p0;
    for (uint64_t t = ts; t <= ts + 400 * kMs; t += kMs) {
        const double p = e.positionAt(t);
        CHECK(p >= prev - 1e-9);   // never a reversal
        prev = p;
    }
    CHECK_FALSE(e.isBusy(ts + 400 * kMs));
    CHECK(e.velocityAt(ts + 400 * kMs) == doctest::Approx(0.0));
    // The scheduled 0.3 never ran, and the stop landed short of the stroke's 0.8.
    CHECK(e.positionAt(ts + 2 * kS) == doctest::Approx(prev));
    CHECK(prev < 0.8);
    kinetic::Anomaly ev;
    while (e.popAnomaly(ev)) CHECK(ev.kind != (uint8_t)AnomalyType::SettleEngaged);
}

// ---- RFC-084: a future-anchored chase point is an ARRIVAL --------------------
namespace {

// Phosphor ph-vdk.26's shape: a 1 Hz sine streamed as bare points every 20 ms,
// each stamped `lead_us` ahead of its send instant, the sends jittered by up to
// `jitter_us` (deterministic LCG, in-order like TCP). Commits exactly as the
// arbiter does: an anchor already due is not one. Reports the time shift that
// best fits the rendered position to the STAMPED curve, and the excursion.
struct LeadFit {
    double lag_s = 0.0;
    double hi = 0.0;
    double lo = 1.0;
};
LeadFit runLeadSine(const Config& cfg, uint64_t lead_us, uint64_t jitter_us) {
    Engine e(cfg, 0.5f);
    const double amp = 0.3;
    auto curve = [&](double t) { return 0.5 + amp * std::sin(2.0 * M_PI * t); };
    const uint64_t dt = 20 * kMs, t_end = 5 * kS;
    std::vector<uint64_t> stamp, sent;
    uint32_t lcg = 12345u;
    for (uint64_t k = 0; k * dt + lead_us < t_end; ++k) {
        lcg = lcg * 1664525u + 1013904223u;
        uint64_t a = k * dt + (jitter_us ? uint64_t(lcg >> 8) % jitter_us : 0);
        if (!sent.empty() && a < sent.back()) a = sent.back();
        sent.push_back(a);
        stamp.push_back(k * dt + lead_us);
    }
    std::vector<double> pos;
    size_t next = 0;
    for (uint64_t t = 0; t < t_end; t += kMs) {
        for (; next < sent.size() && sent[next] <= t; ++next) {
            Command c;
            c.target = (float)curve((double)stamp[next] * 1e-6);
            c.has_anchor = stamp[next] > t;
            c.anchor_us = stamp[next];
            REQUIRE(e.commit(c, t));
        }
        pos.push_back(e.positionAt(t));
    }
    LeadFit r;
    double best = 1e9;
    const size_t settled = 2000;   // two full periods of warm-up
    for (int lag_ms = -100; lag_ms <= 200; ++lag_ms) {
        double sse = 0.0;
        for (size_t i = settled; i < pos.size(); ++i) {
            const double d = pos[i] - curve((double)i * 1e-3 - lag_ms * 1e-3);
            sse += d * d;
        }
        if (sse < best) { best = sse; r.lag_s = lag_ms * 1e-3; }
    }
    for (size_t i = settled; i < pos.size(); ++i) {
        r.hi = std::max(r.hi, pos[i]);
        r.lo = std::min(r.lo, pos[i]);
    }
    return r;
}

}  // namespace

TEST_CASE("RFC-084: a led sine lags its stamps by the chase budget, never the lead, and does not overshoot") {
    // Factory chase tuning (what motionDefaultTuning ships) and the operator's
    // session tuning, both on the machine's ceilings.
    Config factory;
    factory.limits = liveTuning().limits;
    for (const Config& cfg : {factory, liveTuning()}) {
        // The budget schedule_latency_us states for a samples grant, less the
        // hub's hop: one dense interval (val-091.44).
        const double budget_s = (double)cfg.chase_dense_us * 1e-6;
        const LeadFit r = runLeadSine(cfg, 100 * kMs, 30 * kMs);
        CHECK(r.lag_s <= budget_s);
        CHECK(r.lag_s < 0.020);                 // a lead buys arrival, not delay
        CHECK(r.hi <= 0.8 + 0.01 * 0.3);        // within 1 % of the amplitude
        CHECK(r.lo >= 0.2 - 0.01 * 0.3);
        CHECK(r.hi - r.lo >= 0.6 * 0.98);       // and keeps the stroke
    }
}

TEST_CASE("RFC-084: points stamped at arrival (no lead) plan exactly as before") {
    // The arrival branch only engages on a future anchor; a client with no
    // lead must see the predictive-aim chase unchanged. Both figures were
    // measured on this harness before the arrival branch existed. The same
    // harness then read lag -5 ms / peak 0.8068 at a 100 ms lead with 30 ms
    // jitter (factory), and lag 28 ms (session tuning): what the case above
    // now refuses.
    Config cfg;
    cfg.limits = liveTuning().limits;
    const LeadFit r = runLeadSine(cfg, 0, 0);
    CHECK(r.lag_s == doctest::Approx(-0.005));
    CHECK(r.hi == doctest::Approx(0.806762).epsilon(1e-5));
}
