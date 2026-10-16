// test_kinetic2_handles.cpp -- the handle renderer (Valence RFC-106, kin-y6e)
// on its fixture scripts (tests/handles_fixture.hpp): ceilings, continuity and
// flat holds in sampled reality, the two styles bit for bit, the smoothness,
// handle_floor and trim_max bounds.
// Constraints:
// - Every engine assertion is sampled reality on the 1 ms grid, never the
//   planner's word.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "kinetic2/engine.hpp"
#include "handles_fixture.hpp"

using namespace kinetic2;

namespace {

constexpr uint64_t kMs = 1000;
constexpr float kHold = 0.005f;   // a chord this small is a hold (the kernel's kHoldEps)

struct Run {
    std::vector<State> s;      // every 1 ms, 0..last knot
    std::vector<Solved> sol;   // the solved window before the first sample
    int refused = 0;
};

// The case authored ahead: seeded at the first knot, the rest submitted at 0.
Run render(const handles_fixture::Case& c) {
    Config cfg;
    cfg.limits = {c.vmax, c.amax, c.jmax};
    Engine<> e(cfg, c.p[0]);
    Run r;
    for (int i = 1; i < c.n; ++i) {
        Knot k; k.t_us = uint64_t(c.t_ms[i]) * kMs; k.p = c.p[i];
        if (!e.submit(k, 0)) ++r.refused;
    }
    for (int i = 0; i + 1 < c.n; ++i) r.sol.push_back(e.solved(0, size_t(i)));
    for (unsigned t = 0; t <= c.t_ms[c.n - 1]; ++t) r.s.push_back(e.stateAt(0, uint64_t(t) * kMs));
    return r;
}

// The renderer alone, with the knobs solveWindow gives it from cfg (kept in
// step with solver.hpp by hand), from the first knot at rest.
std::vector<handles::HKnot> renderBare(const handles_fixture::Case& c, const Config& cfg = Config{}) {
    std::vector<handles::HKnot> k(size_t(c.n));
    k[0].p = c.p[0]; k[0].has_v = true;
    for (int i = 1; i < c.n; ++i) { k[size_t(i)].t = float(c.t_ms[i]) * 1e-3f; k[size_t(i)].p = c.p[i]; }
    handles::Cfg hc;
    hc.lim = {c.vmax, c.amax, c.jmax};
    hc.holdEps = kHoldEps; hc.lfloor = cfg.handle_floor; hc.trim = cfg.trim_max * (hc.hi - hc.lo); hc.smoothness = cfg.smoothness;
    hc.trimLast = true; hc.railStop = true;
    handles::render(k.data(), c.n, hc);
    return k;
}

}  // namespace

TEST_CASE("handles invariants: ceilings, continuity and flat holds in sampled reality") {
    for (const handles_fixture::Case& c : handles_fixture::kCases) {
        const std::string name = c.name;
        CAPTURE(name);
        const Run r = render(c);
        float v = 0, a = 0, j = 0, lo = 1e9f, hi = -1e9f, jump_p = -1e9f, jump_v = -1e9f;
        for (size_t t = 0; t < r.s.size(); ++t) {
            const State& s = r.s[t];
            v = std::max(v, std::fabs(s.v)); a = std::max(a, std::fabs(s.a));
            lo = std::min(lo, s.p); hi = std::max(hi, s.p);
            if (!t) continue;
            const State& q = r.s[t - 1];
            j = std::max(j, std::fabs(s.a - q.a) / 1e-3f);
            jump_p = std::max(jump_p, std::fabs(s.p - q.p) - (std::max(std::fabs(s.v), std::fabs(q.v)) * 1e-3f + 2e-4f));
            jump_v = std::max(jump_v, std::fabs(s.v - q.v) - (c.amax * 1e-3f * 1.001f + 1e-5f));
        }
        MESSAGE(name << ": peaks v " << v / c.vmax << " a " << a / c.amax << " j " << j / c.jmax << " of the ceilings");
        CHECK(v <= c.vmax * 1.001f);
        CHECK(a <= c.amax * 1.001f);
        CHECK(j <= c.jmax * 1.001f);    // finite difference over 1 ms: never above the peak
        CHECK(lo >= -1e-3f); CHECK(hi <= 1.0f + 1e-3f);
        CHECK(jump_p <= 0.0f);
        CHECK(jump_v <= 0.0f);
        // Every hold flat at its (trimmed) height, from knot to knot.
        for (int i = 0; i + 1 < c.n; ++i) {
            if (std::fabs(c.p[i + 1] - c.p[i]) > kHold) continue;
            CAPTURE(i);
            const float h = i ? r.sol[size_t(i - 1)].p : c.p[0];
            float bulge = 0.0f;
            for (unsigned t = c.t_ms[i]; t <= c.t_ms[i + 1]; ++t) bulge = std::max(bulge, std::fabs(r.s[t].p - h));
            CHECK(bulge <= 1e-4f);
        }
    }
}

// ---- smoothness, handle_floor, trim_max (Valence RFC-108, kin-tnv) ------------

namespace {

// FNV-1a over raw bytes.
struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void mix(const void* p, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    }
    void f(float x) { mix(&x, sizeof x); }
};

// Every solved field of the bare render of every fixture case.
uint64_t bareHash(float smoothness) {
    Config cfg;
    cfg.smoothness = smoothness;
    Fnv h;
    auto one = [&](const handles_fixture::Case& c) {
        for (const handles::HKnot& m : renderBare(c, cfg)) {
            h.f(m.vel); h.f(m.lIn); h.f(m.lOut); h.f(m.effIn); h.f(m.effOut); h.f(m.dp);
            const unsigned char b = m.infeasible;
            h.mix(&b, 1);
        }
    };
    for (const handles_fixture::Case& c : handles_fixture::kCases) one(c);
    for (const handles_fixture::Case& c : handles_fixture::kRenderCases) one(c);
    return h.h;
}

// The engine's state every 1 ms over every fixture case, authored ahead and
// streamed 250 ms ahead of each knot's predecessor.
uint64_t engineHash(float smoothness) {
    Fnv h;
    for (const handles_fixture::Case& c : handles_fixture::kCases) {
        for (const bool stream : {false, true}) {
            Config cfg;
            cfg.limits = {c.vmax, c.amax, c.jmax};
            cfg.smoothness = smoothness;
            Engine<> e(cfg, c.p[0]);
            int next = 1;
            auto submit = [&](uint64_t now) { Knot k; k.t_us = uint64_t(c.t_ms[next]) * kMs; k.p = c.p[next]; (void)e.submit(k, now); ++next; };
            if (!stream) while (next < c.n) submit(0);
            const uint64_t end = uint64_t(c.t_ms[c.n - 1]) * kMs + 200 * kMs;
            for (uint64_t t = 0; t <= end; t += kMs) {
                while (next < c.n && uint64_t(c.t_ms[next - 1]) * kMs <= t + 250 * kMs) submit(t);
                const State s = e.stateAt(0, t);
                h.f(s.p); h.f(s.v); h.f(s.a);
            }
        }
    }
    return h.h;
}

const handles_fixture::Case& fixture(const char* name) {
    for (const handles_fixture::Case& c : handles_fixture::kCases)
        if (std::string(c.name) == name) return c;
    FAIL("no fixture " << name);
    return handles_fixture::kCases[0];
}

// The most each piece of a bare render passes its end knots, summed.
float overshoot(const std::vector<handles::HKnot>& k) {
    float sum = 0.0f;
    for (size_t i = 0; i + 1 < k.size(); ++i) {
        const handles::Piece q = handles::pieceOf(k.data(), int(i));
        const float lo = std::fmin(0.0f, q.D), hi = std::fmax(0.0f, q.D);
        float ex = 0.0f;
        for (int s = 0; s <= 400; ++s) {
            const float p = handles::evalPiece(q, float(s) / 400.0f).p;
            ex = std::fmax(ex, std::fmax(lo - p, p - hi));
        }
        sum += ex;
    }
    return sum;
}

}  // namespace

TEST_CASE("smoothness 0 and 1 render the pchip and smooth styles bit for bit") {
    // Captured 2026-10-08 at Kinetic 7f7330d, before smoothness replaced the
    // style: handles::Cfg::style Pchip and Smooth for the bare render, the
    // solver's style constant Pchip and Smooth for the engine. The bare smooth
    // hash moved with kin-rfw7: a smooth solve's lengths walk both sides of
    // 1, and a rail_caps piece left over jmax at smoothness 1 now fits. All
    // four moved with kin-9od3:
    // the window wall at half the tolerance, end-acceleration asks held to
    // amax and dropped when no fit honors them, and (the engine) pieces built
    // from the render's curve after an on-curve corner exit. Both smooth
    // hashes moved with kin-4o6r (0x614ccb89b4dc2c36 -> 0xc99cb88b53ab265c,
    // 0xc68708be37c41d4e -> 0xf809a2cf0639093e): a crest under smooth is flat
    // where it took Makima's angle; pchip is unchanged.
    CHECK(bareHash(0.0f) == 0x137dfc85e450fd0dull);
    CHECK(bareHash(1.0f) == 0xc99cb88b53ab265cull);
    CHECK(engineHash(0.0f) == 0xa6fac79f561c2021ull);
    CHECK(engineHash(1.0f) == 0xf809a2cf0639093eull);
}

TEST_CASE("smoothness: crests and hold edges are flat at every value; the lengths change, not the extrema") {
    // The built-in script: free knots well inside the ceilings, so no fit or
    // trim hides what the lerp does. Its through points stay inside their
    // monotone band, so nothing overshoots (kin-4o6r).
    const handles_fixture::Case& c = fixture("sample");
    const std::vector<handles::HKnot> crisp = renderBare(c);
    for (int i = 0; i <= 10; ++i) {
        Config cfg;
        cfg.smoothness = float(i) / 10.0f;
        const std::vector<handles::HKnot> k = renderBare(c, cfg);
        CAPTURE(i);
        CHECK(overshoot(k) == 0.0f);
        bool moved = false;
        for (size_t j = 1; j + 1 < k.size(); ++j) {
            if (k[j].cls == handles::KnotClass::Crest || k[j].cls == handles::KnotClass::Rest) CHECK(k[j].vel == 0.0f);
            moved = moved || k[j].effIn != crisp[j].effIn || k[j].effOut != crisp[j].effOut || k[j].vel != crisp[j].vel;
        }
        CHECK(moved == (i > 0));
    }
}

// ---- the operator's trough (kin-4o6r) -------------------------------------------

namespace {

// A sine-like script of free knots at 300 ms spans: a crest, then a trough of
// two knots 40 ms apart (lo, then lo2), 640 ms a cycle. Streamed 125 ms ahead
// of each knot's predecessor; with expect, each submit first sets the horizon
// 500 ms on. Returns every 1 ms state.
std::vector<State> troughRun(float lo, float lo2, float smoothness, bool expect, std::vector<Knot>& ks) {
    ks.clear();
    for (int cyc = 0; cyc < 6; ++cyc) {
        const uint64_t t = 500 * kMs + uint64_t(cyc) * 640 * kMs;
        for (const std::pair<uint64_t, float>& dp : {std::pair<uint64_t, float>{0, 0.9f}, {300, lo}, {340, lo2}}) {
            Knot k; k.t_us = t + dp.first * kMs; k.p = dp.second; ks.push_back(k);
        }
    }
    Config cfg;
    cfg.limits = {6.0f, 200.0f, 20000.0f};
    cfg.smoothness = smoothness;
    Engine<> e(cfg, ks[0].p);
    std::vector<State> s;
    size_t next = 1;
    for (uint64_t t = 0; t < ks.back().t_us + 600 * kMs; t += kMs) {
        while (next < ks.size() && ks[next - 1].t_us <= t + 125 * kMs) {
            if (expect) e.expect(t + 500 * kMs);
            REQUIRE(e.submit(ks[next++], t));
        }
        s.push_back(e.stateAt(0, t));
    }
    return s;
}

}  // namespace

TEST_CASE("smooth: a crest and a hold-edge trough are passed flat, never past (kin-4o6r)") {
    // Before the ruling the smooth style gave a crest Makima's angle: the
    // step pair's lower knot was passed 0.0015 of the window past at
    // smoothness 1 (0.0043 with expect, and the crest 0.0130), its slope 0.37
    // of the window per second.
    for (const float lo : {0.1f, 0.13f}) {   // a hold pair; the operator's step pair
        for (const float sm : {0.5f, 1.0f}) {
            for (const bool expect : {false, true}) {
                std::vector<Knot> ks;
                const std::vector<State> s = troughRun(lo, 0.1f, sm, expect, ks);
                CAPTURE(lo); CAPTURE(sm); CAPTURE(expect);
                float past = 0.0f, slope = 0.0f;
                // Interior cycles: the launch from rest and the last knot excluded.
                for (size_t i = 1; i + 1 < ks.size(); ++i) {
                    const size_t at = size_t(ks[i].t_us / kMs);
                    const bool crest = i % 3 == 0, low = i % 3 == 2;
                    if (crest) for (size_t j = at - 150; j <= at + 150; ++j) past = std::fmax(past, s[j].p - ks[i].p);
                    if (low) for (size_t j = at - 190; j <= at + 150; ++j) past = std::fmax(past, ks[i].p - s[j].p);
                    if (crest || low) slope = std::fmax(slope, std::fabs(s[at].v));
                }
                MESSAGE("trough " << lo << "/0.1, smoothness " << sm << std::string(expect ? ", expect" : "") << ": past a crest or trough knot " << past << ", slope there " << slope);
                // 0.2 % of the window.
                CHECK(past <= 0.002f);
                // Zero in the render (above); the engine's corner ramp passes
                // the knot within a thousandth of vmax.
                CHECK(slope <= 6e-3f);
            }
        }
    }
}

TEST_CASE("trim_max bounds every trim; a piece it leaves over a ceiling is reported") {
    const handles_fixture::Case& c = fixture("figure_tight");
    auto worst = [&](const Config& cfg, int& infeasible) {
        float m = 0.0f;
        infeasible = 0;
        for (const handles::HKnot& k : renderBare(c, cfg)) { m = std::fmax(m, std::fabs(k.dp)); infeasible += k.infeasible; }
        return m;
    };
    Config cfg;
    int bad = 0;
    const float whole = worst(cfg, bad);
    CHECK(whole > 0.1f);   // the default trims farther than the bound below
    CHECK(bad == 0);
    cfg.trim_max = 0.1f;
    const float bounded = worst(cfg, bad);
    MESSAGE("trim_max 1: worst trim " << whole << "; 0.1: " << bounded << ", " << bad << " pieces over a ceiling");
    CHECK(bounded <= 0.1f + 1e-6f);
    CHECK(bad > 0);
    // Through the engine: the knots stay within the bound and the pieces over
    // a ceiling are reported.
    cfg.limits = {c.vmax, c.amax, c.jmax};
    Engine<> e(cfg, c.p[0]);
    for (int i = 1; i < c.n; ++i) { Knot k; k.t_us = uint64_t(c.t_ms[i]) * kMs; k.p = c.p[i]; REQUIRE(e.submit(k, 0)); }
    for (int i = 1; i < c.n; ++i) CHECK(std::fabs(e.solved(0, size_t(i - 1)).knot_p - c.p[i]) <= 0.1f + kKnotTol);
    int over = 0;
    Anomaly a;
    while (e.popAnomaly(a)) over += a.kind == uint8_t(AnomalyKind::PieceOverCeiling);
    CHECK(over > 0);
}

TEST_CASE("handle_floor: no fitted handle is shorter") {
    const handles_fixture::Case& c = fixture("figure");
    auto shortest = [&](const Config& cfg) {
        const std::vector<handles::HKnot> k = renderBare(c, cfg);
        float m = 1.0f;
        for (size_t i = 0; i + 1 < k.size(); ++i) m = std::fmin(m, std::fmin(k[i].effOut, k[i + 1].effIn));
        return m;
    };
    Config cfg;
    const float dflt = shortest(cfg);
    CHECK(dflt < 0.3f);   // the default fit goes shorter than the floor below
    cfg.handle_floor = 0.3f;
    const float floored = shortest(cfg);
    MESSAGE("shortest handle at the default floor " << dflt << ", at 0.3: " << floored);
    CHECK(floored >= 0.3f - 1e-6f);
}
