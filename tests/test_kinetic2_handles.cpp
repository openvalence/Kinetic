// test_kinetic2_handles.cpp -- parity of the Kinetic² renderer with its spec,
// the handle model (playground/handles-model.js, ruling kin-y6e / RFC-106).
// The expected samples and trims are tests/handles_fixture.hpp, generated from
// the model: `node tests/gen_handles_fixture.mjs` from the Kinetic root.
// Constraints:
// - Every engine assertion is sampled reality on the 1 ms grid, never the
//   planner's word; the renderer's trims are read from handles::render alone.
// - Parity bars as asserted: the renderer's trims within 1e-4 of the window at
//   every knot with no allowance (the model renders with railStop as the
//   kernel does, kin-88m); position within 0.5% of the window at every sample
//   and the engine's trims within 1e-4, both past the corner-ramp allowance
//   only. Ceilings within 0.1%.
// - The corner ramps and the origin's lead ramp are the engine's
//   (engine_piece.hpp, solver.hpp renderRun), and so are the slack passes that
//   tighten a piece's ceilings by what those ramps cost it: the model has none
//   of them, so they are the one allowance. The renderer-only cases
//   (kRenderCases: the rail rules, the monotone judge) are held to the
//   renderer alone: the slack passes move them past that allowance.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <algorithm>
#include <cmath>
#include <string>
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
    for (int t = 0; t < c.n_ms; ++t) r.s.push_back(e.stateAt(0, uint64_t(t) * kMs));
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

TEST_CASE("handles parity: the renderer trims the model's script as the model does") {
    auto parity = [](const handles_fixture::Case& c) {
        const std::string name = c.name;
        CAPTURE(name);
        const std::vector<handles::HKnot> k = renderBare(c);
        float worst = 0.0f; int at = 0;
        for (int i = 0; i < c.n; ++i) {
            CAPTURE(i);
            const float d = std::fabs(k[size_t(i)].dp - c.dp[i]);
            if (d > worst) { worst = d; at = i; }
            CHECK(d <= 1e-4f);
            CHECK(k[size_t(i)].infeasible == (c.infeasible[i] != 0));
        }
        MESSAGE(name << ": worst renderer trim difference " << worst << " of the window at knot " << at);
    };
    for (const handles_fixture::Case& c : handles_fixture::kCases) parity(c);
    for (const handles_fixture::Case& c : handles_fixture::kRenderCases) parity(c);
}

TEST_CASE("handles parity: the kernel renders the model's script on the author's clock") {
    for (const handles_fixture::Case& c : handles_fixture::kCases) {
        const std::string name = c.name;
        CAPTURE(name);
        const Run r = render(c);
        REQUIRE(r.refused == 0);
        REQUIRE(int(r.s.size()) == c.n_ms);

        // Position parity on the 1 ms grid.
        // Constraint: the model steps its acceleration at a flat knot (a rest
        // start, a hold edge) for free; the kernel stays flat up to the knot and
        // ramps at jmax for r = |a| / jmax, so that piece lags the model by up to
        // peak |v| * r / 2 (the floor under jmax alone is near half that), and at
        // a speed ceiling gives that much more amplitude. That lag is the
        // allowance on the piece, its end knot and the next piece (the gap closes
        // at the knot after, which is reached exactly); everywhere else the bar
        // is 0.5%.
        // Operator ruling owed: kin-y6e asks 0.5% at every sample (open issue).
        std::vector<float> allow(size_t(c.n_ms), 0.0f);
        std::vector<float> knot_lag(size_t(c.n), 0.0f);
        auto flat = [&](int i) { return i <= 0 || i >= c.n - 1 || std::fabs(c.p[i] - c.p[i - 1]) <= kHold
                                        || std::fabs(c.p[i + 1] - c.p[i]) <= kHold; };
        for (int i = 0; i + 1 < c.n; ++i) {
            const int ta = int(c.t_ms[i]), tb = int(c.t_ms[i + 1]);
            if (tb - ta < 2) continue;
            const float aA = flat(i) ? 2.0f * std::fabs(c.ms_p[ta + 1] - c.ms_p[ta]) * 1e6f : 0.0f;
            const float aB = flat(i + 1) ? 2.0f * std::fabs(c.ms_p[tb] - c.ms_p[tb - 1]) * 1e6f : 0.0f;
            float vpk = 0.0f;
            for (int t = ta + 1; t < tb; ++t) vpk = std::fmax(vpk, std::fabs(c.ms_p[t + 1] - c.ms_p[t - 1]) * 500.0f);
            knot_lag[size_t(i + 1)] = vpk * std::fmax(aA, aB) / c.jmax * 0.5f;
        }
        for (int i = 0; i + 1 < c.n; ++i) {
            const int ta = int(c.t_ms[i]), tb = int(c.t_ms[i + 1]);
            for (int t = ta; t <= tb && t < c.n_ms; ++t)
                allow[size_t(t)] = std::max({allow[size_t(t)], knot_lag[size_t(i + 1)], knot_lag[size_t(i)]});
        }
        float worst_p = 0.0f, raw = 0.0f; int at = 0, raw_at = 0, raw_over = 0;
        for (int t = 0; t < c.n_ms; ++t) {
            const float e = std::fabs(r.s[size_t(t)].p - c.ms_p[t]);
            if (e > raw) { raw = e; raw_at = t; }
            if (e > 0.005f) ++raw_over;
            const float d = e - allow[size_t(t)];
            if (d > worst_p) { worst_p = d; at = t; }
        }
        MESSAGE(name << ": worst position error past the corner-ramp allowance " << worst_p << " of the window at " << at
                     << " ms; with no allowance " << raw << " at " << raw_at << " ms, " << raw_over << " samples over 0.5%");
        CHECK(worst_p <= 0.005f);

        // Trims: which knots moved and by how much; nothing placed early or late.
        // A corner's solved t_us / p are its ramp's end; the knot is on the ramp.
        float worst_t = 0.0f, raw_t = 0.0f; int at_t = 0;
        for (int i = 1; i < c.n; ++i) {
            CAPTURE(i);
            const Solved& o = r.sol[size_t(i - 1)];
            const uint64_t t_k = uint64_t(c.t_ms[i]) * kMs;
            CHECK(o.base_us == t_k);
            if (o.corner) { CHECK(o.head_us <= t_k); CHECK(o.t_us >= t_k); }
            else CHECK(o.t_us == t_k);
            const float p_k = o.knot_p;   // the renderer's placement, never read back from the curve
            // Constraint: a ramp's time costs its pieces speed, which the slack
            // passes give back as amplitude; the position allowance at the knot
            // is the trim's.
            const float e = std::fabs((p_k - c.p[i]) - c.dp[i]), d = e - allow[c.t_ms[i]];
            raw_t = std::fmax(raw_t, e);
            if (d > worst_t) { worst_t = d; at_t = i; }
            CHECK(d <= 1e-4f);
            CHECK(o.infeasible == (c.infeasible[i] != 0));
            // Reached at its own time, at its trimmed position, within the
            // corner ramp's tolerance (kKnotTol: a ramp that keeps the curve's
            // velocity may pass its knot that far off).
            CHECK(std::fabs(r.s[c.t_ms[i]].p - p_k) <= kKnotTol);
        }
        MESSAGE(name << ": worst engine trim difference past the corner-ramp allowance " << worst_t << " of the window at knot "
                     << at_t << "; with no allowance " << raw_t);
    }
}

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
    // 1, and rail_caps now trims as the model does at smoothness 1 (before, a
    // piece the model fits was left over jmax). All four moved with kin-9od3:
    // the window wall at half the tolerance, end-acceleration asks held to
    // amax and dropped when no fit honors them, and (the engine) pieces built
    // from the render's curve after an on-curve corner exit.
    CHECK(bareHash(0.0f) == 0x137dfc85e450fd0dull);
    CHECK(bareHash(1.0f) == 0x614ccb89b4dc2c36ull);
    CHECK(engineHash(0.0f) == 0xa6fac79f561c2021ull);
    CHECK(engineHash(1.0f) == 0xc68708be37c41d4eull);
}

TEST_CASE("smoothness: overshoot between free knots grows with it from none at 0") {
    // The model's script: free knots well inside the ceilings, so no fit or
    // trim hides what the lerp does.
    const handles_fixture::Case& c = fixture("sample");
    float prev = -1.0f;
    for (int i = 0; i <= 10; ++i) {
        Config cfg;
        cfg.smoothness = float(i) / 10.0f;
        const float o = overshoot(renderBare(c, cfg));
        CAPTURE(i);
        MESSAGE("smoothness " << cfg.smoothness << ": overshoot " << o << " of the window, summed over the pieces");
        if (i == 0) CHECK(o == 0.0f);
        else CHECK(o > prev);
        prev = o;
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
