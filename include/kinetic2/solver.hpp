// kinetic2/solver.hpp -- the lookahead solver: junction values for every
// pending knot, a referee that finds every extremum of a piece, and the spend
// (Blend or Stretch) when the ceilings cannot honor a knot in its time
// Constraints:
// - Solved in time order (RFC-105 promise 3): a knot is spent against the
//   state its predecessor actually ends in, so a spend never cascades
//   silently; it is counted once per axis spent. A re-solve starts at the
//   first knot whose inputs changed (Engine::kResolveDepth); a knot's junction
//   sees at most jerk::kChainKnots knots ahead; a solve stops at the budget
//   (Config::solve_budget) and the knots it did not reach wait for the next.
// - Every spend is found from the referee's ratio (the shape of a trim scales
//   with the share kept, the ceilings with 1/T, 1/T^2, 1/T^3), judged and
//   refined, never by a blind bisection (operator ruling 2026-10-06).
// - Junction velocity: authored when the knot carries one, else the monotone
//   (Fritsch-Butland) slope through its neighbors, so a free knot sequence
//   never overshoots between two knots. Junction acceleration: continuous,
//   the centered difference of the junction velocities; 0 at a hard stop and
//   at the last knot. Promise 2's HARD tail (the brake profile) lands with
//   kin-4gd; until then a hard stop is the smooth stop with v = a = 0.
// - The referee is Kinetic 1's method: every extremum of p, v, a, j on a piece
//   sits at a root of its own derivative, found by bisection inside monotone
//   intervals; a fixed grid steps over a narrow peak and that is a safety
//   hole. Float throughout (the P4 has no double FPU).
// - Zero ceiling = no authority: nothing is legal under it.
#pragma once

#include <algorithm>
#include <cmath>
#ifdef K2_TRACE
#include <cstdio>
#endif
#include <cstddef>
#include <cstdint>

#include "engine_piece.hpp"
#include "types.hpp"

namespace kinetic2 {

// ---- cost counters (tests/bench_kinetic2.cpp) ----------------------------------
// Compiled in only under KINETIC2_STATS; never in firmware. Not thread-safe.
#ifdef KINETIC2_STATS
namespace stats {
struct Counters {
    uint64_t windows = 0;      // solveWindow calls
    uint64_t knots = 0;        // knots solved (solveKnot calls)
    uint64_t judges = 0;       // judgeOnce: one junction fill plus one referee
    uint64_t junctions = 0;    // junction fills
    uint64_t solves = 0;       // jerk::solve calls
    uint64_t chain_knots = 0;  // knots across every jerk::solve (its cost is linear in them)
    uint64_t extrema = 0;      // referee::extremumTaus calls (the referee's cost)
    uint64_t work = 0;         // budget units charged (Config::solve_budget)
};
inline Counters g{};
}  // namespace stats
#define K2_STAT(field, n) (::kinetic2::stats::g.field += (n))
#else
#define K2_STAT(field, n) ((void)0)
#endif

// ---- the referee -------------------------------------------------------------
namespace referee {

inline constexpr float kIllegal = 1e30f;

inline float polyAt(const float* k, int deg, float t) {
    float r = k[deg];
    for (int i = deg - 1; i >= 0; --i) r = r * t + k[i];
    return r;
}

// Bisection steps per root. An extremum's value is second order in the error
// of its abscissa, so 2^-14 of the piece is far below float resolution of the
// peak; twenty steps cost the P4 a third more for nothing (kin-ys0).
inline constexpr int kRootSteps = 14;

// Roots of k (degree deg) in (0,1), given brk: the ascending roots of its
// derivative, which cut [0,1] into intervals it is monotone on.
inline int rootsIn01(const float* k, int deg, const float* brk, int nbrk, float* out) {
    float x[6];
    int n = 0;
    x[n++] = 0.0f;
    for (int i = 0; i < nbrk && n < 5; ++i) x[n++] = brk[i];
    x[n++] = 1.0f;
    int found = 0;
    float fa = polyAt(k, deg, x[0]);
    for (int i = 0; i + 1 < n; ++i) {
        float lo = x[i], hi = x[i + 1];
        const float fb = polyAt(k, deg, hi);
        if (hi > lo && fa != 0.0f && fb != 0.0f && ((fa < 0.0f) != (fb < 0.0f))) {
            const bool neg_lo = fa < 0.0f;
            for (int s = 0; s < kRootSteps; ++s) {
                const float m = 0.5f * (lo + hi);
                if ((polyAt(k, deg, m) < 0.0f) == neg_lo) lo = m; else hi = m;
            }
            out[found++] = 0.5f * (lo + hi);
        }
        fa = fb;
    }
    return found;
}

// Roots in (0,1) of k0 + k1 t + k2 t^2, ascending, in closed form (the
// cancellation-free pair q / k2, k0 / q).
inline int quadraticIn01(const float* k, float* out) {
    int n = 0;
    float r[2];
    if (std::fabs(k[2]) <= 1e-30f) {
        if (std::fabs(k[1]) <= 1e-30f) return 0;
        r[n++] = -k[0] / k[1];
    } else {
        const float disc = k[1] * k[1] - 4.0f * k[2] * k[0];
        if (!(disc > 0.0f)) return 0;
        const float q = -0.5f * (k[1] + (k[1] < 0.0f ? -1.0f : 1.0f) * std::sqrt(disc));
        r[n++] = q / k[2];
        if (q != 0.0f) r[n++] = k[0] / q;
    }
    if (n == 2 && r[1] < r[0]) { const float t = r[0]; r[0] = r[1]; r[1] = t; }
    int m = 0;
    for (int i = 0; i < n; ++i) if (r[i] > 0.0f && r[i] < 1.0f) out[m++] = r[i];
    return m;
}

// 0, 1 and every extremum of p, v, a, j of the tau-polynomial c; at most 12.
inline int extremumTaus(const float* c, float* taus) {
    K2_STAT(extrema, 1);
    int n = 0;
    taus[n++] = 0.0f;
    taus[n++] = 1.0f;
    float rj1[1]; int nj1 = 0;
    if (std::fabs(c[5]) > 1e-30f) {
        const float t = -c[4] / (5.0f * c[5]);
        if (t > 0.0f && t < 1.0f) rj1[nj1++] = t;
    }
    const float kj[3] = {6.0f * c[3], 24.0f * c[4], 60.0f * c[5]};
    float rj[2]; const int nj = quadraticIn01(kj, rj);
    const float ka[4] = {2.0f * c[2], 6.0f * c[3], 12.0f * c[4], 20.0f * c[5]};
    float ra[3]; const int na = rootsIn01(ka, 3, rj, nj, ra);
    const float kv[5] = {c[1], 2.0f * c[2], 3.0f * c[3], 4.0f * c[4], 5.0f * c[5]};
    float rv[4]; const int nv = rootsIn01(kv, 4, ra, na, rv);
    for (int i = 0; i < nj1; ++i) taus[n++] = rj1[i];
    for (int i = 0; i < nj;  ++i) taus[n++] = rj[i];
    for (int i = 0; i < na;  ++i) taus[n++] = ra[i];
    for (int i = 0; i < nv;  ++i) taus[n++] = rv[i];
    return n;
}

// Worst (peak / ceiling) ratio of a piece across v, a, j and the window
// [lo, hi]; > 1 is illegal. A window excursion scores kIllegal: the rail is a
// wall, not a ceiling with a ratio.
// Each ceiling's own peak ratio; all three kIllegal for a window excursion.
struct Ratios { float v = 0.0f, a = 0.0f, j = 0.0f; };

inline float worstRatio(const Piece& q, const Limits& L, float lo, float hi, Ratios* parts = nullptr) {
    if (parts) *parts = Ratios{};
    if (!(q.T > 0.0f)) return 0.0f;   // a hold is legal
    if (parts) *parts = Ratios{kIllegal, kIllegal, kIllegal};
    if (!(L.vmax > 0.0f) || !(L.amax > 0.0f) || !(L.jmax > 0.0f)) return kIllegal;
    const float rT = 1.0f / q.T, rT2 = rT * rT, rT3 = rT2 * rT;
    const float rv = 1.0f / L.vmax, ra = 1.0f / L.amax, rj = 1.0f / L.jmax;
    float taus[12];
    const int n = extremumTaus(q.c, taus);
    float wv = 0.0f, wa = 0.0f, wj = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float t = taus[i], t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        const float* c = q.c;
        const float p = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
        if (p < lo - 1e-6f || p > hi + 1e-6f) return kIllegal;
        const float v = (c[1] + 2.0f * c[2] * t + 3.0f * c[3] * t2 + 4.0f * c[4] * t3 + 5.0f * c[5] * t4) * rT;
        const float a = (2.0f * c[2] + 6.0f * c[3] * t + 12.0f * c[4] * t2 + 20.0f * c[5] * t3) * rT2;
        const float j = (6.0f * c[3] + 24.0f * c[4] * t + 60.0f * c[5] * t2) * rT3;
        wv = std::fmax(wv, std::fabs(v) * rv);
        wa = std::fmax(wa, std::fabs(a) * ra);
        wj = std::fmax(wj, std::fabs(j) * rj);
    }
    if (parts) *parts = Ratios{wv, wa, wj};
    return std::fmax(wv, std::fmax(wa, wj));
}

}  // namespace referee

// ---- the solved knot ---------------------------------------------------------
// What the solver decided for one knot: where and when the curve really passes,
// and with what velocity and acceleration. p and t start as the knot's own and
// move only by a spend.
// What a knot keeps from the previous solve: its time and its junction
// velocity and acceleration. A sample reached at its prior time with its
// prior junction is the committed curve continued, exactly.
struct Prior {
    uint64_t t_us = 0;   // 0: solved for the first time
    float    v = 0.0f;
    float    a = 0.0f;
};

struct Solved {
    uint64_t t_us = 0;
    float    p = 0.0f, v = 0.0f, a = 0.0f;
    float    share = 1.0f;       // Blend: the share of the stroke kept
    float    stretched_s = 0.0f; // Stretch: seconds added
    float    worst = 0.0f;       // the incoming piece's worst ratio after the spend
    bool     clamped = false;    // an authored velocity was cut (reported once)
    uint64_t base_us = 0;        // Stretch scratch: the time before the current spend
    bool     pin_v = false, pin_a = false;   // junction values fixed by a backward relaxation
    // Unreachable under every spend: not rendered, reported PlanFailed; the
    // engine removes it from the timeline. Promise 3 outranks the knot.
    bool     dropped = false;
    // HARD junction: `ramp` is the whole move, Profile::point from the state
    // before the knot, landing at rest on it at t_us (a hold first when it
    // launches from rest with time to spare). CORNER (Corner::Cubic): the
    // head ends at the ramp's start and `ramp` carries the jerk-limited step;
    // t_us / p / v / a are then the ramp's END, where the next piece starts.
    bool     hard = false;
    bool     corner = false;
    uint64_t head_us = 0;
    State    head{};
    Profile  ramp{};
    // The state the piece into this knot was judged from (tooling, and the
    // engine's consistency check: it must equal the previous solved knot).
    State    from{};
    uint64_t from_us = 0;
    // The time every later SEGMENT knot is moved by: the Stretch added at
    // this knot and at every segment before it in the window. A re-solve
    // that starts after this knot starts its segments that much late.
    uint64_t shift_us = 0;
    uint64_t own_us = 0;    // the Stretch this knot spent itself
};

/// ---- minimum-jerk junctions ----------------------------------------------------
// The free junction velocities and accelerations of a chain of quintic pieces
// through fixed knot positions are the ones that minimize the chain's jerk
// energy: a quadratic in the unknowns, one banded linear solve. Local
// estimates (monotone slopes, centered-difference accelerations) were tried
// first and whipped a 60 Hz stream into reversals under re-planning; the
// global minimum never amplifies (RFC-105 workflow (aa)).
// Float (the P4 has no double FPU; a double here was software float and cost
// a 283 ms solve, kin-ys0). The weights scale as 1 / T^5, so the system is
// scaled to a unit diagonal before elimination: a 1 ms piece beside an 8 s one
// spans 1e19 in weight, inside float's range, and the scaling keeps that
// spread out of the pivots. Intervals come from integer microseconds, never
// from a difference of two float times.
namespace jerk {

// The knots a junction solve sees from the knot it decides, so a solve's cost
// is bounded whatever the window holds. A knot's pull on a minimum-jerk
// junction shrinks by about 2.4 per knot between them: cut at 8 knots, the
// decided velocity moves by 0.005 units/s at the median and 0.26 at worst
// (random chains of 20 to 300 ms pieces; 16 knots: 4e-6 and 3e-4), and the
// property suite's hits, spends and drops do not change. Only a bundle or a
// full re-solve reaches the cut; a stream solves one or two knots. The last
// of them ends the chain free, as the knots past it would leave it.
inline constexpr size_t kChainKnots = 8;
inline constexpr size_t kMaxUnknowns = 2 * kChainKnots;
inline constexpr int kHalfBand = 4;   // a knot's two unknowns couple with its neighbors' two

// Jerk energy of one quintic Hermite piece as a quadratic form in
// x = [p0, v0, a0, p1, v1, a1] (real units): E = x^T K x, with
// K[r][c] = kUnit[r][c] * T^(e_r + e_c - 5), e = {0, 1, 2, 0, 1, 2}: the
// T = 1 form (M^T G M over the coefficients c3..c5 and the Gram matrix of the
// jerk basis 6, 24 tau, 60 tau^2 on 0..1), scaled by the powers of T each
// column carries.
struct UnitEnergy {
    float k[6][6];
    constexpr UnitEnergy() : k{} {
        const double M[3][6] = {
            {-10.0, -6.0, -1.5, 10.0, -4.0, 0.5},
            {15.0, 8.0, 1.5, -15.0, 7.0, -1.0},
            {-6.0, -3.0, -0.5, 6.0, -3.0, 0.5},
        };
        const double G[3][3] = {{36.0, 72.0, 120.0}, {72.0, 192.0, 360.0}, {120.0, 360.0, 720.0}};
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c) {
                double acc = 0.0;
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) acc += M[i][r] * G[i][j] * M[j][c];
                k[r][c] = float(acc);
            }
    }
};
inline constexpr UnitEnergy kUnit{};
inline constexpr int kPow[6] = {0, 1, 2, 0, 1, 2};

inline void pieceEnergy(float T, float K[6][6]) {
    const float r1 = 1.0f / T, r2 = r1 * r1, r3 = r2 * r1, r4 = r2 * r2, r5 = r4 * r1;
    const float w[5] = {r5, r4, r3, r2, r1};   // T^(q - 5), q = e_r + e_c
    for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c) K[r][c] = K[c][r] = kUnit.k[r][c] * w[kPow[r] + kPow[c]];
}

// Knots 0..n-1 after the origin; T[k] is the piece into knot k in seconds; v
// and a are read for fixed ones and written for free ones.
struct Chain {
    size_t n = 0;
    float T[kChainKnots] = {}, p[kChainKnots] = {}, v[kChainKnots] = {}, a[kChainKnots] = {};
    bool  fix_v[kChainKnots] = {}, fix_a[kChainKnots] = {};
    float p0 = 0.0f, v0 = 0.0f, a0 = 0.0f;   // the origin, fixed
};

// The banded system's storage, owned by the caller (the engine keeps one per
// axis). Never a static: two engines on two tasks would share it. Never
// thread_local: ESP-IDF carves a task's thread-local block out of that task's
// own stack, and ten kilobytes of it made every task need a ten kilobyte
// stack; the IPC task has one, the board asserted in esp_ipc_init before
// app_main and boot-looped (kin-6tz). Never on the stack: the motion task's.
struct Workspace {
    float A[kMaxUnknowns][2 * kHalfBand + 1];
    float b[kMaxUnknowns];
    float s[kMaxUnknowns];
};

inline void solve(Chain& ch, Workspace& ws) {
    if (ch.n == 0 || ch.n > kChainKnots) return;
    K2_STAT(solves, 1); K2_STAT(chain_knots, ch.n);
    int idx_v[kChainKnots], idx_a[kChainKnots];
    int m = 0;
    for (size_t i = 0; i < ch.n; ++i) {
        idx_v[i] = ch.fix_v[i] ? -1 : m++;
        idx_a[i] = ch.fix_a[i] ? -1 : m++;
    }
    if (m == 0) return;
    // Banded storage: A[r][kHalfBand + (c - r)].
    auto& A = ws.A;
    auto& b = ws.b;
    auto& s = ws.s;
    for (int r = 0; r < m; ++r) { b[r] = 0.0f; for (int c = 0; c < 2 * kHalfBand + 1; ++c) A[r][c] = 0.0f; }
    float K[6][6];
    for (size_t k = 0; k < ch.n; ++k) {
        const float T = ch.T[k];
        if (!(T > 0.0f)) continue;
        // x components: index into unknowns (or -1) and their known values.
        int   ux[6];
        float xv[6];
        ux[0] = -1; xv[0] = k ? ch.p[k - 1] : ch.p0;
        ux[1] = k ? idx_v[k - 1] : -1; xv[1] = k ? ch.v[k - 1] : ch.v0;
        ux[2] = k ? idx_a[k - 1] : -1; xv[2] = k ? ch.a[k - 1] : ch.a0;
        ux[3] = -1; xv[3] = ch.p[k];
        ux[4] = idx_v[k]; xv[4] = ch.v[k];
        ux[5] = idx_a[k]; xv[5] = ch.a[k];
        if (ux[1] < 0 && ux[2] < 0 && ux[4] < 0 && ux[5] < 0) continue;   // every end fixed: no unknown in it
        pieceEnergy(T, K);
        for (int r = 0; r < 6; ++r) {
            if (ux[r] < 0) continue;
            for (int c = 0; c < 6; ++c) {
                if (ux[c] >= 0) {
                    const int d = ux[c] - ux[r];
                    if (d < -kHalfBand || d > kHalfBand) continue;   // never, by ordering
                    A[ux[r]][kHalfBand + d] += K[r][c];
                } else {
                    b[ux[r]] -= K[r][c] * xv[c];
                }
            }
        }
    }
    // Unit diagonal: A' = S A S, b' = S b, x = S y, S = diag(1 / sqrt(A_rr)).
    for (int r = 0; r < m; ++r) { const float d = A[r][kHalfBand]; s[r] = d > 0.0f ? 1.0f / std::sqrt(d) : 0.0f; }
    for (int r = 0; r < m; ++r) {
        b[r] *= s[r];
        for (int cc = r - kHalfBand; cc <= r + kHalfBand; ++cc)
            if (cc >= 0 && cc < m) A[r][kHalfBand + (cc - r)] *= s[r] * s[cc];
    }
    // Banded Gaussian elimination, no pivoting: the energy form is positive
    // definite on the unknowns once every position is fixed.
    for (int r = 0; r < m; ++r) {
        const float piv = A[r][kHalfBand];
        if (!(piv > 1e-12f)) continue;
        for (int rr = r + 1; rr <= r + kHalfBand && rr < m; ++rr) {
            const int off = rr - r;   // A[rr][kHalfBand - off] is the entry under the pivot
            const float f = A[rr][kHalfBand - off] / piv;
            if (f == 0.0f) continue;
            for (int cc = r; cc <= r + kHalfBand && cc < m; ++cc) {
                const int dr = cc - r, drr = cc - rr;
                A[rr][kHalfBand + drr] -= f * A[r][kHalfBand + dr];
            }
            b[rr] -= f * b[r];
        }
    }
    float x[kMaxUnknowns];
    for (int r = m - 1; r >= 0; --r) {
        float acc = b[r];
        for (int cc = r + 1; cc <= r + kHalfBand && cc < m; ++cc) acc -= A[r][kHalfBand + (cc - r)] * x[cc];
        const float piv = A[r][kHalfBand];
        x[r] = piv > 1e-12f ? acc / piv : 0.0f;
    }
    for (size_t i = 0; i < ch.n; ++i) {
        if (idx_v[i] >= 0) ch.v[i] = x[idx_v[i]] * s[idx_v[i]];
        if (idx_a[i] >= 0) ch.a[i] = x[idx_a[i]] * s[idx_a[i]];
    }
}

}  // namespace jerk

// ---- the solver --------------------------------------------------------------
// knots[0..n) pending, in time order; origin is the state the first piece
// starts from. Writes out[settled..n) and reports every spend through
// `report`. out[0..settled) are a previous solve's and stand: unchanged knots
// with unchanged predecessors, never re-judged. Knot 0's piece then starts at
// an origin that moved along it (the reaction horizon), and the rest of a
// quintic from a point on it, to the same end, is that same quintic.
// Returns where it stopped: a knot is started only while the budget lasts
// (Config::solve_budget; `unbounded` ignores it), except that a solve always
// accepts one knot. out[stop..n) wait for the next solve, as authored.
template <typename Report>
inline size_t solveWindow(const State& origin, uint64_t origin_us, const Knot* knots, size_t n,
                          const Config& cfg, Solved* out, Report&& report, jerk::Workspace& ws, const Knot* before = nullptr,
                          uint64_t before_solved_us = 0, uint64_t before_shift_us = 0, const Prior* prior = nullptr,
                          size_t settled = 0, bool unbounded = false) {
    if (settled > n) settled = n;
    if (n == settled) return n;
    K2_STAT(windows, 1);
    const Limits& L = cfg.limits;
    // Work done in this solve, against Config::solve_budget (0: unbounded), in
    // referee passes: one per referee pass (every extremum of one piece) and
    // one per four knots of each banded solve, which cost about the same.
    const uint32_t budget = cfg.solve_budget && !unbounded ? cfg.solve_budget : ~uint32_t(0);
    uint32_t work = 0;
    // The first knot of a solve cannot wait for a later one, so it may run to
    // twice the budget: the bound on one solve is then 2 * solve_budget.
    const uint32_t first_budget = budget > ~uint32_t(0) / 2 ? ~uint32_t(0) : 2 * budget;
    const float lo = std::fmin(0.0f, origin.p), hi = std::fmax(1.0f, origin.p);
    constexpr uint64_t kMinSpanUs = 1000;   // one tick between knots, the floor of every span

    if (settled > 0) {
        out[0].from = origin;
        out[0].from_us = origin_us;
    }
    // THE PLACEMENT RULE. A segment is placed at its authored time plus what
    // is left of the stretch the knot before it carries (shift_us: that knot's
    // placed time less its authored one, the corner's half ramp excluded) once
    // this span has given back what it can: a stretch moves every later
    // segment with it, a new arrival included (one that arrived after the
    // stretched knot retired kept its authored time, the next span lost the
    // stretch and the next spend paid for it: a cascade; so the knot retired
    // last carries its stretch here, before_shift_us), and every span after
    // it catches up as far as its own ceilings allow (absorbable, below): a
    // hold gives everything back, a span with margin runs that much faster
    // than authored, a span at its ceiling nothing. The motion is on the
    // author's clock again as soon as the author left room for it. placeLater
    // applies the same rule to the knots after a spend.
    auto absorbable = [&](size_t k, float prev_p, uint64_t prev_auth) -> uint64_t {
        if (knots[k].t_us <= prev_auth) return 0;
        const uint64_t span = knots[k].t_us - prev_auth;
        const float d = std::fabs(knots[k].p - prev_p);
        if (d <= 1e-6f) return span > kMinSpanUs ? span - kMinSpanUs : 0;   // a hold
        // The fastest legal span for the stroke, as a rest-to-rest cubic
        // (its speed 1.5 d / T, acceleration 6 d / T^2, jerk 12 d / T^3), five
        // percent over; and never under three quarters of the authored span,
        // so the catch-up is a step in pace the author's own margin bounds.
        const float t_min = 1.05f * std::fmax(1.5f * d / L.vmax, std::fmax(std::sqrt(6.0f * d / L.amax), std::cbrt(12.0f * d / L.jmax)));
        const uint64_t floor_us = std::max(uint64_t(t_min * 1e6f), (span * 3) / 4);
        return span > floor_us ? span - floor_us : 0;
    };
    for (size_t i = settled; i < n; ++i) {
        uint64_t t = knots[i].t_us;
        if (!knots[i].sample) {
            bool has_prev = false; uint64_t prev_t = 0, prev_shift = 0, prev_auth = 0; float prev_p = 0.0f;
            if (i > settled || settled > 0) {
                has_prev = true; prev_t = out[i - 1].t_us; prev_p = knots[i - 1].p; prev_auth = knots[i - 1].t_us;
                prev_shift = out[i - 1].shift_us;
            } else if (before && !before->sample) {
                has_prev = true; prev_t = before_solved_us; prev_shift = before_shift_us; prev_p = before->p; prev_auth = before->t_us;
            }
            if (has_prev) {
                const uint64_t give = absorbable(i, prev_p, prev_auth);
                t = knots[i].t_us + (prev_shift > give ? prev_shift - give : 0);
                if (t < prev_t + kMinSpanUs) t = prev_t + kMinSpanUs;
            }
        }
        out[i].t_us = t; out[i].p = knots[i].p;
        out[i].shift_us = knots[i].sample ? 0 : t - knots[i].t_us; out[i].own_us = 0;
        out[i].share = 1.0f; out[i].stretched_s = 0.0f; out[i].worst = 0.0f; out[i].clamped = false;
        out[i].hard = false; out[i].corner = false; out[i].pin_v = false; out[i].pin_a = false; out[i].dropped = false;
    }

    // Monotone slope at knot i from its solved neighbors (the origin on the left
    // of knot 0, nothing past the last knot).
    auto secant = [&](size_t a, size_t b) {   // from a to b, b = a + 1; a == npos is the origin
        const float pa = a == size_t(-1) ? origin.p : out[a].p;
        const uint64_t ta = a == size_t(-1) ? origin_us : out[a].t_us;
        return (out[b].p - pa) / (float(out[b].t_us - ta) * 1e-6f);
    };
    // A velocity at position p is honored within vmax and within what the
    // rail allows on BOTH sides: the fastest legal stop ahead must fit, and so
    // must the fastest legal run-up behind (the curve arrives from there). A
    // velocity the brake could not stop before the wall is illegal whatever
    // follows. Re-applied whenever p moves (a Blend trim moves it).
    auto boundForRail = [&](float v, float p) -> float {
        float cl = std::fmax(-L.vmax, std::fmin(L.vmax, v));
        const float gap = std::fmin(hi - p, p - lo);
        if (cl == 0.0f) return cl;
        if (gap <= 1e-6f) return 0.0f;
        // Stop distance of the fastest legal brake from speed u, at rest in
        // acceleration: the ramp in and out each cover u * amax / (2 jmax)
        // beyond the trapezoid's u^2 / (2 amax); a triangle stop is shorter.
        auto stopDist = [&](float u) { return u * u / (2.0f * L.amax) + u * L.amax / (2.0f * L.jmax); };
        if (stopDist(std::fabs(cl)) <= gap) return cl;
        float u_lo = 0.0f, u_hi = std::fabs(cl);
        for (int it = 0; it < 24; ++it) { const float m = 0.5f * (u_lo + u_hi); if (stopDist(m) <= gap) u_lo = m; else u_hi = m; }
        return (cl > 0.0f ? 1.0f : -1.0f) * u_lo;
    };
    // An authored velocity, bounded (EndVelClamped, reported once per knot).
    auto authored = [&](size_t i) -> float {
        const float v = knots[i].v;
        const float cl = boundForRail(v, out[i].p);
        if (cl != v && !out[i].clamped) { out[i].clamped = true; report(AnomalyKind::EndVelClamped, i, out[i].t_us, out[i].p, cl); }
        return cl;
    };
    auto slopeAt = [&](size_t i) -> float {
        const Knot& k = knots[i];
        if (k.has_v) return authored(i);
        // A free last knot is not the end of anything the solver can see: a
        // live stream's newest sample has a successor on the way. It keeps the
        // secant into it; if nothing arrives by the time it is reached, the
        // engine brakes from there (starvation is the brake, RFC-105 (a)).
        if (i + 1 >= n) {
            if (k.rest_if_last) return 0.0f;   // SPEC 9.6: nothing follows, so it rests
            // A sample behind another sample keeps the STREAM's velocity, the
            // secant over the two authored times: the solved times lag it when
            // the knot was stretched, and a secant over those lagged the
            // stream further on every re-solve (a 150 mm/s ramp settled at a
            // third of its speed). Anything else keeps the secant over the
            // solved times. Bounded like an authored velocity: within vmax and
            // stoppable before the rail, since the engine may have to brake
            // from here.
            float v;
            if (k.sample && i > 0 && knots[i - 1].sample && !out[i - 1].dropped && knots[i].t_us > knots[i - 1].t_us)
                v = (out[i].p - out[i - 1].p) / (float(knots[i].t_us - knots[i - 1].t_us) * 1e-6f);
            else
                v = secant(i == 0 ? size_t(-1) : i - 1, i);
            v = std::fmax(-L.vmax, std::fmin(L.vmax, v));
            const float p = out[i].p;
            const float gap = std::fmin(hi - p, p - lo);
            auto stopDist = [&](float u) { return u * u / (2.0f * L.amax) + u * L.amax / (2.0f * L.jmax); };
            if (v != 0.0f && gap <= 1e-6f) v = 0.0f;
            else if (v != 0.0f && stopDist(std::fabs(v)) > gap) {
                float u_lo = 0.0f, u_hi = std::fabs(v);
                for (int it = 0; it < 24; ++it) { const float m = 0.5f * (u_lo + u_hi); if (stopDist(m) <= gap) u_lo = m; else u_hi = m; }
                v = (v > 0.0f ? 1.0f : -1.0f) * u_lo;
            }
            return v;
        }
        const float dl = secant(i == 0 ? size_t(-1) : i - 1, i), dr = secant(i, i + 1);
        if (dl == 0.0f || dr == 0.0f || (dl < 0.0f) != (dr < 0.0f)) return 0.0f;
        const float hl = float(out[i].t_us - (i == 0 ? origin_us : out[i - 1].t_us)) * 1e-6f;
        const float hr = float(out[i + 1].t_us - out[i].t_us) * 1e-6f;
        // Fritsch-Butland: a weighted harmonic mean, shape-preserving.
        return 3.0f * (hl + hr) / ((2.0f * hr + hl) / dl + (hr + 2.0f * hl) / dr);
    };
    // The junction values of every knot from `first` on, by minimum jerk over
    // the accepted chain: pinned where authored, hard, relaxed, or the bounded
    // secant of a free last knot; free otherwise. Dropped knots are skipped.
    // Knots before `first` are fixed at their solved values.
    // Stream replay compression: a sample that lags sits closer behind the
    // sample before it than it was sent, a tenth closer at ten cadences of
    // lag or more and proportionally less below, so the replay rate changes
    // by a percent per sample at most. At 60 Hz a piece's jerk is about
    // three hundred thousand times the mismatch between its mean speed and
    // the speeds at its ends: a rate step of a tenth in one sample bulged
    // past the ceiling, was stretched, and the lag it was closing grew.
    constexpr float kMaxCompress = 0.1f, kEaseCadences = 10.0f;
    auto compression = [&](uint64_t prev_solved, uint64_t prev_authored, uint64_t spacing) -> float {
        const float lag = prev_solved > prev_authored ? float(prev_solved - prev_authored) : 0.0f;
        return kMaxCompress * std::fmin(1.0f, lag / (kEaseCadences * float(spacing)));
    };
    auto replaySpacing = [&](uint64_t spacing, float c) { return std::max<uint64_t>(uint64_t(float(spacing) * (1.0f - c)), kMinSpanUs); };
    // The chain starts at the last accepted knot before `first`: a piece with
    // both ends fixed holds no unknown, so the knots before it add nothing to
    // the system, and a knot already accepted is never repaired again.
    auto junctions = [&](size_t first) {
        K2_STAT(junctions, 1);
        jerk::Chain ch;
        size_t c0 = size_t(-1);
        for (size_t k = first; k-- > 0;) if (!out[k].dropped) { c0 = k; break; }
        ch.p0 = c0 == size_t(-1) ? origin.p : out[c0].p;
        ch.v0 = c0 == size_t(-1) ? origin.v : out[c0].v;
        ch.a0 = c0 == size_t(-1) ? origin.a : out[c0].a;
        const uint64_t t0_us = c0 == size_t(-1) ? origin_us : out[c0].t_us;
        size_t map[jerk::kChainKnots]; size_t m = 0;
        uint64_t span_us[jerk::kChainKnots];
        for (size_t i = first; i < n && m < jerk::kChainKnots; ++i) {
            if (out[i].dropped) continue;
            map[m] = i;
            span_us[m] = out[i].t_us - (m ? out[map[m - 1]].t_us : t0_us);
            ch.T[m] = float(span_us[m]) * 1e-6f;
            ch.p[m] = out[i].p;
            const bool hard = junctionOf(knots[i]) == Junction::Hard;
            if (hard) { ch.v[m] = out[i].v; ch.a[m] = out[i].a; ch.fix_v[m] = ch.fix_a[m] = true; }
            else if (knots[i].sample && !knots[i].has_v) {
                // An interior sample's junction is free (minimum jerk through
                // the chain, inside the monotone band): the chain is then a
                // smoothing spline through the samples. Pinning the Fritsch-
                // Butland slope at every sample was a few percent off the
                // sine it sampled, and at 60 Hz a piece's jerk is about three
                // hundred thousand times that mismatch: every piece sat at
                // the ceiling and stretched. The first sample keeps its prior
                // junction (the committed curve), and the newest, with nothing
                // after it, the stream's slope.
                // The newest sample's velocity is free as well (the chain's
                // natural end, inside the monotone band): pinned to the
                // stream's slope it was a percent or two off the chain, and
                // at a jerk ceiling of 500 that is a bulge past the ceiling
                // on every piece (a 60 Hz sine crawled at a seventh of its
                // speed). Free, it adapts, so a replay slowed by a stretch
                // speeds back up.
                // The newest sample's acceleration is the stream's own: the
                // second difference of the last three authored samples (the
                // knot retired before the window counts), bounded by amax,
                // and zero with fewer. A resting acceleration there was
                // carried into the committed junction at every sample, and a
                // sine that needs 14 units/s^2 at its samples had to rebuild
                // that inside one cadence: past the jerk ceiling at 500,
                // every piece stretched and the stream crawled. A free one
                // left the stream's first sample from rest still accelerating
                // at 39, which the next piece could not take back.
                float a_tail = 0.0f;
                if (i + 1 >= n && !out[i].pin_a) {
                    size_t l1 = size_t(-1), l2 = size_t(-1);
                    for (size_t k = i; k-- > 0;) { if (out[k].dropped || !knots[k].sample) continue; if (l1 == size_t(-1)) l1 = k; else { l2 = k; break; } }
                    const bool has1 = l1 != size_t(-1) || (before && before->sample);
                    const bool has2 = l2 != size_t(-1) || (l1 != size_t(-1) && before && before->sample);
                    if (has1 && has2) {
                        const float p1 = l1 != size_t(-1) ? out[l1].p : before->p;
                        const float p2 = l2 != size_t(-1) ? out[l2].p : before->p;
                        const uint64_t t1 = l1 != size_t(-1) ? knots[l1].t_us : before->t_us;
                        const uint64_t t2 = l2 != size_t(-1) ? knots[l2].t_us : before->t_us;
                        const uint64_t t0 = knots[i].t_us;
                        if (t0 > t1 && t1 > t2) {
                            const float h01 = float(t0 - t1) * 1e-6f, h12 = float(t1 - t2) * 1e-6f, h02 = float(t0 - t2) * 1e-6f;
                            const float d1 = (out[i].p - p1) / h01, d2 = (p1 - p2) / h12;
                            a_tail = 2.0f * (d1 - d2) / h02;
                            a_tail = std::fmax(-L.amax, std::fmin(L.amax, a_tail));
                        }
                    }
                }
                ch.v[m] = out[i].pin_v ? out[i].v : 0.0f;
                ch.a[m] = out[i].pin_a ? out[i].a : a_tail;
                ch.fix_v[m] = out[i].pin_v;
                ch.fix_a[m] = out[i].pin_a || i + 1 >= n;
            }
            else {
                // Velocity: pinned when authored, relaxed, or the last knot's
                // bounded secant; free otherwise. Acceleration: pinned when
                // relaxed and at the last knot (0: nothing is known beyond it,
                // and an authored stop is a stop); free otherwise. A free
                // sequence may ring by under 0.2 percent of the window at a
                // kink, the price of the smoothest curve (RFC-105 (aa)).
                ch.fix_v[m] = out[i].pin_v || knots[i].has_v || i + 1 >= n;
                ch.v[m] = out[i].pin_v ? out[i].v : slopeAt(i);
                ch.fix_a[m] = out[i].pin_a || i + 1 >= n;   // the last knot rests in acceleration: nothing is known beyond it
                ch.a[m] = out[i].pin_a ? out[i].a : 0.0f;
            }
            ++m;
        }
        ch.n = m;
        { jerk::solve(ch, ws); work += uint32_t((m + 3) / 4); }
        // Shape preservation: a free velocity may not exceed the monotone
        // band of its two secants (three times the smaller, same sign; zero
        // across a sign change), the Fritsch-Carlson condition, so a kink is
        // never overshot. Accelerations are solved again under the clamped
        // velocities. A stream's secants agree, so the band is wide there and
        // the minimum-jerk velocities pass untouched.
        // A sample at a sign change is exempt from the zero: a stream sampled
        // densely reverses BETWEEN two samples, and a zero slope forced onto
        // the nearest one bulged the piece (jerk) and stretched it at every
        // peak of a sine. A sample on a plateau (a hold) still rests.
        bool clamped = false;
        for (size_t k = 0; k < m; ++k) {
            if (ch.fix_v[k]) continue;
            const float pl = k ? ch.p[k - 1] : ch.p0;
            const float dl = (ch.p[k] - pl) / std::fmax(ch.T[k], 1e-6f);
            const float dr = k + 1 < m ? (ch.p[k + 1] - ch.p[k]) / std::fmax(ch.T[k + 1], 1e-6f) : dl;
            float band = 0.0f;
            if (dl == 0.0f || dr == 0.0f) band = 0.0f;
            else if ((dl < 0.0f) != (dr < 0.0f)) { if (knots[map[k]].sample) continue; band = 0.0f; }
            else band = 3.0f * std::fmin(std::fabs(dl), std::fabs(dr));
            const float sgn = dl < 0.0f ? -1.0f : 1.0f;
            float v = ch.v[k];
            if (band == 0.0f) v = 0.0f;
            else if ((v < 0.0f) != (sgn < 0.0f)) v = 0.0f;
            else if (std::fabs(v) > band) v = sgn * band;
            // And stoppable before the rail: any knot may turn out to be the
            // last one (a dropped successor, a starved stream) and the engine
            // then brakes from it.
            if (v != 0.0f) {
                const float gap = std::fmin(hi - ch.p[k], ch.p[k] - lo);
                auto stopDist = [&](float u) { return u * u / (2.0f * L.amax) + u * L.amax / (2.0f * L.jmax); };
                if (gap <= 1e-6f) v = 0.0f;
                else if (stopDist(std::fabs(v)) > gap) {
                    float u_lo = 0.0f, u_hi = std::fabs(v);
                    for (int it = 0; it < 24; ++it) { const float mid = 0.5f * (u_lo + u_hi); if (stopDist(mid) <= gap) u_lo = mid; else u_hi = mid; }
                    v = (v > 0.0f ? 1.0f : -1.0f) * u_lo;
                }
            }
            if (v != ch.v[k]) { ch.v[k] = v; ch.fix_v[k] = true; clamped = true; }
        }
        if (clamped) { jerk::solve(ch, ws); work += uint32_t((m + 3) / 4); }
        // Stoppable before the velocity ceiling, as before the rail: a
        // junction accelerating toward vmax must be able to shed that
        // acceleration under jmax before the velocity reaches it, or the
        // next piece, whatever it turns out to be, runs past the ceiling. A
        // stream at vmax left a junction at 94 percent of vmax still
        // accelerating at 64 percent of amax; no later sample was reachable
        // from it, every one was dropped, and the carriage sagged to rest
        // inside a moving stream.
        bool a_clamped = false;
        for (size_t k = 0; k < m; ++k) {
            if (ch.fix_a[k]) continue;
            const float v = ch.v[k], a = ch.a[k];
            if (a == 0.0f || (a < 0.0f) != (v < 0.0f)) continue;
            const float room = std::fmax(0.0f, L.vmax - std::fabs(v));
            const float a_ok = std::sqrt(2.0f * L.jmax * room);
            if (std::fabs(a) > a_ok) { ch.a[k] = (a < 0.0f ? -1.0f : 1.0f) * a_ok; ch.fix_a[k] = true; a_clamped = true; }
        }
        if (a_clamped) { jerk::solve(ch, ws); work += uint32_t((m + 3) / 4); }
        // Monotone repair: a piece whose knots rise (or fall) must not turn
        // back inside. A velocity sign change inside such a piece is the
        // accelerations' doing; halve both junction accelerations, pin them,
        // solve again; three rounds, then zero. Only the two pieces at the
        // knot being decided (its junction is all a solve keeps; every later
        // piece is repaired when its own knot is decided): over the whole
        // chain, the repair was most of a bundle's cost, and the property
        // suite's hits, spends and drops are the same either way.
        for (int round = 0; round < 4; ++round) {
            bool fixed = false;
            for (size_t k = 0; k < m && k < 2; ++k) {
                const float pl = k ? ch.p[k - 1] : ch.p0;
                const float vl = k ? ch.v[k - 1] : ch.v0, al = k ? ch.a[k - 1] : ch.a0;
                if (span_us[k] == 0) continue;   // a plateau piece (equal knots) must stay flat: checked too
                // A piece between two moving samples may crest between them:
                // a stream sampled densely reverses between samples, and the
                // repair pinned the junction accelerations at every peak of a
                // sine, which bulged the piece and stretched it. A piece into
                // or out of a resting sample (a hold, a dead stop) is still
                // repaired: that is the overshoot the repair exists for.
                const bool left_sample = k ? knots[map[k - 1]].sample : (c0 == size_t(-1) || knots[c0].sample);
                const bool left_rests = k ? (ch.fix_v[k - 1] && ch.v[k - 1] == 0.0f) : (c0 != size_t(-1) && ch.v0 == 0.0f);
                if (knots[map[k]].sample && left_sample && !(ch.fix_v[k] && ch.v[k] == 0.0f) && !left_rests) continue;
                // Nothing to repair with: both end accelerations pinned at
                // zero, or the left one an accepted knot's.
                if ((k == 0 || (ch.fix_a[k - 1] && ch.a[k - 1] == 0.0f)) && ch.fix_a[k] && ch.a[k] == 0.0f) continue;
                const Piece q = Piece::hermite(0, State{pl, vl, al}, span_us[k], State{ch.p[k], ch.v[k], ch.a[k]});
                // At every extremum inside (0,1) the position must stay within
                // the knots' own band; a position outside it is a turn-back.
                ++work;
                float taus[12]; const int nt = referee::extremumTaus(q.c, taus);
                const float band_lo = std::fmin(pl, ch.p[k]) - 1e-5f, band_hi = std::fmax(pl, ch.p[k]) + 1e-5f;
                bool turns = false;
                for (int j = 0; j < nt && !turns; ++j) {
                    const float t = taus[j];
                    if (t <= 0.0f || t >= 1.0f) continue;
                    const float pp = q.at(uint64_t(t * float(span_us[k]))).p;
                    if (pp < band_lo || pp > band_hi) turns = true;
                }
                if (!turns) continue;
                fixed = true;
                if (k) { ch.a[k - 1] = round < 3 ? 0.5f * ch.a[k - 1] : 0.0f; ch.fix_a[k - 1] = true; }
                ch.a[k] = round < 3 ? 0.5f * ch.a[k] : 0.0f; ch.fix_a[k] = true;
            }
            if (!fixed) break;
            { jerk::solve(ch, ws); work += uint32_t((m + 3) / 4); }
        }
        for (size_t k = 0; k < m; ++k) {
            const size_t i = map[k];
            out[i].v = ch.v[k];
            out[i].a = ch.a[k];
            // The ceiling bound again, after the repair's solves moved the
            // free values: a junction at vmax still accelerating outward ran
            // past it on the next piece, which no time could cure (a C1
            // corner's exit fed such a chain).
            const float v = out[i].v, a = out[i].a;
            if (a != 0.0f && (a < 0.0f) == (v < 0.0f)) {
                const float a_ok = std::sqrt(2.0f * L.jmax * std::fmax(0.0f, L.vmax - std::fabs(v)));
                if (std::fabs(a) > a_ok) out[i].a = (a < 0.0f ? -1.0f : 1.0f) * a_ok;
            }
        }
    };

    State prev = origin;          // the state the piece into knot i starts from
    uint64_t prev_us = origin_us;
    State pp = origin;            // the state the piece into the last accepted knot started from
    uint64_t pp_us = origin_us;
    size_t last = size_t(-1);     // the last accepted knot (dropped ones never count)
    if (settled > 0) {
        last = settled - 1;
        prev = State{out[last].p, out[last].v, out[last].a};
        prev_us = out[last].t_us;
        pp = out[last].from; pp_us = out[last].from_us;
    }

    // Solve knot i from prev. rest_end: the knot ends the timeline (everything
    // after it was dropped), so it rests and no backward relaxation runs.
    // Returns false when the knot was dropped.
    // Stream REPLAY. A sample solved before keeps its time (its prior), and
    // the first one its prior junction too: the committed curve continued,
    // exactly. A sample solved for the first time sits nine tenths of its
    // authored spacing behind the sample before it, never before its own
    // authored time: a stream that lags is replayed a tenth faster than it
    // was sent, so the lag closes at the tail and nothing in flight moves.
    // Pulling the knots in flight earlier instead put the whole catch-up
    // into the first piece, which bulged (jerk) and was illegal whenever the
    // stream ran near its own speed; a chain re-expanded behind a floored
    // first knot never closed its lag at all. A sample behind one that was
    // stretched past its prior follows it at nine tenths of the spacing.
    auto hasPrior = [&](size_t k) { return prior && prior[k].t_us != 0 && knots[k].sample; };
    // The time of sample k behind the knot before it (prev_k, at prev_t), or
    // behind the knot retired just before the window when there is none.
    // A sample behind another whose prior time trails the replay rule by a
    // whole cadence or more is placed by the rule again: a stream that began
    // far from a resting carriage had its first samples stretched one by one,
    // and those times, kept, spaced samples authored 10 ms apart at 112, 71,
    // 33 and 28 ms; the smoothing spline wiggled through that, the monotone
    // band zeroed a junction inside a moving stream, and every later sample
    // was unreachable from the rest it left. The window's first sample keeps
    // its time (the committed curve continued).
    auto sampleTime = [&](size_t k, uint64_t prev_t, size_t prev_k) -> uint64_t {
        uint64_t t = hasPrior(k) ? prior[k].t_us : knots[k].t_us;
        uint64_t want = prev_t + kMinSpanUs;
        uint64_t spacing = 0;
        if (prev_k != size_t(-1)) {
            if (knots[prev_k].sample && knots[k].t_us > knots[prev_k].t_us) {
                spacing = knots[k].t_us - knots[prev_k].t_us;
                want = prev_t + replaySpacing(spacing, compression(prev_t, knots[prev_k].t_us, spacing));
            }
        } else if (before && before->sample && knots[k].t_us > before->t_us) {
            spacing = knots[k].t_us - before->t_us;
            want = before_solved_us + replaySpacing(spacing, compression(before_solved_us, before->t_us, spacing));
            if (want < prev_t + kMinSpanUs) want = prev_t + kMinSpanUs;
        }
        if (k > 0 && hasPrior(k) && spacing > 0 && t > want + spacing) t = want;
        return std::max(t, want);
    };
    auto floorSelf = [&](size_t i) { if (out[i].t_us < prev_us + kMinSpanUs) out[i].t_us = prev_us + kMinSpanUs; };
    // Places every knot after i for the junction solve: a sample by the rule
    // above, a segment by the placement rule at the top of the solve (its
    // authored span after the knot before it, a hold at its authored time),
    // each one tick past the knot before. The one home of a stretch's reach:
    // a knot moved later moves the segments after it through this.
    // Knot i's time must be its placed time (authored plus stretch, not a
    // corner's ramp end) when this runs.
    auto placeLater = [&](size_t i) {
        size_t pk = i;
        for (size_t k = i + 1; k < n; ++k) {
            if (out[k].dropped) continue;
            if (knots[k].sample) out[k].t_us = sampleTime(k, out[pk].t_us, pk);
            else {
                const uint64_t prev_shift = knots[pk].sample ? 0 : (out[pk].t_us > knots[pk].t_us ? out[pk].t_us - knots[pk].t_us : 0);
                const uint64_t give = absorbable(k, knots[pk].p, knots[pk].t_us);
                out[k].t_us = knots[k].t_us + (prev_shift > give ? prev_shift - give : 0);
            }
            if (out[k].t_us < out[pk].t_us + kMinSpanUs) out[k].t_us = out[pk].t_us + kMinSpanUs;
            pk = k;
        }
    };
    enum : int { kDropped = 0, kAccepted = 1, kDeferred = -1 };
    auto solveKnot = [&](size_t i, bool rest_end) -> int {
        K2_STAT(knots, 1);
        // The relaxations below may change the last accepted knot; a deferral
        // puts it back.
        const Solved last_entry = last != size_t(-1) ? out[last] : Solved{};
        // A sample: its prior time, or its replay time when new; the first
        // sample of the window keeps its prior junction as well.
        if (knots[i].sample) {
            out[i].t_us = sampleTime(i, prev_us, last);
            if (i == 0 && hasPrior(0) && !out[0].pin_v && !out[0].pin_a) {
                out[0].pin_v = out[0].pin_a = true; out[0].v = prior[0].v; out[0].a = prior[0].a;
            }
        }
        floorSelf(i); placeLater(i);
        // A segment with no end velocity and nothing after it rests (SPEC 9.6).
        const bool rest = rest_end || (i + 1 >= n && knots[i].rest_if_last && !knots[i].has_v);
        // HARD: the fastest legal move from wherever the carriage is to rest
        // on the knot (Profile::point), from any start: moving toward it,
        // away from it, or past it. The knot's time is its authored time, or
        // the profile's end when that is later; time to spare is a hold
        // before the launch from rest, or a slower cruise when moving. A live
        // jog (a HARD sample: its deadline was the sender's estimate) takes
        // the profile's end unreported; a segment reports the Stretch and
        // moves the segments after it by the same time. A quintic head with a
        // bisected cruise speed needed its crawl to be legal first, which a
        // moving start near the deadline never was: the knot fell to one
        // smooth quintic stretched to 1.5 s and a jog decayed instead of
        // turning (Kinetic kin-hnp). Falls through to the smooth path only
        // when the profile breaks a ceiling or the window.
        if (junctionOf(knots[i]) == Junction::Hard) {
            const uint64_t avail = out[i].t_us > prev_us ? out[i].t_us - prev_us : 0;
            float fastest = 0.0f;
            const Profile pr = Profile::point(prev, out[i].p, prev_us, L, float(avail) * 1e-6f, &fastest);
            work += 8;   // point() costs about seven referee passes (its two bisections), the check one
            const float ratio = pr.n < 0 ? referee::kIllegal : pr.worstRatio(L, lo, hi);
            const State end = pr.n > 0 ? pr.atSeconds(pr.duration()) : State{prev.p, 0.0f, 0.0f};
            const bool late = uint64_t(fastest * 1e6f) > avail + 1;
            if (ratio <= 1.0001f && std::fabs(end.p - out[i].p) <= 1e-5f) {
                const uint64_t end_us = pr.end_us();
                const uint64_t add = end_us > out[i].t_us ? end_us - out[i].t_us : 0;
                if (end_us > out[i].t_us) out[i].t_us = end_us;
                placeLater(i);
                out[i].hard = true; out[i].ramp = pr;
                out[i].v = 0.0f; out[i].a = 0.0f; out[i].worst = ratio;
                out[i].stretched_s = late ? float(add) * 1e-6f : 0.0f;
                if (late && add > 0 && !knots[i].sample) report(AnomalyKind::DeadlineStretched, i, out[i].t_us, out[i].p, out[i].stretched_s);
                out[i].from = prev; out[i].from_us = prev_us;
                pp = prev; pp_us = prev_us; last = i;
                out[i].shift_us = knots[i].sample || out[i].t_us < knots[i].t_us ? 0 : out[i].t_us - knots[i].t_us;
                out[i].own_us = knots[i].sample ? 0 : add;
                prev = State{out[i].p, 0.0f, 0.0f};
                prev_us = out[i].t_us;
                return kAccepted;
            }
        }
        if (rest) {
            out[i].pin_a = true; out[i].a = 0.0f;
            if (!knots[i].has_v) { out[i].pin_v = true; out[i].v = 0.0f; }
        }
        // CORNER: an authored C1 knot, moving or not (a reversal is where the
        // author's two cubics disagree most). Each side keeps the
        // acceleration the author's cubic has there; one constant-jerk phase
        // of |da| / jmax, centered on the knot, joins them. A quintic Hermite
        // through p, v and a at both ends, with the cubic's own end
        // accelerations, IS that cubic, so a C1 script renders as its
        // author's curve wherever the ceilings allow it. The newest knot has
        // no right side yet: the piece into it ends with the left cubic's
        // acceleration, and when its successor arrives the re-solve (the new
        // knot and the one before, Engine::kResolveDepth) gives it the ramp.
        // A single C2 acceleration there was the chain's compromise between
        // two cubics 12 apart at a peak: every rise wobbled and every peak
        // shelved (Kinetic kin-7jd). Falls through to the smooth path when
        // the head cannot legally reach the ramp's start.
        if (cfg.corner == Corner::Cubic && !rest && knots[i].family == Family::C1 && knots[i].has_v
            && out[i].t_us > prev_us + 1000) {
            const float vk = authored(i);
            const float pk = out[i].p;
            const uint64_t t_k0 = out[i].t_us;
            // The right cubic's start acceleration is the next span's, which a
            // spend here moves whole (the later segments follow a stretch), so
            // it stands for every attempt below. The right cubic may break a
            // ceiling: the next knot renders it as the author's cubic anyway,
            // saturated or dilated (below, at that knot), so only a window
            // excursion, which no spend sizes, refuses the corner here. Leaving
            // with a cubic's acceleration the next knot then fell to the smooth
            // quintic, whose peak speed is a quarter higher, and every spend on
            // it was priced for that quintic: twice what the cubic asked
            // (Kinetic kin-g5u).
            float right_T = 0.0f, right_pn = 0.0f, right_vn = 0.0f;
            bool has_right = false, right_ok = true;
            if (i + 1 < n && out[i + 1].t_us > out[i].t_us) {
                const float Tout = float(out[i + 1].t_us - out[i].t_us) * 1e-6f;
                const float pn = out[i + 1].p, vn = slopeAt(i + 1);
                right_T = Tout; right_pn = pn; right_vn = vn;
                const float a_r_full = (6.0f * (pn - pk) - Tout * (4.0f * vk + 2.0f * vn)) / (Tout * Tout);
                const float a_e = (6.0f * (pk - pn) + Tout * (2.0f * vk + 4.0f * vn)) / (Tout * Tout);
                const Piece right = Piece::hermite(out[i].t_us, State{pk, vk, a_r_full}, out[i + 1].t_us, State{pn, vn, a_e});
                ++work;
                referee::Ratios rp{};
                const float rw = referee::worstRatio(right, L, lo, hi, &rp);
                right_ok = rw < 0.5f * referee::kIllegal;
                // The right cubic as the next knot will render it: speed-bound
                // alone it is saturated and keeps this acceleration; otherwise
                // it is dilated by the root of the ratio that binds, and the
                // ramp leaves with the dilated cubic's acceleration. Leaving
                // with the authored one pinned a start the dilated head could
                // not keep, and the next knot fell to the smooth path from a
                // state at the acceleration ceiling.
                if (right_ok && rw > 1.0f && !(rp.v > 1.0f && rp.a <= 1.0f && rp.j <= 1.0f))
                    right_T = Tout * std::fmax(rp.v, std::fmax(std::sqrt(rp.a), std::cbrt(rp.j))) * 1.01f;
                has_right = true;
            }
            // The stretch this knot inherited from the knots before it, against
            // Config::late_budget_us: a time spend that would run past the
            // budget is refused and the stroke is cut instead (TRIM below).
            const uint64_t inherited = t_k0 > knots[i].t_us ? t_k0 - knots[i].t_us : 0;
            // Under Stretch time is the policy's own currency and has no budget;
            // under Blend the budget is where the deadline wins back (TRIM).
            auto within = [&](uint64_t add) { return cfg.policy == Policy::Stretch || inherited + add <= cfg.late_budget_us; };
            // Any knot may turn out to be the last (a dropped successor, a
            // starved stream) and the engine then brakes from the state it
            // leaves: that brake must keep the ceilings and the window, or the
            // knot takes the smooth path, whose junctions are bounded so.
            auto stoppable = [&](const State& x) { ++work; return Profile::brake(x, 0, L).worstRatio(L, lo, hi) <= 1.0001f; };
            // One attempt with the knot at t_k: each side keeps the acceleration
            // the author's cubic has there, one constant-jerk phase of |da| /
            // jmax centered on the knot joins them, and the head runs to the
            // ramp's start. A quintic Hermite through p, v and a at both ends,
            // with the cubic's own end accelerations, IS that cubic. The newest
            // knot has no right side yet: its piece ends with the left cubic's
            // acceleration, and the re-solve when its successor arrives (the
            // new knot and the one before, Engine::kResolveDepth) gives it the
            // ramp. A single C2 acceleration there was the chain's compromise
            // between two cubics 12 apart at a peak: every rise wobbled and
            // every peak shelved (Kinetic kin-7jd).
            // a_over: the cubic's own end acceleration over the ceiling (before
            // the clamp), the ratio the dilation sizes from; the clamped head
            // sits exactly on the ceiling and reads 1.00 whatever the time.
            struct Try { bool built = false, legal = false; float w = 0.0f, a_over = 0.0f; referee::Ratios parts{}; float a_l = 0.0f, j = 0.0f, Tr = 0.0f; uint64_t h_us = 0, ts = 0; State start{}; Piece head{}; };
            auto attempt = [&](uint64_t t_k) -> Try {
                Try r;
                const float Tin = float(t_k - prev_us) * 1e-6f;
                const float pk_now = out[i].p;
                float a_l = (6.0f * (prev.p - pk_now) + Tin * (2.0f * prev.v + 4.0f * vk)) / (Tin * Tin);
                r.a_over = std::fabs(a_l) / L.amax;
                a_l = std::fmax(-L.amax, std::fmin(L.amax, a_l));
                const float a_r = has_right
                    ? std::fmax(-L.amax, std::fmin(L.amax, (6.0f * (right_pn - pk_now) - right_T * (4.0f * vk + 2.0f * right_vn)) / (right_T * right_T)))
                    : a_l;
                r.a_l = a_l;
                r.Tr = std::fabs(a_r - a_l) / L.jmax;
                const float h = 0.5f * r.Tr;
                r.h_us = uint64_t(h * 1e6f + 0.5f);
                if (r.h_us == 0) {
                    r.ts = t_k; r.start = State{pk_now, vk, a_l};
                } else {
                    if (t_k <= prev_us + r.h_us + 1000) return r;
                    r.j = (a_r - a_l) / r.Tr;
                    // Walk the mid state back to the ramp's start.
                    const float vs = vk - a_l * h - 0.5f * r.j * h * h;
                    const float ps = pk_now - vs * h - 0.5f * a_l * h * h - r.j * h * h * h / 6.0f;
                    r.start = State{ps, vs, a_l};
                    r.ts = t_k - r.h_us;
                }
                r.head = Piece::hermite(prev_us, prev, r.ts, r.start);
                ++work;
                r.w = referee::worstRatio(r.head, L, lo, hi, &r.parts);
                // At the ceiling by a float's width is at the ceiling (the
                // profile checks allow the same); over it by the cubic's own
                // end acceleration is not, whatever the clamp made of it.
                r.built = true; r.legal = r.w <= 1.0001f && r.a_over <= 1.0001f;
#ifdef K2_TRACE
                std::printf("  corner i=%zu t_k=%.1fms head T=%.1fms prev(p%.4f v%.3f a%.1f) start(p%.4f v%.3f a%.1f) Tr=%.1fms w=%.3g (v%.2f a%.2f j%.2f)\n", i, double(t_k - prev_us) / 1000.0, double(r.ts - prev_us) / 1000.0, prev.p, prev.v, prev.a, r.start.p, r.start.v, r.start.a, r.Tr * 1e3f, r.w, r.parts.v, r.parts.a, r.parts.j);
#endif
                return r;
            };
            // Books an accepted knot: the time it spent moves the later segments
            // with it and is reported as the Stretch it is.
            // Books an accepted knot: placed at nominal_us (its authored time plus
            // the stretch it carries and spent), which moves the later segments
            // through placeLater; it ends at final_us (a corner's ramp end).
            auto book = [&](const State& exit, uint64_t nominal_us, uint64_t final_us, uint64_t add, float w) -> int {
                out[i].t_us = nominal_us;
                placeLater(i);
                out[i].t_us = final_us;
                out[i].worst = w;
                out[i].stretched_s = float(add) * 1e-6f;
                if (add > 0) report(AnomalyKind::DeadlineStretched, i, out[i].t_us, out[i].p, out[i].stretched_s);
                out[i].from = prev; out[i].from_us = prev_us;
                pp = prev; pp_us = prev_us; last = i;
                out[i].shift_us = nominal_us > knots[i].t_us ? nominal_us - knots[i].t_us : 0; out[i].own_us = add;
                prev = exit; prev_us = out[i].t_us;
                return kAccepted;
            };
            // Takes a built, legal attempt whose ramp and exit pass; false falls through.
            auto take = [&](const Try& r, uint64_t add) -> bool {
                if (r.h_us == 0) {
                    if (!stoppable(r.start)) return false;
                    out[i].v = vk; out[i].a = r.a_l;
                    book(r.start, r.ts, r.ts, add, r.w);
                    return true;
                }
                Profile ramp; ramp.start_us = r.ts; ramp.s0 = r.start; ramp.n = 1; ramp.dt[0] = r.Tr; ramp.jerk[0] = r.j; ramp.ends_at_rest = false;
                const State exit = Profile::step(r.start, r.j, r.Tr);
                if (!(ramp.worstRatio(L, lo, hi) <= 1.0f && stoppable(exit))) return false;
                out[i].corner = true; out[i].head_us = r.ts; out[i].head = r.start; out[i].ramp = ramp;
                out[i].p = exit.p; out[i].v = exit.v; out[i].a = exit.a;
                book(exit, r.ts + r.h_us, r.ts + uint64_t(r.Tr * 1e6f + 0.5f), add, r.w);
                return true;
            };
            if (right_ok) {
                const Try r0 = attempt(t_k0);
                if (r0.built && r0.legal && take(r0, 0)) return kAccepted;
                const bool finite0 = r0.built && r0.w < 0.5f * referee::kIllegal;
                // SATURATE: speed alone binds. The author's cubic with its speed
                // clipped at the ceiling (Profile::saturate), the knot later by
                // the cruise's cost: the closest curve the ceiling allows, whole
                // stroke, a few ms late where a rescale of the cubic is late by
                // the whole ratio and a trim short by it (kin-g5u). The ramp
                // follows the saturated head as one profile.
                if (finite0 && !r0.legal && r0.parts.v > 1.0f && r0.parts.a <= 1.0f && r0.parts.j <= 1.0f) {
                    Profile sat;
                    work += 3;   // two closed-form bisections and the referee
                    const uint64_t Th_us = r0.ts - prev_us;
                    if (Profile::saturate(prev, r0.start, float(Th_us) * 1e-6f, L, sat)) {
                        sat.start_us = prev_us;
                        Profile whole = sat;
                        if (r0.h_us == 0 || whole.add(r0.Tr, r0.j)) {
                            const float w = whole.worstRatio(L, lo, hi);
                            const State exit = whole.end();
                            const uint64_t dur_us = uint64_t(sat.duration() * 1e6f + 0.5f);
                            const uint64_t add = dur_us > Th_us ? dur_us - Th_us : 0;
                            if (w <= 1.0001f && std::fabs(sat.end().p - r0.start.p) <= 1e-4f && within(add) && stoppable(exit)) {
                                out[i].hard = true; out[i].corner = false; out[i].ramp = whole;
                                out[i].p = exit.p; out[i].v = exit.v; out[i].a = exit.a;
                                return book(exit, t_k0 + add, prev_us + uint64_t(whole.duration() * 1e6f + 0.5f), add, w);
                            }
                        }
                    }
                }
                // DILATE: an acceleration or jerk ceiling binds (or the speed
                // could not be saturated). The cubic given more time: its speed
                // scales with 1 / T, its acceleration with 1 / T^2, its jerk with
                // 1 / T^3, so the span at ratio 1 is T times the bound ratio's
                // root; judged, one secant refinement. The whole cubic slows, the
                // knot and the later segments move by the same time.
                if (finite0 && !r0.legal) {
                    const float need = std::fmax(std::fmax(r0.parts.v, std::sqrt(r0.a_over)), std::fmax(std::sqrt(r0.parts.a), std::cbrt(r0.parts.j)));
                    const uint64_t Th_us = t_k0 - prev_us;
                    uint64_t add = uint64_t(float(Th_us) * (need * 1.01f - 1.0f)) + 1;
                    Try r1 = attempt(t_k0 + add);
#ifdef K2_TRACE
                    std::printf("  dilate i=%zu need=%.3f add=%.1fms built=%d legal=%d w=%.3g\n", i, need, double(add) / 1000.0, int(r1.built), int(r1.legal), r1.w);
#endif
                    if (r1.built && !r1.legal && r1.w < 0.5f * referee::kIllegal && r1.w < r0.w) {
                        // The secant through the two judges at 0.998, never less
                        // than a percent more time (a ratio at the ceiling by a
                        // float's width moved the secant by nothing).
                        const float x = float(add) * (r0.w - 0.998f) / (r0.w - r1.w);
                        const uint64_t least = add + (Th_us + add) / 100 + 1;
                        add = x > float(least) ? uint64_t(x) + 1 : least;
                        r1 = attempt(t_k0 + add);
                    }
                    if (r1.built && r1.legal && within(add) && take(r1, add)) return kAccepted;
                }
                // TRIM (Blend): the budget is spent, or no time spend took. The cubic
                // with its stroke cut by the ratio that binds, on time (the
                // Blend spend sized for the cubic, not the quintic); refined
                // once on the ratio it leaves.
                if (finite0 && !r0.legal && cfg.policy != Policy::Stretch) {
                    const float need = std::fmax(std::fmax(r0.parts.v, std::sqrt(r0.a_over)), std::fmax(std::sqrt(r0.parts.a), std::cbrt(r0.parts.j)));
                    const float p_full = knots[i].p;
                    float share = 0.998f / need;
                    for (int it = 0; it < 2 && share > 0.05f && share < 1.0f; ++it) {
                        out[i].p = prev.p + share * (p_full - prev.p);
                        const Try rt = attempt(t_k0);
                        if (rt.built && rt.legal) {
                            if (take(rt, 0)) {
                                out[i].share = share;
                                report(AnomalyKind::WaveformScaled, i, out[i].t_us, out[i].p, share);
                                return kAccepted;
                            }
                            break;
                        }
                        if (!(rt.built && rt.w < 0.5f * referee::kIllegal && rt.w > 0.0f)) break;
                        share *= 0.98f / rt.w;
                    }
                    out[i].p = knots[i].p;
                }
            }
            out[i].t_us = t_k0; out[i].corner = false; out[i].hard = false;
        }
        Piece q; float worst = 0.0f; referee::Ratios parts{};
        uint64_t added = 0;   // the Stretch this knot spent, carried by later segments
        // The budget: this knot's first judge is always made; past its limit
        // (twice the budget for the first knot of a solve) no further judge
        // is, and the search that asked stops with the best legal answer it
        // holds (cut). A re-judge of an answer already judged legal (`again`)
        // restores it and is always made.
        bool fresh = true, cut = false;
        const uint32_t limit = i == settled ? first_budget : budget;
        auto build = [&]() {
            // Every attempt re-solves the junctions: a spend moves this knot
            // in time or position, and the velocity solved for the authored
            // knot does not fit the moved one (a far sample stretched to a
            // reachable time kept the secant of its authored 16 ms and was
            // dropped as unreachable). The rail bound follows the knot too: a
            // Blend trim moves p toward the previous end state, which may be
            // toward a wall.
            junctions(i);
            out[i].v = boundForRail(out[i].v, out[i].p);
            return Piece::hermite(prev_us, prev, out[i].t_us, State{out[i].p, out[i].v, out[i].a});
        };
        auto judgeOnce = [&](bool again) {
            if (!again && !fresh && work >= limit) { cut = true; return false; }
            ++work; fresh = false;
            K2_STAT(judges, 1); q = build(); worst = referee::worstRatio(q, L, lo, hi, &parts);
#ifdef K2_TRACE
            std::printf("  judge i=%zu T=%.1fms prev(p%.6f v%.5f a%.4f) knot(p%.6f v%.5f a%.4f) worst=%.3g\n", i, double(out[i].t_us - prev_us) / 1000.0, prev.p, prev.v, prev.a, out[i].p, out[i].v, out[i].a, worst);
#endif
            return worst <= 1.0f; };
        // The newest sample's free velocity is the chain's natural end,
        // which swings past a velocity mismatch at the knot before it by
        // seven eighths: a first sample solved two percent fast after a
        // brake put the next one four percent slow, past a low acceleration
        // ceiling inside one cadence, and no added time cures that (the
        // secant only falls). Illegal free, the sample takes its piece's
        // secant before any time is spent.
        auto judge = [&](bool again = false) {
            if (judgeOnce(again)) return true;
            if (!cut && knots[i].sample && !knots[i].has_v && !out[i].pin_v && i + 1 >= n && out[i].t_us > prev_us) {
                out[i].pin_v = true;
                out[i].v = boundForRail((out[i].p - prev.p) / (float(out[i].t_us - prev_us) * 1e-6f), out[i].p);
                if (judgeOnce(false)) return true;
                out[i].pin_v = false;
            }
            return false;
        };

        // Reports of an attempt are held until the attempt that stands, so a
        // retried knot is counted once.
        struct Held { AnomalyKind k; float detail; };
        Held held[2]; int n_held = 0;
        auto hold = [&](AnomalyKind k, float detail) { if (n_held < 2) held[n_held++] = Held{k, detail}; };

        // The spends, each found from the referee's ratio, never by a blind
        // search (operator ruling 2026-10-06: a bisection re-solved the chain
        // per step and cost 283 ms on the P4). Blend trims toward the
        // previous end state down to the floor; the ceilings outrank the
        // deadline, so a floor still illegal stretches as well. Stretch moves
        // the knot and every later one by the same amount on top of earlier
        // stretches.
        constexpr int kShareJudges = 4;     // Blend: the estimate and its refinements
        constexpr float kShareGap = 0.005f; // Blend stops within half a percent of the stroke
        constexpr int kTimeJudges = 6;      // Stretch: the estimate and its fixed-factor steps
        constexpr int kTimeRefines = 4;     // Stretch: refinements down from the first legal time
        auto spend = [&]() -> bool {
            n_held = 0;
            out[i].share = 1.0f; out[i].stretched_s = 0.0f; added = 0;
            if (judge()) return true;
            if (cfg.policy == Policy::Blend && !knots[i].sample && !cut) {
                // The shape a trim leaves scales with the share kept, so the
                // ratio is near-linear in it: the share at ratio 1 is about
                // 1 / worst, refined on the secant through the last two
                // judges (the previous end state does not scale, hence the
                // refinement). Aimed a fraction under 1 so the refinement
                // lands legal.
                const float p_full = out[i].p;
                const float floor_s = cfg.amplitude_floor;
                auto trial = [&](float sh, bool again) { out[i].p = prev.p + sh * (p_full - prev.p); return judge(again); };
                auto finite = [](float w) { return w < 0.5f * referee::kIllegal; };
                float s_bad = 1.0f;                  // the smallest share judged illegal
                float s_ok = -1.0f;                  // the largest share judged legal
                float sa = 1.0f, wa = worst;         // the judge before the last
                bool has_a = false;
                float sb = 1.0f, wb = worst;         // the last judge
                float sh = finite(worst) ? 0.998f / worst : 0.5f * (1.0f + floor_s);
                for (int it = 0; it < kShareJudges && !cut; ++it) {
                    const float lo_s = s_ok >= 0.0f ? s_ok : floor_s;
                    if (!(sh > lo_s && sh < s_bad)) sh = s_ok >= 0.0f ? 0.5f * (s_ok + s_bad) : (sh <= floor_s ? floor_s : 0.5f * (floor_s + s_bad));
                    if (sh <= lo_s && s_ok >= 0.0f) break;
                    const bool ok = trial(sh, false);
                    if (cut) break;
                    if (ok) s_ok = sh; else s_bad = sh;
                    sa = sb; wa = wb; has_a = true; sb = sh; wb = worst;
                    if (s_ok >= 0.0f && s_bad - s_ok < kShareGap) break;
                    if (!ok && sh <= floor_s) break;   // the floor itself is illegal
                    // Next: the secant through the last two judges at 0.998.
                    sh = has_a && finite(wa) && finite(wb) && wa != wb ? sb + (0.998f - wb) * (sa - sb) / (wa - wb) : -1.0f;
                }
                const float share = s_ok >= 0.0f ? s_ok : floor_s;
                out[i].share = share;
                hold(AnomalyKind::WaveformScaled, share);
                if (s_ok >= 0.0f) { if (trial(share, true)) return true; }
                else if (!cut && s_bad > floor_s) { if (trial(share, false)) return true; }
                else out[i].p = prev.p + share * (p_full - prev.p);
            }
            for (size_t k = i; k < n; ++k) out[k].base_us = out[k].t_us;
            const uint64_t span = out[i].t_us - prev_us;
            // A LONE sample (the only knot) stretched to four times its own
            // span lands at rest: nothing says where the stream goes next, and
            // the secant Stretch would otherwise minimize time against is the
            // fastest legal arrival (a lone far sample ran past its point by
            // its stop distance). A successor re-solves it moving. A stream's
            // first sample from rest stretches less than that and flies
            // through at the stream's speed (resting there, the next sample
            // could not be reached flying and the ramp crawled).
            const bool rest_when_stretched = i == 0 && n == 1 && knots[i].sample && !knots[i].has_v;
            auto place = [&](uint64_t add, bool again) {
                out[i].t_us = out[i].base_us + add;
                floorSelf(i); placeLater(i);
                if (rest_when_stretched) {
                    const bool r = add >= 3 * span;
                    out[i].pin_v = r; out[i].pin_a = r;
                    if (r) { out[i].v = 0.0f; out[i].a = 0.0f; }
                }
                return judge(again);
            };
            // The time from the ratio that bound: a piece's velocity scales
            // as 1 / T, its acceleration as 1 / T^2 and its jerk as 1 / T^3.
            // The junction values are solved again for every time and the
            // previous end state does not scale, so the estimate is judged
            // and refined, never trusted. Legality is not monotone in the
            // time (a piece given too much time for its distance dips; a
            // sample's free junction moves with its time), so an illegal
            // estimate is followed by a short climb from below before any
            // step past it (a ladder from one tick stepped over the half-tick
            // window a sample has near its speed under a low acceleration
            // ceiling, and a sample arriving during the engine's brake went
            // three seconds out to rest).
            constexpr uint64_t kCap = 8000000;   // 8 s: past this the knot is unreachable
            // The span at ratio 1 for a piece's ratios: T times the velocity
            // ratio, the square root of the acceleration ratio, the cube root
            // of the jerk ratio (below 1 for a legal piece). Doubled for a
            // window excursion, which time alone does not size.
            auto need = [](const referee::Ratios& r) -> float {
                if (!(r.v < 0.5f * referee::kIllegal)) return 2.0f;
                return std::fmax(r.v, std::fmax(std::sqrt(r.a), std::cbrt(r.j)));
            };
            auto finite = [](float w) { return w < 0.5f * referee::kIllegal; };
            auto addFor = [&](uint64_t add, float f) -> uint64_t {
                const uint64_t s = uint64_t(float(span + add) * f * 1.002f) + 1;
                return s > span ? s - span : 0;
            };
            // Where the line through two judged (time, ratio) points crosses
            // `at`; 0 when they do not order.
            auto secantAt = [&](uint64_t a0, float w0, uint64_t a1, float w1, float at) -> uint64_t {
                if (!finite(w0) || !finite(w1) || a0 == a1 || w0 == w1) return 0;
                const float x = float(a0) + (w0 - at) * (float(a1) - float(a0)) / (w0 - w1);
                return x > 0.0f ? uint64_t(x) : 0;
            };
            uint64_t add_ok = kCap, add_bad = 0;
            float w_ok = 0.0f, w_bad = worst;
            referee::Ratios parts_ok{};
            bool found = false;
            auto attempt = [&](uint64_t add) {
                if (add > kCap) add = kCap;
                const bool ok = place(add, false);
                if (ok && add < add_ok) { add_ok = add; w_ok = worst; parts_ok = parts; found = true; }
                else if (!ok && !cut && add > add_bad && add < add_ok) { add_bad = add; w_bad = worst; }
                return ok;
            };
            // 1. The estimate from the ratio that bound.
            const float w0 = worst;
            const uint64_t est = std::max<uint64_t>(addFor(0, need(parts)), 100);
            const bool est_ok = attempt(est);
            const referee::Ratios parts_est = parts;
            const float w_est = worst;
            // 2. Illegal there, for a knot whose junction velocity is free:
            // the first legal window may lie below the estimate (the free
            // velocity moves with the time), so a doubling ladder climbs to
            // it first, from a thirty-second of the estimate (five rungs) and
            // never below a tenth of a tick.
            const bool free_v = !knots[i].has_v && !out[i].pin_v;
            if (!est_ok && !cut && free_v)
                for (uint64_t a = std::max<uint64_t>(100, est / 32); a < est && !cut; a *= 2)
                    if (attempt(a)) break;
            // 3. Nothing below: on from the estimate, by the larger of its
            // ratio's span and the secant through the last two judges, never
            // by less than half again, a bounded number of times.
            uint64_t a_prev = 0, add = est;
            float w_prev = w0, w_add = w_est;
            referee::Ratios r_add = parts_est;
            for (int it = 0; it < kTimeJudges && !found && !cut && add < kCap; ++it) {
                const uint64_t next = std::max(std::max(addFor(add, need(r_add)), secantAt(a_prev, w_prev, add, w_add, 0.99f)), add + add / 2);
                a_prev = add; w_prev = w_add;
                add = next;
                attempt(add);
                r_add = parts; w_add = worst;
            }
            // Refined down toward the largest illegal time, until the two are
            // within a hundredth of the span: the secant through the illegal
            // and legal judges at 0.998 (a ratio smooth in the time), then
            // the midpoint (a ratio that steps: the monotone repair halves
            // the junction acceleration at discrete times), in turn.
            for (int it = 0; it < kTimeRefines && found && !cut; ++it) {
                const uint64_t gap = std::max<uint64_t>(200, (span + add_ok) / 100);
                if (add_ok <= add_bad + gap) break;
                uint64_t r = it % 2 == 0 ? secantAt(add_bad, w_bad, add_ok, w_ok, 0.998f) : 0;
                if (!(r > add_bad && r < add_ok)) r = add_bad + (add_ok - add_bad) / 2;
                attempt(r);
            }
            if (found) place(add_ok, true);
            out[i].stretched_s = float(found ? add_ok : kCap) * 1e-6f; added = found ? add_ok : kCap;
            hold(AnomalyKind::DeadlineStretched, out[i].stretched_s);
            return found;
        };
        // Undo a failed attempt's spends so the next attempt starts clean.
        auto restore = [&]() {
            out[i].p = knots[i].p;
            for (size_t k = i; k < n; ++k) out[k].t_us = out[k].base_us;
            floorSelf(i); placeLater(i);
            if (knots[i].sample && !knots[i].has_v && !(i == 0 && hasPrior(0))) out[i].pin_v = false;
        };
        // Times as they stand now are the base a failed attempt restores to.
        for (size_t k = i; k < n; ++k) out[k].base_us = out[k].t_us;

        // A sample's junction solved again with this knot in view (the
        // junction before it stays as judged): the first sample after a
        // brake, solved alone, ended two percent fast, and under a low
        // acceleration ceiling no spend on the next sample could take that
        // back inside a cadence. Its ladder then found the next legal time
        // seconds out, past the sample and back to rest, and the stream
        // queued behind that; zeroed instead, the stream was a staircase.
        auto relaxJoint = [&]() -> bool {
            if (last == size_t(-1) || out[last].hard || !knots[last].sample || knots[last].has_v) return false;
            if (work >= limit) { cut = true; return false; }
            ++work;
            const bool pv = out[last].pin_v, pa = out[last].pin_a;
            const float ov = out[last].v, oa = out[last].a;
            out[last].pin_v = false; out[last].pin_a = false;
            junctions(last);
            const State np{out[last].p, boundForRail(out[last].v, out[last].p), out[last].a};
            const Piece back = Piece::hermite(pp_us, pp, prev_us, np);
#ifdef K2_TRACE
            std::printf("  relaxJoint i=%zu last=%zu old(v%.5f a%.4f) new(v%.5f a%.4f) back=%.3g\n", i, last, ov, oa, np.v, np.a, referee::worstRatio(back, L, lo, hi));
#endif
            if (referee::worstRatio(back, L, lo, hi) > 1.0f || (np.v == ov && np.a == oa)) {
                out[last].pin_v = pv; out[last].pin_a = pa; out[last].v = ov; out[last].a = oa;
                return false;
            }
            out[last].v = np.v; out[last].a = np.a; out[last].pin_v = out[last].pin_a = true;
            prev = np;
            return true;
        };
        bool legal = spend();
        // A sample stretched past three of its spans is the ladder's far
        // answer; the joint solve is tried first and kept when it stretches
        // less.
#ifdef K2_TRACE
        std::printf("  spent i=%zu legal=%d stretched=%.1fms base_span=%.1fms last=%zd\n", i, int(legal), out[i].stretched_s * 1e3f, double(out[i].base_us - prev_us) / 1000.0, (ptrdiff_t)last);
#endif
        if (legal && knots[i].sample && last != size_t(-1)
            && uint64_t(out[i].stretched_s * 1e6f) >= 3 * (out[i].base_us - prev_us)) {
            const Solved keep_last = out[last]; const State keep_prev = prev; const Solved keep_i = out[i];
            const float keep_worst = worst; const uint64_t keep_added = added;
            const Held keep_held[2] = {held[0], held[1]}; const int keep_n_held = n_held;
            // Back to the far answer, as it was judged (no second spend).
            auto backToFar = [&]() {
                out[last] = keep_last; prev = keep_prev; out[i] = keep_i; placeLater(i);
                worst = keep_worst; added = keep_added;
                held[0] = keep_held[0]; held[1] = keep_held[1]; n_held = keep_n_held;
            };
            restore();   // the joint solve sees this knot at its base time, not the far one
            if (relaxJoint()) {
                const bool l2 = spend();
                if (!(l2 && out[i].stretched_s < keep_i.stretched_s)) backToFar();
            } else backToFar();
        }
        // Backward relaxation. When no spend on this knot makes its piece
        // legal, the fault is the state it starts from: the last accepted
        // junction's acceleration (and, for a free knot, velocity) was chosen
        // with its own piece in view and this one not yet. Zero them when
        // that piece stays legal with the change, and spend again.
        if (!legal && !cut && !rest && last != size_t(-1) && !out[last].hard && !out[last].corner) {
            auto relax = [&](bool alsoV) -> bool {
                State np = prev; np.a = 0.0f; if (alsoV) np.v = 0.0f;
                const Piece back = Piece::hermite(pp_us, pp, prev_us, np);
                ++work;
                if (referee::worstRatio(back, L, lo, hi) > 1.0f) return false;
                out[last].a = 0.0f; out[last].pin_a = true;
                if (alsoV) { out[last].v = 0.0f; out[last].pin_v = true; }
                prev = np;
                return true;
            };
            if (relax(false)) { restore(); legal = spend(); }
            if (!legal && !cut && relaxJoint()) { restore(); legal = spend(); }
            if (!legal && !cut && !knots[last].has_v && relax(true)) { restore(); legal = spend(); }
        }
        // Last resort on this side: an outward junction acceleration at the
        // knot itself.
        if (!legal && !cut && !out[i].pin_a) { out[i].a = 0.0f; out[i].pin_a = true; restore(); legal = spend(); }

        // Cut by the budget, and not the first knot of this solve: it waits
        // for the next solve, which starts with it and the whole budget,
        // nothing reported. Only the first knot of a solve settles for the
        // best legal answer the budget found.
        if (cut && i != settled) {
            restore();
            if (last != size_t(-1)) out[last] = last_entry;
            return kDeferred;
        }
        for (int h = 0; h < n_held; ++h) report(held[h].k, i, out[i].t_us, out[i].p, held[h].detail);
        if (!legal) {
            // Unreachable: drop the knot rather than render past a ceiling.
            // The next piece starts where this one would have.
            restore();
            out[i].dropped = true; out[i].share = 1.0f; out[i].stretched_s = 0.0f; out[i].worst = worst;
            report(AnomalyKind::PlanFailed, i, out[i].t_us, knots[i].p, cut ? kDetailBudget : worst);
            return kDropped;
        }
        out[i].worst = worst;
        out[i].from = prev; out[i].from_us = prev_us;
        pp = prev; pp_us = prev_us; last = i;
        out[i].shift_us = knots[i].sample || out[i].t_us < knots[i].t_us ? 0 : out[i].t_us - knots[i].t_us;
        out[i].own_us = added;
        prev = State{out[i].p, out[i].v, out[i].a};
        prev_us = out[i].t_us;
        return kAccepted;
    };

    size_t stop = n;
    bool accepted = settled > 0;
    for (size_t i = settled; i < n; ++i) {
        if (accepted && work >= budget) { stop = i; break; }
        const int r = solveKnot(i, false);
        if (r == kDeferred) { stop = i; break; }
        if (r == kAccepted) accepted = true;
    }
    for (size_t i = stop; i < n; ++i) {
        out[i].v = knots[i].has_v ? knots[i].v : 0.0f; out[i].a = 0.0f;
        out[i].pin_v = out[i].pin_a = false;
    }
    // A knot whose successors were all dropped is reached moving and the
    // engine brakes from it, exactly as a starved stream: no rest pass.
    K2_STAT(work, work);
    return stop;
}

}  // namespace kinetic2
