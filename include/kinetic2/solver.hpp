// kinetic2/solver.hpp -- the lookahead solver: junction values for every
// pending knot, a referee that finds every extremum of a piece, and the spend
// (Blend or Stretch) when the ceilings cannot honor a knot in its time
// Constraints:
// - Solved over the WHOLE pending window in time order (RFC-105 promise 3):
//   a knot is spent against the state its predecessor actually ends in, so a
//   spend never cascades silently; it is counted once per axis spent.
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

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "engine_piece.hpp"
#include "types.hpp"

namespace kinetic2 {

// ---- the referee -------------------------------------------------------------
namespace referee {

inline constexpr float kIllegal = 1e30f;

inline float polyAt(const float* k, int deg, float t) {
    float r = k[deg];
    for (int i = deg - 1; i >= 0; --i) r = r * t + k[i];
    return r;
}

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
            for (int s = 0; s < 20; ++s) {
                const float m = 0.5f * (lo + hi);
                if ((polyAt(k, deg, m) < 0.0f) == neg_lo) lo = m; else hi = m;
            }
            out[found++] = 0.5f * (lo + hi);
        }
        fa = fb;
    }
    return found;
}

// 0, 1 and every extremum of p, v, a, j of the tau-polynomial c; at most 12.
inline int extremumTaus(const float* c, float* taus) {
    int n = 0;
    taus[n++] = 0.0f;
    taus[n++] = 1.0f;
    float rj1[1]; int nj1 = 0;
    if (std::fabs(c[5]) > 1e-30f) {
        const float t = -c[4] / (5.0f * c[5]);
        if (t > 0.0f && t < 1.0f) rj1[nj1++] = t;
    }
    const float kj[3] = {6.0f * c[3], 24.0f * c[4], 60.0f * c[5]};
    float rj[2]; const int nj = rootsIn01(kj, 2, rj1, nj1, rj);
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
inline float worstRatio(const Piece& q, const Limits& L, float lo, float hi) {
    if (!(q.T > 0.0f)) return 0.0f;   // a hold is legal
    if (!(L.vmax > 0.0f) || !(L.amax > 0.0f) || !(L.jmax > 0.0f)) return kIllegal;
    const float rT = 1.0f / q.T, rT2 = rT * rT, rT3 = rT2 * rT;
    const float rv = 1.0f / L.vmax, ra = 1.0f / L.amax, rj = 1.0f / L.jmax;
    float taus[12];
    const int n = extremumTaus(q.c, taus);
    float worst = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float t = taus[i], t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        const float* c = q.c;
        const float p = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
        if (p < lo - 1e-6f || p > hi + 1e-6f) return kIllegal;
        const float v = (c[1] + 2.0f * c[2] * t + 3.0f * c[3] * t2 + 4.0f * c[4] * t3 + 5.0f * c[5] * t4) * rT;
        const float a = (2.0f * c[2] + 6.0f * c[3] * t + 12.0f * c[4] * t2 + 20.0f * c[5] * t3) * rT2;
        const float j = (6.0f * c[3] + 24.0f * c[4] * t + 60.0f * c[5] * t2) * rT3;
        worst = std::fmax(worst, std::fabs(v) * rv);
        worst = std::fmax(worst, std::fabs(a) * ra);
        worst = std::fmax(worst, std::fabs(j) * rj);
    }
    return worst;
}

}  // namespace referee

// ---- the solved knot ---------------------------------------------------------
// What the solver decided for one knot: where and when the curve really passes,
// and with what velocity and acceleration. p and t start as the knot's own and
// move only by a spend.
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
    // HARD junction: the polynomial head ends at (head_us, head) and the brake
    // profile from there lands at rest on the knot. CORNER (Corner::Cubic): the
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
};

// ---- minimum-jerk junctions ----------------------------------------------------
// The free junction velocities and accelerations of a chain of quintic pieces
// through fixed knot positions are the ones that minimize the chain's jerk
// energy: a quadratic in the unknowns, one banded linear solve. Local
// estimates (monotone slopes, centered-difference accelerations) were tried
// first and whipped a 60 Hz stream into reversals under re-planning; the
// global minimum never amplifies (RFC-105 workflow (aa)).
// Double here on purpose: the per-interval weights scale as 1 / T^5 and float
// cannot hold the spread between a 6 ms and a 600 ms interval.
namespace jerk {

inline constexpr size_t kMaxKnots = 64;
inline constexpr size_t kMaxUnknowns = 2 * kMaxKnots;
inline constexpr int kHalfBand = 4;   // a knot's two unknowns couple with its neighbors' two

// Jerk energy of one quintic Hermite piece as a quadratic form in
// x = [p0, v0, a0, p1, v1, a1] (real units): E = x^T K x.
inline void pieceEnergy(double T, double K[6][6]) {
    // c = M x: the tau-polynomial coefficients c3, c4, c5 (c0..c2 carry no jerk).
    const double T2 = T * T;
    const double M[3][6] = {
        {-10.0, -6.0 * T, -1.5 * T2, 10.0, -4.0 * T, 0.5 * T2},
        {15.0, 8.0 * T, 1.5 * T2, -15.0, 7.0 * T, -T2},
        {-6.0, -3.0 * T, -0.5 * T2, 6.0, -3.0 * T, 0.5 * T2},
    };
    // Gram matrix of the jerk basis (6, 24 tau, 60 tau^2) over 0..1.
    const double G[3][3] = {{36.0, 72.0, 120.0}, {72.0, 192.0, 360.0}, {120.0, 360.0, 720.0}};
    const double w = 1.0 / (T2 * T2 * T);
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) {
            double acc = 0.0;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) acc += M[i][r] * G[i][j] * M[j][c];
            K[r][c] = w * acc;
        }
}

// Knots 0..n-1 after the origin; t in seconds relative to the origin; v and a
// are read for fixed ones and written for free ones.
struct Chain {
    size_t n = 0;
    double t[kMaxKnots] = {}, p[kMaxKnots] = {}, v[kMaxKnots] = {}, a[kMaxKnots] = {};
    bool   fix_v[kMaxKnots] = {}, fix_a[kMaxKnots] = {};
    double p0 = 0.0, v0 = 0.0, a0 = 0.0;   // the origin, fixed
};

inline void solve(Chain& ch) {
    if (ch.n == 0 || ch.n > kMaxKnots) return;
    int idx_v[kMaxKnots], idx_a[kMaxKnots];
    int m = 0;
    for (size_t i = 0; i < ch.n; ++i) {
        idx_v[i] = ch.fix_v[i] ? -1 : m++;
        idx_a[i] = ch.fix_a[i] ? -1 : m++;
    }
    if (m == 0) return;
    // Banded storage: A[r][kHalfBand + (c - r)].
    static thread_local double A[kMaxUnknowns][2 * kHalfBand + 1];
    static thread_local double b[kMaxUnknowns];
    for (int r = 0; r < m; ++r) { b[r] = 0.0; for (int c = 0; c < 2 * kHalfBand + 1; ++c) A[r][c] = 0.0; }
    double K[6][6];
    for (size_t k = 0; k < ch.n; ++k) {
        const double t_prev = k ? ch.t[k - 1] : 0.0;
        const double T = ch.t[k] - t_prev;
        if (!(T > 0.0)) continue;
        pieceEnergy(T, K);
        // x components: index into unknowns (or -1) and their known values.
        int    ux[6];
        double xv[6];
        ux[0] = -1; xv[0] = k ? ch.p[k - 1] : ch.p0;
        ux[1] = k ? idx_v[k - 1] : -1; xv[1] = k ? ch.v[k - 1] : ch.v0;
        ux[2] = k ? idx_a[k - 1] : -1; xv[2] = k ? ch.a[k - 1] : ch.a0;
        ux[3] = -1; xv[3] = ch.p[k];
        ux[4] = idx_v[k]; xv[4] = ch.v[k];
        ux[5] = idx_a[k]; xv[5] = ch.a[k];
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
    // Banded Gaussian elimination, no pivoting: the energy form is positive
    // definite on the unknowns once every position is fixed.
    for (int r = 0; r < m; ++r) {
        const double piv = A[r][kHalfBand];
        if (!(std::fabs(piv) > 1e-300)) continue;
        for (int rr = r + 1; rr <= r + kHalfBand && rr < m; ++rr) {
            const int off = rr - r;   // A[rr][kHalfBand - off] is the entry under the pivot
            const double f = A[rr][kHalfBand - off] / piv;
            if (f == 0.0) continue;
            for (int cc = r; cc <= r + kHalfBand && cc < m; ++cc) {
                const int dr = cc - r, drr = cc - rr;
                A[rr][kHalfBand + drr] -= f * A[r][kHalfBand + dr];
            }
            b[rr] -= f * b[r];
        }
    }
    double x[kMaxUnknowns];
    for (int r = m - 1; r >= 0; --r) {
        double acc = b[r];
        for (int cc = r + 1; cc <= r + kHalfBand && cc < m; ++cc) acc -= A[r][kHalfBand + (cc - r)] * x[cc];
        const double piv = A[r][kHalfBand];
        x[r] = std::fabs(piv) > 1e-300 ? acc / piv : 0.0;
    }
    for (size_t i = 0; i < ch.n; ++i) {
        if (idx_v[i] >= 0) ch.v[i] = x[idx_v[i]];
        if (idx_a[i] >= 0) ch.a[i] = x[idx_a[i]];
    }
}

}  // namespace jerk

// ---- the solver --------------------------------------------------------------
// knots[0..n) pending, in time order; origin is the state the first piece
// starts from. Writes out[0..n) and reports every spend through `report`.
template <typename Report>
inline void solveWindow(const State& origin, uint64_t origin_us, const Knot* knots, size_t n,
                        const Config& cfg, Solved* out, Report&& report) {
    if (n == 0) return;
    const Limits& L = cfg.limits;
    const float lo = std::fmin(0.0f, origin.p), hi = std::fmax(1.0f, origin.p);

    for (size_t i = 0; i < n; ++i) {
        out[i].t_us = knots[i].t_us; out[i].p = knots[i].p;
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
        if (cl != v && !out[i].clamped) { out[i].clamped = true; report(AnomalyKind::EndVelClamped, out[i].t_us, out[i].p, cl); }
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
            // Bounded like an authored velocity: within vmax and stoppable
            // before the rail, since the engine may have to brake from here.
            float v = std::fmax(-L.vmax, std::fmin(L.vmax, secant(i == 0 ? size_t(-1) : i - 1, i)));
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
    auto junctions = [&](size_t first) {
        jerk::Chain ch;
        ch.p0 = origin.p; ch.v0 = origin.v; ch.a0 = origin.a;
        size_t map[jerk::kMaxKnots]; size_t m = 0;
        for (size_t i = 0; i < n && m < jerk::kMaxKnots; ++i) {
            if (out[i].dropped) continue;
            map[m] = i;
            ch.t[m] = double(out[i].t_us - origin_us) * 1e-6;
            ch.p[m] = out[i].p;
            const bool hard = junctionOf(knots[i]) == Junction::Hard;
            if (i < first || hard) { ch.v[m] = out[i].v; ch.a[m] = out[i].a; ch.fix_v[m] = ch.fix_a[m] = true; }
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
                ch.a[m] = out[i].pin_a ? out[i].a : 0.0;
            }
            ++m;
        }
        ch.n = m;
        jerk::solve(ch);
        // Shape preservation: a free velocity may not exceed the monotone
        // band of its two secants (three times the smaller, same sign; zero
        // across a sign change), the Fritsch-Carlson condition, so a kink is
        // never overshot. Accelerations are solved again under the clamped
        // velocities. A stream's secants agree, so the band is wide there and
        // the minimum-jerk velocities pass untouched.
        bool clamped = false;
        for (size_t k = 0; k < m; ++k) {
            if (ch.fix_v[k]) continue;
            const double pl = k ? ch.p[k - 1] : ch.p0, tl = k ? ch.t[k - 1] : 0.0;
            const double dl = (ch.p[k] - pl) / std::fmax(ch.t[k] - tl, 1e-6);
            const double dr = k + 1 < m ? (ch.p[k + 1] - ch.p[k]) / std::fmax(ch.t[k + 1] - ch.t[k], 1e-6) : dl;
            double band = 0.0;
            if (dl == 0.0 || dr == 0.0 || (dl < 0.0) != (dr < 0.0)) band = 0.0;
            else band = 3.0 * std::fmin(std::fabs(dl), std::fabs(dr));
            const double sgn = dl < 0.0 ? -1.0 : 1.0;
            double v = ch.v[k];
            if (band == 0.0) v = 0.0;
            else if ((v < 0.0) != (sgn < 0.0)) v = 0.0;
            else if (std::fabs(v) > band) v = sgn * band;
            // And stoppable before the rail: any knot may turn out to be the
            // last one (a dropped successor, a starved stream) and the engine
            // then brakes from it.
            if (v != 0.0) {
                const double gap = std::fmin(hi - ch.p[k], ch.p[k] - lo);
                auto stopDist = [&](double u) { return u * u / (2.0 * L.amax) + u * L.amax / (2.0 * L.jmax); };
                if (gap <= 1e-6) v = 0.0;
                else if (stopDist(std::fabs(v)) > gap) {
                    double u_lo = 0.0, u_hi = std::fabs(v);
                    for (int it = 0; it < 24; ++it) { const double mid = 0.5 * (u_lo + u_hi); if (stopDist(mid) <= gap) u_lo = mid; else u_hi = mid; }
                    v = (v > 0.0 ? 1.0 : -1.0) * u_lo;
                }
            }
            if (v != ch.v[k]) { ch.v[k] = v; ch.fix_v[k] = true; clamped = true; }
        }
        if (clamped) jerk::solve(ch);
        // Monotone repair: a piece whose knots rise (or fall) must not turn
        // back inside. A velocity sign change inside such a piece is the
        // accelerations' doing; halve both junction accelerations, pin them,
        // solve again; three rounds, then zero.
        for (int round = 0; round < 4; ++round) {
            bool fixed = false;
            for (size_t k = 0; k < m; ++k) {
                const double pl = k ? ch.p[k - 1] : ch.p0, tl = k ? ch.t[k - 1] : 0.0;
                const double vl = k ? ch.v[k - 1] : ch.v0, al = k ? ch.a[k - 1] : ch.a0;
                const double T = ch.t[k] - tl;
                if (!(T > 0.0)) continue;   // a plateau piece (equal knots) must stay flat: checked too
                const Piece q = Piece::hermite(0, State{float(pl), float(vl), float(al)}, uint64_t(T * 1e6 + 0.5),
                                               State{float(ch.p[k]), float(ch.v[k]), float(ch.a[k])});
                // At every extremum inside (0,1) the position must stay within
                // the knots' own band; a position outside it is a turn-back.
                float taus[12]; const int nt = referee::extremumTaus(q.c, taus);
                const float band_lo = float(std::fmin(pl, ch.p[k])) - 1e-5f, band_hi = float(std::fmax(pl, ch.p[k])) + 1e-5f;
                bool turns = false;
                for (int j = 0; j < nt && !turns; ++j) {
                    const float t = taus[j];
                    if (t <= 0.0f || t >= 1.0f) continue;
                    const float pp = q.at(uint64_t(t * T * 1e6)).p;
                    if (pp < band_lo || pp > band_hi) turns = true;
                }
                if (!turns) continue;
                fixed = true;
                if (k) { ch.a[k - 1] = round < 3 ? 0.5 * ch.a[k - 1] : 0.0; ch.fix_a[k - 1] = true; }
                ch.a[k] = round < 3 ? 0.5 * ch.a[k] : 0.0; ch.fix_a[k] = true;
            }
            if (!fixed) break;
            jerk::solve(ch);
        }
        for (size_t k = 0; k < m; ++k) {
            const size_t i = map[k];
            if (i < first) continue;
            out[i].v = float(ch.v[k]);
            out[i].a = float(ch.a[k]);
        }
    };

    State prev = origin;          // the state the piece into knot i starts from
    uint64_t prev_us = origin_us;
    State pp = origin;            // the state the piece into the last accepted knot started from
    uint64_t pp_us = origin_us;
    size_t last = size_t(-1);     // the last accepted knot (dropped ones never count)

    // Solve knot i from prev. rest_end: the knot ends the timeline (everything
    // after it was dropped), so it rests and no backward relaxation runs.
    // Returns false when the knot was dropped.
    auto solveKnot = [&](size_t i, bool rest_end) -> bool {
        junctions(i);
        // HARD: cruise as fast as the head can legally reach, then the fastest
        // legal brake landing at rest exactly on the knot. Bisection on the
        // cruise speed; the head is a plain piece into the brake's start
        // state. Falls through to the smooth path when even a crawl fails.
        if (junctionOf(knots[i]) == Junction::Hard) {
            const float d = out[i].p - prev.p;
            const float sgn = d >= 0.0f ? 1.0f : -1.0f;
            float v_lo = 0.0f, v_hi = L.vmax, v_ok = -1.0f;
            Piece head_ok; uint64_t head_us_ok = 0; State head_s_ok{};
            for (int it = 0; it < 16; ++it) {
                const float vc = it == 0 ? 0.0f : 0.5f * (v_lo + v_hi);
                const Profile br = Profile::brake(State{0.0f, sgn * vc, 0.0f}, 0, L);
                const float tb = br.duration();
                const uint64_t tb_us = uint64_t(tb * 1e6f + 0.5f);
                if (out[i].t_us <= prev_us + tb_us + 1000) { if (it) v_hi = vc; continue; }
                const uint64_t tc = out[i].t_us - tb_us;
                const State hs{out[i].p - br.end().p, sgn * vc, 0.0f};
                const Piece head = Piece::hermite(prev_us, prev, tc, hs);
                if (referee::worstRatio(head, L, lo, hi) <= 1.0f) { v_ok = vc; v_lo = vc; head_ok = head; head_us_ok = tc; head_s_ok = hs; }
                else { if (it == 0) break; v_hi = vc; }
            }
            if (v_ok >= 0.0f) {
                out[i].hard = true; out[i].head_us = head_us_ok; out[i].head = head_s_ok;
                out[i].v = 0.0f; out[i].a = 0.0f; out[i].worst = referee::worstRatio(head_ok, L, lo, hi);
                out[i].from = prev; out[i].from_us = prev_us;
                pp = prev; pp_us = prev_us; last = i;
                prev = State{out[i].p, 0.0f, 0.0f};
                prev_us = out[i].t_us;
                return true;
            }
        }
        if (rest_end) {
            out[i].pin_a = true; out[i].a = 0.0f;
            if (!knots[i].has_v) { out[i].pin_v = true; out[i].v = 0.0f; }
        }
        // CORNER: an authored C1 knot still moving. Each side keeps the
        // acceleration the author's cubic has there; one constant-jerk phase
        // of |da| / jmax, centered on the knot, joins them. The knot is hit at
        // its time with its velocity. Falls through to the smooth path when
        // the head cannot legally reach the ramp's start.
        if (cfg.corner == Corner::Cubic && !rest_end && knots[i].family == Family::C1 && knots[i].has_v
            && knots[i].v != 0.0f && i + 1 < n) {
            const float vk = authored(i);
            const float Tin = float(out[i].t_us - prev_us) * 1e-6f;
            const float Tout = float(out[i + 1].t_us - out[i].t_us) * 1e-6f;
            const float pk = out[i].p, pn = out[i + 1].p, vn = slopeAt(i + 1);
            // Cubic Hermite second derivatives at the shared knot.
            float a_l = (6.0f * (prev.p - pk) + Tin * (2.0f * prev.v + 4.0f * vk)) / (Tin * Tin);
            float a_r = (6.0f * (pn - pk) - Tout * (4.0f * vk + 2.0f * vn)) / (Tout * Tout);
            a_l = std::fmax(-L.amax, std::fmin(L.amax, a_l));
            a_r = std::fmax(-L.amax, std::fmin(L.amax, a_r));
            const float Tr = std::fabs(a_r - a_l) / L.jmax, h = 0.5f * Tr;
            const uint64_t h_us = uint64_t(h * 1e6f + 0.5f);
            if (Tr > 0.0f && out[i].t_us > prev_us + h_us + 1000) {
                const float j = (a_r - a_l) / Tr;
                // Walk the mid state back to the ramp's start.
                const float vs = vk - a_l * h - 0.5f * j * h * h;
                const float ps = pk - vs * h - 0.5f * a_l * h * h - j * h * h * h / 6.0f;
                const State start{ps, vs, a_l};
                const uint64_t ts = out[i].t_us - h_us;
                const Piece head = Piece::hermite(prev_us, prev, ts, start);
                if (referee::worstRatio(head, L, lo, hi) <= 1.0f) {
                    Profile ramp; ramp.start_us = ts; ramp.s0 = start; ramp.n = 1; ramp.dt[0] = Tr; ramp.jerk[0] = j; ramp.ends_at_rest = false;
                    const State exit = Profile::step(start, j, Tr);
                    if (ramp.worstRatio(L, lo, hi) <= 1.0f) {
                        out[i].corner = true; out[i].head_us = ts; out[i].head = start; out[i].ramp = ramp;
                        out[i].t_us = ts + uint64_t(Tr * 1e6f + 0.5f);
                        out[i].p = exit.p; out[i].v = exit.v; out[i].a = exit.a;
                        out[i].worst = referee::worstRatio(head, L, lo, hi);
                        out[i].from = prev; out[i].from_us = prev_us;
                        pp = prev; pp_us = prev_us; last = i;
                        prev = exit; prev_us = out[i].t_us;
                        return true;
                    }
                }
            }
        }

        Piece q; float worst = 0.0f;
        auto build = [&]() {
            // The rail bound follows the knot: a Blend trim moves p toward the
            // previous end state, which may be toward a wall.
            out[i].v = boundForRail(out[i].v, out[i].p);
            return Piece::hermite(prev_us, prev, out[i].t_us, State{out[i].p, out[i].v, out[i].a});
        };
        auto judge = [&]() { q = build(); worst = referee::worstRatio(q, L, lo, hi); return worst <= 1.0f; };

        // Reports of an attempt are held until the attempt that stands, so a
        // retried knot is counted once.
        struct Held { AnomalyKind k; float detail; };
        Held held[2]; int n_held = 0;
        auto hold = [&](AnomalyKind k, float detail) { if (n_held < 2) held[n_held++] = Held{k, detail}; };

        // The spends. Blend trims toward the previous end state down to the
        // floor; the ceilings outrank the deadline, so a floor still illegal
        // stretches as well. Stretch moves the knot and every later one by the
        // same amount on top of earlier stretches, from the analytic bound,
        // doubling while illegal up to a cap, then bisection.
        auto spend = [&]() -> bool {
            n_held = 0;
            out[i].share = 1.0f; out[i].stretched_s = 0.0f;
            if (judge()) return true;
            if (cfg.policy == Policy::Blend) {
                const float p_full = out[i].p;
                float s_lo = cfg.amplitude_floor, s_hi = 1.0f, s_ok = -1.0f;
                for (int it = 0; it < 14; ++it) {
                    const float sh = 0.5f * (s_lo + s_hi);
                    out[i].p = prev.p + sh * (p_full - prev.p);
                    if (judge()) { s_ok = sh; s_lo = sh; } else s_hi = sh;
                }
                const float share = s_ok >= 0.0f ? s_ok : cfg.amplitude_floor;
                out[i].p = prev.p + share * (p_full - prev.p);
                out[i].share = share;
                hold(AnomalyKind::WaveformScaled, share);
                if (judge()) return true;
            }
            for (size_t k = i; k < n; ++k) out[k].base_us = out[k].t_us;
            auto place = [&](uint64_t add) { for (size_t k = i; k < n; ++k) out[k].t_us = out[k].base_us + add; return judge(); };
            const uint64_t span = out[i].t_us - prev_us;
            const float d = std::fabs(out[i].p - prev.p);
            const float t_need = 2.0f * std::fmax(std::fmax(1.875f * d / L.vmax, std::sqrt(5.7735f * d / L.amax)),
                                                  std::cbrt(60.0f * d / L.jmax));
            const uint64_t t_hi = uint64_t(t_need * 1e6f) + 1000;
            uint64_t add_lo = 0, add_hi = t_hi > span ? t_hi - span : 1000;
            constexpr uint64_t kCap = 8000000;   // 8 s: past this the knot is unreachable
            while (add_hi < kCap && !place(add_hi)) add_hi *= 2;
            if (add_hi >= kCap) add_hi = kCap;
            bool found = false;
            if (place(add_hi)) {
                found = true;
                for (int it = 0; it < 18 && add_hi > add_lo + 1; ++it) {
                    const uint64_t mid = (add_lo + add_hi) / 2;
                    if (place(mid)) add_hi = mid; else add_lo = mid;
                }
            }
            place(add_hi);
            out[i].stretched_s = float(add_hi) * 1e-6f;
            hold(AnomalyKind::DeadlineStretched, out[i].stretched_s);
            return found;
        };
        // Undo a failed attempt's spends so the next attempt starts clean.
        auto restore = [&]() {
            out[i].p = knots[i].p;
            for (size_t k = i; k < n; ++k) out[k].t_us = out[k].base_us;
        };
        // Times as they stand now are the base a failed attempt restores to.
        for (size_t k = i; k < n; ++k) out[k].base_us = out[k].t_us;

        bool legal = spend();
        // Backward relaxation. When no spend on this knot makes its piece
        // legal, the fault is the state it starts from: the last accepted
        // junction's acceleration (and, for a free knot, velocity) was chosen
        // with its own piece in view and this one not yet. Zero them when
        // that piece stays legal with the change, and spend again.
        if (!legal && !rest_end && last != size_t(-1) && !out[last].hard) {
            auto relax = [&](bool alsoV) -> bool {
                State np = prev; np.a = 0.0f; if (alsoV) np.v = 0.0f;
                const Piece back = Piece::hermite(pp_us, pp, prev_us, np);
                if (referee::worstRatio(back, L, lo, hi) > 1.0f) return false;
                out[last].a = 0.0f; out[last].pin_a = true;
                if (alsoV) { out[last].v = 0.0f; out[last].pin_v = true; }
                prev = np;
                return true;
            };
            if (relax(false)) { restore(); legal = spend(); }
            if (!legal && !knots[last].has_v && relax(true)) { restore(); legal = spend(); }
        }
        // Last resort on this side: an outward junction acceleration at the
        // knot itself.
        if (!legal && !out[i].pin_a) { out[i].a = 0.0f; out[i].pin_a = true; restore(); legal = spend(); }

        for (int h = 0; h < n_held; ++h) report(held[h].k, out[i].t_us, out[i].p, held[h].detail);
        if (!legal) {
            // Unreachable: drop the knot rather than render past a ceiling.
            // The next piece starts where this one would have.
            restore();
            out[i].dropped = true; out[i].share = 1.0f; out[i].stretched_s = 0.0f; out[i].worst = worst;
            report(AnomalyKind::PlanFailed, out[i].t_us, knots[i].p, worst);
            return false;
        }
        out[i].worst = worst;
        out[i].from = prev; out[i].from_us = prev_us;
        pp = prev; pp_us = prev_us; last = i;
        prev = State{out[i].p, out[i].v, out[i].a};
        prev_us = out[i].t_us;
        return true;
    };

    for (size_t i = 0; i < n; ++i) solveKnot(i, false);
    // A knot whose successors were all dropped is reached moving and the
    // engine brakes from it, exactly as a starved stream: no rest pass.
}

}  // namespace kinetic2
