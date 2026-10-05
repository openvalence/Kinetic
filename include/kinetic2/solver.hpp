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
};

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
    }

    // Monotone slope at knot i from its solved neighbors (the origin on the left
    // of knot 0, nothing past the last knot).
    auto secant = [&](size_t a, size_t b) {   // from a to b, b = a + 1; a == npos is the origin
        const float pa = a == size_t(-1) ? origin.p : out[a].p;
        const uint64_t ta = a == size_t(-1) ? origin_us : out[a].t_us;
        return (out[b].p - pa) / (float(out[b].t_us - ta) * 1e-6f);
    };
    // An authored velocity is honored within vmax and never through a rail:
    // at a rail an outward velocity becomes 0 (EndVelClamped, once per knot).
    auto authored = [&](size_t i) -> float {
        const float v = knots[i].v;
        const float p = out[i].p;
        const bool outward = (v > 0.0f && p >= hi - 1e-6f) || (v < 0.0f && p <= lo + 1e-6f);
        const float cl = outward ? 0.0f : std::fmax(-L.vmax, std::fmin(L.vmax, v));
        if (cl != v && !out[i].clamped) { out[i].clamped = true; report(AnomalyKind::EndVelClamped, out[i].t_us, p, cl); }
        return cl;
    };
    auto slopeAt = [&](size_t i) -> float {
        const Knot& k = knots[i];
        if (k.has_v) return authored(i);
        if (i + 1 >= n) return 0.0f;   // nothing after: the curve rests here
        const float dl = secant(i == 0 ? size_t(-1) : i - 1, i), dr = secant(i, i + 1);
        if (dl == 0.0f || dr == 0.0f || (dl < 0.0f) != (dr < 0.0f)) return 0.0f;
        const float hl = float(out[i].t_us - (i == 0 ? origin_us : out[i - 1].t_us)) * 1e-6f;
        const float hr = float(out[i + 1].t_us - out[i].t_us) * 1e-6f;
        // Fritsch-Butland: a weighted harmonic mean, shape-preserving.
        return 3.0f * (hl + hr) / ((2.0f * hr + hl) / dl + (hr + 2.0f * hl) / dr);
    };
    auto accelAt = [&](size_t i, float vi) -> float {
        if (junctionOf(knots[i]) == Junction::Hard) return 0.0f;
        if (i + 1 >= n) return 0.0f;
        const float vl = i == 0 ? origin.v : out[i - 1].v;
        const float vr = slopeAt(i + 1);
        const uint64_t tl = i == 0 ? origin_us : out[i - 1].t_us;
        (void)vi;
        return (vr - vl) / (float(out[i + 1].t_us - tl) * 1e-6f);
    };

    State prev = origin;
    uint64_t prev_us = origin_us;
    for (size_t i = 0; i < n; ++i) {
        auto build = [&]() {
            out[i].v = slopeAt(i);
            out[i].a = accelAt(i, out[i].v);
            return Piece::hermite(prev_us, prev, out[i].t_us, State{out[i].p, out[i].v, out[i].a});
        };
        Piece q = build();
        float worst = referee::worstRatio(q, L, lo, hi);
        if (worst > 1.0f) {
            if (cfg.policy == Policy::Blend) {
                // Trim the stroke toward where the previous piece ends, never
                // below the floor: bisection on the kept share.
                const float p_full = knots[i].p;
                float s_lo = cfg.amplitude_floor, s_hi = 1.0f, s_ok = -1.0f;
                for (int it = 0; it < 14; ++it) {
                    const float s = 0.5f * (s_lo + s_hi);
                    out[i].p = prev.p + s * (p_full - prev.p);
                    q = build();
                    const float w = referee::worstRatio(q, L, lo, hi);
                    if (w <= 1.0f) { s_ok = s; s_lo = s; worst = w; } else s_hi = s;
                }
                if (s_ok < 0.0f) {
                    out[i].p = prev.p + cfg.amplitude_floor * (p_full - prev.p);
                    q = build();
                    worst = referee::worstRatio(q, L, lo, hi);
                    out[i].share = cfg.amplitude_floor;
                    report(AnomalyKind::WaveformScaled, out[i].t_us, out[i].p, out[i].share);
                    report(AnomalyKind::PlanFailed, out[i].t_us, out[i].p, worst);
                } else {
                    out[i].p = prev.p + s_ok * (p_full - prev.p);
                    q = build();
                    out[i].share = s_ok;
                    report(AnomalyKind::WaveformScaled, out[i].t_us, out[i].p, s_ok);
                }
            } else {
                // Move the knot later, and every knot after it by the same
                // amount, until the piece is legal: bisection on added time,
                // up to four times the interval.
                const uint64_t span = out[i].t_us - prev_us;
                // The least time a rest-to-rest stroke of this length can take
                // under each ceiling bounds the search; 2x covers moving ends.
                const float d = std::fabs(out[i].p - prev.p);
                const float t_need = 2.0f * std::fmax(std::fmax(1.875f * d / L.vmax, std::sqrt(5.7735f * d / L.amax)),
                                                      std::cbrt(60.0f * d / L.jmax));
                const uint64_t t_hi = uint64_t(t_need * 1e6f) + 1000;
                uint64_t add_lo = 0, add_hi = t_hi > span ? t_hi - span : 1, add_ok = 0;
                bool found = false;
                for (int it = 0; it < 16; ++it) {
                    const uint64_t add = (add_lo + add_hi) / 2;
                    for (size_t k = i; k < n; ++k) out[k].t_us = knots[k].t_us + add;
                    q = build();
                    const float w = referee::worstRatio(q, L, lo, hi);
                    if (w <= 1.0f) { add_ok = add; add_hi = add; found = true; worst = w; } else add_lo = add + 1;
                    if (add_hi <= add_lo) break;
                }
                const uint64_t add = found ? add_ok : add_hi;
                for (size_t k = i; k < n; ++k) out[k].t_us = knots[k].t_us + add;
                q = build();
                worst = referee::worstRatio(q, L, lo, hi);
                out[i].stretched_s = float(add) * 1e-6f;
                report(AnomalyKind::DeadlineStretched, out[i].t_us, out[i].p, out[i].stretched_s);
                if (!found) report(AnomalyKind::PlanFailed, out[i].t_us, out[i].p, worst);
            }
        }
        out[i].worst = worst;
        prev = State{out[i].p, out[i].v, out[i].a};
        prev_us = out[i].t_us;
    }
}

}  // namespace kinetic2
