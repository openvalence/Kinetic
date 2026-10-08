// kinetic2/handles.hpp -- the handle renderer: a composite cubic Bezier in the
// time-position plane, one angle and two handle lengths per knot, the ceilings
// as bounds on the lengths, amplitude trimmed toward the previous knot
// Constraints:
// - The reference is playground/handles-model.js; a change here changes it
//   first, and the parity test (step C of kin-y6e) holds the two together.
// - u is the curve parameter, never time: v = (dp/du)/(dt/du).
// - Knot times are float seconds from the caller's origin (the caller rebases
//   its uint64_t clock); positions and ceilings share the caller's units.
// - No allocation, no thread_local, no static locals: the solver calls this
//   on the motion tick.
// - A judge reads each of v, a, j at 11 u samples and at its exact turning
//   points (the roots of the next derivative's numerator, bracketed and
//   bisected), about 150 polynomial evaluations against the model's 161 full
//   samples; a legality test stops at the first sample over the bar, the
//   length walk skips the side of 1 that cannot help (lengths of a third
//   only), and a piece whose inputs did not change is not judged again
//   (HKnot memo). A bound on the render's work belongs in the solver, never
//   this file.
// - The model carries Cfg::trimLast and Cfg::railStop (kin-88m): under
//   railStop both hold a trimmed knot's angle to its trimmed chord, cap an
//   angle its span cannot stop, fit the corner ramps' room (Room, aTarget),
//   rank a window excursion below every ceiling ratio, read a hold by rate as
//   well as by distance, and lay an illegal hold flat. HKnot::slack,
//   HKnot::rail, the last knot's brake cap on HKnot::vcap, a moving origin's
//   HKnot::aIn and Piece::da are the engine's (solver.hpp renderRun,
//   engine_piece.hpp); the model has none of them. At their defaults the
//   render is the model's, except that an illegal hold lies flat with
//   railStop off too, smoothness 0 judges monotonicity with railStop off too
//   (the model under railStop only), and overOf judges each peak at its exact
//   turning points where the model samples: the two agree within kTol, which
//   can move a trim that sits on a legality boundary.
// - At smoothness 0 a band-legal piece stays monotone at its scaled lengths
//   (monoOver, kin-ay9): the band guarantees it only at a third. Above 0 only
//   the window judges overshoot.
// - Smoothness (Valence RFC-108 item 6): 0 renders the pchip solve and 1 the
//   smooth solve, bit for bit; between, both are solved and every knot takes
//   (1 - s) pchip + s smooth of its free angle and both lengths, then the
//   ceilings fit and trim the lerped curve. Cost: a mid value solves twice.
// - The solve order (RFC-106 item 7): every angle first, with every length a
//   third (authored kept, else the style's start angle; each held to vmax and
//   its cap, an authored one under railStop to the rail, the origin's live
//   angle never), then four G2 sweeps (solveStyle). Then per piece in knot order
//   one length factor on the k grid (ksAt: 1, 0.95, 1.05, ... 0.2, then 1.85
//   to 2.0, the lower first on a tie), the first legal taken, else the trim
//   (nudge). Then two rounds of solve on the trimmed chords and fit again,
//   plus one round for each last round that capped an angle or asked an end
//   acceleration, at most four (render). The solver renders again with
//   tightened ceilings, at most kSlackPasses (3) times (solver.hpp
//   renderRun). Angles are solved at a third, so a G2 knot whose piece scales
//   loses the match and is G1: the solver rounds its acceleration step with a
//   corner ramp at jmax.
// - Parity with the model (tests/test_kinetic2_handles.cpp): trims within
//   1e-4 of the window; the engine's positions within 0.5% and its trims
//   within 1e-4, both past the corner-ramp allowance.
// See: Valence RFC-106, Kinetic kin-y6e
#pragma once

#include <cmath>
#include <cstdint>

#include "types.hpp"

namespace kinetic2::handles {

inline constexpr float kLMin   = 0.05f;   // shortest handle, share of its span
inline constexpr float kLMax   = 0.95f;   // longest handle
inline constexpr float kThird  = 1.0f / 3.0f;  // the polynomial cubic (the cubic Hermite)
inline constexpr float kTol    = 1e-3f;   // a peak within 0.1% of its ceiling is legal
inline constexpr int   kJudgeN = 160;     // u samples per judged piece; the model's NS
inline constexpr float kTick   = 1e-3f;   // the engine's tick, s: a corner ramp's clearance from a span's end
inline constexpr int   kMemoKey = 16;     // nudge's memo inputs per piece; every input of a fit is in it

enum class KnotClass : uint8_t { End = 0, Rest = 1, Crest = 2, Through = 3 };

// The solver fills lfloor, trim and smoothness from Config (handle_floor,
// trim_max times the window span, smoothness).
struct Cfg {
    Limits lim{};
    float  lo      = 0.0f;     // window, caller units
    float  hi      = 1.0f;
    float  holdEps = 0.005f;   // a chord this small is a hold
    float  lfloor  = 0.15f;    // feel floor on a nudged handle length
    float  trim    = 1.0f;     // farthest a knot moves toward its predecessor
    // Free knots: 0 pchip (crests and hold edges flat and G1, through points
    // G2 by angle, no overshoot between monotone knots); 1 smooth (crests take
    // Makima's angle, through points start from it and are G2 by angle, crests
    // and hold edges G2 by their lengths where both sides accelerate the same
    // way, else G1); the lerp between.
    float  smoothness = 0.0f;
    int    sweeps  = 4;        // G2 sweeps per solve
    // The last knot trims too. The model keeps it (a script's end); the kernel's
    // last knot is the newest of a window and must never render over a ceiling.
    bool   trimLast = false;
    // An authored angle is also held to what the fastest legal brake stops
    // before the nearer wall (the engine brakes from a last knot still moving).
    bool   railStop = false;
};

// One knot of a render. The caller fills t, p, v, has_v; render() fills the rest.
struct HKnot {
    float t = 0.0f;            // s from the caller's origin, strictly increasing
    float p = 0.0f;            // authored position
    float v = 0.0f;            // authored angle, meaningful when has_v
    bool  has_v = false;
    // solved
    KnotClass cls = KnotClass::End;
    bool  g2 = false;
    float dIn = 0.0f, dOut = 0.0f;   // chords, units/s
    float vel = 0.0f;                // the angle
    float lIn = kThird, lOut = kThird;      // solved lengths
    float effIn = kThird, effOut = kThird;  // lengths after the ceilings
    float dp = 0.0f;                 // trim; nonzero = this knot moved
    bool  infeasible = false;        // the piece ending here is over a ceiling
    // The share of each ceiling (v, a, j) the piece ending here is judged
    // against. 1 in the model; the kernel tightens the one the piece it builds
    // (an acceleration step rounded under jmax at an end) goes over.
    float slack[3] = {1.0f, 1.0f, 1.0f};
    // The share of the railStop speed bound an authored angle is held to; the
    // kernel lowers it when the brake from the rendered state leaves the window.
    float rail = 1.0f;
    // The fastest angle the span after this knot stops legally when its
    // successor trims onto it (railStop only; lowered by nudge, kept across
    // rounds and passes).
    float vcap = INFINITY;
    // nudge's memo of the piece ending here: its inputs and its fit (a round
    // or pass that changes none of them reuses it, bit for bit).
    bool  memo = false;
    float aIn = 0.0f;                // the fitted piece into this knot ends in this acceleration
    // The end acceleration the piece out of this knot asks of the piece into
    // it (its start ramp did not fit), and how near (railStop only; kept).
    float aTarget = NAN, aTol = INFINITY;
    float mk[kMemoKey] = {};
    float mi0 = kThird, mi1 = kThird, mdp = 0.0f, mvel = 0.0f, maIn = 0.0f;
    bool  mlegal = false;
    // The pchip solve's angle and lengths, held while a mid smoothness solves smooth.
    float pVel = 0.0f, pIn = kThird, pOut = kThird;
};

inline float rendered(const HKnot& k) { return k.p + k.dp; }

// ---- one piece ----------------------------------------------------------------
// D over T s, end angles s0/s1, handle lengths i0/i1; relative to its start.
struct Piece {
    float T = 0.0f, D = 0.0f, s0 = 0.0f, s1 = 0.0f, i0 = kThird, i1 = kThird;
    // The kernel's start correction: da * t^2/2 * (1 - t/T)^3 added in time,
    // so the start acceleration is the Bezier's plus da while p, v at both
    // ends and the end acceleration stay the Bezier's. 0 in the model.
    float da = 0.0f;
};

struct Eval {
    float t = 0.0f, p = 0.0f, v = 0.0f, a = 0.0f, j = 0.0f;
};

inline Eval evalPiece(const Piece& q, float u) {
    const float t1 = q.i0 * q.T, t2 = q.T - q.i1 * q.T, t3 = q.T;
    const float p1 = q.s0 * q.i0 * q.T, p2 = q.D - q.s1 * q.i1 * q.T, p3 = q.D;
    const float w = 1.0f - u;
    const float b1 = 3.0f * u * w * w, b2 = 3.0f * u * u * w, b3 = u * u * u;
    const float c0 = 3.0f * w * w, c1 = 6.0f * u * w, c2 = 3.0f * u * u;
    const float tp  = c0 * t1 + c1 * (t2 - t1) + c2 * (t3 - t2);
    const float pp  = c0 * p1 + c1 * (p2 - p1) + c2 * (p3 - p2);
    const float tpp = 6.0f * (w * (t2 - 2.0f * t1) + u * (t3 - 2.0f * t2 + t1));
    const float ppp = 6.0f * (w * (p2 - 2.0f * p1) + u * (p3 - 2.0f * p2 + p1));
    const float tp3 = 6.0f * (t3 - 3.0f * t2 + 3.0f * t1);
    const float pp3 = 6.0f * (p3 - 3.0f * p2 + 3.0f * p1);
    const float nA = ppp * tp - pp * tpp;
    const float tp2 = tp * tp, tp3c = tp2 * tp;
    Eval e;
    e.t = b1 * t1 + b2 * t2 + b3 * t3;
    e.p = b1 * p1 + b2 * p2 + b3 * p3;
    e.v = pp / tp;
    e.a = nA / tp3c;
    e.j = ((pp3 * tp - pp * tp3) * tp - 3.0f * nA * tpp) / (tp3c * tp2);
    if (q.da != 0.0f) {
        const float tau = e.t / q.T, w1 = 1.0f - tau;
        e.p += q.da * 0.5f * e.t * e.t * w1 * w1 * w1;
        e.v += q.da * q.T * (tau * w1 * w1 * w1 - 1.5f * tau * tau * w1 * w1);
        e.a += q.da * (w1 * w1 * w1 - 6.0f * tau * w1 * w1 + 3.0f * tau * tau * w1);
        e.j += q.da * (-9.0f * w1 * w1 + 18.0f * tau * w1 - 3.0f * tau * tau) / q.T;
    }
    return e;
}

// The u where t(u) = t (t relative to the piece start). Newton seeded from
// t / T, kept inside a bisection bracket; bounded iterations.
inline float solveU(const Piece& q, float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= q.T) return 1.0f;
    float lo = 0.0f, hi = 1.0f;
    float u = t / q.T;
    const float t1 = q.i0 * q.T, t2 = q.T - q.i1 * q.T, t3 = q.T;
    for (int n = 0; n < 24; ++n) {
        const float w = 1.0f - u;
        const float f = 3.0f * u * w * w * t1 + 3.0f * u * u * w * t2 + u * u * u * t3 - t;
        if (std::fabs(f) <= 1e-7f * q.T) break;
        if (f > 0.0f) hi = u; else lo = u;
        const float d = 3.0f * w * w * t1 + 6.0f * u * w * (t2 - t1) + 3.0f * u * u * (t3 - t2);
        float un = (d > 0.0f) ? u - f / d : 0.5f * (lo + hi);
        if (!(un > lo && un < hi)) un = 0.5f * (lo + hi);
        if (hi - lo <= 1e-7f) break;
        u = un;
    }
    return u;
}

// The worst ceiling ratio of a piece starting at pStart, sampled in u like the
// model; leaving the window counts as 1 + the excursion over the window span.
// With a finite stop, returns a ratio over stop as soon as one sample is (a
// legality test needs no more).
// Each ceiling's ratio apart (x: 1 + the window excursion over its span).
struct Over {
    float v = 0.0f, a = 0.0f, j = 0.0f, x = 1.0f;
};

// The piece's derivative polynomials in u, computed once per judge: T'(u) and
// P'(u) are quadratics (ta0 + ta1 u + ta2 u^2), their second derivatives
// linear, their third constant. v = P'/T', a = nA/T'^3, j = nJ/T'^5 with
// nA = P''T' - P'T'' (cubic) and nJ = (P'''T' - P'T''')T' - 3 nA T'' (quartic).
struct Coef {
    float ta0, ta1, ta2, pa0, pa1, pa2;
    float t1, t2, t3, p1, p2, p3;
};
inline Coef coefOf(const Piece& q) {
    Coef k;
    k.t1 = q.i0 * q.T; k.t2 = q.T - q.i1 * q.T; k.t3 = q.T;
    k.p1 = q.s0 * q.i0 * q.T; k.p2 = q.D - q.s1 * q.i1 * q.T; k.p3 = q.D;
    const float d0 = k.t1, d1 = k.t2 - k.t1, d2 = k.t3 - k.t2;
    const float e0 = k.p1, e1 = k.p2 - k.p1, e2 = k.p3 - k.p2;
    k.ta0 = 3.0f * d0; k.ta1 = 6.0f * (d1 - d0); k.ta2 = 3.0f * (d0 - 2.0f * d1 + d2);
    k.pa0 = 3.0f * e0; k.pa1 = 6.0f * (e1 - e0); k.pa2 = 3.0f * (e0 - 2.0f * e1 + e2);
    return k;
}
struct Deriv { float tp, pp, tpp, ppp, nA, nJ; };
inline Deriv derivAt(const Coef& k, float u) {
    Deriv d;
    d.tp  = k.ta0 + u * (k.ta1 + u * k.ta2);
    d.pp  = k.pa0 + u * (k.pa1 + u * k.pa2);
    d.tpp = k.ta1 + 2.0f * k.ta2 * u;
    d.ppp = k.pa1 + 2.0f * k.pa2 * u;
    d.nA  = d.ppp * d.tp - d.pp * d.tpp;
    d.nJ  = (2.0f * k.pa2 * d.tp - d.pp * 2.0f * k.ta2) * d.tp - 3.0f * d.nA * d.tpp;
    return d;
}
// The numerator of dj/du: nJ' T' - 5 nJ T''. With g = 2 pa2 T' - 2 ta2 P'
// (which is also nA'), nJ = g T' - 3 nA T'' and nJ' = g' T' - 2 g T'' - 6 ta2 nA.
inline float jSlopeNum(const Coef& k, float u) {
    const Deriv d = derivAt(k, u);
    const float g = 2.0f * k.pa2 * d.tp - 2.0f * k.ta2 * d.pp;
    const float gp = 2.0f * k.pa2 * d.tpp - 2.0f * k.ta2 * d.ppp;
    const float nJp = gp * d.tp - 2.0f * g * d.tpp - 6.0f * k.ta2 * d.nA;
    return nJp * d.tp - 5.0f * d.nJ * d.tpp;
}
inline float vAt(const Deriv& d) { return d.pp / d.tp; }
inline float aAt(const Deriv& d) { return d.nA / (d.tp * d.tp * d.tp); }
inline float jAt(const Deriv& d) { const float t2 = d.tp * d.tp; return d.nJ / (t2 * t2 * d.tp); }

// The roots of f in (0, 1) bracketed on nseg equal segments, each bisected
// four times then two secant steps; a root pair inside one segment is missed, which the caller's
// sampled maxima cover. Returns the count written to r (at most cap).
template <typename F>
inline int rootsOf(F&& f, int nseg, float* r, int cap) {
    int n = 0;
    float u0 = 0.0f, f0 = f(0.0f);
    for (int s = 1; s <= nseg && n < cap; ++s) {
        const float u1 = static_cast<float>(s) / static_cast<float>(nseg), f1 = f(u1);
        if ((f0 < 0.0f) != (f1 < 0.0f)) {
            float lo = u0, hi = u1, flo = f0, fhi = f1;
            for (int it = 0; it < 4; ++it) {
                const float mid = 0.5f * (lo + hi), fm = f(mid);
                if ((fm < 0.0f) == (flo < 0.0f)) { lo = mid; flo = fm; } else { hi = mid; fhi = fm; }
            }
            // Two secant steps inside the bracket finish what four halvings started.
            for (int it = 0; it < 2; ++it) {
                const float den = fhi - flo;
                float x = den != 0.0f ? lo - flo * (hi - lo) / den : 0.5f * (lo + hi);
                if (!(x > lo && x < hi)) x = 0.5f * (lo + hi);
                const float fx = f(x);
                if ((fx < 0.0f) == (flo < 0.0f)) { lo = x; flo = fx; } else { hi = x; fhi = fx; }
            }
            r[n++] = 0.5f * (lo + hi);
        }
        u0 = u1; f0 = f1;
    }
    return n;
}

inline float overSampled(const Piece& q, float pStart, const Cfg& c, float stop, Over* parts);

// The worst ceiling ratio of a piece: each of v, a, j at its exact turning
// points (the roots of the next derivative's numerator) and at 11 samples, so
// a sampled model and this judge agree within the tolerance at a fraction of
// the cost; the window from the position's exact turning points. A piece with
// a start correction (da) is sampled like the model.
inline float overOf(const Piece& q, float pStart, const Cfg& c, float stop = INFINITY, Over* parts = nullptr) {
    if (q.da != 0.0f) return overSampled(q, pStart, c, stop, parts);
    const float sv = c.lim.vmax * stop, sa = c.lim.amax * stop, sj = c.lim.jmax * stop;
    const Coef k = coefOf(q);
    float r[6];
    float pv = 0.0f, pa = 0.0f, pj = 0.0f;
    // The ends first, then the middle, then the quarters: a piece over the bar
    // at an end (where short handles spike the jerk) shows at the first sample.
    static constexpr float kOrder[11] = {0.0f, 1.0f, 0.5f, 0.2f, 0.8f, 0.4f, 0.6f, 0.1f, 0.9f, 0.3f, 0.7f};
    // Division-free: each ratio is compared as its numerator against the bar
    // scaled by the matching power of T'(u) (positive on a monotone piece);
    // the peaks themselves are divided once, after the scan.
    float mv = 0.0f, ma = 0.0f, mj = 0.0f;   // the sample maxima, as v, a, j
    for (int s = 0; s <= 10; ++s) {
        const Deriv d = derivAt(k, kOrder[s]);
        const float t2 = d.tp * d.tp, t3 = t2 * d.tp, t5 = t3 * t2;
        const float av = std::fabs(d.pp), aa = std::fabs(d.nA), aj = std::fabs(d.nJ);
        if (av > sv * d.tp || aa > sa * t3 || aj > sj * t5) return 2.0f * stop;
        const float v = av / d.tp, a = aa / t3, j = aj / t5;
        mv = std::fmax(mv, v); ma = std::fmax(ma, a); mj = std::fmax(mj, j);
    }
    pv = mv; pa = ma; pj = mj;
    // Then each peak exactly: |v| where a = 0 (nA, a cubic), |a| where j = 0
    // (nJ, a quartic), |j| where dj/du = 0 (a quintic).
    int n = rootsOf([&](float u) { return derivAt(k, u).nA; }, 6, r, 3);
    for (int i = 0; i < n; ++i) pv = std::fmax(pv, std::fabs(vAt(derivAt(k, r[i]))));
    if (pv > sv) return 2.0f * stop;
    n = rootsOf([&](float u) { return derivAt(k, u).nJ; }, 8, r, 4);
    for (int i = 0; i < n; ++i) pa = std::fmax(pa, std::fabs(aAt(derivAt(k, r[i]))));
    if (pa > sa) return 2.0f * stop;
    n = rootsOf([&](float u) { return jSlopeNum(k, u); }, 10, r, 5);
    for (int i = 0; i < n; ++i) pj = std::fmax(pj, std::fabs(jAt(derivAt(k, r[i]))));
    if (pj > sj) return 2.0f * stop;
    float pMin = std::fmin(pStart, pStart + q.D), pMax = std::fmax(pStart, pStart + q.D);
    {
        // The position's turning points exactly (dp/du is a quadratic).
        const float A = k.p1, B = k.p2 - k.p1, C = k.p3 - k.p2;
        const float qa = A - 2.0f * B + C, qb = 2.0f * (B - A);
        int nr = 0;
        if (std::fabs(qa) > 1e-12f) {
            const float disc = qb * qb - 4.0f * qa * A;
            if (disc >= 0.0f) {
                const float sq = std::sqrt(disc);
                r[nr++] = (-qb - sq) / (2.0f * qa);
                r[nr++] = (-qb + sq) / (2.0f * qa);
            }
        } else if (std::fabs(qb) > 1e-12f) {
            r[nr++] = -A / qb;
        }
        for (int i = 0; i < nr; ++i) {
            if (!(r[i] > 0.0f && r[i] < 1.0f)) continue;
            const float p = pStart + evalPiece(q, r[i]).p;
            pMin = std::fmin(pMin, p);
            pMax = std::fmax(pMax, p);
        }
    }
    const float out = std::fmax(0.0f, std::fmax(c.lo - pMin, pMax - c.hi));
    if (parts) *parts = Over{pv / c.lim.vmax, pa / c.lim.amax, pj / c.lim.jmax, 1.0f + out / (c.hi - c.lo)};
    const float o = std::fmax(pv / c.lim.vmax, std::fmax(pa / c.lim.amax, pj / c.lim.jmax));
    const float wall = c.railStop && out > kTol * (c.hi - c.lo) ? 1e3f : 1.0f;
    return std::fmax(o, wall + out / (c.hi - c.lo));
}

// The model's judge: every u sample of the piece. Kept for a piece with a
// start correction, whose turning points are not the polynomials'.
inline float overSampled(const Piece& q, float pStart, const Cfg& c, float stop, Over* parts) {
    const float sv = c.lim.vmax * stop, sa = c.lim.amax * stop, sj = c.lim.jmax * stop;
    float pv = 0.0f, pa = 0.0f, pj = 0.0f, pMin = pStart, pMax = pStart;
    // Every 16th sample first: a peak over the bar shows early. The maxima do
    // not depend on the order.
    for (int pass = 0; pass < 2; ++pass)
    for (int k = 0; k <= kJudgeN; ++k) {
        if ((k % 16 == 0) != (pass == 0)) continue;
        const Eval e = evalPiece(q, static_cast<float>(k) / kJudgeN);
        pv = std::fmax(pv, std::fabs(e.v));
        pa = std::fmax(pa, std::fabs(e.a));
        pj = std::fmax(pj, std::fabs(e.j));
        if (pv > sv || pa > sa || pj > sj) return 2.0f * stop;
        pMin = std::fmin(pMin, pStart + e.p);
        pMax = std::fmax(pMax, pStart + e.p);
    }
    // The position's turning points exactly (dp/du is a quadratic): a long
    // piece's u samples are coarser than the 1 ms grid that reads the wall.
    if (q.da == 0.0f) {
        const float A = q.s0 * q.i0 * q.T, B = (q.D - q.s1 * q.i1 * q.T) - A, C = q.D - (A + B);
        const float qa = A - 2.0f * B + C, qb = 2.0f * (B - A);
        float r[2];
        int nr = 0;
        if (std::fabs(qa) > 1e-12f) {
            const float disc = qb * qb - 4.0f * qa * A;
            if (disc >= 0.0f) {
                const float sq = std::sqrt(disc);
                r[nr++] = (-qb - sq) / (2.0f * qa);
                r[nr++] = (-qb + sq) / (2.0f * qa);
            }
        } else if (std::fabs(qb) > 1e-12f) {
            r[nr++] = -A / qb;
        }
        for (int i = 0; i < nr; ++i) {
            if (!(r[i] > 0.0f && r[i] < 1.0f)) continue;
            const float p = pStart + evalPiece(q, r[i]).p;
            pMin = std::fmin(pMin, p);
            pMax = std::fmax(pMax, p);
        }
    }
    const float out = std::fmax(0.0f, std::fmax(c.lo - pMin, pMax - c.hi));
    if (parts) *parts = Over{pv / c.lim.vmax, pa / c.lim.amax, pj / c.lim.jmax, 1.0f + out / (c.hi - c.lo)};
    float o = std::fmax(pv / c.lim.vmax, std::fmax(pa / c.lim.amax, pj / c.lim.jmax));
    // Under railStop the window is a wall: leaving it past the tolerance ranks
    // below any ceiling ratio short of a thousand, so the least-over fit stays inside.
    const float wall = c.railStop && out > kTol * (c.hi - c.lo) ? 1e3f : 1.0f;
    return std::fmax(o, wall + out / (c.hi - c.lo));
}

// Closed-form end accelerations of a piece (the algebra of evalPiece at u = 0, 1).
inline float aStartOf(const Piece& q) {
    return 2.0f / (3.0f * q.i0 * q.i0 * q.T * q.T) * (q.D - q.s0 * q.T * (1.0f - q.i1) - q.s1 * q.i1 * q.T) + q.da;
}
inline float aEndOf(const Piece& q) {
    return 2.0f / (3.0f * q.i1 * q.i1 * q.T * q.T) * (-q.D + q.s1 * q.T * (1.0f - q.i0) + q.s0 * q.i0 * q.T);
}

// The start length i0 whose start acceleration is a0 (a re-plan from the live
// state). False when the bracket's sign disagrees or i0 leaves [kLMin, kLMax]:
// the corner ramp absorbs the step then.
inline bool matchStartLength(Piece& q, float a0) {
    const float x = q.D - q.s0 * q.T * (1.0f - q.i1) - q.s1 * q.i1 * q.T;
    if (a0 == 0.0f || x * a0 <= 0.0f) return false;
    const float i0 = std::sqrt(2.0f * x / (3.0f * a0 * q.T * q.T));
    if (!(i0 >= kLMin && i0 <= kLMax)) return false;
    q.i0 = i0;
    return true;
}

// The end length i1 whose end acceleration is a1 (the start of a corner ramp).
// False as matchStartLength.
inline bool matchEndLength(Piece& q, float a1) {
    const float x = -q.D + q.s1 * q.T * (1.0f - q.i0) + q.s0 * q.i0 * q.T;
    if (a1 == 0.0f || x * a1 <= 0.0f) return false;
    const float i1 = std::sqrt(2.0f * x / (3.0f * a1 * q.T * q.T));
    if (!(i1 >= kLMin && i1 <= kLMax)) return false;
    q.i1 = i1;
    return true;
}

// Both end accelerations matched within tol by the two lengths (each moves the
// other end's acceleration, so they alternate). False leaves q unchanged.
inline bool matchEnds(Piece& q, float a0, float a1, float tol) {
    const Piece keep = q;
    for (int it = 0; it < 6; ++it) {
        const bool s = std::fabs(aStartOf(q) - a0) <= tol;
        const bool e = std::fabs(aEndOf(q) - a1) <= tol;
        if (s && e) return true;
        if ((!s && !matchStartLength(q, a0)) || (!e && !matchEndLength(q, a1))) break;
    }
    if (std::fabs(aStartOf(q) - a0) <= tol && std::fabs(aEndOf(q) - a1) <= tol) return true;
    q = keep;
    return false;
}

inline float clampL(float l) { return std::fmin(kLMax, std::fmax(kLMin, l)); }
inline float clampV(float v, const Cfg& c) { return std::fmax(-c.lim.vmax, std::fmin(c.lim.vmax, v)); }
inline float capV(const HKnot& m, float v) { return std::fabs(v) <= m.vcap ? v : (v > 0.0f ? m.vcap : -m.vcap); }

// The fastest speed whose legal brake (u^2 / 2A + u A / 2J) stops within the
// nearer wall's gap from p, times share; the sign of v kept.
inline float railBound(float v, float p, const Cfg& c, float share) {
    const float gap = std::fmin(c.hi - p, p - c.lo);
    if (v == 0.0f) return v;
    if (gap <= 0.0f) return 0.0f;
    const float A = c.lim.amax, J = c.lim.jmax, b = A * A / (2.0f * J);
    const float u = share * (-b + std::sqrt(b * b + 2.0f * gap * A));
    return std::fabs(v) <= u ? v : (v > 0.0f ? u : -u);
}

// ---- classes and angles -----------------------------------------------------------
// End: first or last knot (a rest end until its successor arrives). Rest: a
// chord within holdEps on a side (a hold edge). Crest: the chords change sign.
// Through: the same sign both sides. withDp classifies the trimmed chords.
// A chord within holdEps is a hold, except the one leaving a moving first
// knot (a re-plan's origin in flight; the model's first knot is at rest).
// Under railStop a hold is also slower than holdEps per kHoldSpan: a 60 Hz
// stream's chords are shorter than holdEps while it moves.
inline constexpr float kHoldSpan = 0.1f;   // s; the model's scripts have no shorter span
inline bool holdChord(const HKnot* k, int from, float d, const Cfg& c) {
    const bool slow = !c.railStop || std::fabs(d) * kHoldSpan <= c.holdEps * (k[from + 1].t - k[from].t);
    return std::fabs(d) <= c.holdEps && slow && !(from == 0 && k[0].has_v && k[0].v != 0.0f);
}

inline void classify(HKnot* k, int n, const Cfg& c, bool withDp) {
    auto pe = [&](int i) { return withDp ? rendered(k[i]) : k[i].p; };
    for (int i = 0; i < n; ++i) {
        const bool hasPrev = i > 0, hasNext = i + 1 < n;
        k[i].dIn  = hasPrev ? (pe(i) - pe(i - 1)) / (k[i].t - k[i - 1].t) : 0.0f;
        k[i].dOut = hasNext ? (pe(i + 1) - pe(i)) / (k[i + 1].t - k[i].t) : 0.0f;
        const bool zIn  = !hasPrev || holdChord(k, i - 1, pe(i) - pe(i - 1), c);
        const bool zOut = !hasNext || holdChord(k, i, pe(i + 1) - pe(i), c);
        k[i].cls = (!hasPrev || !hasNext) ? KnotClass::End
                 : (zIn || zOut)          ? KnotClass::Rest
                 : (k[i].dIn * k[i].dOut < 0.0f) ? KnotClass::Crest
                                                  : KnotClass::Through;
    }
}

// PCHIP's angle (Fritsch-Butland weighted harmonic mean), zero at a sign change.
inline float pchipAngle(const HKnot* k, int i) {
    const HKnot& m = k[i];
    if (m.dIn * m.dOut <= 0.0f) return 0.0f;
    const float h1 = m.t - k[i - 1].t, h2 = k[i + 1].t - m.t;
    const float w1 = 2.0f * h2 + h1, w2 = h2 + 2.0f * h1;
    return (w1 + w2) / (w1 / m.dIn + w2 / m.dOut);
}

// Makima's angle (Akima with the modified weights), chords padded at the ends.
// Requires n > 3.
inline float makimaAngle(const HKnot* k, int n, int i) {
    auto d = [&](int j) {
        if (j < 0) return 2.0f * k[0].dOut - k[1].dOut;
        if (j >= n - 1) return 2.0f * k[n - 2].dOut - k[n - 3].dOut;
        return k[j].dOut;
    };
    const float dm2 = d(i - 2), dm1 = d(i - 1), d0 = d(i), d1 = d(i + 1);
    const float w1 = std::fabs(d1 - d0) + std::fabs(d1 + d0) * 0.5f;
    const float w2 = std::fabs(dm1 - dm2) + std::fabs(dm1 + dm2) * 0.5f;
    return (w1 + w2 == 0.0f) ? (dm1 + d0) * 0.5f : (w1 * dm1 + w2 * d0) / (w1 + w2);
}

// The Fritsch-Carlson monotone band for a free angle at a through point.
inline void band(const HKnot& m, float& lo, float& hi) {
    const float b = 3.0f * std::fmin(std::fabs(m.dIn), std::fabs(m.dOut));
    if (m.dIn < 0.0f) { lo = -b; hi = 0.0f; } else { lo = 0.0f; hi = b; }
}

// ---- the solve: angles and lengths before the ceilings --------------------------
// An authored angle (has_v) is kept; every angle is held to +-vmax before any
// length is tried. G2 sweeps: a through point solves its angle in closed form
// (aEnd of the left piece and aStart of the right are both linear in it), a
// crest or hold edge under smooth its lengths (ratio matched, product a ninth).
inline void solveStyle(HKnot* k, int n, const Cfg& c, bool withDp, bool smooth) {
    classify(k, n, c, withDp);
    for (int i = 0; i < n; ++i) {
        HKnot& m = k[i];
        m.g2 = m.cls != KnotClass::End && (m.cls == KnotClass::Through || smooth);
        float base = 0.0f;
        if (m.cls == KnotClass::Crest || m.cls == KnotClass::Through)
            base = (smooth && n > 3) ? makimaAngle(k, n, i)
                 : (m.cls == KnotClass::Through) ? pchipAngle(k, i) : 0.0f;
        // The first knot's angle under railStop is the live velocity (a
        // re-plan's origin), kept exactly.
        m.vel = (i == 0 && c.railStop && m.has_v) ? m.v : capV(m, clampV(m.has_v ? m.v : base, c));
        if (i > 0 && c.railStop && m.has_v) m.vel = railBound(m.vel, withDp ? rendered(m) : m.p, c, m.rail);
        m.lIn = m.lOut = kThird;
    }
    auto pe = [&](const HKnot& m) { return withDp ? rendered(m) : m.p; };
    for (int s = 0; s < c.sweeps; ++s) {
        for (int i = 1; i + 1 < n; ++i) {
            HKnot& m = k[i];
            if (!m.g2) continue;
            const HKnot& L = k[i - 1];
            const HKnot& R = k[i + 1];
            const float TL = m.t - L.t, DL = pe(m) - pe(L), TR = R.t - m.t, DR = pe(R) - pe(m);
            if (m.cls == KnotClass::Through && !m.has_v) {
                const float cL = 2.0f / (3.0f * m.lIn * m.lIn * TL * TL);
                const float cR = 2.0f / (3.0f * m.lOut * m.lOut * TR * TR);
                const float AL = cL * (-DL + L.vel * L.lOut * TL), BL = cL * TL * (1.0f - L.lOut);
                const float AR = cR * (DR - R.vel * R.lIn * TR), BR = -cR * TR * (1.0f - R.lIn);
                float lo, hi;
                band(m, lo, hi);
                m.vel = capV(m, clampV(std::fmin(hi, std::fmax(lo, (AR - AL) / (BL - BR))), c));
            } else if (m.cls != KnotClass::Through) {
                const float XL = 2.0f / (3.0f * TL * TL) * (-DL + m.vel * TL * (1.0f - L.lOut) + L.vel * L.lOut * TL);
                const float XR = 2.0f / (3.0f * TR * TR) * (DR - m.vel * TR * (1.0f - R.lIn) - R.vel * R.lIn * TR);
                if (XL * XR > 0.0f) {
                    const float sr = std::sqrt(std::sqrt(XR / XL));
                    m.lIn = clampL(kThird / sr);
                    m.lOut = clampL(kThird * sr);
                }
            }
        }
    }
}

// The solve at Cfg::smoothness. An authored angle is the same in both solves
// and is kept bit for bit; g2 is the smooth solve's (read by its sweeps only).
inline void solve(HKnot* k, int n, const Cfg& c, bool withDp) {
    const float s = c.smoothness;
    if (!(s > 0.0f) || s >= 1.0f) {
        solveStyle(k, n, c, withDp, s >= 1.0f);
        return;
    }
    solveStyle(k, n, c, withDp, false);
    for (int i = 0; i < n; ++i) { k[i].pVel = k[i].vel; k[i].pIn = k[i].lIn; k[i].pOut = k[i].lOut; }
    solveStyle(k, n, c, withDp, true);
    const float r = 1.0f - s;
    for (int i = 0; i < n; ++i) {
        HKnot& m = k[i];
        if (!m.has_v) m.vel = r * m.pVel + s * m.vel;
        m.lIn = r * m.pIn + s * m.lIn;
        m.lOut = r * m.pOut + s * m.lOut;
    }
}

// ---- the ceilings: bound the lengths, then trim the amplitude -------------------
// The length factors nearest 1 first: 1, 0.95, 1.05, 0.90, ... 0.2, then 1.85..2.0
// (the model's KS order, the lower side first on a tie).
inline constexpr int kKs = 37;
inline float ksAt(int idx) {
    if (idx == 0) return 1.0f;
    if (idx <= 32) {
        const float d = 0.05f * static_cast<float>((idx + 1) / 2);
        return (idx & 1) ? 1.0f - d : 1.0f + d;
    }
    return 1.0f + 0.05f * static_cast<float>(idx - 16);
}

struct Fit {
    float k = 1.0f, i0 = kThird, i1 = kThird, o = 0.0f;
    bool  legal = false;
    float s1 = 0.0f;   // the later knot's angle the fit used (nudge)
};

// A trimmed knot's angle held to the Fritsch-Carlson band of its trimmed
// chord D over T: zero on a zero chord or against it (railStop only).
inline float bandHold(float v, float D, float T) {
    if (v * D <= 0.0f) return 0.0f;
    const float b = 3.0f * std::fabs(D) / T;
    return std::fabs(v) <= b ? v : (v > 0.0f ? b : -b);
}

// The fastest angle at L whose zero-stroke piece to R (R at L's height, at
// rest) is legal at some length factor: every peak of that piece is linear in
// the angle. The window gap is the one the angle heads toward.
// intoFlat: R is a flat knot, so the corner ramp to rest fits in half the span.
inline float zeroStrokeMax(const HKnot& L, const HKnot& R, float pL, float dir, const Cfg& c, bool intoFlat) {
    const float floor = std::fmax(kLMin, c.lfloor);
    const float gap = dir > 0.0f ? c.hi - pL : pL - c.lo;
    float best = 0.0f;
    for (int idx = 0; idx < kKs; ++idx) {
        const float f = ksAt(idx);
        Piece q;
        q.T = R.t - L.t;
        q.s0 = dir;
        q.i0 = std::fmin(kLMax, std::fmax(floor, L.lOut * f));
        q.i1 = std::fmin(kLMax, std::fmax(floor, R.lIn * f));
        float pv = 0.0f, pa = 0.0f, pj = 0.0f, px = 0.0f;
        for (int k = 0; k <= kJudgeN; ++k) {
            const Eval e = evalPiece(q, static_cast<float>(k) / kJudgeN);
            pv = std::fmax(pv, std::fabs(e.v));
            pa = std::fmax(pa, std::fabs(e.a));
            pj = std::fmax(pj, std::fabs(e.j));
            px = std::fmax(px, dir * e.p);
        }
        float s = std::fmin(c.lim.vmax * R.slack[0] / pv, std::fmin(c.lim.amax * R.slack[1] / pa, c.lim.jmax * R.slack[2] / pj));
        if (px > 0.0f) s = std::fmin(s, std::fmax(0.0f, gap) / px);
        if (intoFlat) s = std::fmin(s, c.lim.jmax * R.slack[2] * std::fmax(0.0f, 0.5f * q.T - kTick) / std::fabs(aEndOf(q)));
        best = std::fmax(best, s);
    }
    return best;
}

// Scale both handles of the piece L->R (from pL to pR) by one factor, nearest 1
// that is legal, never under the feel floor; else the least-over factor.
// bound: the caller needs the least-over fit only below it (a legality test
// passes 1 + kTol); a fit at or over it returns some ratio over it. Each
// factor is judged no further than the best so far: the result is the same.
// The kernel's room for the corner ramps at a piece's ends (jmax, one tick
// clear of each end): from aIn at its start (on the start when fromFlat, else
// centered, so also within half of tIn), and to rest at its end when intoFlat.
struct Room {
    float aIn = NAN;          // the acceleration the piece before ends in; NAN: none
    float tIn = INFINITY;     // that piece's span
    bool  fromFlat = false;
    bool  intoFlat = false;
    float aEnd = NAN;         // the end acceleration the next piece's start ramp asks; NAN: free
    float aEndTol = INFINITY; // how far from it the end may be
};

// The start ramp's share of its room (> 1: it does not fit).
inline float startRoom(const Piece& q, const Room& rm, float jmax) {
    if (std::isnan(rm.aIn)) return 0.0f;
    const float tr = std::fabs(aStartOf(q) - rm.aIn) / jmax;
    if (!(tr > 0.0f)) return 0.0f;
    return rm.fromFlat ? (tr + kTick) / (0.5f * q.T) : (0.5f * tr + kTick) / (0.5f * std::fmin(q.T, rm.tIn));
}

inline float roomOver(const Piece& q, const Room& rm, float jmax) {
    float o = startRoom(q, rm, jmax);
    if (rm.intoFlat) {
        const float tr = std::fabs(aEndOf(q)) / jmax;
        if (tr > 0.0f) o = std::fmax(o, (tr + kTick) / (0.5f * q.T));
    }
    if (!std::isnan(rm.aEnd)) o = std::fmax(o, std::fabs(aEndOf(q) - rm.aEnd) / rm.aEndTol);
    return o;
}

// At smoothness 0 a band-legal piece (both angles zero or of its chord's sign
// and at most 3 (1 + kTol) times it) stays monotone at its scaled lengths: 1 +
// the share of its chord it travels backward (an overshoot past its end or a
// reversal inside); 0 when not judged (smoothness above 0, a hold, an angle
// outside the band: an authored angle's overshoot is the author's). Exact:
// P'(u) / 3D is A (1-u)^2 + 2 B u (1-u) + C u^2, which changes sign only when
// B^2 > A C, B < 0.
inline float monoOver(const Piece& q, const Cfg& c) {
    if (c.smoothness > 0.0f || !(std::fabs(q.D) > c.holdEps)) return 0.0f;
    const float m = q.D / q.T, al = q.s0 / m, be = q.s1 / m, b = 3.0f * (1.0f + kTol);
    if (!(al >= 0.0f && al <= b && be >= 0.0f && be <= b)) return 0.0f;
    const float A = al * q.i0, C = be * q.i1, B = 1.0f - A - C;
    if (B >= 0.0f || B * B <= A * C) return 1.0f;
    // The turning points (P' / 3D = A - 2 (A - B) u + (A - 2 B + C) u^2), both in [0, 1].
    const float sq = std::sqrt(B * B - A * C), den = A - 2.0f * B + C;
    const float r1 = (A - B - sq) / den, r2 = (A - B + sq) / den;
    // P / D, the control points 0, A, 1 - C, 1.
    auto P = [&](float u) { const float w = 1.0f - u; return 3.0f * u * w * (w * A + u * (1.0f - C)) + u * u * u; };
    return 1.0f + std::fmax(0.0f, P(r1) - P(r2));
}

inline Fit fitPiece(const HKnot& L, const HKnot& R, float pL, float pR, const Cfg& c, float bound = INFINITY,
                    const Room* room = nullptr, int* hint = nullptr) {
    const float floor = std::fmax(kLMin, c.lfloor);
    Cfg cs = c;
    cs.lim.vmax *= R.slack[0];
    cs.lim.amax *= R.slack[1];
    cs.lim.jmax *= R.slack[2];
    Fit best;
    best.o = INFINITY;
    // Which side of 1 can help, read at k = 1: a speed or window excess wants
    // shorter handles, an acceleration or jerk excess longer ones; the room
    // (a corner ramp's fit) and both at once keep the model's full walk. The
    // walk's order is the model's, so the first legal factor is the same.
    auto judge = [&](float f, float bound_, Over* parts) {
        Piece q;
        q.T = R.t - L.t;
        q.D = pR - pL;
        q.s0 = L.vel;
        q.s1 = R.vel;
        q.i0 = std::fmin(kLMax, std::fmax(floor, L.lOut * f));
        q.i1 = std::fmin(kLMax, std::fmax(floor, R.lIn * f));
        const float ro = std::fmax(room ? roomOver(q, *room, cs.lim.jmax) : 0.0f, monoOver(q, cs));
        const float o = std::fmax(ro, ro > 1.0f + kTol && ro >= bound_ && !parts ? ro
                                      : overOf(q, pL, cs, parts ? INFINITY : bound_, parts));
        return Fit{f, q.i0, q.i1, o, o <= 1.0f + kTol};
    };
    // k = 1 first, with its parts: a speed or window excess wants shorter
    // handles, an acceleration or jerk excess longer ones, so the other side
    // of the grid is skipped; both at once, or a room or reversal that is
    // over, keep the model's full walk. The order is the model's (nearest 1
    // first), so the first legal factor is the same.
    Over parts;
    // With a hint the side is known from the last fit: k = 1 is judged with
    // the early return (its parts are not needed) and the walk stays on that
    // side.
    const bool hinted = hint && *hint > 0 && *hint < kKs;
    Fit f1 = judge(1.0f, hinted ? 1.0f + kTol : INFINITY, hinted ? nullptr : &parts);
    if (f1.legal) return f1;
    if (f1.o < best.o) best = f1;
    bool overV, overAJ, roomOver_;
    // Constraint: the side rule holds for lengths of a third (every pchip
    // solve). A smooth solve's lengths can sum past 1, where longer handles
    // raise the jerk: those walk both sides, as the model does (kin-rfw7).
    if (L.lOut != kThird || R.lIn != kThird) {
        overV = overAJ = roomOver_ = true;
    } else if (hinted) {
        overV = ksAt(*hint) < 1.0f; overAJ = !overV; roomOver_ = false;
    } else {
        overV = parts.v > 1.0f + kTol || parts.x > 1.0f + kTol;
        overAJ = parts.a > 1.0f + kTol || parts.j > 1.0f + kTol;
        roomOver_ = f1.o > std::fmax(std::fmax(parts.v, parts.a), std::fmax(parts.j, parts.x)) + kTol;
    }
    const bool tryDown = !(overAJ && !overV) || roomOver_, tryUp = !(overV && !overAJ) || roomOver_;
    // A legality test (bound at the tolerance) with no factor able to recover
    // the excess at k = 1 skips the walk: a factor of at most 2 cuts the jerk
    // by at most 2^3 on a third-length handle (the floor and the cap bound the
    // lengths tighter), the acceleration by 2^2; the model walks and finds the
    // same, every factor over. The least-over search (a wider bound) walks.
    if (!hinted && bound <= 1.0f + kTol && !roomOver_ && overAJ && !overV && (parts.j > 9.0f || parts.a > 5.0f)) return best;
    auto onSide = [&](int idx) { const float f = ksAt(idx); return (f < 1.0f && tryDown) || (f > 1.0f && tryUp); };
    auto step = [&](int idx, int dir) { do { idx += dir; } while (idx > 0 && idx < kKs && !onSide(idx)); return idx; };
    auto at = [&](int idx) {
        const Fit fs = judge(ksAt(idx), std::fmax(1.0f + kTol, std::fmin(bound, best.o)), nullptr);
        if (fs.o < best.o) best = fs;
        return fs;
    };
    // A trim's bisection fits the same piece at nearby moves, so the legal
    // factor moves little between steps: start at the last one (hint), walk
    // toward 1 while legal, and the last legal factor is the model's first
    // legal in its order whenever the side's legal factors are one run.
    if (hint && *hint > 0 && *hint < kKs && onSide(*hint) && tryDown != tryUp) {
        int h = *hint;
        Fit fh = at(h);
        if (!fh.legal) {
            // The run moved toward 1, or away: look toward 1 first, then away.
            int j = step(h, -1);
            while (j > 0 && !(fh = at(j)).legal) j = step(j, -1);
            if (j <= 0) {
                j = step(h, +1);
                while (j < kKs && !(fh = at(j)).legal) j = step(j, +1);
                if (j >= kKs) return best;
                *hint = j;
                return fh;
            }
            h = j;
        }
        for (int j = step(h, -1); j > 0; j = step(j, -1)) {
            const Fit fj = at(j);
            if (!fj.legal) break;
            fh = fj; h = j;
        }
        *hint = h;
        return fh;
    }
    for (int idx = 1; idx < kKs; ++idx) {
        if (!onSide(idx)) continue;
        const Fit fs = at(idx);
        if (fs.legal) { if (hint) *hint = idx; return fs; }
    }
    return best;
}

// Per piece, fit the lengths; when no factor is legal, move the later knot
// toward the previous knot's rendered position by the least that is legal
// (bisection, 16 steps); if even the whole move is illegal, take the least-over
// of 25/50/75/100% and mark it infeasible. Time never moves. A hold after a
// trimmed knot moves with it; the last knot never trims. Returns the count of
// infeasible pieces.
inline int nudge(HKnot* k, int n, const Cfg& c, bool* capped = nullptr) {
    for (int i = 0; i < n; ++i) {
        k[i].effIn = k[i].lIn;
        k[i].effOut = k[i].lOut;
        k[i].dp = 0.0f;
        k[i].infeasible = false;
    }
    int bad = 0;
    for (int i = 0; i + 1 < n; ++i) {
        HKnot& L = k[i];
        HKnot& R = k[i + 1];
        const float pL = rendered(L);
        // Under railStop (the kernel) the corner ramps at the piece's ends
        // must fit (Room); the origin's start is the engine's lead ramp.
        Room rm;
        bool intoFlat = false;
        if (c.railStop) {
            if (i > 0) {
                const HKnot& K = k[i - 1];
                rm.aIn = L.aIn;
                rm.tIn = L.t - K.t;
                rm.fromFlat = L.vel == 0.0f && K.vel == 0.0f && std::fabs(pL - rendered(K)) <= c.holdEps;
            } else if (L.vel == 0.0f && L.aIn == 0.0f) {
                // An origin at rest leaves on a lead ramp (engine_piece.hpp).
                rm.aIn = 0.0f;
                rm.fromFlat = true;
            }
            intoFlat = i + 2 >= n || holdChord(k, i + 1, k[i + 2].p - R.p, c);
            rm.aEnd = R.aTarget;
            rm.aEndTol = R.aTol;
        }
        const float key[kMemoKey] = {L.p, L.dp, R.p, L.vel, R.vel, L.lOut, R.lIn, R.slack[0], R.slack[1], R.slack[2], pL,
                               rm.aIn, float(rm.fromFlat) + 2.0f * float(intoFlat), rm.aEnd, rm.aEndTol, L.vcap};
        bool hit = R.memo;
        for (int q = 0; hit && q < kMemoKey; ++q) hit = key[q] == R.mk[q] || (std::isnan(key[q]) && std::isnan(R.mk[q]));
        if (hit) {
            L.effOut = R.mi0; R.effIn = R.mi1; R.dp = R.mdp; R.vel = R.mvel; R.aIn = R.maIn;
            R.infeasible = !R.mlegal;
            if (!R.mlegal) ++bad;
            continue;
        }
        // Under railStop a trimmed knot's angle keeps to its trimmed chord, so
        // the whole trim is a zero stroke at rest.
        int hint = 0;   // the trim bisection's last legal factor (fitPiece)
        auto fitAt = [&](float pR, float bound, bool hinted = false) {
            HKnot Rt = R;
            if (c.railStop && pR != R.p) Rt.vel = bandHold(R.vel, pR - pL, R.t - L.t);
            Room r2 = rm;
            r2.intoFlat = intoFlat && Rt.vel == 0.0f;
            Fit f = fitPiece(L, Rt, pL, pR, c, bound, c.railStop ? &r2 : nullptr, hinted ? &hint : nullptr);
            f.s1 = Rt.vel;
            return f;
        };
        const bool hold = holdChord(k, i, R.p - L.p, c);
        const bool canTrim = c.trim > 0.0f && (i + 2 < n || c.trimLast);
        float dp = hold ? L.dp : 0.0f;
        Fit f = fitAt(R.p + dp, INFINITY);
        if (!f.legal && !hold && canTrim) {
            const float dir = (pL > R.p) ? 1.0f : (pL < R.p) ? -1.0f : 0.0f;
            float lo = 0.0f, hi = std::fmin(c.trim, std::fabs(R.p - pL));
            Fit fh = fitAt(R.p + dir * hi, 1.0f + kTol, true);
            if (fh.legal) {
                for (int s = 0; s < 16; ++s) {
                    const float mid = 0.5f * (lo + hi);
                    const Fit fm = fitAt(R.p + dir * mid, 1.0f + kTol, true);
                    if (fm.legal) { hi = mid; fh = fm; } else { lo = mid; }
                }
                f = fh;
                dp = dir * hi;
            } else {
                // Constraint: reachable (kin-ay9). The full move is a zero stroke,
                // illegal while L's angle heads into the span faster than the span
                // turns it (its cap lands a round later, at most four), and a
                // partial move can be legal then: without this the property suite's
                // PieceOverCeiling rose from 48 to 472 (seed 6: speed,
                // acceleration and the window over).
                for (int q = 1; q <= 4; ++q) {
                    const float d = dir * 0.25f * static_cast<float>(q) * hi;
                    const Fit fq = fitAt(R.p + d, f.o);
                    if (fq.o < f.o) { f = fq; dp = d; }
                }
            }
        }
        // An illegal hold (a chord within holdEps rendered too fast) lies flat
        // at its predecessor instead when that is less over. The model has no
        // such piece; its hold chords are long enough to be legal.
        if (!f.legal && hold && canTrim) {
            const Fit ff = fitAt(pL, f.o);
            if (ff.o < f.o) { f = ff; dp = pL - R.p; }
        }
        // Still over (railStop): L's angle is faster than its span can stop.
        // Its cap holds it from the next solve on; the origin's live angle is
        // never capped.
        if (!f.legal && c.railStop && i > 0 && L.vel != 0.0f) {
            // A cap under a thousandth of vmax is zero: the knot is then flat.
            float cap = 0.98f * zeroStrokeMax(L, R, pL, L.vel > 0.0f ? 1.0f : -1.0f, c, intoFlat);
            if (cap < 1e-3f * c.lim.vmax) cap = 0.0f;
            if (cap < std::fabs(L.vel) && cap < L.vcap) {
                L.vcap = cap;
                if (capped) *capped = true;
            }
        }
        L.effOut = f.i0;
        R.effIn = f.i1;
        R.dp = dp;
        R.vel = f.s1;
        {
            Piece q;
            q.T = R.t - L.t; q.D = rendered(R) - pL; q.s0 = L.vel; q.s1 = R.vel; q.i0 = f.i0; q.i1 = f.i1;
            R.aIn = aEndOf(q);
            // Over because the start ramp at L does not fit: the piece into L
            // is asked to end near this piece's start (from the next round).
            if (!f.legal && c.railStop && startRoom(q, rm, c.lim.jmax * R.slack[2]) > 1.0f + kTol) {
                const float span = rm.fromFlat ? 0.5f * q.T - kTick : std::fmin(q.T, rm.tIn) - 2.0f * kTick;
                const float tol = 0.95f * c.lim.jmax * L.slack[2] * span;
                const float target = aStartOf(q);
                if (tol > 0.0f && !(L.aTarget == target && L.aTol == tol)) {
                    L.aTarget = target;
                    L.aTol = tol;
                    if (capped) *capped = true;
                }
            }
        }
        R.infeasible = !f.legal;
        if (!f.legal) ++bad;
        R.memo = true;
        for (int q = 0; q < kMemoKey; ++q) R.mk[q] = key[q];
        R.mi0 = f.i0; R.mi1 = f.i1; R.mdp = dp; R.mvel = f.s1; R.maIn = R.aIn; R.mlegal = f.legal;
    }
    return bad;
}

// Solve, nudge, then two rounds of (solve on the trimmed chords, nudge).
// Returns the count of infeasible pieces (each an anomaly for the caller).
inline int render(HKnot* k, int n, const Cfg& c) {
    if (n < 2) return 0;
    solve(k, n, c, false);
    int bad = nudge(k, n, c);
    // A last round that caps an angle earns one more (railStop only; at most four).
    for (int round = 0, extra = 0; round < 2 + extra; ++round) {
        bool capped = false;
        solve(k, n, c, true);
        bad = nudge(k, n, c, &capped);
        if (capped && round + 1 == 2 + extra && extra < 4) ++extra;
    }
    return bad;
}

// The rendered piece from knot i to knot i + 1, relative to rendered(k[i]) at k[i].t.
inline Piece pieceOf(const HKnot* k, int i) {
    Piece q;
    q.T = k[i + 1].t - k[i].t;
    q.D = rendered(k[i + 1]) - rendered(k[i]);
    q.s0 = k[i].vel;
    q.s1 = k[i + 1].vel;
    q.i0 = k[i].effOut;
    q.i1 = k[i + 1].effIn;
    return q;
}

}  // namespace kinetic2::handles

// Renders the model's SAMPLE with the model's DEF ceilings (mm, window 0..100)
// and prints every knot's solve and trim. Build:
//   g++ -std=c++20 -DKINETIC2_HANDLES_SELFTEST -x c++ include/kinetic2/handles.hpp -o handles_selftest
#ifdef KINETIC2_HANDLES_SELFTEST
#include <cstdio>
int main() {
    namespace h = kinetic2::handles;
    const int at[]  = {0, 500, 700, 900, 1100, 1300, 1800, 1950, 2200, 2600, 3100, 3300, 3400, 3600, 3800, 4300,
                       4400, 4700, 4800, 5100, 5200, 5700, 6200};
    const int pos[] = {10, 10, 90, 10, 90, 10, 10, 60, 100, 0, 0, 70, 50, 100, 20, 20, 80, 30, 90, 40, 95, 10, 10};
    const int n = static_cast<int>(sizeof(at) / sizeof(at[0]));
    h::HKnot k[n];
    for (int i = 0; i < n; ++i) {
        k[i].t = static_cast<float>(at[i]) / 1000.0f;
        k[i].p = static_cast<float>(pos[i]);
    }
    h::Cfg c;
    c.lim.vmax = 1000.0f;
    c.lim.amax = 50000.0f;
    c.lim.jmax = 5e6f;
    c.lo = 0.0f;
    c.hi = 100.0f;
    c.holdEps = 0.5f;
    c.trim = 100.0f;
    const int bad = h::render(k, n, c);
    const char* cls[] = {"end", "rest", "crest", "through"};
    std::printf("i t p dp cls vel effIn effOut infeasible\n");
    for (int i = 0; i < n; ++i)
        std::printf("%d %.3f %.1f %.4f %s %.3f %.4f %.4f %d\n", i, k[i].t, k[i].p, k[i].dp,
                    cls[static_cast<int>(k[i].cls)], k[i].vel, k[i].effIn, k[i].effOut, k[i].infeasible ? 1 : 0);
    std::printf("infeasible %d\n", bad);
    return 0;
}
#endif
