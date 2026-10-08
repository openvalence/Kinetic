// kinetic2/solver.hpp -- the lookahead solver: the handle renderer
// (handles.hpp) over every pending knot, a corner ramp where two pieces meet
// in different accelerations, and the HARD point move
// Constraints:
// - Time never gives: every knot is reached at its own time, or one tick
//   after the knot before it when it was authored closer (kMinSpanUs, the
//   floor of every span). A knot the ceilings cannot reach moves toward its
//   predecessor in position only (handles::nudge), reported KnotTrimmed
//   with the share of its chord kept. Angles are capped at what their spans
//   stop and a trimmed knot's angle at its trimmed chord (kin-88m), so the
//   whole trim is legal for speed, acceleration and the window; a G1 knot
//   whose corner ramp has no room keeps a jerk step, rendered and reported
//   PieceOverCeiling with its worst ratio (operator ruling owed, kin-y6e).
//   Only a HARD knot (a live jog) may land after its time, at the end of
//   Profile::point.
// - Every built piece is judged as the 1 ms grid reads it (the piece, its
//   lead and start correction, the steps beside it, the last knot's brake);
//   nothing over a ceiling is silent. A built piece over a ceiling tightens
//   that ceiling alone for the next pass, from what the render reads of it.
// - A window is rendered whole from the origin: the origin is the render's
//   first knot, its angle the live velocity (never clamped), so a re-plan
//   from mid-flight keeps the velocity exactly and the piece build carries
//   the acceleration (engine_piece.hpp).
// - The renderer's knobs are Config's (smoothness, handle_floor, trim_max);
//   solve_budget is not read yet: every run renders its whole window, and
//   the bound owed is a resumable slice, bit-identical (Valence RFC-108
//   item 8, kin-tnv), never a cut window.
// - A knot's solved state (where the next piece, a starvation brake and a
//   re-plan start) is inside vmax and amax, whatever the piece into it did
//   (emitRun, kin-554).
// - A knot whose two pieces differ in acceleration by more than jmax * kStepS
//   carries a corner ramp at jmax: centered on the knot, ending on it into a
//   flat span, starting on it out of one. Its Solved state is then the ramp's
//   end, where the next piece starts.
// - Float throughout (the P4 has no double FPU).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "engine_piece.hpp"
#include "handles.hpp"
#include "profile.hpp"
#include "types.hpp"

namespace kinetic2 {

// ---- cost counters (tests/bench_kinetic2.cpp) ----------------------------------
// Compiled in only under KINETIC2_STATS; never in firmware. Not thread-safe.
#ifdef KINETIC2_STATS
namespace stats {
struct Counters {
    uint64_t windows = 0;      // solveWindow calls
    uint64_t knots = 0;        // knots rendered
    uint64_t judges = 0;       // renders (handles::render calls)
    uint64_t capped = 0;       // runs kSlackCap stopped
};
inline Counters g{};
}  // namespace stats
#define K2_STAT(field, n) (::kinetic2::stats::g.field += (n))
#else
#define K2_STAT(field, n) ((void)0)
#endif

inline constexpr float    kHoldEps   = 0.005f;   // a chord this small is a hold, window units
inline constexpr uint64_t kMinSpanUs = 1000;     // one tick, the floor of every span
inline constexpr float    kIllegal   = 1e30f;    // a ratio for an illegal profile
inline constexpr float    kKnotTol   = 1e-3f;    // a corner ramp passes its knot this near, window units
inline constexpr int      kWalkBack  = 6;        // rounds a corner ramp's walk-back may take to settle

// ---- the solved knot ---------------------------------------------------------
// Where and when the curve passes the knot, with what velocity and acceleration.
struct Solved {
    uint64_t t_us = 0;
    float    p = 0.0f, v = 0.0f, a = 0.0f;
    float    share = 1.0f;       // the share of the chord kept by a trim
    float    stretched_s = 0.0f; // a HARD knot's seconds past its time; else 0
    float    worst = 0.0f;       // the incoming piece's worst ceiling ratio
    bool     clamped = false;    // an authored velocity was held to vmax (reported once)
    uint64_t base_us = 0;        // the authored time
    bool     pin_v = false, pin_a = false;   // always false; ABI fields
    bool     dropped = false;    // always false: a knot is never dropped; ABI field
    bool     infeasible = false; // the incoming piece is over a ceiling after the whole trim
    float    i0 = handles::kThird, i1 = handles::kThird;   // the incoming piece's handle lengths
    float    a_in = 0.0f;        // the end acceleration the incoming piece is matched to
    float    knot_p = 0.0f;      // where the curve passes the knot at its time: authored, or trimmed
    bool     missed = false;     // the corner ramp passes the knot off its render (knot_p is where)
    // The piece into this knot starts on the corner ramp's exit with no
    // acceleration step (Piece::bezier exact): a step under kStepS beside a
    // ramp at jmax reads over jmax on the 1 ms grid (kin-rfw7).
    bool     exact_start = false;
    // The piece into this knot keeps its handle lengths (Piece::bezier keep):
    // matched to its end accelerations it left a ceiling or the window.
    bool     keep_lengths = false;
    // HARD junction: `ramp` is the whole move, Profile::point from the state
    // before the knot, landing at rest on it at t_us. CORNER: the piece ends at
    // head (head_us) and `ramp` carries the jerk-limited step; t_us / p / v / a
    // are then the ramp's END, where the next piece starts.
    bool     hard = false;
    bool     corner = false;
    uint64_t head_us = 0;
    State    head{};
    Profile  ramp{};
};

inline constexpr size_t kWindowKnots = 64;   // a longer window renders in runs; the knot at a cut is an end

// The renderer's scratch, owned by the caller (one per axis). Never a static:
// two engines on two tasks would share it. Never thread_local: ESP-IDF carves
// it out of every task's stack (kin-6tz). Never on the motion task's stack.
struct Workspace {
    handles::HKnot k[kWindowKnots + 1];
    uint64_t       t_us[kWindowKnots + 1];   // each knot's time, one tick past the one before
    float          built[kWindowKnots + 1];  // each built piece's worst ratio on the 1 ms grid
    // Per knot and ceiling (v, a, j): the ratio that asked its last
    // tightening (the passes before this one) and the worst one this pass.
    float          last[kWindowKnots + 1][3];
    float          ask[kWindowKnots + 1][3];
};

// ---- HARD --------------------------------------------------------------------
// The fastest move from s to rest on the knot (Profile::point); the knot's
// time is its own, or the profile's end when that is later (a live jog,
// unreported). Always that profile, even over a ceiling or past the window:
// from a state its ceilings cannot stop inside (a jog handed motion planned
// under a faster set) it is still the fastest stop they allow, flagged
// infeasible for the report. Never rendered at its authored time instead: one
// tick out, that render ended thousands of times over amax and the starvation
// brake from there ran away (kin-v9z, Nucleus val-hlj). False only for a full
// profile, which point() never builds (it needs at most eleven phases).
inline bool hardKnot(const State& s, uint64_t s_us, const Knot& K, const handles::Cfg& c, Solved& o) {
    const uint64_t avail = K.t_us > s_us ? K.t_us - s_us : 0;
    float fastest = 0.0f;
    const Profile pr = Profile::point(s, K.p, s_us, c.lim, float(avail) * 1e-6f, &fastest);
    if (pr.n < 0) return false;
    const float within = pr.worstRatio(c.lim, -1e30f, 1e30f);
    const float ratio = pr.worstRatio(c.lim, c.lo, c.hi) >= 1e29f ? std::fmax(within, 1.0f + 2.0f * handles::kTol) : within;
    const State end = pr.n > 0 ? pr.atSeconds(pr.duration()) : State{s.p, 0.0f, 0.0f};
    o = Solved{};
    o.base_us = K.t_us;
    o.t_us = pr.end_us() > K.t_us ? pr.end_us() : K.t_us;
    o.stretched_s = uint64_t(fastest * 1e6f) > avail + 1 ? float(o.t_us - K.t_us) * 1e-6f : 0.0f;
    // Where the profile lands: the knot, or its own end when float bisection
    // misses it, so the boundary never steps.
    o.p = std::fabs(end.p - K.p) <= 1e-5f ? K.p : end.p;
    o.knot_p = K.p; o.hard = true; o.ramp = pr; o.worst = ratio;
    o.infeasible = !(ratio <= 1.0001f);
    return true;
}

// ---- the piece the engine renders --------------------------------------------
// From state s at s_us into solved knot k. A HARD knot is its whole profile,
// which starts where the knot was solved from: an origin moved along it (the
// reaction horizon) is on the same curve.
inline Piece buildPiece(const State& s, uint64_t s_us, const Solved& k, const Limits& L) {
    Piece q;
    if (k.hard) {
        q = Piece::profile(k.ramp);
        q.end_us = k.t_us;
    } else if (k.corner) {
        q = Piece::bezier(s_us, s, k.head_us, State{k.head.p, k.head.v, k.a_in}, k.i0, k.i1, L, k.exact_start, k.keep_lengths);
        q.has_tail = true;
        q.tail = k.ramp;
        q.end_us = k.t_us;
    } else {
        q = Piece::bezier(s_us, s, k.t_us, State{k.p, k.v, k.a_in}, k.i0, k.i1, L, k.exact_start, k.keep_lengths);
    }
    return q;
}

// The rendered piece q from u = ua on (de Casteljau): the same curve, so a
// piece built from a corner's exit stays on the render (its handle lengths
// as shares of the shorter span, its start angle the curve's there).
inline handles::Piece tailOf(const handles::Piece& q, float ua) {
    const float t1 = q.i0 * q.T, t2 = q.T - q.i1 * q.T, p1 = q.s0 * t1, p2 = q.D - q.s1 * q.i1 * q.T;
    auto lerp = [ua](float a, float b) { return a + (b - a) * ua; };
    const float tb = lerp(t1, t2), tc = lerp(t2, q.T), te = lerp(tb, tc), tf = lerp(lerp(0.0f, t1), tb);
    const float pb = lerp(p1, p2), pc = lerp(p2, q.D), pe = lerp(pb, pc), pf = lerp(lerp(0.0f, p1), pb);
    handles::Piece r = q;
    r.T = q.T - lerp(tf, te);
    r.D = q.D - lerp(pf, pe);
    if (!(r.T > 0.0f)) return q;
    const float t0 = q.T - r.T, p0 = q.D - r.D;
    r.i0 = (te - t0) / r.T;
    r.i1 = (q.T - tc) / r.T;
    r.s0 = te > t0 ? (pe - p0) / (te - t0) : q.s0;
    return r;
}

// ---- the corner ramp ---------------------------------------------------------
// The ramp at jmax that replaces the acceleration step at a G1 knot of the
// rendered curve (qL into the knot, qR out of it): from the curve's state h1
// before the knot to its acceleration h2 after, the split bisected until the
// ramp gains the curve's velocity, so the pieces on both sides stay on the
// rendered curve. False when no split lands it.
inline bool rampOnCurve(const handles::Piece& qL, const handles::Piece& qR, float jmax,
                        float& h1, float& h2, handles::Eval& eL, handles::Eval& eR) {
    auto at = [&](float f) {
        float Tr = std::fabs(handles::aStartOf(qR) - handles::aEndOf(qL)) / jmax;
        for (int it = 0; it < 4; ++it) {
            eL = handles::evalPiece(qL, handles::solveU(qL, qL.T - f * Tr));
            eR = handles::evalPiece(qR, handles::solveU(qR, (1.0f - f) * Tr));
            Tr = std::fabs(eR.a - eL.a) / jmax;
        }
        h1 = f * Tr;
        h2 = (1.0f - f) * Tr;
        eL = handles::evalPiece(qL, handles::solveU(qL, qL.T - h1));
        eR = handles::evalPiece(qR, handles::solveU(qR, h2));
        return eL.v + Tr * 0.5f * (eL.a + eR.a) - eR.v;
    };
    float lo = 0.0f, hi = 1.0f;
    const bool up = at(lo) > 0.0f;
    if ((at(hi) > 0.0f) == up) return false;
    for (int it = 0; it < 14; ++it) {
        const float mid = 0.5f * (lo + hi);
        if ((at(mid) > 0.0f) == up) lo = mid; else hi = mid;
    }
    at(0.5f * (lo + hi));
    return true;
}

// The jerk-limited step at knot o from aL, the acceleration the built piece
// into it ends in, to aR, the next piece's start. The piece then ends at the
// ramp's start, before the knot, which moves its end acceleration: rounds of
// the walk-back settle it, and the ramp starts from the head's own end
// state. With the rendered pieces (qL, qR) the ramp is rampOnCurve's;
// without them, or when that finds no split, it is centered on the knot (a
// ramp longer than planned ends past its plan while it fits). The ramp keeps
// to half of what remains of each span (from st_us, and tr_us after the
// knot); one that does not fit leaves the step.
// Settled: the ramp from the head's own end passes the knot within kKnotTol,
// or no further off than its plan (a flat knot's turn lands on its height).
// Once two heads end on either side of their plans the walk-back steps by
// false position inside that bracket. An unsettled on-curve ramp gives way to
// the centered one; an unsettled centered walk-back keeps the round whose head
// ends nearest its plan, with that planned ramp: it passes the knot, and the
// acceleration step left at the head is judged by renderRun (kin-1ir).
// from_flat: the ramp starts on the knot; into_flat: it ends on it (a hold
// stays a hold).
inline void cornerAt(Solved& o, float aL, float aR, bool from_flat, bool into_flat, const State& st,
                     uint64_t st_us, uint64_t tr_us, const Limits& L,
                     const handles::Piece* qL = nullptr, const handles::Piece* qR = nullptr) {
    if (std::fabs(aR - aL) <= L.jmax * kStepS) return;
    uint64_t r_us = 0, h_us = 0, use_us = 0;
    State w;
    float jr = 0.0f;
    const float a_in0 = o.a_in, aL0 = aL, aR0 = aR;
    float h1 = 0.0f, h2 = 0.0f;
    handles::Eval eL, eR;
    // Only a piece built near the rendered curve (its end acceleration within
    // a fifth of amax of the curve's) hands the ramp the curve's state; a
    // re-plan from the live state is not on it and takes the knot's ramp.
    bool on_curve = !into_flat && !from_flat && qL && qR
                    && std::fabs(aL - handles::aEndOf(*qL)) <= 0.2f * L.amax
                    && rampOnCurve(*qL, *qR, L.jmax, h1, h2, eL, eR);
    // The ramp on the curve matches its velocity only: one that misses the
    // knot's position by more than kKnotTol takes the knot's centered ramp,
    // which passes the knot exactly.
    if (on_curve) {
        const float jr = (eR.a - eL.a) / (h1 + h2);
        const float at_knot = o.p - qL->D + eL.p + eL.v * h1 + 0.5f * eL.a * h1 * h1 + jr * h1 * h1 * h1 / 6.0f;
        if (std::fabs(at_knot - o.p) > kKnotTol) on_curve = false;
    }
    for (;;) {
        aL = aL0;
        aR = aR0;
        float aP = aL, jP = 0.0f, aH = aL, best = INFINITY;
        float ap = 0.0f, fp = 0.0f, an = 0.0f, fn = 0.0f;   // the bracket: heads ending above / below their plan
        State bw;
        uint64_t br_us = 0, bh_us = 0;
        for (int it = 0; it < (on_curve ? 1 : kWalkBack); ++it) {
            if (on_curve) {
                h_us = uint64_t(h1 * 1e6f + 0.5f);
                r_us = h_us + uint64_t(h2 * 1e6f + 0.5f);
                aL = eL.a;
                aR = eR.a;
            } else {
                r_us = uint64_t(std::fabs(aR - aL) / L.jmax * 1e6f + 0.5f);
                h_us = into_flat ? r_us : from_flat ? 0 : r_us / 2;
            }
            if (r_us == 0 || 2 * (h_us + kMinSpanUs) > o.t_us - st_us || 2 * (r_us - h_us + kMinSpanUs) > tr_us) { o.a_in = a_in0; return; }
            const float Tr = float(r_us) * 1e-6f, h = float(h_us) * 1e-6f;
            const float j = (aR - aL) / Tr;
            w.a = aL;
            if (on_curve) {
                w.v = eL.v;
                w.p = o.p - qL->D + eL.p;
            } else {
                // Through the knot at its velocity (into_flat: ends at rest on it,
                // so a hold stays a hold).
                w.v = o.v - aL * h - 0.5f * j * h * h;
                w.p = o.p - w.v * h - 0.5f * aL * h * h - j * h * h * h / 6.0f;
            }
            // A flat knot where the motion turns is an extremum: the ramp's own
            // turn lands on its height, never past it (a crest on the rail
            // stays in the window). A flat knot the motion passes keeps its
            // position at its time.
            const float disc = aL * aL - 2.0f * j * w.v;
            if (!into_flat && o.v == 0.0f && disc >= 0.0f && (!qL || !qR || qL->D * qR->D <= 0.0f)) {
                const float r = std::sqrt(disc);
                for (const float t : {(-aL - r) / j, (-aL + r) / j})
                    if (t > 0.0f && t < Tr) { w.p += o.p - Profile::step(w, j, t).p; break; }
            }
            aP = aL;
            jP = j;
            aH = aL;
            if (h_us != 0) {
                o.a_in = w.a;
                const Piece head = Piece::bezier(st_us, st, o.t_us - h_us, w, o.i0, o.i1, L, false, o.keep_lengths);
                if (head.T > 0.0f) aH = handles::aEndOf(head.q);
            }
            const float res = aH - w.a;   // the head's end past the acceleration it was built to
            if (std::fabs(res) < best) { best = std::fabs(res); bw = w; br_us = r_us; bh_us = h_us; }
            if (std::fabs(res) <= L.jmax * kStepS) break;
            // The next round plans from where this head ends, or by false
            // position once a bracket exists: the plain step can oscillate.
            if (res > 0.0f) { ap = w.a; fp = res; } else { an = w.a; fn = res; }
            aL = fp > 0.0f && fn < 0.0f ? ap - fp * (an - ap) / (fn - fp) : aH;
        }
        aL = aH;
        w.a = aL;
        const uint64_t f_us = uint64_t(std::fabs(aR - aL) / L.jmax * 1e6f + 0.5f);
        use_us = f_us > (on_curve ? r_us : h_us) && 2 * (f_us - h_us + kMinSpanUs) <= tr_us ? f_us : r_us;
        jr = on_curve ? std::fmax(-L.jmax, std::fmin(L.jmax, (aR - aL) / (float(use_us) * 1e-6f)))
                      : (aR - aL) / (float(use_us) * 1e-6f);
        if (into_flat && aR == 0.0f && aL * w.v < 0.0f) {
            // Lands at rest from the head's own end: its velocity and acceleration
            // reach zero together (the planned w can differ from where the head
            // ends when its end acceleration did not match).
            const uint64_t l_us = uint64_t(-2.0f * w.v / aL * 1e6f + 0.5f);
            if (l_us >= h_us && l_us > 0 && std::fabs(aL) <= L.jmax * float(l_us) * 1e-6f * (1.0f + handles::kTol)
                && 2 * (l_us > h_us ? l_us - h_us + kMinSpanUs : 0) <= tr_us) {
                use_us = l_us;
                jr = -aL / (float(l_us) * 1e-6f);
            }
        }
        const float hk = float(h_us) * 1e-6f;
        State wp = w;
        wp.a = aP;
        const float off = std::fabs(Profile::step(w, jr, hk).p - o.p);
        if (off <= std::fmax(kKnotTol, std::fabs(Profile::step(wp, jP, hk).p - o.p))) break;
        if (on_curve) { on_curve = false; continue; }
        w = bw; r_us = br_us; h_us = bh_us; use_us = r_us;
        jr = (aR - w.a) / (float(r_us) * 1e-6f);
        o.a_in = w.a;
        break;
    }
    const float Tu = float(use_us) * 1e-6f;
    Profile ramp;
    ramp.start_us = o.t_us - h_us; ramp.s0 = w; ramp.n = 1; ramp.dt[0] = Tu; ramp.jerk[0] = jr;
    ramp.ends_at_rest = false;
    // The exit is where the ramp ends, never the knot's state: the next piece
    // starts from it, so the curve has no jump.
    State exit = Profile::step(w, jr, Tu);
    // A ramp into a flat span lands at rest; what is left of v and a is the
    // microsecond rounding of its length, below what a sample reads.
    if (into_flat && aR == 0.0f && std::fabs(exit.v) <= L.vmax * 1e-4f && std::fabs(exit.a) <= L.amax * 1e-3f) {
        exit.v = 0.0f; exit.a = 0.0f;
        ramp.ends_at_rest = true;
    }
    o.corner = true; o.head_us = ramp.start_us; o.head = w; o.ramp = ramp;
    o.t_us = ramp.start_us + use_us;
    o.p = exit.p; o.v = exit.v; o.a = exit.a;
}

// A profile's ratio per ceiling; leaving the window reads as just over. Its
// jerk is jmax by construction (to the microsecond rounding of its phases).
inline handles::Over profOver(const Profile& pr, const handles::Cfg& c) {
    handles::Over o;
    o.v = pr.worstRatio(Limits{c.lim.vmax, INFINITY, c.lim.jmax}, -1e30f, 1e30f);
    o.a = pr.worstRatio(Limits{INFINITY, c.lim.amax, c.lim.jmax}, -1e30f, 1e30f);
    if (pr.worstRatio(c.lim, c.lo, c.hi) >= 1e29f) o.x = 1.0f + 2.0f * handles::kTol;
    return o;
}

// x held to the ceiling when it is past the legality tolerance; a legal value
// is kept bit for bit.
inline float withinCeiling(float x, float ceiling) {
    return std::fabs(x) > ceiling * (1.0f + handles::kTol) ? std::copysign(ceiling, x) : x;
}

// The worst ceiling ratio of a built piece and its lead ramp.
inline float builtOver(const Piece& pc, const handles::Cfg& c) {
    float w = handles::overOf(pc.q, pc.p0, c);
    if (pc.has_lead) {
        const handles::Over pl = profOver(pc.lead, c);
        w = std::fmax(w, std::fmax(std::fmax(pl.v, pl.a), std::fmax(pl.j, pl.x)));
    }
    return w;
}

// ---- one run of the renderer -------------------------------------------------
// Fills out[r - 1] from the render of knots k[0..m) (te: their times), piece by
// piece from state s as the engine will build them, with the corner ramp
// where the two pieces at a knot differ in acceleration.
inline void emitRun(const handles::HKnot* k, const uint64_t* te, int m, const Knot* kn, const State& s,
                    const handles::Cfg& c, Solved* out) {
    State st = s, sp = s;   // the state the piece into knot r - 1 started from: sp
    uint64_t st_us = te[0], sp_us = te[0];
    for (int r = 1; r < m; ++r) {
        const Knot& K = kn[r - 1];
        Solved& o = out[r - 1];
        o = Solved{};
        o.t_us = te[r];
        o.base_us = K.t_us;
        o.p = handles::rendered(k[r]);
        o.knot_p = o.p;
        o.v = k[r].vel;
        o.i0 = k[r - 1].effOut;
        o.i1 = k[r].effIn;
        const float pL = handles::rendered(k[r - 1]);
        const handles::Piece qL = handles::pieceOf(k, r - 1);
        // A corner before this piece ends past its knot: the piece starts on
        // its exit with the render's curve from there on.
        if (st_us > te[r - 1] && st_us < te[r]) {
            const handles::Piece tl = tailOf(qL, handles::solveU(qL, float(st_us - te[r - 1]) * 1e-6f));
            if (tl.i0 >= handles::kLMin && tl.i0 <= handles::kLMax && tl.i1 >= handles::kLMin && tl.i1 <= handles::kLMax
                && std::fabs(tl.s0 - st.v) <= 1e-3f * c.lim.vmax) {
                o.i0 = tl.i0;
                o.i1 = tl.i1;
            }
        }
        o.worst = handles::overOf(qL, pL, c);
        // Infeasible against the ceilings themselves: a piece the render found
        // illegal only under its tightened share (slack) is legal.
        o.infeasible = k[r].infeasible && o.worst > 1.0f + handles::kTol;
        o.clamped = K.has_v && o.v != K.v;
        // A hold after a trim moves with it and keeps its whole (zero) chord:
        // share stays 1 (the ABI reads share < 1 as a trim).
        const float authoredPrev = r > 1 ? kn[r - 2].p : s.p;
        if (k[r].dp != 0.0f && K.p != pL && std::fabs(K.p - authoredPrev) > c.holdEps)
            o.share = (o.p - pL) / (K.p - pL);
        o.a_in = handles::aEndOf(qL);
        const bool from_flat = std::fabs(qL.D) <= c.holdEps && qL.s0 == 0.0f && qL.s1 == 0.0f;
        // A hold after a ramp that landed at rest within kKnotTol of its knot
        // holds where it landed: closing that gap in what is left of the span
        // is a move no jerk ceiling allows.
        const float vr = c.lim.vmax * 1e-4f, ar = c.lim.amax * 1e-3f;
        if (std::fabs(qL.D) <= c.holdEps && std::fabs(qL.s0) <= vr && std::fabs(qL.s1) <= vr && std::fabs(st.v) <= vr
            && std::fabs(st.a) <= ar && std::fabs(o.p - st.p) <= kKnotTol)
            o.p = o.knot_p = st.p;
        Piece in = Piece::bezier(st_us, st, o.t_us, State{o.p, o.v, o.a_in}, o.i0, o.i1, c.lim);
        // The piece starts from the state the one before it built, not the
        // render's knot: one the render fit legal that builds over a ceiling
        // or out of the window takes the nearest legal length factor of its own.
        if (in.T > 0.0f && !(o.worst > 1.0f + handles::kTol) && builtOver(in, c) > 1.0f + handles::kTol) {
            bool found = false;
            for (int keep = 0; keep < 2 && !found; ++keep)
                for (int idx = keep ? 0 : 1; idx < handles::kKs && !found; ++idx) {
                    const float f = handles::ksAt(idx);
                    const float i0 = handles::clampL(o.i0 * f), i1 = handles::clampL(o.i1 * f);
                    const Piece t = Piece::bezier(st_us, st, o.t_us, State{o.p, o.v, o.a_in}, i0, i1, c.lim, false, keep != 0);
                    if (builtOver(t, c) <= 1.0f + handles::kTol) { o.i0 = i0; o.i1 = i1; o.keep_lengths = keep != 0; in = t; found = true; }
                }
        }
        o.a = in.T > 0.0f ? handles::aEndOf(in.q) : 0.0f;
        if (r + 1 < m) {
            const handles::Piece qR = handles::pieceOf(k, r);
            const bool into_flat = std::fabs(qR.D) <= c.holdEps && qR.s0 == 0.0f && qR.s1 == 0.0f;
            cornerAt(o, o.a, handles::aStartOf(qR), from_flat, into_flat, st, st_us, te[r + 1] - te[r], c.lim, &qL, &qR);
        } else if (o.v == 0.0f) {
            cornerAt(o, o.a, 0.0f, from_flat, true, st, st_us, ~uint64_t(0) / 4, c.lim);
        }
        // A corner ramp can pass the knot off its rendered position (a flat
        // knot's turn lands on its height, not at its time): where it passes
        // is knot_p, flagged missed. share stays the render's trim alone.
        if (o.corner && te[r] >= o.head_us && te[r] <= o.t_us) {
            const float pk = Profile::step(o.ramp.s0, o.ramp.jerk[0], float(te[r] - o.head_us) * 1e-6f).p;
            if (std::fabs(pk - o.knot_p) > kKnotTol) {
                o.knot_p = pk;
                o.missed = true;
            }
        }
        // Where the next piece, a starvation brake and a re-plan start: inside
        // the ceilings. A piece no trim makes legal ends past one (a hold a
        // tick after a moving knot ends at several times amax), and the next
        // piece's lead unwound that at jmax, gaining a^2/2J of speed
        // (kin-554). The piece into the knot keeps its own end; it is reported.
        // A corner's exit stays: it is the next piece's own start, reached by
        // the ramp at jmax and judged with it.
        if (!o.corner) { o.v = withinCeiling(o.v, c.lim.vmax); o.a = withinCeiling(o.a, c.lim.amax); }
        // A start step under kStepS where a corner ramp ends reads over jmax
        // on the 1 ms grid when the tick before it is near jmax already: the
        // piece the engine builds carries it exactly, unless that takes the
        // piece further over a ceiling or out of the window (every
        // smoothness, kin-9od3).
        if (r > 1 && out[r - 2].corner) {
            const Piece pl = buildPiece(st, st_us, o, c.lim);
            const uint64_t back = st_us > sp_us + kMinSpanUs ? st_us - kMinSpanUs : sp_us;
            if (pl.T > 0.0f && !pl.has_lead && pl.q.da == 0.0f
                && std::fabs(handles::aStartOf(pl.q) - buildPiece(sp, sp_us, out[r - 2], c.lim).at(back).a)
                       > c.lim.jmax * handles::kTick * (1.0f + handles::kTol)) {
                o.exact_start = true;
                const Piece ex = buildPiece(st, st_us, o, c.lim);
                o.exact_start = handles::overOf(ex.q, ex.p0, c) <= std::fmax(1.0f + handles::kTol, handles::overOf(pl.q, pl.p0, c));
            }
        }
        sp = st;
        sp_us = st_us;
        st = State{o.p, o.v, o.a};
        st_us = o.t_us;
    }
}

// Renders kn[0..cnt) from state s at s_us into out[0..cnt); reports carry the
// window index base + i and the knot's authored position. A built piece (its
// lead and start correction included), corner ramp or last brake over a ceiling (what
// the render alone does not see) tightens the ceilings its pieces are judged
// against by its excess and the run renders again, until a pass tightens
// nothing; what is still over after that is reported PieceOverCeiling.
// Converges by construction: every pass that renders again lowered a share
// (a slack, held to kSlackFloor; the last knot's rail or cap), each only ever
// down. kSlackCap bounds it; a run the cap stops reports every piece still
// asking (PieceOverCeiling), never a time stretch.
inline constexpr int   kSlackCap   = 16;
// A ceiling no tightening reaches (a jerk step at a corner) stops here: a
// share near zero ranked the least-over fit by a ceiling no piece can meet.
inline constexpr float kSlackFloor = 0.25f;
// A tightening that did not cut its excess lowers the share at least this
// much the next time: a share no fit answers reaches kSlackFloor in at most
// 28 such passes.
inline constexpr float kSlackStep  = 0.95f;
template <typename Report>
inline void renderRun(const State& s, uint64_t s_us, const Knot* kn, size_t cnt, const handles::Cfg& c,
                      Solved* out, size_t base, bool last, Report& report, Workspace& ws) {
    K2_STAT(knots, cnt);
    handles::HKnot* k = ws.k;
    uint64_t* te = ws.t_us;
    const int m = int(cnt) + 1;
    k[0] = handles::HKnot{};
    k[0].p = s.p; k[0].v = s.v; k[0].has_v = true; k[0].aIn = s.a;
    te[0] = s_us;
    for (int r = 1; r < m; ++r) {
        const Knot& K = kn[r - 1];
        te[r] = K.t_us > te[r - 1] + kMinSpanUs ? K.t_us : te[r - 1] + kMinSpanUs;
        k[r] = handles::HKnot{};
        k[r].t = float(te[r] - s_us) * 1e-6f;
        k[r].p = K.p; k[r].v = K.v; k[r].has_v = K.has_v;
    }
    for (int r = 0; r < m; ++r)
        for (int x = 0; x < 3; ++x) { ws.last[r][x] = INFINITY; ws.ask[r][x] = 0.0f; }
    bool capped = false;
    for (int pass = 0;; ++pass) {
        K2_STAT(judges, 1);
        handles::render(k, m, c);
        emitRun(k, te, m, kn, s, c, out);
        const bool final = pass == kSlackCap;
        bool again = false;   // final: a piece still asks (the cap stopped the run)
        // Only the ceiling that is over tightens; a window excursion tightens
        // all three. A built piece within half the tolerance of a ceiling
        // tightens too: its u samples read under the 1 ms grid by about that.
        // Speed and acceleration tighten from what the render itself reads of
        // the piece (a lead ramp can put the built piece over while the
        // render is well inside), by at most half per pass.
        auto tighten = [&](int r, const handles::Over& o) {
            if (r >= m) return;
            handles::Over rp;
            handles::overOf(handles::pieceOf(k, r - 1), handles::rendered(k[r - 1]), c, INFINITY, &rp);
            const float rs[3] = {o.v, o.a, o.j}, own[3] = {rp.v, rp.a, INFINITY};
            for (int x = 0; x < 3; ++x) {
                const float w = std::fmax(rs[x], o.x);
                if (w <= 1.0f + 0.5f * handles::kTol) continue;
                ws.ask[r][x] = std::fmax(ws.ask[r][x], w);
                const float s = std::fmin(k[r].slack[x], std::fmax(own[x], 0.5f * k[r].slack[x]));
                float t = s * (1.0f - 0.5f * handles::kTol) / w;
                // Stalled (the last tightening did not cut the excess): at
                // least kSlackStep, so a share no fit answers reaches its floor.
                if (!(w < ws.last[r][x] * (1.0f - 0.5f * handles::kTol))) t = std::fmin(t, kSlackStep * k[r].slack[x]);
                t = std::fmax(kSlackFloor, t);
                if (t < k[r].slack[x]) { if (!final) k[r].slack[x] = t; again = true; }
            }
        };
        float* built = ws.built;
        State st = s;
        uint64_t st_us = s_us;
        float j_end = 0.0f;   // |jerk| where the previous piece (or its ramp) ends
        for (int r = 1; r < m; ++r) {
            const Solved& o = out[r - 1];
            const Piece pc = buildPiece(st, st_us, o, c.lim);
            built[r] = 0.0f;
            float step_ratio = 0.0f;
            if (pc.T > 0.0f && !o.hard) {
                // What the 1 ms grid reads of what the engine builds: the
                // piece with its lead and start correction, and an
                // acceleration step where it ends into a corner ramp; a step a
                // sample interval spans also carries the jerk beside it.
                const float j1ms = c.lim.jmax * 1e-3f;
                handles::Over po;
                float w = handles::overOf(pc.q, pc.p0, c, INFINITY, &po);
                if (pc.has_lead) {
                    const handles::Over pl = profOver(pc.lead, c);
                    po = handles::Over{std::fmax(po.v, pl.v), std::fmax(po.a, pl.a), std::fmax(po.j, pl.j), std::fmax(po.x, pl.x)};
                    w = std::fmax(w, std::fmax(std::fmax(pl.v, pl.a), std::fmax(pl.j, pl.x)));
                }
                const float a0 = pc.has_lead ? pc.lead.end().a : st.a;
                const float da0 = std::fabs(handles::aStartOf(pc.q) - a0);
                const float jStart = std::fabs(handles::evalPiece(pc.q, 0.0f).j);
                const float jBefore = pc.has_lead ? std::fabs(pc.lead.jerk[0]) : j_end;
                if (da0 > c.lim.jmax * kStepS) step_ratio = da0 / j1ms + std::fmax(jStart, jBefore) / c.lim.jmax;
                if (o.corner) {
                    const float da1 = std::fabs(handles::aEndOf(pc.q) - o.ramp.s0.a);
                    const float jj = std::fmax(std::fabs(handles::evalPiece(pc.q, 1.0f).j), std::fabs(o.ramp.jerk[0]));
                    if (da1 > c.lim.jmax * kStepS) step_ratio = std::fmax(step_ratio, da1 / j1ms + jj / c.lim.jmax);
                }
                built[r] = std::fmax(w, step_ratio);
                po.j = std::fmax(po.j, step_ratio);
                if (!o.infeasible) tighten(r, po);
            }
            if (o.corner) {
                const handles::Over pr = profOver(o.ramp, c);
                built[r] = std::fmax(built[r], std::fmax(std::fmax(pr.v, pr.a), std::fmax(pr.j, pr.x)));
                tighten(r, pr);
                tighten(r + 1, pr);
            }
            // The newest knot of a window may be the last: the engine brakes
            // from it, inside the window. An authored angle is held lower.
            if (last && r + 1 == m && !o.hard && (o.v != 0.0f || o.a != 0.0f)) {
                const Profile br = Profile::brake(State{o.p, o.v, o.a}, 0, c.lim);
                const State e = br.end();
                const float gap = o.v > 0.0f ? c.hi - o.p : o.p - c.lo, run = std::fabs(e.p - o.p);
                const float wb = br.worstRatio(c.lim, -1e30f, 1e30f);
                const bool out_of = e.p < c.lo - 1e-6f || e.p > c.hi + 1e-6f;
                built[r] = std::fmax(built[r], out_of ? 1.0f + (run - gap) / (c.hi - c.lo) : wb);
                if (final && (out_of || wb > 1.0f + handles::kTol)) again = true;
                else if (out_of || wb > 1.0f + handles::kTol) {
                    if (kn[r - 1].has_v) k[r].rail *= out_of ? 0.95f * gap / run : 0.95f;
                    // A brake over a ceiling (its speed overshoots while the
                    // acceleration it enters with ramps out) caps the angle.
                    if (wb > 1.0f + handles::kTol) k[r].vcap = std::fmin(k[r].vcap, std::fabs(o.v) / wb * (1.0f - handles::kTol));
                    const float wt = std::fmax(wb, 1.0f + 2.0f * handles::kTol);
                    tighten(r, handles::Over{wt, wt, wt, 1.0f});
                    again = true;
                }
            }
            j_end = o.corner ? std::fabs(o.ramp.jerk[0])
                  : (pc.T > 0.0f && !o.hard) ? std::fabs(handles::evalPiece(pc.q, 1.0f).j) : 0.0f;
            st = State{o.p, o.v, o.a};
            st_us = o.t_us;
        }
        for (int r = 0; r < m; ++r)
            for (int x = 0; x < 3; ++x)
                if (ws.ask[r][x] > 0.0f) { ws.last[r][x] = ws.ask[r][x]; ws.ask[r][x] = 0.0f; }
        if (!again || final) {
            capped = final && again;
            break;
        }
    }
    if (capped) K2_STAT(capped, 1);
    // Over a ceiling as built after the last pass: rendered at its least-over
    // trim and reported (kin-y6e rule 5); after the cap, every piece still
    // asking to tighten.
    const float bar = 1.0f + (capped ? 0.5f : 1.0f) * handles::kTol;
    for (int r = 1; r < m; ++r) {
        Solved& o = out[r - 1];
        if (ws.built[r] > bar) { o.infeasible = true; o.worst = std::fmax(o.worst, ws.built[r]); }
    }
    for (int r = 1; r < m; ++r) {
        const Knot& K = kn[r - 1];
        const Solved& o = out[r - 1];
        const size_t idx = base + size_t(r - 1);
        if (o.clamped) report(AnomalyKind::EndVelClamped, idx, o.t_us, K.p, o.v);
        const float authoredPrev = r > 1 ? kn[r - 2].p : s.p;
        if (k[r].dp != 0.0f && std::fabs(K.p - authoredPrev) > c.holdEps)
            report(AnomalyKind::KnotTrimmed, idx, o.t_us, K.p, o.share);
        if (o.infeasible) report(AnomalyKind::PieceOverCeiling, idx, o.t_us, K.p, o.worst);
    }
}

// ---- CHASE: samples ------------------------------------------------------------
// A sample carries position only (operator ruling 2026-10-08, kin-j6g): a run
// of samples renders as ONE fastest legal move from s to rest on the newest
// of them (Profile::point, no hold), never a Bezier, never trimmed. The
// earlier samples of the run are passed wherever the profile is at their
// times; the newest lands at its time, or at the profile's end when that is
// later (stretched, like a HARD knot). Each new sample re-plans the run from
// the live origin, so a stream in motion never reaches the rest. False when
// the profile is full or leaves the window: the run then renders as knots.
inline bool chaseRun(const State& s, uint64_t s_us, const Knot* kn, size_t cnt, const handles::Cfg& c, Solved* out) {
    const Knot& last = kn[cnt - 1];
    float fastest = 0.0f;
    const Profile pr = Profile::point(s, last.p, s_us, c.lim, 0.0f, &fastest);
    const float ratio = pr.n < 0 ? kIllegal : pr.worstRatio(c.lim, c.lo, c.hi);
    if (!(ratio <= 1.0001f)) return false;
    const uint64_t end_us = pr.n > 0 ? pr.end_us() : s_us;
    uint64_t prev_us = s_us;
    for (size_t r = 0; r < cnt; ++r) {
        const Knot& K = kn[r];
        Solved& o = out[r];
        o = Solved{};
        o.base_us = K.t_us;
        o.knot_p = K.p;
        o.hard = true;
        o.ramp = pr;
        o.worst = ratio;
        const bool newest = r + 1 == cnt;
        uint64_t t_us = K.t_us > prev_us + kMinSpanUs ? K.t_us : prev_us + kMinSpanUs;
        if (newest && end_us > t_us) t_us = end_us;
        o.t_us = t_us;
        o.stretched_s = newest && end_us > K.t_us ? float(end_us - K.t_us) * 1e-6f : 0.0f;
        const State st = pr.n > 0 ? pr.atSeconds(float(t_us - s_us) * 1e-6f) : State{s.p, 0.0f, 0.0f};
        o.p = st.p; o.v = st.v; o.a = st.a;
        if (newest || t_us >= end_us) { o.p = last.p; o.v = 0.0f; o.a = 0.0f; }
        prev_us = t_us;
    }
    return true;
}

// ---- the solver --------------------------------------------------------------
// knots[0..n) pending, in time order; origin is the state the first piece
// starts from. Writes out[0..n) and reports every trim, every infeasible piece
// and every clamped authored velocity through `report`. Returns n: a window is
// always solved whole.
template <typename Report>
inline size_t solveWindow(const State& origin, uint64_t origin_us, const Knot* knots, size_t n,
                          const Config& cfg, Solved* out, Report&& report, Workspace& ws) {
    if (n == 0) return 0;
    K2_STAT(windows, 1);
    handles::Cfg c;
    c.lim = cfg.limits;
    c.lo = std::fmin(0.0f, origin.p);
    c.hi = std::fmax(1.0f, origin.p);
    c.holdEps = kHoldEps;
    c.lfloor = cfg.handle_floor;
    c.trim = cfg.trim_max * (c.hi - c.lo);
    c.smoothness = cfg.smoothness;
    c.trimLast = true;
    c.railStop = true;
    State s = origin;
    uint64_t s_us = origin_us;
    size_t i = 0;
    while (i < n) {
        if (knots[i].sample && !knots[i].has_v) {
            // A run of position-only samples: the chase.
            size_t j = i + 1;
            while (j < n && j - i < kWindowKnots && knots[j].sample && !knots[j].has_v) ++j;
            if (chaseRun(s, s_us, knots + i, j - i, c, out + i)) {
                s = State{out[j - 1].p, 0.0f, 0.0f};
                s_us = out[j - 1].t_us;
                i = j;
                continue;
            }
        }
        if (junctionOf(knots[i]) == Junction::Hard && hardKnot(s, s_us, knots[i], c, out[i])) {
            if (out[i].infeasible) report(AnomalyKind::PieceOverCeiling, i, out[i].t_us, knots[i].p, out[i].worst);
            s = State{out[i].p, 0.0f, 0.0f};
            s_us = out[i].t_us;
            ++i;
            continue;
        }
        size_t j = i + 1;
        while (j < n && j - i < kWindowKnots && junctionOf(knots[j]) != Junction::Hard) ++j;
        renderRun(s, s_us, knots + i, j - i, c, out + i, i, j == n, report, ws);
        s = State{out[j - 1].p, out[j - 1].v, out[j - 1].a};
        s_us = out[j - 1].t_us;
        i = j;
    }
    return n;
}

}  // namespace kinetic2

