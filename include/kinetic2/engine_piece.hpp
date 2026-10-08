// kinetic2/engine_piece.hpp -- one rendered piece: a cubic Bezier in the
// time-position plane (handles.hpp) with an optional profile tail after it
// Constraints:
// - u is the curve parameter, never time: every sample solves t(u) = t from
//   t / T on every call (never from the previous sample's u), so a sample
//   depends on its time alone.
// - Both end accelerations are matched by the handle lengths when that stays
//   inside the ceilings, else the start alone; an unmatched start becomes a
//   lead ramp at jmax (at most half the span) when the piece rebuilt after it
//   lands near the ramp's end, and the start correction (handles::Piece::da)
//   carries what is left: the acceleration is continuous where a piece
//   starts. An unmatched end is the piece's own; the solver ramps from it at
//   a corner.
// - T = 0 is a hold at p0. Float throughout.
#pragma once

#include <cmath>
#include <cstdint>

#include "handles.hpp"
#include "profile.hpp"
#include "types.hpp"

namespace kinetic2 {

// An acceleration step a 1 ms sample reads as 1% of jmax: below it, ends match.
inline constexpr float kStepS = 1e-5f;

struct Piece {
    uint64_t start_us = 0;
    uint64_t end_us   = 0;
    float    T        = 0.0f;   // seconds of the Bezier; 0 = a hold at p0
    float    p0       = 0.0f;   // the Bezier's start position
    uint64_t bez_us   = 0;      // the Bezier's start time
    handles::Piece q{};
    // A lead ramp: from start_us to bez_us the profile renders, carrying the
    // start acceleration toward the Bezier's.
    bool     has_lead = false;
    Profile  lead{};
    // A tail (a HARD junction, a corner ramp, brake()): from tail.start_us the
    // profile renders instead of the Bezier. has_tail with T == 0 is a bare
    // profile.
    bool     has_tail = false;
    Profile  tail{};

    static Piece profile(const Profile& pr) {
        Piece q; q.start_us = pr.start_us; q.end_us = pr.end_us(); q.T = 0.0f; q.p0 = pr.s0.p;
        q.has_tail = true; q.tail = pr;
        return q;
    }

    static Piece hold(float p, uint64_t from) {
        Piece h; h.start_us = from; h.end_us = from; h.bez_us = from; h.T = 0.0f; h.p0 = p;
        return h;
    }

    // Both end accelerations matched (a0, a1), else the start alone, by the
    // lengths, when the matched piece is no further over a ceiling than the
    // unmatched one; false (the start unmatched) leaves q unchanged.
    static bool match(handles::Piece& q, float a0, float a1, const Limits& L) {
        const float tol = L.jmax * kStepS;
        handles::Cfg c; c.lim = L; c.lo = -1e30f; c.hi = 1e30f;
        const handles::Piece keep = q;
        const float bound = std::fmax(1.0f + handles::kTol, handles::overOf(keep, 0.0f, c));
        if (handles::matchEnds(q, a0, a1, tol) && handles::overOf(q, 0.0f, c) <= bound) return true;
        q = keep;
        if (std::fabs(handles::aStartOf(q) - a0) <= tol) return true;
        if (handles::matchStartLength(q, a0) && handles::overOf(q, 0.0f, c) <= bound) return true;
        q = keep;
        return false;
    }

    // From s0 at t0 to s1 at t1 with the render's handle lengths i0 / i1, both
    // end accelerations matched as match() allows; else a lead ramp, else the
    // start correction.
    static Piece bezier(uint64_t t0, const State& s0, uint64_t t1, const State& s1, float i0, float i1,
                        const Limits& L) {
        Piece pc; pc.start_us = t0; pc.end_us = t1; pc.bez_us = t0; pc.p0 = s0.p;
        if (t1 <= t0) { pc.p0 = s1.p; return pc; }
        handles::Piece q{float(t1 - t0) * 1e-6f, s1.p - s0.p, s0.v, s1.v, i0, i1};
        if (!match(q, s0.a, s1.a, L)) {
            // The render's lengths stay: the renderer sized the start ramp's
            // room for them (handles::startRoom). A length moved toward the
            // start acceleration here rendered a lone 200 ms segment as a 150
            // ms crawl and a spike (kin-b1d).
            // The piece rebuilt after the ramp starts where the ramp ends: the
            // ramp aims at the rebuilt piece's start until the two agree. One
            // that still misses by more than 0.1 ms of jmax is no lead.
            const handles::Piece q0 = q;
            for (int round = 0; round < 4; ++round) {
                const float da = handles::aStartOf(q) - s0.a;
                const uint64_t tr_us = uint64_t(std::fabs(da) / L.jmax * 1e6f + 0.5f);
                if (!(tr_us > 0 && 2 * tr_us + 2000 <= t1 - t0)) break;
                const float tr = float(tr_us) * 1e-6f;
                Profile lead; lead.start_us = t0; lead.s0 = s0; lead.n = 1; lead.dt[0] = tr;
                lead.jerk[0] = da / tr; lead.ends_at_rest = false;
                const State e = Profile::step(s0, lead.jerk[0], tr);
                pc.has_lead = true; pc.lead = lead; pc.bez_us = t0 + tr_us; pc.p0 = e.p;
                q = handles::Piece{float(t1 - pc.bez_us) * 1e-6f, s1.p - e.p, e.v, s1.v, i0, i1};
                if (match(q, e.a, s1.a, L) || std::fabs(handles::aStartOf(q) - e.a) <= L.jmax * kStepS) break;
            }
            if (pc.has_lead && std::fabs(handles::aStartOf(q) - pc.lead.end().a) > L.jmax * 1e-4f) {
                q = q0;
                pc.has_lead = false; pc.lead = Profile{}; pc.bez_us = t0; pc.p0 = s0.p;
            }
            q.da = (pc.has_lead ? pc.lead.end().a : s0.a) - handles::aStartOf(q);
        }
        pc.q = q; pc.T = q.T;
        return pc;
    }

    State at(uint64_t t) const {
        if (has_tail && t >= tail.start_us) return tail.at(t);
        if (has_lead && t < bez_us) return lead.at(t);
        State s;
        if (T <= 0.0f) { s.p = p0; return s; }
        // Before its start the piece holds its start state.
        const float rel = t <= bez_us ? 0.0f : float(t - bez_us) * 1e-6f;
        const handles::Eval e = handles::evalPiece(q, handles::solveU(q, rel));
        s.p = p0 + e.p; s.v = e.v; s.a = e.a;
        return s;
    }
};

}  // namespace kinetic2
