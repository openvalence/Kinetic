// kinetic2/engine_piece.hpp -- one quintic Hermite piece in normalized tau
// Constraints: coefficients in tau; the derivatives are scaled by T on
// evaluation; T = 0 is a hold at c[0]. Float throughout.
#pragma once

#include <cstdint>

#include "profile.hpp"
#include "types.hpp"

namespace kinetic2 {

// A quintic Hermite piece in normalized tau over T seconds, from (p0, v0, a0)
// at tau = 0 to (p1, v1, a1) at tau = 1. Coefficients in tau; the derivatives
// are scaled back by T on evaluation.
struct Piece {
    uint64_t start_us = 0;
    uint64_t end_us   = 0;
    float    T        = 0.0f;   // seconds of the polynomial head; 0 = a hold at c[0]
    float    c[6]     = {};
    // A brake tail (RFC-105 HARD junction, and brake()): from tail.start_us
    // the profile renders instead of the polynomial. has_tail with T == 0 is
    // a bare profile.
    bool     has_tail = false;
    Profile  tail{};

    static Piece profile(const Profile& pr) {
        Piece q; q.start_us = pr.start_us; q.end_us = pr.end_us(); q.T = 0.0f; q.c[0] = pr.s0.p;
        q.has_tail = true; q.tail = pr;
        return q;
    }

    static Piece hold(float p, uint64_t from) {
        Piece h; h.start_us = from; h.end_us = from; h.T = 0.0f; h.c[0] = p;
        return h;
    }

    static Piece hermite(uint64_t t0, const State& s0, uint64_t t1, const State& s1) {
        Piece q; q.start_us = t0; q.end_us = t1;
        const float T = float(t1 - t0) * 1e-6f;
        q.T = T;
        // Boundary derivatives in tau units.
        const float p0 = s0.p, m0 = s0.v * T, k0 = s0.a * T * T;
        const float p1 = s1.p, m1 = s1.v * T, k1 = s1.a * T * T;
        q.c[0] = p0;
        q.c[1] = m0;
        q.c[2] = 0.5f * k0;
        q.c[3] = 10.0f * (p1 - p0) - 6.0f * m0 - 4.0f * m1 - 1.5f * k0 + 0.5f * k1;
        q.c[4] = -15.0f * (p1 - p0) + 8.0f * m0 + 7.0f * m1 + 1.5f * k0 - k1;
        q.c[5] = 6.0f * (p1 - p0) - 3.0f * (m0 + m1) - 0.5f * (k0 - k1);
        return q;
    }

    State at(uint64_t t) const {
        State s;
        if (has_tail && t >= tail.start_us) return tail.at(t);
        if (T <= 0.0f) { s.p = c[0]; return s; }
        float tau = float(t - start_us) * 1e-6f / T;
        if (tau < 0.0f) tau = 0.0f;
        if (tau > 1.0f) tau = 1.0f;
        const float t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
        s.p = c[0] + c[1] * tau + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
        const float dp = c[1] + 2.0f * c[2] * tau + 3.0f * c[3] * t2 + 4.0f * c[4] * t3 + 5.0f * c[5] * t4;
        const float ddp = 2.0f * c[2] + 6.0f * c[3] * tau + 12.0f * c[4] * t2 + 20.0f * c[5] * t3;
        s.v = dp / T;
        s.a = ddp / (T * T);
        return s;
    }
};

}  // namespace kinetic2
