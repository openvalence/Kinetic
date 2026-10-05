// kinetic2/profile.hpp -- constant-jerk phase profiles: the brake
// Constraints:
// - A profile is up to four phases of constant jerk from a start state; the
//   state is integrated in closed form per phase, never stepped.
// - brake() is the fastest legal stop from any (p, v, a) under amax and jmax:
//   ramp to the decel ceiling, hold it, ramp out, so v and a reach 0 together.
//   When the entry acceleration already overshoots the stop (v would cross 0
//   before a can ramp to 0), a first phase ramps a to 0 and the stop is
//   planned again from there, the one case that needs the fourth phase.
// - Jerk never exceeds jmax by construction; |a| peaks at a phase boundary;
//   |v| and p peak at a boundary or where a or v cross 0 inside a phase. The
//   referee here checks exactly those points.
#pragma once

#include <cmath>
#include <cstdint>

#include "types.hpp"

namespace kinetic2 {

struct Profile {
    static constexpr int kMaxPhases = 4;
    uint64_t start_us = 0;
    State    s0{};
    int      n = 0;
    float    dt[kMaxPhases] = {};   // seconds
    float    jerk[kMaxPhases] = {};

    float duration() const { float T = 0.0f; for (int i = 0; i < n; ++i) T += dt[i]; return T; }
    uint64_t end_us() const { return start_us + uint64_t(duration() * 1e6f + 0.5f); }

    static State step(const State& s, float j, float t) {
        State o;
        o.p = s.p + s.v * t + 0.5f * s.a * t * t + j * t * t * t / 6.0f;
        o.v = s.v + s.a * t + 0.5f * j * t * t;
        o.a = s.a + j * t;
        return o;
    }

    // The state at seconds `t` from the start, held at the end past it.
    State atSeconds(float t) const {
        State s = s0;
        for (int i = 0; i < n; ++i) {
            if (t <= dt[i]) return step(s, jerk[i], t < 0.0f ? 0.0f : t);
            s = step(s, jerk[i], dt[i]);
            t -= dt[i];
        }
        s.v = 0.0f; s.a = 0.0f;   // a profile ends at rest by construction; pin it
        return s;
    }
    State at(uint64_t t_us) const {
        return atSeconds(t_us <= start_us ? 0.0f : float(t_us - start_us) * 1e-6f);
    }
    State end() const { return atSeconds(duration() + 1.0f); }

    // The fastest legal stop from s under L. A state already at rest gives an
    // empty profile.
    static Profile brake(const State& s, uint64_t start_us, const Limits& L) {
        Profile pr; pr.start_us = start_us; pr.s0 = s;
        const float J = L.jmax, A = L.amax;
        if (std::fabs(s.v) < 1e-9f && std::fabs(s.a) < 1e-9f) return pr;
        // Direction of travel to stop: the velocity's, else the acceleration's.
        const float sgn = s.v != 0.0f ? (s.v > 0.0f ? 1.0f : -1.0f) : (s.a > 0.0f ? 1.0f : -1.0f);
        // Decel-positive frame: u = -sgn v (<= 0, rising to 0), b = -sgn a.
        const float speed = std::fabs(s.v);
        const float b0 = -sgn * s.a;
        // Trapezoid at the ceiling?
        float t1 = (A - b0) / J;
        if (t1 < 0.0f) t1 = 0.0f;
        const float du1 = b0 * t1 + 0.5f * J * t1 * t1;
        const float bpk_trap = b0 + J * t1;
        const float du3 = bpk_trap * bpk_trap / (2.0f * J);
        const float t2 = (speed - du1 - du3) / A;
        if (t2 >= 0.0f) {
            pr.n = 3;
            pr.dt[0] = t1; pr.jerk[0] = -sgn * J;
            pr.dt[1] = t2; pr.jerk[1] = 0.0f;
            pr.dt[2] = bpk_trap / J; pr.jerk[2] = sgn * J;
            return pr;
        }
        // Triangle: the peak decel that stops exactly.
        const float bpk2 = 0.5f * (2.0f * J * speed + b0 * b0);
        const float bpk = std::sqrt(bpk2 > 0.0f ? bpk2 : 0.0f);
        if (bpk >= b0) {
            pr.n = 2;
            pr.dt[0] = (bpk - b0) / J; pr.jerk[0] = -sgn * J;
            pr.dt[1] = bpk / J;        pr.jerk[1] = sgn * J;
            return pr;
        }
        // Overshoot: a is already too large to stop in time. Ramp it to 0,
        // then stop from the reversed state.
        const float t0 = b0 / J;
        pr.n = 1; pr.dt[0] = t0; pr.jerk[0] = sgn * J;
        const State mid = step(s, pr.jerk[0], t0);
        State mid0 = mid; mid0.a = 0.0f;
        const Profile rest = brake(mid0, 0, L);
        for (int i = 0; i < rest.n && pr.n < kMaxPhases; ++i) { pr.dt[pr.n] = rest.dt[i]; pr.jerk[pr.n] = rest.jerk[i]; ++pr.n; }
        return pr;
    }

    // Worst (peak / ceiling) ratio over v, a and the window; jerk is at most
    // jmax by construction. A window excursion is illegal outright.
    float worstRatio(const Limits& L, float lo, float hi) const {
        if (!(L.vmax > 0.0f) || !(L.amax > 0.0f)) return 1e30f;
        float worst = 0.0f;
        State s = s0;
        auto judge = [&](const State& x) {
            if (x.p < lo - 1e-6f || x.p > hi + 1e-6f) worst = 1e30f;
            worst = std::fmax(worst, std::fabs(x.v) / L.vmax);
            worst = std::fmax(worst, std::fabs(x.a) / L.amax);
        };
        judge(s);
        for (int i = 0; i < n; ++i) {
            const float j = jerk[i], T = dt[i];
            // a crosses 0 inside the phase: a |v| extremum.
            if (j != 0.0f) { const float t = -s.a / j; if (t > 0.0f && t < T) judge(step(s, j, t)); }
            // v crosses 0 inside the phase: a p extremum. v(t) = v + a t + j t^2 / 2.
            if (j != 0.0f) {
                const float disc = s.a * s.a - 2.0f * j * s.v;
                if (disc >= 0.0f) {
                    const float r = std::sqrt(disc);
                    for (const float t : {(-s.a - r) / j, (-s.a + r) / j}) if (t > 0.0f && t < T) judge(step(s, j, t));
                }
            } else if (s.a != 0.0f) { const float t = -s.v / s.a; if (t > 0.0f && t < T) judge(step(s, 0.0f, t)); }
            s = step(s, j, T);
            judge(s);
        }
        return worst;
    }
};

}  // namespace kinetic2
