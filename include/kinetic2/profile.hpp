// kinetic2/profile.hpp -- constant-jerk phase profiles: the brake and the
// point-to-point move to rest (a HARD knot)
// Constraints:
// - A profile is up to twelve phases of constant jerk from a start state; the
//   state is integrated in closed form per phase, never stepped.
// - brake() is the fastest legal stop from any (p, v, a) under amax and jmax:
//   ramp to the decel ceiling, hold it, ramp out, so v and a reach 0 together.
//   When the entry acceleration already overshoots the stop (v would cross 0
//   before a can ramp to 0), a first phase ramps a to 0 and the stop is
//   planned again from there, the one case that needs the fourth phase.
// - brake() and point() start from an acceleration inside amax (startOf()):
//   an entry past it (a piece over a ceiling, a ceiling lowered under the
//   motion) starts at amax. Unwound at jmax instead, it gained a^2/2J of
//   speed: 38 window units/s from 3892 at the factory set (kin-554).
// - Jerk never exceeds jmax by construction; |a| peaks at a phase boundary;
//   |v| and p peak at a boundary or where a or v cross 0 inside a phase. The
//   referee here checks exactly those points.
#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "types.hpp"

namespace kinetic2 {

struct Profile {
    // point(): 12, see there. A chase run (solver.hpp chaseRun) is converge()
    // (3 at most) and point() from its end, which has no acceleration (10).
    static constexpr int kMaxPhases = 13;
    uint64_t start_us = 0;
    State    s0{};
    int      n = 0;
    float    dt[kMaxPhases] = {};   // seconds
    float    jerk[kMaxPhases] = {};
    bool     ends_at_rest = true;   // a brake; false for a corner ramp, which keeps moving

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
        if (ends_at_rest) { s.v = 0.0f; s.a = 0.0f; }   // a brake ends at rest by construction; pin it
        return s;
    }
    State at(uint64_t t_us) const {
        return atSeconds(t_us <= start_us ? 0.0f : float(t_us - start_us) * 1e-6f);
    }
    State end() const { return atSeconds(duration() + 1.0f); }

    // s with its acceleration held to amax: where every profile starts.
    static State startOf(State s, const Limits& L) {
        if (std::fabs(s.a) > L.amax) s.a = std::copysign(L.amax, s.a);
        return s;
    }

    // The fastest legal stop from s under L. A state already at rest gives an
    // empty profile.
    static Profile brake(const State& entry, uint64_t start_us, const Limits& L) {
        const State s = startOf(entry, L);
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

    // The speed s reaches when its acceleration ramps out at jmax at once.
    static float freeSpeed(const State& s, const Limits& L) { return s.v + s.a * std::fabs(s.a) / (2.0f * L.jmax); }

    // s within L at the bar a chase run judges at: its speed, its acceleration
    // and freeSpeed(). Past the last, no jerk-limited move keeps the speed
    // inside vmax.
    static bool inside(const State& s, const Limits& L) {
        const float bar = 1.0001f;
        return std::fabs(s.v) <= L.vmax * bar && std::fabs(s.a) <= L.amax * bar && std::fabs(freeSpeed(s, L)) <= L.vmax * bar;
    }

    // The fastest return inside vmax and amax from a state outside them (a
    // ceiling lowered under the motion): the velocity changes to the nearest
    // one vmax allows, times share, at cap's jerk and up to its acceleration
    // (never under amax: cap is the set the motion was planned under), ending
    // with none; an acceleration alone over amax ramps out. An entry
    // acceleration past cap's starts at it (startOf, kin-554). Empty when s
    // is inside() and share is 1.
    static Profile converge(const State& entry, uint64_t start_us, const Limits& L, const Limits& cap, float share = 1.0f) {
        const Limits C{L.vmax, std::fmax(L.amax, cap.amax), cap.jmax};
        const State s = startOf(entry, C);
        Profile pr; pr.start_us = start_us; pr.s0 = s; pr.ends_at_rest = false;
        if (share == 1.0f && inside(s, L)) return pr;
        const float v_free = freeSpeed(s, C);
        const float v_to = share * std::fmax(-L.vmax, std::fmin(L.vmax, v_free));
        const Profile d = brake(State{0.0f, s.v - v_to, s.a}, 0, C);
        for (int i = 0; i < d.n; ++i) {
            if (pr.n && pr.jerk[pr.n - 1] == d.jerk[i]) pr.dt[pr.n - 1] += d.dt[i];
            else (void)pr.add(d.dt[i], d.jerk[i]);
        }
        return pr;
    }

    // Appends one phase; false when the profile is full.
    bool add(float t, float j) {
        if (!(t > 0.0f)) return true;
        if (n >= kMaxPhases) return false;
        dt[n] = t; jerk[n] = j; ++n;
        return true;
    }

    // The fastest motion from any state s to rest at `target` under L, taking
    // at least `at_least` seconds; `fastest` gets the least time it needs.
    // - Moving toward the target, or away from it: one jerk-limited change of
    //   velocity to the cruise speed (through zero, toward the target, when
    //   moving away), the cruise, then brake() landing on the target. The
    //   cruise speed is the highest whose run fits the distance, by bisection
    //   on closed-form distances (no referee), 24 halvings: float resolution.
    // - Unable to stop short of the target (already past it, or too fast):
    //   brake() to rest past it, then the same from rest back to it.
    // - Time to spare: from rest the carriage holds, then launches, landing
    //   exactly at at_least; moving, the cruise is slowed to take at_least,
    //   and when no cruise takes that long it lands early and rests there.
    // At most 12 phases: a 4-phase brake, a hold, a 3-phase launch from rest,
    // the cruise and a 3-phase stop. A full profile returns n = -1.
    static Profile point(const State& entry, float target, uint64_t start_us, const Limits& L, float at_least = 0.0f,
                         float* fastest = nullptr) {
        const State s = startOf(entry, L);
        Profile pr; pr.start_us = start_us; pr.s0 = s;
        float need = 0.0f;
        if (L.vmax > 0.0f && L.amax > 0.0f && L.jmax > 0.0f) {
            const Profile stop = brake(s, 0, L);
            const State se = stop.n ? stop.atSeconds(stop.duration()) : State{s.p, 0.0f, 0.0f};
            const float dir = target >= s.p ? 1.0f : -1.0f;
            const bool passes = dir * (se.p - s.p) > dir * (target - s.p) + 1e-7f;
            bool ok = true;
            if (passes) {
                for (int i = 0; i < stop.n; ++i) ok = ok && pr.add(stop.dt[i], stop.jerk[i]);
                const float t0 = stop.duration();
                float rest = 0.0f;
                ok = ok && reach(pr, State{se.p, 0.0f, 0.0f}, target, L, at_least - t0, rest);
                need = t0 + rest;
            } else {
                ok = reach(pr, s, target, L, at_least, need);
            }
            if (!ok) pr.n = -1;
        }
        if (fastest) *fastest = need;
        return pr;
    }

    // point() without the overshoot: the fastest stop from s does not pass
    // the target. Appends the phases; false when they do not fit.
    static bool reach(Profile& pr, const State& s, float target, const Limits& L, float at_least, float& fastest) {
        const float dir = target >= s.p ? 1.0f : -1.0f;
        const float u = dir * s.v, b = dir * s.a, D = dir * (target - s.p);
        const bool at_rest = s.v == 0.0f && s.a == 0.0f;
        fastest = 0.0f;
        if (at_rest && D <= 1e-7f) return pr.add(at_least, 0.0f);
        // Displacement of a profile from p = 0; the change of velocity to w is
        // the brake of the velocity relative to w, carried along at w.
        auto disp = [](const Profile& q) { return q.n ? q.atSeconds(q.duration()).p : 0.0f; };
        auto run = [&](float w, Profile& tr, Profile& st) {
            tr = brake(State{0.0f, u - w, b}, 0, L);
            st = brake(State{0.0f, w, 0.0f}, 0, L);
            return disp(tr) + w * tr.duration() + disp(st);
        };
        Profile tr, st;
        auto cruiseFor = [&](float w, float f) { return w > 1e-6f && f < D ? (D - f) / w : 0.0f; };
        float w = L.vmax;
        float f = run(w, tr, st);
        if (f > D) {
            float lo = 0.0f, hi = L.vmax;
            for (int it = 0; it < 24; ++it) {
                const float mid = 0.5f * (lo + hi);
                if (run(mid, tr, st) <= D) lo = mid; else hi = mid;
            }
            w = lo;
            f = run(w, tr, st);
        }
        float tc = cruiseFor(w, f);
        fastest = tr.duration() + tc + st.duration();
        float hold = 0.0f;
        if (at_least > fastest) {
            if (at_rest) hold = at_least - fastest;
            else {
                // The highest cruise speed that still takes at_least.
                float lo = 0.0f, hi = w;
                for (int it = 0; it < 24; ++it) {
                    const float mid = 0.5f * (lo + hi);
                    const float fm = run(mid, tr, st);
                    if (fm <= D && tr.duration() + cruiseFor(mid, fm) + st.duration() >= at_least) lo = mid; else hi = mid;
                }
                if (lo > 0.0f) { w = lo; f = run(w, tr, st); tc = cruiseFor(w, f); }
                else f = run(w, tr, st);
            }
        }
        bool ok = pr.add(hold, 0.0f);
        for (int i = 0; i < tr.n; ++i) ok = ok && pr.add(tr.dt[i], dir * tr.jerk[i]);
        ok = ok && pr.add(tc, 0.0f);
        for (int i = 0; i < st.n; ++i) ok = ok && pr.add(st.dt[i], dir * st.jerk[i]);
        return ok;
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
