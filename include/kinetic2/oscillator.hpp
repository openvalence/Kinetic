// kinetic2/oscillator.hpp -- the oscillation modulator (Valence RFC-103): an
// additive stage after the planner that yields first under the ceilings
// Constraints:
// - Additive on the planned state, never on the timeline: the engine knows
//   nothing about it, the caller sums (the hub's arbiter, the wasm shim).
// - Yields first: the amplitude actually rendered is the largest that keeps
//   planned + oscillation inside every ceiling at this instant, never more
//   than asked; a full-speed stroke sheds the oscillation to nothing and
//   `amplitudeEffective()` says so. The planned motion is never touched.
// - Every shape is band-limited by construction: square edges and saw
//   flybacks are quintic ramps whose duration is set by the ceilings, so the
//   stage can never be the thing that exceeds jmax.
// - Dwells hold the crest and the trough for a share of the period, which
//   lengthens the period (RFC-095's additive rule, applied to a cycle).
// - Deterministic: phase advances from the caller's clock in whole
//   microseconds; no state but the phase and the last effective amplitude.
#pragma once

#include <cmath>
#include <cstdint>

#include "types.hpp"

namespace kinetic2 {

enum class OscShape : uint8_t { Sine = 0, Square = 1, Saw = 2, SawReverse = 3 };

struct OscParams {
    bool     enabled      = false;
    float    frequency    = 1.0f;   // Hz, 0 .. osc_max_hz (the hub's declared limit)
    float    amplitude    = 0.0f;   // window units, peak (half the swing)
    OscShape shape        = OscShape::Sine;
    float    dwell_crest  = 0.0f;   // share of one period held at the crest
    float    dwell_trough = 0.0f;   // share of one period held at the trough
};

class Oscillator {
public:
    static constexpr float kPi = 3.14159265358979f;

    const OscParams& params() const { return _p; }
    void set(const OscParams& p) {
        _p = p;
        if (!p.enabled) { _phase = 0.0f; _eff = 0.0f; }
        measurePeaks();
    }

    // The amplitude the last apply() rendered (RFC-103 `osc.amplitude_effective`).
    float amplitudeEffective() const { return _eff; }
    bool  active() const { return _p.enabled && _eff > 0.0f; }

    // The planned state plus the oscillation at now_us, clamped by the
    // ceilings. The window is the caller's clamp (the oscillation never
    // leaves it: the sum is limited to [lo, hi] by amplitude, not by cutting).
    State apply(const State& planned, uint64_t now_us, const Limits& L, float lo, float hi) {
        if (!_p.enabled || !(_p.frequency > 0.0f) || !(_p.amplitude > 0.0f)) { _eff = 0.0f; _last_us = now_us; return planned; }
        // Phase advances by the elapsed clock; the first call seeds it.
        if (_seeded) { const float dt = float(now_us - _last_us) * 1e-6f; _phase = std::fmod(_phase + dt / period(), 1.0f); }
        _seeded = true; _last_us = now_us;

        // Unit waveform and its derivatives at this phase, for amplitude 1.
        float u, du, ddu, dddu;
        shape(_phase, u, du, ddu, dddu);

        // Yield first: the headroom under each ceiling after the planned
        // motion, divided by the oscillation's own peak demand at amplitude 1,
        // bounds the amplitude; the window bounds it too.
        float A = _p.amplitude;
        // The moving part of a cycle takes 1 / frequency whatever the dwells
        // add, so its angular rate is the plain one; the dwells only stretch
        // the period.
        const float w = 2.0f * kPi * _p.frequency;
        const float pv = std::fmax(0.0f, L.vmax - std::fabs(planned.v)), pa = std::fmax(0.0f, L.amax - std::fabs(planned.a));
        const float pj = L.jmax;   // the planner's jerk is not in State; the stage keeps its own jerk under jmax
        const float dv = _pk_dv * w, da = _pk_da * w * w, dj = _pk_dj * w * w * w;
        if (dv > 0.0f) A = std::fmin(A, pv / dv);
        if (da > 0.0f) A = std::fmin(A, pa / da);
        if (dj > 0.0f) A = std::fmin(A, pj / dj);
        A = std::fmin(A, std::fmin(planned.p - lo, hi - planned.p));
        A = std::fmax(0.0f, A);
        _eff = A;

        State s = planned;
        s.p += A * u;
        s.v += A * du * w;
        s.a += A * ddu * w * w;
        (void)dddu;
        return s;
    }

private:
    // One period in seconds, dwells included.
    float period() const { return periodShare() / _p.frequency; }
    // The period as a multiple of the undwelled one: 1 + both dwell shares.
    // A saw has no rest at its extremes to hold (its ramp arrives moving), so
    // the saw shapes ignore the dwells: a documented constraint, not a bug.
    bool dwells() const { return _p.shape != OscShape::Saw && _p.shape != OscShape::SawReverse && (_p.dwell_crest > 0.0f || _p.dwell_trough > 0.0f); }
    float periodShare() const { return dwells() ? 1.0f + std::fmax(0.0f, _p.dwell_crest) + std::fmax(0.0f, _p.dwell_trough) : 1.0f; }

    // Peak |du|, |ddu|, |dddu| of the unit shape per unit omega, measured on
    // a fine grid of one moving cycle when the shape is set: the analytic
    // derivatives and, for the jerk, also the finite difference of ddu, so a
    // junction between pieces can never hide a step the ceilings would see.
    void measurePeaks() {
        _pk_dv = _pk_da = _pk_dj = 0.0f;
        constexpr int N = 4000;
        float u, du, ddu, dddu, ddu_prev = 0.0f;
        const float m = 1.0f / periodShare();
        for (int i = 0; i <= N; ++i) {
            const float ph = float(i) / float(N) * 0.999999f;   // the whole period, dwells included
            shape(ph, u, du, ddu, dddu);
            _pk_dv = std::fmax(_pk_dv, std::fabs(du));
            _pk_da = std::fmax(_pk_da, std::fabs(ddu));
            _pk_dj = std::fmax(_pk_dj, std::fabs(dddu));
            if (i) _pk_dj = std::fmax(_pk_dj, std::fabs(ddu - ddu_prev) / (2.0f * kPi * m / float(N)));
            ddu_prev = ddu;
        }
    }

    // A quintic Hermite on x in 0..1 from (p0, m0, k0) to (p1, m1, k1): the
    // value and three derivatives with respect to x.
    static void hermite5(float x, float p0, float m0, float k0, float p1, float m1, float k1,
                         float& v, float& dv, float& ddv, float& dddv) {
        x = std::fmin(1.0f, std::fmax(0.0f, x));
        const float c0 = p0, c1 = m0, c2 = 0.5f * k0;
        const float c3 = 10.0f * (p1 - p0) - 6.0f * m0 - 4.0f * m1 - 1.5f * k0 + 0.5f * k1;
        const float c4 = -15.0f * (p1 - p0) + 8.0f * m0 + 7.0f * m1 + 1.5f * k0 - k1;
        const float c5 = 6.0f * (p1 - p0) - 3.0f * (m0 + m1) - 0.5f * (k0 - k1);
        const float x2 = x * x, x3 = x2 * x, x4 = x3 * x, x5 = x4 * x;
        v = c0 + c1 * x + c2 * x2 + c3 * x3 + c4 * x4 + c5 * x5;
        dv = c1 + 2.0f * c2 * x + 3.0f * c3 * x2 + 4.0f * c4 * x3 + 5.0f * c5 * x4;
        ddv = 2.0f * c2 + 6.0f * c3 * x + 12.0f * c4 * x2 + 20.0f * c5 * x3;
        dddv = 6.0f * c3 + 24.0f * c4 * x + 60.0f * c5 * x2;
    }

    // Ramp share of a period for the band-limited shapes: a square edge or a
    // saw flyback takes this share of the period, as a rest-to-rest quintic.
    static constexpr float kRamp = 0.15f;

    // Quintic rest-to-rest smoothstep s(x) on 0..1 and its derivatives.
    static void smooth(float x, float& s, float& ds, float& dds, float& ddds) {
        x = std::fmin(1.0f, std::fmax(0.0f, x));
        const float x2 = x * x, x3 = x2 * x, x4 = x3 * x, x5 = x4 * x;
        s = 10.0f * x3 - 15.0f * x4 + 6.0f * x5;
        ds = 30.0f * x2 - 60.0f * x3 + 30.0f * x4;
        dds = 60.0f * x - 180.0f * x2 + 120.0f * x3;
        ddds = 60.0f - 360.0f * x + 360.0f * x2;
    }

    // u in -1..1 over one period of phase 0..1 with the dwells folded in:
    // the moving part occupies share m = 1 / periodShare of the period, the
    // crest dwell follows the crest, the trough dwell follows the trough.
    // Derivatives are per unit of the UNDWELLED phase angle (2 pi per moving
    // period), which is what apply() scales by omega.
    void shape(float ph, float& u, float& du, float& ddu, float& dddu) const {
        const float m = 1.0f / periodShare();
        const float dc = dwells() ? std::fmax(0.0f, _p.dwell_crest) / periodShare() : 0.0f;
        // Phase layout: [0, m/2) rise to the crest, [m/2, m/2 + dc) crest dwell,
        // then the fall, then the trough dwell to 1.
        float t;   // 0..1 within the moving cycle, rise then fall
        if (ph < 0.5f * m) t = ph / m;
        else if (ph < 0.5f * m + dc) { u = 1.0f; du = ddu = dddu = 0.0f; return; }
        else if (ph < m + dc) t = (ph - dc) / m;
        else { u = -1.0f; du = ddu = dddu = 0.0f; return; }
        const float ang = 2.0f * kPi * t;
        switch (_p.shape) {
            case OscShape::Sine: {
                if (dwells()) {
                    // A held extreme must be reached at rest in acceleration, which a
                    // sine never is: with a dwell the halves are rest-to-rest quintics.
                    float sv, ds, dds, ddds;
                    const float k = 1.0f / kPi;   // a half cycle is pi of angle
                    if (t < 0.5f) { smooth(2.0f * t, sv, ds, dds, ddds); u = -1.0f + 2.0f * sv; du = 2.0f * ds * k; ddu = 2.0f * dds * k * k; dddu = 2.0f * ddds * k * k * k; }
                    else { smooth(2.0f * t - 1.0f, sv, ds, dds, ddds); u = 1.0f - 2.0f * sv; du = -2.0f * ds * k; ddu = -2.0f * dds * k * k; dddu = -2.0f * ddds * k * k * k; }
                    return;
                }
                // -cos so the cycle starts at the trough and rises to the crest at t = 0.5.
                u = -std::cos(ang); du = std::sin(ang); ddu = std::cos(ang); dddu = -std::sin(ang);
                return;
            }
            case OscShape::Square: {
                // Edges at t = 0 (rising) and t = 0.5 (falling), each a quintic of share kRamp.
                float s, ds, dds, ddds;
                const float r = kRamp;
                if (t < r) { smooth(t / r, s, ds, dds, ddds); u = -1.0f + 2.0f * s; const float k = 1.0f / (2.0f * kPi * r); du = 2.0f * ds * k; ddu = 2.0f * dds * k * k; dddu = 2.0f * ddds * k * k * k; }
                else if (t < 0.5f) { u = 1.0f; du = ddu = dddu = 0.0f; }
                else if (t < 0.5f + r) { smooth((t - 0.5f) / r, s, ds, dds, ddds); u = 1.0f - 2.0f * s; const float k = 1.0f / (2.0f * kPi * r); du = -2.0f * ds * k; ddu = -2.0f * dds * k * k; dddu = -2.0f * ddds * k * k * k; }
                else { u = -1.0f; du = ddu = dddu = 0.0f; }
                return;
            }
            case OscShape::Saw:
            case OscShape::SawReverse: {
                // Saw: a linear rise over (1 - r) of the cycle, then a quintic flyback of share r.
                // SawReverse mirrors it in time.
                const float r = kRamp;
                const float tt = _p.shape == OscShape::Saw ? t : 1.0f - t;
                const float sgn = _p.shape == OscShape::Saw ? 1.0f : -1.0f;
                const float slope = 2.0f / (1.0f - r);   // du/dt on the linear part
                if (tt < 1.0f - r) { u = -1.0f + 2.0f * tt / (1.0f - r); du = sgn * slope / (2.0f * kPi); ddu = dddu = 0.0f; }
                else {
                    // The flyback keeps the ramp's slope at both ends, so the
                    // velocity is continuous and only the quintic's own
                    // acceleration and jerk appear.
                    float v, dv, ddv, dddv;
                    hermite5((tt - (1.0f - r)) / r, 1.0f, slope * r, 0.0f, -1.0f, slope * r, 0.0f, v, dv, ddv, dddv);
                    const float k = 1.0f / (2.0f * kPi * r);
                    u = v; du = sgn * dv * k; ddu = ddv * k * k; dddu = sgn * dddv * k * k * k;
                }
                return;
            }
        }
        u = du = ddu = dddu = 0.0f;
    }

    OscParams _p{};
    float     _pk_dv = 1.0f, _pk_da = 1.0f, _pk_dj = 1.0f;
    float     _phase = 0.0f;
    float     _eff = 0.0f;
    uint64_t  _last_us = 0;
    bool      _seeded = false;
};

}  // namespace kinetic2
