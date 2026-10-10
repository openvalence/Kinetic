// kinetic2/oscillator.hpp -- the oscillator stage (Valence RFC-103, SPEC 9.7): a
// periodic offset on the planned position that yields first under the ceilings
// and never leaves the window
// Constraints:
// - Additive on the plan, never on the timeline: the engine knows nothing
//   about it. The caller hands render() the planned positions on a uniform
//   grid and sums what it returns (the hub's strip, the wasm shim).
// - The ceilings hold on the SUM'S POSITIONS, the envelope's own derivatives
//   included. The amplitude rendered is the target (the least the asked
//   amplitude, the window and every ceiling's headroom allow over the next
//   fade, planned jerk included) smoothed by a quadratic B-spline one fade
//   long, and the budget factors (kappa) pay for the smoothing: the
//   oscillation is shed before the planned motion that needs the headroom
//   arrives. Proven while the plan and the drive inside the look-ahead do not
//   change; a plan changed with less notice than a fade (a PAUSE brake, a knot
//   landing inside the look-ahead) can exceed a ceiling until the fade ends.
// - The window bounds the amplitude and never cuts the waveform: every shape
//   is normalized to a peak of exactly 1.
// - Fixed (the default): frequency, shape and dwells change only at rest: a
//   change fades the oscillation out, latches, and fades it in from the
//   trough. Amplitude changes ride the envelope.
// - Driven (OscParams::driven; SPEC 9.7 drives, the osc.drive stream): a plain
//   sine whose frequency and amplitude follow drive() while it runs. The
//   frequency is the drive's steps through the envelope's three boxes,
//   integrated into the phase, never jumped, its slew paid for in the budget;
//   the amplitude rides the envelope. A point lands no nearer than
//   driveLeadUs() past the head, so the look-ahead never changes under the
//   budget. Nothing is asked from kDriveQuietUs past the last point: it fades
//   to rest. Fixed to driven and back goes through rest.
// - Fixed storage, no allocation: the caller hosts the object (about 7 KB).
//
// The hub, driven (val-o9r): set() with enabled and driven. Per osc.drive
// sample, drive(t_base + t_off, hz, amplitude), each parameter mapped through
// its bounds (SPEC 8.11), a fixed one passing its field's value. The speed and
// position drives: drive(t0 + driveLeadUs(step), ...) from the plan at that
// instant, every few ticks (kDriveMax). render() per tick as when fixed.
// driveLeadUs() is the osc-drive grant's schedule_latency_us (SPEC 5.4): a
// player that leads by it lands on its stamps.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace kinetic2 {

// The registry's osc_shapes numbers (Valence registry.yaml): never renumber.
enum class OscShape : uint8_t { Sine = 0, Square = 1, Saw = 2, SawReverse = 3 };

struct OscParams {
    bool     enabled      = false;
    float    frequency    = 1.0f;   // Hz: the moving part of one cycle takes 1 / frequency
    float    amplitude    = 0.0f;   // window units, peak (half the swing)
    OscShape shape        = OscShape::Sine;
    float    dwell_crest  = 0.0f;   // share of the moving cycle held at the crest
    float    dwell_trough = 0.0f;   // share of the moving cycle held at the trough
    bool     driven       = false;  // frequency and amplitude from drive(), a plain sine: the four above unused
};

// MaxOut: the most samples one render() returns. MaxFade: the longest fade,
// in grid steps (kFadeMaxUs at the caller's step).
template <size_t MaxOut = 128, size_t MaxFade = 150>
class Oscillator {
    static_assert(MaxFade >= 3, "a fade is three boxes of at least one step");

public:
    static constexpr float kPi = 3.14159265358979f;
    // A fade lasts three periods, capped here: the longer the fade, the less
    // its derivatives cost and the further ahead the plan must be known.
    static constexpr uint32_t kFadeMaxUs = 150000;

    // Planned positions render() reads before t0 and past its look-ahead,
    // for the finite differences.
    static constexpr size_t kPlanEdge = 3;
    // Grid steps of plan render() reads past its last output: plan[] holds
    // n + lookahead(step_us) + 2 * kPlanEdge positions.
    static constexpr size_t lookahead(uint32_t step_us) {
        return std::min<size_t>(MaxFade / 3, kFadeMaxUs / 3 / step_us) * 3;
    }

    // Driven: a point lands no nearer than this past the head (the fade and
    // the stencils it reads), with one render() per step.
    static constexpr uint32_t driveLeadUs(uint32_t step_us) {
        return uint32_t(3 * driveBox(step_us) + 2 * kPlanEdge) * step_us;
    }
    // Driven: nothing is asked this long past the last point (SPEC
    // stream_quiet_release_ms).
    static constexpr uint64_t kDriveQuietUs = 500000;
    // Driven: the points held, from the one the smoothing reaches behind the
    // head to the last stamp ahead: about 120 Hz under the 250 ms lead cap.
    static constexpr size_t kDriveMax = 64;

    const OscParams& params() const { return _want; }
    // Takes effect at the next render(): amplitude through the envelope,
    // frequency, shape, dwells and driven through a fade to rest.
    void set(const OscParams& p) {
        _want = p;
        if (!(_want.frequency > 0.0f)) _want.frequency = 0.0f;
        if (!(_want.amplitude > 0.0f)) _want.amplitude = 0.0f;
        if (!(_want.dwell_crest > 0.0f)) _want.dwell_crest = 0.0f;
        if (!(_want.dwell_trough > 0.0f)) _want.dwell_trough = 0.0f;
    }
    // Driven: from t_us on the drive asks hz and amplitude (window units,
    // peak). A point nearer than driveLeadUs() past the head lands there, and
    // replaces every point at or after its time. False when kDriveMax points
    // are held: the point is dropped.
    bool drive(uint64_t t_us, float hz, float amplitude) {
        if (_seeded) t_us = std::max<uint64_t>(t_us, _head_us + driveLeadUs(_step_us));
        while (_drv_n > 0 && _drv[_drv_n - 1].t_us >= t_us) --_drv_n;
        if (_drv_n == kDriveMax) return false;
        _drv[_drv_n++] = DrivePoint{t_us, hz > 0.0f ? hz : 0.0f, amplitude > 0.0f ? amplitude : 0.0f};
        return true;
    }
    // Stops at once, phase at the trough: for a caller that stopped the motion
    // itself (an e-stop, a reset of the plan). Parameters and drive untouched.
    void reset() {
        _hist.fill(0.0f);
        _n_pred = 0;
        _n_dph = 0;
        _phase = 0.0f;
        _eff = 0.0f;
        _shaped = false;
    }
    // The window's span moved by `k` (old over new): the envelope keeps its
    // physical size, and the target moves it from there.
    void rescale(float k) {
        for (float& x : _hist) x *= k;
        for (size_t i = 0; i < _n_pred; ++i) _pred[i] *= k;
        _eff *= k;
    }

    // The amplitude the last render() gave its first sample (RFC-103
    // `osc.amplitude_effective`), window units.
    float amplitudeEffective() const { return _eff; }
    bool  active() const { return _eff > 0.0f; }
    // The ceilings or the window cut the target below the asked amplitude.
    bool  shaped() const { return _shaped; }
    // Disabled and faded out: render() would return zeros.
    bool  idle() const { return !_want.enabled && quiet(); }

    // out[i] = the oscillation at t0 + i * step_us, window units, i in [0, n).
    // plan[j] = the planned position at t0 + (j - kPlanEdge) * step_us. L: the ceilings
    // the sum keeps; lo, hi: the window. hold: shed it (PAUSE): it fades out,
    // and its phase restarts at the trough once it has. t0 advances by whole
    // steps between calls; the same t0 with the same plan and drive renders
    // the same.
    void render(uint64_t t0_us, uint32_t step_us, const float* plan, size_t n, const Limits& L, float lo,
                float hi, bool hold, float* out) {
        n = std::min(n, MaxOut);
        const float dt = float(step_us) * 1e-6f;
        // Advance the head: the targets the last call predicted are history
        // now, one per whole step; the phase by the exact time elapsed, or
        // driven, by the phase steps the last call rendered.
        if (_seeded && t0_us > _head_us) {
            const uint64_t k = (t0_us - _head_us + step_us / 2) / step_us;
            for (uint64_t i = 0; i < k && i < MaxFade; ++i) push(i < _n_pred ? _pred[size_t(i)] : 0.0f);
            if (_latched && _on.driven) {
                if (_n_dph > 0) {
                    const size_t m = size_t(std::min<uint64_t>(k, _n_dph));
                    for (size_t i = 0; i < m; ++i) _phase = frac(_phase + _dph[i]);
                    const double rest = double(k - m) * double(_dph[_n_dph - 1]);
                    _phase = frac(_phase + float(rest - std::floor(rest)));
                }
            } else if (_latched && _on.frequency > 0.0f) {
                const double adv = double(t0_us - _head_us) * 1e-6 * double(_on.frequency) / double(periodShare());
                _phase = frac(_phase + float(adv - std::floor(adv)));
            }
        }
        _seeded = true;
        _head_us = t0_us;
        _step_us = step_us;
        prune(t0_us, step_us);

        const bool zero = hold || !_want.enabled || (!_want.driven && !(_want.frequency > 0.0f));
        if (quiet()) {
            // At rest: a new frequency, shape or dwell latches, and a shed
            // oscillation starts its next period at the trough.
            if (!_latched || !sameShape(_on, _want)) {
                _on = _want;
                _latched = true;
                measurePeaks();
                _phase = 0.0f;
            }
            if (zero) _phase = 0.0f;
        }
        const bool fading = zero || !sameShape(_on, _want);
        const bool drv = _on.driven;

        const size_t h = drv ? driveBox(step_us) : boxSteps(step_us);
        const size_t ln = 3 * h;   // the look-ahead: one fade
        const size_t past = 3 * h - 3;
        // Driven: fq[j] = the frequency at t0 + j * step for j in [-G, n + ln +
        // kPlanEdge), the drive's steps through the envelope's three boxes, and
        // _b[j] = the amplitude asked, j in [0, n + ln).
        const size_t G = ln + kPlanEdge;
        const float* fq = _fq.data() + G;
        if (drv) {
            const size_t m = G + n + ln + kPlanEdge;
            stairs(t0_us, step_us, G, m, n + ln, 0.25f / dt);
            if (h > 1) {
                box(_fq.data(), m, h, h - 1);
                box(_fq.data(), m, h, 2 * h - 2);
                box(_fq.data(), m, h, 3 * h - 3);
            }
            // At rest with nothing asked: the next period starts at the trough.
            if (quiet() && _b[0] == 0.0f) _phase = 0.0f;
        }
        const float ask = drv ? _b[0] : _want.amplitude;
        // s: the committed targets of the fade behind the head, then T, the
        // targets of the outputs, which the boxes below smooth into the
        // envelope in place.
        float* s = _s.data();
        for (size_t i = 0; i < past; ++i) s[i] = _hist[(_hist_at + MaxFade - past + i) % MaxFade];
        // B: the largest amplitude each instant allows, from the planned
        // state by finite differences on the grid.
        if (fading) {
            std::fill(s + past, s + past + n, 0.0f);
        } else {
            // The plan's headroom under each ceiling at sample j, its
            // derivatives as the grid's own differences measure them: the
            // largest of every stencil that touches sample j.
            const float i1 = 1.0f / dt, i2 = 1.0f / (dt * dt), i3 = 1.0f / (dt * dt * dt);
            auto d2 = [](const float* q) { return std::fabs(q[1] - 2.0f * q[0] + q[-1]); };
            auto d3 = [](const float* q) { return std::fabs(q[0] - 3.0f * q[-1] + 3.0f * q[-2] - q[-3]); };
            auto room = [&](size_t j, float& hv, float& ha, float& hj) {
                const float* p = plan + j + kPlanEdge;
                const float v = std::fmax(std::fabs(p[1] - p[0]), std::fabs(p[0] - p[-1])) * i1;
                const float a = std::fmax(d2(p - 1), std::fmax(d2(p), d2(p + 1))) * i2;
                const float jk = std::fmax(std::fmax(d3(p), d3(p + 1)), std::fmax(d3(p + 2), d3(p + 3))) * i3;
                hv = std::fmax(0.0f, L.vmax - std::fabs(v));
                ha = std::fmax(0.0f, L.amax - std::fabs(a));
                hj = std::fmax(0.0f, L.jmax - std::fabs(jk));
            };
            if (drv) {
                // The sine's budget with the frequency moving: the kappa terms
                // below with the frequency's own slew (w1, w2) added, 1 / F
                // the fade's inverse, and the same 5 % margin.
                const float iF = 1.0f / (float(ln) * dt), tw = 2.0f * kPi;
                for (size_t j = 0; j < n + ln; ++j) {
                    float hv, ha, hj;
                    room(j, hv, ha, hj);
                    const float* q = fq + j;
                    const float w = tw * q[0];
                    const float w1 = tw * std::fmax(std::fabs(q[1] - q[0]), std::fabs(q[0] - q[-1])) * i1;
                    const float w2 = tw * std::fmax(d2(q - 1), std::fmax(d2(q), d2(q + 1))) * i2;
                    const float p0 = plan[j + kPlanEdge];
                    float b = std::fmin(_b[j], std::fmin(p0 - lo, hi - p0));
                    b = std::fmin(b, 0.95f * hv / (w + 2.25f * iF));
                    b = std::fmin(b, 0.95f * ha / (w * w + (4.5f * w + 18.0f * iF) * iF + w1));
                    b = std::fmin(b, 0.95f * hj / (w * w * w + (6.75f * w * w + (54.0f * w + 108.0f * iF) * iF) * iF +
                                                   (6.75f * iF + 3.0f * w) * w1 + w2));
                    _b[j] = std::fmax(0.0f, b);
                }
            } else {
                const float w = 2.0f * kPi * _on.frequency, dw = float(3 * h) * dt * w;
                // The smoothing's cost per ceiling (oscillator.hpp header), and a
                // 5 % margin for the grid.
                const float kv = 0.95f / (1.0f + 2.25f / (dw * _pk_dv));
                const float ka = 0.95f / (1.0f + (4.5f * _pk_dv / dw + 18.0f / (dw * dw)) / _pk_da);
                const float kj = 0.95f / (1.0f + (6.75f * _pk_da / dw + 54.0f * _pk_dv / (dw * dw) +
                                                 108.0f / (dw * dw * dw)) / _pk_dj);
                const float cv = kv / (_pk_dv * w), ca = ka / (_pk_da * w * w), cj = kj / (_pk_dj * w * w * w);
                for (size_t j = 0; j < n + ln; ++j) {
                    float hv, ha, hj;
                    room(j, hv, ha, hj);
                    const float p0 = plan[j + kPlanEdge];
                    float b = std::fmin(_want.amplitude, std::fmin(p0 - lo, hi - p0));
                    b = std::fmin(b, cv * hv);
                    b = std::fmin(b, ca * ha);
                    b = std::fmin(b, cj * hj);
                    _b[j] = std::fmax(0.0f, b);
                }
            }
            // T: the least B over the fade ahead (a monotonic deque, front
            // the least, walked backward).
            size_t qh = 0, qt = 0;
            for (size_t j = n + ln; j-- > 0;) {
                while (qt > qh && _b[_q[qt - 1]] >= _b[j]) --qt;
                _q[qt++] = uint16_t(j);
                if (_q[qh] > j + ln) ++qh;
                if (j < n) s[past + j] = _b[_q[qh]];
            }
        }
        // The targets next call commits.
        for (size_t j = 0; j < n; ++j) _pred[j] = s[past + j];
        _n_pred = n;
        _shaped = !fading && n > 0 && s[past] < ask * 0.999f;
        const size_t m = past + n;
        if (h > 1 && m >= h) {
            box(s, m, h, h - 1);
            box(s, m, h, 2 * h - 2);
            box(s, m, h, 3 * h - 3);
        }
        _eff = n > 0 ? std::fmax(0.0f, s[past]) : 0.0f;
        if (drv) {
            // The sine by rotation, each step at its own sample's frequency.
            float c = std::cos(2.0f * kPi * _phase), sn = std::sin(2.0f * kPi * _phase);
            for (size_t j = 0; j < n; ++j) {
                const float e = std::fmax(0.0f, s[past + j]);
                out[j] = e == 0.0f ? 0.0f : -e * c;
                _dph[j] = fq[j] * dt;
                turn(c, sn, 2.0f * kPi * _dph[j]);
            }
            _n_dph = n;
            return;
        }
        _n_dph = 0;
        const float d = dphi(dt);
        // The plain sine by rotation, one cos and sin a call rather than one a
        // sample: the trig is most of a render on a core without a fast libm.
        const bool sine = _on.shape == OscShape::Sine && !dwells();
        float c = std::cos(2.0f * kPi * _phase), sn = std::sin(2.0f * kPi * _phase);
        const float cd = std::cos(2.0f * kPi * d), sd = std::sin(2.0f * kPi * d);
        for (size_t j = 0; j < n; ++j) {
            const float e = std::fmax(0.0f, s[past + j]);
            float u = -c;
            if (sine) {
                const float c1 = c * cd - sn * sd;
                sn = sn * cd + c * sd;
                c = c1;
            }
            if (e == 0.0f) { out[j] = 0.0f; continue; }
            if (!sine) {
                float du, ddu, dddu;
                shape(frac(_phase + float(j) * d), u, du, ddu, dddu);
            }
            out[j] = e * u * _scale;
        }
    }

private:
    static float frac(float x) { return x - std::floor(x); }
    static bool sameShape(const OscParams& a, const OscParams& b) {
        if (a.driven || b.driven) return a.driven == b.driven;
        return a.frequency == b.frequency && a.shape == b.shape && a.dwell_crest == b.dwell_crest &&
               a.dwell_trough == b.dwell_trough;
    }
    bool quiet() const {
        for (const float x : _hist) if (x != 0.0f) return false;
        return true;
    }
    void push(float t) {
        _hist[_hist_at] = t;
        _hist_at = (_hist_at + 1) % MaxFade;
    }
    // Phase per grid step: the moving cycle takes 1 / frequency, the dwells
    // stretch the period past it.
    float dphi(float dt) const { return _on.frequency > 0.0f ? dt * _on.frequency / periodShare() : 0.0f; }
    // One box of the fade, in steps: three periods, capped at kFadeMaxUs.
    size_t boxSteps(uint32_t step_us) const {
        const float fade = _on.frequency > 0.0f ? std::fmin(3.0f / _on.frequency, float(kFadeMaxUs) * 1e-6f)
                                                : float(kFadeMaxUs) * 1e-6f;
        const size_t h = size_t(std::lround(fade / (3.0f * float(step_us) * 1e-6f)));
        return std::clamp<size_t>(h, 1, lookahead(step_us) / 3);
    }
    // Driven: one box of the fade, the longest, whatever the frequency does.
    static constexpr size_t driveBox(uint32_t step_us) { return std::max<size_t>(1, lookahead(step_us) / 3); }
    // S[i] = the mean of S[i - h + 1 .. i] for i in [first, m), in place,
    // backward so every read is of an unwritten entry.
    static void box(float* s, size_t m, size_t h, size_t first) {
        const float inv = 1.0f / float(h);
        float sum = 0.0f;
        for (size_t k = m - h; k < m; ++k) sum += s[k];
        for (size_t i = m; i-- > first;) {
            const float x = s[i];
            s[i] = sum * inv;
            sum -= x;
            if (i >= h) sum += s[i - h];
        }
    }
    // (c, s) turned by x radians, |x| <= pi / 2: Taylor to x^13, under 1e-8.
    static void turn(float& c, float& s, float x) {
        const float x2 = x * x;
        const float cx = 1.0f - x2 / 2.0f * (1.0f - x2 / 12.0f * (1.0f - x2 / 30.0f * (1.0f - x2 / 56.0f *
                         (1.0f - x2 / 90.0f * (1.0f - x2 / 132.0f)))));
        const float sx = x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f * (1.0f - x2 / 72.0f *
                         (1.0f - x2 / 110.0f * (1.0f - x2 / 156.0f))))));
        const float c1 = c * cx - s * sx;
        s = s * cx + c * sx;
        c = c1;
    }
    // Driven: drops the points the grid no longer reads, keeping the one in
    // force where the smoothing's reach behind the head begins.
    void prune(uint64_t t0_us, uint32_t step_us) {
        const uint64_t back = uint64_t(3 * driveBox(step_us) + kPlanEdge) * step_us;
        if (t0_us <= back) return;
        size_t d = 0;
        while (d + 1 < _drv_n && _drv[d + 1].t_us <= t0_us - back) ++d;
        if (d == 0) return;
        std::copy(_drv.begin() + d, _drv.begin() + _drv_n, _drv.begin());
        _drv_n -= d;
    }
    // Driven: the drive's steps on the grid. _fq[k] = the frequency at t0 +
    // (k - back) * step for k in [0, m), at most hz_max (the rotation's
    // reach); _b[j] = the amplitude asked at t0 + j * step for j in [0, nb).
    // Before the first point the frequency is its own; nothing is asked
    // before it, kDriveQuietUs past the last, or at 0 Hz.
    void stairs(uint64_t t0_us, uint32_t step_us, size_t back, size_t m, size_t nb, float hz_max) {
        size_t p = 0;
        for (size_t k = 0; k < m; ++k) {
            const int64_t t = int64_t(t0_us) + (int64_t(k) - int64_t(back)) * int64_t(step_us);
            while (p + 1 < _drv_n && int64_t(_drv[p + 1].t_us) <= t) ++p;
            const DrivePoint* d = _drv_n > 0 ? &_drv[p] : nullptr;
            _fq[k] = d ? std::fmin(d->hz, hz_max) : 0.0f;
            if (k < back || k - back >= nb) continue;
            const bool live = d && int64_t(d->t_us) <= t && t - int64_t(d->t_us) <= int64_t(kDriveQuietUs);
            _b[k - back] = live && d->hz > 0.0f ? d->amp : 0.0f;
        }
    }

    // A saw has no rest at its extremes to hold (its ramp arrives moving), so
    // the saw shapes ignore the dwells: a constraint of the shape.
    bool dwells() const { return _on.shape != OscShape::Saw && _on.shape != OscShape::SawReverse && (_on.dwell_crest > 0.0f || _on.dwell_trough > 0.0f); }
    float periodShare() const { return dwells() ? 1.0f + _on.dwell_crest + _on.dwell_trough : 1.0f; }

    // Peaks of |u|, |du|, |ddu|, |dddu| over one period, measured on a fine
    // grid when a shape latches, then normalized so |u| peaks at exactly 1:
    // the analytic derivatives and, for the jerk, also the finite difference
    // of ddu, so a junction between pieces can never hide a step.
    void measurePeaks() {
        float pu = 0.0f, pv = 0.0f, pa = 0.0f, pj = 0.0f;
        constexpr int N = 4000;
        float u, du, ddu, dddu, ddu_prev = 0.0f;
        const float m = 1.0f / periodShare();
        for (int i = 0; i <= N; ++i) {
            const float ph = float(i) / float(N) * 0.999999f;
            shape(ph, u, du, ddu, dddu);
            pu = std::fmax(pu, std::fabs(u));
            pv = std::fmax(pv, std::fabs(du));
            pa = std::fmax(pa, std::fabs(ddu));
            pj = std::fmax(pj, std::fabs(dddu));
            if (i) pj = std::fmax(pj, std::fabs(ddu - ddu_prev) / (2.0f * kPi * m / float(N)));
            ddu_prev = ddu;
        }
        _scale = pu > 0.0f ? 1.0f / pu : 1.0f;
        _pk_dv = std::fmax(1e-6f, pv * _scale);
        _pk_da = std::fmax(1e-6f, pa * _scale);
        _pk_dj = std::fmax(1e-6f, pj * _scale);
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
    // saw flyback takes this share of the period, as a quintic.
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

    // u over one period of phase 0..1 with the dwells folded in, before
    // normalization (_scale): the moving part occupies share m = 1 /
    // periodShare of the period, the crest dwell follows the crest, the trough
    // dwell follows the trough. Derivatives are per unit of the UNDWELLED
    // phase angle (2 pi per moving cycle), which render()'s budget scales by
    // omega.
    void shape(float ph, float& u, float& du, float& ddu, float& dddu) const {
        const float m = 1.0f / periodShare();
        const float dc = dwells() ? _on.dwell_crest / periodShare() : 0.0f;
        float t;   // 0..1 within the moving cycle, rise then fall
        if (ph < 0.5f * m) t = ph / m;
        else if (ph < 0.5f * m + dc) { u = 1.0f; du = ddu = dddu = 0.0f; return; }
        else if (ph < m + dc) t = (ph - dc) / m;
        else { u = -1.0f; du = ddu = dddu = 0.0f; return; }
        const float ang = 2.0f * kPi * t;
        switch (_on.shape) {
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
                // Saw: a linear rise over (1 - r) of the cycle, then a quintic flyback of share r
                // that keeps the ramp's slope at both ends (it overshoots, which _scale absorbs).
                // SawReverse mirrors it in time.
                const float r = kRamp;
                const float tt = _on.shape == OscShape::Saw ? t : 1.0f - t;
                const float sgn = _on.shape == OscShape::Saw ? 1.0f : -1.0f;
                const float slope = 2.0f / (1.0f - r);   // du/dt on the linear part
                if (tt < 1.0f - r) { u = -1.0f + 2.0f * tt / (1.0f - r); du = sgn * slope / (2.0f * kPi); ddu = dddu = 0.0f; }
                else {
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

    OscParams _want{};   // as set()
    OscParams _on{};     // as latched: what renders
    bool      _latched = false;
    float     _scale = 1.0f;   // 1 / the shape's peak |u|
    float     _pk_dv = 1.0f, _pk_da = 1.0f, _pk_dj = 1.0f;
    float     _phase = 0.0f;   // at the head
    uint64_t  _head_us = 0;
    bool      _seeded = false;
    float     _eff = 0.0f;
    bool      _shaped = false;
    // The committed targets of the last MaxFade steps (a ring), and the
    // targets the last render() predicted for its outputs.
    std::array<float, MaxFade> _hist{};
    size_t    _hist_at = 0;
    std::array<float, MaxOut> _pred{};
    size_t    _n_pred = 0;
    // Scratch, sized for one render(): B then T, the box sequence, the deque.
    std::array<float, MaxOut + MaxFade> _b{};
    std::array<float, MaxOut + MaxFade> _s{};
    std::array<uint16_t, MaxOut + MaxFade> _q{};
    // Driven: the points (time order), the frequency on the grid, and the
    // phase step of each output the last render() made.
    struct DrivePoint { uint64_t t_us; float hz, amp; };
    std::array<DrivePoint, kDriveMax> _drv{};
    size_t    _drv_n = 0;
    uint32_t  _step_us = 0;
    std::array<float, MaxOut + 2 * MaxFade + 2 * kPlanEdge> _fq{};
    std::array<float, MaxOut> _dph{};
    size_t    _n_dph = 0;
};

}  // namespace kinetic2
