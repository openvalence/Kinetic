// kinetic2/engine.hpp -- Engine<DoF>: knots in, a sampled trajectory out
// Constraints:
// - One entry for motion: submit(axis, knot). Segments, samples and strokes are
//   turned into knots by the caller (Nucleus's arbiter, the wasm shim); the
//   engine never sees a wire format.
// - Event-driven, never clocked: a piece is built when the sampler first needs
//   it, from the state the previous piece ends in, so the rendered curve is
//   continuous in p and v by construction. Sampling is polynomial evaluation.
// - Single-threaded: every call on one Engine comes from one task.
// - SKELETON (kin-nb9): pieces are quintic Hermite through consecutive knots
//   with junction velocities taken as authored or estimated from the chord,
//   junction acceleration 0, and NO ceiling referee yet. The lookahead solver
//   (kin-ahl) replaces pieceFor(); the brake profile (kin-4gd) replaces the
//   trapezoid estimate in brake(). Both are marked below.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "timeline.hpp"
#include "types.hpp"

namespace kinetic2 {

// A quintic Hermite piece in normalized tau over T seconds, from (p0, v0, a0)
// at tau = 0 to (p1, v1, a1) at tau = 1. Coefficients in tau; the derivatives
// are scaled back by T on evaluation.
struct Piece {
    uint64_t start_us = 0;
    uint64_t end_us   = 0;
    float    T        = 0.0f;   // seconds; 0 = a hold at c[0]
    float    c[6]     = {};

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

template <size_t DoF = 1, size_t Capacity = 64>
class Engine {
public:
    static_assert(DoF >= 1, "an engine needs an axis");

    explicit Engine(const Config& cfg, float p0 = 0.5f) : _cfg(cfg) {
        for (size_t ax = 0; ax < DoF; ++ax) resetAxis(ax, p0, 0);
    }

    // ---- lifecycle ----------------------------------------------------------
    // Forget everything and hold at p from now. Used at power-up, after an
    // e-stop, and when the caller's position truth moved under the plan.
    void resetAt(size_t axis, float p, uint64_t now_us) { resetAxis(axis, p, now_us); }
    void resetAt(float p, uint64_t now_us) { resetAt(0, p, now_us); }

    const Config& config() const { return _cfg; }
    void setConfig(const Config& c) { _cfg = c; }
    void setLimits(const Limits& l) { _cfg.limits = l; }

    // ---- the one entry ------------------------------------------------------
    // A knot strictly in the future and after the axis's newest knot. False
    // (and a KnotRefused anomaly) otherwise: a sender that goes backwards or
    // overfills is a bug on its side, counted here, never absorbed.
    bool submit(size_t axis, const Knot& k, uint64_t now_us) {
        Axis& a = _ax[axis];
        if (!std::isfinite(k.p) || (k.has_v && !std::isfinite(k.v))) return refuse(k, now_us, kDetailNonFinite);
        if (k.t_us <= now_us) return refuse(k, now_us, kDetailPast);
        if (a.tl.full()) return refuse(k, now_us, kDetailTimelineFull);
        if (!a.tl.push(k)) return refuse(k, now_us, kDetailPast);
        // A piece already built toward "nothing after this knot" assumed a
        // stop there; a successor changes that junction, so rebuild lazily.
        a.piece_valid = false;
        return true;
    }
    bool submit(const Knot& k, uint64_t now_us) { return submit(0, k, now_us); }

    // ---- brake --------------------------------------------------------------
    // Drop every pending knot and come to rest under amax from the current
    // state. ponytail: a trapezoid estimate of the stop (jerk unbounded in the
    // estimate); kin-4gd replaces it with the jerk-limited brake profile.
    bool brake(uint64_t now_us) {
        for (size_t ax = 0; ax < DoF; ++ax) {
            Axis& a = _ax[ax];
            const State s = stateAt(ax, now_us);
            a.tl.clear();
            a.piece_valid = false;
            a.origin = s;
            a.origin_us = now_us;
            if (std::fabs(s.v) > 1e-6f) {
                const float t_stop = std::fabs(s.v) / _cfg.limits.amax;
                Knot stop;
                stop.t_us = now_us + uint64_t(t_stop * 1e6f) + 1;
                stop.p = s.p + 0.5f * s.v * t_stop;
                stop.v = 0.0f; stop.has_v = true; stop.family = Family::C1;
                a.tl.push(stop);
                record(AnomalyKind::SettleEngaged, now_us, stop.p, s.v);
            }
        }
        return true;
    }

    // ---- sampling -----------------------------------------------------------
    State stateAt(size_t axis, uint64_t now_us) {
        Axis& a = _ax[axis];
        // Retire knots the clock has passed; their end state becomes the origin.
        while (!a.tl.empty() && a.tl.at(0).t_us <= now_us) {
            ensurePiece(a);
            a.origin = a.piece.at(a.tl.at(0).t_us);
            a.origin.a = 0.0f;   // skeleton: junction acceleration is 0
            a.origin_us = a.tl.at(0).t_us;
            a.tl.popFront();
            a.piece_valid = false;
        }
        ensurePiece(a);
        return a.piece.at(now_us);
    }
    float positionAt(uint64_t now_us) { return stateAt(0, now_us).p; }
    float velocityAt(uint64_t now_us) { return stateAt(0, now_us).v; }

    // Motion left to render on any axis.
    bool isBusy(uint64_t now_us) const {
        for (size_t ax = 0; ax < DoF; ++ax) {
            const Axis& a = _ax[ax];
            if (!a.tl.empty() && a.tl.newest().t_us > now_us) return true;
            if (a.tl.empty() && std::fabs(a.origin.v) > 1e-6f) return true;
        }
        return false;
    }

    size_t pending(size_t axis = 0) const { return _ax[axis].tl.size(); }

    // ---- anomalies ----------------------------------------------------------
    bool popAnomaly(Anomaly& out) {
        if (_an_n == 0) return false;
        out = _an[_an_head];
        _an_head = (_an_head + 1) % kAnomalyRing;
        --_an_n;
        return true;
    }

private:
    static constexpr size_t kAnomalyRing = 16;

    struct Axis {
        Timeline<Capacity> tl;
        State    origin{};         // the state the next piece starts from
        uint64_t origin_us = 0;
        Piece    piece{};
        bool     piece_valid = false;
    };

    void resetAxis(size_t ax, float p, uint64_t now_us) {
        Axis& a = _ax[ax];
        a.tl.clear();
        a.origin = State{p, 0.0f, 0.0f};
        a.origin_us = now_us;
        a.piece = Piece::hold(p, now_us);
        a.piece_valid = true;
    }

    // The piece from the origin to the first pending knot, or a hold.
    // SKELETON junction rule: the end velocity is the knot's when authored,
    // else the chord slope toward the knot after it (0 when none follows).
    void ensurePiece(Axis& a) {
        if (a.piece_valid) return;
        if (a.tl.empty()) {
            a.piece = Piece::hold(a.origin.p, a.origin_us);
            a.origin.v = 0.0f; a.origin.a = 0.0f;
        } else {
            const Knot& k = a.tl.at(0);
            State end{k.p, 0.0f, 0.0f};
            if (k.has_v) end.v = k.v;
            else if (a.tl.size() >= 2) {
                const Knot& n = a.tl.at(1);
                const float Tn = float(n.t_us - k.t_us) * 1e-6f;
                const float Tp = float(k.t_us - a.origin_us) * 1e-6f;
                // Catmull-Rom style: the slope across the neighbors.
                end.v = (n.p - a.origin.p) / (Tn + Tp);
            }
            a.piece = Piece::hermite(a.origin_us, a.origin, k.t_us, end);
        }
        a.piece_valid = true;
    }

    bool refuse(const Knot& k, uint64_t now_us, float detail) {
        record(AnomalyKind::KnotRefused, now_us, k.p, detail);
        return false;
    }
    void record(AnomalyKind kind, uint64_t t, float target, float detail) {
        Anomaly& an = _an[(_an_head + _an_n) % kAnomalyRing];
        if (_an_n == kAnomalyRing) _an_head = (_an_head + 1) % kAnomalyRing; else ++_an_n;
        an.kind = uint8_t(kind); an.seq = ++_an_seq; an.t_us = t; an.target = target; an.detail = detail;
    }

    Config   _cfg;
    Axis     _ax[DoF];
    Anomaly  _an[kAnomalyRing]{};
    size_t   _an_head = 0, _an_n = 0;
    uint16_t _an_seq = 0;
};

}  // namespace kinetic2
