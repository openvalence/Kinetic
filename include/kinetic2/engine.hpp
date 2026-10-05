// kinetic2/engine.hpp -- Engine<DoF>: knots in, a sampled trajectory out
// Constraints:
// - One entry for motion: submit(axis, knot). Segments, samples and strokes are
//   turned into knots by the caller (Nucleus's arbiter, the wasm shim); the
//   engine never sees a wire format.
// - Event-driven, never clocked: a piece is built when the sampler first needs
//   it, from the state the previous piece ends in, so the rendered curve is
//   continuous in p and v by construction. Sampling is polynomial evaluation.
// - Single-threaded: every call on one Engine comes from one task.
// - The pending window is solved as a whole (solver.hpp) whenever it changes,
//   lazily at the next sample: junction values, the referee, the spend. A
//   piece is then one solved interval. The brake profile (kin-4gd) replaces
//   the trapezoid estimate in brake(), marked below.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "engine_piece.hpp"
#include "solver.hpp"
#include "timeline.hpp"
#include "types.hpp"

namespace kinetic2 {

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
        // An axis at rest has been holding since its origin: the first piece
        // starts now, not when the hold began.
        if (a.tl.empty()) a.origin_us = now_us;
        if (!a.tl.push(k)) return refuse(k, now_us, kDetailPast);
        // A successor changes the junction of the knot before it: re-solve
        // the window lazily, at the next sample.
        a.solved_valid = false;
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
            a.solved_valid = false;
            a.origin = s;
            a.origin_us = now_us;
            if (std::fabs(s.v) > 1e-6f) {
                // Stop time of a jerk-limited decel: ramp in, hold amax, ramp out.
                const float t_stop = std::fabs(s.v) / _cfg.limits.amax + _cfg.limits.amax / _cfg.limits.jmax;
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
        // Retire knots the clock has passed (at their SOLVED time: Stretch may
        // have moved them); the solved junction state becomes the origin.
        ensureSolved(a);
        while (!a.tl.empty() && a.sol[0].t_us <= now_us) {
            const Solved& k = a.sol[0];
            a.origin = State{k.p, k.v, k.a};
            a.origin_us = k.t_us;
            a.tl.popFront();
            for (size_t i = 0; i + 1 < a.n_sol; ++i) a.sol[i] = a.sol[i + 1];
            if (a.n_sol) --a.n_sol;
            a.piece_valid = false;
        }
        ensurePiece(a);
        return a.piece.at(now_us);
    }
    float positionAt(uint64_t now_us) { return stateAt(0, now_us).p; }
    float velocityAt(uint64_t now_us) { return stateAt(0, now_us).v; }

    // Motion left to render on any axis. Solves first: Stretch may have moved
    // the last knot past the time the sender asked for.
    bool isBusy(uint64_t now_us) {
        for (size_t ax = 0; ax < DoF; ++ax) {
            Axis& a = _ax[ax];
            ensureSolved(a);
            if (a.n_sol && a.sol[a.n_sol - 1].t_us > now_us) return true;
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
        Solved   sol[Capacity]{};  // the solved window, aligned with tl
        size_t   n_sol = 0;
        bool     solved_valid = false;
        Piece    piece{};
        bool     piece_valid = false;
    };

    void resetAxis(size_t ax, float p, uint64_t now_us) {
        Axis& a = _ax[ax];
        a.tl.clear();
        a.origin = State{p, 0.0f, 0.0f};
        a.origin_us = now_us;
        a.n_sol = 0;
        a.solved_valid = true;
        a.piece = Piece::hold(p, now_us);
        a.piece_valid = true;
    }

    // Solve the whole pending window from the origin. Knots are copied out of
    // the ring once so the solver sees them contiguous.
    void ensureSolved(Axis& a) {
        if (a.solved_valid) return;
        Knot tmp[Capacity];
        const size_t n = a.tl.size();
        for (size_t i = 0; i < n; ++i) tmp[i] = a.tl.at(i);
        solveWindow(a.origin, a.origin_us, tmp, n, _cfg, a.sol,
                    [this](AnomalyKind k, uint64_t t, float target, float detail) { record(k, t, target, detail); });
        a.n_sol = n;
        a.solved_valid = true;
        a.piece_valid = false;
    }

    // The piece from the origin to the first solved knot, or a hold.
    void ensurePiece(Axis& a) {
        ensureSolved(a);   // a re-solve invalidates the piece
        if (a.piece_valid) return;
        if (a.n_sol == 0) {
            a.piece = Piece::hold(a.origin.p, a.origin_us);
            a.origin.v = 0.0f; a.origin.a = 0.0f;
        } else {
            const Solved& k = a.sol[0];
            a.piece = Piece::hermite(a.origin_us, a.origin, k.t_us, State{k.p, k.v, k.a});
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
