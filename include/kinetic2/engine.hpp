// kinetic2/engine.hpp -- Engine<DoF>: knots in, a sampled trajectory out
// Constraints:
// - One entry for motion: submit(axis, knot), and truncateAfter() to replace
//   what is queued (the segments flush). Segments, samples and strokes are
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
        // Inside an explicit brake (pause, e-stop) the brake wins. A starvation
        // brake is the engine's own guess that nothing follows; a knot proves
        // it wrong, so the engine re-plans from where the carriage is.
        if (a.explicit_brake && k.t_us <= a.origin_us) return refuse(k, now_us, kDetailPast);
        if (!a.explicit_brake && a.tl.empty() && a.piece_valid && a.piece.has_tail && now_us < a.piece.end_us) replanFromBrake(a, now_us);
        if (a.explicit_brake && now_us >= a.origin_us) a.explicit_brake = false;
        if (a.tl.full()) return refuse(k, now_us, kDetailTimelineFull);
        // An axis at rest has been holding since its origin: the first piece
        // starts now, not when the hold began. A brake in flight keeps its end.
        if (a.tl.empty() && a.origin_us < now_us) a.origin_us = now_us;
        commitHorizon(axis, now_us);
        if (!a.tl.push(k)) return refuse(k, now_us, kDetailPast);
        a.solved_valid = false;
        a.piece_valid = false;
        return true;
    }
    bool submit(const Knot& k, uint64_t now_us) { return submit(0, k, now_us); }

    // ---- supersede ----------------------------------------------------------
    // The segments flush (Valence SPEC 5.4 "Supersede, the segments flush",
    // RFC-087): drops every pending knot authored at or after t_us and keeps
    // the ones before it. The curve through the reaction horizon is committed
    // first and never moves (RFC-105 (bb)). When t_us lies past the horizon
    // and inside the dropped plan, a knot at t_us carries that plan's (p, v)
    // there, so the motion in flight hands off at t_us as it would to any
    // successor (SPEC 9.6) and a knot submitted after t_us chains from it; a
    // t_us within a tick of the horizon or before it drops the whole pending
    // window and the hand-off is the horizon. Never an anomaly: a flush is the
    // sender's intent. Returns the knots dropped; 0 changed nothing.
    size_t truncateAfter(size_t axis, uint64_t t_us, uint64_t now_us) {
        Axis& a = _ax[axis];
        if (a.tl.empty() || a.tl.newest().t_us < t_us) return 0;
        commitHorizon(axis, now_us);
        ensureSolved(a);   // may drop unreachable knots; sol is aligned with tl after it
        const size_t n = a.tl.size();
        const bool past_horizon = t_us > a.origin_us + 1000;
        size_t keep = 0;
        if (past_horizon) while (keep < n && a.tl.at(keep).t_us < t_us) ++keep;
        if (keep == n) return 0;
        const bool handoff = past_horizon && (keep == 0 || a.sol[keep - 1].t_us < t_us);
        const State hs = handoff ? planAt(a, t_us) : State{};
        a.tl.truncate(keep);
        a.n_sol = keep;
        {
            size_t m = 0;
            for (size_t i = 0; i < a.rep_n; ++i)
                if (a.rep_t[i] < t_us) { a.rep_t[m] = a.rep_t[i]; a.rep_m[m] = a.rep_m[i]; ++m; }
            a.rep_n = m;
        }
        if (handoff) {
            Knot h;
            h.t_us = t_us; h.p = hs.p; h.v = hs.v; h.has_v = true; h.family = Family::C2;
            (void)a.tl.push(h);   // after every kept knot, with room: one was dropped
        }
        a.solved_valid = false;
        a.piece_valid = false;
        return n - keep;
    }
    size_t truncateAfter(uint64_t t_us, uint64_t now_us) { return truncateAfter(0, t_us, now_us); }

    // The newest pending knot as authored (a flush's hand-off knot included);
    // with nothing pending, the origin as a knot carrying its (p, v).
    Knot newest(size_t axis = 0) const {
        const Axis& a = _ax[axis];
        if (!a.tl.empty()) return a.tl.newest();
        Knot k;
        k.t_us = a.origin_us; k.p = a.origin.p; k.v = a.origin.v; k.has_v = true;
        return k;
    }

    // ---- brake --------------------------------------------------------------
    // Drop every pending knot and stop as fast as the ceilings allow from the
    // current state (profile.hpp). The brake wins: the origin moves to its
    // end, at rest, so a knot submitted meanwhile chains from there and one
    // before its end is refused as past.
    bool brake(uint64_t now_us) {
        for (size_t ax = 0; ax < DoF; ++ax) {
            Axis& a = _ax[ax];
            const State s = stateAt(ax, now_us);
            a.tl.clear();
            a.n_sol = 0;
            a.solved_valid = true;
            a.has_committed = false;
            const Profile pr = Profile::brake(s, now_us, _cfg.limits);
            if (pr.n == 0) { a.origin = State{s.p, 0.0f, 0.0f}; a.origin_us = now_us; a.piece = Piece::hold(s.p, now_us); a.piece_valid = true; continue; }
            a.piece = Piece::profile(pr);
            a.piece_valid = true;
            a.origin = pr.end();
            a.origin_us = pr.end_us();
            a.explicit_brake = true;
            record(AnomalyKind::SettleEngaged, now_us, a.origin.p, s.v);
        }
        return true;
    }

    // ---- sampling -----------------------------------------------------------
    State stateAt(size_t axis, uint64_t now_us) {
        Axis& a = _ax[axis];
        if (a.has_committed) {
            if (now_us < a.committed.end_us) return a.committed.at(now_us);
            a.has_committed = false;
            a.piece_valid = false;
            // With nothing accepted after the committed curve and its end
            // still moving, the stream starved: brake from there.
            ensureSolved(a);
            if (a.tl.empty() && (std::fabs(a.origin.v) > 1e-6f || std::fabs(a.origin.a) > 1e-6f)) {
                const Profile pr = Profile::brake(a.origin, a.origin_us, _cfg.limits);
                a.piece = Piece::profile(pr);
                a.piece_valid = true;
                record(AnomalyKind::SettleEngaged, a.origin_us, pr.end().p, a.origin.v);
                a.origin = pr.end();
                a.origin_us = pr.end_us();
                return a.piece.at(now_us);
            }
        }
        // Retire knots the clock has passed (at their SOLVED time: Stretch may
        // have moved them); the solved junction state becomes the origin.
        ensureSolved(a);
        // A brake in flight renders until its end; the origin already sits there.
        if (a.tl.empty() && a.piece_valid && a.piece.has_tail && now_us < a.piece.end_us) return a.piece.at(now_us);
        while (!a.tl.empty() && a.sol[0].t_us <= now_us) {
            const Solved& k = a.sol[0];
            a.origin = State{k.p, k.v, k.a};
            a.origin_us = k.t_us;
            a.before = a.tl.at(0); a.before_solved_us = k.t_us; a.has_before = true;
            a.tl.popFront();
            for (size_t i = 0; i + 1 < a.n_sol; ++i) a.sol[i] = a.sol[i + 1];
            if (a.n_sol) --a.n_sol;
            a.piece_valid = false;
            // The last knot reached while still moving: a starved stream (or a
            // script that ended moving). The only honest rendering is the
            // brake from that state, landing at rest; it yields to a new knot.
            if (a.tl.empty() && (std::fabs(a.origin.v) > 1e-6f || std::fabs(a.origin.a) > 1e-6f)) {
                const Profile pr = Profile::brake(a.origin, a.origin_us, _cfg.limits);
                a.piece = Piece::profile(pr);
                a.piece_valid = true;
                record(AnomalyKind::SettleEngaged, a.origin_us, pr.end().p, a.origin.v);
                a.origin = pr.end();
                a.origin_us = pr.end_us();
                a.n_sol = 0; a.solved_valid = true;
                return a.piece.at(now_us);
            }
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
            if (a.has_committed && a.committed.end_us > now_us) return true;
            ensureSolved(a);
            if (a.n_sol && a.sol[a.n_sol - 1].t_us > now_us) return true;
            if (a.tl.empty() && a.piece_valid && a.piece.has_tail && a.piece.end_us > now_us) return true;
            if (a.tl.empty() && std::fabs(a.origin.v) > 1e-6f) return true;   // never after a brake: its origin is at rest
        }
        return false;
    }

    size_t pending(size_t axis = 0) const { return _ax[axis].tl.size(); }
    // The solver's decision for pending knot i (tooling: the tuner shows the
    // share and the stretch per knot). Solves first.
    const Solved& solved(size_t axis, size_t i) { ensureSolved(_ax[axis]); return _ax[axis].sol[i]; }

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
        jerk::Workspace ws{};      // the solver's banded system, this axis's own
        size_t   n_sol = 0;
        bool     solved_valid = false;
        Piece    piece{};
        bool     piece_valid = false;
        bool     explicit_brake = false;   // brake(): refuses knots before its end
        Piece    committed{};              // the curve kept through the reaction horizon
        bool     has_committed = false;
        // Anomaly kinds already reported per pending knot (by its authored
        // time): a re-solve finds the same spends again and must not report
        // them again. Pruned as knots leave the timeline.
        uint64_t rep_t[Capacity] = {};
        uint16_t rep_m[Capacity] = {};
        size_t   rep_n = 0;
        // The knot retired last, as authored: a stream's previous sample,
        // which the solver's derivatives at the first knot need.
        Knot     before{};
        uint64_t before_solved_us = 0;   // when it was reached
        bool     has_before = false;
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
        a.explicit_brake = false;
        a.has_committed = false;
        a.rep_n = 0;
        a.has_before = false;
    }

    // Solve the whole pending window from the origin. Knots are copied out of
    // the ring once so the solver sees them contiguous.
    void ensureSolved(Axis& a) {
        if (a.solved_valid) return;
        Knot tmp[Capacity];
        const size_t n = a.tl.size();
        for (size_t i = 0; i < n; ++i) tmp[i] = a.tl.at(i);
        // Forget the reported kinds of knots no longer pending.
        {
            const uint64_t oldest = n ? tmp[0].t_us : ~uint64_t(0);
            size_t m = 0;
            for (size_t i = 0; i < a.rep_n; ++i)
                if (a.rep_t[i] >= oldest) { a.rep_t[m] = a.rep_t[i]; a.rep_m[m] = a.rep_m[i]; ++m; }
            a.rep_n = m;
        }
        auto report = [this, &a, &tmp, n](AnomalyKind k, size_t i, uint64_t t, float target, float detail) {
            static_assert(uint8_t(AnomalyKind::KnotRefused) < 16, "the reported mask is one halfword");
            const uint16_t bit = uint16_t(1u << uint8_t(k));
            if (i < n) {
                const uint64_t key = tmp[i].t_us;
                size_t j = 0;
                while (j < a.rep_n && a.rep_t[j] != key) ++j;
                if (j == a.rep_n) {
                    if (a.rep_n == Capacity) { for (size_t q = 1; q < Capacity; ++q) { a.rep_t[q - 1] = a.rep_t[q]; a.rep_m[q - 1] = a.rep_m[q]; } --a.rep_n; j = a.rep_n; }
                    a.rep_t[j] = key; a.rep_m[j] = 0; ++a.rep_n;
                }
                if (a.rep_m[j] & bit) return;
                a.rep_m[j] |= bit;
            }
            record(k, t, target, detail);
        };
        // Each knot's time and junction acceleration from the previous solve
        // (aligned with the ring: pops and erases shift both), zero time for a
        // knot solved for the first time. A sample keeps its lag from it.
        Prior prior[Capacity];
        for (size_t i = 0; i < n; ++i) prior[i] = i < a.n_sol ? Prior{a.sol[i].t_us, a.sol[i].v, a.sol[i].a} : Prior{};
        solveWindow(a.origin, a.origin_us, tmp, n, _cfg, a.sol, report, a.ws, a.has_before ? &a.before : nullptr, a.before_solved_us, prior);
        // A knot the solver dropped leaves the timeline for good.
        size_t m = 0;
        for (size_t i = 0; i < n; ++i) {
            if (a.sol[i].dropped) { a.tl.erase(m); continue; }
            a.sol[m++] = a.sol[i];
        }
        a.n_sol = m;
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
            a.piece = pieceInto(a.origin, a.origin_us, a.sol[0]);
        }
        a.piece_valid = true;
    }

    // The piece from state s at s_us into solved knot k.
    Piece pieceInto(const State& s, uint64_t s_us, const Solved& k) const {
        Piece q;
        // A HARD head solved from rest may start later than s_us: the piece
        // holds s until then (Piece::at clamps).
        const uint64_t t0 = (k.hard && k.from_us > s_us) ? k.from_us : s_us;
        if (k.hard) {
            q = Piece::hermite(t0, s, k.head_us, k.head);
            q.has_tail = true;
            q.tail = Profile::brake(k.head, k.head_us, _cfg.limits);
            q.end_us = k.t_us;
        } else if (k.corner) {
            q = Piece::hermite(s_us, s, k.head_us, k.head);
            q.has_tail = true;
            q.tail = k.ramp;
            q.end_us = k.t_us;
        } else {
            q = Piece::hermite(s_us, s, k.t_us, State{k.p, k.v, k.a});
        }
        return q;
    }

    // The solved window's state at t at or past the origin, knot to knot.
    State planAt(Axis& a, uint64_t t) {
        ensureSolved(a);
        State s = a.origin;
        uint64_t s_us = a.origin_us;
        for (size_t i = 0; i < a.n_sol; ++i) {
            const Solved& k = a.sol[i];
            if (t < k.t_us) return pieceInto(s, s_us, k).at(t);
            s = State{k.p, k.v, k.a};
            s_us = k.t_us;
        }
        return s;
    }

    // A starvation brake in flight is kept through the reaction horizon and
    // the next knot is solved from the state there, like any curve in flight
    // (RFC-105 (bb)). The brake's end stays the origin only when the horizon
    // is past it.
    void replanFromBrake(Axis& a, uint64_t now_us) {
        const uint64_t tr = now_us + _cfg.react_us;
        if (tr < a.piece.end_us) {
            a.committed = a.piece; a.committed.end_us = tr;
            a.origin = a.piece.at(tr); a.origin_us = tr;
            a.has_committed = true;
        }
    }

    // An axis in motion keeps the curve it is on through the reaction
    // horizon (Config::react_us), or through the next knot when that is
    // nearer, and re-plans from the state there. The curve under the
    // carriage never moves (a re-plan from the piece's start moved it by
    // 67 mm on the bench), the re-plan never starts inside a piece too
    // short to bend legally, and nothing freezes a one-knot guess into
    // later motion: RFC-105 (bb). Once per sample: a commit already made
    // stands until the sampler passes it.
    void commitHorizon(size_t axis, uint64_t now_us) {
        Axis& a = _ax[axis];
        if (a.tl.empty() || now_us <= a.origin_us || a.has_committed) return;
        (void)stateAt(axis, now_us);   // retires what is due, builds the piece
        if (!a.tl.empty()) {
            const uint64_t tr = now_us + _cfg.react_us;
            const Solved& k0 = a.sol[0];
            // A knot within one tick past the horizon counts as reached:
            // left pending, it was re-solved one tick past the horizon
            // with its prior junction, a piece no quintic can make legal,
            // and the stretch that followed grew the stream's lag.
            if (k0.t_us <= tr + 1000) {
                // Commit through the knot: its piece is kept whole.
                a.committed = a.piece;
                a.origin = State{k0.p, k0.v, k0.a};
                a.origin_us = k0.t_us;
                a.before = a.tl.at(0); a.before_solved_us = k0.t_us; a.has_before = true;
                a.tl.popFront();
                for (size_t i = 0; i + 1 < a.n_sol; ++i) a.sol[i] = a.sol[i + 1];
                if (a.n_sol) --a.n_sol;
            } else {
                // Commit the curve up to the horizon and re-plan from there.
                a.committed = a.piece;
                a.committed.end_us = tr;
                a.origin = a.piece.at(tr);
                a.origin_us = tr;
            }
            a.has_committed = true;
        } else if (!a.explicit_brake && a.piece_valid && a.piece.has_tail && now_us < a.piece.end_us) {
            // The knot due at this instant retired inside that call and
            // starved the stream: the brake engaged and moved the origin
            // to its end. Solved from there, the new knot's piece began
            // in the future and the carriage teleported to the brake's
            // end (a 29 mm step on the playground's starved stream).
            replanFromBrake(a, now_us);
        }
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
