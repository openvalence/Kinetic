// kinetic2/engine.hpp -- Engine<DoF>: knots in, a sampled trajectory out
// Constraints:
// - One entry for motion: submit(axis, knot), and truncateAfter() to replace
//   what is queued (the segments flush). Segments, samples and strokes are
//   turned into knots by the caller (Nucleus's arbiter, the wasm shim); the
//   engine never sees a wire format.
// - Event-driven, never clocked: a piece is built when the sampler first needs
//   it, from the state the previous piece ends in, so the rendered curve is
//   continuous in p and v by construction. Sampling evaluates one piece
//   (engine_piece.hpp).
// - Single-threaded: every call on one Engine comes from one task.
// - The pending window is solved (solver.hpp) whenever it changes, lazily at
//   the next sample: the handle render of the whole window. A piece is then
//   one solved interval. The brake profile (kin-4gd) replaces the trapezoid
//   estimate in brake(), marked below.
// - A solve renders the whole window (Config::solve_budget is not read yet).
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

    // The caller's frame moved under the plan (its window units now mean
    // other positions): `s` is the state at now_us in the new frame. Every
    // pending knot stays and the window re-solves from s; the curve in
    // flight, committed or not, is replaced. Nothing pending: the brake from
    // s while it moves (an explicit brake stays explicit), else a hold. Set
    // the new frame's limits first: the brake and the solve read them.
    void reseedAt(size_t axis, const State& s, uint64_t now_us) {
        Axis& a = _ax[axis];
        (void)stateAt(axis, now_us);   // retires every knot already due
        const bool braking = a.explicit_brake && now_us < a.origin_us;
        const bool moving = std::fabs(s.v) > 1e-6f || std::fabs(s.a) > 1e-6f;
        a.has_committed = false;
        a.replan_open = false;
        a.explicit_brake = false;
        boundary(a, moving ? s : State{s.p, 0.0f, 0.0f}, now_us);
        a.piece = Piece::hold(s.p, now_us);
        a.piece_valid = a.tl.empty();
        a.n_sol = 0;
        a.solved_valid = a.tl.empty();
        if (!braking && !(a.tl.empty() && moving)) return;
        const Profile pr = Profile::brake(s, now_us, _cfg.limits);
        if (pr.n == 0) return;
        if (a.tl.empty()) a.piece = Piece::profile(pr);
        else { a.committed = Piece::profile(pr); a.has_committed = true; }
        boundary(a, pr.end(), pr.end_us());
        a.explicit_brake = braking;
    }
    void reseedAt(const State& s, uint64_t now_us) { reseedAt(0, s, now_us); }

    const Config& config() const { return _cfg; }
    // The next solve (at the next submit or flush) renders the whole window
    // under it; the curve in flight is not re-solved.
    void setConfig(const Config& c) { _cfg = c; }
    void setLimits(const Limits& l) { Config c = _cfg; c.limits = l; setConfig(c); }

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
        // An explicit brake in flight is kept whole: the knot chains from its
        // end. Dropped with the piece, the plan stepped to the brake's end (a
        // jog or a return submitted during a pause brake, kin-v9z). A curve
        // committed through its knot is never a brake, whatever tail it ends
        // in: re-planned, its knot was dropped (a reversal passed moving,
        // kin-554).
        if (a.tl.empty() && !a.has_committed && a.piece_valid && a.piece.has_tail && now_us < a.piece.end_us) {
            if (a.explicit_brake) { a.committed = a.piece; a.has_committed = true; }
            else replanFromBrake(a, now_us);
        }
        if (a.explicit_brake && now_us >= a.origin_us) a.explicit_brake = false;
        if (a.tl.full()) return refuse(k, now_us, kDetailTimelineFull);
        // An axis at rest has been holding since its origin: the first piece
        // starts now, not when the hold began. A brake in flight keeps its end.
        if (a.tl.empty() && a.origin_us < now_us) boundary(a, a.origin, now_us);
        commitHorizon(axis, now_us, true);
        if (!a.tl.push(k)) { a.replan_open = false; return refuse(k, now_us, kDetailPast); }
        a.solved_valid = false;
        a.piece_valid = false;
        return true;
    }
    bool submit(const Knot& k, uint64_t now_us) { return submit(0, k, now_us); }

    // ---- supersede ----------------------------------------------------------
    // The segments flush (Valence SPEC 5.4 "Supersede, the segments flush",
    // RFC-087): drops every pending knot authored AFTER t_us and keeps the
    // ones at or before it: a segment whose start is at or after t_us is
    // replaced, the one ending exactly at t_us is not (SPEC 9.6, RFC-087), so
    // a bundle that begins where the queue ends changes nothing: the knot at
    // t_us keeps the author's velocity, never a hand-off's. The curve through
    // the reaction horizon is committed
    // first and never moves (RFC-105 (bb)). When t_us lies past the horizon
    // and inside the dropped plan, a knot at t_us carries that plan's (p, v)
    // there, so the motion in flight hands off at t_us as it would to any
    // successor (SPEC 9.6) and a knot submitted after t_us chains from it; a
    // t_us within a tick of the horizon or before it drops the whole pending
    // window and the hand-off is the horizon. Never an anomaly: a flush is the
    // sender's intent. Returns the knots dropped; 0 changed nothing. A HARD
    // knot counts at its solved time when that is later (a live jog landing
    // at its profile's end); a corner's later solved time is its ramp's end
    // and never counts. A live jog authored
    // before the flush but still under way is the move the flush replaces
    // (kin-hnp).
    size_t truncateAfter(size_t axis, uint64_t t_us, uint64_t now_us) {
        Axis& a = _ax[axis];
        if (a.tl.empty() || (a.tl.newest().t_us <= t_us && !a.tl.newest().sample)) return 0;
        commitHorizon(axis, now_us);
        // The hand-off needs the plan at t_us: sol is aligned with tl after it.
        ensureSolved(a);
        const size_t n = a.tl.size();
        auto due = [&](size_t i) {
            const Knot& k = a.tl.at(i);
            return a.sol[i].hard && a.sol[i].t_us > k.t_us ? a.sol[i].t_us : k.t_us;
        };
        if (n == 0 || due(n - 1) <= t_us) return 0;
        const bool past_horizon = t_us > a.origin_us + 1000;
        size_t keep = 0;
        if (past_horizon) while (keep < n && due(keep) <= t_us) ++keep;
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
            h.t_us = t_us; h.p = hs.p; h.v = hs.v; h.has_v = true;
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
            if (pr.n == 0) { boundary(a, State{s.p, 0.0f, 0.0f}, now_us); a.piece = Piece::hold(s.p, now_us); a.piece_valid = true; continue; }
            a.piece = Piece::profile(pr);
            a.piece_valid = true;
            boundary(a, pr.end(), pr.end_us());
            a.explicit_brake = true;
            record(AnomalyKind::SettleEngaged, now_us, a.origin.p, s.v);
        }
        return true;
    }

    // ---- sampling -----------------------------------------------------------
    State stateAt(size_t axis, uint64_t now_us) {
        Axis& a = _ax[axis];
        // Solved first: a solve may undo a re-plan and extend the committed curve.
        ensureSolved(a);
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
                boundary(a, pr.end(), pr.end_us());
                return a.piece.at(now_us);
            }
        }
        // Retire knots the clock has passed (at their SOLVED time: a corner's
        // ramp end, a late HARD knot); the solved state becomes the origin.
        ensureSolved(a);
        // A brake in flight renders until its end; the origin already sits there.
        if (a.tl.empty() && a.piece_valid && a.piece.has_tail && now_us < a.piece.end_us) return a.piece.at(now_us);
        while (!a.tl.empty()) {
            if (a.sol[0].t_us > now_us) break;
            const Solved& k = a.sol[0];
            boundary(a, State{k.p, k.v, k.a}, k.t_us);
            const size_t cnt = a.tl.size();
            a.tl.popFront();
            for (size_t i = 0; i + 1 < cnt; ++i) a.sol[i] = a.sol[i + 1];
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
                boundary(a, pr.end(), pr.end_us());
                a.n_sol = 0; a.solved_valid = true;
                return a.piece.at(now_us);
            }
        }
        ensurePiece(a);
        return a.piece.at(now_us);
    }
    float positionAt(uint64_t now_us) { return stateAt(0, now_us).p; }
    float velocityAt(uint64_t now_us) { return stateAt(0, now_us).v; }

    // ---- lookahead ----------------------------------------------------------
    // The position at t0 + i * step_us for i in [0, n): bit for bit what
    // stateAt(axis, t) returns at each of those times when nothing is
    // submitted, truncated, braked or reset meanwhile and the times only
    // advance (stateAt() replayed on copies). Solves a dirty window first and
    // writes nothing else: it retires no knot, engages no brake, records no
    // anomaly. One buildPiece per knot interval the times reach, never one
    // per time. A time before the axis's last sample is not the plan.
    void peek(size_t axis, uint64_t t0, uint32_t step_us, size_t n, float* p_out) {
        Axis& a = _ax[axis];
        ensureSolved(a);
        const size_t nk = a.n_sol;
        size_t k = 0;                 // the first knot still pending
        bool committed = a.has_committed;
        bool valid = a.piece_valid;
        State o = a.origin;
        uint64_t o_us = a.origin_us;
        Piece pc = a.piece;
        auto moving = [](const State& s) { return std::fabs(s.v) > 1e-6f || std::fabs(s.a) > 1e-6f; };
        auto brakeFromOrigin = [&](uint64_t t) {
            pc = Piece::profile(Profile::brake(o, o_us, _cfg.limits));
            valid = true;
            return pc.at(t).p;
        };
        auto at = [&](uint64_t t) {
            if (committed) {
                if (t < a.committed.end_us) return a.committed.at(t).p;
                committed = false;
                valid = false;
                if (k == nk && moving(o)) return brakeFromOrigin(t);
            }
            if (k == nk && valid && pc.has_tail && t < pc.end_us) return pc.at(t).p;
            while (k < nk && a.sol[k].t_us <= t) {
                const Solved& s = a.sol[k++];
                o = State{s.p, s.v, s.a};
                o_us = s.t_us;
                valid = false;
                if (k == nk && moving(o)) return brakeFromOrigin(t);
            }
            if (!valid) {
                pc = k == nk ? Piece::hold(o.p, o_us) : buildPiece(o, o_us, a.sol[k], _cfg.limits);
                valid = true;
            }
            return pc.at(t).p;
        };
        for (size_t i = 0; i < n; ++i) p_out[i] = at(t0 + uint64_t(i) * step_us);
    }

    // Motion left to render on any axis. Solves first: a HARD knot may land
    // after the time the sender asked for.
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
    // The authored start of the segment in flight (the piece toward the first
    // pending knot): the state and time of the knot retired before it, or of
    // the accept on an axis at rest. plan.start / plan.elapsed / plan.duration.
    State segStart(size_t axis, uint64_t* t_us) const {
        if (t_us) *t_us = _ax[axis].seg_us;
        return _ax[axis].seg;
    }
    // The solver's decision for pending knot i (tooling: the tuner shows the
    // share per knot). Solves first.
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
        // Where the piece toward the first pending knot was authored to start
        // (the knot retired before it, a brake's end, the accept on an axis
        // at rest): the segment in flight, for a census. A re-plan from the
        // reaction horizon moves the origin along the curve, never this.
        State    seg{};
        uint64_t seg_us = 0;
        Solved   sol[Capacity]{};  // the solved window, aligned with tl
        Workspace ws{};      // the renderer's scratch, this axis's own
        size_t   n_sol = 0;        // solved knots; equals tl.size() once solved
        bool     solved_valid = false;
        Piece    piece{};
        bool     piece_valid = false;
        bool     explicit_brake = false;   // brake(): refuses knots before its end
        Piece    committed{};              // the curve kept through the reaction horizon
        bool     has_committed = false;
        // Anomaly kinds already reported per pending knot (by its authored
        // time): a re-solve finds the same trims again and must not report
        // them again. Pruned as knots leave the timeline.
        uint64_t rep_t[Capacity] = {};
        uint16_t rep_m[Capacity] = {};
        size_t   rep_n = 0;
        // A re-plan from the horizon toward the newest free knot, open until
        // the next solve: the piece it replaced and that knot's solved state.
        // A first piece the solve finds over a ceiling is undone: the curve is
        // committed through the knot as the rest end it was rendered as.
        bool     replan_open = false;
        Piece    replan_piece{};
        Solved   replan_k0{};
    };

    // The origin moves to a segment boundary: the segment in flight starts there.
    static void boundary(Axis& a, const State& s, uint64_t t_us) {
        a.origin = s; a.origin_us = t_us;
        a.seg = s; a.seg_us = t_us;
    }

    void resetAxis(size_t ax, float p, uint64_t now_us) {
        Axis& a = _ax[ax];
        a.tl.clear();
        boundary(a, State{p, 0.0f, 0.0f}, now_us);
        a.n_sol = 0;
        a.solved_valid = true;
        a.piece = Piece::hold(p, now_us);
        a.piece_valid = true;
        a.explicit_brake = false;
        a.has_committed = false;
        a.rep_n = 0;
        a.replan_open = false;
    }

    // Render the pending window from the origin, whole. Knots are copied out
    // of the ring once so the solver sees them contiguous.
    void ensureSolved(Axis& a) {
        if (a.solved_valid) return;
        const size_t n = a.tl.size();
        Knot tmp[Capacity];
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
            static_assert(uint8_t(AnomalyKind::PieceOverCeiling) < 16, "the reported mask is one halfword");
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
        const bool undoable = a.replan_open && n > 1;
        a.replan_open = false;
        if (undoable) keepReports(a);
        solveWindow(a.origin, a.origin_us, tmp, n, _cfg, a.sol, report, a.ws);
        a.n_sol = n;
        a.solved_valid = true;
        a.piece_valid = false;
        if (undoable && a.sol[0].infeasible) {
            // The successor came too late to re-solve its predecessor inside
            // the ceilings: the piece in flight is kept through that knot.
            restoreReports(a);
            const Solved& k0 = a.replan_k0;
            a.committed = a.replan_piece;
            a.has_committed = true;
            boundary(a, State{k0.p, k0.v, k0.a}, k0.t_us);
            a.tl.popFront();
            a.solved_valid = false;
            ensureSolved(a);
        }
    }

    void keepReports(const Axis& a) {
        for (size_t i = 0; i < kAnomalyRing; ++i) _an_keep[i] = _an[i];
        _an_head_keep = _an_head; _an_n_keep = _an_n; _an_seq_keep = _an_seq;
        for (size_t i = 0; i < a.rep_n; ++i) { _rep_t_keep[i] = a.rep_t[i]; _rep_m_keep[i] = a.rep_m[i]; }
        _rep_n_keep = a.rep_n;
    }
    void restoreReports(Axis& a) {
        for (size_t i = 0; i < kAnomalyRing; ++i) _an[i] = _an_keep[i];
        _an_head = _an_head_keep; _an_n = _an_n_keep; _an_seq = _an_seq_keep;
        for (size_t i = 0; i < _rep_n_keep; ++i) { a.rep_t[i] = _rep_t_keep[i]; a.rep_m[i] = _rep_m_keep[i]; }
        a.rep_n = _rep_n_keep;
    }

    // The piece from the origin to the first solved knot, or a hold.
    void ensurePiece(Axis& a) {
        ensureSolved(a);   // a re-solve invalidates the piece
        if (a.piece_valid) return;
        if (a.n_sol == 0) {
            a.piece = Piece::hold(a.origin.p, a.origin_us);
            a.origin.v = 0.0f; a.origin.a = 0.0f;
        } else {
            a.piece = buildPiece(a.origin, a.origin_us, a.sol[0], _cfg.limits);
        }
        a.piece_valid = true;
    }

    // The solved window's state at t at or past the origin, knot to knot.
    State planAt(Axis& a, uint64_t t) {
        ensureSolved(a);
        State s = a.origin;
        uint64_t s_us = a.origin_us;
        for (size_t i = 0; i < a.n_sol; ++i) {
            const Solved& k = a.sol[i];
            if (t < k.t_us) return buildPiece(s, s_us, k, _cfg.limits).at(t);
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
    void commitHorizon(size_t axis, uint64_t now_us, bool successor = false) {
        Axis& a = _ax[axis];
        if (a.tl.empty() || now_us <= a.origin_us || a.has_committed) return;
        (void)stateAt(axis, now_us);   // retires what is due, builds the piece
        if (!a.tl.empty()) {
            const uint64_t tr = now_us + _cfg.react_us;
            const Solved& k0 = a.sol[0];
            // A knot within one tick past the horizon counts as reached:
            // left pending, it would be re-solved from one tick before it,
            // a piece too short to bend legally. So does a corner whose knot
            // is within the horizon (re-planned from inside its ramp, the
            // knot would lie behind the origin): the curve is committed
            // through. So is a knot in the later half of its piece whose
            // state was solved with a successor in view or authored: a
            // re-plan from inside the piece is a new Bezier, not the rest of
            // this one, and too short to bend it trims the knot. Never the
            // newest free knot: committed through, it stays the rest end it
            // was rendered as, and a stream would stop at every knot.
            const uint64_t span = k0.base_us > a.origin_us ? k0.base_us - a.origin_us : 0;
            // Never a chased sample: committed through, the stream would come
            // to its rest at every sample instead of re-planning toward the newest.
            const bool chased = a.tl.at(0).sample && !a.tl.at(0).has_v;
            // A HARD knot (a live jog, a chase run) is reached at its solved
            // time, never its authored one: committed through on that, a jog's
            // whole move stood and the next jog waited it out (kin-g1f).
            const bool fixed = !chased && !k0.hard && (a.tl.at(0).has_v || a.tl.size() > 1) && k0.base_us <= tr + span / 2;
            if (k0.t_us <= tr + 1000 || (!k0.hard && k0.base_us <= tr + 1000) || fixed) {
                // Commit through the knot: its piece is kept whole.
                a.committed = a.piece;
                boundary(a, State{k0.p, k0.v, k0.a}, k0.t_us);
                const size_t cnt = a.tl.size();
                a.tl.popFront();
                for (size_t i = 0; i + 1 < cnt; ++i) a.sol[i] = a.sol[i + 1];
                if (a.n_sol) --a.n_sol;
            } else {
                // Commit the curve up to the horizon and re-plan from there.
                if (successor && a.tl.size() == 1 && !a.tl.at(0).has_v) {
                    a.replan_open = true;
                    a.replan_piece = a.piece;
                    a.replan_k0 = k0;
                }
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
    // The anomaly ring as it stood before a solve an open re-plan may undo.
    Anomaly  _an_keep[kAnomalyRing]{};
    size_t   _an_head_keep = 0, _an_n_keep = 0;
    uint16_t _an_seq_keep = 0;
    uint64_t _rep_t_keep[Capacity] = {};
    uint16_t _rep_m_keep[Capacity] = {};
    size_t   _rep_n_keep = 0;
    Anomaly  _an[kAnomalyRing]{};
    size_t   _an_head = 0, _an_n = 0;
    uint16_t _an_seq = 0;
};

}  // namespace kinetic2
