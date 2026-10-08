// kinetic2/types.hpp -- the vocabulary of Kinetic²: knots, junctions, limits,
// the planner options, anomalies, the sampled state
// Constraints:
// - Float-first: every quantity a caller sees or the sampler computes is float.
//   Time is uint64_t microseconds on the caller's clock; deltas become float
//   seconds only inside a piece (never an absolute time as float).
// - Anomaly kinds are wire-visible: they keep their meaning and numbering
//   (the per-kind counter tables in Nucleus and valencesim index by them).
//   New kinds append.
// - Normalized units: position 0..1 across the caller's travel window;
//   velocity, acceleration and jerk in window units per s, s^2, s^3.
// See: Valence RFC-105 (the promises), Nucleus val-7p2 (the rulings)
#pragma once

#include <cstdint>

namespace kinetic2 {

inline constexpr const char* kVersion = "2.0.0-dev";

// ---- Limits -----------------------------------------------------------------
// Ceilings, never targets. The machine owns these numbers (the jerk setting is
// the machine's business); the planner never exceeds and never second-guesses.
struct Limits {
    float vmax = 3.0f;     // window units/s
    float amax = 30.0f;    // window units/s^2
    float jmax = 500.0f;   // window units/s^3
};

// ---- Junction kinds (RFC-105 promise 2) -------------------------------------
// Derived from whether an end velocity was given and whether the knot is a
// sample; never sent.
enum class Junction : uint8_t {
    Smooth   = 0,  // no end velocity: the renderer solves the angle (Config::smoothness)
    Authored = 1,  // end velocity given: the knot's angle
    Hard     = 2,  // a live jog (a sample at rest): the fastest move to rest on the knot
};

// ---- Knot -------------------------------------------------------------------
// The one thing every source produces. A segment is a knot at anchor + duration
// carrying the target and the sender's end velocity; a sample is a knot at
// arrival + the declared latency carrying the sample; a stroke generator emits
// its turnarounds as knots. The timeline orders them by t_us.
struct Knot {
    uint64_t t_us   = 0;        // when the curve passes through p, engine clock
    float    p      = 0.5f;     // window units
    float    v      = 0.0f;     // window units/s, meaningful when has_v
    bool     has_v  = false;
    // A sample (sources.hpp): a run of position-only samples is the chase
    // (solver.hpp chaseRun); a sample at rest (has_v, v = 0) is a live jog,
    // the HARD junction, the one knot that may land after its time.
    bool     sample = false;
    // Accepted and not read: the handle renderer rests every newest knot
    // without an authored velocity (SPEC 9.6) until its successor frees it,
    // or while the stream owner expects one (Engine::expect), whatever this
    // says. sources.hpp still sets it for a segment without an
    // end velocity; whether a sample keeps a non-rest angle is the owed
    // samples ruling (kin-y6e, consumers kin-ebc).
    bool     rest_if_last = false;
};

// The junction a knot renders, by the rules above. Only a SAMPLE is Hard: an
// authored segment with v = 0 is a reversal or a hold in the author's curve,
// rendered through, never braked to rest.
constexpr Junction junctionOf(const Knot& k) {
    if (!k.has_v) return Junction::Smooth;
    if (k.sample && k.v == 0.0f) return Junction::Hard;
    return Junction::Authored;
}

// ---- Config -----------------------------------------------------------------
// Every member but limits and solve_budget is a planner option in the RFC-105
// sense: a setup-category catalog field on the hub, tunable from the client.
// Every member is read (solve_budget's reader is owed, below): one the
// planner stops reading leaves this struct. Add
// one only where one choice is more accurate in one place and less in another
// (operator ruling 2026-10-05); the default is the behavior the bench says is
// optimal. See: Valence RFC-108 (the set).
struct Config {
    Limits   limits{};
    // How free knots render: 0 crisp (pchip: no overshoot between monotone
    // knots, acceleration may step at a crest), 1 smooth (acceleration
    // continuous where the rules allow; a through point may overshoot, a
    // crest or hold edge is never passed), the lerp
    // of the two between (handles.hpp; a mid value solves twice). 0..1.
    float    smoothness     = 0.0f;
    // The shortest handle a ceiling fit may leave a piece, share of its span:
    // the sharpest a fit may bend it. Held to at least handles::kLMin.
    float    handle_floor   = 0.15f;
    // The farthest a knot moves toward its predecessor to fit the ceilings,
    // share of the window span. Below 1 a piece no trim within it makes legal
    // renders over a ceiling, reported PieceOverCeiling, the newest knot's
    // included. 0 turns trims off.
    float    trim_max       = 1.0f;
    // A knot arriving while the axis moves re-plans from the state this far
    // ahead of now; the curve up to there is committed. Long enough that the
    // re-plan never starts inside a piece too short to bend legally, short
    // enough that a one-knot guess never freezes into the motion (both were
    // measured: 6 ms pieces spiraled into reversals, a committed knot froze a
    // start-up velocity into every later piece).
    uint32_t react_us       = 4000;
    // The most work one planner run may do. FIXED BY THE MACHINE, never a
    // catalog field (operator ruling 2026-10-06). Not read yet: every run
    // renders its whole window; the bound is a resumable slice, bit-identical
    // to an unbounded render (Valence RFC-108 item 8, kin-tnv). 0: unbounded.
    uint32_t solve_budget   = 96;
};

// ---- Anomaly ----------------------------------------------------------------
// One event per axis spent, so the counts read as a diagnosis. Index-aligned
// with Valence's motion-anomaly event kinds (RFC-108); new kinds append.
enum class AnomalyKind : uint8_t {
    None             = 0,
    SettleEngaged    = 1,   // the timeline ran dry mid-motion: braked to rest. detail = v at engagement
    EndVelClamped    = 2,   // an authored end velocity exceeded vmax or the wall bound. detail = the clamped v
    KnotTrimmed      = 3,   // a knot trimmed toward its predecessor. detail = the share of its chord kept
    DwellZeroed      = 4,   // the same target re-commanded (a hold): its end velocity dropped
    KnotRefused      = 5,   // a knot not accepted onto the timeline. detail: a sentinel below
    PieceOverCeiling = 6,   // a piece no trim makes legal renders at its least-over trim. detail = its worst ceiling ratio
};

// KnotRefused detail sentinels.
inline constexpr float kDetailNonFinite    = -99.0f;
inline constexpr float kDetailPast         = -95.0f;   // t_us at or before the newest knot
inline constexpr float kDetailTimelineFull = -96.0f;

struct Anomaly {
    uint8_t  kind   = 0;      // AnomalyKind
    uint16_t seq    = 0;      // rolling event id
    uint64_t t_us   = 0;      // engine time at record
    float    target = 0.0f;   // the knot's p
    float    detail = 0.0f;   // kind-specific
};

// ---- Sampled state ----------------------------------------------------------
struct State {
    float p = 0.5f;
    float v = 0.0f;
    float a = 0.0f;
};

}  // namespace kinetic2
