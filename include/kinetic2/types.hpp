// kinetic2/types.hpp -- the vocabulary of Kinetic²: knots, junctions, limits,
// policies, anomalies, the sampled state
// Constraints:
// - Float-first: every quantity a caller sees or the sampler computes is float.
//   Time is uint64_t microseconds on the caller's clock; deltas become float
//   seconds only inside a piece (never an absolute time as float).
// - Wire-visible ordinals are pinned: Policy keeps Kinetic 1's values (Stretch
//   0, Blend 5: stored in NVS and carried by the catalog select) and Anomaly
//   kinds 0..10 keep Kinetic 1's meaning and numbering (the per-kind counter
//   tables in Nucleus and valencesim index by them). New kinds append.
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

// ---- Policy -----------------------------------------------------------------
// What to spend when the ceilings cannot honor a knot in its time. Decided over
// the whole lookahead window (RFC-105 promise 3), never per segment.
enum class Policy : uint8_t {
    Stretch = 0,   // keep the stroke, move the knot (spends duration)
    Blend   = 5,   // keep the deadline, trim amplitude down to its floor
};

// ---- Curve family (mirrors the Valence registry `curve_families`) -----------
enum class Family : uint8_t { Unspecified = 0, C1 = 1, C2 = 2, Step = 3 };

// ---- Junction kinds (RFC-105 promise 2) -------------------------------------
// Derived from the family and whether an end velocity was given; never sent.
enum class Junction : uint8_t {
    Smooth   = 0,  // no end velocity, C2: v and a free, the smoothest curve through
    Authored = 1,  // end velocity pinned; a continuous under C2, free to step under C1
    Hard     = 2,  // end velocity 0 under C1: the fastest legal brake lands at the knot
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
    Family   family = Family::Unspecified;
};

// The junction a knot renders, by the rules above. Unspecified behaves as C2.
constexpr Junction junctionOf(const Knot& k) {
    if (!k.has_v) return Junction::Smooth;
    if (k.family == Family::C1 && k.v == 0.0f) return Junction::Hard;
    return Junction::Authored;
}

// ---- Corner (RFC-105 planner option `corner`) --------------------------------
// How an AUTHORED C1 knot with a nonzero velocity renders. Both exist until
// the tuner proves which earns its place (operator 2026-10-05).
enum class Corner : uint8_t {
    Continuous = 0,  // the junction acceleration is smoothed through (default)
    Cubic      = 1,  // each side keeps the author's cubic acceleration, joined
                     // by a jerk-limited ramp centered on the knot
};

// ---- Config -----------------------------------------------------------------
// Every member here is a planner option in the RFC-105 sense: a setup-category
// catalog field on the hub, tunable from the client. Add one only where one
// choice is more accurate in one place and less in another (operator ruling
// 2026-10-05); the default is the behavior the bench says is optimal.
struct Config {
    Limits  limits{};
    Policy  policy          = Policy::Blend;
    float   amplitude_floor = 0.25f;   // Blend never trims a stroke below this share of it
    uint32_t lookahead_us   = 250000;  // how far past now the solver considers knots
    Corner  corner          = Corner::Continuous;
    // A knot arriving while the axis moves re-plans from the state this far
    // ahead of now; the curve up to there is committed. Long enough that the
    // re-plan never starts inside a piece too short to bend legally, short
    // enough that a one-knot guess never freezes into the motion (both were
    // measured: 6 ms pieces spiraled into reversals, a committed knot froze a
    // start-up velocity into every later piece).
    uint32_t react_us       = 4000;
};

// ---- Anomaly ----------------------------------------------------------------
// One event per axis spent, so the counts read as a diagnosis. Kinds 0..10 are
// Kinetic 1's and keep their numbers (several will never be emitted by Kinetic²
// and stay reserved); new kinds append at 11+.
enum class AnomalyKind : uint8_t {
    None              = 0,
    PlanFailed        = 1,   // detail: a sentinel below
    SettleEngaged     = 2,   // the timeline ran dry mid-motion: braked to rest. detail = v at engagement
    EndVelClamped     = 3,   // an authored end velocity exceeded vmax or the wall bound. detail = the clamped v
    DeadlineStretched = 4,   // Stretch moved a knot later. detail = seconds added
    WaveformFallback  = 5,   // reserved (Kinetic 1's Ruckig guard); never emitted
    WaveformScaled    = 6,   // Blend trimmed amplitude. detail = achieved share 0..1
    WaveformCentered  = 7,   // retired in Kinetic 1; never emitted
    HandoffBounded    = 8,   // reserved; the solver owns junction velocities, so nothing to bound
    WaveformSmoothed  = 9,   // reserved; never emitted
    DwellZeroed       = 10,  // the same target re-commanded (a hold): its end velocity dropped
    KnotRefused       = 11,  // a knot not accepted onto the timeline. detail: a sentinel below
};

// PlanFailed / KnotRefused detail sentinels.
inline constexpr float kDetailNonFinite   = -99.0f;
inline constexpr float kDetailPast        = -95.0f;   // t_us at or before the newest knot
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
