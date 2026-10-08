// kinetic2/types.hpp -- the vocabulary of Kinetic²: knots, junctions, limits,
// policies, anomalies, the sampled state
// Constraints:
// - Float-first: every quantity a caller sees or the sampler computes is float.
//   Time is uint64_t microseconds on the caller's clock; deltas become float
//   seconds only inside a piece (never an absolute time as float).
// - Wire-visible ordinals are pinned: Policy values (Stretch 0, Blend 5) are
//   stored in NVS and carried by the catalog select, and Anomaly kinds keep
//   their meaning and numbering (the per-kind counter tables in Nucleus and
//   valencesim index by them). New kinds append.
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
// Not read by Kinetic²: time never gives, amplitude does (kin-y6e). The
// ordinals are wire-pinned (NVS, the catalog select) and stay.
enum class Policy : uint8_t {
    Stretch = 0,
    Blend   = 5,
};

// ---- Curve family (mirrors the Valence registry `curve_families`) -----------
enum class Family : uint8_t { Unspecified = 0, C1 = 1, C2 = 2, Step = 3 };

// ---- Junction kinds (RFC-105 promise 2) -------------------------------------
// Derived from the family and whether an end velocity was given; never sent.
enum class Junction : uint8_t {
    Smooth   = 0,  // no end velocity, C2: v and a free, the smoothest curve through
    Authored = 1,  // end velocity pinned; a continuous under C2, free to step under C1
    Hard     = 2,  // a live jog (a C1 sample at rest): the fastest move to rest on the knot
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
    // A sample (sources.hpp): rendered as any knot at its own time, trimmed
    // like one; a HARD sample (a live jog, junctionOf) is the one knot that
    // may land after its time.
    bool     sample = false;
    // Accepted and not read: the handle renderer rests every newest knot
    // without an authored velocity (SPEC 9.6) until its successor frees it,
    // whatever this says. sources.hpp still sets it for a segment without an
    // end velocity; whether a sample keeps a non-rest angle is the owed
    // samples ruling (kin-y6e, consumers kin-ebc).
    bool     rest_if_last = false;
};

// The junction a knot renders, by the rules above. Unspecified behaves as C2.
// Only a SAMPLE is Hard: an authored segment with v = 0 is a reversal or a
// hold in the author's curve, rendered with the author's accelerations under
// C1 (Corner::Cubic) and smoothly under C2. A C1 funscript reversal made
// Hard braked to rest at every peak (Kinetic kin-ecn, kin-7jd).
constexpr Junction junctionOf(const Knot& k) {
    if (!k.has_v) return Junction::Smooth;
    if (k.sample && k.family == Family::C1 && k.v == 0.0f) return Junction::Hard;
    return Junction::Authored;
}

// ---- Corner (RFC-105 planner option `corner`) --------------------------------
// How an AUTHORED C1 knot renders. Cubic is the default: the machine matches
// the author's curve (operator ruling 2026-10-07); Continuous stays as the
// option. The handle renderer (solver.hpp) does not read it: every knot
// renders the Cubic way until kin-tnv maps the option onto the renderer.
enum class Corner : uint8_t {
    Continuous = 0,  // the junction acceleration is smoothed through
    Cubic      = 1,  // each side keeps the author's cubic acceleration, joined
                     // by a jerk-limited ramp centered on the knot (default)
};

// ---- Config -----------------------------------------------------------------
// Every member here is a planner option in the RFC-105 sense: a setup-category
// catalog field on the hub, tunable from the client. Add one only where one
// choice is more accurate in one place and less in another (operator ruling
// 2026-10-05); the default is the behavior the bench says is optimal.
struct Config {
    Limits  limits{};
    // policy, amplitude_floor and corner are not read by the handle renderer:
    // time never gives, a trim may take the whole chord, and every G1 knot
    // takes the corner ramp (Corner::Continuous is a no-op). They stay for
    // the ABI and the catalog until kin-tnv maps or retires them.
    Policy  policy          = Policy::Blend;
    float   amplitude_floor = 0.25f;
    uint32_t lookahead_us   = 250000;  // how far past now the solver considers knots
    Corner  corner          = Corner::Cubic;
    // A knot arriving while the axis moves re-plans from the state this far
    // ahead of now; the curve up to there is committed. Long enough that the
    // re-plan never starts inside a piece too short to bend legally, short
    // enough that a one-knot guess never freezes into the motion (both were
    // measured: 6 ms pieces spiraled into reversals, a committed knot froze a
    // start-up velocity into every later piece).
    uint32_t react_us       = 4000;
    // Ignored: the handle renderer is never late (time never gives). Kept so
    // the layout and the catalog stay as they are.
    uint32_t late_budget_us = 100000;
    // The most work one solve may do on the motion tick. The handle renderer
    // renders the whole window and does not read it yet; kin-tnv bounds it
    // per tick before a Nucleus pin bump. Measured 2026-10-07 on x86 -O2
    // (bench_kinetic2): a 3-knot funscript plan 0.17 ms mean, 0.47 ms max; a
    // 60 Hz stream 0.39 ms mean, 1.2 ms max; a 64-knot bundle 5.4 ms; the
    // P4's float FPU is 10 to 20 times slower. FIXED BY THE MACHINE, never a
    // catalog field (operator ruling 2026-10-06). 0: unbounded.
    uint32_t solve_budget   = 96;
};

// ---- Anomaly ----------------------------------------------------------------
// One event per axis spent, so the counts read as a diagnosis. Every number is
// pinned; a kind marked never emitted stays reserved, new kinds append at 13+.
enum class AnomalyKind : uint8_t {
    None              = 0,
    PlanFailed        = 1,   // the knot was dropped. Never emitted by Kinetic² (no knot is dropped); consumers read it as a drop
    SettleEngaged     = 2,   // the timeline ran dry mid-motion: braked to rest. detail = v at engagement
    EndVelClamped     = 3,   // an authored end velocity exceeded vmax or the wall bound. detail = the clamped v
    DeadlineStretched = 4,   // a knot placed late. Never emitted by Kinetic² (time never gives)
    WaveformFallback  = 5,   // reserved; never emitted
    WaveformScaled    = 6,   // a knot trimmed toward its predecessor. detail = the share of its chord kept
    WaveformCentered  = 7,   // reserved; never emitted
    HandoffBounded    = 8,   // reserved; the solver owns junction velocities, so nothing to bound
    WaveformSmoothed  = 9,   // reserved; never emitted
    DwellZeroed       = 10,  // the same target re-commanded (a hold): its end velocity dropped
    KnotRefused       = 11,  // a knot not accepted onto the timeline. detail: a sentinel below
    PieceOverCeiling  = 12,  // a piece no trim makes legal renders at its least-over trim. detail = its worst ceiling ratio
};

// KnotRefused detail sentinels.
inline constexpr float kDetailNonFinite   = -99.0f;
inline constexpr float kDetailPast        = -95.0f;   // t_us at or before the newest knot
inline constexpr float kDetailTimelineFull = -96.0f;
inline constexpr float kDetailBudget       = -97.0f;   // reserved; never emitted by Kinetic²

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
