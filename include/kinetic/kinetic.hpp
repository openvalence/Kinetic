// Kinetic — jerk-limited dual-mode motion core (quintic waveform + Ruckig).
//
// PURPOSE
// -------
// The jerk-limited motion core: every incoming
// motion command becomes ONE planned trajectory computed from the engine's
// ACTUAL current kinematic state (position + velocity + acceleration), and the
// existing ~1 kHz stream sampler simply evaluates it. Event-driven, never
// clocked — a plan is computed only when a command arrives (or when a stream
// starves mid-glide, see SETTLE below).
//
// THE DIVISION OF LABOR (measured, not assumed -- see the part-1 bench)
// --------------------------------------------------------------------
// Ruckig Community is a superb POINT-TO-POINT planner and a poor waveform
// INTERPOLATOR: its minimum_duration-stretched profiles are bang-cruise-bang
// (no smoothness objective -- following interior points is the Pro "waypoints"
// feature). Measured on the v4 sparse-sine bench: RMS deviation 0.090 for the
// stretched Ruckig profile vs 0.005 for a quintic Hermite, at 28 vs 8 peak
// accel. Handing Ruckig perfect boundary conditions (af = source) does not
// change the shape. Hence:
//
//   * WAVEFORM (any point carrying a duration): a HERMITE segment in the
//     client's declared family from the current (p,v,a) to (target,
//     G-velocity, af-estimate) over exactly the commanded duration. Every one
//     is CEILING-SCANNED at plan time; a segment that demands more than
//     vmax/amax/jmax (or leaves the window) is handled by InfeasiblePolicy.
//     Since 0.7.0 a waveform segment may also arrive with ONE-SEGMENT
//     LOOKAHEAD (Command::next_chord), which arms the RFC-008 handoff sanity
//     guard: the sender's end velocity is bounded to the Fritsch-Carlson knot
//     limit of the segment that FOLLOWS, so an infeasible handoff is caught
//     BEFORE it forces the shape/amplitude/deadline sacrifices. See
//     boundHandoffVelocity.
//   * CHASE (bare high-rate points, no duration): the future is unknown ->
//     Ruckig chases the point stream under the ceilings, replanning per point
//     from the sampled state (C2-continuous, retarget-while-moving is the
//     normal case). For DENSE streams (mean interval <= 60 ms) the engine aims
//     one interval AHEAD of the newest point with the estimated stream velocity
//     as the arrival velocity -- chasing the newest stale point by construction
//     lags ~4 intervals (measured). Sparse/isolated points get pure
//     point-chase (no invented velocity).
//   * SETTLE: a trajectory that ends still-moving with no fresh command gets
//     a one-time jerk-limited brake-to-rest via Ruckig's velocity interface.
//     A boundary event, not a clock loop. Guarded by a GRACE WINDOW
//     (Config::settle_grace_us): a stream whose next command is merely a few ms
//     late is not a starved stream, and braking on ms-scale transport jitter is
//     worse than holding (see maybeSettle).
//
// THE TWO AXES AN INFEASIBLE SEGMENT MAY SPEND
// --------------------------------------------
// AMPLITUDE and SHAPE, and nothing else. Duration is spent only by Stretch,
// which is the other contract entirely (see InfeasiblePolicy).
//
// SHAPE is the span's END HANDLE. A span's curve is set by its boundary
// tangent magnitudes; lerping the end handle toward the span's own CHORD SLOPE,
// by a fraction alpha, walks the curve continuously from the sender's spline
// (alpha = 0) toward the flattest shape this span can take:
//     handle >> chord  -> overshoot bulge, peak |v| at the ENDS (measured 36x
//                         the span's mean velocity on a shallow Makima span)
//     handle == chord  -> straight line, peak |v| = 1.0x mean, peak |a| = 0
//     handle == 0      -> smoothstep S-curve, peak |v| = 1.5x mean, mid-span
// so demand falls from EITHER side of the chord, which is what makes it a
// feasibility knob rather than a one-directional fudge.
//
// alpha = 1 IS NOT A LITERAL STRAIGHT LINE. Only the END handle is ours to
// move -- the START is the machine's ACTUAL (v, a), and planning from it is
// doctrine, not a preference. So alpha = 1 means "the straightest curve
// reachable FROM THE STATE THE MACHINE IS ACTUALLY IN". Sufficient anyway: the
// reduction PROPAGATES, because segment N's adopted vf IS segment N+1's actual
// starting v. Do not describe this as reaching linear interpolation.
//
// WHAT THE SHAPE AXIS CAN AND CANNOT BUY. A min-jerk quintic peaks at
// 15/8 = 1.875x its mean velocity; a straight line at 1.0x. So the entire
// shape budget is worth a 1.875x velocity headroom factor and not one unit
// more, because MEAN velocity is |target-p|/T with both terms pinned by the
// sender. Once mean velocity alone exceeds the ceiling, no handle length on
// earth helps and AMPLITUDE must go -- which is why the both-axes-spent case is
// ROUTINE rather than exotic: it is simply "conservative limits + aggressive
// script", the most common infeasible state in the field.
//
// TERMINAL CASE (both budgets spent, still illegal): the Ruckig guard takes the
// segment -- the whole stroke, late, DeadlineStretched. That is the honest
// "your machine cannot do this" answer.
//
// SAFETY / BOUNDS
// ---------------
// Ruckig Community has NO position limits and quintics can bulge, so the
// Engine owns the window:
//   * command targets are clamped to [0,1] at commit;
//   * requested end velocities are clamped so the machine can brake before
//     the wall beyond the target (|vf|² ≤ amax·dist_to_wall, and ≤ vmax);
//   * quintic plans are legality-scanned (v/a/j ceilings AND window bounds)
//     before adoption — illegal shapes fall back to the Ruckig guard;
//   * sampled OUTPUT position is clamped to [0,1]; the arbiter's window clamp
//     remains the hard physical backstop downstream.
// Limits are CEILINGS, never targets (the doctrine, verbatim).
//
// UNITS & TIME
// ------------
//   position : normalized 0..1 across the configured stroke window
//   velocity : normalized units per second (TCode G wire value / 1000)
//   time     : microseconds, uint64, injected by the caller (esp_timer on
//              target, synthetic in native tests — fully deterministic)
// Public API is float; planning math is double (plan-time only — the 1 kHz
// sample path is a polynomial/profile evaluation).
//
// THREADING
// ---------
// Single-threaded by contract: ALL of
// commit/positionAt/velocityAt/snapshot run on Core 1 (the sampler task).
// Cross-core command handoff stays OUTSIDE this class. No locks, no heap in
// steady state (Ruckig's waypoint vectors stay empty in community mode).
//
// Hardware-free: std headers + vendored lib/ruckig only. Native-tested in
// test/native/test_kinetic.
#pragma once

#include <cstdint>
#include <cmath>
#include <optional>
#include <utility>

#include <ruckig/ruckig.hpp>

namespace kinetic {

inline constexpr const char* kVersion = "0.8.0";

// ---- Configuration ----------------------------------------------------------

// Kinematic ceilings in normalized units (per second^n). The firmware glue
// derives these from the mm-domain input limit set / stroke window length.
struct Limits {
    float vmax = 3.0f;     // units/s   (3 = three full strokes per second)
    float amax = 30.0f;    // units/s^2
    float jmax = 500.0f;   // units/s^3
};

// What to sacrifice when the wire commands a move the machine physically
// cannot execute in the commanded duration. This is a statement of INTENT, not
// a tuning knob: both answers are correct, for different senders.
//
//   Stretch - range-first. Keep the whole stroke, overrun the deadline. Right
//             for a sender whose amplitude is the content (manual point moves,
//             "go here" commands), and the only contract that spends DURATION.
//   Blend   - timing-first (DEFAULT). Keep the deadline and spend the two
//             axes an infeasible segment actually has, AMPLITUDE and SHAPE,
//             together in the ratio `infeasible_blend` sets and only as far as
//             legality demands, so a segment 10 % over gives up about 10 % of
//             the ray rather than 100 % of one axis. Right for a SCHEDULED
//             sender (funscript segments over Valence 0x0085): the next
//             segment arrives on its own clock regardless of whether we
//             finished, so an overrun plan is PREEMPTED mid-flight. Measured
//             on the virtual machine (window 500 mm, vmax 1.1, amax 16): a 0->1
//             stroke chain at 400 ms/segment under Stretch achieved only 36 %
//             of the commanded amplitude AND ran phase-lagged -- it loses range
//             and timing both.
//
// THE BUDGETS ARE FLOORS, NOT DECORATION. `infeasible_amplitude_budget` and
// `infeasible_smooth_budget` bound how far down the ray the search may go, and
// the doctrine's amplitude exception (motion-control.md, operator ruling
// 2026-09-02) is written against them: amplitude is the one quantity a ceiling
// may shape, and only this far. A shape still illegal AT the floor is the
// Ruckig guard's honest business, not a reason to keep spending.
//
// ORDINALS ARE PINNED. Stretch is 0 and Blend is 5 because those are the values
// already persisted in NVS and already carried by the device catalog's select;
// the four ordinals between them named policies deleted 2026-09-02 (docs/
// reviews/kinetic-2026-09-02/02-waveform-referee-chain.md section 5) and the
// HOST maps a stored one onto Blend. Renumbering to close the gap would
// silently re-point every stored setting.
enum class InfeasiblePolicy : uint8_t {
    Stretch = 0,   // range-first: keep the full stroke, overrun the deadline
    Blend   = 5,   // timing-first: spend amplitude and shape together, to the budgets
};

// THE ORDINAL OF THE LAST POLICY, and the ONE home for it. Every wire clamp,
// NVS load clamp and name table off the engine is a restatement of this number,
// and restating it is how a policy ships unreachable: 0.9.0 added Blend and left
// four device-side tables plus three bounds at 4, so an operator selecting blend
// (5) was CLAMPED to 4 and silently got a policy they never asked for. Adding a
// policy means bumping this and letting the compiler find the rest -- never
// editing a literal. It is the ORDINAL BOUND, never the policy COUNT: the
// values are sparse (0 and 5), so a table indexed by ordinal is
// kInfeasiblePolicyMax + 1 entries long and the gap resolves to whatever the
// host maps it to.
inline constexpr uint8_t kInfeasiblePolicyMax = (uint8_t)InfeasiblePolicy::Blend;

// Which CURVE FAMILY the waveform path reconstructs a segment with.
//
// WHY THIS EXISTS. A funscript rendered through Pchip or Makima is a C1 CUBIC
// HERMITE spline: those two interpolators differ ONLY in the rule they use to
// pick knot tangents, and given endpoint positions plus endpoint tangents the
// cubic on a span is uniquely determined. So {target, duration, end_vel} on
// channel 0x0085 is not a lossy summary of the sender's curve — for any C1
// cubic-Hermite family it is a COMPLETE ENCODING of it. Nothing is missing from
// the wire; what differs is what we rebuild it WITH.
//
// A C2 quintic CANNOT reproduce a C1 cubic across a knot, by construction: the
// script's acceleration genuinely STEPS there, and a C2 curve is required to
// make curvature continuous. Rounding that step off is not smoothing noise, it
// is deleting script content — and the `af` the quintic needs is not even on the
// wire, so it is estimated as a backward difference of consecutive handoff
// velocities, i.e. an estimate of a quantity that is genuinely TWO-VALUED at the
// knot it is estimated at. The operator identified this on hardware as motion
// that tracked the script's positions but not its character.
//
//   FollowClient — the sender declares its curve family and we honor it. NO
//                  WIRE SIGNALING EXISTS YET (that is the pending RFC), so with
//                  nothing declared this resolves to C2 — i.e. today's behavior,
//                  byte for byte. This value is the safe default and stays that
//                  way; when the RFC lands, only the resolution changes.
//   ForceC1      — always cubic. Reproduces a Pchip/Makima span exactly.
//   ForceC2      — always quintic. The pre-0.8.0 engine.
//
// SCOPE — READ BEFORE MOVING THE CALL SITES. C1 applies to WAVEFORM SEGMENT
// commits only, never to chase points, settle, or the Ruckig guard. A cubic
// takes four boundary conditions (p, v) → (target, vf) and therefore DROPS the
// machine's current acceleration `a`, so a plan begins with an acceleration STEP
// relative to what the carriage is actually doing. At a script knot that step is
// exactly right — the script has one there too. Anywhere else (a preemption
// mid-span, a chase replan, a settle) it would be a spurious torque step with no
// authoring behind it, which is why those paths keep the quintic.
//
// THE STEP IS DELIBERATELY UNBOUNDED. Nothing clamps it, because clamping it is
// precisely "manufacture curvature continuity the script did not ask for" — the
// thing C1 mode exists to stop doing. The jerk ceiling remains a MACHINE SAFETY
// limit and is not repurposed as a shape control. Consequence worth knowing
// before hardware: a knot between two very differently-sloped spans commands a
// large torque step, and how that feels is a measurement, not a derivation.
enum class CurvePolicy : uint8_t {
    FollowClient = 0,   // honor the sender's declared family (RFC-030)
    ForceC1      = 1,   // cubic Hermite — reproduces the script's own spline
    ForceC2      = 2,   // quintic — curvature-continuous
};

// WaveformCommand::client_curve_family's "c1_cubic". Mirrored rather than
// included: this header stays valence-free, so the registry numbering is
// documented (see WaveformCommand) and restated here, never imported.
inline constexpr uint8_t kClientCurveC1Cubic = 1;

// policy + declaration -> reconstruction family, in ONE place. Free and public
// because a caller can need the answer BEFORE commitWaveform adopts the command:
// the bench's sender-curve overlay draws the client's own curve at commit time,
// and an overlay that picks its family by a private re-derivation is how a tuner
// ends up comparing a cubic against a quintic and blaming the planner for the
// difference.
constexpr bool resolveCubic(CurvePolicy policy, uint8_t client_family) {
    switch (policy) {
        case CurvePolicy::ForceC1: return true;
        case CurvePolicy::ForceC2: return false;
        case CurvePolicy::FollowClient: return client_family == kClientCurveC1Cubic;
    }
    return false;
}

struct Config {
    Limits limits{};
    // Chase jerk scales with the MOVE's own demand: a replan corner spends
    // jerk proportional to demand/vmax (kneed at kChaseJerkKneeFrac) instead
    // of the mechanical ceiling, so slow content stops carrying a 50 Hz notch
    // train while fast content keeps full authority. Softer-only by
    // construction (jerkCeil).
    bool  chase_jerk_scale = true;
    float chase_jerk_floor = 0.15f;   // never below this fraction of jmax
    // Cold-start governor. The FIRST plan out of rest (Idle/Settle, v~0) is a
    // POSITIONING move -- park to the stream's opening position -- not
    // content; planned at limits.vmax it shoots across the window (the
    // script-start dart, sd-d77). That one commit clamps vmax here; the
    // deadline guards then stretch duration instead. 0 = disabled.
    float recovery_vmax = 0.0f;

    // CHASE feedforward + predictive aim (dense streams only, see gate
    // below): the engine differentiates the incoming point stream, aims
    // chase_lookahead intervals ahead of the newest point, and asks Ruckig to
    // arrive there AT the estimated stream velocity. Tuned on the bench
    // graphs — chasing the stale newest point with damped arrival velocity
    // measured ~4 intervals of lag.
    bool     chase_feedforward = true;
    float    chase_ff_gain     = 0.9f;    // damping on the velocity estimate
    // Intervals of predictive aim: enough lead to cancel the plan's own
    // tracking lag and no more. 1.3 is the P4 bench value (bd val-091.14) --
    // the aim then leads ~25 ms at a 20 ms cadence against a measured 23-27 ms
    // of tracking lag. The older 3.0 was swept on the S3, where clumpy
    // arrivals disabled the feedforward often enough to hide how much lead it
    // really buys.
    float    chase_lookahead   = 1.3f;
    // Arrive matching the stream's estimated CURVATURE too (target accel).
    // With af forced to 0 every chase plan is a long flatten-out tail that a
    // curving stream preempts forever — chronic lag (bench-measured).
    bool     chase_accel_ff    = true;
    // Second-order predictive aim. The first-order aim (target + v_est·look)
    // extrapolates along a STRAIGHT line, which is exactly wrong where a
    // waveform turns: approaching a crest the velocity EMA still reads the
    // pre-crest positive velocity, so the aim is thrown PAST the crest.
    // Measured against a source sine cresting at the rail: aim overshot the
    // ideal crest by +0.022, clamped to 1.0, and applyEndVelGuard then saw
    // dist-to-wall = 0 → vf forced to 0 → the carriage PARKED at the rail,
    // dead stop, 127–138 ms per stroke (the source sine's own tangency dwell
    // is 0.35 % — ~8x excess flat time), plus clustered Ruckig PlanFailed
    // drops through the approach. Adding the ½·a_est·look² term pulls the aim
    // back exactly where the stream is turning, because a_est is negative
    // there. Uses the SAME acap-limited estimate as the af feedforward so a
    // noisy second difference cannot fling the aim.
    // This flag gates the WHOLE second-order idea ("predictive aim v2"): the
    // ½·a_est·look² aim term AND the matching arrival-velocity extrapolation
    // (arrive at v_est + a_est·look, the velocity the stream will have when we
    // get there — not the one it has now). Off = pre-v2 behavior, byte for
    // byte.
    bool     chase_aim_accel_extrap = true;
    // Streams with mean interval above this are NOT dense: no feedforward, no
    // extrapolation — isolated points plan a plain point-chase to rest.
    // (The legacy LIVE_TRIGGER idea, relaxed to admit 20 Hz app streams.)
    uint32_t chase_dense_us    = 60000;
    // Estimator resets after a stream gap this long.
    uint32_t chase_stale_us    = 400000;

    // ---- Infeasible-segment handling (WAVEFORM path only) -------------------
    // Blend by measurement: the same decision the retired corner policies made
    // discretely, made continuously instead. On the async-tune bench (GoogleCat,
    // 50-150 mm window, curve follow -> c1, 1000 mm/s / 50000 mm/s2) Blend at
    // its best setting beat the best corner policy on the same objective
    // (regret 0.226 vs 0.248) and, unlike Stretch, did not overrun the deadline
    // to do it. Stretch remains selectable because its contract is different,
    // not because it scores better.
    InfeasiblePolicy infeasible_policy = InfeasiblePolicy::Blend;

    // Curve family for waveform-segment reconstruction. FollowClient is today's
    // behavior byte for byte until the curve_family wire signaling lands.
    CurvePolicy curve_policy = CurvePolicy::FollowClient;

    // ---- The two spend budgets (Blend) --------------------------------------
    // THE END OF THE RAY, in each axis. Both are FRACTIONS in [0, 1], clamped
    // on use (a config push is not a trusted input), and the search probes AT
    // this corner: a shape still illegal there falls to the Ruckig guard rather
    // than spending further.
    //
    // infeasible_smooth_budget — max alpha: how far the end handle may be
    //   lerped toward the chord slope. 0 = never touch the sender's curve
    //   (degenerates to pure amplitude spending); 1 = the flattest curve
    //   reachable from the machine's actual state, which is NOT the same thing
    //   as a straight line (see InfeasiblePolicy). 0.5 is the operator's
    //   Blender-derived starting point: halving the handle length was enough to
    //   pull a bulged span back parallel to its own chord.
    //
    // infeasible_amplitude_budget — max fraction of the COMMANDED stroke that
    //   may be surrendered. 0.5 = may shrink to the segment midpoint (the
    //   midpoint-anchored geometry's f = 0); 1.0 = may decline to move at all.
    //   Kept at 0.5 by default because a stroke shortened past its own midpoint
    //   has stopped being the motion the script described.
    float infeasible_smooth_budget    = 0.5f;
    float infeasible_amplitude_budget = 0.5f;
    // Bisection depth on the ray. Each step costs ONE curve build + one legality
    // scan and no Ruckig call, which is what makes this the cheap plan-time
    // dial. CLAMPED to [1, 10] on use. 6 resolves the sacrifice scalar to 1/64,
    // well under anything perceptible.
    uint8_t infeasible_blend_steps = 6;

    // ---- InfeasiblePolicy::Blend — the one slider ---------------------------
    // WHAT AN INFEASIBLE SEGMENT GIVES UP, as a single exchange rate:
    //   0.0  keep AMPLITUDE nothing, keep SHAPE everything — surrender reach
    //   1.0  keep AMPLITUDE everything, surrender shape (flattens toward the chord)
    //   0.5  both give equally
    // Clamped [0, 1] on use; a config push is not a trusted input.
    //
    // 0.5 BY MEASUREMENT, and the honest version of that claim: swept over 10
    // recordings x 6 perturbations (window tight/wide/offset, halved speed,
    // weakened accel) = 60 cases, 42 of which actually exercise the policy.
    // Scored as scale-free per-case regret on the operator's own objective —
    // shape match AND amplitude, (1-shape_corr) and |1-reach_ratio|.
    //
    // THE OPTIMUM MOVES WITH THE EXCHANGE RATE, which is exactly why this is a
    // slider and not a constant:
    //   shape weighted ~10x reach -> 0.875 (regret 0.236)
    //   weighted EQUALLY          -> 0.5   (regret 0.239)
    //   reach weighted >= shape   -> 0.5   (regret 0.197 / 0.110)
    // 0.5 is the equal-weight optimum and wins 3 of the 5 weightings tried, so
    // it is the neutral default; 0.875 is the pick if shape matters much more.
    //
    // WHAT IS NOT AMBIGUOUS: the low end is wrong. blend <= 0.25 scores regret
    // ~0.72 against ~0.24 at the top — surrendering AMPLITUDE first is a bad
    // trade almost everywhere, because a small shape concession keeps the plan
    // FEASIBLE while lost reach is both visible and often still falls through to
    // the Ruckig guard. The two cases where 0.0 won had a spread of 0.17, i.e.
    // they were ties.
    float infeasible_blend = 0.5f;

    // ---- Handoff sanity guard (RFC-008; WAVEFORM path only) -----------------
    // Fritsch-Carlson chord factor `k` for the one-segment-lookahead bound on
    // an explicit wire end velocity (see boundHandoffVelocity below for the
    // derivation and the measured pathology it exists for).
    //
    // 1.5 is not a fudge: capping BOTH endpoint tangents of a span at
    // k*|chord| makes the Fritsch-Carlson sum condition alpha + beta <= 2k,
    // and 2k <= 3 is the classic sufficient condition for a shape-preserving
    // (non-overshooting) Hermite — exactly at k = 1.5. Raise toward 3.0 for
    // the looser per-tangent box if the machine ends up flatter than the
    // sender through long shallow runs; this is the ONE knob for handoff
    // aggressiveness.
    //
    // 0 DISABLES the guard entirely (pre-0.7.0 behavior, byte for byte) —
    // that is the A/B switch for comparing machine-side bounding against a
    // client that still carries its own limiter. CLAMPED to [0, 8] on use: a
    // config push is not a trusted input.
    //
    // The guard only ever engages when the CALLER supplies a lookahead
    // (Command::has_next_chord). An engine fed no lookahead behaves exactly as
    // it did before this knob existed, whatever k says.
    float handoff_chord_factor = 1.5f;

    // ---- DWELL SPAN (the dwell rule, see commitWaveform) --------------------
    // Normalized distance under which a timed segment re-commanding the
    // previous segment's target is a HOLD. A protocol value, so this header
    // never spells it: the glue injects it (Valence: the registry's
    // limits.segment_dwell_span, SPEC 9.6). 0 disables the rule.
    float dwell_span_norm = 0.0f;

    // ---- OVERSHOOT GUARD (option A of the 2026-07-30 bench shoot-out) -------
    // The legality scan constrains v/a/j and the WINDOW. It has never
    // constrained "did this plan sail past the endpoint the sender asked for",
    // so a plan may arc far beyond the commanded target and still be legal as
    // long as it stays inside the window.
    //
    // MEASURED (GoogleCat, t=28.168): the machine sits at 186.8 mm doing
    // +1000 mm/s (at vmax) when a segment says "be at 100 mm — 86.8 mm BELOW —
    // in 875 ms". The C1 cubic that satisfies those four boundary conditions
    // over exactly that duration peaks at 305.3 mm: 118 mm past the target,
    // SEVEN TIMES the 16.7 mm a full-authority brake would have cost. Momentum
    // is not the cause — the polynomial is, because a fixed duration plus fixed
    // endpoints uniquely determines the shape and it must SPEND those 875 ms.
    //
    // The allowance is therefore physical, not a taste knob: a plan may pass its
    // own endpoint by as much as STOPPING THERE WOULD CARRY IT ANYWAY, and no
    // further. Overshoot that momentum forces is honest; overshoot the polynomial
    // invented is not.
    //
    // "WOULD CARRY IT ANYWAY" IS MEASURED, NOT A FORMULA — see
    // physicalBandExcess(). The 0.9.0 guard used the closed form v0^2/(2*amax),
    // which ignores the JERK ceiling, and jerk is what dominates a hard stop:
    // braking from 800 mm/s at amax 50 000 mm/s^2 costs 6.4 mm on paper and
    // ~15 mm in fact, because reaching full deceleration takes 25 ms at jmax and
    // the carriage covers 20 mm getting there. A guard built on the paper number
    // is ~2.3x too strict — it rejected plans that were already near-physical and
    // handed them to the flat Ruckig fallback, which is exactly how the knob
    // measured WORSE THAN OFF on OvershootTestThrobbing (mean excursion 4.63 ->
    // 7.40 mm) and non-monotone in its own value. Both were this.
    //
    // 0 disables (pre-guard behavior, byte for byte). Values > 1 are a slack
    // multiplier on the physical floor for operators who want the shape back.
    float overshoot_guard = 1.0f;

    // Slack ON TOP of the physical floor, as a fraction of the segment's own
    // commanded chord. THIS IS WHAT MAKES THE GUARD SELECTIVE, and without it the
    // guard is not worth having.
    //
    // The physical floor alone is an ABSOLUTE bound, so it fires on every stroke
    // whose excursion exceeds it — including strokes that overshoot by 2 % of
    // their own travel, which nobody can feel and which the ceiling scan was
    // right to pass. Each of those rejections buys a straight line, because the
    // Ruckig fallback cruises at vmax on anything near saturation. That is how a
    // correct bound still produced the operator's "some strokes go suddenly
    // linear": measured on GoogleCat at the operator's window, slack 0 took the
    // fallback on 55 segments and pushed flattening 12.2 % -> 25.7 % to remove an
    // excursion of 7.4 mm that was 0.47x its own stroke, i.e. not a defect.
    //
    // Throbbing is a RATIO, not a distance — the complaint is a move that travels
    // further past its target than the move itself was long. Allowing a quarter
    // of the chord on top of the physical floor says exactly that. Measured
    // against slack 0 (2026-07-30, 12 recordings, window 150-350, guard 1, the
    // C1 family the machine actually ships): InterpTest1 flattening
    // 35.7 % -> 5.1 % with 22 fallbacks -> 0, GoogleCat 23.8 -> 20.8 % and
    // 13 -> 2, SYN-truncated 30.8 -> 23.8 % and 9 -> 0 — all for about 0.5 mm of
    // worst-case excursion. The absolute bound was spending a straight line to
    // remove excursions of 0.03x their own stroke.
    //
    // 0 restores the absolute-only bound. The whole guard disarms at
    // overshoot_guard 0 regardless of this value.
    float overshoot_chord_slack = 0.25f;

    // ---- Settle grace (see maybeSettle) -------------------------------------
    // How long an expired plan may HOLD its end state before the engine
    // concludes the stream is starved and brakes to rest. Sized as
    // min(1.5 · estimated stream interval, this cap) and only applied while a
    // recent stream estimate exists — an isolated point move still settles
    // immediately. 0 disables the grace (pre-0.4 behavior: brake the instant
    // the plan expires).
    uint32_t settle_grace_us = 30000;
};

// ---- Command (POD, queue-safe — the InterpSegment successor) ----------------
struct Command {
    float    target       = 0.5f;   // normalized 0..1
    uint32_t duration_us  = 0;      // I<ms> * 1000 when present
    float    end_vel      = 0.0f;   // units/s — TCode G wire value / 1000
    bool     has_end_vel  = false;  // true → v4 gradient handoff velocity
    bool     has_duration = false;  // true → WAVEFORM, at any duration

    // ---- ONE-SEGMENT LOOKAHEAD (RFC-008 handoff sanity guard) ---------------
    // The mean speed (|Δtarget| / duration, same normalized units/s as
    // end_vel) of the segment that FOLLOWS this one, when the caller already
    // holds it. The firmware's Valence pacing ring schedules 0x0085 segments
    // ~120 ms ahead of their start, so at the moment a segment is handed to
    // the engine its successor is frequently already queued — this is that
    // knowledge, and nothing else. It is NOT a command, it never plans
    // anything, and it is a magnitude (never signed): it exists solely to
    // bound end_vel at the knot the two segments share.
    //
    // false = "no successor is known" (tail of a stream, a durationless chase
    // point next, or a caller with no lookahead at all) → the guard does not
    // engage and the command behaves exactly as it did pre-0.7.0. Guessing a
    // chord we do not have would trim well-behaved senders for free, which is
    // a feel regression; not guessing costs nothing, because an unbounded
    // handoff still meets the legality scan + Ruckig guard downstream.
    // RFC-049c evaluated and REJECTED an own-chord fallback here — see
    // commitWaveform's note beside boundHandoffVelocity's call site.
    float    next_chord     = 0.0f;
    bool     has_next_chord = false;

    // ---- Anchor (anchored commit) --------------------------------------------
    // The command's time in the engine's own clock domain. Its meaning follows
    // the command kind (SPEC 5.4, RFC-084): a DURATION-carrying segment STARTS
    // at its anchor; a bare chase point is REACHED at its anchor (its arrival
    // time), so a future-anchored point is planned to arrive there from where
    // the previous point arrives. Either way release jitter never becomes
    // rendered geometry. false = plan at arrival (pre-0.9 behavior).
    uint64_t anchor_us  = 0;
    bool     has_anchor = false;

    // ---- RFC-030: the sender's DECLARED curve family ------------------------
    // Values mirror the Valence registry's `curve_families` table verbatim
    // (this header stays valence-free, so the numbering is documented, not
    // included): 0 = unspecified, 1 = c1_cubic, 2 = c2_quintic, 3 = step.
    // Consumed ONLY by the waveform path's FollowClient resolution — 1 selects
    // the cubic reconstruction, everything else keeps the quintic (today's
    // behavior, including `step`, honestly: no step renderer exists yet).
    // Callers with no wire knowledge leave it 0 and nothing changes.
    uint8_t  client_curve_family = 0;
};

// ---- RFC-008 handoff sanity guard -------------------------------------------
// "The machine plans for the worst so clients don't have to."
//
// THE FAILURE THIS EXISTS FOR (measured live, MFP plugin v0.2.1-0.2.3 against
// Valence Sim, 2026-07-25): a funscript axis on Makima interpolation produced a
// handoff velocity of 1.816 norm/s into a span whose own mean velocity is
// 0.050 norm/s — 36x over. The client was computing a MATHEMATICALLY CORRECT
// spline tangent; there is simply no monotone quintic that covers 0.050 of
// span-mean displacement while ARRIVING at 1.816. The legality scan rejected
// the quintic, the Ruckig guard took the segment, and a "slow, simple" script
// rendered as straight-line strokes with a flat-topped velocity trace. Worse,
// the af backward-difference estimator then carried the oversized value into
// the NEXT segment's boundary conditions.
//
// THE CRITICAL DETAIL: bounding end_vel against the CURRENT segment's own
// chord is NOT sufficient. 1.816 was entirely sane relative to its own span
// (whose chord was ~3.0 norm/s) and absurd only relative to the NEXT one.
// A guard that cannot see forward cannot catch this class at all.
//
// THE BOUND: the handoff lives at the KNOT shared by two segments, so it must
// be feasible for BOTH —
//
//     |end_vel|  <=  k * min(|chord_in|, |chord_out|)
//
// which is the Fritsch-Carlson knot limiter (the same bound the MFP plugin
// currently carries client-side, applied where RFC-008 says it belongs). Sign
// is preserved: a wrong-signed handoff stays wrong-signed but bounded, so the
// ONLY thing this changes is pathological SIZE. A zero chord on either side (a
// plateau, or "the target IS where the next segment ends") forces 0 — arriving
// at a hold still moving is exactly the kind of lie the Ground Truth doctrine
// exists to prevent.
//
// WHY min AND NOT chord_out ALONE: min() is strictly stronger, it satisfies
// "never exceeds the following chord's limit" by construction, and it stays
// monotone in |chord_out| (min(a, ·) is non-decreasing). It also costs nothing
// to compute — chord_in is |target - p| / T, which the engine already knows
// exactly, from the machine's ACTUAL position rather than a sender's guess.
//
// IN-BOUNDS INPUT IS RETURNED BIT-FOR-BIT UNCHANGED. That is a hard contract,
// not an implementation detail: a guard that perturbs a well-behaved client's
// handoff is a feel regression for every well-behaved client, which is a worse
// bug than the one it was written to fix.
//
// `chord_in` / `chord_out` are non-negative magnitudes in units/s. A NEGATIVE
// value means "unknown" and disables the bound (never bound on a guess); so
// does k <= 0, which is the guard's off switch. Non-finite anything is passed
// through — Engine::commit() rejects non-finite command inputs upstream, and a
// numeric guard is not the place to relitigate that.
inline float boundHandoffVelocity(float end_vel, float chord_in, float chord_out,
                                  float k) noexcept {
    if (!(k > 0.0f)) return end_vel;                        // guard disabled
    if (k > 8.0f) k = 8.0f;                                 // config push is untrusted
    if (!std::isfinite(end_vel) || !std::isfinite(chord_in) ||
        !std::isfinite(chord_out)) return end_vel;
    if (chord_in < 0.0f || chord_out < 0.0f) return end_vel; // "unknown" side
    const float limit = k * (chord_in < chord_out ? chord_in : chord_out);
    const float mag   = end_vel < 0.0f ? -end_vel : end_vel;
    if (mag <= limit) return end_vel;                        // UNTOUCHED
    return end_vel < 0.0f ? -limit : limit;
}

enum class Mode : uint8_t {
    Idle     = 0,   // holding position, no planned motion
    Waveform = 1,   // executing a quintic v4 segment (or its Ruckig fallback)
    Chase    = 2,   // tracking a bare point stream
    Settle   = 3    // braking to rest: stream starvation, or brake() (a STOP)
};

// APPEND-ONLY (wire-visible: 0x0081 `plan_kind` is a select field whose option
// list is indexed by this enum, and SystemState::sm_plan_kind mirrors it).
//
// Quintic and Cubic are BOTH Hermite polynomials sharing one evaluator — a cubic
// is stored as a quintic with c[4] = c[5] = 0. They are separate VALUES rather
// than one "hermite" value because the operator has to be able to see which
// family actually ran; a readout saying "quintic" while a cubic executes is a
// ground-truth defect, and it cost a live debugging session to notice.
//
// ANY new Hermite family MUST be added to isHermite() in the same change. Every
// evaluation site branches "Hermite → quinticAt, else → Ruckig trajectory", so a
// kind that is Hermite in spirit but missing from that predicate does not fail
// loudly — it silently evaluates as a Ruckig profile that was never planned.
enum class PlanKind : uint8_t { None = 0, Quintic = 1, Ruckig = 2, Cubic = 3 };

// ---- Anomaly instrumentation (same drain pattern as the cubic engine) -------
enum class AnomalyType : uint8_t {
    None              = 0,
    // Plan rejected; previous plan kept. detail = (float)ruckig::Result, or a
    // local sentinel: -99 non-finite command input, -98 non-finite trajectory
    // duration, -97 anchor beyond the scheduled-lead bound, -96 schedule queue
    // full (kScheduleDepth anchored plans already parked).
    PlanFailed        = 1,
    SettleEngaged     = 2,  // stream starved mid-glide → brake plan. detail = end velocity
    EndVelClamped     = 3,  // requested vf cut by wall/vmax guard. detail = clamped vf
    DeadlineStretched = 4,  // commanded duration infeasible; guard profile runs longer. detail = actual s
    WaveformFallback  = 5,  // quintic broke a ceiling/window → Ruckig shaped the segment instead. detail = worst ratio
    WaveformScaled    = 6,  // the stroke was shrunk to hold the deadline. detail = achieved fraction 0..1, target = the shortened target
    WaveformCentered   = 7,  // RETIRED 2026-09-02 with the centering control law. NEVER EMITTED; the value is held so the counter tables stay index-aligned
    HandoffBounded    = 8,  // RFC-008: a wire end velocity was cut to the Fritsch-Carlson knot bound of the FOLLOWING segment. detail = the accepted (bounded) vf
    WaveformSmoothed  = 9,  // the span's END handle was lerped toward the chord to make the shape legal. detail = alpha spent, 0..1 (1 = straight line)
    DwellZeroed       = 10  // the SAME target was re-commanded (a hold) so its declared arrival velocity was zeroed. detail = the vf that was dropped
};
// APPEND-ONLY. Existing values are pinned: the firmware (SystemState::
// sm_anom_kind + kSmAnomalyNames) and the sim (MachineSim.h mirror) index
// per-kind counter tables by this enum, and their drain loops bounds-check
// against the NAME table — a kind with no name there is dropped, not
// miscounted. A RETIRED kind keeps its value forever (WaveformCentered = 7):
// the tables are indexed by ordinal, so closing the gap would re-point every
// counter after it.
// HandoffBounded = 8 SPENT that width: SM_ANOM_KINDS went 8 -> 9 in the same
// change, together with kSmAnomalyNames, the sim's mirror of it, and the
// per-kind field list on the 0x0088 kinetic-diag channel.
// WaveformSmoothed = 9 SPENT the next slot: SM_ANOM_KINDS went 9 -> 10, same
// three-place update (names, sim mirror, 0x0088 field list).
// DwellZeroed = 10 SPENT the next: SM_ANOM_KINDS went 10 -> 11, same three
// places again.

// ANOMALY VOCABULARY FOR THE INFEASIBLE PATHS (one event per AXIS spent, so
// the counts read as a diagnosis rather than a pile):
//   Stretch : WaveformFallback (+ DeadlineStretched) — shape AND deadline lost.
//   Blend   : WaveformSmoothed when the end handle was lerped (detail = alpha),
//             WaveformScaled when the stroke was shortened (detail = the
//             achieved fraction). Independent: a segment may report one, the
//             other, or both, and it reports exactly what it spent.
//             WaveformFallback means the search could not make the shape legal
//             even at the budget floor and the Ruckig guard took the segment.
//   Dwell   : DwellZeroed — a re-commanded hold's declared arrival velocity was
//             dropped. Its own kind because it is a different referee from the
//             RFC-008 bound, which is the kind it used to borrow.
//   Handoff : HandoffBounded — the SENDER'S HANDOFF was reshaped, one segment
//             BEFORE any of the above could happen. This is the only kind on
//             this list that reports a preventive act rather than a rescue,
//             and it is deliberately loud for that reason: it means the client
//             is asking for a knot velocity its own next segment cannot
//             absorb, and the operator whose script is fighting the planner
//             should be able to find that out. A well-behaved sender never
//             produces one. It can legitimately co-occur with the rescue kinds
//             on the SAME segment: bounding a handoff is not a promise that the
//             rest of the segment fits.

struct Anomaly {
    uint8_t  kind   = 0;      // AnomalyType
    uint16_t seq    = 0;      // rolling event id
    uint64_t t_us   = 0;      // engine time at record
    float    target = 0.0f;   // command target 0..1
    float    detail = 0.0f;   // kind-specific (see AnomalyType)
};

// ---- Telemetry snapshot (WebUI planned-path overlay feed) -------------------
struct Snapshot {
    float    pos        = 0.5f;
    float    vel        = 0.0f;   // units/s
    float    acc        = 0.0f;   // units/s^2
    float    start      = 0.5f;   // active plan's start position
    float    target     = 0.5f;   // active plan's end position
    float    duration_s = 0.0f;   // active plan duration (0 = holding)
    float    elapsed_s  = 0.0f;
    uint8_t  mode       = 0;      // Mode
    uint8_t  plan_kind  = 0;      // PlanKind of the active plan
    uint32_t plans      = 0;      // successful plans since reset
    uint32_t failures   = 0;      // PlanFailed count since reset
    // SHARPNESS of the active plan: its PEAK JERK as a fraction of
    // Limits::jmax. Lower = rounder. For a Ruckig plan this is exactly the
    // ceiling it was planned under (Ruckig's profiles are bang-bang in jerk);
    // for a Hermite it is the scanned peak of its own shape, so the field means
    // the same thing on both plan kinds instead of being a policy artifact.
    //
    // NOT monotone across the Hermite/Ruckig boundary, and that is real, not a
    // bug: a velocity-saturated Ruckig double-S at its critical jerk can be
    // GENTLER in jerk than the Hermite that was just rejected for exceeding
    // vmax (measured: Hermite peak j/jmax 0.13 -> 0.083 on the same segment).
    float    sharpness  = 1.0f;
};

// ---- The plan as DATA -------------------------------------------------------
// One piece of a plan: everything a caller needs to evaluate it and nothing
// about the engine that produced it. This is what lets a SECOND CORE render the
// plan with no engine of its own to mutate.
// Constraints:
// - `traj` is BORROWED. It points into the Engine and is valid only until the
//   next call that changes the plan. Evaluate and drop it; never a member.
// - Evaluate through Engine::evalPiece, the one home for the past-expiry coast
//   rule. A renderer that reimplements it will disagree with the engine.
struct PlanPiece {
    PlanKind kind       = PlanKind::None;
    uint64_t start_us   = 0;    // this piece's time origin
    double   duration_s = 0.0;
    double   c[6]       = {};   // Hermite kinds: quintic in normalized tau
    double   T          = 0.0;  // Hermite kinds: duration, seconds
    const ruckig::Trajectory<1>* traj = nullptr;   // PlanKind::Ruckig only
    double   hold       = 0.0;  // PlanKind::None: the position held
};

// The plan in flight, its NEXT scheduled successor, and the bounds the sampler
// applies to both. `next` is meaningful only while `next_ok`, and it takes over
// at its own `start_us` -- the same instant the engine promotes it.
// CONSTRAINT: the view carries ONE successor even when several are queued
// (`queued` says how many), because a renderer needs the next boundary and
// nothing past it. A renderer whose horizon can span two anchors must
// republish on promotion rather than read further ahead here.
struct PlanView {
    PlanPiece active;
    PlanPiece next;
    bool      next_ok        = false;
    uint8_t   queued         = 0;     // anchored plans parked, `next` included
    double    lo             = 0.0;   // window backstop, sampleClampedNoSettle
    double    hi             = 1.0;
    double    coast_cap_s    = 0.0;   // past-expiry coast bounds, see evalPiece
    double    coast_max_norm = 0.0;
};

// ---- Engine -----------------------------------------------------------------
class Engine {
public:
    // ---- Sender-curve reconstruction (ANALYZER / tooling API) ---------------
    // Rebuild the curve a SENDER described, from the SENDER'S OWN boundary
    // conditions instead of the machine's live state.
    //
    // This is deliberately NOT how the engine plans. The engine always plans
    // from the machine's actual (p, v, a) — that is the motion doctrine and it
    // is not negotiable. These helpers exist so a diagnostic tool can draw
    // three lines at once: what the client ASKED for, what was PLANNED, and
    // what the machine DID. The gap between the first two is exactly the
    // infeasibility, which is otherwise invisible.
    //
    // They call the same two builders the waveform path uses, so the reference
    // line cannot drift from the thing it is a reference for.
    static void senderCurve(bool cubic, double p, double v, double a,
                            double target, double vf, double af, double T,
                            double* c) {
        if (cubic) buildCubic(p, v, target, vf, T, c);
        else       buildQuintic(p, v, a, target, vf, af, T, c);
    }
    // Evaluate such a curve at normalized tau ∈ [0,1]; derivatives are
    // real-time (per second), matching Snapshot's units. Same algebra as the
    // engine's own quinticAt — a cubic simply carries c[4] = c[5] = 0.
    static void evalCurve(const double* c, double T, double tau,
                          double& p, double& v, double& a) {
        p = ((((c[5]*tau + c[4])*tau + c[3])*tau + c[2])*tau + c[1])*tau + c[0];
        v = ((((5*c[5]*tau + 4*c[4])*tau + 3*c[3])*tau + 2*c[2])*tau + c[1]) / T;
        a = (((20*c[5]*tau + 12*c[4])*tau + 6*c[3])*tau + 2*c[2]) / (T * T);
    }

    explicit Engine(const Config& cfg = {}, float start_pos = 0.5f)
        : _cfg(cfg) {
        resetAt(start_pos, 0);
    }

    // ---- Lifecycle ----------------------------------------------------------
    // Hard-reset to a static hold at `pos`. Used on home/estop/resume/stream
    // rising-edge (seed at the machine's actual position).
    // CONSTRAINT: a reset voids the PLAN and the PIPELINE, never the STREAM.
    // The re-seed door fires precisely because the stream is alive, so the
    // cadence estimate survives (it is a property of the sender, not of the
    // plan) and `_reset_cold` carries the one thing a re-seed does mean: the
    // next plan is a cold start. See "One activity clock",
    // .claude/rules/motion-control.md.
    void resetAt(float pos, uint64_t now_us) {
        // THE SEED IS HONEST, in or out of the window (sd-6b2.10): clamping it
        // does not make the state safe, it makes it false, and every entry plan
        // then starts from a position the carriage is not at. The window is
        // enforced by the referees and the output clamp, both of which judge
        // against the plan's own entry.
        _hold_pos   = (double)pos;
        _seed_lo    = windowLo(_hold_pos);
        _seed_hi    = windowHi(_hold_pos);
        _mode       = Mode::Idle;
        _kind       = PlanKind::None;
        _sched_n    = 0;
        _plan_start = now_us;
        _prev_vf_ok = false;
        // The dwell rule compares against the PREVIOUS segment's target. A
        // reset may move the frame that target is normalized in, so it is
        // dropped here; reseedAt() is the at-rest door that keeps it.
        _prev_wave_tgt_ok = false;
        _chase_arrive_ok = false;
        _plan_jerk_frac = 1.0f;
        _plans      = 0;
        _failures   = 0;
        _last_activity_us = now_us;   // the seed itself is activity
        _reset_cold = true;
        _plan_lim   = _cfg.limits;
    }

    // resetAt() for a carriage that merely came to REST in an unchanged frame:
    // the position is refreshed, and the dwell rule's previous target is KEPT
    // (SPEC 9.6 keys it on the previous accepted target on the same source,
    // with no reset clause). A hold re-sent after the machine stopped is
    // still a hold; forgetting the target here let its stale tangent whip
    // through it (val-091.59). Never for a frame move, estop or power loss.
    void reseedAt(float pos, uint64_t now_us) {
        const bool   tgt_ok = _prev_wave_tgt_ok;
        const double tgt    = _prev_wave_tgt;
        resetAt(pos, now_us);
        _prev_wave_tgt_ok = tgt_ok;
        _prev_wave_tgt    = tgt;
    }

    // Ceiling updates take effect at the NEXT plan (an in-flight trajectory
    // is an immutable polynomial planned under the limits of its time).
    // `_plan_lim` follows so the public referees judge against the configured
    // ceilings between commits; commit() re-derives it per plan.
    void setLimits(const Limits& l) { _cfg.limits = l; _plan_lim = l; }
    const Config& config() const { return _cfg; }
    void setChaseFeedforward(bool on, float gain) {
        _cfg.chase_feedforward = on;
        _cfg.chase_ff_gain     = gain;
    }
    // Wholesale live-tuning update (firmware pushes the WebUI/API-tuned
    // config every sampler tick — same-core with commit(), no lock needed).
    void setConfig(const Config& c) { _cfg = c; _plan_lim = c.limits; }

    // ---- Command entry (Core 1, after queue drain) --------------------------
    // Plan a new trajectory NOW from the current sampled state. Returns false
    // if the input was rejected (previous plan keeps executing).
    bool commit(const Command& cmd, uint64_t now_us) {
        // Input guard: a non-finite target/velocity is an upstream parser bug —
        // reject HERE, deterministically. detail -99 marks the local guard.
        if (!std::isfinite(cmd.target) || !std::isfinite(cmd.end_vel)) {
            _failures++;
            recordAnomaly(AnomalyType::PlanFailed, 0.0f, -99.0f, now_us);
            return false;
        }
        // A queued plan whose anchor has passed IS the plan in flight, so the
        // queue is promoted before anything below reads the active plan.
        promoteDue(now_us);

        // Anchor at the command's SCHEDULED start when the caller carries one
        // (Command::anchor_us): the engine executes the wire timeline, not the
        // arrival timeline, so release jitter never becomes rendered geometry
        // (sd-ar3 chord-join notch). kAnchorMaxLateUs < kCoastCapS keeps the
        // sampled coast state uncapped inside the bound.
        //
        // A FUTURE anchor is a SCHEDULED PLAN, planned now from the state the
        // machine will have at that instant and parked in the schedule queue
        // until it starts (architecture.md section 2: intents cross the link
        // with anchor times precisely so the motion processor evaluates them
        // ahead of time). That state is the END of the last plan queued before
        // it, so a whole client lookahead is planned in arrival order without
        // any of it waiting on the machine.
        //
        // A FUTURE-anchored CHASE point is an ARRIVAL (RFC-084): its plan
        // starts where the previous future point arrives (or now, if that has
        // passed) and is stretched to reach the point AT its anchor, so a
        // client's lead buys on-time arrival and never delays the chase.
        uint64_t t0 = now_us;
        bool sched  = false;
        bool arrive = false;
        if (cmd.has_anchor && cmd.anchor_us < now_us) {
            t0 = cmd.anchor_us;
            if (now_us - t0 > kAnchorMaxLateUs) t0 = now_us - kAnchorMaxLateUs;
        } else if (cmd.has_anchor && cmd.anchor_us > now_us) {
            if (cmd.anchor_us - now_us > kAnchorMaxLeadUs) {
                _failures++;
                recordAnomaly(AnomalyType::PlanFailed, cmd.target,
                              kDetailAnchorLead, now_us);
                return false;
            }
            t0     = cmd.anchor_us;
            arrive = !cmd.has_duration;
            if (arrive) {
                t0 = (_chase_arrive_ok && _chase_arrive_us > now_us &&
                      _chase_arrive_us < cmd.anchor_us)
                         ? _chase_arrive_us : now_us;
            }
            sched = t0 > now_us;
        }
        // The point's own time: the estimator's cadence reads it, never the
        // plan start an arrival chose.
        const uint64_t t_pt = arrive ? cmd.anchor_us : t0;

        // A command OWNS its anchor onward: every queued plan starting at or
        // after t0 was planned from a state that no longer holds. Dropped
        // BEFORE the state is read, so an equal anchor is a replacement and an
        // immediate command clears the queue.
        dropScheduledFrom(t0);
        if (sched && _sched_n == kScheduleDepth) {
            _failures++;
            recordAnomaly(AnomalyType::PlanFailed, cmd.target,
                          kDetailScheduleFull, now_us);
            return false;
        }

        // Predecessor state: the queue tail's end when queued, else the plan in
        // flight.
        double p, v, a;
        sampleScheduleTail(t0, p, v, a);

        const double target = clamp01(cmd.target);
        // Estimator feeds BOTH planners, on anchored time (due spacing is the
        // stream's true cadence, arrival spacing carries the jitter). A
        // duration-carrying segment's rate is its own chord over its own span:
        // its target is where the machine will be at anchor + T, so anchor
        // spacing is not that segment's rate and reading it as one poisons
        // every consumer whenever two anchors sit close together.
        const double seg_T = cmd.has_duration
                                 ? (double)cmd.duration_us * 1e-6 : 0.0;
        updateEstimator(target, t_pt,
                        seg_T > 0.0 ? (target - p) / seg_T : 0.0, seg_T);

        // Cold-start governor (Config::recovery_vmax): the opening plan out
        // of rest traverses park->content at positioning gentleness. COLD is
        // rest AND silence on the ONE activity clock (stamped by plan ends as
        // well as commits: a hold segment is content), or the re-seed flag.
        // Keying on rest alone, or on commits alone, clamped ordinary strokes
        // to the user limit (2026-08-10; the 6.9 s rail hold, 2026-09-02).
        // The mode read here is the PREDECESSOR's, for the same reason (p,v,a)
        // is: a queued plan's start state is the tail's end state.
        const Mode pred_mode =
            _sched_n > 0 ? schedAt(_sched_n - 1).mode : _mode;
        const bool cold = _cfg.recovery_vmax > 0.0f &&
                          _cfg.recovery_vmax < _cfg.limits.vmax &&
                          (pred_mode == Mode::Idle || pred_mode == Mode::Settle) &&
                          std::fabs(v) < 1e-3 &&
                          (_reset_cold ||
                           (now_us > _last_activity_us &&
                            now_us - _last_activity_us > kColdStartGapUs));
        _reset_cold = false;
        // A SCHEDULED plan stamps the clock when it is PROMOTED, never at
        // arrival: the activity clock is "content has been rendered up to
        // here", and a future anchor leading real time is what let a catch-up
        // burst pin every liveness test at once.
        if (!sched) noteActivity(t0);
        _plan_lim = _cfg.limits;
        if (cold) _plan_lim.vmax = _cfg.recovery_vmax;
        // ONE CHANNEL, ONE PLANNER: every duration-carrying command is a
        // waveform span at ANY duration, so a stream never switches planners
        // mid-stroke. A zero or absurd duration is rejected by the legality
        // referee, never by a routing floor. Bare points are chase's.
        // A scheduled command runs the SAME path into the SAME active slot:
        // the incumbent trades places with the queue's free slot for the plan
        // and trades back, so no planner or adopter knows a plan can be
        // scheduled.
        if (sched) swapPlan(schedAt(_sched_n));
        const double arrive_s = arrive ? (double)(cmd.anchor_us - t0) * 1e-6 : 0.0;
        const bool ok = cmd.has_duration
                            ? commitWaveform(cmd, p, v, a, target, t0)
                            : commitChase(cmd, p, v, a, target, t0, arrive_s);
        if (sched) {
            swapPlan(schedAt(_sched_n));
            if (ok) ++_sched_n;   // a rejected plan leaves the slot as scratch
        }
        // The arrival chain: only a planned arrival extends it.
        _chase_arrive_ok = ok && arrive;
        _chase_arrive_us = cmd.anchor_us;
        if (ok) _plans++;
        return ok;
    }

    // ---- STOP (Core 1, after queue drain) ------------------------------------
    // The SETTLE brake on demand: drops every scheduled plan, then brakes the
    // plan in flight to rest from its sampled (p, v, a) under the current
    // limits -- jerk-limited, windowed, never a reversal. False when nothing is
    // moving. Records no SettleEngaged: a stop is not a starved stream.
    bool brake(uint64_t now_us) {
        promoteDue(now_us);
        dropScheduledFrom(now_us);
        _chase_arrive_ok = false;
        if (!isBusy(now_us)) return false;
        double p, v, a;
        sampleRaw(now_us, p, v, a);
        planBrake(p, v, a, now_us, p, v, /*starved=*/false);
        return true;
    }

    // ---- Evaluation (Core 1, ~1 kHz hot path) -------------------------------
    // May engage the SETTLE transition when the clock runs past a trajectory
    // that ends moving.
    float positionAt(uint64_t now_us) {
        double p, v, a;
        sampleClamped(now_us, p, v, a);
        return (float)p;
    }

    float velocityAt(uint64_t now_us) {
        double p, v, a;
        sampleClamped(now_us, p, v, a);
        return (float)v;
    }

    float accelerationAt(uint64_t now_us) {
        double p, v, a;
        sampleClamped(now_us, p, v, a);
        return (float)a;
    }

    // The UNCLAMPED sampled state: the same path the sampler takes, minus the
    // backstop. A window regression asserted on positionAt's output asserts on
    // the clamp itself and can never fail.
    void rawSampleAt(uint64_t now_us, double& p, double& v, double& a) {
        maybeSettle(now_us);
        sampleRaw(now_us, p, v, a);
    }

    // ---- The plan as data, for a renderer on another core -------------------
    // The plan in flight and its scheduled successor, WITH NO SIDE EFFECTS: it
    // does not settle, promote, or advance anything. That is the whole point --
    // a renderer needs the plan, not a mutable engine, and every other sampler
    // here mutates. A caller that wants promotion and settle calls positionAt
    // or snapshot instead.
    // Evaluate the pieces with evalPiece; `next` takes over at its own
    // start_us. Borrowed Ruckig pointer, see PlanPiece.
    PlanView planView() const {
        PlanView pv;
        pv.active = activePiece();
        pv.queued = (uint8_t)_sched_n;
        if (_sched_n > 0) {
            pv.next = slotPiece(schedAt(0));
            pv.next_ok = true;
        }
        pv.lo = windowLo(planEntry());
        pv.hi = windowHi(planEntry());
        pv.coast_cap_s = kCoastCapS;
        pv.coast_max_norm = kCoastMaxNorm;
        return pv;
    }

    // Evaluate one piece at an absolute instant, UNCLAMPED. THE one home for
    // the past-expiry coast: sampleRaw is a call to this function, so a
    // renderer built on a PlanPiece and the engine's own sampler cannot
    // disagree about what a plan does after it expires.
    static void evalPiece(const PlanPiece& pc, uint64_t now_us, double& p,
                          double& v, double& a) {
        if (pc.kind == PlanKind::None) {
            p = pc.hold; v = 0.0; a = 0.0;
            return;
        }
        double t = now_us <= pc.start_us
                       ? 0.0
                       : (double)(now_us - pc.start_us) * 1e-6;
        const double dur = pc.duration_s;
        double over = 0.0;
        if (t >= dur) {
            over = t - dur;
            if (over > kCoastCapS) over = kCoastCapS;
            t = dur;
        }
        if (pc.kind == PlanKind::Ruckig) pc.traj->at_time(t, p, v, a);
        else evalCurve(pc.c, pc.T, dur > 0 ? t / dur : 1.0, p, v, a);
        if (over > 0.0) {
            bool capped = over >= kCoastCapS;
            if (std::fabs(v) > 1e-9) {
                const double wall = v > 0.0 ? 1.0 + kCoastMaxNorm
                                            : -kCoastMaxNorm;
                const double room = (wall - p) / v;
                if (room < over) {
                    over   = room > 0.0 ? room : 0.0;
                    capped = true;
                }
            }
            p += v * over;
            a = 0.0;
            // At EITHER cap the position stops advancing, so the reported
            // velocity must stop too: one state, one story.
            if (capped) v = 0.0;
        }
    }

    // Time-aware "does the plan still have motion left to render?" — the
    // sampler gates on this exactly as it did on the cubic's isBusy(). A
    // trajectory pending SETTLE still counts as busy (it is still moving).
    bool isBusy(uint64_t now_us) const {
        if (_sched_n > 0) return true;   // scheduled plans are motion to come
        if (_kind == PlanKind::None) return false;
        if (elapsedS(now_us) < planDuration()) return true;
        double p, v, a;
        planEndState(p, v, a);
        return std::fabs(v) > kRestVel;
    }

    Mode     mode() const { return _mode; }
    PlanKind planKind() const { return _kind; }
    uint64_t lastPlanUs() const { return _plan_start; }
    // The stream estimator's cadence, seconds; 0 = never measured. Read-only:
    // nothing in the plan path reads it back through here.
    double   streamIntervalS() const { return _est_dt_ema; }

    Snapshot snapshot(uint64_t now_us) {
        maybeSettle(now_us);
        Snapshot s;
        double p, v, a;
        sampleClampedNoSettle(now_us, p, v, a);
        s.pos = (float)p;
        s.vel = (float)v;
        s.acc = (float)a;
        if (_kind != PlanKind::None) {
            double pe, ve, ae;
            planEndState(pe, ve, ae);
            s.target     = (float)pe;
            double ps, vs, as;
            if (isHermite()) quinticAt(0.0, ps, vs, as);
            else                            _traj.at_time(0.0, ps, vs, as);
            s.start      = (float)ps;
            s.duration_s = (float)planDuration();
            const double el = elapsedS(now_us);
            s.elapsed_s  = (float)(el < planDuration() ? el : planDuration());
        } else {
            s.target = (float)_hold_pos;
            s.start  = (float)_hold_pos;
        }
        s.mode      = (uint8_t)_mode;
        s.plan_kind = (uint8_t)_kind;
        s.plans     = _plans;
        s.failures  = _failures;
        s.sharpness = _plan_jerk_frac;
        return s;
    }

    // ---- Anomaly drain (Core 1, single-threaded — no lock) ------------------
    bool popAnomaly(Anomaly& out) {
        if (_anom_count == 0) return false;
        const uint8_t read =
            (uint8_t)((_anom_write + kAnomalyDepth - _anom_count) % kAnomalyDepth);
        out = _anom_ring[read];
        _anom_count--;
        return true;
    }

private:
    static constexpr double   kRestVel      = 1e-4;   // units/s: "stopped"
    static constexpr uint8_t  kAnomalyDepth = 16;
    static constexpr int      kScanSteps    = 64;     // quintic legality grid
    // Overshoot-guard floor, normalized: 0.2 % of the stroke window. A plan
    // that lands exactly on its target still shows rounding-sized excursions on
    // a 64-point grid, and rejecting those would send perfectly good segments to
    // the guard for nothing.
    static constexpr double   kOvershootFloor = 0.002;
    // Slack on the Ruckig legality referee. A profile that STARTS at the
    // ceiling with adverse acceleration must overshoot it a little on the way
    // back inside — the jerk limit says so, and no planner can avoid it (swept:
    // 1.029 worst over 129 600 time-optimal cases at the mechanical ceiling).
    // That inherited overshoot is physics, not a planning error, so it must not
    // be mistaken for one. The pathology this referee exists to catch is 1.4x
    // and up, nowhere near this band.
    static constexpr double   kRuckigLegalEps = 0.05;
    // What a referee reports when the question itself is degenerate (no jerk
    // authority, no duration): illegal, and FINITE so a telemetry consumer
    // averaging anomaly details does not inherit an inf.
    static constexpr double   kIllegalRatio = 1e6;
    // ROUNDING, NOT GRACE. The window term steps from 0 to 1.0 the instant a
    // sample leaves [0,1], so a curve commanded to land EXACTLY on a rail is
    // scored by whichever side of zero its last coefficient sum rounds to: the
    // same rail-to-rail segment reads legal or illegal on nothing. 1e-6
    // normalized is 0.1 um on the machine's 100 mm window, four orders under
    // MotionArbiter's 0.5 mm wall and under one microstep. The retired 0.02
    // window GRACE is not coming back (see the note in pointWorst).
    static constexpr double   kWindowRoundEps = 1e-6;
    static constexpr double   kAimCapS      = 0.060;  // predictive aim ceiling
    // The v EMA's smoothing factor, and the group delay it implies in units of
    // the stream interval: dt*(1-a)/a. One home for both, because the aim
    // de-lags with a number updateEstimator owns (see commitChase).
    static constexpr double   kVEmaAlpha       = 0.35;
    static constexpr double   kVEmaLagIntervals = (1.0 - kVEmaAlpha) / kVEmaAlpha;
    // Settle grace = min(this × estimated stream interval, settle_grace_us).
    // 1.5 intervals: one whole interval of lateness is normal transport
    // scheduling, half of another is the margin before it means something.
    static constexpr double   kSettleGraceMult = 1.5;
    // Coast-past-expiry bound (sampleRaw): 2× the grace cap, so the coast
    // always outlives the window in which settle takes over.
    static constexpr double   kCoastCapS = 0.060;
    // ...and bounded in DISTANCE too, because the time cap alone permits
    // vmax * kCoastCapS, which on the machine's 100 mm window at 1000 mm/s is
    // 60 mm of extrapolated position that commit() then seeds the next plan
    // from. The bound is how far OUTSIDE the window the coast may reach, not
    // how far it may travel: extrapolating a plausible 60 mm INSIDE the window
    // is the chord-join continuity the coast exists for, while 60 mm outside it
    // is a position the machine can never have been at. 5 % of the window is
    // 5 mm, an order above MotionArbiter's own 0.5 mm wall.
    static constexpr double   kCoastMaxNorm = 0.05;
    // Anchored-commit lateness bound; MUST stay under kCoastCapS (see
    // commit()).
    static constexpr uint64_t kAnchorMaxLateUs = 50000;
    // Scheduled-anchor LEAD bound, and the ceiling on how long a scheduled
    // successor may suppress the settle boundary. At or above the largest
    // schedule horizon a hub may grant, `schedule_horizon_max_ms` (1000 ms,
    // Valence registry, RFC-087), never under it.
    static constexpr uint64_t kAnchorMaxLeadUs = 1000000;
    static constexpr float    kDetailAnchorLead = -97.0f;   // see PlanFailed
    // Anchored plans parked at once, sized to the largest schedule horizon
    // (RFC-087): a full segments bundle is 32 segments, and at the 1000 ms
    // horizon that is 1000 ms of 32/s, every one parked ahead of its start.
    // CEILING: segments under ~31 ms streamed across the full 1000 ms horizon
    // overflow and are refused (kDetailScheduleFull), which is countable,
    // unlike the silent eviction a shallower queue performs. Each slot is a
    // whole PlanSlot (a Ruckig trajectory, ~0.6 KB): this depth is the bulk of
    // the Engine's size, and the Engine lives in INTERNAL RAM.
    static constexpr size_t   kScheduleDepth = 32;
    static constexpr float    kDetailScheduleFull = -96.0f;  // see PlanFailed

    // A complete plan, off to one side. A queued slot holds a command
    // committed AHEAD of its anchor, planned from the state its predecessor
    // ends in, so promotion at the anchor is continuous in p and v by
    // construction and never renders a piece from tau = 0 at the wrong time.
    struct PlanSlot {
        ruckig::Trajectory<1> traj;
        double   q_c[6] = {};
        double   q_T = 0.0;
        double   p0 = 0.0;
        uint64_t start = 0;
        PlanKind kind = PlanKind::None;
        Mode     mode = Mode::Idle;
        float    jerk_frac = 1.0f;
    };
    // Chase jerk-scale knee: demand fraction of vmax at which full jerk
    // authority returns (see commitChase).
    static constexpr double   kChaseJerkKneeFrac = 0.5;
    // Release time constant of the demand peak-hold (updateEstimator).
    static constexpr double   kSpPeakReleaseS = 0.7;
    // Command silence that makes the next from-rest commit a COLD start.
    // Above the sparsest legitimate content cadence (~1 s point spacing).
    static constexpr uint64_t kColdStartGapUs = 2000000;

    static double clamp01(double x) {
        return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
    }

    // THE WINDOW A PLAN ANSWERS TO: the stroke window widened to take in the
    // plan's own entry position, [min(0, p0), max(1, p0)]. An in-window plan is
    // judged and clamped exactly as it was; an out-of-window one may never get
    // further out than it started, which is what makes an entry move plannable
    // instead of a lie (sd-6b2.10).
    static double windowLo(double p0) { return p0 < 0.0 ? p0 : 0.0; }
    static double windowHi(double p0) { return p0 > 1.0 ? p0 : 1.0; }
    // The entry the window is taken FROM, never further out than the SEED's:
    // a chain may only ever get closer to the window, and a plan's transient
    // rail excursion must not widen the backstop for the plans after it.
    double windowEntry(double p0) const {
        return p0 < _seed_lo ? _seed_lo : (p0 > _seed_hi ? _seed_hi : p0);
    }
    // Entry position of the plan in flight; a hold IS its own entry.
    double planEntry() const {
        return windowEntry(_kind == PlanKind::None ? _hold_pos : _plan_p0);
    }
    double clampWindow(double x) const {
        const double lo = windowLo(planEntry()), hi = windowHi(planEntry());
        return x < lo ? lo : (x > hi ? hi : x);
    }

    double elapsedS(uint64_t now_us) const {
        return now_us <= _plan_start ? 0.0
                                     : (double)(now_us - _plan_start) * 1e-6;
    }

    // The activity clock only ever moves forward: a late anchor is evidence
    // about the past, never a retraction of what has already been rendered.
    void noteActivity(uint64_t t_us) {
        if (t_us > _last_activity_us) _last_activity_us = t_us;
    }

    // ---- Active-plan evaluation ---------------------------------------------
    // Is the active plan one of the Hermite families (evaluated by quinticAt,
    // a cubic being a quintic with two zero high-order terms)? See PlanKind:
    // every branch below reads "Hermite -> quinticAt, else -> Ruckig", so a
    // missing kind here evaluates as a trajectory that was never planned.
    bool isHermite() const {
        return _kind == PlanKind::Quintic || _kind == PlanKind::Cubic;
    }

    double planDuration() const {
        return isHermite() ? _q_T
             : _kind == PlanKind::Ruckig  ? _traj.get_duration()
             : 0.0;
    }

    void planEndState(double& p, double& v, double& a) const {
        if (isHermite())                     quinticAt(1.0, p, v, a);
        else if (_kind == PlanKind::Ruckig)  _traj.at_time(_traj.get_duration(), p, v, a);
        else { p = _hold_pos; v = 0.0; a = 0.0; }
    }

    // Raw kinematic state (UNCLAMPED position — planning continuity must see
    // the true polynomial state even during a transient wall excursion).
    // Past expiry the state COASTS at the end velocity, capped at kCoastCapS:
    // freezing here stamped a flat spot into every chord join whose successor
    // arrived after plan expiry (the 5 ms pacing-drain beat guarantees ~half
    // do), felt as speed-scaled notching; a plan ending at rest coasts
    // nowhere, and maybeSettle stays the stop authority.
    void sampleRaw(uint64_t now_us, double& p, double& v, double& a) const {
        evalPiece(activePiece(), now_us, p, v, a);
    }

    // The active plan and the scheduled successor AS DATA. Cheap by design (six
    // doubles and a borrowed pointer), which is what lets sampleRaw route
    // through evalPiece rather than hold a second copy of the coast rule.
    PlanPiece activePiece() const {
        PlanPiece pc;
        pc.kind       = _kind;
        pc.start_us   = _plan_start;
        pc.duration_s = planDuration();
        pc.hold       = _hold_pos;
        if (isHermite()) {
            for (int i = 0; i < 6; i++) pc.c[i] = _q_c[i];
            pc.T = _q_T;
        } else if (_kind == PlanKind::Ruckig) {
            pc.traj = &_traj;
        }
        return pc;
    }

    PlanPiece slotPiece(const PlanSlot& s) const {
        PlanPiece pc;
        pc.kind     = s.kind;
        pc.start_us = s.start;
        pc.hold     = _hold_pos;
        if (s.kind == PlanKind::Quintic || s.kind == PlanKind::Cubic) {
            for (int i = 0; i < 6; i++) pc.c[i] = s.q_c[i];
            pc.T          = s.q_T;
            pc.duration_s = s.q_T;
        } else if (s.kind == PlanKind::Ruckig) {
            pc.traj       = &s.traj;
            pc.duration_s = s.traj.get_duration();
        }
        return pc;
    }

    // The state a plan anchored at t_us starts from: the queue tail evaluated
    // there (its own bounded coast past expiry), else the plan in flight.
    void sampleScheduleTail(uint64_t t_us, double& p, double& v,
                            double& a) const {
        if (_sched_n == 0) { sampleRaw(t_us, p, v, a); return; }
        evalPiece(slotPiece(schedAt(_sched_n - 1)), t_us, p, v, a);
    }

    // THE ONE CLAMPED SAMPLER. The window clamp is the hard backstop, and a
    // clamped position that still reports outward velocity tells the follower
    // to keep driving into a rail whose position channel says "stopped" -- on
    // the RP2350 that channel becomes step rate, not telemetry. Velocity and
    // acceleration are zeroed only while the position term is SATURATED AND
    // pointing outward: at the wall on the way back in, the motion is real.
    void sampleClampedNoSettle(uint64_t now_us, double& p, double& v,
                               double& a) const {
        sampleRaw(now_us, p, v, a);
        const double lo = windowLo(planEntry()), hi = windowHi(planEntry());
        if (p < lo) {
            p = lo;
            if (v < 0.0) { v = 0.0; a = 0.0; }
        } else if (p > hi) {
            p = hi;
            if (v > 0.0) { v = 0.0; a = 0.0; }
        }
    }

    void sampleClamped(uint64_t now_us, double& p, double& v, double& a) {
        maybeSettle(now_us);
        sampleClampedNoSettle(now_us, p, v, a);
    }

    // Quintic evaluation at normalized tau ∈ [0,1] (real-time derivatives).
    void quinticAt(double tau, double& p, double& v, double& a) const {
        const double* c = _q_c;
        p = ((((c[5]*tau + c[4])*tau + c[3])*tau + c[2])*tau + c[1])*tau + c[0];
        v = ((((5*c[5]*tau + 4*c[4])*tau + 3*c[3])*tau + 2*c[2])*tau + c[1]) / _q_T;
        a = (((20*c[5]*tau + 12*c[4])*tau + 6*c[3])*tau + 2*c[2]) / (_q_T * _q_T);
    }

    // ---- WAVEFORM (v4 / timed segments): quintic + Ruckig guard -------------
    bool commitWaveform(const Command& cmd, double p, double v, double a,
                        double target, uint64_t now_us) {
        // RFC-030: adopt the command's declared family BEFORE any curve is
        // built — every later re-solve of this plan (Scale, the budgeted
        // search) reads it through waveformIsCubic() and therefore re-solves
        // in the SAME family the sender declared.
        _client_curve_family = cmd.client_curve_family;
        const double T = (double)cmd.duration_us * 1e-6;
        // Disarmed before ANY referee can run on this commit: the previous
        // segment's bound is not this one's. Armed for real from this entry
        // state further down.
        _oshoot_allow = -1.0;

        // End velocity: wire G when present, else the stream estimate (an
        // I-only stream shouldn't come to rest at every point).
        double vf = cmd.has_end_vel ? (double)cmd.end_vel
                  : (_cfg.chase_feedforward && _est_ema_ok && streamIsDense())
                        ? _est_v_ema * (double)_cfg.chase_ff_gain
                        : 0.0;

        // DWELL RULE: the SAME target re-commanded is a hold; honoring its
        // declared vf whips the machine through the hold point on every
        // re-send, and each whip displaces p, which is why this tests the
        // TARGET, never position (2026-08-09: cost 54 mm of dropped steps).
        // Strokes alternate targets, so a real stroke chain never matches.
        const bool dwell = _prev_wave_tgt_ok &&
            std::fabs(target - _prev_wave_tgt) < (double)_cfg.dwell_span_norm;
        _prev_wave_tgt = target;
        _prev_wave_tgt_ok = true;
        if (dwell && cmd.has_end_vel && vf != 0.0) {
            // Its OWN kind: a dwell zeroing is not the RFC-008 knot bound, and
            // sharing that kind made the anomaly census unreadable.
            recordAnomaly(AnomalyType::DwellZeroed, (float)target, (float)vf,
                          now_us);
            vf = 0.0;
        }

        // ---- RFC-008 HANDOFF SANITY GUARD (one-segment lookahead) -----------
        // Runs FIRST, ahead of every other treatment of vf, because it is the
        // only one that answers "is this handoff a physically meaningful thing
        // for the sender to have asked for at all?" — the wall/vmax guard
        // below answers a different question (does it fit the window and the
        // ceilings) and the measured pathology passed that one comfortably.
        //
        // chord_in is measured from the machine's ACTUAL position, which is
        // the true chord of the motion being planned (|target - p| over the
        // commanded T) — better ground truth than the sender's own script
        // geometry, and free. chord_out comes from the caller's lookahead; no
        // lookahead means no bound (see Command::has_next_chord).
        //
        // RFC-049c NOTE (the panel's "sparse-segment scheduling-depth
        // backstop" ask, H11): a variant bounding chord_out against chord_in
        // itself when no lookahead is present was PROTOTYPED and REJECTED
        // here, not merely deferred -- it clamps SOME declared down-stroke end
        // velocities on a chain whose own dynamics have already pulled chord_in
        // below the k-factor bound, which is a motion-quality change that pass
        // could not re-validate. Left OPEN, per Valence RFC-049(c) (Valence
        // repo): a real fix needs the scheduling-depth signal to come
        // from somewhere that can tell "a successor is coming, just not yet
        // queued" apart from "this is genuinely the last segment" — which
        // chord_in alone cannot do — rather than trading the tail case's
        // documented honesty clause for an unverified motion-quality risk.
        //
        // Engages only on an EXPLICIT wire handoff. When has_end_vel is false
        // vf is the engine's OWN stream estimate, which is already conservative
        // and is not the sender's claim to sanity-check.
        // NEGATIVE means "no lookahead" (boundHandoffVelocity's own unknown
        // sentinel); the infeasible search re-applies the bound with it, so the
        // smoothness axis cannot hand back the velocity this guard just took.
        const float bound_chord =
            (cmd.has_end_vel && cmd.has_next_chord && T > 0.0) ? cmd.next_chord
                                                               : -1.0f;
        if (bound_chord >= 0.0f) {
            const float chord_in = (float)(std::fabs(target - p) / T);
            const float bounded  = boundHandoffVelocity(
                (float)vf, chord_in, bound_chord, _cfg.handoff_chord_factor);
            if (bounded != (float)vf) {
                // NEVER silent. detail = the ACCEPTED velocity, matching the
                // EndVelClamped convention ("what you actually got").
                recordAnomaly(AnomalyType::HandoffBounded, (float)target,
                              bounded, now_us);
                vf = (double)bounded;
            }
        }
        // The ACCEPTED handoff, post-RFC-008 guard and pre wall/vmax guard.
        // This — not the raw wire value — is what the af series is built from
        // below: the backward difference is an estimate of the SENDER'S
        // curvature at the knot, and once a handoff has been bounded the
        // sender's claimed velocity there is precisely the number we have
        // decided not to believe. Feeding it forward is how one oversized
        // tangent used to poison the NEXT segment's boundary conditions too.
        // Identical to the old behavior whenever the guard does not engage,
        // which is every well-behaved stream.
        const double vf_handoff = vf;
        vf = applyEndVelGuard(vf, target, now_us);

        // End acceleration: the wire carries no af, but consecutive G values
        // imply it (backward difference). Measured on the bench: af=source
        // cuts waveform RMS ~3x vs af=0; the backward difference approximates
        // that. Only trusted between consecutive G-bearing commands of a
        // dense-ish segment chain.
        double af = 0.0;
        if (cmd.has_end_vel && _prev_vf_ok && now_us > _prev_vf_us) {
            const double gap = (double)(now_us - _prev_vf_us) * 1e-6;
            // FLOORED AT THE SEGMENT'S OWN SPAN. The gap is the sender's knot
            // spacing, and a 1 ms gap against a 20 ms span scaled af by 1000x
            // -- an estimate of the sender's curvature cannot be sharper than
            // the segment it is a boundary condition for (sd-6b2.3).
            if (gap < 3.0 * T) af = (vf_handoff - _prev_vf) / std::fmax(gap, T);
        }
        if (cmd.has_end_vel) {
            _prev_vf = vf_handoff;   // accepted handoff, pre wall/vmax guard
            _prev_vf_us = now_us;
            _prev_vf_ok = true;
        } else {
            _prev_vf_ok = false;
        }

        // ---- ARM THE OVERSHOOT GUARD FOR THIS SEGMENT -----------------------
        // Once per commit, before any curve exists, because every candidate the
        // search bisects over shares the same entry state and therefore the same
        // physical floor.
        _oshoot_allow = armOvershootAllow(p, v, a, target, vf, T);

        // Build the quintic in normalized tau; scaled boundary derivatives.
        double c[6];
        buildWaveformCurve(p, v, a, target, vf, af, T, c);

        const double worst = quinticWorstRatio(c, T, _oshoot_allow);
        if (worst <= 1.0) {
            // The commanded segment is LEGAL: the machine delivers all of it,
            // on the clock, as the sender's own curve. Nothing is spent, so
            // nothing is reported.
            adoptQuintic(c, T, now_us);
            return true;
        }

        // The commanded shape is illegal. Before surrendering the deadline to
        // the Ruckig guard, the timing-first policy asks the other question:
        // how much of the shape and the stroke does this segment have to give
        // up to run on the clock it was handed? (See InfeasiblePolicy -- for a
        // scheduled sender that is the only non-degenerate answer, because the
        // next segment preempts us anyway.)
        if (_cfg.infeasible_policy == InfeasiblePolicy::Blend &&
            commitWaveformBudgeted(p, v, a, target, vf, af, T, bound_chord,
                                   now_us)) {
            return true;
        }

        // Quintic broke a ceiling or the window → the Ruckig guard takes the
        // segment: stretched to the deadline when feasible, physical minimum
        // (+ DeadlineStretched) when not. This is the "absurd wire command"
        // path the old cubic executed verbatim.
        recordAnomaly(AnomalyType::WaveformFallback, (float)target,
                      (float)worst, now_us);

        // THE DEADLINE IS KEPT HERE, and switching the guarded case to a
        // TIME-OPTIMAL plan instead was tried and rejected, not overlooked. The
        // premise looked sound — a stretched profile must spend T, and spending T
        // is what manufactures an arc — but the measurement says otherwise:
        // whole shelf, window 50-150, guard 1, 2026-07-30, per-segment excursion
        // came back unchanged (12 recordings, largest move 0.66 -> 0.72 mm, i.e.
        // the wrong way) while sender rms rose on 8 of 12 (GoogleCat 1.83 ->
        // 2.14, synth-sine 0.89 -> 1.09, SYN-mixed 2.80 -> 3.25). Arriving early
        // and holding costs the sender's timing and buys no excursion back.
        const bool ok = planRuckig(p, v, a, target, vf, 0.0, T, now_us);
        if (ok) {
            _mode = Mode::Waveform;
            if (_traj.get_duration() > T * 1.02 + 0.001) {
                recordAnomaly(AnomalyType::DeadlineStretched, (float)target,
                              (float)_traj.get_duration(), now_us);
            }
        }
        return ok;
    }

    // Quintic Hermite coefficients in normalized tau ∈ [0,1] for the boundary
    // conditions (p,v,a) → (target, vf, af) over duration T. Factored out of
    // commitWaveform so the infeasible search re-solves the SAME curve family
    // against a shrunk target without duplicating (or drifting from) the
    // algebra — one copy of the math, one shape.
    static void buildQuintic(double p, double v, double a, double target,
                             double vf, double af, double T, double* c) {
        const double V0 = v * T, A0 = a * T * T;
        const double VF = vf * T, AF = af * T * T;
        const double R1 = target - p - V0 - A0 / 2.0;
        const double R2 = VF - V0 - A0;
        const double R3 = AF - A0;
        c[0] = p; c[1] = V0; c[2] = A0 / 2.0;
        c[3] =  10.0 * R1 - 4.0 * R2 + R3 / 2.0;
        c[4] = -15.0 * R1 + 7.0 * R2 - R3;
        c[5] =   6.0 * R1 - 3.0 * R2 + R3 / 2.0;
    }

    // Cubic Hermite for (p, v) → (target, vf) over T, written into the SAME six
    // coefficients with c[4] = c[5] = 0. A cubic IS a quintic with two zero
    // high-order terms, so every downstream consumer — the legality scan, the
    // peak-jerk closed form, adoptQuintic, the sampler — works unchanged and
    // cannot disagree about what it is looking at. One evaluator, two families.
    //
    // Deliberately takes NO `a` and NO `af`: that is the whole point (see
    // CurvePolicy). The plan starts at whatever acceleration the cubic's own
    // algebra implies, which is how the script's knot step gets reproduced.
    //
    // Note a cubic has CONSTANT jerk within a span (6·c3/T³), so the legality
    // scan's jerk term becomes a single number per plan rather than a sweep —
    // still the same referee, just an easier question.
    static void buildCubic(double p, double v, double target, double vf,
                           double T, double* c) {
        const double V0 = v * T, VF = vf * T, d = target - p;
        c[0] = p;
        c[1] = V0;
        c[2] =  3.0 * d - 2.0 * V0 - VF;
        c[3] = -2.0 * d + V0 + VF;
        c[4] = 0.0;
        c[5] = 0.0;
    }

    // policy + declaration -> reconstruction family, in ONE place. Static and
    // public because a caller can need the answer BEFORE commitWaveform adopts
    // the command — the bench's sender-curve overlay draws the client's own
    // curve at commit time, and an overlay that picks its family by a private
    // re-derivation is how a tuner ends up comparing a cubic against a quintic
    // and calling the difference a planner error.
    // Is the WAVEFORM path reconstructing with a cubic right now? As of RFC-030
    // the curve_family wire signaling EXISTS, so FollowClient finally has
    // something to follow: the family the ACTIVE waveform command declared. The
    // machine override still outranks the declaration, exactly as the policy
    // enum promises.
    bool waveformIsCubic() const { return resolveCubic(_cfg.curve_policy, _client_curve_family); }

    // THE waveform-path curve builder. Every sizing rule (the plain commit and
    // every trial of the infeasible search) goes through here rather than
    // calling buildQuintic directly, so the family is chosen in exactly one
    // place and no path can silently disagree with another about it.
    void buildWaveformCurve(double p, double v, double a, double target,
                            double vf, double af, double T, double* c) const {
        if (waveformIsCubic()) buildCubic(p, v, target, vf, T, c);
        else                   buildQuintic(p, v, a, target, vf, af, T, c);
    }

    void adoptQuintic(const double* c, double T, uint64_t now_us) {
        // quinticAt divides by _q_T; a zero span is not a plan. Structural,
        // matching quinticPeakJerk right below.
        if (!(T > 0.0)) return;
        for (int i = 0; i < 6; i++) _q_c[i] = c[i];
        _q_T        = T;
        _plan_p0    = c[0];
        _kind       = waveformIsCubic() ? PlanKind::Cubic : PlanKind::Quintic;
        _mode       = Mode::Waveform;
        _plan_start = now_us;
        _plan_jerk_frac = (float)(quinticPeakJerk(c, T) /
                                  std::fmax((double)_cfg.limits.jmax, 1e-9));
    }

    // Peak |jerk| of a quintic, closed form — no scan needed. In real time
    // j(tau) = (60·c5·tau² + 24·c4·tau + 6·c3) / T³, a QUADRATIC in tau, so its
    // extremum over [0,1] is one of the two endpoints or the vertex. Used for
    // Snapshot::sharpness so the field means "peak jerk of this plan" on both
    // plan kinds rather than "whatever ceiling the policy happened to use".
    static double quinticPeakJerk(const double* c, double T) {
        if (!(T > 0.0)) return 0.0;
        const double T3 = T * T * T;
        auto j = [&](double tau) {
            return std::fabs((60.0 * c[5] * tau + 24.0 * c[4]) * tau
                             + 6.0 * c[3]) / T3;
        };
        double pk = std::fmax(j(0.0), j(1.0));
        if (std::fabs(c[5]) > 1e-300) {
            const double tv = -c[4] / (5.0 * c[5]);   // dj/dtau = 0
            if (tv > 0.0 && tv < 1.0) pk = std::fmax(pk, j(tv));
        }
        return pk;
    }

    // ---- THE SMOOTHNESS AXIS (0.8.0) ----------------------------------------
    // handle reduction toward the chord
    // Lerp the span's END handle toward its own chord slope by `alpha`, so the
    // curve walks continuously from the sender's spline (alpha = 0) to a dead
    // straight line (alpha = 1). See InfeasiblePolicy for the derivation, the
    // 1.875x headroom ceiling this axis can buy, and why alpha = 1 is C0.
    //
    // ONLY THE END MOVES: the start (p, v, a) is the machine's ACTUAL state and
    // planning from it is doctrine, not a preference. Sufficient anyway — the
    // reduction propagates, because segment N's blended vf IS segment N+1's
    // actual starting v.
    //
    // af GOES TO ZERO rather than toward some blended curvature, for two
    // reasons: a straight line HAS no curvature, and af was never on the wire
    // in the first place (it is a backward-difference ESTIMATE of the sender's
    // curvature — see the af series in commitWaveform). It is the least
    // authoritative number in the boundary set, so it is the first that should
    // give way.
    static void blendEndTowardChord(double p, double target, double T,
                                    double alpha, double& vf, double& af) {
        if (!(T > 0.0) || alpha <= 0.0) return;
        const double al    = alpha > 1.0 ? 1.0 : alpha;
        const double chord = (target - p) / T;
        vf = (1.0 - al) * vf + al * chord;
        af = (1.0 - al) * af;
    }

    // One trial of the two-axis search: build the quintic for (amplitude f,
    // smoothness alpha) and return its worst ceiling/window ratio. THE ONLY
    // place that knows how the two axes compose, so the searches below cannot
    // drift apart from each other.
    //   f     in [-1, 1] : midpoint-anchored amplitude (f = 1 is the full
    //                      commanded stroke, f = 0 stops at the segment
    //                      midpoint, f = -1 does not move). Same geometry
    //                      the amplitude budget floors, so the floor and the
    //                      search size strokes the same way.
    //   alpha in [0, 1]  : handle reduction, 0 is the sender's own curve.
    // The end velocity is scaled by the same (1+f)/2 as the travel: a shortened
    // stroke that still demanded the full handoff velocity would be annihilated
    // by applyEndVelGuard at the wall anyway.
    // `chord_out` is the successor chord that arms RFC-008, or NEGATIVE for
    // "no lookahead" (boundHandoffVelocity's own unknown sentinel). The bound
    // is RE-APPLIED after the blend, measured against THIS trial's own chord:
    // lerping the end handle toward the chord raises |vf| again, and a bound
    // that only ran before the smoothing axis is a bound the smoothing axis
    // can undo. `out_vf` reports the end velocity the trial actually adopted,
    // which is what the next segment's af series must be built from.
    double budgetedTrial(double p, double v, double a, double target, double vf,
                         double af, double T, double f, double alpha,
                         double& out_ep, double* out_c, float chord_out = -1.0f,
                         double* out_vf = nullptr,
                         bool early_exit = false) const {
        const double mid = 0.5 * (p + target);
        const double ep  = clamp01(mid + f * (target - mid));
        double tvf = vf * 0.5 * (1.0 + f);
        double taf = af;
        blendEndTowardChord(p, ep, T, alpha, tvf, taf);
        if (chord_out >= 0.0f && T > 0.0) {
            tvf = (double)boundHandoffVelocity(
                (float)tvf, (float)(std::fabs(ep - p) / T), chord_out,
                _cfg.handoff_chord_factor);
        }
        buildWaveformCurve(p, v, a, ep, tvf, taf, T, out_c);
        out_ep = ep;
        if (out_vf != nullptr) *out_vf = tvf;
        return quinticWorstRatio(out_c, T, _oshoot_allow, early_exit);
    }

    // ---- InfeasiblePolicy::Blend -- the one search --------------------------
    // Spend AMPLITUDE and SHAPE together, in the ratio the slider sets, only as
    // far as legality demands and never past the two budgets. Returns false when
    // the shape is still illegal AT the budget floor -- the routine
    // "conservative limits + aggressive script" case -- which falls through to
    // the Ruckig guard.
    bool commitWaveformBudgeted(double p, double v, double a, double target,
                                double vf, double af, double T, float chord_out,
                                uint64_t now_us) {
        if (!(T > 0.0)) return false;

        const int asteps = _cfg.infeasible_blend_steps < 1 ? 1
                         : (_cfg.infeasible_blend_steps > 10 ? 10
                            : (int)_cfg.infeasible_blend_steps);

        double c[6], ep = target;
        double adopted_alpha = 0.0, adopted_f = 1.0;
        double adopted_vf = vf;
        bool   found = false;

        // ---- ONE SLIDER, BOTH AXES AT ONCE ----------------------------------
        // Spending one axis to EXHAUSTION before touching the other is why an
        // infeasible segment used to arrive as a straight line: alpha was driven
        // to 1 (the chord) rather than to whatever it actually needed.
        //
        // This walks a RAY instead. A single sacrifice scalar s in [0,1] moves
        // BOTH axes together, in a ratio the slider sets:
        //     shape loss      alpha(s)     = s * blend * k
        //     amplitude loss  (1 - f)/2    = s * (1 - blend) * k
        // and the search returns the SMALLEST s that is legal. Degradation is
        // therefore proportional and continuous — a segment that is 10% over
        // gives up about 10% of the ray, not 100% of one axis.
        //
        // THE RAY MUST REACH THE BUDGET CORNER, AND `k` IS WHAT MAKES IT.
        // Without it the two losses were `s*blend` and `s*(1-blend)`, so s = 1
        // landed on the straight LINE BETWEEN the corners rather than on one —
        // at blend 0.5 the search exhausted itself with half of BOTH budgets
        // unspent, and everything past that point fell through to the Ruckig
        // guard, the flattest and latest answer available: measured on
        // OvershootTestThrobbing, 82 of 221 segments took the guard. That is the
        // operator's "some strokes go suddenly linear", and it was this.
        // k = 1 / max(blend, 1 - blend) rescales the ray so s = 1 always lands
        // on whichever budget the direction hits first.
        //
        // The endpoints are UNCHANGED by k (it is 1 at both): blend = 1 spends
        // only smoothness, blend = 0 spends only amplitude. Everything between
        // them is the whole point of the knob.
        //
        // THE RAY ENDS AT THE BUDGETS, and that is the amplitude budget's whole
        // job: `infeasible_amplitude_budget` is the max FRACTION of the stroke
        // that may be surrendered, and surrendered(f) = (1-f)/2, so the FLOOR is
        // f = 1 - 2*budget. Without the caps s = 1 reaches f = -1: endpoint = p,
        // zero travel, a curve that must brake, reverse and return, whose peak
        // accel on a rail slam is ~4v/T. A shape still illegal at the budget
        // floor is the Ruckig guard's honest business.
        //
        // FEASIBILITY IS NOT MONOTONE IN f, SO THE GRID IS WALKED, NOT
        // BISECTED. A shortened span carries the same boundary conditions over
        // less travel, so it curves harder and can break a ceiling the longer
        // one did not. A bisection's invariant assumes monotonicity: it needs a
        // known-legal end to start from, so an illegal ray END discarded a
        // search whose interior was legal, and what it returned was only "the
        // smallest legal point this particular descent happened to land on".
        // Walking i/steps upward returns the SMALLEST LEGAL POINT ON THE GRID,
        // full stop, needs no end probe, and costs the same `steps` scans in
        // the worst case (measured on the review corpus: adoptions 107 -> 702
        // at identical worst-case cost). Every probe exits at its first
        // over-ceiling candidate, and the loop leaves the ADOPTED trial's
        // coefficients in `c` -- there is no confirming re-trial to pay for.
        if (_cfg.infeasible_policy == InfeasiblePolicy::Blend) {
            double bl = (double)_cfg.infeasible_blend;
            bl = bl < 0.0 ? 0.0 : (bl > 1.0 ? 1.0 : bl);
            const double kmax = bl > 1.0 - bl ? bl : 1.0 - bl;
            const double k    = kmax > 1e-9 ? 1.0 / kmax : 1.0;
            // blend_steps is Blend's own step count. It used to be
            // max of two knobs, one of which belonged to another policy and
            // set this one's plan-time cost.
            const int steps = asteps;
            double acap = (double)_cfg.infeasible_smooth_budget;
            acap = acap < 0.0 ? 0.0 : (acap > 1.0 ? 1.0 : acap);
            double lcap = (double)_cfg.infeasible_amplitude_budget;
            lcap = lcap < 0.0 ? 0.0 : (lcap > 1.0 ? 1.0 : lcap);
            auto at = [&](double s, double& fo, double& ao) {
                ao = s * bl * k;
                if (ao > acap) ao = acap;
                double loss = s * (1.0 - bl) * k;
                if (loss > lcap) loss = lcap;
                fo = 1.0 - 2.0 * loss;
            };
            for (int i = 1; i <= steps; i++) {
                double fs, as;
                at((double)i / steps, fs, as);
                if (budgetedTrial(p, v, a, target, vf, af, T, fs, as, ep, c,
                                  chord_out, &adopted_vf, true) <= 1.0) {
                    adopted_alpha = as;
                    adopted_f     = fs;
                    found         = true;
                    break;
                }
            }
            // Not found here falls through to the Ruckig guard below.
            //
            // A SECOND SWEEP OF SMOOTHNESS PAST THE BUDGET, AT FULL AMPLITUDE,
            // WAS TRIED HERE AND REJECTED, not merely skipped: it does find
            // more legal shapes, and they are worse ones. Measured 2026-07-30
            // at window 50-150, guard 1, whole shelf — InterpTest1 per-segment
            // excursion 0.23 -> 3.42 mm max and sender rms 2.08 -> 4.20,
            // GoogleCat 0.16 -> 1.15 mm. A curve that satisfies the guard
            // against its OWN band can still hand the next segment a boundary
            // state that does not, and taking it costs the deadline honesty the
            // guard path at least keeps.
        }
        if (!found) return false;          // both axes spent → Ruckig guard

        adoptQuintic(c, T, now_us);
        // The af series is a backward difference of the velocities the machine
        // ACTUALLY ended at. The search moved this one (amplitude scaling, the
        // blend, the re-applied RFC-008 bound), so feeding the pre-search value
        // forward would derive the next segment's af from a velocity this plan
        // never reaches. Only when the sender declared a handoff at all: with
        // no wire vf the series is not armed (see commitWaveform).
        if (_prev_vf_ok) _prev_vf = adopted_vf;

        // ---- Telemetry: one event per axis actually spent -------------------
        // Never silent, and never a lie about WHICH axis paid. The two are
        // independent: a segment may spend shape alone, amplitude alone, or
        // both, and it reports exactly what it spent.
        if (adopted_alpha > 1e-6) {
            recordAnomaly(AnomalyType::WaveformSmoothed, (float)ep,
                          (float)adopted_alpha, now_us);
        }
        if (adopted_f < 1.0 - 1e-6) {
            const double adist = std::fabs(target - p);
            const double achieved =
                adist > 1e-9 ? std::fabs(ep - p) / adist : 1.0;
            recordAnomaly(AnomalyType::WaveformScaled, (float)ep,
                          (float)achieved, now_us);
        }
        return true;
    }

    // The predicate itself, in whatever precision the caller scans in. `rvc`
    // and `rac` are the RECIPROCAL ceilings, hoisted by the caller: a scan
    // divides by the same two numbers at every point, and on a single-precision
    // FPU a divide costs an order more than a multiply. A ZERO reciprocal
    // disarms its axis, which is the vc > 0 / ac > 0 guard as arithmetic.
    template <typename R>
    static R pointWorstR(R pp, R vv, R aa, R p0, R lo, R hi, R allow, R rvc,
                         R rac) {
        R worst = std::fabs(vv) * rvc;
        worst = std::fmax(worst, std::fabs(aa) * rac);
        const R wlo = p0 < (R)0 ? p0 : (R)0;
        const R whi = p0 > (R)1 ? p0 : (R)1;
        if (pp < wlo - (R)kWindowRoundEps)
            worst = std::fmax(worst, (R)1 + (wlo - pp));
        if (pp > whi + (R)kWindowRoundEps)
            worst = std::fmax(worst, (R)1 + (pp - whi));
        if (allow >= (R)0) {
            const R excess = std::fmax(lo - pp, pp - hi);
            if (excess > allow)
                worst = std::fmax(worst, (R)1 + (excess - allow));
        }
        return worst;
    }

    // ---- Extremum candidates: where a polynomial referee must look ----------
    // A FIXED GRID STEPS OVER A NARROW PEAK, and on the axes this referee
    // polices that is a silent safety hole. Every extremum of p, v, a and j on
    // [0,1] sits at a root of its OWN derivative, and the chain is p' = v,
    // v' = a, a' = j, so one descent down the chain brackets every peak.
    //
    // Roots come from BISECTION INSIDE MONOTONE INTERVALS, not from the cubic
    // and quartic radical forms: the shipping family is a cubic (c[4] = c[5] =
    // 0), so the leading coefficient is zero on the hot path and every radical
    // form needs a degree dispatch to survive that. A sign change on an
    // interval where the polynomial is monotone cannot hide a root, and a
    // tangential root is not an extremum, so nothing is missed.

    // Horner over `deg` + 1 ascending coefficients.
    static float polyAt(const float* k, int deg, float t) {
        float r = k[deg];
        for (int i = deg - 1; i >= 0; i--) r = r * t + k[i];
        return r;
    }

    // Roots of `k` (degree `deg`) in (0,1), given `brk`, the ascending roots of
    // its derivative, which cut [0,1] into intervals it is monotone on. Writes
    // at most `deg` roots, ascending.
    static int rootsIn01(const float* k, int deg, const float* brk, int nbrk,
                         float* out) {
        float x[6];
        int n = 0;
        x[n++] = 0.0f;
        for (int i = 0; i < nbrk && n < 5; i++) x[n++] = brk[i];
        x[n++] = 1.0f;
        int found = 0;
        float fa = polyAt(k, deg, x[0]);
        for (int i = 0; i + 1 < n; i++) {
            float lo = x[i], hi = x[i + 1];
            const float fb = polyAt(k, deg, hi);
            // A root ON an interval end is already a candidate tau (the ends
            // are the endpoints and the derivative's own roots), so only a
            // strict sign change needs bracketing.
            if (hi > lo && fa != 0.0f && fb != 0.0f &&
                ((fa < 0.0f) != (fb < 0.0f))) {
                const bool neg_lo = fa < 0.0f;
                // 20 halvings of an interval no wider than 1 is 1e-6 in tau,
                // and a peak is flat in its own neighborhood.
                for (int s = 0; s < 20; s++) {
                    const float m = 0.5f * (lo + hi);
                    if ((polyAt(k, deg, m) < 0.0f) == neg_lo) lo = m;
                    else                                      hi = m;
                }
                out[found++] = 0.5f * (lo + hi);
            }
            fa = fb;
        }
        return found;
    }

    // 0, 1 and every extremum of p/v/a/j for the curve `c`; returns the count,
    // at most 12. These are tau-polynomial coefficients, so the derivatives
    // carry the T powers the caller divides out.
    static int extremumTaus(const float* c, float* taus) {
        int n = 0;
        taus[n++] = 0.0f;
        taus[n++] = 1.0f;
        // dj/dtau = 24c4 + 120c5 tau: the |j| extremum, exact.
        float rj1[1];
        int nj1 = 0;
        if (std::fabs(c[5]) > 1e-30f) {
            const float t = -c[4] / (5.0f * c[5]);
            if (t > 0.0f && t < 1.0f) rj1[nj1++] = t;
        }
        const float kj[3] = {6.0f * c[3], 24.0f * c[4], 60.0f * c[5]};
        float rj[2];
        const int nj = rootsIn01(kj, 2, rj1, nj1, rj);          // |a| extrema
        const float ka[4] = {2.0f * c[2], 6.0f * c[3], 12.0f * c[4],
                             20.0f * c[5]};
        float ra[3];
        const int na = rootsIn01(ka, 3, rj, nj, ra);            // |v| extrema
        const float kv[5] = {c[1], 2.0f * c[2], 3.0f * c[3], 4.0f * c[4],
                             5.0f * c[5]};
        float rv[4];
        const int nv = rootsIn01(kv, 4, ra, na, rv);            // p extrema
        for (int i = 0; i < nj1; i++) taus[n++] = rj1[i];
        for (int i = 0; i < nj;  i++) taus[n++] = rj[i];
        for (int i = 0; i < na;  i++) taus[n++] = ra[i];
        for (int i = 0; i < nv;  i++) taus[n++] = rv[i];
        return n;
    }

    // ---- ONE definition of legal, shared by both referees --------------------
    // Public because the native suite pins the two referees to a single answer
    // on curves both planners can draw; pure queries, they adopt nothing.
public:
    // Scores one sampled point: v/a ceilings, stroke window, overshoot band.
    // `p0` is the judged curve's ENTRY position and sets the window
    // [min(0, p0), max(1, p0)] (see windowLo/windowHi); `lo`/`hi` are that
    // curve's own endpoints, the overshoot band; `allow` < 0 disarms the
    // band; > 1.0 = illegal. Jerk is the quintic referee's own term (Ruckig
    // cannot violate it, see ruckigWorstRatio). Window grace and band are
    // spelled ONCE here so the two planners cannot disagree about legality.
    //
    // WINDOW, WITH NO GRACE BAND. A plan permitted to bulge past the rail
    // arrives there still DRIVING OUTWARD and hands the follower momentum to
    // absorb (async-tune bench 2026-07-30: 69 samples pinned at the top rail,
    // worst +217 mm/s, 625 mm of travel on a 500 mm rail). The retired 0.02
    // was also 12x looser than MotionArbiter's own 0.5 mm wall.
    // kWindowRoundEps is not that band coming back: it is four orders smaller
    // and exists because the term is a STEP, so a curve landing exactly on a
    // rail was scored by which side of zero it rounded to.
    //
    // The BAND is two-sided: the measured pathology is a backswing AWAY from
    // the target (a move from 186.8 mm to 100 mm arcing up to 305 mm), which
    // a one-sided test scores negative and waves through.
    double pointWorst(double pp, double vv, double aa, double p0, double lo,
                      double hi, double allow) const {
        const double vc = _plan_lim.vmax, ac = _plan_lim.amax;
        return pointWorstR(pp, vv, aa, windowEntry(p0), lo, hi, allow,
                           vc > 0.0 ? 1.0 / vc : 0.0,
                           ac > 0.0 ? 1.0 / ac : 0.0);
    }

    // Worst (peak / ceiling) ratio across v/a/j ceilings AND window bounds,
    // evaluated at every extremum of the curve. > 1.0 = illegal quintic.
    // The allowance is a PARAMETER, never ambient state: consecutive commits
    // plan different curves from different entry states, and a member read
    // here judged one commit's span with another's bound.
    //
    // `early_exit` returns at the first candidate over 1.0. A search probe only
    // needs the VERDICT; only a trial about to be adopted, and the commit-entry
    // scan whose ratio becomes the WaveformFallback detail, need the number.
    double quinticWorstRatio(const double* c, double T, double oshoot_allow,
                             bool early_exit = false) const {
        if (!(T > 0.0)) return kIllegalRatio;
        const float jc = _plan_lim.jmax;
        // A ZERO JERK CEILING IS NOT "no jerk limit", it is NO JERK AUTHORITY:
        // nothing a planner can draw is legal under it. Dividing instead gave
        // 0/0 = NaN on a curve whose high-order terms are zero, and fmax drops
        // a NaN, so the scan reported LEGAL and the referee was off (sd-6b2.3).
        if (!(jc > 0.0f)) return kIllegalRatio;
        // PLAN-TIME ALGEBRA IS DOUBLE, THE SCAN IS FLOAT: the coefficients are
        // converted once, here. A threshold test already carrying
        // kRuckigLegalEps does not need 15 digits, and neither the S3's FPU nor
        // the RP2350's has double at all (cpp-style.md).
        float fc[6];
        for (int i = 0; i < 6; i++) fc[i] = (float)c[i];
        const float p0f = (float)windowEntry(c[0]);
        // Reciprocals hoisted out of the scan: the same six divisors at every
        // candidate, and a soft-float divide costs an order more than a
        // multiply.
        const float rT  = 1.0f / (float)T;
        const float rT2 = rT * rT, rT3 = rT2 * rT;
        const float rjc = 1.0f / jc;
        const float rvc = _plan_lim.vmax > 0.0f ? 1.0f / _plan_lim.vmax : 0.0f;
        const float rac = _plan_lim.amax > 0.0f ? 1.0f / _plan_lim.amax : 0.0f;
        float worst = 0.0f;
        // ---- overshoot-guard preamble (all no-ops when the guard is off) ----
        // The band is this trial's OWN endpoints, c[0] and the curve at tau = 1
        // (the sum of the coefficients), so a shortened or smoothed candidate
        // is judged against the stroke it actually draws.
        //
        // The PHYSICAL floor is not recomputed here. It belongs to the commit,
        // not to the trial: it is the excursion physics forces on the move from
        // the caller's entry state, which every candidate in a bisection shares,
        // and measuring it means a Ruckig solve inside the innermost loop of
        // two searches. See _oshoot_allow.
        //
        // THE CHORD SLACK IS PER-TRIAL and must stay that way: it is a fraction
        // of the stroke THIS candidate draws (overshoot_chord_slack is a ratio,
        // "further past its target than the move itself was long"). Carrying the
        // full commanded chord's slack into a shortened trial makes the guard
        // weakest on the shortest candidates, which are the ones that bulge.
        float oshoot_lo = 0.0f, oshoot_hi = 0.0f, allow = (float)oshoot_allow;
        if (oshoot_allow >= 0.0) {
            float p_end = 0.0f;
            for (int k = 0; k < 6; k++) p_end += fc[k];
            oshoot_lo = std::fmin(fc[0], p_end);
            oshoot_hi = std::fmax(fc[0], p_end);
            allow += _cfg.overshoot_chord_slack * (oshoot_hi - oshoot_lo);
        }
        float taus[12];
        const int n = extremumTaus(fc, taus);
        for (int i = 0; i < n; i++) {
            const float t  = taus[i];
            const float pp = ((((fc[5]*t + fc[4])*t + fc[3])*t + fc[2])*t + fc[1])*t + fc[0];
            const float vv = (((5*fc[5]*t + 4*fc[4])*t + 3*fc[3])*t + 2*fc[2])*t + fc[1];
            const float aa = ((20*fc[5]*t + 12*fc[4])*t + 6*fc[3])*t + 2*fc[2];
            const float jj = (60*fc[5]*t + 24*fc[4])*t + 6*fc[3];
            worst = std::fmax(worst, std::fabs(jj) * rT3 * rjc);
            worst = std::fmax(worst, pointWorstR(pp, vv * rT, aa * rT2, p0f,
                                                 oshoot_lo, oshoot_hi, allow,
                                                 rvc, rac));
            if (early_exit && worst > 1.0f) break;
        }
        return (double)worst;
    }

    // The SAME referee, for a Ruckig profile. Deliberately identical in shape
    // and in what it calls legal: it scores every sample through the SAME
    // pointWorst predicate, with the same window and the same band allowance,
    // so the two planners cannot disagree about what "legal" means. Pinned by
    // the coincident-curve sweep in test/native/test_kinetic.
    //
    // WHY THIS HAS TO EXIST — RUCKIG IS NOT A LEGALITY ORACLE. `max_velocity`
    // is an input to Ruckig's profile SEARCH, not a postcondition of its
    // output: when the jerk ceiling is too low to turn the boundary state
    // around, Community returns a profile that sails straight through the
    // velocity ceiling (and, further down, through the stroke window) rather
    // than reporting infeasible. Measured on the captured chase→segment
    // handoff (p 0.798, v −0.468, a −16, → 0.700 at vf −1.095, vmax 1.1):
    //
    //     jerk ceiling   peak |v| / vmax   position span
    //         4000            0.995         [0.700 .. 0.798]   legal
    //          400            0.995         [0.700 .. 0.798]   legal
    //          200            1.007         [0.648 .. 0.798]
    //          115            1.434         [0.457 .. 0.801]   <- the capture
    //          100            1.589         [0.369 .. 0.808]
    //           50            2.753         [-0.605 .. 0.853]  <- off the rail
    //
    // Jerk is NOT scanned: Ruckig's profiles are bang-bang in jerk at exactly
    // the ceiling it was handed, so the sample grid would only ever rediscover
    // that ceiling. Velocity, acceleration and the window are the properties it
    // can actually miss.
    // A Ruckig profile is PIECEWISE, so it keeps the fixed grid: there is no
    // single polynomial whose derivative roots would name its extrema. The
    // predicate, the reciprocal hoist and the float scan are the quintic
    // referee's, so "legal" still means exactly one thing.
    double ruckigWorstRatio(const ruckig::Trajectory<1>& traj,
                            double oshoot_allow, bool early_exit = false) const {
        const double dur = traj.get_duration();
        if (!(dur > 0.0) || !std::isfinite(dur)) return 0.0;
        double entry, ev, ea;
        traj.at_time(0.0, entry, ev, ea);
        const float p0f = (float)windowEntry(entry);
        float oshoot_lo = 0.0f, oshoot_hi = 0.0f, allow = (float)oshoot_allow;
        if (oshoot_allow >= 0.0) {
            double p1, vv, aa;
            traj.at_time(dur, p1, vv, aa);
            oshoot_lo = (float)std::fmin(entry, p1);
            oshoot_hi = (float)std::fmax(entry, p1);
            allow += _cfg.overshoot_chord_slack * (oshoot_hi - oshoot_lo);
        }
        const float rvc = _plan_lim.vmax > 0.0f ? 1.0f / _plan_lim.vmax : 0.0f;
        const float rac = _plan_lim.amax > 0.0f ? 1.0f / _plan_lim.amax : 0.0f;
        float worst = 0.0f;
        for (int i = 0; i <= kScanSteps; i++) {
            double pp, vv, aa;
            traj.at_time(dur * (double)i / kScanSteps, pp, vv, aa);
            worst = std::fmax(worst, pointWorstR((float)pp, (float)vv, (float)aa,
                                                 p0f, oshoot_lo, oshoot_hi,
                                                 allow, rvc, rac));
            if (early_exit && worst > 1.0f) break;
        }
        return (double)worst;
    }

private:
    // THE OVERSHOOT GUARD'S REFERENCE: how far outside the commanded band the
    // machine travels when it is trying its hardest not to. The TIME-OPTIMAL
    // plan brakes with every bit of authority there is, so whatever excursion
    // survives it is the excursion this move physically costs — jerk ceiling,
    // velocity ceiling, requested arrival velocity and all. Anything beyond it
    // is the polynomial's invention, and that is exactly the line the guard
    // needs to draw.
    //
    // MEASURING BEATS THE CLOSED FORM because the closed form is wrong by the
    // jerk term (see Config::overshoot_guard for the 6.4 mm vs ~15 mm case that
    // made the old knob actively harmful). It is also self-correcting: a machine
    // with more jerk authority gets a tighter allowance with nothing to retune.
    //
    // ONE Ruckig solve per commit that arms the guard (a waveform segment,
    // from its own entry state). Returns < 0 when Ruckig has no opinion, which
    // disarms the guard for that curve rather than inventing a bound.
    double physicalBandExcess(double p, double v, double a, double target,
                              double vf) {
        ruckig::InputParameter<1> in;
        in.current_position[0]     = p;
        in.current_velocity[0]     = v;
        in.current_acceleration[0] = a;
        in.target_position[0]      = target;
        in.target_velocity[0]      = vf;
        in.target_acceleration[0]  = 0.0;
        in.max_velocity[0]         = _plan_lim.vmax;
        in.max_acceleration[0]     = _plan_lim.amax;
        in.max_jerk[0]             = _plan_lim.jmax;
        ruckig::Trajectory<1> traj;
        if ((int)_calc.calculate(in, traj) < 0) return -1.0;
        const double dur = traj.get_duration();
        if (!(dur > 0.0) || !std::isfinite(dur)) return -1.0;
        const double lo = std::fmin(p, target), hi = std::fmax(p, target);
        double ex = 0.0;
        for (int i = 0; i <= kScanSteps; i++) {
            double pp, vv, aa;
            traj.at_time(dur * (double)i / kScanSteps, pp, vv, aa);
            ex = std::fmax(ex, std::fmax(lo - pp, pp - hi));
        }
        return ex;
    }

    // The overshoot bound for ONE commit, from THAT commit's entry state.
    // Returns < 0 when the guard is off or Ruckig declines to answer: no
    // answer means no bound, an invented one was the guard's original mistake.
    double armOvershootAllow(double p, double v, double a, double target,
                             double vf, double T) {
        if (!(_cfg.overshoot_guard > 0.0f) || !(T > 0.0)) return -1.0;
        const double floor_mm = physicalBandExcess(p, v, a, target, vf);
        if (floor_mm < 0.0) return -1.0;
        // The PHYSICAL floor only. The chord-slack term is a fraction of the
        // stroke a candidate actually draws, so it is added per trial by the
        // two worst-ratio referees, never here from the commanded chord.
        return (double)_cfg.overshoot_guard * floor_mm + kOvershootFloor;
    }

    // ---- CHASE (bare / short-interval points) -------------------------------
    // arrive_s > 0: an ARRIVAL plan (RFC-084) that reaches `target` exactly
    // arrive_s after now_us when the ceilings allow, at the stream's own
    // velocity and curvature. No predictive aim: the plan already knows when
    // it must be there, so aiming past the point would only add overshoot.
    // 0 = the time-optimal chase of a point that is already due.
    bool commitChase(const Command& cmd, double p, double v, double a,
                     double target, uint64_t now_us, double arrive_s = 0.0) {
        // A bare point declares no band, so there is nothing for the guard to
        // measure excursion against. Disarmed explicitly: the softened-plan
        // legality recheck in planRuckig reads this member (chase plans at a
        // demand-scaled jerk ceiling, see chase_jerk_scale).
        _oshoot_allow = -1.0;
        // RFC-030: a bare point declares no family either, and inheriting the
        // last segment's would re-solve a later plan in a family this command
        // never named.
        _client_curve_family = 0;
        double aim = target;
        double vf  = 0.0;
        double af  = 0.0;
        if (arrive_s > 0.0 && _cfg.chase_feedforward && _est_ema_ok &&
            streamIsDense()) {
            const double acap  = 0.5 * (double)_plan_lim.amax;
            const double a_est = _est_a_ema < -acap ? -acap
                               : _est_a_ema >  acap ?  acap : _est_a_ema;
            // The de-lagged estimate IS the stream at this point's instant.
            const double v_pt = _cfg.chase_aim_accel_extrap
                                    ? _est_v_ema + kVEmaLagIntervals * _est_dt_ema * a_est
                                    : _est_v_ema;
            vf = applyEndVelGuard(v_pt * (double)_cfg.chase_ff_gain, aim, now_us);
            if (_cfg.chase_accel_ff) af = a_est * (double)_cfg.chase_ff_gain;
        } else if (arrive_s <= 0.0 && _cfg.chase_feedforward && _est_ema_ok &&
                   streamIsDense()) {
            // Predictive aim: the newest point is already ~1 interval stale
            // and the plan needs time to get there — aim ahead along the
            // stream's motion, arrive AT its velocity and curvature. (The
            // legacy live-mode extrapolation, reborn with real dynamics.)
            const double look =
                std::fmin(_est_dt_ema * (double)_cfg.chase_lookahead, kAimCapS);
            // acap-limited stream curvature. Limited BEFORE it is used
            // anywhere: the accel estimate is a second difference of a jittery
            // signal, and it feeds both the arrival acceleration and (below)
            // the aim POSITION, where an unlimited spike would fling the aim.
            const double acap  = 0.5 * (double)_plan_lim.amax;
            const double a_est = _est_a_ema < -acap ? -acap
                               : _est_a_ema >  acap ?  acap : _est_a_ema;
            // DE-LAG before extrapolating. The v EMA describes the stream one
            // group delay ago (kVEmaLagIntervals of an interval), so using it
            // as the velocity NOW leaves a residue IN PHASE with the target --
            // which is amplitude, not lead. That is why raising lookahead used
            // to buy lead at the cost of stroke. Carried forward by the same
            // acap-limited a_est, so a noisy second difference cannot fling it.
            // Under the v2 flag with the two terms above: it is the same
            // curvature correction, and the flag's contract is that clearing
            // it reverts every a_est-derived term at once.
            const double v_now = _cfg.chase_aim_accel_extrap
                                     ? _est_v_ema + kVEmaLagIntervals * _est_dt_ema * a_est
                                     : _est_v_ema;
            const double v_est = v_now * (double)_cfg.chase_ff_gain;
            // Second-order aim: a straight-line extrapolation is wrong exactly
            // where a waveform turns, and the turn near a rail is where being
            // wrong costs the most (aim clamps to the wall → end-vel guard
            // sees dist-to-wall 0 → forced vf = 0 → the carriage parks at the
            // rail). ½·a_est·look² pulls the aim back through the crest.
            aim = clamp01(_cfg.chase_aim_accel_extrap
                              ? target + v_est * look + 0.5 * a_est * look * look
                              : target + v_est * look);
            // Arrival velocity must belong to the same instant as the aim.
            // Aiming `look` seconds ahead but requesting the velocity the
            // stream has NOW is internally inconsistent: through a crest the
            // stream is decelerating, so v_est(now) is too fast for the point
            // we plan to be at, and the plan arrives still climbing — straight
            // into the wall/overshoot the second-order aim term was added to
            // fix. Extrapolate the velocity over the same horizon with the
            // same acap-limited a_est (so a noisy second difference cannot
            // fling it either). One flag, one coherent "predictive aim v2":
            // with chase_aim_accel_extrap off, both the aim's a-term and this
            // revert to the pre-v2 behavior byte for byte.
            const double vf_req = _cfg.chase_aim_accel_extrap
                                      ? v_est + a_est * look
                                      : v_est;
            vf  = applyEndVelGuard(vf_req, aim, now_us);
            // Same damping as the arrival velocity: arriving at
            // chase_ff_gain of the stream's speed but at ALL of its
            // acceleration is two incompatible requests, and the accel one
            // undoes the other inside the plan.
            if (_cfg.chase_accel_ff) af = a_est * (double)_cfg.chase_ff_gain;
        } else if (cmd.has_end_vel) {
            vf = applyEndVelGuard((double)cmd.end_vel, aim, now_us);
        }
        double j_ovr = 0.0;
        if (_cfg.chase_jerk_scale) {
            const double vm = (double)_plan_lim.vmax;
            // The MOVE ceiling is the stream's recent PEAK speed, never a
            // local average: an EMA dips at every crest and de-claws the
            // turn exactly where authority is needed (sd-d77.1 bench).
            double dem = std::fmax(std::fabs(v), std::fabs(vf));
            if (_est_ema_ok) dem = std::fmax(dem, _est_sp_pk);
            // Knee: full authority at kChaseJerkKneeFrac of vmax -- tracking
            // jerk follows content jerk (~omega^3), not the velocity fraction.
            double r = dem / (vm > 1e-9 ? kChaseJerkKneeFrac * vm : 1.0);
            const double fl = (double)_cfg.chase_jerk_floor;
            if (r < fl) r = fl;
            if (r > 1.0) r = 1.0;
            j_ovr = (double)_plan_lim.jmax * r;
        }
        bool ok = planRuckig(p, v, a, aim, vf, af, arrive_s, now_us, j_ovr);
        // A softened plan may be DECLINED (legality recheck); the mechanical
        // ceiling is always available as the hard fallback.
        if (!ok && j_ovr > 0.0)
            ok = planRuckig(p, v, a, aim, vf, af, arrive_s, now_us);
        if (ok) _mode = Mode::Chase;
        return ok;
    }

    // ---- Shared Ruckig point-planner (chase, guard fallback) ----------------
    // min_dur 0 = time-optimal; > 0 = stretch toward the deadline.
    // j_ovr 0 = plan at the mechanical jerk ceiling; > 0 = plan at a SOFTER one
    // (chase's demand scaling — jerkCeil enforces "softer only, never harder").
    bool planRuckig(double p, double v, double a, double target, double vf,
                    double af, double min_dur, uint64_t now_us,
                    double j_ovr = 0.0) {
        const double jc = jerkCeil(j_ovr);
        ruckig::InputParameter<1> in;
        in.current_position[0]     = p;
        in.current_velocity[0]     = v;
        in.current_acceleration[0] = a;
        in.target_position[0]      = target;
        in.target_velocity[0]      = vf;
        in.target_acceleration[0]  = af;
        in.max_velocity[0]         = _plan_lim.vmax;
        in.max_acceleration[0]     = _plan_lim.amax;
        in.max_jerk[0]             = jc;
        if (min_dur > 0.0) in.minimum_duration = min_dur;

        ruckig::Trajectory<1> traj;
        const ruckig::Result res = _calc.calculate(in, traj);
        if ((int)res < 0) {
            _failures++;
            recordAnomaly(AnomalyType::PlanFailed, (float)target,
                          (float)(int)res, now_us);
            return false;
        }
        // Ruckig reports ErrorTrajectoryDuration rather than a non-finite
        // duration, so this is belt and braces -- and it is the check
        // probeRuckigDuration already made while the ADOPTER did not.
        if (!std::isfinite(traj.get_duration())) {
            _failures++;
            recordAnomaly(AnomalyType::PlanFailed, (float)target, -98.0f,
                          now_us);
            return false;
        }
        // ---- EVERY ADOPTED TRAJECTORY IS SCORED -----------------------------
        // Ruckig is not a legality oracle (see ruckigWorstRatio): max_velocity
        // is an input to its profile SEARCH, not a postcondition of its output.
        // This score used to run only for a softened ceiling, so every TERMINAL
        // adoption -- chase at full authority, the chase hard fallback, the
        // waveform guard -- was adopted unrefereed, and the only surviving
        // defense was the output clamp (sd-6b2.2).
        //
        // A SOFTENED plan is REFUSED: the caller asked for a gentler ceiling on
        // its own authority, it additionally pins minimum_duration, and
        // Ruckig's stretched profile families are not the time-optimal ones --
        // so the caller re-plans at the mechanical ceiling it already knows is
        // available (chase's retry). NOT a plan failure: nothing failed, this
        // shape was declined.
        //
        // A TERMINAL plan is ADOPTED and REPORTED, detail = the worst ratio.
        // There is no harder plan to fall back to, and a stream left with no
        // plan at all is worse than one whose excursion is named in the census.
        const double worst = ruckigWorstRatio(traj, _oshoot_allow);
        if (worst > 1.0 + kRuckigLegalEps) {
            recordAnomaly(AnomalyType::WaveformFallback, (float)target,
                          (float)worst, now_us);
            if (j_ovr > 0.0 && jc < (double)_plan_lim.jmax) return false;
        }
        _traj       = traj;
        _kind       = PlanKind::Ruckig;
        _plan_start = now_us;
        _plan_p0    = p;
        _plan_jerk_frac =
            _plan_lim.jmax > 0.0f ? (float)(jc / (double)_plan_lim.jmax)
                                    : 1.0f;
        return true;
    }

    // The jerk ceiling a plan actually runs under. A positive override is a
    // caller asking for a SOFTER profile (chase's demand scaling); it is clamped
    // to the mechanical ceiling here, in ONE place, so no caller can hand Ruckig
    // a jerk the machine cannot survive. Non-positive = "use the configured
    // ceiling".
    double jerkCeil(double j_ovr) const {
        const double jc = (double)_plan_lim.jmax;
        if (!(j_ovr > 0.0)) return jc;
        return j_ovr < jc ? j_ovr : jc;
    }

    // Wall guard + ceiling for a requested end velocity: the machine must be
    // able to brake to rest inside the window beyond the target. Trapezoid
    // bound vf² ≤ amax·dist — conservative margin for the jerk-limited tail
    // at sane jmax/amax ratios; the sampler clamp is the hard backstop.
    double applyEndVelGuard(double vf, double target, uint64_t now_us) {
        const double out = endVelBound(vf, target);
        if (std::fabs(out - vf) > 1e-6) {
            recordAnomaly(AnomalyType::EndVelClamped, (float)target,
                          (float)out, now_us);
        }
        return out;
    }

    // The bound itself, with no telemetry side effect — a speculative caller
    // needs to ask the guard's question about endpoints it may never adopt.
    double endVelBound(double vf, double target) const {
        const double vcap = _plan_lim.vmax;
        if (vf >  vcap) vf =  vcap;
        if (vf < -vcap) vf = -vcap;
        const double dist = vf > 0.0 ? (1.0 - target) : target;
        const double vmax_wall =
            std::sqrt((double)_plan_lim.amax * std::fmax(dist, 0.0));
        if (std::fabs(vf) > vmax_wall) {
            vf = vf > 0.0 ? vmax_wall : -vmax_wall;
        }
        return vf;
    }

    // ---- Stream estimator (velocity + cadence of the incoming points) -------
    // Fed by every commit; consumed by chase aim and waveform vf/af fill-ins.
    // EMAs are deliberately calm (the ±ms arrival jitter of real transports
    // otherwise buzzes straight into the acceleration trace — bench-measured).
    // `span` > 0 (a duration-carrying segment) supplies the CONTENT timeline:
    // that segment's rate is `chord` over `span`, never the anchor difference,
    // because its target is where the machine will be at anchor + span. `span`
    // == 0 (a bare point) differences the anchors. Staleness and the
    // differencing base stay on the anchor clock either way.
    void updateEstimator(double target, uint64_t now_us, double chord,
                         double span) {
        if (_est_valid && now_us > _est_last_us) {
            const uint64_t gap = now_us - _est_last_us;
            if (gap <= _cfg.chase_stale_us) {
                const double cad = span > 0.0 ? span : (double)gap * 1e-6;
                // A bare point declares no timeline and the hub resolves a
                // sample that arrived late to NOW, so the anchor difference IS
                // the arrival gap. Dividing a uniform-cadence chord by a
                // jittered gap reads HIGH -- E[1/dt] exceeds 1/E[dt] -- and the
                // predictive aim renders that over-read as stroke: 20 ms of
                // jitter on a 20 ms cadence measured +17% amplitude on the P4
                // (bd val-091.14). Difference against the LEARNED cadence
                // instead; the gap still teaches it, one line down. Phosphor
                // .claude/rules/webui.md T18: arrival time is a hint, never a
                // timeline.
                const double dt  = span > 0.0 ? span
                                 : (_est_ema_ok && _est_dt_ema > 0.0 ? _est_dt_ema : cad);
                const double raw = span > 0.0
                                       ? chord
                                       : (target - _est_last_target) / dt;
                if (_est_ema_ok) {
                    const double v_prev = _est_v_ema;
                    // Peak-hold speed with first-order release (chase jerk
                    // scale): instant attack, decays toward the local speed
                    // over kSpPeakReleaseS. Never an EMA (crest-dip trap).
                    const double sp = std::fabs(raw);
                    if (sp > _est_sp_pk) _est_sp_pk = sp;
                    else _est_sp_pk += (dt / kSpPeakReleaseS) *
                                       (sp - _est_sp_pk);
                    _est_v_ema  += kVEmaAlpha * (raw - _est_v_ema);
                    _est_dt_ema += 0.30 * (cad - _est_dt_ema);
                    // Stream curvature: differentiate the (already smoothed)
                    // velocity EMA, then smooth again — accel estimates are
                    // second differences of a jittery signal, treat gently.
                    const double a_raw = (_est_v_ema - v_prev) / dt;
                    _est_a_ema += 0.25 * (a_raw - _est_a_ema);
                } else {
                    _est_v_ema  = raw;
                    _est_sp_pk  = std::fabs(raw);
                    _est_dt_ema = cad;
                    _est_a_ema  = 0.0;
                    _est_ema_ok = true;
                }
            } else {
                _est_ema_ok = false;   // stale stream → forget the dynamics
            }
        }
        _est_last_target = target;
        _est_last_us     = now_us;
        _est_valid       = true;
    }

    bool streamIsDense() const {
        return _est_ema_ok &&
               _est_dt_ema * 1e6 <= (double)_cfg.chase_dense_us;
    }

    // ---- Settle grace -------------------------------------------------------
    // How long an expired plan may hold its end state before we call the
    // stream starved. Sized from the stream's OWN cadence — a sender pacing
    // segments every 167 ms is not late until it is late BY that stream's
    // standards — and capped, because the grace exists to absorb transport
    // jitter, not to invent a hold. Zero (no cadence estimate yet, stale
    // stream, or knob disabled) restores the pre-0.4 brake-on-expiry.
    double settleGraceS(uint64_t now_us) const {
        if (_cfg.settle_grace_us == 0) return 0.0;
        // A cadence NEVER measured is an isolated point move: brake promptly.
        if (_est_dt_ema <= 0.0) return 0.0;
        // Staleness reads the ONE activity clock, which plan ends stamp too:
        // from commits alone a segment longer than chase_stale_us starved its
        // own grace (field trace 2026-09-02, the periodic hitch).
        if (now_us > _last_activity_us &&
            (now_us - _last_activity_us) > _cfg.chase_stale_us) {
            return 0.0;                                // the stream really is gone
        }
        // CONSTRAINT: grace never outlives the coast (kCoastCapS).
        const double cap = std::fmin((double)_cfg.settle_grace_us * 1e-6,
                                     kCoastCapS);
        // A cadence forgotten by a hold longer than chase_stale_us is not a
        // dead stream (the staleness check above already said alive): coast
        // the full cap rather than braking at the first plan end after the
        // hold (field trace 2026-09-02, the settle after every long hold).
        double g = _est_ema_ok ? std::fmin(kSettleGraceMult * _est_dt_ema, cap) : cap;
        return g;
    }

    // ---- Starve-settle ------------------------------------------------------
    // The clock ran past a plan that ends moving and no fresh command
    // replanned it → plan a jerk-limited brake-to-rest from the end state
    // (velocity control interface; lands wherever braking lands, clamped by
    // the sampler at the walls). One-time boundary event.
    //
    // ...but ONLY after the grace window. A plan expiring a few milliseconds
    // before its successor arrives is not a starved stream, it is a network.
    // Measured on the firmware's Valence path, whose 5 ms segment-pacing
    // drain guarantees exactly that jitter: a 14-segment funscript chain
    // produced 14 settles and 27 PlanKind flips at 5 ms of arrival lag,
    // against 1 and 1 at 0 ms — the engine was reacting to the transport, not
    // to the sender. Each of those settles is a Ruckig brake plan that the
    // next segment preempts ~5 ms later, so it costs plan time, corrupts the
    // mode/plan telemetry, and (worst) bleeds the velocity the next segment
    // was counting on inheriting. The grace window holds at the endpoint for
    // ms-scale stream jitter (degrading smoothly) and keeps the brake for a
    // real starvation.
    // The §11.3 600 ms Valence deadman remains the actual starvation
    // authority; this window only stops the engine from panicking on ms-scale
    // pacing noise.
    void maybeSettle(uint64_t now_us) {
        promoteDue(now_us);
        // A scheduled successor means the stream is alive BY DEFINITION, and
        // it was planned from the coasted state the sampler will actually
        // render: a settle here would both lie about starvation and break the
        // join. Bounded by kAnchorMaxLeadUs.
        if (_sched_n > 0) return;
        if (_kind == PlanKind::None) return;        // nothing in flight to end
        if (_mode == Mode::Settle) {
            settleToIdle(now_us);
            return;
        }
        const double dur = planDuration();
        if (elapsedS(now_us) < dur) return;
        // A plan END is activity: the machine rendered content up to this
        // instant, whatever happens next (see noteActivity).
        noteActivity(_plan_start + (uint64_t)(dur * 1e6 + 0.5));

        double p, v, a;
        planEndState(p, v, a);
        if (std::fabs(v) <= kRestVel) {
            // Ended at rest — collapse to a plain hold. (No grace needed: a
            // hold IS the end state, and a fresh command replans from it
            // identically whether we collapsed or not.)
            _hold_pos = clampWindow(p);
            _kind     = PlanKind::None;
            _mode     = Mode::Idle;
            return;
        }

        // Grace: the state COASTS at the end velocity (sampleRaw); measured
        // from PLAN EXPIRY, never re-arms.
        const double grace = settleGraceS(now_us);
        if (elapsedS(now_us) - dur < grace) return;

        // Brake from the coasted state at the anchor instant (plan end +
        // grace); grace = 0 keeps the pre-0.9 start state exactly.
        //
        // READ that state from sampleRaw rather than recomputing it. The two
        // used to be separate copies of "position plus end velocity times
        // elapsed", so the coast's own caps applied to what the SAMPLER
        // reported and not to what the brake was PLANNED FROM: the brake
        // entered kCoastCapS * vmax outside the window on a state the sampler
        // had already stopped advancing. One coast, one definition.
        const uint64_t coast_end_us =
            _plan_start + (uint64_t)(dur * 1e6 + 0.5)
                        + (uint64_t)(grace * 1e6 + 0.5);
        double cp, cv, ca;
        sampleRaw(coast_end_us, cp, cv, ca);
        // Anchor the settle at the moment the COAST ended (plan end + grace),
        // not at this sample's clock, so the brake follows the coasted state
        // seamlessly. Anchoring at plan end alone would be a bug once a grace
        // exists: the brake profile would be entered `grace` seconds deep, and
        // the very first sample would JUMP up to vmax·grace (30 mm on the
        // operator's 200 mm window at a 30 ms grace). With grace = 0 this is
        // byte-identical to the pre-0.4 anchor.
        planBrake(cp, cv, ca, coast_end_us, p, v, /*starved=*/true);
    }

    // The one brake-to-rest planner, SETTLE's and brake()'s: from (cp, cv, ca)
    // at end_us, jerk-limited through Ruckig's velocity interface, windowed.
    // (p, v) are what the anomaly records name.
    void planBrake(double cp, double cv, double ca, uint64_t end_us, double p, double v,
                   bool starved) {
        ruckig::InputParameter<1> in;
        in.control_interface       = ruckig::ControlInterface::Velocity;
        in.current_position[0]     = cp;
        in.current_velocity[0]     = cv;
        in.current_acceleration[0] = ca;
        in.target_velocity[0]      = 0.0;
        in.target_acceleration[0]  = 0.0;
        in.max_velocity[0]         = _cfg.limits.vmax;
        in.max_acceleration[0]     = _cfg.limits.amax;
        in.max_jerk[0]             = _cfg.limits.jmax;

        ruckig::Trajectory<1> traj;
        const ruckig::Result res = _calc.calculate(in, traj);
        if ((int)res < 0) {
            // Should be unreachable: a brake from a legal state is always
            // feasible — hard-hold the end position.
            _failures++;
            recordAnomaly(AnomalyType::PlanFailed, (float)p,
                          (float)(int)res, end_us);
            _hold_pos = clampWindow(p);
            _kind     = PlanKind::None;
            _mode     = Mode::Idle;
            return;
        }
        // WINDOW THE BRAKE. A velocity-interface brake has no position target,
        // so it lands wherever braking lands: at field limits v^2/2a is 12.5 mm
        // past the rail on a 100 mm window, and the hold clamp then erases the
        // discrepancy, leaving the engine's belief and the machine's position
        // different by exactly the overshoot (sd-6b2.2). Re-planned as
        // a POSITION move to the rail it would cross, arriving at rest, the
        // plan ENDS where the machine ends. The transient excursion that is
        // physics (you cannot stop in less than v^2/2a) survives either way;
        // the false belief does not. One extra solve, only when the brake needs
        // it.
        if (ruckigWorstRatio(traj, -1.0) > 1.0 + kRuckigLegalEps) {
            in.control_interface  = ruckig::ControlInterface::Position;
            in.target_position[0] = cv > 0.0 ? 1.0 : 0.0;
            in.target_velocity[0] = 0.0;
            ruckig::Trajectory<1> railed;
            if ((int)_calc.calculate(in, railed) >= 0 &&
                std::isfinite(railed.get_duration()))
                traj = railed;
        }
        if (starved)
            recordAnomaly(AnomalyType::SettleEngaged, (float)p, (float)v, end_us);
        _traj       = traj;
        _kind       = PlanKind::Ruckig;
        _plan_start = end_us;
        _plan_p0    = cp;
        _mode       = Mode::Settle;
        _plan_jerk_frac = 1.0f;   // a brake is planned at the full ceiling
        _plans++;
    }

    // A finished SETTLE collapses to Idle hold at its landing position.
    void settleToIdle(uint64_t now_us) {
        if (_kind == PlanKind::None || _mode != Mode::Settle) return;
        if (elapsedS(now_us) < planDuration()) return;
        double p, v, a;
        planEndState(p, v, a);
        noteActivity(_plan_start + (uint64_t)(planDuration() * 1e6 + 0.5));
        _hold_pos = clampWindow(p);
        _kind     = PlanKind::None;
        _mode     = Mode::Idle;
    }

    // ---- The schedule queue -------------------------------------------------
    // Anchored plans in strictly increasing anchor order, a ring so promotion
    // and refusal cost no copies. Slots at or past _sched_n are scratch.
    PlanSlot&       schedAt(size_t i)       { return _sched[(_sched_head + i) % kScheduleDepth]; }
    const PlanSlot& schedAt(size_t i) const { return _sched[(_sched_head + i) % kScheduleDepth]; }

    void dropScheduledFrom(uint64_t t_us) {
        while (_sched_n > 0 && schedAt(_sched_n - 1).start >= t_us) --_sched_n;
    }

    void swapPlan(PlanSlot& s) {
        std::swap(_traj, s.traj);
        for (int i = 0; i < 6; i++) std::swap(_q_c[i], s.q_c[i]);
        std::swap(_q_T, s.q_T);
        std::swap(_plan_p0, s.p0);
        std::swap(_plan_start, s.start);
        std::swap(_kind, s.kind);
        std::swap(_mode, s.mode);
        std::swap(_plan_jerk_frac, s.jerk_frac);
    }

    // Promotion is IN PLACE at the anchor the slot was planned for, so the
    // geometry joins with no jump. The activity clock is stamped here, at that
    // anchor, and not when the command arrived. Loops: a service pass that
    // straddles two anchors promotes both, in order.
    void promoteDue(uint64_t now_us) {
        while (_sched_n > 0 && now_us >= schedAt(0).start) {
            const uint64_t at = schedAt(0).start;
            swapPlan(schedAt(0));
            _sched_head = (_sched_head + 1) % kScheduleDepth;
            --_sched_n;
            noteActivity(at);
        }
    }

    void recordAnomaly(AnomalyType kind, float target, float detail,
                       uint64_t now_us) {
        Anomaly& slot = _anom_ring[_anom_write];
        slot.kind   = (uint8_t)kind;
        slot.seq    = _anom_seq++;
        slot.t_us   = now_us;
        slot.target = target;
        slot.detail = detail;
        _anom_write = (uint8_t)((_anom_write + 1) % kAnomalyDepth);
        if (_anom_count < kAnomalyDepth) _anom_count++;
    }

    // ---- State --------------------------------------------------------------
    Config                _cfg;
    ruckig::Ruckig<1>     _calc;        // offline calculate() only — no cycle time
    ruckig::Trajectory<1> _traj;        // active Ruckig plan (chase/guard/settle)
    double                _q_c[6] = {}; // active quintic (normalized tau)
    double                _q_T = 0.0;   // quintic duration, seconds
    PlanKind              _kind = PlanKind::None;
    uint64_t              _plan_start = 0;
    double                _hold_pos = 0.5;
    // Entry position of the active plan. The window the referees and the
    // output clamp judge against is [min(0, p0), max(1, p0)] (sd-6b2.10), so
    // an out-of-window plan may only move inward and an in-window one is
    // judged exactly as it was.
    double                _plan_p0 = 0.5;
    // Widest window this chain may answer to, from the seed (windowEntry).
    double                _seed_lo = 0.0;
    double                _seed_hi = 1.0;
    Mode                  _mode = Mode::Idle;
    PlanSlot              _sched[kScheduleDepth];   // schedule queue, ring
    size_t                _sched_head = 0;
    size_t                _sched_n = 0;
    // Peak jerk of the ACTIVE plan as a fraction of limits.jmax (see
    // Snapshot::sharpness). Telemetry only — nothing in the sample path reads
    // it; the plan is already an immutable polynomial.
    float                 _plan_jerk_frac = 1.0f;
    // RFC-030: the curve family the ACTIVE waveform command declared (registry
    // curve_families numbering; 0 = undeclared). Adopted at commitWaveform so
    // every re-solve of the same plan resolves FollowClient identically.
    uint8_t               _client_curve_family = 0;
    // Overshoot allowance for the commit IN PROGRESS, in window fractions.
    // < 0 = not armed, which is also the resting value. INVARIANT: every commit
    // path writes it before any referee runs (commitWaveform disarms at entry
    // and arms from its own entry state, commitChase disarms), and the referees
    // take it as a PARAMETER.
    double                _oshoot_allow = -1.0;

    // Stream estimator
    bool     _est_valid = false;
    bool     _est_ema_ok = false;
    double   _est_v_ema = 0.0;
    double   _est_sp_pk = 0.0;    // peak-hold |chord speed|, released
    double   _est_a_ema = 0.0;
    double   _est_dt_ema = 0.0;
    double   _est_last_target = 0.5;
    uint64_t _est_last_us = 0;

    // Previous wire G (for the backward-difference af estimate)
    bool     _prev_vf_ok = false;
    double   _prev_vf = 0.0;
    uint64_t _prev_vf_us = 0;
    // THE activity clock: the last instant the machine was known to be
    // executing content, engine clock. Stamped by every commit (at its
    // anchor) and every plan end; EVERY "is the stream alive" test reads
    // this one member (docs/reviews/kinetic-2026-09-02).
    uint64_t _last_activity_us = 0;
    bool     _reset_cold = true;   // resetAt: the next plan is a cold start
    Limits   _plan_lim;            // ceilings of the plan in flight (commit())
    // Previous waveform TARGET (dwell rule): a hold is the same target
    // re-commanded, never just "happens to be near" -- a shortened chain lands
    // near its NEXT target legitimately.
    bool     _prev_wave_tgt_ok = false;
    double   _prev_wave_tgt = 0.0;
    // The arrival chain (RFC-084): the anchor the last planned chase arrival
    // reaches, where the next future point's plan starts.
    bool     _chase_arrive_ok = false;
    uint64_t _chase_arrive_us = 0;

    // Counters + anomaly ring
    uint32_t _plans = 0;
    uint32_t _failures = 0;
    Anomaly  _anom_ring[kAnomalyDepth];
    uint8_t  _anom_write = 0;
    uint8_t  _anom_count = 0;
    uint16_t _anom_seq = 0;
};

} // namespace kinetic
