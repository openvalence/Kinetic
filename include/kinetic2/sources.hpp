// kinetic2/sources.hpp -- how each wire source becomes a knot
// Constraints:
// - The engine sees knots only; this is the one place a segment or a sample
//   is interpreted (RFC-105 promise 1 and 2 are promises about THESE lines).
// - A sample is a knot at arrival + the declared latency: one sample behind,
//   interpolated between the two knots the engine knows, never extrapolated
//   past the newest. The latency is the hub's schedule_latency_us for the
//   grant, exact, not a budget.
// - A segment is a knot at anchor + duration carrying the target and the
//   sender's end velocity, the knot's angle when given (types.hpp junctionOf).
//   With no end velocity it rests when it is the last knot (SPEC 9.6) and
//   runs through once a successor is queued.
#pragma once

#include <cstdint>

#include "types.hpp"

namespace kinetic2 {

// A bare sample that arrived at arrival_us, rendered latency_us later.
constexpr Knot knotFromSample(float p, uint64_t arrival_us, uint32_t latency_us) {
    Knot k;
    k.t_us = arrival_us + latency_us;
    k.p = p;
    k.has_v = false;            // position only: a run of samples is the chase (solver.hpp chaseRun)
    k.sample = true;
    return k;
}

// A timed segment: reach target at anchor + duration, with the sender's end
// velocity when it gave one (the wire sentinel for "none" is the caller's).
constexpr Knot knotFromSegment(float target, uint32_t duration_us, bool has_end_vel, float end_vel,
                               uint64_t anchor_us) {
    Knot k;
    k.t_us = anchor_us + duration_us;
    k.p = target;
    k.has_v = has_end_vel;
    k.v = has_end_vel ? end_vel : 0.0f;
    k.rest_if_last = !has_end_vel;   // SPEC 9.6: unspecified with no successor is rest
    return k;
}

}  // namespace kinetic2
