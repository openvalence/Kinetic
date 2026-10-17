// kinetic2_wasm -- the Kinetic² C ABI: one kinetic2::Engine<1> per handle, fed
// knots in window units on a caller-owned clock.
// Constraints:
// - Fixed layouts, no padding, little-endian. A field is only ever appended.
// - Times are microseconds on the caller's clock; the engine never advances
//   it. kinetic2_sample must be called with non-decreasing times between
//   resets: sampling retires knots the clock has passed.
// - Positions are window units, 0..1 over the travel window.
// - SINGLE-THREADED per handle.
// See: README.md "Playground and WebAssembly"
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the ABI structs --------------------------------------------------------

// The state at one instant, 24 bytes (read by kinetic2_sample).
typedef struct kinetic2_state {
    double   t_us;        // the time sampled
    float    p;           // window units
    float    v;           // window units/s
    float    a;           // window units/s^2
    uint8_t  busy;        // 1 while the timeline or a brake has motion left
    uint8_t  reserved[3]; // zero
} kinetic2_state;

// One solved knot, 40 bytes: where the curve will pass and what the trim cost.
typedef struct kinetic2_knot {
    double   t_us;        // solved time (a corner's ramp end; a live jog may land later)
    float    p;
    float    v;           // the junction velocity the solver chose or kept
    float    a;           // the junction acceleration
    float    share;       // the share of the stroke a trim kept, 1 = whole
    float    stretched_s; // a live jog's seconds past its time; else 0
    float    worst;       // the incoming piece's worst ceiling ratio after the trim
    uint8_t  dropped;     // always 0: a knot is never dropped
    uint8_t  clamped;     // an authored velocity was cut
    uint8_t  pin_v;       // always 0; ABI field
    uint8_t  pin_a;       // always 0; ABI field
    uint8_t  reserved[4]; // zero
} kinetic2_knot;

// One recorded anomaly, 24 bytes. kind: kinetic2::AnomalyKind.
typedef struct kinetic2_anomaly {
    double   t_us;
    float    target;      // the knot's p
    float    detail;      // kind-specific (types.hpp)
    uint16_t seq;
    uint8_t  kind;
    uint8_t  reserved[5]; // zero
} kinetic2_anomaly;

// One knot as the last render left it, 56 bytes (read by kinetic2_render): what
// the handles page draws. Lengths are shares of the span on that side.
typedef struct kinetic2_render_knot {
    float    p;           // where the render passes it: authored plus dp, window units
    float    dp;          // the trim, toward the knot before; 0 = not moved
    float    vel;         // the angle, window units/s
    float    l_in, l_out;     // the handle lengths the style solved
    float    eff_in, eff_out; // the lengths after the ceilings
    float    span_in, span_out; // seconds to the knot before and after in the render; 0 none
    float    share;       // kinetic2_knot's; 1 for the origin
    float    worst;       // kinetic2_knot's; 0 for the origin
    float    ramp_s;      // a corner ramp's duration, s; 0 none
    uint8_t  cls;         // kinetic2::handles::KnotClass: 0 end, 1 rest, 2 crest, 3 through
    uint8_t  g2;          // 1 renders G2, 0 G1
    uint8_t  corner;      // a corner ramp at this knot
    uint8_t  infeasible;  // the piece into it is over a ceiling after the whole trim
    uint8_t  clamped;     // kinetic2_knot's
    uint8_t  reserved[3]; // zero
} kinetic2_render_knot;

typedef struct kinetic2_handle kinetic2_handle;

// ---- the calls --------------------------------------------------------------

// Default kinetic2::Config, at rest at 0.5 at time 0. Null when out of memory.
kinetic2_handle* kinetic2_create(void);
void kinetic2_destroy(kinetic2_handle* h);

// The ceilings and the planner options (kinetic2::Config): smoothness 0..1,
// handle_floor at least 0.05 and under a third, trim_max 0..1. 1 applied, 0
// refused (a non-finite or non-positive ceiling, an option out of its range).
// Takes effect at the next submit or reset; the committed curve is never
// re-planned.
int kinetic2_configure(kinetic2_handle* h, float vmax, float amax, float jmax,
                       float smoothness, float handle_floor, float trim_max,
                       uint32_t react_us);

// Clears the timeline and anomalies; at rest at p from now_us.
void kinetic2_reset(kinetic2_handle* h, float p, double now_us);

// One knot: the curve passes p at t_us. has_v 0 leaves the angle to the
// solver. now_us is when the sender submits it: a knot arriving while the
// axis moves re-plans from the reaction horizon. flags: bit 0 = sample (a run
// of position-only samples is the chase; a sample with has_v and v = 0 is a
// live jog, the fastest move to rest on it), bit 1 = rest if last (accepted
// and not read: every newest free knot rests until a successor frees it,
// SPEC 9.6, whatever the bit says). 1 accepted, 0 refused (an anomaly says
// why: KnotRefused with a detail sentinel from types.hpp).
int kinetic2_submit(kinetic2_handle* h, double t_us, float p, int has_v, float v,
                    double now_us, uint32_t flags);
#define KINETIC2_KNOT_SAMPLE       0x1u
#define KINETIC2_KNOT_REST_IF_LAST 0x2u

// Brake to rest from the state at now_us; knots before the brake ends are
// refused. 1 when anything was moving.
int kinetic2_brake(kinetic2_handle* h, double now_us);

// The segments flush (Valence RFC-087): drops every pending knot at or after
// t_us; the motion in flight hands off there, or at the reaction horizon when
// t_us is not past it (kinetic2::Engine::truncateAfter). A knot after the
// hand-off is then accepted. Returns the knots taken off the window (dropped,
// or committed through the reaction horizon); 0 changed nothing.
uint32_t kinetic2_truncate_after(kinetic2_handle* h, double t_us, double now_us);

// The state at now_us. Non-decreasing between resets.
void kinetic2_sample(kinetic2_handle* h, double now_us, kinetic2_state* out);

// Knots still ahead on the timeline, and the i-th of them as solved.
// 1 written, 0 when i is out of range.
uint32_t kinetic2_pending(kinetic2_handle* h);
int kinetic2_solved(kinetic2_handle* h, uint32_t i, kinetic2_knot* out);

// The pending window as the solve rendered it: i = 0 the origin, i = 1..pending
// pending knot i - 1. Solves first. 1 written; 0 when i is out of range or the
// window did not render as one handle run (nothing pending, or a sample or
// live-jog knot in it: those are moves, not renders).
int kinetic2_render(kinetic2_handle* h, uint32_t i, kinetic2_render_knot* out);

// Oldest recorded anomaly, 1 written, 0 when none. The ring keeps 16.
int kinetic2_pop_anomaly(kinetic2_handle* h, kinetic2_anomaly* out);

// "kinetic2 <kinetic2::kVersion>", static storage.
const char* kinetic2_version(void);

#ifdef __cplusplus
}
#endif
