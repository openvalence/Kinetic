// kinetic2_wasm -- the Kinetic² C ABI: one kinetic2::Engine<1> per handle, fed
// knots in window units on a caller-owned clock.
// Constraints:
// - Fixed layouts, no padding, little-endian. A field is only ever appended.
// - Times are microseconds on the caller's clock; the engine never advances
//   it. kinetic2_sample must be called with non-decreasing times between
//   resets: sampling retires knots the clock has passed.
// - Positions are window units, 0..1 over the travel window.
// - SINGLE-THREADED per handle.
// See: README.md "WebAssembly"
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

// One solved knot, 40 bytes: where the curve will pass and what the spend cost.
typedef struct kinetic2_knot {
    double   t_us;        // solved time (Stretch may have moved it later)
    float    p;
    float    v;           // the junction velocity the solver chose or kept
    float    a;           // the junction acceleration
    float    share;       // Blend: the share of the stroke kept, 1 = whole
    float    stretched_s; // Stretch: seconds added
    float    worst;       // the incoming piece's worst ceiling ratio after the spend
    uint8_t  dropped;     // unreachable under every spend: not rendered
    uint8_t  clamped;     // an authored velocity was cut
    uint8_t  pin_v;       // junction velocity fixed by a backward relaxation
    uint8_t  pin_a;
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

typedef struct kinetic2_handle kinetic2_handle;

// ---- the calls --------------------------------------------------------------

// Default kinetic2::Config, at rest at 0.5 at time 0. Null when out of memory.
kinetic2_handle* kinetic2_create(void);
void kinetic2_destroy(kinetic2_handle* h);

// The planner options (kinetic2::Config). policy: 0 Stretch, 5 Blend.
// corner: 0 Continuous, 1 Cubic. 1 applied, 0 refused (non-finite or
// non-positive ceiling, floor outside 0..1, unknown enum). Takes effect at the
// next submit or reset; the committed curve is never re-planned.
int kinetic2_configure(kinetic2_handle* h, float vmax, float amax, float jmax,
                       uint8_t policy, float amplitude_floor, uint32_t lookahead_us,
                       uint8_t corner, uint32_t react_us);

// Clears the timeline and anomalies; at rest at p from now_us.
void kinetic2_reset(kinetic2_handle* h, float p, double now_us);

// One knot: the curve passes p at t_us. has_v 0 leaves the junction to the
// solver; family: 0 unspecified, 1 C1, 2 C2 (C1 with v = 0 is a hard stop).
// now_us is when the sender submits it: a knot arriving while the axis moves
// re-plans from the reaction horizon. flags: bit 0 = sample (soft deadline,
// stretched alone, never trimmed), bit 1 = rest if last (a free knot with
// nothing after it rests, SPEC 9.6; a successor frees it). 1 accepted, 0
// refused (an anomaly says why: KnotRefused with a detail sentinel from
// types.hpp).
int kinetic2_submit(kinetic2_handle* h, double t_us, float p, int has_v, float v,
                    uint8_t family, double now_us, uint32_t flags);
#define KINETIC2_KNOT_SAMPLE       0x1u
#define KINETIC2_KNOT_REST_IF_LAST 0x2u

// Brake to rest from the state at now_us; knots before the brake ends are
// refused. 1 when anything was moving.
int kinetic2_brake(kinetic2_handle* h, double now_us);

// The state at now_us. Non-decreasing between resets.
void kinetic2_sample(kinetic2_handle* h, double now_us, kinetic2_state* out);

// Knots still ahead on the timeline, and the i-th of them as solved.
// 1 written, 0 when i is out of range.
uint32_t kinetic2_pending(kinetic2_handle* h);
int kinetic2_solved(kinetic2_handle* h, uint32_t i, kinetic2_knot* out);

// Oldest recorded anomaly, 1 written, 0 when none. The ring keeps 16.
int kinetic2_pop_anomaly(kinetic2_handle* h, kinetic2_anomaly* out);

// "kinetic2 <kinetic2::kVersion>", static storage.
const char* kinetic2_version(void);

#ifdef __cplusplus
}
#endif
