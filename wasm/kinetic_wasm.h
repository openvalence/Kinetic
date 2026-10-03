// kinetic_wasm -- the engine-only C ABI: one kinetic::Engine per handle, fed
// segments in millimeters and stepped on a caller-owned clock
// Constraints:
// - Fixed layouts, no padding, little-endian. A field is only ever appended.
// - Times are the handle's own clock, microseconds from create; only
//   kinetic_step advances it.
// - Millimeters are the travel window's frame; the engine frame is that window
//   normalized to 0..1.
// - SINGLE-THREADED per handle.
// See: README.md "WebAssembly"
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the ABI structs --------------------------------------------------------

// One evaluated tick, 64 bytes.
typedef struct kinetic_sample {
    double   t_us;           // handle clock at this sample
    double   p;              // plan position, engine frame, UNCLAMPED
    double   v;              // engine frame units/s
    double   a;              // engine frame units/s^2
    float    position_mm;    // window-clamped plan position
    float    velocity_mm_s;
    float    accel_mm_s2;
    float    target_mm;      // where the active plan ends
    uint32_t anomalies;      // bit k: kinetic::AnomalyType k recorded since the previous step
    uint8_t  mode;           // kinetic::Mode
    uint8_t  plan_kind;      // kinetic::PlanKind
    uint8_t  flags;          // KINETIC_FLAG_* below
    uint8_t  reserved;       // zero
    uint32_t plans;          // successful plans since create or set_window
    uint32_t reserved2;      // zero
} kinetic_sample;

// kinetic_sample::flags
#define KINETIC_FLAG_BUSY     0x01u  // the plan has motion left to render
#define KINETIC_FLAG_SHAPED   0x02u  // Blend spent amplitude or shape to hold a deadline
#define KINETIC_FLAG_FALLBACK 0x04u  // the Ruckig guard took a segment, or a deadline stretched
#define KINETIC_FLAG_CLAMPED  0x08u  // raw p is outside the window: the output backstop is acting
#define KINETIC_FLAG_REFUSED  0x10u  // a submit since the previous step was refused

typedef struct kinetic_handle kinetic_handle;

// ---- the calls --------------------------------------------------------------

// Ceilings in mm units over a rail of rail_mm; window = the whole rail, at
// rest at 0 mm, default kinetic::Config. Null on a non-finite or non-positive
// argument, or when out of memory.
kinetic_handle* kinetic_create(float vmax_mm_s, float amax_mm_s2, float jmax_mm_s3, float rail_mm);
void kinetic_destroy(kinetic_handle* h);

// 1 applied, 0 refused (non-finite, hi <= lo, or outside 0..rail). Voids the
// plan and re-seeds at rest at the current position, limits re-derived for the
// new span.
int kinetic_set_window(kinetic_handle* h, float lo_mm, float hi_mm);

// One waveform segment: reach target_mm at start_us + dur_ms. end_vel_mm_s NaN
// = unspecified. start_us at or before now = due now. curve_family: 0
// unspecified, 1 C1 cubic, 2 C2 quintic. 1 accepted, 0 refused by the planner
// (anchor beyond its lead bound, queue full, non-finite), -1 zero duration.
int kinetic_submit_segment(kinetic_handle* h, float target_mm, uint32_t dur_ms,
                           float end_vel_mm_s, double start_us, uint8_t curve_family);

// Advances the clock by dt_s, rounded to whole microseconds, and samples.
void kinetic_step(kinetic_handle* h, double dt_s, kinetic_sample* out);

double kinetic_now_us(const kinetic_handle* h);

// "kinetic <kinetic::kVersion>", static storage.
const char* kinetic_version(void);

#ifdef __cplusplus
}
#endif
