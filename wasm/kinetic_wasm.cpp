// kinetic_wasm -- kinetic_wasm.h on one kinetic::Engine per handle: the mm
// frame, the clock and the sample, nothing else
// Constraints:
// - NO PLANNING LIVES HERE. Shaping, policies and the Ruckig guard are
//   kinetic.hpp; a planning rule written here is one no engine consumer runs.
// - DETERMINISTIC: an integer microsecond clock owned by the handle, no wall
//   clock, no randomness. Same calls in, same bits out on every IEEE-754 host
//   built with -ffp-contract=off and without -ffast-math.
// - One heap allocation per handle, at create (the engine is KB-scale);
//   nothing allocates on submit or step.
// - The caller's stack must hold commit()'s KB-scale Ruckig temporaries
//   (CMakeLists.txt sizes the wasm stack).
// See: kinetic_wasm.h (the contract), README.md "WebAssembly"

#include "kinetic_wasm.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <new>

#include "kinetic/kinetic.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define KINETIC_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define KINETIC_EXPORT extern "C"
#endif

static_assert(sizeof(kinetic_sample) == 64, "kinetic_sample layout is ABI");
static_assert(offsetof(kinetic_sample, position_mm) == 32 &&
                  offsetof(kinetic_sample, anomalies) == 48 &&
                  offsetof(kinetic_sample, mode) == 52 &&
                  offsetof(kinetic_sample, plans) == 56,
              "kinetic_sample layout is ABI");
static_assert(uint8_t(kinetic::AnomalyType::DwellZeroed) < 32, "the anomaly mask is one word");

namespace {

using kinetic::AnomalyType;

constexpr uint32_t kindBit(AnomalyType k) { return 1u << uint8_t(k); }

constexpr uint32_t kShapedMask   = kindBit(AnomalyType::WaveformScaled) | kindBit(AnomalyType::WaveformSmoothed);
constexpr uint32_t kFallbackMask = kindBit(AnomalyType::WaveformFallback) | kindBit(AnomalyType::DeadlineStretched);
constexpr uint32_t kRefusedMask  = kindBit(AnomalyType::PlanFailed);

bool positive(float x) { return std::isfinite(x) && x > 0.0f; }

}  // namespace

// ---- the handle -------------------------------------------------------------

struct kinetic_handle {
    float vmax, amax, jmax, rail;   // mm units, as created
    float lo = 0.0f, hi = 0.0f;     // the window, mm
    uint64_t now_us = 0;
    bool refused = false;
    kinetic::Engine engine;

    kinetic_handle(float v, float a, float j, float r)
        : vmax(v), amax(a), jmax(j), rail(r), hi(r), engine(kinetic::Config{}, 0.0f) {
        engine.setLimits(limitsFor(hi - lo));
    }

    kinetic::Limits limitsFor(float span) const {
        kinetic::Limits l;
        l.vmax = vmax / span;
        l.amax = amax / span;
        l.jmax = jmax / span;
        return l;
    }

    float toNorm(float mm) const { return (mm - lo) / (hi - lo); }
    float toMm(double norm) const { return float(lo + norm * double(hi - lo)); }
};

// ---- the C ABI --------------------------------------------------------------

KINETIC_EXPORT kinetic_handle* kinetic_create(float vmax_mm_s, float amax_mm_s2, float jmax_mm_s3,
                                              float rail_mm) {
    if (!(positive(vmax_mm_s) && positive(amax_mm_s2) && positive(jmax_mm_s3) && positive(rail_mm)))
        return nullptr;
    return new (std::nothrow) kinetic_handle(vmax_mm_s, amax_mm_s2, jmax_mm_s3, rail_mm);
}

KINETIC_EXPORT void kinetic_destroy(kinetic_handle* h) { delete h; }

// The seed is the raw plan position carried into the new frame, in or out of
// the new window: the engine plans an inward entry from an honest seed.
KINETIC_EXPORT int kinetic_set_window(kinetic_handle* h, float lo_mm, float hi_mm) {
    if (h == nullptr) return 0;
    if (!(std::isfinite(lo_mm) && std::isfinite(hi_mm) && hi_mm > lo_mm && lo_mm >= 0.0f &&
          hi_mm <= h->rail))
        return 0;
    double p, v, a;
    h->engine.rawSampleAt(h->now_us, p, v, a);
    const float pos_mm = h->toMm(p);
    h->lo = lo_mm;
    h->hi = hi_mm;
    h->engine.setLimits(h->limitsFor(hi_mm - lo_mm));
    h->engine.resetAt(h->toNorm(pos_mm), h->now_us);
    return 1;
}

KINETIC_EXPORT int kinetic_submit_segment(kinetic_handle* h, float target_mm, uint32_t dur_ms,
                                          float end_vel_mm_s, double start_us, uint8_t curve_family) {
    if (h == nullptr) return 0;
    if (dur_ms == 0) return -1;
    kinetic::Command c;
    c.target       = h->toNorm(target_mm);
    c.duration_us  = dur_ms * 1000u;
    c.has_duration = true;
    c.has_end_vel  = !std::isnan(end_vel_mm_s);
    c.end_vel      = c.has_end_vel ? end_vel_mm_s / (h->hi - h->lo) : 0.0f;
    c.has_anchor   = true;
    c.anchor_us    = start_us > double(h->now_us) ? uint64_t(std::llround(start_us)) : h->now_us;
    c.client_curve_family = curve_family;
    if (h->engine.commit(c, h->now_us)) return 1;
    h->refused = true;
    return 0;
}

KINETIC_EXPORT void kinetic_step(kinetic_handle* h, double dt_s, kinetic_sample* out) {
    if (h == nullptr || out == nullptr) return;
    const double whole_us = std::round(dt_s * 1e6);
    if (whole_us > 0.0) h->now_us += uint64_t(whole_us);

    double p, v, a;
    h->engine.rawSampleAt(h->now_us, p, v, a);   // engages SETTLE first, as the sampler does
    const kinetic::Snapshot s = h->engine.snapshot(h->now_us);
    const kinetic::PlanView pv = h->engine.planView();

    uint32_t mask = 0;
    kinetic::Anomaly an;
    while (h->engine.popAnomaly(an))
        if (an.kind < 32) mask |= 1u << an.kind;

    uint8_t flags = 0;
    if (h->engine.isBusy(h->now_us)) flags |= KINETIC_FLAG_BUSY;
    if (mask & kShapedMask) flags |= KINETIC_FLAG_SHAPED;
    if (mask & kFallbackMask) flags |= KINETIC_FLAG_FALLBACK;
    if (p < pv.lo || p > pv.hi) flags |= KINETIC_FLAG_CLAMPED;
    if (h->refused || (mask & kRefusedMask)) flags |= KINETIC_FLAG_REFUSED;
    h->refused = false;

    const float span = h->hi - h->lo;
    std::memset(out, 0, sizeof(*out));
    out->t_us          = double(h->now_us);
    out->p             = p;
    out->v             = v;
    out->a             = a;
    out->position_mm   = h->toMm(s.pos);
    out->velocity_mm_s = s.vel * span;
    out->accel_mm_s2   = float(a) * span;
    out->target_mm     = h->toMm(s.target);
    out->anomalies     = mask;
    out->mode          = s.mode;
    out->plan_kind     = s.plan_kind;
    out->flags         = flags;
    out->plans         = s.plans;
}

KINETIC_EXPORT double kinetic_now_us(const kinetic_handle* h) {
    return h != nullptr ? double(h->now_us) : 0.0;
}

KINETIC_EXPORT const char* kinetic_version() {
    static char buf[32] = {};
    if (buf[0] == '\0') {
        size_t n = 0;
        for (const char* part : {"kinetic ", kinetic::kVersion})
            for (; *part != '\0' && n + 1 < sizeof(buf); ++part) buf[n++] = *part;
    }
    return buf;
}
