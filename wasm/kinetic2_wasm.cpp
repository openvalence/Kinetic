// kinetic2_wasm -- the Kinetic² C ABI behind kinetic2_wasm.h. Plumbing only:
// every planning decision is the engine's.
// See: README.md "WebAssembly"
#include "kinetic2_wasm.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <new>

#include "kinetic2/engine.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define KINETIC_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define KINETIC_EXPORT extern "C"
#endif

static_assert(sizeof(kinetic2_state) == 24, "kinetic2_state layout is ABI");
static_assert(sizeof(kinetic2_knot) == 40, "kinetic2_knot layout is ABI");
static_assert(sizeof(kinetic2_anomaly) == 24, "kinetic2_anomaly layout is ABI");
static_assert(offsetof(kinetic2_knot, dropped) == 32, "kinetic2_knot layout is ABI");
static_assert(offsetof(kinetic2_anomaly, seq) == 16, "kinetic2_anomaly layout is ABI");

namespace {

bool positive(float x) { return std::isfinite(x) && x > 0.0f; }

uint64_t toUs(double t) {
    if (!(t > 0.0)) return 0;
    return uint64_t(t + 0.5);
}

}  // namespace

struct kinetic2_handle {
    kinetic2::Engine<1, 64> engine{kinetic2::Config{}};
};

KINETIC_EXPORT kinetic2_handle* kinetic2_create(void) {
    return new (std::nothrow) kinetic2_handle{};
}

KINETIC_EXPORT void kinetic2_destroy(kinetic2_handle* h) { delete h; }

KINETIC_EXPORT int kinetic2_configure(kinetic2_handle* h, float vmax, float amax, float jmax,
                                      uint8_t policy, float amplitude_floor, uint32_t lookahead_us,
                                      uint8_t corner, uint32_t react_us) {
    if (!h || !positive(vmax) || !positive(amax) || !positive(jmax)) return 0;
    if (!(amplitude_floor >= 0.0f && amplitude_floor <= 1.0f)) return 0;
    if (policy != uint8_t(kinetic2::Policy::Stretch) && policy != uint8_t(kinetic2::Policy::Blend)) return 0;
    if (corner > uint8_t(kinetic2::Corner::Cubic)) return 0;
    kinetic2::Config c = h->engine.config();
    c.limits = kinetic2::Limits{vmax, amax, jmax};
    c.policy = kinetic2::Policy(policy);
    c.amplitude_floor = amplitude_floor;
    c.lookahead_us = lookahead_us;
    c.corner = kinetic2::Corner(corner);
    c.react_us = react_us;
    h->engine.setConfig(c);
    return 1;
}

KINETIC_EXPORT void kinetic2_reset(kinetic2_handle* h, float p, double now_us) {
    if (!h) return;
    if (!std::isfinite(p)) p = 0.5f;
    h->engine.resetAt(p, toUs(now_us));
    kinetic2::Anomaly drop;
    while (h->engine.popAnomaly(drop)) {}
}

KINETIC_EXPORT int kinetic2_submit(kinetic2_handle* h, double t_us, float p, int has_v, float v,
                                   uint8_t family, double now_us, uint32_t flags) {
    if (!h) return 0;
    kinetic2::Knot k;
    k.t_us = toUs(t_us);
    k.p = p;
    k.has_v = has_v != 0;
    k.v = k.has_v ? v : 0.0f;
    k.sample = (flags & KINETIC2_KNOT_SAMPLE) != 0;
    k.rest_if_last = (flags & KINETIC2_KNOT_REST_IF_LAST) != 0;
    k.family = family <= uint8_t(kinetic2::Family::Step) ? kinetic2::Family(family) : kinetic2::Family::Unspecified;
    return h->engine.submit(k, toUs(now_us)) ? 1 : 0;
}

KINETIC_EXPORT int kinetic2_brake(kinetic2_handle* h, double now_us) {
    if (!h) return 0;
    return h->engine.brake(toUs(now_us)) ? 1 : 0;
}

KINETIC_EXPORT void kinetic2_sample(kinetic2_handle* h, double now_us, kinetic2_state* out) {
    if (!h || !out) return;
    const uint64_t t = toUs(now_us);
    const kinetic2::State s = h->engine.stateAt(0, t);
    std::memset(out, 0, sizeof *out);
    out->t_us = double(t);
    out->p = s.p;
    out->v = s.v;
    out->a = s.a;
    out->busy = h->engine.isBusy(t) ? 1 : 0;
}

KINETIC_EXPORT uint32_t kinetic2_pending(kinetic2_handle* h) {
    return h ? uint32_t(h->engine.pending(0)) : 0;
}

KINETIC_EXPORT int kinetic2_solved(kinetic2_handle* h, uint32_t i, kinetic2_knot* out) {
    if (!h || !out || i >= h->engine.pending(0)) return 0;
    const kinetic2::Solved& s = h->engine.solved(0, i);
    std::memset(out, 0, sizeof *out);
    out->t_us = double(s.t_us);
    out->p = s.p;
    out->v = s.v;
    out->a = s.a;
    out->share = s.share;
    out->stretched_s = s.stretched_s;
    out->worst = s.worst;
    out->dropped = s.dropped ? 1 : 0;
    out->clamped = s.clamped ? 1 : 0;
    out->pin_v = s.pin_v ? 1 : 0;
    out->pin_a = s.pin_a ? 1 : 0;
    return 1;
}

KINETIC_EXPORT int kinetic2_pop_anomaly(kinetic2_handle* h, kinetic2_anomaly* out) {
    if (!h || !out) return 0;
    kinetic2::Anomaly a;
    if (!h->engine.popAnomaly(a)) return 0;
    std::memset(out, 0, sizeof *out);
    out->t_us = double(a.t_us);
    out->target = a.target;
    out->detail = a.detail;
    out->seq = a.seq;
    out->kind = a.kind;
    return 1;
}

KINETIC_EXPORT const char* kinetic2_version(void) {
    static char text[32] = {};
    if (!text[0]) {
        std::strcpy(text, "kinetic2 ");
        std::strncat(text, kinetic2::kVersion, sizeof text - 10);
    }
    return text;
}
