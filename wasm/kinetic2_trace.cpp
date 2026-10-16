// kinetic2_trace -- kinetic2.wasm's C ABI with the planner's trace hooks
// (include/kinetic2/trace.hpp) recording into one fixed buffer: the
// playground's planner steps (playground/steps.js reads it).
// Constraints:
// - A hook only reads: the 1 ms samples match kinetic2.wasm bit for bit
//   (playground/check.mjs asserts it).
// - The record layout is kinetic2_trace_rec, 96 bytes, and the K_* kinds;
//   steps.js reads both by these numbers. A field or kind is only appended.
// - Past kCap records are counted (kinetic2_trace_lost), never written.
// - Single-threaded, like the ABI: plain statics, never thread_local.
// See: playground/README.md "Planner steps"
#include <cstddef>
#include <cstdint>

namespace k2t {
template <class K> void pchip(int i, const K& m, float h1, float h2, float w1, float w2, float r);
void sweep(int s, int i, float AL, float BL, float AR, float BR, float lo, float hi, float v);
template <class K> void style(const K* k, int n, bool smooth, int phase);
template <class K> void round(const K* k, int n, int tag, int r);
template <class K> void piece(int i, bool hit, const K& L, const K& R, float pL);
template <class Q, class C> void judge(float f, const Q& q, float pL, const C& cs, float ro, float o);
template <class F> void fit(float pR, const F& f);
template <class K, class F> void pieceEnd(int i, const K& L, const K& R, const F& f, float dp, bool hold, bool canTrim);
template <class S, class C> void solve(const S& origin, uint64_t origin_us, std::size_t n, const C& c);
template <class K> void run(const K* k, const uint64_t* te, int m, int me, std::size_t base);
void pass(int p, bool rescue);
template <class K> void passEnd(const K* k, const float* built, int me, bool again, bool over);
template <class O> void runEnd(const O* out, int me);
template <class O> void chase(std::size_t i, std::size_t j, const O* out);
template <class O> void hard(std::size_t i, const O& o);
inline int phase = 0, retry = 0, judged = 0;
}  // namespace k2t

#define K2T_PCHIP(i, m, h1, h2, w1, w2, r) k2t::pchip(i, m, h1, h2, w1, w2, r)
#define K2T_SWEEP(s, i, AL, BL, AR, BR, lo, hi, v) k2t::sweep(s, i, AL, BL, AR, BR, lo, hi, v)
#define K2T_STYLE(k, n, smooth, ph) k2t::style(k, n, smooth, ph)
#define K2T_ROUND(k, n, tag, r) k2t::round(k, n, tag, r)
#define K2T_PIECE(i, hit, L, R, pL) k2t::piece(i, hit, L, R, pL)
#define K2T_PHASE(x) (k2t::phase = (x))
#define K2T_RETRY() (k2t::retry = 1)
#define K2T_JUDGE(f, q, pL, cs, ro, o) k2t::judge(f, q, pL, cs, ro, o)
#define K2T_FIT(pR, f) k2t::fit(pR, f)
#define K2T_PIECE_END(i, L, R, f, dp, hold, canTrim) k2t::pieceEnd(i, L, R, f, dp, hold, canTrim)
#define K2T_SOLVE(origin, origin_us, n, c) k2t::solve(origin, origin_us, n, c)
#define K2T_RUN(k, te, m, me, base) k2t::run(k, te, m, me, base)
#define K2T_PASS(p, rescue) k2t::pass(p, rescue)
#define K2T_PASS_END(k, built, me, again, over) k2t::passEnd(k, built, me, again, over)
#define K2T_RUN_END(out, me) k2t::runEnd(out, me)
#define K2T_CHASE(i, j, out) k2t::chase(i, j, out)
#define K2T_HARD(i, o) k2t::hard(i, o)

#include "kinetic2_wasm.cpp"

// One record: kind, three ints, twenty floats.
struct kinetic2_trace_rec {
    int32_t kind, a, b, c;
    float   f[20];
};
static_assert(sizeof(kinetic2_trace_rec) == 96, "kinetic2_trace_rec layout is read by steps.js");

namespace k2t {
namespace h = kinetic2::handles;
enum : int32_t {
    K_SOLVE = 1, K_RUN = 2, K_RKNOT = 3, K_PASS = 4, K_STYLE = 5, K_KNOT = 6, K_ROUND = 7, K_PIECE = 8,
    K_JUDGE = 9, K_FIT = 10, K_PIECE_END = 11, K_PASS_END = 12, K_BUILT = 13, K_EMIT = 14, K_CHASE = 15, K_HARD = 16,
    K_PCHIP = 17, K_SWEEP = 18,
};
inline constexpr uint32_t kCap = 131072;
static kinetic2_trace_rec g_rec[kCap];
static kinetic2_trace_rec g_spill;
static uint32_t g_n = 0, g_lost = 0;

kinetic2_trace_rec& put(int32_t kind, int32_t a = 0, int32_t b = 0, int32_t c = 0) {
    kinetic2_trace_rec& r = g_n < kCap ? g_rec[g_n++] : (++g_lost, g_spill);
    r = kinetic2_trace_rec{};
    r.kind = kind; r.a = a; r.b = b; r.c = c;
    return r;
}
template <class... T>
void fill(kinetic2_trace_rec& r, T... v) {
    static_assert(sizeof...(T) <= 20, "a record holds twenty floats");
    int i = 0;
    ((r.f[i++] = float(v)), ...);
}

// Every knot of a render as it stands: flags cls | g2 << 2 | has_v << 3 | infeasible << 4 | memo << 5.
template <class K>
void knots(const K* k, int n) {
    for (int i = 0; i < n; ++i) {
        const K& m = k[i];
        const int fl = int(m.cls) | int(m.g2) << 2 | int(m.has_v) << 3 | int(m.infeasible) << 4 | int(m.memo) << 5;
        fill(put(K_KNOT, i, fl), m.t, m.p, m.dp, m.vel, m.lIn, m.lOut, m.effIn, m.effOut, m.dIn, m.dOut, m.vcap, m.aIn,
             m.slack[0], m.slack[1], m.slack[2], m.rail, m.v);
    }
}
template <class K> void pchip(int i, const K& m, float h1, float h2, float w1, float w2, float r) {
    fill(put(K_PCHIP, i), h1, h2, w1, w2, m.dIn, m.dOut, r);
}
// Each knot's angle as the last hook saw it: a sweep's angle before is the
// start angle (style, phase 0) or the sweep before it (solveStyle changes a
// through point's angle only there).
static float g_vel[kinetic2::kWindowKnots + 2];
// One sweep at a through point: the closed form (the planner's own operands),
// the band, the angle before and after, and the acceleration each side of the
// knot reaches at each (AL + BL v from the left piece, AR + BR v from the right).
void sweep(int s, int i, float AL, float BL, float AR, float BR, float lo, float hi, float v) {
    const float v0 = g_vel[i];
    g_vel[i] = v;
    fill(put(K_SWEEP, s, i), (AR - AL) / (BL - BR), lo, hi, v0, v, AL + BL * v0, AR + BR * v0, AL + BL * v, AR + BR * v);
}
template <class K> void style(const K* k, int n, bool smooth, int ph) {
    put(K_STYLE, ph, smooth, n);
    knots(k, n);
    for (int i = 0; i < n && i < int(kinetic2::kWindowKnots + 2); ++i) g_vel[i] = k[i].vel;
}
template <class K> void round(const K* k, int n, int tag, int r) { put(K_ROUND, tag, r, n); knots(k, n); }
template <class K> void piece(int i, bool hit, const K& L, const K& R, float pL) {
    phase = 0; retry = 0; judged = 0;
    fill(put(K_PIECE, i, hit), pL, R.p, R.t - L.t, L.vel, R.vel, L.lOut, R.lIn, R.mi0, R.mi1, R.mdp, R.mlegal, L.t, R.t, L.dp, R.mvel);
}
// The judged piece and its peaks per ceiling, each as a ratio of the share it
// was judged against (cs), from overOf itself. Only the length walk at the
// knot as submitted (phase 0) is recorded; a trim test's walk is counted on
// its fit.
template <class Q, class C> void judge(float f, const Q& q, float pL, const C& cs, float ro, float o) {
    ++judged;
    if (phase != 0) return;
    h::Over pr;
    (void)h::overOf(q, pL, cs, INFINITY, &pr);
    fill(put(K_JUDGE, phase, o <= 1.0f + h::kTol, retry), f, q.T, q.D, q.s0, q.s1, q.i0, q.i1, pL, o, ro, pr.v, pr.a, pr.j, pr.x,
         cs.lim.vmax, cs.lim.amax, cs.lim.jmax);
}
template <class F> void fit(float pR, const F& f) {
    fill(put(K_FIT, phase, f.legal, retry), pR, f.k, f.i0, f.i1, f.o, f.s1, judged);
    judged = 0;
}
template <class K, class F> void pieceEnd(int i, const K& L, const K& R, const F& f, float dp, bool hold, bool canTrim) {
    fill(put(K_PIECE_END, i, f.legal, int(hold) | int(canTrim) << 1), f.k, f.i0, f.i1, f.o, dp, R.vel, L.vcap, L.aTarget, R.p,
         h::rendered(L), R.t - L.t, L.vel);
}
template <class S, class C> void solve(const S& o, uint64_t origin_us, std::size_t n, const C& c) {
    fill(put(K_SOLVE, int32_t(n), 0, int32_t(origin_us)), double(origin_us) * 1e-6, o.p, o.v, o.a, c.lim.vmax, c.lim.amax, c.lim.jmax,
         c.smoothness, c.lfloor, c.trim, c.lo, c.hi, c.holdEps);
}
template <class K> void run(const K* k, const uint64_t* te, int m, int me, std::size_t base) {
    fill(put(K_RUN, m, me, int32_t(base)), double(te[0]) * 1e-6);
    for (int r = 0; r < m; ++r) fill(put(K_RKNOT, r, k[r].has_v, int32_t(te[r])), k[r].t, k[r].p, k[r].v);
}
void pass(int p, bool rescue) { put(K_PASS, p, rescue); }
template <class K> void passEnd(const K* k, const float* built, int me, bool again, bool over) {
    put(K_PASS_END, me, again, over);
    for (int r = 1; r < me; ++r) fill(put(K_BUILT, r), built[r], k[r].slack[0], k[r].slack[1], k[r].slack[2], k[r].rail, k[r].vcap);
}
// The solved knots: flags corner | infeasible << 1 | hard << 2 | missed << 3 | clamped << 4.
template <class O> void runEnd(const O* out, int me) {
    for (int r = 1; r < me; ++r) {
        const O& o = out[r - 1];
        const int fl = int(o.corner) | int(o.infeasible) << 1 | int(o.hard) << 2 | int(o.missed) << 3 | int(o.clamped) << 4;
        fill(put(K_EMIT, r, fl), double(o.t_us) * 1e-6, o.p, o.v, o.a, o.share, o.worst, o.knot_p, double(o.head_us) * 1e-6,
             o.corner ? o.ramp.duration() : 0.0f, o.corner ? o.ramp.jerk[0] : 0.0f, o.i0, o.i1, o.a_in, o.stretched_s,
             double(o.base_us) * 1e-6);
    }
}
template <class O> void chase(std::size_t i, std::size_t j, const O* out) {
    for (std::size_t r = i; r < j; ++r)
        fill(put(K_CHASE, int32_t(r), int32_t(i), int32_t(j)), double(out[r].t_us) * 1e-6, out[r].p, out[r].v, out[r].a, out[r].worst,
             out[r].stretched_s, out[r].infeasible, double(out[r].base_us) * 1e-6);
}
template <class O> void hard(std::size_t i, const O& o) {
    fill(put(K_HARD, int32_t(i), o.infeasible), double(o.t_us) * 1e-6, o.p, o.worst, o.stretched_s, double(o.base_us) * 1e-6);
}
}  // namespace k2t

KINETIC_EXPORT const kinetic2_trace_rec* kinetic2_trace_records(void) { return k2t::g_rec; }
KINETIC_EXPORT uint32_t kinetic2_trace_count(void) { return k2t::g_n; }
KINETIC_EXPORT uint32_t kinetic2_trace_lost(void) { return k2t::g_lost; }
KINETIC_EXPORT void kinetic2_trace_clear(void) { k2t::g_n = 0; k2t::g_lost = 0; }

// One piece as the judge reads it, from the planner's own functions, into
// out[0..682): 101 x (u, t, p, v, a, j) by evalPiece; the judge's 11 samples
// x (u, v, a, j); the turning points of v (count, 3 x (u, v)), of a (count,
// 4 x (u, a)), of j (count, 5 x (u, j)) by rootsOf; then overOf's ratio and
// its parts (o, v, a, j, x). Returns the floats written.
KINETIC_EXPORT int kinetic2_trace_curve(float T, float D, float s0, float s1, float i0, float i1, float pL, float vmax, float amax,
                                        float jmax, float lo, float hi, float* out) {
    namespace h = kinetic2::handles;
    h::Piece q;
    q.T = T; q.D = D; q.s0 = s0; q.s1 = s1; q.i0 = i0; q.i1 = i1;
    h::Cfg c;
    c.lim = kinetic2::Limits{vmax, amax, jmax};
    c.lo = lo; c.hi = hi; c.railStop = true;
    int n = 0;
    for (int s = 0; s <= 100; ++s) {
        const float u = float(s) / 100.0f;
        const h::Eval e = h::evalPiece(q, u);
        for (const float x : {u, e.t, e.p, e.v, e.a, e.j}) out[n++] = x;
    }
    // overOf's sample order (handles.hpp, overOf kOrder).
    static constexpr float kOrder[11] = {0.0f, 1.0f, 0.5f, 0.2f, 0.8f, 0.4f, 0.6f, 0.1f, 0.9f, 0.3f, 0.7f};
    const h::Coef k = h::coefOf(q);
    for (const float u : kOrder) {
        const h::Deriv d = h::derivAt(k, u);
        for (const float x : {u, h::vAt(d), h::aAt(d), h::jAt(d)}) out[n++] = x;
    }
    float r[6];
    auto roots = [&](int cnt, int cap, auto&& val) {
        out[n] = float(cnt);
        for (int i = 0; i < cap; ++i) {
            out[n + 1 + 2 * i] = i < cnt ? r[i] : 0.0f;
            out[n + 2 + 2 * i] = i < cnt ? val(h::derivAt(k, r[i])) : 0.0f;
        }
        n += 1 + 2 * cap;
    };
    roots(h::rootsOf([&](float u) { return h::derivAt(k, u).nA; }, 6, r, 3), 3, [](const h::Deriv& d) { return h::vAt(d); });
    roots(h::rootsOf([&](float u) { return h::derivAt(k, u).nJ; }, 8, r, 4), 4, [](const h::Deriv& d) { return h::aAt(d); });
    roots(h::rootsOf([&](float u) { return h::jSlopeNum(k, u); }, 10, r, 5), 5, [](const h::Deriv& d) { return h::jAt(d); });
    h::Over pr;
    out[n++] = h::overOf(q, pL, c, INFINITY, &pr);
    for (const float x : {pr.v, pr.a, pr.j, pr.x}) out[n++] = x;
    return n;
}
