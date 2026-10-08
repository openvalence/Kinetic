// bench_kinetic2.cpp -- the solver's cost per submit, counted (kin-ys0). Built
// with KINETIC2_STATS; the counts are the proxy for the P4, whose single
// precision FPU makes wall time on this host meaningless as an absolute.
// A plan is a submit and the first sample after it (Nucleus times that
// sample as plan_us). Every scenario samples on the 1 ms motion tick.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "kinetic2/engine.hpp"
#include "kinetic2/sources.hpp"

using namespace kinetic2;

namespace {

constexpr uint64_t kMs = 1000;
constexpr float kRailMm = 268.0f;
float g_smoothness = 0.0f;   // Config::smoothness of every scenario (argv[2])

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    float uni(float a, float b) { return a + (b - a) * float(next() & 0xFFFFFF) / float(0x1000000); }
};

struct Tally {
    const char* name;
    int plans = 0, refused = 0, over = 0, trimmed = 0;
    float share_sum = 0.0f;   // trim shares, summed
    int refused_full = 0, refused_past = 0;
    float lag_max_ms = -1.0f, lag_end_ms = 0.0f;   // streams: the newest knot's solved time behind its authored time
    uint64_t judges = 0, knots = 0;   // renders (handles::render calls) and knots rendered
    uint64_t max_judges = 0, max_knots = 0;
    double wall_us = 0.0, max_wall_us = 0.0;
    uint64_t idle_max_judges = 0;   // a tick with no submit that still rendered
    float worst_v = 0.0f, worst_a = 0.0f, worst_j = 0.0f;
    float dev_max_mm = -1.0f, dev_mean_mm = 0.0f, within1 = 0.0f;   // funscript: rendered vs the PCHIP curve
};

// Runs one plan (a submit callback, then the first sample) and books its cost.
template <typename Fn>
State plan(Engine<1, 64>& e, Tally& t, uint64_t now, Fn&& submit) {
    const stats::Counters c0 = stats::g;
    const auto w0 = std::chrono::steady_clock::now();
    submit();
    if (e.pending()) (void)e.solved(0, e.pending() - 1);
    const State s = e.stateAt(0, now);
    const auto w1 = std::chrono::steady_clock::now();
    const stats::Counters& c1 = stats::g;
    const uint64_t dj = c1.judges - c0.judges, dk = c1.knots - c0.knots;
    const double us = std::chrono::duration<double, std::micro>(w1 - w0).count();
    ++t.plans;
    t.judges += dj; t.knots += dk; t.wall_us += us;
    if (dj > t.max_judges) t.max_judges = dj;
    if (dk > t.max_knots) t.max_knots = dk;
    if (us > t.max_wall_us) t.max_wall_us = us;
    return s;
}

State idle(Engine<1, 64>& e, Tally& t, uint64_t now) {
    const uint64_t j0 = stats::g.judges;
    const State s = e.stateAt(0, now);
    if (stats::g.judges - j0 > t.idle_max_judges) t.idle_max_judges = stats::g.judges - j0;
    return s;
}

void drainInto(Engine<1, 64>& e, Tally& t) {
    Anomaly a;
    while (e.popAnomaly(a)) {
        if (a.kind == uint8_t(AnomalyKind::KnotRefused)) { ++t.refused; if (a.detail == kDetailTimelineFull) ++t.refused_full; if (a.detail == kDetailPast) ++t.refused_past; }
        if (a.kind == uint8_t(AnomalyKind::PieceOverCeiling)) ++t.over;
        if (a.kind == uint8_t(AnomalyKind::KnotTrimmed)) { ++t.trimmed; t.share_sum += a.detail; }
    }
}

struct Peaks {
    State prev{}; float pv = 0.0f; bool have = false; bool have_a = false;
    void add(const State& s, const Limits& L, Tally& t) {
        t.worst_v = std::fmax(t.worst_v, std::fabs(s.v) / L.vmax);
        t.worst_a = std::fmax(t.worst_a, std::fabs(s.a) / L.amax);
        if (have) t.worst_j = std::fmax(t.worst_j, std::fabs(s.a - prev.a) / 1e-3f / L.jmax);
        prev = s; have = true;
    }
};

/// (A) Funscript as Phosphor sends it: segments 100 to 250 ms long, each
// authored 250 ms before its start, every one with an end velocity, the PCHIP
// slope at its knot (zero at a turnaround). Strokes of 0.05 to 0.6 run past
// the ceilings (vmax 2, amax 100, jmax 10000); `gentle` keeps them to 0.03 to
// 0.2 over 150 to 300 ms, which the ceilings allow. The match is the rendered
// position against the PCHIP curve itself, in mm on a 268 mm rail.
Tally funscript(uint32_t budget, bool gentle, const char* name) {
    Tally t{name};
    Config cfg; cfg.limits = {2.0f, 100.0f, 10000.0f};
    cfg.solve_budget = budget;
    cfg.smoothness = g_smoothness;
    Engine<1, 64> e(cfg, 0.5f);
    Rng r(7);
    const int n = 60;
    std::vector<float> p(n + 2), d(n + 2), v(n + 2, 0.0f);
    std::vector<uint64_t> at(n + 2);
    p[0] = 0.5f;
    float dir = 1.0f;
    for (int i = 1; i < n + 2; ++i) {
        if (r.uni(0.0f, 1.0f) < 0.7f) dir = -dir;
        float q = p[i - 1] + dir * (gentle ? r.uni(0.03f, 0.2f) : r.uni(0.05f, 0.6f));
        if (q > 0.97f) q = 0.97f - r.uni(0.0f, 0.1f);
        if (q < 0.03f) q = 0.03f + r.uni(0.0f, 0.1f);
        p[i] = q;
        d[i] = gentle ? r.uni(150.0f, 300.0f) : r.uni(100.0f, 250.0f);
    }
    at[0] = 300 * kMs;
    for (int i = 1; i < n + 2; ++i) at[i] = at[i - 1] + uint64_t(d[i] * 1000.0f);
    // PCHIP (Fritsch-Carlson weighted harmonic mean), zero where the secants
    // disagree in sign.
    for (int i = 1; i <= n; ++i) {
        const float h0 = d[i] * 1e-3f, h1 = d[i + 1] * 1e-3f;
        const float s0 = (p[i] - p[i - 1]) / h0, s1 = (p[i + 1] - p[i]) / h1;
        if (s0 == 0.0f || s1 == 0.0f || (s0 > 0.0f) != (s1 > 0.0f)) continue;
        const float w1 = 2.0f * h1 + h0, w2 = h1 + 2.0f * h0;
        v[i] = (w1 + w2) / (w1 / s0 + w2 / s1);
    }
    auto pchipAt = [&](uint64_t tt) -> float {
        if (tt <= at[0]) return p[0];
        for (int i = 1; i <= n; ++i) {
            if (tt > at[i]) continue;
            const float h = float(at[i] - at[i - 1]) * 1e-6f, s = float(tt - at[i - 1]) * 1e-6f / h;
            const float h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s, h01 = -2 * s * s * s + 3 * s * s, h11 = s * s * s - s * s;
            return h00 * p[i - 1] + h10 * h * v[i - 1] + h01 * p[i] + h11 * h * v[i];
        }
        return p[n];
    };
    Peaks pk;
    int next = 1;
    const uint64_t end = at[n] + 1000 * kMs;
    double dev_sum = 0.0; float dev_max = 0.0f; int dev_n = 0, within1 = 0;
    for (uint64_t now = 0; now < end; now += kMs) {
        State s;
        if (next <= n && at[next - 1] <= now + 250 * kMs) {
            s = plan(e, t, now, [&] {
                while (next <= n && at[next - 1] <= now + 250 * kMs) {
                    // The arbiter's hold: a start past the newest knot is a rest until it.
                    if (next == 1) { Knot h; h.t_us = at[0]; h.p = p[0]; h.has_v = true; (void)e.submit(h, now); }
                    (void)e.submit(knotFromSegment(p[next], uint32_t(at[next] - at[next - 1]), true, v[next], at[next - 1]), now);
                    ++next;
                }
            });
        } else {
            s = idle(e, t, now);
        }
        pk.add(s, cfg.limits, t);
        drainInto(e, t);
        if (now >= at[0] && now <= at[n]) {
            const float dev = std::fabs(s.p - pchipAt(now)) * kRailMm;
            dev_sum += dev; dev_max = std::fmax(dev_max, dev); ++dev_n; if (dev <= 1.0f) ++within1;
        }
    }
    drainInto(e, t);
    t.dev_max_mm = dev_max; t.dev_mean_mm = float(dev_sum / dev_n); t.within1 = float(within1) / float(dev_n);
    return t;
}

// (B) Jog scrub: a slider swept across an 84 mm window at 20 Hz, each move a
// knot at rest at the park time from the newest knot (has_v, v = 0), the
// jog ceilings 200 mm/s and 200 mm/s^2 on a 268 mm rail. The queue grows
// faster than it drains, as on the bench. `live` is the jog as Nucleus now
// sends it: each move supersedes the queue at the reaction horizon
// (truncateAfter) and is a HARD knot (a sample at rest) due as soon as possible,
// so the solver's Profile::point sets its time.
Tally jogScrub(uint32_t budget, bool live = false) {
    Tally t{live ? "B live jog" : "B jog scrub"};
    Config cfg; cfg.limits = {200.0f / kRailMm, 200.0f / kRailMm, 5.0e6f / kRailMm};
    cfg.solve_budget = budget;
    cfg.smoothness = g_smoothness;
    Engine<1, 64> e(cfg, 0.5f);
    auto parkUs = [&](float dd) {
        const Limits& L = cfg.limits;
        const float tt = std::fmax(std::fmax(1.875f * dd / L.vmax, std::sqrt(5.7735f * dd / L.amax)), std::cbrt(60.0f * dd / L.jmax));
        return uint32_t(tt * 1e6f) + 1000u;
    };
    uint64_t newest_us = 0; float newest_p = 0.5f;
    Peaks pk;
    const uint64_t end = 12000 * kMs;
    for (uint64_t now = 0; now < end; now += kMs) {
        State s;
        if (now % (50 * kMs) == 0 && now < 10000 * kMs) {
            const float target = 0.5f + (42.0f / kRailMm) * std::sin(2.0f * 3.14159265f * 0.4f * float(now) * 1e-6f);
            s = plan(e, t, now, [&] {
                if (e.pending() == 0 && !e.isBusy(now)) { const float here = e.stateAt(0, now).p; e.resetAt(here, now); newest_us = now; newest_p = here; }
                const uint64_t from = newest_us > now ? newest_us : now;
                Knot k = knotFromSample(target, from, parkUs(std::fabs(target - newest_p)));
                k.has_v = true;
                k.sample = false;   // a parked target: an authored rest, not a live jog
                if (live) {
                    (void)e.truncateAfter(now, now);
                    const Knot h = e.newest();
                    k = knotFromSample(target, h.t_us > now ? h.t_us : now, 1000);
                    k.has_v = true;
                }
                if (e.submit(k, now)) { newest_us = k.t_us; newest_p = k.p; }
            });
        } else {
            s = idle(e, t, now);
        }
        pk.add(s, cfg.limits, t);
        drainInto(e, t);
    }
    drainInto(e, t);
    return t;
}

// (C) A 60 Hz sample stream at a 57 ms latency: a 1.2 Hz sweep of `amp`;
// 0.25 stays inside vmax 2, 0.3 runs past it at its centers (amax 100, jmax
// 10000).
Tally stream60(uint32_t budget, float amp, const char* name) {
    Tally t{name};
    Config cfg; cfg.limits = {2.0f, 100.0f, 10000.0f};
    cfg.solve_budget = budget;
    cfg.smoothness = g_smoothness;
    Engine<1, 64> e(cfg, 0.5f);
    Peaks pk;
    const uint64_t end = 11000 * kMs;
    uint64_t next = 0;
    int i = 0;
    for (uint64_t now = 0; now < end; now += kMs) {
        State s;
        if (now >= next && now < 10000 * kMs) {
            const float p = 0.5f + amp * std::sin(2.0f * 3.14159265f * 1.2f * float(now) * 1e-6f);
            const Knot k = knotFromSample(p, now, 57000);
            s = plan(e, t, now, [&] { (void)e.submit(k, now); });
            if (const size_t np = e.pending()) {
                const uint64_t tail = e.solved(0, np - 1).t_us;
                t.lag_end_ms = tail > k.t_us ? float(tail - k.t_us) * 1e-3f : 0.0f;
                t.lag_max_ms = std::fmax(t.lag_max_ms, t.lag_end_ms);
            }
            next += (i++ % 3 == 2) ? 16 * kMs : 17 * kMs;
        } else {
            s = idle(e, t, now);
        }
        pk.add(s, cfg.limits, t);
        drainInto(e, t);
    }
    drainInto(e, t);
    return t;
}

// (D) The worst window: 64 infeasible segments in one bundle, solved once.
Tally fullWindow(uint32_t budget) {
    Tally t{"D 64-knot bundle"};
    Config cfg; cfg.limits = {2.0f, 100.0f, 10000.0f};
    cfg.solve_budget = budget;
    cfg.smoothness = g_smoothness;
    Engine<1, 64> e(cfg, 0.5f);
    Rng r(11);
    Peaks pk;
    (void)plan(e, t, 0, [&] {
        uint64_t at = 0; float dir = 1.0f;
        for (int i = 0; i < 64; ++i) {
            at += uint64_t(r.uni(60.0f, 200.0f) * 1000.0f);
            dir = -dir;
            (void)e.submit(knotFromSegment(0.5f + dir * r.uni(0.1f, 0.45f), uint32_t(at), true, 0.0f, 0), 0);
        }
    });
    for (uint64_t now = 1; now < 20000 * kMs; now += kMs) { pk.add(e.stateAt(0, now), cfg.limits, t); drainInto(e, t); }
    drainInto(e, t);
    return t;
}

void print(const Tally& t) {
    const double n = t.plans ? double(t.plans) : 1.0;
    std::printf("%-17s plans %4d | renders/plan max %4llu mean %5.1f | knots max %3llu | host us max %8.1f mean %7.1f"
                " | idle renders max %llu | refused %d (full %d, past %d) over %d trimmed %d (mean share %.3f) | peak v %.3f a %.3f j %.3f\n",
                t.name, t.plans, (unsigned long long)t.max_judges, double(t.judges) / n, (unsigned long long)t.max_knots,
                t.max_wall_us, t.wall_us / n, (unsigned long long)t.idle_max_judges, t.refused, t.refused_full, t.refused_past,
                t.over, t.trimmed, t.trimmed ? t.share_sum / float(t.trimmed) : 0.0f, t.worst_v, t.worst_a, t.worst_j);
    if (t.lag_max_ms >= 0.0f) std::printf("%-17s lag: max %.1f ms, at the end %.1f ms\n", "", t.lag_max_ms, t.lag_end_ms);
    if (t.dev_max_mm >= 0.0f) std::printf("%-17s vs PCHIP: max %.2f mm, mean %.3f mm, %.1f%% of ticks within 1 mm\n", "", t.dev_max_mm, t.dev_mean_mm, 100.0f * t.within1);
}

}  // namespace

int main(int argc, char** argv) {
    const uint32_t budget = argc > 1 ? uint32_t(std::atoi(argv[1])) : Config{}.solve_budget;
    g_smoothness = argc > 2 ? float(std::atof(argv[2])) : 0.0f;
    std::printf("solve_budget %u smoothness %g\n", budget, double(g_smoothness));
    print(funscript(budget, false, "A funscript"));
    print(funscript(budget, true, "A gentle"));
    print(jogScrub(budget));
    print(jogScrub(budget, true));
    print(stream60(budget, 0.25f, "C 60 Hz stream"));
    print(stream60(budget, 0.3f, "C+ 60 Hz past v"));
    print(fullWindow(budget));
    return 0;
}
