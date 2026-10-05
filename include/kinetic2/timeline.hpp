// kinetic2/timeline.hpp -- the knot timeline: one ordered ring of knots per axis
// Constraints:
// - Knots arrive in time order per source; the timeline refuses a knot at or
//   before its newest (a sender that goes backwards is a bug on its side, and
//   the refusal is counted, never absorbed).
// - Fixed capacity, no allocation: sized by the caller to the largest schedule
//   horizon it will grant (32 segments per bundle over a 1000 ms horizon at
//   the reference hub). Consumed knots are retired by the engine, never by
//   the timeline on its own, so the past is available to the solver for as
//   long as the engine wants it.
// - Single-threaded. One task owns an Engine and its timelines.
#pragma once

#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace kinetic2 {

template <size_t Capacity>
class Timeline {
public:
    static_assert(Capacity >= 2, "a timeline needs room for a pair of knots");

    size_t size() const { return _n; }
    bool   empty() const { return _n == 0; }
    bool   full() const { return _n == Capacity; }
    static constexpr size_t capacity() { return Capacity; }

    // The i-th knot from the oldest, 0 <= i < size().
    const Knot& at(size_t i) const { return _ring[(_head + i) % Capacity]; }
    Knot&       at(size_t i)       { return _ring[(_head + i) % Capacity]; }
    const Knot& newest() const { return at(_n - 1); }

    // Accepts a knot strictly after the newest. False when full or not after.
    bool push(const Knot& k) {
        if (_n == Capacity) return false;
        if (_n > 0 && k.t_us <= newest().t_us) return false;
        _ring[(_head + _n) % Capacity] = k;
        ++_n;
        return true;
    }

    // Retires the oldest knot.
    void popFront() {
        if (_n == 0) return;
        _head = (_head + 1) % Capacity;
        --_n;
    }

    void clear() { _head = 0; _n = 0; }

    // Removes the i-th knot, closing the gap. The engine uses it for a knot the
    // solver dropped as unreachable.
    void erase(size_t i) {
        if (i >= _n) return;
        for (size_t k = i; k + 1 < _n; ++k) at(k) = at(k + 1);
        --_n;
    }

    // Index of the first knot with t_us > t, or size() when none.
    size_t firstAfter(uint64_t t) const {
        size_t lo = 0, hi = _n;
        while (lo < hi) {
            const size_t mid = (lo + hi) / 2;
            if (at(mid).t_us > t) hi = mid; else lo = mid + 1;
        }
        return lo;
    }

private:
    Knot   _ring[Capacity]{};
    size_t _head = 0;
    size_t _n    = 0;
};

}  // namespace kinetic2
