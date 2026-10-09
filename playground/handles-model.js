// handles-model.js -- a funscript as a composite cubic Bezier in the time-position
// plane (kin-d0o): the script's knots on the author's clock, one angle (velocity)
// per knot, two handle lengths (time influence, fractions of the adjacent spans),
// G0/G1/G2 per knot, and the ceilings as bounds on the lengths. Design playground
// only: not the kernel, not wired to the wasm.
// Constraints:
// - u is the curve parameter, never time: velocity is (dp/du) / (dt/du).
// - Angles are mm/s, lengths are fractions of the span in (LMIN, LMAX); a third
//   is the polynomial cubic, so an untouched G1 knot renders exact PCHIP.
// - Loads as a plain script (window.HandlesModel) and as CommonJS (the check).
// - railStop (with trimLast) renders as the kernel does (include/kinetic2/handles.hpp,
//   Cfg::railStop), behind its constants and in its order; off, the render is the
//   playground's. Left out, as engine state the model does not have: the slack
//   passes, the rail share and the last knot's brake cap (solver.hpp renderRun),
//   the origin's live acceleration and the start correction (engine_piece.hpp);
//   the model's first knot is at rest.
(function (root, factory) {
  const m = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = m;
  root.HandlesModel = m;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const LMIN = 0.05, LMAX = 0.95, THIRD = 1 / 3, TOL = 1e-3, NS = 160;  // one sampling for the judge and the drawing
  const TICK = 1e-3, HOLD_SPAN = 0.1;  // the kernel's kTick and kHoldSpan, s (railStop only)
  // smoothness: 0 is pchip, crests flat and G1 (PCHIP as drawn, the corner ramp takes
  // the acceleration step), through points G2 by their angle; 1 is smooth, Makima's
  // angles at through points, crests and hold edges flat and G2 by their lengths
  // (they are never passed; kin-4o6r); between, the lerp of the two
  // solves' free angles and lengths (the kernel's Config::smoothness, Valence RFC-108).
  // lfloor: the feel floor on a nudged handle length (a shorter handle is a harder
  // ramp); trim: how far a knot no length can reach may move toward the previous
  // knot's actual position, mm (amplitude gives, time never). trimLast: the last knot
  // trims too; railStop: the kernel's rail rules (see the constraints above).
  const DEF = { vmax: 1000, amax: 50000, jmax: 5e6, lfloor: 0.15, trim: 100, holdEps: 0.5, lo: 0, hi: 100, smoothness: 0, sweeps: 4, trimLast: false, railStop: false };

  // ---- one piece ----------------------------------------------------------
  // D mm over T s, end velocities s0/s1 (mm/s), handle lengths i0/i1. Sampled
  // in u; t and p absolute; v, a and j analytic (j = (da/du) / (dt/du)).
  function sample(t0, p0, D, T, s0, s1, i0, i1, n) {
    const P = [[0, 0], [i0 * T, s0 * i0 * T], [T - i1 * T, D - s1 * i1 * T], [T, D]];
    const t = new Float64Array(n + 1), p = new Float64Array(n + 1), v = new Float64Array(n + 1), a = new Float64Array(n + 1), j = new Float64Array(n + 1);
    const tp3 = 6 * (P[3][0] - 3 * P[2][0] + 3 * P[1][0] - P[0][0]), pp3 = 6 * (P[3][1] - 3 * P[2][1] + 3 * P[1][1] - P[0][1]);
    let peakV = 0, peakA = 0, peakJ = 0;
    for (let k = 0; k <= n; k++) {
      const u = k / n, w = 1 - u;
      const b0 = w * w * w, b1 = 3 * u * w * w, b2 = 3 * u * u * w, b3 = u * u * u;
      const c0 = 3 * w * w, c1 = 6 * u * w, c2 = 3 * u * u;
      const tp = c0 * (P[1][0] - P[0][0]) + c1 * (P[2][0] - P[1][0]) + c2 * (P[3][0] - P[2][0]);
      const pp = c0 * (P[1][1] - P[0][1]) + c1 * (P[2][1] - P[1][1]) + c2 * (P[3][1] - P[2][1]);
      const tpp = 6 * (w * (P[2][0] - 2 * P[1][0] + P[0][0]) + u * (P[3][0] - 2 * P[2][0] + P[1][0]));
      const ppp = 6 * (w * (P[2][1] - 2 * P[1][1] + P[0][1]) + u * (P[3][1] - 2 * P[2][1] + P[1][1]));
      t[k] = t0 + b0 * P[0][0] + b1 * P[1][0] + b2 * P[2][0] + b3 * P[3][0];
      p[k] = p0 + b0 * P[0][1] + b1 * P[1][1] + b2 * P[2][1] + b3 * P[3][1];
      const nA = ppp * tp - pp * tpp;
      v[k] = pp / tp; a[k] = nA / (tp * tp * tp);
      j[k] = ((pp3 * tp - pp * tp3) * tp - 3 * nA * tpp) / (tp * tp * tp * tp * tp);
      if (Math.abs(v[k]) > peakV) peakV = Math.abs(v[k]);
      if (Math.abs(a[k]) > peakA) peakA = Math.abs(a[k]);
      if (Math.abs(j[k]) > peakJ) peakJ = Math.abs(j[k]);
    }
    let pMin = Infinity, pMax = -Infinity;
    for (let k = 0; k <= n; k++) { if (p[k] < pMin) pMin = p[k]; if (p[k] > pMax) pMax = p[k]; }
    return { t, p, v, a, j, peakV, peakA, peakJ, pMin, pMax, aStart: a[0], aEnd: a[n], vStart: v[0], vEnd: v[n],
      handles: [[t0 + P[1][0], p0 + P[1][1]], [t0 + P[2][0], p0 + P[2][1]]] };
  }
  // closed-form end accelerations of a piece (the same algebra as sample)
  function endAccel(D, T, s0, s1, i0, i1) {
    const aStart = 2 / (3 * i0 * i0 * T * T) * (D - s0 * T * (1 - i1) - s1 * i1 * T);
    const aEnd = 2 / (3 * i1 * i1 * T * T) * (-D + s1 * T * (1 - i0) + s0 * i0 * T);
    return { aStart, aEnd };
  }
  const clampL = (l) => Math.min(LMAX, Math.max(LMIN, l));

  // ---- script -> knots ------------------------------------------------------
  // knot: { t s, p mm (authored), pm mm | null (moved by hand), type 'auto'|'G0'|'G1'|'G2',
  //         man: { v?, vIn?, vOut?, lIn?, lOut? } hand-set values }
  function fromFunscript(doc, cfg) {
    const c = { ...DEF, ...cfg };
    const acts = (doc.actions || []).slice().sort((x, y) => x.at - y.at);
    const knots = [];
    for (const a of acts) {
      const t = a.at / 1000, p = c.lo + (c.hi - c.lo) * Math.min(100, Math.max(0, a.pos)) / 100;
      if (knots.length && t - knots[knots.length - 1].t < 1e-6) { knots[knots.length - 1].p = p; continue; }
      knots.push(newKnot(t, p));
    }
    return knots;
  }
  function newKnot(t, p) { return { t, p, pm: null, type: 'auto', man: {} }; }
  const pos = (k) => (k.pm == null ? k.p : k.pm);

  // ---- classes --------------------------------------------------------------
  // end: first or last knot. rest: a zero chord on a side (a hold edge).
  // crest: the chords change sign. through: the same sign both sides.
  // A chord d from knot `from` within holdEps is a hold; under railStop also slower
  // than holdEps per HOLD_SPAN, and never the chord leaving a first knot whose
  // hand-set angle moves (a re-plan's origin in flight).
  function holdChord(knots, from, d, c) {
    if (!c.railStop) return Math.abs(d) <= c.holdEps;
    const v0 = knots[0].man.v;
    return Math.abs(d) <= c.holdEps && Math.abs(d) * HOLD_SPAN <= c.holdEps * (knots[from + 1].t - knots[from].t)
      && !(from === 0 && v0 != null && v0 !== 0);
  }
  function classify(knots, c, withDp) {
    const pe = (k) => pos(k) + (withDp ? k.dp || 0 : 0);
    for (let i = 0; i < knots.length; i++) {
      const k = knots[i], prev = knots[i - 1], next = knots[i + 1];
      k.dIn = prev ? (pe(k) - pe(prev)) / (k.t - prev.t) : 0;
      k.dOut = next ? (pe(next) - pe(k)) / (next.t - k.t) : 0;
      const zIn = !prev || holdChord(knots, i - 1, pe(k) - pe(prev), c), zOut = !next || holdChord(knots, i, pe(next) - pe(k), c);
      k.cls = (!prev || !next) ? 'end' : (zIn || zOut) ? 'rest' : (k.dIn * k.dOut < 0) ? 'crest' : 'through';
    }
  }
  // PCHIP's angle (Fritsch-Butland weighted harmonic mean), zero at a sign change
  function pchipAngle(k, prev, next) {
    if (k.dIn * k.dOut <= 0) return 0;
    const h1 = k.t - prev.t, h2 = next.t - k.t, w1 = 2 * h2 + h1, w2 = h2 + 2 * h1;
    return (w1 + w2) / (w1 / k.dIn + w2 / k.dOut);
  }
  // Makima's angle (Akima with the modified weights), chords padded at the ends
  function makimaAngle(knots, i) {
    const d = (j) => { if (j < 0) return 2 * d(0) - d(1); if (j >= knots.length - 1) return 2 * d(knots.length - 2) - d(knots.length - 3); return knots[j].dOut; };
    const dm2 = d(i - 2), dm1 = d(i - 1), d0 = d(i), d1 = d(i + 1);
    const w1 = Math.abs(d1 - d0) + Math.abs(d1 + d0) / 2, w2 = Math.abs(dm1 - dm2) + Math.abs(dm1 + dm2) / 2;
    return w1 + w2 === 0 ? (dm1 + d0) / 2 : (w1 * dm1 + w2 * d0) / (w1 + w2);
  }
  // the monotone band for a free angle at a through point
  function band(k) { const m = 3 * Math.min(Math.abs(k.dIn), Math.abs(k.dOut)); return k.dIn < 0 ? [-m, 0] : [0, m]; }
  // railStop: an angle held to the knot's cap (vcap, set by nudge)
  const capV = (k, v) => (Math.abs(v) <= k.vcap ? v : v > 0 ? k.vcap : -k.vcap);
  // railStop: the fastest speed whose legal brake (u^2 / 2A + u A / 2J) stops within
  // the nearer wall's gap from p, the sign of v kept
  function railBound(v, p, c) {
    const gap = Math.min(c.hi - p, p - c.lo);
    if (v === 0) return v;
    if (gap <= 0) return 0;
    const b = c.amax * c.amax / (2 * c.jmax), u = -b + Math.sqrt(b * b + 2 * gap * c.amax);
    return Math.abs(v) <= u ? v : v > 0 ? u : -u;
  }

  // ---- the solve: angles and lengths before the ceilings --------------------
  function solve(knots, cfg, withDp) {
    const c = { ...DEF, ...cfg }, s = c.smoothness;
    if (!(s > 0) || s >= 1) { solveStyle(knots, c, withDp, s >= 1); return; }
    solveStyle(knots, c, withDp, false);
    const p = knots.map((k) => [k.vIn, k.vOut, k.lIn, k.lOut]);
    solveStyle(knots, c, withDp, true);
    knots.forEach((k, i) => {
      if (k.man.v == null) { k.vIn = (1 - s) * p[i][0] + s * k.vIn; k.vOut = (1 - s) * p[i][1] + s * k.vOut; }
      k.lIn = (1 - s) * p[i][2] + s * k.lIn; k.lOut = (1 - s) * p[i][3] + s * k.lOut;
    });
  }
  function solveStyle(knots, c, withDp, smooth) {
    classify(knots, c, withDp);
    const pe = (k) => pos(k) + (withDp ? k.dp || 0 : 0);
    for (let i = 0; i < knots.length; i++) {
      const k = knots[i], prev = knots[i - 1], next = knots[i + 1], m = k.man;
      k.eff = k.cls === 'end' ? 'G1' : k.type !== 'auto' ? k.type : (k.cls === 'through' || smooth) ? 'G2' : 'G1';
      const base = k.cls !== 'through' ? 0 : smooth && knots.length > 3 ? makimaAngle(knots, i) : pchipAngle(k, prev, next);
      if (k.eff === 'G0') { k.vIn = m.vIn ?? k.dIn; k.vOut = m.vOut ?? k.dOut; }
      else { const v = m.v ?? base; k.vIn = k.vOut = v; }
      k.lIn = m.lIn ?? THIRD; k.lOut = m.lOut ?? THIRD;
      k.note = [];
      const cv = (v) => Math.max(-c.vmax, Math.min(c.vmax, v));
      if (cv(k.vIn) !== k.vIn || cv(k.vOut) !== k.vOut) { k.vIn = cv(k.vIn); k.vOut = cv(k.vOut); k.note.push('angle held to the speed ceiling'); }
      if (c.railStop) {
        // the first knot's hand-set angle is the live velocity, kept exactly
        if (i === 0 && m.v != null) k.vIn = k.vOut = m.v;
        else { k.vIn = capV(k, k.vIn); k.vOut = capV(k, k.vOut); }
        if (i > 0 && m.v != null) { k.vIn = railBound(k.vIn, pe(k), c); k.vOut = railBound(k.vOut, pe(k), c); }
      }
    }
    // G2 sweeps: a through point solves its angle, a crest or hold edge its lengths
    for (let s = 0; s < c.sweeps; s++) {
      for (let i = 1; i < knots.length - 1; i++) {
        const k = knots[i], L = knots[i - 1], R = knots[i + 1];
        if (k.eff !== 'G2') continue;
        const TL = k.t - L.t, DL = pe(k) - pe(L), TR = R.t - k.t, DR = pe(R) - pe(k);
        if (k.cls === 'through' && k.man.v == null) {
          // aEnd(left) = AL + BL*s, aStart(right) = AR + BR*s, both linear in the angle s
          const cL = 2 / (3 * k.lIn * k.lIn * TL * TL), cR = 2 / (3 * k.lOut * k.lOut * TR * TR);
          const AL = cL * (-DL + L.vOut * L.lOut * TL), BL = cL * TL * (1 - L.lOut);
          const AR = cR * (DR - R.vIn * R.lIn * TR), BR = -cR * TR * (1 - R.lIn);
          let v = (AR - AL) / (BL - BR);
          const [lo, hi] = band(k); const clamped = v < lo || v > hi; v = Math.max(-c.vmax, Math.min(c.vmax, Math.min(hi, Math.max(lo, v))));
          if (c.railStop) v = capV(k, v);
          k.vIn = k.vOut = v; if (clamped && s === c.sweeps - 1) k.note.push('angle held to the monotone band');
        } else if (k.cls !== 'through' && k.man.lIn == null && k.man.lOut == null) {
          // aEnd(left) = XL / lIn^2, aStart(right) = XR / lOut^2: match by the ratio, keep the product at a ninth
          const XL = 2 / (3 * TL * TL) * (-DL + k.vIn * TL * (1 - L.lOut) + L.vOut * L.lOut * TL);
          const XR = 2 / (3 * TR * TR) * (DR - k.vOut * TR * (1 - R.lIn) - R.vIn * R.lIn * TR);
          if (XL * XR > 0) {
            const r = Math.sqrt(XR / XL);
            k.lIn = clampL(THIRD / Math.sqrt(r)); k.lOut = clampL(THIRD * Math.sqrt(r));
          } else if (s === c.sweeps - 1) k.note.push('no lengths match: the sides accelerate opposite ways');
        }
      }
    }
  }

  // ---- the ceilings: bound the lengths, then trim the amplitude -------------
  // Per piece: scale both handles by one factor (nearest 1 that is legal, never
  // under the feel floor); if none is, move the later knot toward the previous
  // knot's actual position by the least that is legal (bisection; the stroke
  // keeps its time and loses height). A knot that was reachable never moves.
  // Under railStop the window is a wall: leaving it past half the tolerance ranks
  // below any ceiling ratio short of a thousand, so the least-over fit stays inside
  // (the other half is the engine's: a built piece reads a little past its render).
  const over = (r, c) => {
    const out = Math.max(0, c.lo - r.pMin, r.pMax - c.hi), wall = c.railStop && out > 0.5 * TOL * (c.hi - c.lo) ? 1e3 : 1;
    return Math.max(r.peakV / c.vmax, r.peakA / c.amax, r.peakJ / c.jmax, wall + out / (c.hi - c.lo));
  };
  const KS = []; for (let k = 0.2; k <= 2.0001; k += 0.05) KS.push(k);
  KS.sort((x, y) => Math.abs(x - 1) - Math.abs(y - 1));

  // railStop: the kernel's room for the corner ramps at a piece's ends (jmax, one
  // TICK clear of each end): from aIn at its start (on the start when fromFlat, else
  // centered, so also within half of tIn), to rest at its end when intoFlat, and
  // near aEnd (within aEndTol) when the next piece's start ramp asked for it.
  // e: the piece's endAccel. Over 1: a ramp does not fit.
  function startRoom(e, T, room, jmax) {
    if (Number.isNaN(room.aIn)) return 0;
    const tr = Math.abs(e.aStart - room.aIn) / jmax;
    if (!(tr > 0)) return 0;
    return room.fromFlat ? (tr + TICK) / (0.5 * T) : (0.5 * tr + TICK) / (0.5 * Math.min(T, room.tIn));
  }
  function roomOver(e, T, room, jmax) {
    let o = startRoom(e, T, room, jmax);
    if (room.intoFlat) { const tr = Math.abs(e.aEnd) / jmax; if (tr > 0) o = Math.max(o, (tr + TICK) / (0.5 * T)); }
    if (!Number.isNaN(room.aEnd)) o = Math.max(o, Math.abs(e.aEnd - room.aEnd) / room.aEndTol);
    return o;
  }
  // railStop: a trimmed knot's angle held to the Fritsch-Carlson band of its
  // trimmed chord D over T, zero on a zero chord or against it
  function bandHold(v, D, T) {
    if (v * D <= 0) return 0;
    const b = 3 * Math.abs(D) / T;
    return Math.abs(v) <= b ? v : v > 0 ? b : -b;
  }
  // railStop: the fastest angle at L whose zero-stroke piece to R (R at L's height,
  // at rest) is legal at some length factor (every peak is linear in the angle),
  // toward the wall dir heads for; intoFlat: the corner ramp to rest fits in half the span;
  // the end acceleration stays within R's ask (aTarget, aTol) when it has one
  function zeroStrokeMax(L, R, pL, dir, c, intoFlat) {
    const T = R.t - L.t, floor = Math.max(LMIN, c.lfloor), gap = dir > 0 ? c.hi - pL : pL - c.lo;
    let best = 0;
    for (const k of KS) {
      const i0 = Math.min(LMAX, Math.max(floor, L.lOut * k)), i1 = Math.min(LMAX, Math.max(floor, R.lIn * k));
      const r = sample(0, 0, 0, T, dir, 0, i0, i1, NS);
      let px = 0; for (const p of r.p) px = Math.max(px, dir * p);
      let s = Math.min(c.vmax / r.peakV, c.amax / r.peakA, c.jmax / r.peakJ);
      if (px > 0) s = Math.min(s, Math.max(0, gap) / px);
      if (intoFlat) s = Math.min(s, c.jmax * Math.max(0, 0.5 * T - TICK) / Math.abs(endAccel(0, T, dir, 0, i0, i1).aEnd));
      if (!Number.isNaN(R.aTarget)) {
        const e1 = endAccel(0, T, dir, 0, i0, i1).aEnd;
        s = Math.min(s, Math.max(0, (R.aTol + (e1 > 0 ? R.aTarget : -R.aTarget)) / Math.abs(e1)));
      }
      best = Math.max(best, s);
    }
    return best;
  }
  // railStop, pchip: a band-legal piece (both angles zero or of its chord's sign and at
  // most 3 (1 + TOL) times it) stays monotone at its scaled lengths: 1 + the share of
  // its chord it travels backward (an overshoot past its end or a reversal inside),
  // read from the samples; 0 when not judged (a hold, an angle outside the band)
  function monoOver(r, D, T, s0, s1, c) {
    if (!(Math.abs(D) > c.holdEps)) return 0;
    const m = D / T, al = s0 / m, be = s1 / m, b = 3 * (1 + TOL);
    if (!(al >= 0 && al <= b && be >= 0 && be <= b)) return 0;
    const sg = Math.sign(D);
    let top = -Infinity, back = 0;
    for (const p of r.p) { const x = sg * p; if (x > top) top = x; else back = Math.max(back, top - x); }
    return 1 + back / Math.abs(D);
  }
  // s1: R's angle for this fit (bandHold may lower it); room: railStop's corner room
  function fitPiece(L, R, pL, pR, c, s1 = R.vIn, room = null) {
    const T = R.t - L.t, D = pR - pL, floor = Math.max(LMIN, c.lfloor);
    const cl = (l, hand) => (hand ? clampL(l) : Math.min(LMAX, Math.max(floor, l)));
    let best = null;
    for (const k of KS) {
      const i0 = cl(L.lOut * k, L.man.lOut != null), i1 = cl(R.lIn * k, R.man.lIn != null);
      const r = sample(L.t, pL, D, T, L.vOut, s1, i0, i1, NS);
      let o = over(r, c);
      if (room) o = Math.max(o, roomOver(endAccel(D, T, L.vOut, s1, i0, i1), T, room, c.jmax));
      if (c.railStop && !(c.smoothness > 0)) o = Math.max(o, monoOver(r, D, T, L.vOut, s1, c));
      if (o <= 1 + TOL) return { k, i0, i1, o, legal: true };
      if (!best || o < best.o) best = { k, i0, i1, o, legal: false };
    }
    return best;
  }
  // Returns the infeasible pieces; under railStop, steps.capped is set when a cap
  // or an end-acceleration ask changed (render then earns a round).
  function nudge(knots, cfg) {
    const c = { ...DEF, ...cfg }, rs = c.railStop, n = knots.length;
    for (const k of knots) { k.effIn = k.lIn; k.effOut = k.lOut; k.dp = 0; k.infeasible = false; }
    const steps = [];
    for (let i = 0; i < n - 1; i++) {
      const L = knots[i], R = knots[i + 1];
      const pL = pos(L) + L.dp;
      // railStop: the corner ramps at the piece's ends must fit; the origin's
      // start is the engine's lead ramp
      let room = null, intoFlat = false;
      if (rs) {
        room = { aIn: NaN, tIn: Infinity, fromFlat: false, intoFlat: false, aEnd: R.aTarget, aEndTol: R.aTol };
        if (i > 0) {
          const K = knots[i - 1];
          room.aIn = L.aIn; room.tIn = L.t - K.t;
          room.fromFlat = L.vIn === 0 && K.vOut === 0 && Math.abs(pL - (pos(K) + K.dp)) <= c.holdEps;
        } else if (L.vOut === 0 && L.aIn === 0) { room.aIn = 0; room.fromFlat = true; }
        intoFlat = i + 2 >= n || holdChord(knots, i + 1, pos(knots[i + 2]) - pos(R), c);
      }
      // railStop: a trimmed knot's angle keeps to its trimmed chord, so the whole trim is a zero stroke at rest
      const fitAt = (pR, flat) => {
        const s1 = rs && (pR !== pos(R) || flat) ? bandHold(R.vIn, pR - pL, R.t - L.t) : R.vIn;
        const f = fitPiece(L, R, pL, pR, c, s1, room && { ...room, intoFlat: intoFlat && s1 === 0 });
        f.s1 = s1;
        return f;
      };
      // a hold after a trimmed knot moves with it: the hold stays flat at the trimmed height
      const hold = holdChord(knots, i, pos(R) - pos(L), c);
      const canTrim = c.trim > 0 && (i + 1 < n - 1 || c.trimLast);
      let f, dp;
      const search = () => {
        f = fitAt(pos(R) + (hold ? L.dp : 0)); dp = hold ? L.dp : 0;
        if (!f.legal && !hold && canTrim) {
          const dir = Math.sign(pL - pos(R)); let lo = 0, hi = Math.min(c.trim, Math.abs(pos(R) - pL));
          let fh = fitAt(pos(R) + dir * hi);
          if (fh.legal) {
            for (let s = 0; s < 16; s++) { const m = (lo + hi) / 2, fm = fitAt(pos(R) + dir * m); if (fm.legal) { hi = m; fh = fm; } else lo = m; }
            f = fh; dp = dir * hi;
          } else { // nothing reachable: take the least-over position on the way
            for (const q of [0.25, 0.5, 0.75, 1]) { const fq = fitAt(pos(R) + dir * q * hi); if (fq.o < f.o) { f = fq; dp = dir * q * hi; } }
          }
        }
        // railStop: an illegal hold lies flat at its predecessor instead when that is less over,
        // at rest on a zero chord too (an authored angle kept there is a zero stroke no cap makes legal)
        if (rs && !f.legal && hold && canTrim) { const ff = fitAt(pL, true); if (ff.o < f.o) { f = ff; dp = pL - pos(R); } }
      };
      search();
      // railStop: an end-acceleration ask no fit honors legally is dropped (it is the
      // next piece's jerk, never this piece's speed or acceleration)
      if (rs && !f.legal && !Number.isNaN(room.aEnd)) {
        const fa = f, da = dp;
        room.aEnd = NaN; room.aEndTol = Infinity;
        search();
        if (!f.legal) { f = fa; dp = da; }
      }
      if (rs) {
        // still over: L's angle is faster than its span can stop; its cap holds it
        // from the next solve on (under a thousandth of vmax it is zero)
        if (!f.legal && i > 0 && L.vOut !== 0) {
          let cap = 0.98 * zeroStrokeMax(L, R, pL, L.vOut > 0 ? 1 : -1, c, intoFlat);
          if (cap < 1e-3 * c.vmax) cap = 0;
          if (cap < Math.abs(L.vOut) && cap < L.vcap) { L.vcap = cap; steps.capped = true; }
        }
        if (f.s1 !== R.vIn) R.vIn = R.vOut = f.s1;
        const T = R.t - L.t, e = endAccel(pos(R) + dp - pL, T, L.vOut, R.vIn, f.i0, f.i1);
        R.aIn = e.aEnd;
        // over because the start ramp at L does not fit: the piece into L is asked
        // to end near this piece's start, held to amax (from the next round)
        if (!f.legal && startRoom(e, T, room, c.jmax) > 1 + TOL) {
          const span = room.fromFlat ? 0.5 * T - TICK : Math.min(T, room.tIn) - 2 * TICK, tol = 0.95 * c.jmax * span;
          const target = Math.max(-c.amax, Math.min(c.amax, e.aStart));
          if (tol > 0 && !(L.aTarget === target && L.aTol === tol)) { L.aTarget = target; L.aTol = tol; steps.capped = true; }
        }
      }
      L.effOut = f.i0; R.effIn = f.i1; R.dp = dp; R.infeasible = !f.legal;
      if (!f.legal) steps.push({ i: i + 1, over: f.o });
    }
    return steps;
  }

  // ---- render: solve, nudge, sample, steps ----------------------------------
  function render(knots, cfg) {
    const c = { ...DEF, ...cfg };
    if (knots.length < 2) return { pieces: [], knots, report: { infeasible: 0, over: 0, steps: [] } };
    // railStop's caps and end-acceleration asks start fresh each render, as a kernel window's do
    if (c.railStop) for (const k of knots) { k.vcap = Infinity; k.aIn = 0; k.aTarget = NaN; k.aTol = Infinity; }
    solve(knots, c);
    let bad = nudge(knots, c);
    // angles follow the trimmed chords; a last round that caps an angle earns one more (railStop, at most four)
    for (let round = 0, extra = 0; round < 2 + extra; round++) {
      solve(knots, c, true); bad = nudge(knots, c);
      if (bad.capped && round + 1 === 2 + extra && extra < 4) extra++;
    }
    const pieces = [];
    for (let i = 0; i < knots.length - 1; i++) {
      const L = knots[i], R = knots[i + 1];
      const pL = pos(L) + L.dp, pR = pos(R) + R.dp;
      const r = sample(L.t, pL, pR - pL, R.t - L.t, L.vOut, R.vIn, L.effOut, R.effIn, NS);
      r.over = over(r, c); r.i = i; pieces.push(r);
    }
    // what the motor meets at each knot: a velocity step (G0) and an acceleration step
    const steps = [];
    for (let i = 1; i < knots.length - 1; i++) {
      const a = pieces[i - 1], b = pieces[i];
      const dv = b.vStart - a.vEnd, da = b.aStart - a.aEnd;
      steps.push({ i, dv, da, rampMs: Math.abs(da) / c.jmax * 1000, cornerMs: Math.abs(dv) / c.amax * 1000 });
    }
    const counts = { end: 0, rest: 0, crest: 0, through: 0 };
    for (const k of knots) counts[k.cls]++;
    return { pieces, knots, steps, report: { infeasible: bad.length, overPieces: pieces.filter((p) => p.over > 1 + TOL).length, counts, bad } };
  }

  // a small built-in script: holds, a fast stroke, a mid-travel point, a sawtooth
  const SAMPLE = { actions: [
    { at: 0, pos: 10 }, { at: 500, pos: 10 }, { at: 700, pos: 90 }, { at: 900, pos: 10 }, { at: 1100, pos: 90 }, { at: 1300, pos: 10 },
    { at: 1800, pos: 10 }, { at: 1950, pos: 60 }, { at: 2200, pos: 100 }, { at: 2600, pos: 0 }, { at: 3100, pos: 0 },
    { at: 3300, pos: 70 }, { at: 3400, pos: 50 }, { at: 3600, pos: 100 }, { at: 3800, pos: 20 }, { at: 4300, pos: 20 },
    { at: 4400, pos: 80 }, { at: 4700, pos: 30 }, { at: 4800, pos: 90 }, { at: 5100, pos: 40 }, { at: 5200, pos: 95 }, { at: 5700, pos: 10 }, { at: 6200, pos: 10 },
  ] };

  return { DEF, LMIN, LMAX, THIRD, SAMPLE, sample, endAccel, fromFunscript, newKnot, pos, classify, pchipAngle, makimaAngle, band, solve, nudge, render };
});
