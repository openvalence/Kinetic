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
(function (root, factory) {
  const m = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = m;
  root.HandlesModel = m;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const LMIN = 0.05, LMAX = 0.95, THIRD = 1 / 3, TOL = 1e-3, NS = 160;  // one sampling for the judge and the drawing
  // style: 'pchip' keeps crests flat and G1 (PCHIP as drawn, the corner ramp takes the
  // acceleration step) and makes through points G2 by their angle; 'smooth' takes
  // Makima's angles and makes every knot G2 (lengths at crests and hold edges).
  // lfloor: the feel floor on a nudged handle length (a shorter handle is a harder
  // ramp); trim: how far a knot no length can reach may move toward the previous
  // knot's actual position, mm (amplitude gives, time never).
  const DEF = { vmax: 1000, amax: 50000, jmax: 5e6, lfloor: 0.15, trim: 100, holdEps: 0.5, lo: 0, hi: 100, style: 'pchip', sweeps: 4 };

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
  function classify(knots, holdEps, withDp) {
    const pe = (k) => pos(k) + (withDp ? k.dp || 0 : 0);
    for (let i = 0; i < knots.length; i++) {
      const k = knots[i], prev = knots[i - 1], next = knots[i + 1];
      k.dIn = prev ? (pe(k) - pe(prev)) / (k.t - prev.t) : 0;
      k.dOut = next ? (pe(next) - pe(k)) / (next.t - k.t) : 0;
      const zIn = !prev || Math.abs(pe(k) - pe(prev)) <= holdEps, zOut = !next || Math.abs(pe(next) - pe(k)) <= holdEps;
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

  // ---- the solve: angles and lengths before the ceilings --------------------
  function solve(knots, cfg, withDp) {
    const c = { ...DEF, ...cfg };
    classify(knots, c.holdEps, withDp);
    const pe = (k) => pos(k) + (withDp ? k.dp || 0 : 0);
    for (let i = 0; i < knots.length; i++) {
      const k = knots[i], prev = knots[i - 1], next = knots[i + 1], m = k.man;
      const smooth = c.style === 'smooth';
      k.eff = k.cls === 'end' ? 'G1' : k.type !== 'auto' ? k.type : (k.cls === 'through' || smooth) ? 'G2' : 'G1';
      const base = (k.cls === 'end' || k.cls === 'rest') ? 0 : smooth && knots.length > 3 ? makimaAngle(knots, i) : k.cls === 'through' ? pchipAngle(k, prev, next) : 0;
      if (k.eff === 'G0') { k.vIn = m.vIn ?? k.dIn; k.vOut = m.vOut ?? k.dOut; }
      else { const v = m.v ?? base; k.vIn = k.vOut = v; }
      k.lIn = m.lIn ?? THIRD; k.lOut = m.lOut ?? THIRD;
      k.note = [];
      const cv = (v) => Math.max(-c.vmax, Math.min(c.vmax, v));
      if (cv(k.vIn) !== k.vIn || cv(k.vOut) !== k.vOut) { k.vIn = cv(k.vIn); k.vOut = cv(k.vOut); k.note.push('angle held to the speed ceiling'); }
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
  const over = (r, c) => Math.max(r.peakV / c.vmax, r.peakA / c.amax, r.peakJ / c.jmax, 1 + Math.max(0, c.lo - r.pMin, r.pMax - c.hi) / (c.hi - c.lo));
  const KS = []; for (let k = 0.2; k <= 2.0001; k += 0.05) KS.push(k);
  KS.sort((x, y) => Math.abs(x - 1) - Math.abs(y - 1));
  function fitPiece(L, R, pL, pR, c) {
    const T = R.t - L.t, D = pR - pL, floor = Math.max(LMIN, c.lfloor);
    const cl = (l, hand) => (hand ? clampL(l) : Math.min(LMAX, Math.max(floor, l)));
    let best = null;
    for (const k of KS) {
      const i0 = cl(L.lOut * k, L.man.lOut != null), i1 = cl(R.lIn * k, R.man.lIn != null);
      const r = sample(L.t, pL, D, T, L.vOut, R.vIn, i0, i1, NS);
      const o = over(r, c);
      if (o <= 1 + TOL) return { k, i0, i1, o, legal: true };
      if (!best || o < best.o) best = { k, i0, i1, o, legal: false };
    }
    return best;
  }
  function nudge(knots, cfg) {
    const c = { ...DEF, ...cfg };
    for (const k of knots) { k.effIn = k.lIn; k.effOut = k.lOut; k.dp = 0; k.infeasible = false; }
    const steps = [];
    for (let i = 0; i < knots.length - 1; i++) {
      const L = knots[i], R = knots[i + 1];
      const pL = pos(L) + L.dp;
      // a hold after a trimmed knot moves with it: the hold stays flat at the trimmed height
      const hold = Math.abs(pos(R) - pos(L)) <= c.holdEps;
      let f = fitPiece(L, R, pL, pos(R) + (hold ? L.dp : 0), c), dp = hold ? L.dp : 0;
      if (!f.legal && !hold && c.trim > 0 && i + 1 < knots.length - 1) {
        const dir = Math.sign(pL - pos(R)); let lo = 0, hi = Math.min(c.trim, Math.abs(pos(R) - pL));
        let fh = fitPiece(L, R, pL, pos(R) + dir * hi, c);
        if (fh.legal) {
          for (let n = 0; n < 16; n++) { const m = (lo + hi) / 2, fm = fitPiece(L, R, pL, pos(R) + dir * m, c); if (fm.legal) { hi = m; fh = fm; } else lo = m; }
          f = fh; dp = dir * hi;
        } else { // nothing reachable: take the least-over position on the way
          for (const q of [0.25, 0.5, 0.75, 1]) { const fq = fitPiece(L, R, pL, pos(R) + dir * q * hi, c); if (fq.o < f.o) { f = fq; dp = dir * q * hi; } }
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
    solve(knots, c);
    let bad = nudge(knots, c);
    for (let round = 0; round < 2; round++) { solve(knots, c, true); bad = nudge(knots, c); }  // angles follow the trimmed chords
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
