// handles.js -- the handles page (kin-d0o): a funscript through the real
// Kinetic² planner (kinetic2.wasm), drawn as the planner rendered it: the
// curve and its velocity, acceleration and jerk against the ceilings, each
// knot's class, angle, handles, trim and verdict.
// Constraints:
// - Every drawn value is the planner's: the 1 ms samples, and per knot the
//   render of the last solve that rendered it with both neighbors
//   (kinetic2_render, wasm/kinetic2_wasm.h). Nothing is recomputed here.
// - kinetic2-trace.wasm loads only with the planner steps (steps.js).
// - Inputs are the planner's only: knot times and positions, an authored angle
//   per knot, the ceilings, smoothness, handle floor and max trim.
// - Every change is a full replay from reset; nothing is cached across edits.
// - Plain script, no module; the wasm needs the page served over http.
// See: playground/README.md "The handles page"
'use strict';
const $ = (s) => document.querySelector(s);
// The canvas palette: play.css's tokens for the theme shown, read again on "themechange" and
// changed in place (steps.js holds this object). bad: over a ceiling, as on the playground.
const TOKENS = { bg: '--bg', raised: '--bg-raised', sunken: '--bg-sunken', l1: '--line-1', l2: '--line-2', l3: '--line-3', l4: '--line-4',
  text: '--text', t2: '--text-2', t3: '--text-3', muted: '--muted', blue: '--reality', violet: '--intent', hi: '--hi', bad: '--bad',
  v: '--plot-v', a: '--plot-a', j: '--plot-j' };
const C = {};
function palette() { const cs = getComputedStyle(document.documentElement); for (const k in TOKENS) C[k] = cs.getPropertyValue(TOKENS[k]).trim(); }
palette();
// a palette color (#RRGGBB) at alpha a
const alpha = (hex, a) => `rgba(${parseInt(hex.slice(1, 3), 16)},${parseInt(hex.slice(3, 5), 16)},${parseInt(hex.slice(5, 7), 16)},${a})`;
const MONO = 'ui-monospace, "SF Mono", Consolas, monospace';
const SANS = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const CLSN = ['end', 'rest', 'crest', 'through'];
const CLS = (cls) => [C.muted, C.t3, C.text, C.blue][cls];   // by class, CLSN's order
const SLOTS = 64;   // the engine's pending knots (kinetic2_wasm.cpp Engine<1, 64>)
// the built-in script: holds, a fast stroke, a mid-travel point, a sawtooth
const SAMPLE = { actions: [
  { at: 0, pos: 10 }, { at: 500, pos: 10 }, { at: 700, pos: 90 }, { at: 900, pos: 10 }, { at: 1100, pos: 90 }, { at: 1300, pos: 10 },
  { at: 1800, pos: 10 }, { at: 1950, pos: 60 }, { at: 2200, pos: 100 }, { at: 2600, pos: 0 }, { at: 3100, pos: 0 },
  { at: 3300, pos: 70 }, { at: 3400, pos: 50 }, { at: 3600, pos: 100 }, { at: 3800, pos: 20 }, { at: 4300, pos: 20 },
  { at: 4400, pos: 80 }, { at: 4700, pos: 30 }, { at: 4800, pos: 90 }, { at: 5100, pos: 40 }, { at: 5200, pos: 95 }, { at: 5700, pos: 10 }, { at: 6200, pos: 10 },
] };

let cfg = { vmax: 1000, amax: 50000, jmax: 5e6, lfloor: 0.15, trim: 100, lo: 0, hi: 100, smoothness: 0, rail: 500 };
// knot: { t s, p mm (authored), pm mm | null (moved by hand), v mm/s | null (angle set by hand) }
let knots = [], R = null, sel = null, T = null;
let view = { t0: 0, t1: 6 };
let drag = null, dragRaf = 0;
let cursor = null, playing = false, playT0 = 0, raf = 0;
const plot = $('#plot'), strips = $('#strips'), overlay = $('#overlay'), soverlay = $('#soverlay'), rail = $('#rail');
const PAD = { l: 44, r: 12, t: 10, b: 20 };
const pos = (k) => (k.pm == null ? k.p : k.pm);
const winW = () => cfg.hi - cfg.lo;
const mm = (x) => cfg.lo + x * winW();   // window units to mm

// ---- the planner ----------------------------------------------------------------
async function boot() {
  try {
    const res = await fetch('./kinetic2.wasm');
    if (!res.ok) throw new Error(res.status + ' ' + res.statusText);
    const { instance } = await WebAssembly.instantiate(await res.arrayBuffer(), {});
    const K = instance.exports;
    if (!K.kinetic2_render || K.kinetic2_submit.length !== 7) throw new Error('kinetic2.wasm does not match this page; rebuild it');
    K._initialize();
    T = { K, h: K.kinetic2_create(), dv: new DataView(K.memory.buffer), sBuf: K.malloc(24), kBuf: K.malloc(40), aBuf: K.malloc(24), rBuf: K.malloc(56) };
  } catch (e) {
    $('#name').textContent = 'the planner did not load';
    $('#status').textContent = 'kinetic2.wasm did not load (' + e.message + '). Build it and serve this folder: see playground/README.md.';
    return;
  }
  load(SAMPLE, 'built-in');
}

// One replay: configure, reset at the first knot, submit the rest on its time
// (64 ahead, the next as each retires), sample every 1 ms. onSolve(W, live,
// first) runs after each solve, live the knot index of each pending knot.
function replay(W, onSolve) {
  const { K, h, dv, sBuf, kBuf } = W, Wmm = winW(), w = (p) => (p - cfg.lo) / Wmm;
  const ok = K.kinetic2_configure(h, cfg.vmax / Wmm, cfg.amax / Wmm, cfg.jmax / Wmm, cfg.smoothness,
    Math.min(0.33, Math.max(0.05, cfg.lfloor)), Math.min(1, Math.max(0, cfg.trim / Wmm)), 4000);
  const t0 = Math.round(knots[0].t * 1e6), z = Math.floor(t0 / 1000);
  K.kinetic2_reset(h, w(pos(knots[0])), t0);
  const n = Math.round((knots[knots.length - 1].t + 0.3) * 1000) + 1;
  const P = new Float32Array(n), V = new Float32Array(n), A = new Float32Array(n), J = new Float32Array(n);
  const live = [];
  let next = 1, solves = 0, refused = 0, mv = 0, ma = 0, mj = 0, over = 0;
  for (let i = 0; i < n; i++) {
    if (i < z) { P[i] = w(pos(knots[0])); continue; }   // before the origin the axis rests on it
    const now = Math.max(i * 1000, t0);
    let sub = false;
    while (next < knots.length && K.kinetic2_pending(h) < SLOTS) {
      const k = knots[next];
      if (K.kinetic2_submit(h, Math.round(k.t * 1e6), w(pos(k)), k.v == null ? 0 : 1, (k.v ?? 0) / Wmm, now, 2)) live.push(next); else refused++;
      next++; sub = true;
    }
    if (sub) {
      K.kinetic2_solved(h, 0, kBuf);   // solves the window
      while (live.length > K.kinetic2_pending(h)) live.shift();   // an undone re-plan retires one
      if (onSolve) onSolve(W, live, solves === 0);
      solves++;
    }
    K.kinetic2_sample(h, now, sBuf);
    P[i] = dv.getFloat32(sBuf + 8, true); V[i] = dv.getFloat32(sBuf + 12, true); A[i] = dv.getFloat32(sBuf + 16, true);
    J[i] = i > z ? (A[i] - A[i - 1]) * 1000 : 0;
    mv = Math.max(mv, Math.abs(V[i])); ma = Math.max(ma, Math.abs(A[i])); mj = Math.max(mj, Math.abs(J[i]));
    if (Math.abs(V[i]) > cfg.vmax / Wmm * 1.001 || Math.abs(A[i]) > cfg.amax / Wmm * 1.001 || Math.abs(J[i]) > cfg.jmax / Wmm * 1.001) over++;
    while (live.length > K.kinetic2_pending(h)) live.shift();
  }
  return { n, P, V, A, J, max: { v: mv, a: ma, j: mj }, over, ok, refused };
}

// The knots of one solve (kinetic2_render): a knot is written from a solve
// where it has a knot on both sides (window index 2 on), or from the first
// solve, whose origin is the first knot at rest.
function readSolve({ K, h, dv, rBuf }, live, first, D) {
  for (let r = first ? 0 : 2; r <= live.length; r++) {
    if (!K.kinetic2_render(h, r, rBuf)) return;
    const f = (o) => dv.getFloat32(rBuf + o, true), u = (o) => dv.getUint8(rBuf + o);
    D[r === 0 ? 0 : live[r - 1]] = { p: f(0), dp: f(4), vel: f(8), lIn: f(12), lOut: f(16), ei: f(20), eo: f(24), TL: f(28), TR: f(32),
      share: f(36), worst: f(40), rampT: f(44), cls: u(48), g2: u(49), corner: u(50), inf: u(51), clamped: u(52) };
  }
}

// ---- the script -------------------------------------------------------------------
function fromFunscript(doc) {
  const acts = (doc.actions || []).slice().sort((x, y) => x.at - y.at), out = [];
  for (const a of acts) {
    const t = a.at / 1000, p = cfg.lo + winW() * Math.min(100, Math.max(0, a.pos)) / 100;
    if (out.length && t - out[out.length - 1].t < 1e-6) { out[out.length - 1].p = p; continue; }
    out.push({ t, p, pm: null, v: null });
  }
  return out;
}
function load(doc, label) {
  knots = fromFunscript(doc); sel = null;
  $('#name').textContent = `${label}: ${knots.length} knots`;
  fitView(); update();
}
function update() {
  if (!T) return;
  if (knots.length < 2) { R = null; draw(); return; }
  const D = new Array(knots.length).fill(null);
  R = replay(T, (W, live, first) => readSolve(W, live, first, D));
  R.D = D;
  // per sample: 1 v, 2 a, 4 j over its ceiling, 8 in a piece no trim keeps inside
  const Wmm = winW(), fl = new Uint8Array(R.n);
  for (let i = 0; i < R.n; i++)
    fl[i] = (Math.abs(R.V[i]) > cfg.vmax / Wmm * 1.001 ? 1 : 0) | (Math.abs(R.A[i]) > cfg.amax / Wmm * 1.001 ? 2 : 0) | (Math.abs(R.J[i]) > cfg.jmax / Wmm * 1.001 ? 4 : 0);
  for (let i = 1; i < knots.length; i++) if (D[i] && D[i].inf) for (let s = Math.round(knots[i - 1].t * 1000); s <= Math.round(knots[i].t * 1000); s++) fl[s] |= 8;
  R.fl = fl;
  const msg = [!R.ok && 'The planner refused these ceilings or options; it kept the last ones it took.', R.refused && `The planner refused ${R.refused} knots.`].filter(Boolean);
  $('#status').textContent = msg.join(' ');
  showSteps();
  draw(); inspector(); stats(); drawCursor();
}
function fitView() {
  if (!knots.length) return;
  const t1 = knots[knots.length - 1].t, pad = Math.max(0.05, t1 * 0.02);
  view = { t0: knots[0].t - pad, t1: t1 + pad };
}

// ---- geometry -----------------------------------------------------------------------
function dims(cv) {
  const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  if (cv.width !== Math.round(r.width * dpr) || cv.height !== Math.round(r.height * dpr)) { cv.width = Math.round(r.width * dpr); cv.height = Math.round(r.height * dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}
const X = (t, w) => PAD.l + (t - view.t0) / (view.t1 - view.t0) * (w - PAD.l - PAD.r);
const Tx = (x, w) => view.t0 + (x - PAD.l) / (w - PAD.l - PAD.r) * (view.t1 - view.t0);
const Y = (p, h) => PAD.t + (cfg.hi - p) / (cfg.hi - cfg.lo) * (h - PAD.t - PAD.b);
const P = (y, h) => cfg.hi - (y - PAD.t) / (h - PAD.t - PAD.b) * (cfg.hi - cfg.lo);
const dOf = (i) => (R && R.D[i]) || null;
const knotP = (i) => { const d = dOf(i); return d ? mm(d.p) : pos(knots[i]); };

// a knot's two handle ends in (s, mm): its angle times each handle's time extent
function handleEnds(i) {
  const k = knots[i], d = dOf(i), out = {};
  if (!d) return out;
  const p = mm(d.p), v = d.vel * winW();
  if (i > 0 && d.TL > 0) out.in = [k.t - d.ei * d.TL, p - v * d.ei * d.TL];
  if (i < knots.length - 1 && d.TR > 0) out.out = [k.t + d.eo * d.TR, p + v * d.eo * d.TR];
  return out;
}
function visibleHandles() {
  const w = plot.getBoundingClientRect().width;
  const pxPerS = (w - PAD.l - PAD.r) / (view.t1 - view.t0);
  const dense = knots.length > 1 ? pxPerS * ((knots[knots.length - 1].t - knots[0].t) / (knots.length - 1)) > 70 : true;
  const set = new Set();
  if (dense) knots.forEach((_, i) => set.add(i));
  if (sel != null) { set.add(sel); if (sel > 0) set.add(sel - 1); if (sel < knots.length - 1) set.add(sel + 1); }
  return set;
}
const nudged = (d) => !!d && (Math.abs(d.ei - d.lIn) > 1e-6 || Math.abs(d.eo - d.lOut) > 1e-6);

// The visible 1 ms samples of arr, about two per pixel: each step draws its
// first value and its range, in the color of the worst flag inside it.
function series(ctx, w, arr, yOf, colorOf) {
  const i0 = Math.max(0, Math.floor(view.t0 * 1000)), i1 = Math.min(R.n - 1, Math.ceil(view.t1 * 1000));
  const st = Math.max(1, Math.floor((i1 - i0) / (2 * w)));
  let col = null, px = 0, py = 0;
  for (let i = i0; i <= i1; i += st) {
    let lo = arr[i], hi = arr[i], f = 0;
    for (let k = i; k < Math.min(i1 + 1, i + st); k++) { if (arr[k] < lo) lo = arr[k]; if (arr[k] > hi) hi = arr[k]; f |= R.fl[k]; }
    const c = colorOf(f), x = X(i / 1000, w), y = yOf(arr[i]);
    if (c !== col) { if (col) ctx.stroke(); ctx.strokeStyle = c; ctx.beginPath(); ctx.moveTo(col ? px : x, col ? py : y); col = c; }
    ctx.lineTo(x, y);
    if (st > 1 && lo !== hi) { ctx.lineTo(x, yOf(lo)); ctx.lineTo(x, yOf(hi)); ctx.lineTo(x, y); }
    px = x; py = y;
  }
  if (col) ctx.stroke();
}

// ---- drawing ------------------------------------------------------------------------
function ticks(ctx, w, h) {
  const span = view.t1 - view.t0, step = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 60].find((s) => span / s <= 12) || 60;
  ctx.font = `10px ${MONO}`; ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.strokeStyle = C.l2; ctx.lineWidth = 1;
  for (let t = Math.ceil(view.t0 / step) * step; t <= view.t1; t += step) {
    const x = X(t, w); ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, h - PAD.b); ctx.stroke();
    ctx.fillText(step >= 1 ? `${t.toFixed(0)} s` : `${(t * 1000).toFixed(0)}`, x, h - 6);
  }
}
function draw() {
  const { ctx, w, h } = dims(plot);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  ticks(ctx, w, h);
  ctx.textAlign = 'right'; ctx.fillStyle = C.muted;
  for (const p of [cfg.lo, (cfg.lo + cfg.hi) / 2, cfg.hi]) ctx.fillText(p.toFixed(0), PAD.l - 6, Y(p, h) + 3);
  ctx.strokeStyle = C.l2; ctx.beginPath(); ctx.moveTo(PAD.l, Y(cfg.lo, h)); ctx.lineTo(w - PAD.r, Y(cfg.lo, h)); ctx.moveTo(PAD.l, Y(cfg.hi, h)); ctx.lineTo(w - PAD.r, Y(cfg.hi, h)); ctx.stroke();
  if (!R) { drawStrips(); return; }
  ctx.save(); ctx.beginPath(); ctx.rect(PAD.l, 0, w - PAD.l - PAD.r, h); ctx.clip();
  // the script as authored, faint
  ctx.strokeStyle = C.l3; ctx.setLineDash([3, 4]); ctx.lineWidth = 1; ctx.beginPath();
  knots.forEach((k, i) => (i ? ctx.lineTo(X(k.t, w), Y(k.p, h)) : ctx.moveTo(X(k.t, w), Y(k.p, h)))); ctx.stroke(); ctx.setLineDash([]);
  // the planner's motion, red where a sample is over a ceiling, pink in a piece no trim keeps inside
  ctx.lineWidth = 2;
  series(ctx, w, R.P, (x) => Y(mm(x), h), (f) => (f & 7 ? C.bad : f & 8 ? C.hi : C.blue));
  // handles
  for (const i of visibleHandles()) {
    const d = dOf(i); if (!d) continue;
    const e = handleEnds(i), kx = X(knots[i].t, w), ky = Y(mm(d.p), h);
    for (const side of ['in', 'out']) {
      if (!e[side]) continue;
      const len = side === 'in' ? d.ei : d.eo, pre = side === 'in' ? d.lIn : d.lOut, moved = Math.abs(len - pre) > 1e-6;
      ctx.strokeStyle = moved ? C.violet : (i === sel ? C.text : C.t3); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(kx, ky); ctx.lineTo(X(e[side][0], w), Y(e[side][1], h)); ctx.stroke();
      ctx.fillStyle = C.bg; ctx.beginPath(); ctx.arc(X(e[side][0], w), Y(e[side][1], h), 3.5, 0, 7); ctx.fill(); ctx.stroke();
      if (moved) { // the length before the ceilings, ghosted
        const s = side === 'in' ? -1 : 1, T_ = side === 'in' ? d.TL : d.TR, v = d.vel * winW();
        const g = [knots[i].t + s * pre * T_, mm(d.p) + s * v * pre * T_];
        ctx.strokeStyle = C.muted; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.arc(X(g[0], w), Y(g[1], h), 3.5, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      }
    }
  }
  // knots
  knots.forEach((k, i) => {
    const d = dOf(i), x = X(k.t, w), y = Y(knotP(i), h);
    if (x < PAD.l - 5 || x > w - PAD.r + 5) return;
    if ((d && d.dp !== 0) || k.pm != null) { ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.arc(x, Y(k.p, h), 3.5, 0, 7); ctx.stroke(); }
    ctx.fillStyle = !d ? C.muted : d.inf ? C.bad : CLS(d.cls); ctx.beginPath(); ctx.arc(x, y, i === sel ? 5 : 3.5, 0, 7); ctx.fill();
    if (i === sel) { ctx.strokeStyle = C.text; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 8, 0, 7); ctx.stroke(); }
    if (k.v != null && d) { ctx.fillStyle = C.violet; ctx.font = `9px ${MONO}`; ctx.textAlign = 'center'; ctx.fillText(d.g2 ? 'G2' : 'G1', x, y - 11); }
  });
  ctx.restore();
  drawStrips();
}
function drawStrips() {
  const { ctx, w, h } = dims(strips);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  if (!R) return;
  const Wmm = winW();
  const rows = [['V', 1, 'velocity mm/s', cfg.vmax, C.v], ['A', 2, 'acceleration mm/s²', cfg.amax, C.a], ['J', 4, 'jerk mm/s³', cfg.jmax, C.j]];
  const rh = (h - 8) / 3;
  const i0 = Math.max(0, Math.floor(view.t0 * 1000)), i1 = Math.min(R.n - 1, Math.ceil(view.t1 * 1000));
  rows.forEach(([key, bit, label, ceil, color], r) => {
    const y0 = 4 + r * rh, y1 = y0 + rh - 6, arr = R[key];
    let peak = ceil * 1.2;
    for (let i = i0; i <= i1; i++) peak = Math.max(peak, Math.abs(arr[i]) * Wmm);
    peak = Math.min(peak, ceil * 3);
    const YY = (val) => y0 + (y1 - y0) / 2 - val / peak * (y1 - y0) / 2;
    ctx.fillStyle = C.raised; ctx.fillRect(PAD.l, y0, w - PAD.l - PAD.r, y1 - y0);
    ctx.save(); ctx.beginPath(); ctx.rect(PAD.l, y0, w - PAD.l - PAD.r, y1 - y0); ctx.clip();
    ctx.strokeStyle = C.l2; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(PAD.l, YY(0)); ctx.lineTo(w - PAD.r, YY(0)); ctx.stroke();
    ctx.strokeStyle = C.bad; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(PAD.l, YY(ceil)); ctx.lineTo(w - PAD.r, YY(ceil)); ctx.moveTo(PAD.l, YY(-ceil)); ctx.lineTo(w - PAD.r, YY(-ceil)); ctx.stroke(); ctx.setLineDash([]);
    ctx.lineWidth = 1.5;
    series(ctx, w, arr, (x) => YY(x * Wmm), (f) => (f & bit ? C.bad : color));
    if (sel != null) { const x = X(knots[sel].t, w); ctx.strokeStyle = C.text; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke(); }
    ctx.restore();
    ctx.fillStyle = C.muted; ctx.font = `10px ${SANS}`; ctx.textAlign = 'left'; ctx.fillText(`${label}   ceiling ${fmt(ceil)}`, PAD.l + 4, y0 + 11);
  });
}
const fmt = (x, d = 0) => (Math.abs(x) >= 1e6 ? `${(x / 1e6).toFixed(2)}e6` : Number(x).toFixed(d));

// ---- inspector and stats ------------------------------------------------------------
// the peaks of the samples from knot a to knot b, mm units
function peaks(a, b) {
  const W = winW(); let v = 0, ac = 0, j = 0;
  for (let s = Math.round(knots[a].t * 1000); s <= Math.round(knots[b].t * 1000); s++) { v = Math.max(v, Math.abs(R.V[s])); ac = Math.max(ac, Math.abs(R.A[s])); j = Math.max(j, Math.abs(R.J[s])); }
  return { v: v * W, a: ac * W, j: j * W };
}
function inspector() {
  const k = sel != null ? knots[sel] : null, d = sel != null ? dOf(sel) : null;
  $('#k-none').hidden = !!k; $('#k-box').hidden = !k; $('#k-idx').textContent = k ? `${sel + 1} of ${knots.length}` : '';
  if (!k) return;
  const rows = [['time', `${(k.t * 1000).toFixed(0)} ms`]];
  if (d) {
    const W = winW(), prev = knots[sel - 1], next = knots[sel + 1];
    rows.push(['class', `<span class="cls-${CLSN[d.cls]}">${CLSN[d.cls]}</span>`], ['renders as', d.g2 ? 'G2' : 'G1'],
      ['angle', `${fmt(d.vel * W)} mm/s, ${k.v != null ? 'set by hand' : sel === 0 ? 'at rest' : 'solved'}`],
      ['lengths', `${d.lIn.toFixed(3)} in, ${d.lOut.toFixed(3)} out`],
      ['in ms', [prev && `${(d.ei * d.TL * 1000).toFixed(0)} in`, next && `${(d.eo * d.TR * 1000).toFixed(0)} out`].filter(Boolean).join(', ')]);
    const pieceRow = (a, b, label) => { const pk = peaks(a, b), wr = dOf(b) ? dOf(b).worst : 0; return [label, `v ${fmt(pk.v)}  a ${fmt(pk.a)}  j ${fmt(pk.j)}${wr > 1.001 ? ` <span class="bad">over ×${wr.toFixed(2)}</span>` : ''}`]; };
    if (prev) rows.push(pieceRow(sel - 1, sel, 'piece in'));
    if (next) rows.push(pieceRow(sel, sel + 1, 'piece out'));
    if (nudged(d)) rows.push(['after ceilings', `<dd class="nudge">${d.ei.toFixed(3)} in, ${d.eo.toFixed(3)} out`]);
    if (d.dp !== 0) rows.push(['trimmed', `<dd class="nudge">${d.dp > 0 ? '+' : ''}${(d.dp * W).toFixed(2)} mm toward the previous knot, keeps ${(d.share * 100).toFixed(1)}%`]);
    if (d.inf) rows.push(['', '<dd class="bad">no legal length, even trimmed']);
    if (d.corner) rows.push(['corner ramp', `${(d.rampT * 1000).toFixed(1)} ms at this jerk`]);
  }
  $('#k-info').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt>${b.startsWith('<dd') ? b + '</dd>' : `<dd>${b}</dd>`}`).join('');
  $('#k-vrow').hidden = sel === 0;
  setIfIdle('#k-v', d ? (d.vel * winW()).toFixed(0) : ''); setIfIdle('#k-p', pos(k).toFixed(1));
  const notes = [d && d.clamped && 'angle held to the speed ceiling', k.v != null && 'angle set by hand', k.pm != null && 'moved by hand', sel === 0 && 'the origin: the motion starts here at rest'].filter(Boolean);
  $('#k-note').textContent = notes.join('. ');
}
function setIfIdle(sel_, v) { const el = $(sel_); if (document.activeElement !== el) el.value = v; }
function stats() {
  if (!R) return;
  const D = R.D, c = [0, 0, 0, 0];
  for (const d of D) if (d) c[d.cls]++;
  const inf = D.filter((d) => d && d.inf).length, tr = D.filter((d) => d && d.dp !== 0);
  const rows = [['knots', knots.length], ['holds', c[1]], ['crests', c[2]], ['through', c[3]],
    ['pieces over', `<dd class="${inf ? 'bad' : ''}">${inf} of ${knots.length - 1}`],
    ['samples over', `<dd class="${R.over ? 'bad' : ''}">${R.over} of ${R.n}`],
    ['nudged', D.filter(nudged).length],
    ['trimmed', `${tr.length} knots, ${tr.reduce((m, d) => m + Math.abs(d.dp) * winW(), 0).toFixed(1)} mm`],
    ['longest ramp', `${(D.reduce((m, d) => Math.max(m, d ? d.rampT : 0), 0) * 1000).toFixed(1)} ms`]];
  $('#stats').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt>${String(b).startsWith('<dd') ? b + '</dd>' : `<dd>${b}</dd>`}`).join('');
}

// ---- pointer --------------------------------------------------------------------------
function hit(ev) {
  const r = plot.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top, w = r.width, h = r.height;
  for (const i of visibleHandles()) {
    if (i === 0) continue;   // the origin rests: no angle to set
    const e = handleEnds(i);
    for (const side of ['in', 'out']) if (e[side] && Math.hypot(X(e[side][0], w) - x, Y(e[side][1], h) - y) < 7) return { kind: 'handle', i, side };
  }
  let best = null;
  knots.forEach((k, i) => { const d = Math.hypot(X(k.t, w) - x, Y(knotP(i), h) - y); if (d < 9 && (!best || d < best.d)) best = { kind: 'knot', i, d }; });
  return best;
}
// a drag replays at most once per frame
function dragUpdate() { if (!dragRaf) dragRaf = requestAnimationFrame(() => { dragRaf = 0; update(); }); }
plot.addEventListener('pointerdown', (ev) => {
  if (ev.button !== 0) return;
  const t = hit(ev);
  const r = plot.getBoundingClientRect();
  if (t) { sel = t.i; drag = { ...t }; }
  else drag = { kind: 'pan', x0: ev.clientX, v0: { ...view }, w: r.width };
  plot.setPointerCapture(ev.pointerId); update();
});
plot.addEventListener('pointermove', (ev) => {
  const r = plot.getBoundingClientRect(), w = r.width, h = r.height, x = ev.clientX - r.left, y = ev.clientY - r.top;
  if (!drag) { const t = hit(ev); plot.style.cursor = t ? 'pointer' : 'grab'; if (!playing) readout(Tx(x, w), P(y, h)); return; }
  if (drag.kind === 'pan') { const dt = (ev.clientX - drag.x0) / (drag.w - PAD.l - PAD.r) * (drag.v0.t1 - drag.v0.t0); view = { t0: drag.v0.t0 - dt, t1: drag.v0.t1 - dt }; draw(); drawCursor(); return; }
  const k = knots[drag.i];
  if (drag.kind === 'knot') { k.pm = Math.min(cfg.hi, Math.max(cfg.lo, P(y, h))); dragUpdate(); return; }
  // a handle: its angle points at the pointer; its length stays the planner's
  const tt = Tx(x, w), pp = P(y, h), p = knotP(drag.i), dt = drag.side === 'in' ? k.t - tt : tt - k.t;
  if (dt > 1e-4) { k.v = (drag.side === 'in' ? p - pp : pp - p) / dt; dragUpdate(); }
});
plot.addEventListener('pointerup', () => { drag = null; });
plot.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  const r = plot.getBoundingClientRect(), w = r.width, tc = Tx(ev.clientX - r.left, w), f = ev.deltaY > 0 ? 1.2 : 1 / 1.2;
  view = { t0: tc - (tc - view.t0) * f, t1: tc + (view.t1 - tc) * f }; draw(); drawCursor();
}, { passive: false });
function readout(t, p) { $('#readout').textContent = `${(t * 1000).toFixed(0)} ms   ${p.toFixed(1)} mm`; }

// ---- playback: the planner's samples at real speed, the carriage on the rail ----
function stateAt(t) {
  if (!R) return null;
  const i = Math.min(R.n - 1, Math.max(0, Math.round(t * 1000))), W = winW();
  return { p: mm(R.P[i]), v: R.V[i] * W, a: R.A[i] * W };
}
function drawCursor() {
  const { ctx, w, h } = dims(overlay); ctx.clearRect(0, 0, w, h);
  const s2 = dims(soverlay); s2.ctx.clearRect(0, 0, s2.w, s2.h);
  drawRail();
  if (cursor == null) return;
  const x = X(cursor, w);
  for (const [c, hh] of [[ctx, h], [s2.ctx, s2.h]]) { c.strokeStyle = C.text; c.lineWidth = 1; c.beginPath(); c.moveTo(x, 0); c.lineTo(x, hh); c.stroke(); }
  const st = stateAt(cursor);
  if (st) { ctx.fillStyle = C.text; ctx.beginPath(); ctx.arc(x, Y(st.p, h), 4, 0, 7); ctx.fill(); $('#readout').textContent = `${(cursor * 1000).toFixed(0)} ms   ${st.p.toFixed(1)} mm   ${st.v.toFixed(0)} mm/s   ${st.a.toFixed(0)} mm/s²`; }
}
function drawRail() {
  const { ctx, w, h } = dims(rail); ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  const top = 12, bot = h - 12, YR = (v) => bot - v / cfg.rail * (bot - top);
  ctx.fillStyle = C.l1; ctx.fillRect(w / 2 - 5, top, 10, bot - top);
  ctx.fillStyle = alpha(C.blue, 0.18); ctx.fillRect(w / 2 - 12, YR(cfg.hi), 24, YR(cfg.lo) - YR(cfg.hi));
  ctx.strokeStyle = C.blue; ctx.lineWidth = 1; ctx.strokeRect(w / 2 - 12, YR(cfg.hi), 24, YR(cfg.lo) - YR(cfg.hi));
  ctx.fillStyle = C.muted; ctx.font = `9px ${MONO}`; ctx.textAlign = 'center';
  ctx.fillText(`${cfg.rail}`, w / 2, top - 3); ctx.fillText('0', w / 2, bot + 10);
  const st = cursor == null ? null : stateAt(cursor);
  const p = st ? st.p : (knots.length ? pos(knots[0]) : cfg.lo);
  ctx.fillStyle = C.text; ctx.fillRect(w / 2 - 16, YR(p) - 3, 32, 6);
}
function frame(now) {
  if (!playing) return;
  cursor = (now - playT0) / 1000;
  const end = knots.length ? knots[knots.length - 1].t + 0.3 : 0;
  if (cursor > end) { playT0 = now; cursor = 0; }
  drawCursor(); raf = requestAnimationFrame(frame);
}
function togglePlay() {
  playing = !playing; $('#play').textContent = playing ? 'Pause' : 'Play';
  if (playing) { playT0 = performance.now() - (cursor || 0) * 1000; raf = requestAnimationFrame(frame); } else cancelAnimationFrame(raf);
}
$('#play').addEventListener('click', togglePlay);

// ---- controls ---------------------------------------------------------------------------
$('#file').addEventListener('change', (ev) => {
  const f = ev.target.files[0]; if (!f) return;
  f.text().then((txt) => { try { load(JSON.parse(txt), f.name); } catch (e) { $('#status').textContent = `not a funscript: ${e.message}`; } });
});
document.addEventListener('dragover', (ev) => ev.preventDefault());
document.addEventListener('drop', (ev) => { ev.preventDefault(); const f = ev.dataTransfer.files[0]; if (f) f.text().then((txt) => load(JSON.parse(txt), f.name)); });
$('#sample').addEventListener('click', () => load(SAMPLE, 'built-in'));
$('#autoall').addEventListener('click', () => { for (const k of knots) { k.v = null; k.pm = null; } update(); });
$('#fit').addEventListener('click', () => { fitView(); draw(); drawCursor(); });
$('#k-auto').addEventListener('click', () => { if (sel == null) return; const k = knots[sel]; k.v = null; k.pm = null; update(); });
$('#k-prev').addEventListener('click', () => { if (sel > 0) { sel--; update(); } });
$('#k-next').addEventListener('click', () => { if (sel != null && sel < knots.length - 1) { sel++; update(); } });
$('#k-v').addEventListener('change', (ev) => { if (sel == null || sel === 0) return; const v = Number(ev.target.value); if (ev.target.value === '' || !isFinite(v)) return; knots[sel].v = v; update(); });
$('#k-p').addEventListener('change', (ev) => { if (sel == null) return; knots[sel].pm = Math.min(cfg.hi, Math.max(cfg.lo, Number(ev.target.value))); update(); });
for (const [id, key] of [['#c-vmax', 'vmax'], ['#c-amax', 'amax'], ['#c-jmax', 'jmax'], ['#c-lfloor', 'lfloor'], ['#c-trim', 'trim']]) {
  $(id).value = cfg[key];
  $(id).addEventListener('change', (ev) => { const v = Number(ev.target.value); if (isFinite(v) && v >= 0) { cfg[key] = v; update(); } });
}
// the window: size and offset on the rail; the script rescales into it
function setWindow(lo, hi) {
  const old = { lo: cfg.lo, hi: cfg.hi }; cfg.lo = lo; cfg.hi = hi; cfg.trim = hi - lo; $('#c-trim').value = cfg.trim;
  const f = (p) => lo + (p - old.lo) / (old.hi - old.lo) * (hi - lo);
  for (const k of knots) { k.p = f(k.p); if (k.pm != null) k.pm = f(k.pm); if (k.v != null) k.v *= (hi - lo) / (old.hi - old.lo); }
  syncWindowInputs(); update();
}
function syncWindowInputs() {
  const size = cfg.hi - cfg.lo;
  $('#c-rail').value = cfg.rail; $('#c-size').max = cfg.rail; $('#c-size').value = size; $('#c-sizeout').value = `${size} mm`;
  $('#c-off').max = Math.max(0, cfg.rail - size); $('#c-off').value = cfg.lo; $('#c-offout').value = `${cfg.lo} mm`;
}
$('#c-size').addEventListener('input', (ev) => { const size = Math.min(cfg.rail, Math.max(5, Number(ev.target.value))); const lo = Math.min(cfg.lo, cfg.rail - size); setWindow(lo, lo + size); });
$('#c-off').addEventListener('input', (ev) => { const size = cfg.hi - cfg.lo, lo = Math.min(cfg.rail - size, Math.max(0, Number(ev.target.value))); setWindow(lo, lo + size); });
$('#c-rail').addEventListener('change', (ev) => {
  const r = Number(ev.target.value); if (!isFinite(r) || r < 10) return; cfg.rail = r;
  const size = Math.min(cfg.hi - cfg.lo, r), lo = Math.min(cfg.lo, r - size); setWindow(lo, lo + size);
});
syncWindowInputs();
const syncSmooth = () => { $('#c-smooth').value = cfg.smoothness; $('#c-smoothout').textContent = cfg.smoothness.toFixed(2); };
syncSmooth();
$('#c-smooth').addEventListener('input', (ev) => { cfg.smoothness = Number(ev.target.value); syncSmooth(); update(); });
document.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT' || ev.target.tagName === 'SELECT') return;
  if (ev.target.closest && ev.target.closest('.strip')) return;   // the site header's own keys
  if (ev.key === 'Escape') { sel = null; update(); }
  if (ev.key === ' ') { ev.preventDefault(); togglePlay(); }
  if (ev.key === 'ArrowLeft' && sel > 0) { sel--; update(); }
  if (ev.key === 'ArrowRight' && sel != null && sel < knots.length - 1) { sel++; update(); }
});
new ResizeObserver(() => { draw(); drawCursor(); }).observe(plot);
new ResizeObserver(() => { drawStrips(); drawCursor(); }).observe(strips);
const legend = () => { $('#legend').innerHTML = [['hold edge', CLS(1)], ['crest', CLS(2)], ['through point', CLS(3)], ['nudged handle', C.violet], ['over a ceiling', C.bad], ['infeasible knot', C.hi]]
  .map(([n, c]) => `<li><svg width="10" height="10"><circle cx="5" cy="5" r="4" fill="${c}"/></svg>${n}</li>`).join(''); };
legend();
addEventListener('themechange', () => { palette(); legend(); draw(); drawCursor(); if (stepper) stepper.redraw(); });

// ---- the planner steps ------------------------------------------------------------------
// steps.js, in mm, on its own kinetic2-trace.wasm. Loaded the first time the
// steps are shown, never on the page's load.
let stepper = null, stepsOn = false, stepsLoading = false;
function showSteps() {
  $('#steps').hidden = !stepsOn;
  if (!stepsOn) { if (stepper) stepper.pause(); return; }
  if (stepper) { stepper.update(); return; }
  if (stepsLoading) return;
  stepsLoading = true;
  import('./steps.js').then((m) => m.init($('#steps'), {
    replay: (W) => replay(W), fit: (cv) => { const d = dims(cv); return { g: d.ctx, w: d.w, h: d.h }; }, C, MONO, SANS,
    state: () => { const W = winW(); return { o: { vmax: cfg.vmax / W, amax: cfg.amax / W, jmax: cfg.jmax / W } }; },
    units: () => ({ k: winW(), off: cfg.lo, pd: 2, vd: 1, pu: ' mm', v: 'mm/s', a: 'mm/s²', j: 'mm/s³' }),
  })).then((s) => { stepper = s; if (stepsOn) s.update(); },
    (e) => { stepsOn = false; $('#steps').hidden = true; syncStepsToggle(); $('#status').textContent = 'The planner steps did not load (' + e.message + ').'; })
    .finally(() => { stepsLoading = false; });
}
function syncStepsToggle() { $('#steps-toggle').setAttribute('aria-pressed', String(stepsOn)); }
$('#steps-toggle').addEventListener('click', () => { stepsOn = !stepsOn; syncStepsToggle(); update(); });

// for handles.check.mjs
window.__handles = {
  state: () => ({ knots, R, sel, view, cfg, cursor, playing }),
  setWindow, replay, load,
  xy: (i) => { const r = plot.getBoundingClientRect(); return { x: r.left + X(knots[i].t, r.width), y: r.top + Y(knotP(i), r.height) }; },
  handleXY: (i, side) => { const r = plot.getBoundingClientRect(), e = handleEnds(i)[side]; return { x: r.left + X(e[0], r.width), y: r.top + Y(e[1], r.height) }; },
};
boot();
