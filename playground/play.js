// play.js -- the Kinetic² playground: drives kinetic2.wasm through its C ABI and
// draws what the planner did.
// Constraints:
// - Layouts are the ABI structs in wasm/kinetic2_wasm.h; never guess an offset.
// - kinetic2_sample times must be non-decreasing between resets (sampling
//   retires knots), so every render is a full replay from reset.
// - The solved window is read after each submit and whenever a knot retires;
//   anomalies are drained after every call because the ring keeps 16.

const $ = (s) => document.querySelector(s);
const C = { bg: '#08090B', sunken: '#040507', l1: '#121419', l2: '#1D2026', l3: '#272B31', l4: '#33373E',
  text: '#ECEFF4', t2: '#C3C8D1', t3: '#A8AEB9', muted: '#666C78', blue: '#4DA6FF', violet: '#A78BFA', hi: '#FF5CB3' };
const MONO = 'ui-monospace, "SF Mono", Consolas, monospace';
const SANS = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const TOL = 1e-3;
// kinetic2_submit flags (wasm/kinetic2_wasm.h). Streamed: every knot is a
// sample. Authored ahead: a free knot rests when nothing follows it.
const KNOT_SAMPLE = 0x1, KNOT_REST_IF_LAST = 0x2;
const SPEND = 'rgba(77,166,255,0.55)';   // the reality blue, dimmed so spend marks never hide the trace   // relative slack before a sample counts as over a ceiling

const DEF = { vmax: 3, amax: 30, jmax: 2000, policy: 'blend', floor: 0.25, corner: 'cubic', react: 4, look: 250 };
const SPANS = [1, 2, 5, 10];
const KIND = { 1: 'PlanFailed', 2: 'SettleEngaged', 3: 'EndVelClamped', 4: 'DeadlineStretched', 6: 'WaveformScaled', 10: 'DwellZeroed', 11: 'KnotRefused' };
const SENTINEL = { '-99': 'kDetailNonFinite: a non-finite value', '-95': 'kDetailPast: at or before now or the newest knot', '-96': 'kDetailTimelineFull: all 64 knot slots are pending' };

// ---- state -----------------------------------------------------------------
// knot: { t: s, p: window, fam: 2 C2 | 1 C1, v: null (free) | u/s }
let S = { T: 2, mode: 'ahead', lat: 100, o: { ...DEF }, p0: 0.5, knots: [] };
let sel = null;          // a knot object, 'start', or null
let R = null;            // the last replay
let replays = 0;
let cursor = null;       // seconds, or null
let playing = false, playT0 = 0;
const reduced = matchMedia('(prefers-reduced-motion: reduce)');

// ---- wasm ------------------------------------------------------------------
let K, h, dv, sBuf, kBuf, aBuf;

async function boot() {
  let bytes;
  try {
    const res = await fetch('./kinetic2.wasm');
    if (!res.ok) throw new Error(res.status + ' ' + res.statusText);
    bytes = await res.arrayBuffer();
  } catch (e) {
    $('#version').textContent = 'kinetic2.wasm not found';
    status('kinetic2.wasm did not load (' + e.message + '). Build it and serve this folder: see playground/README.md.');
    return;
  }
  const { instance } = await WebAssembly.instantiate(bytes, {});
  K = instance.exports;
  if (K.kinetic2_submit.length !== 8) {
    $('#version').textContent = 'kinetic2.wasm is out of date';
    status(`kinetic2.wasm predates this page: kinetic2_submit takes ${K.kinetic2_submit.length} arguments, 8 expected. Rebuild the kinetic2_wasm target.`);
    K = null;
    return;
  }
  K._initialize();
  h = K.kinetic2_create();
  dv = new DataView(K.memory.buffer);   // fixed memory: ALLOW_MEMORY_GROWTH=0
  sBuf = K.malloc(24); kBuf = K.malloc(40); aBuf = K.malloc(24);
  $('#version').textContent = cstr(K.kinetic2_version());
  buildUi();
  if (!loadHash()) preset('stroke', false);
  syncUi();
  render();
}

function cstr(ptr) {
  const m = new Uint8Array(K.memory.buffer);
  let s = '';
  for (let i = ptr; m[i]; i++) s += String.fromCharCode(m[i]);
  return s;
}

// ---- replay ------------------------------------------------------------------
// The engine erases a dropped knot from the timeline mid-window, so
// kinetic2_solved never reports dropped = 1: drops are read from PlanFailed
// (its target is the knot's authored p), retirements from the front.
function replay() {
  const o = S.o;
  const cfgOk = K.kinetic2_configure(h, o.vmax, o.amax, o.jmax, o.policy === 'blend' ? 5 : 0, o.floor,
    Math.round(o.look * 1000), o.corner === 'cubic' ? 1 : 0, Math.round(o.react * 1000));
  K.kinetic2_reset(h, S.p0, 0);
  const ks = [...S.knots].sort((a, b) => a.t - b.t);
  const lat = S.mode === 'stream' ? S.lat / 1000 : Infinity;
  const ev = ks.map((k) => ({ k, at: Math.max(0, k.t - lat) }));
  const n = Math.round(S.T * 1000) + 1;
  const P = new Float32Array(n), V = new Float32Array(n), A = new Float32Array(n), J = new Float32Array(n);
  const stepP = new Uint8Array(n), stepV = new Uint8Array(n);
  const solved = new Map(), refused = new Set(), dropped = new Set(), live = [], anomalies = [], submits = [];
  let e = 0;
  const drain = () => {
    while (K.kinetic2_pop_anomaly(h, aBuf)) {
      const a = { t: dv.getFloat64(aBuf, true) / 1e6, target: dv.getFloat32(aBuf + 8, true),
        detail: dv.getFloat32(aBuf + 12, true), seq: dv.getUint16(aBuf + 16, true), kind: dv.getUint8(aBuf + 18) };
      anomalies.push(a);
      if (a.kind === 1) {
        const i = live.findIndex((k) => Math.fround(k.p) === a.target);
        if (i >= 0) { dropped.add(live[i]); live.splice(i, 1); }
      }
    }
  };
  const snap = () => {
    K.kinetic2_solved(h, 0, kBuf);   // solves the window if it changed
    drain();
    const m = K.kinetic2_pending(h);
    while (live.length > m) live.shift();
    for (let i = 0; i < m; i++) {
      if (!K.kinetic2_solved(h, i, kBuf)) break;
      solved.set(live[i], {
        t: dv.getFloat64(kBuf, true) / 1e6, p: dv.getFloat32(kBuf + 8, true), v: dv.getFloat32(kBuf + 12, true),
        a: dv.getFloat32(kBuf + 16, true), share: dv.getFloat32(kBuf + 20, true), stretched: dv.getFloat32(kBuf + 24, true),
        worst: dv.getFloat32(kBuf + 28, true), dropped: dv.getUint8(kBuf + 32), clamped: dv.getUint8(kBuf + 33),
        pin_v: dv.getUint8(kBuf + 34), pin_a: dv.getUint8(kBuf + 35) });
    }
  };
  for (let i = 0; i < n; i++) {
    const tus = i * 1000;
    let submitted = false;
    while (e < ev.length && Math.round(ev[e].at * 1e6) <= tus) {
      const { k, at } = ev[e++];
      const now = Math.round(at * 1e6);
      // A knot marked `jog` is a live jog: a C1 sample at rest is the HARD junction.
      const flags = S.mode === 'stream' || k.jog ? KNOT_SAMPLE : k.v == null ? KNOT_REST_IF_LAST : 0;
      const ok = K.kinetic2_submit(h, Math.round(k.t * 1e6), k.p, k.v == null ? 0 : 1, k.v ?? 0, k.fam, now, flags);
      submits.push({ k, at: now / 1e6, ok });
      drain();
      if (ok) live.push(k); else refused.add(k);
      submitted = true;
    }
    if (submitted) snap();
    K.kinetic2_sample(h, tus, sBuf);
    P[i] = dv.getFloat32(sBuf + 8, true);
    V[i] = dv.getFloat32(sBuf + 12, true);
    A[i] = dv.getFloat32(sBuf + 16, true);
    J[i] = i ? (A[i] - A[i - 1]) * 1000 : 0;
    drain();
    if (K.kinetic2_pending(h) !== live.length) snap();
  }
  // A step: the sampled p (or v) moved by more than the trapezoid of the
  // sampled v (or a) explains over one millisecond.
  const tolP = 1e-5, tolV = 1e-5 + o.jmax * 1e-6;
  let mv = 0, ma = 0, mj = 0, pmin = Infinity, pmax = -Infinity, over = 0, steps = 0, firstStep = null;
  for (let i = 0; i < n; i++) {
    mv = Math.max(mv, Math.abs(V[i])); ma = Math.max(ma, Math.abs(A[i])); mj = Math.max(mj, Math.abs(J[i]));
    pmin = Math.min(pmin, P[i]); pmax = Math.max(pmax, P[i]);
    if (Math.abs(V[i]) > o.vmax * (1 + TOL) || Math.abs(A[i]) > o.amax * (1 + TOL) || Math.abs(J[i]) > o.jmax * (1 + TOL)) over++;
    if (!i) continue;
    stepP[i] = Math.abs(P[i] - P[i - 1] - 0.0005 * (V[i] + V[i - 1])) > tolP ? 1 : 0;
    stepV[i] = Math.abs(V[i] - V[i - 1] - 0.0005 * (A[i] + A[i - 1])) > tolV ? 1 : 0;
    if (stepP[i] || stepV[i]) { steps++; if (firstStep == null) firstStep = i / 1000; }
  }
  replays++;
  return { n, P, V, A, J, stepP, stepV, solved, refused, dropped, anomalies, submits, cfgOk,
    max: { v: mv, a: ma, j: mj }, pmin, pmax, over, steps, firstStep };
}

// ---- layout ------------------------------------------------------------------
const plot = $('#plot'), overlay = $('#overlay'), rail = $('#rail');
let L = null;   // panel geometry from the last draw

function fit(cv) {
  const r = cv.getBoundingClientRect(), d = devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * d)), hh = Math.max(1, Math.round(r.height * d));
  if (cv.width !== w || cv.height !== hh) { cv.width = w; cv.height = hh; }
  const g = cv.getContext('2d');
  g.setTransform(d, 0, 0, d, 0, 0);
  return { g, w: r.width, h: r.height };
}

function layout(w, hh) {
  const left = 58, right = 10, top = 6, bottom = 24, gap = 12;
  const ph = (hh - top - bottom - gap * 3) / 4;
  const x0 = left, x1 = w - right;
  const o = S.o, m = R.max;
  const sym = (ceil, seen) => Math.max(ceil * 1.18, seen * 1.06);
  const defs = [
    { key: 'P', name: 'position', unit: 'window', lo: -0.06, hi: 1.06, ceil: null },
    { key: 'V', name: 'velocity', unit: 'u/s', ceil: o.vmax, r: sym(o.vmax, m.v) },
    { key: 'A', name: 'acceleration', unit: 'u/s²', ceil: o.amax, r: sym(o.amax, m.a) },
    { key: 'J', name: 'jerk', unit: 'u/s³', ceil: o.jmax, r: sym(o.jmax, m.j) },
  ];
  const panels = defs.map((d, i) => {
    const y0 = top + i * (ph + gap), y1 = y0 + ph;
    const lo = d.r ? -d.r : d.lo, hi = d.r ? d.r : d.hi;
    return { ...d, x0, x1, y0, y1, lo, hi, y: (v) => y1 - (v - lo) / (hi - lo) * (y1 - y0) };
  });
  const tx = (t) => x0 + (t / S.T) * (x1 - x0);
  const xt = (x) => (x - x0) / (x1 - x0) * S.T;
  return { panels, x0, x1, tx, xt, w, h: hh, bottom: hh - bottom };
}

const fmt = (v) => {
  const a = Math.abs(v);
  if (a >= 1000) return (v / 1000).toFixed(a >= 10000 ? 0 : 1) + 'k';
  if (a >= 10) return v.toFixed(0);
  if (a >= 1) return v.toFixed(1);
  return v.toFixed(2);
};

// ---- drawing -----------------------------------------------------------------
function draw() {
  const { g, w, h: hh } = fit(plot);
  L = layout(w, hh);
  g.clearRect(0, 0, w, hh);
  const step = [0.05, 0.1, 0.25, 0.5, 1, 2, 2.5, 5]
    .find((q) => Math.abs(S.T / q - Math.round(S.T / q)) < 1e-9 && (q / S.T) * (L.x1 - L.x0) >= 56) || S.T;
  g.font = '11px ' + MONO;
  for (const pn of L.panels) {
    g.fillStyle = C.sunken;
    g.fillRect(pn.x0, pn.y0, pn.x1 - pn.x0, pn.y1 - pn.y0);
    g.strokeStyle = C.l1; g.lineWidth = 1;
    for (let t = step; t < S.T - 1e-9; t += step) {
      const x = Math.round(L.tx(t)) + 0.5;
      g.beginPath(); g.moveTo(x, pn.y0); g.lineTo(x, pn.y1); g.stroke();
    }
    g.strokeStyle = C.l3;
    g.strokeRect(pn.x0 + 0.5, pn.y0 + 0.5, pn.x1 - pn.x0 - 1, pn.y1 - pn.y0 - 1);
    const ticks = pn.key === 'P' ? [0, 0.5, 1] : Math.abs(pn.y(pn.ceil) - pn.y(0)) >= 14 ? [-pn.ceil, 0, pn.ceil] : [0];
    g.textAlign = 'right'; g.textBaseline = 'middle';
    for (const v of ticks) {
      const y = Math.round(pn.y(v)) + 0.5;
      const isCeil = pn.key === 'P' ? v !== 0.5 : v !== 0;
      g.strokeStyle = isCeil ? C.l4 : C.l2;
      g.setLineDash(isCeil ? [4, 4] : []);
      g.beginPath(); g.moveTo(pn.x0, y); g.lineTo(pn.x1, y); g.stroke();
      g.setLineDash([]);
      g.fillStyle = isCeil ? C.t3 : C.muted;
      g.fillText(pn.key === 'P' ? String(v) : (v > 0 ? '+' : '') + fmt(v), pn.x0 - 6, y);
    }
    if (pn.ceil && pn.hi > pn.ceil * 1.6) {
      g.fillStyle = C.hi;
      g.fillText('+' + fmt(pn.hi), pn.x0 - 6, pn.y0 + 7);
      g.fillText('-' + fmt(pn.hi), pn.x0 - 6, pn.y1 - 7);
    }
    g.save();
    g.beginPath(); g.rect(pn.x0, pn.y0, pn.x1 - pn.x0, pn.y1 - pn.y0); g.clip();
    trace(g, pn);
    if (pn.key === 'P') knots(g, pn);
    g.restore();
    g.textAlign = 'left'; g.textBaseline = 'top'; g.font = '11px ' + SANS;
    const label = pn.name + '  ' + pn.unit;
    const lw = g.measureText(label).width;
    g.fillStyle = 'rgba(4,5,7,0.8)'; g.fillRect(pn.x0 + 1, pn.y0 + 1, lw + 12, 17);
    g.fillStyle = C.t2; g.fillText(label, pn.x0 + 6, pn.y0 + 4);
    g.font = '11px ' + MONO;
  }
  g.fillStyle = C.t3; g.textBaseline = 'top';
  for (let t = 0; t <= S.T + 1e-9; t += step) {
    const x = L.tx(t);
    g.textAlign = t === 0 ? 'left' : t >= S.T - 1e-9 ? 'right' : 'center';
    g.fillText(+t.toFixed(2) + (t >= S.T - 1e-9 ? ' s' : ''), x, L.bottom + 6);
  }
  drawOverlay();
}

function trace(g, pn) {
  const arr = R[pn.key], n = R.n, ceil = pn.ceil;
  const lim = ceil ? ceil * (1 + TOL) : null;
  const bad = (v) => (lim ? Math.abs(v) > lim : v < -1e-4 || v > 1 + 1e-4);
  const step = pn.key === 'P' ? R.stepP : pn.key === 'V' ? R.stepV : null;
  g.lineWidth = 1.5; g.lineJoin = 'round';
  g.strokeStyle = C.blue;
  g.beginPath();
  // ponytail: one vertex per ms; decimate per pixel column if 10 s spans get slow
  for (let i = 0; i < n; i++) {
    const x = L.tx(i / 1000), y = pn.y(arr[i]);
    i ? g.lineTo(x, y) : g.moveTo(x, y);
  }
  g.stroke();
  g.strokeStyle = C.hi; g.lineWidth = 2.5;
  g.beginPath();
  for (let i = 1; i < n; i++) {
    if (!bad(arr[i]) && !bad(arr[i - 1]) && !(step && step[i])) continue;
    g.moveTo(L.tx((i - 1) / 1000), pn.y(arr[i - 1]));
    g.lineTo(L.tx(i / 1000), pn.y(arr[i]));
  }
  g.stroke();
}

function mark(g, shape, x, y, r, fill, stroke, lw = 1.5) {
  g.beginPath();
  if (shape === 'diamond') { g.moveTo(x, y - r - 1); g.lineTo(x + r + 1, y); g.lineTo(x, y + r + 1); g.lineTo(x - r - 1, y); g.closePath(); }
  else if (shape === 'square') g.rect(x - r, y - r, r * 2, r * 2);
  else g.arc(x, y, r, 0, Math.PI * 2);
  if (fill) { g.fillStyle = fill; g.fill(); }
  if (stroke) { g.strokeStyle = stroke; g.lineWidth = lw; g.stroke(); }
}

function arrow(g, x0, y0, x1, y1, color) {
  g.strokeStyle = color; g.fillStyle = color; g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1); g.stroke();
  const a = Math.atan2(y1 - y0, x1 - x0), s = 6;
  x1 -= 5 * Math.cos(a); y1 -= 5 * Math.sin(a);
  g.beginPath(); g.moveTo(x1, y1);
  g.lineTo(x1 - s * Math.cos(a - 0.45), y1 - s * Math.sin(a - 0.45));
  g.lineTo(x1 - s * Math.cos(a + 0.45), y1 - s * Math.sin(a + 0.45));
  g.closePath(); g.fill();
}

function knots(g, pn) {
  const sx = L.tx(0) + 5, sy = pn.y(S.p0);
  mark(g, 'square', sx, sy, 4.5, C.violet, sel === 'start' ? C.text : null);
  for (const k of S.knots) {
    if (k.t > S.T + 1e-9) continue;
    const x = L.tx(k.t), y = pn.y(k.p), shape = k.fam === 1 ? 'diamond' : 'circle';
    const s = R.solved.get(k);
    if (R.refused.has(k) || R.dropped.has(k) || (s && s.dropped)) {
      mark(g, shape, x, y, 6, null, C.hi);
      g.strokeStyle = C.hi; g.lineWidth = 1.5;
      g.beginPath(); g.moveTo(x - 3.5, y - 3.5); g.lineTo(x + 3.5, y + 3.5); g.moveTo(x + 3.5, y - 3.5); g.lineTo(x - 3.5, y + 3.5); g.stroke();
    } else if (s) {
      const sxk = L.tx(s.t), syk = pn.y(s.p);
      const trimmed = s.share < 0.999, stretched = s.stretched > 1e-6;
      if (trimmed || stretched) {
        if (stretched) arrow(g, x, y, sxk, syk, SPEND);
        else { g.strokeStyle = SPEND; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x, y); g.lineTo(sxk, syk); g.stroke(); }
        mark(g, shape, x, y, 5.5, C.sunken, C.violet);
        mark(g, shape, sxk, syk, 4, C.blue, null);
      } else {
        mark(g, shape, x, y, 4.5, C.violet, null);
      }
      if (s.clamped) {
        g.fillStyle = C.hi; g.fillRect(x + 6, y - 15, 9, 10);
        g.fillStyle = C.sunken; g.font = 'bold 9px ' + MONO; g.textAlign = 'center'; g.textBaseline = 'middle';
        g.fillText('v', x + 10.5, y - 10);
      }
    } else {
      mark(g, shape, x, y, 4.5, C.violet, null);
    }
    if (k === sel) mark(g, 'circle', x, y, 9, null, C.text, 1);
  }
}

function sampleAt(t) {
  const i = Math.max(0, Math.min(R.n - 1, Math.round(t * 1000)));
  return { i, t: i / 1000, p: R.P[i], v: R.V[i], a: R.A[i], j: R.J[i] };
}

function drawOverlay() {
  const { g, w, h: hh } = fit(overlay);
  g.clearRect(0, 0, w, hh);
  const s = cursor == null ? null : sampleAt(cursor);
  if (!s) $('#readout').textContent = 'hover the plot to read values';
  if (s && L) {
    const x = Math.round(L.tx(s.t)) + 0.5;
    g.strokeStyle = C.t3; g.lineWidth = 1;
    for (const pn of L.panels) {
      g.beginPath(); g.moveTo(x, pn.y0); g.lineTo(x, pn.y1); g.stroke();
      const v = R[pn.key][s.i];
      mark(g, 'circle', x, Math.max(pn.y0, Math.min(pn.y1, pn.y(v))), 3, C.text, null);
    }
    $('#readout').textContent = `t ${s.t.toFixed(3)} s   p ${s.p.toFixed(4)}   v ${s.v.toFixed(3)} u/s   a ${s.a.toFixed(2)} u/s²   j ${s.j.toFixed(1)} u/s³`;
  }
  drawRail(s);
}

function drawRail(s) {
  const { g, w, h: hh } = fit(rail);
  g.clearRect(0, 0, w, hh);
  const top = L ? L.panels[0].y0 : 6, bot = L ? L.bottom : hh - 24;
  const cx = Math.round(w / 2) - 6;
  const y = (p) => bot - p * (bot - top);
  g.fillStyle = C.sunken; g.fillRect(cx - 4, top, 8, bot - top);
  g.strokeStyle = C.l3; g.strokeRect(cx - 3.5, top + 0.5, 7, bot - top - 1);
  g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'center';
  g.textBaseline = 'bottom'; g.fillText('1', cx, top - 1 < 10 ? top + 12 : top - 1);
  g.textBaseline = 'top'; g.fillText('0', cx, bot + 4);
  const p = s ? s.p : S.p0, v = s ? s.v : 0;
  const yc = y(Math.max(-0.05, Math.min(1.05, p)));
  g.fillStyle = C.blue; g.fillRect(cx - 12, yc - 5, 24, 10);
  g.fillStyle = C.sunken; g.fillRect(cx - 12, yc - 0.5, 24, 1);
  const bx = cx + 18, vlen = Math.max(-1, Math.min(1, v / S.o.vmax)) * 0.25 * (bot - top);
  g.fillStyle = C.l2; g.fillRect(bx - 1, y(0.5) - 0.25 * (bot - top), 2, 0.5 * (bot - top));
  g.fillStyle = Math.abs(v) > S.o.vmax * (1 + TOL) ? C.hi : C.violet;
  g.fillRect(bx - 2, Math.min(y(0.5), y(0.5) - vlen), 4, Math.abs(vlen));
}

// ---- interaction -------------------------------------------------------------
let drag = null, raf = 0;
function schedule() { if (!raf) raf = requestAnimationFrame(() => { raf = 0; render(); }); }

function hit(x, y) {
  const pn = L.panels[0];
  if (Math.hypot(x - (L.tx(0) + 5), y - pn.y(S.p0)) < 10) return 'start';
  let best = null, bd = 10;
  for (const k of S.knots) {
    const d = Math.hypot(x - L.tx(k.t), y - pn.y(k.p));
    if (d < bd) { bd = d; best = k; }
  }
  return best;
}

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const snapTo = (v, q) => Math.round(v / q) * q;

function pointerPos(ev) { const r = overlay.getBoundingClientRect(); return { x: ev.clientX - r.left, y: ev.clientY - r.top }; }
function inPos(x, y) { const pn = L.panels[0]; return x >= pn.x0 - 12 && x <= pn.x1 && y >= pn.y0 && y <= pn.y1; }

overlay.addEventListener('pointerdown', (ev) => {
  if (!L || ev.button !== 0) return;
  const { x, y } = pointerPos(ev);
  if (!inPos(x, y)) return;
  const pn = L.panels[0];
  let k = hit(x, y);
  if (!k) {
    let t = clamp(L.xt(x), 0.001, S.T), p = clamp((pn.y1 - y) / (pn.y1 - pn.y0) * (pn.hi - pn.lo) + pn.lo, 0, 1);
    if (ev.ctrlKey) { t = Math.max(0.05, snapTo(t, 0.05)); p = snapTo(p, 0.05); }
    k = { t: +t.toFixed(6), p: +p.toFixed(5), fam: 2, v: null };
    S.knots.push(k);
  }
  sel = k;
  drag = { k, x, y, raw: k === 'start' ? { t: 0, p: S.p0 } : { t: k.t, p: k.p } };
  overlay.setPointerCapture(ev.pointerId);
  changed();
});

overlay.addEventListener('pointermove', (ev) => {
  if (!L) return;
  const { x, y } = pointerPos(ev);
  if (!drag) {
    if (!playing) { cursor = x >= L.x0 && x <= L.x1 ? clamp(L.xt(x), 0, S.T) : cursor; drawOverlay(); }
    overlay.style.cursor = inPos(x, y) && hit(x, y) ? 'grab' : 'crosshair';
    return;
  }
  const pn = L.panels[0], gain = ev.shiftKey ? 0.1 : 1;
  drag.raw.t += (x - drag.x) / (L.x1 - L.x0) * S.T * gain;
  drag.raw.p -= (y - drag.y) / (pn.y1 - pn.y0) * (pn.hi - pn.lo) * gain;
  drag.x = x; drag.y = y;
  let t = drag.raw.t, p = drag.raw.p;
  if (ev.ctrlKey) { t = snapTo(t, 0.05); p = snapTo(p, 0.05); }
  if (drag.k === 'start') S.p0 = +clamp(p, 0, 1).toFixed(5);
  else { drag.k.t = +clamp(t, 0.001, S.T).toFixed(6); drag.k.p = +clamp(p, 0, 1).toFixed(5); }
  changed();
});

const endDrag = () => { drag = null; };
overlay.addEventListener('pointerup', endDrag);
overlay.addEventListener('pointercancel', endDrag);
overlay.addEventListener('pointerleave', () => { if (!drag && !playing) { cursor = null; drawOverlay(); } });
overlay.addEventListener('contextmenu', (ev) => {
  ev.preventDefault();
  const { x, y } = pointerPos(ev);
  const k = L && hit(x, y);
  if (k && k !== 'start') remove(k);
});

function remove(k) {
  S.knots = S.knots.filter((q) => q !== k);
  if (sel === k) sel = null;
  changed();
}

addEventListener('keydown', (ev) => {
  const tag = ev.target.tagName;
  const typing = tag === 'INPUT' && ev.target.type !== 'range' || tag === 'SELECT' || tag === 'TEXTAREA';
  if (typing) return;
  if (ev.code === 'Space') { ev.preventDefault(); togglePlay(); }
  else if ((ev.key === 'Delete' || ev.key === 'Backspace') && sel && sel !== 'start') { ev.preventDefault(); remove(sel); }
  else if (ev.key === 'Escape') { sel = null; changed(); }
});

// ---- play ------------------------------------------------------------------
function togglePlay() {
  if (reduced.matches) return;
  playing = !playing;
  $('#play').textContent = playing ? 'Pause' : 'Play';
  if (playing) { playT0 = performance.now() - (cursor ?? 0) * 1000; requestAnimationFrame(tick); }
}
function tick(now) {
  if (!playing) return;
  cursor = ((now - playT0) / 1000) % S.T;
  drawOverlay();
  requestAnimationFrame(tick);
}
function motionPref() {
  const b = $('#play');
  b.disabled = reduced.matches;
  b.title = reduced.matches ? 'Reduced motion is on: hover the plot to read values' : 'Play / pause (Space)';
  if (reduced.matches) { playing = false; b.textContent = 'Play'; }
}
reduced.addEventListener('change', motionPref);

// ---- presets ---------------------------------------------------------------
const PRESETS = {
  stroke: { label: 'Stroke', title: '0.2 to 0.8 at 0.5 s, back to 0.2 at 1.0 s, a C1 stop at the end', make: () => ({
    T: 2, mode: 'ahead', p0: 0.2, knots: [{ t: 0.5, p: 0.8, fam: 2, v: null }, { t: 1.0, p: 0.2, fam: 1, v: 0 }] }) },
  staircase: { label: 'Staircase', title: 'Five steps of 0.15 every 300 ms, each a C1 knot at rest', make: () => ({
    T: 2, mode: 'ahead', p0: 0.1, knots: [1, 2, 3, 4, 5].map((i) => ({ t: 0.3 * i, p: +(0.1 + 0.15 * i).toFixed(2), fam: 1, v: 0 })) }) },
  scrub: { label: 'Scrub', title: 'A 60 Hz stream of a 1 Hz sine, streamed at 16 ms latency. Capped at 60 knots (1 s): the ABI holds 64 pending knots', make: () => ({
    T: 2, mode: 'stream', lat: 16, p0: 0.15, knots: Array.from({ length: 60 }, (_, i) => {
      const t = (i + 1) / 60;
      return { t: +t.toFixed(6), p: +(0.5 - 0.35 * Math.cos(2 * Math.PI * t)).toFixed(5), fam: 2, v: null };
    }) }) },
  hardstop: { label: 'Hard stop', title: 'A live jog to 0.5 (a C1 sample at rest: the fastest move, braked onto it), then onward', make: () => ({
    T: 2, mode: 'ahead', p0: 0.1, knots: [{ t: 0.6, p: 0.5, fam: 1, v: 0, jog: true }, { t: 1.2, p: 0.9, fam: 2, v: null }] }) },
  overreach: { label: 'Overreach', title: '0.05 to 0.95 in 150 ms: Blend trims it, Stretch moves it', make: () => ({
    T: 1, mode: 'ahead', p0: 0.05, knots: [{ t: 0.15, p: 0.95, fam: 2, v: null }] }) },
  starved: { label: 'Starved stream', title: 'Streamed, a knot every 100 ms for 0.5 s, then nothing: the brake engages', make: () => ({
    T: 1, mode: 'stream', lat: 100, p0: 0.2, knots: [1, 2, 3, 4, 5].map((i) => ({ t: 0.1 * i, p: +(0.2 + 0.1 * i).toFixed(2), fam: 2, v: null })) }) },
};

function preset(name, render_ = true) {
  const p = PRESETS[name].make();
  S = { ...S, lat: p.lat ?? S.lat, T: p.T, mode: p.mode, p0: p.p0, knots: p.knots };
  sel = null; cursor = null;
  if (render_) { syncUi(); changed(); }
}

// ---- share -----------------------------------------------------------------
let hashTimer = 0;
function saveHash() {
  clearTimeout(hashTimer);
  hashTimer = setTimeout(() => {
    const o = S.o;
    const j = JSON.stringify({ v: 1, T: S.T, m: S.mode, l: S.lat, p0: S.p0,
      o: [o.vmax, o.amax, o.jmax, o.policy, o.floor, o.corner, o.react, o.look],
      k: S.knots.map((k) => (k.jog ? [k.t, k.p, k.fam, k.v, 1] : [k.t, k.p, k.fam, k.v])) });
    const b = btoa(j).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
    history.replaceState(null, '', '#' + b);
  }, 250);
}

function loadHash() {
  if (location.hash.length < 2) return false;
  try {
    const j = JSON.parse(atob(location.hash.slice(1).replace(/-/g, '+').replace(/_/g, '/')));
    const num = (v, lo, hi, d) => (Number.isFinite(v) ? clamp(v, lo, hi) : d);
    const o = Array.isArray(j.o) ? j.o : [];
    S = {
      T: SPANS.includes(j.T) ? j.T : 2,
      mode: j.m === 'stream' ? 'stream' : 'ahead',
      lat: num(j.l, 0, 500, 100),
      p0: num(j.p0, 0, 1, 0.5),
      o: { vmax: num(o[0], 0.1, 30, DEF.vmax), amax: num(o[1], 1, 300, DEF.amax), jmax: num(o[2], 10, 10000, DEF.jmax),
        policy: o[3] === 'stretch' ? 'stretch' : 'blend', floor: num(o[4], 0, 1, DEF.floor), corner: o[5] === 'cubic' ? 'cubic' : 'continuous',
        react: num(o[6], 0, 50, DEF.react), look: num(o[7], 50, 1000, DEF.look) },
      knots: (Array.isArray(j.k) ? j.k : []).slice(0, 128).filter((k) => Array.isArray(k) && Number.isFinite(k[0]) && Number.isFinite(k[1]))
        .map((k) => ({ t: clamp(k[0], 0.001, 10), p: clamp(k[1], 0, 1), fam: k[2] === 1 ? 1 : 2, v: Number.isFinite(k[3]) ? clamp(k[3], -100, 100) : null,
                       ...(k[4] === 1 ? { jog: true } : {}) })),
    };
    return true;
  } catch {
    status('The link did not decode; loaded the Stroke preset instead.');
    return false;
  }
}

// ---- controls ----------------------------------------------------------------
const OPTS = [
  { key: 'vmax', label: 'vmax', unit: 'u/s', log: [0.1, 30] },
  { key: 'amax', label: 'amax', unit: 'u/s²', log: [1, 300] },
  { key: 'jmax', label: 'jmax', unit: 'u/s³', log: [10, 10000] },
  { key: 'policy', label: 'policy', choices: [['blend', 'Blend'], ['stretch', 'Stretch']] },
  { key: 'floor', label: 'amplitude floor', lin: [0, 1, 0.01] },
  { key: 'corner', label: 'corner', choices: [['continuous', 'Continuous'], ['cubic', 'Cubic']] },
  { key: 'react', label: 'reaction horizon', unit: 'ms', lin: [0, 50, 1] },
  { key: 'look', label: 'lookahead', unit: 'ms', lin: [50, 1000, 10] },
];
const syncers = [];

function seg(el, items, get, set) {
  for (const [val, text, title] of items) {
    const b = document.createElement('button');
    b.textContent = text; if (title) b.title = title;
    b.addEventListener('click', () => { set(val); syncUi(); changed(); });
    el.append(b);
    syncers.push(() => b.setAttribute('aria-pressed', String(get() === val)));
  }
}

function buildUi() {
  seg($('#span'), SPANS.map((t) => [t, t + ' s']), () => S.T, (v) => { S.T = v; });
  seg($('#mode'), [['ahead', 'Authored ahead', 'Every knot submitted at time 0'], ['stream', 'Streamed', 'Each knot submitted latency ahead of its time']],
    () => S.mode, (v) => { S.mode = v; });
  const lat = $('#lat');
  lat.addEventListener('input', () => { S.lat = +lat.value; syncUi(); changed(); });
  syncers.push(() => { lat.value = S.lat; $('#latout').textContent = S.lat + ' ms'; $('#latwrap').hidden = S.mode !== 'stream'; });

  for (const [name, p] of Object.entries(PRESETS)) {
    const b = document.createElement('button');
    b.className = 'btn'; b.textContent = p.label; b.title = p.title; b.dataset.preset = name;
    b.addEventListener('click', () => preset(name));
    $('#presets').append(b);
  }

  const box = $('#opts');
  for (const d of OPTS) {
    const wrap = document.createElement('div'); wrap.className = 'opt';
    const head = document.createElement('div'); head.className = 'head';
    head.innerHTML = `<span>${d.label}</span><span class="muted">${d.unit || ''}</span>`;
    wrap.append(head);
    if (d.choices) {
      const s = document.createElement('div'); s.className = 'seg';
      seg(s, d.choices, () => S.o[d.key], (v) => { S.o[d.key] = v; });
      wrap.append(s);
    } else {
      const ctl = document.createElement('div'); ctl.className = 'ctl';
      const r = document.createElement('input'); r.type = 'range';
      const nf = document.createElement('input'); nf.type = 'number';
      r.setAttribute('aria-label', d.label); nf.setAttribute('aria-label', d.label + ' value');
      let toR, fromR;
      if (d.log) {
        const [a, b] = d.log;
        r.min = 0; r.max = 1000; r.step = 1;
        nf.min = a; nf.max = b; nf.step = 'any';
        toR = (v) => Math.round(Math.log(v / a) / Math.log(b / a) * 1000);
        fromR = (x) => +(a * Math.pow(b / a, x / 1000)).toPrecision(3);
      } else {
        const [a, b, st] = d.lin;
        r.min = nf.min = a; r.max = nf.max = b; r.step = nf.step = st;
        toR = (v) => v; fromR = (x) => +x;
      }
      const [lo, hi] = d.log || d.lin;
      r.addEventListener('input', () => { S.o[d.key] = fromR(+r.value); nf.value = S.o[d.key]; changed(); });
      nf.addEventListener('change', () => {
        const v = +nf.value;
        if (Number.isFinite(v)) S.o[d.key] = clamp(v, lo, hi);
        syncUi(); changed();
      });
      syncers.push(() => { r.value = toR(S.o[d.key]); if (document.activeElement !== nf) nf.value = S.o[d.key]; });
      ctl.append(r, nf); wrap.append(ctl);
    }
    box.append(wrap);
  }
  $('#defaults').addEventListener('click', () => { S.o = { ...DEF }; syncUi(); changed(); });

  $('#play').addEventListener('click', togglePlay);
  $('#copy').addEventListener('click', async () => {
    clearTimeout(hashTimer); saveHash();
    setTimeout(async () => {
      try { await navigator.clipboard.writeText(location.href); status('Link copied.', true); }
      catch { status('Copy failed; the link is in the address bar.'); }
    }, 300);
  });

  const num = (id, fn) => $(id).addEventListener('change', (ev) => { const v = +ev.target.value; if (Number.isFinite(v)) fn(v); changed(); });
  num('#i-p0', (v) => { S.p0 = clamp(v, 0, 1); });
  num('#i-t', (v) => { if (sel && sel !== 'start') sel.t = clamp(v, 0.001, 10); });
  num('#i-p', (v) => { if (sel && sel !== 'start') sel.p = clamp(v, 0, 1); });
  num('#i-v', (v) => { if (sel && sel !== 'start' && sel.v != null) sel.v = v; });
  $('#i-fam').addEventListener('change', (ev) => { sel.fam = +ev.target.value; changed(); });
  $('#i-vmode').addEventListener('change', (ev) => { sel.v = ev.target.value === 'set' ? 0 : null; changed(); });
  $('#i-del').addEventListener('click', () => sel && sel !== 'start' && remove(sel));

  $('#legend').innerHTML = [
    ['<rect x="3" y="3" width="8" height="8" fill="#A78BFA"/>', 'start (at rest)'],
    ['<circle cx="7" cy="7" r="4.5" fill="#A78BFA"/>', 'C2 knot'],
    ['<path d="M7 1.5 12.5 7 7 12.5 1.5 7Z" fill="#A78BFA"/>', 'C1 knot'],
    ['<circle cx="7" cy="7" r="5" fill="none" stroke="#A78BFA" stroke-width="1.5"/>', 'authored, moved by the spend'],
    ['<circle cx="7" cy="7" r="4" fill="#4DA6FF"/>', 'solved'],
    ['<line x1="7" y1="1" x2="7" y2="13" stroke="#4DA6FF" stroke-width="1.5"/>', 'Blend trim'],
    ['<path d="M1 7H10M8 3.5 12.5 7 8 10.5Z" stroke="#4DA6FF" stroke-width="1.5" fill="#4DA6FF"/>', 'Stretch'],
    ['<circle cx="7" cy="7" r="5.5" fill="none" stroke="#FF5CB3" stroke-width="1.5"/><path d="M4 4 10 10M10 4 4 10" stroke="#FF5CB3" stroke-width="1.5"/>', 'dropped or refused'],
    ['<rect x="2" y="2" width="10" height="10" fill="#FF5CB3"/><text x="7" y="10.5" font-size="9" text-anchor="middle" font-family="monospace" fill="#040507" font-weight="bold">v</text>', 'end velocity clamped'],
    ['<path d="M1 7H13" stroke="#33373E" stroke-dasharray="3 2" stroke-width="1.5"/>', 'ceiling'],
    ['<path d="M1 7H13" stroke="#FF5CB3" stroke-width="2.5"/>', 'over a ceiling by more than 0.1%, or a step between two 1 ms samples'],
  ].map(([svg, text]) => `<li><svg width="14" height="14" viewBox="0 0 14 14" aria-hidden="true">${svg}</svg>${text}</li>`).join('');

  new ResizeObserver(() => { if (R) draw(); }).observe($('#plotbox'));
  motionPref();
}

function syncUi() { for (const f of syncers) f(); }

// ---- inspector + anomalies -----------------------------------------------------
function inspector() {
  $('#insp-none').hidden = !!sel;
  $('#insp-start').hidden = sel !== 'start';
  $('#insp-knot').hidden = !sel || sel === 'start';
  const keep = (id, v) => { const el = $(id); if (document.activeElement !== el) el.value = v; };
  if (sel === 'start') keep('#i-p0', S.p0);
  if (!sel || sel === 'start') return;
  const k = sel;
  keep('#i-t', k.t); keep('#i-p', k.p);
  $('#i-fam').value = String(k.fam);
  $('#i-vmode').value = k.v == null ? 'free' : 'set';
  $('#i-vrow').hidden = k.v == null;
  if (k.v != null) keep('#i-v', k.v);
  $('#i-hint').textContent = k.v == null ? 'The solver picks the junction velocity.'
    : k.fam === 1 && k.v === 0 ? 'C1 with 0: a hard stop, the fastest legal brake lands here.'
    : k.fam === 1 ? 'C1: acceleration may step here (corner option).' : 'C2: velocity pinned, acceleration continuous.';
  const s = R.solved.get(k), dl = $('#i-solved');
  if (R.refused.has(k)) { dl.innerHTML = '<dt>status</dt><dd class="bad">refused (see anomalies)</dd>'; return; }
  if (R.dropped.has(k)) { dl.innerHTML = '<dt>status</dt><dd class="bad">dropped: PlanFailed (see anomalies)</dd>'; return; }
  if (!s) { dl.innerHTML = '<dt>status</dt><dd>not submitted in this span</dd>'; return; }
  const row = (a, b, bad) => `<dt>${a}</dt><dd${bad ? ' class="bad"' : ''}>${b}</dd>`;
  dl.innerHTML = row('t', (s.t).toFixed(4) + ' s') + row('p', s.p.toFixed(4)) + row('v', s.v.toFixed(3) + ' u/s') + row('a', s.a.toFixed(2) + ' u/s²')
    + row('share', (s.share * 100).toFixed(1) + '%', s.share < 0.999) + row('stretched', (s.stretched * 1000).toFixed(1) + ' ms', s.stretched > 1e-6)
    + row('worst', s.worst.toFixed(3)) + row('pinned', [s.pin_v && 'v', s.pin_a && 'a'].filter(Boolean).join(', ') || 'none')
    + row('dropped', s.dropped ? 'yes' : 'no', s.dropped) + row('clamped', s.clamped ? 'yes' : 'no', s.clamped);
}

function meaning(a) {
  const d = a.detail;
  const sentinel = SENTINEL[String(Math.round(d))];
  switch (a.kind) {
    case 1: return 'knot dropped: ' + (sentinel || (d >= 1e29 ? 'no legal piece under any spend (kIllegal)' : `worst ceiling ratio ${d.toFixed(2)} after every spend`));
    case 2: return `braked at ${Math.abs(d).toFixed(2)} u/s: the timeline ran dry`;
    case 3: return `end velocity cut to ${d.toFixed(2)} u/s`;
    case 4: return `deadline moved ${(d * 1000).toFixed(0)} ms later`;
    case 6: return `Blend kept ${(d * 100).toFixed(0)}% of the stroke`;
    case 10: return 'same target re-commanded: a hold, its end velocity dropped';
    case 11: return 'knot refused: ' + (sentinel || 'detail ' + d);
    default: return 'detail ' + d;
  }
}

function anomalies() {
  const rows = R.anomalies.slice(0, 200).map((a) => `<tr><td class="mono">${(a.t * 1000).toFixed(0)} ms</td><td class="k">${KIND[a.kind] || 'kind ' + a.kind}</td><td class="mono">${a.target.toFixed(3)}</td><td>${meaning(a)}</td></tr>`);
  $('#anoms tbody').innerHTML = rows.join('') || '<tr><td colspan="4" class="muted">None: every knot landed as authored.</td></tr>';
  const by = {};
  for (const a of R.anomalies) { const nm = KIND[a.kind] || 'kind ' + a.kind; by[nm] = (by[nm] || 0) + 1; }
  $('#ancount').textContent = R.anomalies.length ? `${R.anomalies.length}: ` + Object.entries(by).map(([k, v]) => `${k} ${v}`).join(', ') : '';
  if (R.anomalies.length > 200) $('#anoms tbody').insertAdjacentHTML('beforeend', `<tr><td colspan="4" class="muted">${R.anomalies.length - 200} more not shown</td></tr>`);
}

// ---- glue ------------------------------------------------------------------
function status(text, ok) { const el = $('#status'); el.textContent = text; el.style.color = ok ? 'var(--text-3)' : ''; }

function render() {
  S.knots.sort((a, b) => a.t - b.t);
  R = replay();
  if (!R.cfgOk) status('kinetic2_configure refused these options; the previous ones are in force.');
  else if ($('#status').textContent.startsWith('kinetic2_configure')) status('');
  if (cursor != null) cursor = Math.min(cursor, S.T);
  draw();
  inspector();
  anomalies();
  window.__k2 = {
    replays, samples: R.n, max: { ...R.max }, ceil: { v: S.o.vmax, a: S.o.amax, j: S.o.jmax }, over: R.over,
    p: { min: R.pmin, max: R.pmax }, steps: R.steps, firstStep: R.firstStep, dropped: R.dropped.size, refused: R.refused.size, anomalies: R.anomalies.map((a) => ({ ...a, name: KIND[a.kind] })),
    knots: S.knots.map((k) => ({ ...k, solved: R.solved.get(k) || null, refused: R.refused.has(k), dropped: R.dropped.has(k) })),
    start: S.p0, mode: S.mode, T: S.T, layout: L && { x0: L.x0, x1: L.x1, y0: L.panels[0].y0, y1: L.panels[0].y1, lo: L.panels[0].lo, hi: L.panels[0].hi },
  };
}

function changed() { if (!K) return; saveHash(); schedule(); }

boot();
