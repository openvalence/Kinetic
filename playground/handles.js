// handles.js -- the handles page (kin-d0o): draws the composite Bezier of
// handles-model.js, lets the operator set each knot's class, drag its handles
// (aligned, as in Blender) and move it, and shows velocity, acceleration and
// jerk against the ceilings with every nudge visible.
// Constraints:
// - Every change is a full render of the model; nothing is cached across edits.
// - Plain script, no module: the page must open from file://.
'use strict';
const M = window.HandlesModel;
const $ = (s) => document.querySelector(s);
const C = { bg: '#08090B', l2: '#1D2026', l3: '#272B31', text: '#ECEFF4', t3: '#A8AEB9', muted: '#666C78',
  blue: '#4DA6FF', violet: '#A78BFA', hi: '#FF5CB3', v: '#5BD48A', a: '#F2A94A', j: '#C9A2FF', red: '#FF5A5A' };
const MONO = 'ui-monospace, "SF Mono", Consolas, monospace';
const SANS = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const CLS = { end: C.muted, rest: C.t3, crest: C.text, through: C.blue };

let cfg = { ...M.DEF, rail: 500 };
let knots = [], R = null, sel = null, scriptName = '';
let view = { t0: 0, t1: 6 };
let drag = null;
let cursor = null, playing = false, playT0 = 0, raf = 0;
const plot = $('#plot'), strips = $('#strips'), overlay = $('#overlay'), soverlay = $('#soverlay'), rail = $('#rail');
const PAD = { l: 44, r: 12, t: 10, b: 20 };

// ---- model ----------------------------------------------------------------
function load(doc, label) {
  knots = M.fromFunscript(doc, cfg); scriptName = label; sel = null;
  $('#name').textContent = `${label}: ${knots.length} knots`;
  fitView(); update();
}
function update() {
  R = M.render(knots, cfg);
  draw(); inspector(); stats(); drawCursor();
}
function fitView() {
  if (!knots.length) return;
  const t1 = knots[knots.length - 1].t, pad = Math.max(0.05, t1 * 0.02);
  view = { t0: knots[0].t - pad, t1: t1 + pad };
}

// ---- geometry ---------------------------------------------------------------
function dims(cv) {
  const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  if (cv.width !== Math.round(r.width * dpr) || cv.height !== Math.round(r.height * dpr)) { cv.width = Math.round(r.width * dpr); cv.height = Math.round(r.height * dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: r.width, h: r.height };
}
const X = (t, w) => PAD.l + (t - view.t0) / (view.t1 - view.t0) * (w - PAD.l - PAD.r);
const T = (x, w) => view.t0 + (x - PAD.l) / (w - PAD.l - PAD.r) * (view.t1 - view.t0);
const Y = (p, h) => PAD.t + (cfg.hi - p) / (cfg.hi - cfg.lo) * (h - PAD.t - PAD.b);
const P = (y, h) => cfg.hi - (y - PAD.t) / (h - PAD.t - PAD.b) * (cfg.hi - cfg.lo);

// the two handle ends of a knot, in (s, mm)
function handleEnds(i) {
  const k = knots[i], prev = knots[i - 1], next = knots[i + 1], p = M.pos(k) + k.dp, out = {};
  if (prev) { const TL = k.t - prev.t; out.in = [k.t - k.effIn * TL, p - k.vIn * k.effIn * TL]; }
  if (next) { const TR = next.t - k.t; out.out = [k.t + k.effOut * TR, p + k.vOut * k.effOut * TR]; }
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

// ---- drawing ----------------------------------------------------------------
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
  if (!R) return;
  ctx.save(); ctx.beginPath(); ctx.rect(PAD.l, 0, w - PAD.l - PAD.r, h); ctx.clip();
  // the script as authored, faint
  ctx.strokeStyle = C.l3; ctx.setLineDash([3, 4]); ctx.lineWidth = 1; ctx.beginPath();
  knots.forEach((k, i) => (i ? ctx.lineTo(X(k.t, w), Y(k.p, h)) : ctx.moveTo(X(k.t, w), Y(k.p, h)))); ctx.stroke(); ctx.setLineDash([]);
  // the curve, red where a sample is over a ceiling
  for (const pc of R.pieces) {
    if (pc.t[pc.t.length - 1] < view.t0 || pc.t[0] > view.t1) continue;
    ctx.lineWidth = 2;
    for (let n = 1; n < pc.t.length; n++) {
      const bad = Math.abs(pc.v[n]) > cfg.vmax * 1.001 || Math.abs(pc.a[n]) > cfg.amax * 1.001 || Math.abs(pc.j[n]) > cfg.jmax * 1.001;
      ctx.strokeStyle = bad ? C.red : (knots[pc.i + 1].infeasible ? C.hi : C.blue);
      ctx.beginPath(); ctx.moveTo(X(pc.t[n - 1], w), Y(pc.p[n - 1], h)); ctx.lineTo(X(pc.t[n], w), Y(pc.p[n], h)); ctx.stroke();
    }
  }
  // handles
  const vis = visibleHandles();
  for (const i of vis) {
    const k = knots[i], e = handleEnds(i), kx = X(k.t, w), ky = Y(M.pos(k) + k.dp, h);
    for (const side of ['in', 'out']) {
      if (!e[side]) continue;
      const nudged = Math.abs((side === 'in' ? k.effIn - k.lIn : k.effOut - k.lOut)) > 1e-6;
      ctx.strokeStyle = nudged ? C.violet : (i === sel ? C.text : C.t3); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(kx, ky); ctx.lineTo(X(e[side][0], w), Y(e[side][1], h)); ctx.stroke();
      ctx.fillStyle = C.bg; ctx.beginPath(); ctx.arc(X(e[side][0], w), Y(e[side][1], h), 3.5, 0, 7); ctx.fill(); ctx.stroke();
      if (nudged) { // the length before the ceilings, ghosted
        const prev = knots[i - 1], next = knots[i + 1], p = M.pos(k) + k.dp;
        const g = side === 'in' ? [k.t - k.lIn * (k.t - prev.t), p - k.vIn * k.lIn * (k.t - prev.t)] : [k.t + k.lOut * (next.t - k.t), p + k.vOut * k.lOut * (next.t - k.t)];
        ctx.strokeStyle = C.muted; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.arc(X(g[0], w), Y(g[1], h), 3.5, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      }
    }
  }
  // knots
  knots.forEach((k, i) => {
    const x = X(k.t, w), y = Y(M.pos(k) + k.dp, h);
    if (x < PAD.l - 5 || x > w - PAD.r + 5) return;
    if (k.dp !== 0 || k.pm != null) { ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.arc(x, Y(k.p, h), 3.5, 0, 7); ctx.stroke(); }
    ctx.fillStyle = k.infeasible ? C.red : CLS[k.cls]; ctx.beginPath(); ctx.arc(x, y, i === sel ? 5 : 3.5, 0, 7); ctx.fill();
    if (i === sel) { ctx.strokeStyle = C.text; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, 8, 0, 7); ctx.stroke(); }
    if (k.type !== 'auto') { ctx.fillStyle = C.violet; ctx.font = `9px ${MONO}`; ctx.textAlign = 'center'; ctx.fillText(k.type, x, y - 11); }
  });
  ctx.restore();
  drawStrips();
}
function drawStrips() {
  const { ctx, w, h } = dims(strips);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
  if (!R) return;
  const rows = [['v', 'velocity mm/s', cfg.vmax, C.v], ['a', 'acceleration mm/s²', cfg.amax, C.a], ['j', 'jerk mm/s³', cfg.jmax, C.j]];
  const rh = (h - 8) / 3;
  rows.forEach(([key, label, ceil, color], r) => {
    const y0 = 4 + r * rh, y1 = y0 + rh - 6;
    let peak = ceil * 1.2;
    for (const pc of R.pieces) { if (pc.t[pc.t.length - 1] < view.t0 || pc.t[0] > view.t1) continue; const pk = key === 'v' ? pc.peakV : key === 'a' ? pc.peakA : pc.peakJ; if (pk > peak) peak = pk; }
    peak = Math.min(peak, ceil * 3);
    const YY = (val) => y0 + (y1 - y0) / 2 - val / peak * (y1 - y0) / 2;
    ctx.fillStyle = '#0D0F13'; ctx.fillRect(PAD.l, y0, w - PAD.l - PAD.r, y1 - y0);
    ctx.save(); ctx.beginPath(); ctx.rect(PAD.l, y0, w - PAD.l - PAD.r, y1 - y0); ctx.clip();
    ctx.strokeStyle = C.l2; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(PAD.l, YY(0)); ctx.lineTo(w - PAD.r, YY(0)); ctx.stroke();
    ctx.strokeStyle = C.red; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(PAD.l, YY(ceil)); ctx.lineTo(w - PAD.r, YY(ceil)); ctx.moveTo(PAD.l, YY(-ceil)); ctx.lineTo(w - PAD.r, YY(-ceil)); ctx.stroke(); ctx.setLineDash([]);
    for (const pc of R.pieces) {
      if (pc.t[pc.t.length - 1] < view.t0 || pc.t[0] > view.t1) continue;
      const s = pc[key];
      ctx.lineWidth = 1.5;
      for (let n = 1; n < s.length; n++) {
        ctx.strokeStyle = Math.abs(s[n]) > ceil * 1.001 ? C.red : color;
        ctx.beginPath(); ctx.moveTo(X(pc.t[n - 1], w), YY(s[n - 1])); ctx.lineTo(X(pc.t[n], w), YY(s[n])); ctx.stroke();
      }
    }
    // steps at the knots
    for (const st of R.steps) {
      const k = knots[st.i], x = X(k.t, w); if (x < PAD.l || x > w - PAD.r) continue;
      const a = R.pieces[st.i - 1], b = R.pieces[st.i];
      const va = key === 'v' ? a.vEnd : key === 'a' ? a.aEnd : null, vb = key === 'v' ? b.vStart : key === 'a' ? b.aStart : null;
      if (va == null || Math.abs(va - vb) < peak * 0.01) continue;
      ctx.strokeStyle = C.red; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, YY(va)); ctx.lineTo(x, YY(vb)); ctx.stroke();
    }
    if (sel != null) { const x = X(knots[sel].t, w); ctx.strokeStyle = C.text; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke(); }
    ctx.restore();
    ctx.fillStyle = C.muted; ctx.font = `10px ${SANS}`; ctx.textAlign = 'left'; ctx.fillText(`${label}   ceiling ${fmt(ceil)}`, PAD.l + 4, y0 + 11);
  });
}
const fmt = (x, d = 0) => (Math.abs(x) >= 1e6 ? `${(x / 1e6).toFixed(2)}e6` : Number(x).toFixed(d));

// ---- inspector and stats ----------------------------------------------------
function inspector() {
  const k = sel != null ? knots[sel] : null;
  $('#k-none').hidden = !!k; $('#k-box').hidden = !k; $('#k-idx').textContent = k ? `${sel + 1} of ${knots.length}` : '';
  if (!k) return;
  const st = R.steps.find((s) => s.i === sel);
  const rows = [['time', `${(k.t * 1000).toFixed(0)} ms`], ['class', `<span class="cls-${k.cls}">${k.cls}</span>`], ['renders as', k.eff],
    ['angle', k.eff === 'G0' ? `${fmt(k.vIn)} in, ${fmt(k.vOut)} out` : `${fmt(k.vIn)} mm/s`],
    ['lengths', `${k.lIn.toFixed(3)} in, ${k.lOut.toFixed(3)} out`],
    ['in ms', `${knots[sel - 1] ? (k.lIn * (k.t - knots[sel - 1].t) * 1000).toFixed(0) + ' in' : ''}${knots[sel + 1] ? `${knots[sel - 1] ? ', ' : ''}${(k.lOut * (knots[sel + 1].t - k.t) * 1000).toFixed(0)} out` : ''}`]];
  const pieceRow = (pc, label) => pc && [label, `v ${fmt(pc.peakV)}  a ${fmt(pc.peakA)}  j ${fmt(pc.peakJ)}${pc.over > 1.001 ? ` <span class="bad">over ×${pc.over.toFixed(2)}</span>` : ''}`];
  for (const r of [pieceRow(R.pieces[sel - 1], 'piece in'), pieceRow(R.pieces[sel], 'piece out')]) if (r) rows.push(r);
  const nudged = Math.abs(k.effIn - k.lIn) > 1e-6 || Math.abs(k.effOut - k.lOut) > 1e-6;
  if (nudged) rows.push(['after ceilings', `<dd class="nudge">${k.effIn.toFixed(3)} in, ${k.effOut.toFixed(3)} out`]);
  if (k.dp !== 0) rows.push(['trimmed', `<dd class="nudge">${k.dp > 0 ? '+' : ''}${k.dp.toFixed(2)} mm toward the previous knot`]);
  if (k.infeasible) rows.push(['', '<dd class="bad">no legal length, even trimmed']);
  if (st) rows.push(['at the knot', `Δv ${fmt(st.dv)}, Δa ${fmt(st.da)}`], ['corner ramp', `${st.rampMs.toFixed(1)} ms at this jerk${st.dv ? `, ${st.cornerMs.toFixed(1)} ms for the velocity step` : ''}`]);
  $('#k-info').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt>${b.startsWith('<dd') ? b + '</dd>' : `<dd>${b}</dd>`}`).join('');
  $('#k-type').value = k.type;
  const g0 = k.eff === 'G0';
  $('#k-vrow').hidden = g0; $('#k-vinrow').hidden = !g0; $('#k-voutrow').hidden = !g0;
  setIfIdle('#k-v', k.vIn.toFixed(0)); setIfIdle('#k-vin', k.vIn.toFixed(0)); setIfIdle('#k-vout', k.vOut.toFixed(0));
  setIfIdle('#k-lin', k.lIn.toFixed(2)); setIfIdle('#k-lout', k.lOut.toFixed(2)); setIfIdle('#k-p', M.pos(k).toFixed(1));
  const notes = [...k.note];
  if (k.cls === 'rest' && k.eff === 'G2') notes.push('a hold edge: a cubic leaves rest with acceleration, so the corner ramp takes the step');
  if (Object.keys(k.man).length) notes.push('hand-set: ' + Object.keys(k.man).join(', '));
  $('#k-note').textContent = notes.join('. ');
}
function setIfIdle(sel_, v) { const el = $(sel_); if (document.activeElement !== el) el.value = v; }
function stats() {
  if (!R) return;
  const c = R.report.counts || {}, longest = R.steps.reduce((m, s) => Math.max(m, s.rampMs), 0);
  const rows = [['knots', knots.length], ['holds', c.rest || 0], ['crests', c.crest || 0], ['through', c.through || 0],
    ['pieces over', `<dd class="${R.report.overPieces ? 'bad' : ''}">${R.report.overPieces} of ${R.pieces.length}`],
    ['infeasible', `<dd class="${R.report.infeasible ? 'bad' : ''}">${R.report.infeasible}`],
    ['nudged', knots.filter((k) => Math.abs(k.effIn - k.lIn) > 1e-6 || Math.abs(k.effOut - k.lOut) > 1e-6).length],
    ['trimmed', `${knots.filter((k) => k.dp !== 0).length} knots, ${knots.reduce((m, k) => m + Math.abs(k.dp), 0).toFixed(1)} mm`], ['longest ramp', `${longest.toFixed(1)} ms`]];
  $('#stats').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt>${String(b).startsWith('<dd') ? b + '</dd>' : `<dd>${b}</dd>`}`).join('');
}

// ---- pointer ----------------------------------------------------------------
function hit(ev) {
  const r = plot.getBoundingClientRect(), x = ev.clientX - r.left, y = ev.clientY - r.top, w = r.width, h = r.height;
  for (const i of visibleHandles()) {
    const e = handleEnds(i);
    for (const side of ['in', 'out']) if (e[side] && Math.hypot(X(e[side][0], w) - x, Y(e[side][1], h) - y) < 7) return { kind: 'handle', i, side };
  }
  let best = null;
  knots.forEach((k, i) => { const d = Math.hypot(X(k.t, w) - x, Y(M.pos(k) + k.dp, h) - y); if (d < 9 && (!best || d < best.d)) best = { kind: 'knot', i, d }; });
  return best;
}
plot.addEventListener('pointerdown', (ev) => {
  if (ev.button !== 0) return;
  const t = hit(ev);
  const r = plot.getBoundingClientRect();
  if (t && t.kind === 'handle') { sel = t.i; drag = { ...t }; }
  else if (t && t.kind === 'knot') { sel = t.i; drag = { kind: 'knot', i: t.i, moved: false }; }
  else drag = { kind: 'pan', x0: ev.clientX, v0: { ...view }, w: r.width };
  plot.setPointerCapture(ev.pointerId); update();
});
plot.addEventListener('pointermove', (ev) => {
  const r = plot.getBoundingClientRect(), w = r.width, h = r.height, x = ev.clientX - r.left, y = ev.clientY - r.top;
  if (!drag) { const t = hit(ev); plot.style.cursor = t ? 'pointer' : 'grab'; if (!playing) readout(T(x, w), P(y, h)); return; }
  if (drag.kind === 'pan') { const dt = (ev.clientX - drag.x0) / (drag.w - PAD.l - PAD.r) * (drag.v0.t1 - drag.v0.t0); view = { t0: drag.v0.t0 - dt, t1: drag.v0.t1 - dt }; draw(); drawCursor(); return; }
  const k = knots[drag.i];
  if (drag.kind === 'knot') { k.pm = Math.min(cfg.hi, Math.max(cfg.lo, P(y, h))); drag.moved = true; update(); return; }
  // a handle: the end lands under the pointer; Shift keeps the angle
  const prev = knots[drag.i - 1], next = knots[drag.i + 1];
  const span = drag.side === 'in' ? k.t - prev.t : next.t - k.t;
  const tt = T(x, w), pp = P(y, h), p = M.pos(k) + k.dp;
  const dt = Math.min(M.LMAX * span, Math.max(M.LMIN * span, drag.side === 'in' ? k.t - tt : tt - k.t));
  const l = dt / span;
  if (drag.side === 'in') k.man.lIn = l; else k.man.lOut = l;
  if (!ev.shiftKey) {
    const v = (drag.side === 'in' ? p - pp : pp - p) / dt;
    if (k.eff === 'G0') { if (drag.side === 'in') k.man.vIn = v; else k.man.vOut = v; }
    else { k.man.v = v; if (k.type === 'auto' || k.type === 'G2') k.type = 'G1'; }
  }
  update();
});
plot.addEventListener('pointerup', () => { drag = null; });
plot.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  const r = plot.getBoundingClientRect(), w = r.width, tc = T(ev.clientX - r.left, w), f = ev.deltaY > 0 ? 1.2 : 1 / 1.2;
  view = { t0: tc - (tc - view.t0) * f, t1: tc + (view.t1 - tc) * f }; draw(); drawCursor();
}, { passive: false });
function readout(t, p) { $('#readout').textContent = `${(t * 1000).toFixed(0)} ms   ${p.toFixed(1)} mm`; }

// ---- playback: the rendered motion at real speed, the carriage on the rail --
function stateAt(t) {
  if (!R || !R.pieces.length) return null;
  const P = R.pieces;
  if (t <= P[0].t[0]) return { p: P[0].p[0], v: 0, a: 0 };
  const last = P[P.length - 1];
  if (t >= last.t[last.t.length - 1]) return { p: last.p[last.p.length - 1], v: 0, a: 0 };
  let lo = 0, hi = P.length - 1;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (P[m].t[0] <= t) lo = m; else hi = m - 1; }
  const pc = P[lo]; let a = 0, b = pc.t.length - 1;
  while (b - a > 1) { const m = (a + b) >> 1; if (pc.t[m] <= t) a = m; else b = m; }
  const f = (t - pc.t[a]) / Math.max(1e-9, pc.t[b] - pc.t[a]);
  return { p: pc.p[a] + f * (pc.p[b] - pc.p[a]), v: pc.v[a] + f * (pc.v[b] - pc.v[a]), a: pc.a[a] + f * (pc.a[b] - pc.a[a]) };
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
  const top = 12, bot = h - 12, YR = (mm) => bot - mm / cfg.rail * (bot - top);
  ctx.fillStyle = '#12151B'; ctx.fillRect(w / 2 - 5, top, 10, bot - top);
  ctx.fillStyle = 'rgba(77,166,255,0.18)'; ctx.fillRect(w / 2 - 12, YR(cfg.hi), 24, YR(cfg.lo) - YR(cfg.hi));
  ctx.strokeStyle = C.blue; ctx.lineWidth = 1; ctx.strokeRect(w / 2 - 12, YR(cfg.hi), 24, YR(cfg.lo) - YR(cfg.hi));
  ctx.fillStyle = C.muted; ctx.font = `9px ${MONO}`; ctx.textAlign = 'center';
  ctx.fillText(`${cfg.rail}`, w / 2, top - 3); ctx.fillText('0', w / 2, bot + 10);
  const st = cursor == null ? null : stateAt(cursor);
  const p = st ? st.p : (knots.length ? M.pos(knots[0]) : cfg.lo);
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

// ---- controls ---------------------------------------------------------------
$('#file').addEventListener('change', (ev) => {
  const f = ev.target.files[0]; if (!f) return;
  f.text().then((txt) => { try { load(JSON.parse(txt), f.name); } catch (e) { $('#status').textContent = `not a funscript: ${e.message}`; } });
});
document.addEventListener('dragover', (ev) => ev.preventDefault());
document.addEventListener('drop', (ev) => { ev.preventDefault(); const f = ev.dataTransfer.files[0]; if (f) f.text().then((txt) => load(JSON.parse(txt), f.name)); });
$('#sample').addEventListener('click', () => load(M.SAMPLE, 'built-in'));
$('#autoall').addEventListener('click', () => { for (const k of knots) { k.type = 'auto'; k.man = {}; k.pm = null; } update(); });
$('#fit').addEventListener('click', () => { fitView(); draw(); drawCursor(); });
$('#k-type').addEventListener('change', (ev) => { if (sel == null) return; knots[sel].type = ev.target.value; update(); });
$('#k-auto').addEventListener('click', () => { if (sel == null) return; const k = knots[sel]; k.type = 'auto'; k.man = {}; k.pm = null; update(); });
$('#k-prev').addEventListener('click', () => { if (sel > 0) { sel--; update(); } });
$('#k-next').addEventListener('click', () => { if (sel != null && sel < knots.length - 1) { sel++; update(); } });
for (const [id, key] of [['#k-v', 'v'], ['#k-vin', 'vIn'], ['#k-vout', 'vOut'], ['#k-lin', 'lIn'], ['#k-lout', 'lOut']]) {
  $(id).addEventListener('change', (ev) => {
    if (sel == null) return; const k = knots[sel], v = Number(ev.target.value); if (!isFinite(v)) return;
    k.man[key] = key.startsWith('l') ? Math.min(M.LMAX, Math.max(M.LMIN, v)) : v;
    if (key === 'v' && (k.type === 'auto' || k.type === 'G2')) k.type = 'G1';
    update();
  });
}
$('#k-p').addEventListener('change', (ev) => { if (sel == null) return; knots[sel].pm = Math.min(cfg.hi, Math.max(cfg.lo, Number(ev.target.value))); update(); });
for (const [id, key] of [['#c-vmax', 'vmax'], ['#c-amax', 'amax'], ['#c-jmax', 'jmax'], ['#c-lfloor', 'lfloor'], ['#c-trim', 'trim'], ['#c-hold', 'holdEps']]) {
  $(id).value = cfg[key];
  $(id).addEventListener('change', (ev) => { const v = Number(ev.target.value); if (isFinite(v) && v >= 0) { cfg[key] = v; update(); } });
}
// the window: size and offset on the rail; the script rescales into it
function setWindow(lo, hi) {
  const old = { lo: cfg.lo, hi: cfg.hi }; cfg.lo = lo; cfg.hi = hi; cfg.trim = hi - lo; $('#c-trim').value = cfg.trim;
  for (const k of knots) { k.p = lo + (k.p - old.lo) / (old.hi - old.lo) * (hi - lo); if (k.pm != null) k.pm = lo + (k.pm - old.lo) / (old.hi - old.lo) * (hi - lo); }
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
  if (ev.key === 'Escape') { sel = null; update(); }
  if (ev.key === ' ') { ev.preventDefault(); togglePlay(); }
  if (ev.key === 'ArrowLeft' && sel > 0) { sel--; update(); }
  if (ev.key === 'ArrowRight' && sel != null && sel < knots.length - 1) { sel++; update(); }
});
new ResizeObserver(() => { draw(); drawCursor(); }).observe(plot);
new ResizeObserver(() => { drawStrips(); drawCursor(); }).observe(strips);
$('#legend').innerHTML = [['hold edge', CLS.rest], ['crest', CLS.crest], ['through point', CLS.through], ['nudged handle', C.violet], ['over a ceiling', C.red], ['infeasible knot', C.hi]]
  .map(([n, c]) => `<li><svg width="10" height="10"><circle cx="5" cy="5" r="4" fill="${c}"/></svg>${n}</li>`).join('');
// for handles.check.mjs
window.__handles = {
  state: () => ({ knots, R, sel, view, cfg, cursor, playing }),
  setWindow,
  xy: (i) => { const r = plot.getBoundingClientRect(); return { x: r.left + X(knots[i].t, r.width), y: r.top + Y(M.pos(knots[i]) + knots[i].dp, r.height) }; },
  handleXY: (i, side) => { const r = plot.getBoundingClientRect(), e = handleEnds(i)[side]; return { x: r.left + X(e[0], r.width), y: r.top + Y(e[1], r.height) }; },
};
load(M.SAMPLE, 'built-in');
