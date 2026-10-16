// steps.js -- the planner steps: plays through what Kinetic² computed for the
// knots on the page, read from kinetic2-trace.wasm's trace records.
// Constraints:
// - Every number shown is a trace record's, or the trace build's own planner
//   functions' (kinetic2_trace_curve); the plan is never recomputed here. The
//   bisection's bracket is the tested moves labeled by their verdicts.
// - Records are wasm/kinetic2_trace.cpp's: 96 bytes, kind, a, b, c, f[20].
// - Loaded only when the steps are shown (play.js showSteps); the normal page
//   never fetches this file or the trace build.
// See: playground/README.md "Planner steps"

const REC = 24;   // int32 words per record
const KD = { SOLVE: 1, RUN: 2, RKNOT: 3, PASS: 4, STYLE: 5, KNOT: 6, ROUND: 7, PIECE: 8, JUDGE: 9, FIT: 10, PIECE_END: 11, PASS_END: 12, BUILT: 13, EMIT: 14, CHASE: 15, HARD: 16 };
const TOL = 1e-3;
const CLS = ['end', 'rest', 'crest', 'thru'];
const KGRID = Array.from({ length: 37 }, (_, i) => +(0.2 + 0.05 * i).toFixed(2));   // the length factors, 0.2 to 2 (handles.hpp ksAt)
const CURVE_N = 682;   // floats kinetic2_trace_curve writes
const TWEEN = 420;     // ms from one step's picture to the next
const DWELL = { 0: 1500, 1: 650 };   // ms a step holds while playing, by depth, at 1x

let api, C, MONO, SANS, W, host, el = {};
let solves = [], solveIx = 0, picked = -1, steps = [], idx = 0, everyJudge = false, R = null, cfg = null;
let shown = null, from = null, to = null, t0 = 0, playing = false, lastTick = 0, held = 0, raf = 0, ride0 = 0;
let curveBuf = 0;
const curveCache = new Map();
const reduced = matchMedia('(prefers-reduced-motion: reduce)');

export async function init(root, a) {
  api = a; ({ C, MONO, SANS } = a); host = root;
  const res = await fetch('./kinetic2-trace.wasm');
  if (!res.ok) throw new Error('kinetic2-trace.wasm ' + res.status);
  const { instance } = await WebAssembly.instantiate(await res.arrayBuffer(), {});
  const K = instance.exports;
  if (!K.kinetic2_trace_curve || K.kinetic2_submit.length !== 7) throw new Error('kinetic2-trace.wasm does not match this page; rebuild it');
  K._initialize();
  W = { K, h: K.kinetic2_create(), dv: new DataView(K.memory.buffer), sBuf: K.malloc(24), kBuf: K.malloc(40), aBuf: K.malloc(24) };
  curveBuf = K.malloc(CURVE_N * 4);
  build();
  return { update, pause: () => setPlaying(false) };
}

// ---- the trace ---------------------------------------------------------------
function update() {
  const K = W.K;
  K.kinetic2_trace_clear();
  R = api.replay(W);
  const n = K.kinetic2_trace_count(), base = K.kinetic2_trace_records() >> 2;
  const i32 = new Int32Array(K.memory.buffer), f32 = new Float32Array(K.memory.buffer);
  const recs = new Array(n);
  for (let r = 0; r < n; r++) {
    const o = base + r * REC;
    recs[r] = { kind: i32[o], a: i32[o + 1], b: i32[o + 2], c: i32[o + 3], f: Array.from(f32.subarray(o + 4, o + REC)) };
  }
  solves = [];
  for (let r = 0; r < n; r++) if (recs[r].kind === KD.SOLVE) solves.push({ at: r, recs: null });
  solves.forEach((s, i) => { s.recs = recs.slice(s.at, i + 1 < solves.length ? solves[i + 1].at : n); s.render = s.recs.some((x) => x.kind === KD.RUN); });
  curveCache.clear();
  // The solve shown: the one picked, else the render with the most knots (authored ahead: the only solve), else the largest chase.
  const most = (ok) => solves.reduce((b, s, i) => (ok(s) && (b < 0 || s.recs[0].a > solves[b].recs[0].a) ? i : b), -1);
  const render = most((s) => s.render);
  solveIx = picked >= 0 && picked < solves.length ? picked : render >= 0 ? render : Math.max(0, most(() => true));
  el.solve.innerHTML = solves.map((s, i) => `<option value="${i}">solve ${i + 1} of ${solves.length}: ${s.recs[0].a} knots at ${(s.recs[0].f[0] * 1000).toFixed(0)} ms${s.render ? '' : ', chase'}</option>`).join('');
  el.solve.value = String(solveIx);
  el.solvewrap.hidden = solves.length < 2;
  el.lost.textContent = K.kinetic2_trace_lost() ? `The trace buffer filled: ${K.kinetic2_trace_lost()} records past it are missing.` : '';
  rebuild();
}

function rebuild() {
  const ri = steps[idx]?.ri ?? 0;
  steps = solves.length ? stepsOf(solves[solveIx].recs) : [];
  steps.push(samplesStep());
  idx = Math.max(0, steps.findIndex((s) => s.ri >= ri));
  if (idx < 0) idx = 0;
  el.scrub.max = String(Math.max(0, steps.length - 1));
  list();
  go(idx, false);
}

// ---- the steps ---------------------------------------------------------------
// Each step: kind, title, lines [[text, class]], ri (its record), depth (1:
// inside a piece), snap (the knots as they stand), and what its panels show.
function stepsOf(rs) {
  const out = [];
  const S0 = rs[0];
  cfg = { t0: S0.f[0], p: S0.f[1], v: S0.f[2], a: S0.f[3], lim: [S0.f[4], S0.f[5], S0.f[6]], smooth: S0.f[7], floor: S0.f[8], trim: S0.f[9], lo: S0.f[10], hi: S0.f[11], holdEps: S0.f[12] };
  let cur = null, snap = null, dirty = true, run = null, pass = 0, round = 0, styleN = 0, piece = null, built = null;
  const detailed = () => everyJudge || (pass === 0 && round === 0);
  const snapNow = () => { if (dirty) { snap = Object.fromEntries(Object.entries(cur).map(([k, v]) => [k, v.slice()])); dirty = false; } return snap; };
  const push = (s) => { out.push({ depth: 0, ...s, snap: cur ? snapNow() : null, run }); };
  const knotsFrom = (r0, n) => {
    for (let i = 0; i < n; i++) {
      const k = rs[r0 + 1 + i], f = k.f;
      cur.cls[i] = k.b & 3; cur.vel[i] = f[3]; cur.lIn[i] = f[4]; cur.lOut[i] = f[5]; cur.ei[i] = f[6]; cur.eo[i] = f[7]; cur.dp[i] = f[2];
      cur.inf[i] = (k.b >> 4) & 1; cur.s0[i] = f[12]; cur.s1[i] = f[13]; cur.s2[i] = f[14];
    }
    dirty = true;
  };
  for (let r = 0; r < rs.length; r++) {
    const x = rs[r], f = x.f;
    switch (x.kind) {
      case KD.RUN: {
        const m = x.a, me = x.b;
        run = { m, me, base: x.c, te0: f[0], knots: rs.slice(r + 1, r + 1 + m).map((k) => ({ t: k.f[0], p: k.f[1], v: k.f[2], hasV: k.b, te: k.c })), emit: [] };
        const z = () => run.knots.map(() => 0);
        cur = { t: run.knots.map((k) => k.t), p: run.knots.map((k) => k.p), dp: z(), vel: run.knots.map((k) => (k.hasV ? k.v : 0)), lIn: z(), lOut: z(), ei: z(), eo: z(), cls: z().map(() => -1), inf: z(), s0: z().map(() => 1), s1: z().map(() => 1), s2: z().map(() => 1) };
        dirty = true; pass = 0; round = 0;
        const o = run.knots[0];
        push({ kind: 'knots', stage: 0, ri: r, title: `Knots in: ${me - 1} after the origin`, lines: [
          [`> ${me - 1} knot${me === 2 ? '' : 's'} to render, ${fx(run.knots[me - 1].t, 3)} s ahead`, 'head'],
          [`  origin at p ${fx(o.p, 4)}, ${o.v ? 'moving ' + fx(o.v, 3) + ' u/s' : 'at rest'}`],
          [`  ceilings v ${num(cfg.lim[0])}  a ${num(cfg.lim[1])}  j ${num(cfg.lim[2])}`],
          ...(m > me ? [['  + a provisional successor: the newest knot renders through', 'muted']] : []),
          ['  time and position only, every knot on its own time']] });
        r += m;
        break;
      }
      case KD.PASS:
        pass = x.a; round = 0; styleN = 0;
        if (pass > 0) {
          const tight = (built || []).filter((b) => b.f[1] < 1 || b.f[2] < 1 || b.f[3] < 1);
          push({ kind: 'pass', stage: 4, ri: r, title: `Pass ${pass + 1}: render again${x.b ? ' (rescue)' : ''}`, lines: [
            [`> pass ${pass + 1}: render again under the tightened shares`, 'head'],
            ...tight.slice(0, 6).map((b) => [`  piece ${b.a - 1}→${b.a}: v ×${fx(b.f[1], 3)} a ×${fx(b.f[2], 3)} j ×${fx(b.f[3], 3)}`]),
            ...(x.b ? [['  rescue: a corner ramp may end on its knot']] : [])] });
        }
        break;
      case KD.STYLE: {
        knotsFrom(r, x.c);
        const smooth = x.b === 1, ph = x.a;
        if (detailed()) {
          if (ph === 0 && styleN === 0) {
            const cnt = [0, 1, 2, 3].map((c) => cur.cls.filter((v) => v === c).length);
            push({ kind: 'classify', stage: 1, ri: r, title: round ? `Round ${round + 1}: classify the trimmed chords` : 'Classify each knot by its chords', lines: [
              ['> classify by the chords in and out', 'head'],
              [`  ${cnt[3]} through · ${cnt[2]} crest · ${cnt[1]} hold edge · ${cnt[0]} end`],
              ['  a crest or hold edge is flat; a through point takes an angle']] });
          }
          if (ph === 0) push({ kind: 'angles', stage: 2, ri: r, title: `Start angles: ${smooth ? 'Makima' : 'pchip'}`, lines: [
            [`> through points: ${smooth ? "Makima's angle" : "pchip's weighted harmonic mean"}`, 'head'],
            ['  crests, hold edges and ends: zero slope'],
            ['  an authored angle is kept, each held to vmax']] });
          else push({ kind: 'sweeps', stage: 3, ri: r, title: `G2 sweeps (${smooth ? 'smooth' : 'pchip'})`, lines: [
            ['> four G2 sweeps: acceleration matched both sides', 'head'],
            ...changed(out, cur.vel, 'vel', 'angle')] });
        }
        styleN++;
        r += x.c;
        break;
      }
      case KD.ROUND: {
        knotsFrom(r, x.c);
        r += x.c;
        if (x.a === 0 || x.a === 2) {
          if (x.a === 2) round = x.b;
          const third = cfg.smooth === 0;
          if (detailed() || x.a === 2) push({ kind: 'solved', stage: 4, ri: r - x.c, title: x.a === 0 ? (third ? 'Every handle a third' : `Blend at smoothness ${fx(cfg.smooth, 2)}`) : `Round ${round + 1}: solve again on the trimmed chords`, lines: x.a === 0
            ? [[third ? '> every handle a third of its span' : `> lerp pchip and smooth by ${fx(cfg.smooth, 2)}`, 'head'], [third ? '  the cubic Hermite, before any ceiling' : '  angles and both lengths, before any ceiling']]
            : [['> solve again on the trimmed chords', 'head'], ...changed(out, cur.vel, 'vel', 'angle')] });
          // nudge starts each piece from the solved lengths, untrimmed
          for (let i = 0; i < cur.t.length; i++) { cur.eo[i] = cur.lOut[i]; cur.ei[i] = cur.lIn[i]; cur.dp[i] = 0; cur.inf[i] = 0; }
          dirty = true;
        } else {
          const trimmed = cur.dp.filter((d) => d !== 0).length, over = cur.inf.filter(Boolean).length;
          push({ kind: 'round', stage: 4, ri: r - x.c, title: `Round ${round + 1} fitted: ${trimmed} trimmed, ${over} over`, lines: [
            [`> round ${round + 1}: every piece fitted`, 'head'], [`  ${trimmed} knot${trimmed === 1 ? '' : 's'} trimmed, ${over} piece${over === 1 ? '' : 's'} over a ceiling`, over ? 'bad' : 'ok']] });
          round = x.b + 1; styleN = 0;
        }
        break;
      }
      case KD.PIECE: {
        const i = x.a;
        piece = { i, hit: x.b, pL: f[0], Rp: f[1], T: f[2], Lvel: f[3], Rvel: f[4], tL: f[11], tR: f[12], judges: [], fits: [], f0: null, bracket: null };
        if (x.b) {
          cur.eo[i] = f[7]; cur.ei[i + 1] = f[8]; cur.dp[i + 1] = f[9]; cur.vel[i + 1] = f[14]; cur.inf[i + 1] = f[10] ? 0 : 1; dirty = true;
          const last = out[out.length - 1];
          if (last && last.kind === 'memo' && last.to === i - 1 && last.pass === pass && last.round === round) {
            last.to = i; last.title = `Pieces ${last.from}→${last.from + 1} to ${i}→${i + 1} unchanged: reused`;
            last.lines[0][0] = `> pieces ${last.from}→${last.from + 1} to ${i}→${i + 1}: inputs unchanged`;
            last.snap = snapNow();
          } else push({ kind: 'memo', stage: 4, ri: r, from: i, to: i, pass, round, focus: i, title: `Piece ${i}→${i + 1} unchanged: reused`, lines: [[`> piece ${i}→${i + 1}: inputs unchanged`, 'head'], ['  its last fit is kept, bit for bit']] });
        }
        break;
      }
      case KD.JUDGE: {
        const j = { k: f[0], T: f[1], D: f[2], s0: f[3], s1: f[4], i0: f[5], i1: f[6], pL: f[7], o: f[8], ro: f[9], rv: f[10], ra: f[11], rj: f[12], rx: f[13], lim: [f[14], f[15], f[16]], legal: !!x.b, retry: x.c };
        piece.judges.push(j);
        if (detailed()) {
          const n = piece.judges.length, why = overWhy(j);
          push({ kind: 'judge', stage: 4, depth: 1, ri: r, focus: piece.i, piece, judge: j, n, title: `Piece ${piece.i}→${piece.i + 1} · k ${fx(j.k, 2)} · ${j.legal ? 'legal' : 'over'}`, lines: [
            ...(n === 1 ? [[`> piece ${piece.i}→${piece.i + 1}: ${fx(j.D, 4)} in ${ms(j.T)} ms${j.retry ? ', without the end ask' : ''}`, 'head'], ['  scale both handles by k, nearest 1 first']] : []),
            [`  k ${fx(j.k, 2)}: handles ${fx(j.i0, 3)}, ${fx(j.i1, 3)}`],
            [j.legal ? `  legal: v ×${fx(j.rv, 3)}, a ×${fx(j.ra, 3)}, j ×${fx(j.rj, 3)}` : `  over: ${why}`, j.legal ? 'ok' : 'bad']] });
        }
        break;
      }
      case KD.FIT: {
        const ft = { pR: f[0], k: f[1], i0: f[2], i1: f[3], o: f[4], s1: f[5], judged: f[6], legal: !!x.b, phase: x.a };
        if (x.a === 0) { piece.f0 = ft; break; }
        piece.fits.push(ft);
        const move = Math.abs(ft.pR - piece.Rp), full = Math.abs(piece.Rp - piece.pL);
        if (x.a === 1) piece.bracket = [0, move];
        else if (x.a >= 10 && x.a < 30 && piece.bracket) piece.bracket = ft.legal ? [piece.bracket[0], move] : [move, piece.bracket[1]];
        if (detailed()) {
          const keep = full > 0 ? Math.abs(ft.pR - piece.pL) / full : 1;
          const what = x.a === 1 ? 'the full move' : x.a < 30 ? `halving ${x.a - 9}` : x.a < 40 ? `quarter ${x.a - 30}` : 'flat at the previous knot';
          push({ kind: 'trial', stage: 4, depth: 1, ri: r, focus: piece.i, piece, fit: ft, bracket: piece.bracket && piece.bracket.slice(), title: `Piece ${piece.i}→${piece.i + 1} · trim, ${what} · ${ft.legal ? 'legal' : 'over'}`, lines: [
            ...(x.a === 1 ? [['> no factor is legal: trim toward the previous knot', 'head'], ['  the time stays; the position moves']] : []),
            [`  ${what}: move ${fx(move, 4)}, keeps ${pc(keep)}`],
            [ft.legal ? `  legal at k ${fx(ft.k, 2)} (${ft.judged} judged)` : `  over ×${fx(ft.o, 3)} (${ft.judged} judged)`, ft.legal ? 'ok' : 'bad']] });
        }
        break;
      }
      case KD.PIECE_END: {
        const i = x.a, e = { k: f[0], i0: f[1], i1: f[2], o: f[3], dp: f[4], s1: f[5], legal: !!x.b, hold: x.c & 1 };
        cur.eo[i] = e.i0; cur.ei[i + 1] = e.i1; cur.dp[i + 1] = e.dp; cur.vel[i + 1] = e.s1; cur.inf[i + 1] = e.legal ? 0 : 1; dirty = true;
        const full = Math.abs(piece.Rp - piece.pL), keep = full > 0 ? Math.abs(piece.Rp + e.dp - piece.pL) / full : 1;
        push({ kind: 'piece', stage: 4, ri: r, focus: i, piece, end: e, title: `Piece ${i}→${i + 1}: ${e.legal ? (e.dp ? `trimmed, keeps ${pc(keep)}` : `k ${fx(e.k, 2)}, legal`) : `over ×${fx(e.o, 3)}`}`, lines: [
          [`> piece ${i}→${i + 1}: ${piece.judges.length} factor${piece.judges.length === 1 ? '' : 's'} judged${piece.fits.length ? `, ${piece.fits.length} trim tests` : ''}`, 'head'],
          [`  handles ${fx(e.i0, 3)}, ${fx(e.i1, 3)} at k ${fx(e.k, 2)}`],
          ...(e.dp ? [[`  knot ${i + 1} moved ${fx(Math.abs(e.dp), 4)} toward knot ${i}`]] : []),
          [e.legal ? '  legal' : `  no trim keeps it inside: least over ×${fx(e.o, 3)}`, e.legal ? 'ok' : 'bad']] });
        break;
      }
      case KD.PASS_END: {
        built = rs.slice(r + 1, r + x.a);
        const over = built.filter((b) => b.f[0] > 1 + TOL);
        push({ kind: 'built', stage: 4, ri: r, built, again: x.b, title: `Built: ${over.length ? over.length + ' over' : 'all inside'}${x.b ? ', tighten' : ''}`, lines: [
          ['> build each piece as the engine will, 1 ms grid', 'head'],
          ...over.slice(0, 5).map((b) => [`  piece ${b.a - 1}→${b.a} built ×${fx(b.f[0], 3)}`, 'bad']),
          [x.b ? '  tighten the ceilings it went over, render again' : '  nothing to tighten', x.b ? '' : 'ok']] });
        r += x.a - 1;
        break;
      }
      case KD.EMIT: {
        run.emit.push({ r: x.a, fl: x.b, t: f[0], p: f[1], v: f[2], a: f[3], share: f[4], worst: f[5], knotP: f[6], head: f[7], rampT: f[8], rampJ: f[9] });
        if (rs[r + 1]?.kind !== KD.EMIT) {
          const corners = run.emit.filter((e) => e.fl & 1).length, over = run.emit.filter((e) => e.fl & 2).length, trims = run.emit.filter((e) => e.share < 0.999).length;
          push({ kind: 'result', stage: 5, ri: r, title: `Solved: ${run.emit.length} knot${run.emit.length === 1 ? '' : 's'}, ${corners} corner ramp${corners === 1 ? '' : 's'}`, lines: [
            ['> solved knots, as the engine will build them', 'head'],
            [`  ${corners} corner ramp${corners === 1 ? '' : 's'} at jmax where accelerations differ`],
            [`  ${trims} trimmed · ${over} over a ceiling`, over ? 'bad' : 'ok']] });
        }
        break;
      }
      case KD.CHASE:
        if (rs[r - 1]?.kind !== KD.CHASE) {
          const n = x.c - x.b, last = rs[r + n - 1];
          push({ kind: 'chase', stage: 5, ri: r, title: `Chase: ${n} sample${n === 1 ? '' : 's'}, one move to rest`, lines: [
            ['> position-only samples: the chase', 'head'],
            [`  one fastest legal move to rest on the newest, p ${fx(last.f[1], 4)}`],
            [`  lands at ${fx(last.f[0], 3)} s${last.f[5] > 0 ? `, ${ms(last.f[5])} ms late` : ''}`, last.f[6] ? 'bad' : 'ok']] });
        }
        break;
      case KD.HARD:
        push({ kind: 'hard', stage: 5, ri: r, title: `Live jog to ${fx(f[1], 4)}`, lines: [
          ['> a live jog: the fastest move to rest on it', 'head'],
          [`  lands at ${fx(f[0], 3)} s${f[3] > 0 ? `, ${ms(f[3])} ms past its time` : ''}`],
          [x.b ? `  over ×${fx(f[2], 3)}` : '  legal', x.b ? 'bad' : 'ok']] });
        break;
    }
  }
  return out;
}

// Lines naming the knots whose value moved since the last step that showed it.
function changed(out, now, key, what) {
  const prev = [...out].reverse().find((s) => s.snap && s.snap[key] && s.snap[key].length === now.length);
  if (!prev) return [];
  const moved = now.map((v, i) => [i, prev.snap[key][i], v]).filter(([, a, b]) => a !== b);
  if (!moved.length) return [[`  no ${what} moved`]];
  return moved.slice(0, 4).map(([i, a, b]) => [`  knot ${i}: ${what} ${fx(a, 3)} → ${fx(b, 3)} u/s`]).concat(moved.length > 4 ? [[`  and ${moved.length - 4} more`]] : []);
}

function overWhy(j) {
  const parts = [['v', j.rv], ['a', j.ra], ['j', j.rj], ['window', j.rx]].filter(([, x]) => x > 1 + TOL).map(([n, x]) => `${n} ×${fx(x, 3)}`);
  if (j.ro > 1 + TOL) parts.push(`room ×${fx(j.ro, 3)}`);
  return parts.join(', ') || `×${fx(j.o, 3)}`;
}

function samplesStep() {
  return { kind: 'samples', stage: 6, depth: 0, ri: Infinity, snap: null, title: 'The 1 ms samples', lines: [
    ['> the engine samples the built pieces every 1 ms', 'head'],
    [`  ${R.n} samples · peaks v ${num(R.max.v)} a ${num(R.max.a)} j ${num(R.max.j)}`],
    [R.over ? `  ${R.over} samples over a ceiling` : '  every sample within its ceilings', R.over ? 'bad' : 'ok']] };
}

// ---- views: what each panel shows at a step, as numbers a tween can blend ------
function viewOf(i) {
  const s = steps[i];
  const base = steps.slice(0, i + 1).reverse().find((x) => x.snap) || null;
  const k = base ? base.snap : null;
  const v = { kind: s.kind, stage: s.stage, i, k: null, focus: null, detail: null, scope: null };
  const tR = (R.n - 1) / 1000 - t0of();
  v.tEnd = k ? Math.max(k.t[k.t.length - 1] * 1.03, s.stage === 6 ? tR : 0, 0.05) : tR;
  if (k) {
    v.k = { t: k.t, p: k.p.slice(), dp: k.dp.slice(), vel: k.vel.slice(), eo: k.eo.slice(), ei: k.ei.slice(), inf: k.inf, cls: k.cls };
    const run = base.run;
    v.me = run.me; v.m = run.m;
    v.aCls = s.stage >= 1 ? 1 : 0; v.aAng = s.stage >= 2 ? 1 : 0; v.aHnd = s.stage >= 4 && s.stage < 6 ? 1 : 0; v.aRes = s.stage === 5 ? 1 : 0;
    v.reveal = s.stage === 6 ? 1 : 0;
    // the piece under the judge: its tested lengths and end
    const pc = s.judge ? { i0: s.judge.i0, i1: s.judge.i1, pR: s.piece.pL + s.judge.D, s1: s.judge.s1 }
      : s.fit ? { i0: s.fit.i0, i1: s.fit.i1, pR: s.fit.pR, s1: s.fit.s1 } : null;
    if (pc) {
      const i0 = s.focus;
      v.k.eo[i0] = pc.i0; v.k.ei[i0 + 1] = pc.i1; v.k.dp[i0 + 1] = pc.pR - k.p[i0 + 1]; v.k.vel[i0 + 1] = pc.s1;
    }
    if (s.focus != null && s.focus + 1 < k.t.length) v.focus = [k.t[s.focus], k.t[s.focus + 1], 1];
    if (s.kind === 'result') v.emit = run.emit;
  } else if (steps[i].kind === 'samples' || !k) {
    v.reveal = s.kind === 'samples' ? 1 : 0;
  }
  v.detail = detailOf(s, v);
  v.scope = scopeOf(s);
  return v;
}

function pieceCp(s, v) {
  const i = s.focus, k = v.k, T = k.t[i + 1] - k.t[i];
  const pL = k.p[i] + k.dp[i], pR = k.p[i + 1] + k.dp[i + 1];
  return [k.t[i], pL, k.t[i] + k.eo[i] * T, pL + k.vel[i] * k.eo[i] * T, k.t[i + 1] - k.ei[i + 1] * T, pR - k.vel[i + 1] * k.ei[i + 1] * T, k.t[i + 1], pR];
}

function detailOf(s, v) {
  if (['judge', 'trial', 'piece', 'memo'].includes(s.kind) && v.k) {
    const cp = pieceCp(s, v), i = s.focus, T = v.k.t[i + 1] - v.k.t[i];
    const Rp = v.k.p[i + 1];
    const ps = [cp[1], cp[3], cp[5], cp[7], Rp];
    let pA = Math.min(...ps), pB = Math.max(...ps);
    const pad = Math.max(0.02, (pB - pA) * 0.18);
    const d = { kind: 'piece', cp, win: [v.k.t[i] - 0.12 * T, v.k.t[i + 1] + 0.12 * T, pA - pad, pB + pad], ghost: [v.k.t[i + 1], Rp], gA: v.k.dp[i + 1] !== 0 || s.kind === 'trial' ? 1 : 0, i };
    if (s.kind === 'judge' || s.kind === 'trial' || s.kind === 'piece') {
      const judged = s.kind === 'judge' ? s.piece.judges.slice(0, s.n) : s.piece.judges;
      // the length walk at the knot as submitted; a trim test's own walk is counted, not drawn
      if (s.kind !== 'trial') d.cells = KGRID.map((kg) => {
        const j = judged.find((x) => Math.abs(x.k - kg) < 1e-3);
        return j ? [j.rv, j.ra, j.rj, j.legal ? 0 : 2] : null;
      });
      d.kmark = s.judge ? s.judge.k : s.fit ? s.fit.k : s.end ? s.end.k : 1;
      d.verdict = s.judge ? s.judge.legal : s.fit ? s.fit.legal : s.end ? s.end.legal : true;
      d.L = [s.judge ? s.judge.i0 : s.fit ? s.fit.i0 : s.end.i0, s.judge ? s.judge.i1 : s.fit ? s.fit.i1 : s.end.i1];
    }
    if (s.kind === 'trial' && s.bracket) {
      const dir = Math.sign(s.piece.pL - s.piece.Rp) || 1;
      d.bracket = [s.piece.Rp + dir * s.bracket[0], s.piece.Rp + dir * s.bracket[1]];
      d.test = [v.k.t[i + 1], s.fit.pR];
    }
    return d;
  }
  if (s.kind === 'built') return { kind: 'built', bars: s.built.map((b) => [b.a, b.f[0]]) };
  if (s.kind === 'result' && v.emit) return { kind: 'result' };
  if (s.kind === 'samples') return { kind: 'samples' };
  if (v.k && ['knots', 'classify', 'angles', 'sweeps', 'solved', 'round', 'pass'].includes(s.kind)) return { kind: 'table' };
  return { kind: 'text' };
}

// The piece a step judges, through the trace build's own evaluator.
function scopeOf(s) {
  let q = null;
  if (s.judge) { const j = s.judge; q = [j.T, j.D, j.s0, j.s1, j.i0, j.i1, j.pL, ...j.lim]; }
  else if (s.fit || s.end) {
    const p = s.piece, x = s.fit || s.end, snap = s.snap, i = s.focus + 1;
    const lim = [cfg.lim[0] * snap.s0[i], cfg.lim[1] * snap.s1[i], cfg.lim[2] * snap.s2[i]];
    const pR = s.fit ? s.fit.pR : p.Rp + s.end.dp;
    q = [p.T, pR - p.pL, p.Lvel, x.s1, x.i0, x.i1, p.pL, ...lim];
  }
  if (s.kind === 'samples' || s.kind === 'result') return { kind: 't' };
  if (!q) return { kind: 'none' };
  const c = curveOf(q);
  return { kind: 'u', V: c.dense.map((d) => d[3]), A: c.dense.map((d) => d[4]), J: c.dense.map((d) => d[5]), lim: q.slice(7), dots: s.judge ? c.samples : null,
    roots: s.judge ? c.roots : null, ratio: s.judge ? [s.judge.rv, s.judge.ra, s.judge.rj] : c.over.slice(1, 4) };
}

function curveOf(q) {
  const key = q.join(',');
  if (curveCache.has(key)) return curveCache.get(key);
  const K = W.K;
  K.kinetic2_trace_curve(...q, cfg.lo, cfg.hi, curveBuf);
  const a = Array.from(new Float32Array(K.memory.buffer, curveBuf, CURVE_N));
  const dense = Array.from({ length: 101 }, (_, i) => a.slice(i * 6, i * 6 + 6));
  const samples = Array.from({ length: 11 }, (_, i) => a.slice(606 + i * 4, 610 + i * 4));
  const rootsAt = (o, cap) => Array.from({ length: a[o] }, (_, i) => [a[o + 1 + 2 * i], a[o + 2 + 2 * i]]).slice(0, cap);
  const c = { dense, samples, roots: [rootsAt(650, 3), rootsAt(657, 4), rootsAt(666, 5)], over: a.slice(677, 682) };
  curveCache.set(key, c);
  return c;
}

// ---- the tween ---------------------------------------------------------------
function mix(a, b, u) {
  if (typeof a === 'number' && typeof b === 'number') return a + (b - a) * u;
  if (Array.isArray(a) && Array.isArray(b) && a.length === b.length) return b.map((x, i) => mix(a[i], x, u));
  if (a && b && typeof a === 'object' && typeof b === 'object' && !Array.isArray(a) && !Array.isArray(b) && a.kind === b.kind) {
    const o = {};
    for (const k in b) o[k] = k in a ? mix(a[k], b[k], u) : b[k];
    return o;
  }
  return b;
}
const ease = (u) => (u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2);

function go(i, animate = true) {
  idx = Math.max(0, Math.min(steps.length - 1, i));
  const next = viewOf(idx);
  from = shown && animate && !reduced.matches ? shown : next;
  to = next;
  t0 = performance.now();
  held = 0;
  if (to.kind === 'samples') ride0 = performance.now();
  el.scrub.value = String(idx);
  el.pos.textContent = `${idx + 1} / ${steps.length}`;
  el.title.textContent = steps[idx].title;
  // for check.mjs: the steps, the one shown, and the trace build's samples
  window.__k2steps = { idx, solves: solves.length, P: R.P, lost: W.K.kinetic2_trace_lost(), steps: steps.map((s) => ({ kind: s.kind, title: s.title, depth: s.depth })) };
  log();
  list(true);
  kick();
}

function kick() { if (!raf) raf = requestAnimationFrame(frame); }
function frame(now) {
  raf = 0;
  const u = Math.min(1, (now - t0) / TWEEN);
  shown = u >= 1 ? to : mixView(from, to, ease(u));
  draw(shown, now);
  typeLog(now);
  if (playing) {
    if (u >= 1) {
      held += now - (lastTick || now);
      if (held >= DWELL[steps[idx].depth] / +el.speed.value) {
        if (idx + 1 < steps.length) go(idx + 1); else setPlaying(false);
      }
    }
    lastTick = now;
  }
  if (!host.hidden && (u < 1 || playing || (to.kind === 'samples' && !reduced.matches) || typing)) kick();
}
function mixView(a, b, u) {
  const v = { ...b };
  for (const key of ['k', 'focus', 'aCls', 'aAng', 'aHnd', 'aRes', 'reveal', 'tEnd']) if (a[key] != null && b[key] != null) v[key] = mix(a[key], b[key], u);
  if (a.k && b.k && a.k.t.length !== b.k.t.length) v.k = b.k;
  else if (v.k) v.k = { ...v.k, t: b.k.t, cls: b.k.cls, inf: b.k.inf };   // classes and verdicts switch, never blend
  if (a.detail && b.detail && a.detail.kind === b.detail.kind) v.detail = mix(a.detail, b.detail, u);
  if (a.scope && b.scope && a.scope.kind === b.scope.kind && a.scope.kind === 'u') v.scope = { ...b.scope, V: mix(a.scope.V, b.scope.V, u), A: mix(a.scope.A, b.scope.A, u), J: mix(a.scope.J, b.scope.J, u), lim: mix(a.scope.lim, b.scope.lim, u) };
  v.fade = b.detail && a.detail && a.detail.kind === b.detail.kind ? 1 : u;
  return v;
}

function setPlaying(on) {
  playing = on && steps.length > 1;
  if (playing && idx + 1 >= steps.length) go(0);
  el.play.textContent = playing ? 'Pause' : 'Play';
  lastTick = 0; held = 0;
  if (playing) kick();
}

// ---- drawing -------------------------------------------------------------------
const fx = (x, d) => (Number.isFinite(x) ? x.toFixed(d) : String(x));
const ms = (s) => Math.round(s * 1000);
const pc = (x) => (x * 100).toFixed(x > 0.999 || x < 0.001 ? 0 : 1) + '%';
const num = (v) => { const a = Math.abs(v); return a >= 1e4 ? (v / 1000).toFixed(0) + 'k' : a >= 1000 ? (v / 1000).toFixed(1) + 'k' : a >= 10 ? v.toFixed(0) : a >= 1 ? v.toFixed(1) : v.toFixed(2); };
const BAD = '#FF4D5E';
const okc = (ok) => (ok ? C.blue : BAD);

function draw(v, now) {
  drawCurve(v, now);
  drawDetail(v, now);
  drawScope(v, now);
}

function frameBox(g, w, hh, title) {
  g.clearRect(0, 0, w, hh);
  g.font = '10px ' + MONO; g.fillStyle = C.muted; g.textAlign = 'left'; g.textBaseline = 'top';
  g.fillText(title, 10, 8);
}

// The ride's time from the origin: the samples step runs a cursor along them.
const t0of = () => (cfg ? cfg.t0 : 0);
function rideT(now) {
  const T = (R.n - 1) / 1000 - t0of();
  return reduced.matches ? 0 : ((now - ride0) / 1000) % T;
}

function drawCurve(v, now) {
  const { g, w, h: hh } = api.fit(el.curve);
  frameBox(g, w, hh, 'TIME S · POSITION');
  const x0 = 44, x1 = w - 12, y0 = 26, y1 = hh - 22;
  const tEnd = v.tEnd;
  const lo = cfg ? cfg.lo : 0, hi = cfg ? cfg.hi : 1;
  const X = (t) => x0 + (t / tEnd) * (x1 - x0), Y = (p) => y1 - (p - lo + 0.06) / (hi - lo + 0.12) * (y1 - y0);
  g.strokeStyle = C.l2; g.lineWidth = 1; g.setLineDash([4, 4]);
  for (const p of [lo, hi]) { g.beginPath(); g.moveTo(x0, Y(p) + 0.5); g.lineTo(x1, Y(p) + 0.5); g.stroke(); }
  g.setLineDash([]);
  g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'right'; g.textBaseline = 'middle';
  g.fillText(String(+lo.toFixed(2)), x0 - 6, Y(lo)); g.fillText(String(+hi.toFixed(2)), x0 - 6, Y(hi));
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (const t of [0, tEnd / 2, tEnd]) g.fillText(fx(t + t0of(), 2) + ' s', Math.min(x1 - 14, Math.max(x0 + 14, X(t))), y1 + 6);
  const k = v.k;
  // the 1 ms samples, revealed left to right
  if (v.reveal > 0) {
    const t0s = t0of(), n = R.n, cut = v.reveal * tEnd;
    g.save(); g.beginPath(); g.rect(x0, 0, X(cut) - x0 + 2, hh); g.clip();
    g.strokeStyle = C.blue; g.lineWidth = 2.5; g.beginPath();
    let first = true;
    for (let i = Math.max(0, Math.round(t0s * 1000)); i < n; i++) {
      const t = i / 1000 - t0s; if (t > tEnd) break;
      first ? g.moveTo(X(t), Y(R.P[i])) : g.lineTo(X(t), Y(R.P[i])); first = false;
    }
    g.stroke(); g.restore();
    if (v.kind === 'samples' && to.kind === 'samples') {
      const t = rideT(now), i = Math.min(n - 1, Math.round((t + t0s) * 1000));
      g.strokeStyle = C.t3; g.setLineDash([2, 3]); g.beginPath(); g.moveTo(X(t), y0); g.lineTo(X(t), y1); g.stroke(); g.setLineDash([]);
      dot(g, X(t), Y(R.P[i]), 5, C.blue);
    }
  }
  if (!k) return;
  const n = k.t.length, P = (i) => k.p[i] + k.dp[i];
  // focus box
  if (v.focus) {
    const [ta, tb] = v.focus;
    g.strokeStyle = C.blue; g.setLineDash([3, 3]); g.lineWidth = 1.2;
    g.strokeRect(X(ta) - 8, y0 - 2, X(tb) - X(ta) + 16, y1 - y0 + 4); g.setLineDash([]);
  }
  // chords
  g.strokeStyle = C.l4; g.lineWidth = 1.2; g.setLineDash([4, 4]);
  g.globalAlpha = 1 - 0.6 * (v.aHnd || 0);
  g.beginPath();
  for (let i = 0; i + 1 < n; i++) { g.moveTo(X(k.t[i]), Y(P(i))); g.lineTo(X(k.t[i + 1]), Y(P(i + 1))); }
  g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  // the render: one cubic per piece, from its control points
  if (v.aHnd > 0) {
    for (let i = 0; i + 1 < n; i++) {
      const T = k.t[i + 1] - k.t[i];
      const c = [k.t[i], P(i), k.t[i] + k.eo[i] * T, P(i) + k.vel[i] * k.eo[i] * T, k.t[i + 1] - k.ei[i + 1] * T, P(i + 1) - k.vel[i + 1] * k.ei[i + 1] * T, k.t[i + 1], P(i + 1)];
      const bad = k.inf[i + 1], focus = v.focus && Math.abs(v.focus[0] - k.t[i]) < 1e-9;
      g.globalAlpha = v.aHnd * (1 - 0.65 * v.reveal);
      g.strokeStyle = bad ? BAD : C.blue; g.lineWidth = focus ? 3 : 2;
      g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.bezierCurveTo(X(c[2]), Y(c[3]), X(c[4]), Y(c[5]), X(c[6]), Y(c[7])); g.stroke();
      // handles
      g.strokeStyle = C.violet; g.lineWidth = 1; g.globalAlpha = v.aHnd * (focus ? 1 : 0.55) * (1 - v.reveal);
      g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.lineTo(X(c[2]), Y(c[3])); g.moveTo(X(c[6]), Y(c[7])); g.lineTo(X(c[4]), Y(c[5])); g.stroke();
      dot(g, X(c[2]), Y(c[3]), 2.5, C.violet); dot(g, X(c[4]), Y(c[5]), 2.5, C.violet);
      g.globalAlpha = 1;
    }
  }
  // knots: authored, and where a trim moved them
  for (let i = 0; i < n; i++) {
    const x = X(k.t[i]), ya = Y(k.p[i]), yr = Y(P(i)), prov = i >= (v.me || n);
    if (Math.abs(k.dp[i]) > 1e-7) {
      g.strokeStyle = C.t3; g.setLineDash([2, 2]); g.lineWidth = 1.2;
      g.beginPath(); g.arc(x, ya, 5.5, 0, Math.PI * 2); g.stroke(); g.setLineDash([]);
      g.strokeStyle = SPEND(); g.beginPath(); g.moveTo(x, ya); g.lineTo(x, yr); g.stroke();
    }
    g.fillStyle = C.sunken; g.strokeStyle = prov ? C.muted : k.inf[i] ? BAD : C.text; g.lineWidth = 2;
    if (prov) g.setLineDash([2, 2]);
    g.beginPath(); g.arc(x, yr, i === 0 ? 4.5 : 5, 0, Math.PI * 2); g.fill(); g.stroke(); g.setLineDash([]);
    // its angle
    if (v.aAng > 0) {
      const sx = (x1 - x0) / tEnd, sy = (y1 - y0) / (hi - lo + 0.12), dx = 1, dy = -k.vel[i] * sy / sx, L = 16 / Math.hypot(dx, dy);
      g.globalAlpha = v.aAng * (1 - v.reveal); g.strokeStyle = k.cls[i] === 3 ? C.blue : C.text; g.lineWidth = 2.2; g.lineCap = 'round';
      g.beginPath(); g.moveTo(x - dx * L, yr - dy * L); g.lineTo(x + dx * L, yr + dy * L); g.stroke(); g.lineCap = 'butt'; g.globalAlpha = 1;
    }
    if (v.aCls > 0 && k.cls[i] >= 0 && n <= 40) {
      g.globalAlpha = v.aCls * (1 - v.reveal); g.fillStyle = k.cls[i] === 3 ? C.blue : C.t3; g.font = '10px ' + MONO; g.textAlign = 'center';
      const up = i > 0 && i + 1 < n && P(i) > (P(i - 1) + P(i + 1)) / 2;
      g.textBaseline = up ? 'bottom' : 'top';
      g.fillText(i === 0 ? 'origin' : prov ? 'provisional' : CLS[k.cls[i]], x, yr + (up ? -9 : 9)); g.globalAlpha = 1;
    }
  }
  // the solved knots: a corner ramp's span
  if (v.aRes > 0 && v.emit) {
    g.globalAlpha = v.aRes;
    for (const e of v.emit) {
      const t = e.t - t0of();
      if (e.fl & 1) { g.strokeStyle = C.hi; g.lineWidth = 3; g.beginPath(); g.moveTo(X(e.head - cfg.t0), Y(e.knotP)); g.lineTo(X(t), Y(e.p)); g.stroke(); }
      dot(g, X(t), Y(e.p), 3.5, e.fl & 2 ? BAD : C.blue);
    }
    g.globalAlpha = 1;
  }
}
const SPEND = () => 'rgba(77,166,255,0.55)';
function dot(g, x, y, r, fill) { g.fillStyle = fill; g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill(); }

function drawDetail(v, now) {
  const { g, w, h: hh } = api.fit(el.detail);
  const s = steps[idx], d = v.detail;
  const title = d.kind === 'piece' ? `DETAIL · PIECE ${s.focus}→${s.focus + 1}` : d.kind === 'built' ? 'DETAIL · BUILT, WORST RATIO PER PIECE' : d.kind === 'result' ? 'DETAIL · SOLVED KNOTS' : d.kind === 'samples' ? 'DETAIL · 1 MS GRID, 128 MS AHEAD' : 'DETAIL · KNOTS';
  frameBox(g, w, hh, title);
  g.globalAlpha = v.fade ?? 1;
  if (d.kind === 'piece') detailPiece(g, w, hh, d, s);
  else if (d.kind === 'table') detailTable(g, w, hh, v, s);
  else if (d.kind === 'built') detailBuilt(g, w, hh, d);
  else if (d.kind === 'result') detailResult(g, w, hh, v);
  else if (d.kind === 'samples') detailStrip(g, w, hh, now);
  else { g.fillStyle = C.t2; g.font = '13px ' + MONO; g.textAlign = 'left'; g.textBaseline = 'top'; s.lines.forEach((l, i) => g.fillText(l[0].trim(), 12, 32 + i * 20)); }
  g.globalAlpha = 1;
}

function detailPiece(g, w, hh, d, s) {
  const ruler = !!d.cells, x0 = 46, x1 = w - 14, y0 = 30, y1 = hh - (ruler ? 96 : 26);
  const [ta, tb, pa, pb] = d.win;
  const X = (t) => x0 + (t - ta) / (tb - ta) * (x1 - x0), Y = (p) => y1 - (p - pa) / (pb - pa) * (y1 - y0);
  const c = d.cp;
  g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'right'; g.textBaseline = 'middle';
  g.fillText(fx(pb, 3), x0 - 6, y0 + 4); g.fillText(fx(pa, 3), x0 - 6, y1 - 4);
  // the knot's time stays
  g.strokeStyle = C.l3; g.setLineDash([2, 4]); g.lineWidth = 1;
  g.beginPath(); g.moveTo(X(c[6]), y0); g.lineTo(X(c[6]), y1); g.stroke(); g.setLineDash([]);
  if (d.bracket) {
    g.strokeStyle = C.t2; g.lineWidth = 1.5;
    for (const p of d.bracket) { g.beginPath(); g.moveTo(X(c[6]) - 22, Y(p)); g.lineTo(X(c[6]) + 22, Y(p)); g.stroke(); }
  }
  if (d.gA > 0.01) {
    g.globalAlpha = d.gA; g.strokeStyle = C.t3; g.setLineDash([2, 2]); g.lineWidth = 1.5;
    g.beginPath(); g.arc(X(d.ghost[0]), Y(d.ghost[1]), 7, 0, Math.PI * 2); g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  }
  g.strokeStyle = C.t3; g.lineWidth = 1; g.setLineDash([3, 3]);
  g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); for (let j = 2; j < 8; j += 2) g.lineTo(X(c[j]), Y(c[j + 1])); g.stroke(); g.setLineDash([]);
  g.strokeStyle = d.verdict === false ? BAD : C.blue; g.lineWidth = 3;
  g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.bezierCurveTo(X(c[2]), Y(c[3]), X(c[4]), Y(c[5]), X(c[6]), Y(c[7])); g.stroke();
  g.strokeStyle = C.violet; g.lineWidth = 2;
  g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.lineTo(X(c[2]), Y(c[3])); g.moveTo(X(c[6]), Y(c[7])); g.lineTo(X(c[4]), Y(c[5])); g.stroke();
  dot(g, X(c[2]), Y(c[3]), 4.5, C.violet); dot(g, X(c[4]), Y(c[5]), 4.5, C.violet);
  for (const j of [0, 6]) { g.fillStyle = C.sunken; g.strokeStyle = C.text; g.lineWidth = 2.2; g.beginPath(); g.arc(X(c[j]), Y(c[j + 1]), 6, 0, Math.PI * 2); g.fill(); g.stroke(); }
  if (d.L) {
    g.font = '11px ' + MONO; g.fillStyle = C.violet; g.textAlign = 'center'; g.textBaseline = 'bottom';
    g.fillText(fx(d.L[0], 3), X(c[2]), Y(c[3]) - 8); g.fillText(fx(d.L[1], 3), X(c[4]), Y(c[5]) - 8);
  }
  if (d.verdict != null && s.kind !== 'memo') pill(g, w - 14, 26, d.verdict ? 'LEGAL' : 'OVER', d.verdict);
  if (!ruler) return;
  // the length factors: v, a, j, and the verdict per factor judged
  const rx = (k) => x0 + (k - 0.2) / 1.8 * (x1 - x0), ry0 = hh - 80;
  g.font = '10px ' + MONO; g.fillStyle = C.muted; g.textAlign = 'right'; g.textBaseline = 'middle';
  ['v', 'a', 'j', 'all'].forEach((nm, row) => g.fillText(nm, x0 - 8, ry0 + row * 12 + 5));
  const cw = Math.max(3, (x1 - x0) / 37 - 2);
  KGRID.forEach((k, ci) => {
    const cell = d.cells[ci];
    for (let row = 0; row < 4; row++) {
      g.fillStyle = !cell ? C.l2 : row < 3 ? okc(cell[row] <= 1 + TOL) : okc(cell[3] === 0);
      g.fillRect(rx(k) - cw / 2, ry0 + row * 12, cw, 10);
    }
  });
  g.fillStyle = C.text; const mx = rx(Math.min(2, Math.max(0.2, d.kmark)));
  g.beginPath(); g.moveTo(mx, ry0 - 2); g.lineTo(mx - 5, ry0 - 10); g.lineTo(mx + 5, ry0 - 10); g.closePath(); g.fill();
  g.textAlign = 'center'; g.textBaseline = 'top'; g.fillStyle = C.muted;
  for (const k of [0.2, 0.5, 1, 1.5, 2]) g.fillText(String(k), rx(k), ry0 + 50);
  g.textAlign = 'left'; g.fillText('k, the length factor · grey: not judged', x0, ry0 + 64);
}

function pill(g, xr, y, text, ok) {
  g.font = '600 11px ' + SANS; const tw = g.measureText(text).width + 18;
  g.fillStyle = ok ? C.blue : BAD; roundRect(g, xr - tw, y - 2, tw, 20, 10); g.fill();
  g.fillStyle = C.sunken; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(text, xr - tw / 2, y + 8);
}
function roundRect(g, x, y, w, h, r) { g.beginPath(); g.moveTo(x + r, y); g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r); g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath(); }

// The knots as they stand: the column this step computes is the bright one.
function detailTable(g, w, hh, v, s) {
  const k = v.k, n = k.t.length;
  const hot = { knots: [1, 2], classify: [3], angles: [4], sweeps: [4], solved: [5, 6], round: [7], pass: [] }[s.kind] || [];
  const prev = steps.slice(0, idx).reverse().find((x) => x.snap && x.snap !== s.snap)?.snap;
  const cols = [['k', 0.06], ['t s', 0.2], ['p', 0.35], ['class', 0.38], ['angle', 0.65], ['in', 0.77], ['out', 0.88], ['trim', 0.99]].map(([nm, f]) => [nm, Math.round(f * (w - 16))]);
  const rows = Math.max(1, Math.floor((hh - 52) / 17));
  g.font = '10px ' + MONO; g.textBaseline = 'top';
  cols.forEach(([nm, x], ci) => { g.fillStyle = hot.includes(ci) ? C.blue : C.muted; g.textAlign = ci === 3 ? 'left' : 'right'; g.fillText(nm, x, 28); });
  const start = Math.max(0, Math.min(n - rows, (s.focus ?? 0) - Math.floor(rows / 2)));
  g.font = '12px ' + MONO;
  for (let r = 0; r < Math.min(rows, n); r++) {
    const i = start + r, y = 46 + r * 17;
    const vals = [String(i), fx(k.t[i], 3), fx(k.p[i], 4), i === 0 ? 'origin' : k.cls[i] >= 0 ? CLS[k.cls[i]] : '', fx(k.vel[i], 3),
      fx(k.ei[i], 3), fx(k.eo[i], 3), k.dp[i] ? fx(k.dp[i], 4) : ''];
    cols.forEach(([, x], ci) => {
      const moved = prev && ((ci === 4 && prev.vel[i] !== k.vel[i]) || (ci === 7 && prev.dp[i] !== k.dp[i]));
      g.fillStyle = ci === 7 && k.inf[i] ? BAD : moved && hot.includes(ci) ? C.text : hot.includes(ci) ? C.blue : C.t3;
      if (ci >= 4 && s.stage < (ci === 4 ? 2 : 4)) return;
      g.textAlign = ci === 3 ? 'left' : 'right';
      g.fillText(vals[ci], x, y);
    });
  }
  if (n > rows) { g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'left'; g.fillText(`${start + 1}–${start + rows} of ${n}`, 12, hh - 16); }
}

function detailBuilt(g, w, hh, d) {
  const x0 = 46, x1 = w - 14, y0 = 34, y1 = hh - 26, top = Math.max(1.25, ...d.bars.map((b) => Math.min(3, b[1])));
  const Y = (r) => y1 - Math.min(r, top) / top * (y1 - y0), bw = Math.max(2, (x1 - x0) / d.bars.length - 2);
  g.strokeStyle = C.t3; g.setLineDash([4, 4]); g.beginPath(); g.moveTo(x0, Y(1)); g.lineTo(x1, Y(1)); g.stroke(); g.setLineDash([]);
  g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'right'; g.textBaseline = 'middle'; g.fillText('×1', x0 - 6, Y(1));
  d.bars.forEach(([r, x], i) => {
    const bx = x0 + i * (bw + 2);
    g.fillStyle = okc(x <= 1 + TOL); g.fillRect(bx, Y(x), bw, y1 - Y(x));
    if (x > 1 + TOL && d.bars.length <= 24) { g.fillStyle = BAD; g.textAlign = 'center'; g.textBaseline = 'bottom'; g.fillText('×' + fx(x, 2), bx + bw / 2, Y(x) - 2); }
  });
  g.fillStyle = C.muted; g.textAlign = 'left'; g.textBaseline = 'top'; g.fillText('pieces in order; the dashed line is every ceiling', x0, y1 + 6);
}

function detailResult(g, w, hh, v) {
  const e = v.emit, cols = [['k', 0.06], ['t s', 0.22], ['p', 0.38], ['v', 0.53], ['a', 0.66], ['share', 0.81], ['worst', 0.96]].map(([nm, f]) => [nm, Math.round(f * (w - 16))]);
  const rows = Math.max(1, Math.floor((hh - 52) / 17));
  g.font = '10px ' + MONO; g.textBaseline = 'top'; g.textAlign = 'right';
  cols.forEach(([nm, x]) => { g.fillStyle = C.muted; g.fillText(nm, x, 28); });
  g.font = '12px ' + MONO;
  e.slice(0, rows).forEach((q, r) => {
    const y = 46 + r * 17, vals = [String(q.r), fx(q.t, 3), fx(q.p, 4), fx(q.v, 3), fx(q.a, 1), pc(q.share), fx(q.worst, 3)];
    cols.forEach(([, x], ci) => { g.fillStyle = ci === 6 && q.fl & 2 ? BAD : ci === 5 && q.share < 0.999 ? C.text : C.t3; g.fillText(vals[ci], x, y); });
  });
  if (e.length > rows) { g.fillStyle = C.muted; g.font = '10px ' + MONO; g.textAlign = 'left'; g.fillText(`${rows} of ${e.length}`, 12, hh - 16); }
}

function detailStrip(g, w, hh, now) {
  const t = rideT(now), ic = Math.round((t + t0of()) * 1000), x0 = 16, x1 = w - 16, y0 = 36, y1 = hh - 40;
  const pxms = (x1 - x0) / 160, n0 = ic - 32;
  let pmin = Infinity, pmax = -Infinity;
  for (let k = 0; k <= 160; k++) { const i = n0 + k; if (i >= 0 && i < R.n) { pmin = Math.min(pmin, R.P[i]); pmax = Math.max(pmax, R.P[i]); } }
  const span = Math.max(0.02, pmax - pmin), mid = (pmax + pmin) / 2, Y = (p) => (y0 + y1) / 2 - (p - mid) / span * (y1 - y0) * 0.9;
  g.fillStyle = 'rgba(77,166,255,0.08)'; g.fillRect(x0 + 32 * pxms, y0, 128 * pxms, y1 - y0);
  for (let k = 0; k <= 160; k++) {
    const i = n0 + k; if (i < 0 || i >= R.n) continue;
    g.globalAlpha = k < 32 ? 0.3 : 1;
    dot(g, x0 + k * pxms, Y(R.P[i]), 1.6, C.blue);
  }
  g.globalAlpha = 1;
  g.strokeStyle = C.text; g.beginPath(); g.moveTo(x0 + 32 * pxms, y0); g.lineTo(x0 + 32 * pxms, y1); g.stroke();
  g.fillStyle = C.t2; g.font = '12px ' + MONO; g.textAlign = 'left'; g.textBaseline = 'top';
  const i = Math.max(0, Math.min(R.n - 1, ic));
  g.fillText(`${i} ms   p ${fx(R.P[i], 4)}   v ${fx(R.V[i], 3)}   a ${fx(R.A[i], 1)}`, x0, y1 + 12);
}

function drawScope(v, now) {
  const { g, w, h: hh } = api.fit(el.scope);
  const sc = v.scope;
  frameBox(g, w, hh, sc.kind === 'u' ? 'SCOPE · V, A, J OVER U, THE CURVE PARAMETER' : sc.kind === 't' ? 'SCOPE · V, A, J OVER TIME, 1 MS SAMPLES' : 'SCOPE');
  if (sc.kind === 'none') { g.fillStyle = C.muted; g.font = '12px ' + SANS; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('v, a and j show when a piece is judged', w / 2, hh / 2); return; }
  const rows = ['v', 'a', 'j'], x0 = 40, x1 = w - (w > 520 ? 150 : 84), rh = (hh - 40) / 3;
  const o = api.state().o, lims = sc.kind === 'u' ? sc.lim : [o.vmax, o.amax, o.jmax];
  rows.forEach((nm, r) => {
    const yc = 30 + rh * r + rh / 2, arr = sc.kind === 'u' ? [sc.V, sc.A, sc.J][r] : [R.V, R.A, R.J][r];
    let peak = lims[r];
    for (const x of arr) if (Number.isFinite(x)) peak = Math.max(peak, Math.abs(x));
    const sy = (rh / 2 - 4) / (peak * 1.05), Y = (x) => yc - x * sy;
    g.strokeStyle = C.l2; g.beginPath(); g.moveTo(x0, yc); g.lineTo(x1, yc); g.stroke();
    g.strokeStyle = C.t3; g.setLineDash([4, 4]);
    for (const sg of [1, -1]) { g.beginPath(); g.moveTo(x0, Y(sg * lims[r])); g.lineTo(x1, Y(sg * lims[r])); g.stroke(); }
    g.setLineDash([]);
    g.fillStyle = C.text; g.font = '13px ' + MONO; g.textAlign = 'left'; g.textBaseline = 'middle'; g.fillText(nm, 12, yc);
    const lim = lims[r] * (1 + TOL);
    if (sc.kind === 'u') {
      const N = arr.length, X = (u) => x0 + u * (x1 - x0);
      g.lineWidth = 2;
      for (let i = 1; i < N; i++) {
        g.strokeStyle = Math.abs(arr[i]) > lim || Math.abs(arr[i - 1]) > lim ? BAD : C.blue;
        g.beginPath(); g.moveTo(X((i - 1) / (N - 1)), Y(arr[i - 1])); g.lineTo(X(i / (N - 1)), Y(arr[i])); g.stroke();
      }
      if (sc.dots && v === to) sc.dots.forEach((d) => { g.fillStyle = Math.abs(d[r + 1]) > lim ? BAD : C.sunken; g.strokeStyle = C.text; g.lineWidth = 1.4; g.beginPath(); g.arc(X(d[0]), Y(d[r + 1]), 3, 0, Math.PI * 2); g.fill(); g.stroke(); });
      if (sc.roots) sc.roots[r].forEach(([u, x]) => { g.fillStyle = C.violet; g.beginPath(); g.moveTo(X(u), Y(x) - 4); g.lineTo(X(u) - 4, Y(x) - 11); g.lineTo(X(u) + 4, Y(x) - 11); g.closePath(); g.fill(); });
    } else {
      const n = R.n, T = (n - 1) / 1000, X = (t) => x0 + t / T * (x1 - x0);
      g.lineWidth = 1.5; g.strokeStyle = C.blue; g.beginPath();
      for (let i = 0; i < n; i += 2) i ? g.lineTo(X(i / 1000), Y(arr[i])) : g.moveTo(X(0), Y(arr[0]));
      g.stroke();
      if (v.kind === 'samples') { const t = rideT(now) + t0of(); g.strokeStyle = C.t3; g.beginPath(); g.moveTo(X(t), yc - rh / 2); g.lineTo(X(t), yc + rh / 2); g.stroke(); }
    }
    // the peak against its ceiling: the number that decides
    const ratio = sc.kind === 'u' ? sc.ratio[r] : [R.max.v / o.vmax, R.max.a / o.amax, R.max.j / o.jmax][r];
    const over = ratio > 1 + TOL;
    g.textAlign = 'left'; g.fillStyle = over ? BAD : C.blue; g.font = (over ? '600 ' : '') + '14px ' + MONO;
    g.fillText('×' + fx(ratio, 3), x1 + 10, yc - 7);
    g.fillStyle = C.muted; g.font = '10px ' + MONO; g.fillText(num(ratio * lims[r]) + ' / ' + num(lims[r]), x1 + 10, yc + 9);
  });
}

// ---- the log -------------------------------------------------------------------
let typing = false, typeFrom = 0;
function log() {
  const lines = [];
  for (let i = Math.max(0, idx - 12); i <= idx; i++) for (const l of steps[i].lines) lines.push([...l, i === idx]);
  const show = lines.slice(-11);
  el.log.innerHTML = show.map(([t, c, now]) => `<li class="${c || ''}${now ? ' now' : ''}" data-t="${esc(t)}">${now && !reduced.matches ? '' : esc(t)}</li>`).join('');
  typing = !reduced.matches; typeFrom = performance.now();
}
function typeLog(now) {
  if (!typing) return;
  const li = [...el.log.querySelectorAll('li.now')];
  let done = true;
  li.forEach((e, i) => {
    const t = e.dataset.t, n = Math.floor(Math.max(0, (now - typeFrom - i * 90) / 260) * t.length);
    e.textContent = t.slice(0, Math.max(0, Math.min(t.length, n)));
    if (n < t.length) done = false;
  });
  if (done) typing = false;
}
const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;');

// ---- the step list (rows are drawn only where the list is scrolled) -------------
const ROW = 24;
function list(scrollTo) {
  const box = el.list, inner = el.listInner;
  inner.style.height = steps.length * ROW + 'px';
  if (scrollTo) {
    const y = idx * ROW;
    if (y < box.scrollTop || y + ROW > box.scrollTop + box.clientHeight) box.scrollTop = y - box.clientHeight / 2;
  }
  const a = Math.max(0, Math.floor(box.scrollTop / ROW) - 4), b = Math.min(steps.length, a + Math.ceil(box.clientHeight / ROW) + 8);
  let html = '';
  for (let i = a; i < b; i++) {
    const s = steps[i], v = s.judge ? s.judge.legal : s.fit ? s.fit.legal : s.end ? s.end.legal : null;
    html += `<button class="strow d${s.depth}${i === idx ? ' cur' : ''}" style="top:${i * ROW}px" data-i="${i}"${i === idx ? ' aria-current="step"' : ''}>`
      + `<span class="n mono">${i + 1}</span>${v == null ? '' : `<span class="mark ${v ? 'ok' : 'bad'}"></span>`}${esc(s.title)}</button>`;
  }
  inner.innerHTML = html;
}

// ---- the panel -------------------------------------------------------------------
function build() {
  host.innerHTML = `
  <div class="sthead">
    <h2>Planner steps <span class="muted small">every number is the planner's own, read from its trace</span></h2>
    <div class="bar">
      <button class="btn" id="st-prev" title="Previous step (Left)">Prev</button>
      <button class="btn primary" id="st-play" title="Play / pause the steps (Space)">Play</button>
      <button class="btn" id="st-next" title="Next step (Right)">Next</button>
      <input id="st-scrub" type="range" min="0" max="0" step="1" value="0" aria-label="Step">
      <output id="st-pos" class="mono"></output>
      <select id="st-speed" aria-label="Speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select>
      <label class="inline" title="Every factor and trim test in every round and pass, not only the first round"><input type="checkbox" id="st-all"> every judge</label>
      <span id="st-solvewrap" hidden><select id="st-solve" aria-label="Solve"></select></span>
    </div>
    <p class="sttitle" id="st-title"></p>
  </div>
  <div class="stgrid">
    <div class="stpanel stcurve"><canvas id="st-curve" aria-label="The render: knots, angles and handles"></canvas></div>
    <div class="stpanel stdetail"><canvas id="st-detail" aria-label="The current piece or knot, up close"></canvas></div>
    <div class="stpanel stscope"><canvas id="st-scope" aria-label="Velocity, acceleration and jerk against the ceilings"></canvas></div>
    <div class="stpanel stlog"><p class="ptitle mono">COMPUTING</p><ol id="st-log" class="mono" aria-live="polite"></ol></div>
  </div>
  <div class="stlist" id="st-list" aria-label="Steps"><div id="st-list-inner"></div></div>
  <p class="muted small" id="st-lost"></p>`;
  const q = (s) => host.querySelector(s);
  el = { prev: q('#st-prev'), play: q('#st-play'), next: q('#st-next'), scrub: q('#st-scrub'), pos: q('#st-pos'), speed: q('#st-speed'), all: q('#st-all'),
    solve: q('#st-solve'), solvewrap: q('#st-solvewrap'), title: q('#st-title'), curve: q('#st-curve'), detail: q('#st-detail'), scope: q('#st-scope'),
    log: q('#st-log'), list: q('#st-list'), listInner: q('#st-list-inner'), lost: q('#st-lost') };
  el.prev.addEventListener('click', () => { setPlaying(false); go(idx - 1); });
  el.next.addEventListener('click', () => { setPlaying(false); go(idx + 1); });
  el.play.addEventListener('click', () => setPlaying(!playing));
  el.scrub.addEventListener('input', () => { setPlaying(false); go(+el.scrub.value); });
  el.all.addEventListener('change', () => { everyJudge = el.all.checked; rebuild(); });
  el.solve.addEventListener('change', () => { solveIx = picked = +el.solve.value; steps = []; rebuild(); });
  el.list.addEventListener('scroll', () => list(false));
  el.listInner.addEventListener('click', (ev) => { const b = ev.target.closest('[data-i]'); if (b) { setPlaying(false); go(+b.dataset.i); } });
  host.tabIndex = -1;
  host.addEventListener('keydown', (ev) => {
    if (ev.target.tagName === 'SELECT' || (ev.target.tagName === 'INPUT' && ev.target.type !== 'range' && ev.target.type !== 'checkbox')) return;
    const k = { ArrowLeft: -1, ArrowRight: 1, Home: -Infinity, End: Infinity }[ev.key];
    if (k != null) { ev.preventDefault(); setPlaying(false); go(k === -Infinity ? 0 : k === Infinity ? steps.length - 1 : idx + k); }
    else if (ev.code === 'Space') { ev.preventDefault(); ev.stopPropagation(); setPlaying(!playing); }
  });
  new ResizeObserver(() => { if (shown) { draw(shown, performance.now()); list(false); } }).observe(host);
  reduced.addEventListener('change', () => kick());
}
