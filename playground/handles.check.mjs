// handles.check.mjs -- serves the playground and drives handles.html headless:
// the page draws the planner's result (its samples and kinetic2_render's knots
// equal kinetic2-trace.wasm's samples and trace records bit for bit, each knot
// on the curve), edits reach the planner, the trace build loads only with the
// planner steps, no console errors, no horizontal scroll at three viewports.
// Run: node playground/handles.check.mjs
//   PLAYWRIGHT_DIR: a node_modules holding playwright (default ../Phosphor/node_modules)
//   SHOTS_DIR: where the screenshots go (default the OS temp dir)
//   KINETIC2_WASM, KINETIC2_TRACE_WASM: the modules (default build-wasm/wasm/)
import { copyFileSync, existsSync, mkdirSync, readFileSync } from 'node:fs';
import { createServer } from 'node:http';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { dirname, extname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
let failures = 0;
const check = (ok, what) => { console.log((ok ? 'ok   ' : 'FAIL ') + what); if (!ok) failures++; };

const pwDir = process.env.PLAYWRIGHT_DIR || resolve(here, '../../Phosphor/node_modules');
let chromium = null;
try { chromium = createRequire(join(pwDir, 'noop.js'))('playwright').chromium; } catch { console.log('FAIL Playwright not found under ' + pwDir + '; set PLAYWRIGHT_DIR'); process.exit(1); }
for (const [env, name] of [['KINETIC2_WASM', 'kinetic2.wasm'], ['KINETIC2_TRACE_WASM', 'kinetic2-trace.wasm']]) {
  const src = process.env[env] || resolve(here, '../build-wasm/wasm/' + name);
  if (existsSync(src)) copyFileSync(src, join(here, name));
  if (!existsSync(join(here, name))) { console.log(`FAIL no ${name}: build the kinetic2_wasm and kinetic2_trace_wasm targets`); process.exit(1); }
}
const shots = process.env.SHOTS_DIR || join(tmpdir(), 'kinetic2-playground');
mkdirSync(shots, { recursive: true });
const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.wasm': 'application/wasm' };
const server = createServer((req, res) => {
  const path = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const file = resolve(here, '.' + path);
  if (!file.startsWith(here) || !existsSync(file)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'content-type': MIME[extname(file)] || 'application/octet-stream' });
  res.end(readFileSync(file));
});
await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
const url = `http://127.0.0.1:${server.address().port}/handles.html`;
const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1280, height: 900 }, colorScheme: 'dark' });
const errors = [], requested = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
page.on('request', (r) => requested.push(new URL(r.url()).pathname));
await page.goto(url);
await page.waitForFunction(() => window.__handles && window.__handles.state().R);

const st = () => page.evaluate(() => {
  const s = window.__handles.state(), D = s.R.D, cls = [0, 0, 0, 0];
  for (const d of D) if (d) cls[d.cls]++;
  const k = s.sel == null ? null : s.knots[s.sel], d = s.sel == null ? null : D[s.sel];
  return { n: s.knots.length, have: D.filter(Boolean).length, cls, sel: s.sel, v: k ? k.v : null, pm: k ? k.pm : null,
    vel: d ? d.vel * (s.cfg.hi - s.cfg.lo) : null, clamped: d ? d.clamped : 0, over: s.R.over,
    inf: D.filter((x) => x && x.inf).length, trimmed: D.filter((x) => x && x.dp !== 0).length,
    nudged: D.filter((x) => x && (Math.abs(x.ei - x.lIn) > 1e-6 || Math.abs(x.eo - x.lOut) > 1e-6)).length };
});
// The page's drawing (kinetic2.wasm) against kinetic2-trace.wasm on the same
// knots: every 1 ms sample equal bit for bit; every knot's kinetic2_render data
// equal bit for bit to the trace records of its last render round (K_KNOT of the
// last odd K_ROUND per run, K_EMIT; wasm/kinetic2_trace.cpp); every knot drawn
// on the curve at its time within kKnotTol (solver.hpp, 1e-3 of the window).
// The trace build goes in as bytes: a fetch here would count as the page's.
const traceB64 = readFileSync(join(here, 'kinetic2-trace.wasm')).toString('base64');
const drawsPlanner = () => page.evaluate(async (b64) => {
  if (!window.__k2t) {
    const { instance } = await WebAssembly.instantiate(Uint8Array.from(atob(b64), (c) => c.charCodeAt(0)), {});
    const K = instance.exports; K._initialize();
    window.__k2t = { K, h: K.kinetic2_create(), dv: new DataView(K.memory.buffer), sBuf: K.malloc(24), kBuf: K.malloc(40), aBuf: K.malloc(24) };
  }
  const REC = 24, K_RUN = 2, K_ROUND = 7, K_EMIT = 14, F = Math.fround;
  const s = window.__handles.state(), R = s.R, W = window.__k2t, K = W.K, TD = new Array(s.knots.length).fill(null);
  const readTrace = (live, first) => {
    const n = K.kinetic2_trace_count(), b0 = K.kinetic2_trace_records() >> 2;
    const i32 = new Int32Array(K.memory.buffer), f32 = new Float32Array(K.memory.buffer);
    let run = null;
    const flush = () => {
      if (!run || run.fin < 0) return;
      for (let r = first ? 0 : 2; r < run.me; r++) {
        const ix = r === 0 ? 0 : live[run.base + r - 1];
        if (ix == null) continue;
        const o = b0 + (run.fin + 1 + r) * REC, fl = i32[o + 2], f = (x) => f32[o + 4 + x];
        const d = { p: F(f(1) + f(2)), dp: f(2), vel: f(3), lIn: f(4), lOut: f(5), ei: f(6), eo: f(7),
          TL: r > 0 ? F(f(0) - f32[o - REC + 4]) : 0, TR: r + 1 < run.m ? F(f32[o + REC + 4] - f(0)) : 0,
          share: 1, worst: 0, rampT: 0, cls: fl & 3, g2: (fl >> 2) & 1, corner: 0, inf: 0, clamped: 0 };
        const e = run.emit[r];
        if (e != null) {
          const q = b0 + e * REC, ef = i32[q + 2];
          Object.assign(d, { corner: ef & 1, inf: (ef >> 1) & 1, clamped: (ef >> 4) & 1, share: f32[q + 8], worst: f32[q + 9], rampT: f32[q + 12] });
        }
        TD[ix] = d;
      }
    };
    for (let r = 0; r < n; r++) {
      const o = b0 + r * REC, kind = i32[o];
      if (kind === K_RUN) { flush(); run = { m: i32[o + 1], me: i32[o + 2], base: i32[o + 3], fin: -1, emit: [] }; }
      else if (kind === K_ROUND && (i32[o + 1] & 1)) run.fin = r;
      else if (kind === K_EMIT) run.emit[i32[o + 1]] = r;
    }
    flush();
    K.kinetic2_trace_clear();
  };
  K.kinetic2_trace_clear();
  const r = window.__handles.replay(W, (_, live, first) => readTrace(live, first));
  let diff = r.n !== R.n ? 1 : 0;
  for (let i = 0; i < R.n && !diff; i++) if (!Object.is(r.P[i], R.P[i]) || !Object.is(r.V[i], R.V[i]) || !Object.is(r.A[i], R.A[i])) diff = i + 1;
  let off = 0, missing = 0, knotDiff = null;
  s.knots.forEach((k, i) => {
    const d = R.D[i], t = TD[i];
    if (!d) { missing++; return; }
    off = Math.max(off, Math.abs(R.P[Math.round(k.t * 1000)] - d.p));
    if (!knotDiff) for (const key of Object.keys(d)) if (!t || !Object.is(d[key], t[key])) { knotDiff = `knot ${i} ${key}: ${d[key]} rendered, ${t && t[key]} traced`; break; }
  });
  return { diff, off, missing, knotDiff, n: R.n, over: R.over };
}, traceB64);
const exact = async (what) => {
  const r = await drawsPlanner();
  check(r.diff === 0 && r.missing === 0 && r.off <= 1e-3 && !r.knotDiff, `${what}: ${r.n} samples equal kinetic2-trace.wasm's bit for bit${r.diff ? ` (first difference at ${r.diff - 1} ms)` : ''}; every knot's render equals its trace records${r.knotDiff ? ` (not: ${r.knotDiff})` : ''}; every knot on the curve within ${r.off.toExponential(1)} of the window${r.missing ? `, ${r.missing} knots without planner data` : ''}`);
};
// The theme (the site's head script and toggle, play.css's tokens): the most
// common opaque color of a canvas, sampled every 7 px, and the theme state.
const dominant = (sel) => page.evaluate((sel) => {
  const cv = document.querySelector(sel), d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data, n = {};
  for (let y = 0; y < cv.height; y += 7) for (let x = 0; x < cv.width; x += 7) {
    const i = (y * cv.width + x) * 4;
    if (d[i + 3] === 255) { const k = '#' + [d[i], d[i + 1], d[i + 2]].map((v) => v.toString(16).padStart(2, '0')).join('').toUpperCase(); n[k] = (n[k] || 0) + 1; }
  }
  return Object.entries(n).sort((a, b) => b[1] - a[1])[0]?.[0];
}, sel);
const themeState = () => page.evaluate(() => ({ theme: document.documentElement.dataset.theme, stored: localStorage.getItem('theme'),
  label: document.querySelector('.theme').getAttribute('aria-label'), bad: getComputedStyle(document.documentElement).getPropertyValue('--bad').trim() }));
const frames = () => page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
const setNum = async (id, v) => { await page.fill(id, String(v)); await page.dispatchEvent(id, 'change'); };

let s = await st();
check(s.n === 23 && s.have === 23, `page: the built-in script, ${s.n} knots, every one with the planner's data`);
check(s.cls.join() === '2,8,12,1', `page: the planner's classes, end ${s.cls[0]}, holds ${s.cls[1]}, crests ${s.cls[2]}, through ${s.cls[3]}`);
check(await page.evaluate(() => !document.querySelector('#k-type, #k-lin, #k-lout, #k-vin, #k-vout, #c-hold, #steps-div')), 'page: no control the planner takes no input for');
await exact('built-in at the defaults');

// select a crest and drag its out handle: the angle is set by hand and the planner takes it
const i = 2;
const k = await page.evaluate((i) => window.__handles.xy(i), i);
await page.mouse.click(k.x, k.y);
s = await st(); check(s.sel === i, `page: clicking a knot selects it (${s.sel})`);
const hnd = await page.evaluate((i) => window.__handles.handleXY(i, 'out'), i);
await page.mouse.move(hnd.x, hnd.y); await page.mouse.down(); await page.mouse.move(hnd.x + 15, hnd.y - 25, { steps: 5 }); await page.mouse.up();
await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
s = await st(); check(s.v != null && (s.clamped || Math.abs(s.vel - s.v) <= 1e-3 * Math.max(1, Math.abs(s.v))), `page: dragging a handle sets the angle by hand and the planner renders it (${s.v && s.v.toFixed(1)} set, ${s.vel && s.vel.toFixed(1)} drawn)`);
const k7 = await page.evaluate(() => window.__handles.xy(7));
await page.mouse.move(k7.x, k7.y); await page.mouse.down(); await page.mouse.move(k7.x, k7.y + 30, { steps: 4 }); await page.mouse.up();
await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
await exact('an edited script (an angle set, a knot moved)');
await page.mouse.click(k.x, k.y);
await page.click('#k-auto');
s = await st(); check(s.v == null && s.pm == null, 'page: Free this knot clears the angle and the move');
await page.click('#autoall');

// tight ceilings: the planner nudges and trims, and the page draws exactly that
await setNum('#c-vmax', 400);
s = await st(); check(s.nudged + s.trimmed > 0, `page: a 400 mm/s ceiling, ${s.nudged} handles nudged, ${s.trimmed} knots trimmed`);
await exact('speed 400 mm/s');
await setNum('#c-vmax', 1000); await setNum('#c-jmax', 1e6);
await exact('jerk 1e6 mm/s³');
await setNum('#c-jmax', 5e6);
await page.evaluate(() => window.__handles.setWindow(100, 400));
const win = await page.evaluate(() => { const st = window.__handles.state(); return { lo: st.cfg.lo, hi: st.cfg.hi, p0: st.knots[0].p, pmax: Math.max(...st.knots.map((k) => k.p)) }; });
check(win.lo === 100 && win.hi === 400 && win.p0 === 130 && win.pmax === 400, `page: the script rescaled into a 300 mm window (first knot ${win.p0} mm, highest ${win.pmax} mm)`);
await exact('a 300 mm window at 1000 mm/s');
await page.evaluate(() => { document.querySelector('#c-smooth').value = '1'; document.querySelector('#c-smooth').dispatchEvent(new Event('input')); });
await exact('the 300 mm window at smoothness 1');
await page.evaluate(() => { document.querySelector('#c-smooth').value = '0'; document.querySelector('#c-smooth').dispatchEvent(new Event('input')); window.__handles.setWindow(0, 100); });

// a script longer than the planner's 64 pending knots: submitted as they retire
await page.evaluate(() => { const a = []; let t = 0; for (let i = 0; i < 300; i++) { a.push({ at: t, pos: i % 2 ? 90 : 10 + (i % 7) * 5 }); t += 150 + (i % 5) * 60; } window.__handles.load({ actions: a }, 'long'); });
s = await st(); check(s.n === 300 && s.have === 300, `page: a 300-knot script, every knot with the planner's data`);
await exact('the 300-knot script');
await page.click('#sample');

await page.click('#play');
await page.waitForTimeout(400);
const play = await page.evaluate(() => { const st = window.__handles.state(); return { playing: st.playing, cursor: st.cursor }; });
check(play.playing && play.cursor > 0.2, `page: playback runs (cursor at ${(play.cursor * 1000).toFixed(0)} ms after 400 ms)`);
await page.click('#play');

// the theme: the toggle flips it live and keeps the choice only while it differs
// from the system's (the site's localStorage.theme); the system's scheme is followed
let th = await themeState();
check(th.theme === 'dark' && th.stored === null && th.bad === '#FF4D5E' && await dominant('#plot') === '#08090B', `theme: dark under a dark system, the plot in the dark palette (${th.theme})`);
await page.click('.theme'); await frames();
th = await themeState();
const lightPlot = await dominant('#plot');
check(th.theme === 'light' && th.stored === 'light' && th.label === 'Switch to dark theme' && th.bad === '#B8182A' && lightPlot === '#F3F2EE',
  `theme: the toggle turns it light without a reload, stored, the plot redrawn (${lightPlot}), over a ceiling in #B8182A`);
await exact('the light theme');
await page.click('.theme'); await frames();
th = await themeState();
check(th.theme === 'dark' && th.stored === null && await dominant('#plot') === '#08090B', 'theme: toggled back to the system\'s dark, nothing stored');
await page.emulateMedia({ colorScheme: 'light' }); await frames();
th = await themeState();
check(th.theme === 'light' && th.stored === null && await dominant('#plot') === '#F3F2EE', 'theme: a light system scheme turns the page light live');
await page.emulateMedia({ colorScheme: 'dark' }); await frames();

// the viewports, with and without the planner steps, in both themes
const fits = async (label) => {
  for (const scheme of ['dark', 'light']) {
    await page.emulateMedia({ colorScheme: scheme });
    for (const [w, h] of [[1280, 720], [1920, 1080], [390, 844]]) {
      await page.setViewportSize({ width: w, height: h });
      await frames();
      check(await page.evaluate(() => document.documentElement.scrollWidth <= document.documentElement.clientWidth && document.querySelector('#stripbox').getBoundingClientRect().height > 100),
        `page${label} at ${w}x${h}, ${scheme}: no horizontal scroll, the strips shown`);
      await page.screenshot({ path: join(shots, `handles${label ? '-steps' : ''}-${scheme}-${w}.png`), fullPage: true });
    }
  }
  await page.emulateMedia({ colorScheme: 'dark' });
  await page.setViewportSize({ width: 1280, height: 900 });
};
await fits('');

// the planner steps: never fetched while off; on, steps.js on the page's planner
check(!requested.some((p) => /steps\.js|kinetic2-trace\.wasm/.test(p)), 'page steps: neither steps.js nor kinetic2-trace.wasm is fetched while off');
const dv = () => page.evaluate(() => ({ n: window.__k2steps.steps.length, idx: window.__k2steps.idx, kinds: [...new Set(window.__k2steps.steps.map((s) => s.kind))] }));
await page.click('#steps-toggle');
await page.waitForFunction(() => window.__k2steps);
let d = await dv();
check(d.n > 20 && ['classify', 'sweep', 'judge', 'piece', 'result', 'samples'].every((k) => d.kinds.includes(k)), `page steps: the built-in script, ${d.n} steps`);
check(requested.filter((p) => /kinetic2-trace\.wasm/.test(p)).length === 1 && requested.filter((p) => /kinetic2\.wasm/.test(p)).length === 1, 'page steps: one trace module fetched, with the steps; the page runs kinetic2.wasm');
await page.click('#st-next'); await page.click('#st-next');
d = await dv(); check(d.idx === 2, `page steps: Next walks the steps (step ${d.idx + 1})`);
await setNum('#c-vmax', 400);
d = await dv(); check(d.kinds.includes('trial'), 'page steps: at 400 mm/s the trim tests show');
await exact('speed 400 mm/s with the steps shown');
await setNum('#c-vmax', 1000);
const stepsPicture = () => page.evaluate(() => document.querySelector('#steps canvas').toDataURL());
const stepsDark = await stepsPicture();
await page.click('.theme'); await frames(); await frames();
check(await stepsPicture() !== stepsDark, 'page steps: the steps redraw in the light palette without a reload');
await page.click('.theme'); await frames();
await fits(' steps');
await page.click('#steps-toggle');
check(await page.evaluate(() => document.querySelector('#steps').hidden), 'page steps: toggled off, the panel hides');
check(errors.length === 0, `page: zero console errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
console.log('screenshots in ' + shots);
await browser.close(); server.close();
console.log(failures ? `${failures} FAILED` : 'PASS');
process.exit(failures ? 1 : 0);
