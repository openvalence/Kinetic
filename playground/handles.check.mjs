// handles.check.mjs -- checks handles-model.js in node, then (when Playwright is
// reachable) serves the playground and drives handles.html headless.
// Run: node playground/handles.check.mjs
//   PLAYWRIGHT_DIR: a node_modules holding playwright (default ../Phosphor/node_modules)
//   SHOTS_DIR: where the screenshots go (default the OS temp dir)
import { existsSync, mkdirSync, readFileSync } from 'node:fs';
import { createServer } from 'node:http';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { dirname, extname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);
const M = require(join(here, 'handles-model.js'));

let failures = 0;
const check = (ok, what) => { console.log((ok ? 'ok   ' : 'FAIL ') + what); if (!ok) failures++; };
const within = (r, c) => r.peakV <= c.vmax * 1.001 && r.peakA <= c.amax * 1.001 && r.peakJ <= c.jmax * 1.001;

// ---- the model ---------------------------------------------------------------
{ // the figure: 150 mm strokes between crests at 200 ms fit at a length of 0.25
  const c = { vmax: 1000, amax: 50000, jmax: 5e6, box: 0, lo: 0, hi: 150 };
  const knots = M.fromFunscript({ actions: [{ at: 0, pos: 0 }, { at: 200, pos: 100 }, { at: 400, pos: 0 }, { at: 600, pos: 100 }, { at: 800, pos: 0 }] }, c);
  const R = M.render(knots, c);
  check(R.pieces.every((p) => Math.abs(p.peakV - 1000) < 2 && within(p, c)), `figure strokes: every piece peaks at the speed ceiling and inside the others (${R.pieces.map((p) => p.peakV.toFixed(0)).join(', ')})`);
  check(knots.slice(1, -1).every((k) => Math.abs(k.effIn - 0.25) < 0.011 && Math.abs(k.effOut - 0.25) < 0.011), `figure strokes: the crests' handles are nudged to 0.25 (${knots.slice(1, -1).map((k) => k.effIn.toFixed(2)).join(', ')})`);
  check(R.steps.every((s) => Math.abs(s.da) < 50), 'figure strokes: symmetric crests stay G2 after the nudge');
}
{ // the mid-travel point of the figure: G2 picks the angle 850 and the sides agree
  const c = { vmax: 5000, amax: 5e5, jmax: 5e8, box: 0, lo: 0, hi: 200 };
  const knots = M.fromFunscript({ actions: [{ at: 0, pos: 0 }, { at: 150, pos: 50 }, { at: 400, pos: 100 }] }, c);
  const R = M.render(knots, c);
  check(knots[1].cls === 'through' && Math.abs(knots[1].vIn - 850) < 1, `through point: G2 angle ${knots[1].vIn.toFixed(0)} mm/s (850)`);
  check(Math.abs(R.steps[0].da) < 50, `through point: acceleration step ${R.steps[0].da.toFixed(0)} (none)`);
  knots[1].type = 'G1'; M.render(knots, c);
  check(Math.abs(knots[1].vIn - 511) < 1, `the same knot as G1: PCHIP angle ${knots[1].vIn.toFixed(0)} mm/s (511)`);
  knots[1].type = 'G0'; M.render(knots, c);
  check(Math.abs(knots[1].vIn - 667) < 1 && Math.abs(knots[1].vOut - 400) < 1, `the same knot as G0: the chords ${knots[1].vIn.toFixed(0)} in, ${knots[1].vOut.toFixed(0)} out`);
}
{ // the built-in script: classes, no NaN, the ceilings after the nudge, the box
  const knots = M.fromFunscript(M.SAMPLE, {});
  const R = M.render(knots, {});
  const c = R.report.counts;
  check(c.end === 2 && c.rest === 8 && c.crest === 12 && c.through === 1, `built-in: classes end ${c.end}, holds ${c.rest}, crests ${c.crest}, through ${c.through}`);
  check(R.pieces.every((p) => [...p.p, ...p.v, ...p.a, ...p.j].every(Number.isFinite)), 'built-in: every sample finite');
  check(R.pieces.every((p, i) => knots[i + 1].infeasible || within(p, M.DEF)), 'built-in: every piece not marked infeasible is inside the ceilings');
  const noTrim = M.render(M.fromFunscript(M.SAMPLE, {}), { vmax: 800, trim: 0 }), trimmed = M.fromFunscript(M.SAMPLE, {}), withTrim = M.render(trimmed, { vmax: 800 });
  const towardPrev = trimmed.every((k, i) => k.dp === 0 || Math.sign(k.dp) === Math.sign(trimmed[i - 1].p + trimmed[i - 1].dp - k.p));
  check(noTrim.report.infeasible > 0 && withTrim.report.infeasible === 0 && towardPrev, `built-in at 800 mm/s: ${noTrim.report.infeasible} infeasible untrimmed, ${withTrim.report.infeasible} trimmed (${trimmed.filter((k) => k.dp !== 0).length} knots moved toward the previous knot, ${trimmed.reduce((m, k) => m + Math.abs(k.dp), 0).toFixed(1)} mm)`);
  { // a run of strokes beyond the machine: every top trims to the reachable height, the bottoms stay where the run started
    const fast = M.fromFunscript({ actions: [{ at: 0, pos: 0 }, { at: 150, pos: 100 }, { at: 300, pos: 0 }, { at: 450, pos: 100 }, { at: 600, pos: 0 }, { at: 750, pos: 100 }, { at: 900, pos: 0 }] }, {});
    const RF = M.render(fast, { vmax: 400 });
    const tops = fast.filter((k, i) => i % 2 === 1).map((k) => (k.p + k.dp).toFixed(1)), bottoms = fast.filter((k, i) => i % 2 === 0 && i > 0 && i < 6).map((k) => (k.p + k.dp).toFixed(1));
    check(RF.report.infeasible === 0 && new Set(tops).size === 1 && bottoms.every((b) => b === '0.0') && Number(tops[0]) < 60, `fast run at 400 mm/s: tops trimmed to ${tops[0]} mm, bottoms stay at 0, nothing infeasible`);
  }
  const smooth = M.fromFunscript(M.SAMPLE, {}), RS = M.render(smooth, { smoothness: 1 });
  check(smooth.filter((k) => k.eff === 'G2').length === smooth.length - 2 && RS.report.overPieces === 0 && smooth.filter((k) => k.cls === 'rest' || k.cls === 'crest').every((k) => k.vIn === 0), `built-in, smooth style: every inner knot G2, crests and hold edges flat, nothing over (longest ramp ${Math.max(...RS.steps.map((s) => s.rampMs)).toFixed(1)} ms where the window bound it)`);
  const tight = M.render(M.fromFunscript(M.SAMPLE, {}), { vmax: 300, trim: 0 });
  check(tight.report.infeasible > 0 && tight.report.overPieces === tight.report.infeasible, `built-in at 300 mm/s with no trim: ${tight.report.infeasible} infeasible pieces, each marked`);
}
{ // duplicate times collapse, pos clamps to the window
  const knots = M.fromFunscript({ actions: [{ at: 0, pos: 0 }, { at: 100, pos: 120 }, { at: 100, pos: 50 }, { at: 300, pos: -5 }] }, { lo: 0, hi: 100 });
  check(knots.length === 3 && knots[1].p === 50 && knots[2].p === 0, 'funscript: duplicate times collapse to the last, positions clamp to the window');
}

// ---- the page ----------------------------------------------------------------
const pwDir = process.env.PLAYWRIGHT_DIR || resolve(here, '../../Phosphor/node_modules');
let chromium = null;
try { chromium = createRequire(join(pwDir, 'noop.js'))('playwright').chromium; } catch { console.log('skip  Playwright not found under ' + pwDir + '; set PLAYWRIGHT_DIR for the page checks'); }
if (chromium) {
  const shots = process.env.SHOTS_DIR || join(tmpdir(), 'kinetic2-playground');
  mkdirSync(shots, { recursive: true });
  const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css' };
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
  const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
  const errors = [];
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(url);
  await page.waitForFunction(() => window.__handles && window.__handles.state().R);
  const st = () => page.evaluate(() => { const s = window.__handles.state(); return { n: s.knots.length, sel: s.sel, type: s.sel == null ? null : s.knots[s.sel].type, man: s.sel == null ? null : s.knots[s.sel].man, report: s.report, over: s.R.report.overPieces, infeasible: s.R.report.infeasible, trimmed: s.knots.filter((k) => k.dp !== 0).length, nudged: s.knots.filter((k) => Math.abs(k.effIn - k.lIn) > 1e-6 || Math.abs(k.effOut - k.lOut) > 1e-6).length }; });
  let s = await st();
  check(s.n === M.SAMPLE.actions.length, `page: the built-in script loaded, ${s.n} knots`);
  // select a crest and drag its out handle: the angle is hand-set and the knot becomes G1
  const i = 2;
  const k = await page.evaluate((i) => window.__handles.xy(i), i);
  await page.mouse.click(k.x, k.y);
  s = await st(); check(s.sel === i, `page: clicking a knot selects it (${s.sel})`);
  const hnd = await page.evaluate((i) => window.__handles.handleXY(i, 'out'), i);
  await page.mouse.move(hnd.x, hnd.y); await page.mouse.down(); await page.mouse.move(hnd.x + 15, hnd.y - 25, { steps: 5 }); await page.mouse.up();
  s = await st(); check(s.type === 'G1' && s.man.v != null && s.man.lOut != null, `page: dragging a handle sets the angle and length by hand and the knot renders G1 (${JSON.stringify(s.man)})`);
  await page.click('#k-auto');
  s = await st(); check(s.type === 'auto' && Object.keys(s.man).length === 0, 'page: Auto this knot clears the hand-set values');
  await page.selectOption('#k-type', 'G0');
  s = await st(); check(s.type === 'G0', 'page: the type select sets G0');
  // a tight speed ceiling nudges handles and marks what cannot be met
  await page.fill('#c-vmax', '400'); await page.press('#c-vmax', 'Enter'); await page.dispatchEvent('#c-vmax', 'change');
  s = await st(); check(s.nudged > 0, `page: a 400 mm/s ceiling nudges ${s.nudged} handles`);
  // the window: a bigger window asks more of the same script, the carriage follows the render
  await page.fill('#c-vmax', '1000'); await page.dispatchEvent('#c-vmax', 'change');
  await page.evaluate(() => window.__handles.setWindow(100, 400));
  s = await st(); check(s.over === 0 && s.infeasible === 0 && s.trimmed > 0, `page: a 300 mm window at 1000 mm/s renders inside the ceilings by trimming ${s.trimmed} knots (${s.nudged} handles nudged)`);
  const win = await page.evaluate(() => { const st = window.__handles.state(); return { lo: st.cfg.lo, hi: st.cfg.hi, p0: st.knots[0].p, pmax: Math.max(...st.knots.map((k) => k.p)) }; });
  check(win.lo === 100 && win.hi === 400 && win.p0 === 130 && win.pmax === 400, `page: the script rescaled into the window (first knot ${win.p0} mm, highest ${win.pmax} mm)`);
  await page.click('#play');
  await page.waitForTimeout(400);
  const play = await page.evaluate(() => { const st = window.__handles.state(); return { playing: st.playing, cursor: st.cursor }; });
  check(play.playing && play.cursor > 0.2, `page: playback runs (cursor at ${(play.cursor * 1000).toFixed(0)} ms after 400 ms)`);
  await page.click('#play');
  await page.screenshot({ path: join(shots, 'handles-1280.png') });
  await page.setViewportSize({ width: 400, height: 900 });
  await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
  const wide = await page.evaluate(() => document.documentElement.scrollWidth > document.documentElement.clientWidth);
  check(!wide, 'page: no horizontal scroll at phone width');
  await page.screenshot({ path: join(shots, 'handles-400.png') });
  check(errors.length === 0, `page: zero console errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
  console.log('screenshots in ' + shots);
  await browser.close(); server.close();
}
console.log(failures ? `${failures} FAILED` : 'PASS');
process.exit(failures ? 1 : 0);
