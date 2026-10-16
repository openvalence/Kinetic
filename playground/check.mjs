// check.mjs -- serves the playground and drives it headless: every preset
// renders with zero console errors and no sample over a ceiling, a knot can be
// added and Shift-dragged, the share link round-trips, and the planner steps
// load only when shown and trace the same samples.
// Constraints:
// - Needs build-wasm/wasm/kinetic2.wasm and kinetic2-trace.wasm (README.md
//   "Playground and WebAssembly") and
//   Playwright with chromium from ../../Phosphor/node_modules, or PLAYWRIGHT_DIR.
// - KINETIC2_WASM and KINETIC2_TRACE_WASM override the module paths.
// - Screenshots land in SHOTS_DIR (default: the OS temp dir).
// Usage: node playground/check.mjs
import { copyFileSync, mkdirSync, readFileSync, existsSync } from 'node:fs';
import { createServer } from 'node:http';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { dirname, extname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '..');
const wasm = process.env.KINETIC2_WASM || join(root, 'build-wasm/wasm/kinetic2.wasm');
if (!existsSync(wasm)) { console.error('missing ' + wasm + ': build the kinetic2_wasm target first'); process.exit(2); }
copyFileSync(wasm, join(here, 'kinetic2.wasm'));
const traceWasm = process.env.KINETIC2_TRACE_WASM || join(root, 'build-wasm/wasm/kinetic2-trace.wasm');
if (!existsSync(traceWasm)) { console.error('missing ' + traceWasm + ': build the kinetic2_trace_wasm target first'); process.exit(2); }
copyFileSync(traceWasm, join(here, 'kinetic2-trace.wasm'));

const pwDir = process.env.PLAYWRIGHT_DIR || resolve(root, '../Phosphor/node_modules');
const { chromium } = createRequire(join(pwDir, 'noop.js'))('playwright');
const shots = process.env.SHOTS_DIR || join(tmpdir(), 'kinetic2-playground');
mkdirSync(shots, { recursive: true });

const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.css': 'text/css', '.wasm': 'application/wasm' };
const server = createServer((req, res) => {
  const path = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const file = resolve(here, '.' + (path === '/' ? '/index.html' : path));
  if (!file.startsWith(here) || !existsSync(file)) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'content-type': MIME[extname(file)] || 'application/octet-stream' });
  res.end(readFileSync(file));
});
await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
const url = `http://127.0.0.1:${server.address().port}/`;

// Presets the engine is known to fail, with the bead that tracks the fault.
// A known failure prints XFAIL; one that starts passing fails the run (XPASS)
// so this list is pruned when the engine is fixed.
const KNOWN = {};

let failures = 0;
const check = (ok, what, known) => {
  const tag = known ? (ok ? 'XPASS' : 'XFAIL') : ok ? 'ok   ' : 'FAIL ';
  console.log(tag + ' ' + what + (known ? ` [${known}]` : ''));
  if (known ? ok : !ok) failures++;
};

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1280, height: 900 } });
const errors = [];
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
page.on('pageerror', (e) => errors.push(String(e)));
const requested = [];
page.on('request', (r) => requested.push(new URL(r.url()).pathname));

const k2 = () => page.evaluate(() => window.__k2);
async function settle(prev) {
  await page.waitForFunction((n) => window.__k2 && window.__k2.replays > n, prev ?? 0);
  await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
  return k2();
}

function ceilings(name, s, known) {
  const lim = (m, c) => m <= c * (1 + 1e-3);
  check(lim(s.max.v, s.ceil.v) && lim(s.max.a, s.ceil.a) && lim(s.max.j, s.ceil.j) && s.steps === 0,
    `${name}: max |v| ${s.max.v.toFixed(3)}/${s.ceil.v}, |a| ${s.max.a.toFixed(2)}/${s.ceil.a}, |j| ${s.max.j.toFixed(1)}/${s.ceil.j}, `
    + `${s.steps} p/v steps${s.steps ? ' (first at ' + (s.firstStep * 1000).toFixed(0) + ' ms)' : ''} over ${s.samples} samples`, known);
  const by = {};
  for (const a of s.anomalies) by[a.name] = (by[a.name] || 0) + 1;
  console.log(`      p ${s.p.min.toFixed(4)}..${s.p.max.toFixed(4)}, refused ${s.refused}; anomalies: ${Object.entries(by).map(([k, v]) => k + ' ' + v).join(', ') || 'none'}`);
}

await page.goto(url);
await page.waitForFunction(() => window.__k2 || document.querySelector('#status').textContent);
if (!(await page.evaluate(() => window.__k2))) {
  console.error('FAIL page did not start: ' + await page.textContent('#status'));
  await browser.close(); server.close(); process.exit(1);
}
let s = await settle();
check(/^kinetic2 /.test(await page.textContent('#version')), 'version string: ' + await page.textContent('#version'));

const presets = await page.$$eval('#presets button', (bs) => bs.map((b) => b.dataset.preset));
for (const name of presets) {
  const before = errors.length;
  await page.click(`#presets button[data-preset="${name}"]`);
  s = await settle(s.replays);
  check(errors.length === before, `${name}: no console errors`);
  ceilings(name, s, KNOWN[name]);
  await page.screenshot({ path: join(shots, name + '.png') });
}

// Overreach: time never gives, the stroke is trimmed.
await page.click('#presets button[data-preset="overreach"]');
s = await settle(s.replays);
check(s.knots[0].solved && s.knots[0].solved.stretched === 0 && s.knots[0].solved.share < 1,
  `overreach: on time (stretched ${(s.knots[0].solved?.stretched * 1000).toFixed(1)} ms), trimmed to ${(s.knots[0].solved?.share * 100).toFixed(0)}%`);

// Add a knot, then Shift-drag it: fine mode moves it a tenth of the pointer.
await page.click('#presets button[data-preset="stroke"]');
s = await settle(s.replays);
const box = await page.locator('#overlay').boundingBox();
const Ly = s.layout, T = s.T;
const at = (t, p) => ({ x: box.x + Ly.x0 + (t / T) * (Ly.x1 - Ly.x0), y: box.y + Ly.y1 - (p - Ly.lo) / (Ly.hi - Ly.lo) * (Ly.y1 - Ly.y0) });
const n0 = s.knots.length;
const a = at(1.5, 0.5);
await page.mouse.click(a.x, a.y);
s = await settle(s.replays);
const added = s.knots.find((k) => Math.abs(k.t - 1.5) < 0.01);
check(s.knots.length === n0 + 1 && added && Math.abs(added.p - 0.5) < 0.01, `click added a knot at t ${added?.t.toFixed(3)} p ${added?.p.toFixed(3)}`);
const r0 = s.replays;
await page.mouse.move(a.x, a.y);
await page.mouse.down();
await page.keyboard.down('Shift');
await page.mouse.move(a.x + 100, a.y - 80, { steps: 10 });
await page.mouse.up();
await page.keyboard.up('Shift');
s = await settle(r0);
const moved = s.knots.find((k) => k.t > 1.4 && k.t < 1.7);
const dt = moved.t - added.t, dp = moved.p - added.p;
const wantDt = 0.1 * 100 / (Ly.x1 - Ly.x0) * T, wantDp = 0.1 * 80 / (Ly.y1 - Ly.y0) * (Ly.hi - Ly.lo);
check(Math.abs(dt - wantDt) < wantDt * 0.2 && Math.abs(dp - wantDp) < wantDp * 0.2 && s.replays > r0,
  `Shift-drag moved the knot dt ${(dt * 1000).toFixed(2)} ms (want ${(wantDt * 1000).toFixed(2)}), dp ${dp.toFixed(4)} (want ${wantDp.toFixed(4)}), ${s.replays - r0} replays`);
ceilings('edited', s);
await page.screenshot({ path: join(shots, 'edited.png') });

// Play sweeps the cursor; reduced motion disables it.
await page.mouse.move(box.x + 5, box.y + box.height + 40);
await page.keyboard.press('Space');
const r1 = await page.textContent('#readout');
await page.waitForTimeout(300);
const r2 = await page.textContent('#readout');
await page.keyboard.press('Space');
check(r1 !== r2 && r2.startsWith('t '), `Space plays: readout "${r2.slice(0, 16)}" moves`);
await page.emulateMedia({ reducedMotion: 'reduce' });
check(await page.waitForFunction(() => document.querySelector('#play').disabled, null, { timeout: 3000 }).then(() => true, () => false),
  'reduced motion disables play');
await page.emulateMedia({ reducedMotion: 'no-preference' });

// The share link round-trips.
await page.waitForTimeout(400);
const shared = page.url();
const p2 = await browser.newPage({ viewport: { width: 1280, height: 900 } });
await p2.goto(shared);
await p2.waitForFunction(() => window.__k2);
const s2 = await p2.evaluate(() => window.__k2);
check(s2.knots.length === s.knots.length && Math.abs(s2.knots.at(-1).t - s.knots.at(-1).t) < 1e-6, `share link round-trips ${s2.knots.length} knots`);
await p2.close();

// The planner steps: never fetched while off; on, the trace build's samples
// are the page's bit for bit, and the steps cover the walk, the trim and the result.
check(!requested.some((p) => /steps\.js|kinetic2-trace\.wasm/.test(p)), 'planner steps not fetched while off');
const stepsOf = async () => {
  await page.waitForFunction(() => window.__k2steps && window.__k2steps.P.length === window.__k2.P.length);
  return page.evaluate(() => {
    const a = window.__k2.P, b = window.__k2steps.P;
    let same = a.length === b.length;
    for (let i = 0; same && i < a.length; i++) same = a[i] === b[i];
    const by = {};
    for (const s of window.__k2steps.steps) by[s.kind] = (by[s.kind] || 0) + 1;
    return { same, by, n: window.__k2steps.steps.length, idx: window.__k2steps.idx, steps: window.__k2steps.steps, lost: window.__k2steps.lost };
  });
};
await page.click('#presets button[data-preset="overreach"]');
s = await settle(s.replays);
await page.click('#steps-toggle');
let st = await stepsOf();
check(st.same && st.lost === 0, `overreach steps: the trace build's ${s.samples} samples match kinetic2.wasm bit for bit`);
check(st.by.judge > 1 && st.by.trial > 1 && st.by.piece >= 1 && st.by.result === 1 && st.by.samples === 1
  && st.steps.some((x) => x.kind === 'trial' && / legal$/.test(x.title)) && st.steps.some((x) => x.kind === 'trial' && / over$/.test(x.title)),
  `overreach steps: ${st.n} (${Object.entries(st.by).map(([k, v]) => k + ' ' + v).join(', ')})`);
await page.click('#st-next');
await page.click('#st-next');
st = await stepsOf();
check(st.idx === 2, `Next steps forward: at step ${st.idx + 1}`);
await page.selectOption('#st-speed', '4');
await page.click('#st-play');
await page.waitForTimeout(1500);
st = await stepsOf();
await page.click('#st-play');
check(st.idx > 2, `Play walks the steps: at step ${st.idx + 1}`);
await (await page.$('#steps')).screenshot({ path: join(shots, 'steps-overreach.png') });
const n1 = st.n;
await page.check('#st-all');
st = await stepsOf();
check(st.n >= n1, `every judge: ${st.n} steps (${n1} without)`);
await page.uncheck('#st-all');
// A user-built script: a click adds a knot and the steps follow it.
await page.click('#presets button[data-preset="stroke"]');
s = await settle(s.replays);
st = await stepsOf();
const n2 = st.n, b2 = await page.locator('#overlay').boundingBox();
await page.mouse.click(b2.x + s.layout.x0 + (1.5 / s.T) * (s.layout.x1 - s.layout.x0), b2.y + s.layout.y1 - (0.9 - s.layout.lo) / (s.layout.hi - s.layout.lo) * (s.layout.y1 - s.layout.y0));
s = await settle(s.replays);
st = await stepsOf();
check(st.same && st.n > n2 && s.knots.length === 3, `an added knot re-traces: ${n2} -> ${st.n} steps, samples match`);
// A piece no trim keeps inside: rendered over a ceiling, its steps red.
const over = { v: 2, T: 1, m: 'ahead', l: 100, p0: 0.5, o: [3, 30, 2000, 0, 0.15, 1, 4], d: 1, k: [[0.137, 0.33, null], [0.337, 0.78, 3.6], [0.356, 0.84, null]] };
const p3 = await browser.newPage({ viewport: { width: 1280, height: 900 } });
const e3 = [];
p3.on('console', (m) => { if (m.type() === 'error') e3.push(m.text()); });
p3.on('pageerror', (e) => e3.push(String(e)));
await p3.goto(url + '#' + Buffer.from(JSON.stringify(over)).toString('base64url'));
await p3.waitForFunction(() => window.__k2 && window.__k2steps);
const s3 = await p3.evaluate(() => ({ an: window.__k2.anomalies.map((a) => a.name), steps: window.__k2steps.steps }));
check(s3.an.includes('PieceOverCeiling') && s3.an.includes('KnotTrimmed') && s3.steps.some((x) => x.kind === 'piece' && /over ×/.test(x.title)) && e3.length === 0,
  `over a ceiling, from a link with the steps on: ${s3.steps.length} steps, anomalies ${[...new Set(s3.an)].join(', ')}`);
await p3.evaluate((i) => { const r = document.querySelector('#st-scrub'); r.value = i; r.dispatchEvent(new Event('input')); }, s3.steps.findIndex((x) => x.kind === 'piece' && /over ×/.test(x.title)));
await p3.waitForTimeout(600);
await (await p3.$('#steps')).screenshot({ path: join(shots, 'steps-over.png') });
await p3.close();

// Phone width: stacked, no horizontal scroll.
await page.setViewportSize({ width: 400, height: 900 });
await page.click('#presets button[data-preset="stroke"]');
s = await settle(s.replays);
const sw = await page.evaluate(() => [document.documentElement.scrollWidth, innerWidth]);
check(sw[0] <= sw[1], `phone: scrollWidth ${sw[0]} <= ${sw[1]} (planner steps shown)`);
await page.screenshot({ path: join(shots, 'phone.png'), fullPage: true });
await page.click('#steps-toggle');
check(await page.waitForFunction(() => document.querySelector('#steps').hidden, null, { timeout: 3000 }).then(() => true, () => false), 'planner steps hide when toggled off');

check(errors.length === 0, 'no console errors overall' + (errors.length ? ': ' + errors.join(' | ') : ''));
console.log('screenshots: ' + shots);
await browser.close();
server.close();
console.log(failures ? `${failures} FAILED` : 'PASS');
process.exit(failures ? 1 : 0);
