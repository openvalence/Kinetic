// check.mjs -- serves the playground and drives it headless: every preset
// renders with zero console errors and no sample over a ceiling, a knot can be
// added and Shift-dragged, the share link round-trips.
// Constraints:
// - Needs build-wasm/wasm/kinetic2.wasm (README.md "WebAssembly") and
//   Playwright with chromium from ../../Phosphor/node_modules, or PLAYWRIGHT_DIR.
// - KINETIC2_WASM overrides the module path.
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
  console.log(`      p ${s.p.min.toFixed(4)}..${s.p.max.toFixed(4)}, dropped ${s.dropped}, refused ${s.refused}; anomalies: ${Object.entries(by).map(([k, v]) => k + ' ' + v).join(', ') || 'none'}`);
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

// Phone width: stacked, no horizontal scroll.
await page.setViewportSize({ width: 400, height: 900 });
await page.click('#presets button[data-preset="stroke"]');
s = await settle(s.replays);
const sw = await page.evaluate(() => [document.documentElement.scrollWidth, innerWidth]);
check(sw[0] <= sw[1], `phone: scrollWidth ${sw[0]} <= ${sw[1]}`);
await page.screenshot({ path: join(shots, 'phone.png') });

check(errors.length === 0, 'no console errors overall' + (errors.length ? ': ' + errors.join(' | ') : ''));
console.log('screenshots: ' + shots);
await browser.close();
server.close();
console.log(failures ? `${failures} FAILED` : 'PASS');
process.exit(failures ? 1 : 0);
