# Kinetic² playground

The playground is a static page that runs the Kinetic² planner, compiled to
WebAssembly (`wasm/kinetic2_wasm.h`), in the browser. Author knots on the position trace,
tune the planner options, and read what the planner did: the curve, its
velocity, acceleration and jerk against the ceilings, the solved knots, and
every anomaly the engine recorded.

Deployed at <https://openvalence.org/Kinetic/>.

## Run it locally

Build the module (README.md "Playground and WebAssembly"):

```sh
emcmake cmake -S . -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm --target kinetic2_wasm
```

`cmake --build build-wasm` builds both modules, `kinetic2.wasm` and
`kinetic2-trace.wasm` (the planner steps). Then either:

- `node playground/check.mjs`: copies both modules from `build-wasm/wasm/` into
  `playground/`, serves the folder on a free localhost port, and drives it
  headless with Playwright (see below). Set `PLAYWRIGHT_DIR` to a
  `node_modules` holding `playwright` (default: `../Phosphor/node_modules`)
  and `SHOTS_DIR` for the screenshots (default: the OS temp dir).
- or copy both modules next to `index.html` and serve `playground/` with any
  static server (`python -m http.server -d playground`). Opening the file
  directly does not work: browsers refuse `fetch` over `file://`.

`playground/kinetic2.wasm` and `playground/kinetic2-trace.wasm` are build
products and are gitignored.

## What check.mjs asserts

For every preset: zero console errors, and over the 1 ms grid no |v|, |a| or
|jerk| above its ceiling by more than 0.1% and no step in position or
velocity between two samples. Presets with a known engine fault are listed
in `KNOWN` with their bead and print `XFAIL`; one that starts passing fails
the run so the list gets pruned. It also adds a knot by clicking, Shift-drags
it (fine mode, a tenth of the pointer), checks the share link round-trips,
and checks the phone layout has no horizontal scroll. One screenshot per
preset at 1280x900 and one at 400x900.

For the planner steps: neither `steps.js` nor `kinetic2-trace.wasm` is fetched
while they are off; on, the trace build's 1 ms samples equal `kinetic2.wasm`'s
bit for bit; Overreach yields length-walk, trim and result steps; Next, Play and
every judge work; an added knot re-traces; a link with a piece no trim keeps
inside opens with the steps on and shows that piece over; no horizontal scroll
at phone width with the steps shown.

## How a render works

Every change is a full replay: configure, reset at the start state, submit
the knots, and sample a 1 ms grid. Authored ahead, every knot is submitted at
time 0, and a knot without an end velocity carries `KINETIC2_KNOT_REST_IF_LAST`
so a script that ends on it lands at rest. Streamed, each knot is submitted
`latency` before its time with `KINETIC2_KNOT_SAMPLE`. Sampling retires knots, so the solved
window is read after each batch of submits and whenever a knot retires.
Jerk is the difference of the sampled acceleration over 1 ms.

## Keys

| Key | Action |
| --- | --- |
| click | add a knot on the position trace |
| drag | move a knot (the start knot moves in position only) |
| Shift while dragging | fine, 0.1x |
| Ctrl while dragging | snap to 50 ms and 0.05 window |
| Delete / Backspace | remove the selected knot |
| right-click | remove the knot under the pointer |
| Space | play / pause |
| Esc | deselect |

## Planner steps

Show planner steps plays through what the planner computed for the knots on the
page, like a debugger: Prev, Play, Next, the scrubber, a speed, and the step
list. Each step animates from the last. Every number comes from the planner:
`kinetic2-trace.wasm` is the same planner with the hooks in
`include/kinetic2/trace.hpp` defined to record (`wasm/kinetic2_trace.cpp`), and
`kinetic2_trace_curve` evaluates a piece with the planner's own functions.
`steps.js` and the trace module load the first time the steps are shown.

The steps, in the planner's order (`handles.hpp` render, `solver.hpp` renderRun):

| Step | What it shows |
| --- | --- |
| Knots in | the run's knots, the origin's state, the ceilings |
| Classify | end, hold edge, crest or through point, from the chords |
| Start angles | pchip or Makima angles; a through point up close with pchip's spans, weights and result |
| G2 sweep n of 4 | the angles each sweep moves, the acceleration from each side meeting, the monotone band; a mid smoothness shows both solves and the blend |
| Every handle a third | the solved lengths, before any ceiling |
| Piece, k | one length factor judged: v, a and j over u with the judge's 11 samples and turning points, the factor grid |
| Piece, trim | one trim test: the full move, a halving, a quarter, or flat; the bracket of the bisection |
| Piece | what the piece keeps: its factor, its trim, legal or over |
| Pieces unchanged | a memo hit: inputs equal to the last round's, fit reused |
| Round fitted, Round n | each round's count of trims and pieces over; the solve on the trimmed chords |
| Built, Pass n | each piece built as the engine builds it; the ceilings tightened for the next pass |
| Solved | the knots as the engine gets them, corner ramps included |
| Chase, Live jog | a run of samples, or a jog, as one move to rest |
| The 1 ms samples | the engine's samples, with a cursor riding them |

By default the length walk and the trim tests are shown for the first round of
the first pass; every judge shows them in every round and pass. Streamed knots
are solved once per submit: the solve picker chooses one, and a streamed
window is samples, so it is one chase step.

The panels: the render (knots, angles, handles, the trimmed knots, the piece
in focus), the piece or knots up close, the scope (v, a, j over u against the
ceilings the piece is judged under, or over time for the samples), and the log.
Illegal values are red, legal ones blue.

The trace keeps 131072 records; a window that needs more says so under the
step list. A Kinetic change that moves a decision moves its hook with it.

The handles page has the same steps, in mm, on its own planner instance: the
knots it draws, the first one the origin at rest, an angle set by hand as the
knot's angle.

## The handles page

`handles.html` runs a funscript through the planner and draws it as the planner
renders it (kin-d0o): a composite cubic Bézier in the time-position plane, the
knots on the author's clock, one angle (the knot's velocity) and two handle
lengths per knot (the time extent of each handle as a share of its span; a
third is the exact PCHIP cubic), the ceilings as bounds on the lengths (Valence
RFC-106). It loads `kinetic2-trace.wasm` up front: the curve and the velocity,
acceleration and jerk strips are the 1 ms samples, bit for bit those of
`kinetic2.wasm`; each knot's class (end, hold edge, crest, through point),
G1 or G2, angle, handle lengths before and after the ceilings, trim and
verdict are the trace records of the last render round that had the knot
between two others. The normal module reports none of those per knot, which is
why the page takes the trace build: about 80 KB against 70 KB, with 32 MB of
linear memory against 2 MB (the trace buffer, zeroed pages).

The knots go in as a sender's would: the first is the origin at rest, the
next 64 are submitted at its time, and each later one as a slot frees, so a
script of any length renders. What the page sets is what the planner takes:
knot positions, an angle per knot (drag a handle; the length stays the
planner's), the ceilings, smoothness, handle floor and max trim. The hold
threshold is the planner's, 0.005 of the window. The window (size and offset
on the rail) rescales the script live; Play runs the samples at real speed
with the carriage on the rail strip.

It needs the page served over http (the module is fetched).
`node playground/handles.check.mjs` copies both modules from
`build-wasm/wasm/`, serves the folder and drives the page headless: every 1 ms
sample equal to `kinetic2.wasm`'s and every knot on the curve, for the built-in
script, an edited one, 400 mm/s, a jerk of 1e6, a 300 mm window, smoothness 1
and a 300-knot script; a dragged handle sets the angle the planner renders;
the planner steps on the page's instance; zero console errors, no horizontal
scroll at 1280x720, 1920x1080 and 390x844 with the steps on and off.

## Pages

`.github/workflows/pages.yml` builds the module on every push to main and
publishes the folder (the handles page included) without the check scripts
and this README, plus `kinetic2.wasm` and `kinetic2-trace.wasm`. The repo's
Pages source must be set to GitHub Actions (Settings, Pages, Source), once,
by a maintainer.
