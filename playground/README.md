# Kinetic² playground

The playground is a static page that runs the Kinetic² planner, compiled to
WebAssembly (`wasm/kinetic2_wasm.h`), in the browser. Author knots on the position trace,
tune the planner options, and read what the planner did: the curve, its
velocity, acceleration and jerk against the ceilings, the solved knots, and
every anomaly the engine recorded.

Deployed at <https://openvalence.org/Kinetic/>.

## Run it locally

Build the module (README.md "WebAssembly"):

```sh
emcmake cmake -S . -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-wasm --target kinetic2_wasm
```

Then either:

- `node playground/check.mjs`: copies `build-wasm/wasm/kinetic2.wasm` into
  `playground/`, serves the folder on a free localhost port, and drives it
  headless with Playwright (see below). Set `PLAYWRIGHT_DIR` to a
  `node_modules` holding `playwright` (default: `../Phosphor/node_modules`)
  and `SHOTS_DIR` for the screenshots (default: the OS temp dir).
- or copy the wasm next to `index.html` and serve `playground/` with any
  static server (`python -m http.server -d playground`). Opening the file
  directly does not work: browsers refuse `fetch` over `file://`.

`playground/kinetic2.wasm` is a build product and is gitignored.

## What check.mjs asserts

For every preset: zero console errors, and over the 1 ms grid no |v|, |a| or
|jerk| above its ceiling by more than 0.1% and no step in position or
velocity between two samples. Presets with a known engine fault are listed
in `KNOWN` with their bead and print `XFAIL`; one that starts passing fails
the run so the list gets pruned. It also adds a knot by clicking, Shift-drags
it (fine mode, a tenth of the pointer), checks the share link round-trips,
and checks the phone layout has no horizontal scroll. One screenshot per
preset at 1280x900 and one at 400x900.

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

## The handles page

`handles.html` is a second page, pure JS, no wasm: a funscript as a composite
cubic Bézier in the time-position plane (kin-d0o, the design under study for
the renderer). The script's knots stay on the author's clock; each knot has
one angle (its velocity) and two handle lengths (the time extent of each
handle as a fraction of its span; a third is the exact PCHIP cubic). A knot
is classified from its chords: a hold edge, a crest (the chords change sign)
or a through point (same sign). Its class can be set by hand: G0 renders a
corner with two angles, G1 one angle with free lengths, G2 continuous
acceleration (the angle for a through point, the lengths for a crest). The
smoothness sets the defaults: at 0 (PCHIP) crests and hold edges stay G1 as
drawn, the corner ramp takes their acceleration step, and through points are
G2 by their angle; at 1 through points take Makima's angles and crests and
hold edges stay flat, G2 by their lengths; between, the free angles and lengths of the two
blend (the kernel's Config::smoothness). The
ceilings, the window included, bound the lengths per piece: speed from above, acceleration and jerk
from below. A piece no length satisfies may move its later knot inside the
tolerance box; what remains is marked infeasible. Handles drag as in Blender
(aligned; Shift keeps the angle), knots drag in position, and the strips show
velocity, acceleration and jerk against the ceilings with every nudge drawn.
The window (size and offset on the rail) rescales the script live, so the same
script can be watched asking more or less of the machine; Play runs the
rendered motion at real speed with the carriage on the rail strip.

It opens from `file://` (plain scripts, no fetch). `node playground/handles.check.mjs`
checks the model in node (the figure numbers: 150 mm strokes at 200 ms
between crests fit at a length of 0.25 under 1000 mm/s and 50000 mm/s²; a
through point's G2 angle; the built-in script's classes and ceilings) and,
with Playwright reachable, drives the page headless: load, select, drag a
handle, set a class, tighten a ceiling, zero console errors, no horizontal
scroll at phone width, two screenshots.

## Pages

`.github/workflows/pages.yml` builds the module on every push to main and
publishes the folder (the handles page included) without the check scripts
and this README, plus `kinetic2.wasm`. The repo's
Pages source must be set to GitHub Actions (Settings, Pages, Source), once,
by a maintainer.
