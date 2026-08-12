# Post-Flight Analysis Web Tool (proof of concept)

Runs the repository's `flight_report` Python package **unmodified, entirely in the
browser** via [Pyodide](https://pyodide.org) (CPython compiled to WebAssembly).
Users drag in a flight's files and get the full HTML report — nothing to install,
no server-side processing, flight data never leaves their machine.

## How it works

- `index.html` / `app.js` — drop zone, live per-module progress list, report
  viewer (iframe), download button.
- `worker.js` — a Web Worker that loads Pyodide v0.29.1 from the jsDelivr CDN,
  loads numpy/pandas/matplotlib/jinja2, unpacks `payload/flight_report_src.zip`
  into the virtual filesystem, and runs the same code path as
  `python -m flight_report run` (parse → analysis modules → `render_report()`).
  Running in a worker keeps the page responsive during the ~1 min of analysis.
- `build.py` — packages the Python source (the `flight_report/` package plus the
  four sibling top-level modules it imports) into `payload/flight_report_src.zip`
  and copies the sample flight from `examples/flights/` into `payload/sample/`.

`payload/` is generated — re-run `build.py` after changing anything under
`Data_Analysis/`.

## Run locally

```
python3 build.py
python3 -m http.server 8642
```

Then open http://localhost:8642. (Any static file server works; `file://` does
not, because the page uses a Web Worker and `fetch`.)

## Deploy

Copy this directory (after `build.py`) to any static host — GitHub Pages works.
Only static files are served; the analysis runs client-side. First visit
downloads ~25 MB of Pyodide runtime + packages from the CDN (cached by the
browser afterwards).

## Measured on the sample flight (Rolly Polly 54mm — 5.2 MB / 154,188 frames)

| | |
|---|---|
| Engine start, cold cache | ~25 s (one time; ~25 MB from CDN) |
| Engine start, warm cache | ~12 s |
| Flight report, in browser | ~21 s |
| Flight report size | ~9.0 MB (5.7 MB of it CesiumJS, for the 3D globe) |
| Detailed report size | ~9.4 MB |
| Source payload zip | 2.5 MB (was 715 KB before Cesium was vendored) |

The first analysis on a machine pays a one-off WebAssembly/matplotlib warm-up
that the browser then caches, including across reloads. Module results match the
CLI exactly.

## The report

`flight_report` writes one file, `<stem>_report.html`. There used to be a second
"detailed" level and a selector to choose between them; the sections worth
keeping were folded into this one and the split was removed, because the two
drifted — the same word meant different instants in each.

It is the headline read. Summary card (apogee,
  max speed, peak boost acceleration, time to apogee, flight time, landing
  distance and bearing), a 3D flight path on satellite imagery, then interactive
  charts running position -> velocity -> acceleration, roll control, apogee
  detection, stability, per-sensor sample rates and log health. No static
  figures, no raw JSON dump, no parser stats.

### Flight metadata

`Flight.metadata` holds facts the log cannot know, keyed by name. Nothing reads
it today. It was fed by two entry-screen fields — liftoff mass and motor
designation — which existed only to unlock the Motor Performance section; both
the fields and the section were removed, because two inputs between dropping a
log and reading a report is two too many. Recorded as a potential enhancement in
[#750](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/750), which is
also where the reasoning behind the removed figures lives.

The attribute and the worker's pass-through are kept deliberately: they are the
whole cost of bringing that section back.

### Deployment & Recovery

Answers the three questions a flyer asks after a recovery, all without needing
any metadata entry:

- **Was apogee called on time?** The lag between the true barometric peak and the
  `alt_apogee` flag, and how far below the peak the vehicle was when it fired.
  Reported in both directions — an early call means still climbing, a late one
  means already descending — because early is the safer and more common tuning
  and calling it "altitude lost" would be wrong by sign.
- **Did the pyro channels do their job?** Per-channel fire time relative to
  apogee, plus continuity. Only an *enabled* channel that failed to fire raises a
  warning; channels configured off are reported as motor ejection, which is the
  normal case for smaller rockets.
- **Was the descent survivable?** Mean descent rate, drogue-to-main transition
  when there is one, and touchdown rate, with a warning above 10 m/s.

### Moving between the two

Each report links to the other from a pill in the top-left — "Detailed data →"
and "← Flight report". The link is only emitted when both files are actually
written (i.e. `--level both`), since a link to a report that was never generated
is worse than no link. In the web tool, the **Report** selector picks which one
to generate.

### Units

A metric/imperial toggle sits in the top-right of every report. Values render in
SI carrying their unit as data attributes, and the toggle rewrites them in place
— including chart y-data and axis labels — so one file serves both systems, the
switch needs no regeneration, and a downloaded report keeps working. The choice
is remembered in `localStorage` and survives moving between the two reports.

Conversions are keyed by *purpose*, not just by unit, because one SI unit can
need more than one imperial idiom: airspeed becomes mph, but descent and
touchdown rates become ft/s, which is how the hobby quotes a sink rate. Units
that read the same in both systems (seconds, G, degrees) must be declared
explicitly, so a typo raises rather than silently passing through unconverted.

### 3D flight path

The flight report carries a rotatable 3D trajectory (`scatter3d`) built from the
nav-filter ENU position, with a grey ground-track shadow at z=0 — a bare line in
space is hard to read, and the shadow is what shows the drift. The scene uses
`aspectmode: "data"` so a 355 m climb over 200 m of drift reads truthfully
instead of being stretched to fill the box.

Two deliberate choices:

- **It costs ~550 KB, not a new library.** We swapped the vendored `plotly-basic`
  bundle (1.07 MB) for `plotly-gl3d` (1.62 MB), which is a superset — it still
  carries the `scatter` type every 2D chart uses. A separate 3D library, or
  Plotly's full bundle at 4.35 MB, would have cost far more.
- **It is windowed to launch→touchdown**, unlike the 2D charts. The nav filter
  runs for the whole session and the pad minutes are thousands of samples piled
  on the origin: slow to draw and not part of the trajectory. Within the window
  nothing is dropped (32,875 points on the sample flight; redraw ~18 ms). The 2D
  viewport renderer does not apply here — it is a windowing scheme over a 1-D x
  axis, and a trajectory has no such axis.

The metric/imperial toggle rescales all three axes and relabels the scene.

### Recovery map

The **Deployment & Recovery** section carries a Leaflet map of the GNSS track
over OpenStreetMap tiles, with the pad and the last fix marked — the fastest way
to work out which field to walk into. Notes:

- **It needs the network at view time**, unlike everything else in the report.
  Opened offline, the track, markers, scale bar and controls still draw (they are
  SVG); only the imagery is missing. The container is styled as graph paper and
  failed tiles resolve to a transparent pixel, so an offline map reads as a plot
  rather than a broken page.
- **Tiles are deliberately not cached into the file.** The OSM tile policy
  forbids bulk downloading and states plainly that offline use of
  `tile.openstreetmap.org` is not permitted; a report with baked-in tiles is a
  redistributed tile archive. Degrading gracefully is the compliant option.
- Attribution is required and is left visible in the corner. The tile URL is
  `https://tile.openstreetmap.org/{z}/{x}/{y}.png` — HTTPS, no subdomain
  sharding, both required by the policy.
- **Before shipping this to many customers**, check the policy again. Casual
  human viewing is explicitly permitted, but OSM's tile servers are donated
  infrastructure and a product with a large user base is expected to use its own
  or a commercial tile source.

#### Basemaps, and why not Google

The layer switcher offers the same free basemaps the Android app uses: OSM
streets (default, worldwide) and USGS Imagery / Imagery+Topo / Topo from
`basemap.nationalmap.gov` (satellite, **United States only**). USGS publishes no
tiles above zoom 16, so those layers set `maxNativeZoom: 16` and let Leaflet
upscale — without it a recovery map fitted to a few hundred metres of track sits
past z16 and renders nothing, which looks like a broken layer rather than a
zoom limit.

The Android app's fourth source, Google Satellite, is deliberately **not**
carried over. It is the Google **Map Tiles API** (session token + API key over
`tile.googleapis.com`), not the Maps SDK, and it does not port to a report we
hand to customers:

- The key would sit in plain text in every distributed report. HTTP-referrer
  restrictions cannot protect it — a report opened from `file://` sends no usable
  referrer, and reports get opened from arbitrary hosts.
- Billing is per map load by the *reader*, not per report generated, with no
  automatic cap. Every customer opening a report spends against your account.

This asymmetry is a Google billing-structure fact, not an implementation choice:
their mobile SDKs are free while the equivalent web usage is metered. The
practical consequence is that satellite imagery on the web comes from USGS here,
which costs nothing and needs no key, at the price of US-only coverage. If
worldwide satellite becomes a requirement, that is a paid-tile decision to make
deliberately.
- Leaflet's default marker uses PNGs from a relative `images/` path that does not
  exist in a standalone file, so the map uses `circleMarker` (pure SVG) instead.
  The ENU ground-track chart moved to the detailed report, where exact metres
  off the pad matter more than which field to search.

### Settings snapshot

Detailed-report-only, and shows **configuration only** — the sidecar's measured
results (`max_altitude_m` and friends) belong to the summary card, and printing
them in both places invites the two to disagree. Rendered as grouped tables with
dotted keys (`pyro.ch1.trigger_mode`) rather than a wall of JSON, so it can be
scanned and diffed against another flight.

Raw 1 kHz inertial data lives in the detailed report on purpose: it is several
hundred thousand samples, most of them the rocket sitting on the pad. Keeping it
out is what makes the flight report ~2 MB against the detailed report's ~9 MB
on a 154k-frame flight — and it makes the browser tool roughly 3x faster for the
common case, since the flight report renders no matplotlib figures at all.

## Interactive charts

The report opens with a **Flight Overview** section of six zoomable Plotly
charts — altitude (baro/GNSS/nav filter together), velocity, acceleration,
angular rate, GNSS ground track, and rocket state — with launch, burnout,
apogee and landing marked. Drag to zoom, double-click to reset, drag an axis to
pan, click the legend to hide a series, camera button to save a PNG.

They come from the shared `flight_report` package, so the CLI produces the same
interactive report; nothing about them is browser-tool-specific. The 26 static
matplotlib figures remain below as the full reference set.

Implementation notes worth knowing:

- No Plotly Python dependency. `flight_report/charts.py` builds trace and layout
  dicts by hand and `render.py` serializes them, against the vendored
  `flight_report/vendor/plotly-gl3d.min.js` (1.62 MB, MIT). The Plotly Python
  wheel would cost ~10 MB of download and ~38 MB unpacked in the Pyodide FS.
- The library is inlined once per report, so a downloaded report stays a single
  self-contained file that works offline.
- The altitude chart's y-axis is scaled to the bulk of the data. The ejection
  charge produces a barometric spike near −900 m that would otherwise squash the
  real 0–75 m flight into a few pixels; every sample is still in the chart, and
  the toolbar's Autoscale button fits them.
- **Every sample is embedded at full resolution and nothing is decimated away.**
  Drawing 10^5 SVG points per trace would be slow, so the report ships a small
  viewport renderer: the overview draws a min/max *envelope* (~1200 buckets, so
  every peak and trough survives — an nth-sample decimation would silently drop
  the ejection transient and peak-G spikes), and each zoom re-slices from the
  full arrays. Zoom in far enough and you are looking at raw samples. Measured on
  a 154k-frame flight: 203,193 accelerometer samples draw as 7,200 envelope
  points at full extent, and a 0.05 s window renders its 45 real samples.
- Samples are drawn as **points with no connecting line**, matching the static
  figures — every `plot_*` function in `plot_flight_data_mini` uses `ax.scatter`
  and none use `ax.plot`. A line through the samples reads as continuous data
  and, at full-flight zoom, visually swallows the markers completely. Size and
  opacity scale with density (2 px at 0.45 opacity when thousands are on screen,
  7 px opaque below a few hundred), so a dense run reads as a band of samples
  and a sparse one as individual dots. The GNSS ground track keeps a connecting
  line, since there the order of the points is the information.

## Notes / known limits

- The report header shows the in-browser virtual path
  (`/data/flights/run2/<rocket>/flight_….bin`) where the CLI shows a real one.
  Cosmetic; would need a small change in the shared report template.
- Scipy is deliberately not loaded; `roll_pid` uses its built-in numpy fallback.
- Each run gets a fresh virtual directory, because the analysis discovers
  sidecars and `lora_*.csv` by scanning the flight file's parent — reusing one
  directory would let an earlier flight's files leak into a later report.
- Payload fetches use `cache: "no-cache"` so a rebuilt `payload/` is never served
  stale from the browser cache. `index.html` / `app.js` / `worker.js` themselves
  are still subject to normal caching — after deploying an update, a hard refresh
  (or waiting out the host's max-age) guarantees users get the new version.
