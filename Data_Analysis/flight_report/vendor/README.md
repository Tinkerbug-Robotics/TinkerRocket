# Vendored assets

## plotly-gl3d.min.js

plotly.js (gl3d bundle, minified) v2.35.2 — MIT licensed, © 2012-2024 Plotly, Inc.

Source: <https://cdn.jsdelivr.net/npm/plotly.js-gl3d-dist-min@2.35.2>

Embedded verbatim into every report that contains interactive charts, so a
downloaded report stays self-contained and works offline. The *gl3d* bundle
(1.62 MB) carries `scatter` — every 2D chart in the report — plus `scatter3d`
for the flight-path view. It replaced the *basic* bundle (1.07 MB): the delta is
only ~550 KB because basic's trace set is a subset, so 3D was cheaper than
adding a second library. Neither bundle ships WebGL `scattergl`, which measured
unnecessary: SVG `scatter` renders four 30k-point traces in ~80 ms and zooms in
~17 ms.

To update, replace the file and bump the version above.

## cesium.js / cesium-widgets.css

CesiumJS 1.144.0 — Apache-2.0, © 2011-2024 Cesium GS, Inc. and contributors.

Source: <https://cdn.jsdelivr.net/npm/cesium@1.144.0/Build/Cesium/Cesium.js> and
`Widgets/widgets.css`. The **IIFE** build specifically, not the ES-module one:
it is the only build that embeds its worker bundle, which is what lets a report
carry Cesium in one file.

`sha256(cesium.js) = 686f5204599c46fd0430519bf7cbb4181c4bb11851c662ceb504c87a07486502`

Inlined into the flight report when it contains a 3D globe, and **patched at
render time** — the file on disk is byte-identical to upstream so that updating
stays a file swap. See `../cesium_bundle.py` for the patch and why it exists:
Cesium boots each worker with `importScripts(<blob: url>)`, which Chrome refuses
from an opaque `file://` origin, and every terrain and geometry worker dies. The
patch inlines the worker source instead. It verifies it matches exactly once and
raises otherwise, so a version bump that reshapes that bootstrap fails the build
rather than shipping a report whose globe is permanently blank.

Four files Cesium resolves relative to the page (`Assets/…`) are not shipped and
are disabled at runtime in `report.html.j2`: the skybox and moon textures, the
approximate-terrain-heights table, and the IAU2006 earth-orientation data.
Together they are ~2 MB of decoration for a view that never leaves the launch
site, and the first two are load-bearing — they fetch from inside the render
loop, so a 404 there stops rendering permanently.

Imagery and terrain are fetched at view time from Esri (`services.arcgisonline.com`
and `elevation3d.arcgis.com`), both keyless and both sending
`Access-Control-Allow-Origin: *` even for the `Origin: null` a `file://` page
sends — which matters more here than for Leaflet, because WebGL textures require
CORS where a plain `<img>` tile does not. No tiles are cached into the report.

To update, replace both files, bump the version and hash above, and re-run the
test suite — `test_cesium_inlined_once_and_patched` is what catches a bundle that
no longer takes the patch.

## leaflet.js / leaflet.css

Leaflet 1.9.4 — BSD-2-Clause, © Vladimir Agafonkin and contributors.

Source: <https://unpkg.com/leaflet@1.9.4/dist/leaflet.js> and `leaflet.css`.

Inlined into reports containing a map. Two things to know before touching this:

`leaflet.css` references `images/marker-icon.png`, `images/layers.png` and
`images/layers-2x.png` relative to itself. In a standalone HTML file those 404,
so the report must not use `L.marker` or the layers control — it uses
`L.circleMarker`, which is pure SVG. The unused rules are harmless.

Tiles are fetched at view time from `tile.openstreetmap.org` and are deliberately
never cached into the report: the OSM tile policy forbids bulk downloading and
redistributing tile archives. See the webtool README for the full constraints.
