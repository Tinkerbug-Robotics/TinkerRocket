# Offline map tiles for no-signal launch sites

**Issue:** _to be filed_ (tracking issue TBD)
**Status:** Trial 1 (read-through cache) landed. Trial 0 cleared — standardizing on **Apple** (global, online-only) + **USGS** (cacheable). Esri dropped; international imagery deferred to [#231](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/231).
**Last updated:** 2026-06-03
**Mockups:** [`offline-maps-mockups.html`](offline-maps-mockups.html) (open in a browser)

Launch sites frequently have no cell reception. Both map surfaces in the iOS app draw on Apple's MapKit basemap, and **Apple's tiles cannot be pre-downloaded or cached for offline use** — there is no public API and persisting them is against the MapKit terms. So at the field with no signal, the imagery behind the pins is blank in:

- **Reverse Drift Cast** — `DriftCastMapView`, plus the ArcGIS ground texture in `Trajectory3DView` ([DriftCastView.swift](../../TinkerRocketApp/TinkerRocketApp/Views/DriftCastView.swift))
- **Rocket Map** — `RocketMapView`, used for live tracking and recovery ([MapView.swift](../../TinkerRocketApp/TinkerRocketApp/Views/MapView.swift))

GPS and telemetry already work with no cell signal (`device.lastValidRocketFix`, distance/bearing, the landing predictor). **The only thing offline-broken today is the basemap imagery.** This plan adds the ability to pre-download imagery for a chosen area while connected, store it on-device, and serve it later with no signal — shared by both map views and the 3D texture.

This document is the contract that the implementation phases build against.

## Guiding principle: trial-driven and reversible

The hard risk here is not technical, it's **subjective**: cache-permissive imagery (USGS / Esri) may look worse than Apple's polished hybrid basemap. So the first trial is a cheap on-device A/B at a real field, behind a debug toggle, before any download/cache machinery is built. Every phase ends in something testable with an explicit go/no-go, and the custom layer stays swappable so we can fall back to the Apple basemap (online) at any point.

## Phasing & trials

| Phase | Deliverable | Decision gate |
|---|---|---|
| **Trial 0** | A `CachingTileOverlay` stub (no caching yet) added to `RocketMapView` behind a hidden source toggle. Compare USGS Imagery / USGS Topo / Esri vs. Apple Hybrid on-device, online, **at an actual launch field**. | **Is the imagery good enough to be worth it?** If no → try another source, or stop and shelve the feature. Nothing else proceeds until this clears. |
| **Trial 1** | Read-through on-disk cache behind the overlay. View an area online, enable Airplane Mode, confirm those tiles still render. | Does offline render actually work, and is the cache hit-path fast enough to not jank panning? |
| **Trial 2** | `TileDownloader` + a minimal "save center + radius to max-zoom N" action with a live size estimate and progress. Download a field, Airplane Mode, verify. | Are tile counts / sizes / download times acceptable for a real field area? Is the estimate trustworthy? |
| **Phase 3** | Share the overlay + cache into `DriftCastMapView`; route the `Trajectory3DView` texture through the same cache. | Both views + 3D work offline from one download. |
| **Phase 4** | `OfflineRegionStore` + region manager UI (list / size / delete / re-download), offline indicator, missing-tile treatment, source toggle as a shipped control. | Feature-complete, manageable storage, no silent failures. |

Trials 0–2 are deliberately throwaway-friendly: each is a few hundred lines behind a flag, and any of them can kill or redirect the feature.

## Resolved design decisions

| # | Decision |
|---|---|
| 1 | **Move the savable layer off Apple's basemap.** Add an `MKTileOverlay` subclass with `canReplaceMapContent = true` to both map views. Apple basemap stays as the default *online* layer; the custom source is what gets cached. |
| 2 | **One shared offline subsystem**, not per-view. A single download serves `DriftCastMapView`, `RocketMapView`, and the `Trajectory3DView` texture. New module under `TinkerRocketApp/.../Maps/Offline/`. |
| 3 | **Primary tile source: USGS *The National Map*** (`USGSImageryOnly`, `USGSTopo`). Public domain, free, no API key, caching/offline explicitly permitted, US coverage (our default site is Wallops Island, VA). Source is pluggable so Trial 0 can compare alternatives. |
| 4 | **Cache lives in Application Support, not Caches.** The OS purges `Caches/` under storage pressure, which would silently delete exactly the data the user pre-downloaded for the field. Mark the directory excluded-from-backup. |
| 5 | **Pre-download by center + radius + max-zoom**, not free-draw rectangles — bounds tile counts and is the natural "the field is here, this big" gesture. Always show a tile-count + MB estimate *before* committing. |
| 6 | **Annotations/overlays are untouched.** Pins (launch/landing/guidance/rocket) and polylines (descent/boost/predicted) render on top of the tile overlay exactly as today, so existing map features carry over with no change. `mapType` becomes irrelevant for the offline layer and is replaced by a source picker. |
| 7 | **Out of scope (v1):** vector maps, worldwide coverage, routing/search, automatic background pre-fetch, sharing regions between devices, non-US imagery. |

---

## 1. Architecture

A single shared subsystem, injected into both map views and the 3D texture fetch.

```
                    ┌─────────────────────────────────────────────┐
                    │            OfflineMapsManager (facade)        │
                    │  - current TileSource (Apple | USGS img | …)  │
                    │  - download(region) / regions / delete        │
                    └───────────────┬───────────────┬──────────────┘
                                    │               │
              ┌─────────────────────▼──┐     ┌──────▼───────────────┐
              │   TileDownloader       │     │  OfflineRegionStore   │
              │  region → [z,x,y] →    │     │  manifest of saved    │
              │  bounded concurrent    │     │  regions (JSON on disk)│
              │  fetch + progress      │     └───────────────────────┘
              └───────────┬────────────┘
                          │ writes
                ┌─────────▼──────────┐         reads
                │  OfflineTileCache  │◄───────────────────┐
                │  Application Support│                    │
                │  /tiles/<src>/z/x/y │                    │
                └─────────┬──────────┘                     │
                          │                                │
        ┌─────────────────▼─────────────────┐   ┌──────────┴─────────────┐
        │  CachingTileOverlay : MKTileOverlay│   │ Trajectory3DView texture│
        │  loadTile: disk → (online) network │   │ fetch (routed through   │
        │  canReplaceMapContent = true        │   │ the same cache)         │
        └──────┬───────────────────┬──────────┘   └────────────────────────┘
               │                   │
       ┌───────▼──────┐    ┌───────▼───────┐
       │ RocketMapView│    │DriftCastMapView│   ← add overlay; pins/lines unchanged
       └──────────────┘    └────────────────┘
```

### Components

- **`TileSource`** — enum/struct describing a provider: display name, URL template (`{z}/{x}/{y}`), tile format, attribution, `cacheable: Bool`, min/max zoom. `apple` is a special non-tile case (uses MapKit basemap, `cacheable = false`).
- **`OfflineTileCache`** — maps `(source, z, x, y)` → file path under `Application Support/OfflineTiles/<source>/<z>/<x>/<y>.<ext>`. `tileData(...) -> Data?`, `store(...)`, `bytesOnDisk(for:)`, `removeTiles(in:)`.
- **`CachingTileOverlay: MKTileOverlay`** — override `loadTile(at:result:)`: return cached bytes if present; else if online, fetch via the URL template, persist, return; else return a neutral "not downloaded" tile. `canReplaceMapContent = true`.
- **`TileDownloader`** — given a `RegionSpec` (center, radiusMeters, maxZoom), enumerate the `(z,x,y)` set (§3), dedupe, fetch with bounded concurrency (~4–6), persist through `OfflineTileCache`, publish `@Published progress`. Cancelable; resumable (skips tiles already cached).
- **`OfflineRegionStore`** — persisted manifest (`[Region]`: id, name, center, radius, zoomRange, source, tileCount, bytes, dateSaved). Backs the region-manager UI.
- **`OfflineMapsManager`** — `ObservableObject` facade injected via `@EnvironmentObject`, holding the active `TileSource` and orchestrating the above. Both map views observe it.

### How each view plugs in

- **`RocketMapView` / `DriftCastMapView`**: in `makeUIView`, if the active source is cacheable, add a `CachingTileOverlay` (level `.aboveLabels` is moot since it replaces content); in `updateUIView`, swap the overlay when the source changes. Everything else (annotations, polylines, region logic) is unchanged.
- **`Trajectory3DView`**: its `fetchArcGISImagery(...)` becomes a call into `OfflineTileCache` (assemble from cached tiles, or cache the single export image keyed by bbox) so the 3D ground texture is available offline too.

---

## 2. Tile source decision

| Source | Cacheable | Key | Coverage | Notes |
|---|---|---|---|---|
| Apple MapKit | ❌ | — | Global | Default *online* layer; cannot be saved. |
| **USGS Imagery / Topo** ⭐ | ✅ | none | US | Public domain, caching permitted. `https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/tile/{z}/{y}/{x}`. |
| Esri World Imagery | ✅* | none | Global | Already used by `Trajectory3DView`. *Verify current ToS for persistent offline caching + attribution before relying on it. |
| OSM / MapTiler / Mapbox | ✅ | key (most) | Global / vector | For non-US or topo styling later; OSM policy discourages bulk download. |

**Decided (Trial 0):** standardize on **Apple Standard/Hybrid** (global, online-only) + **USGS Imagery / Imagery+Topo / Topo** (public-domain, cacheable). Esri World Imagery looked best online and zoomed deeper, but its free terms restrict the persistent offline caching this feature is built around — and Apple already covers global *online* — so it was dropped. Only USGS is offered for offline download, gated on `TileSource.isCacheable`. International (non-US) cacheable imagery is deferred to [#231](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/231).

---

## 3. Tile math & size estimation

Standard slippy-map tiling. For zoom `z`, `n = 2^z`:

```
x = floor((lon + 180) / 360 * n)
y = floor((1 - asinh(tan(lat_rad)) / π) / 2 * n)
```

For a region (center + radius), compute the lat/lon bbox, then for each `z` in `[minZoom…maxZoom]` take the tile rectangle `[xmin…xmax] × [ymin…ymax]`; union across zooms, dedupe.

**Size estimate** (shown before download): `Σ_z (tilesAtZoom_z) × avgTileBytes`, with `avgTileBytes ≈ 25 KB` for imagery (refined empirically in Trial 2). Rough feel for a launch field:

| Area (radius) | maxZoom | ~tiles | ~size |
|---|---|---|---|
| 5 km | 16 | ~500 | ~12 MB |
| 5 km | 17 | ~1,800 | ~45 MB |
| 10 km | 17 | ~6,500 | ~160 MB |
| 10 km | 18 | ~26,000 | ~640 MB |

The estimate table *is* the argument for center+radius+capped-zoom and an explicit pre-download confirmation: each extra zoom ~4×'s the count. Default the slider to z16–17 and warn above a threshold (e.g. > 200 MB).

---

## 4. UI / UX

See [`offline-maps-mockups.html`](offline-maps-mockups.html) for rendered screens. Summary:

- **Source picker** (replaces the map-type toggle): Apple Standard · Apple Hybrid · USGS Imagery · USGS Topo. A small badge shows the active source and an "Offline" pill when no network. This control is also the Trial 0 A/B tool.
- **Save Area sheet**: a map centered on a draggable pin (or "use current location"), a translucent radius circle, a **Radius** stepper and a **Detail (max zoom)** slider, a live `~N tiles · ~M MB · z10–z17` readout, and a **Download** button (disabled until estimate < cap or user confirms a large download).
- **Download progress**: determinate bar, `1,840 / 3,200 tiles · 27 MB`, **Cancel**. Survives backgrounding; resumable.
- **Offline Regions manager**: list of saved regions (name, date, size, zoom range, source), per-row delete, total storage line, "Add region". Reachable from both DriftCast and the dashboard.
- **Missing-tile treatment**: offline + outside a saved area/zoom → a neutral hatched tile with a one-line "Outside saved area" hint, never a crash or infinite spinner.

Entry points: a "layers/offline" button on `RocketMapView`'s existing floating control stack, and the DriftCast map toolbar.

---

## 5. Risks & mitigations

| Risk | Mitigation |
|---|---|
| **Imagery looks worse than Apple** (the big one) | Trial 0 on-device A/B *before* any build-out; ship USGS topo too; keep Apple as default online layer. |
| Tile-count / size explosion | center+radius+capped zoom; estimate before download; warn over threshold; per-region + total caps. |
| `Caches/` purged → offline data vanishes at the field | store in Application Support, excluded-from-backup. |
| Provider ToS / rate limits | USGS (public domain) as primary; bounded concurrency; attribution; verify Esri terms before using it for *saved* tiles. |
| Storage bloat over time | region manager with sizes + delete; show total; optional age-based prompt. |
| Jank from disk reads while panning | async `loadTile`, in-memory LRU on top of disk; measured in Trial 1. |

---

## 6. Sequencing summary

1. **Trial 0** — source A/B behind a flag in `RocketMapView`. *Gate: worth it?*
2. **Trial 1** — read-through cache; airplane-mode render.
3. **Trial 2** — downloader + estimate + progress; download a field, verify offline.
4. **Phase 3** — share into DriftCast + 3D texture.
5. **Phase 4** — region store, manager UI, offline indicator, polish.

## 7. Open questions

1. ~~Primary saved source~~ — **Resolved (Trial 0):** Apple + USGS (Imagery / Imagery+Topo / Topo); USGS-only caching; Esri dropped; international deferred to [#231](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/231).
2. Acceptable default size cap / warning threshold per region?
3. Region selection: center **+ radius** (recommended) vs. free-draw rectangle?
4. Should "Save this area" auto-suggest a region centered on the DriftCast launch point / last rocket fix?
5. File a GitHub tracking issue now (to number this plan per the `docs/plans` convention)?
