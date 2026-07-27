# TinkerRocket Cross-Platform Design Language

*Started 2026-07-27, during the Android Phase 3 build-out.*

Both apps stay native (SwiftUI / Compose) and keep platform idioms —
"similar look and feel" means a shared **design language**, not identical
widgets: the same information hierarchy, section order, color semantics, and
terminology, so a user moving between phones never has to re-learn the app.

## When alignment happens

- **Android screens align as they are built** (now) — each new screen follows
  this doc, which is cheaper than reworking later.
- **iOS back-ports** happen as small standalone changes when Android
  introduces something better (the reverse of the port's usual direction).
  Each is tracked in `docs/android-parity-ledger.md` under **UI parity** so
  neither platform silently drifts.
- A full side-by-side pass (screenshots of every screen pair) happens at
  Android v1.0 — divergences either get fixed or get a documented rationale.

## Dashboard conventions (settled)

| Element | Rule |
|---|---|
| Section order | identity header → staleness banner → state → power → battery/GNSS/link stat row → relayed rockets (BS links) → flight-event flags → sensor health → pyro → altitude |
| Flight-event flag chips | LAUNCH · BURNOUT · APOGEE · LANDED · LOG as chips that **illuminate green** as each event latches (born on Android 2026-07-27; **back-port to iOS pending** — ledger) |
| Staleness | worsen-only overlay; red banner wording "STALE DATA — link degraded"; never silently show stale values as live |
| Pyro tiles | green **only** on continuity AND live data; gray = open/stale; dark gray = fired; "ARMED" in the section title when armed |
| Sensor health | one dot per subsystem, green/amber/red/gray = OK/DEGRADED/BAD/NA, horizontal row |
| Power section | power state text + single action button; **hidden behind the #377 gate** until telemetry confirms state |
| Color semantics | green = good/latched, amber = degraded, red = bad/stale-alert, purple = focused/selected, gray = inactive/unknown |
| Focused rocket | purple-filled chip labeled "focused"; unfocused chips show their live state text |

## Terminology (both platforms)

"Rocket power" (not "FC power"), "Rockets via base station", "focused",
"cont / open / fired", state strings verbatim from firmware (READY,
INFLIGHT, DESCENT, LANDED…).

## Back-port queue (Android → iOS)

| Item | Status |
|---|---|
| Flight-event flag chips on the dashboard | pending — small DashboardView addition |
