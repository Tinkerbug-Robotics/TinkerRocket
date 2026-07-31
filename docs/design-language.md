# TinkerRocket Cross-Platform Design Language

*Started 2026-07-27, during the Android Phase 3 build-out.*

Both apps stay native (SwiftUI / Compose).  **iOS is the design reference**
(decision 2026-07-30, superseding the earlier idiom-first stance): Android
aligns closely with the existing iOS app in content AND look-and-feel —
same hierarchy, same semantic colors, same component shapes — except where
platform differences make that difficult or unnatural.  What stays native:
navigation mechanics (back gesture/bar vs swipe-back), touch feedback
(ripple vs highlight), system dialogs and permission sheets, and fonts
(SF Pro is Apple-licensed; Roboto renders the same sizes/weights).

## Design tokens (the mechanism)

`design/tokens.json` is the single source of truth: the Apple system-color
values the iOS app has always rendered, lifted into semantic roles
(savedFlights, driftCast, scan, statusScanning…), plus the surface family
(systemGray6 cards), the TinkerRocket radius (10), and the spacing scale.
`tools/gen_design_tokens.py` generates `DesignTokens.swift` and
`DesignTokens.kt`; CI (`docs` job) fails if the generated files drift from
the JSON.  Screens reach for roles — never raw palette colors — and Android
composes the shared component vocabulary in `app/theme/DesignSystem.kt`
(TrActionButton, TrCompactButton, TrStatusPill, TrCard, TrSignalBars),
which are Compose renderings of the shapes the iOS app is built from.

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
| Section order | **iOS layout is canon** (user decision 2026-07-31), one deliberate exception: the power section stays near the top with the full dashboard visible in the OFF state (Android behavior — power on without leaving the screen; iOS still swaps to a battery+PowerOn-only view when off). Order: identity header → staleness banner → state banner → **power** → flight summary (Current/Max) → battery/GNSS/link row → bearing + roster (BS links) → status → flight-event flags → sensor health → pyro → controls → tools. Storage bar lives on the FILES screen (iOS placement), not the dashboard |
| Flight-event flag chips | LAUNCH · BURNOUT · APOGEE · LANDED as chips that **illuminate green** as each event latches (born on Android 2026-07-27; on both platforms since 2026-07-30, same lit green #2E7D32). NO LOG chip on either platform since 2026-07-30 — the Status card owns logging on both |
| State banner | one big colored label; #382 mapping (READY renders as PRELAUNCH); the COLOR alone carries acquiring-vs-ready (orange/green). No text badge under the label (removed 2026-07-31 — "EKF Ready" beside an amber EKF dot read as a contradiction; the sensor dots are the authority on live quality) |
| Staleness | worsen-only overlay; red banner wording "STALE DATA — link degraded"; never silently show stale values as live |
| Pyro tiles | green **only** on continuity AND live data; gray = open/stale; dark gray = fired; "ARMED" in the section title when armed |
| Sensor health | one dot per subsystem, green/amber/red/gray = OK/DEGRADED/BAD/NA, horizontal row |
| Power section | power state text + single action button; **hidden behind the #377 gate** until telemetry confirms state |
| Color semantics | green = good/latched, amber = degraded, red = bad/stale-alert, purple = focused/selected, gray = inactive/unknown |
| Focused rocket | purple-filled chip labeled "focused"; unfocused chips show their live state text |

## Terminology (both platforms)

"Rocket power" (not "FC power"), "Rockets via base station", "focused",
"cont / open / fired", state strings verbatim from firmware (READY,
PRELAUNCH, INFLIGHT, LANDED… — there is no DESCENT state; descent is
INFLIGHT + the apogee flag, a lesson the virtual rocket's announcer outage
taught on 2026-07-30).

## Back-port queue (Android → iOS)

| Item | Status |
|---|---|
| Flight-event flag chips on the dashboard | **done 2026-07-30** — `FlightEventFlagsView` |
| Sensor-health dot row (one dot per subsystem, green/amber/red/gray = OK/DEGRADED/BAD/NA, name under each dot) | **done 2026-07-30** — `HealthDotRow` replaced the labeled grid inside HealthCardView; the go/no-go banner stays (iOS-only, on the Android queue) |
