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
the JSON.  **Android screens reach for roles — never raw palette colors.**

**iOS does the opposite, on purpose (decision 2026-08-12).**  It keeps calling
SwiftUI's `.green`/`.red`/… directly and never reads a color token.  Those
literals ARE the system colors, so iOS tracks Apple's retunes and Increase
Contrast for free; a frozen hex tracks neither.  The tokens are a
*transcription of iOS for Android's benefit*, not an independent brand
palette — so pointing iOS at them would make the reference implementation
read back its own copy, and a stale one.  It does go stale: iOS 26 retuned
seven of the nine entries (purple `#AF52DE` → `#CB30E0`).  Re-run
`tools/capture_ios_palette.swift` after each major iOS release.

The split is by *kind*, not by file: colors have a live system value, so iOS
takes them from the OS; shapes and spacing do not, so iOS shares those
(`TRShape.radiusButton`) and only Android's colors come from the JSON.

Android composes the shared component vocabulary in `app/theme/DesignSystem.kt`
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
| Pyro tiles | iOS tile form on both platforms (2026-08-09): systemGray5 tile, "CH n" + badge ladder — FIRED beats CONT/NO CONT, which renders only while armed or for 5 s after that tile's Test Continuity tap (single-reveal state), with a TESTING spinner during the 2.5 s cmd-35 round trip; continuity trusted **only** from a LIVE frame (#297 — stale fails safe to NO CONT); per-channel trigger text ("%.1fs after apogee" / "%.0f‹unit› on descent") or "Disabled" from the cmd-20 readback; Test Continuity hidden while armed/fired/INFLIGHT. Android keeps "ARMED" in the section title (deliberate addition atop the iOS form); iOS keeps tap-tile-to-configure (Android edits pyro in Settings) |
| Sensor health | one dot per subsystem, green/amber/red/gray = OK/DEGRADED/BAD/NA, horizontal row |
| Power section | power state text + single action button; **hidden behind the #377 gate** until telemetry confirms state |
| Color semantics | green = good/latched, amber = degraded, red = bad/stale-alert, purple = focused/selected, gray = inactive/unknown |
| Focused rocket | purple-filled chip labeled "focused"; unfocused chips show their live state text |

## Map conventions (settled 2026-08-10)

| Element | Rule |
|---|---|
| Floating controls | **Nothing renders directly on tile imagery.** Every badge, button, and label over a map sits on a plate — iOS `.ultraThinMaterial`, Android the systemGray6 token fill (`TrMapPlate`; Compose has no live-blur, and opaque is the stronger choice over imagery anyway). Contrast over imagery is a coin flip you lose: a blue-on-imagery source chip was unreadable over pale desert satellite while the plated attribution bar two corners away was perfect (Pixel 8, 2026-08-10) |
| Basemap selection | A **menu** listing every source, the active one checked, one glyph per source — never a cycling button. A control that reveals one of five options per tap is how a shipped basemap reads as missing |
| Offline distinction | `TileSource.pickerLabel` on both platforms: "(online)" appended to any source a saved offline area cannot cover (provider terms). Shown on every source row AND on the active-source plate — it is what "will this still draw at the launch site" depends on. "Manage offline maps…" is the last item of the same menu, where it reads as the answer to those markers |
| Active basemap | Named on the **bottom-leading attribution plate** (source name bold, provider attribution under it), not on the control that changes it — which is a bare 44pt/44dp glyph |
| Control column | Top-trailing, source picker above recenter, 44pt/44dp square plates, 12pt/dp gutter. Divergence: Android hides recenter while follow is live (it would be a no-op); iOS always shows it |

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
