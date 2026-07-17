# #390 — Multi-rocket / multi-device model: evaluation & redesign

Status: evaluation complete (2026-07-16); design v3 below. Revisions:
v2 — multiple co-flying rocket computers (e.g. two stages of one vehicle)
are a first-class case, so the app displays a *set* of rockets, not one
"active" rocket. v3 — one phone operating **two base-station/rocket pairs**
(switch between them; no need to view both at once) is a first-class case,
so pair grouping + per-BS focus + network-scoped rocket identity.
Implementation on branch `app/390-multi-rocket-model`.

## 1. Evaluation — how the current model actually behaves

### The concepts today

| Thing | Where | What it really is |
|---|---|---|
| `BLEDevice` | app | one **BLE link** (rocket unit or base station), typed from the advertised name prefix |
| `BLEFleet.activeDeviceID` | app | which **link** the dashboard renders; `nil` falls back to `devices.first` |
| `RemoteRocket` | app | per-rocket telemetry snapshot demuxed from a BS link by `source_rocket_id` |
| `tracked_rockets[4]` | BS fw | per-rocket state (telemetry, seq, per-rocket log/freq-lock state from #381) |
| `active_rocket_idx` | BS fw | **whichever rocket was heard last** — drives BLE forwarding + stale re-push |

Per-rocket *data* plumbing exists end-to-end (tracked slots → `rid` on every
BLE frame → `RemoteRocket`). What is missing is a per-rocket *subject*: both
ends resolve "which rocket am I showing / commanding" by **recency race** or
**broadcast**.

### Confirmed defects / confusions (verified against source)

1. **Two-rocket telemetry flip-flop.** Every relayed frame from *any* rocket
   overwrites the BS link's `device.telemetry`
   (`BLEDevice.parseTelemetryData`, relayed branch) — and BS-side
   `active_rocket_idx = last slot heard` (`main.cpp:4239`). With two powered
   rockets every dashboard card alternates between them at packet rate.
2. **Voice callouts interleave rockets.** The relayed branch feeds
   `flightAnnouncer` for **every** relayed frame regardless of `rid`
   (`BLEDevice.swift:1628`).
3. **All BS-mediated actions are LoRa broadcasts.** Camera (1), logging (23),
   sim (5/6/7), servo test (24/25) go out with `target_rid = 0xFF`
   (`buildUplinkPacket` default). Every rocket in range acts. The OC filters
   by rid — and the targeted relay (BS cmd 50) exists — but the app never
   uses it (`sendRelayCommand` has zero call sites).
4. **Toggle idempotence keys off the wrong rocket.** The camera/logging
   toggles compute desired state from single globals
   (`last_known_camera_recording`, `last_known_rocket_logging`) = last-heard
   rocket, then broadcast. Rocket A's toggle can be computed from rocket B's
   state and applied to both.
5. **Dead control:** "Ground Test" renders enabled on a BS link but the BS
   dispatch has no cmd 15/16 — the tap is silently dropped.
6. **Relayed rockets are invisible in the normal config.** `DeviceChipBar`
   renders only when `devices.count > 1`; `remoteRockets` appear nowhere
   else. Phone↔BS (the standard flight setup) never shows them, and the
   chips are passive (can't select, no actions, no staleness).
7. **Silent subject fallback.** `activeDevice` → `devices.first` when unset;
   on disconnect the active link silently jumps to whatever is first.
8. **Stale/SYNCING status follows the last-heard rocket** (BS re-push uses
   `active_rocket_idx`), so the #95 banner can describe a different rocket
   than the operator cares about.
9. **Hop-follow is single-rocket by construction** (one radio, one
   schedule); with two hopping rockets the BS chases whichever packet
   arrived last (#40/#41 caveat). Needs an explicit "which rocket owns the
   radio" rather than recency.
10. **Profile header vs relayed rocket mismatch.** `ActiveRocketHeader`
    shows the app's active *profile* even when the BS relays a different
    physical rocket; there is no linkage or warning.
11. **Rocket identity ignores the network.** `rocketID` is only unique per
    network id, but the app keys per-rocket state (`lastValidRocketFixes`,
    `RemoteRocket` merge) on `rid` alone — two pairs on different networks
    with the same rid would cross-contaminate (latent today, real once
    two-pair operation is supported).

What is already sound and stays: per-rocket BS logging + CSV `rocket_id`
column + aggregate close policy (#381); `lastValidRocketFixes` latching
(#140); per-device syncer/announcer attach (#375); power-state gating
(#377); read-only BS settings (#285).

## 2. Design v3 — roster / display set / per-BS focus / pairs

Driving use cases, in priority order:
(a) one rocket + one BS (today's normal day);
(b) **multiple rocket computers flying together** (booster + sustainer);
(c) **two BS/rocket pairs, one phone** — view one pair at a time, switch
    freely;
(d) direct BLE to a rocket with no BS, possibly alongside (a)–(c).

Four concepts replace "active device":

1. **Roster** — every rocket known right now, discovered via any link
   (direct BLE and/or BS relay), identified by **(networkID, rocketID)**
   (fixes defect 11; a direct link + a relayed appearance of the same key
   merge into one entry). Always visible when connected.
2. **Display set** (app-side, multi-select) — which roster rockets have a
   live section on the dashboard, within the foreground pair (below).
   Every newly-discovered rocket auto-displays; chips toggle. Pure view
   filter: hiding a rocket never changes radios or commands.
3. **BS focus** (radio-side, exactly one rocket **per base station**,
   sticky) — the rocket that BS's radio is *dedicated* to: hop-follow,
   in-flight heartbeats, stale re-push subject, default target for legacy
   untargeted uplinks. Switched explicitly (chip context menu / BS screen);
   pushed to that BS via cmd 45 and re-sent on every BS connect. Never
   ping-pongs on its own:
   - unset → latches onto the first rocket heard (**first**-heard sticky);
   - focused rocket silent > 30 s AND another rocket fresh → one-way
     fallback (log + UI note), still no back-and-forth;
   - explicit app pin disables even that fallback.
4. **Pair (foreground base station)** — a "pair" is a base station plus the
   rockets it currently carries; it emerges from radio reality, zero
   configuration. With ≥2 base stations connected, a **pair switcher** at
   the top of the dashboard picks which BS's world is on screen (its BS
   strip, its rockets, its display toggles). One tap switches pairs.
   Direct-link rockets that no BS carries are always shown (explicitly
   connected = strong intent). With one BS there is no switcher — layout
   is identical to v2.

Display set, focus, and foreground pair are independent: both stages stay
displayed while the BS radio focuses the sustainer; pair B keeps relaying
in the background while pair A is on screen.

### Two-stage walkthrough (booster rid 1 + sustainer rid 2, one BS)

- Both rockets appear in the roster and auto-display: two dashboard
  sections, each live at its own packet rate.
- Focus on the sustainer. Booster telemetry keeps flowing (fixed-frequency
  mode) — focus only decides hop-follow/heartbeats/re-push/default target.
- "Start Camera" in the booster's section is **targeted** at rid 1 via the
  existing BS relay (cmd 50), desired-state computed from the *booster's*
  telemetry. The sustainer's camera is untouched.
- **Hopping constraint (physics, not software):** one BS radio can follow
  one hop schedule. In hop mode the non-focused rocket is only heard when
  channels coincide → its section goes stale during flight. Dual-stage ops
  should fly fixed-frequency (or a second BS — see pairs). The UI says this
  explicitly on the non-focused section when hopping is armed/active.

### Two-pair walkthrough (BS-A + rocket 1, BS-B + rocket 2)

- Both BSes connect over BLE (the fleet already supports multi-connect).
  Pair switcher appears: [BS-A] [BS-B].
- Foreground BS-A: BS-A strip + rocket 1's section. Switch to BS-B: one
  tap, instant (both streams were flowing the whole time — no
  reconnection).
- Each BS has its own sticky focus (naturally its own rocket — first
  heard). Commands from a rocket's section route: fresh direct link ▸
  foreground BS carrying it ▸ freshest other BS carrying it.
- Same-network pairs co-located: each BS genuinely hears both rockets, so
  both appear in either pair's roster — truthful; hide the other pair's
  rocket via its chip if it's noise. Different-network pairs: rosters are
  naturally disjoint; (nid, rid) identity keeps caches separate.
- Voice callouts follow the **foreground pair's focused rocket**.
- The `netid_drops` warning ("a device is on the wrong network ID") stops
  shouting when the BS is successfully tracking its focused rocket — with
  two different-network pairs at one site, cross-network packets are
  expected, not a fault. (It stays prominent when the BS is tracking
  nothing — that's the original #150 diagnostic.)

### App model

```
RocketKey = (networkID: UInt8, rocketID: UInt8)
RocketSubject (roster entry, Identifiable by RocketKey)
  ├─ key, displayName, unitID?
  ├─ direct: BLEDevice?                    // live direct link, if any
  ├─ relays: [(bs: BLEDevice, remote: RemoteRocket)]
  ├─ telemetry: TelemetryData              // best source: fresh direct ▸ freshest relay
  ├─ freshness: live / stale(age) / lost   // app-computed from per-link lastSeen
  └─ capabilities: full / relayed          // drives per-section control gating
BLEFleet
  ├─ devices: [BLEDevice]                  // links, unchanged
  ├─ rockets: [RocketSubject]              // merged roster (all pairs)
  ├─ baseStations: [BLEDevice]
  ├─ foregroundBSID: UUID?                 // pair on screen (nil = auto/single)
  ├─ displayedRocketKeys: Set<RocketKey>   // default: all discovered
  └─ bsFocus: [UUID: UInt8]                // per-BS radio focus (cmd 45 each)
```

- Roster identity is `(nid, rid)` → BLE reconnects can no longer move or
  duplicate a subject, and two pairs can't collide. A direct link whose
  identity hasn't read back yet (rid 0) shows as an "identifying…"
  provisional entry until it resolves, then merges.
- **The `device.telemetry` conflation dies**: relayed frames update only
  their `RemoteRocket`. A BS link's frames additionally feed a new
  `bsStatus` (BS battery/logging/silence-countdown/netid-drops +
  data-status) — the BS strip reads that, never rocket fields.
  `device.telemetry` remains meaningful only on direct rocket links.
- `lastValidRocketFixes` re-keys to `RocketKey`.
- Voice announcer: foreground pair's focused rocket only (per-rocket voices
  are a follow-up).
- Per-rocket action routing: fresh direct ▸ foreground BS carrying it ▸
  freshest BS carrying it, via targeted relay `cmd 50(rid, inner, payload)`
  with desired-state computed from that rocket's telemetry (fixes defect 4
  app-side, and works on TODAY'S BS firmware — cmd 50 already ships).
- Capability gating per section: relay-only rockets keep Camera / Logging /
  Simulate / Servo Test; Pyro, Power On/Off, Mag cal, Settings edit, rocket
  file downloads are shown disabled with "needs a direct connection"
  (Ground Test becomes direct-only — removes the dead button).

### BS firmware (small, additive)

- `BLE_BS_CMD_SET_FOCUS_ROCKET = 45`, payload `[rid]` (`0` = auto). RAM-only;
  app re-sends on every BS connect (BS reboot bounces BLE, so the pin
  self-heals). Effects while the rid is tracked: `active_rocket_idx` pins →
  stale re-push + `data_status` describe the focused rocket; hop-follow and
  in-flight heartbeats key off it; untargeted uplinks default to it instead
  of `0xFF`.
- Auto mode (no cmd 45, old app): first-heard sticky + 30 s one-way
  fallback replaces last-heard flapping.
- `BLE_BS_CMD_SET_BS_LOGGING = 46`, payload `[on]`: decoupled BS SD-card
  logging control for the BS screen. Legacy cmd 23 (coupled BS+rocket
  broadcast toggle) stays for old apps; the new app stops using it for
  rockets (targeted cmd 50(rid, 23, [desired]) instead).
- Network-wide commands stay broadcast: LoRa reconfig (10), hop mode (17),
  scan mask/park.
- Wire-code checker: add both constants to RocketComputerTypes.h (BS
  namespace free at 45/46; OC-namespace overlap is fine per checker
  policy).

### GUI / IA

- **Pair switcher** (only when ≥2 BSes connected): segmented chips naming
  each BS; selecting swaps the dashboard scope. Auto-foreground the sole /
  first BS; a second BS appearing never steals the screen.
- **Units bar, always visible when connected** (replaces the >1-device chip
  bar): the foreground pair's rocket chips — freshness dot, link badges
  (BLE glyph = direct, antenna glyph = relayed), focus badge (scope glyph)
  on the focused one; **tap toggles display**, context menu = Focus base
  station here / Hide / Disconnect (direct links). Direct-only rockets
  always listed. Then the BS chip (tap → BS detail screen), then "+ Add".
- **Dashboard = BS strip + one section per displayed rocket.**
  - BS strip (thin, foreground BS): name, battery, SD logging state, focus
    ("following <rocket>"), tap → BS screen.
  - Rocket section header: name, source line ("Direct BLE" / "via <BS> ·
    ch7 · 2 s"), staleness age when not live, LOST state (last seen + last
    fix retained) instead of vanishing.
  - One displayed rocket → exactly today's full card stack under the
    header. Multiple → sections collapsible at the header (chevron);
    collapsed shows a one-line summary (state · alt · speed · battery).
    Focused rocket sorts first.
- **BS detail screen** (new): BS battery, storage bar, SD logging toggle
  (cmd 46) + silence countdown, focus picker, frequency scan, network
  settings, BS files, OTA entry. These leave the main dashboard.
- **Map**: one marker per displayed rocket of the foreground pair (+ direct
  rockets), from the (nid,rid)-keyed fix cache; focused rocket highlighted.
- No rockets seen yet (BS only): existing "Searching for rocket…" state
  under the BS strip.

### Out of scope (follow-ups)

Multi-network provisioning/onboarding (the app still provisions new
devices onto its single stored network id; *operating* mixed-network pairs
works because identity/caches are (nid,rid)-scoped), per-rocket voice
profiles, BS-relayed rocket file downloads, profile↔rocket auto-binding UX
beyond the existing suggestion, showing known-but-disconnected BSes in the
pair switcher as one-tap connect targets.

## 3. Implementation order (commits at bench-testable seams)

1. App: RocketKey/roster merge + display set + per-BS focus state + pair
   foreground + per-rocket streams (kill the `device.telemetry` relay
   mirror; add `bsStatus`) + announcer filter + fix-cache re-key + unit
   tests (pure merge/routing/focus logic testable without CoreBluetooth).
2. App: pair switcher + units bar + sectioned dashboard + BS strip + BS
   detail screen.
3. App: per-section capability gating + targeted cmd-50 action routing
   (works against current BS firmware) + netid_drops soften.
4. BS fw: cmd 45 focus pin (+ sticky auto fallback) + cmd 46 decoupled SD
   toggle + hop/heartbeat/re-push pinning + build + host-testable focus
   helper where practical.
5. App: push cmd 45 on BS connect + focus change; map multi-marker.
6. Bench: BS + 2 rockets — both sections live in fixed mode, targeted
   camera/logging per rocket, focus stickiness across silence, stale banner
   describes the focused rocket. Two-BS pair switching if hardware allows
   (V2 + V3 BS).
