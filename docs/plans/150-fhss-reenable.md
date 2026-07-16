# #150 — LoRa frequency hopping re-enable: design + regulatory analysis

Status: implementation on branch `lora/150-fhss-gui`. Bench evidence sections
are filled in after Seam A/B runs (placeholders marked ⏳).

This is the design record #150 asked for: the worldwide regulatory survey, the
FCC compliance math for the existing #133 hop scheme, the concrete diffs needed
to make it compliant, and the alternatives we rejected. The user-facing change
is an iOS **link-mode picker**: *Fixed channel* (today's #136 scheme, still the
default) vs *Frequency hopping* (the #133 scheme, re-enabled and hardened).

## 1. What ships

- `lora_hop_disabled` (cmd 17, end-to-end plumbing already existed) becomes a
  user-facing link mode. Boot honors NVS (`hopdis`, default **1** = fixed);
  the #136 boot force is gone. The #136 rendezvous-preset boot force stays —
  fixed-mode behavior is unchanged.
- **Adaptive dwell** replaces the compile-time `LORA_HOP_DWELL_PACKETS = 4`:
  `loraHopDwellForConfig(sf, bw, frame_len, cr)` derives packets-per-channel
  from real airtime, clamped to [0..4]; 0 = hopping not permitted at that
  modulation (firmware refuses cmd-17 enable, app greys the option via the
  `lhdw` config-JSON key).
- `shouldHopInState()` gains LANDED (2 Hz telemetry continues after landing;
  a fixed channel would carry ~2.1 s of airtime per 10 s — 5× the FCC bound).
- Name beacons are suppressed while hopping (they would ride hop channels the
  BS already follows, and at SF10 a single beacon inside a dwell visit blows
  the occupancy line).
- Channel set: hopping uses **all channels** by default (69 at BW250); the
  restored Frequency Scan view can push a noise skip-mask (cmd 15) whose
  active count is floored at `loraFhssMinChannels` (50 at BW ≤ 250 kHz).
- Network-ID flow restored (name UI → fnv1a8 → cmd 41 push; boot force
  lifted; identity NVS migrates instead of wiping; nid-mismatch drops are
  counted and surfaced to the app as `nidd`).
- BS→rocket **mode resync**: if frame evidence contradicts the BS's mode, the
  BS re-uplinks cmd 17 (10 s cooldown). Direction A (BS hopping, rocket says
  `NO_HOP` in a hop state) needs a 6-frame streak; direction B (BS fixed,
  rocket announces hopping) acts immediately.
- **`0xFE` off-schedule sentinel** (review fix): rendezvous-visit and
  scan-pause frames stamp `next_channel_idx = 0xFE` ("hopping, momentarily
  off-schedule") instead of `NO_HOP`. A fixed-mode BS gets its direction-B
  evidence exactly while the rocket is parked and *listening* on the shared
  rendezvous — the one window a cmd-17 push can land — and a following BS
  knows not to misread the visit. Requires both ends flashed together (same
  coordinated-re-flash rule the dwell change already imposes).
- **Graceful hop disable** (review fix): a cmd-17 disable taken while the BS
  is following a hop drains its 8 uplink retries *on the hop channel* before
  the BS flips, retunes, and sends the config readback (≤1.5 s). The naive
  order retuned away before the first retry fired, so the rocket essentially
  never heard the disable.
- **CR link gate** (review fix): dwell is CR-dependent, but a CR-only cmd-10
  change is unverifiable over the air (explicit-header RX decodes any payload
  CR, so the transaction's commit criterion is CR-blind) — a one-sided commit
  would silently give the two ends different dwells. Hopping is therefore
  only offered at the factory CR (4/5); any other CR computes `lhdw` = 0.
- **Implicit home-channel skip** (review fix): hop-session transition packets
  (activation bootstrap, visit/pause re-bootstraps) transmit on
  `lora_freq_mhz` outside the schedule. If that frequency is also a hop-grid
  channel, a transition packet plus a scheduled dwell visit could stack past
  the 400 ms occupancy line in one window — so both ends implicitly skip the
  coincident grid slot (withheld only if it would breach the channel floor).
  The factory 915.0 is off-grid at BW250, so today's default is unaffected.
- **cmd-60 scan gating** (review fix): the coordinated-pause scan path is
  only taken in hopping mode — in fixed mode a recently-heard LANDED rocket
  (LANDED now being a hop state) would otherwise trigger a pointless cmd-16 +
  a RESUMING stall that #136 never had.

## 2. Regulatory survey (worldwide, per tinkerbug's request)

| Region | Rule | Hopping requirements | Power | Consequence for us |
|---|---|---|---|---|
| US — FCC 15.247 | 902–928 MHz | 20 dB BW ≥ 250 kHz → **≥ 25 channels**, ≤ 0.4 s occupancy per frequency per **10 s**; 20 dB BW < 250 kHz → **≥ 50 channels**, per **20 s**. Spacing ≥ max(25 kHz, 20 dB BW); hop channel BW ≤ 500 kHz | 0.25 W (25–49 ch), **1 W (≥ 50 ch)** | The design target. We hold ≥ 50 active channels at BW ≤ 250 → 1 W-tier-ready (chip max is 22 dBm today). |
| Canada — ISED RSS-247 | 902–928 MHz | Harmonized with FCC 15.247 | same | US design carries over unchanged. |
| EU — ETSI EN 300 220-2 | 863–870 MHz | FHSS requires ~≥ 47 channels of ≤ 100 kHz occupied BW, **and duty-cycle limits (0.1 %/1 % by sub-band) still apply to the whole transmission** (or polite spectrum access LBT+AFA) | ≤ 25 mW typical; 500 mW only in 869.4–869.65 @ 10 % | Hopping buys nothing in EU — wrong band, wrong channel width, and no duty/power relief. EU operation = fixed sub-band + duty cycle. Hopping stays a region-gated feature; our 902–928 plan is US/CA-only. |
| Australia — ACMA LIPD 2025 | 915–928 MHz | FHSS transmitters **and** digital-modulation transmitters both authorized | 1 W EIRP either way | Hopping optional (no power benefit). Needs a 915–928 channel subset — a future channel-plan table entry, not a redesign (the plan constants are already centralized). |
| New Zealand — GURL (short-range devices) | 915–928 MHz | similar to AU (verify before enabling) | ~1 W | Same subset as AU. |
| Japan — ARIB STD-T108 | 920–928 MHz | Channelized, LBT (≥ 128 µs carrier sense) | 20 mW class | Hopping is not the compliance mechanism; out of scope. |

Key corrections to #150's framing that came out of this survey:

1. **The occupancy limit binds transmit airtime, not wall-clock residency.**
   "0.4 s within 10 s" is the accumulated time *transmitting* on a frequency
   inside the window. Dwelling (radio tuned, mostly listening) is free.
2. **The issue's "0.4 s in 20 s" is the < 250 kHz case.** At our 250 kHz
   channels the window is 10 s (still 0.4 s occupancy).
3. **Fixed-channel 250 kHz LoRa is not DTS.** 15.247(a)(2) digital-modulation
   (DTS) operation requires ≥ 500 kHz of 6 dB bandwidth. A fixed 125/250 kHz
   LoRa carrier falls to 15.249 (≈ 0.75 mW ERP) or amateur Part 97 (licensed).
   The in-tree comments claiming "we operate as DTS, no dwell limit" were
   wrong in both directions and have been rewritten. Practical upshot:
   **hopping is the properly-compliant higher-power mode**; fixed mode is the
   low-power / EU / bench mode — which is exactly why both stay selectable.
4. BW500 fixed-channel *is* DTS-eligible (≈ 500 kHz 6 dB BW) — the one preset
   where fixed mode legitimately reaches 1 W. Worth remembering if a "Fast"
   preset returns.

## 3. FCC compliance math for the #133 scheme

Operating point: band plan 902–928 MHz, spacing 1.5×BW (69 channels at
BW250), SF8/BW250/CR4:5, preamble 12, 66-byte telemetry frame
(`sizeof(LoRaData)`, static-asserted), unconditional 2 Hz cadence.

Airtime (Semtech AN1200.13, `loraTimeOnAirMs`, shared by the OC scheduler, BS
follower, and the host tests):

| Modulation | ToA (66 B) | Dwell (≤ 390 ms budget) | Per-visit occupancy | Notes |
|---|---|---|---|---|
| SF7/BW250 | 64 ms | 4 | 256 ms | |
| SF8/BW250 (default) | 112 ms | 3 | 336 ms | old fixed dwell=4 → 448 ms = **over the line**; this is the #150 dwell fix |
| SF9/BW250 | 203 ms | 1 | 203 ms | +2.5 dB range rung |
| SF10/BW250 | 386 ms | 1 | 386 ms | **+5 dB long-range rung**, 14 ms margin — frame growth breaks this first (gtest-pinned) |
| SF11+/BW250, SF9+/BW125 | > 390 ms | 0 | — | hopping refused; fixed mode only |
| SF7/BW125 | 127 ms | 3 | 381 ms | 20 s window applies (BW < 250) |
| BW500 SF7–SF11 | 32–345 ms | 4/4/3/2/1 | ≤ 345 ms | |

- **Budget = 390 ms, not 400**: headroom for formula-vs-measured drift while
  keeping the SF10/BW250 rung. (An earlier plan draft said 380, computed
  against a stale 62-byte frame note; the real frame is 66 B → 386 ms, so 380
  would have silently dropped the long-range rung.) The binding check is the
  `ComplianceInvariant` gtest: `dwell × ToA ≤ 400 ms` for every configurable
  modulation.
- **Revisit bound**: per-visit occupancy is sufficient because a full cycle
  (`n_active × dwell / 2 Hz`) outlasts the window even at the floor count:
  worst case 25 × 1 / 2 = 12.5 s ≥ 10 s (BW500), 50 × 1 / 2 = 25 s ≥ 20 s
  (BW125). Gtest-pinned. **Raising the telemetry rate re-opens this** —
  BW125 hopping stops being legal near 4 Hz; the tests carry that warning.
- **Equal channel use**: the seq-anchored round-robin is exactly uniform over
  whole cycles from any epoch (reboot/seq-reset included); the u16 wrap adds
  at most one dwell-window of spread per 65536-packet epoch. Both gtest-pinned.
- **Channel count**: 69 ≥ 50 at BW250 with an empty mask; `loraSelectChannelSet`
  floors masked sets at 50 (BW ≤ 250) / 25 (BW500). ≥ 50 active also satisfies
  the 1 W FHSS power tier — relevant only if a PA ever lands (LLCC68 tops out
  at 22 dBm ≈ 158 mW, inside the 0.25 W tier regardless).
- **Hopping in all transmitting states**: PRELAUNCH/INFLIGHT/LANDED all hop.
  INIT/READY (and MAG_CAL) remain on the rendezvous channel: that is the
  acquisition phase, and beacon cadence there is the fixed-mode story, not
  the FHSS story.
- **BS side**: the BS transmits only inside the 150 ms post-RX safe window on
  the channel it follows, ~one heartbeat per 10 s plus user commands — far
  under the occupancy line on any single frequency.

## 4. Range policy (user decision, 2026-07-15)

Adaptive dwell keeps SF8→SF10 @ BW250 hopping-legal — the "BW250 ladder":
SF9 ≈ +2.5 dB, SF10 ≈ +5 dB over today, and SF10/BW250 sits only ~3 dB short
of the old BW125 MaxRange preset (halving BW costs 3 dB of sensitivity but
halves airtime — the winning trade under a dwell cap). The true max-range
corner (BW125 @ SF10+, SF11+) cannot fit a single packet in the window and
stays **fixed-channel only**, with the GUI explaining why.

With a future 1 W front-end, hopping SF10/BW250 @ 30 dBm out-ranges fixed
SF10/BW125 @ 22 dBm by ~5 dB — long-term, hopping is the range play, not the
range sacrifice.

## 5. Rejected alternatives

- **App-layer packet fragmentation** (split the frame across 4–5 short
  packets on different channels to reach BW125): the preamble dominates at
  high SF — a 16-byte fragment at SF10/BW125 still costs ~290 ms, the frame
  needs 4–5 fragments ≈ 1.2–1.4 s total airtime, telemetry falls to ~0.5 Hz,
  any lost fragment costs the whole frame, and both ends grow a
  fragmentation/reassembly protocol. ~3 dB for a lot of machinery. Rejected.
- **Automatic post-acquire noise scan** (the original #133 flow): re-adds the
  scan state machine to the critical path for marginal default-channel
  quality. Hopping starts on the full channel set instead; the manual scan
  remains available. (Scan-and-move helpers stay in source, unused.)
- **Reducing preamble or trimming the frame to widen the dwell budget**:
  coordinated-reflash risk on the acquisition path for margin we don't need
  once the budget is airtime-derived.

## 6. Future path: true mid-packet FHSS on an SX1276 daughterboard

The LLCC68 (SX126x family) cannot retune mid-packet (`SetRfFrequency` is
standby-only), and Semtech's LR-FHSS is transmit-only — neither carries a
two-way link. The **SX127x** line is the only LoRa silicon with genuine
intra-packet hopping: `RegHopPeriod` sets a hop period in symbols, the radio
raises `FhssChangeChannel` at each boundary, the host retunes while the modem
keeps demodulating, and the receiver follows in lockstep from the shared
table.

The V8 radio daughterboard (#409, `projects/radio_board/`, `IRadioLink` UART
modem) is the natural home: an SX1276 board variant services the hop
interrupts locally on the daughterboard (hard-real-time work stays off the
host), exposes the same UART protocol plus a hop-table/config message, and
makes SF11/BW125 — with care SF12 — hopping-legal at up to 1 W with the
130-channel BW125 plan: roughly +8–12 dB over the BW250 ladder top (2.5–4×
distance). Caveats recorded now: at SF12 the preamble alone dwells ~0.5 s on
the start channel before hopping begins (needs short-preamble or
start-channel rotation; SF11 is clean), and packets that long mean ~0.3 Hz
telemetry — a tracking/recovery mode, not live telemetry.

## 7. Reliability notes + adversarial review outcome

A 27-agent adversarial review (5 dimension-focused finders, every finding
verified by 2 independent skeptics) ran over the firmware diff before bench
time. 9 findings confirmed, all addressed:

1. **nid resurrection (critical, fixed)**: the #136 nid boot-force was
   RAM-only, so fielded devices still store the divergent #133-era nids
   (rocket 180 / BS 0) that the force had masked for months. Deleting the
   force would have made them live on the first post-flash boot — a total,
   silent link kill with no over-the-air recovery (both directions are
   nid-filtered). Fix: the identity v0→v1 migration resets nid to the
   compile-time default exactly once (the pre-upgrade *effective* nid was
   always the default, so this is the behavior-preserving migration); `un`
   and `rid` are preserved.
2. **Direction-B resync undeliverable (critical, fixed)**: visit/pause frames
   stamped `NO_HOP`, and 915.0 is off the hop grid, so a fixed BS never got
   usable evidence nor a delivery window. Fixed by the `0xFE` sentinel: the
   evidence arrives on frames sent while the rocket is parked on the
   rendezvous *listening*, and the push queued off that frame lands in the
   same visit.
3. **cmd-17 disable ordering (critical, fixed)**: the BS retuned to the
   static channel before its own disable retries fired. Fixed by the
   graceful drain (retries complete on the hop channel first).
4. **BS power-cycle rejoin only at factory rendezvous (major, mitigated)**:
   a rebooted BS re-acquires a hopping rocket via the post-visit bootstrap on
   `lora_freq_mhz` — which works whenever the operating channel is the
   factory 915 (today: always; there is no shipped UI that moves it).
   Residual corner once the scan UI returns: if the session moved the
   channel and the BS reboots, telemetry degrades to visit bursts until the
   *rocket* is power-cycled — the retained rendezvous boot force then resets
   `lora_freq_mhz` to 915 and the pair re-converges. Documented, accepted.
5. **LANDED scan parity break (minor, fixed)**: cmd-60 gated on
   `!lora_hop_disabled` so fixed-mode post-flight scans run direct, as in
   #136.
6. **CR-dependent dwell divergence (major, fixed)**: CR link gate (above).
7. **Bootstrap occupancy stacking (major, fixed)**: implicit home-channel
   skip (above); the one-time activation bootstrap on the off-grid factory
   915 cannot stack with any scheduled visit.
8/9. Duplicates/variants of 4 and 5 — covered by the same fixes.

Refuted by the verifiers (no change): the direction-A streak-vs-visit-length
concern (visit frames no longer stamp `NO_HOP` at all), and a suspected
stale-modulation window in the hop-follow during cmd-10 VERIFYING.

Standing reliability notes:

- #133's redesign was never field-validated; the biggest hazard family is the
  DIO1 edge-ISR/level-poll flag protocol around `hopToFrequencyMHz()` — the
  same family as the #520 rx_done double-latch (root-fixed in #521 for the
  steady-state RX path). The TX-stuck watchdog mitigation stays; Bench Seam A
  soaks the hot retune path on both backends (direct SPI and UART modem).
- Stale NVS `hopdis=0` from pre-#150 cmd-17 experiments would have booted
  devices straight into hopping once the boot force lifted —
  `LORA_NVS_SCHEMA_VERSION` v4 wipes it; defaults flip to disabled.
- Post-upgrade nid drift can no longer be silent: identity NVS migrates (with
  the one-time v0 reset above), the drop counter is user-visible (`nidd`),
  and the network-name UI shows the device-readback nid rather than an
  app-local copy.
- Mixed-version fleets are NOT supported across this change (0xFE sentinel +
  dwell derivation both moved): flash BS and rocket together — the same rule
  #133's shared-dwell constant already imposed.

## 8. Bench evidence

### Seam A — RUN 2026-07-15 (BS V2 + OC V7 full rocket stack, ~2.5 h): PASS

- **Boot/migration**: schema-v4 wipe + identity-migration *effects* verified
  (nid=0, hopdis=1 defaults, schema stamps current); the one-time migration
  log lines raced past capture attach both times (USB-CDC port timing) —
  verified via readback instead.
- **Fixed-mode regression**: byte-identical to #136 — 2.4 Hz RX, 0 CRC
  fail, 0 netid drop, heartbeats + TX-window (learned cadence 501–505 ms).
- **SF8/dwell-3 soak**: 30 min, **3,596 packets, 0 loss, 0 CRC, 0 TX-wdog,
  0 silence, 0 drift-repush** across ~1,200 live retunes (no #521-class
  incident). Beacon suppression held the entire session (last beacon RX =
  the second hopping engaged).
- **SF9/dwell-1 sessions**: 1,123 + 593 + more packets, all 0 loss —
  hop-every-packet timing holds. Adaptive dwell observed live: `lhdw`
  3 ↔ 1 tracked cmd-10 SF changes, computed by the firmware itself.
- **cmd-10 transactional moves**: SF8→SF9→SF8, verify-on-NEW + COMMIT both
  directions.
- **Graceful disable drain**: multiple clean passes — `draining retries on
  the hop channel first` → rocket applied same-second → `drain complete —
  fixed mode` ≤1.5 s.
- **BS power-cycle mid-hop**: fully autonomous recovery in ~70 s (silence
  fallback → rendezvous visit → post-visit bootstrap re-anchor), hop mode
  persisted through the reboot via NVS.
- **OC reboot mid-hop** (via reflash): BS silence-fallback fired, full
  re-acquire after power-on + GPS refix; recovery push-home reconciled the
  SF split (boot-forced SF8 rocket vs SF9 BS) as designed.
- **0xFE machinery exercised live**: visit frames + post-visit bootstrap
  rejoins after both power-cycle scenarios.
- **Bench findings → fixed same session** (commit 51f134d): (1) enable
  handoff collided with the BS's own cmd-17 mirror-retry train → rocket
  now defers activation 1.5 s; (2) the single bootstrap frame was eaten by
  three different BS-deaf windows (retry train, heartbeat TX, recovery
  reconfigure racing a slow-rendezvous quiet window) → every (re)bootstrap
  now sends a dwell-count of packets announcing the schedule-entry channel
  (occupancy = one dwell visit, FCC-neutral); (3) heartbeat-vs-bootstrap
  race → BS holds heartbeats 4 s after sending an enable. Post-fix enable
  handoff measured at ~1 s (activation → BS follow).
- **Open follow-up (issue chip filed)**: dwell-1 uplink *burst* delivery is
  intermittent — two 8-retry cmd-17 trains were fully missed while
  heartbeats delivered ~50%; self-heals ≤60 s via visit + resync, and the
  flight-standard SF8/dwell-3 preset is unaffected (338/338-session uplink
  health). Also observed (pre-existing, not #150): the BS recovery cycle
  and the rocket slow-rendezvous duty cycle can starve each other's
  windows around state transitions.
- **Not run**: BS V3 + OC V8 UART-modem pair (V8 radio daughterboard
  parked on the respin); a full LANDED sim-flight variant — the LANDED
  machinery is identical to the PRELAUNCH cases exercised, deferred to
  Seam B.
- End state: both ends restored to flight-standard 915/SF8/fixed, all
  counters clean.

### Dwell-1 uplink fix — VERIFIED 2026-07-15 (commit ed6a41d)

Root cause of the burst misses: the flat `UPLINK_RX_RESERVE_MS = 140` was
sized for SF8's ~82 ms downlink; at SF9 (~203 ms) the TX-window gate
sanctioned attempts colliding with the head of the next downlink, and at
dwell-1 one collision breaks the hop-follow.  Fixed with an
airtime-derived reserve + a no-TX-while-retune-pending guard.  Automated
repro (3 disable/enable cycles at SF9/dwell-1): every cmd-17 delivered
same-second (was 0/8 per burst), BS followed each re-activation at +1 s.
Mode toggles: ~40–75 s → ~1–3 s.

### iOS Phases 4–5 — DONE (commit b0c9d9a), adversarially reviewed

32-agent review, 12 confirmed findings, all fixed pre-commit — notably:
Frequency-Scan Auto-Apply and the TX-power stepper are hidden/locked
while hopping (both ride cmd 10, which the firmware refuses mid-hop and
the app previously reported as success); the network section + sync is
device-agnostic (a BS-only sync could split the fleet with no in-app fix
for a drifted rocket); `networkIdForName` remaps fnv1a8's 1-in-256
zero-hash to 1 (0 collides with the firmware default and the app's unset
sentinel); the hop/nidd readouts live on the real dashboard card (first
draft put them on a never-instantiated view).  Simulator suite: 181/181.

### Seam B — RUN 2026-07-16 morning (app-driven E2E): PASS

1. **Settings surfaces** ✅ — Link Mode picker, FCC footer, TX-power lock
   while hopping, Network section with mismatch badge all rendered and
   behaved as designed (user screenshots).
2. **Mode toggling from the app** ✅ — mode applied <1 s; the observed
   "slow engage" was decomposed into (a) dormant-by-design in READY
   (GPS refix ≈ 2.5 min) and (b) one lost-handoff heal — both now
   narrated by the three-state tile badge (armed / engaging… / active).
3. **Mid-hop Frequency Scan** ✅ — coordinated pause (cmd 16 received
   same-second) → 5-pass scan → `65/69 channels active (min FCC floor
   50)` mask push (cmd 15 received) → resume.  Finding: the BS's push
   landed at the rocket's resume deadline (pause clock starts at cmd-16
   receipt, BS spends ~2 s confirming the park first) → resume-bootstrap
   collision, ~46 s fallback heal.  Fixed: pause slack 2 s → 5 s.
4. **NetID mismatch drill** ✅ — BS synced to 122 while rocket at 0:
   orange dashboard banner with live drop count (64 → 137), stale
   telemetry as expected; rocket-side sync restored the link.  Finding:
   the lifetime counter kept the banner up after healing — fixed by
   reporting drops over BLE only within 30 s of the last actual drop.
   nid persistence across a rocket power-cycle observed (`nid=122` on
   boot).
5. **Full sim flight** ✅ — PRELAUNCH hop engage in ~1 s of the state
   transition (3-packet bootstrap), continuous session through boost
   (max 99 m/s) and descent with **2 packets lost, 1 CRC, 0 wdog, 0 nid
   drops**, and the first live **LANDED-mode hopping** (`nextCh`
   advancing after touchdown) — the FCC change #150 exists for.
   Cosmetic finding fixed: the entry-channel announce in bootstrap
   frames false-fired the mask-drift warn on session-start frames.

Still deferred: UART-modem pair (BS V3 + OC V8) until the V8 radio
daughterboard respin returns.
