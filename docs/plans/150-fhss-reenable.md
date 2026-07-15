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
- BS→rocket **mode resync**: if frame evidence contradicts the BS's mode for
  ≥6 consecutive frames (10 s cooldown), the BS re-uplinks cmd 17. Rendezvous
  visits and coordinated-scan pauses legitimately stamp `NO_HOP`, hence the
  consecutive-frames filter.

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

## 7. Reliability notes (what re-enabling actually risks)

- #133's redesign was never field-validated; the biggest hazard family is the
  DIO1 edge-ISR/level-poll flag protocol around `hopToFrequencyMHz()` — the
  same family as the #520 rx_done double-latch (root-fixed in #521 for the
  steady-state RX path). The TX-stuck watchdog mitigation stays; Bench Seam A
  soaks the hot retune path on both backends (direct SPI and UART modem).
- Stale NVS `hopdis=0` from pre-#150 cmd-17 experiments would have booted
  devices straight into hopping once the boot force lifted —
  `LORA_NVS_SCHEMA_VERSION` v4 wipes it; defaults flip to disabled.
- The #133-era nid regression (schema wipe → BS nid=0 vs rocket nid=180 →
  silent 100 % packet drop) is triple-guarded: identity NVS migrates instead
  of wiping, the drop counter is user-visible (`nidd`), and the network-name
  UI shows the device-readback nid rather than an app-local copy.

## 8. Bench evidence

⏳ Seam A (firmware, pre-app): fixed-mode regression, SF8 dwell-3 soak ≥ 30 min
(crc_fail / low_snr / TX-wdog / drift-repush / observed-loss), SF9 dwell-1
soak with heartbeat traffic, OC-reboot resync heal, UART-modem pair soak,
BS power-cycle during LANDED-hop.

⏳ Seam B (E2E with app): onboarding + provisioning with network name, mode
picker both directions, mid-hop manual scan (cmd-16 pause → cmd-15 mask →
floor holds), deliberate nid mismatch → `nidd` visible instead of silent
blackout.
