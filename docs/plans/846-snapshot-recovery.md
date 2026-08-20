# #846 — In-flight reboot recovery on V9/V10 (port the mini's NAND-stream design)

**Problem:** the FC's reboot recovery (#104) reads its last FlightSnapshotData
back from the OC's MRAM. V9/V10 deleted the MRAM (#822), so recovery is
loudly unavailable there. The mini already solved the same problem for its
single-MCU architecture: snapshots ride the NAND log stream and a boot-time
tail-scan finds the last one. Porting that naively misses a key difference —
on the two-MCU pair, **the FC and OC reboot independently**, and that split
makes most of the problem easier, not harder.

## The case analysis that drives the design

| Who reset mid-flight | What survives | Recovery source |
|---|---|---|
| FC only (its own panic/brownout) | OC RAM | **plain RAM cache** on the OC |
| Both (pack brownout) | NAND | OC boot tail-scan of the brownout-recovered flight → re-seeds the RAM cache |
| OC only | FC keeps flying (#859 hold) | none needed — the FC never asks |

The MRAM's non-volatility was only ever load-bearing in the *both-reset* case.
The common case (FC-only reset — post-#859 the FC's pad-hold even keeps the
rail up through OC faults, so FC resets are the ones that leave a running
system asking for recovery) needs nothing but RAM.

## Design

1. **OC RAM snapshot cache** (all board revisions): the latest SNAPSHOT_MSG
   wire frame + validity, updated in processFrame under the existing #383
   stale-replay guard. `GET_FLIGHT_SNAPSHOT` serves the cache first; the
   V7/V8 MRAM slot becomes the fallback seed, not the primary store.
2. **Snapshots enter the NAND stream** (if they don't already — survey
   confirms): ~230 B at 10 Hz ≈ 2.3 KB/s, the same budget the mini pays.
3. **OC boot re-seed**: when `scanForBrownoutRecovery()` recovered a flight
   THIS boot (the OC itself died mid-flight), tail-scan that recovered
   flight — mini's window/carry algorithm, reused — for the newest INFLIGHT
   snapshot and seed the RAM cache before the FC can ask. Scoping the scan to
   the flight recovered *this boot* bounds staleness structurally: a cleanly
   landed flight is finalized and never scanned; a new flight is a new
   allocation.
4. **FC retries (#364)**: the single 20 ms exchange becomes a bounded retry
   loop; and failure paths stop erasing the snapshot (#834 item 3) — with the
   stream + cache design there is nothing destructive to do on failure.
5. **No wire change**: same SNAPSHOT_MSG / GET_FLIGHT_SNAPSHOT / clear types,
   same frame layout, same FC-side CRC+magic+sim validation.

## V8 bench

Byte-compatible layering: the MRAM slot write stays; the RAM cache sits in
front of it. The only V8-visible change is that recovery keeps working even
if the MRAM read fails — strictly additive.

## Verification plan

- Pure tail-scan policy extracted host-testably (mini's scan is inline in
  flight.cpp; the port gets a shared, tested implementation).
- Host tests over the recovered-flight scan: torn tail, snapshot straddling a
  page boundary, LANDED-last declines, sim-flag propagation.
- Bench (#847): FC-reset-mid-sim → recovery from RAM cache; OC+FC reset
  mid-sim → recovery from NAND tail-scan.

## Implementation record (2026-08-20)

Landed as designed, with survey-driven adjustments:

- **Item (b) was already true**: the OC enqueues every received frame
  byte-exact into the NAND stream before dispatch — the snapshot frames were
  in the flight log all along. Only the cache, the re-seed, and the serve
  changed.
- **"Clear" is a LANDED-overwrite, not an erase** (there is no clear wire
  type): the FC's clearFlightSnapshot sends a LANDED-state SNAPSHOT_MSG. The
  RAM cache inherits that invalidation for free.
- **Re-seed gates** (from the mini's design, adapted): MRAM boards skip; a
  true POWERON never seeds (a cold start must never let last week's recovered
  flight answer a pad-side FC panic with INFLIGHT); the newest
  `flight_recovered_` entry must be the newest flight overall; and an NVS
  marker (`snaprec`: id + final_bytes, since ids are reused after deletes)
  seeds each recovered flight at most once. The FC still applies
  magic/version/INFLIGHT/CRC32/sim validation — every OC gate is belt on
  braces.
- **#364**: five attempts ~350 ms apart (≤ ~1.75 s, unexpected-reset boots
  only), each re-probing `out_ready` (full-length read — a short read desyncs
  the OC's slave TX ring, #399) and sending I2C_TX_RESYNC after a failed
  snapshot read (#402; the bus is idle during setup). A parsed frame —
  restore, sim refusal, or a LANDED clear — is definitive and stops retries;
  only transport failures continue.
- **#834 item 3**: the boot-time clear now fires only on NORMAL boots
  (`!reboot_recovery && !unexpected_reset`) — a failed recovery preserves the
  record for the next attempt.
- **Shared scan**: `TR_FlightLog/include/SnapshotTailScan.h` (framing-only
  validation via the CRC component; last-valid-wins). The mini keeps its own
  copy for now — unifying it is follow-up. Host-tested against the real
  TR_FlightLog at both page geometries (12 tests: straddle, torn tail,
  corrupt CRC, window bound, last-wins).
- **Serve-TTL (self-review catch)**: removing the clear-on-failure (#834-3)
  meant a stale INFLIGHT snapshot could sit in the OC indefinitely — an FC
  WDT reset on the bench hours later would have restored INFLIGHT on the
  ground. The cache now refuses to serve frames older than the FC's
  MAX_FLIGHT_TIME (10 min): beyond that the flight would have timed out to
  LANDED, so serving is definitionally wrong. Live 10 Hz frames refresh the
  stamp, so an active flight never expires; an expired cache also blocks the
  V8 MRAM fallback (same frame, same age). Residual: a V8 whose OC also
  rebooted serves MRAM with no age bound (no clock survived) — bench-only
  hardware, bounded post-restore by the MAX_FLIGHT timeout, documented.

## Adversarial review (2026-08-20, 38-agent workflow)

Four lenses + two-skeptic verification: 17 findings, **14 confirmed** (4
critical), 3 refuted. All fixed. The two that mattered most were defects in
this change, both in the same direction — a stale INFLIGHT frame reaching the
FC and re-arming pyro on a grounded rocket:

1. **[critical] The #383 stale-replay guard silently dropped the FC's LANDED
   clear.** The clear carries `flight_elapsed_ms` = FC uptime (a few seconds,
   since `launch_time_millis` is 0 at boot), which lands inside the guard's
   10 s backstep below a real flight's elapsed — so the store kept a genuine
   INFLIGHT frame that a later FC panic would restore on the ground. Fixed by
   exempting non-INFLIGHT frames from the guard entirely: a clear can only
   ever disarm, and elapsed-ms comparisons are meaningless across an FC
   reboot anyway. (Pre-existing for the V8 MRAM slot; this change would have
   extended it to V9/V10 and made it the primary serve path.)
2. **[critical] The #364 retry's `out_ready` re-probe could never succeed** —
   it passed the buffer length to `unpackMessage`, which requires the exact
   frame length. Fixed by deleting the re-probe: gating on `out_ready` *is*
   #364's complaint, and the snapshot ask is its own liveness test. That also
   retired a finding about the re-probe burning the OC's queued-command
   deliveries.
3. **[critical] A POWERON boot never marked a recovered flight evaluated**,
   leaving it for some later fault reset to seed from. Cold boots now retire
   it without seeding.
4. **[critical/major] The serve-TTL covered only the RAM cache**, not the
   V7/V8 MRAM fallback — and it measured cache age, not flight age. Now one
   `snapshotServable()` gate covers both stores and bounds *both* the frame's
   own `flight_elapsed_ms` and the time since we stored it, and requires the
   frame be INFLIGHT.
5. **[major] The once-marker was written at seed time**, before the FC ever
   read the frame — a second reset would have burned the flight's only
   chance. Now written on *consumption* (or on decline).
6. **[major] A read error mid-scan returned a stale frame as success.** Now
   fails: past an unreadable page the stream's real last word — possibly a
   LANDED clear — is unknowable.
7. **[minor→real] Pure last-wins re-seeded exactly the replayed frames #383
   blocks live** (the OC logs every frame *before* the guard runs, so a DMA
   replay can sit after newer frames). The scan now reports both the stream's
   last frame (does the flight end here?) and the highest-`flight_elapsed_ms`
   frame (what to restore from).
8. **[minor] Cache read/write race** between the I2S parser task and the I2C
   serve path — now under a `portMUX`; **NVS-unavailable now fails closed**
   (no seed rather than an untrackable repeat).

Refuted (kept as-is): the retry window vs the OC's pre-I2C-slave boot, the
no-reply GET leaving the slave ring empty, and the I2C_TX_RESYNC grace-window
timing.
