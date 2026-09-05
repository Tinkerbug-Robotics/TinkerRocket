# #1176 — In-flight reboot recovery across a POWER-ON reset

**Problem:** on 2026-08-29 the V8 rocket computer lost power for an instant at
T+0.451 s, rebooted, and then did nothing at all for the remaining ~75 s of the
flight — no telemetry, no logging, no deployment. The vehicle was saved by a
second computer in the nosecone. Every layer of the shipped recovery machinery
(#104, #364, #846, #825, #848) refused to act, and each refused *by design*.

This document records what was verified about that failure, the design that
follows from it, and the staged plan. It is the standing justification for the
work; read it before changing anything in this area.

---

## 1. What actually happened, verified

All five claims below were tested by independent reviewers instructed to refute
them. File:line anchors are against `e2334aa`-era `main`.

The single sentence version: **every gate in the recovery path asks "was this
reset a fault?", and a power interruption destroys the evidence needed to answer
that question.**

| # | Layer | Behaviour on a power-loss boot | Verdict |
|---|---|---|---|
| 1 | OC rail restore, `rail_restore_policy.h:48` | `if (reset_is_poweron) return false;` — first gate, short-circuits before retained state is consulted. `PWR_PIN` driven LOW. | CONFIRMED |
| 2 | FC snapshot ask, `flight_computer/main.cpp:4026-4030` | POWERON is absent from the five-value fault whitelist, so `GET_FLIGHT_SNAPSHOT` is never sent. | CONFIRMED |
| 3 | FC clear, `flight_computer/main.cpp:4279-4281` | The same boolean drives an else-side action that **overwrites the recovery record** with a LANDED frame. | CONFIRMED |
| 4 | OC #846 re-seed, `out_computer/main.cpp:6838` | Retires the recovered flight permanently on a cold boot. V9/V10 only. | PARTIAL |
| 5 | Boot latency | ~5.6 s floor, 8–20 s typical, up to 60–75 s with a sick GNSS, **never** with a dead barometer. | PARTIAL |

### The decisive facts

**The rail never came back, so the FC never booted.** On V7/V8 the FC has no
hold pin of its own (`PWR_HOLD_PIN = -1`), so the OC's `PWR_PIN` is its only
power. With `boot_rail_restored` false the OC drives that pin LOW and sits in a
BLE-only idle: the radio, the flight log, the I2S link and the I2C slave all
live exclusively inside `initPeripherals()`, which never runs. That accounts for
the whole 75 s of silence, not merely for the failure to resume.

**The OC also destroys the evidence, unread.** `rail_rtc = {magic, 0, 0, 0}` at
`out_computer/main.cpp:7341-7344` erases the retained "the rail was ON" flag on
a POWERON boot before anything reads it.

**The data needed to fly the rest of the flight had survived.** The V8's last
snapshot — `magic 0xF1A75A7E`, version 4, `rocket_state = 3` (INFLIGHT),
`sim_flight = 0`, `flight_elapsed_ms = 401` — was written into non-volatile MRAM
50 ms before the cut. The MRAM serve path (`out_computer/main.cpp:3614-3623`) is
not keyed on reset reason at all and would have served it. Nothing ever asked.

**The failure is intermittent by dip depth.** A shallow sag gives
`ESP_RST_BROWNOUT`, and recovery works today. Only a dip deep enough to trip the
power-on reset lands in this hole — which is exactly why it does not reproduce
on a bench.

### Why the reset reason cannot carry the fix

On the ESP32-S3, `soc/esp32s3/include/soc/reset_reasons.h:38-40` defines
`RESET_REASON_CHIP_POWER_ON`, `RESET_REASON_CHIP_BROWN_OUT` and
`RESET_REASON_CHIP_SUPER_WDT` **all as `0x01`**. The silicon physically cannot
distinguish a chip-level brownout from a power-on. The ESP32-P4 additionally
emits `ESP_RST_CPU_LOCKUP`, `ESP_RST_PWR_GLITCH` and (for `SYS_PSDET`, a
supply glitch > 50 ns) `ESP_RST_UNKNOWN`, none of which appear in any whitelist
in this tree.

So the register is both unsound and incomplete. Widening the fault list to
include POWERON is *not* an option either: it would make every deliberate cold
start on the pad look like a flight in progress. `ESP_RST_SW` cannot be folded
in either — that is what the operator's own power-off produces, where booting
rail-LOW is load-bearing.

**The fix must rest on a power-loss-surviving record that a flight was in
progress, not on why the processor reset.** The mini already does exactly this
(`rocket_computer_mini/main/main.cpp:784-806`), keyed on an NVS flag, and it is
the one place in the tree that recovers correctly after a full power loss.

---

## 2. The architecture

Split the one question `esp_reset_reason()` is asked into two, with **opposite
failure directions**:

- **Liveness** — should this boot come back alive, restore state, log and
  transmit? Fail toward *yes*. A false positive costs a skipped self-test and a
  state that cannot deploy anything. Decided by a durable NVS **flight token**.
- **Arming** — may a recovery-deployment channel ever leave Idle? Fail toward
  *no*. A false positive is the one that hurts people. Decided by a live,
  this-boot **motion interlock**, on measurements no ground handling produces.

The two are independent, and conflating them is what makes this problem look
unsolvable: the safe answer for liveness is the dangerous answer for arming.

---

## 3. Staging, and why this order

The safety machinery ships and flies **before** the permissive widening. Shipped
in the other order, the firmware becomes more willing to restore a flight while
nothing yet constrains what a restored flight may do.

| Step | Content | State |
|---|---|---|
| 1 | `RecoveryArmGate.h` + 21 host tests, wired to nothing | **DONE** |
| 2 | OC-side retirement hardening of the V7/V8 MRAM slot | **DONE** |
| 3 | **FC boot hygiene — no recovery semantics** | **DONE** |
| 4 | Interlock + delayed apogee arm, on today's fault-reset path only | **DONE** |
| 5 | The behaviour change: the flight token and the POWERON widening | **DONE** |
| 6 | Escape hatch (clear the token over BLE) | not started |
| 7 | Mini parity | after 4 has flown |
| 8 | Follow-ups, filed not bundled | — |

### Step 3, delivered

Four fin-motion sites ran before the airborne question was ever asked. #1119
reported the 4.2 s wiggle; the other three were unreported, and one of them —
`servo_control.begin()`'s trailing `setPulse(0)` — was gated only on the pins
being mapped, not on servo control being enabled at all.

Fixed **structurally rather than with a predicate**: `begin()` commands nothing,
and the self-test moved to the first READY tick. Reaching READY is positive
proof the vehicle is not resuming a flight, because a restored flight enters
INFLIGHT directly from setup and never passes through READY. There is no reset
reason to test and therefore nothing to get wrong.

The sweep became a state machine serviced across ticks. It could not be
relocated as a blocking call: the flight task carries a 5 s panic watchdog, and
4.2 s of blocking there is 84% of the panic deadline.

Also bounded two unconditional boot hangs in `SensorCollector::begin()` — an
unbounded barometer retry and a `while (1)` on a configuration failure. Either
one stopped the FC booting at all, on **every** reset reason.

---

### Step 4, delivered

`RecoveryArmGate` is wired into the existing fault-reset recovery. Two barriers
now stand between a stored record and an energised channel: the Idle ->
ArmSettle branch (the only place a channel can leave Idle) additionally requires
the interlock to have Opened on live evidence from this boot's own sensors, and
the restored apogee is withheld from both trigger conditions until Open has
persisted. `pyro_apogee_detected` itself is restored normally so the snapshot
keeps round-tripping.

A refuted restore is driven to LANDED, not back to READY, so the post-flight
lockout, the power-hold release discipline and the snapshot clear all apply
unchanged — and the app-side test-fire path stays refused throughout without new
code, because `isCommandLockoutState` already covers INFLIGHT and LANDED sets
`post_flight_lockout`. That closes both energising paths.

**A normal launch is bit-identical.** `recovery_gate_active` is set only in the
restore block, so both new conjuncts collapse to their old form on any flight
that was not restored.

**The capability this costs on the fault-reset path, stated plainly.** With no
sensorless backstop (decision 1) and no altitude arm (decision 5), a restored
flight whose barometer is dead, which is under a chute at 1 g, not spinning, and
whose GNSS has not yet acquired, has no arm available and will not deploy.
Previously a time-after-apogee channel would have fired on the stored apogee
with no live evidence at all. The exposure is bounded by GNSS: a cold fix takes
~30 s against a ~40 s drogue descent from 600 m, and the GNSS arm opens on 5 m/s
of vertical speed of either sign. This is the direct consequence of two
deliberate rulings and is recorded here so it is not rediscovered as a surprise.

### Steps 2 and 5, delivered

**Step 2** moves retirement of the V7/V8 MRAM snapshot slot to the OC, on any
boot that is not a recovery context. It had to land first: the only thing that
previously cleared that slot for a flight which never reached LANDED was the
FC's setup-time clear — the very clear step 5 loosens.

**Step 5** is the behaviour change. A durable NVS token in its own `flighttok`
partition records that a flight was in progress and never cleanly ended, and
tier 2 of the boot rail decision consults it whenever tier 1 declined. Tier 1
is untouched and stays first, because it wins the ~0.8 s decay race on fault
resets and must not grow a millisecond.

Details worth knowing before changing any of it:

- **Entry to tier 2 is `!boot_rail_restored`, not "the reset was a power-on".**
  A glitch reset can lose the RTC magic tier 1 depends on while reporting
  something else entirely, so keying on POWERON would exclude the resets that
  need tier 2 most.
- **The two crash-loop bounds compose.** `tier1_stood_down` and
  `deliberate_off` are captured *before* the existing blocks consume them, so
  tier 2 cannot silently re-arm a tier-1 stand-down.
- **Fail closed in both directions.** The restore-attempt counter is the only
  bound on repeated restores and it is a *write*, so it is written and read back
  before the rail goes up; a failure refuses rather than restoring unbounded.
- **The INFLIGHT write edge is PRELAUNCH → INFLIGHT**, not the launch flag's
  rising edge, so a restored flight — which enters INFLIGHT with no PRELAUNCH
  behind it — can never re-stamp its own token and refill its own budget.
- **The token carries a snapshot frame**, because on V9/V10 there are no NAND
  pages for roughly the first half second of a flight. Its `flight_elapsed_ms`
  is ~0, which is correct by construction: this source is reached only when
  neither the RAM cache nor the tail scan produced anything, and a flight old
  enough to have been running would have pages. The residual, if the tail scan
  fails for an unrelated reason on an old flight, is a restarted flight clock —
  bounded, and failing toward a late deployment rather than a spurious one,
  because the frame carries no apogee and the interlock still gates arming.
- **The partition moves nothing.** `flighttok` is appended at `0x620000`, so the
  existing `nvs` keeps its offset and size and the stored calibration, LoRa
  configuration and identity all survive the table change. Boards still need a
  USB reflash rather than an over-the-air update, because the partition table
  itself changes.

The FC also stops gating its ask on the reset reason: the OC's answer is the
authority, which is sound only because step 2 made the OC retire its stores on
every non-recovery boot. Its clear predicate moves from "the reset looked
normal" to "the OC positively said there is no flight" — strictly tighter, and
it preserves #834 item 3 because no answer now means no clear on every reset
reason.

## 4. What the adversarial pass killed

Four lenses attacked the first full design. Recording the dead ends so they are
not re-proposed:

- **A "locked until proven airborne" deployment interlock with no fallback** is
  not the safe default it looks like. On a vehicle whose barometer died or whose
  static port blocked, locked-forever *is* the incident's own outcome. Any
  interlock needs a bounded, baro-independent path to arming.
- **Refutation on a bespoke at-rest signature** contradicts the stuck-port
  invariant `MainDeployGate` was built around: a frozen port emits
  positive-looking evidence (fresh, in band, zero rate). Use the shipped
  `kinematics.quiescent_flag` instead of writing a second detector.
- **A low-and-slow "travel since boot" arm** is satisfiable by carrying a
  recovered rocket downhill. Deleted.
- **Asking for the snapshot from `loop_fc`** violates the documented I2C
  wire-alignment invariant (`TR_I2C_Interface.cpp:283-296`) and desyncs the
  slave TX ring permanently. All asking stays in `setup_fc`, where the bus is
  idle by construction.
- **A variable-length negative reply** breaks the fixed-length master read. Any
  "no snapshot" reply must be padded to exactly `kSnapFrameLen`.
- **Binding the token to a flight id at the launch edge** cannot work:
  `flightlogBeginFlight()` returns before an id exists.
- **On V9/V10 there are no NAND pages to tail-scan for roughly the first half
  second of flight**, so a reboot at the incident's own T+0.451 s has nothing to
  recover from even with a perfect token. The token must carry a snapshot frame
  itself.

---

## 5. Decisions — SETTLED 2026-09-05

All eight are the owner's rulings. Implement to these; do not re-litigate them
without saying so.

1. **Sensorless backstop — DROPPED.** A recovery boot that comes back with both
   the barometer and the IMU dead never arms. Nothing arms without live sensor
   evidence, on any path. The cost is accepted: a genuine flight that loses both
   sensors across the reboot makes no deployment. This removes the design's only
   nonzero ground-fire probability.
2. **The one in-flight flash write — ACCEPTED.** The token plus its snapshot
   frame is written at the launch edge on the Core-0 flush task. This is what
   keeps V9/V10 covered for the first ~0.5 s of flight, which is precisely when
   the V8 was cut.
3. **Token store — DEDICATED PARTITION, NOW.** Owner's direction: *"There is no
   issue with losing data on existing boards, don't design around it through at
   least V10 and mini V1."* So no NVS-preservation work. (It happens to be free
   anyway: `out_computer/partitions.csv` ends at `0x620000` and the base flash
   size is 8 MB, so a `flighttok` partition appended there moves nothing and
   fits on both the 8 MB V7/V8 and the 16 MB V9/V10 boards. Note the documented
   trap — IDF does not re-apply `sdkconfig.defaults` to an existing sdkconfig
   and this project keeps one per build dir, so delete or reconfigure every
   `build_<rev>` when the table changes.)
4. **Battery-connect auto power-on — ACCEPTED, with a distinct indication.** A
   token-driven boot gets an LED pattern or tone unlike a normal one, so it is
   unmistakable at the prep table that a board came up believing a flight is in
   progress.
5. **Slow/low never arms.** Owner's ruling: *"There is no need to arm if the
   rocket is under a main chute. If we are slow or low don't come back and arm
   any charges."* The low-and-slow arm stays deleted, and this is now a stated
   principle rather than a threshold trade. Under a main at ~5 m/s nothing arms;
   under a drogue at ~15 m/s it does, so the main charge still fires — the case
   that actually matters.
6. **Stale-token self-heal — TWO POWER CYCLES.** Refutation drives to LANDED and
   reuses that one-shot, which already handles the power hold, the post-flight
   lockout and the snapshot clear. ~30 s of proven stillness, using the shipped
   quiescence detector rather than a second at-rest test.
7. **`kMaxPoweronRestores` = 5.**
8. **Recovery-unsettled signal — OUT-COMPUTER LOCAL GATE.** No wire-format
   change. Forced by fact as much as preference: every bit of both telemetry
   flag bytes is already allocated (`NSF_*` bits 0-7 and `NSF2_*` bits 0-7 are
   all taken), so a verdict on the wire would need a struct change with parity
   ripple through both apps, the base station and every parser.

### Why the arming conditions use rate and acceleration, never altitude

Raised by the owner and worth recording, because it constrains the whole
interlock: after a reboot there is nothing to integrate and GNSS is ~30 s away,
so how is "slow or low" measurable at all?

Velocity is never integrated. Two signals are available within milliseconds and
both are instantaneous:

- **Barometric rate**, from differentiating successive pressure samples.
  Absolute AGL needs a ground reference and is badly wrong if a stale token
  supplies a foreign one; *rate* merely scales with that reference, so a few
  percent of reference error costs a few percent of rate error.
- **IMU specific force and angular rate**, read directly. Under thrust,
  free-fall, spin and sitting-still are all separable with no baseline.

GNSS cannot carry this and is a corroborating input only. Absolute altitude is
the one quantity a reboot genuinely cannot establish — which is why the review
had already deleted the only arm that used it, independently of the owner's
ruling above. The two conclusions agree.

## 6. Gate zero: measured, 2026-09-05

`Data_Analysis/replay_recovery_arm_gate.py` drives the real `RecoveryArmGate`
together with the shipped `TR_KinematicChecks`, both compiled out of the
firmware tree by `_recovery_arm_gate_shim.cpp` — the same no-drift pattern
`replay_deployment_detector.py` uses. Run across **27 flights** (the whole
`TestFlights` corpus with a launch and a usable IMU stream).

Two fidelity choices make the numbers mean something, and both make the result
worse rather than flattering it:

- **The altitude filter is rebuilt cold at the simulated reboot.** A recovery
  boot has no filter history. Replaying against the log's own warm
  `baro_alt_rate` would never exercise the settle window.
- **GNSS is suppressed for 30 s after the reboot.** `V_BCKP` is on the switched
  rail, so a recovery boot cold-starts the receiver. Before this was modelled
  the replay showed GNSS carrying almost every flight — an input the vehicle
  would not have had.

| Scenario | Sensors | Opened | Gate open (min / med / max) | Carried by |
|---|---|---|---|---|
| Reboot at T+0.5 s (the incident) | healthy | **27 / 27** | 1.00 / 3.15 / 10.96 s | free-fall 20, spin 3, descent 2, boost 2 |
| Reboot at apogee + 3 s (under drogue) | healthy | **17 / 17** | 1.60 / 1.60 / 1.65 s | descent 17 |
| Reboot at T+0.5 s | baro dead | 26 / 27 | 1.00 / 3.15 / 31.0 s | free-fall 20, spin 3, boost 2, gnss 1 |
| Reboot at apogee + 3 s | baro dead | **10 / 17** | 2.94 / 31.0 / 31.0 s | gnss 5, spin 5 |

The restored apogee is released exactly 1.00 s after the gate opens in every
case, as designed.

### What the replay found

**The under-drogue case is tight and reliable.** With a healthy barometer, a
reboot mid-descent opens the gate in 1.60 s and releases the apogee at 2.60 s,
on every flight, with no spread worth reporting. That is the case the main
charge depends on, and it is the answer to "does this cost us a deployment".

**The boost arm barely works, and that is a real finding.** Every flight is
under thrust at T+0.5 s, yet the acceleration arm carried only 2 of 27. The
30 m/s² bar is crossed constantly during boost but not *continuously*: the
recorded vibration is ±300 m/s² about the thrust level, so the unbroken 1 s
hold keeps resetting. Twenty flights fell through to free-fall instead, i.e.
they armed at burnout + 2 s rather than during boost. If boost-phase arming is
wanted, the hold has to become a leaky accumulator rather than an unbroken run
— the pattern `MainDeployGate`'s stuck-port check already uses.

**The dead-barometer cost is now measured, not reasoned.** With no barometer,
a mid-descent reboot fails to deploy on **7 of 17 flights**, and the ones that
succeed wait ~31 s for GNSS. The failures are all short flights (14-22 s total)
that land before the receiver acquires. This is the direct, quantified
consequence of dropping the sensorless backstop (decision 1) and refusing to
arm on altitude (decision 5). It was accepted deliberately; the number is
recorded here so the trade is visible rather than implied.

**The spin arm earns its place.** It carried 3 flights with healthy sensors and
5 of the 10 successful dead-barometer descents — exactly the case the header
argued it exists for.

### What gate zero could NOT test

The logs contain almost no pre-launch data (1-3 s), because a log session opens
at launch detect. So the corpus cannot answer "does the gate stay shut on a
rocket that is being handled", and the pad segments hoped for do not exist. The
owner has ruled out the hand-carry and taped-port captures (no local terrain,
no reliable way to block the port). **The remaining ground evidence is
therefore a stationary bench capture**, which is cheap: power a board, leave it
still for fifteen minutes, download. That would confirm the gate never opens
and that refutation fires at ~30 s. Until then the ground case rests on the
per-arm reasoning in `RecoveryArmGate.h`, not on data.

## 7. Bench gates before any of this flies

No threshold in this design has been measured; all are reasoned from source and
physics. In particular:

- Replay the 2026-08-29 logs through the interlock and confirm it opens on the
  real flight and stays shut on the pad segment.
- A pack-cut on a shake rig, and a V9 cut at T+0.3 s, to confirm the token
  survives and the restore lands.
- A deployment-channel witness test with a dummy load and a current probe,
  covering **both** energising paths — `servicePyroChannels` and the
  synchronous `PYRO_FIRE_TEST` — since the latter was missing from the first
  draft of the reachability argument.
