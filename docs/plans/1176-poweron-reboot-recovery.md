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
| 1 | Pure policy headers + host tests, wired to nothing | not started |
| 2 | OC-side retirement hardening of the V7/V8 MRAM slot | not started |
| 3 | **FC boot hygiene — no recovery semantics** | **DONE** |
| 4 | Interlock + delayed apogee arm, on today's fault-reset path only | not started |
| 5 | The behaviour change: the flight token and the POWERON widening | blocked on §5 |
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

## 5. Decisions reserved to the owner

Step 5 is blocked on these. They are policy and field-workflow calls, not
engineering unknowns.

1. **Auto power-on.** With a live token, connecting the battery powers the board
   up by itself rather than waiting for the app's power button. This is a real
   change to the prep-table workflow and should probably have a distinct LED or
   audible signature.
2. **Where the token lives.** A dedicated NVS partition is robust against the
   shared namespace filling, but a partition-table change means existing boards
   need a full erase-and-reflash rather than an OTA.
3. **One flash write in flight.** The token is written at the launch edge. The
   alternative removes every in-flight write but loses V9/V10 coverage for the
   first half-second of flight — the window the incident actually fell in.
4. **Slow-main vehicles.** The low-and-slow arm was deleted. Confirm no vehicle
   descends under a main slower than ~5 m/s, and this costs nothing.

## 6. Bench gates before any of this flies

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
