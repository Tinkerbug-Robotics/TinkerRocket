# rocket-computer-mini — firmware changes for the supercap / UVLO / arm-protection rework

> # SUPERSEDED IN PART — read this before implementing anything below
>
> **Written 2026-08-26 against hardware that has since been reworked twice. Checked
> against the routed board on 2026-09-04. Three sections describe parts that are not
> on this board.** Nothing here is implemented in firmware, and some of it must never be.
>
> | § | State on the fabricated board |
> |---|---|
> | 1. Brownout detector | **Rationale gone; the setting is unreviewed.** The TPS3840 supervisor (`U45`) and load switch (`U42`) this section defends against were deleted. The rail now holds a flat 3.0 V from the `TPS61094` until the hold-up cap is spent, so the level must be re-derived against 3.0 V, not against a 2.86–2.94 V supervisor trip. |
> | 2. Pin assignments | **Corrected in place below** (2026-09-04). The table is the only part of this file that tracks the board. |
> | 3. Pyro arming | **RETRACTED IN FULL.** There is no charge pump, no `ARM_CLK`, and no hardware one-shot. Arming is now two static consent lines. See [`arm-watchdog-rework.md`](arm-watchdog-rework.md). |
> | 4. `CAP_ACTIVE` handler | **RETRACTED IN FULL.** No `U40`, no `R127`, no LM66100. The signal is `VBUCK_OK` and its polarity is inverted. See [`holdup-tps61094-rework.md`](holdup-tps61094-rework.md); the parking and shedding *order* in that section is still the right shape. |
> | 5. Supercap charge gate | **Intent stands, every number is wrong.** Corrected inline. |
> | 6. Behaviour that needs no code | **Stands, except** the "cut itself" and "re-enable bounce" bullets, which describe `U45`. |
> | 7. Open verifications | **RETRACTED.** Every row names a deleted part. |

The hardware as fabricated:

- Pyro fires **from the pack** (`VBAT_CON` tap); the old 2200 µF store and its
  150 Ω charge resistor are gone from the firing path. *(Still true.)*
- The +3V3 rail carries a **5 F supercap** (`C130`) charged and boosted by a single
  `TPS61094` (`U47`): 100 mA charge, 2.5 V termination, and a flat 3.0 V out while
  running on stored energy. The 2 F part, its ideal diode (`U40`), its 33 Ω charge
  resistor (`R120`) and the isolating switch (`U44`) are all gone.
- **There is no supervisor.** `U45` and `U42` were deleted with the rest of the chain.
- The pyro **ARM gate is driven by two static consent lines**, `FC_ARM` (flight
  computer GPIO44) and `OC_ARM_EN` (out computer GPIO11), both of which must be high.
  There is no charge pump and no hardware one-shot ceiling; the windowed watchdog that
  briefly replaced them was itself removed on 2026-09-02 in favour of a firmware
  heartbeat.

---

## 1. Brownout detector — REQUIRED, both MCUs

Set the ESP32-S3 brownout detector to the **lowest available level (≈2.44 V;
2.56 V also acceptable)** in both images (FC and OC):

```
menuconfig → Component config → ESP System Settings →
    Brownout Detector → voltage level → 2.44 V
```

(`CONFIG_ESP_BROWNOUT_DET_LVL_SEL_*` in `sdkconfig.defaults` — pin it there, not
in a local sdkconfig, so fresh builds inherit it.)

**Why:** the supervisor owns the shutdown. Its worst-case cut is 2.86 V at the
sense node (≥2.83 V at the loads). If the chip's own detector sits above that
(several selectable levels are), the MCU resets itself *while power is still
good* — a messy MCU-only reset with the NAND possibly mid-write, exactly the
event the supervisor exists to prevent. Below 2.83 V the chip BOD never fires,
because the rail is already dead. The supervisor must win the race; the chip
detector becomes a never-fires backstop.

Do this **before** flying the new power stage; it is one config line per image.

## 2. Pin assignments — WIRED 2026-08-26, **two rows superseded since**

> **`ARM_CLK` and `CAP_ACTIVE` no longer exist.** Two later reworks replaced
> them, and this table is corrected in place below rather than deleted so the
> old names remain findable:
> [`arm-watchdog-rework.md`](arm-watchdog-rework.md) (2026-08-28) retired the
> LEDC arm clock for a supervised-MCU arm, and
> [`holdup-tps61094-rework.md`](holdup-tps61094-rework.md) (2026-08-29) replaced
> the LM66100 hold-up chain with a TPS61094. **Section 4 below still describes
> the LM66100 behaviour and its polarity is now inverted** — read the hold-up
> rework doc before implementing it. The netlist is the authority for all of
> this; the PCB file has not been re-synced and still carries the old names.

| Signal | Pin | Notes |
|---|---|---|
| ~~`ARM_CLK`~~ ~~→ `WDT_PET`~~ → **gone** | **FC GPIO8 is now `ISM6HG256_INT1`** (pad 13) | Both the LEDC arm clock and the windowed-watchdog pet line that replaced it are gone; the watchdog was removed 2026-09-02 for a firmware heartbeat. **Do not drive FC GPIO8** — it is the IMU interrupt input. Arm intent is `FC_ARM` on **FC GPIO44** (pad 50), the out computer's consent is `OC_ARM_EN` on **OC GPIO11** (pad 16), and both must be high to arm. **The flight computer has no hardware console left**: GPIO43 became `IND_1` on 2026-09-04, so console is USB or the GPIO matrix. The out computer keeps both UART0 pads. |
| ~~`CAP_ACTIVE`~~ → `VBUCK_OK` | **FC GPIO3 only** (pad 8, `ADC1_CH2`) | The out computer was taken off this net on 2026-09-03 — its GPIO34 has no ADC. The TPS61094 has no status pin, so the flight computer watches a `V_BUCK` divider, **`R137` 100 k / `R138` 360 k** (`R138` was drawn as 1 M; #1000 lowered it to land the node in ADC range), with **`C152` 100 nF** across the tap so the 78 kΩ source can drive the SAR sample-and-hold. **Read it as an ADC, not as a logic level**: 2.71 V with the buck up, 2.39 V on cap energy, a 325 mV window. Polarity is **inverted** against the old `CAP_ACTIVE`: high = buck present, low = riding the cap. There is no `R127` and no LM66100 on this board. The divider still gives the strapping pad a defined level at reset, which is why GPIO3 is an acceptable home for it. |
| `V_SCAP_ADC` | **OC GPIO8** (ADC1) — wired | the FC has no ADC-capable pin left, so the pre-arm supercap check lives on the OC. Arm commands already flow through the OC, so the gate sits in the right place; the OC refuses/forwards accordingly. |

FC free pins after the rework: **none**. GPIO43 and GPIO44 both went to
`IND_1` and `FC_ARM`, and the 2026-09-03 sensor-SPI swap spent the rest; the two
spare pads left on the flight computer are GPIO47 and GPIO48, which are bare. The
`ESP_SDA` / `ESP_SCL` link pair moved to **FC GPIO36 / GPIO37** in that swap —
GPIO33 and GPIO35, which older tables give for it, are `PYRO4_FIRE` and
`PYRO2_FIRE` now.

Extend `board_*.h` with all three, then run the netlist parity sweep
(`kicad-cli` GPIO→net table vs the header — see the board-header parity
procedure); nothing in CI catches a mismatch.

## 3. Pyro arming — ~~REQUIRED changes to every ARM path~~ RETRACTED

> **Do not implement this section.** There is no charge pump, no `ARM_CLK` net and no
> hardware one-shot on this board. **FC GPIO8, which the section tells you to clock, is
> the IMU interrupt input** — configuring LEDC on it breaks the IMU and arms nothing.
> The real path is two static consent lines, `FC_ARM` (FC GPIO44) and `OC_ARM_EN`
> (OC GPIO11), both high to arm, with the out computer holding the veto; see
> [`arm-watchdog-rework.md`](arm-watchdog-rework.md). Kept below only so the old
> names stay searchable.


**`PYRO_ARM` no longer exists.** Arming is one operation: **run a 50% duty
square wave on `ARM_CLK` (FC GPIO8)** — LEDC, ~20–200 kHz, 50 kHz suggested
(the pump's 100 nF is a heavy GPIO load; slower keeps the swing full). The
pump doubles off V_MCU_SWTCH to ~6 V of gate drive. Gate is up in ~1 ms; the
existing 10 ms settle covers it. **To disarm: stop the clock** — the gate
bleeds through R22 in ~15 ms.

Every path that arms needs this, not just the flight path:

- the in-flight fire state machine (`pyroSetArm` / ArmSettle→Firing),
- `PYRO_FIRE_TEST` (ground test-fire),
- `PYRO_CONT_TEST` (continuity arms momentarily).

**Hardware ceiling (rework 2, senses the gate itself):** continuous armed time
longer than **1.5 s (worst-case; ~2.4 s typ, 3.3 s max)** trips the one-shot.
There is no firmware hole left: the timer integrates `ARM_GATE`, so *any* path
that charges the gate is timed. On trip the crowbar **latches** — pyro stays
disarmed until the **FC domain power-cycles** (OC drops `POWER_SWITCH`; that is
the recovery path, and the OC should do it if it ever sees a ceiling trip
reported). Fire windows must finish inside 1.5 s; today's 10 ms + 200–500 ms
is fine.

**Spacing rule: leave ≥ 0.5 s between ARM cycles.** The timer auto-resets from
the falling gate through D14+R130 (τ ≈ 0.2 s); back-to-back CONT→FIRE cycles
with no gap accumulate timer charge and can false-trip the ceiling.

**Deliberately not a firmware concern:** an LEDC channel keeps clocking through
a CPU hang. That hole is what the one-shot covers. Do not add a watchdog that
tries to duplicate it, and do not move the clock to bit-banging for "safety" —
the hardware ceiling is the safety.

Note the fire-duration knob is now real: firing comes from the pack, so
`PYRO_FIRE_DURATION_MS` (200) actually changes delivered energy. No change
required for e-matches; revisit only if hotter igniters return to scope.

## 4. ~~`CAP_ACTIVE` handler — REQUIRED~~ RETRACTED (the order of operations still applies)

> **`U40`, `R127` and the LM66100 do not exist on this board.** The signal is
> `VBUCK_OK`, it is an ADC reading rather than a logic level, and its sense is
> inverted — see the §2 row. The numbered park-and-shed sequence below is still the
> right shape for a hold-up event; the polarity discussion, the first-boot blip and
> the time budgets are not. Hold-up on the fabricated board is ~21 s at 190 mA and
> ~13.6 s at 300 mA, from 5 F terminating at 2.5 V into a flat 3.0 V rail.


U40's status pin, pulled up by R127: it changes state when the rail transfers
from the buck to the supercap — i.e. **VBATT/buck is gone and the computer is
living on stored energy**.

**Polarity (verified 2026-08-26, LM66100 ds + review):** with U40's CE tied to
VOUT, ST is pulled **LOW while the buck powers the rail** (chip disabled,
blocking reverse) and goes **Hi-Z — pulled HIGH by R127 — on transfer to the
cap**. So: idle low, assert high. Expect **one false HIGH blip at first boot**
while an empty supercap charges past ~0.25 V — debounce or ignore it below a
V_SCAP threshold. (This only works because CE is on VOUT; CE at GND would leave
ST permanently Hi-Z.)

On assert, in order:

1. **Park the log.** Finish the in-flight NAND page, write a snapshot, stop
   opening new writes. This is the whole reason the warning exists — the
   supervisor cut is clean for the *hardware*; only firmware can make it clean
   for the *data*.
2. **Shed LoRa TX** (drop to RX/idle; FC forwards the event to the OC). No
   longer safety-critical — the CT delay on U45 defuses the re-enable bounce —
   but it roughly doubles the time budget.
3. **Log the event** with timestamp and V_SCAP; raise the advisory (quiet
   status line, per the GUI convention — never recolor the state banner).
4. **After the log is parked: shed the FC domain.** FC confirms the park over
   the mini-link; the OC then drops `POWER_SWITCH` (U30 EN). This matters for
   shutdown cleanliness, not just runtime: at the trip point the load-release
   rebound through the cap's ~0.46 Ω can exceed the supervisor's ~100 mV
   hysteresis, and a full dual-MCU load can bounce the rail several times at
   the very end. With the FC already off, the residual OC-only load rebounds
   less than the hysteresis and the final cut is a single clean edge.

**Time budget from assert to cut** (from 3.3 V to the 2.86–2.94 V trip):
~3.3 s at 190 mA cruise / ~1.7 s during TX, typical; **2.1 / 1.0 s worst-case**.
Step 1 must fit comfortably inside one second.

If it deasserts (buck came back — brief upstream transient): log it, resume
normal operation, re-enable TX.

## 5. Supercap charge gate — REQUIRED for arming, recommended elsewhere

`V_SCAP_ADC` is V_SCAP ÷ 2 (100 k / 100 k, `R125`/`R126`) on **out-computer
GPIO8** (pad 13), with `C144` 100 nF at the pin: full scale ~1.25 V at the pin for a
2.5 V cap — configure ADC attenuation accordingly and calibrate the 2:1 ratio.

- **Pre-arm check:** refuse pyro arm (and fail the preflight-checklist step)
  until the hold-up cap is charged. **The 3.1 V threshold this line used to give is
  unreachable** — the `TPS61094` terminates charging at **2.5 V**, so a 3.1 V gate
  refuses arming forever. Gate at roughly **2.3 V** instead, and re-derive it from
  the hold-up seconds actually wanted. Rationale is unchanged: after a pad
  power-cycle the cap is empty, and at the 100 mA charge current 5 F takes about
  **two minutes** to reach 2.5 V (not the 5.5 min through 33 Ω this line used to
  claim — `R120` is gone). Without the gate, a quick cycle-and-launch flies with no
  hold-up and nothing would say so.
- **Telemetry:** report V_SCAP (or a charged/charging flag) in sensor_health so
  the app can show "backup charging, n%". Adding a logged field means a log
  format version bump — batch it with the next format change rather than
  spending one on this.

## 6. Behaviour that needs no code — but must be tested

- **Firing-sag ride-through (ACCEPTED design behavior).** A hot igniter can sag
  VBAT_CON below the eFuse's 6.4 V cutoff mid-fire; the computer rides the
  supercap and reboots nothing. **The mandatory bench test is the full fire
  with USB UNPLUGGED** — the TPS2121 silently switches to USB power and masks
  the entire event on a bench setup.
- **The cut itself.** Below the trip, the rail drops dead in ~1 ms. There is no
  firmware role; the MCU simply stops. When the buck returns, the board cold
  boots and the normal snapshot-recovery path runs. Bench-test exactly this
  sequence: pull VBATT mid-log → confirm the parked log and snapshot survive →
  restore VBATT → confirm clean boot and recovery.
- **Re-enable bounce.** With hysteresis (75–125 mV) possibly under the ESR bump
  (89–141 mV), U45 may attempt re-enables every ~6 ms after a cut; each aborts
  in microseconds and the rail stays effectively dead. The MCU never sees these.
  Don't chase "ghost 100 mV blips" on a dead rail as a bug.
- **GNSS through transients.** The hold-up keeps the LC86G powered through any
  survivable sag, so no cold-start handling is needed — that's the point.

## 7. ~~Open verifications (hardware side, but they set firmware limits)~~ RETRACTED

> Every row below names a part that was deleted in the 2026-08-29 hold-up rework.
> The live open items live on the tracker and in
> [`holdup-tps61094-rework.md`](holdup-tps61094-rework.md).


| Item | Sets |
|---|---|
| LM66100 ST polarity | the `CAP_ACTIVE` edge direction (§4) |
| TPS3840**x29** released variant + stock | whether the 2.86–2.94 V numbers stand |
| E220 + LC86G minimum supply at 2.83 V | whether the floor argument closes for every load |
| ESR_DC of C130 (~380 mΩ inferred, not published) | the §4 time budgets |
