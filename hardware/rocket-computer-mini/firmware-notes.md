# rocket-computer-mini — firmware changes for the supercap / UVLO / arm-protection rework

**Status: hardware is drawn and wired into the sheets (2026-08-26) — ARM_CLK,
CAP_ACTIVE and the supervisor chain all connect; layout not started. Nothing below
is implemented in firmware.** This is the handoff list: what firmware must change, what it should
change, and what it can now stop doing.

The hardware this responds to:

- Pyro fires **from the pack** (`VBAT_CON` tap); the old 2200 µF store and its
  150 Ω charge resistor are gone from the firing path.
- The +3V3 rail carries a **2 F supercap** (C130, CHP5R5L205R-TWQ) behind an ideal
  diode (U40) and a 33 Ω charge resistor (R120), isolated from the buck by U44.
- A **TPS3840x29 supervisor (U45) cuts the rail at 2.86–2.94 V** through load
  switch U42, so a dying rail never drags the MCUs and flash through the
  undefined region — the cut is a clean edge, and restart on buck return is
  unconditional (release 2.93–3.07 V vs buck-min 3.234 V).
- The pyro **ARM gate is charge-pump driven** (ARM_CLK) with a **hardware
  one-shot ceiling** (2.3–4.3 s) that crowbars the gate independent of firmware.

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

## 2. Pin assignments — WIRED 2026-08-26 (board headers still needed)

| Signal | Pin | Notes |
|---|---|---|
| `ARM_CLK` | **FC GPIO8** | LEDC output. GPIO8 was `PYRO_ARM`, which the one-shot rework eliminated — the console UART is fully restored (GPIO43/44 both free again). |
| `CAP_ACTIVE` | **FC GPIO3** + **OC GPIO34** | wired to both MCUs. GPIO3 is the JTAG-select strap: R127's 100 k pull-up to +3V3 gives it a *defined* high at boot (better than floating). If the LM66100 ST polarity check shows it idles LOW, revisit this pin. |
| `V_SCAP_ADC` | **OC GPIO8** (ADC1) — wired | the FC has no ADC-capable pin left, so the pre-arm supercap check lives on the OC. Arm commands already flow through the OC, so the gate sits in the right place; the OC refuses/forwards accordingly. |

FC free pins after the rework: GPIO43/44 (console — keep them) and nothing
else. The rework *returned* a pin: `PYRO_ARM` no longer exists in hardware.

Extend `board_*.h` with all three, then run the netlist parity sweep
(`kicad-cli` GPIO→net table vs the header — see the board-header parity
procedure); nothing in CI catches a mismatch.

## 3. Pyro arming — REQUIRED changes to every ARM path

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

## 4. `CAP_ACTIVE` handler — REQUIRED

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

`V_SCAP_ADC` is V_SCAP ÷ 2 (100 k / 100 k, R125/R126): full scale ~1.65 V at
the pin (~1.73 V full scale with DEF high) — configure ADC attenuation accordingly and calibrate the 2:1 ratio.

- **Pre-arm check:** refuse pyro arm (and fail the preflight-checklist step)
  until V_SCAP ≥ ~3.1 V. Rationale: after a pad power-cycle the cap is empty
  and recharges through 33 Ω at τ = 66 s — **~5.5 min to full**. Without the
  gate, a quick cycle-and-launch flies with no hold-up and nothing would say so.
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

## 7. Open verifications (hardware side, but they set firmware limits)

| Item | Sets |
|---|---|
| LM66100 ST polarity | the `CAP_ACTIVE` edge direction (§4) |
| TPS3840**x29** released variant + stock | whether the 2.86–2.94 V numbers stand |
| E220 + LC86G minimum supply at 2.83 V | whether the floor argument closes for every load |
| ESR_DC of C130 (~380 mΩ inferred, not published) | the §4 time budgets |
