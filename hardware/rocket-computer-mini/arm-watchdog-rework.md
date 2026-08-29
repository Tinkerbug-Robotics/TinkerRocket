# Pyro ARM rework 3: supervised-MCU arm (replaces charge pump + one-shot)

**Status: DRAWN ON THE MINI 2026-08-28 (schematic + PCB deletions; the 5 new
footprints await placement). V10 not yet drawn.** Applies to rocket-computer-mini
and rocket-computer V10 identically (same refdes). Supersedes the charge-pump
ARM + one-shot ceiling drawn 2026-08-26.

**Mini drawing verification (2026-08-28):** netlist diff against pre-edit baseline
is exactly the spec — components 223→215 (−13 +5), net membership changes only the
intended ones (D4's V_MCU_SWTCH feed preserved); ERC 839→803 with only deletions'
noise and the two newly-used GPIOs resolved, no new violation classes; PCB: 13
footprints removed (the cluster had zero routed tracks), R22 revalued on both
sides, zones refilled with all 15 filled areas unchanged; `check_board_parity.py`
passes (mini exemption stands — its unplaced set is now exactly the 5 new parts);
bom.csv regenerated and reconciled 0/0/0 against the netlist (1 M and 74LVC1G17
lines dropped, TPS3813 line added). Remaining for layout: place U46, C139, R132,
R133, D16 (Update from Schematic imports them plus the net renames), route
WDT_PET / FC_ARM / OC_ARM_EN / FC_CHIP_PU / ARM_GATE — all signal-width. The
pre-existing J2 MPN stale-prop and U9 pin-9 parity notes on main are unrelated.

## Requirement

The pyro arm switch (the low-side GND↔PYRO_GND splice) must not conduct unless the
flight computer is powered, executing scheduled application code, and deliberately
commanding arm — and, independently, the out computer consents. No static pin state,
stuck peripheral, or single-MCU fault may arm it. Disarm must not depend on firmware
cooperation.

## Motivating defect (found 2026-08-28)

The 2026-08-26 pump design anchors D12 to V_MCU_SWTCH, creating a static DC path
rail → D12 → D13 → ARM_GATE that needs no clock: the gate parks at ~2.7–2.9 V
(rail − 2·Vf) whenever the FC rail is up. AON7534 Vgs(th) is 1.4–2.2 V, so the
"dead-man" property is defeated; the one-shot timer (referenced to ARM_GATE) then
phantom-trips ~10 s after every boot and after every normal disarm, latching Q12
into a 150–300 mW burn (SOT-416 rating: 150 mW) through an unresisted diode string.
The GND-anchored variant was already rejected in review (2.7 V, not a doubler);
the rail anchor fixes the amplitude but imports this DC back-path. Root cause class:
hand-rolled analog safety logic — second latent defect in this block (first: the
Ii×R asymptote caught 2026-08-26).

## Architecture

Four elements, each doing one job:

1. **FC intent** — a plain static FC GPIO (`FC_ARM`) is the *only source* of the
   arm-gate node. FC unpowered, in reset, or rebooting ⇒ pin low/high-Z ⇒ node low
   through the bleed. No pump: the FET is logic-level driven.
2. **FC liveness** — a windowed-watchdog supervisor. The FC must toggle WDI within
   a bounded window (too slow *and too fast* both trip). Its open-drain RESET
   wire-ANDs into the FC's existing CHIP_PU RC — a non-petting FC is **hardware
   rebooted**, which clears `FC_ARM` (GPIOs revert to input on reset). This is why
   RESET must drive CHIP_PU and not clamp the gate node directly: the TPS3813
   reset is a ~25 ms *pulse*, and a gate-node clamp would release between pulses
   while a wedged FC held its arm pin high. Resetting the FC makes the pulse
   sufficient — the reboot itself removes the gate source.
3. **OC consent** — an OC GPIO (`OC_ARM_EN`) vetoes via a Schottky clamp on the
   arm-gate node: OC drives it low to veto (default), high to consent. If the OC
   is dead/high-Z the veto is lost, but FC intent + FC liveness remain — never
   weaker than the pump design, which had no OC term at all.
4. **OC nuclear veto** (already exists, unchanged) — the OC can drop the FC domain
   (FC_EN_OC / POWER_SWITCH).

Fire still additionally requires the FC per-channel fire FETs, so a false fire
needs faults in **two independent MCUs**.

### Supervisor variant: L30, not K33 — do not "upgrade" this

TPS3813**L30** (VIT 2.58–2.70 V). The K33's threshold band (2.87–3.00 V max)
**overlaps the supercap hold-up floor** (TPS3840PL29 cuts +3V3 at 2.86–2.94 V):
a K33 could hold the FC in reset during a legitimate ride-through sitting at
2.9–3.0 V, spending up to the last ~100 mV (~0.3–0.6 s at mini load) of designed
hold-up with the FC dead. The L30 asserts only below 2.58–2.70 V, which the rail
never legitimately reaches (the supervisor cuts it first); the FC's own BOD (2.44 V)
remains the last resort. J25 (2.2–2.3 V) is an acceptable tertiary — its voltage
monitor sits below BOD and is inert, but the watchdog function is identical.

## Circuit

```
FC_ARM (FC GPIO, static) ──R132 5.11k──●── ARM_GATE ──R21 100R── U9 gate
                                       │                          │
                OC_ARM_EN (OC GPIO) ─K|D16 (veto: OC low = clamp) R22 100k → GND
                                       │
              (node has no other sources; WDT does NOT connect here)

WDT_PET (FC GPIO, toggled 250 ms) ── WDI ┐
                                  TPS3813L30DBV  VDD = V_MCU_SWTCH + C139 100n
        FC CHIP_PU RC (R110/C114) ── RESET┘ (open-drain, wire-AND)
WDI also ──R133 100k── GND    straps: WDT = VDD, WDR = GND
```

- **Window**: WDT=VDD, WDR=GND ⇒ upper limit 2–3 s, ratio 1:32 ⇒ lower boundary
  ~63–94 ms. Firmware pets every ~250 ms. Upper limit clears ESP32 boot-to-app
  (~0.5 s) after each reset release; pets closer than ~94 ms trip (catches
  runaway petting loops). External-cap programming exists if these need tuning.
- **WDT supply = V_MCU_SWTCH**: the supervisor monitors the FC's actual rail and
  sequences CHIP_PU at domain power-on (td ≈ 25 ms). With the FC domain off the
  WDT is dead — safe, because the gate source (`FC_ARM`) is dead too.
- **Armed levels**: node ≈ 3.15–3.3 V (5.11k/100k divider off the ~3.3–3.46 V
  rail); U9 gate ≈ node (µA through R21). Veto sink ≈ 0.63 mA into the OC pin.
  WDI pulldown 100k vs IIH ±25 nA — floats low with the FC domain off.
- **RESET into CHIP_PU**: verify R110 (sink through it ≪ IOL) and C114 (discharge
  transient through the open-drain FET) values at draw time; TI supervisors drive
  exactly this RC in their reference circuits.

### Pin assignments

| Board | FC_ARM | WDT_PET (WDI) | OC_ARM_EN | Notes |
|---|---|---|---|---|
| mini | FC **GPIO44** (U0RXD — console becomes TX-only; needs sign-off) | FC **GPIO8** (ex-ARM_CLK) | OC **GPIO11** (pin 16; GPIO9–12 all free — 9/10 kept as ADC1 spares) | Console log stream (GPIO43 TX) is untouched; bench input moves to USB-Serial-JTAG if wired |
| V10 | **TBD** — P4 has only USB (DM/DP) and MIPI pads unconnected; needs a P4 pin-mux check or a traded pin | P4 **GPIO16** (pad 17, ex-ARM_CLK) | S3 **GPIO14** (pad 19; 17/18 also free) | V10 rework parts are unplaced, so no layout cost |

Boot-safety criterion for `FC_ARM`: the pin must not drive high at any point in ROM
boot (no strapping pull-up). GPIO44 boots as UART RX (input) — safe; scope-verify
on first article.

## Fault matrix

| Condition | Result |
|---|---|
| FC unpowered / in reset / crashed, pins reverted | Node has no source → disarmed |
| FC wedged, `FC_ARM` left high, pets stop | WDT reboots FC within ≤3 s → pins revert → disarmed; stays disarmed through reboot (pin low until app re-asserts) |
| FC wedged but a peripheral (LEDC-class) keeps running | Irrelevant — no peripheral can pet WDI or hold `FC_ARM` through a reset |
| Runaway loop petting continuously | Pets violate the ~94 ms lower boundary → trip → reboot |
| App crash before first pet after reboot | Reset loop at ~2.5 s period, disarmed throughout |
| OC dead / consent pin high-Z | OC veto lost; FC intent + liveness remain (≥ pump design, which had no OC term) |
| OC stuck consenting + FC healthy | Arm follows FC alone — two-MCU property lost for that fault only; fire still needs FC channel command |
| FC rail droops in ride-through (≥2.86 V) | No WDT action (L30 trips <2.7 V); FC flies on |
| WDT die fails stuck-released | Arm = FC static + OC static (still two-MCU, liveness proof lost) |
| WDT die fails stuck-asserted | FC held in reset — mission loss, equivalent to FC hardware death; accepted (supervisor FIT ≪ the failure classes it removes) |
| In-flight genuine wedge | FC hard-rebooted; snapshot v4 recovery restores flight state; disarmed until app re-arms |

Residual (all designs share it): a wedge *inside* a still-correctly-petting app.
Bounded by the window discipline: pet only from the scheduled flight task, never
from an ISR-only or peripheral path.

## Component delta (per board)

**Delete (13):** C131, C132, C133, D12, D13, D14, D15, Q12, R121, R129, R130,
R131, U41 — the entire ARM_PUMP / ARM_TMR / ARM_CROW network and both on-sheet
notes ("Dead-man arm", "Hardware limit on fire duration").

**Add (5):** U46 TPS3813L30DBVR (SOT-23-6), C139 100 nF (stocked line), R132
5.11 k (the line R22 vacates), R133 100 k (stocked line), D16 BAT54XV2 (stocked
line).

**Change:** R22 5.11 k → 100 k (same 0402 land). Keep R21, U9, R110/C114.

**BOM lines:** +1 (TPS3813); −1 both boards (74LVC1G17 was single-use); mini
additionally −1 (1 M line was R121/R131-only; V10 keeps 1 M via R44/R77). C133's
22 µF D-cell note in the workbook ("one-shot ±10%→±20% fast-corner") becomes
obsolete — drop it at the next workbook pass. Net placements: −8.

**Arm FET gate drive** is now ~3.15–3.3 V. V10's CSD16323Q3 likely carries a
2.5 V RDS(on) spec (verify datasheet). The mini's AON7534 is spec'd only at
4.5/10 V — either bench-prove the 10 A fire pulse at Vgs 3.0 V cold, or swap to
a 2.5 V-spec'd 30 V DFN3x3 (sourcing pass TBD).

## Firmware delta

- Delete LEDC ARM_CLK entirely. GPIO8 (mini) / P4 GPIO16 (V10) becomes a plain
  toggle pin, petted every ~250 ms **from the scheduled flight task** (petting
  must survive fire busy-waits and flash operations — pet from an IRAM-safe timer
  during long flash erases, or schedule around them; the window cannot be paused,
  by design).
- `FC_ARM` raised/dropped around arm windows (replaces pump start/stop). The
  "≥0.5 s between ARM cycles" rule dies with the one-shot; arm/disarm cycles are
  now free.
- No latch: after a WDT reboot, re-arming works as soon as the app is up and the
  OC still consents. If latch-until-power-cycle semantics are wanted, the OC
  enforces them in policy (drop consent on observing an FC reboot; its FC_EN_OC
  power veto is the hardware backstop).
- OC: `OC_ARM_EN` low at boot, raised only on explicit arm command, dropped on
  disarm/anomaly/FC-heartbeat loss. **No OC light sleep while consenting** — the
  pin floats high-Z in light sleep (V9 GPIO12 lesson) and the veto is lost.
- Mini console becomes TX-only (GPIO44 → FC_ARM); bench interactive input moves
  to USB-Serial-JTAG.

## Bench / first-article items

1. Scope `FC_ARM` (GPIO44) through a full ROM boot — must never drive high.
2. FC held in reset: WDI floats → verify RESET pulses and node stays <0.3 V.
3. Wedge simulation (pet task suspended, `FC_ARM` forced high): confirm reboot
   within window and node drop; measure armed-overhang duration (~µs, the
   CHIP_PU fall).
4. Runaway simulation (pet at 10 ms): confirm trip.
5. OC veto: consent low with `FC_ARM` high — node ≤0.3 V, sink ~0.6 mA.
6. Ride-through: supercap droop to ~2.9 V — confirm NO reset (L30 margin).
7. Fire pulse at Vgs 3.0 V, cold, on the fitted arm FET (or complete the FET swap).
8. Verify R110/C114 against RESET IOL/transient.

## Sourcing (checked 2026-08-28; live pages only, per the PL29 lesson)

- **TPS3813L30DBVR (preferred):** live stock UNVERIFIED — LCSC search API and
  distributor pages bot-walled today; only stale aggregator ladders visible
  (~$1.0–1.3@100). Verify on the distributor's own page at buy time.
- **TPS3813K33DBVR (fallback, verified):** LCSC **C93238**, 997 in stock,
  $1.0986@100 (LCSC API, 2026-08-28). Electrically identical except VIT; cost of
  fallback: forfeits up to ~100 mV / ~0.3–0.6 s of worst-case ride-through
  (FC held in reset below VIT 2.87–3.00 V). Usable if L30 is dry; do not prefer.
- **TPS3813J25DBVR (tertiary):** VIT below FC BOD — watchdog intact, voltage
  monitor inert. Stock unchecked.
- Fleet need: 2/system (mini + V10) → ~200 for a 100-build.

## Proposed sheet note (replaces both deleted notes — do not apply without owner OK)

> Supervised arm (rework 3): ARM_GATE's only source is FC_ARM through R132;
> the FC's reset state is the disarm path. U46 window-watchdog (WDT=VDD,
> WDR=GND: pet 94 ms–2 s) hard-resets the FC via CHIP_PU if scheduled petting
> stops — a reset clears FC_ARM by construction, so no clamp on this node is
> needed or fitted. D16: OC veto (low = disarm; high-Z loses only the OC term).
> L30 threshold variant is mandatory: K33's VIT overlaps the 2.86–2.94 V
> hold-up floor and can hold the FC in reset during a legitimate ride-through.
> Pet only from scheduled task code — never from an ISR-only or peripheral path.
