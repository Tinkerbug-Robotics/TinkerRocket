# Servo bench notes

Pad-time servo behaviour and the hardware-vs-software triage procedure, for
bench-testing the fin-tab servos before a flight day. Companion to the
in-app **Servo Test** and **Fin Layout** views. See issues #345 / #406 / #407.

## What the firmware does on the pad

- **Boot.** The servos wiggle one at a time (`0→1→2→3`, ~350 ms each) so only
  one inrush spike hits the shared rail at a time, then settle at the
  *trimmed* neutral (0° through the fin calibration, **not** the raw pulse
  midpoint) and hold it for ~6 s before the pad relax kicks in. So on boot you
  should see each tab sweep, then all four sit straight for a few seconds.
- **Pad trim previews live.** Applying a servo config / trim from the app on
  the pad drives the tabs to the freshly-trimmed neutral and holds it ~6 s
  (the `servo_pad_wake_until_ms` wake window) instead of relaxing immediately —
  so you can see where the trim actually lands.
- **Anti-backlash settle (#407).** Every pad-time "go to neutral" (boot settle,
  trim preview, servo-test stop) approaches neutral from a *consistent* side:
  the tab is driven a few degrees past neutral, then settled back over a couple
  of loop ticks. This loads the gear mesh the same way each time, so the rest
  position is repeatable even on a slightly worn servo. It masks **mild**
  backlash only — it is **not** a substitute for replacing a servo with gross
  hysteresis (see below).
- **Pad relax.** Outside those wake windows the servos are relaxed (0 % duty,
  no pulse train) to cut ~150 mA/servo of holding current; they re-energise
  automatically at launch detect.

## `[SERVO TEST]` serial line — hardware-vs-software triage

Each servo-test command prints the commanded angles and the per-channel pulse
widths actually written to the LEDC hardware:

```
[SERVO TEST] Angles: [0.0, 0.0, 0.0, 0.0] deg -> pulses [1500, 1500, 1500, 1500] us
```

The `-> pulses [...] us` are the **software ground truth** (angle → fin
calibration → +bias → clamp). Use them to split hardware faults from software:

- Pulses look right but the **tab doesn't move / lands off** → mechanical
  (linkage, gear backlash, worn pot, dead servo, no power on that pin).
- Pulses look **wrong** → software (fin cal, bias, servo min/max µs, or the
  profile's range/cal pair). Check the active profile's servo min/max µs vs its
  fin travel; the app defaults are 1000–2000 µs ↔ ±60°.

## Two-direction hysteresis test (backlash / servo-health check)

To decide whether a servo needs replacing (e.g. servo 3, ~10° hysteresis,
issue #407):

1. From **Servo Test**, command the servo to **+20°**, then back to **0°**.
   Note where the tab physically lands.
2. Command the same servo to **−20°**, then back to **0°**. Note the tab again.
3. The two `0°` positions **must land identically**. The `-> pulses` are the
   same both times (software commands the same neutral), so any *physical*
   divergence is mechanical slop — gear backlash or a worn pot.

A repeatable divergence of several degrees (≈10° on the failed servo 3) means
gear backlash / worn pot: no trim value or firmware change makes it repeatable,
and in flight the roll controller commands through the dead zone (limit-cycling
/ soft authority). **Replace the servo before flying it on a control fin** and
re-run this test on the replacement — it must pass (identical landing) before
that slot is trusted.
