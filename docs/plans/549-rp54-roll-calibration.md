# #549 — the sim's roll plant, fitted to a real flight

**Status:** done 2026-09-12. Scenario **(e)** in `tinkerrocket-sim/src/tinkerrocket_sim/simulation/scenarios.py`
(`roll_flight_20260829_config` on `build_rollypolly_54`) reproduces the 2026-08-29 Rolly Polly 54 mm
flight's roll ring-down and coast; `tests/test_scenarios.py::test_scenario_e_rp54_flight_20260829`
is the regression. Scenario (c), the 2026-05-17-*like* kick on the hand-tuned RollyPolly III
plant, is kept unchanged — it is a different vehicle and a different question.

## The flight

`TestFlights/2026_08_29 BARC L2/Rolly Polly - 54 mm/flight_20260829_164814.bin` — F52C
(the passive sim says 71.4 m/s / 243 m against 71.0 / 247 flown; E30T and F67C are not close),
apogee at T+7.7 s, the only flight of the day with roll control active on real fins. Roll control
engaged from the rail (delay 0 ms; the airspeed gate, #1175, did not exist yet), and the log
carries everything the fit needs: the gyro at 1.65 kHz, the EKF speed and the PID output
`roll_cmd` at 495 Hz. The binary is read directly (`binary-is-the-source-of-truth`).

What happened, in the traces: a ±20° bang-bang for the first 0.15 s (full gain at 7 m/s), then
from T+0.36 s the roll rate climbs to **+892 °/s at T+0.50 s while the tab is already pinned at
+20° opposing it** — a torque burst during the burn that full authority could not hold — after
which the pinned tab drives the rate to −600 °/s and the loop rings down through 2 s. In coast
the controller holds a standing **+1.3…+2.3°** of tab against a built-in roll trim, with the rate
inside ±10 °/s. The vehicle arced to 24° off vertical by burnout and kept ~26 m/s of horizontal
speed through coast.

## The tool

`tinkerrocket-sim/scripts/fit_roll_flight.py <flight.bin> --out DIR [--motor F52C] [--launch-angle 66]`

It produces the numbers in `RP54_FLIGHT_20260829` in four stages, in order of how well one
flight constrains each:

1. **The controller, identified.** The firmware law is known (`TR_ServoControl`: rate-null PID,
   `(V_ref/V)²` gain schedule capped at 3 with `V_ref` 50 / `V_min` 25, integral separation at
   40 °/s, ±20° clamp), and both its input (the rate, the speed) and its output (`roll_cmd`) are
   logged. Replaying the logged rate through the law and least-squares fitting Kp/Ki to the
   logged command gives **Kp = 0.1204, Ki = 0.0100, Kd = 0** at **0.33° RMS over 3813 ticks**
   (command RMS 4.96°). That is the profile's default (`pidKp 0.12 / pidKi 0.01`), recovered
   from the flight rather than assumed, and it also confirms the sign convention the sim mirrors
   (the firmware passes `−gyro_x`, so the error is `+gyro_x`).
2. **The misalignment.** An open-loop fit of `ṗ = A·V²·(δ + m) − B·V·p` on the coast window
   (2.5–7.7 s, driven by the logged command through the servo model) pins the built-in trim at
   **m = −2.0…−2.3°** — the number the standing tab command cancels. `A` and `B` trade off there
   (the coast is nearly steady state), so they are not taken from this stage.
3. **The kick.** The residual roll acceleration over 0.15–0.62 s with the fitted plant subtracted
   is a burst peaking at ~45,000 °/s² at T+0.48 s. The sim models a kick as an instantaneous
   `roll_kick_dps`, **sized to the flown peak and not fitted**: left free, the optimiser shrinks
   it to ~320 °/s to buy a better first swing, which is the wrong trade for a regression that
   has to exercise the kick as flown.
4. **Authority, damping, servo lag and delay — closed loop.** The ring-down is a closed-loop
   oscillation (the command is a function of the rate), so an open-loop fit of it is
   ill-conditioned: with `B` free the optimiser runs it to a bound and with `B` pinned the
   residual barely moves. So these four are fitted by running the real sim — same firmware
   controller, the gains from stage 1, the motor, the trim from stage 2, the kick — and
   minimising the window-normalised RMS rate error against the flight over everything after the
   kick (Nelder-Mead, 221 sims; the kick window is reported, not scored — an impulse cannot match
   a 0.15 s burst). The sim launches at **66°** so its coast airspeed, and with it the `V²`
   authority, matches the flight's arc; altitude is deliberately not matched (184 m vs 247 m).

Result (`RP54_FLIGHT_20260829`):

| parameter | value | note |
|---|---|---|
| `A` | −0.122 °/s² per ° of tab per (m/s)² | negative = the tab opposes the rate, the firmware's stabilising sign; `Kt_ref = A·I·V_ref²/n` (−5.5e-3 N·m/deg per tab at 95 m/s) |
| `misalign_deg` | −2.0 | stage 2 |
| `B` = K/I | 0.687 1/m | damping time constant 0.03 s at 45 m/s |
| `servo_tau_s` | 28 ms | first-order horn lag |
| `servo_cmd_delay_s` | 86 ms | **new `SimConfig` field**: transport delay before the servo sees a command (the 56 Hz PWM frame + controller-to-pulse latency); no first-order servo reproduces the ring-down phase without it |
| `kick_dps` at `kick_time_s` | 892 °/s at T+0.45 s | the flown peak, by construction |
| `I_roll` | 8.3e-4 kg·m² | **not measured** — the definition default; it appears only in `Kt/I`, so a real number rescales `Kt_ref` and changes nothing the scenario reproduces |

Fit residual against the flight, rate RMS per window:

| window | error | flight | verdict |
|---|---|---|---|
| 0.15–0.62 s (the kick) | 331 °/s | 362 °/s | exercised, not matched — an impulse standing in for a 0.15 s burst; not scored |
| 0.62–1.20 s (first swing) | 69 | 104 | the sim under-swings (flown −600 °/s) |
| 1.20–2.50 s (ring-down) | 7.5 | 20.1 | matched |
| 2.50–4.50 s (coast) | 3.5 | 8.7 | matched |
| 4.50–7.70 s (late coast) | 1.1 | 2.8 | matched |

Sensor seeds 1–4 move every window error by < 0.1 °/s: the dynamics are deterministic, so the
**tolerance is the fit residual, not the seed spread**.

## The regression

`test_scenario_e_rp54_flight_20260829` brackets each window at flown ± residual, rounded, with a
lower edge as well as an upper one (a scenario that stopped exercising the loop must fail, not pass
by being quiet): kick peak 700–1100 °/s with the tab pinning; first swing RMS 40–130; ring-down
10–30; coast 4–13; late coast < 5; standing trim 1–3° of tab; no saturation after T+1.0 s.
Scenario (e)'s own numbers: 896 / 81 / 16.8 / 7.2 / 2.7 °/s and 1.6° of trim.

## What it does and does not constrain

Constrained: the controller (near-exactly), the roll trim, the closed-loop ring-down and coast
behaviour of the (authority, damping, lag, delay) set. **Not** constrained: those four
individually (they trade off — the coast wants ~40 % less authority at 1–2° than the ring-down
does at 20°, which is the tab's `Kt(δ)` nonlinearity the sim can carry but this flight cannot
pin); the kick's true shape; `I_roll` on its own; anything in pitch/yaw. A second flight with a
different kick, or a bench measurement of the servo's step response, is what separates them.

## Traps met

- The forward-simulation fit of the ring-down is degenerate: huge authority with huge damping is
  a quasi-static solution that fits as well as the physical one. Fit the closed loop instead.
- A first-order servo lag alone cannot reproduce the ring-down phase; the loop needs a pure
  delay — hence `servo_cmd_delay_s`. Without it, the identified gains on a physical authority
  make the sim's loop limit-cycle at 200–400 °/s where the flight settled.
- The sim's coast airspeed is a launch-angle question: straight up in still air it decays to
  13 m/s by apogee where the flight kept 25–30 m/s, a 5× difference in `V²` authority.
