# Flight Computer

The Flight Computer (FC) is the part of the rocket that decides things. It reads the
sensors, estimates where the vehicle is and how it is pointed, runs the control law
that drives the fin servos, decides when the rocket has launched and when it has
reached apogee, and fires the pyro channels. Everything it produces — telemetry,
events, log frames — is handed to the [Out Computer](out-computer.md) to store and
transmit.

> **New here?** Read [Overview](README.md) first for how the three boards fit
> together, and the [Out Computer](out-computer.md) page for the other half of the
> rocket-side story.

---

## Why it is its own processor

The FC exists so that nothing on the flight path ever has to wait. It runs no radio,
touches no flash, and serves no file transfers — all of that lives on the Out Computer
behind a link the FC can ignore. What is left is a loop that reads sensors, updates a
filter, and writes servo commands, at a rate that must not vary.

That constraint shows up everywhere in this file. It is why the sensor drivers get
their own core, why the I2C link to the Out Computer is *polled by the FC* rather than
interrupt-driven, and why that poll is skipped entirely in flight.

## At a glance

| | |
|---|---|
| **Chip** | ESP32-P4, dual core |
| **Entry point** | [`app_main`](../../tinkerrocket-idf/projects/flight_computer/main/main.cpp) → `setup_fc()`, then a `flight` task spinning `loop_fc()` |
| **Source** | one file, [`projects/flight_computer/main/main.cpp`](../../tinkerrocket-idf/projects/flight_computer/main/main.cpp) (~7,350 lines) |
| **Navigation** | [section map](generated/flight-computer-map.md) — 28 sections, 12 of them inside `loop_fc` |
| **Flight loop** | 1000 Hz |
| **Estimator** | 15-state error-state EKF at 500 Hz (loop rate ÷ `EKF_DECIMATION`) |
| **Sensors** | IMU 1920 Hz, barometer 500 Hz, magnetometer 200 Hz, GNSS 18 Hz |
| **Outputs** | 1–4 fin servos, 4 pyro channels, camera control, piezo, status LED |
| **Talks to the OC** | I2S out (telemetry, master TX) + I2C out (command poll, master) |

## Two cores, five tasks

This is the FC's defining structure, and it is the opposite arrangement from the Out
Computer. Sensor acquisition gets its own core so that nothing the flight loop does can
starve it.

```mermaid
flowchart LR
    subgraph C0["Core 0 — sensor core"]
        direction TB
        IMU["Poll IMU Data<br/>prio 4, 8 KB"]
        GNSS["Poll GNSS Data<br/>prio 3, 4 KB"]
        SEND["I2S Sender<br/>prio 2, 4 KB"]
    end
    subgraph C1["Core 1 — flight core"]
        direction TB
        FLIGHT["flight — loop_fc()<br/>prio MAX-1, 16 KB"]
        OTA["FC OTA RX<br/>prio 3, 4 KB"]
    end
    IMU -->|"queue"| FLIGHT
    GNSS -->|"queue"| FLIGHT
    FLIGHT -->|"queue"| SEND
```

The flight task runs at `configMAX_PRIORITIES - 1` — the highest priority in the
system — and is subscribed to the task watchdog. Its 16 KB stack is not arbitrary: the
EKF's `timeUpdate()` and `measUpdate()` allocate roughly 7.5 KB of temporary 15×15
matrices on the stack per call.

The queues between the poll tasks and the flight loop are what make this work. They are
also what a past bug ate through: when a blocking I2C poll stalled `loop_fc()`, samples
piled up and were silently dropped, with nothing in the telemetry to say so. The IMU
queue is now deep enough to ride out a stall, and an overflow raises `NSF2_FC_IMU_DROP`
in the telemetry flags so the drop is visible rather than inferred (#474).

## What one loop pass does

`loop_fc()` is half the file. It runs freely at whatever rate the hardware allows, but
the flight logic inside is gated to `FLIGHT_LOOP_UPDATE_RATE` (1000 Hz).

1. **Drain the sensors.** *All* pending IMU samples are pulled each pass — the chip runs
   at 1920 Hz, about two samples per pass — and every one is forwarded to the I2S log so
   the recorded rate follows the sensor's own output rate rather than the loop rate.
   Only the freshest sample feeds the EKF and control path.
2. **Publish magnetometer calibration status**, if a calibration is running.
3. **Compute pressure altitude.** The ground reference tracks continuously through the
   pre-flight states and then *freezes* at `PRELAUNCH` — see Gotchas.
4. **Update the EKF** (every other pass, so 500 Hz).
5. **Poll the Out Computer over I2C** for a pending command — skipped in flight.
6. **Dispatch that command** — roughly 60 handlers, the largest block in the file.
7. **Run kinematic checks**: launch, apogee, and landing detection with per-sensor health
   feeding an adaptive quorum.
8. **Run the state machine**, including pyro servicing and the control law.
9. **Pack and send telemetry** (`NonSensorData` at 500 Hz).
10. **Service sound, LED, and camera** timers, then periodic diagnostics.

## Flight states

```mermaid
stateDiagram-v2
    [*] --> INITIALIZATION
    INITIALIZATION --> READY: >1 s uptime<br/>IMU + baro alive
    READY --> PRELAUNCH: OC ready<br/>GNSS ≥4 sats, >3 s stable
    READY --> INFLIGHT: launch detected<br/><i>degraded — gates unmet</i>
    PRELAUNCH --> INFLIGHT: launch detected
    INFLIGHT --> LANDED: descent + settled
    LANDED --> [*]: terminal until reboot
    READY --> MAG_CALIBRATION: app command
    MAG_CALIBRATION --> READY: accept / abort
```

`READY → PRELAUNCH` is the gate that says the vehicle is genuinely ready to fly: the
Out Computer is answering, and GNSS has held a fix with at least four satellites for
three seconds. Entering `PRELAUNCH` freezes the barometric ground reference and runs a
pyro continuity check.

The `READY → INFLIGHT` edge is the honest one. If launch is detected without those gates
met — no GNSS lock, Out Computer not answering — the FC does not refuse to fly. It
promotes straight to `INFLIGHT` through the same entry path in a degraded mode: guidance
off, reference-position freeze skipped, ground pressure taken from whatever the pad gave
it. A rocket that has left the pad is in flight whether or not the software approves.

`LANDED` is terminal. It sets `post_flight_lockout`, which is re-asserted at the top of
the state machine on every pass, so no command and no re-triggered launch detect can
start a second flight without a reboot (#317).

`MAG_CALIBRATION` is a bench-only state entered by app command and refused in
`PRELAUNCH`/`INFLIGHT`/`LANDED`. While in it, kinematic checks are skipped, EKF init is
inhibited, and pyro servicing cannot run — the user is physically tumbling the rocket,
and every automatic path that could misread that has to be closed.

## Estimation

A 15-state error-state EKF fuses IMU, barometer, magnetometer, and GNSS. The IMU feed is
converted from the sensor library's FLU convention (X forward, Y left, Z up) to the
filter's FRD (X forward, Y right, Z down) on the way in.

Two gates are worth knowing:

- **GNSS acceptance** requires a 3D fix, a minimum satellite count, a horizontal-accuracy
  bound, and a genuinely new fix timestamp. Initialization applies tighter accuracy plus
  a low-velocity check, since the filter's init assumes a stationary pad.
- **Accelerometer attitude correction is disabled during powered flight and coast**
  (`use_ahrs_acc`), because specific force is nowhere near 1 g and gravity is not
  recoverable from it. It comes back on after apogee: blanket-disabling it through
  descent starves the filter of its gravity reference and freezes the velocity estimate.

Baro has its own hazard. Above roughly Mach 0.76 the static port reading is unusable, so
a lockout suppresses barometric apogee voting between 260 m/s (on) and 240 m/s (off).

## Control

Two mutually exclusive modes, chosen in the app.

**Roll control** nulls roll rate with a PID driving the fin tabs through the control
mixer. It starts at launch plus a configurable `roll_delay`, so the boost phase — where
fin authority is highest and the vehicle is least in need of help — is left alone.

**Guidance** adds a proportional-navigation law on top. It is gated on the *same*
`roll_delay`, which is the subtle part: guidance begins a fixed time after **launch**,
not after burnout. To keep guidance out of the boost phase, set `roll_delay` to
approximately the motor burn time.

Guidance additionally requires a healthy EKF, sufficient airspeed, and a tilt within
limits. The tilt check is a **latch** — once tripped, guidance is off for the remainder
of the flight and the vehicle reverts to roll-only. Before `roll_delay` elapses, and
whenever the EKF is unhealthy, the control path falls back to a pure gyro rate-null that
needs no state estimate at all.

## Pyro

Four independent channels, each with a configurable trigger mode and value, continuity
sensing, and an arm/fire pair. They are serviced only from the `INFLIGHT` state, and
`pyroSafeAll()` runs on landing.

The initialization sequence is deliberately hand-rolled — see Gotchas. This is the one
part of the FC that can do something irreversible to a person standing nearby.

## Talking to the Out Computer

Two links, opposite directions, different roles.

**I2S carries telemetry out.** The FC is master TX, streaming packed sensor frames
continuously. A sender task on core 0 owns the write so the flight loop never blocks on
it. During an OTA the link flips: the FC becomes slave RX and receives a firmware image
through the same pins.

**I2C carries commands in.** The FC is master and polls the Out Computer every 250 ms,
reading a combined `[status][optional config]` response of exactly 96 bytes. The protocol
is pipelined — read the previous query's response, then send the next query — so the Out
Computer gets a full poll interval to prepare each answer.

That poll is **skipped entirely during `INFLIGHT`**, with one exception: a simulated
flight keeps polling so a stop command can land mid-run. In a real flight, no app or
ground command reaches the FC at all.

---

## Gotchas

Things that have cost real bench time.

**Never use `gpio_reset_pin()` on a pyro output.** It briefly enables the internal
~50 kΩ pull-up as part of its disable configuration. The gate drivers are pre-biased
NPNs with a 2.2 kΩ base resistor and 47 kΩ base-emitter pull-down, so that pull-up biases
the base above V<sub>BE</sub> for the microseconds between reset and the subsequent
pull-up-disable — long enough to momentarily turn on the ARM and FIRE MOSFETs and twitch
the squib rail at boot. `safePyroOutputInit()` exists for exactly this: it pre-loads the
output register to 0, detaches any peripheral signal, selects plain GPIO on the IO MUX,
and only then enables drive, so the pad goes from high-Z straight to driving low.

**The barometric ground reference freezes at `PRELAUNCH`, and it has to.** If it kept
updating, the pressure ratio would stay near 1.0, computed altitude would stay near zero,
the filtered altitude rate would never cross the launch threshold, and launch detection
would deadlock. The reference tracks through `INITIALIZATION` and `READY` — typically ten
seconds or more — so it is well settled by then.

**Do not clear `out_pending_command` when you dispatch it.** The Out Computer repeats
each command across several polls for I2C reliability, and that field mirrors what the OC
is currently reporting. Clearing it at dispatch makes the reset fire between polls and
every repeat re-executes the command. The dedup key is `last_processed_cmd`, and it
resets only when the OC actually reports 0.

**Attitude drifts on the pad and that is not a bug.** Sitting vertical puts the vehicle
at an Euler-angle singularity, so roll and yaw trade off against each other freely while
the quaternion stays perfectly steady. Read the quaternion, not the Euler triple, when
judging pad attitude.

**A test mode active at launch is force-cleared.** Ground test, servo test, and servo
replay all live in an `else` chain ahead of the state machine, so a test left running
would suppress `PRELAUNCH → INFLIGHT` and pyro servicing for an entire flight — no
drogue, no main, ballistic return. Launch detection now clears any active test as a
failsafe (#363). A bench false positive merely drops you back into `READY`, which is safe.

**The EKF decimation gate must be computed before GNSS is consumed.** The filter runs
every other pass, and marking a fix consumed on an off tick loses it — the acceptance
gate goes false before any EKF tick sees the fix, and the else-branch then injects a
zeroed measurement with a never-processed timestamp, corrupting position, velocity, and
heading aiding (#367).

**`sdkconfig` is generated and untracked, and it overrides `sdkconfig.defaults`.** Same
trap as the other firmwares: editing the defaults file does nothing while a stale
`sdkconfig` sits beside it, and a symbol that no longer exists fails silently. Delete
`sdkconfig` and rebuild when a config change appears to have no effect.

---

## Where to look next

- [Section map](generated/flight-computer-map.md) — every region of `main.cpp` with line
  ranges and links, regenerated from the source banners
- [Out Computer](out-computer.md) — where this board's telemetry goes
- Components: `TR_Sensor_Collector`, `TR_GpsInsEKF`, `TR_KinematicChecks`, `TR_PID`,
  `TR_ControlMixer`, `TR_GuidancePN`, `TR_ServoControl_ledc_mult`, `TR_Orientation`
- Shared wire contract: [`RocketComputerTypes.h`](../../tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h)
- Host tests for the flight-critical math live in [`tests_cpp/`](../../tests_cpp/)
