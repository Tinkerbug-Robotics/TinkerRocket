#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// --- Board revision (#411, tracking #408) ---
// Pins + part-presence flags live in the per-board headers; everything in
// this file is board-independent policy and must not fork per revision.
//
// EXACTLY ONE of these must be set — there is no default. A wrong pyro pin
// map is completely silent at runtime (the board boots, continuity reads
// correctly, and the wrong channel fires or no channel fires at all), so the
// flight computer refuses to build rather than guess:
//
//   V7 legacy board:  idf.py -B build_v7 -DTR_BOARD_V7=1 build
//   V8 bench boards:  idf.py -B build_v8 -DTR_BOARD_V8=1 build
//   V9/V10 boards:    idf.py -B build_v9 -DTR_BOARD_V9=1 build
//
// main/CMakeLists.txt enforces the same rule at configure time with a clearer
// message; this check is the backstop for anything that includes config.h
// without going through it.
#ifndef TR_BOARD_V7
#define TR_BOARD_V7 0
#endif
#ifndef TR_BOARD_V8
#define TR_BOARD_V8 0
#endif
#ifndef TR_BOARD_V9
#define TR_BOARD_V9 0
#endif
#if (TR_BOARD_V7 + TR_BOARD_V8 + TR_BOARD_V9) != 1
#error "Set exactly one board revision: -DTR_BOARD_V7=1, -DTR_BOARD_V8=1 or -DTR_BOARD_V9=1. There is no default — the V8 and V9 pyro maps differ (ARM 5 vs 16, FIRE 2/3 swapped) and a wrong map fires the wrong channel silently."
#endif
#if TR_BOARD_V9
#include "board/board_v9.h"
#define TR_BOARD_REV_STR "V9/V10"
#elif TR_BOARD_V8
#include "board/board_v8.h"
#define TR_BOARD_REV_STR "V8"
#else
#include "board/board_v7.h"
#define TR_BOARD_REV_STR "V7"
#endif

struct config : board_pins
{

    // ### SPI parameters (pins in board header) ###
    static constexpr uint32_t SPI_SPEED = 10'000'000; // 10 MHz

    // ### IIS2MDC I2C parameters (pins in board header) ###
    static constexpr uint32_t IIS2MDC_I2C_FREQ_HZ = 400'000;
    static constexpr uint8_t IIS2MDC_I2C_ADDR = 0x1E;

    // ### Data Update Rates (Hz) ###
    static constexpr uint16_t FLIGHT_LOOP_UPDATE_RATE = 1000;
    // Requested > 10 Hz so the M10 runs at its 4-constellation ceiling
    // (~10 Hz).  The actual rate depends on satellite count; asking for
    // 18 Hz ensures the receiver doesn't self-limit below 10 Hz and the
    // poll task checks frequently enough (every 28 ms) to catch every fix.
    static constexpr uint16_t GNSS_UPDATE_RATE = 18;
    static constexpr uint16_t BMP585_UPDATE_RATE = 500;
    // MMC5983MA hardware supports 1/10/20/50/100/200/1000 Hz only.
    // 200 Hz is the highest step that fits within I2C budget.
    static constexpr uint16_t MMC5983MA_UPDATE_RATE = 200;
    // Collector-construction IMU rate.  No longer the effective default: the
    // shipped default setting is IMU_RATE_DYNAMIC (3840 Hz to deployment, then
    // 960 Hz), which setup() stages over this value before begin().  This
    // constant is what the collector is built with and the fallback the NVS
    // whitelist logs against.  The user can also select a FIXED 960/1920/3840
    // Hz from the app (BLE cmd 67); the FC persists the choice in NVS
    // ("imu"/"rate") and stages it into the collector before begin().  The
    // link budget below is unchanged — dynamic only ever runs at the 3840 and
    // 960 endpoints already covered here.  The poll task's
    // handoff queue + the loop's drain-all consumption log EVERY sample
    // regardless of loop rate (~980/s); control/EKF/guidance still consume
    // the freshest sample at loop rate.  Link budget on the 44100 Hz I2S
    // (176.4 KB/s): ISM6 framed = rate x 30 B, so 1920 -> ~58 KB/s and
    // 3840 -> ~115 KB/s; with all other streams total inflow is ~99 KB/s
    // at 1920 and ~156 KB/s (88%) at 3840 — no link bump needed, verified
    // against the post-#467/#469 OC budgets (parser ~10x headroom, MRAM
    // staging ~8% duty, NAND flush ~50% duty at 3840).  Bench gauges for
    // any new rate: [GAP DIAG] imu_q_drops, FC I2S enqueue drops, OC
    // rx_ovf/ring_peak, and the .bin per-type rates.
    static constexpr uint16_t ISM6HG256_UPDATE_RATE = 1920;
    static constexpr uint16_t NON_SENSOR_UPDATE_RATE = 500;
    // Guidance telemetry (GUIDANCE_TELEM_MSG) log rate while guidance is active.
    // Emitted from inside the NonSensor TX block, so it must divide
    // NON_SENSOR_UPDATE_RATE; 500 = lockstep with NonSensor/roll_cmd (frames
    // time-aligned 1:1). The guidance law recomputes once per flight-loop
    // iteration (~980 Hz, on the freshest IMU sample), so values are fresh at
    // any rate up to NON_SENSOR_UPDATE_RATE.
    static constexpr uint16_t GUIDANCE_TELEM_RATE_HZ = 500;
    static_assert(GUIDANCE_TELEM_RATE_HZ >= 1 &&
                  GUIDANCE_TELEM_RATE_HZ <= NON_SENSOR_UPDATE_RATE,
                  "GUIDANCE_TELEM_RATE_HZ must be in [1, NON_SENSOR_UPDATE_RATE]");
    // ISM6 full-scale configuration used by SensorConverter (and shared to OUT).
    static constexpr uint8_t ISM6_LOW_G_FS_G = 16;
    static constexpr uint16_t ISM6_HIGH_G_FS_G = 256;
    static constexpr uint16_t ISM6_GYRO_FS_DPS = 4000;

    // ### Sensor Frame Rotations ###
    // ISM6 frame -> board frame rotation (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float ISM6HG256_ROT_Z_DEG = -45.0f;
    // MMC5983MA -> board frame (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float MMC5983MA_ROT_Z_DEG = 180.0f;
    // IIS2MDC -> board frame (deg), CCW positive about +Z.
    // +90 deg from bench characterization (#204): with board +X pointed
    // true north the IIS2MDC reads Earth's horizontal field on its -Y axis
    // (and on -X when board +X points east) -> chip +Y is parallel to
    // board -X, a +90 deg sensor->board rotation.  Validated against two
    // static logs (board+X -> N and board+X -> E, 2026-06-17).
    static constexpr float IIS2MDC_ROT_Z_DEG = 90.0f;

    // Board -> rocket mounting orientation (TR_Orientation code, 0-23).
    // 0 = board +X toward the nose (the historical assumption).  Set to a
    // different code when the board is mounted off-axis: nose_sel*4 + clock,
    // nose_sel 0..5 = +X,-X,+Y,-Y,+Z,-Z, clock = quarter-turns about the
    // nose.  Applied after the per-chip rotations above, so those stay
    // PCB facts and this stays an airframe fact.  Pad-gravity auto-detect
    // and the app-side setting arrive in later phases; until then this
    // constant is the manual override.
    static constexpr uint8_t BOARD_TO_ROCKET_ORIENT = 0;

    // ### Camera Controls ###
    // Camera type is a RUNTIME property (runtime_camera_type, set over BLE and
    // persisted in NVS).  CAMERA_TYPE is only the first-boot default for a
    // board that has never been told otherwise.  Nothing gates camera
    // *behaviour* at compile time: the old USE_GOPRO flag made the whole GoPro
    // path unreachable no matter what the app selected, which is why "set it to
    // GoPro" silently did nothing for so long.
    static constexpr uint8_t CAMERA_TYPE = 2;  // 0=none, 1=GoPro, 2=RunCam
    // Still compile-time: whether to install the UART2 driver at boot for the
    // RunCam protocol.  Left true so a board can be switched to RunCam at
    // runtime; the pins stay parked high-Z until a RunCam start attaches them.
    static constexpr bool USE_RUNCAM = true;

    // ### GoPro control (pin in board header: CAM_SHUTTER_PIN) ###
    // A Hero 10 toggles video recording on a short pull-to-ground of the
    // shutter line.  Power is the CAM_ACT gate (RUNCAM_PWR_PIN), shared with
    // the RunCam path.  Two invariants, both safety-critical on V8:
    //
    //   1. NEVER assert the shutter while the gate is open.  V8 switches the
    //      camera's RETURN (Q3 low-side) and J6.2 is always-on raw VCC, so an
    //      unpowered camera floats toward VBATT.  Pulling J6.4 to board ground
    //      then does not "press a button" — it BRIDGES the open low-side
    //      switch (VCC -> camera internals -> that pad's clamp -> the P4 pad ->
    //      GND), limited only by the camera's impedance because V8 has no
    //      series resistor.  goproShutterAssert() interlocks on camera_gate_on,
    //      and the shutter is parked BEFORE the gate ever drops.
    //   2. NEVER resend the start press.  START_RECORDING (RunCam) is
    //      idempotent so that path resends it; a GoPro shutter press is a
    //      TOGGLE, so a "reliability" resend would stop the recording, and
    //      there is no feedback channel to notice.
    //
    // Release is open-drain-high (high-Z), never a driven high — an FC-side
    // drive or even a weak pull on a resistor-less V8 camera net is the same
    // phantom-feed that broke battery cold-start (see the RunCam note below).
    //
    // The timings below are STARTING POINTS, not measurements — no build in
    // this repo's history has ever executed a GoPro pulse.  Sweep them on the
    // bench and record what the Hero 10 actually wants.
    // Both waits are deliberately generous.  Nothing is racing them — the
    // camera is started on the pad, and the stop is already 30 s behind LANDED
    // (CAMERA_STOP_DELAY_MS) — so the cost of waiting is nil and the cost of
    // being short is a press the camera slept through, or a truncated MP4.
    static constexpr uint32_t GOPRO_GATE_SETTLE_MS = 300;    // gate up -> safe to claim the pad
    static constexpr uint32_t GOPRO_BOOT_MS        = 7000;   // gate up -> first press
    static constexpr uint16_t GOPRO_PULSE_MS       = 200;    // start press width
    static constexpr uint16_t GOPRO_STOP_PULSE_MS  = 200;    // stop press width (separate knob)
    static constexpr uint32_t GOPRO_FINALIZE_MS    = 10000;  // stop press -> gate off (file close)

    // Time to keep the camera rolling after LANDED before issuing the stop.
    // Captures post-impact footage and, critically, removes the inline
    // delay() that previously stalled the main loop on the LANDED edge
    // (which saturated i2s_tx_queue and dropped END_FLIGHT; see #141).
    static constexpr uint32_t CAMERA_STOP_DELAY_MS = 30000;  // 30 s

    // RunCam UART baud (pins in board header; wiring history in board_v7.h
    // and #234 — the bench-confirmed orientation is RX=31/TX=30/PWR=32).
    static constexpr uint32_t RUNCAM_BAUD = 115200;

    // RunCam record-start timing (serviceCameraStart poll loop, #251 follow-up).
    // The camera cold-boots on every power-on and only answers UART once ready.
    // A fixed wait + single probe raced the boot time and, on a miss, fell back
    // to a record-less power toggle (camera on, not recording — the intermittent
    // failure).  Instead we poll GET_DEVICE_INFO from BOOT_MIN_MS to BOOT_MAX_MS
    // and fire START_RECORDING the moment it answers (or blind at the deadline).
    static constexpr uint32_t RUNCAM_BOOT_MIN_MS      = 2000;  // earliest probe
    static constexpr uint32_t RUNCAM_BOOT_MAX_MS      = 8000;  // give up, record blind
    static constexpr uint32_t RUNCAM_PROBE_INTERVAL_MS = 250;  // poll cadence
    static constexpr uint32_t RUNCAM_PROBE_READ_MS    = 60;    // per-probe RX wait
    static constexpr uint8_t  RUNCAM_RECORD_RESENDS   = 2;     // extra START sends
    static constexpr uint32_t RUNCAM_RECORD_RESEND_MS = 150;   // resend spacing
    // Battery-only starts leave the camera dark (no LED, UART silent) even
    // though the same rail runs it fine once booted (its own button boots it,
    // recording included) — bench-observed 2026-08-16 on V8; USB/charger
    // attached masks it.  Charged-pack test eliminated voltage level and
    // source impedance.  Prime suspect: PHANTOM POWER.  V8 wires the UART
    // pins straight to J6.3/4 with no series resistance (R30/R32 are a V9
    // addition — netlist-verified against the archived V8 project), and V8
    // switches the camera's RETURN (Q3 low-side; J6.2 is always-on VCC), so
    // the FC's idle-high TX back-feeds the "off" camera through its ESD
    // diodes and its power-on-reset never sees a clean rise.  Fix: the UART
    // pins are PARKED high-impedance whenever the camera is unpowered and
    // attached only while the gate is up.  A silent probe window gets a
    // park+power-cycle retry (off long enough for the camera's own load to
    // drain it) before the blind fallback.  NEVER drive these pins low to
    // "discharge" the branch on V8 — with no series resistors that dumps the
    // camera's stored charge straight into the P4 pads.
    static constexpr uint8_t  RUNCAM_POWER_RETRIES    = 2;     // extra power-cycle attempts
    static constexpr uint32_t RUNCAM_RETRY_OFF_MS     = 3000;  // parked+unpowered between attempts

    // ### Pyro timing (pins in board header) ###
    // One shared arming FET feeds all four squib drivers. ARM is raised
    // only momentarily — for the PRELAUNCH continuity check and again
    // during a fire pulse — never latched in flight.
    // E-matches commit in <10 ms; 200 ms is ignition margin while bounding
    // the I2t a shorted match can dump into the channel FET on 2S.
    static constexpr uint32_t PYRO_FIRE_DURATION_MS    = 200;
    // Time between raising ARM and reading CONT / pulsing FIRE, giving
    // the upstream arming FET its turn-on settle margin.
    static constexpr uint32_t PYRO_ARM_SETTLE_MS       = 10;

    // ### Servo Controls (pins in board header) ###
    static constexpr int SERVO_BIAS_1 = 0;
    static constexpr int SERVO_BIAS_2 = 0;
    static constexpr int SERVO_BIAS_3 = 0;
    static constexpr int SERVO_BIAS_4 = 0;

    static constexpr int SERVO_HZ = 333;
    static constexpr int SERVO_MIN_US = 1000;
    static constexpr int SERVO_MAX_US = 2000;

    static constexpr float ROLL_RATE_SET_POINT = 0.0f;
    // #267 retune for the corrected fin-angle calibration.  The old command-kp
    // hid a fixed PHYSICAL roll authority: 6/14 flew 0.04 cmd-kp x3 scale, #170
    // flew 0.02 cmd-kp x6 scale -> both ~0.12 physical kp.  Now command ==
    // physical fin-deg (1:1), so KP=0.12 reproduces that flown authority, and it
    // sits in the SIL crisp-roll-null band.  KI dropped to a small seed: the
    // FLOWN ki was ~0, high ki winds up on transients, and the SIL roll plant
    // (Cl_p) isn't validated enough to pin it -> KI is a BENCH/FLIGHT-tune knob,
    // raise it if a steady roll persists.  (Prior #170 SIL tune: 0.02/0.03;
    // RP IV 2026-03-08: 0.04/0.001/0.0003.)  Runtime-overridable via NVS / app.
    static constexpr float KP = 0.12f;
    static constexpr float KI = 0.01f;
    static constexpr float KD = 0.0f;
    static constexpr float MIN_CMD = -20.0f;   // max commanded fin deflection (deg); app-tunable via PIDConfigData min/max
    static constexpr float MAX_CMD = 20.0f;
    // #267 servo fin-angle calibration: the physical fin deflection (deg) at
    // SERVO_MIN_US and SERVO_MAX_US.  The fin is bolted directly to the servo arm
    // (1:1), so the full 1000-2000us travel = +/-60 deg of fin.  Kept separate
    // from the command clamp above so a commanded fin angle maps to the pulse
    // that produces that *physical* deflection (was: ±cmd stretched over full
    // travel -> 6x over-deflection).
    static constexpr float FIN_MIN_DEG = -60.0f;
    static constexpr float FIN_MAX_DEG = 60.0f;
    // 1-pole LP filter cutoff on the PID D-term. The raw backward-difference
    // derivative amplifies sample-to-sample gyro noise (≈1 dps Δ / 2 ms =
    // 500 dps/s "rate") into visible servo flutter even when the rocket is
    // barely moving. Cutoff sits well above the 2.2 Hz crossover from the
    // flight tune so damping is preserved, but below the 100+ Hz noise
    // floor. 0 disables the filter (legacy behavior).
    static constexpr float D_FILTER_CUTOFF_HZ = 10.0f;

    // Integral-separation anti-windup threshold (deg/s). The roll-rate PID
    // integrator is frozen while |rate error| exceeds this, so a launch roll
    // transient can't wind it up — which, with the higher KI above, would
    // otherwise overshoot hard on an opposing-sign kick. Validated at 40 via
    // SIL (#170). Runtime-overridable via NVS "iwind" / the app. <=0 disables.
    static constexpr float INTEGRAL_SEP_THRESHOLD_DPS = 40.0f;

    static constexpr bool USE_SERVO_CONTROL = true;
    static constexpr bool SERVO_WIGGLE_ON_BOOT = true;

    // Relax the servos (stop the PWM pulse train) whenever we're sitting on the
    // pad in READY/PRELAUNCH and after LANDED.  The digital PTK 7308 servos hold
    // position stiffly and draw ~150 mA each while a valid pulse train is
    // present; cutting the pulses lets them go limp so the pad/post-flight idle
    // draw collapses to the servos' quiescent current.  PWM (and full holding
    // torque) resumes automatically the instant control commands them at launch.
    // NOTE: only saves current if the servo actually relaxes on signal loss —
    // some digital servos latch their last position regardless.  The future
    // servo-power MOSFET is the guaranteed fix; this is the interim lever.
    static constexpr bool SERVO_RELAX_ON_PAD = true;

    // #345 follow-up: pad-trim wake window (ms).  With SERVO_RELAX_ON_PAD the
    // servos are relaxed every tick on the pad, so a live trim/servo-config
    // change would be invisible — setBias only updates offsets, and the next
    // idle() strips the pulse train.  When a servo-config command lands on the
    // pad we drive the servos to the freshly-trimmed centre and hold them
    // energised for this long so the fin visibly moves to the new trim; each
    // further change re-arms the window, and it relaxes again once the operator
    // stops adjusting.  Only spent while actively trimming on the bench.
    static constexpr uint32_t SERVO_PAD_TRIM_WAKE_MS = 6000;

    // EKF pad heading: compass heading of board Z+ (deg, 0=North, 90=East)
    static constexpr float PAD_HEADING_DEG = 0.0f;

    // Magnetic declination fallback (deg, EAST-positive) used until a good GPS
    // fix lets TR_GeoMag (WMM2025) compute the true value from lat/lon/date.
    // 0 = the EKF heading tracks MAGNETIC north until the WMM value is set.
    static constexpr float MAGNETIC_DECLINATION_DEG = 0.0f;

    // GNSS quality gates — layered rejection of bad fixes
    // Note: h_acc is uint8_t (max 255). Many receivers report 255 = "unknown".
    // Set thresholds to 0 to disable h_acc gating when receiver doesn't populate it.
    static constexpr uint8_t  GNSS_MIN_SATS        = 4;      // minimum satellite count
    static constexpr float    GNSS_MAX_HACC_M       = 0.0f;  // max h_acc for EKF update (0 = disable)
    static constexpr float    GNSS_MAX_HACC_INIT_M  = 0.0f;  // max h_acc for EKF init  (0 = disable)
    static constexpr float    GNSS_MAX_VEL_INIT_MPS = 2.0f;  // max velocity magnitude for EKF init
    static constexpr uint8_t  GNSS_MIN_SATS_INIT    = 5;     // minimum sats for EKF init (stricter)

    // EKF scorecard health (#303): the filter counts as "converged" when its
    // worst-axis attitude and velocity covariance sit below these.  stabilizeP()
    // caps the diagonals at P_MAX_ATT=10 (~180° σ) and P_MAX_VEL=1e4 (~100 m/s σ),
    // so a diverged filter pins at the cap while isHealthy() (NaN-only) still reads
    // true — these bars catch that.  Tuned from a 34 s stationary bench run
    // (Data_Analysis/analyze_ekf_cov.py, replay through this same EKF): converged
    // worst-axis attitude settled to ~0.01–0.028 rad² (1σ ~6–9.5°) and velocity to
    // ~0.03–1.0 (m/s)² (1σ ~0.2–1 m/s).  Bars sit a few× above that — pass a
    // converged filter, flag one that hasn't converged or is pinned at the cap.
    static constexpr float    EKF_ATT_VAR_OK        = 0.1f;   // rad²,  worst attitude axis (~18° 1σ)
    static constexpr float    EKF_VEL_VAR_OK        = 4.0f;   // (m/s)², worst velocity axis (~2 m/s 1σ)

    // Barometer spike rejection (#450: rate-aware).  A sample is rejected only
    // if BOTH hold between it and the previous sample the EKF gate consumed:
    //   |delta|        > BARO_SPIKE_THRESH_M   (absolute deadband), AND
    //   |delta| / dt   > BARO_SPIKE_RATE_MPS   (apparent rate over the ACTUAL
    //                                           pair spacing from timestamps)
    // Real subsonic flight through any legitimate sampling gap stays far below
    // the rate limit (a 66 ms gap at 100 m/s is 6.6 m but only 100 m/s), while
    // glitches — an in-band SPI-corrupted sample or an ejection-charge pressure
    // transient at the ~2 ms ODR — show km/s-scale apparent rates.
    static constexpr float BARO_SPIKE_THRESH_M  = 5.0f;
    static constexpr float BARO_SPIKE_RATE_MPS  = 500.0f;

    // Barometer fusion throttle: minimum spacing between baro samples ACCEPTED
    // into the EKF measurement update (0 disables).  Fusing at the full
    // ~487 Hz ODR buys nothing — altitude dynamics are slow against the sample
    // rate — but each fusion pays the EKF's 15x15 covariance update.  When
    // #450 restored every-sample fusion, per-tick EKF cost tripled
    // (554 -> ~1650 us) and the flight loop fell ~960 -> ~715/s.  20 ms
    // (50 Hz) keeps altitude aiding far above its information rate while
    // making the covariance cost negligible.  Spike/freshness tracking in
    // BaroGatePolicy still inspects every sample; only fusion is rate-limited.
    static constexpr uint32_t BARO_FUSE_MIN_INTERVAL_US = 20000u;

    // Transonic barometer lockout: shock waves near Mach 1 cause large static
    // pressure errors (the "transonic jump").  Lock out baro updates when EKF
    // speed estimate exceeds LOCK_ON and keep them locked until speed drops
    // below LOCK_OFF (hysteresis prevents chatter at the boundary).
    // Set BARO_MACH_LOCKOUT_ON to 0 to disable.
    static constexpr float BARO_MACH_LOCKOUT_ON  = 260.0f;  // m/s (~Mach 0.76)
    static constexpr float BARO_MACH_LOCKOUT_OFF = 240.0f;  // m/s (~Mach 0.70)

    // EKF decimation: run EKF predict+update every Nth flight-loop iteration.
    // Navigation at 400–600 Hz is more than sufficient for rocket flight.
    // Setting this to 2 halves EKF compute load; the sensor read/log path
    // still runs every iteration so no IMU samples are lost.
    static constexpr uint8_t EKF_DECIMATION = 2;

    // Cascaded angle control: outer loop P-gain. Tuned to 2.0 via SIL (#170) —
    // 4.0 gave too narrow a deceleration band (rate_cap/kp_angle) so the
    // angle-hold rang; 2.0 returns to target cleanly within the controlled
    // window. Runtime-overridable via NVS "kpang" / the app.
    static constexpr float KP_ANGLE = 2.0f;
    static constexpr bool USE_ANGLE_CONTROL = false;  // default; overridden at runtime via app

    // Cascaded angle control: cap on outer-loop rate command (deg/s).
    // Prevents a large wrapped angle error from demanding rates the inner
    // loop / actuators cannot deliver, and keeps the inner-loop setpoint
    // sane during spin-recovery so the controller doesn't whipsaw direction.
    // Runtime-overridable via NVS key "rcap".
    static constexpr float KP_ANGLE_RATE_CAP_DPS = 60.0f;

    // Roll control activation delay after launch (ms).  Keeps fins neutral
    // for this many ms after launch detection before engaging any roll control.
    static constexpr uint16_t ROLL_CONTROL_DELAY_MS = 0;  // default; overridden at runtime via app

    // Gain scheduling — V_ref is the speed where base gains apply.
    // Gains scale as (V_ref/V)², capped at 3.0. At low speed, gains
    // increase to compensate for reduced aerodynamic effectiveness.
    static constexpr bool  GAIN_SCHEDULE_ENABLED = true;
    static constexpr float GAIN_SCHEDULE_V_REF = 50.0f;
    static constexpr float GAIN_SCHEDULE_V_MIN = 25.0f;

    // ### Guidance (Proportional Navigation — coast only) ###
    // Textbook 3D PN (Yanushevsky Ch.2, eq 1.11 / 2.23)
    // Master enable (default off; overridden at runtime via NVS "guid_en")
    static constexpr bool GUIDANCE_ENABLED = false;
    // PN navigation constant (dimensionless, 3-5 typical)
    static constexpr float PN_NAV_GAIN = 5.0f;
    // Max lateral acceleration command (m/s^2)
    static constexpr float PN_MAX_ACCEL_MPS2 = 20.0f;
    // Target altitude above pad (m) — set above expected apogee
    static constexpr float PN_TARGET_ALT_M = 600.0f;
    // Outer-loop angle P gains (pitch and yaw)
    static constexpr float PN_KP_PITCH_ANGLE = 4.0f;
    static constexpr float PN_KP_YAW_ANGLE = 4.0f;
    // Pitch/yaw inner-loop rate PID gains (start same as roll)
    static constexpr float PN_PITCH_KP = 0.04f;
    static constexpr float PN_PITCH_KI = 0.001f;
    static constexpr float PN_PITCH_KD = 0.0003f;
    static constexpr float PN_YAW_KP = 0.04f;
    static constexpr float PN_YAW_KI = 0.001f;
    static constexpr float PN_YAW_KD = 0.0003f;
    // Max individual fin deflection in guided mode (deg)
    static constexpr float PN_MAX_FIN_DEG = 15.0f;
    // Tilt-limit safety gate: if the off-vertical angle of the nose (from EKF
    // attitude) exceeds the phase limit, guidance is LATCHED off and the
    // vehicle falls back to roll-only for the rest of the flight. Guidance has
    // little authority once badly tilted (lateral accel ∝ q ∝ V², collapsing
    // in coast) and a heading-error could push the wrong way, so a hard tilt
    // ceiling is a safety floor. Boost limit is tighter (a vertical boost
    // should stay near-vertical); coast allows more before giving up.
    static constexpr float GUIDANCE_TILT_LIMIT_BOOST_DEG = 15.0f;
    static constexpr float GUIDANCE_TILT_LIMIT_COAST_DEG = 20.0f;
    // Fin→servo layout (cruciform mix shared by roll / ground-test / guidance).
    // FIN_AZIMUTH_n_DEG = ring-POSITION azimuth of servo n's fin (0 = top slot,
    // 90 = right, … looking from the rear — what the app's ring GUI shows).
    // The mixer maps position→tangential force (a fin's lift is perpendicular
    // to its radial arm): deflection_n = sign_n·(pitch·sin(az_n) −
    // yaw·cos(az_n)) + roll — so with the "+" defaults the right/left pair
    // carries pitch (elevators) and the top/bottom pair carries yaw (rudders).
    // Bit n of FIN_REVERSE_MASK negates servo n's pitch/yaw (tilt) response;
    // FIN_ROLL_REVERSE_MASK bit n independently negates its roll response.
    static constexpr float   FIN_AZIMUTH_0_DEG = 0.0f;
    static constexpr float   FIN_AZIMUTH_1_DEG = 90.0f;
    static constexpr float   FIN_AZIMUTH_2_DEG = 180.0f;
    static constexpr float   FIN_AZIMUTH_3_DEG = 270.0f;
    static constexpr uint8_t FIN_REVERSE_MASK      = 0;
    static constexpr uint8_t FIN_ROLL_REVERSE_MASK = 0;
    // Minimum airspeed for guidance (below this, fins have no authority)
    static constexpr float PN_MIN_SPEED_MPS = 15.0f;
    // Delay after burnout before engaging guidance (ms)
    static constexpr uint16_t PN_COAST_DELAY_MS = 0;
    // Guided-mode accel(m/s^2) -> fin(deg) scale (coast path; was an inline literal)
    static constexpr float PN_ACCEL_TO_FIN_DEG = 4.0f;
    // Target mode + point (ENU m rel. pad). OVERHEAD (0) = directly over pad (E=N=0).
    static constexpr uint8_t PN_TARGET_MODE = 0;  // GUIDE_TARGET_OVERHEAD
    static constexpr float   PN_TARGET_E_M  = 0.0f;
    static constexpr float   PN_TARGET_N_M  = 0.0f;

    // --- Guidance law selection (#534) ---------------------------------
    // Which law the FC configures at boot.  GUIDE_LAW_PN (0) preserves every
    // flight flown to date; GUIDE_LAW_STATION_KEEP (1) is the PD regulator
    // from #335 (no 1/range singularity, degrades gracefully).  Overridden at
    // runtime by NVS "glw" and by the app's GuidanceConfigData push, so this
    // is only the factory default.  FLIPPING THIS TO 1 IS A ONE-LINE CHANGE
    // AND IS DELIBERATELY DEFERRED to #534's acceptance sim A/B — do not flip
    // it until the sim run says station-keep beats PN on this airframe.
    static constexpr uint8_t GUIDANCE_LAW_DEFAULT = 0;  // GUIDE_LAW_PN

    // Station-keep PD gains: a_lat = -kp*offset - kd*vel (ENU horizontal).
    // Seeded from the 6-DOF sim's guidance scenario, NOT from the library
    // constructor (which is 0.5 / 1.5) and NOT from the closed_loop_sim
    // dataclass defaults (0.5 / 1.0) — the scenario is the only one of the
    // three that has actually been flown in the sim:
    //   tinkerrocket-sim/src/tinkerrocket_sim/simulation/scenarios.py:164-165
    //     pn_kp_pos=0.8, pn_kd_vel=1.5
    // Damping check at these values: wn = sqrt(kp) = 0.89 rad/s,
    // zeta = kd / (2*sqrt(kp)) = 0.84 — slightly under critical, ~7 s natural
    // period, which is long compared to a ~15 s coast.  PROVISIONAL: final
    // gains are pending the #534 acceptance sim A/B (PN vs station-keep on the
    // same scenario); keep these in lockstep with scenarios.py and with
    // RocketProfile.swift's pnKpPos/pnKdVel defaults.
    static constexpr float PN_KP_POS_PER_S2 = 0.8f;
    static constexpr float PN_KD_VEL_PER_S  = 1.5f;

    // Max horizontal aim-point radius from the pad (m), station-keep only.
    // The FC owns magnitude policy — TR_GuidancePN.h explicitly refuses to
    // range-limit the aim point because a silent library clamp would make the
    // FLOWN target disagree with the uploaded and logged one.
    //
    // Why 100 m: the PD term alone saturates long before that.  At
    // kp = 0.8 /s^2 an offset of 25 m commands 0.8*25 = 20 m/s^2, i.e. the
    // ENTIRE PN_MAX_ACCEL_MPS2 budget, and this airframe cannot deliver even
    // that — the 67 mm fin tabs give near-negligible pitch/yaw authority on a
    // G80 (surface size is the lever, not stability margin).  Past ~25 m the
    // law is a saturated open-loop lean, so the limit is not a control bound
    // but a SANITY bound: 4x the saturation radius leaves ample room for a
    // real Drift-Cast (#435) wind offset (a ~600 m apogee flight drifts tens
    // of metres) while making a unit error, a garbage float, or a latitude
    // accidentally sent as metres impossible to fly.
    // REJECT, never clamp — see the handler.
    static constexpr float GUIDANCE_MAX_AIM_RADIUS_M = 100.0f;

    // ### Burnout Detection ###
    // Consecutive FLIGHT-LOOP TICKS of negative body-X accel required to
    // latch burnout_detected. #568: counted per burnoutDetectStep() call —
    // once per loop_fc() pass (~1 kHz), NOT per IMU sample (the IMU logs at
    // up to 3840 Hz; the loop consumes the freshest sample each tick). At
    // ~1 kHz loop rate 50 ticks ≈ 50 ms of sustained deceleration — long
    // enough to filter vibration / sensor noise / wind gusts that briefly
    // drop body_ax below zero during boost, short enough to catch real
    // motor cutoff within one PN coast delay tick. See issue #197.
    static constexpr uint16_t BURNOUT_NEG_HYSTERESIS = 50;

    // ### Deployment Detection ###
    // Tunables for tr::deploymentDetectStep (TR_KinematicChecks/
    // DeploymentDetector.h) — see that header for what each path measures and
    // where it fails.  Counts are FLIGHT-LOOP TICKS (~1 kHz), same convention
    // as BURNOUT_NEG_HYSTERESIS above and LANDING_IMPACT_COUNT in
    // TR_KinematicChecks: the loop steps the detector once per pass no matter
    // what ODR the IMU is logging at.
    //
    // FAST path — the ejection charge's two coincident signatures.
    // 8 g: post-burnout coast decel is ~1-2 g on the motors we fly and never
    // exceeds ~12 g even in a descent tumble (the LANDING_IMPACT_G rationale),
    // while an ejection charge and canopy snap read tens of g. accel_norm
    // switches to the high-g channel near low-g saturation, so the shock is
    // measured, not clipped.
    static constexpr float    DEPLOY_SHOCK_G          = 8.0f;
    static constexpr float    DEPLOY_SHOCK_MS2        = DEPLOY_SHOCK_G * 9.80665f;
    static constexpr uint16_t DEPLOY_SHOCK_COUNT      = 3;      // ~3 ms
    // 3 m between consecutive BMP585 samples (500 Hz, ~0.1-0.3 m RMS noise).
    // Real motion moves indicated altitude ~0.2 m per sample even at 100 m/s,
    // so this is ~10 sigma of noise and an order of magnitude past flight.
    static constexpr float    DEPLOY_BARO_STEP_M      = 3.0f;
    // Shock and baro step must both be this recent. Wide enough to also catch
    // the case where the FC sees the pressure spike at the charge but the
    // acceleration only at canopy inflation a fraction of a second later.
    static constexpr uint32_t DEPLOY_COINCIDENCE_MS   = 250;
    //
    // SLOW path — descent-rate collapse (the canopy signature).
    // Arm above 18 m/s, confirm sustained below 10 m/s. Drogue descent runs
    // 25-30 m/s and a main 5-7 m/s, so the gap is wide; the 200 ms dwell
    // rejects the momentary rate wobble at inflation.
    static constexpr float    DEPLOY_BALLISTIC_MPS    = 18.0f;
    static constexpr float    DEPLOY_CANOPY_MPS       = 10.0f;
    static constexpr uint16_t DEPLOY_CANOPY_COUNT     = 200;    // ~200 ms
    //
    // No detection this soon after launch, on top of the burnout gate. Covers
    // a short-burn motor whose burnout latches while the airframe is still in
    // the high-vibration part of the flight.
    static constexpr uint32_t DEPLOY_LAUNCH_LOCKOUT_MS = 1000;

    // ### Ground Test ###
    // Attitude-hold gain: virtual accel command per unit tilt (m/s² per rad)
    // Higher = more aggressive correction.  Only affects direction test on ground.
    static constexpr float GROUND_TEST_TILT_GAIN = 10.0f;
    // Accel-to-fin gain used on the ground (same as flight accel_to_fin_deg)
    static constexpr float GROUND_TEST_ACCEL_TO_FIN = 4.0f;
    // Roll-rate deadband (dps) for the ground-test roll PID input.
    // On the bench there is no aerodynamic damping, so the PID will react
    // to IMU noise with fin commands, the servo motion vibrates the board,
    // the gyro sees more noise, and a positive-feedback limit cycle can
    // drive the output to the ±MAX_CMD clamp even with the rocket still.
    // Zeroing the rate below this threshold breaks the feedback loop.
    // Stationary gyro noise is ~0.1 dps; any real roll you care about is
    // many tens of dps — 2 dps is well clear of noise and far below signal.
    static constexpr float GROUND_TEST_ROLL_RATE_DEADBAND_DPS = 2.0f;

    // ### Indicators (pins in board header) ###
    static constexpr bool ENABLE_SOUNDS = false;
    static constexpr uint16_t BLUE_LED_FLASH_MS = 40;
    static constexpr uint32_t HEARTBEAT_BEEP_INTERVAL_MS = 1000;
    static constexpr uint16_t HEARTBEAT_BEEP_FREQ_HZ = 2200;
    static constexpr uint16_t HEARTBEAT_BEEP_DURATION_MS = 50;
    // Set true if you also want periodic beeps during INFLIGHT.
    static constexpr bool HEARTBEAT_BEEP_IN_FLIGHT = false;

    // ### I2C Parameters (command/config channel only; pins in board header) ###
    static constexpr uint8_t ESP_I2C_ADR = 0x42;
    static constexpr uint32_t ESP_I2C_FREQ_HZ = 400'000; // Reduced — only commands now
    static constexpr uint16_t I2S_TX_QUEUE_LEN = 512;    // Must handle ~2160 frames/sec burst rate
    // #402: after sending I2C_TX_RESYNC the FC suspends ALL I2C polling for
    // this window so the OC can reset its slave device at guaranteed bus-idle
    // (a reset racing an in-flight read destroys it — #279).  Sized to cover
    // the OC's observed loop-stall worst cases (#398, ~1.2 s) with margin; the
    // detector re-sends every 8 further failures if the OC lagged past it.
    static constexpr uint32_t I2C_RESYNC_GRACE_MS = 2000;

    // ### I2S Parameters (high-frequency telemetry FC→OC; pins in board
    //     header) ###
    // I2S bandwidth = sample_rate * 4 bytes (16-bit stereo).
    // Higher rate = faster DMA buffer turnover = less stale data.
    // 44100 Hz = 176 KB/s.  Raised from 22050 with the IMU 960 -> 1920 Hz
    // step: at 22050 the link was already ~76% full (~67 KB/s) and doubling
    // the IMU frames would not fit.  New steady-state budget ~97 KB/s (55%):
    // IMU 58 K + NonSensor 24 K + baro 10 K + mag/GNSS/power/misc ~5 K.
    // IMPORTANT: must match the OC's I2S_SAMPLE_RATE — flash both together.
    static constexpr uint32_t I2S_SAMPLE_RATE = 44100;  // Must match OC
 
    // ### Execution Cores ###

    // Sensor (IMU, GNSS, and power) receiver
    static constexpr uint8_t SENSOR_CORE = 0;

};

#endif // CONFIG_H
