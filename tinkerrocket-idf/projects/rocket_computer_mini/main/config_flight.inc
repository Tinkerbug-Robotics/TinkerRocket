// config_flight.inc — flight-side policy constants for the rocket-computer-mini.
//
// STRUCT-BODY FRAGMENT: textually included inside `struct config : board_pins`
// by config.h.  Bare `static constexpr` members only — no #pragma once, no
// includes, no namespace.
//
// Every value here is copied verbatim from the flight_computer's
// projects/flight_computer/main/config.h unless a comment says otherwise, so
// ported flight.cpp code compiles unchanged and future FC tuning changes can
// be cross-applied by diffing the two files.  The two deliberate departures
// are GNSS_UPDATE_RATE (LC86G ceiling) and the EKF_DECIMATION re-bench note.
//
// Constants owned by dropped subsystems (servo/PID/roll/fin/guidance/camera/
// piezo/I2S/I2C-link) are deliberately absent — nothing may reference them.

    // ### SPI parameters (pins in board_v1.h) ###
    static constexpr uint32_t SPI_SPEED = 10'000'000; // 10 MHz

    // ### IIS2MDC I2C parameters ###
    // On the mini the magnetometer rides the shared PWR_SDA/PWR_SCL bus with
    // the INA230 (one i2c_master bus, created in main.cpp, handle passed into
    // the collector).  The frequency is per-device (scl_speed_hz).
    static constexpr uint32_t IIS2MDC_I2C_FREQ_HZ = 400'000;
    static constexpr uint8_t IIS2MDC_I2C_ADDR = 0x1E;

    // ### Data Update Rates (Hz) ###
    static constexpr uint16_t FLIGHT_LOOP_UPDATE_RATE = 1000;
    // MINI: 10 Hz, not the FC's 18.  The Quectel LC86G tops out at 10 Hz
    // ($PAIR050,100); asking for more does nothing, and the LC86 driver takes
    // the requested value literally rather than self-limiting the way the
    // u-blox path did.  The collector's poll task checks at half this period
    // (50 ms), fast enough to catch every 10 Hz epoch.
    static constexpr uint16_t GNSS_UPDATE_RATE = 10;
    static constexpr uint16_t BMP585_UPDATE_RATE = 500;
    // MMC5983MA is NOT FITTED on the mini (board_v1.h USE_MMC5983MA=false).
    // The constant survives only because the collector ctor takes the rate
    // positionally; the part is never initialized.
    static constexpr uint16_t MMC5983MA_UPDATE_RATE = 200;
    // Collector-construction IMU rate.  Not the effective default: the shipped
    // default setting is IMU_RATE_DYNAMIC (3840 Hz to deployment, then 960 Hz),
    // which flight_setup() stages over this value before begin(); the user can
    // pin a fixed 960/1920/3840 via the app (NVS "imu"/"rate").  The FC's I2S
    // link-budget analysis for these rates does not carry over — on the mini
    // every sample goes straight into the TR_LogToFlash ring and NAND flush on
    // the SAME processor.  3840 Hz ≈ 115 KB/s of ISM6 frames alone; the ring/
    // flush budget on the S3 is UNVERIFIED (bench gauges: imu_q_drops,
    // ring_overruns/ring_highwater from the comms side's log stats).
    static constexpr uint16_t ISM6HG256_UPDATE_RATE = 1920;
    static constexpr uint16_t NON_SENSOR_UPDATE_RATE = 500;
    // ISM6 full-scale configuration used by SensorConverter and the collector.
    static constexpr uint8_t ISM6_LOW_G_FS_G = 16;
    static constexpr uint16_t ISM6_HIGH_G_FS_G = 256;
    static constexpr uint16_t ISM6_GYRO_FS_DPS = 4000;

    // ### Sensor Frame Rotations ###
    // !!! COPIED FROM THE FLIGHT COMPUTER (V7/V8 layouts) — these are PCB
    // placement facts, and the mini is a NEW layout.  They MUST be re-derived
    // from the mini's placement (or bench-characterized the way #204 did for
    // the IIS2MDC) before first flight.  Wrong Z rotations silently corrupt
    // heading and roll-rate sign.
    // ISM6 frame -> board frame rotation (deg), CCW positive about +Z.
    static constexpr float ISM6HG256_ROT_Z_DEG = -45.0f;   // VERIFY on mini layout
    // IIS2MDC -> board frame (deg), CCW positive about +Z (#204 method).
    static constexpr float IIS2MDC_ROT_Z_DEG = 90.0f;      // VERIFY on mini layout

    // Board -> rocket mounting orientation (TR_Orientation code, 0-23).
    // 0 = board +X toward the nose.  Pad-gravity auto-detect refines this at
    // runtime; this constant is the manual boot default.
    static constexpr uint8_t BOARD_TO_ROCKET_ORIENT = 0;

    // ### Pyro timing (pins in board_v1.h) ###
    // One shared arming FET feeds all four squib drivers.  ARM is raised only
    // momentarily — for a fire pulse or an explicit test — never latched in
    // flight.  E-matches commit in <10 ms; 200 ms is ignition margin while
    // bounding the I2t a shorted match can dump into the channel FET on 2S.
    static constexpr uint32_t PYRO_FIRE_DURATION_MS    = 200;
    // Time between raising ARM and reading CONT / pulsing FIRE, giving the
    // arming FET its turn-on settle margin.
    static constexpr uint32_t PYRO_ARM_SETTLE_MS       = 10;

    // EKF pad heading: compass heading of board Z+ (deg, 0=North, 90=East)
    static constexpr float PAD_HEADING_DEG = 0.0f;

    // Magnetic declination fallback (deg, EAST-positive) used until a good GPS
    // fix lets TR_GeoMag (WMM2025) compute the true value from lat/lon/date.
    static constexpr float MAGNETIC_DECLINATION_DEG = 0.0f;

    // GNSS quality gates — layered rejection of bad fixes.
    // h_acc is uint8_t (max 255); LC86G accuracy comes from $PQTMEPE and may
    // read 255 = "unknown", so the h_acc gates stay disabled (0) like the FC.
    static constexpr uint8_t  GNSS_MIN_SATS        = 4;      // minimum satellite count
    static constexpr float    GNSS_MAX_HACC_M       = 0.0f;  // max h_acc for EKF update (0 = disable)
    static constexpr float    GNSS_MAX_HACC_INIT_M  = 0.0f;  // max h_acc for EKF init  (0 = disable)
    static constexpr float    GNSS_MAX_VEL_INIT_MPS = 2.0f;  // max velocity magnitude for EKF init
    static constexpr uint8_t  GNSS_MIN_SATS_INIT    = 5;     // minimum sats for EKF init (stricter)
    // #834 item 4: minimum pad fixes averaged into ref_alt_m before the GNSS
    // AGL backstop may authorise a main deploy (mirrors the FC's config.h).
    static constexpr uint32_t MAIN_DEPLOY_REF_MIN_FIXES = 300;

    // EKF scorecard health (#303): converged when worst-axis attitude and
    // velocity covariance sit below these (see FC config.h for the bench
    // derivation).
    static constexpr float    EKF_ATT_VAR_OK        = 0.1f;   // rad², worst attitude axis (~18° 1σ)
    static constexpr float    EKF_VEL_VAR_OK        = 4.0f;   // (m/s)², worst velocity axis (~2 m/s 1σ)

    // Barometer spike rejection (#450: rate-aware) — reject only when BOTH the
    // absolute deadband and the apparent rate over the actual pair spacing are
    // exceeded.
    static constexpr float BARO_SPIKE_THRESH_M  = 5.0f;
    static constexpr float BARO_SPIKE_RATE_MPS  = 500.0f;

    // Barometer fusion throttle: minimum spacing between baro samples ACCEPTED
    // into the EKF measurement update (0 disables).  Fusing at the full ODR
    // tripled EKF cost on the P4 (#450: 554 → ~1650 µs); 50 Hz keeps altitude
    // aiding far above its information rate.
    static constexpr uint32_t BARO_FUSE_MIN_INTERVAL_US = 20000u;

    // Transonic barometer lockout with hysteresis (shock-induced static
    // pressure error near Mach 1).  0 disables.
    static constexpr float BARO_MACH_LOCKOUT_ON  = 260.0f;  // m/s (~Mach 0.76)
    static constexpr float BARO_MACH_LOCKOUT_OFF = 240.0f;  // m/s (~Mach 0.70)

    // ==================================================================
    // EKF decimation: run EKF predict+update every Nth flight-loop tick.
    //
    // !!! S3 RE-BENCH REQUIRED before first flight.  The value 2 was
    // benchmarked on the ESP32-P4 (554 µs/tick average, ~480 Hz EKF from a
    // ~960 Hz loop).  The ESP32-S3 is a different core at 240 MHz with a
    // different FPU; if it cannot hold a ~1 kHz flight loop with decimation 2,
    // the ONLY sanctioned fix is raising this to 4 — the detector hysteresis
    // counts (BURNOUT_NEG_HYSTERESIS, DEPLOY_*_COUNT, and the counters inside
    // TR_KinematicChecks) assume ~1 kHz kinematicChecks() calls and must NOT
    // be silently retuned to compensate.  Gauge: the 1 Hz [TIMING] line
    // (ekf avg/max, loop cnt/s).
    // ==================================================================
    static constexpr uint8_t EKF_DECIMATION = 2;

    // ### Burnout Detection ###
    // Consecutive FLIGHT-LOOP TICKS of negative body-X accel required to latch
    // burnout_detected.  #568: counted per burnoutDetectStep() call — once per
    // loop pass (~1 kHz), NOT per IMU sample.  ~50 ms of sustained
    // deceleration.  See issue #197.
    static constexpr uint16_t BURNOUT_NEG_HYSTERESIS = 50;

    // ### Deployment Detection ###
    // Tunables for tr::deploymentDetectStep (TR_KinematicChecks/
    // DeploymentDetector.h).  Counts are FLIGHT-LOOP TICKS (~1 kHz), same
    // convention as BURNOUT_NEG_HYSTERESIS.  Full rationale in the FC config.h.
    //
    // FAST path — the ejection charge's two coincident signatures.
    static constexpr float    DEPLOY_SHOCK_G          = 8.0f;
    static constexpr float    DEPLOY_SHOCK_MS2        = DEPLOY_SHOCK_G * 9.80665f;
    static constexpr uint16_t DEPLOY_SHOCK_COUNT      = 3;      // ~3 ms
    static constexpr float    DEPLOY_BARO_STEP_M      = 3.0f;
    static constexpr uint32_t DEPLOY_COINCIDENCE_MS   = 250;
    // SLOW path — descent-rate collapse (the canopy signature).
    static constexpr float    DEPLOY_BALLISTIC_MPS    = 18.0f;
    static constexpr float    DEPLOY_CANOPY_MPS       = 10.0f;
    static constexpr uint16_t DEPLOY_CANOPY_COUNT     = 200;    // ~200 ms
    // No detection this soon after launch, on top of the burnout gate.
    static constexpr uint32_t DEPLOY_LAUNCH_LOCKOUT_MS = 1000;

    // ### Execution Cores ###
    // Sensor (IMU + GNSS) poll tasks.  The flight task owns core 1.
    static constexpr uint8_t SENSOR_CORE = 0;
