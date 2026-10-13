#pragma once
// Bench-only section profiler for the flight computer's flight task
// (-DTR_LOOP_PROFILE=1 on the idf.py command line). Never set for flight.
//
// The model is a lap timer. Each TR_LP_LAP(X) charges everything that ran on
// this core since the previous lap -- cycles, plus the two Xtensa performance
// counters where the core has them -- to section X. The laps are placed so that
// they cover the whole pass with no gaps, so the sections add up to the wall
// time of the core: a section is "what ran between the lap before it and its
// own lap", interrupts and any blocking included. The sections are listed in
// pass order.
//
// The EKF component marks its own sub-steps (the ekf.* sections), so they sit
// inside the flight task's sequence like any other lap. Everything here
// compiles to nothing unless TR_LOOP_PROFILE is set, and the implementation
// (tr_lp_lap, the report) lives in the flight computer's main/loop_profile.h.

#if defined(TR_LOOP_PROFILE) && TR_LOOP_PROFILE && defined(ESP_PLATFORM)
#define TR_LP_ENABLED 1
#else
#define TR_LP_ENABLED 0
#endif

#if TR_LP_ENABLED
#include <stdint.h>

// id, report name. Pass order.
#define TR_LP_SECTIONS(X)                                                        \
    X(YIELD,       "yield+wdt")      /* taskYIELD + task WDT reset, between passes */ \
    X(PRE,         "pre")            /* OTA-valid mark, OTA pump, I2S TX recovery */ \
    X(IMU_DRAIN,   "imu_drain")      /* ISM6 queue pops, window add, batch + enqueue */ \
    X(SENSORS,     "sensors")        /* baro, mags, GNSS: get, convert, enqueue */ \
    X(PAD_MISC,    "cfg+magcal")     /* config report, mag-cal status + verify */ \
    X(GATE,        "gate")           /* the 1 kHz gate test */                   \
    X(IMU_MEAN,    "imu_mean")       /* window mean, convertISM6HG256Data */     \
    X(BARO_ALT,    "baro_alt")       /* ground reference, powf altitude */       \
    X(ORIENT,      "orient")         /* accel select, pad orientation estimator */ \
    X(EKF_PREP,    "ekf_prep")       /* EKF inputs, GNSS gates, feed, ref pos */ \
    X(EKF_INIT,    "ekf_init")       /* ekf.init and its WMM calls (once) */     \
    X(EKF_IN,      "ekf.in")         /* updateCore steps 1-6 */                  \
    X(EKF_NAV,     "ekf.nav")        /* quaternion, velocity, double position */ \
    X(EKF_COV,     "ekf.cov")        /* Jacobian, Q, P propagation */            \
    X(EKF_ACCEL,   "ekf.accel")      /* accelMeasUpdate */                       \
    X(EKF_MAG,     "ekf.mag")        /* magMeasUpdate */                         \
    X(EKF_GNSS,    "ekf.gnss")       /* measUpdate + GNSS heading aids */        \
    X(EKF_OUT,     "ekf.out")        /* updateCore steps 11-12 */                \
    X(EKF_ZUPT,    "ekf.zupt+logs")  /* landed ZUPT, canopy/shock log checks */  \
    X(EKF_BARO,    "ekf.baro")       /* baro gate + baroMeasUpdate */            \
    X(EKF_EXTRACT, "ekf_extract")    /* outputs, LLA -> local ENU */             \
    X(I2C_POLL,    "i2c_poll")       /* OC status poll, 4 Hz */                  \
    X(CMD,         "cmd_dispatch")   /* OC command dispatch + bus release */     \
    X(PAD_CAL,     "pad_cal")        /* sensor calibration completion */         \
    X(KINEMATICS,  "kinematics")     /* GnssAscentGate + kinematicChecks */      \
    X(GATES,       "deploy_gates")   /* main deploy, recovery arm, deployment */ \
    X(TESTMODE,    "test_modes")     /* #363/#1121 failsafes, ground test */     \
    X(STATE,       "state_machine")  /* flight state machine */                  \
    X(SIM,         "sim")            /* sim edges + sim diag */                  \
    X(TELEM,       "telemetry")      /* NonSensorData, health, I2S enqueue */    \
    X(SERVICES,    "led+camera")     /* boot cue, heartbeat, LED, camera */      \
    X(DIAG,        "diag_print")     /* the 1 Hz console diagnostics */          \
    X(REPORT,      "profile_report") /* this profiler's own report */

namespace tr_lp {
enum Section : uint8_t {
#define TR_LP_ENUM(id, name) id,
    TR_LP_SECTIONS(TR_LP_ENUM)
#undef TR_LP_ENUM
    SECTION_COUNT
};
}  // namespace tr_lp

extern "C" void tr_lp_lap(uint32_t section);
#define TR_LP_LAP(s) tr_lp_lap((uint32_t)::tr_lp::s)
#else
#define TR_LP_LAP(s) ((void)0)
#endif
