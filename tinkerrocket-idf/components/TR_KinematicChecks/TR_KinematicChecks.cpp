#include <TR_KinematicChecks.h>
#include <algorithm>

namespace {
// Impact-detected landing fast path (see issue #113).
// Descent tumbling never exceeds ~12 g; impact on RolyPoly 5/3/26 peaked at
// 218 g. 15 g cleanly separates them with no false positives in descent.
// Gated on apogee + low altitude so launch (~7 g) and ejection (~22 g at
// apogee) cannot trigger.
constexpr float    LANDING_IMPACT_G       = 15.0f;
constexpr float    LANDING_IMPACT_ALT_M   = 20.0f;
constexpr uint16_t LANDING_IMPACT_COUNT   = 5;       // ~5 ms at 1 kHz
constexpr float    G_MS2                  = 9.80665f;

// Baro rate-gate (#166): rejects |Δpalt|/dt above any plausible rocket
// dynamic. Self-scales with dt so legitimate large changes after a sensor
// dropout still pass.  Ejection spikes are O(10-100 km/s), max plausible
// rocket dynamic is ~200 m/s, so 300 m/s gives wide headroom both ways.
constexpr float    MAX_BARO_RATE_MS         = 300.0f;
// After this many consecutive rejects the gate accepts the next sample
// unconditionally and reseeds, preventing a permanently wedged sensor
// from blinding the detector for the rest of the flight.
constexpr uint8_t  MAX_CONSEC_BARO_REJECTS  = 10;

// Apogee sub-detector leaky-counter thresholds (#166).
// Each test runs +1 on pass / -1 on miss (clamped) so a single false
// positive can un-vote once conditions reverse, instead of latching for
// the rest of the flight. HI thresholds preserve original first-fire
// timing: ~6 ms at 1 kHz for baro/pitch/vel, ~800 ms at 5 Hz for GPS.
constexpr uint8_t  APOGEE_COUNT_MAX         = 10;
constexpr uint8_t  APOGEE_COUNT_HI          = 6;
constexpr uint8_t  GPS_APOGEE_COUNT_MAX     = 8;
constexpr uint8_t  GPS_APOGEE_COUNT_HI      = 4;
// GPS apogee fires on sustained Doppler descent faster than this (m/s).  GPS
// *velocity* is real-time; GPS *position* lags ~4 s and dives during boost
// (#237/#242), so the old altitude-below-peak test fired wildly off.
constexpr float    GPS_VEL_APOGEE_DESCENT_MPS = 1.0f;
// Max age of the last GPS fix for GPS to count as an apogee voter (#262).
// Deliberately tight — boost-to-apogee is only ~6-8 s and GPS runs ~18 Hz
// (~55 ms/fix, worst observed gap ~136 ms), so a fix older than this is stale
// for an apogee decision (a 5 s-old fix would be from the launch rail).  Much
// tighter than the landing vote's 5 s gate, which is a slower ground event.
constexpr uint32_t GPS_APOGEE_FRESH_MS     = 500;

// Launch accel-only fallback (#258): when the baro is INVALID, latch launch on
// sustained high-G alone.  Deliberately a much higher bar than the baro-
// confirmed path (3 g vs 2 g, 250 vs 50 samples) so handling/transport can't
// fake it — a missed launch means recovery never arms (ballistic), so we
// over-confirm.  Gated on !baro_healthy, so accel never decides launch while
// the baro is valid.
//
// The counter is in flight-loop iterations, and the flight-logic block is
// gated at flight_loop_period = 1e6 / config::FLIGHT_LOOP_UPDATE_RATE
// (main.cpp:208, 4051) = 1 kHz, so 250 samples is ~250 ms of wall clock.
// Halved from 500 (~500 ms): a short-burn motor's thrust curve tapers, so the
// *continuous* >3 g window is shorter than the burn time, and a 500 ms gate
// could sit out an entire small-motor boost and never latch — the exact
// ballistic outcome the fallback exists to prevent.  Both counters reset on a
// single sample <= 20 m/s2, so it takes a quarter second of UNINTERRUPTED >3 g.
// A restrained motor burn reads ~1 g (thrust balanced by the rail nets out of
// specific force), a drop is 0 g in free fall and milliseconds at impact, and
// linear hand motion, bumps and vibration are oscillatory — they cross back
// through the 2 g reset floor and can never accumulate.
//
// The one non-boost stimulus that is NOT oscillatory is sustained ROTATION:
// centripetal acceleration is steady, so swinging the airframe with the board
// ~1 m from the pivot at ~5.5 m/s holds 30 m/s2 for as long as the swing lasts.
// A quarter second of that clears the bar.  Unquantified on a real prep bench,
// and the halving is what brings it into reach — but it is not load-bearing on
// its own: entering INFLIGHT arms nothing (enterInflight leaves ARM low and all
// channels Idle), both pyro triggers are apogee-gated, and the apogee vote needs
// burnout_detected, which a nose-up airframe cannot produce.
constexpr float    LAUNCH_ACCEL_FALLBACK_MS2   = 30.0f;  // ~3 g
constexpr uint16_t LAUNCH_ACCEL_FALLBACK_COUNT = 250;    // sustained samples (~250 ms at 1 kHz)

// Baro settle window after burnout. Thrust tail-off can snap the bay pressure
// back from its boost-suction offset (7/05 V2 F1: indicated altitude fell 15 m
// in 0.25 s at burnout while climbing at 46 m/s), which satisfies the baro
// apogee test the instant its burnout gate opens — the boost over-read has
// already ratcheted max_altitude high.  Apogee physically cannot be at
// burnout, so the baro voter is ignored for this long after burnout is first
// seen.  The Layer-2 backstop below is NOT gated by this (its 30 m drop
// threshold is far beyond the transient, and it must stay available in the
// degraded case).
constexpr uint32_t BARO_BURNOUT_SETTLE_MS  = 1000;

// Layer-2 apogee backstop (#257). When the primary vote can't reach quorum
// (e.g. an unhealthy EKF leaves < 2 healthy voters), a healthy + mach-unlocked
// baro that has dropped this far below the running peak while still descending
// is unambiguously "past apogee", independent of the EKF/IMU.  Altitude-
// relative (not time-based), so it is rocket-agnostic; ~30 m past apogee is
// ~24 m/s, fine for a drogue.  Reuses the APOGEE_COUNT_HI/MAX debounce.
constexpr float    APOGEE_BACKSTOP_DROP_M     = 30.0f;

// Landing slow-detector hysteresis (#166). Slow detectors evaluate inside
// the 1 Hz landing_check_dt gate, so HI=4 → 4 s to fire (matches the
// original ``landing_checks > 4`` first-fire timing).
constexpr uint8_t  LANDING_SLOW_COUNT_MAX   = 8;
constexpr uint8_t  LANDING_SLOW_COUNT_HI    = 4;

// Landing sub-detector thresholds.
// Lower bound is -50 m, not 0, because ground-pressure drift across a
// 60-90 s flight can leave a settled palt several m below the pre-launch
// reference (Roly Poly GTV 5/9 settled at palt ≈ -8 m). The rate-gate
// upstream filters deep spikes, so this is just a defensive backstop.
constexpr float    BARO_STABLE_PALT_MIN     = -50.0f;
constexpr float    BARO_STABLE_PALT_MAX     = 50.0f;
constexpr float    BARO_STABLE_DELTA_MAX    = 2.0f;     // |Δpalt| over 1 s
constexpr float    BARO_STABLE_MAX_ALT_MIN  = 15.0f;    // rocket must have flown
constexpr float    GYRO_QUIET_DPS           = 20.0f;
constexpr float    GPS_STATIONARY_SPEED_MS  = 1.0f;     // EKF speed proxy
constexpr float    ACCEL_1G_TOLERANCE_MS2   = 2.94f;    // ≈ 0.3 g

// Extended quiescence (#824) — a deliberately baro-independent path to
// LANDED.  The vote below now requires baro_stable, so a dead or
// out-of-band barometer would otherwise leave the flight running until the
// 10-minute flight-time backstop.  This detector answers "the airframe has
// stopped moving" from the IMU alone, with thresholds picked so a descent
// under canopy cannot reach them.  Longest uninterrupted run while
// descending under chute, measured over 15 logged flights (2026-05..08):
//
//     |roll| < 20 dps  (the vote's threshold)   :  1 s
//     |roll| <  5 dps                           :  1 s
//     ||a|-g| < 0.3 g  (the vote's tolerance)   : 37 s   <-- why accel_1g
//                                                            latches aloft
//     ||a|-g| < 0.05 g                          :  8 s
//     both tight gates together                 :  1 s
//
// while a stationary rocket (233 s of pre-launch log) held |roll| < 5 dps
// for 109 s uninterrupted.  HI=30 therefore sits ~30x above anything a
// canopy has produced and well inside what a rocket lying in a field
// sustains.  The accel gate is not redundant: it independently caps the
// exposure at ~8 s for an airframe that descends without rolling, which is
// the one case the roll gate alone would miss.
constexpr float    QUIESCENT_GYRO_DPS       = 5.0f;
constexpr float    QUIESCENT_ACCEL_TOL_MS2  = 0.49f;    // ≈ 0.05 g
constexpr uint8_t  QUIESCENT_COUNT_HI       = 30;       // ~30 s net quiet
constexpr uint8_t  QUIESCENT_COUNT_MAX      = 45;
}

TR_KinematicChecks::TR_KinematicChecks()
{
    launch_flag = false;
    alt_landed_flag = false;
    alt_apogee_flag = false;
    vel_u_apogee_flag = false;
    gps_apogee_flag = false;
    pitch_apogee_flag = false;
    apogee_flag = false;
    apogee_backstop_flag = false;
    launch_count = 0;
    launch_count_hi = 0;
    max_altitude = 0.0f;
    max_speed = 0.0f;
    landing_check_time = 0;
    landing_look_back_alt = 0.0;
    landing_check_dt = 1000;
    apogee_count = 0;
    vel_apogee_count_ = 0;
    pitch_apogee_count_ = 0;
    backstop_descent_count_ = 0;
    impact_seen_count = 0;
    impact_flag = false;
    baro_stable_flag = false;
    gyro_quiet_flag = false;
    gps_stationary_flag = false;
    accel_1g_flag = false;
    quiescent_flag = false;
    baro_stable_count_ = 0;
    gyro_quiet_count_ = 0;
    gps_stationary_count_ = 0;
    accel_1g_count_ = 0;
    quiescent_count_ = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;
    burnout_seen_ = false;
    burnout_seen_ms_ = 0;
    baro_gate_init_ = false;
    palt_accepted_ = 0.0f;
    last_baro_gate_ms_ = 0;
    consec_baro_rejects_ = 0;

    // KF init
    kf_init_ = false;
    alt_est = 0.0f;
    d_alt_est_ = 0.0f;
    P00_ = 10.0f;  P01_ = 0.0f;
    P10_ = 0.0f;   P11_ = 10.0f;

    // Tunable altitude filter parameters
    // R_ ~ altitude measurement variance; BMP585 noise is ~0.17m, but vibration
    // and barometric formula error add noise, so we use a conservative 1 m^2.
    R_  = 1.0f; // m^2
    // Qz_: slow random drift in altitude (bias/random walk)
    Qz_ = 0.05f; // m^2/s
    // Qv_: allows acceleration to change vertical speed; larger → more responsive rate.
    // Rockets experience high accelerations (>20g), so this needs to be large enough
    // to let the velocity estimate track rapid changes.
    Qv_ = 5.0f; // (m/s)^2
}

void TR_KinematicChecks::reset()
{
    launch_flag = false;
    alt_landed_flag = false;
    alt_apogee_flag = false;
    vel_u_apogee_flag = false;
    gps_apogee_flag = false;
    pitch_apogee_flag = false;
    apogee_flag = false;
    apogee_backstop_flag = false;
    launch_count = 0;
    launch_count_hi = 0;
    max_altitude = 0.0f;
    max_speed = 0.0f;
    landing_check_time = 0;
    landing_look_back_alt = 0.0;
    apogee_count = 0;
    vel_apogee_count_ = 0;
    pitch_apogee_count_ = 0;
    backstop_descent_count_ = 0;
    impact_seen_count = 0;
    impact_flag = false;
    baro_stable_flag = false;
    gyro_quiet_flag = false;
    gps_stationary_flag = false;
    accel_1g_flag = false;
    quiescent_flag = false;
    baro_stable_count_ = 0;
    gyro_quiet_count_ = 0;
    gps_stationary_count_ = 0;
    accel_1g_count_ = 0;
    quiescent_count_ = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;
    baro_gate_init_ = false;
    palt_accepted_ = 0.0f;
    last_baro_gate_ms_ = 0;
    consec_baro_rejects_ = 0;
    burnout_seen_ = false;
    burnout_seen_ms_ = 0;
    kf_init_ = false;
    alt_est = 0.0f;
    d_alt_est_ = 0.0f;
    P00_ = 10.0f;  P01_ = 0.0f;
    P10_ = 0.0f;   P11_ = 10.0f;
}

void TR_KinematicChecks::kinematicChecks(float pressure_altitude,
                                         float acc_mag,
                                         float position[3],
                                         float velocity[3],
                                         float roll_rate,
                                         bool  new_baro,
                                         float gps_altitude,
                                         bool  new_gps,
                                         float pitch_rad,
                                         bool  burnout_detected,
                                         bool  baro_locked_out,
                                         float gps_vel_u,
                                         bool  ekf_healthy,
                                         bool  baro_healthy)
{
    // Snapshot apogee_flag so the rising-edge reset below sees the
    // state *before* this tick's apogee voting fires (#192).
    const bool apogee_was_set = apogee_flag;

    // ### Baro rate-gate (#166) ###
    // Reject obvious spikes (ejection charges, sensor glitches) before they
    // poison the KF or trip the landing fast path.  The rate test self-scales
    // with dt so a single missed sample doesn't make the next legitimate
    // reading look like a spike.  After MAX_CONSEC_BARO_REJECTS in a row the
    // next sample is accepted unconditionally so a wedged sensor reseeds.
    if (new_baro)
    {
        if (baro_gate_init_)
        {
            uint32_t dms = millis() - last_baro_gate_ms_;
            float dt = (dms < 1u) ? 0.001f : (dms * 0.001f);
            float rate = fabs(pressure_altitude - palt_accepted_) / dt;
            if (rate > MAX_BARO_RATE_MS &&
                consec_baro_rejects_ < MAX_CONSEC_BARO_REJECTS)
            {
                consec_baro_rejects_++;
                new_baro = false;
                pressure_altitude = palt_accepted_;
            }
            else
            {
                consec_baro_rejects_ = 0;
                palt_accepted_ = pressure_altitude;
                last_baro_gate_ms_ = millis();
            }
        }
        else
        {
            palt_accepted_ = pressure_altitude;
            last_baro_gate_ms_ = millis();
            baro_gate_init_ = true;
        }
    }
    else if (baro_gate_init_)
    {
        pressure_altitude = palt_accepted_;
    }

    // ### Altitude ###

    // Kalman-filter altitude and altitude rate.
    // Predict step runs every call; update step only when new_baro is true
    // to avoid double-counting stale measurements.
    // Runs before launch check so d_alt_est_ is current for altitude rate confirmation.
    updateAltKF(pressure_altitude, new_baro);

    // ### Check for Launch Conditions ###

    // 50 consecutive readings of >20 m/s^2 (~42ms at 1200 Hz) AND
    // Kalman-filtered altitude rate > 1.0 m/s (confirms actual upward motion).
    // This rejects handling bumps which produce short accel spikes but no altitude change.
    if (launch_flag == false)
    {
        if (acc_mag > 20.0)
        {
            launch_count++;
            // Separate sustained-high-G counter for the accel-only fallback.
            if (acc_mag > LAUNCH_ACCEL_FALLBACK_MS2) launch_count_hi++;
            else                                     launch_count_hi = 0;

            if (launch_count > 50 && d_alt_est_ > 1.0)
            {
                // Primary: accel + baro-confirmed climb (fast).
                launch_flag = true;
            }
            else if (!baro_healthy &&
                     launch_count_hi > LAUNCH_ACCEL_FALLBACK_COUNT)
            {
                // #258 accel-only fallback — ONLY when the baro is invalid.
                // On a healthy baro this branch is unreachable (the primary
                // latches first), so accel never decides launch unless the baro
                // can't.  A quarter second of sustained >3 g with a dead baro is
                // an unambiguous boost; latching here is what keeps recovery from
                // never arming (ballistic) on a baro failure.
                launch_flag = true;
            }
        }
        else
        {
            launch_count = 0;
            launch_count_hi = 0;
        }
    }

    // Maximum achieved altitude (KF-smoothed — raw P_alt during boost saw
    // ±15 m sample-to-sample spikes on GTV 67mm 2026-05-09 that the previous
    // 50 m spike-reject window let through, ratcheting max_altitude above the
    // true climb and tripping the baro apogee detector during ascent — #142).
    // First reading always accepted.
    if (max_altitude == 0.0f || fabs(alt_est - max_altitude) < 50.0f) {
        max_altitude = std::max(max_altitude, alt_est);
    }

    // ================================================================
    // ### Landing Detection: impact + a vote + quiescence (#166, #824) ###
    // Three ways to latch, in the order they are tested below.
    //
    //  1. Fast path (impact): fires master immediately on unambiguous
    //     high-G ground contact, gated on apogee + ground proximity.
    //  2. Slow path: an N-1-of-N vote over four settle-state detectors
    //     (Baro-stable, Gyro-quiet, GPS-stationary, Accel-1g) — independent
    //     signals, so a single noise source cannot drive a false landed.
    //     N is 3 or 4, not always 4: the GPS voter is counted only while
    //     its fix is fresh.  Baro-stable is MANDATORY rather than merely
    //     one of N, because it is the only detector here that carries
    //     altitude information and a quorum reachable without it is
    //     unsound (#824).
    //  3. Extended quiescence: a separate, deliberately baro-independent
    //     path, so that making Baro-stable mandatory cannot strand a
    //     flight whose barometer is dead or whose landing site sits
    //     outside the Baro-stable band.
    //
    // Master ``alt_landed_flag`` latches once true.
    // ================================================================

    // --- Fast path: high-G impact, gated on apogee + ground proximity.
    // The baro rate-gate upstream already filters glitch spikes, but the
    // defensive ``palt > -10`` lower bound mirrors the gate's intent so
    // this path is robust even if the gate is bypassed in future.
    // One-shot latch — impact_flag stays true once a real impact fires.
    if (apogee_flag
     && pressure_altitude > -10.0f
     && pressure_altitude < LANDING_IMPACT_ALT_M
     && acc_mag > LANDING_IMPACT_G * G_MS2)
    {
        impact_seen_count++;
        if (impact_seen_count >= LANDING_IMPACT_COUNT)
        {
            impact_flag = true;
        }
    }
    else
    {
        impact_seen_count = 0;
    }

    // --- Slow detectors: evaluated at 1 Hz inside the existing time gate.
    if (millis() > landing_check_time + landing_check_dt)
    {
        float landing_altitude_change = fabs(pressure_altitude - landing_look_back_alt);
        landing_look_back_alt = pressure_altitude;
        landing_check_time = millis();

        // Sub 1: Baro-stable (was the original slow path; +lower bound)
        //
        // Gated on baro_healthy (#824).  This detector is now mandatory in the
        // vote, on the grounds that it is the only one carrying altitude
        // information — so it must not be satisfiable by a barometer that has
        // stopped telling the truth.  It would be: pressure_altitude retains
        // its last value when the sensor stops responding, which makes
        // landing_altitude_change exactly 0 and looks maximally "stable".  A
        // dead baro frozen at a reading inside the band would then hand the
        // vote its mandatory voter for free, and gyro_quiet + accel_1g could
        // carry the rest at altitude — reopening #824 through the very
        // detector that was supposed to close it.
        const bool baro_stable_pass = (baro_healthy
                                       && pressure_altitude >= BARO_STABLE_PALT_MIN
                                       && pressure_altitude <= BARO_STABLE_PALT_MAX
                                       && landing_altitude_change < BARO_STABLE_DELTA_MAX
                                       && max_altitude > BARO_STABLE_MAX_ALT_MIN);
        if (baro_stable_pass) {
            if (baro_stable_count_ < LANDING_SLOW_COUNT_MAX) baro_stable_count_++;
        } else {
            if (baro_stable_count_ > 0) baro_stable_count_--;
        }
        if (baro_stable_count_ >= LANDING_SLOW_COUNT_HI) baro_stable_flag = true;
        else if (baro_stable_count_ == 0)                baro_stable_flag = false;

        // Sub 2: Gyro-quiet — roll_rate (future: generalize to all axes).
        const bool gyro_quiet_pass = (fabs(roll_rate) < GYRO_QUIET_DPS);
        if (gyro_quiet_pass) {
            if (gyro_quiet_count_ < LANDING_SLOW_COUNT_MAX) gyro_quiet_count_++;
        } else {
            if (gyro_quiet_count_ > 0) gyro_quiet_count_--;
        }
        if (gyro_quiet_count_ >= LANDING_SLOW_COUNT_HI) gyro_quiet_flag = true;
        else if (gyro_quiet_count_ == 0)                gyro_quiet_flag = false;

        // Sub 3: GPS-stationary — EKF speed proxy; excluded if GPS stale.
        if (gps_available_ && (millis() - last_gps_time_ms_) < 5000)
        {
            const float speed = sqrt(velocity[0]*velocity[0] +
                                     velocity[1]*velocity[1] +
                                     velocity[2]*velocity[2]);
            const bool gps_stationary_pass = (speed < GPS_STATIONARY_SPEED_MS);
            if (gps_stationary_pass) {
                if (gps_stationary_count_ < LANDING_SLOW_COUNT_MAX) gps_stationary_count_++;
            } else {
                if (gps_stationary_count_ > 0) gps_stationary_count_--;
            }
            if (gps_stationary_count_ >= LANDING_SLOW_COUNT_HI) gps_stationary_flag = true;
            else if (gps_stationary_count_ == 0)                gps_stationary_flag = false;
        }

        // Sub 4: Accel-1g floor.
        const bool accel_1g_pass = (fabs(acc_mag - G_MS2) < ACCEL_1G_TOLERANCE_MS2);
        if (accel_1g_pass) {
            if (accel_1g_count_ < LANDING_SLOW_COUNT_MAX) accel_1g_count_++;
        } else {
            if (accel_1g_count_ > 0) accel_1g_count_--;
        }
        if (accel_1g_count_ >= LANDING_SLOW_COUNT_HI) accel_1g_flag = true;
        else if (accel_1g_count_ == 0)                accel_1g_flag = false;

        // Sub 5: Extended quiescence — deliberately NOT one of the votes
        // below; it is an independent path to landed that survives a dead
        // barometer (#824).  Both IMU gates are far tighter than the vote's,
        // and it needs ~30 s of net quiet rather than 4 s.
        //
        // Accumulates only after apogee.  A rocket on the pad is quiescent by
        // definition, and the rising-edge reset further down runs AFTER the
        // vote — so without this gate a full counter carried off the pad would
        // latch LANDED on the first post-apogee tick, before the reset could
        // clear it.  (The four voting detectors escape that ordering only
        // because baro_stable additionally requires max_altitude > 15 m.)
        //
        // When the barometer is healthy the altitude must also be unchanging.
        // The IMU gates alone cannot tell "sitting in a field" from "descending
        // smoothly", so where an altitude reference exists this insists on one;
        // it is dropped only when the baro is unusable, which is the situation
        // this detector exists for.  Note the check is Δpalt only, with no
        // band: that is what lets a rocket that landed hundreds of metres off
        // the pad reference — never able to satisfy baro_stable — still finish
        // its flight.
        const bool quiescent_alt_ok = (!baro_healthy ||
                                       landing_altitude_change < BARO_STABLE_DELTA_MAX);
        const bool quiescent_pass = (apogee_flag &&
                                     quiescent_alt_ok &&
                                     fabs(roll_rate) < QUIESCENT_GYRO_DPS &&
                                     fabs(acc_mag - G_MS2) < QUIESCENT_ACCEL_TOL_MS2);
        if (quiescent_pass) {
            if (quiescent_count_ < QUIESCENT_COUNT_MAX) quiescent_count_++;
        } else {
            if (quiescent_count_ > 0) quiescent_count_--;
        }
        if (quiescent_count_ >= QUIESCENT_COUNT_HI) quiescent_flag = true;
        else if (quiescent_count_ == 0)             quiescent_flag = false;
    }

    // --- Voting: impact alone fires; otherwise N-1 of N over slow flags.
    // Gated on apogee_flag — a static rocket on the pad trips gyro_quiet,
    // gps_stationary, and accel_1g (3 of 4 votes), which would otherwise
    // fire landed pre-flight (caught on bench 2026-05-18).
    if (!alt_landed_flag && apogee_flag)
    {
        if (impact_flag)
        {
            alt_landed_flag = true;
        }
        else if (quiescent_flag)
        {
            // Baro-independent path (#824).  Kept out of the vote so it still
            // works in exactly the cases the vote cannot: a dead barometer, or
            // a landing site far enough off the pad elevation to fall outside
            // the BARO_STABLE_PALT band.  Costs ~30 s of extra flight time in
            // those cases; the alternative was the 10-minute backstop.
            alt_landed_flag = true;
        }
        else
        {
            uint8_t available = 0;
            uint8_t passed = 0;

            available++; if (baro_stable_flag) passed++;
            available++; if (gyro_quiet_flag)  passed++;
            if (gps_available_ && (millis() - last_gps_time_ms_) < 5000)
            {
                available++; if (gps_stationary_flag) passed++;
            }
            available++; if (accel_1g_flag) passed++;

            // baro_stable is mandatory, not merely one vote among equals
            // (#824).  It is the only altitude-aware detector in the set:
            // gyro_quiet and accel_1g both pass under a canopy at terminal
            // velocity (an accelerometer reads 1 g in steady descent exactly
            // as it does sitting on the ground), so with GPS stale the old
            // 2-of-3 rule could be satisfied entirely by altitude-blind
            // evidence and cut the main deploy.  Requiring baro_stable means
            // every latch has at least one detector that actually knows the
            // rocket is near the ground.
            if (available >= 2 && passed >= 2 && passed >= (available - 1)
                && baro_stable_flag)
            {
                alt_landed_flag = true;
            }
        }
    }

    // ### GPS freshness tracking ###
    // Mark GPS available + stamp the last fix for the voting staleness gate.
    // max_gps_altitude_ is still tracked (cheap, kept for diagnostics) but the
    // GPS apogee test no longer uses it — it fires on Doppler velocity now
    // (#237/#242), which is real-time, unlike the ~4 s-lagging GPS position.
    if (new_gps && launch_flag)
    {
        gps_available_ = true;
        last_gps_time_ms_ = millis();
        max_gps_altitude_ = fmax(max_gps_altitude_, gps_altitude);
    }

    // ================================================================
    // ### Apogee Detection (gated on launch + burnout) ###
    // Four independent tests, each a leaky counter with hysteresis so a
    // single bad sample (EKF wobble, baro glitch, momentary attitude
    // excursion) doesn't latch a vote for the rest of the flight (#166).
    // N-1 of N voting on the sub-flags; master apogee_flag still latches
    // once true.
    // ================================================================
    if (launch_flag && burnout_detected)
    {
        // Stamp the first time burnout is seen — starts the baro settle window.
        if (!burnout_seen_)
        {
            burnout_seen_ = true;
            burnout_seen_ms_ = millis();
        }

        // --- Test 1: Velocity (EKF vertical velocity negative) ---
        const bool vel_pass = (position[2] > 15.0f && velocity[2] < 0.0f);
        if (vel_pass) {
            if (vel_apogee_count_ < APOGEE_COUNT_MAX) vel_apogee_count_++;
        } else {
            if (vel_apogee_count_ > 0) vel_apogee_count_--;
        }
        if (vel_apogee_count_ >= APOGEE_COUNT_HI)      vel_u_apogee_flag = true;
        else if (vel_apogee_count_ == 0)               vel_u_apogee_flag = false;

        // --- Test 2: Baro altitude (filtered altitude decreasing) ---
        // Uses KF-smoothed alt_est rather than raw pressure_altitude — boost
        // noise on the raw signal was previously tripping this test during
        // ascent (#142). Velocity gate (d_alt_est_ < 20 m/s) is preserved.
        // Ignored during the post-burnout settle window (see
        // BARO_BURNOUT_SETTLE_MS): the bay-pressure snap-back at thrust
        // tail-off reads as a >10 m altitude drop and otherwise casts a false
        // vote the moment this gate opens.
        const bool baro_settled = (millis() - burnout_seen_ms_ >= BARO_BURNOUT_SETTLE_MS);
        const bool baro_pass = (baro_settled &&
                                alt_est > 15.0f &&
                                alt_est < max_altitude - 5.0f &&
                                d_alt_est_ < 20.0f);
        if (baro_pass) {
            if (apogee_count < APOGEE_COUNT_MAX) apogee_count++;
        } else {
            if (apogee_count > 0) apogee_count--;
        }
        if (apogee_count >= APOGEE_COUNT_HI)            alt_apogee_flag = true;
        else if (apogee_count == 0)                     alt_apogee_flag = false;

        // --- Test 3: GPS Doppler vertical velocity (descending) ---
        // Was "GPS altitude below running max", but GPS *position* lags ~4 s
        // and dives during boost (#242) — firing this wildly off (III at
        // T+3.6 s, II at T+9.9 s vs true apogee #237).  GPS *velocity*
        // (Doppler) is real-time, so a sustained descent tracks true apogee.
        // Gated only on a fresh fix — NO gps-altitude gate, since that lag is
        // the bug.  Post-burnout only, so an ascent excursion can't fire it.
        if (new_gps && gps_available_)
        {
            const bool gps_pass = (gps_vel_u < -GPS_VEL_APOGEE_DESCENT_MPS);
            if (gps_pass) {
                if (gps_apogee_count_ < GPS_APOGEE_COUNT_MAX) gps_apogee_count_++;
            } else {
                if (gps_apogee_count_ > 0) gps_apogee_count_--;
            }
            if (gps_apogee_count_ >= GPS_APOGEE_COUNT_HI) gps_apogee_flag = true;
            else if (gps_apogee_count_ == 0)              gps_apogee_flag = false;
        }

        // --- Test 4: Pitch below horizontal ---
        // NED convention: +90° = nose up, 0° = horizontal, negative = past horizontal.
        // -5° threshold avoids false trigger from coast oscillation.
        const bool pitch_pass = (pitch_rad < -0.087f);
        if (pitch_pass) {
            if (pitch_apogee_count_ < APOGEE_COUNT_MAX) pitch_apogee_count_++;
        } else {
            if (pitch_apogee_count_ > 0) pitch_apogee_count_--;
        }
        if (pitch_apogee_count_ >= APOGEE_COUNT_HI)     pitch_apogee_flag = true;
        else if (pitch_apogee_count_ == 0)              pitch_apogee_flag = false;

        // --- Dynamic N-1 of N voting with health-aware quorum (#237/#257) ---
        // Only HEALTHY voters count toward `available`, so a positively-faulted
        // voter (dead baro, diverged EKF) no longer *raises* the bar via
        // `available - 1` — the healthy voters become sufficient.  The
        // `passed >= 2` floor is kept, so we never fire on fewer than two
        // concurring voters during ascent.  Velocity and pitch are both
        // EKF-derived, so an unhealthy EKF excludes BOTH (no double-counting a
        // single sick filter).  With < 2 healthy voters the vote cannot fire —
        // the Layer-2 baro backstop below covers that degraded case.
        if (!apogee_flag)
        {
            uint8_t available = 0;
            uint8_t passed = 0;

            // Velocity (EKF) — available only when the EKF is healthy
            if (ekf_healthy)
            {
                available++;
                if (vel_u_apogee_flag) passed++;
            }

            // Baro — excluded during mach lockout or when baro is unhealthy
            if (!baro_locked_out && baro_healthy)
            {
                available++;
                if (alt_apogee_flag) passed++;
            }

            // GPS (Doppler) — non-EKF voter, available on a FRESH fix (#262).
            // Re-enabled after the GNSS dynamic-model change fixed the boost
            // corruption that originally forced its exclusion (#237/#242): the
            // detector is post-burnout only, velocity-based (no laggy-altitude
            // gate), and requires GPS_APOGEE_COUNT_HI sustained descending
            // samples, so a transient can't fire it.  Being non-EKF, it restores
            // voter diversity exactly when the EKF voters and/or baro are at
            // risk — baro mach-locked-out (the #262 common-mode case) or a
            // degraded EKF.  Gated on GPS_APOGEE_FRESH_MS freshness (500 ms
            // since #262 — this comment used to claim 5 s).
            const bool gps_fresh = gps_available_ &&
                                   (millis() - last_gps_time_ms_) < GPS_APOGEE_FRESH_MS;
            if (gps_fresh)
            {
                available++;
                if (gps_apogee_flag) passed++;
            }

            // Pitch (EKF) — available only when the EKF is healthy
            if (ekf_healthy)
            {
                available++;
                if (pitch_apogee_flag) passed++;
            }

            // Quorum: a floor of 2 concurring voters, then N-1 of N — but relax
            // to N-2 once there are > 3 voters (#262).  Adding GPS as a 4th voter
            // under a strict N-1 would turn the nominal 2-of-3 into 3-of-4 and
            // *delay* apogee (observed +0.56 s in flight replay); N-2-when-N>3
            // keeps 4-voter sensitivity at 2-of-4 (== the old 2-of-3) while the
            // 2-concurring floor still bars any single-sensor trigger.  Three or
            // fewer voters (incl. the mach-lockout case: vel+gps+pitch) stay at
            // N-1 → 2-of-3, with GPS supplying the non-EKF diversity.
            const uint8_t need = (available > 3) ? (uint8_t)(available - 2)
                                                 : (uint8_t)(available - 1);
            if (available >= 2 && passed >= 2 && passed >= need)
            {
                apogee_flag = true;
            }
        }
    } // end burnout gate

    // --- Layer 2: baro-only descent backstop (#257; decoupled from burnout, #556) ---
    // Last-resort net for the degraded case where the primary vote cannot
    // reach quorum (e.g. an unhealthy EKF leaves < 2 healthy voters).  A
    // healthy baro that has fallen >= APOGEE_BACKSTOP_DROP_M below the running
    // peak while still descending is unambiguously past apogee — a condition
    // unreachable during a normal boost, so there is no false-positive
    // exposure.  alt_est / d_alt_est_ / max_altitude all come from the
    // baro-only KF and are updated every call, so this works with a dead EKF
    // *and* a dead IMU.
    //
    // #556: this block is deliberately OUTSIDE the burnout_detected gate above.
    // burnout_detected only ever latches from a fresh-IMU accel sample
    // (main.cpp burnoutDetectStep, gated on ism6_fresh).  If the IMU dies
    // during boost before burnout latches, the gated vote never runs — and
    // while this backstop was still nested inside that gate, apogee was never
    // declared, so drogue/main never fired and the vehicle came in ballistic.
    //
    // #556 (bench-found): the backstop must ALSO ignore baro_locked_out (the
    // transonic mach lockout).  That lockout is driven by the EKF velocity,
    // which a dead IMU freezes/corrupts — so it latches true and never releases
    // (it clears only when speed < BARO_MACH_LOCKOUT_OFF), silently vetoing the
    // very backstop meant to survive a dead IMU.  The vote's baro test (above)
    // still honours the lockout; the backstop does not need it — the
    // 30 m-below-peak + descending + APOGEE_COUNT_HI debounce is self-gating
    // against any ascent / boost / transonic transient (a sustained 30 m drop
    // below the running max cannot occur while climbing), so dropping the
    // lockout gate here removes no real protection and restores the dead-IMU
    // recovery path.  Latches apogee_flag exactly like the vote, enabling the
    // full drogue + main sequence; apogee_backstop_flag records that Layer 2
    // (not the vote) was the path that fired, for post-flight diagnostics.
    if (launch_flag &&
        baro_healthy &&
        alt_est > 15.0f &&
        alt_est <= max_altitude - APOGEE_BACKSTOP_DROP_M &&
        d_alt_est_ < 0.0f)
    {
        if (backstop_descent_count_ < APOGEE_COUNT_MAX) backstop_descent_count_++;
    }
    else if (backstop_descent_count_ > 0)
    {
        backstop_descent_count_--;
    }
    if (backstop_descent_count_ >= APOGEE_COUNT_HI && !apogee_flag)
    {
        apogee_flag = true;
        apogee_backstop_flag = true;
    }

    // ### Apogee rising edge: reset landing sub-flag counters (#192) ###
    // The 1 Hz sub-detectors above accumulate evidence regardless of
    // flight state, so a rocket flying straight pre-apogee (low roll
    // rate, ~0 m/s in EKF frame, ~1g during coast) latches gyro_quiet /
    // gps_stationary / accel_1g before apogee.  The apogee gate on the
    // master vote prevents a false LANDED, but the latched sub-flags
    // would carry pre-apogee history into the post-apogee vote and erode
    // the 3-of-4 margin.  Zero counters + flags at the transition so
    // post-apogee voting reflects only post-apogee evidence.
    if (apogee_flag && !apogee_was_set)
    {
        baro_stable_count_    = 0;
        gyro_quiet_count_     = 0;
        gps_stationary_count_ = 0;
        accel_1g_count_       = 0;
        quiescent_count_      = 0;
        baro_stable_flag    = false;
        gyro_quiet_flag     = false;
        gps_stationary_flag = false;
        accel_1g_flag       = false;
        quiescent_flag      = false;
    }

    // ### Pre-Apogee Max Speed ###
    float speed = sqrt(velocity[0]*velocity[0]+
                       velocity[1]*velocity[1]+
                       velocity[2]*velocity[2]);
    if (!apogee_flag && speed > max_speed)
    {
        max_speed = speed;
    }

}

// ======================================================================
// 1D Constant-Velocity Kalman Filter
//
// State: x = [ z, vz ]^T
// Model: z_k+1 = z_k + vz_k*dt + w_z
//        vz_k+1 = vz_k + w_v
// Process noise: Q = [[Qz*dt, 0],[0, Qv*dt]]
// Measurement: y = z + v,  R = R_
//
// The predict step runs every call to propagate the state forward.
// The update step only runs when new_measurement is true, so that
// stale (repeated) baro readings are not double-counted.
//
// Returns filtered altitude; updates d_alt_est_ (altitude rate).
// ======================================================================
float TR_KinematicChecks::updateAltKF(float z_meas, bool new_measurement)
{
    uint32_t now = millis();
    float dt = 0.02f; // default 20 ms if first call
    if (kf_init_) {
        uint32_t dms = now - last_ms_;
        // clamp dt between 0.5 ms and 200 ms to avoid numeric extremes
        dt = constrain(dms * 0.001f, 0.0005f, 0.200f);
    }
    last_ms_ = now;

    if (!kf_init_)
    {
        // Initialize state to first measurement
        alt_est = z_meas;
        d_alt_est_ = 0.0f;
        // Covariance already set in ctor
        kf_init_ = true;
        return alt_est;
    }

    // ---- Predict ----
    // x = F x
    // F = [[1, dt],
    //      [0, 1]]
    float z_pred  = alt_est + d_alt_est_ * dt;
    float vz_pred = d_alt_est_;

    // P = F P F^T + Q
    const float F00 = 1.0f, F01 = dt;
    const float F10 = 0.0f, F11 = 1.0f;

    float P00p = F00*P00_ + F01*P10_;
    float P01p = F00*P01_ + F01*P11_;
    float P10p = F10*P00_ + F11*P10_;
    float P11p = F10*P01_ + F11*P11_;

    float P00pp = P00p*F00 + P01p*F01;
    float P01pp = P00p*F10 + P01p*F11;
    float P10pp = P10p*F00 + P11p*F01;
    float P11pp = P10p*F10 + P11p*F11;

    // Q (scaled by dt): allow slow drift in z and change in vz
    float Q00 = Qz_ * dt;
    float Q11 = Qv_ * dt;

    P00pp += Q00;
    P11pp += Q11;

    if (new_measurement)
    {
        // ---- Update (measure z) ----
        // y = H x + v, H = [1 0]
        // S = H P H^T + R = P00pp + R
        float S  = P00pp + R_;
        float K0 = P00pp / S;   // gain for z
        float K1 = P10pp / S;   // gain for vz

        float innov = z_meas - z_pred;

        // x = x + K * innov
        alt_est    = z_pred  + K0 * innov;
        d_alt_est_ = vz_pred + K1 * innov;

        // P = (I - K H) P
        // (I - K H) = [[1-K0,   0],
        //              [-K1 ,   1]]
        float P00n = (1.0f - K0) * P00pp;
        float P01n = (1.0f - K0) * P01pp;
        float P10n = P10pp - K1 * P00pp;
        float P11n = P11pp - K1 * P01pp;

        P00_ = P00n;
        P11_ = P11n;
        // Enforce covariance symmetry to prevent float drift over many iterations
        P01_ = 0.5f * (P01n + P10n);
        P10_ = P01_;
    }
    else
    {
        // No new measurement — keep predicted state and covariance
        alt_est    = z_pred;
        d_alt_est_ = vz_pred;

        P00_ = P00pp;
        P11_ = P11pp;
        P01_ = 0.5f * (P01pp + P10pp);
        P10_ = P01_;
    }

    return alt_est;
}

