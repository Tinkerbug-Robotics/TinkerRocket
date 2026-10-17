//
//  TR_Sensor_Collector_Sim.cpp
//  TinkerRocket FlightComputer
//
//  Drop-in wrapper around SensorCollector.  When sim is active, consumes
//  real sensor data (maintaining hardware timing) and replaces output
//  values with simulated 1D flight data.
//
//  Physics model adapted from OutComputer/FlightSimulator.h.
//

#include "TR_Sensor_Collector_Sim.h"
#include "SimPadAlign.h"
#include "SimSensorModel.h"
#include <TR_GeoMag.h>
#include <cmath>
#include <esp_timer.h>
// SensorCollector.h no longer transitively pulls in compat.h, but this
// file still uses the Arduino-shim millis()/micros() heavily.  Pulled
// in explicitly here until item #5 of issue #21 slims the sim too.
#include <compat.h>
// The `Serial` object this file prints through is the SparkFun shim in the
// u-blox driver (sfe_bus.h). It used to arrive transitively through
// SensorCollector.h; on boards where the collector selects the LC86 GNSS
// driver instead (TR_GNSS_DRIVER_LC86) that path is gone, so include it
// explicitly. The shim is compiled unconditionally, so this is
// declaration-only on every board.
#include <sfe_bus.h>

// ============================================================================
// Construction / Delegation
// ============================================================================

SensorCollectorSim::SensorCollectorSim(SensorCollector& real)
    : real_(real) {}

void SensorCollectorSim::begin(uint8_t imu_execution_core)
{
    real_.begin(imu_execution_core);
}

// ============================================================================
// Debug passthrough
// ============================================================================

void SensorCollectorSim::getISM6HG256DebugSnapshot(ISM6HG256DebugSnapshot& snapshot_out) const
{
    real_.getISM6HG256DebugSnapshot(snapshot_out);
}

void SensorCollectorSim::getMMC5983MADebugSnapshot(MMC5983MADebugSnapshot& snapshot_out) const
{
    real_.getMMC5983MADebugSnapshot(snapshot_out);
}

void SensorCollectorSim::getPollTimingSnapshot(PollTimingSnapshot& snapshot_out) const
{
    real_.getPollTimingSnapshot(snapshot_out);
}

void SensorCollectorSim::resetPollTimingSnapshot()
{
    real_.resetPollTimingSnapshot();
}

// ============================================================================
// Calibration (no-op — sim data is synthetic, no real bias to correct)
// ============================================================================

bool SensorCollectorSim::startCalibration(float rotation_z_deg)
{
    if (isSimActive())
    {
        // Nothing to measure and nothing changes; the caller re-publishes
        // the stored calibration, which is what the rocket keeps using.
        Serial.println("[SIM] startCalibration refused — no-op in sim mode, stored calibration kept");
        return false;
    }
    // Sim not active — delegate to real hardware
    return real_.startCalibration(rotation_z_deg);
}

SensorCalPoll SensorCollectorSim::pollCalibration()
{
    const SensorCalPoll v = real_.pollCalibration();
    if (v == SensorCalPoll::Committed)
    {
        // Copy results so FlightComputer can read them from the wrapper.  A
        // rejected run leaves real_'s values as they were: nothing to copy.
        hg_bias_x       = real_.hg_bias_x;
        hg_bias_y       = real_.hg_bias_y;
        hg_bias_z       = real_.hg_bias_z;
        cal_gravity_mag = real_.cal_gravity_mag;
    }
    return v;
}

void SensorCollectorSim::cancelCalibration()
{
    real_.cancelCalibration();
}

bool SensorCollectorSim::setIIS2MDCHardIronOffset(int16_t cx, int16_t cy, int16_t cz)
{
    if (isSimActive())
    {
        // No physical IIS2MDC interaction in sim mode; cal flow is gated on
        // READY and we don't enter sim from there, so this branch is mostly
        // defensive.
        return false;
    }
    return real_.setIIS2MDCHardIronOffset(cx, cy, cz);
}

void SensorCollectorSim::getIIS2MDCDebugSnapshot(IIS2MDCDebugSnapshot& snapshot_out) const
{
    real_.getIIS2MDCDebugSnapshot(snapshot_out);
}

// ============================================================================
// Sim Control
// ============================================================================

void SensorCollectorSim::configureSim(const SimConfigData& cfg)
{
    cfg_ = cfg;
    // mass_kg outside any plausible vehicle (>50 kg / <10 g) almost certainly
    // means a raw cmd-5 BLE payload (grams) reached us without the OC's /1000
    // conversion — the sim will sit on the pad (or go ballistic) silently.
    //
    // #1154 item 11: this used to log and then configure anyway. It is a
    // REFUSAL now, because the thrust integrator divides by this number
    // unguarded (`thrust_n / mass_kg` below) and a zero or denormal mass
    // poisons every synthetic sensor with NaN — attitude, altitude, rates, all
    // of it — which reaches the flight loop looking like data. A sim that
    // refuses to start says what is wrong; one that starts with NaN does not.
    if (!(cfg_.mass_kg >= 0.010f) || cfg_.mass_kg > 50.0f)
    {
        configured_ = false;
        ESP_LOGE("SIM", "REFUSING sim config: implausible mass_kg=%.4f — grams "
                 "sent where kg expected? (BLE cmd-5 mass is grams; OC "
                 "converts). Nothing will start until a valid config arrives.",
                 (double)cfg_.mass_kg);
        return;
    }
    configured_ = true;
    ESP_LOGI("SIM", "Config: mass=%.3fkg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
             (double)cfg_.mass_kg, (double)cfg_.thrust_n,
             (double)cfg_.burn_time_s, (double)cfg_.descent_rate_mps);
}

void SensorCollectorSim::configureSimRotation(float ism6_rot_z_deg)
{
    float rad = ism6_rot_z_deg * (M_PI / 180.0f);
    ism6_inv_c_ = cosf(rad);
    ism6_inv_s_ = sinf(rad);
    ESP_LOGI("SIM", "ISM6 rotation: %.1f deg (inv_c=%.4f, inv_s=%.4f)",
             (double)ism6_rot_z_deg, (double)ism6_inv_c_, (double)ism6_inv_s_);
}

void SensorCollectorSim::configureSimIis2mdcRotation(float rot_z_deg)
{
    const float rad = rot_z_deg * (float)M_PI / 180.0f;
    iis_inv_c_ = cosf(rad);
    iis_inv_s_ = sinf(rad);
    ESP_LOGI("SIM", "IIS2MDC rotation: %.1f deg", (double)rot_z_deg);
}

void SensorCollectorSim::configureSimBoardToRocket(const float R[9])
{
    // Store the transpose (rocket→board); flag identity to skip the
    // multiply in the hot encode path.
    bool identity = true;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            const float v = R[c * 3 + r];  // transpose
            b2r_inv_[r * 3 + c] = v;
            const float ref = (r == c) ? 1.0f : 0.0f;
            if (fabsf(v - ref) > 1e-6f) identity = false;
        }
    }
    b2r_identity_ = identity;
}

void SensorCollectorSim::simPadReference(float up_out[3], float mag_out[3]) const
{
    sim_pad_align::simPadReference(LAUNCH_ANGLE_RAD, B_NORTH, B_EAST, B_DOWN,
                                   up_out, mag_out);
}

void SensorCollectorSim::configureSimPadAlignment(const float up_rocket[3],
                                                  const float mag_rocket[3])
{
    float up_sim[3], mag_sim[3];
    simPadReference(up_sim, mag_sim);

    bool used_mag = false;
    const bool ok = sim_pad_align::solvePadAlignment(
        up_sim, mag_sim, up_rocket, mag_rocket, pad_align_, &used_mag);

    if (!ok)
    {
        pad_align_identity_ = true;
        ESP_LOGW("SIM", "pad align: measured gravity unusable — sim starts "
                        "UNALIGNED (expect a gyro-bias excursion, #508)");
        return;
    }

    pad_align_identity_ = false;
    ESP_LOGI("SIM", "pad align: %.1f deg (%s) — sim frame carried onto the "
                    "board's resting attitude; no handover step",
             (double)padAlignmentAngleDeg(),
             used_mag ? "gravity+field" : "gravity only");
}

float SensorCollectorSim::padAlignmentAngleDeg() const
{
    if (pad_align_identity_) return 0.0f;
    return sim_pad_align::rotationAngleDeg(pad_align_);
}

void SensorCollectorSim::startSim(float ground_pressure_pa)
{
    if (replay_on_)
    {
        // The rig replay owns the sensors until a Stop: a physics flight over
        // it would pair the rig's sky with a vertical trajectory.
        Serial.println("[SIM] startSim refused -- the rig replay is running (Stop first)");
        return;
    }
    if (!configured_)
    {
        Serial.println("[SIM] ERROR: startSim called but not configured (missing config cmd?)");
        return;
    }

    altitude_m_      = 0.0f;
    velocity_mps_    = 0.0f;
    accel_mps2_      = 0.0f;
    sim_time_s_      = 0.0f;
    pitch_rad_       = LAUNCH_ANGLE_RAD;
    pitch_rate_rps_  = 0.0f;

    ground_pressure_pa_ = (ground_pressure_pa > 50000.0f)
        ? ground_pressure_pa : 101325.0f;

    phase_ = SIM_PRELAUNCH;
    phase_start_ms_ = millis();
    last_step_us_   = micros();
    gnss_timer_us_  = micros();
    last_gnss_real_us_ = micros();

    Serial.println("[SIM] Started");
}

void SensorCollectorSim::stopSim()
{
    if (replay_on_)
        ESP_LOGW("SIM", "RIG REPLAY stopped: the real sensors from here");
    replay_on_ = false;
    phase_ = SIM_IDLE;
    // Drop the pad alignment: it belongs to the attitude the board was resting
    // at for THAT run, and a stale one would skew the next sim start.
    pad_align_identity_ = true;
    for (int i = 0; i < 9; ++i) pad_align_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    Serial.println("[SIM] Stopped");
}

bool SensorCollectorSim::isSimActive() const
{
    return phase_ != SIM_IDLE;
}

// ============================================================================
// The rig replay (SimReplay.h, #1600)
// ============================================================================

#if defined(TR_SIM_REPLAY) && TR_SIM_REPLAY
// The flight the build carries (CMakeLists: -DTR_SIM_REPLAY_BLOB=FILE)
extern const uint8_t sim_replay_bin_start[] asm("_binary_sim_replay_bin_start");
extern const uint8_t sim_replay_bin_end[] asm("_binary_sim_replay_bin_end");
#endif

namespace
{
const char* clockName(uint8_t st)
{
    switch (st)
    {
    case sim_replay::ReplayClock::kWaiting:   return "waiting for the rig's fixes";
    case sim_replay::ReplayClock::kFollowing: return "on the receiver's clock";
    case sim_replay::ReplayClock::kFrozen:    return "frozen for the flight";
    case sim_replay::ReplayClock::kMissed:    return "MISSED (the first fix came after the pad)";
    default:                                  return "?";
    }
}
}  // namespace

bool SensorCollectorSim::startReplay()
{
#if defined(TR_SIM_REPLAY) && TR_SIM_REPLAY
    const size_t len = (size_t)(sim_replay_bin_end - sim_replay_bin_start);
    if (!replay_.load(sim_replay_bin_start, len))
    {
        ESP_LOGE("SIM", "RIG REPLAY: the carried flight does not load (%s; %u bytes)",
                 replay_.why(), (unsigned)len);
        return false;
    }
    const sim_replay::Header& h = replay_.header();
    // The field the flight computer will model at this pad and date (its
    // declination and magnitude gate use the same call): the replay's
    // magnetometer then agrees with the heading the flight's attitude has.
    static constexpr double kDeg2Rad = M_PI / 180.0;
    TR_GeoMag::fieldNED_uT(h.lat_deg * kDeg2Rad, h.lon_deg * kDeg2Rad, h.alt_m,
                           (double)h.decimal_year, replay_b_ned_);
    portENTER_CRITICAL(&replay_mux_);
    replay_clock_.reset(h);
    portEXIT_CRITICAL(&replay_mux_);
    // From boot: nothing to hand over from, so no pad alignment (#508) -- the
    // attitude is the flight's own, which the rig's GNSS has to agree with.
    pad_align_identity_ = true;
    for (int i = 0; i < 9; ++i) pad_align_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    replay_last_have_t_ = false;
    replay_on_ = true;
    phase_ = SIM_PRELAUNCH;   // active: what the dry-fire gate and the log key on
    phase_start_ms_ = millis();
    const double week = std::floor(h.ignition_gps_s / 604800.0);
    ESP_LOGW("SIM", "RIG REPLAY: '%s', %u samples at %u Hz, t %+.2f .. %+.2f s; ignition at GPS "
             "week %.0f TOW %.3f; the rig's file spans t %+.1f .. %+.1f s; pad %.7f, %.7f, %.1f m; "
             "field N %.1f E %.1f D %.1f uT (%.4f)",
             h.name, (unsigned)h.samples, (unsigned)h.rate_hz, h.t0_s, replay_.tLast(), week,
             h.ignition_gps_s - week * 604800.0, h.scen_start_s, h.scen_end_s, h.lat_deg, h.lon_deg,
             h.alt_m, (double)replay_b_ned_[0], (double)replay_b_ned_[1], (double)replay_b_ned_[2],
             (double)h.decimal_year);
    return true;
#else
    return false;
#endif
}

void SensorCollectorSim::replayStatus(ReplayStatus& out) const
{
    portENTER_CRITICAL(&replay_mux_);
    out.clock_state = replay_clock_.state();
    out.fixes = replay_clock_.fixes();
    out.outside = replay_clock_.outside();
    out.restarts = replay_clock_.restarts();
    out.last_fix_t_s = (float)replay_clock_.lastFixT();
    portEXIT_CRITICAL(&replay_mux_);
    out.t_s = replay_last_t_;
    out.have_t = replay_last_have_t_;
    out.phase = replay_last_phase_;
}

void SensorCollectorSim::replaySampleAt(uint32_t time_us, sim_replay::Sample& s, bool is_imu)
{
    // The sample's instant on the 64-bit clock its 32-bit stamp was cut from
    // (micros()); it is recent, either side of now.
    const int64_t now = esp_timer_get_time();
    const int64_t mcu = now - (int64_t)(int32_t)((uint32_t)now - time_us);
    double t = 0.0;
    portENTER_CRITICAL(&replay_mux_);
    const bool have = replay_clock_.tAt(mcu, t);
    portEXIT_CRITICAL(&replay_mux_);
    if (have)
        replay_.at(t, s);
    else
        replay_.padAt((double)mcu * 1.0e-6, s);
    if (is_imu)
    {
        replay_last_t_ = (float)t;
        replay_last_have_t_ = have;
        replay_last_phase_ = s.phase;
    }
}

void SensorCollectorSim::replayField(const sim_replay::Sample& s, float b[3]) const
{
    // R(q)^T B: the NED field in the rocket frame (q turns FLU into NED)
    const float w = s.q[0], x = s.q[1], y = s.q[2], z = s.q[3];
    const float* n = replay_b_ned_;
    b[0] = (1.0f - 2.0f * (y * y + z * z)) * n[0] + 2.0f * (x * y + w * z) * n[1] + 2.0f * (x * z - w * y) * n[2];
    b[1] = 2.0f * (x * y - w * z) * n[0] + (1.0f - 2.0f * (x * x + z * z)) * n[1] + 2.0f * (y * z + w * x) * n[2];
    b[2] = 2.0f * (x * z + w * y) * n[0] + 2.0f * (y * z - w * x) * n[1] + (1.0f - 2.0f * (x * x + y * y)) * n[2];
}

void SensorCollectorSim::replayNoteFix(const gnss_raw::TimedFix& f)
{
    portENTER_CRITICAL(&replay_mux_);
    const uint8_t before = replay_clock_.state();
    replay_clock_.onFix(f.gps_s, f.mcu_us);
    const uint8_t after = replay_clock_.state();
    const double t = replay_clock_.lastFixT();
    portEXIT_CRITICAL(&replay_mux_);
    if (after != before)
        ESP_LOGW("SIM", "RIG REPLAY clock: %s (a fix at t %+.2f s)", clockName(after), t);
}

// ============================================================================
// Data Getters
// ============================================================================

bool SensorCollectorSim::getISM6HG256Data(ISM6HG256Data& data_out)
{
    bool have_data = real_.getISM6HG256Data(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    if (replay_on_)
    {
        // The rig replay: the flight at this sample's own instant, which the
        // receiver's clock places (#1600).
        sim_replay::Sample s;
        replaySampleAt(data_out.time_us, s, true);
        static constexpr float kRad2Deg = 180.0f / (float)M_PI;
        packISM6(data_out.time_us, s.f[0], s.f[1], s.f[2],
                 s.w[0] * kRad2Deg, s.w[1] * kRad2Deg, s.w[2] * kRad2Deg, data_out);
        return true;
    }

    // Real IMU data arrived at the hardware rate — step physics and replace
    // VALUES.  The sample keeps its own time_us: real_ stamped it in the core-0
    // poll task when the SPI read happened, and that is the clock a real
    // flight's log carries.  Restamping here with the loop's micros() (what
    // this did until #910) made every flight-loop stall look like a
    // simultaneous multi-sensor outage — the loop drains the IMU queue late,
    // so the backlog all took the drain time — and made a sim's sample rate
    // meaningless as a benchmark against a real flight.  now_us is still the
    // right clock for stepping the physics, which is per drain, not per sample.
    const uint32_t now_us = micros();
    const uint32_t sample_us = data_out.time_us;
    const float dt = (float)(now_us - last_step_us_) * 1.0e-6f;
    last_step_us_ = now_us;
    const float clamped_dt = (dt > 0.1f) ? 0.1f : dt;
    sim_time_s_ += clamped_dt;

    stepPhysics(clamped_dt);

#if defined(TR_SIM_DEAD_IMU) && TR_SIM_DEAD_IMU
    // #556 bench validation: emulate an IMU that dies at burnout.  The flight
    // physics are stepped above, so baro/GNSS keep flying the FULL trajectory
    // (ascent -> apogee -> descent); but from the boost->coast boundary on we
    // stop DELIVERING fresh IMU samples.  The main loop derives IMU freshness
    // from this getter returning true, so ism6 goes stale, burnout_detected
    // never latches, and the burnout-gated apogee vote never runs — leaving the
    // baro-only Layer-2 backstop as the ONLY path to apogee.  That is exactly
    // the dead-IMU condition #556 must survive.  The IMU stays live through
    // PRELAUNCH/POWERED so launch still latches normally (launch detection
    // requires acc_mag > 20).  Compile-guarded AND sim-only: it can never
    // affect a real flight or a normal sim run.
    if (phase_ >= SIM_COASTING)
    {
        static bool announced = false;
        if (!announced) { announced = true; Serial.println("[SIM] DEAD-IMU: withholding IMU from coast (#556)"); }
        return false;
    }
#endif

    encodeISM6(sample_us, data_out);
    return true;
}

bool SensorCollectorSim::getBMP585Data(BMP585Data& data_out)
{
    bool have_data = real_.getBMP585Data(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    if (replay_on_)
    {
        sim_replay::Sample s;
        replaySampleAt(data_out.time_us, s, false);
        const uint32_t t_us = data_out.time_us;
        memset(&data_out, 0, sizeof(data_out));
        data_out.time_us  = t_us;
        data_out.press_q6 = (uint32_t)lroundf(s.p_pa * 64.0f);
        data_out.temp_q16 = (int32_t)(20.0f * 65536.0f);
        return true;
    }

    // Replace with simulated pressure/temperature
    encodeBMP585(data_out.time_us, data_out);   // value swap only; the stamp is the sensor's (#910)
    return true;
}

bool SensorCollectorSim::getMMC5983MAData(MMC5983MAData& data_out)
{
    bool have_data = real_.getMMC5983MAData(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    if (replay_on_)
    {
        sim_replay::Sample s;
        replaySampleAt(data_out.time_us, s, false);
        float b[3];
        replayField(s, b);
        packMMC5983MA(data_out.time_us, b[0], b[1], b[2], data_out);
        return true;
    }

    // Replace with simulated magnetometer
    encodeMMC5983MA(data_out.time_us, data_out);   // value swap only; the stamp is the sensor's (#910)
    return true;
}

bool SensorCollectorSim::getIIS2MDCData(IIS2MDCData& data_out)
{
    bool have_data = real_.getIIS2MDCData(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    if (replay_on_)
    {
        sim_replay::Sample s;
        replaySampleAt(data_out.time_us, s, false);
        float b[3];
        replayField(s, b);
        packIIS2MDC(data_out.time_us, b[0], b[1], b[2], data_out);
        return true;
    }

    // Replace with simulated magnetometer (#463). This path used to pass the
    // REAL bench field through during a sim: on IIS2MDC boards (MMC not
    // populated) the EKF then fused the bench's fixed heading against the
    // sim's flying attitude every sample — a sustained innovation the
    // heading-axis gyro bias absorbed (~±20–200 dps at sim start depending
    // on bench orientation, decaying all flight).
    encodeIIS2MDC(data_out.time_us, data_out);   // value swap only; the stamp is the sensor's (#910)
    return true;
}

bool SensorCollectorSim::getGNSSData(GNSSData& data_out)
{
    bool have_data = real_.getGNSSData(data_out);

    // the rig replay: the receiver's own fixes, from the flight's sky
    if (phase_ == SIM_IDLE || replay_on_) return have_data;

    if (have_data)
    {
        // Real GNSS data arrived — consume it and replace with simulated values,
        // keeping the receiver's own stamp (#910, as for the IMU above).
        last_gnss_real_us_ = micros();
        encodeGNSS(data_out.time_us, data_out);
        return true;
    }

    // Fallback: if no real GNSS data for >100ms (indoor testing), generate
    // simulated GNSS on a timer at 25 Hz so the state machine can see a
    // GPS lock and transition READY → PRELAUNCH.
    const uint32_t now_us = micros();
    if ((now_us - last_gnss_real_us_) > GNSS_FALLBACK_TIMEOUT_US &&
        (now_us - gnss_timer_us_) >= GNSS_FALLBACK_INTERVAL_US)
    {
        gnss_timer_us_ = now_us;
        encodeGNSS(now_us, data_out);
        return true;
    }

    return false;
}

bool SensorCollectorSim::getGNSSSatData(GNSSSatData& data_out)
{
    const bool have_data = real_.getGNSSSatData(data_out);
    if (phase_ == SIM_IDLE || replay_on_) return have_data;
    return false;   // consumed, not forwarded — see the header
}

bool SensorCollectorSim::getGNSSRtcmFrame(uint8_t* out, size_t cap, size_t& len,
                                          uint32_t& time_us)
{
    const bool have_frame = real_.getGNSSRtcmFrame(out, cap, len, time_us);
    if (phase_ == SIM_IDLE || replay_on_) return have_frame;
    return false;   // consumed, not forwarded — see the header
}

bool SensorCollectorSim::getGNSSRawEpoch(gnss_raw::TimedRawEpoch& out)
{
    const bool have = real_.getGNSSRawEpoch(out);
    if (phase_ == SIM_IDLE || replay_on_) return have;
    return false;   // consumed, not forwarded — see the header
}

bool SensorCollectorSim::getGNSSFix(gnss_raw::TimedFix& out)
{
    const bool have = real_.getGNSSFix(out);
    if (have && replay_on_) replayNoteFix(out);   // its GPS time sets the replay's clock
    if (phase_ == SIM_IDLE || replay_on_) return have;
    return false;   // consumed, not forwarded — see the header
}

bool SensorCollectorSim::getGNSSEphemeris(uint8_t* out, size_t cap, size_t& len)
{
    const bool have = real_.getGNSSEphemeris(out, cap, len);
    if (phase_ == SIM_IDLE || replay_on_) return have;
    return false;   // consumed, not forwarded — see the header
}

#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
bool SensorCollectorSim::getGNSSBroadcastIono(int8_t raw[8], uint16_t& gps_day) const
{
    if (phase_ == SIM_IDLE || replay_on_) return real_.getGNSSBroadcastIono(raw, gps_day);
    return false;   // the real sky's: not while a sim runs (kept, not consumed) — see the header
}
#endif

// ============================================================================
// Physics
// ============================================================================

void SensorCollectorSim::stepPhysics(float dt)
{
    const uint32_t elapsed_ms = millis() - phase_start_ms_;

    switch (phase_)
    {
    case SIM_PRELAUNCH:
        stepPrelaunch(elapsed_ms);
        break;
    case SIM_POWERED:
        stepPowered(dt);
        break;
    case SIM_COASTING:
        stepCoasting(dt);
        break;
    case SIM_DESCENT:
        stepDescent(dt);
        break;
    case SIM_LANDED:
    {
        velocity_mps_   = 0.0f;
        accel_mps2_     = 0.0f;
        pitch_rad_      = 0.0f;
        pitch_rate_rps_ = 0.0f;
        // Hold until the FC's own state machine reaches LANDED (#971).
        //
        // This used to be a fixed 9000 ms, chosen from a comment that budgeted
        // "5 consecutive 1-second checks (5s) plus a 2-second state-machine
        // debounce = 7s minimum".  That budget was wrong in a way that only
        // shows on a sim: a sim never produces an impact (accel is a flat 1 g,
        // never the 15 g LANDING_IMPACT_G spike), so alt_landed_flag can only
        // come from the SLOW vote, whose sub-flags run a leaky counter at 1 Hz.
        // Measured on flight_20260827_122854: the flag took 7.0 s, and the FC
        // then wants `> 2000` ms MORE of it — i.e. >9.0 s against a 9.0 s hold.
        // Every sim flight missed LANDED by a margin of zero, leaving the FC in
        // INFLIGHT with the log open until the 10-minute backstop.
        //
        // Asking the FC directly means this cannot drift out of sync again if
        // the detector's timing changes.  The timeout is only a backstop so a
        // detector that never fires cannot hang the sim forever.
        const sim_landed::Exit decision =
            sim_landed::decide(fc_rocket_state_, (uint8_t)LANDED, elapsed_ms);
        if (decision == sim_landed::Exit::FcLanded)
        {
            phase_ = SIM_IDLE;
            ESP_LOGI("SIM", "Complete (FC reached LANDED after %.1f s hold)",
                     (double)(elapsed_ms / 1000.0f));
        }
        else if (decision == sim_landed::Exit::GaveUp)
        {
            phase_ = SIM_IDLE;
            ESP_LOGW("SIM", "Complete (gave up after %.1f s — FC never left "
                     "state %u; landing detection did not fire)",
                     (double)(elapsed_ms / 1000.0f), (unsigned)fc_rocket_state_);
        }
        break;
    }
    default:
        break;
    }
}

void SensorCollectorSim::stepPrelaunch(uint32_t elapsed_ms)
{
    altitude_m_     = 0.0f;
    velocity_mps_   = 0.0f;
    accel_mps2_     = 0.0f;
    pitch_rad_      = LAUNCH_ANGLE_RAD;
    pitch_rate_rps_ = 0.0f;

    if (elapsed_ms >= (uint32_t)PRELAUNCH_DURATION_MS)
    {
        phase_ = SIM_POWERED;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: POWERED");
    }
}

void SensorCollectorSim::stepPowered(float dt)
{
    const float thrust_accel = cfg_.thrust_n / cfg_.mass_kg;
    const float drag = dragAccel(velocity_mps_, altitude_m_);
    const float net_accel = thrust_accel - GRAVITY - drag;

    velocity_mps_ += net_accel * dt;
    altitude_m_   += velocity_mps_ * dt;
    if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

    accel_mps2_ = net_accel;

    const float burn_elapsed = (float)(millis() - phase_start_ms_) * 0.001f;
    if (burn_elapsed >= cfg_.burn_time_s)
    {
        phase_ = SIM_COASTING;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: COASTING");
    }
}

void SensorCollectorSim::stepCoasting(float dt)
{
    const float drag = dragAccel(velocity_mps_, altitude_m_);
    const float net_accel = -GRAVITY - drag;

    velocity_mps_ += net_accel * dt;
    altitude_m_   += velocity_mps_ * dt;
    if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

    accel_mps2_ = net_accel;

    // --- Pitch dynamics: gravity turn ---
    // Gravity acting on a stable rocket (CP aft of CG) creates a
    // restoring moment that tips the nose toward the velocity vector.
    // M_grav = m * g * sin(θ) * -(CP-CG distance)
    // Sign: when pitch > 0 (nose up), moment is negative (tips nose down).
    const float M_grav = cfg_.mass_kg * GRAVITY * sinf(pitch_rad_) * (-CP_CG_DIST_M);

    // Aerodynamic pitch damping (opposes rotation, proportional to q_dyn)
    float M_damp = 0.0f;
    const float v_abs = fabsf(velocity_mps_);
    if (v_abs > 1.0f)
    {
        const float q_dyn = 0.5f * airDensity(altitude_m_) * v_abs * v_abs;
        M_damp = -C_DAMP * q_dyn * REFERENCE_AREA / v_abs * pitch_rate_rps_;
    }

    const float pitch_accel = (M_grav + M_damp) / I_TRANSVERSE;
    pitch_rate_rps_ += pitch_accel * dt;
    pitch_rad_      += pitch_rate_rps_ * dt;

    if (velocity_mps_ <= 0.0f)
    {
        velocity_mps_ = 0.0f;
        phase_ = SIM_DESCENT;
        phase_start_ms_ = millis();
        ESP_LOGI("SIM", "Phase: DESCENT (apogee ~%.0fm, pitch=%.1f deg)",
                 (double)altitude_m_, (double)(pitch_rad_ * 180.0f / 3.14159265f));
    }
}

void SensorCollectorSim::stepDescent(float dt)
{
    const float rate = (cfg_.descent_rate_mps > 0.0f)
        ? cfg_.descent_rate_mps : 5.0f;
    velocity_mps_ = -rate;
    altitude_m_  += velocity_mps_ * dt;
    accel_mps2_   = 0.0f;

    // Under parachute: pendulum relaxes pitch toward 0° (horizontal)
    const float M_pend = -cfg_.mass_kg * GRAVITY * sinf(pitch_rad_) * 0.1f;
    const float M_damp_chute = -0.5f * pitch_rate_rps_;
    pitch_rate_rps_ += (M_pend + M_damp_chute) / I_TRANSVERSE * dt;
    pitch_rad_      += pitch_rate_rps_ * dt;

    if (altitude_m_ <= 0.0f)
    {
        altitude_m_   = 0.0f;
        velocity_mps_ = 0.0f;
        phase_ = SIM_LANDED;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: LANDED");
    }
}

// ============================================================================
// Physics helpers
// ============================================================================

float SensorCollectorSim::airDensity(float alt_m) const
{
    const float x = 1.0f - 2.2558e-5f * alt_m;
    return (x > 0.0f) ? 1.225f * powf(x, 4.2559f) : 0.0f;
}

float SensorCollectorSim::pressureAtAltitude(float alt_m) const
{
    const float x = 1.0f - alt_m / 44330.0f;
    return (x > 0.0f) ? ground_pressure_pa_ * powf(x, 5.255f) : 0.0f;
}

float SensorCollectorSim::dragAccel(float velocity, float alt) const
{
    if (cfg_.mass_kg <= 0.0f) return 0.0f;
    const float v = fabsf(velocity);
    const float rho = airDensity(alt);
    const float f_drag = 0.5f * CD * REFERENCE_AREA * rho * v * v;
    const float a_drag = f_drag / cfg_.mass_kg;
    return (velocity > 0.0f) ? a_drag : -a_drag;
}

// ============================================================================
// Sensor encoding (inverse of SensorConverter formulas)
// ============================================================================

void SensorCollectorSim::encodeISM6(uint32_t time_us, ISM6HG256Data& out)
{
    // Specific force in body frame.
    // Body X = along rocket axis (forward), Z = perpendicular (down in body).
    //
    // The sim tracks vertical motion + pitch angle.  Gravity is [0, 0, -g]
    // in ENU.  Projected into body frame by pitch angle:
    //   body_ax = accel_vertical * sin(pitch) + g * sin(pitch)
    //           = (accel_mps2_ + GRAVITY) * sin(pitch)
    //   body_az = (accel_mps2_ + GRAVITY) * cos(pitch)
    //
    // Simplification: accel_mps2_ is the net vertical acceleration (excl gravity).
    // specific_force_vertical = accel_mps2_ + GRAVITY (what a vertical accel would read).
    // Project onto body axes via pitch:
    // #512: the specific-force DIRECTION is the world "up" seen in the body
    // frame; the magnitude is the vertical specific force. This was already
    // correct — it is the gyro and the field that were inconsistent with it.
    const float sf_vert = accel_mps2_ + GRAVITY;  // specific force if perfectly vertical
    float up_b[3];
    sim_sensor_model::upInBody(pitch_rad_, up_b);
    float body_ax = sf_vert * up_b[0];   // axial (along rocket)
    float body_ay = sf_vert * up_b[1];
    float body_az = sf_vert * up_b[2];   // perpendicular

    // Carry the sim's pad frame onto the board's actual resting attitude (#508)
    // so the real→sim handover has no gravity/attitude step.  Identity unless a
    // pad alignment was solved.
    applyPadAlign(body_ax, body_ay, body_az);

    // Gyro: body rate for the current pitch rate, through the same inverse chain
    // as accel (rocket → board → sensor).
    //
    // #512: this used to be +pitch_rate on Y, which is the WRONG SIGN. In FLU a
    // positive rotation about +Y pitches the nose DOWN, while `pitch` measures
    // nose-UP — so a rising nose is a NEGATIVE omega_y. The old sign made the
    // accelerometer's gravity vector rotate 180° opposite to the gyro, i.e. the
    // sim emitted a motion no rigid body can perform, and the EKF could never
    // fuse it. gyroInBody() owns the sign now, alongside upInBody/fieldInBody, so
    // the three cannot drift apart again.
    float gyro_b[3];
    sim_sensor_model::gyroInBody(pitch_rate_rps_ * (180.0f / 3.14159265f), gyro_b);
    float gyro_body_x_dps = gyro_b[0];
    float gyro_body_y_dps = gyro_b[1];
    float gyro_body_z_dps = gyro_b[2];
    // Same rigid rotation as the accel: the body rates must be expressed in the
    // same re-oriented frame or the attitude the EKF integrates won't match the
    // gravity vector it sees (#508).
    applyPadAlign(gyro_body_x_dps, gyro_body_y_dps, gyro_body_z_dps);

    packISM6(time_us, body_ax, body_ay, body_az,
             gyro_body_x_dps, gyro_body_y_dps, gyro_body_z_dps, out);
}

void SensorCollectorSim::packISM6(uint32_t time_us, float ax, float ay, float az,
                                  float gx_dps, float gy_dps, float gz_dps,
                                  ISM6HG256Data& out) const
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Rocket frame → board frame (inverse of the mounting orientation;
    // identity for the standard +X-nose mounting).
    rocketToBoard(ax, ay, az);

    // Inverse rotation: board-frame → sensor-frame (ISM6 board rotation about Z)
    // Only rotates X/Y; Z passes through
    const float sensor_ax =  ax * ism6_inv_c_ + ay * ism6_inv_s_;
    const float sensor_ay = -ax * ism6_inv_s_ + ay * ism6_inv_c_;
    const float sensor_az =  az;

    // Low-G accelerometer (±16g range)
    out.acc_low_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    out.acc_low_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    out.acc_low_raw.z = (int16_t)constrain(
        lroundf(sensor_az / ACC_LOW_MS2_PER_LSB), -32768, 32767);

    // High-G accelerometer (±256g range)
    out.acc_high_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
    out.acc_high_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
    out.acc_high_raw.z = (int16_t)constrain(
        lroundf(sensor_az / ACC_HIGH_MS2_PER_LSB), -32768, 32767);

    rocketToBoard(gx_dps, gy_dps, gz_dps);
    const float sensor_gx =  gx_dps * ism6_inv_c_ + gy_dps * ism6_inv_s_;
    const float sensor_gy = -gx_dps * ism6_inv_s_ + gy_dps * ism6_inv_c_;
    const float sensor_gz =  gz_dps;
    out.gyro_raw.x = (int16_t)constrain(
        lroundf(sensor_gx / GYRO_DPS_PER_LSB), -32768, 32767);
    out.gyro_raw.y = (int16_t)constrain(
        lroundf(sensor_gy / GYRO_DPS_PER_LSB), -32768, 32767);
    out.gyro_raw.z = (int16_t)constrain(
        lroundf(sensor_gz / GYRO_DPS_PER_LSB), -32768, 32767);
}

void SensorCollectorSim::encodeBMP585(uint32_t time_us, BMP585Data& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us  = time_us;
    out.press_q6 = (uint32_t)(pressureAtAltitude(altitude_m_) * 64.0f);
    out.temp_q16 = (int32_t)(20.0f * 65536.0f);  // 20 deg C
}

void SensorCollectorSim::encodeMMC5983MA(uint32_t time_us, MMC5983MAData& out)
{
    // Earth magnetic field at ~38N latitude (NED frame):
    //   North ~ 22 µT, East ~ 5 µT, Down ~ 42 µT
    // Rotate into body frame by pitch angle.
    // Body X = along rocket axis, Y = lateral, Z = perpendicular (body-down)
    // Pitch rotation: nose-up angle from horizontal.
    //   body_x =  North * sin(pitch) + Down * cos(pitch)
    //   body_y =  East
    //   body_z = -North * cos(pitch) + Down * sin(pitch)

    // #512: was (N·sp + D·cp, E, -N·cp + D·sp) — a rotation whose implied "up"
    // sat 90°+ away from the accelerometer's. fieldInBody() projects the NED
    // field onto the actual body axes, so it now rotates in lockstep with
    // upInBody/gyroInBody.
    float body[3];
    sim_sensor_model::fieldInBody(pitch_rad_, B_NORTH, B_EAST, B_DOWN, body);
    float body_x = body[0];
    float body_y = body[1];
    float body_z = body[2];

    // Pad-frame alignment (#508) — keeps the field continuous across the
    // real→sim handover, so no heading step winds the gyro bias either.
    applyPadAlign(body_x, body_y, body_z);

    packMMC5983MA(time_us, body_x, body_y, body_z, out);
}

void SensorCollectorSim::packMMC5983MA(uint32_t time_us, float bx, float by, float bz,
                                       MMC5983MAData& out) const
{
    static constexpr float COUNTS_PER_UT = 131072.0f / 800.0f;
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Rocket frame → board frame (inverse mounting orientation), matching
    // the forward board→rocket rotation the converter now applies to mag.
    rocketToBoard(bx, by, bz);

    out.mag_x = (uint32_t)(lroundf(bx * COUNTS_PER_UT) + 131072);
    out.mag_y = (uint32_t)(lroundf(by * COUNTS_PER_UT) + 131072);
    out.mag_z = (uint32_t)(lroundf(bz * COUNTS_PER_UT) + 131072);
}

void SensorCollectorSim::encodeIIS2MDC(uint32_t time_us, IIS2MDCData& out)
{
    // Same Earth field + body projection as encodeMMC5983MA (#463, corrected #512).
    float body[3];
    sim_sensor_model::fieldInBody(pitch_rad_, B_NORTH, B_EAST, B_DOWN, body);
    float body_x = body[0];
    float body_y = body[1];
    float body_z = body[2];

    // Pad-frame alignment (#508) — see encodeISM6.
    applyPadAlign(body_x, body_y, body_z);

    packIIS2MDC(time_us, body_x, body_y, body_z, out);
}

void SensorCollectorSim::packIIS2MDC(uint32_t time_us, float bx, float by, float bz,
                                     IIS2MDCData& out) const
{
    // Counts at the scale of the chip behind the seam (#1312): 0.15 µT/LSB on
    // the IIS2MDC, 100/3750 on the mini's QMC5883P — so the converter, which
    // reads the same seam, reproduces the simulated field on either board.
    const float COUNTS_PER_UT = 1.0f / real_.magLsbToUt();
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Rocket frame → board frame (inverse mounting), then board → sensor
    // frame (inverse of the converter's sensor→board +Z rotation, then of
    // the chip's axis signs, magTypeChipSign), so the forward conversion
    // chain reproduces the simulated field exactly.
    rocketToBoard(bx, by, bz);
    const uint8_t mag_type = real_.magType();
    const float sensor_x = magTypeChipSign(mag_type, 0) * (bx * iis_inv_c_ + by * iis_inv_s_);
    const float sensor_y = magTypeChipSign(mag_type, 1) * (-bx * iis_inv_s_ + by * iis_inv_c_);
    const float sensor_z = magTypeChipSign(mag_type, 2) * bz;

    out.mag_x = (int16_t)lroundf(sensor_x * COUNTS_PER_UT);
    out.mag_y = (int16_t)lroundf(sensor_y * COUNTS_PER_UT);
    out.mag_z = (int16_t)lroundf(sensor_z * COUNTS_PER_UT);
}

void SensorCollectorSim::encodeGNSS(uint32_t time_us, GNSSData& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Fixed simulated location
    out.lat_e7 = (int32_t)(38.0 * 1e7);
    out.lon_e7 = (int32_t)(-122.0 * 1e7);

    out.alt_mm     = (int32_t)(altitude_m_ * 1000.0f);
    out.vel_u_mmps = (int32_t)(velocity_mps_ * 1000.0f);
    out.vel_e_mmps = 0;
    out.vel_n_mmps = 0;

    // Simulate good GPS fix
    out.fix_mode = 3;   // 3D fix
    out.num_sats = 12;
    out.pdop_x10 = 15;  // PDOP 1.5
    out.h_acc_m  = 2;
    out.v_acc_m  = 3;

    // Timestamp
    out.year   = 2025;
    out.month  = 1;
    out.day    = 1;
    out.hour   = 12;
    out.minute = 0;
    out.second = (uint8_t)((uint32_t)sim_time_s_ % 60);
    out.milli_second = (uint16_t)(fmodf(sim_time_s_, 1.0f) * 1000.0f);
}
