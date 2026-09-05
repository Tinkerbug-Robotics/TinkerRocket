#include "TR_ServoControl_ledc_mult.h"
#include <algorithm>
#include <cmath>

TR_ServoControl::TR_ServoControl(uint8_t servo_pin_1,
                                 uint8_t servo_pin_2,
                                 uint8_t servo_pin_3,
                                 uint8_t servo_pin_4,
                                 int bias_us_1,
                                 int bias_us_2,
                                 int bias_us_3,
                                 int bias_us_4,
                                 int servo_hz,
                                 int min_us,
                                 int max_us,
                                 float kp,
                                 float ki,
                                 float kd,
                                 float min_cmd_in,
                                 float max_cmd_in)
    : servo_hz(servo_hz),
      servo_min_us(min_us),
      servo_max_us(max_us),
      pid(kp, ki, kd, max_cmd_in, min_cmd_in),
      pid_setpoint(0.0f),
      roll_cmd_deg(0.0f),
      roll_cmd_us(min_us),
      min_cmd(min_cmd_in),
      max_cmd(max_cmd_in),
      fin_min_deg_(min_cmd_in),
      fin_max_deg_(max_cmd_in),
      kp_base(kp),
      ki_base(ki),
      kd_base(kd),
      gain_schedule_enabled(false),
      gain_schedule_v_ref(95.0f),
      gain_schedule_v_min(30.0f),
      kp_angle_(4.0f)
{
    servo_pin_[0] = servo_pin_1;
    servo_pin_[1] = servo_pin_2;
    servo_pin_[2] = servo_pin_3;
    servo_pin_[3] = servo_pin_4;

    servo_bias_us_[0] = bias_us_1;
    servo_bias_us_[1] = bias_us_2;
    servo_bias_us_[2] = bias_us_3;
    servo_bias_us_[3] = bias_us_4;

    // compute each servo’s mid-point (shared min/max with individual bias)
    for (int i = 0; i < 4; ++i) {
        servo_mid_us_[i] = ((min_us + max_us) / 2) + servo_bias_us_[i];
    }
}

void TR_ServoControl::begin() {
    // setup one LEDC timer per servo to avoid jitter when values change
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode      = LEDC_MODE;
        timer_conf.duty_resolution = LEDC_RESOLUTION;
        timer_conf.timer_num       = LEDC_TIMERS[i];
        timer_conf.freq_hz         = static_cast<uint32_t>(servo_hz);
        timer_conf.clk_cfg         = LEDC_AUTO_CLK;
        esp_err_t terr = ledc_timer_config(&timer_conf);

        ledc_channel_config_t channel_conf = {};
        channel_conf.gpio_num   = servo_pin_[i];
        channel_conf.speed_mode = LEDC_MODE;
        channel_conf.channel    = LEDC_CHANNELS[i];
        // intr_type removed: deprecated in IDF v6 (the driver handles the LEDC
        // interrupt internally). channel_conf is zero-initialized above, which
        // is equivalent to the old LEDC_INTR_DISABLE (== 0) anyway.
        channel_conf.timer_sel  = LEDC_TIMERS[i];
        channel_conf.duty       = 0;
        channel_conf.hpoint     = 0;
        esp_err_t cerr = ledc_channel_config(&channel_conf);

        // A silent failure here leaves one servo dead with everything else
        // healthy (command path fine, no pulse on the pad) — make it loud.
        if (terr != ESP_OK || cerr != ESP_OK) {
            ESP_LOGE("SERVO", "begin: servo %d (pin %d) LEDC config FAILED timer=%s channel=%s",
                     i + 1, (int)servo_pin_[i],
                     esp_err_to_name(terr), esp_err_to_name(cerr));
        }
    }
    // Deliberately NO motion here.  ledc_channel_config() above left every
    // channel at duty 0, which is precisely the state idle() produces: the
    // output is held low, no pulse train reaches the servos, and the fins stay
    // where they are.  Record that so idle() stays idempotent and the first
    // real drive command resumes PWM through the normal path.
    //
    // This used to be setPulse(0) — a simultaneous four-channel snap to the raw
    // pulse mid-point, issued from setup_fc before the firmware had asked
    // whether the vehicle was still airborne, and gated only on the pins being
    // mapped rather than on servo control being enabled at all.
    is_idle_ = true;
}

void TR_ServoControl::setSetpoint(float setpoint) {
    pid_setpoint = setpoint;
}

void TR_ServoControl::control(float roll_rate) {
    controlToSetpoint(pid_setpoint, roll_rate);
}

void TR_ServoControl::controlToSetpoint(float setpoint, float roll_rate) {
    // PID output (deg) within [min_cmd..max_cmd]
    roll_cmd_deg = pid.computePID(setpoint, roll_rate);
    roll_cmd_deg = constrain(roll_cmd_deg, min_cmd, max_cmd);

    // Pure roll: every fin takes the SAME commanded deflection (that is what
    // makes a couple about the roll axis), negated on the servos whose
    // roll-reverse bit is set.  Driving through setServoAngles() rather than a
    // single broadcast pulse is what lets the per-servo sign exist at all, and
    // it reuses the identical fin-calibration (#267) deg->us map and per-servo
    // bias the broadcast applied, so with mask 0 the pulses are unchanged.
    float angles[4];
    for (int i = 0; i < 4; ++i) {
        angles[i] = (roll_reverse_mask_ & (1u << i)) ? -roll_cmd_deg : roll_cmd_deg;
    }
    setServoAngles(angles);
    // Keep the legacy servo-1 telemetry reading (getRollCmdUs); setServoAngles
    // records every channel, the broadcast path this replaced recorded only
    // channel 0.
    roll_cmd_us = last_pulse_us_[0];
}

// #267: physical fin-angle (deg) -> servo pulse (us).  The line is defined by
// (fin_min_deg_ -> servo_min_us) and (fin_max_deg_ -> servo_max_us), so a
// commanded fin angle produces the pulse that yields that physical deflection,
// independent of the roll command clamp.
int TR_ServoControl::usFromFinDeg(float fin_deg) const {
    float span_deg = fin_max_deg_ - fin_min_deg_;
    if (span_deg == 0.0f) return (servo_min_us + servo_max_us) / 2;
    float norm = (fin_deg - fin_min_deg_) / span_deg;
    return servo_min_us + static_cast<int>(norm * (servo_max_us - servo_min_us));
}

void TR_ServoControl::setFinCalibration(float finMinDeg, float finMaxDeg) {
    fin_min_deg_ = finMinDeg;
    fin_max_deg_ = finMaxDeg;
}

// Boot "servos alive" check.  Wiggles each servo individually, in order
// 0->1->2->3, so only ONE servo draws movement (inrush) current at a time.
// Sweeping all four at once stacked four inrush spikes on the shared rail,
// which could sag it enough to brown out the camera on the pad — a bandaid for
// that, until the next PCB's servo-power MOSFET.  Do NOT parallelise the
// sequencing to save time; the one-at-a-time order is the point.
//
// Now a state machine rather than 12 blocking delay()s.  The old blocking form
// cost 4.2 s inside setup_fc, ahead of the in-flight reboot check, and it could
// not simply be relocated into the flight loop: that task is subscribed to a
// 5 s panic watchdog (esp_task_wdt with trigger_panic), so a 4.2 s blocking
// call there would sit at 84% of the panic deadline with no margin for a slow
// tick.  Serviced across ticks it costs nothing and cannot trip the watchdog.
static_assert(TR_ServoControl::kWiggleDurationMs == 4u * 3u * 350u,
              "wiggle duration must match 4 servos x 3 positions x step dwell");

void TR_ServoControl::beginWiggle(uint32_t now_ms) {
    wiggle_active_ = true;
    wiggle_step_   = 0;
    wiggle_at_ms_  = now_ms;   // first step is commanded on the next service call
}

void TR_ServoControl::serviceWiggle(uint32_t now_ms) {
    if (!wiggle_active_) return;
    // Signed compare so the tick counter can wrap safely (same idiom as
    // serviceNeutralSettle and the pad-relax wake gate in main.cpp).
    if ((int32_t)(now_ms - wiggle_at_ms_) < 0) return;

    constexpr uint8_t kSteps = 4 * 3;
    if (wiggle_step_ >= kSteps) {          // sweep complete
        wiggle_active_ = false;
        return;
    }

    const int mid     = (servo_min_us + servo_max_us) / 2;
    const int servo   = wiggle_step_ / 3;
    const int phase   = wiggle_step_ % 3;
    const int pulse   = (phase == 0) ? servo_min_us
                      : (phase == 1) ? servo_max_us
                                     : mid;
    setPulseChannel(servo, pulse);

    // Note the dwell is applied AFTER the final step too: the deactivation
    // branch above is reached only once wiggle_at_ms_ has expired again, so the
    // closing centre command is actually held rather than being cut short by
    // the caller seeing isWiggling() go false on the same tick it was issued.
    ++wiggle_step_;
    wiggle_at_ms_ = now_ms + kWiggleStepMs;
}

void TR_ServoControl::stowControl() {
    setPulse(0);
}

void TR_ServoControl::idle() {
    // Already relaxed — don't re-issue the duty writes every loop tick.
    if (is_idle_) return;
    // 0% duty holds the GPIO low for the whole period: no rising edge, so the
    // servo sees no pulse train and (if it honours signal loss) relaxes.  We
    // stay inside the normal LEDC duty API rather than ledc_stop(), so the next
    // setPulse()/setServoAngles() resumes pulses through the identical path.
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[i], 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[i]);
    }
    is_idle_ = true;
}

void TR_ServoControl::setServoAngles(const float angles[4]) {
    is_idle_ = false;  // commanding a pulse resumes PWM after idle()
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        // Clamp the commanded fin angle to the physical fin-calibration range
        // (fin_min_deg_..fin_max_deg_), then map via that calibration (#267) so
        // the fin reaches the commanded *physical* angle.  Clamp to the fin
        // range — NOT the roll/guidance command authority (min_cmd/max_cmd):
        // this path drives both the flight mixer (already pre-clamped to its own
        // authority upstream) and the manual servo test, and the test must be
        // able to sweep to the calibrated endpoints so servo_min_us<->fin_min_deg
        // and servo_max_us<->fin_max_deg are honoured per model instead of being
        // capped at the (narrower, model-varying) flight authority.  Fin cal
        // defaults to [min_cmd,max_cmd] until configured, so behaviour is
        // unchanged until a real fin calibration is set.
        float angle = constrain(angles[i], fin_min_deg_, fin_max_deg_);
        int pulse_us = usFromFinDeg(angle) + servo_bias_us_[i];
        pulse_us = saturateCommand(pulse_us);
        last_pulse_us_[i] = pulse_us;

        uint32_t max_duty = (1u << LEDC_RESOLUTION) - 1;
        uint32_t duty = (static_cast<uint32_t>(pulse_us)
                        * static_cast<uint32_t>(servo_hz)
                        * max_duty) / 1000000u;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[i], duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[i]);
    }
}

void TR_ServoControl::beginNeutralSettle(uint32_t now_ms) {
    // Phase 1: drive a few degrees past neutral on every channel (a higher
    // pulse — usFromFinDeg is monotonic — so all four load their gear mesh from
    // the same side).  setServoAngles clamps to the fin-cal range, so a narrow
    // cal simply parks at its endpoint.  serviceNeutralSettle() completes the
    // move to neutral once kNeutralSettleHoldMs has elapsed.
    const float overshoot[4] = {
        kNeutralSettleOvershootDeg, kNeutralSettleOvershootDeg,
        kNeutralSettleOvershootDeg, kNeutralSettleOvershootDeg };
    setServoAngles(overshoot);
    neutral_settle_active_ = true;
    neutral_settle_at_ms_  = now_ms + kNeutralSettleHoldMs;
}

void TR_ServoControl::serviceNeutralSettle(uint32_t now_ms) {
    if (!neutral_settle_active_) return;
    // Signed compare so the tick counter can wrap safely (same idiom as the
    // pad-relax wake gate in main.cpp).
    if ((int32_t)(now_ms - neutral_settle_at_ms_) < 0) return;  // still overshooting
    // Phase 2: settle to the commanded neutral, approached from the high side.
    const float neutral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    setServoAngles(neutral);
    neutral_settle_active_ = false;
}

int TR_ServoControl::saturateCommand(int command) {
    if (command < servo_min_us) return servo_min_us;
    if (command > servo_max_us) return servo_max_us;
    return command;
}

void TR_ServoControl::setPulseChannel(int channel, int base_pulse_us) {
    // Drive a single servo channel.  base_pulse_us is the nominal pulse, 0
    // meaning “centre”; the per-servo bias is applied here exactly as setPulse
    // does, so driving one channel matches driving all.
    if (channel < 0 || channel >= LEDC_CHANNEL_COUNT) return;
    is_idle_ = false;  // commanding a pulse resumes PWM after idle()

    int pulse_us = (base_pulse_us == 0) ? servo_mid_us_[channel]
                                        : base_pulse_us + servo_bias_us_[channel];
    pulse_us = saturateCommand(pulse_us);
    last_pulse_us_[channel] = pulse_us;

    uint32_t max_duty = (1u << LEDC_RESOLUTION) - 1;
    uint32_t duty     = (static_cast<uint32_t>(pulse_us)
                        * static_cast<uint32_t>(servo_hz)
                        * max_duty) / 1000000u;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[channel], duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[channel]);

    // record last command from servo0’s perspective
    if (channel == 0) roll_cmd_us = pulse_us;
}

void TR_ServoControl::setPulse(int base_pulse_us) {
    // base_pulse_us is computed from control(), 0 means “centre”.  Drive all
    // four channels (intentionally simultaneous — the flight control path
    // updates every servo each tick).
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        setPulseChannel(i, base_pulse_us);
    }
}

float TR_ServoControl::getRollCmdDeg() {
    return roll_cmd_deg;
}

int TR_ServoControl::getRollCmdUs() {
    return roll_cmd_us;
}

void TR_ServoControl::setBias(int servoIndex, int biasUs) {
    if (servoIndex < 0 || servoIndex >= 4) return;
    servo_bias_us_[servoIndex] = biasUs;
    servo_mid_us_[servoIndex] = ((servo_min_us + servo_max_us) / 2) + biasUs;
}

void TR_ServoControl::setServoTiming(int hz, int minUs, int maxUs) {
    servo_hz = hz;
    servo_min_us = minUs;
    servo_max_us = maxUs;
    // Recompute all mid-points with new min/max
    for (int i = 0; i < 4; ++i) {
        servo_mid_us_[i] = ((minUs + maxUs) / 2) + servo_bias_us_[i];
    }
    // Reconfigure all LEDC timers with new frequency
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode      = LEDC_MODE;
        timer_conf.duty_resolution = LEDC_RESOLUTION;
        timer_conf.timer_num       = LEDC_TIMERS[i];
        timer_conf.freq_hz         = static_cast<uint32_t>(hz);
        timer_conf.clk_cfg         = LEDC_AUTO_CLK;
        ledc_timer_config(&timer_conf);
    }
    // Re-center servos with new timing
    setPulse(0);
}

void TR_ServoControl::setPIDGains(float kp, float ki, float kd) {
    kp_base = kp;
    ki_base = ki;
    kd_base = kd;
    pid.setKp(kp);
    pid.setKi(ki);
    pid.setKd(kd);
}

void TR_ServoControl::setPIDLimits(float minCmd, float maxCmd) {
    min_cmd = minCmd;
    max_cmd = maxCmd;
    pid.setMinCmd(minCmd);
    pid.setMaxCmd(maxCmd);
}

void TR_ServoControl::setPIDDerivativeFilterCutoffHz(float fc_hz) {
    pid.setDerivativeFilterCutoffHz(fc_hz);
}

void TR_ServoControl::setPIDIntegralSeparationThreshold(float threshold) {
    pid.setIntegralSeparationThreshold(threshold);
}

void TR_ServoControl::resetPID() {
    pid.reset();
}

void TR_ServoControl::enableGainSchedule(float v_ref, float v_min) {
    gain_schedule_enabled = true;
    gain_schedule_v_ref = v_ref;
    gain_schedule_v_min = v_min;
}

void TR_ServoControl::disableGainSchedule() {
    gain_schedule_enabled = false;
    // Restore base gains
    pid.setKp(kp_base);
    pid.setKi(ki_base);
    pid.setKd(kd_base);
}

void TR_ServoControl::applyGainSchedule(float velocity_ms) {
    if (!gain_schedule_enabled) {
        return;
    }
    float v = std::max(std::fabs(velocity_ms), gain_schedule_v_min);
    float v_ratio = gain_schedule_v_ref / v;
    float scale = v_ratio * v_ratio;
    // Cap the scale factor so servo saturates at ~50 deg/s error, not gyro noise
    scale = std::min(scale, GAIN_SCHEDULE_SCALE_CAP);

    // Reset I-term when gain scale changes significantly to prevent
    // accumulated integral from spiking the output after a step change in Ki.
    if (fabs(scale - prev_gain_scale_) > 0.1f) {
        pid.resetIntegral();
    }
    prev_gain_scale_ = scale;

    pid.setKp(kp_base * scale);
    pid.setKi(ki_base * scale);
    pid.setKd(kd_base * scale);
}

void TR_ServoControl::controlWithGainSchedule(float roll_rate, float velocity_ms) {
    applyGainSchedule(velocity_ms);
    control(roll_rate);
}

void TR_ServoControl::controlAngle(float target_roll_deg,
                                   float actual_roll_deg,
                                   float roll_rate_dps,
                                   float velocity_ms,
                                   float kp_angle,
                                   float rate_cap_dps) {
    // ── Outer loop: angle error → rate command ──
    // Wrap angle error to [-180, +180] degrees
    float angle_error = target_roll_deg - actual_roll_deg;
    while (angle_error > 180.0f)  angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;
    float rate_cmd = kp_angle * angle_error;  // deg/s

    // Cap the outer-loop rate command. With a 180 deg wrapped error and
    // kp_angle=4, this would otherwise demand 720 dps -- a rate the inner
    // loop can't deliver, which also drives integrator windup and direction
    // whipsawing during spin recovery.
    if (rate_cap_dps > 0.0f) {
        rate_cmd = constrain(rate_cmd, -rate_cap_dps, rate_cap_dps);
    }

    // ── Inner loop: PID on rate error with gain scheduling ──
    // roll_rate_dps is passed in negated (–gyro_x) to match servo sign
    // convention, so negate rate_cmd to keep setpoint in the same frame.
    // The angle-loop rate command applies to THIS tick only — pid_setpoint is
    // never mutated here (#372), so a later rate-null tick (profile segment
    // change or the #265 EKF-health fallback) nulls to the configured
    // setpoint instead of holding the last angle-loop rate command.
    applyGainSchedule(velocity_ms);
    controlToSetpoint(-rate_cmd, roll_rate_dps);
}