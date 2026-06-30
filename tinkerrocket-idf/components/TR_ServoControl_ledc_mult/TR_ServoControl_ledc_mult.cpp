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
        ledc_timer_config(&timer_conf);

        ledc_channel_config_t channel_conf = {};
        channel_conf.gpio_num   = servo_pin_[i];
        channel_conf.speed_mode = LEDC_MODE;
        channel_conf.channel    = LEDC_CHANNELS[i];
        channel_conf.intr_type  = LEDC_INTR_DISABLE;
        channel_conf.timer_sel  = LEDC_TIMERS[i];
        channel_conf.duty       = 0;
        channel_conf.hpoint     = 0;
        ledc_channel_config(&channel_conf);
    }
    // centre all servos
    setPulse(0);
}

void TR_ServoControl::setSetpoint(float setpoint) {
    pid_setpoint = setpoint;
}

void TR_ServoControl::control(float roll_rate) {
    // PID output (deg) within [min_cmd..max_cmd]
    roll_cmd_deg = pid.computePID(pid_setpoint, roll_rate);
    roll_cmd_deg = constrain(roll_cmd_deg, min_cmd, max_cmd);

    // Map the commanded fin angle (deg) to a servo pulse via the physical
    // fin calibration (#267) — NOT the command-clamp span.
    int base_pulse = usFromFinDeg(roll_cmd_deg);
    base_pulse     = saturateCommand(base_pulse);
    // drive all servos
    setPulse(base_pulse);
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

void TR_ServoControl::wiggle() {
    // Boot "servos alive" check.  Wiggle each servo individually, in order
    // 0->1->2->3, so only ONE servo draws movement (inrush) current at a time.
    // Sweeping all four at once stacked four inrush spikes on the shared rail,
    // which could sag it enough to brown out the camera on the pad — a
    // bandaid for that, until the next PCB's servo-power MOSFET.  Each servo
    // returns to centre before the next begins; the others hold their
    // begin()-set centre while one sweeps.
    const int mid = (servo_min_us + servo_max_us) / 2;
    constexpr int kSettleMs = 350;  // ample for an 8 g servo to traverse + be seen
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        setPulseChannel(i, servo_min_us);  // min
        delay(kSettleMs);
        setPulseChannel(i, servo_max_us);  // max
        delay(kSettleMs);
        setPulseChannel(i, mid);           // back to centre
        delay(kSettleMs);
    }
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
        // Clamp to the command limit (max deflection), then map via the physical
        // fin calibration (#267) so the fin reaches the commanded *physical* angle
        // — no longer truncated to / scaled by the roll-PID clamp.
        float angle = constrain(angles[i], min_cmd, max_cmd);
        int pulse_us = usFromFinDeg(angle) + servo_bias_us_[i];
        pulse_us = saturateCommand(pulse_us);

        uint32_t max_duty = (1u << LEDC_RESOLUTION) - 1;
        uint32_t duty = (static_cast<uint32_t>(pulse_us)
                        * static_cast<uint32_t>(servo_hz)
                        * max_duty) / 1000000u;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[i], duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[i]);
    }
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

void TR_ServoControl::controlWithGainSchedule(float roll_rate, float velocity_ms) {
    if (gain_schedule_enabled) {
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
    pid_setpoint = -rate_cmd;
    controlWithGainSchedule(roll_rate_dps, velocity_ms);
}