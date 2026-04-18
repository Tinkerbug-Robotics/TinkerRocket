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

    // normalise to [0..1]
    float span_deg = max_cmd - min_cmd;
    float norm     = (roll_cmd_deg - min_cmd) / span_deg;
    // convert to pulse (centre bias and servo range)
    int base_pulse = servo_min_us + static_cast<int>(norm * (servo_max_us - servo_min_us));
    base_pulse     = saturateCommand(base_pulse);
    // drive all servos
    setPulse(base_pulse);
}

void TR_ServoControl::wiggle() {
    // sweep through min/mid/max for all servos
    //for (int i = 0; i < 4; ++i) {
        setPulse(((servo_min_us + servo_max_us) / 2)); // mid
        delay(1250);
        setPulse(servo_min_us); // min
        delay(1500);
        setPulse(servo_max_us); // max
        delay(1500);
        setPulse(((servo_min_us + servo_max_us) / 2)); // back to mid
    //}
}

void TR_ServoControl::stowControl() {
    setPulse(0);
}

void TR_ServoControl::setServoAngles(const float angles[4]) {
    float span_deg = max_cmd - min_cmd;
    if (span_deg <= 0.0f) return;

    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        float angle = constrain(angles[i], min_cmd, max_cmd);
        float norm = (angle - min_cmd) / span_deg;
        int pulse_us = servo_min_us
                     + static_cast<int>(norm * (servo_max_us - servo_min_us))
                     + servo_bias_us_[i];
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

void TR_ServoControl::setPulse(int base_pulse_us) {
    // base_pulse_us is computed from control(), 0 means “centre”
    for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        // compute final pulse for this servo by adding its bias
        int pulse_us;
        if (base_pulse_us == 0) {
            pulse_us = servo_mid_us_[i];
        } else {
            pulse_us = base_pulse_us + servo_bias_us_[i];
        }
        pulse_us = saturateCommand(pulse_us);

        uint32_t max_duty = (1u << LEDC_RESOLUTION) - 1;
        uint32_t duty     = (static_cast<uint32_t>(pulse_us)
                            * static_cast<uint32_t>(servo_hz)
                            * max_duty) / 1000000u;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[i], duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[i]);

        // record last command from servo0’s perspective
        if (i == 0) roll_cmd_us = pulse_us;
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
        scale = std::min(scale, 3.0f);

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
                                   float kp_angle) {
    // ── Outer loop: angle error → rate command ──
    // Wrap angle error to [-180, +180] degrees
    float angle_error = target_roll_deg - actual_roll_deg;
    while (angle_error > 180.0f)  angle_error -= 360.0f;
    while (angle_error < -180.0f) angle_error += 360.0f;
    float rate_cmd = kp_angle * angle_error;  // deg/s

    // ── Inner loop: PID on rate error with gain scheduling ──
    // roll_rate_dps is passed in negated (–gyro_x) to match servo sign
    // convention, so negate rate_cmd to keep setpoint in the same frame.
    pid_setpoint = -rate_cmd;
    controlWithGainSchedule(roll_rate_dps, velocity_ms);
}