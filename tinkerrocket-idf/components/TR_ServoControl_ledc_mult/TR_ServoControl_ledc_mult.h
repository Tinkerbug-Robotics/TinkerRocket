// TR_ServoControl.h
#ifndef TR_SERVO_CONTROL_H
#define TR_SERVO_CONTROL_H

#include <compat.h>
#include <driver/ledc.h>
#include "TR_PID.h"

class TR_ServoControl {
public:
    TR_ServoControl(uint8_t servo_pin_1,
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
                    float max_cmd_in);

    // configure timers/channels and centre all servos
    void begin();
    // set desired roll‑rate setpoint (deg/s)
    void setSetpoint(float setpoint);
    // update PWM outputs from measured roll-rate (deg/s)
    void control(float roll_rate);
    // sweep from mid->min->max->mid
    void wiggle();
    // return servos to centre
    void stowControl();
    // set each servo to an individual angle (degrees), bypassing PID
    void setServoAngles(const float angles[4]);
    // last computed roll command (deg)
    float getRollCmdDeg();
    // last computed pulse width (µs) of servo1
    int   getRollCmdUs();

    // Runtime configuration setters
    void setBias(int servoIndex, int biasUs);
    void setServoTiming(int hz, int minUs, int maxUs);
    void setPIDGains(float kp, float ki, float kd);
    void setPIDLimits(float minCmd, float maxCmd);
    // See TR_PID::setDerivativeFilterCutoffHz — rejects measurement noise
    // on the D term. fc_hz<=0 disables.
    void setPIDDerivativeFilterCutoffHz(float fc_hz);

    // Reset PID internal state (for replay / test sessions)
    void resetPID();

    // Gain scheduling configuration
    void enableGainSchedule(float v_ref, float v_min);
    void disableGainSchedule();
    bool isGainScheduleEnabled() const { return gain_schedule_enabled; }
    // update PWM outputs with velocity-based gain scaling
    void controlWithGainSchedule(float roll_rate, float velocity_ms);

    // ── Cascaded angle → rate controller ──
    // Outer loop: rate_cmd = kp_angle * (target_roll_deg - actual_roll_deg)
    // Inner loop: PID on (rate_cmd - roll_rate_dps) with gain scheduling
    void controlAngle(float target_roll_deg,
                      float actual_roll_deg,
                      float roll_rate_dps,
                      float velocity_ms,
                      float kp_angle);
    void setAngleControlKpAngle(float kp) { kp_angle_ = kp; }
    float getAngleControlKpAngle() const { return kp_angle_; }

private:
    // update all four servos to a single nominal pulse
    void setPulse(int base_pulse_us);
    int  saturateCommand(int command);

    // arrays for each servo pin, bias and mid positions
    uint8_t servo_pin_[4];
    int     servo_bias_us_[4];
    int     servo_mid_us_[4];

    // shared config
    int   servo_hz;
    int   servo_min_us;
    int   servo_max_us;

    // PID for roll‑rate control
    float pid_setpoint;
    TR_PID pid;
    float roll_cmd_deg;
    int   roll_cmd_us;
    float min_cmd;
    float max_cmd;

    // Base PID gains (used as reference for gain scheduling)
    float kp_base;
    float ki_base;
    float kd_base;

    // Gain scheduling
    bool  gain_schedule_enabled;
    float gain_schedule_v_ref;
    float gain_schedule_v_min;

    // Cascaded angle control
    float kp_angle_;

    // Previous gain schedule scale factor (for I-term reset on large changes)
    float prev_gain_scale_ = 1.0f;

    // LEDC channels/timers for four servos
    static constexpr int LEDC_CHANNEL_COUNT = 4;
    static constexpr ledc_channel_t LEDC_CHANNELS[LEDC_CHANNEL_COUNT] = {
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3 };
    static constexpr ledc_timer_t   LEDC_TIMERS [LEDC_CHANNEL_COUNT] = {
        LEDC_TIMER_0,   LEDC_TIMER_1,   LEDC_TIMER_2,   LEDC_TIMER_3   };
    static constexpr ledc_mode_t    LEDC_MODE       = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_12_BIT;
};

#endif // TR_SERVO_CONTROL_H