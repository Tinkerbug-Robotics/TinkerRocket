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
    // Relax the servos by stopping the PWM pulse train (0% duty -> output held
    // low, no servo pulses).  A digital servo that honours signal loss goes
    // limp and stops drawing holding current; one that latches keeps holding.
    // Idempotent — only touches the hardware on the first call after a drive.
    // Any drive command (control*/setServoAngles/stowControl/setPulse) resumes
    // PWM automatically, so call wake() implicitly just by commanding a pulse.
    void idle();
    // True while the pulse train is stopped via idle().
    bool isIdle() const { return is_idle_; }
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
    // See TR_PID::setIntegralSeparationThreshold — freezes the integrator
    // during large transients (e.g. a roll kick) to prevent windup.
    void setPIDIntegralSeparationThreshold(float threshold);

    // Physical fin-angle <-> servo-pulse calibration (#267): the fin angle (deg)
    // at servo_min_us and at servo_max_us.  Decouples the deg->us scale from the
    // command clamp (min_cmd/max_cmd) so a commanded fin angle maps to the pulse
    // that produces that *physical* deflection.
    void setFinCalibration(float finMinDeg, float finMaxDeg);
    float getFinMinDeg() const { return fin_min_deg_; }
    float getFinMaxDeg() const { return fin_max_deg_; }

    // Reset PID internal state (for replay / test sessions)
    void resetPID();

    // Max gain-schedule scale factor. Caps (V_ref/V)² so servos saturate on
    // real ~50 deg/s errors at low speed, not on gyro noise.
    static constexpr float GAIN_SCHEDULE_SCALE_CAP = 3.0f;

    // Gain scheduling configuration
    void enableGainSchedule(float v_ref, float v_min);
    void disableGainSchedule();
    bool isGainScheduleEnabled() const { return gain_schedule_enabled; }

    // Live PID gains / limits — base (pre-gain-schedule) values, i.e. what
    // was loaded from NVS or config and pushed via setPIDGains/setPIDLimits.
    // Used to snapshot the flown settings into the flight log (#165).
    float getKp() const { return kp_base; }
    float getKi() const { return ki_base; }
    float getKd() const { return kd_base; }
    float getMinCmd() const { return min_cmd; }
    float getMaxCmd() const { return max_cmd; }

    // Live servo trim / timing — for the flight settings snapshot (#165).
    int getServoBiasUs(int servoIndex) const {
        return (servoIndex >= 0 && servoIndex < 4) ? servo_bias_us_[servoIndex] : 0;
    }
    int getServoHz() const { return servo_hz; }
    int getServoMinUs() const { return servo_min_us; }
    int getServoMaxUs() const { return servo_max_us; }
    // update PWM outputs with velocity-based gain scaling
    void controlWithGainSchedule(float roll_rate, float velocity_ms);

    // ── Cascaded angle → rate controller ──
    // Outer loop: rate_cmd = clamp(kp_angle * (target - actual), ±rate_cap_dps)
    // Inner loop: PID on (rate_cmd - roll_rate_dps) with gain scheduling.
    // rate_cap_dps caps the outer-loop rate command to prevent a wrapped
    // angle error from demanding rates the actuators can't track (which
    // also causes the controller to whipsaw direction during spin recovery).
    // Pass a large value (e.g. 10000) to effectively disable the cap.
    void controlAngle(float target_roll_deg,
                      float actual_roll_deg,
                      float roll_rate_dps,
                      float velocity_ms,
                      float kp_angle,
                      float rate_cap_dps);
    void setAngleControlKpAngle(float kp) { kp_angle_ = kp; }
    float getAngleControlKpAngle() const { return kp_angle_; }

private:
    // update all four servos to a single nominal pulse
    void setPulse(int base_pulse_us);
    int  saturateCommand(int command);
    // Map a physical fin angle (deg) to a servo pulse (us) via the fin
    // calibration (fin_min_deg_->servo_min_us, fin_max_deg_->servo_max_us). #267
    int  usFromFinDeg(float fin_deg) const;

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
    // Fin angle (deg) at servo_min_us / servo_max_us — the physical us<->deg
    // calibration (#267).  Defaults to min_cmd/max_cmd (legacy behaviour) until
    // setFinCalibration() is called with the real airframe values.
    float fin_min_deg_;
    float fin_max_deg_;

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

    // True while idle() has stopped the pulse train.  Cleared by any drive
    // (setPulse/setServoAngles), which re-asserts a real duty and so resumes
    // PWM without an explicit wake step.
    bool is_idle_ = false;

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