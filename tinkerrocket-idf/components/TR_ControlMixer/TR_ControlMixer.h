// TR_ControlMixer.h
// 3-axis fin mixing for cruciform (4-fin) configuration with
// independent pitch/yaw rate PIDs and gain scheduling.
#ifndef TR_CONTROL_MIXER_H
#define TR_CONTROL_MIXER_H

#include <compat.h>

#include "TR_PID.h"

class TR_ControlMixer {
public:
    TR_ControlMixer();

    /// Configure the mixer with PID gains for pitch/yaw inner-loop rate controllers.
    /// max_fin_deflection_deg: absolute max per-fin deflection (deg)
    /// v_ref / v_min: gain-scheduling reference and minimum airspeed (m/s)
    void configure(float pitch_kp, float pitch_ki, float pitch_kd,
                   float yaw_kp,   float yaw_ki,   float yaw_kd,
                   float max_fin_deflection_deg,
                   float v_ref, float v_min);

    /// Full 3-axis cascaded control update.
    ///
    /// pitch_angle_cmd_deg  : desired pitch tilt from guidance (deg)
    /// yaw_angle_cmd_deg    : desired yaw tilt from guidance (deg)
    /// pitch_angle_actual_deg: current pitch angle from EKF (deg)
    /// yaw_angle_actual_deg  : current yaw angle from EKF (deg)
    /// pitch_rate_dps       : measured pitch rate, body-frame (deg/s)
    /// yaw_rate_dps         : measured yaw rate, body-frame (deg/s)
    /// roll_cmd_deg         : roll fin command from existing roll PID (deg)
    /// speed_mps            : current airspeed for gain scheduling (m/s)
    /// kp_pitch_angle       : outer-loop P gain for pitch (rate_cmd/angle_error)
    /// kp_yaw_angle         : outer-loop P gain for yaw
    /// dt                   : time step in seconds
    void update(float pitch_angle_cmd_deg,
                float yaw_angle_cmd_deg,
                float pitch_angle_actual_deg,
                float yaw_angle_actual_deg,
                float pitch_rate_dps,
                float yaw_rate_dps,
                float roll_cmd_deg,
                float speed_mps,
                float kp_pitch_angle,
                float kp_yaw_angle,
                float dt);

    /// Retrieve the 4 individual fin deflection angles (deg).
    /// Order: [0]=top, [1]=right, [2]=bottom, [3]=left (looking from rear).
    void getFinDeflections(float deflections[4]) const;

    /// Individual axis commands for telemetry (deg of fin deflection).
    float getPitchFinCmd() const { return pitch_fin_cmd_; }
    float getYawFinCmd()   const { return yaw_fin_cmd_; }

    /// Configure the fin→servo mix from per-servo fin-position azimuths (deg —
    /// the ring slot each fin occupies, NOT its force direction; the mixer maps
    /// position→tangential force internally), a tilt reverse bitmask (bit i ⇒
    /// negate servo i pitch/yaw response) and a roll reverse bitmask (bit i ⇒
    /// negate its roll response, independently).  Recomputes the pitch/yaw/roll
    /// mix coefficients used by both update() and mixToFins().  Defaults
    /// {0,90,180,270} + masks 0 reproduce the cruciform "+".
    void setFinLayout(const float azimuth_deg[4], uint8_t reverse_mask,
                      uint8_t roll_reverse_mask);

    /// Allocate a (roll,pitch,yaw) fin command to the 4 servos using the configured
    /// fin layout, each clamped to ±max_fin_deg.  Stateless allocation (no PID state) —
    /// the FC roll / ground-test / coast-guidance paths all call this so every mix
    /// site shares one layout.
    void mixToFins(float roll_cmd_deg, float pitch_fin_cmd, float yaw_fin_cmd,
                   float max_fin_deg, float out_deflections[4]) const;

    /// Flown fin layout — for the telemetry / flight-log snapshot.
    float   getFinAzimuthDeg(int i) const { return (i >= 0 && i < 4) ? fin_azimuth_deg_[i] : 0.0f; }
    uint8_t getFinReverseMask() const { return fin_reverse_mask_; }
    uint8_t getFinRollReverseMask() const { return fin_roll_reverse_mask_; }

    /// Reset all PID internal state.
    void reset();

    /// Enable/disable V^2 gain scheduling for pitch/yaw rate PIDs.
    void enableGainSchedule(float v_ref, float v_min);
    void disableGainSchedule();

    // Apply a 1-pole LP filter on the pitch/yaw D terms to reject gyro
    // measurement noise. See TR_PID::setDerivativeFilterCutoffHz.
    void setDerivativeFilterCutoffHz(float fc_hz);

private:
    TR_PID pitch_rate_pid_;
    TR_PID yaw_rate_pid_;

    float max_fin_deg_;
    float deflections_[4];
    float pitch_fin_cmd_;
    float yaw_fin_cmd_;

    // Base PID gains (unscaled)
    float pitch_kp_base_, pitch_ki_base_, pitch_kd_base_;
    float yaw_kp_base_,   yaw_ki_base_,   yaw_kd_base_;

    // Gain scheduling
    bool  gain_sched_enabled_;
    float v_ref_;
    float v_min_;
    float prev_gain_scale_;

    void applyGainSchedule(float speed_mps);

    // Cruciform mixing coefficients (control/FRD body frame, positions viewed
    // from the rear):
    //   deflection_i = pitch_mix_[i]·pitch + yaw_mix_[i]·yaw + roll_mix_[i]·roll
    // az_i is servo i's fin RING-POSITION azimuth (0 = top, 90 = right,
    // 180 = bottom, 270 = left).  A fin's tangential lift is perpendicular to
    // its radial arm — the top/bottom pair are yaw rudders, the right/left
    // pair pitch elevators — so setFinLayout() maps position → force:
    //   pitch_mix_[i]=tilt·sin(az_i), yaw_mix_[i]=tilt·cos(az_i) (tilt=-1 if
    // servo i tilt-reversed), roll_mix_[i]=roll (roll=-1 if servo i
    // roll-reversed) — tilt and roll signs are independent.  Default "+"
    // {0,90,180,270}: right/left carry pitch, top/bottom carry yaw.
    float   pitch_mix_[4] = {  0.0f, +1.0f,  0.0f, -1.0f };
    float   yaw_mix_[4]   = { +1.0f,  0.0f, -1.0f,  0.0f };
    float   roll_mix_[4]  = { +1.0f, +1.0f, +1.0f, +1.0f };
    float   fin_azimuth_deg_[4] = { 0.0f, 90.0f, 180.0f, 270.0f };
    uint8_t fin_reverse_mask_ = 0;
    uint8_t fin_roll_reverse_mask_ = 0;
};

#endif // TR_CONTROL_MIXER_H
