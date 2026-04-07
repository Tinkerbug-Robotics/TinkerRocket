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

    /// Reset all PID internal state.
    void reset();

    /// Enable/disable V^2 gain scheduling for pitch/yaw rate PIDs.
    void enableGainSchedule(float v_ref, float v_min);
    void disableGainSchedule();

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

    // Cruciform mixing coefficients (looking from rear, FRD body frame):
    //   Servo 0 (top/0deg)    -> pitch +1, yaw  0, roll +1
    //   Servo 1 (right/90deg) -> pitch  0, yaw +1, roll +1
    //   Servo 2 (bottom/180deg)-> pitch -1, yaw  0, roll +1
    //   Servo 3 (left/270deg) -> pitch  0, yaw -1, roll +1
    static constexpr float PITCH_MIX[4] = { +1.0f,  0.0f, -1.0f,  0.0f };
    static constexpr float YAW_MIX[4]   = {  0.0f, +1.0f,  0.0f, -1.0f };
    static constexpr float ROLL_MIX[4]  = { +1.0f, +1.0f, +1.0f, +1.0f };
};

#endif // TR_CONTROL_MIXER_H
