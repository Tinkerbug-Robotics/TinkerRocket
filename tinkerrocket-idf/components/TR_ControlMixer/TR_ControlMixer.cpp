#include "TR_ControlMixer.h"
#include <algorithm>
#include <cmath>

TR_ControlMixer::TR_ControlMixer()
    : pitch_rate_pid_(0.04f, 0.001f, 0.0003f, 15.0f, -15.0f),
      yaw_rate_pid_  (0.04f, 0.001f, 0.0003f, 15.0f, -15.0f),
      max_fin_deg_(15.0f),
      pitch_fin_cmd_(0.0f),
      yaw_fin_cmd_(0.0f),
      pitch_kp_base_(0.04f), pitch_ki_base_(0.001f), pitch_kd_base_(0.0003f),
      yaw_kp_base_(0.04f),   yaw_ki_base_(0.001f),   yaw_kd_base_(0.0003f),
      gain_sched_enabled_(false),
      v_ref_(95.0f),
      v_min_(30.0f),
      prev_gain_scale_(1.0f)
{
    for (int i = 0; i < 4; ++i) {
        deflections_[i] = 0.0f;
    }
}

void TR_ControlMixer::configure(float pitch_kp, float pitch_ki, float pitch_kd,
                                float yaw_kp,   float yaw_ki,   float yaw_kd,
                                float max_fin_deflection_deg,
                                float v_ref, float v_min)
{
    pitch_kp_base_ = pitch_kp;
    pitch_ki_base_ = pitch_ki;
    pitch_kd_base_ = pitch_kd;
    yaw_kp_base_   = yaw_kp;
    yaw_ki_base_   = yaw_ki;
    yaw_kd_base_   = yaw_kd;
    max_fin_deg_   = max_fin_deflection_deg;
    v_ref_         = v_ref;
    v_min_         = v_min;

    pitch_rate_pid_.setKp(pitch_kp);
    pitch_rate_pid_.setKi(pitch_ki);
    pitch_rate_pid_.setKd(pitch_kd);
    pitch_rate_pid_.setMinCmd(-max_fin_deflection_deg);
    pitch_rate_pid_.setMaxCmd( max_fin_deflection_deg);

    yaw_rate_pid_.setKp(yaw_kp);
    yaw_rate_pid_.setKi(yaw_ki);
    yaw_rate_pid_.setKd(yaw_kd);
    yaw_rate_pid_.setMinCmd(-max_fin_deflection_deg);
    yaw_rate_pid_.setMaxCmd( max_fin_deflection_deg);
}

void TR_ControlMixer::applyGainSchedule(float speed_mps)
{
    if (!gain_sched_enabled_) return;

    float v = std::max(fabsf(speed_mps), v_min_);
    float v_ratio = v_ref_ / v;
    float scale = v_ratio * v_ratio;
    scale = std::min(scale, 3.0f);  // cap to avoid excessive gains at low speed

    // Reset integral terms when gain scale changes significantly
    if (fabsf(scale - prev_gain_scale_) > 0.1f) {
        pitch_rate_pid_.resetIntegral();
        yaw_rate_pid_.resetIntegral();
    }
    prev_gain_scale_ = scale;

    pitch_rate_pid_.setKp(pitch_kp_base_ * scale);
    pitch_rate_pid_.setKi(pitch_ki_base_ * scale);
    pitch_rate_pid_.setKd(pitch_kd_base_ * scale);

    yaw_rate_pid_.setKp(yaw_kp_base_ * scale);
    yaw_rate_pid_.setKi(yaw_ki_base_ * scale);
    yaw_rate_pid_.setKd(yaw_kd_base_ * scale);
}

void TR_ControlMixer::update(float pitch_angle_cmd_deg,
                             float yaw_angle_cmd_deg,
                             float pitch_angle_actual_deg,
                             float yaw_angle_actual_deg,
                             float pitch_rate_dps,
                             float yaw_rate_dps,
                             float roll_cmd_deg,
                             float speed_mps,
                             float kp_pitch_angle,
                             float kp_yaw_angle,
                             float dt)
{
    // Apply gain scheduling if enabled
    applyGainSchedule(speed_mps);

    // --- Outer loop: angle error -> rate command (P-only) ---
    float pitch_angle_error = pitch_angle_cmd_deg - pitch_angle_actual_deg;
    float yaw_angle_error   = yaw_angle_cmd_deg   - yaw_angle_actual_deg;

    float pitch_rate_cmd = kp_pitch_angle * pitch_angle_error;  // deg/s
    float yaw_rate_cmd   = kp_yaw_angle   * yaw_angle_error;    // deg/s

    // --- Inner loop: rate PID -> fin deflection command ---
    // Use explicit-dt overload for platform independence
    pitch_fin_cmd_ = pitch_rate_pid_.computePID(pitch_rate_cmd, pitch_rate_dps, dt);
    yaw_fin_cmd_   = yaw_rate_pid_.computePID(yaw_rate_cmd,   yaw_rate_dps, dt);

    // --- Mix into 4 fin deflections ---
    for (int i = 0; i < 4; ++i) {
        float d = pitch_mix_[i] * pitch_fin_cmd_
                + yaw_mix_[i]   * yaw_fin_cmd_
                + roll_mix_[i]  * roll_cmd_deg;

        // Per-fin deflection clamp
        deflections_[i] = constrain(d, -max_fin_deg_, max_fin_deg_);
    }
}

void TR_ControlMixer::getFinDeflections(float deflections[4]) const
{
    for (int i = 0; i < 4; ++i) {
        deflections[i] = deflections_[i];
    }
}

void TR_ControlMixer::setFinLayout(const float azimuth_deg[4], uint8_t reverse_mask,
                                   uint8_t roll_reverse_mask)
{
    constexpr float DEG2RAD = 0.01745329252f;
    fin_reverse_mask_      = reverse_mask;
    fin_roll_reverse_mask_ = roll_reverse_mask;
    for (int i = 0; i < 4; ++i) {
        fin_azimuth_deg_[i] = azimuth_deg[i];
        float c = cosf(azimuth_deg[i] * DEG2RAD);
        float s = sinf(azimuth_deg[i] * DEG2RAD);
        if (fabsf(c) < 1e-6f) c = 0.0f;   // snap cardinal angles to exact 0 / ±1
        if (fabsf(s) < 1e-6f) s = 0.0f;
        // Position→moment mapping.  A deflected fin's lift is tangential
        // (perpendicular to its radial arm — that is what makes roll), and
        // crossing that force with the CG→fin axial arm puts the resulting
        // TRANSVERSE moment along the fin's own position direction p̂(az):
        //   r⃗ × F⃗ = (−L·x̂ + r·p̂) × (F·(x̂ × p̂)) = r·F·x̂ + L·F·p̂
        // (first term = roll, hence position-independent roll_mix).  So a
        // top/bottom fin torques about the vertical axis — a rudder, i.e.
        // yaw = cos(az) — and a side fin torques about the lateral axis — an
        // elevator, i.e. pitch = sin(az).  The pre-fix cos/sin order assumed
        // a fin's authority pointed along its own radius, which swapped the
        // pitch/yaw pairs: tilt response came out reflected about the 45°
        // diagonal, so no airframe rotation could cancel it (a reflection is
        // not a rotation) and ground tests read as non-corrective.
        // NOTE: this fixes the pair ASSIGNMENT.  The overall tilt sign still
        // depends on linkage/servo-direction conventions and is carried by
        // reverse_mask (flip all four bits to invert tilt globally) — confirm
        // on the bench with the app's per-fin jog.
        // Tilt (pitch/yaw) and roll signs are independent — see FinConfigData.
        float tilt_sign = (reverse_mask      & (1u << i)) ? -1.0f : 1.0f;
        float roll_sign = (roll_reverse_mask & (1u << i)) ? -1.0f : 1.0f;
        pitch_mix_[i] = tilt_sign * s;
        yaw_mix_[i]   = tilt_sign * c;
        roll_mix_[i]  = roll_sign;
    }
}

void TR_ControlMixer::mixToFins(float roll_cmd_deg, float pitch_fin_cmd, float yaw_fin_cmd,
                                float max_fin_deg, float out_deflections[4]) const
{
    for (int i = 0; i < 4; ++i) {
        float d = pitch_mix_[i] * pitch_fin_cmd
                + yaw_mix_[i]   * yaw_fin_cmd
                + roll_mix_[i]  * roll_cmd_deg;
        out_deflections[i] = constrain(d, -max_fin_deg, max_fin_deg);
    }
}

void TR_ControlMixer::reset()
{
    pitch_rate_pid_.reset();
    yaw_rate_pid_.reset();
    pitch_fin_cmd_ = 0.0f;
    yaw_fin_cmd_   = 0.0f;
    prev_gain_scale_ = 1.0f;
    for (int i = 0; i < 4; ++i) {
        deflections_[i] = 0.0f;
    }
}

void TR_ControlMixer::enableGainSchedule(float v_ref, float v_min)
{
    gain_sched_enabled_ = true;
    v_ref_ = v_ref;
    v_min_ = v_min;
}

void TR_ControlMixer::disableGainSchedule()
{
    gain_sched_enabled_ = false;
    // Restore base gains
    pitch_rate_pid_.setKp(pitch_kp_base_);
    pitch_rate_pid_.setKi(pitch_ki_base_);
    pitch_rate_pid_.setKd(pitch_kd_base_);
    yaw_rate_pid_.setKp(yaw_kp_base_);
    yaw_rate_pid_.setKi(yaw_ki_base_);
    yaw_rate_pid_.setKd(yaw_kd_base_);
}

void TR_ControlMixer::setDerivativeFilterCutoffHz(float fc_hz)
{
    pitch_rate_pid_.setDerivativeFilterCutoffHz(fc_hz);
    yaw_rate_pid_.setDerivativeFilterCutoffHz(fc_hz);
}
