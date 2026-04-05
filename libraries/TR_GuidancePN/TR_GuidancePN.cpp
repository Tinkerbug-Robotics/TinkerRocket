#include "TR_GuidancePN.h"
#include <math.h>

TR_GuidancePN::TR_GuidancePN()
    : N_(3.0f),
      max_accel_mps2_(20.0f),
      target_alt_m_(600.0f),
      a_cmd_e_(0.0f),
      a_cmd_n_(0.0f),
      a_cmd_u_(0.0f),
      los_angle_deg_(0.0f),
      v_cl_(0.0f),
      range_m_(0.0f),
      lateral_offset_m_(0.0f),
      active_(false),
      cpa_reached_(false)
{
}

void TR_GuidancePN::configure(float nav_gain,
                               float max_accel_mps2,
                               float target_alt_m)
{
    N_              = nav_gain;
    max_accel_mps2_ = max_accel_mps2;
    target_alt_m_   = target_alt_m;
}

bool TR_GuidancePN::update(const float pos_enu[3],
                            const float vel_ned[3],
                            float dt)
{
    (void)dt;  // Not needed for textbook PN (no filtering)

    // Extract position ENU
    float pos_E = pos_enu[0];
    float pos_N = pos_enu[1];
    float pos_U = pos_enu[2];

    // Lateral offset from pad vertical axis
    lateral_offset_m_ = sqrtf(pos_E * pos_E + pos_N * pos_N);

    // --- Range vector R (ENU): target - missile ---
    // Target is at (0, 0, target_alt_m_) in ENU
    float R_e = 0.0f - pos_E;
    float R_n = 0.0f - pos_N;
    float R_u = target_alt_m_ - pos_U;
    float r = sqrtf(R_e * R_e + R_n * R_n + R_u * R_u);
    range_m_ = r;

    if (r < 1.0f) {
        // Too close to target
        a_cmd_e_ = 0.0f;
        a_cmd_n_ = 0.0f;
        a_cmd_u_ = 0.0f;
        active_ = false;
        return false;
    }

    // --- Velocity in ENU (convert from NED input) ---
    float vel_e = vel_ned[1];          // ENU east = NED east
    float vel_n = vel_ned[0];          // ENU north = NED north
    float vel_u = -vel_ned[2];         // ENU up = -NED down

    // --- Range rate R_dot (ENU) ---
    // Target is stationary, so R_dot = -V_missile
    float R_dot_e = -vel_e;
    float R_dot_n = -vel_n;
    float R_dot_u = -vel_u;

    // --- Scalar range rate r_dot = (R · R_dot) / r ---
    float r_dot = (R_e * R_dot_e + R_n * R_dot_n + R_u * R_dot_u) / r;

    // --- Closing velocity v_cl = -r_dot ---
    v_cl_ = -r_dot;

    // Flag closest point of approach
    if (v_cl_ <= 0.0f) {
        cpa_reached_ = true;
    }

    // --- LOS angle: angle between range vector R and velocity vector V ---
    float V_mag = sqrtf(vel_e * vel_e + vel_n * vel_n + vel_u * vel_u);
    if (V_mag > 1.0f) {
        float cos_los = (R_e * vel_e + R_n * vel_n + R_u * vel_u) / (r * V_mag);
        if (cos_los > 1.0f) cos_los = 1.0f;
        if (cos_los < -1.0f) cos_los = -1.0f;
        los_angle_deg_ = acosf(cos_los) * (180.0f / (float)M_PI);
    } else {
        los_angle_deg_ = 0.0f;
    }

    // --- Per-axis LOS rates (eq 1.11) ---
    // λ_dot_s = (R_dot_s · r - R_s · r_dot) / r²
    float r_sq = r * r;
    float lam_dot_e = (R_dot_e * r - R_e * r_dot) / r_sq;
    float lam_dot_n = (R_dot_n * r - R_n * r_dot) / r_sq;
    float lam_dot_u = (R_dot_u * r - R_u * r_dot) / r_sq;

    // --- PN acceleration commands (eq 2.23) ---
    // a_cs = N · v_cl · λ_dot_s
    a_cmd_e_ = N_ * v_cl_ * lam_dot_e;
    a_cmd_n_ = N_ * v_cl_ * lam_dot_n;
    a_cmd_u_ = N_ * v_cl_ * lam_dot_u;

    // Clamp total acceleration magnitude
    float a_mag = sqrtf(a_cmd_e_ * a_cmd_e_ + a_cmd_n_ * a_cmd_n_ + a_cmd_u_ * a_cmd_u_);
    if (a_mag > max_accel_mps2_ && a_mag > 0.0f) {
        float scale = max_accel_mps2_ / a_mag;
        a_cmd_e_ *= scale;
        a_cmd_n_ *= scale;
        a_cmd_u_ *= scale;
    }

    active_ = true;
    return true;
}

void TR_GuidancePN::reset()
{
    a_cmd_e_         = 0.0f;
    a_cmd_n_         = 0.0f;
    a_cmd_u_         = 0.0f;
    los_angle_deg_   = 0.0f;
    v_cl_            = 0.0f;
    range_m_         = 0.0f;
    lateral_offset_m_ = 0.0f;
    active_          = false;
    cpa_reached_     = false;
}
