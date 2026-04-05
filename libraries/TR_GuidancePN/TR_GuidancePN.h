// TR_GuidancePN.h
// Proportional Navigation guidance for vertical station-keeping above the
// launch pad during coast phase.
//
// Implements textbook 3D PN (Yanushevsky Ch.2):
//   Eq 1.11: λ_dot_s = (R_dot_s·r - R_s·r_dot) / r²
//   Eq 2.23: a_cs = N · v_cl · λ_dot_s
//
// Outputs ENU acceleration commands.  The control layer is responsible
// for converting these to body-frame fin commands.

#ifndef TR_GUIDANCE_PN_H
#define TR_GUIDANCE_PN_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#include <cmath>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef constrain
#define constrain(x, lo, hi) std::clamp((x), (lo), (hi))
#endif
template<typename T> static inline T _tr_max(T a, T b) { return a > b ? a : b; }
#define max(a,b) _tr_max((a),(b))
#endif

class TR_GuidancePN {
public:
    TR_GuidancePN();

    /// Set all guidance parameters.
    /// nav_gain         : PN navigation constant N (dimensionless, 3-5 typical)
    /// max_accel_mps2   : maximum lateral acceleration command (m/s^2)
    /// target_alt_m     : target altitude above pad (m), typically above apogee
    void configure(float nav_gain,
                   float max_accel_mps2,
                   float target_alt_m);

    /// Main guidance update.  Call at EKF rate (~500-1200 Hz).
    ///
    /// pos_enu  : [E, N, U] position relative to launch pad (m)
    /// vel_ned  : [vN, vE, vD] velocity (m/s)  -- NOTE: NED order
    /// dt       : time step since last call (s)
    ///
    /// Returns true if guidance is producing valid commands.
    bool update(const float pos_enu[3],
                const float vel_ned[3],
                float dt);

    /// ENU acceleration commands (m/s^2) — primary output.
    float getAccelEastCmd()  const { return a_cmd_e_; }
    float getAccelNorthCmd() const { return a_cmd_n_; }
    float getAccelUpCmd()    const { return a_cmd_u_; }

    /// LOS angle between range vector and velocity vector (deg), for telemetry.
    float getLosAngleDeg() const { return los_angle_deg_; }

    /// Closing velocity (m/s), for telemetry.  Positive = approaching target.
    float getClosingVelocity() const { return v_cl_; }

    /// Range to target (m), for telemetry.
    float getRange() const { return range_m_; }

    /// Distance from pad vertical axis (m), for telemetry.
    float getLateralOffset() const { return lateral_offset_m_; }

    /// Whether guidance produced valid commands on the last update.
    bool isActive() const { return active_; }

    /// Whether closest point of approach has been reached (v_cl <= 0).
    bool isCpaReached() const { return cpa_reached_; }

    /// Reset all internal state.
    void reset();

private:
    // Configuration
    float N_;                   // navigation constant
    float max_accel_mps2_;
    float target_alt_m_;

    // Outputs
    float a_cmd_e_;
    float a_cmd_n_;
    float a_cmd_u_;
    float los_angle_deg_;
    float v_cl_;
    float range_m_;
    float lateral_offset_m_;
    bool  active_;
    bool  cpa_reached_;
};

#endif // TR_GUIDANCE_PN_H
