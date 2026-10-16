// RawGnssEKFGeometry.h -- TR_RawGnssEKF's per-satellite geometry, the one
// definition its two sources share: TR_RawGnssEKF.cpp (each step and update,
// in internal RAM on the Beetle) and TR_RawGnssCarrier.cpp (once an epoch: the
// carrier's screen and fusion, and the boost screen, kept in flash --
// raw_gnss_iram.lf). The range, range rate and line of sight every update
// predicts, the receiver taken back to an epoch, and a correlated error's
// variance factor. Internal: not part of the filter's interface. Each source
// gets its own copies (an unnamed namespace, as they were in
// TR_RawGnssEKF.cpp): the per-step code keeps its own in internal RAM, inlined
// or not as the compiler chose before, and the once-an-epoch code its own in
// flash.
#ifndef TR_RAW_GNSS_EKF_GEOMETRY_H
#define TR_RAW_GNSS_EKF_GEOMETRY_H

#include <cmath>

#include "RawMeas.h"

namespace gnss_raw
{
namespace ekf_geom
{
namespace
{

constexpr double kC      = 299792458.0;
constexpr double kOmegaE = 7.2921151467e-5;     // rad/s, the Sagnac turn

// The predicted range (without the receiver clock), range rate (without its
// drift) and line of sight: the satellite at transmit time, turned by the
// Earth's rotation over the travel time (tc_ekf.py _los).
struct Los
{
    double rho, rate, u[3], vs[3];
};

void lineOfSight(const double rs[3], const double vs[3], const double r[3], const double v[3], Los& L)
{
    double d0 = rs[0] - r[0], d1 = rs[1] - r[1], d2 = rs[2] - r[2];
    double rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    double c = 1.0, s = 0.0, rr[3] = {rs[0], rs[1], rs[2]};
    for (int it = 0; it < 2; ++it)
    {
        // the Earth's turn over the travel time, under 1e-4 rad: two terms
        // each give its cosine and sine to double precision, where libm's
        // (software doubles on the ESP32-S3) cost 4% of a satellite's update
        const double th = kOmegaE * rho / kC;
        const double th2 = th * th;
        c = 1.0 - 0.5 * th2;
        s = th * (1.0 - th2 / 6.0);
        rr[0] = c * rs[0] + s * rs[1];
        rr[1] = -s * rs[0] + c * rs[1];
        rr[2] = rs[2];
        d0 = rr[0] - r[0]; d1 = rr[1] - r[1]; d2 = rr[2] - r[2];
        rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
    }
    L.vs[0] = c * vs[0] + s * vs[1];
    L.vs[1] = -s * vs[0] + c * vs[1];
    L.vs[2] = vs[2];
    L.rho = rho;
    L.u[0] = d0 / rho; L.u[1] = d1 / rho; L.u[2] = d2 / rho;
    L.rate = L.u[0] * (L.vs[0] - v[0]) + L.u[1] * (L.vs[1] - v[1]) + L.u[2] * (L.vs[2] - v[2]);
}

void lineOfSight(const RawMeas& m, const double r[3], const double v[3], Los& L)
{
    lineOfSight(m.sat_pos, m.sat_vel, r, v, L);
}

// The range acceleration a still receiver sees from the satellite's own
// motion: along the line of sight, plus the line of sight turning.
double satRangeAccel(const RawMeas& m, const Los& L, const double v[3])
{
    double dvv = 0.0, ua = 0.0;
    for (int k = 0; k < 3; ++k)
    {
        const double dv = L.vs[k] - v[k];
        dvv += dv * dv;
        ua += L.u[k] * m.sat_acc[k];
    }
    return ua + (dvv - L.rate * L.rate) / L.rho;
}

// Variance factor for a first-order Gauss-Markov error sampled every dt and
// fused as if white: (1 + rho) / (1 - rho), rho = exp(-dt / tau).
double corrInflation(double dt, double tau)
{
    if (tau <= 0.0 || dt <= 0.0)
        return 1.0;
    const double rho = std::exp(-dt / tau);
    return rho < 1.0 - 1e-12 ? (1.0 + rho) / (1.0 - rho) : 1e12;
}

// The receiver at the epoch: now less the NED displacement and velocity
// change since (both NED, turned to ECEF by the present frame).
template <typename T>
void backDate(const double r[3], const double v[3], const double Tm[3][3], const T dp[3], const T dv[3],
              double r_e[3], double v_e[3])
{
    for (int i = 0; i < 3; ++i)
    {
        double sp = 0.0, sv = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            sp += Tm[k][i] * static_cast<double>(dp[k]);
            sv += Tm[k][i] * static_cast<double>(dv[k]);
        }
        r_e[i] = r[i] - sp;
        v_e[i] = v[i] - sv;
    }
}

}  // namespace
}  // namespace ekf_geom
}  // namespace gnss_raw

#endif  // TR_RAW_GNSS_EKF_GEOMETRY_H
