// RawMeas.h — a receiver epoch's raw observations, and the corrected
// per-satellite measurements a tightly coupled GNSS/INS filter takes (#1600)
//
// RawEpoch is what any receiver reports, in its own words but SI units: per
// satellite, the pseudorange, the range rate and C/N0. (The LC86G's come from
// RTCM MSM7: Lc86RawEpoch.h in its driver fills one.) RawMeasBuilder turns
// an epoch into RawMeas, corrected so the filter only adds its own receiver
// state: the satellite placed at the transmit time, its clock, relativity
// and group delay, and the ionosphere and troposphere taken out, with each
// measurement's noise from its C/N0. As the bench prototype it mirrors
// (gnss repo sim/tinkerrocket_gnss/gnss_raw.py, RawMeas in tc_ekf.py).
#ifndef GNSS_RAW_MEAS_H
#define GNSS_RAW_MEAS_H

#include <cstdint>

namespace gnss_raw
{

// One satellite's signal as the receiver reported it.
struct RawObs
{
    uint8_t  sys;         // gnss_sys_t: GPS, Galileo, BeiDou
    uint8_t  prn;
    bool     has_pr;
    bool     has_rate;
    float    cn0_dbhz;
    double   pr_m;        // pseudorange, m
    double   rate_mps;    // range rate, m/s (the MSM7 phase-range rate; = -lambda * Doppler)
    uint32_t lock_ms;     // the receiver's minimum lock time, ms (0 if it gives none)
};

// One receiver epoch: every satellite measured at one GPS instant.
struct RawEpoch
{
    static constexpr int kMaxObs = 48;
    double tow_s = 0.0;   // GPS time of week of the epoch, s
    int    n = 0;
    RawObs obs[kMaxObs] = {};
};

// An epoch as a receiver driver hands it to a raw-GNSS filter: with its GPS
// instant placed on the MCU clock (the LC86G driver's GnssTimeSync), so the
// filter can apply it at its own time however late it arrives.
struct TimedRawEpoch
{
    RawEpoch epoch;
    int64_t  mcu_us = 0;      // the epoch's GPS instant on the MCU clock (esp_timer, us); 0 = not known yet
    int64_t  arrival_us = 0;  // when its last frame arrived (MCU, us)
    uint16_t gps_week = 0;    // its full GPS week; 0 = not known yet
};

// One satellite at one epoch, corrected (see above). sat_pos is the
// satellite at the transmit time in the ECEF frame of that instant; the
// filter turns it by the Earth's rotation over the signal's travel (Sagnac).
struct RawMeas
{
    uint8_t sys;          // gnss_sys_t
    uint8_t prn;
    bool    has_pr;
    bool    has_rr;
    double  sat_pos[3];   // m
    double  sat_vel[3];   // m/s
    double  sat_acc[3];   // m/s^2, to predict a late Doppler
    double  pr;           // m, with the receiver clock still in it
    double  rr;           // m/s, with the receiver clock drift still in it
    float   sigma_pr;     // white part, m
    float   sigma_pr_corr;// time-correlated part (first-order Gauss-Markov), m
    float   tau_pr;       // its correlation time, s
    float   sigma_rr;     // m/s
    float   tau_rr;       // s
};

}  // namespace gnss_raw

#endif  // GNSS_RAW_MEAS_H
