// RawMeas.h — a receiver epoch's raw observations, and the corrected
// per-satellite measurements a tightly coupled GNSS/INS filter takes (#1600)
//
// RawEpoch is what any receiver reports, in its own words but SI units: per
// satellite, the pseudorange, the range rate, the carrier phase and C/N0, and
// the lock time and half-cycle flag that say whether the phase is continuous
// from the epoch before. (The LC86G's come from
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
    bool     has_phase;   // a carrier phase (phase_m)
    bool     halfcyc;     // its half-cycle ambiguity unresolved (MSM7 DF420): it can step by half a cycle
    double   phase_m;     // the carrier phase as a range, m (MSM7: the rough range DF397 + DF398 plus the
                          // fine phase range DF406). It carries the receiver's own integer per lock: its
                          // changes are the measurement (RawMeasBuilder's carrier range rate)
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

// The receiver's own fix as a driver hands it to the raw-GNSS shadow, which
// checks it against the IMU (FixCheck.h, #1508): with its GPS instant on the
// MCU clock, so it can be set beside the IMU's motion at that instant.
struct TimedFix
{
    double  lat_rad = 0.0, lon_rad = 0.0;
    double  alt_m = 0.0;          // above the ellipsoid (the receiver's altitude plus its geoid separation)
    float   vel_ned[3] = {};      // m/s
    uint8_t fix_mode = 0;         // as GNSSData: 3 = a 3-D fix
    uint8_t num_sats = 0;
    int64_t mcu_us = 0;           // its GPS instant on the MCU clock (esp_timer, us); 0 = not known yet
    int64_t arrival_us = 0;       // when its sentence's last byte arrived (MCU, us)
    double  gps_s = 0.0;          // its GPS time, week * 604800 + time of week (s); 0 = not known
};

// One satellite at one epoch, corrected (see above). sat_pos is the
// satellite at the transmit time in the ECEF frame of that instant; the
// filter turns it by the Earth's rotation over the signal's travel (Sagnac).
//
// The carrier range rate (has_cr): the satellite's carrier phase range since
// the epoch before, cr_dphi_m over cr_dt, both ends corrected as its
// pseudorange is (the satellite clock, and with a receiver position the
// ionosphere -- the phase's advance -- and the troposphere). sat_pos_prev is
// the satellite at the earlier epoch's transmit time, so the filter predicts
// the interval's range change from the two geometries (TR_RawGnssEKF's
// carrier update). It carries the receiver's clock change over the interval:
// the filter takes that as a common rate of the epoch.
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
    bool    has_cr;       // a carrier range rate (RawMeasBuilder's gates passed at both ends)
    float   sigma_cr;     // its 1-sigma, m/s: both ends' phase noise over the interval, floored
    float   cr_dt;        // the interval, s
    double  cr_dphi_m;    // the corrected phase range's change over it, m
    double  sat_pos_prev[3];   // the satellite at the earlier epoch's transmit time, m (ECEF of that instant)
};

}  // namespace gnss_raw

#endif  // GNSS_RAW_MEAS_H
