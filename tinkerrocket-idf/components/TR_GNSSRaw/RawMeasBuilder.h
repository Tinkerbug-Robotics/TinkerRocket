// RawMeasBuilder.h — a receiver epoch into the corrected measurements a
// tightly coupled filter takes (#1600)
//
// For each satellite with a pseudorange and a healthy ephemeris within two
// hours: the transmit time from the pseudorange and the satellite clock; the
// satellite's position there, and its velocity and acceleration from
// positions half a second either side; the satellite clock (with relativity
// and the signal's group delay) added back to the pseudorange; with a
// receiver position, the broadcast ionosphere (scaled to the signal's
// frequency) and the Saastamoinen troposphere taken out and satellites under
// the elevation mask dropped; the range rate less the satellite clock drift.
//
// Noise per measurement, from C/N0 (a PX1105R, 20 min static, 2026-09-26):
// a white and a time-correlated pseudorange part and a white range-rate part,
// sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10). Then the LC86G's own, measured
// on the COCOM rig (2026-10-12):
//   - its correlated pseudorange error is half the PX1105R's (pr_corr_scale);
//   - a satellite missing for more than relock_gap_s has relocked, and its
//     pseudorange starts biased: relock_sigma_m more, decaying over
//     relock_tau_s;
//   - a tracking loop under stress gives a poor Doppler: rr_dyn_sigma per g
//     of specific force above 1 g (the largest since the last epoch, which
//     the caller passes as g_peak), and rr_relock_sigma at a relock.
// All of it as the bench prototype that scored the filter (gnss repo
// sim/tinkerrocket_gnss/gnss_raw.py corrected() and sim/tc_ekf_cocom.py
// meas_at()).
//
// Double precision: once per satellite per epoch (1 Hz on the LC86G). Pure
// and host-compilable (tests_cpp/test_raw_gnss_meas.cpp).
#ifndef RAW_MEAS_BUILDER_H
#define RAW_MEAS_BUILDER_H

#include "EphemerisStore.h"
#include "RawMeas.h"
#include "gnss/eph.h"

namespace gnss_raw
{

class RawMeasBuilder
{
public:
    struct Config
    {
        double el_mask_deg     = 3.0;
        bool   troposphere     = true;    // Saastamoinen
        double doppler_step_hz = 0.0;     // a receiver's Doppler resolution (SkyTraq 1 Hz), whose rounding joins sigma_rr
        double pr_corr_scale   = 0.5;     // LC86G
        double rr_scale        = 1.0;
        double relock_sigma_m  = 40.0;    // 0 = off
        double relock_tau_s    = 10.0;
        double relock_gap_s    = 2.5;     // 2.5 epochs at the LC86G's 1 Hz
        double rr_dyn_sigma    = 0.5;     // m/s per g above 1 g
        double rr_relock_sigma = 3.0;     // m/s
        // Each satellite's orbit is solved exactly at most this often and
        // carried between by its velocity and acceleration (0: exactly at
        // every epoch). Over 5 s that is 2 mm and 1 mm/s; it cuts what the
        // satellites cost the flight computer several fold.
        double sat_refresh_s   = 5.0;
    };

    // The error model's fit (sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10)).
    static constexpr double kPrWhiteA = 0.026, kPrWhiteB = 0.157;   // m
    static constexpr double kPrCorrA  = 1.94,  kPrCorrB  = 3.5;     // m
    static constexpr double kPrTauS   = 15.0;
    static constexpr double kRrWhiteA = 0.0,   kRrWhiteB = 0.2;     // m/s
    static constexpr double kRrTauS   = 0.1;

    RawMeasBuilder() { reset(); }
    explicit RawMeasBuilder(const Config& cfg) : cfg_(cfg) { reset(); }

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // The GPS broadcast (Klobuchar) ionosphere. Until it is set, only the
    // model's night-time 5 ns is taken out.
    void setIono(const gps_iono_t& io) { iono_ = io; }
    const gps_iono_t& iono() const { return iono_; }

    // Forget every satellite's lock history.
    void reset();

    // Note which satellites the epoch has, for their relock age, without
    // building anything (build() does this itself).
    void noteEpoch(const RawEpoch& ep);

    // The corrected measurements of one epoch into out[0..cap), how many.
    // r_approx: the receiver, ECEF m, for the atmosphere and the elevation
    // mask (nullptr, or not yet a position on the Earth: neither is applied).
    // g_peak: the largest |specific force| / g - 1 since the last epoch.
    int build(const RawEpoch& ep, const EphemerisStore& eph, const double* r_approx,
              double g_peak, RawMeas* out, int cap);

    // Seconds since the satellite last (re)appeared, at GPS time of week tow.
    double relockAge(int sys, int prn, double tow) const;

private:
    static constexpr int kSys = 3;
    static constexpr int kPrn = 64;

    // A satellite's state at one instant, from one ephemeris.
    struct SatState
    {
        uint8_t  sys = 0, prn = 0;
        bool     valid = false;
        int      iode = 0;
        double   toe = 0.0;
        double   t_ref = 0.0;        // the system time it is at (a transmit time)
        double   pos[3] = {}, vel[3] = {}, acc[3] = {};
        double   clk = 0.0;          // s: with relativity, less the group delay
        double   clk_rate = 0.0;     // s/s
        uint32_t used = 0;           // for replacement, least recently used first
    };
    static constexpr int kSatCache = 40;
    const SatState& satState(const gps_eph_t& e, int sys, int prn, double t_sv);

    Config     cfg_;
    gps_iono_t iono_ = {};
    SatState   cache_[kSatCache];
    uint32_t   cache_tick_ = 0;
    double     first_[kSys][kPrn];
    double     last_[kSys][kPrn];
    bool       seen_[kSys][kPrn];
};

}  // namespace gnss_raw

#endif  // RAW_MEAS_BUILDER_H
