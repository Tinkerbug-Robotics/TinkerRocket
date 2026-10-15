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
// on the COCOM rig (2026-10-12) -- each of them, and the noise model itself,
// the receiver's profile's (ReceiverProfile.h; the config's defaults are the
// LC86G's):
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
// The carrier range rate (config carrier, the profile's cr_mode not off). For
// each satellite with a phase the builder keeps its end of the epoch -- the
// time, the phase range corrected as the pseudorange is (the satellite clock;
// with a receiver position the ionosphere, which advances the phase, and the
// troposphere, which a climbing rocket leaves at up to a decimetre a second),
// lock time, C/N0, half-cycle flag, and the satellite where it transmitted --
// and the next epoch built differences its own against it (RawMeas has_cr,
// cr_dphi_m, cr_dt, sat_pos_prev, sigma_cr) when, at both ends:
//   - the lock time has not gone back, is at least cr_min_lock_ms, and the
//     later end's covers the interval (no relock between them);
//   - the half-cycle flag is clear;
//   - C/N0 is at or above the system's carrier floor;
//   - the corrections were the same (a position at both, one ionosphere);
// and the interval is more than half epoch_period_s and under
// cr_max_interval_s. A satellite that is missing, relocks or is not built in
// an epoch starts over. sigma_cr: both ends' phase sigmas (phaseSigmaM at
// their C/N0) in quadrature over the interval, at least cr_sigma_floor_mps;
// the filter adds its roll-rate term from the gyro. The ends are the last
// epoch built's satellites, up to an epoch's worth (RawEpoch::kMaxObs).
//
// Double precision: once per satellite per epoch (1 Hz on the LC86G). Pure
// and host-compilable (tests_cpp/test_raw_gnss_meas.cpp).
#ifndef RAW_MEAS_BUILDER_H
#define RAW_MEAS_BUILDER_H

#include "EphemerisStore.h"
#include "RawMeas.h"
#include "ReceiverProfile.h"
#include "gnss/eph.h"

namespace gnss_raw
{

class RawMeasBuilder
{
public:
    // The error model's fit (sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10)): the
    // config's defaults, the LC86G profile's.
    static constexpr double kPrWhiteA = 0.026, kPrWhiteB = 0.157;   // m
    static constexpr double kPrCorrA  = 1.94,  kPrCorrB  = 3.5;     // m
    static constexpr double kPrTauS   = 15.0;
    static constexpr double kRrWhiteA = 0.0,   kRrWhiteB = 0.2;     // m/s
    static constexpr double kRrTauS   = 0.1;

    // The receiver's settings are its profile's (ReceiverProfile.h,
    // applyReceiverProfile); these defaults are the LC86G's.
    struct Config
    {
        double el_mask_deg     = 3.0;
        bool   troposphere     = true;    // Saastamoinen
        // each system's signal (gnss_sys_t): its carrier and C/N0 floors, its
        // carrier phase's noise
        GnssSignalProfile sig[kProfileSys] = {
            {GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true},    // GPS L1 C/A
            {GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 35.0f, 0.0024f, 0.0011f, false},   // Galileo E1
            {1561.098e6, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true},         // BeiDou B1I
            {GNSS_FREQ_L1_HZ, 0.0f, 0.0f, 32.5f, 0.0045f, 0.0014f, true}};   // QZSS L1 C/A
        double pr_white_a      = kPrWhiteA, pr_white_b = kPrWhiteB;   // the noise model's fit
        double pr_corr_a       = kPrCorrA, pr_corr_b = kPrCorrB;
        double pr_tau_s        = kPrTauS;
        double rr_white_a      = kRrWhiteA, rr_white_b = kRrWhiteB;
        double rr_tau_s        = kRrTauS;
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
        // The carrier range rate (below): built unless the profile's mode is
        // off, between epochs epoch_period_s apart (more than half that, at
        // most cr_max_interval_s), each end locked cr_min_lock_ms or more
        bool     carrier            = true;
        uint32_t cr_min_lock_ms     = 1000;
        double   epoch_period_s     = 1.0;
        double   cr_max_interval_s  = 1.5;
        double   cr_sigma_floor_mps = 0.008;
    };

    RawMeasBuilder() { reset(); }
    explicit RawMeasBuilder(const Config& cfg) : cfg_(cfg) { reset(); }

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // The GPS broadcast (Klobuchar) ionosphere. Until it is set, only the
    // model's night-time 5 ns is taken out. (A carrier range rate does not
    // span a change of it.)
    void setIono(const gps_iono_t& io)
    {
        iono_ = io;
        iono_gen_ = static_cast<uint8_t>((iono_gen_ + 1) & 0x7F);
    }
    const gps_iono_t& iono() const { return iono_; }

    // The parameters from the 8 bytes GPS broadcasts them in (IS-GPS-200
    // 20.3.3.5.1.7: alpha0..3 then beta0..3, two's complement, at 2^-30,
    // 2^-27, 2^-24, 2^-24 s/semicircle^n and 2^11, 2^14, 2^16, 2^16 s/semicircle^n).
    static gps_iono_t ionoFromBroadcast(const int8_t b[8])
    {
        static constexpr double kScale[8] = {
            9.313225746154785e-10, 7.450580596923828e-09, 5.960464477539063e-08, 5.960464477539063e-08,
            2048.0, 16384.0, 65536.0, 65536.0};
        gps_iono_t io = {};
        io.valid = 1;
        for (int k = 0; k < 4; ++k)
        {
            io.alpha[k] = b[k] * kScale[k];
            io.beta[k] = b[4 + k] * kScale[4 + k];
        }
        return io;
    }

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
    // The same in parts, for a filter that builds a sky between its steps:
    // begin() takes the epoch (its satellites' relock ages, the receiver, the
    // g peak), then next() goes on from where it stopped through at most
    // max_obs of its observations, building into out[n..cap); it returns the
    // new n. done() once it has been through every observation. next() is
    // given the same epoch each time; r_approx is copied at begin.
    void begin(const RawEpoch& ep, const double* r_approx, double g_peak);
    int  next(const RawEpoch& ep, const EphemerisStore& eph, RawMeas* out, int n, int cap, int max_obs);
    bool done(const RawEpoch& ep) const { return next_obs_ >= ep.n; }

    // Seconds since the satellite last (re)appeared, at GPS time of week tow.
    double relockAge(int sys, int prn, double tow) const;

    // Take the satellite as relocked at tow, as if it had just reappeared: its
    // pseudoranges from then on carry the relock sigma (a jump the shadow's
    // SatJumpCheck saw, #1508).
    void markRelock(int sys, int prn, double tow);

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

    // A satellite's end of a carrier range rate (above): its carrier phase at
    // one epoch.
    struct CarrierEnd
    {
        uint8_t  sys = 0, prn = 0;
        bool     halfcyc = false;
        uint8_t  model = 0;          // the corrections it took (atmosphere, ionosphere's generation)
        float    cn0 = 0.0f;
        float    sigma_m = 0.0f;     // its phase's 1-sigma, m
        uint32_t lock_ms = 0;
        double   tow = 0.0;
        double   phase = 0.0;        // the phase range, corrected, m
        double   sat_pos[3] = {};    // the satellite at its transmit time
    };
    void carrierRate(const RawObs& o, const GnssSignalProfile& sig, double tow, double phase,
                     const double rs[3], RawMeas& m);

    Config     cfg_;
    gps_iono_t iono_ = {};
    SatState   cache_[kSatCache];
    uint32_t   cache_tick_ = 0;
    // the epoch being built (begin, next)
    int        next_obs_ = 0;
    bool       have_pos_ = false;
    double     r_approx_[3] = {};
    double     lat_ = 0.0, lon_ = 0.0, hgt_ = 0.0;
    double     el_mask_ = 0.0, g_over_ = 0.0;
    double     first_[kSys][kPrn];
    double     last_[kSys][kPrn];
    bool       seen_[kSys][kPrn];
    // the carrier: the last epoch built's ends and the one being built's
    // (cr_side_ the last's), its corrections' model, setIono's generation
    CarrierEnd cr_end_[2][RawEpoch::kMaxObs];
    int        cr_n_[2] = {0, 0};
    int        cr_side_ = 0;
    bool       cr_done_ = true;
    uint8_t    cr_model_ = 0;
    uint8_t    iono_gen_ = 0;
};

}  // namespace gnss_raw

#endif  // RAW_MEAS_BUILDER_H
