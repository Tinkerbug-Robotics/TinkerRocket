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
//
// The cheaper build (Config::lite; off unless the receiver profile's
// build_lite sets it). A receiver fused at 10 Hz with its pseudorange once a
// second (the mosaic-G5) pays for each satellite's elevation, ionosphere and
// troposphere on every epoch, though the Doppler takes none of them and the
// code is fused on one epoch a second. With lite, in RawMeasBuilderLite.cpp:
//   - each satellite's elevation, azimuth and ionosphere are worked out on
//     the first epoch built in each atmo_period_s of GPS time (the profile's
//     pr_period_s: the shadow fuses the code on the first epoch of each such
//     period, RawGnssShadow's pr_period_s) and kept for the period's other
//     epochs, whose code feeds only the jump check (20 m), the clock's step
//     test and the pad survey (and the fusion, where the period's first epoch
//     was not fused); the troposphere is this epoch's receiver's, mapped by
//     the kept elevation's sine carried on at the rate the satellite's own
//     motion gave it then. (A satellite's elevation changes up to 0.008 deg
//     a second, which near the 3 deg mask moves the mapped troposphere up to
//     8 cm in 0.9 s. Carried on, the code is within 3 mm of the full build's
//     at every elevation for a static receiver; the receiver's own motion,
//     not carried, adds the rest: at 300 m/s, at most 1.1 cm climbing
//     straight up and 3.2 cm 45 deg off the vertical, both at 3-5 deg, and
//     1.3 cm above 5 deg.) The elevation mask goes by the kept elevation. A
//     satellite with a carrier phase is worked out every epoch where the
//     carrier range rate is built, and a new ionosphere is taken at once;
//   - each satellite's 5 s orbit solve (sat_refresh_s) falls on a phase of its
//     own (its cache slot's), so the solves spread over the 5 s instead of
//     landing on one epoch. Where every satellite comes due at once -- they
//     return after more than sat_refresh_s unbuilt (a sky lost in the boost
//     and found again in the coast, the chute's empty sky), or a new GPS
//     ephemeris for all of them -- at most lite_solves are solved an epoch
//     once the caller's filter has taken its clock (beginWith's spread): in
//     the cap's order, GPS first, then by C/N0. A satellite whose solve waits
//     is left out of the epoch (waiting(): the cap holds its place), or, where
//     its last solve is within sat_refresh_s, built from that as before;
//   - the C/N0 noise takes one exp where it took three pows, kept with the
//     satellite while its C/N0 stays the same (three epochs in four on the
//     mosaic's 0.25 dB-Hz steps), and the relock terms no exp once a
//     satellite has held its lock for 40 relock_tau_s;
//   - the receiver's latitude, longitude and height come from a caller that
//     has them (beginWith: the GNSS task's stage has the filter's), and the
//     trigonometry and the troposphere's receiver part once an epoch.
// The epoch the atmosphere is worked out on gets the full build's
// corrections: the elevation and troposphere in the same arithmetic, the
// ionosphere from the azimuth's cosine and sine (no atan2) to a few parts in
// 10^16, all at the receiver's geodetic position as given rather than worked
// back out of ECEF (well under a millimetre); the noise is the same to a few
// parts in 10^16.
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
        // The cheaper build (above): off. atmo_period_s is its atmosphere's
        // period (0: worked out on every epoch). lite_solves: the orbits it
        // solves an epoch at most where they come due at once (0: no limit).
        // A solve is ~0.38 M instructions on the P4 (soft double, ~1.3 ms),
        // so the filter's slice budget (2.5 ms) holds two; four, because the
        // filter's clock and drift step tests take four satellites an epoch
        // (TR_RawGnssEKF beginGnssRaw) and the cap picks four GPS first, so
        // the first epoch after a gap has them. At the cap of 16 the epochs
        // after a gap then cost ~2.0 M instructions for four epochs, where
        // the solves at once cost 7.1 M on the first.
        bool     lite               = false;
        double   atmo_period_s      = 0.0;
        int      lite_solves        = 4;
    };

    // lite: a satellite held its lock this many relock_tau_s or more takes no
    // relock term (exp(-40) of it is under a part in 10^17 of either sigma)
    static constexpr double kLiteFreshMax = 40.0;

    RawMeasBuilder() { reset(); }
    explicit RawMeasBuilder(const Config& cfg) : cfg_(cfg) { reset(); }

    void setConfig(const Config& cfg)
    {
        cfg_ = cfg;
        for (LiteSat& l : lite_)
            l.sig_cn0 = -1.0f;            // the cheaper build's noise, worked out anew
    }
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
    // begin() and next() from a caller that has more at hand, for the cheaper
    // build (Config::lite; without it they are begin() and next()): lla, the
    // receiver's geodetic latitude, longitude (rad) and height (m) -- r_approx's
    // own -- so they are not worked back out of it; noted, the epoch's
    // satellites already noted at its time (noteEpoch), so not again; spread,
    // the orbits that come due at once solved lite_solves an epoch (the
    // caller's filter has taken its clock, which its first epoch takes from
    // the whole sky: false solves them all, as begin() does); and eph_of[i],
    // ep.obs[i]'s ephemeris as eph.select gives it at the epoch's time
    // (nullptr: none), so it is not looked up again (eph_of nullptr: it is).
    void beginWith(const RawEpoch& ep, const double* r_approx, const double* lla, double g_peak, bool noted,
                   bool spread);
    int  nextWith(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of, RawMeas* out, int n,
                  int cap, int max_obs);
    // The cheaper build's satellites of system sys left out of the epoch being
    // built (or last built) while their orbits' solves wait (spread), a bit a
    // PRN; 0 without lite.
    uint64_t waiting(int sys) const { return sys >= 0 && sys < kSys ? lite_wait_[sys] : 0; }

    // The noise of one measurement as the cheaper build gives it, at C/N0 cn0
    // (dB-Hz), relock age age_s, the signal's carrier and g_over (the stress
    // term's g): sigma_pr, sigma_pr_corr and sigma_rr. The full build's model
    // (above) with one exp for its three 10^(-(C/N0 - 35)/10), and no relock
    // term past kLiteFreshMax relock_tau_s. A sim's epochs, which no builder
    // places, take theirs from here too (tools/raw_replay/sim_epoch.h).
    static void liteSigmas(const Config& c, double cn0, double age_s, double carrier_hz, double g_over,
                           float& sigma_pr, float& sigma_pr_corr, float& sigma_rr);
    // The atmo_period_s period GPS time of week tow_s falls in, as the cheaper
    // build counts them: on the millisecond, as RawGnssShadow's pr_period_s
    // periods, so a period's first epoch is the one the shadow fuses the code on.
    static int64_t litePeriodOf(double tow_s, double period_s);

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
    // dts, iono, trop: the satellite clock (s) and the atmosphere (m) the
    // pseudorange was corrected by
    void carrierRate(const RawObs& o, const GnssSignalProfile& sig, double tow, double dts, double iono,
                     double trop, const double rs[3], RawMeas& m);

    // The cheaper build (RawMeasBuilderLite.cpp). Beside each cache slot: when
    // its orbit is next solved (its own phase of sat_refresh_s), and its
    // satellite's atmosphere as last worked out -- the period, the
    // ionosphere's generation, the elevation, the ionosphere (m) and the
    // cosine of the zenith angle the troposphere is mapped by (0: none).
    // And the noise's C/N0 part as it last worked it out (liteNoise), and at
    // which C/N0.
    struct LiteSat
    {
        double  t_next = 0.0;        // the satellite's time its next solve is due at
        int64_t atmo_k = -1;         // the atmo_period_s period worked out in (< 0: none)
        uint8_t atmo_gen = 0;
        float   cz_rate = 0.0f;      // cz's rate then from the satellite's own motion, 1/s
        double  t_atmo = 0.0;        // the epoch's time of week it was worked out at
        double  el = 0.0, iono = 0.0, cz = 0.0;
        float   sig_cn0 = -1.0f;     // (< 0: none)
        float   sig_pr = 0.0f;
        double  sig_corr = 0.0, sig_rr2 = 0.0;
    };
    static void liteNoise(const Config& c, double cn0, double carrier_hz, float& sigma_pr, double& s_corr,
                          double& s_rr2);
    static void liteRelock(const Config& c, double age_s, double g_over, double s_corr, double s_rr2,
                           float& sigma_pr_corr, float& sigma_rr);
    double klobucharLite(double ca, double sa, double el, double t) const;
    int  nextLite(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of, RawMeas* out, int n,
                  int cap, int max_obs);
    int  findLite(int sys, int prn) const;
    bool keptLite(int slot, double t_sv) const;
    bool liteObs(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of, int i,
                 const gps_eph_t*& e, double& t_sv) const;
    void rankLite(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of, int from);
    int  satSlotLite(const gps_eph_t& e, int sys, int prn, double t_sv, int slot, bool kept);
    void atmoLite(const GnssSignalProfile& sig, const double rs[3], const double vs[3], double tow, LiteSat& a);
    void zenithLite();
    void beginLite(const RawEpoch& ep, bool spread);

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
    // the cheaper build: each cache slot's (above), and the epoch's -- its
    // atmosphere period, and the receiver's trigonometry and troposphere at
    // the zenith, each worked out once an epoch when a satellite first needs it
    LiteSat    lite_[kSatCache];
    int64_t    lite_k_ = -1;
    bool       lite_geo_ = false, lite_zen_ = false;
    double     lite_sl_ = 0.0, lite_cl_ = 0.0, lite_so_ = 0.0, lite_co_ = 0.0;
    bool       lite_trop_ok_ = false;
    double     lite_dry_z_ = 0.0, lite_wet_z_ = 0.0;
    // and the epoch's orbit solves (spread): whether they are held to
    // lite_solves; how many it has made; whether the satellites that must be
    // solved to be built were ranked (rankLite), which of them may be (a bit
    // a PRN), how many of those wait their turn in the epoch; and those left out
    bool       lite_spread_ = false, lite_ranked_ = false;
    int        lite_solved_ = 0, lite_reserved_ = 0;
    uint64_t   lite_allow_[kSys] = {};
    uint64_t   lite_wait_[kSys] = {};
};

}  // namespace gnss_raw

#endif  // RAW_MEAS_BUILDER_H
