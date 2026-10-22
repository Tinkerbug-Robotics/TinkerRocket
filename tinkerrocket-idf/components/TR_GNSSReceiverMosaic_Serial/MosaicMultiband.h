// MosaicMultiband.h — one Doppler per satellite from every band the mosaic-G5 tracks (the multi-band evaluation: the harness alone, no firmware includes it)
//
// The flight's raw-GNSS filter fuses one Doppler row per satellite, from its
// L1-band signal (GPS L1 C/A, Galileo E1, BeiDou B1I: mosaicL1Pick). The
// mosaic also tracks GPS L2C and L5, Galileo E6, E5a and E5b, BeiDou B1C, B2a
// and B3I, and MeasEpoch carries every one of them. Here each satellite's
// other bands are folded into its row's range rate, so the filter still sees
// the same satellites, the same rows in the same order, and only a cell's
// rate_mps (and, where a band votes its anchor out, has_rate) changes.
//
// What the bands measure (the noise measurement, V9 + the Goblin's helix,
// 10/16 spin rig and the 10/15-16 mast captures; gnss-attitude-study/
// v9-mosaic-filter-plan/evidence/multiband/):
//   - each band's Doppler noise is the same in Hz, 0.53-0.56 Hz x
//     10^(-(C/N0 - 35)/20) for one 20 Hz epoch, so in m/s it scales with the
//     band's wavelength (MosaicMbBand's a, b below: the fit per band);
//   - the bands' errors on one satellite are independent once the receiver's
//     clock is out (|r| <= 0.05), except GPS L2 P(Y), which is L1 C/A's
//     (r 0.94-0.98: never joined); so inverse-variance weights are optimal;
//   - every band has the same lag structure (-0.6 at 100 ms), so the pairs
//     (MosaicRawEpoch.h averageDopplerPairs) and the combination multiply;
//   - the helix's carrier wind-up is about 0.99 cycle a turn on every band:
//     W * lambda_b on band b's range rate, W the same on every band and
//     satellite for a pure roll. A combined row would carry it at its
//     weighted wavelength, which the filter's correction (at its system's
//     wavelength) would not take off; here each band is referred to its
//     anchor's wavelength first (MosaicMbWindup::REF), with the epoch's
//     wind-up W_hat. By default that is the epoch's own estimate (no gyro):
//     each (satellite, band) pair gives (r_b - r_a) / (lambda_b - lambda_a),
//     in which the geometry, both clocks and the troposphere cancel, weighted
//     by (lambda_b - lambda_a)^2 / (sigma_a^2 + sigma_b^2) from both bands'
//     C/N0; the estimate is the weighted mean of the values within w_clip_k
//     of their own sigmas of the weighted median (mbWindupEstimate). It
//     needs at least w_min_values of them and a 1-sigma of at most
//     w_max_sigma_hz: an unweighted median of as few as 3 strayed past 2 Hz
//     in a quarter of the 10/17 weak sky's epochs, and the referral moved the
//     rows by about 0.1 m/s there. With w_smooth_q it is smoothed in time (a
//     scalar Kalman filter on a random walk, restarted after a gap or a step).
//     The caller can give the gyro's instead (w_src GYRO: W = -k p / 2pi
//     from the roll rate p, MosaicMbWindupIn), or as the fallback where the
//     estimate has too little (AUTO).
//
// The rules (each counted in MosaicMultibandStats):
//   - a row is combined only where its anchor's own Doppler is usable (valid,
//     its C/N0 at or above anchor_cn0_min). Code, carrier, lock, half-cycle
//     and C/N0 stay the anchor's;
//   - a band joins only from the anchor's own receiver channel, in the join
//     set and of the satellite's system, from the main antenna, with a valid
//     Doppler and C/N0, a lock time of at least min_lock_s (a relock drops it
//     to 0, so it stays out a second: the loop's settling stays out), its C/N0
//     at least its floor + cn0_hyst_db to join and its floor to stay, and a
//     Doppler lag within max_lag_ms of the anchor's. The extra bands' lock
//     times are their own (lockCapMs does not hold them: the Doppler does not
//     step at a clock jump). Each band's C/N0 carries its own signal's offset
//     (SbfMeasSignal::cn0Dbhz: none for GPS L1P and L2P). The half-cycle flag
//     is the carrier's, not the Doppler's: it gates nothing (a half-cycle
//     slip shows as a Doppler spike, which the screen catches);
//   - no wind-up for the referral (the estimate with too few values or too
//     little weight, no gyro's where it may stand in): the epoch goes
//     anchor-only;
//   - the inter-band screen, per satellite, needs neither gyro nor geometry:
//     d_b = r_b - r_a - W_hat (lambda_b - lambda_a) against
//     max(screen_k * sqrt(sigma_a^2 + sigma_b^2), screen_floor_mps +
//     screen_lag_s * |a_hat|), a_hat the anchor's own Doppler change since the
//     epoch before over that interval. A band off leaves for the epoch; an
//     anchor off against at least two bands that agree with each other is
//     voted out (the row's Doppler dropped, its code kept: anchor_vote); more
//     than half the epoch's bands off and the epoch goes anchor-only;
//   - the combination: r_c = sum(w r') / sum(w), w = 1 / sigma^2 from each
//     band's own C/N0, r' each band's range rate referred to the anchor's
//     wavelength. Its sigma sigma_c = (sum w)^(-1/2) is reported, not used:
//     the row keeps its anchor's sigma unless the caller expresses it
//     (express_sigma: an effective C/N0 on the profile's L1 model, mbEffCn0Q);
//   - the drift guard (guard): a fault on the anchor alone reaches the
//     combined row diluted by the anchor's weight (about 0.4-0.6), and one
//     under the screen's threshold passes. Each combined row's anchor less
//     the others' combination is averaged per satellite (a leaky mean,
//     guard_tau_s); once that mean is more than guard_k of its own sigmas
//     and guard_floor_mps off, the satellite's Doppler is dropped (its code
//     kept, as the vote does) until the mean comes back within guard_release
//     of that threshold. An anchor and one other band can't say which of the
//     two drifted, and the row's Doppler must contain its anchor's (the
//     pairs judge it by the anchor's lock), so the row loses its Doppler
//     rather than keep the others'.
//
// With the Dopplers in pairs, both ends of a pair go through the same
// combination (the grid epoch's rawEpochFromMeasEpochMb, the epoch between's
// noteMeasEpochDopplersMb), and averageDopplerPairs then runs unchanged on the
// combined rates: averaging and combining are both linear, every band is
// measured at the same MeasEpoch instants, so the pair's middle is the same
// 25 ms back.
//
// With cfg.on false, rawEpochFromMeasEpochMb and noteMeasEpochDopplersMb are
// rawEpochFromMeasEpoch and noteMeasEpochDopplers. With it on, the anchors'
// cells are built by the same lines as theirs, in the same single pass over
// the block (the extra bands captured on the way), so every anchor field but
// rate_mps and has_rate is theirs (tests_cpp/test_mosaic_multiband.cpp, on
// synthetic blocks and, behind TR_SBF_PRIVATE_CAPTURE, the 10/16 captures).
//
// The caller owns the scratch (MosaicMbScratch, a few KB): never on a stack.
// Header-only and pure.
#ifndef MOSAIC_MULTIBAND_H
#define MOSAIC_MULTIBAND_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "MosaicRawEpoch.h"
#include "RawMeas.h"
#include "SbfMeas.h"
#include "gnss/types.h"

namespace mosaic
{

static constexpr int kMbSigs = 40;                              // SBF signal numbers 0-39 (4.1.10)
static constexpr int kMbMaxBands = 6;                           // extra bands one satellite's capture holds
static constexpr int kMbMaxCombine = 3;                         // extra bands one combination takes at most
static constexpr int kMbObs = gnss_raw::RawEpoch::kMaxObs;      // cells, and captured satellites
static constexpr int64_t kMbPrevMaxMs = 250;                    // the epoch before's joins and rate count this long

// A signal's carrier frequency, Hz (sbfCarrierHz's, less GLONASS, whose
// FDMA bands never join): 0 for none.
constexpr double mbCarrierHz(int sig)
{
    switch (sig)
    {
        case 0: case 1: case 5: case 6: case 13: case 17: case 24: case 32: case 33: case 37: case 38:
            return 1575.42e6;
        case 2: case 3: case 7:
            return 1227.60e6;
        case 4: case 14: case 15: case 20: case 25: case 26: case 39:
            return 1176.45e6;
        case 12: return 1202.025e6;
        case 19: case 27: return 1278.75e6;
        case 21: case 29: case 34: return 1207.14e6;
        case 22: return 1191.795e6;
        case 28: return 1561.098e6;
        case 30: return 1268.52e6;
        default: return 0.0;
    }
}

// The wavelengths, m, by signal number: c / f once, at compile time (the same
// double the adapter's kMosaicC / sbfCarrierHz gives).
struct MosaicMbLambdaTable
{
    double v[kMbSigs];
    constexpr MosaicMbLambdaTable() : v()
    {
        for (int i = 0; i < kMbSigs; ++i)
            v[i] = mbCarrierHz(i) > 0.0 ? kMosaicC / mbCarrierHz(i) : 0.0;
    }
};
static constexpr MosaicMbLambdaTable kMbLambda{};

// The system (gnss_sys_t) a signal number belongs to; 0xFF for one that never joins.
constexpr uint8_t mbSigSys(int sig)
{
    switch (sig)
    {
        case 0: case 1: case 2: case 3: case 4: case 5:
            return GNSS_SYS_GPS;
        case 17: case 19: case 20: case 21: case 22:
            return GNSS_SYS_GAL;
        case 13: case 14: case 28: case 29: case 30: case 34:
            return GNSS_SYS_BDS;
        default:
            return 0xFF;
    }
}

// The anchor's signal number by system (mosaicL1Pick's): GPS L1 C/A, Galileo E1, BeiDou B1I.
constexpr uint8_t mbAnchorSig(uint8_t gsys)
{
    return gsys == GNSS_SYS_GAL ? SBF_SIG_GAL_E1 : (gsys == GNSS_SYS_BDS ? SBF_SIG_BDS_B1I : SBF_SIG_GPS_L1CA);
}

// One band's settings: whether it may join, its Doppler noise for one 20 Hz
// epoch (sigma^2 = scale^2 (a^2 + b^2 10^(-(C/N0 - 35)/10)), m/s), its C/N0
// floor and its Doppler lag (ms; a band more than max_lag_ms off its anchor's
// stays out). An anchor's own entry gives its model and lag.
struct MosaicMbBand
{
    bool  join = false;
    float a = 0.0f;
    float b = 0.0f;
    float scale = 1.0f;
    float cn0_min = 25.0f;
    float lag_ms = 0.0f;
};

// What the combined row's wind-up is made to be:
//   REF   every band referred to its anchor's wavelength with the epoch's
//         W_hat: the row carries exactly its anchor's wind-up;
//   LAM   no referral: the row carries its weighted wavelength
//         (MosaicMbOut::lam_wu), for the caller to correct with the gyro;
//   NONE  no referral and nothing told (the error left in: the arm that shows it).
enum class MosaicMbWindup : uint8_t { REF, LAM, NONE };

// Where the referral's W_hat comes from:
//   EST   the epoch's own estimate (mbWindupEstimate, smoothed with w_smooth_q);
//   GYRO  the caller's, from the gyro (MosaicMbWindupIn);
//   AUTO  the estimate where it has enough values and weight, else the gyro's.
// None (the estimate short, no gyro's that may stand in): the epoch goes anchor-only.
enum class MosaicMbWSrc : uint8_t { EST, GYRO, AUTO };

// The wind-up the caller gives an epoch from the gyro (w_src GYRO, AUTO): W = -k p / 2pi, Hz (m/s per m of
// wavelength: RawGnssWindup.h rollRate(1, k, p)), p the roll rate about the nose (rad/s) as the IMU path last
// published it, k the antenna's cycles a turn (the profile's windup_k, 0.99 on the Goblin's helix). ok false:
// none (no sample within its age bound, or the gyro at its rail). Roll alone: the rate across the nose adds a
// part each satellite has its own (Wu's), which a single W cannot carry.
struct MosaicMbWindupIn
{
    bool  ok = false;
    float hz = 0.0f;
};

struct MosaicMultibandConfig
{
    bool  on = false;
    MosaicMbBand band[kMbSigs];
    float anchor_cn0_min[GNSS_SYS_COUNT] = {};   // dB-Hz by gnss_sys_t: the anchor's Doppler floor (cn0_min_rr)
    float min_lock_s = 1.0f;          // a band's lock time at least this to join
    float cn0_hyst_db = 2.0f;         // its floor + this to join, its floor to stay
    float screen_k = 5.0f;            // the screen's sigmas
    float screen_floor_mps = 0.10f;   //   at least this
    float screen_lag_s = 0.003f;      //   plus this times |a_hat| (lag differences between bands under thrust)
    // the wind-up estimate (mbWindupEstimate): values from bands this far from their anchor's wavelength, the
    // weighted median's neighbours within w_clip_k of their own sigmas averaged, at least w_min_values of them;
    // smoothed in time as a random walk of w_smooth_q Hz^2/s (0: each epoch's own), restarted after w_smooth_gap_s
    // without an estimate or at a step past w_smooth_step_k sigmas that the next epoch confirms; used with a 1-sigma
    // of at most w_max_sigma_hz. Sized on the 10/17 weak sky, the 10/15 indoor mast and its turning runs
    // (gnss-attitude-study/v9-mosaic-filter-plan/evidence/multiband/scripts/prep/wu_tail.cpp): each epoch's own
    // weighted estimate has a 1-sigma of 1.2 Hz on the weak sky, the smoothed one 0.5 Hz, and its error past 2 Hz
    // went from a quarter of the epochs (the first, unweighted median) to none
    float w_min_dlam_m = 0.02f;
    int   w_min_values = 6;
    float w_max_sigma_hz = 0.75f;
    float w_clip_k = 3.0f;
    float w_smooth_q = 1.0f;
    float w_smooth_gap_s = 0.25f;
    float w_smooth_step_k = 4.0f;
    MosaicMbWSrc w_src = MosaicMbWSrc::EST;
    bool  anchor_vote = true;         // an anchor off against two agreeing bands: its Doppler dropped
    int   max_bands = kMbMaxCombine;  // extra bands a combination takes at most (the highest C/N0 first)
    float max_lag_ms = 1.0f;
    MosaicMbWindup windup = MosaicMbWindup::REF;
    bool  express_sigma = false;      // work out each combined row's effective C/N0 (MosaicMbOut::rr_cn0_q)
    float rr_a = 0.01f, rr_b = 0.075f;   // the profile's L1 white Doppler model it is expressed on
    // the drift guard: each satellite's anchor less its other bands' combination (referred), a leaky mean over
    // guard_tau_s (a running mean while it fills); past max(guard_k of its sigma, guard_floor_mps) the Doppler is
    // dropped until the mean is back within guard_release of that; its state starts over after guard_gap_s without
    // a combination. Sized on the four real V9 logs' quiet data (prep/wu_tail.cpp --guard, prep/mb_arms.py): inside
    // their scored windows the mean reached at most 5.3 of its sigmas (the cone-on rig, a GPS satellite at 29
    // dB-Hz) and 0.7-4.0 on the others, so 8; the floor keeps a mean of a few mm/s on strong signals from tripping.
    // Past the windows, at the rig's stop (the whole rig jolted, every band off by up to 1 m/s), it trips
    bool  guard = true;
    float guard_tau_s = 3.0f;
    float guard_k = 8.0f;
    float guard_floor_mps = 0.06f;
    float guard_release = 0.5f;
    float guard_gap_s = 0.5f;
};

// The measured noise (gnss-attitude-study/v9-mosaic-filter-plan/evidence/
// multiband: the V9 + Goblin helix spin rig of 10/16, cone off and on pooled,
// the Doppler against its own carrier rate with the receiver clock out, one
// 20 Hz epoch, a fitted to 0 on every band), the C/N0 floor 25 dB-Hz on every
// band, every lag 0 (the profile's rr_lag_s: 0 +- 2-4 ms on every signal),
// and the join set the design names: GPS L2C and L5, Galileo E6, E5a and E5b,
// BeiDou B1C, B2a and B3I. Never GPS L1P or L2P (semi-codeless: L2P's
// Doppler is L1 C/A's). on stays as it was.
inline void mosaicMbMeasured(MosaicMultibandConfig& c)
{
    for (int i = 0; i < kMbSigs; ++i)
        c.band[i] = MosaicMbBand{};
    auto set = [&](int sig, bool join, float b) {
        c.band[sig].join = join;
        c.band[sig].a = 0.0f;
        c.band[sig].b = b;
        c.band[sig].scale = 1.0f;
        c.band[sig].cn0_min = 25.0f;
        c.band[sig].lag_ms = 0.0f;
    };
    set(0, false, 0.104f);    // GPS L1 C/A (anchor)
    set(3, true, 0.129f);     // GPS L2C
    set(4, true, 0.139f);     // GPS L5
    set(17, false, 0.105f);   // Galileo E1 (anchor)
    set(19, true, 0.128f);    // Galileo E6
    set(20, true, 0.142f);    // Galileo E5a
    set(21, true, 0.137f);    // Galileo E5b
    set(28, false, 0.103f);   // BeiDou B1I (anchor)
    set(13, true, 0.104f);    // BeiDou B1C
    set(14, true, 0.140f);    // BeiDou B2a
    set(30, true, 0.128f);    // BeiDou B3I
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.anchor_cn0_min[k] = 25.0f;
}

// The design's provisional model before the measurement: equal noise in Hz on
// every band, L1 C/A's measured 0.104 m/s scaled by lambda_b / lambda_L1
// (the same join set).
inline void mosaicMbEqualHz(MosaicMultibandConfig& c)
{
    mosaicMbMeasured(c);
    const double l1 = kMbLambda.v[0];
    for (int i = 0; i < kMbSigs; ++i)
        if (c.band[i].b > 0.0f)
            c.band[i].b = static_cast<float>(0.104 * kMbLambda.v[i] / l1);
}

// A band's Doppler sigma at C/N0 cn0 (dB-Hz), m/s; 0 with no model.
inline double mbSigma(const MosaicMbBand& m, double cn0)
{
    const double q = std::pow(10.0, -(cn0 - 35.0) / 10.0);
    const double a = m.a, b = m.b;
    return static_cast<double>(m.scale) * std::sqrt(a * a + b * b * q);
}

// The effective C/N0 that gives a row's white Doppler term, on the profile's
// L1 model (a, b), ratio times what its anchor's C/N0 gives: SBF's code (0.25
// dB-Hz above 10). 0: the row's own (no gain, or a model that cannot say);
// 254, the top code, where ratio times the anchor's sigma is at or under a
// (the L1 fit's floor, taken as not averaging down).
inline uint8_t mbEffCn0Q(double cn0_anchor, double ratio, double a, double b)
{
    if (!(ratio > 0.0) || !(ratio < 1.0) || !(b > 0.0))
        return 0;
    const double qa = std::pow(10.0, -(cn0_anchor - 35.0) / 10.0);
    const double s2 = ratio * ratio * (a * a + b * b * qa);
    if (!(s2 > a * a))
        return 254;
    const double q = (s2 - a * a) / (b * b);
    const double cn0 = 35.0 - 10.0 * std::log10(q);
    long code = std::lround((cn0 - 10.0) / 0.25);
    const long own = std::lround((cn0_anchor - 10.0) / 0.25);
    if (code <= own)
        return 0;
    if (code > 254)
        code = 254;
    return static_cast<uint8_t>(code);
}

inline double mbCn0OfQ(uint8_t q) { return q * 0.25 + 10.0; }

// ── The capture and its scratch ────────────────────────────────────────

// One extra band, as captured from the block.
struct MosaicMbCap
{
    uint8_t  sig = 0;
    uint8_t  ch = 0;           // its receiver channel (its Type1's)
    uint16_t lock_s = 65535;   // as the reader gives it: 65535 none (a Type2's 255 too)
    float    cn0 = 0.0f;       // dB-Hz, with its own signal's offset (SbfMeasSignal::cn0Dbhz: none for GPS L1P, L2P)
    float    dr = 0.0f;        // its range rate less its satellite's base_mps, m/s
};

// One satellite's extra bands this epoch.
struct MosaicMbSat
{
    uint8_t     sys = 0, prn = 0;
    uint8_t     n = 0;
    bool        has_base = false;
    double      base_mps = 0.0;   // its first captured band's range rate (the others as floats from it)
    MosaicMbCap c[kMbMaxBands];
};

// A cell's band that passed the join rule, while its epoch is combined.
struct MosaicMbCand
{
    uint8_t sig = 0;
    uint8_t flagged = 0;
    float   cn0 = 0.0f;
    double  lam = 0.0;
    double  d = 0.0;       // r_b - r_a, m/s
    double  dp = 0.0;      // and less W_hat (lambda_b - lambda_a)
    double  sigma = 0.0;   // its model sigma, m/s
};

// The epoch before's, by satellite: what joined it (a bit a signal number)
// and the anchor's own range rate (a_hat).
struct MosaicMbPrev
{
    uint8_t  sys = 0, prn = 0;
    bool     has_rate = false;
    uint64_t joined = 0;
    double   rate_mps = 0.0;
};

// A wind-up value, Hz ((r_b - r_a) / (lambda_b - lambda_a)), and its weight, 1/Hz^2
// ((lambda_b - lambda_a)^2 / (sigma_a^2 + sigma_b^2)).
struct MosaicMbWVal
{
    float v = 0.0f;
    float w = 0.0f;
};

// The drift guard's state for one satellite: the leaky mean m of its anchor less its other bands' combination
// (m/s), the leaky mean v of that difference's variance, F the mean's share of it (m's variance v F: 1/n while
// it fills, then the leak's alpha / (2 - alpha)), and whether its Doppler is dropped.
struct MosaicMbGuard
{
    uint8_t  sys = 0, prn = 0;
    bool     flagged = false;
    uint32_t n = 0;
    int64_t  last_ms = 0;   // its last combination
    double   m = 0.0, v = 0.0, F = 1.0;
};

struct MosaicMbScratch
{
    int          n = 0;
    MosaicMbSat  sat[kMbObs];
    uint8_t      anchor_ch[kMbObs] = {};        // each cell's anchor's receiver channel
    uint8_t      ncand[kMbObs] = {};
    MosaicMbCand cand[kMbObs][kMbMaxCombine];
    MosaicMbWVal w_vals[kMbObs * kMbMaxCombine];
    // each cell's, while its epoch is combined (here, not on the stack)
    double       anchor_rate[kMbObs] = {};
    double       sigma_a[kMbObs] = {};
    bool         usable[kMbObs] = {};
    bool         prev_valid = false;
    int64_t      prev_rx_ms = 0;
    int          prev_n = 0;
    MosaicMbPrev prev[kMbObs];
    // the wind-up estimate smoothed in time (w_smooth_q): its value (Hz), variance, the instant it stands at and
    // the last estimate it took; an estimate a step off it, waiting for the next to confirm it
    bool         ws_ok = false;
    double       ws_hz = 0.0, ws_P = 0.0;
    int64_t      ws_ms = 0, ws_meas_ms = 0;
    bool         ws_step = false;
    float        ws_step_hz = 0.0f, ws_step_sig = 0.0f;
    // the drift guard, by satellite
    int          guard_n = 0;
    MosaicMbGuard guard[kMbObs];
};

// What each cell's row became.
enum class MosaicMbRow : uint8_t
{
    NONE,      // not looked at (no cell)
    NO_RATE,   // its anchor's Doppler not usable (none, or under its floor): left as it came
    ALONE,     // usable, no band joined: its anchor's alone
    EPOCH,     // the epoch went anchor-only (no wind-up for the referral, or the screen's majority)
    VOTED,     // its anchor voted out by two agreeing bands: its Doppler dropped
    COMBINED,  // one or more bands joined
    DRIFT,     // its anchor drifted off its other bands (the drift guard): its Doppler dropped
};

struct MosaicMbBandOut
{
    uint8_t sig = 0;
    uint8_t joined = 0;   // 1 joined; 0 flagged by the screen (or the epoch went anchor-only)
    float   cn0 = 0.0f;
    float   w = 0.0f;     // its share of the combination's weight
    float   dp = 0.0f;    // r_b - r_a less W_hat (lambda_b - lambda_a), m/s
};

// The last combination's side outputs, by cell.
struct MosaicMbOut
{
    int64_t     rx_ms = 0;
    int         n = 0;
    uint8_t     epoch = 0;     // 0 combined; 1 anchor-only (no wind-up for the referral); 2 anchor-only (the screen's
                               //   majority)
    int         w_n = 0;       // wind-up values
    float       w_hat = 0.0f;  // the epoch's wind-up the referral took, Hz (m/s per m of wavelength)
    float       w_sig = 0.0f;  //   its 1-sigma, Hz (0 the gyro's)
    uint8_t     w_src = 0;     //   from: 0 none (anchor-only), 1 the estimate, 2 the gyro
    bool        w_est_ok = false;   // the epoch's own estimate (before any smoothing) had enough values and weight
    int         w_kept = 0;         //   its values kept by the clip
    float       w_est = 0.0f;       //   its value, Hz (with too few values, what they gave)
    float       w_est_sig = 0.0f;   //   and 1-sigma
    bool        w_ext_ok = false;   // the caller's from the gyro
    float       w_ext = 0.0f;       //   Hz
    uint8_t     sys[kMbObs] = {}, prn[kMbObs] = {};
    float       cn0[kMbObs] = {};          // the anchor's C/N0, dB-Hz
    MosaicMbRow row[kMbObs] = {};
    uint8_t     bands[kMbObs] = {};        // bands joined
    uint64_t    mask[kMbObs] = {};         //   a bit a signal number
    float       sigma_a[kMbObs] = {};      // the anchor's model sigma, m/s (0: no C/N0)
    float       sigma_c[kMbObs] = {};      // the combination's (the anchor's where none joined)
    double      lam_wu[kMbObs] = {};       // the wavelength the row's wind-up is at, m (its anchor's but with LAM, NONE)
    float       d_mps[kMbObs] = {};        // the row's rate change: combined less its anchor's own
    uint8_t     rr_cn0_q[kMbObs] = {};     // with express_sigma: the effective C/N0 (mbEffCn0Q), 0 its own
    uint8_t     ncand[kMbObs] = {};
    MosaicMbBandOut det[kMbObs][kMbMaxCombine];
    // the drift guard, the rows it saw this epoch: the leaky mean of the anchor less its other bands (m/s) and
    // that mean's 1-sigma (0: not looked at)
    float       g_m[kMbObs] = {};
    float       g_sig[kMbObs] = {};
};

struct MosaicMultibandStats
{
    uint32_t epochs = 0;            // combinations run (grid and off-grid epochs)
    uint32_t epochs_few_w = 0;      //   anchor-only: no wind-up for the referral (the estimate short, no gyro's)
    uint32_t epochs_screen = 0;     //   anchor-only: more than half the epoch's bands flagged
    uint32_t epochs_w_est = 0;      //   referred with the estimate
    uint32_t epochs_w_gyro = 0;     //     with the gyro's
    // the estimate against the gyro's, the epochs with both (the estimate as the referral would take it, smoothed):
    // the difference's sum and sum of squares, Hz, and the regression through 0 of the one on the other
    uint32_t wx_n = 0;
    double   wx_sum = 0.0, wx_sum2 = 0.0, wx_xy = 0.0, wx_xx = 0.0;
    // the drift guard
    uint32_t guard_trips = 0;       // satellites flagged (each time one is)
    uint32_t guard_rows = 0;        // rows whose Doppler it dropped
    uint32_t guard_looks = 0;       // rows it averaged
    float    guard_max_z = 0.0f;    // the largest |mean| / its sigma seen
    float    guard_max_abs = 0.0f;  // the largest |mean| (m/s) of 20 combinations' worth or more (F <= 0.05)
    uint32_t sats = 0;              // usable anchors looked at
    uint32_t sats_combined = 0;     //   with a band joined
    uint32_t cand[kMbSigs] = {};    // bands that passed the join rule, by signal
    uint32_t joins[kMbSigs] = {};   //   and joined
    uint32_t screened = 0;          // bands the screen took out (anchor kept)
    uint32_t voted = 0;             // anchors voted out
    uint32_t comp_changes = 0;      // a satellite's joined bands changed since the epoch before (both combined or alone)
    uint32_t full = 0;              // bands (or satellites) the capture had no room for
    float    w_last = 0.0f, w_max_abs = 0.0f;
    double   w_sum2 = 0.0;
    uint32_t w_epochs = 0;
    double   ratio_sum = 0.0;       // sigma_c / sigma_a over combined rows
    uint32_t ratio_n = 0;
};

// The median of v[0..n) (n >= 1; reorders v).
inline float mbMedian(float* v, int n)
{
    const int h = n / 2;
    std::nth_element(v, v + h, v + n);
    const float hi = v[h];
    if (n % 2 != 0)
        return hi;
    const float lo = *std::max_element(v, v + h);
    return 0.5f * (lo + hi);
}

// The epoch's wind-up estimate, Hz, from n values (each (r_b - r_a) /
// (lambda_b - lambda_a), weighted (lambda_b - lambda_a)^2 / (sigma_a^2 +
// sigma_b^2)): the weighted median, then the weighted mean of the values
// within clip_k of their own sigmas (w^-1/2) of it; its 1-sigma (the kept
// weights' sum)^(-1/2). Two values of one satellite share its anchor's noise,
// so that 1-sigma is somewhat low; the floors here were sized with it as it
// is. kept: the values the mean took. True with at least min_values kept
// and a 1-sigma of at most max_sigma_hz (hz, sig and kept are set either
// way, 0 with no value). Reorders v. Float throughout: the P4's FPU.
inline bool mbWindupEstimate(MosaicMbWVal* v, int n, int min_values, float max_sigma_hz, float clip_k, float& hz,
                             float& sig, int& kept)
{
    hz = 0.0f;
    sig = 0.0f;
    kept = 0;
    if (n <= 0)
        return false;
    std::sort(v, v + n, [](const MosaicMbWVal& x, const MosaicMbWVal& y) { return x.v < y.v; });
    float wt = 0.0f;
    for (int i = 0; i < n; ++i)
        wt += v[i].w;
    if (!(wt > 0.0f))
        return false;
    // the weighted median: where the running weight reaches half (between two values at exactly half, their mean)
    float run = 0.0f, med = v[n - 1].v;
    for (int i = 0; i < n; ++i)
    {
        run += v[i].w;
        if (run >= 0.5f * wt)
        {
            med = (run == 0.5f * wt && i + 1 < n) ? 0.5f * (v[i].v + v[i + 1].v) : v[i].v;
            break;
        }
    }
    float sw = 0.0f, swv = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        const float d = v[i].v - med;
        if (d * d * v[i].w <= clip_k * clip_k)
        {
            sw += v[i].w;
            swv += v[i].w * v[i].v;
            kept++;
        }
    }
    if (!(sw > 0.0f))
        return false;
    hz = swv / sw;
    sig = 1.0f / std::sqrt(sw);
    return kept >= min_values && sig <= max_sigma_hz;
}

inline void mbBegin(MosaicMbScratch& scr) { scr.n = 0; }

// An extra band of the block (one mosaicL1Pick does not TAKE), kept if it
// could join: the main antenna, in the join set, of its satellite's system
// (no BeiDou GEO), a valid Doppler, C/N0 and wavelength.
inline void mbCapture(const SbfMeasSignal& s, const MosaicMultibandConfig& c, MosaicMbScratch& scr,
                      MosaicMultibandStats& st)
{
    if (s.antenna != 0 || s.sig >= kMbSigs || !c.band[s.sig].join || !s.dop_valid || !s.cn0Valid())
        return;
    uint8_t prn = 0;
    const SbfSys sys = sbfSvid(s.svid, prn);
    uint8_t gsys;
    if (sys == SbfSys::GPS)
        gsys = GNSS_SYS_GPS;
    else if (sys == SbfSys::GAL)
        gsys = GNSS_SYS_GAL;
    else if (sys == SbfSys::BDS && prn > 5 && prn < 59)
        gsys = GNSS_SYS_BDS;
    else
        return;
    if (mbSigSys(s.sig) != gsys)
        return;
    const double lam = kMbLambda.v[s.sig];
    if (!(lam > 0.0))
        return;
    int at = scr.n;
    for (int i = 0; i < scr.n; ++i)
        if (scr.sat[i].sys == gsys && scr.sat[i].prn == prn)
        {
            at = i;
            break;
        }
    if (at == scr.n)
    {
        if (scr.n >= kMbObs)
        {
            st.full++;
            return;
        }
        MosaicMbSat& ns = scr.sat[scr.n++];
        ns.sys = gsys;
        ns.prn = prn;
        ns.n = 0;
        ns.has_base = false;
    }
    MosaicMbSat& m = scr.sat[at];
    if (m.n >= kMbMaxBands)
    {
        st.full++;
        return;
    }
    const double r = -lam * s.doppler_hz;
    if (!m.has_base)
    {
        m.has_base = true;
        m.base_mps = r;
    }
    MosaicMbCap& k = m.c[m.n++];
    k.sig = s.sig;
    k.ch = s.rx_channel;
    k.cn0 = static_cast<float>(s.cn0Dbhz());
    k.lock_s = s.lock_s;
    k.dr = static_cast<float>(r - m.base_mps);
}

// The drift guard's entry for a satellite: its own, or (make) a new one -- the
// one combined longest ago taken over when every entry is in use; nullptr
// with none and make false.
inline MosaicMbGuard* mbGuardOf(MosaicMbScratch& scr, uint8_t sys, uint8_t prn, bool make)
{
    for (int j = 0; j < scr.guard_n; ++j)
        if (scr.guard[j].sys == sys && scr.guard[j].prn == prn)
            return &scr.guard[j];
    if (!make)
        return nullptr;
    int at = scr.guard_n;
    if (at >= kMbObs)
    {
        at = 0;
        for (int j = 1; j < kMbObs; ++j)
            if (scr.guard[j].last_ms < scr.guard[at].last_ms)
                at = j;
    }
    else
    {
        scr.guard_n++;
    }
    MosaicMbGuard& g = scr.guard[at];
    g = MosaicMbGuard{};
    g.sys = sys;
    g.prn = prn;
    return &g;
}

// Whether the guard holds a satellite's Doppler out now: flagged at its last
// combination, no more than guard_gap_s ago (a satellite with nothing to
// compare for longer starts over, its flag gone).
inline bool mbGuardHolds(MosaicMbGuard* g, int64_t rx_ms, const MosaicMultibandConfig& c)
{
    if (g == nullptr || g->n == 0)
        return false;
    if (rx_ms < g->last_ms || rx_ms - g->last_ms > static_cast<int64_t>(c.guard_gap_s * 1000.0f))
    {
        const uint8_t sys = g->sys, prn = g->prn;
        *g = MosaicMbGuard{};
        g->sys = sys;
        g->prn = prn;
        return false;
    }
    return g->flagged;
}

// The guard's update with a combination's x, its anchor less the others'
// combination (m/s), of variance vx: the leaky mean (a running mean while it
// fills: the leak's alpha 1 - exp(-dt / guard_tau_s), at least 1/n), its
// sigma, and the flag -- set past max(guard_k sigma, guard_floor_mps), let go
// within guard_release of that. m and sig: the mean and its sigma, for out.
inline void mbGuardUpdate(MosaicMbGuard& g, int64_t rx_ms, double x, double vx, const MosaicMultibandConfig& c,
                          MosaicMultibandStats& st, float& m, float& sig)
{
    if (g.n > 0 && (rx_ms <= g.last_ms || rx_ms - g.last_ms > static_cast<int64_t>(c.guard_gap_s * 1000.0f)))
    {
        const uint8_t sys = g.sys, prn = g.prn;
        g = MosaicMbGuard{};
        g.sys = sys;
        g.prn = prn;
    }
    const double leak = g.n == 0 ? 1.0
                                 : 1.0 - std::exp(-static_cast<double>(rx_ms - g.last_ms) * 1.0e-3 / c.guard_tau_s);
    const double fill = 1.0 / static_cast<double>(g.n + 1);
    const double a = fill > leak ? fill : leak;
    g.m += a * (x - g.m);
    g.v += a * (vx - g.v);
    g.F = (1.0 - a) * (1.0 - a) * g.F + a * a;
    g.n++;
    g.last_ms = rx_ms;
    const double sm = std::sqrt(g.v * g.F);
    const double am = std::fabs(g.m);
    const double thr = std::max(static_cast<double>(c.guard_k) * sm, static_cast<double>(c.guard_floor_mps));
    st.guard_looks++;
    if (sm > 0.0 && am / sm > st.guard_max_z)
        st.guard_max_z = static_cast<float>(am / sm);
    if (g.F <= 0.05 && am > st.guard_max_abs)
        st.guard_max_abs = static_cast<float>(am);
    if (!g.flagged && am > thr)
    {
        g.flagged = true;
        st.guard_trips++;
    }
    else if (g.flagged && am < c.guard_release * thr)
    {
        g.flagged = false;
    }
    m = static_cast<float>(g.m);
    sig = static_cast<float>(sm);
}

// The epoch's W_hat for the referral from its nw values (scr.w_vals) and the
// caller's (w_src): the estimate (each epoch's own, or smoothed in time with
// w_smooth_q), else (AUTO, GYRO) the gyro's. True with one, out's w_* fields
// saying which; false: the epoch goes anchor-only.
inline bool mbWindupPick(int nw, int64_t rx_ms, const MosaicMultibandConfig& c, const MosaicMbWindupIn& wext,
                         MosaicMbScratch& scr, MosaicMultibandStats& st, MosaicMbOut& out)
{
    float est = 0.0f, sig = 0.0f;
    int kept = 0;
    const bool est_ok = mbWindupEstimate(scr.w_vals, nw, c.w_min_values, c.w_max_sigma_hz, c.w_clip_k, est, sig, kept);
    out.w_n = nw;
    out.w_kept = kept;
    out.w_est = est;
    out.w_est_sig = sig;
    out.w_est_ok = est_ok;
    out.w_ext_ok = wext.ok;
    out.w_ext = wext.ok ? wext.hz : 0.0f;
    out.w_src = 0;
    out.w_hat = 0.0f;
    out.w_sig = 0.0f;
    // the estimate as the referral takes it: each epoch's own, or smoothed (a scalar Kalman filter on a random walk
    // of w_smooth_q Hz^2/s, which takes every epoch with w_min_values values at its own weight and coasts through
    // one without). An estimate more than w_smooth_step_k sigmas off is held back an epoch: the next one, off the
    // same way and within that of it, starts the filter over there (a step in the roll rate); else it is dropped (a
    // single bad epoch)
    bool use_ok = est_ok;
    double use = est, use_sig = sig;
    if (c.w_smooth_q > 0.0f)
    {
        const bool meas = kept >= c.w_min_values;
        const bool live = scr.ws_ok && rx_ms > scr.ws_ms &&
                          rx_ms - scr.ws_meas_ms <= static_cast<int64_t>(c.w_smooth_gap_s * 1000.0f);
        if (live)
        {
            scr.ws_P += static_cast<double>(c.w_smooth_q) * static_cast<double>(rx_ms - scr.ws_ms) * 1.0e-3;
            scr.ws_ms = rx_ms;
            if (meas)
            {
                const double k2 = static_cast<double>(c.w_smooth_step_k) * c.w_smooth_step_k;
                const double R = static_cast<double>(sig) * sig;
                const double nu = static_cast<double>(est) - scr.ws_hz;
                const double S = scr.ws_P + R;
                if (nu * nu > k2 * S)
                {
                    const double dp = static_cast<double>(est) - scr.ws_step_hz;
                    if (scr.ws_step && dp * dp <= k2 * (R + static_cast<double>(scr.ws_step_sig) * scr.ws_step_sig))
                    {
                        scr.ws_hz = est;   // the step confirmed: started over at this epoch's
                        scr.ws_P = R;
                        scr.ws_step = false;
                        scr.ws_meas_ms = rx_ms;
                    }
                    else
                    {
                        scr.ws_step = true;   // held back for the next to confirm
                        scr.ws_step_hz = est;
                        scr.ws_step_sig = sig;
                    }
                }
                else
                {
                    const double K = scr.ws_P / S;
                    scr.ws_hz += K * nu;
                    scr.ws_P *= 1.0 - K;
                    scr.ws_step = false;
                    scr.ws_meas_ms = rx_ms;
                }
            }
        }
        else if (meas)
        {
            scr.ws_ok = true;
            scr.ws_hz = est;
            scr.ws_P = static_cast<double>(sig) * sig;
            scr.ws_ms = scr.ws_meas_ms = rx_ms;
            scr.ws_step = false;
        }
        else
        {
            scr.ws_ok = false;
        }
        use = scr.ws_hz;
        use_sig = std::sqrt(scr.ws_P);
        use_ok = scr.ws_ok && use_sig <= c.w_max_sigma_hz;
    }
    if (use_ok && wext.ok)
    {
        // the estimate as the referral would take it, against the gyro's
        const double d = use - static_cast<double>(wext.hz);
        st.wx_n++;
        st.wx_sum += d;
        st.wx_sum2 += d * d;
        st.wx_xy += use * wext.hz;
        st.wx_xx += static_cast<double>(wext.hz) * wext.hz;
    }
    if (c.w_src != MosaicMbWSrc::GYRO && use_ok)
    {
        out.w_src = 1;
        out.w_hat = static_cast<float>(use);
        out.w_sig = static_cast<float>(use_sig);
        st.epochs_w_est++;
    }
    else if (c.w_src != MosaicMbWSrc::EST && wext.ok)
    {
        out.w_src = 2;
        out.w_hat = wext.hz;
        st.epochs_w_gyro++;
    }
    else
    {
        return false;
    }
    st.w_last = out.w_hat;
    st.w_sum2 += static_cast<double>(out.w_hat) * out.w_hat;
    st.w_epochs++;
    if (std::fabs(out.w_hat) > st.w_max_abs)
        st.w_max_abs = std::fabs(out.w_hat);
    return true;
}

// The combination over an epoch's cells (RawObs or MosaicDopplerCell: sys,
// prn, has_rate, cn0_dbhz, rate_mps), their anchors' channels in
// scr.anchor_ch, the extra bands in scr.sat, the gyro's wind-up in wext: each
// usable row's rate_mps combined (or its has_rate cleared where its anchor is
// voted out or the drift guard holds it), by the rules at the top of this
// file. out says what each row became.
template <class Cell>
inline void mbCombine(Cell* cells, int n, int64_t rx_ms, const MosaicMultibandConfig& c, const MosaicMbWindupIn& wext,
                      MosaicMbScratch& scr, MosaicMultibandStats& st, MosaicMbOut& out)
{
    out.rx_ms = rx_ms;
    out.n = n;
    out.epoch = 0;
    out.w_n = 0;
    out.w_hat = 0.0f;
    st.epochs++;
    const bool prev_ok = scr.prev_valid && rx_ms > scr.prev_rx_ms && rx_ms - scr.prev_rx_ms <= kMbPrevMaxMs;
    const double prev_dt = prev_ok ? static_cast<double>(rx_ms - scr.prev_rx_ms) * 1.0e-3 : 0.0;
    auto prevOf = [&](uint8_t sys, uint8_t prn) -> const MosaicMbPrev* {
        if (!prev_ok)
            return nullptr;
        for (int j = 0; j < scr.prev_n; ++j)
            if (scr.prev[j].sys == sys && scr.prev[j].prn == prn)
                return &scr.prev[j];
        return nullptr;
    };
    double* const anchor_rate = scr.anchor_rate;
    double* const sigma_a = scr.sigma_a;
    bool* const usable = scr.usable;
    int nw = 0;
    for (int i = 0; i < n; ++i)
    {
        Cell& e = cells[i];
        out.sys[i] = e.sys;
        out.prn[i] = e.prn;
        out.cn0[i] = e.cn0_dbhz;
        out.row[i] = MosaicMbRow::NO_RATE;
        out.bands[i] = 0;
        out.mask[i] = 0;
        out.d_mps[i] = 0.0f;
        out.rr_cn0_q[i] = 0;
        out.ncand[i] = 0;
        out.g_m[i] = 0.0f;
        out.g_sig[i] = 0.0f;
        scr.ncand[i] = 0;
        anchor_rate[i] = e.rate_mps;
        const uint8_t asig = mbAnchorSig(e.sys);
        const double lam_a = kMbLambda.v[asig];
        out.lam_wu[i] = lam_a;
        const double sa = e.cn0_dbhz > 0.0f ? mbSigma(c.band[asig], e.cn0_dbhz) : 0.0;
        sigma_a[i] = sa;
        out.sigma_a[i] = static_cast<float>(sa);
        out.sigma_c[i] = static_cast<float>(sa);
        const float floor_a = e.sys < GNSS_SYS_COUNT ? c.anchor_cn0_min[e.sys] : 0.0f;
        usable[i] = e.has_rate && !(e.cn0_dbhz < floor_a) && sa > 0.0;
        if (!usable[i])
            continue;
        st.sats++;
        out.row[i] = MosaicMbRow::ALONE;
        const MosaicMbSat* m = nullptr;
        for (int j = 0; j < scr.n && m == nullptr; ++j)
            if (scr.sat[j].sys == e.sys && scr.sat[j].prn == e.prn)
                m = &scr.sat[j];
        if (m == nullptr)
            continue;
        const MosaicMbPrev* p = prevOf(e.sys, e.prn);
        const uint64_t was = p != nullptr ? p->joined : 0;
        const int cap = c.max_bands < kMbMaxCombine ? (c.max_bands > 0 ? c.max_bands : 0) : kMbMaxCombine;
        for (int k = 0; k < m->n; ++k)
        {
            const MosaicMbCap& b = m->c[k];
            if (b.ch != scr.anchor_ch[i])
                continue;
            const MosaicMbBand& mb = c.band[b.sig];
            if (!(mb.b > 0.0f || mb.a > 0.0f))
                continue;
            if (b.lock_s == 65535 || static_cast<float>(b.lock_s) < c.min_lock_s)
                continue;
            const float cn0 = b.cn0;
            const bool in_before = (was >> b.sig) & 1u;
            if (cn0 < mb.cn0_min + (in_before ? 0.0f : c.cn0_hyst_db))
                continue;
            if (std::fabs(mb.lag_ms - c.band[asig].lag_ms) > c.max_lag_ms)
                continue;
            st.cand[b.sig]++;
            MosaicMbCand cd;
            cd.sig = b.sig;
            cd.cn0 = cn0;
            cd.lam = kMbLambda.v[b.sig];
            cd.d = (m->base_mps - e.rate_mps) + static_cast<double>(b.dr);
            cd.sigma = mbSigma(mb, cn0);
            uint8_t& nc = scr.ncand[i];
            if (nc < cap)
            {
                scr.cand[i][nc++] = cd;
            }
            else if (cap > 0)
            {
                // full: the weakest out for a stronger one
                int lo = 0;
                for (int q = 1; q < nc; ++q)
                    if (scr.cand[i][q].cn0 < scr.cand[i][lo].cn0)
                        lo = q;
                if (cd.cn0 > scr.cand[i][lo].cn0)
                    scr.cand[i][lo] = cd;
            }
        }
        // its wind-up values, each weighted by its inverse variance in Hz -- not from a satellite the drift guard
        // holds (its anchor's drift is in every one of them)
        if (c.guard && mbGuardHolds(mbGuardOf(scr, e.sys, e.prn, false), rx_ms, c))
            continue;
        for (int q = 0; q < scr.ncand[i]; ++q)
        {
            const MosaicMbCand& cd = scr.cand[i][q];
            const double dl = cd.lam - lam_a;
            if (std::fabs(dl) >= c.w_min_dlam_m && nw < kMbObs * kMbMaxCombine)
            {
                scr.w_vals[nw].v = static_cast<float>(cd.d / dl);
                scr.w_vals[nw].w = static_cast<float>(dl * dl / (sa * sa + cd.sigma * cd.sigma));
                nw++;
            }
        }
    }

    // the epoch's wind-up, then the screen
    float w_hat = 0.0f;
    int total = 0, flagged = 0;
    bool epoch_alone = false;
    if (!mbWindupPick(nw, rx_ms, c, wext, scr, st, out))
    {
        // no wind-up to refer by (the estimate short, no gyro's to stand in; an epoch with no band at all counts
        // here too): anchor-only
        epoch_alone = true;
        out.epoch = 1;
        st.epochs_few_w++;
    }
    else
    {
        w_hat = out.w_hat;
        for (int i = 0; i < n; ++i)
        {
            if (!usable[i] || scr.ncand[i] == 0)
                continue;
            const MosaicMbPrev* p = prevOf(cells[i].sys, cells[i].prn);
            const double a_hat = p != nullptr && p->has_rate ? (anchor_rate[i] - p->rate_mps) / prev_dt : 0.0;
            const double thr_lag = c.screen_floor_mps + c.screen_lag_s * std::fabs(a_hat);
            const double lam_a = kMbLambda.v[mbAnchorSig(cells[i].sys)];
            const double sa = sigma_a[i];
            for (int q = 0; q < scr.ncand[i]; ++q)
            {
                MosaicMbCand& cd = scr.cand[i][q];
                cd.dp = cd.d - static_cast<double>(w_hat) * (cd.lam - lam_a);
                const double thr = std::max(c.screen_k * std::sqrt(sa * sa + cd.sigma * cd.sigma), thr_lag);
                cd.flagged = std::fabs(cd.dp) > thr ? 1 : 0;
                total++;
                flagged += cd.flagged;
            }
        }
        if (2 * flagged > total)
        {
            epoch_alone = true;
            out.epoch = 2;
            st.epochs_screen++;
        }
    }

    for (int i = 0; i < n; ++i)
    {
        Cell& e = cells[i];
        const uint8_t nc = scr.ncand[i];
        out.ncand[i] = nc;
        for (int q = 0; q < nc; ++q)
        {
            const MosaicMbCand& cd = scr.cand[i][q];
            MosaicMbBandOut& bo = out.det[i][q];
            bo.sig = cd.sig;
            bo.joined = 0;
            bo.cn0 = cd.cn0;
            bo.w = 0.0f;
            bo.dp = static_cast<float>(epoch_alone && out.epoch == 1 ? cd.d : cd.dp);
        }
        if (!usable[i])
            continue;
        // the drift guard's hold: the satellite's Doppler out, whatever this epoch has to compare it with
        MosaicMbGuard* const g = c.guard ? mbGuardOf(scr, e.sys, e.prn, false) : nullptr;
        auto drift = [&]() {
            e.has_rate = false;
            e.rate_mps = anchor_rate[i];
            out.row[i] = MosaicMbRow::DRIFT;
            st.guard_rows++;
        };
        if (nc == 0 || epoch_alone)
        {
            if (nc > 0)
                out.row[i] = MosaicMbRow::EPOCH;
            if (mbGuardHolds(g, rx_ms, c))
                drift();
            continue;
        }
        const MosaicMbPrev* p = prevOf(e.sys, e.prn);
        const double a_hat = p != nullptr && p->has_rate ? (anchor_rate[i] - p->rate_mps) / prev_dt : 0.0;
        const double thr_lag = c.screen_floor_mps + c.screen_lag_s * std::fabs(a_hat);
        int nf = 0;
        bool vote = false;
        for (int q = 0; q < nc; ++q)
            nf += scr.cand[i][q].flagged;
        if (c.anchor_vote && nf >= 2)
            for (int q = 0; q < nc && !vote; ++q)
                for (int r = q + 1; r < nc && !vote; ++r)
                {
                    const MosaicMbCand& x = scr.cand[i][q];
                    const MosaicMbCand& y = scr.cand[i][r];
                    if (!x.flagged || !y.flagged)
                        continue;
                    const double thr = std::max(c.screen_k * std::sqrt(x.sigma * x.sigma + y.sigma * y.sigma),
                                                thr_lag);
                    vote = std::fabs(x.dp - y.dp) <= thr;
                }
        if (vote)
        {
            e.has_rate = false;
            out.row[i] = MosaicMbRow::VOTED;
            st.voted++;
            continue;
        }
        st.screened += static_cast<uint32_t>(nf);
        const double lam_a = kMbLambda.v[mbAnchorSig(e.sys)];
        const double sa = sigma_a[i];
        double wsum = 1.0 / (sa * sa);
        double num = 0.0;
        double lnum = lam_a * wsum;
        double wo = 0.0, onum = 0.0;   // the others alone: their weights and their referred differences, summed
        uint64_t mask = 0;
        int nb = 0;
        for (int q = 0; q < nc; ++q)
        {
            const MosaicMbCand& cd = scr.cand[i][q];
            if (cd.flagged)
                continue;
            const double w = 1.0 / (cd.sigma * cd.sigma);
            wsum += w;
            num += w * (c.windup == MosaicMbWindup::REF ? cd.dp : cd.d);
            lnum += w * cd.lam;
            wo += w;
            onum += w * cd.dp;
            mask |= uint64_t{1} << cd.sig;
            nb++;
        }
        if (nb == 0)
        {
            // every band screened out: the anchor's alone (ALONE)
            if (mbGuardHolds(g, rx_ms, c))
                drift();
            continue;
        }
        if (c.guard)
        {
            // the anchor less the others' combination, referred: its leaky mean against its own noise
            MosaicMbGuard* const gg = g != nullptr ? g : mbGuardOf(scr, e.sys, e.prn, true);
            mbGuardUpdate(*gg, rx_ms, -onum / wo, sa * sa + 1.0 / wo, c, st, out.g_m[i], out.g_sig[i]);
            if (gg->flagged)
            {
                drift();
                continue;
            }
        }
        for (int q = 0; q < nc; ++q)
            if (!scr.cand[i][q].flagged)
            {
                st.joins[scr.cand[i][q].sig]++;
                out.det[i][q].joined = 1;
                out.det[i][q].w = static_cast<float>(1.0 / (scr.cand[i][q].sigma * scr.cand[i][q].sigma) / wsum);
            }
        const double dr = num / wsum;
        e.rate_mps = anchor_rate[i] + dr;
        out.row[i] = MosaicMbRow::COMBINED;
        out.bands[i] = static_cast<uint8_t>(nb);
        out.mask[i] = mask;
        out.d_mps[i] = static_cast<float>(dr);
        const double sc = 1.0 / std::sqrt(wsum);
        out.sigma_c[i] = static_cast<float>(sc);
        out.lam_wu[i] = c.windup == MosaicMbWindup::REF ? lam_a : lnum / wsum;
        st.sats_combined++;
        st.ratio_sum += sc / sa;
        st.ratio_n++;
        if (c.express_sigma)
            out.rr_cn0_q[i] = mbEffCn0Q(e.cn0_dbhz, sc / sa, c.rr_a, c.rr_b);
    }

    // composition changes, and this epoch's state for the next
    for (int i = 0; i < n; ++i)
    {
        if (!usable[i] || out.row[i] == MosaicMbRow::VOTED || out.row[i] == MosaicMbRow::DRIFT)
            continue;
        const MosaicMbPrev* p = prevOf(cells[i].sys, cells[i].prn);
        if (p != nullptr && p->has_rate && p->joined != out.mask[i])
            st.comp_changes++;
    }
    scr.prev_n = 0;
    for (int i = 0; i < n && scr.prev_n < kMbObs; ++i)
    {
        MosaicMbPrev& p = scr.prev[scr.prev_n++];
        p.sys = cells[i].sys;
        p.prn = cells[i].prn;
        p.has_rate = usable[i] && out.row[i] != MosaicMbRow::VOTED && out.row[i] != MosaicMbRow::DRIFT;
        p.joined = out.mask[i];
        p.rate_mps = anchor_rate[i];
    }
    scr.prev_valid = true;
    scr.prev_rx_ms = rx_ms;
}

// rawEpochFromMeasEpoch (MosaicRawEpoch.h), with each satellite's other bands
// combined into its cells' range rates (cfg.on; off: exactly it). Its
// arguments, then the configuration, the scratch, the stats and the side
// outputs; last, the gyro's wind-up for the epoch (w_src GYRO, AUTO).
inline bool rawEpochFromMeasEpochMb(const uint8_t* b, size_t len, gnss_raw::RawEpoch& out, MosaicJumpState& js,
                                    MosaicEpochInfo& info, const MosaicMultibandConfig& cfg, MosaicMbScratch& scr,
                                    MosaicMultibandStats& st, MosaicMbOut& mbo,
                                    int64_t arrival_us = kMosaicNoArrival,
                                    const MosaicMbWindupIn& wext = MosaicMbWindupIn{})
{
    if (!cfg.on)
        return rawEpochFromMeasEpoch(b, len, out, js, info, arrival_us);
    out.n = 0;
    info = MosaicEpochInfo{};
    SbfMeasReader rd;
    info.status = rd.begin(b, len);
    const SbfMeasHeader& h = rd.header();
    info.tow_ms = h.tow_ms;
    info.wnc = h.wnc;
    info.common_flags = h.common_flags;
    if (info.status != SbfStatus::OK)
        return false;
    noteClockJumps(h, js, info, arrival_us);
    info.lock_cap_ms = lockCapMs(h, js);
    out.tow_s = h.tow_ms * 1.0e-3;
    mbBegin(scr);

    SbfMeasSignal s;
    while (rd.next(s))
    {
        uint8_t gsys = 0, prn = 0;
        const MosaicPick pick = mosaicL1Pick(s, gsys, prn);
        if (pick == MosaicPick::GEO)
        {
            info.geo++;
            continue;
        }
        if (pick != MosaicPick::TAKE)
        {
            mbCapture(s, cfg, scr, st);
            continue;
        }

        // rawEpochFromMeasEpoch's cell, line for line
        const bool cn0_ok = s.cn0Valid();
        const bool has_pr = cn0_ok && s.pr_valid;
        int at = out.n;
        for (int i = 0; i < out.n; ++i)
            if (out.obs[i].sys == gsys && out.obs[i].prn == prn)
            {
                at = i;
                break;
            }
        if (at < out.n)
        {
            info.dupes++;
            if (out.obs[at].has_pr || !has_pr)
                continue;
        }
        else if (out.n >= gnss_raw::RawEpoch::kMaxObs)
        {
            info.over++;
            continue;
        }
        else
        {
            out.n++;
            if (gsys == GNSS_SYS_GPS) info.n_gps++;
            else if (gsys == GNSS_SYS_GAL) info.n_gal++;
            else info.n_bds++;
        }

        const double lambda = kMosaicC / sbfCarrierHz(s.sig);
        gnss_raw::RawObs& o = out.obs[at];
        o.sys = gsys;
        o.prn = prn;
        o.cn0_dbhz = cn0_ok ? static_cast<float>(s.cn0Dbhz()) : 0.0f;
        o.has_pr = has_pr;
        o.pr_m = has_pr ? s.prM() : 0.0;
        o.has_rate = cn0_ok && s.dop_valid;
        o.rate_mps = o.has_rate ? -lambda * s.doppler_hz : 0.0;
        const uint32_t lock_ms = static_cast<uint32_t>(s.lock_s) * 1000U;
        o.lock_ms = s.lockValid() ? (lock_ms < info.lock_cap_ms ? lock_ms : info.lock_cap_ms) : 0U;
        o.has_phase = has_pr && s.carrier_valid && s.lockValid();
        o.halfcyc = s.half_cycle;
        o.phase_m = o.has_phase ? s.phaseRangeM(lambda) : 0.0;
        scr.anchor_ch[at] = s.rx_channel;   // its bands come from this channel
    }
    mbCombine(out.obs, out.n, static_cast<int64_t>(h.wnc) * 604800000LL + h.tow_ms, cfg, wext, scr, st, mbo);
    return out.n > 0;
}

// noteMeasEpochDopplers (MosaicRawEpoch.h), each cell's range rate combined
// as rawEpochFromMeasEpochMb combines a grid epoch's (cfg.on; off: exactly it).
inline bool noteMeasEpochDopplersMb(const uint8_t* b, size_t len, MosaicJumpState& js, MosaicEpochInfo& info,
                                    MosaicDopplerSet& set, const MosaicMultibandConfig& cfg, MosaicMbScratch& scr,
                                    MosaicMultibandStats& st, MosaicMbOut& mbo,
                                    int64_t arrival_us = kMosaicNoArrival,
                                    const MosaicMbWindupIn& wext = MosaicMbWindupIn{})
{
    if (!cfg.on)
        return noteMeasEpochDopplers(b, len, js, info, set, arrival_us);
    set.valid = false;
    set.n = 0;
    info = MosaicEpochInfo{};
    SbfMeasReader rd;
    info.status = rd.begin(b, len);
    const SbfMeasHeader& h = rd.header();
    info.tow_ms = h.tow_ms;
    info.wnc = h.wnc;
    info.common_flags = h.common_flags;
    if (info.status != SbfStatus::OK)
        return false;
    noteClockJumps(h, js, info, arrival_us);
    info.lock_cap_ms = lockCapMs(h, js);
    set.rx_ms = static_cast<int64_t>(h.wnc) * 604800000LL + h.tow_ms;
    mbBegin(scr);

    SbfMeasSignal s;
    while (rd.next(s))
    {
        uint8_t gsys = 0, prn = 0;
        if (mosaicL1Pick(s, gsys, prn) != MosaicPick::TAKE)
        {
            mbCapture(s, cfg, scr, st);
            continue;
        }
        // noteMeasEpochDopplers' cell, line for line
        const bool cn0_ok = s.cn0Valid();
        const bool has_pr = cn0_ok && s.pr_valid;
        int at = set.n;
        for (int i = 0; i < set.n; ++i)
            if (set.c[i].sys == gsys && set.c[i].prn == prn)
            {
                at = i;
                break;
            }
        if (at < set.n)
        {
            if (set.c[at].has_pr || !has_pr)
                continue;
        }
        else if (set.n >= gnss_raw::RawEpoch::kMaxObs)
        {
            continue;
        }
        else
        {
            set.n++;
        }
        MosaicDopplerCell& c = set.c[at];
        c.sys = gsys;
        c.prn = prn;
        c.has_pr = has_pr;
        c.cn0_dbhz = cn0_ok ? static_cast<float>(s.cn0Dbhz()) : 0.0f;
        c.has_rate = cn0_ok && s.dop_valid;
        c.rate_mps = c.has_rate ? -(kMosaicC / sbfCarrierHz(s.sig)) * s.doppler_hz : 0.0;
        const uint32_t lock_ms = static_cast<uint32_t>(s.lock_s) * 1000U;
        c.lock_ms = s.lockValid() ? (lock_ms < info.lock_cap_ms ? lock_ms : info.lock_cap_ms) : 0U;
        scr.anchor_ch[at] = s.rx_channel;
    }
    mbCombine(set.c, set.n, set.rx_ms, cfg, wext, scr, st, mbo);
    set.valid = true;
    return true;
}

// A paired grid row's sigma ratio and wind-up wavelength, from its two ends'
// side outputs (the grid end's cell i, the epoch between's cell j; j < 0: the
// row was not averaged, its grid end's alone): the mean of two halves the
// sum of their variances, so the ratio is sqrt((sc_i^2 + sc_j^2) /
// (sa_i^2 + sa_j^2)), and its wind-up sits at the mean of the two wavelengths.
inline void mbPairedRow(const MosaicMbOut& g, int i, const MosaicMbOut& o, int j, double& ratio, double& lam_wu)
{
    const double sa_i = g.sigma_a[i], sc_i = g.sigma_c[i];
    ratio = sa_i > 0.0 ? sc_i / sa_i : 1.0;
    lam_wu = g.lam_wu[i];
    if (j < 0)
        return;
    const double sa_j = o.sigma_a[j], sc_j = o.sigma_c[j];
    const double den = sa_i * sa_i + sa_j * sa_j;
    if (den > 0.0)
        ratio = std::sqrt((sc_i * sc_i + sc_j * sc_j) / den);
    lam_wu = 0.5 * (g.lam_wu[i] + o.lam_wu[j]);
}

}  // namespace mosaic

#endif  // MOSAIC_MULTIBAND_H
