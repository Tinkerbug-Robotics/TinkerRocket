// MosaicMultiband.h — one Doppler per satellite from every band the mosaic-G5 tracks (the mosaic image's driver, config.h TR_RAW_MULTIBAND, and tools/raw_replay's --rr-multiband)
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
//     wind-up W_hat. By default that is the epoch's own estimate: each
//     satellite gives one value, its anchor's and bands' range rates
//     regressed on their wavelengths (the slope; the geometry, both clocks
//     and the troposphere cancel, and the anchor's error, which every
//     difference r_b - r_a carries, is counted once), with its own 1-sigma
//     from each band's C/N0. No satellite holds more than w_max_share of the
//     weight; the estimate is the capped weighted mean of the satellites
//     within w_clip_k of their own sigmas of the capped weighted median
//     (mbWindupEstimate), from at least w_min_sats satellites and
//     w_min_values values. Smoothed in time (w_smooth_q: a scalar Kalman
//     filter on a random walk), each satellite is gated against the
//     prediction and a step is taken only where the gyro's agrees with it,
//     or without one where the satellites that moved outweigh those that
//     stayed (two at least), confirmed by the next epoch: so one satellite's
//     anchor fault cannot carry the estimate (the review of a08bdcde: one
//     satellite holding 0.5-0.9 of the old per-value weight moved W_hat by
//     1.4-5 Hz). A satellite the drift guard holds, or that the vote at the
//     prediction would put out, gives no value. The caller can give the
//     gyro's instead (w_src GYRO: W = -k p / 2pi from the roll rate p,
//     MosaicMbWindupIn), or as the fallback where the estimate has too
//     little (AUTO); given at all, it also checks the estimate (w_check).
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
//   - the estimate against the gyro's (w_check, where the caller gives the
//     gyro's): further apart than max(x_k of their sigmas, x_floor_hz) for
//     x_on epochs running and every epoch goes anchor-only until they agree
//     for x_off running -- the image's rows, which neither can mislead;
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
//     rather than keep the others'. The difference is taken at the gyro's
//     wind-up where given (the estimate carries each satellite's own pull on
//     it); its threshold carries the screen's allowance for acceleration; an
//     epoch whose vote puts the anchor out feeds the mean too. A hold lasts
//     while the satellite is tracked, whatever the epoch has to compare (every
//     band screened out, the epoch anchor-only): it ends on agreement, or with
//     the track (untracked for guard_gap_s). The review of a08bdcde found the
//     hold let go exactly when the fault outgrew the screen, the anchor then
//     going back in alone with the whole fault;
//   - the anchors' common offset (common): each satellite's anchor less all
//     its bands, at the gyro's wind-up (else W_hat), a leaky mean by
//     satellite, and by scope (each system, and every satellite) the capped
//     weighted median of those means against max(common_k of its sigma,
//     common_floor_mps + the acceleration allowance), with a hold. Every
//     satellite's scope tripped and no system looked at on its own saying
//     otherwise: the epoch goes anchor-only, the image's rows, whose common
//     offset the receiver clock's drift takes (a08bdcde turned it into
//     satellite-specific errors). A system tripped, or more than half its
//     satellites (two at least) found off every band they have, all one way,
//     by the screen: its rows go anchor-only together (sys_action ANCHOR) or
//     lose their Dopplers together (DROP) -- never thinned by the vote to the
//     single-band ones, which a08bdcde left in alone with the whole fault
//     (the evaluation's GPS-wide 1 m/s step: 1 m/s against the image's
//     29-241 mm/s on the rigs).
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

// The combiner's working arithmetic: float in the images, whose CPUs (the P4, the S3) have a single-precision FPU
// and do double in software -- a double multiply-add took 1.9 us on the P4, the whole combination ~6 ms an epoch
// (MosaicMultibandBench.h). Absolute range rates (up to ~1 km/s, kept to 0.1 mm/s) stay double: the anchors' own,
// a satellite's captured base and the epoch before's. Everything taken from their differences -- the bands' and
// anchors' sigmas and weights, the wind-up, the screen, the guard's and the monitor's means, each row's change --
// is in this. -DMOSAIC_MB_REAL=double builds the reference the float one was checked against.
#ifndef MOSAIC_MB_REAL
#define MOSAIC_MB_REAL float
#endif
using mbr = MOSAIC_MB_REAL;

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

// What a system's rows become when its anchors are off their other bands together (the monitor or the screen):
//   ANCHOR  anchor-only, the image's rows (a fault on them all reaches the filter as the image's would);
//   DROP    their Dopplers dropped, as the vote drops one satellite's (their code kept): the default.
enum class MosaicMbSysAct : uint8_t { ANCHOR, DROP };

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

// The gyro's wind-up as a driver reads it decoding a MeasEpoch: the IMU path's last roll rate p (rad/s, about the
// nose), have false before the first, age_us since it was published; none (ok false) older than max_age_us or at
// the gyro's rail. W = -k p / 2pi (RawGnssWindup.h rollRate(1, k, p)). The mosaic image's driver and
// tools/raw_replay both take it from here.
inline MosaicMbWindupIn mbWindupFromGyro(bool have, double p_rps, int64_t age_us, double k,
                                         int64_t max_age_us = 20000, double rail_rps = 4000.0 * 3.14159265358979323846 / 180.0)
{
    MosaicMbWindupIn w;
    w.ok = have && age_us >= 0 && age_us <= max_age_us && std::fabs(p_rps) < rail_rps;
    w.hz = w.ok ? static_cast<float>(-k * p_rps / (2.0 * 3.14159265358979323846)) : 0.0f;
    return w;
}

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
    // the wind-up estimate (mbWindupEstimate): one value a satellite (its bands' range rates regressed on their
    // wavelengths: the anchor's error, in every difference, taken once), from satellites with at least one band
    // w_min_dlam_m off the anchor's wavelength, no one satellite more than w_max_share of the weight; the capped
    // weighted median's neighbours within w_clip_k of their own sigmas averaged, at least w_min_sats satellites and
    // w_min_values values of them. Smoothed in time as a random walk of w_smooth_q Hz^2/s (0: each epoch's own):
    // each satellite gated against the prediction (w_gate_k of their sigmas), a step taken only where the gyro's
    // agrees or the satellites that moved outweigh those that stayed, and the next epoch confirms it; restarted after
    // w_smooth_gap_s without an estimate; used with a 1-sigma of at most w_max_sigma_hz. Sized on the 10/17 weak sky,
    // the 10/15 indoor mast and its turning runs (gnss-attitude-study/v9-mosaic-filter-plan/evidence/multiband/
    // scripts/prep/wu_tail.cpp) and on the review's capture case (one strong satellite's anchor alone off: break_guard.cpp)
    float w_min_dlam_m = 0.02f;
    int   w_min_values = 6;
    int   w_min_sats = 3;
    float w_max_share = 0.35f;
    float w_max_sigma_hz = 0.75f;
    float w_clip_k = 3.0f;
    float w_smooth_q = 1.0f;
    float w_smooth_gap_s = 0.25f;
    float w_gate_k = 4.0f;
    MosaicMbWSrc w_src = MosaicMbWSrc::EST;
    // the estimate against the gyro's wind-up (whatever w_src, where the caller gives the gyro's): the gyro's own
    // allowance w_gyro_sig_hz plus w_gyro_lag_s times the rate its wind-up changes at; apart by more than max(x_k of
    // the two's sigmas, x_floor_hz) for x_on epochs running and the epochs go anchor-only until x_off agree running
    bool  w_check = true;
    float w_gyro_sig_hz = 0.25f;
    float w_gyro_lag_s = 0.1f;
    float x_k = 5.0f;
    float x_floor_hz = 1.0f;
    int   x_on = 4;
    int   x_off = 10;
    // the anchors' common offset (the monitor): each satellite's anchor less its other bands at W_ref (the gyro's,
    // else W_hat); by scope (each system, and every satellite) with common_min_sats or more, the capped weighted
    // median, a leaky mean over common_tau_s, tripped past max(common_k of its sigma, common_floor_mps + screen_lag_s
    // |a|), let go within common_release of that once tripped for common_hold_s. Every satellite's scope tripped, no
    // system looked at on its own saying otherwise: the epoch anchor-only (the image's rows: the clock's drift takes
    // an offset common to every anchor). A system tripped -- or more than half its satellites, two at least, found
    // off every band they have, all one way, by the screen -- its rows by sys_action: DROP, as the vote would drop
    // each (on the real logs' system-wide faults ANCHOR left the image's 0.3-1.7 m/s errors: the filter does not take
    // a system's anchors off together well; DROP left 12-37 mm/s: gnss-attitude-study/v9-mosaic-filter-plan/
    // evidence/multiband/scripts/fix/fault_runs_fix.py)
    bool  common = true;
    int   common_min_sats = 3;
    float common_tau_s = 1.0f;
    float common_k = 8.0f;
    float common_floor_mps = 0.05f;
    float common_release = 0.5f;
    float common_hold_s = 1.0f;
    float common_gap_s = 1.0f;
    MosaicMbSysAct sys_action = MosaicMbSysAct::DROP;
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
    // past them, at the rig's stop (the whole rig jolted, every band off by up to 1 m/s), it trips. The floor carries
    // the screen's allowance for lag differences under acceleration (screen_lag_s times the leaky mean of |a_hat|).
    // A hold lasts while the satellite is tracked: epochs with nothing to compare (every band screened out, the epoch
    // anchor-only) keep it and, the leak's step at most guard_max_dt_s, neither add to its mean nor wash it out; it
    // ends on agreement, or when the satellite goes untracked for more than guard_gap_s (a new track)
    bool  guard = true;
    float guard_tau_s = 3.0f;
    float guard_k = 8.0f;
    float guard_floor_mps = 0.06f;
    float guard_release = 0.5f;
    float guard_gap_s = 0.5f;
    float guard_max_dt_s = 0.1f;
    uint32_t guard_vote_min_n = 20;
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

// What the mosaic image flies (config.h TR_RAW_MULTIBAND) and tools/raw_replay's --receiver mosaic replays: the
// measured model, on, each anchor's floor the receiver profile's Doppler floor (cn0_min_rr by gnss_sys_t) and the
// builder's white Doppler model (rr_a, rr_b: only an effective C/N0 would use it). The rest is the defaults above:
// the wind-up referred with the combiner's own estimate, checked against the gyro's where the driver gives it, the
// anchor's sigma kept. Each band's lag stays 0: on the 2026-10-24 Goblin boost (7.5 g) the bands lagged their
// anchors by 0.3-1.7 ms (GPS L2C/L5, Galileo E5b/E6, BeiDou B2a) and 4.8-14 ms (Galileo E5a, BeiDou B1C/B3I);
// the screen's allowance (screen_lag_s) takes those, and gating the lagging bands out gave back half of what the
// combination gained on that flight's boost.
inline void mosaicMbFlight(MosaicMultibandConfig& c, const float cn0_min_rr[GNSS_SYS_COUNT], double rr_a, double rr_b)
{
    mosaicMbMeasured(c);
    c.on = true;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.anchor_cn0_min[k] = cn0_min_rr[k];
    c.rr_a = static_cast<float>(rr_a);
    c.rr_b = static_cast<float>(rr_b);
}

// 10^(-(cn0 - 35)/10), the C/N0's share of a band's noise. SBF gives C/N0 in quarter dB, so each quarter's value
// in 0-100 dB-Hz is worked out once (in double, then rounded to mbr): mbSigma ran ~75 times an epoch
// (MosaicMultibandBench.h, on the P4). Anything else is worked each time, in mbr. One caller at a time (the mosaic
// driver's poll task; a harness's one thread).
inline mbr mbCn0Term(mbr cn0)
{
    static mbr tab[401];
    static bool have[401];
    const mbr k = cn0 * mbr(4);
    if (k >= 0 && k <= 400 && k == std::floor(k))
    {
        const int i = static_cast<int>(k);
        if (!have[i])
        {
            tab[i] = static_cast<mbr>(std::pow(10.0, -(static_cast<double>(cn0) - 35.0) / 10.0));
            have[i] = true;
        }
        return tab[i];
    }
    return std::pow(mbr(10), -(cn0 - mbr(35)) / mbr(10));
}

// A band's Doppler sigma at C/N0 cn0 (dB-Hz), m/s; 0 with no model.
inline mbr mbSigma(const MosaicMbBand& m, mbr cn0)
{
    const mbr q = mbCn0Term(cn0);
    const mbr a = m.a, b = m.b;
    return static_cast<mbr>(m.scale) * std::sqrt(a * a + b * b * q);
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
    mbr     lam = 0;
    mbr     d = 0;       // r_b - r_a, m/s
    mbr     dp = 0;      // and less W_hat (lambda_b - lambda_a)
    mbr     sigma = 0;   // its model sigma, m/s
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

// One satellite's wind-up value for an epoch: its anchor's and its candidate bands' range rates regressed on their
// wavelengths (weights 1/sigma^2 from each band's own C/N0), so the anchor's error, which every difference r_b - r_a
// carries, is counted once and not as so many independent values. v its slope (Hz), sig its 1-sigma (Hz), w its own
// weight (1/sig^2) and u that capped (mbCapWeights); the cell and system it came from, its values (bands at least
// w_min_dlam_m from the anchor's wavelength); kept by the epoch's estimate, left out by the smoother's gate.
struct MosaicMbWSat
{
    float   v = 0.0f;
    float   sig = 0.0f;
    float   w = 0.0f;
    float   u = 0.0f;
    int16_t cell = 0;
    uint8_t sys = 0, prn = 0;
    uint8_t nval = 0;
    uint8_t kept = 0;
    uint8_t gated = 0;
};

// Room for the estimate's sorts and capped weights (in the scratch, not on a stack).
struct MosaicMbWork
{
    float I[kMbObs] = {};
    float u[kMbObs] = {};
    int   idx[kMbObs] = {};
    int   sel[kMbObs] = {};
};

// A satellite's state (the drift guard's, and the monitor's): the leaky mean m of its anchor less its other bands'
// combination (m/s, at its combinations), the leaky mean v of that difference's variance, F the mean's share of it
// (m's variance v F: 1/n while it fills, then the leak's alpha / (2 - alpha)), the leaky mean of its anchor's |a_hat|
// (m/s^2: the acceleration allowance), and whether its Doppler is dropped; for the monitor the same of its anchor
// less all its candidate bands at W_ref (every epoch it has any). Its hold lasts while the satellite is tracked.
struct MosaicMbGuard
{
    uint8_t  sys = 0, prn = 0;
    bool     flagged = false;
    uint32_t n = 0;
    int64_t  last_ms = 0;   // its last comparison
    int64_t  seen_ms = 0;   // the last epoch its anchor's Doppler was usable
    mbr      m = 0, v = 0, F = 1, a_abs = 0;
    uint32_t xn = 0;
    int64_t  x_last_ms = 0;
    mbr      xm = 0, xv = 0, xF = 1, xa = 0;
};

// The anchors' common offset for one scope (one system's satellites, or every satellite): the capped weighted median
// of its satellites' leaky means (m/s) and its threshold at the last look (seen_ms, or the screen finding the
// system's anchors off), tripped since since_ms.
struct MosaicMbMon
{
    bool     flagged = false;
    int64_t  since_ms = 0, seen_ms = 0;
    float    m = 0.0f, thr = 0.0f;
};

struct MosaicMbScratch
{
    int          n = 0;
    MosaicMbSat  sat[kMbObs];
    uint8_t      anchor_ch[kMbObs] = {};        // each cell's anchor's receiver channel
    uint8_t      ncand[kMbObs] = {};
    MosaicMbCand cand[kMbObs][kMbMaxCombine];
    // each cell's, while its epoch is combined (here, not on the stack)
    double       anchor_rate[kMbObs] = {};
    mbr          sigma_a[kMbObs] = {};
    mbr          a_hat[kMbObs] = {};
    bool         usable[kMbObs] = {};
    bool         held[kMbObs] = {};      // the drift guard holds it
    int8_t       off[kMbObs] = {};       // the screen: every band flagged, all one way (+1 the anchor above them all)
    bool         has_x[kMbObs] = {};     // the monitor's: its leaky mean of its anchor less its other bands at
    mbr          x[kMbObs] = {};         //   W_ref, that mean's variance, and its leaky mean of |a_hat|
    mbr          xv[kMbObs] = {};
    mbr          xa[kMbObs] = {};
    int          nws = 0;
    MosaicMbWSat wsat[kMbObs];
    MosaicMbWork work;
    bool         prev_valid = false;
    int64_t      prev_rx_ms = 0;
    int          prev_n = 0;
    MosaicMbPrev prev[kMbObs];
    // the wind-up estimate smoothed in time (w_smooth_q): its value (Hz), variance, the instant it stands at and
    // the last estimate it took; a step the satellites (or the gyro) agree on, waiting for the next to confirm it
    bool         ws_ok = false;
    mbr          ws_hz = 0, ws_P = 0;
    int64_t      ws_ms = 0, ws_meas_ms = 0;
    bool         ws_step = false;
    float        ws_step_hz = 0.0f, ws_step_sig = 0.0f;
    // the estimate against the gyro's: tripped (anchor-only), the epochs running that disagreed / agreed, the last
    // comparison; the gyro's wind-up the epoch before (its rate of change widens the allowance)
    bool         x_trip = false;
    int          x_bad = 0, x_good = 0;
    int64_t      x_ms = 0;
    bool         g_have = false;
    float        g_hz = 0.0f;
    int64_t      g_ms = 0;
    // the anchors' common offset by scope: each system (gnss_sys_t), then every satellite; which systems were
    // tripped, and the epoch anchor-only for it, the epoch before (the trips counted)
    MosaicMbMon  mon[GNSS_SYS_COUNT + 1];
    bool         sys_was[GNSS_SYS_COUNT] = {};
    bool         common_was = false;
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
    EPOCH,     // the epoch went anchor-only (no wind-up for the referral, the screen's majority, the estimate against
               //   the gyro's, or an offset common to every anchor)
    VOTED,     // its anchor voted out by two agreeing bands: its Doppler dropped
    COMBINED,  // one or more bands joined
    DRIFT,     // its anchor drifted off its other bands (the drift guard): its Doppler dropped
    SYSTEM,    // its system's anchors off their other bands together: anchor-only (sys_action ANCHOR) or its Doppler
               //   dropped (DROP)
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
    uint8_t     epoch = 0;     // 0 combined; anchor-only: 1 no wind-up for the referral, 2 the screen's majority, 3 the
                               //   estimate and the gyro's apart, 4 an offset common to every anchor
    int         w_n = 0;       // wind-up values (bands)
    int         w_sats = 0;    //   their satellites
    float       w_hat = 0.0f;  // the epoch's wind-up the referral took, Hz (m/s per m of wavelength)
    float       w_sig = 0.0f;  //   its 1-sigma, Hz (0 the gyro's)
    uint8_t     w_src = 0;     //   from: 0 none (anchor-only), 1 the estimate, 2 the gyro
    bool        w_est_ok = false;   // the epoch's own estimate (before any smoothing) had enough satellites and weight
    int         w_kept = 0;         //   its satellites kept by the clip
    float       w_est = 0.0f;       //   its value, Hz (with too few, what they gave)
    float       w_est_sig = 0.0f;   //   and 1-sigma
    int         w_gated = 0;        // satellites the smoother's prediction left out
    bool        w_ext_ok = false;   // the caller's from the gyro
    float       w_ext = 0.0f;       //   Hz
    uint8_t     x_state = 0;        // the estimate against the gyro's: 0 not compared, 1 agree, 2 apart; +4 tripped
    float       x_d = 0.0f;         //   the estimate less the gyro's, Hz, and the threshold
    float       x_thr = 0.0f;
    uint8_t     sys_flags = 0;      // the common offset: bit g system g tripped, bit 4 every satellite's scope, bit 5
                                    //   the epoch anchor-only for it; bit 6 a system's anchors off at once (the screen)
    float       mon_r = 0.0f;       //   the largest |mean| / its threshold over the scopes this epoch
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
    float       x_mps[kMbObs] = {};        // the monitor's anchor less its other bands at W_ref, m/s (0: none)
};

struct MosaicMultibandStats
{
    uint32_t epochs = 0;            // combinations run (grid and off-grid epochs)
    uint32_t epochs_few_w = 0;      //   anchor-only: no wind-up for the referral (the estimate short, no gyro's)
    uint32_t epochs_screen = 0;     //   anchor-only: more than half the epoch's bands flagged
    uint32_t epochs_xcheck = 0;     //   anchor-only: the estimate and the gyro's apart (w_check)
    uint32_t epochs_common = 0;     //   anchor-only: an offset common to every anchor (the monitor)
    uint32_t epochs_w_est = 0;      //   referred with the estimate
    uint32_t epochs_w_gyro = 0;     //     with the gyro's
    // the estimate against the gyro's, the epochs with both (the estimate as the referral would take it, smoothed):
    // the difference's sum and sum of squares, Hz, and the regression through 0 of the one on the other
    uint32_t wx_n = 0;
    double   wx_sum = 0.0, wx_sum2 = 0.0, wx_xy = 0.0, wx_xx = 0.0;
    uint32_t x_trips = 0;           // the check tripped (each time)
    float    x_max_r = 0.0f;        // the largest |difference| / its threshold
    // the estimate
    uint32_t w_gated = 0;           // satellite-epochs the smoother's prediction left out
    uint32_t w_steps = 0;           // steps taken (the satellites or the gyro agreeing)
    uint32_t w_steps_refused = 0;   // epochs whose estimate stood past the gate with too little agreement
    uint32_t w_vote_out = 0;        // satellite-epochs the vote at the prediction left out of the estimate
    // the anchors' common offset
    uint32_t common_trips = 0;      // the epochs sent anchor-only for it (each time it starts)
    uint32_t sys_trips[GNSS_SYS_COUNT] = {};   // a system's rule acting (each time it starts), by gnss_sys_t
    uint32_t sys_rows = 0;          // rows the system rule took (anchor-only or dropped)
    uint32_t sys_instant = 0;       // system-epochs a system's anchors were off at once (the screen)
    float    mon_max_r = 0.0f;      // the largest |mean| / its threshold over every scope
    // the drift guard
    uint32_t guard_trips = 0;       // satellites flagged (each time one is)
    uint32_t guard_rows = 0;        // rows whose Doppler it dropped
    uint32_t guard_looks = 0;       // rows it averaged
    uint32_t guard_held_nocmp = 0;  // of guard_rows, held with nothing to compare (every band screened, anchor-only)
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

// The weights of n satellites from their own I[k] (1/sigma^2), capped so that none holds more than share of their
// sum: u_k = min(I_k, tau), tau found by water-filling (the strongest capped first). With too few satellites for
// that (n share <= 1), all equal; share 1 or more (or none), their own. idx: room for n indices.
inline void mbCapWeights(const float* I, float* u, int n, float share, int* idx)
{
    if (n <= 0)
        return;
    if (!(share > 0.0f) || !(share < 1.0f))
    {
        for (int k = 0; k < n; ++k)
            u[k] = I[k];
        return;
    }
    if (static_cast<float>(n) * share <= 1.0f)
    {
        for (int k = 0; k < n; ++k)
            u[k] = 1.0f;
        return;
    }
    float total = 0.0f;
    for (int k = 0; k < n; ++k)
    {
        u[k] = I[k];
        total += I[k];
        int j = k;
        while (j > 0 && I[idx[j - 1]] < I[k])
        {
            idx[j] = idx[j - 1];
            --j;
        }
        idx[j] = k;
    }
    if (!(total > 0.0f) || I[idx[0]] <= share * total)
        return;
    // c of them capped at tau, the rest their own: tau = share (c tau + rest), so tau = share rest / (1 - share c);
    // the first c at which the next is under tau (each earlier c failing keeps tau under the c-th: see the review)
    float rest = total;
    for (int c = 1; c < n; ++c)
    {
        rest -= I[idx[c - 1]];
        const float den = 1.0f - share * static_cast<float>(c);
        if (!(den > 0.0f))
            break;
        const float tau = share * rest / den;
        if (I[idx[c]] <= tau)
        {
            for (int k = 0; k < c; ++k)
                u[idx[k]] = tau;
            return;
        }
    }
    for (int k = 0; k < n; ++k)
        u[k] = 1.0f;
}

// The epoch's wind-up estimate, Hz, from n satellites' values (one a satellite, with its own 1-sigma: MosaicMbWSat),
// no satellite's weight more than max_share of the sum (mbCapWeights): the capped weighted median, then the
// weighted mean of the satellites within clip_k of their own sigmas of it, their weights capped again among
// themselves. Its 1-sigma: sqrt(sum u^2 sigma^2) / sum u (a capped weight is not the inverse variance, so not
// (sum u)^-1/2). Marks each satellite kept, with its capped weight (u; 0 not kept). True with at least min_sats
// satellites and min_values values kept and
// a 1-sigma of at most max_sigma_hz (hz, sig, kept_sats and kept_values set either way, 0 with none). Reorders s.
// Float throughout: the P4's FPU.
inline bool mbWindupEstimate(MosaicMbWSat* s, int n, int min_sats, int min_values, float max_sigma_hz, float clip_k,
                             float max_share, MosaicMbWork& wk, float& hz, float& sig, int& kept_sats,
                             int& kept_values)
{
    hz = 0.0f;
    sig = 0.0f;
    kept_sats = 0;
    kept_values = 0;
    for (int i = 0; i < n; ++i)
        s[i].kept = 0;
    if (n <= 0)
        return false;
    std::sort(s, s + n, [](const MosaicMbWSat& a, const MosaicMbWSat& b) { return a.v < b.v; });
    for (int i = 0; i < n; ++i)
        wk.I[i] = s[i].w;
    mbCapWeights(wk.I, wk.u, n, max_share, wk.idx);
    float wt = 0.0f;
    for (int i = 0; i < n; ++i)
        wt += wk.u[i];
    if (!(wt > 0.0f))
        return false;
    // the weighted median: where the running weight reaches half (between two values at exactly half, their mean)
    float run = 0.0f, med = s[n - 1].v;
    for (int i = 0; i < n; ++i)
    {
        run += wk.u[i];
        if (run >= 0.5f * wt)
        {
            med = (run == 0.5f * wt && i + 1 < n) ? 0.5f * (s[i].v + s[i + 1].v) : s[i].v;
            break;
        }
    }
    int m = 0;
    for (int i = 0; i < n; ++i)
    {
        const float d = s[i].v - med;
        if (d * d <= clip_k * clip_k * s[i].sig * s[i].sig)
            wk.sel[m++] = i;
    }
    if (m == 0)
        return false;
    for (int k = 0; k < m; ++k)
        wk.I[k] = s[wk.sel[k]].w;
    mbCapWeights(wk.I, wk.u, m, max_share, wk.idx);
    float sw = 0.0f, swv = 0.0f, sv2 = 0.0f;
    for (int i = 0; i < n; ++i)
        s[i].u = 0.0f;
    for (int k = 0; k < m; ++k)
    {
        MosaicMbWSat& x = s[wk.sel[k]];
        sw += wk.u[k];
        swv += wk.u[k] * x.v;
        sv2 += wk.u[k] * wk.u[k] * x.sig * x.sig;
        x.kept = 1;
        x.u = wk.u[k];
        kept_values += x.nval;
    }
    if (!(sw > 0.0f))
        return false;
    kept_sats = m;
    hz = swv / sw;
    sig = std::sqrt(sv2) / sw;
    return kept_sats >= min_sats && kept_values >= min_values && sig <= max_sigma_hz;
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
// one seen longest ago taken over when every entry is in use; nullptr with
// none and make false.
inline MosaicMbGuard* mbGuardOf(MosaicMbScratch& scr, uint8_t sys, uint8_t prn, bool make, int64_t rx_ms = 0)
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
            if (scr.guard[j].seen_ms < scr.guard[at].seen_ms)
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
    g.seen_ms = rx_ms;
    return &g;
}

// A tracked satellite's guard entry (its anchor's Doppler usable this epoch), if it has one: started over (its hold
// with it) when the satellite went untracked for more than guard_gap_s -- a new track -- and seen now. Its hold
// lasts as long as the satellite is tracked, whatever the epochs have to compare: it ends only on agreement (the
// leaky mean back within guard_release of its threshold) or with the track.
inline MosaicMbGuard* mbGuardTouch(MosaicMbScratch& scr, uint8_t sys, uint8_t prn, int64_t rx_ms,
                                   const MosaicMultibandConfig& c)
{
    MosaicMbGuard* g = mbGuardOf(scr, sys, prn, false);
    if (g == nullptr)
        return nullptr;
    if (rx_ms < g->seen_ms || rx_ms - g->seen_ms > static_cast<int64_t>(c.guard_gap_s * 1000.0f))
    {
        *g = MosaicMbGuard{};
        g->sys = sys;
        g->prn = prn;
    }
    g->seen_ms = rx_ms;
    return g;
}

// A leaky mean (the guard's and the monitor's): x of variance vx and the acceleration a_abs folded in, the leak's
// alpha 1 - exp(-dt / tau) with dt at most max_dt_s (time with nothing to compare neither adds to the mean nor
// washes it out), at least 1/n while it fills.
// exp(-dt / tau_s) for mbLeak, the last few (dt, tau) pairs kept: an epoch's steps repeat (50 ms, the cap), ~40 an
// epoch; to the bit what it would be. One caller at a time.
inline mbr mbLeakExp(mbr dt, float tau_s)
{
    struct Memo
    {
        mbr   dt = 0;
        float tau = 0.0f;
        mbr   v = 0;
        bool  ok = false;
    };
    static Memo memo[4];
    static int next = 0;
    for (const Memo& e : memo)
        if (e.ok && e.dt == dt && e.tau == tau_s)
            return e.v;
    const mbr v = std::exp(-dt / tau_s);
    memo[next].dt = dt;
    memo[next].tau = tau_s;
    memo[next].v = v;
    memo[next].ok = true;
    next = (next + 1) & 3;
    return v;
}

inline void mbLeak(uint32_t& n, int64_t& last_ms, mbr& m, mbr& v, mbr& F, mbr& a_abs, int64_t rx_ms,
                   mbr x, mbr vx, mbr a, float tau_s, float max_dt_s)
{
    mbr dt = static_cast<mbr>(rx_ms - last_ms) * mbr(1.0e-3);
    if (dt > max_dt_s)
        dt = max_dt_s;
    const mbr leak = n == 0 ? mbr(1) : mbr(1) - mbLeakExp(dt, tau_s);
    const mbr fill = mbr(1) / static_cast<mbr>(n + 1);
    const mbr al = fill > leak ? fill : leak;
    m += al * (x - m);
    v += al * (vx - v);
    a_abs += al * (std::fabs(a) - a_abs);
    F = (mbr(1) - al) * (mbr(1) - al) * F + al * al;
    n++;
    last_ms = rx_ms;
}

// The guard's update with a combination's x, its anchor less the others' combination (m/s), of variance vx, and its
// anchor's a_hat: the leaky mean, its sigma, and the flag -- set past max(guard_k sigma, guard_floor_mps +
// screen_lag_s |a|), |a| the leaky mean of |a_hat| (the screen's allowance for lag differences between the bands
// under acceleration), let go within guard_release of that. m and sig: the mean and its sigma, for out.
inline void mbGuardUpdate(MosaicMbGuard& g, int64_t rx_ms, mbr x, mbr vx, mbr a_hat,
                          const MosaicMultibandConfig& c, MosaicMultibandStats& st, float& m, float& sig)
{
    if (g.n > 0 && rx_ms <= g.last_ms)
        return;   // this instant already counted
    mbLeak(g.n, g.last_ms, g.m, g.v, g.F, g.a_abs, rx_ms, x, vx, a_hat, c.guard_tau_s, c.guard_max_dt_s);
    const mbr sm = std::sqrt(g.v * g.F);
    const mbr am = std::fabs(g.m);
    const mbr thr = std::max(static_cast<mbr>(c.guard_k) * sm,
                                static_cast<mbr>(c.guard_floor_mps) + static_cast<mbr>(c.screen_lag_s) * g.a_abs);
    st.guard_looks++;
    if (sm > 0 && am / sm > st.guard_max_z)
        st.guard_max_z = static_cast<float>(am / sm);
    if (g.F <= mbr(0.05) && am > st.guard_max_abs)
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

// A cell's candidates screened at wind-up W: each one's dp (r_b - r_a less W (lambda_b - lambda_a)) and flag
// (more than max(screen_k of the pair's sigma, thr_lag) off), written to cand with write; the flagged count, with
// vote (two flagged bands within that of each other: its anchor out, with anchor_vote) and off (every band flagged
// and all one way: +1 the anchor above them all, -1 below, else 0).
inline int mbScreen(MosaicMbCand* cand, int nc, mbr sa, mbr lam_a, mbr W, mbr thr_lag,
                    const MosaicMultibandConfig& c, bool write, bool& vote, int& off)
{
    int nf = 0, above = 0, below = 0;
    mbr dp[kMbMaxCombine] = {};
    uint8_t fl[kMbMaxCombine] = {};
    for (int q = 0; q < nc; ++q)
    {
        const MosaicMbCand& cd = cand[q];
        dp[q] = cd.d - W * (cd.lam - lam_a);
        const mbr thr = std::max(c.screen_k * std::sqrt(sa * sa + cd.sigma * cd.sigma), thr_lag);
        fl[q] = std::fabs(dp[q]) > thr ? 1 : 0;
        nf += fl[q];
        if (fl[q])
            (dp[q] < 0 ? above : below)++;
        if (write)
        {
            cand[q].dp = dp[q];
            cand[q].flagged = fl[q];
        }
    }
    vote = false;
    if (c.anchor_vote && nf >= 2)
        for (int q = 0; q < nc && !vote; ++q)
            for (int r = q + 1; r < nc && !vote; ++r)
            {
                if (!fl[q] || !fl[r])
                    continue;
                const mbr thr = std::max(c.screen_k * std::sqrt(cand[q].sigma * cand[q].sigma +
                                                                    cand[r].sigma * cand[r].sigma),
                                            thr_lag);
                vote = std::fabs(dp[q] - dp[r]) <= thr;
            }
    off = (nc > 0 && nf == nc) ? (above == nc ? 1 : (below == nc ? -1 : 0)) : 0;
    return nf;
}

// The epoch's W_hat for the referral from its satellites' values (ws, nws; nvals values in all) and the caller's
// (w_src): the estimate (each epoch's own, or smoothed in time with w_smooth_q), else (AUTO, GYRO) the gyro's.
// The smoothing (a scalar Kalman filter on a random walk of w_smooth_q Hz^2/s) gates each satellite against its
// prediction (more than w_gate_k of their sigmas apart: left out) and takes the rest, at least two, in one capped
// weighted mean; it coasts through an epoch with fewer. A step (the epoch's own estimate past the gate) is taken only
// where the gyro's agrees with it (the gyro's at hand) or, without one, where the satellites that moved to it outweigh
// those that stayed, two at least -- and then only once the next epoch confirms it: one satellite, or one bad epoch,
// never moves it. With the gyro's at hand and w_check, the estimate as the referral would take it is compared with
// it each epoch: apart by more than max(x_k of their sigmas, x_floor_hz) for x_on epochs running and every epoch
// goes anchor-only (out.x_state + 4) until x_off running agree. True with a W_hat, out's w_* fields saying which;
// false: the epoch goes anchor-only (out.x_state says whether the check sent it).
inline bool mbWindupPick(MosaicMbWSat* ws, int nws, int nvals, int64_t rx_ms, const MosaicMultibandConfig& c,
                         const MosaicMbWindupIn& wext, MosaicMbScratch& scr, MosaicMultibandStats& st, MosaicMbOut& out)
{
    // the gyro's allowance: its own, plus its lag times the rate its wind-up changes at
    float sg = c.w_gyro_sig_hz;
    if (wext.ok)
    {
        if (scr.g_have && rx_ms > scr.g_ms && rx_ms - scr.g_ms <= kMbPrevMaxMs)
            sg += c.w_gyro_lag_s * std::fabs(wext.hz - scr.g_hz) / (static_cast<float>(rx_ms - scr.g_ms) * 1.0e-3f);
        scr.g_have = true;
        scr.g_hz = wext.hz;
        scr.g_ms = rx_ms;
    }
    else
    {
        scr.g_have = false;
    }
    float est = 0.0f, sig = 0.0f;
    int kept = 0, kept_values = 0;
    const bool est_ok = mbWindupEstimate(ws, nws, c.w_min_sats, c.w_min_values, c.w_max_sigma_hz, c.w_clip_k,
                                         c.w_max_share, scr.work, est, sig, kept, kept_values);
    // enough satellites and values, whatever its sigma: the smoother starts on it, and judges a step by it
    const bool est_enough = kept >= c.w_min_sats && kept_values >= c.w_min_values;
    out.w_n = nvals;
    out.w_sats = nws;
    out.w_kept = kept;
    out.w_est = est;
    out.w_est_sig = sig;
    out.w_est_ok = est_ok;
    out.w_gated = 0;
    out.w_ext_ok = wext.ok;
    out.w_ext = wext.ok ? wext.hz : 0.0f;
    out.w_src = 0;
    out.w_hat = 0.0f;
    out.w_sig = 0.0f;
    out.x_state = 0;
    out.x_d = 0.0f;
    out.x_thr = 0.0f;
    bool use_ok = est_ok;
    mbr use = est, use_sig = sig;
    if (c.w_smooth_q > 0.0f)
    {
        const bool live = scr.ws_ok && rx_ms > scr.ws_ms &&
                          rx_ms - scr.ws_meas_ms <= static_cast<int64_t>(c.w_smooth_gap_s * 1000.0f);
        if (live)
        {
            mbr P = scr.ws_P + static_cast<mbr>(c.w_smooth_q) * static_cast<mbr>(rx_ms - scr.ws_ms) * mbr(1.0e-3);
            scr.ws_ms = rx_ms;
            const mbr k2 = static_cast<mbr>(c.w_gate_k) * c.w_gate_k;
            for (int i = 0; i < nws; ++i)
            {
                const mbr nu = static_cast<mbr>(ws[i].v) - scr.ws_hz;
                ws[i].gated = nu * nu > k2 * (P + static_cast<mbr>(ws[i].sig) * ws[i].sig) ? 1 : 0;
                out.w_gated += ws[i].gated;
            }
            st.w_gated += static_cast<uint32_t>(out.w_gated);
            const mbr R = static_cast<mbr>(sig) * sig;
            const mbr nu_e = static_cast<mbr>(est) - scr.ws_hz;
            bool step = false;
            if (est_enough && nu_e * nu_e > k2 * (P + R))
            {
                // the epoch's estimate past the gate: a step (or one bad epoch) only if the gyro's agrees with it, or
                // without one, if the satellites that moved to it (nearer it than the prediction, and past half the
                // gate from that) outweigh those that stayed (the other way round), two at least
                if (wext.ok)
                {
                    const mbr dg = static_cast<mbr>(est) - wext.hz;
                    step = dg * dg <= k2 * (R + static_cast<mbr>(sg) * sg);
                }
                else
                {
                    for (int i = 0; i < nws; ++i)
                        scr.work.I[i] = ws[i].w;
                    mbCapWeights(scr.work.I, scr.work.u, nws, c.w_max_share, scr.work.idx);
                    const mbr h2 = mbr(0.25) * k2;
                    mbr moved = 0, stayed = 0;
                    int nmoved = 0;
                    for (int i = 0; i < nws; ++i)
                    {
                        const mbr de = static_cast<mbr>(ws[i].v) - est;
                        const mbr nu = static_cast<mbr>(ws[i].v) - scr.ws_hz;
                        const mbr s2 = static_cast<mbr>(ws[i].sig) * ws[i].sig;
                        if (de * de < nu * nu && nu * nu > h2 * (P + s2))
                        {
                            moved += scr.work.u[i];
                            nmoved++;
                        }
                        else if (nu * nu <= de * de && de * de > h2 * (R + s2))
                        {
                            stayed += scr.work.u[i];
                        }
                    }
                    step = nmoved >= 2 && moved > stayed;
                }
                if (!step)
                    st.w_steps_refused++;
            }
            if (step)
            {
                const mbr dp = static_cast<mbr>(est) - scr.ws_step_hz;
                if (scr.ws_step && dp * dp <= k2 * (R + static_cast<mbr>(scr.ws_step_sig) * scr.ws_step_sig))
                {
                    scr.ws_hz = est;   // the step confirmed: started over at this epoch's
                    P = R;
                    scr.ws_step = false;
                    scr.ws_meas_ms = rx_ms;
                    st.w_steps++;
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
                // the satellites within the gate, their weights capped among themselves: one mean, its own variance
                int m = 0;
                for (int i = 0; i < nws; ++i)
                    if (!ws[i].gated)
                    {
                        scr.work.sel[m] = i;
                        scr.work.I[m++] = ws[i].w;
                    }
                if (m >= 2)
                {
                    mbCapWeights(scr.work.I, scr.work.u, m, c.w_max_share, scr.work.idx);
                    mbr sw = 0, swv = 0, sv2 = 0;
                    for (int k = 0; k < m; ++k)
                    {
                        const MosaicMbWSat& x = ws[scr.work.sel[k]];
                        sw += scr.work.u[k];
                        swv += scr.work.u[k] * static_cast<mbr>(x.v);
                        sv2 += static_cast<mbr>(scr.work.u[k]) * scr.work.u[k] * x.sig * x.sig;
                    }
                    if (sw > 0)
                    {
                        const mbr z = swv / sw, Rm = sv2 / (sw * sw);
                        const mbr K = P / (P + Rm);
                        scr.ws_hz += K * (z - scr.ws_hz);
                        P *= mbr(1) - K;
                        scr.ws_step = false;
                        scr.ws_meas_ms = rx_ms;
                    }
                }
            }
            scr.ws_P = P;
        }
        else if (est_enough)
        {
            scr.ws_ok = true;
            scr.ws_hz = est;
            scr.ws_P = static_cast<mbr>(sig) * sig;
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
        const mbr d = use - static_cast<mbr>(wext.hz);
        st.wx_n++;
        st.wx_sum += static_cast<double>(d);
        st.wx_sum2 += static_cast<double>(d) * static_cast<double>(d);
        st.wx_xy += static_cast<double>(use) * static_cast<double>(wext.hz);
        st.wx_xx += static_cast<double>(wext.hz) * static_cast<double>(wext.hz);
        if (c.w_check)
        {
            const mbr thr = std::max(static_cast<mbr>(c.x_k) * std::sqrt(use_sig * use_sig +
                                                                               static_cast<mbr>(sg) * sg),
                                        static_cast<mbr>(c.x_floor_hz));
            const bool apart = std::fabs(d) > thr;
            out.x_state = apart ? 2 : 1;
            out.x_d = static_cast<float>(d);
            out.x_thr = static_cast<float>(thr);
            if (std::fabs(d) / thr > st.x_max_r)
                st.x_max_r = static_cast<float>(std::fabs(d) / thr);
            scr.x_bad = apart ? scr.x_bad + 1 : 0;
            scr.x_good = apart ? 0 : scr.x_good + 1;
            scr.x_ms = rx_ms;
            if (!scr.x_trip && scr.x_bad >= c.x_on)
            {
                scr.x_trip = true;
                st.x_trips++;
            }
            else if (scr.x_trip && scr.x_good >= c.x_off)
            {
                scr.x_trip = false;
            }
        }
    }
    if (scr.x_trip && (rx_ms < scr.x_ms || rx_ms - scr.x_ms > 1000))
    {
        scr.x_trip = false;   // nothing compared for a second: the check has nothing to say
        scr.x_bad = scr.x_good = 0;
    }
    if (scr.x_trip)
    {
        out.x_state += 4;
        return false;
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
    st.w_sum2 += static_cast<double>(out.w_hat) * static_cast<double>(out.w_hat);
    st.w_epochs++;
    if (std::fabs(out.w_hat) > st.w_max_abs)
        st.w_max_abs = std::fabs(out.w_hat);
    return true;
}

// The combination over an epoch's cells (RawObs or MosaicDopplerCell: sys,
// prn, has_rate, cn0_dbhz, rate_mps), their anchors' channels in
// scr.anchor_ch, the extra bands in scr.sat, the gyro's wind-up in wext: each
// usable row's rate_mps combined (or its has_rate cleared where its anchor is
// voted out, the drift guard holds it, or its system's anchors are off together
// with sys_action DROP), by the rules at the top of this file. out says what each
// row became.
template <class Cell>
inline void mbCombine(Cell* cells, int n, int64_t rx_ms, const MosaicMultibandConfig& c, const MosaicMbWindupIn& wext,
                      MosaicMbScratch& scr, MosaicMultibandStats& st, MosaicMbOut& out)
{
    out.rx_ms = rx_ms;
    out.n = n;
    out.epoch = 0;
    out.w_n = 0;
    out.w_sats = 0;
    out.w_hat = 0.0f;
    out.sys_flags = 0;
    out.mon_r = 0.0f;
    st.epochs++;
    const bool prev_ok = scr.prev_valid && rx_ms > scr.prev_rx_ms && rx_ms - scr.prev_rx_ms <= kMbPrevMaxMs;
    const mbr prev_dt = prev_ok ? static_cast<mbr>(rx_ms - scr.prev_rx_ms) * mbr(1.0e-3) : mbr(0);
    auto prevOf = [&](uint8_t sys, uint8_t prn) -> const MosaicMbPrev* {
        if (!prev_ok)
            return nullptr;
        for (int j = 0; j < scr.prev_n; ++j)
            if (scr.prev[j].sys == sys && scr.prev[j].prn == prn)
                return &scr.prev[j];
        return nullptr;
    };
    double* const anchor_rate = scr.anchor_rate;
    mbr* const sigma_a = scr.sigma_a;
    bool* const usable = scr.usable;
    MosaicMbWSat* const ws = scr.wsat;
    int nws = 0, nvals = 0;
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
        out.x_mps[i] = 0.0f;
        scr.ncand[i] = 0;
        scr.held[i] = false;
        scr.off[i] = 0;
        scr.has_x[i] = false;
        scr.a_hat[i] = 0;
        anchor_rate[i] = e.rate_mps;
        const uint8_t asig = mbAnchorSig(e.sys);
        const mbr lam_a = static_cast<mbr>(kMbLambda.v[asig]);
        out.lam_wu[i] = kMbLambda.v[asig];
        const mbr sa = e.cn0_dbhz > 0.0f ? mbSigma(c.band[asig], e.cn0_dbhz) : mbr(0);
        sigma_a[i] = sa;
        out.sigma_a[i] = static_cast<float>(sa);
        out.sigma_c[i] = static_cast<float>(sa);
        const float floor_a = e.sys < GNSS_SYS_COUNT ? c.anchor_cn0_min[e.sys] : 0.0f;
        usable[i] = e.has_rate && !(e.cn0_dbhz < floor_a) && sa > 0;
        if (!usable[i])
            continue;
        st.sats++;
        out.row[i] = MosaicMbRow::ALONE;
        // the drift guard: its hold lasts while the satellite is tracked
        if (c.guard)
        {
            const MosaicMbGuard* g = mbGuardTouch(scr, e.sys, e.prn, rx_ms, c);
            scr.held[i] = g != nullptr && g->flagged;
        }
        const MosaicMbPrev* p = prevOf(e.sys, e.prn);
        scr.a_hat[i] = p != nullptr && p->has_rate ? static_cast<mbr>(anchor_rate[i] - p->rate_mps) / prev_dt : mbr(0);
        const MosaicMbSat* m = nullptr;
        for (int j = 0; j < scr.n && m == nullptr; ++j)
            if (scr.sat[j].sys == e.sys && scr.sat[j].prn == e.prn)
                m = &scr.sat[j];
        if (m == nullptr)
            continue;
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
            cd.lam = static_cast<mbr>(kMbLambda.v[b.sig]);
            cd.d = static_cast<mbr>((m->base_mps - e.rate_mps) + static_cast<double>(b.dr));
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
        // its wind-up value: its anchor's and its bands' range rates regressed on their wavelengths (relative to the
        // anchor's: x = lambda - lambda_a, y = r - r_a, the anchor at 0, 0), the slope and its sigma -- not from a
        // satellite the drift guard holds (its anchor's drift is in every difference)
        if (scr.held[i] || scr.ncand[i] == 0)
            continue;
        mbr s0 = mbr(1) / (sa * sa), sx = 0, sy = 0, sxx = 0, sxy = 0;
        int nval = 0;
        for (int q = 0; q < scr.ncand[i]; ++q)
        {
            const MosaicMbCand& cd = scr.cand[i][q];
            const mbr dl = cd.lam - lam_a, w = mbr(1) / (cd.sigma * cd.sigma);
            s0 += w;
            sx += w * dl;
            sy += w * cd.d;
            sxx += w * dl * dl;
            sxy += w * dl * cd.d;
            nval += std::fabs(dl) >= c.w_min_dlam_m ? 1 : 0;
        }
        const mbr I = sxx - sx * sx / s0;
        if (nval == 0 || !(I > 0) || nws >= kMbObs)
            continue;
        MosaicMbWSat& v = ws[nws++];
        v.v = static_cast<float>((sxy - sx * sy / s0) / I);
        v.sig = static_cast<float>(mbr(1) / std::sqrt(I));
        v.w = static_cast<float>(I);
        v.u = 0.0f;
        v.cell = static_cast<int16_t>(i);
        v.sys = e.sys;
        v.prn = e.prn;
        v.nval = static_cast<uint8_t>(nval);
        v.kept = 0;
        v.gated = 0;
        nvals += nval;
    }

    // the vote at the smoother's prediction, where it is live: a satellite whose anchor it would vote out stays out
    // of the estimate -- unless those are most of the weight (then the roll rate moved: the smoother's to judge)
    if (c.anchor_vote && c.w_smooth_q > 0.0f && nws > 0 && scr.ws_ok && rx_ms > scr.ws_ms &&
        rx_ms - scr.ws_meas_ms <= static_cast<int64_t>(c.w_smooth_gap_s * 1000.0f))
    {
        mbr wall = 0, wout = 0;
        uint8_t outv[kMbObs] = {};
        for (int k = 0; k < nws; ++k)
        {
            const int i = ws[k].cell;
            const mbr lam_a = static_cast<mbr>(kMbLambda.v[mbAnchorSig(cells[i].sys)]);
            const mbr thr_lag = c.screen_floor_mps + c.screen_lag_s * std::fabs(scr.a_hat[i]);
            bool vote = false;
            int off = 0;
            mbScreen(scr.cand[i], scr.ncand[i], sigma_a[i], lam_a, scr.ws_hz, thr_lag, c, false, vote, off);
            outv[k] = vote ? 1 : 0;
            wall += ws[k].w;
            wout += vote ? ws[k].w : 0.0f;
        }
        if (wout > 0 && mbr(2) * wout <= wall)
        {
            int m = 0;
            for (int k = 0; k < nws; ++k)
            {
                if (outv[k])
                {
                    nvals -= ws[k].nval;
                    st.w_vote_out++;
                    continue;
                }
                ws[m++] = ws[k];
            }
            nws = m;
        }
    }
    scr.nws = nws;

    // the epoch's wind-up (and the check against the gyro's), then the screen
    float w_hat = 0.0f;
    int total = 0, flagged = 0;
    bool epoch_alone = false;
    const bool have_w = mbWindupPick(ws, nws, nvals, rx_ms, c, wext, scr, st, out);
    if (!have_w)
    {
        epoch_alone = true;
        if (out.x_state >= 4)
        {
            out.epoch = 3;   // the estimate and the gyro's apart
            st.epochs_xcheck++;
        }
        else
        {
            // no wind-up to refer by (the estimate short, no gyro's to stand in; an epoch with no band at all counts
            // here too)
            out.epoch = 1;
            st.epochs_few_w++;
        }
    }
    else
    {
        w_hat = out.w_hat;
        for (int i = 0; i < n; ++i)
        {
            if (!usable[i] || scr.ncand[i] == 0)
                continue;
            const mbr lam_a = static_cast<mbr>(kMbLambda.v[mbAnchorSig(cells[i].sys)]);
            const mbr thr_lag = c.screen_floor_mps + c.screen_lag_s * std::fabs(scr.a_hat[i]);
            bool vote = false;
            int off = 0;
            flagged += mbScreen(scr.cand[i], scr.ncand[i], sigma_a[i], lam_a, w_hat, thr_lag, c, true, vote, off);
            total += scr.ncand[i];
            scr.off[i] = static_cast<int8_t>(off);
        }
        if (2 * flagged > total)
        {
            epoch_alone = true;
            out.epoch = 2;
            st.epochs_screen++;
        }
    }

    // the anchors' common offset: each satellite's anchor less its other bands (all its candidates) at W_ref (the
    // gyro's where given, else W_hat), a leaky mean of it by satellite, and each scope's capped weighted median of
    // those means against its threshold, with a hold; and a system whose anchors the screen finds off their bands at
    // once. The means are taken by satellite before the median, so one satellite's fault stays one outlier among
    // them (a median of each epoch's values would lean towards it by the others' noise)
    bool sys_hit[GNSS_SYS_COUNT] = {};
    bool epoch_common = false;
    if (c.common)
    {
        const bool have_ref = wext.ok || have_w;
        const mbr w_ref = wext.ok ? static_cast<mbr>(wext.hz) : static_cast<mbr>(w_hat);
        if (have_ref)
            for (int i = 0; i < n; ++i)
            {
                if (!usable[i] || scr.ncand[i] == 0)
                    continue;
                const mbr lam_a = static_cast<mbr>(kMbLambda.v[mbAnchorSig(cells[i].sys)]);
                mbr wo = 0, on = 0;
                for (int q = 0; q < scr.ncand[i]; ++q)
                {
                    const MosaicMbCand& cd = scr.cand[i][q];
                    const mbr w = mbr(1) / (cd.sigma * cd.sigma);
                    wo += w;
                    on += w * (cd.d - w_ref * (cd.lam - lam_a));
                }
                MosaicMbGuard* const g = mbGuardOf(scr, cells[i].sys, cells[i].prn, true, rx_ms);
                if (g->xn == 0 || rx_ms > g->x_last_ms)
                    mbLeak(g->xn, g->x_last_ms, g->xm, g->xv, g->xF, g->xa, rx_ms, -on / wo,
                           sigma_a[i] * sigma_a[i] + mbr(1) / wo, scr.a_hat[i], c.common_tau_s, c.guard_max_dt_s);
                scr.has_x[i] = true;
                scr.x[i] = g->xm;
                scr.xv[i] = g->xv * g->xF;
                scr.xa[i] = g->xa;
                out.x_mps[i] = static_cast<float>(g->xm);
            }
        bool looked[GNSS_SYS_COUNT + 1] = {};
        for (int g = 0; g <= GNSS_SYS_COUNT; ++g)
        {
            int m = 0;
            mbr aa = 0;
            for (int i = 0; i < n; ++i)
                if (scr.has_x[i] && scr.xv[i] > 0 && (g == GNSS_SYS_COUNT || cells[i].sys == g))
                {
                    scr.work.sel[m] = i;
                    scr.work.I[m++] = static_cast<float>(mbr(1) / scr.xv[i]);
                    aa += scr.xa[i];
                }
            if (m < c.common_min_sats)
                continue;
            mbCapWeights(scr.work.I, scr.work.u, m, c.w_max_share, scr.work.idx);
            // the capped weighted median (an insertion sort of the scope's few by their means)
            for (int k = 1; k < m; ++k)
            {
                const int si = scr.work.sel[k];
                const float ui = scr.work.u[k];
                int j = k;
                while (j > 0 && scr.x[scr.work.sel[j - 1]] > scr.x[si])
                {
                    scr.work.sel[j] = scr.work.sel[j - 1];
                    scr.work.u[j] = scr.work.u[j - 1];
                    --j;
                }
                scr.work.sel[j] = si;
                scr.work.u[j] = ui;
            }
            mbr wt = 0, sv = 0;
            for (int k = 0; k < m; ++k)
            {
                wt += scr.work.u[k];
                sv += static_cast<mbr>(scr.work.u[k]) * scr.work.u[k] * scr.xv[scr.work.sel[k]];
            }
            mbr run = 0, med = scr.x[scr.work.sel[m - 1]];
            for (int k = 0; k < m; ++k)
            {
                run += scr.work.u[k];
                if (run >= mbr(0.5) * wt)
                {
                    med = (run == mbr(0.5) * wt && k + 1 < m)
                              ? mbr(0.5) * (scr.x[scr.work.sel[k]] + scr.x[scr.work.sel[k + 1]])
                              : scr.x[scr.work.sel[k]];
                    break;
                }
            }
            // a median's variance: (pi / 2) that of the mean with the same weights
            const mbr sm = std::sqrt(mbr(1.5708) * sv) / wt;
            const mbr thr = std::max(static_cast<mbr>(c.common_k) * sm,
                                        static_cast<mbr>(c.common_floor_mps) +
                                            static_cast<mbr>(c.screen_lag_s) * aa / m);
            MosaicMbMon& mg = scr.mon[g];
            mg.m = static_cast<float>(med);
            mg.thr = static_cast<float>(thr);
            mg.seen_ms = rx_ms;
            looked[g] = true;
            const mbr r = std::fabs(med) / thr;
            out.mon_r = std::max(out.mon_r, static_cast<float>(r));
            if (r > st.mon_max_r)
                st.mon_max_r = static_cast<float>(r);
            if (!mg.flagged && r > 1)
            {
                mg.flagged = true;
                mg.since_ms = rx_ms;
            }
            else if (mg.flagged && r < c.common_release &&
                     rx_ms - mg.since_ms >= static_cast<int64_t>(c.common_hold_s * 1000.0f))
            {
                mg.flagged = false;
            }
        }
        // the screen's verdict at once: a system with more than half its satellites (two at least) off every band
        // they have, all one way
        if (have_w)
            for (int g = 0; g < GNSS_SYS_COUNT; ++g)
            {
                int nb = 0, up = 0, down = 0;
                for (int i = 0; i < n; ++i)
                    if (usable[i] && scr.ncand[i] > 0 && cells[i].sys == g)
                    {
                        nb++;
                        up += scr.off[i] > 0;
                        down += scr.off[i] < 0;
                    }
                const int k = up > down ? up : down;
                if (k >= 2 && 2 * k > nb && (up == 0 || down == 0))
                {
                    MosaicMbMon& mg = scr.mon[g];
                    mg.flagged = true;
                    mg.since_ms = mg.seen_ms = rx_ms;
                    st.sys_instant++;
                    out.sys_flags |= 0x40;
                }
            }
        auto fresh = [&](const MosaicMbMon& mg) {
            return mg.flagged && rx_ms >= mg.seen_ms &&
                   rx_ms - mg.seen_ms <= static_cast<int64_t>(c.common_gap_s * 1000.0f);
        };
        for (int g = 0; g < GNSS_SYS_COUNT; ++g)
        {
            if (scr.mon[g].flagged && !fresh(scr.mon[g]))
                scr.mon[g] = MosaicMbMon{};   // not looked at for common_gap_s: nothing to say
            if (fresh(scr.mon[g]))
            {
                out.sys_flags |= static_cast<uint8_t>(1u << g);
                sys_hit[g] = true;
            }
        }
        MosaicMbMon& ma = scr.mon[GNSS_SYS_COUNT];
        if (ma.flagged && !fresh(ma))
            ma = MosaicMbMon{};
        if (fresh(ma))
        {
            // every satellite's: an offset common to the anchors unless a system looked at on its own says otherwise
            // (its mean not at least half its threshold the same way: then it is one system's, its own rule's)
            out.sys_flags |= 0x10;
            bool against = false;
            for (int g = 0; g < GNSS_SYS_COUNT; ++g)
                if (looked[g] && !(scr.mon[g].m * ma.m > 0.0f &&
                                   std::fabs(scr.mon[g].m) >= c.common_release * scr.mon[g].thr))
                    against = true;
            epoch_common = !against;
        }
        if (epoch_common)
        {
            out.sys_flags |= 0x20;
            for (int g = 0; g < GNSS_SYS_COUNT; ++g)
                sys_hit[g] = false;
            if (!epoch_alone)
            {
                epoch_alone = true;
                out.epoch = 4;
                st.epochs_common++;
            }
        }
        for (int g = 0; g < GNSS_SYS_COUNT; ++g)
        {
            st.sys_trips[g] += sys_hit[g] && !scr.sys_was[g] ? 1u : 0u;
            scr.sys_was[g] = sys_hit[g];
        }
        st.common_trips += epoch_common && !scr.common_was ? 1u : 0u;
        scr.common_was = epoch_common;
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
            bo.dp = static_cast<float>(have_w ? cd.dp : cd.d);
        }
        if (!usable[i])
            continue;
        const bool held = scr.held[i];
        auto drift = [&](bool cmp) {
            e.has_rate = false;
            e.rate_mps = anchor_rate[i];
            out.row[i] = MosaicMbRow::DRIFT;
            st.guard_rows++;
            if (!cmp)
                st.guard_held_nocmp++;
        };
        const bool sys_on = e.sys < GNSS_SYS_COUNT && sys_hit[e.sys];
        if (sys_on && c.sys_action == MosaicMbSysAct::DROP)
        {
            if (held)
            {
                drift(false);
                continue;
            }
            e.has_rate = false;
            out.row[i] = MosaicMbRow::SYSTEM;
            st.sys_rows++;
            continue;
        }
        if (nc == 0 || epoch_alone || sys_on)
        {
            if (nc > 0)
            {
                out.row[i] = sys_on && !epoch_alone ? MosaicMbRow::SYSTEM : MosaicMbRow::EPOCH;
                if (sys_on && !epoch_alone)
                    st.sys_rows++;
            }
            if (held)
                drift(false);   // its hold stands: nothing here to judge it by
            continue;
        }
        const mbr thr_lag = c.screen_floor_mps + c.screen_lag_s * std::fabs(scr.a_hat[i]);
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
                    const mbr thr = std::max(c.screen_k * std::sqrt(x.sigma * x.sigma + y.sigma * y.sigma),
                                                thr_lag);
                    vote = std::fabs(x.dp - y.dp) <= thr;
                }
        const mbr dw = wext.ok ? static_cast<mbr>(wext.hz) - w_hat : mbr(0);
        // the guard fed with the anchor less all its bands, flagged or not, where the screen put every band out: the
        // vote is its disagreement too, and so is a last band screened out -- so an epoch the vote misses (one band
        // under the screen) finds the satellite held, not combined, and an anchor that outgrows its only band's
        // screen at once goes the way of one that outgrows it slowly. Once its mean has guard_vote_min_n
        // combinations behind it: a one-epoch glitch does not fill a new mean and hold the satellite. True: held
        auto feedAll = [&]() {
            MosaicMbGuard* const gv = c.guard ? mbGuardOf(scr, e.sys, e.prn, false) : nullptr;
            if (gv == nullptr || gv->n < c.guard_vote_min_n)
                return gv != nullptr && gv->flagged;
            const mbr lam_a = static_cast<mbr>(kMbLambda.v[mbAnchorSig(e.sys)]);
            mbr wa = 0, an = 0, adl = 0;
            for (int q = 0; q < nc; ++q)
            {
                const MosaicMbCand& cd = scr.cand[i][q];
                const mbr w = mbr(1) / (cd.sigma * cd.sigma);
                wa += w;
                an += w * cd.dp;
                adl += w * (cd.lam - lam_a);
            }
            float gm = 0.0f, gs = 0.0f;
            mbGuardUpdate(*gv, rx_ms, -an / wa + dw * adl / wa, sigma_a[i] * sigma_a[i] + mbr(1) / wa, scr.a_hat[i], c,
                          st, gm, gs);
            out.g_m[i] = gm;
            out.g_sig[i] = gs;
            return gv->flagged;
        };
        if (vote)
        {
            e.has_rate = false;
            out.row[i] = MosaicMbRow::VOTED;
            st.voted++;
            feedAll();
            continue;
        }
        st.screened += static_cast<uint32_t>(nf);
        const mbr lam_a = static_cast<mbr>(kMbLambda.v[mbAnchorSig(e.sys)]);
        const mbr sa = sigma_a[i];
        mbr wsum = mbr(1) / (sa * sa);
        mbr num = 0;
        mbr lnum = lam_a * wsum;
        mbr wo = 0, onum = 0;   // the others alone: their weights and their referred differences, summed
        mbr odl = 0;            //   and their wavelengths less the anchor's
        uint64_t mask = 0;
        int nb = 0;
        for (int q = 0; q < nc; ++q)
        {
            const MosaicMbCand& cd = scr.cand[i][q];
            if (cd.flagged)
                continue;
            const mbr w = mbr(1) / (cd.sigma * cd.sigma);
            wsum += w;
            num += w * (c.windup == MosaicMbWindup::REF ? cd.dp : cd.d);
            lnum += w * cd.lam;
            wo += w;
            onum += w * cd.dp;
            odl += w * (cd.lam - lam_a);
            mask |= uint64_t{1} << cd.sig;
            nb++;
        }
        if (nb == 0)
        {
            // every band screened out: the anchor's alone (ALONE) -- or, held (before or by this), its Doppler out:
            // the screen flagging its last band is more of the disagreement, not an end to it
            if (feedAll() || held)
                drift(false);
            continue;
        }
        if (c.guard)
        {
            // the anchor less the others' combination, referred at the gyro's wind-up where given (the estimate's
            // carries any satellite's own pull on it, the faulted one's too), else W_hat: its leaky mean against its
            // own noise
            MosaicMbGuard* const gg = mbGuardOf(scr, e.sys, e.prn, true, rx_ms);
            mbGuardUpdate(*gg, rx_ms, -onum / wo + dw * odl / wo, sa * sa + mbr(1) / wo, scr.a_hat[i], c, st, out.g_m[i],
                          out.g_sig[i]);
            if (gg->flagged)
            {
                drift(true);
                continue;
            }
        }
        for (int q = 0; q < nc; ++q)
            if (!scr.cand[i][q].flagged)
            {
                st.joins[scr.cand[i][q].sig]++;
                out.det[i][q].joined = 1;
                out.det[i][q].w = static_cast<float>(mbr(1) / (scr.cand[i][q].sigma * scr.cand[i][q].sigma) / wsum);
            }
        const mbr dr = num / wsum;
        e.rate_mps = anchor_rate[i] + static_cast<double>(dr);
        out.row[i] = MosaicMbRow::COMBINED;
        out.bands[i] = static_cast<uint8_t>(nb);
        out.mask[i] = mask;
        out.d_mps[i] = static_cast<float>(dr);
        const mbr sc = mbr(1) / std::sqrt(wsum);
        out.sigma_c[i] = static_cast<float>(sc);
        out.lam_wu[i] = c.windup == MosaicMbWindup::REF ? kMbLambda.v[mbAnchorSig(e.sys)] : static_cast<double>(lnum / wsum);
        st.sats_combined++;
        st.ratio_sum += static_cast<double>(sc / sa);
        st.ratio_n++;
        if (c.express_sigma)
            out.rr_cn0_q[i] = mbEffCn0Q(static_cast<double>(e.cn0_dbhz), static_cast<double>(sc / sa),
                                        static_cast<double>(c.rr_a), static_cast<double>(c.rr_b));
    }

    // composition changes, and this epoch's state for the next
    auto dropped = [&](int i) {
        return out.row[i] == MosaicMbRow::VOTED || out.row[i] == MosaicMbRow::DRIFT ||
               (out.row[i] == MosaicMbRow::SYSTEM && !cells[i].has_rate);
    };
    for (int i = 0; i < n; ++i)
    {
        if (!usable[i] || dropped(i))
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
        p.has_rate = usable[i] && !dropped(i);
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
    const double sa_i = static_cast<double>(g.sigma_a[i]), sc_i = static_cast<double>(g.sigma_c[i]);
    ratio = sa_i > 0.0 ? sc_i / sa_i : 1.0;
    lam_wu = g.lam_wu[i];
    if (j < 0)
        return;
    const double sa_j = static_cast<double>(o.sigma_a[j]), sc_j = static_cast<double>(o.sigma_c[j]);
    const double den = sa_i * sa_i + sa_j * sa_j;
    if (den > 0.0)
        ratio = std::sqrt((sc_i * sc_i + sc_j * sc_j) / den);
    lam_wu = 0.5 * (g.lam_wu[i] + o.lam_wu[j]);
}

}  // namespace mosaic

#endif  // MOSAIC_MULTIBAND_H
