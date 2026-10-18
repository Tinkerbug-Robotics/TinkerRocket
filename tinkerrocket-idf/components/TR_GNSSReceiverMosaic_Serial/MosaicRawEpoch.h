// MosaicRawEpoch.h — the mosaic-G5's MeasEpoch as the receiver-neutral observations TR_GNSSRaw builds a raw-GNSS filter's measurements from, and its time tag
//
// Mirrors Lc86RawEpoch.h: one cell per satellite, on the L1-band signal
// whose ephemeris and group delay the filter has (SbfNav.h) and whose
// carrier the receiver profile names: GPS L1 C/A (signal 0), Galileo E1 (17)
// and BeiDou B1I (28), each found wherever MeasEpoch put it (the master
// Type1 or a Type2), from the main antenna. BeiDou GEO is left out (gnss/eph.h
// cannot place it, and a GEO with no ephemeris would only count as tracked
// without one), and so is everything else: GLONASS, SBAS, QZSS, NavIC, and
// BeiDou tracked on B1C alone (B1C code needs the B-CNAV1 group delay).
// There is no satellite cap here: at most 28 picks per epoch on the 10/16
// captures, and the measurement stage caps what it builds.
//
// Units are the LC86G adapter's (RawMeas.h):
//   - pr_m: the pseudorange, m;
//   - rate_mps = -lambda * Doppler: the range rate, positive with the range
//     growing (SBF's Doppler is positive for an approaching satellite). Unlike
//     the LC86G's MSM7 rate it keeps the receiver's clock drift (about 80 m/s
//     on the 10/16 captures); the filter's clock-drift state takes it, seeded
//     from the first epoch's Doppler;
//   - phase_m = pr + carrier offset * lambda: the carrier as a range, with
//     has_phase only when the code, the carrier and the lock time are all
//     valid; halfcyc ObsInfo bit 2;
//   - lock_ms: the lock time (1 s resolution), held to 254 s and to the time
//     since the last clock jump (lockCapMs);
//   - cn0_dbhz = CN0 * 0.25 + 10; a do-not-use C/N0 leaves the cell with no
//     measurement at all.
//
// Clock jumps are left in (the 10/24 rule). The mosaic free-runs its clock
// and steps it a whole millisecond when its bias reaches 0.5 ms: every code
// and carrier moves c * 1 ms together (-299,792.7 m at CumClkJumps 252 -> 251
// on the 10/16 cone-off capture) while the Doppler does not. tow_s, pr_m and
// phase_m stay as the receiver gave them, so tow_s - pr_m / c is still each
// satellite's transmit time and the builder places it right; the filter's
// whole-millisecond clock backstop takes the code step; and every cell's
// lock_ms counts from the jump (lockCapMs), so no carrier interval spans the
// step, whichever epochs between never reach the builder.
//
// The time tag (plan section 2). TOW is receiver time: the GPS instant of an
// epoch is TOW - RxClkBias (t_sys = t_rx - RxClkBias, Reference Guide), so
//   mcu_us = mcuAt(epochGpsS(WNc, TOW, dtrMs(bias, drift, dt, jumps)))
// with the bias of the last PVTGeodetic whose own MeasEpoch was noted
// (pvtAnchorsJumps) carried forward by its drift and by the clock jumps since
// it (a -1 ms jump takes 1 ms off the bias). ppsResidualUs checks a
// whole-second epoch's tag against its 1PPS edge, and notePpsTag keeps that
// check over a run. Which PVT may carry the bias how long, and when the 1PPS
// says the tag can be trusted: MosaicTimeTag.h.
//
// Header-only and pure (tests_cpp/test_mosaic_raw_epoch.cpp).
#ifndef MOSAIC_RAW_EPOCH_H
#define MOSAIC_RAW_EPOCH_H

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "RawMeas.h"
#include "SbfMeas.h"
#include "gnss/types.h"

namespace mosaic
{

static constexpr double kMosaicC = 299792458.0;   // m/s, as gnss/atmo.h's GNSS_C
static constexpr uint32_t kMosaicLockMaxMs = 254000;   // a Type2's LockTime clips at 254 s
static constexpr int64_t  kMosaicMeasGapMs = 2000;     // MeasEpochs this far apart: a receiver reset (MosaicJumpState)
static constexpr int64_t  kMosaicNoArrival = INT64_MIN; // a MeasEpoch's arrival not given: the count alone
static constexpr double   kMosaicAliasTolMs = 64.0;     // the arrivals place a jump the count shows mod 256 within this
static constexpr long     kMosaicAliasMaxK = 8;         //   up to 8 * 256 ms either way (more: a reset)
static constexpr double   kMosaicArrDecayPerS = 100e-6; // the earliest arrival forgets 100 us a second (crystals, delays)

// The receiver's clock jumps, followed from one MeasEpoch to the next.
// CumClkJumps is the jumps since the receiver started, in ms mod 256; a step
// of more than 2 ms (a step in the sky it tracks, on the RF rig), time going
// backwards, or more than kMosaicMeasGapMs of receiver time since the last
// MeasEpoch, and the count starts over there (j_ms, rebases). The last two
// are a receiver reset (or a different log): its clock is lost (resets);
// across a big step it is kept (jsum_ms). The gap is for a reset whose count
// lands within 2 ms of the last (CumClkJumps starts again at 0, a count of
// 254 before reads as a +2 ms jump): a reset is seconds of no MeasEpoch with
// a time, while a lost sky still sends one every epoch. A driver that adapts only
// the epochs on its fuse grid shows the others to noteMeasEpochClock, so the
// count (and the PVT bias it carries forward with dtrMs) misses no jump; a
// header seen twice (noted, then adapted) counts once. Where the last jump or
// start-over was seen (cut_ms) holds every later cell's lock time
// (lockCapMs).
//
// Given each MeasEpoch's arrival on the MCU clock, the count's 256 ms
// ambiguity is resolved too. The receiver's time less the arrival stamp
// stays put but for the clock jumps (the crystals move it microseconds a
// second, the receiver's delay a few ms), and a stamp can be late (the
// reading task held off) but never early: so its most lately (arr_hi_us, run
// on by the jumps and forgetting kMosaicArrDecayPerS) is where an on-time
// epoch sits. An epoch that sits ahead of it by about a k * 256 ms the count
// cannot show (a sky come back early on the RF rig: the receiver steps its
// clock forward by that and its epochs skip ahead) is that jump, k > 0: a
// count of 0 or -1 ms read alone would carry the bias across it 256 ms or
// more wrong. Ahead by anything else, or by k * 256 ms plus more than
// kMosaicAliasTolMs, the time moved by something the count does not hold:
// a reset. A sky come back late makes the receiver step its clock back:
// where its epochs go on on their grid the arrivals fall behind by the step,
// which is the count's plus k * 256 ms, k < 0, placed only where the count
// shows a step (behind with none is a late stamp); where it repeats its
// epochs' times, time goes backwards, which is a reset unless the arrivals
// show it as the count's step plus k * 256 ms, k <= 0. Either way the
// arrival itself must come later than the last. |k| beyond
// kMosaicAliasMaxK is a reset: a receiver whose time moved seconds set it
// anew.
// jsum_ms keeps the resolved jumps across a big jump's start-over, so the
// time tag can carry a bias across one (MosaicTimeTag.h); only a reset
// (resets) loses it. And how late the last epoch came against the earliest
// (late_us) tells how far the link is backed up.
struct MosaicJumpState
{
    bool     have = false;        // an epoch with CumClkJumps seen since the last start-over
    uint8_t  last_cum = 0;
    int64_t  last_ms = 0;         // that epoch's receiver time, ms (WNc * 604800000 + TOW)
    int32_t  j_ms = 0;            // the clock jumps since the start-over, ms
    uint32_t jumps = 0;           // epochs that jumped
    uint32_t rebases = 0;         // start-overs after the first epoch
    bool     cut = false;         // a jump or a start-over seen: the carrier is continuous only since cut_ms
    int64_t  cut_ms = 0;          // the receiver time of the epoch that showed it, ms
    // across the start-overs a big jump makes (the receiver's clock kept)
    int64_t  jsum_ms = 0;         // the clock jumps since the last reset, ms, each resolved (d + 256 k)
    uint32_t resets = 0;          // start-overs that lost the clock: a gap, time back, a jump the count cannot hold
    uint32_t resolved = 0;        // jumps the arrivals placed past the count's 256 ms (k != 0)
    // the receiver's time against the arrival stamps (when given)
    bool     arr_have = false;
    int64_t  arr_hi_us = 0;       // receiver time less arrival, us: the most lately (the earliest arrival)
    int64_t  late_us = 0;         // how far the last epoch sat below it: its stamp's lateness, us
    int64_t  last_arr_us = 0;     // the last epoch's arrival
};

// What one MeasEpoch gave, beside the cells.
struct MosaicEpochInfo
{
    SbfStatus status = SbfStatus::SHORT;
    uint32_t  tow_ms = kTowDnu;
    uint16_t  wnc = kWncDnu;
    uint8_t   common_flags = 0;
    bool      jumped = false;    // the clock jumped jump_ms at this epoch
    bool      rebased = false;   // the jump count started over at this epoch (j_ms from 0)
    bool      reset = false;     //   and lost the receiver's clock (MosaicJumpState::resets)
    int8_t    jump_ms = 0;       // CumClkJumps's step, mod 256
    int32_t   jump_full_ms = 0;  // the step the arrivals placed (jump_ms + 256 k), or jump_ms
    uint32_t  lock_cap_ms = kMosaicLockMaxMs;   // every cell's lock_ms at most this (lockCapMs)
    int32_t   j_ms = 0;          // MosaicJumpState::j_ms after this epoch
    uint8_t   n_gps = 0, n_gal = 0, n_bds = 0;
    uint8_t   geo = 0;           // BeiDou GEO B1I cells left out
    uint8_t   dupes = 0;         // a satellite's signal seen twice: the second left out
    uint8_t   over = 0;          // picks past RawEpoch::kMaxObs left out
};

// Follow CumClkJumps across one epoch's header (revision 1 on; a revision-0
// block says nothing, and nothing changes). arrival_us: the block's arrival
// on the MCU clock (its last byte, as the driver stamps it), or
// kMosaicNoArrival for the count alone (see MosaicJumpState).
inline void noteClockJumps(const SbfMeasHeader& h, MosaicJumpState& js, MosaicEpochInfo& info,
                           int64_t arrival_us = kMosaicNoArrival)
{
    if (h.has_cum_clk_jumps)
    {
        const int64_t t = static_cast<int64_t>(h.wnc) * 604800000LL + h.tow_ms;
        const bool arr = arrival_us != kMosaicNoArrival;
        const int64_t o_us = arr ? t * 1000 - arrival_us : 0;
        // the earliest arrival lately, run on to this epoch (the jump at it not yet in)
        const int64_t hi_us = js.arr_hi_us - static_cast<int64_t>(std::llround(
                                  kMosaicArrDecayPerS * static_cast<double>(t > js.last_ms ? t - js.last_ms : 0) * 1.0e3));
        int32_t jump = 0;
        if (!js.have)
        {
            js.j_ms = 0;
            js.jsum_ms = 0;
            info.rebased = true;
        }
        else
        {
            const int d = static_cast<int8_t>(static_cast<uint8_t>(h.cum_clk_jumps - js.last_cum));
            jump = d;
            bool uncounted = false;
            bool back_counted = false;
            if (arr && js.arr_have && arrival_us > js.last_arr_us)
            {
                // against the earliest arrival, beyond what the count says: k * 256 ms the count
                // cannot show, or something it does not hold at all
                const double past_ms = static_cast<double>(o_us - hi_us) * 1.0e-3 - d;
                const long k = std::lround(past_ms / 256.0);
                const bool placed = k != 0 && k >= -kMosaicAliasMaxK && k <= kMosaicAliasMaxK &&
                                    std::fabs(past_ms - 256.0 * static_cast<double>(k)) <= kMosaicAliasTolMs;
                if (t >= js.last_ms)
                {
                    // ahead (a late stamp only sits lower): a forward step past the count
                    if (past_ms > kMosaicAliasTolMs)
                    {
                        if (placed && k > 0)
                        {
                            jump = d + static_cast<int32_t>(256 * k);
                            js.resolved++;
                        }
                        else
                        {
                            uncounted = true;
                        }
                    }
                    // behind, where the count shows a step: a step back past it, its epochs going on
                    // on their grid (behind with no step counted is a late stamp)
                    else if (past_ms < -kMosaicAliasTolMs && d != 0 && placed && k < 0)
                    {
                        jump = d + static_cast<int32_t>(256 * k);
                        js.resolved++;
                    }
                }
                else if (placed && k < 0)
                {
                    // time back by a step back past the count (a sky come back late)
                    jump = d + static_cast<int32_t>(256 * k);
                    js.resolved++;
                    back_counted = true;
                }
                else if (std::fabs(past_ms) <= kMosaicAliasTolMs)
                {
                    back_counted = true;   // time back by the step the count shows
                }
            }
            info.jumped = jump != 0;
            info.jump_ms = static_cast<int8_t>(d);
            info.jump_full_ms = jump;
            const bool reset = (t < js.last_ms && !back_counted) || t - js.last_ms > kMosaicMeasGapMs || uncounted;
            if (reset || jump > 2 || jump < -2)
            {
                js.j_ms = 0;
                js.rebases++;
                info.rebased = true;
                if (reset)
                {
                    js.resets++;
                    js.jsum_ms = 0;
                    info.reset = true;
                }
                else
                {
                    js.jsum_ms += jump;
                }
            }
            else
            {
                js.j_ms += jump;
                js.jsum_ms += jump;
            }
            if (info.jumped) js.jumps++;
            if (info.jumped || info.rebased)
            {
                js.cut = true;
                js.cut_ms = t;
            }
        }
        if (arr)
        {
            if (!js.arr_have || info.reset)
            {
                js.arr_hi_us = o_us;
            }
            else
            {
                const int64_t run_on = hi_us + static_cast<int64_t>(jump) * 1000;
                js.arr_hi_us = o_us > run_on ? o_us : run_on;
            }
            js.arr_have = true;
            js.late_us = js.arr_hi_us - o_us;
            js.last_arr_us = arrival_us;
        }
        js.have = true;
        js.last_cum = h.cum_clk_jumps;
        js.last_ms = t;
    }
    info.j_ms = js.j_ms;
}

// The most lock time a cell of this epoch may claim, ms. 254 s: a Type1's
// LockTime runs to 65534 s but a Type2's clips at 254, and a satellite's L1
// signal moves between the two levels from epoch to epoch (hundreds of times
// on each 10/16 capture), so past 254 s its lock would go back at a switch,
// which RawMeasCarrier takes for a slip; a relock still shows, as a drop to
// 0-1 s. And no more than the time since the last clock jump or start-over:
// the uncorrected carrier steps c * 1 ms there, so its continuous span starts
// over, and an interval from an epoch before it fails the builder's rule that
// the later end's lock covers the interval. That holds whichever epochs
// between were lost on the way (a full queue, one not published, one the
// stage dropped as stale or left unbuilt), which a lock zeroed at one epoch
// alone would not; it costs the carrier the first cr_min_lock_ms after each
// jump (about one an hour).
inline uint32_t lockCapMs(const SbfMeasHeader& h, const MosaicJumpState& js)
{
    uint32_t cap = kMosaicLockMaxMs;
    if (js.cut)
    {
        const int64_t since = static_cast<int64_t>(h.wnc) * 604800000LL + h.tow_ms - js.cut_ms;
        if (since < static_cast<int64_t>(cap))
            cap = since > 0 ? static_cast<uint32_t>(since) : 0U;   // time back (revision 0, untracked): none
    }
    return cap;
}

// The clock-jump count of a MeasEpoch that is not adapted (off the fuse
// grid): its header only. False if the reader refuses the block. arrival_us
// as noteClockJumps takes it.
inline bool noteMeasEpochClock(const uint8_t* b, size_t len, MosaicJumpState& js, MosaicEpochInfo& info,
                               int64_t arrival_us = kMosaicNoArrival)
{
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
    return true;
}

// One MeasEpoch block (CRC checked) into out: GPS L1 C/A, Galileo E1 and
// BeiDou B1I, one cell per satellite, in block order. False, with out.n = 0,
// for a block the reader refuses (wrong, short, bad layout, scrambled, no
// time) or one with no cell to give; info says which. arrival_us as
// noteClockJumps takes it.
inline bool rawEpochFromMeasEpoch(const uint8_t* b, size_t len, gnss_raw::RawEpoch& out, MosaicJumpState& js,
                                  MosaicEpochInfo& info, int64_t arrival_us = kMosaicNoArrival)
{
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

    SbfMeasSignal s;
    while (rd.next(s))
    {
        if (s.antenna != 0)
            continue;
        uint8_t prn;
        const SbfSys sys = sbfSvid(s.svid, prn);
        uint8_t gsys;
        if (sys == SbfSys::GPS && s.sig == SBF_SIG_GPS_L1CA)
            gsys = GNSS_SYS_GPS;
        else if (sys == SbfSys::GAL && s.sig == SBF_SIG_GAL_E1)
            gsys = GNSS_SYS_GAL;
        else if (sys == SbfSys::BDS && s.sig == SBF_SIG_BDS_B1I)
        {
            if (prn <= 5 || prn >= 59)
            {
                info.geo++;
                continue;
            }
            gsys = GNSS_SYS_BDS;
        }
        else
            continue;

        const bool cn0_ok = s.cn0Valid();
        const bool has_pr = cn0_ok && s.pr_valid;
        // one cell per satellite: a second copy (another channel) only
        // replaces a first that had no code
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
    }
    return out.n > 0;
}

// Whether an epoch is on the fuse grid: its TOW a whole number of periods
// into the week (period 0: every epoch). The profile's periods (50, 100, 200,
// 1000 ms) divide the week, so the grid runs on across the week's end.
inline bool onFuseGrid(uint32_t tow_ms, uint32_t period_ms)
{
    return tow_ms != kTowDnu && (period_ms == 0 || tow_ms % period_ms == 0);
}

// An epoch's GPS time, continuous seconds (week * 604800 + time of week),
// from its receiver time stamp and the receiver's clock bias then (dtr_ms,
// receiver minus GPS): t_sys = t_rx - bias. The whole milliseconds are added
// as integers, so the result is as good as a double at ~1.5e9 s (0.25 us).
inline double epochGpsS(uint16_t wnc, uint32_t tow_ms, double dtr_ms)
{
    const int64_t rx_ms = static_cast<int64_t>(wnc) * 604800000LL + tow_ms;
    return (static_cast<double>(rx_ms) - dtr_ms) * 1.0e-3;
}

// The receiver's clock bias (ms, receiver minus GPS) at an epoch dt_s after
// the PVTGeodetic that stated pvt_bias_ms (RxClkBias) and drift_ppm
// (RxClkDrift, positive with the receiver's clock fast), across
// jumps_since_pvt ms of clock jumps since it (MosaicJumpState::j_ms now less
// at that PVT, while rebases has not changed): a -1 ms jump takes 1 ms off.
// Only a PVT that pvtAnchorsJumps accepted may be carried. The receiver's
// clock is free-running (it only steps whole milliseconds, and only with a
// fix), so the carry holds across a lost sky too: its drift is known to a
// few ppb, a few microseconds over a minute.
inline double dtrMs(double pvt_bias_ms, double drift_ppm, double dt_s, int32_t jumps_since_pvt)
{
    return pvt_bias_ms + drift_ppm * 1.0e-3 * dt_s + jumps_since_pvt;
}

// Whether a PVTGeodetic (its WNc and TOW) may anchor dtrMs, with
// MosaicJumpState::j_ms taken as it arrives: only when the MeasEpoch of its
// own epoch was the last one noted (Stream7 sends the PVT just after it: 65,865
// of 65,866 on the 10/16 captures, the other with no MeasEpoch at all). Its
// RxClkBias already holds any jump at its epoch; if that MeasEpoch was lost (a
// CRC failure, an overrun) at a jump, the count would take the jump only at
// the next MeasEpoch, after the PVT, and every tag carried from that PVT would
// hold it twice -- 1 ms off until the next PVT. Passed over, the PVT before
// it (if young enough) stands in.
inline bool pvtAnchorsJumps(const MosaicJumpState& js, uint16_t wnc, uint32_t tow_ms)
{
    return js.have && js.last_ms == static_cast<int64_t>(wnc) * 604800000LL + tow_ms;
}

// The PPS tag check for a whole-second epoch (TOW % 1000 == 0): its tag
// mcu_us against the 1PPS edge of the same GPS second. The pulse marks the
// GPS second (spps ... GPS), while the receiver's second N falls at GPS time
// N - bias, so a right tag sits bias * 1000 us before the edge -- up to
// 500 us with the clock free-running, which is why the plain difference
// cannot be the check. rx_clk_bias_ms is the receiver's own statement for
// that very epoch (the PVTGeodetic with the same TOW), not the
// carried-forward dtrMs the tag used, so a slip in the tag's arithmetic (a
// whole second, a week, a millisecond of jumps, the bias's sign) shows here.
// A right tag leaves about the clock line's own noise (0.03 ms rms);
// |residual| > 200 us is a bad tag. Saturates at the int32 range (a week's
// slip).
inline int32_t ppsResidualUs(int64_t mcu_us, int64_t pps_edge_us, double rx_clk_bias_ms)
{
    const double r = static_cast<double>(mcu_us - pps_edge_us) + rx_clk_bias_ms * 1000.0;
    if (!(r < 2147483647.0)) return 2147483647;
    if (!(r > -2147483648.0)) return -2147483647 - 1;
    return static_cast<int32_t>(std::lround(r));
}

// The PPS tag check kept over a run, one whole second at a time (the
// driver's RawStats).
struct PpsTagStats
{
    uint32_t checks = 0;            // whole seconds judged (the line had the crystal's drift)
    uint32_t over = 0;              //   with |residual| over the bar
    int32_t  max_us = 0;            //   the largest |residual|
    int32_t  last_us = 0;           // the last residual, signed, judged or not
    int64_t  last_s = 0;            //   its GPS second (a held one's never: see held_last_us)
    uint32_t settling = 0;          // seconds checked before the line had the drift: not judged
    int32_t  settling_max_us = 0;   //   their largest |residual|
    uint32_t late = 0;              // edges the interrupt stamped late: judged where they were due
    int32_t  late_max_us = 0;       //   the latest, us past due
    // seconds whose epoch was not handed over timed (the tag held back:
    // MosaicTimeTag.h), checked all the same, never judged
    uint32_t held = 0;
    int32_t  held_max_us = 0;       //   their largest |residual|
    int32_t  held_last_us = 0;      //   the last one, signed
    // the last second checked and its edge, if that was on time: where the
    // next second's edge is due
    int64_t  prev_s = 0;
    int64_t  prev_edge_us = 0;
    bool     prev_on_time = false;
};

// One whole second's check: its GPS second, its epoch's tag, the 1PPS edge
// paired with that second as the interrupt stamped it, the clock line's MCU
// length of the second that ended there (mcuAt(gps_s) - mcuAt(gps_s - 1)),
// whether the line has fitted the MCU crystal's drift yet
// (GnssTimeSync::driftFitted), and the epoch's own RxClkBias. Two things
// other than a bad tag move the residual, and each is kept apart:
//   - An edge can be stamped late, never early: a flash write holds off the
//     GPIO interrupt (its service is not IRAM-safe), for milliseconds at an
//     erase. Such an edge comes more than the bar after where the second
//     before's edge puts it (one line second on; a crystal moves that by
//     100 us at most), and the tag is judged against where it was due, so a
//     late edge neither reads as a bad tag nor hides one. A line that drifts
//     off the edges moves them a few tens of us a second, never the bar, and
//     stays in the check.
//   - Until the line has fitted the drift (20 s of its 10 s blocks, ~30 s
//     after the first edge) it runs flat, and its tags part from the edges
//     by the crystal's ppm each second: ~0.4 ms by then at the V9's +16 ppm.
//     A known, passing error of the line, not a slip in the tag's arithmetic:
//     those seconds are counted and not judged.
// A second whose epoch was not handed over timed (published false: its tag
// held back) is checked the same way, late edges and all, and counted apart:
// the judged seconds are what the filter got. Returns the residual, against
// where the edge was due when it came late.
inline int32_t notePpsTag(PpsTagStats& s, int64_t gps_s, int64_t tag_mcu_us, int64_t edge_us, int64_t second_us,
                          bool line_settled, double rx_clk_bias_ms, int32_t bar_us, bool published = true)
{
    int64_t ref = edge_us;
    bool late = false;
    if (s.prev_on_time && s.prev_s == gps_s - 1)
    {
        const int64_t due = s.prev_edge_us + second_us;
        if (edge_us - due > bar_us)
        {
            late = true;
            ref = due;
            const int64_t l = edge_us - due;
            const int32_t l32 = l > 2147483647 ? 2147483647 : static_cast<int32_t>(l);
            s.late++;
            if (l32 > s.late_max_us) s.late_max_us = l32;
        }
    }
    s.prev_s = gps_s;
    s.prev_on_time = !late;
    if (!late) s.prev_edge_us = edge_us;

    const int32_t res = ppsResidualUs(tag_mcu_us, ref, rx_clk_bias_ms);
    const int32_t mag = (res == -2147483647 - 1) ? 2147483647 : (res < 0 ? -res : res);
    if (!published)
    {
        s.held++;
        s.held_last_us = res;
        if (mag > s.held_max_us) s.held_max_us = mag;
        return res;
    }
    s.last_us = res;
    s.last_s = gps_s;
    if (!line_settled)
    {
        s.settling++;
        if (mag > s.settling_max_us) s.settling_max_us = mag;
        return res;
    }
    s.checks++;
    if (mag > s.max_us) s.max_us = mag;
    if (mag > bar_us) s.over++;
    return res;
}

}  // namespace mosaic

#endif  // MOSAIC_RAW_EPOCH_H
