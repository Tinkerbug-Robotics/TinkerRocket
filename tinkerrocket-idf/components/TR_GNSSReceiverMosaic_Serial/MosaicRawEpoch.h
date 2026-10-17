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
//     valid; lock_ms the lock time (1 s resolution); halfcyc ObsInfo bit 2;
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
// lock_ms is 0 on the first epoch with cells after the jump, so no carrier
// interval spans the step.
//
// The time tag (plan section 2). TOW is receiver time: the GPS instant of an
// epoch is TOW - RxClkBias (t_sys = t_rx - RxClkBias, Reference Guide), so
//   mcu_us = mcuAt(epochGpsS(WNc, TOW, dtrMs(bias, drift, dt, jumps)))
// with the bias of the last PVTGeodetic carried forward by its drift and by
// the clock jumps since it (a -1 ms jump takes 1 ms off the bias).
// ppsResidualUs checks a whole-second epoch's tag against its 1PPS edge.
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

// The receiver's clock jumps, followed from one MeasEpoch to the next.
// CumClkJumps is the jumps since the receiver started, in ms mod 256; a step
// of more than 2 ms, or time going backwards, is a receiver reset (or a
// different log), and the count starts over there. A driver that adapts only
// the epochs on its fuse grid shows the others to noteMeasEpochClock, so the
// count (and the PVT bias it carries forward with dtrMs) misses no jump; a
// header seen twice (noted, then adapted) counts once.
struct MosaicJumpState
{
    bool     have = false;        // an epoch with CumClkJumps seen since the last start-over
    uint8_t  last_cum = 0;
    int64_t  last_ms = 0;         // that epoch's receiver time, ms (WNc * 604800000 + TOW)
    int32_t  j_ms = 0;            // the clock jumps since the start-over, ms
    uint32_t jumps = 0;           // epochs that jumped
    uint32_t rebases = 0;         // start-overs after the first epoch
    uint32_t jumps_at_cells = 0;  // `jumps` when rawEpochFromMeasEpoch last gave cells
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
    int8_t    jump_ms = 0;
    bool      lock_reset = false;   // a jump at this epoch or since the last one with cells: every lock_ms is 0
    int32_t   j_ms = 0;          // MosaicJumpState::j_ms after this epoch
    uint8_t   n_gps = 0, n_gal = 0, n_bds = 0;
    uint8_t   geo = 0;           // BeiDou GEO B1I cells left out
    uint8_t   dupes = 0;         // a satellite's signal seen twice: the second left out
    uint8_t   over = 0;          // picks past RawEpoch::kMaxObs left out
};

// Follow CumClkJumps across one epoch's header (revision 1 on; a revision-0
// block says nothing, and nothing changes).
inline void noteClockJumps(const SbfMeasHeader& h, MosaicJumpState& js, MosaicEpochInfo& info)
{
    if (h.has_cum_clk_jumps)
    {
        const int64_t t = static_cast<int64_t>(h.wnc) * 604800000LL + h.tow_ms;
        if (!js.have)
        {
            js.j_ms = 0;
            info.rebased = true;
        }
        else
        {
            const int d = static_cast<int8_t>(static_cast<uint8_t>(h.cum_clk_jumps - js.last_cum));
            info.jumped = d != 0;
            info.jump_ms = static_cast<int8_t>(d);
            if (t < js.last_ms || d > 2 || d < -2)
            {
                js.j_ms = 0;
                js.rebases++;
                info.rebased = true;
            }
            else
            {
                js.j_ms += d;
            }
            if (info.jumped) js.jumps++;
        }
        js.have = true;
        js.last_cum = h.cum_clk_jumps;
        js.last_ms = t;
    }
    info.j_ms = js.j_ms;
}

// The clock-jump count of a MeasEpoch that is not adapted (off the fuse
// grid): its header only. False if the reader refuses the block.
inline bool noteMeasEpochClock(const uint8_t* b, size_t len, MosaicJumpState& js, MosaicEpochInfo& info)
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
    noteClockJumps(h, js, info);
    return true;
}

// One MeasEpoch block (CRC checked) into out: GPS L1 C/A, Galileo E1 and
// BeiDou B1I, one cell per satellite, in block order. False, with out.n = 0,
// for a block the reader refuses (wrong, short, bad layout, scrambled, no
// time) or one with no cell to give; info says which.
inline bool rawEpochFromMeasEpoch(const uint8_t* b, size_t len, gnss_raw::RawEpoch& out, MosaicJumpState& js,
                                  MosaicEpochInfo& info)
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
    noteClockJumps(h, js, info);
    // no carrier interval may span a jump: not at this epoch, nor at one
    // between it and the last epoch the builder had (decimated, or empty)
    info.lock_reset = js.jumps != js.jumps_at_cells;
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
        o.lock_ms = (s.lockValid() && !info.lock_reset) ? static_cast<uint32_t>(s.lock_s) * 1000U : 0U;
        o.has_phase = has_pr && s.carrier_valid && s.lockValid();
        o.halfcyc = s.half_cycle;
        o.phase_m = o.has_phase ? s.phaseRangeM(lambda) : 0.0;
    }
    if (out.n > 0)
        js.jumps_at_cells = js.jumps;
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
inline double dtrMs(double pvt_bias_ms, double drift_ppm, double dt_s, int32_t jumps_since_pvt)
{
    return pvt_bias_ms + drift_ppm * 1.0e-3 * dt_s + jumps_since_pvt;
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

}  // namespace mosaic

#endif  // MOSAIC_RAW_EPOCH_H
