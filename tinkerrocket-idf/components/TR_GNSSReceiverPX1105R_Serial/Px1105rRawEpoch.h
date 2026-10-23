// Px1105rRawEpoch.h — the PX1105R's 0xE5 epoch as the receiver-neutral
// observations TR_GNSSRaw builds a raw-GNSS filter's measurements from, and
// its time tag (header-only, pure)
//
// Mirrors MosaicRawEpoch.h: one cell per satellite, on the L1-band signal
// whose ephemeris and group delay the filter has (SkyTraqNav.h) -- GPS L1 C/A,
// Galileo E1 and BeiDou B1I, signal type 0 -- and nothing else: the L5 band's
// signals (type 4), GLONASS, SBAS, QZSS, NavIC, and BeiDou GEO (C01-C05,
// C59-C63: gnss/eph.h cannot place a D2 orbit).
//
// Units are the other receivers' (RawMeas.h):
//   - pr_m: the pseudorange, m, with the receiver's clock bias in it;
//   - rate_mps = -lambda * Doppler, the Doppler moved to the middle of its
//     whole-hertz step first (SkyTraqBinary.h dopplerHz). Like the mosaic's it
//     keeps the receiver's clock drift (-185 m/s on the 09-26 capture); the
//     filter's drift state takes it. It is the carrier's own change over the
//     epoch before, rounded: against the carrier phase it sits 25 ms (half
//     the 50 ms epoch) behind its tag on both rig flights tried (2026-10-21,
//     residual 0.047 m/s median = the 1 Hz step's), the profile's rr_lag_s;
//   - phase_m = lambda * the accumulated carrier, which follows the code
//     (code minus carrier drifts 0.03 m/s on the 20-minute static run, the
//     ionosphere), with has_phase only when the code, the carrier and a lock
//     are all there and no cycle slip is flagged; halfcyc the channel's "half
//     cycle not resolved" bit;
//   - lock_ms: the lock time indicator's lower bound (lockTimeMinMs), held to
//     the time since that satellite's last flagged slip and since the
//     receiver's last clock jump (Px1105rLockState), so no carrier interval
//     spans either;
//   - cn0_dbhz: the receiver's whole dB-Hz.
//
// The time tag. Without a 1PPS (the carrier's J3 pin 5 is the receiver's
// RXD2) the MCU clock is placed against GPS time by when each 0xE5 arrives
// (lc86::GnssTimeSync, the LC86G's envelope, with the profile's
// epoch_latency_s). The epoch's GPS instant is its time of week less the
// receiver's clock bias, from the 0xDF that follows each 0xE5: the last one,
// carried 50 ms by its drift (-0.6 us a second), as the mosaic carries its
// PVT's (MosaicRawEpoch.h dtrMs), and on through the 0xDF without a fix that
// follow it (Px1105rClock, biasAt). The PX1105R free-runs its clock; a whole
// millisecond step in the bias between two 0xDF is a clock jump.
#ifndef PX1105R_RAW_EPOCH_H
#define PX1105R_RAW_EPOCH_H

#include <cmath>
#include <cstdint>

#include "RawMeas.h"
#include "SkyTraqBinary.h"
#include "gnss/types.h"

namespace px1105r
{

static constexpr double kC = 299792458.0;            // m/s, as gnss/atmo.h's GNSS_C
static constexpr double kFreqB1iHz = 1561.098e6;
static constexpr int    kMaxPrn = 64;

// The filter's system for an 0xE5 cell, or -1 for one it does not take.
inline int filterSystem(const skytraq::RawCell& c)
{
    if (c.sig != skytraq::kSigL1)
        return -1;
    switch (c.gnss)
    {
    case skytraq::kGnssGps:
        return (c.svid >= 1 && c.svid <= 32) ? GNSS_SYS_GPS : -1;
    case skytraq::kGnssGal:
        return (c.svid >= 1 && c.svid <= 36) ? GNSS_SYS_GAL : -1;
    case skytraq::kGnssBds:
        return (c.svid >= 6 && c.svid <= 58) ? GNSS_SYS_BDS : -1;   // MEO / IGSO only
    default:
        return -1;
    }
}

inline double carrierHz(int sys)
{
    return sys == GNSS_SYS_BDS ? kFreqB1iHz : GNSS_FREQ_L1_HZ;
}

// Each satellite's last carrier break (a flagged slip), and the receiver's
// last clock jump, on the receiver's clock (week * 604800000 + TOW, ms), so a
// lock never claims to run across either.
struct Px1105rLockState
{
    int64_t slip_ms[GNSS_SYS_COUNT][kMaxPrn] = {};   // 0: none seen
    bool    have_cut = false;
    int64_t cut_ms = 0;                              // the last clock jump (or a restart of the receiver's time)
    uint32_t slips = 0;
};

// The receiver's clock, from its 0xDF: the bias to take off a time of week,
// carried to a later epoch by the drift, and its jumps. Only an 0xDF with a
// fix states a bias referenced to GPS time; the last one is carried by its
// drift across the 0xDF without a fix that follow it, for as long as they
// keep coming: each within kStateGapMs of the one before on the receiver's
// clock, time going forward, and one within kStateGapMs before the epoch.
// That spans a receiver reacquiring (its first 0xE5 come 0.4-1.1 s before its
// own fix: the RF rig's flights, 2026-10-21) and its speed limit's fix block,
// through which it keeps sending raw measurements (Traveler IV on the rig,
// 2026-10-23: no fix T+6.2..T+282 at 20 Hz of 0xDF, their bias and drift held
// at the last fix's; carried 276 s, the bias came within 0.6-7.2 us of the
// returning fix's on nine flights, under a centimetre of either end's motion).
// A break in the stream (a gap, or time going back: a receiver restarting,
// 15.5 s of silence on one of them) ends the carry until the next fix, and
// kBiasCarryMaxMs bounds it. A jump while there is no fix shows only at the
// next fix (none was seen: the bias holds still without one).
static constexpr int64_t kBiasCarryMaxMs = 600000;   // ~16 us at the rig's worst drift error, 26 ns a second
static constexpr int64_t kStateGapMs = 2000;         // 0xDF further apart than this: the clock line broken
static constexpr int64_t kFixGapMs = 2000;           // fixes further apart than this: the jump test cannot tell, a cut

struct Px1105rClock
{
    bool     have = false;           // an 0xDF with a time (week > 0) has come
    bool     fix = false;            //   and whether the last one had a fix
    int64_t  rx_ms = 0;              //   its epoch on the receiver's clock (its GPS time plus the last fix's bias)
    bool     have_fix = false;       // an 0xDF with a fix has come:
    double   bias_ms = 0.0;          //   its bias, receiver minus GPS, ms (0xDF clock bias / c)
    double   drift_ms_per_s = 0.0;   //   its rate, ms a second (0xDF clock drift / c)
    int64_t  fix_rx_ms = 0;          //   its epoch on the receiver's clock
    bool     carry = false;          //   the 0xDF unbroken since it: its bias may be carried
    uint32_t jumps = 0;              // whole-millisecond steps of the bias between fixes
    uint32_t restarts = 0;           // fixes further than kFixGapMs apart, or time going back (a reset or an outage)
    uint32_t breaks = 0;             // breaks in the 0xDF without a fix that ended a carry
};

inline int64_t rxMs(uint16_t week, uint32_t tow_ms)
{
    return static_cast<int64_t>(week) * 604800000LL + tow_ms;
}

// Note one 0xDF. Its epoch on the receiver's clock is its GPS time plus its
// bias (it states GPS time, the 0xE5's less the bias), rounded to the
// receiver's millisecond. One with no time yet (week 0: the receiver's
// all-zero state before its first fix) is not noted at all. Returns true when
// the bias stepped between two fixes (a clock jump), or the fixes were too
// far apart to tell: the caller cuts every lock there
// (Px1105rLockState::cut_ms).
inline bool noteRcvState(const skytraq::RcvState& s, Px1105rClock& clk)
{
    if (s.week == 0)
        return false;
    const bool fix = s.nav_state >= skytraq::kNavFix2d;
    const double bias_ms = fix ? s.clk_bias_m / kC * 1.0e3 : clk.bias_ms;
    const double drift = static_cast<double>(s.clk_drift_mps) / kC * 1.0e3;
    const int64_t rx = static_cast<int64_t>(std::llround((s.week * 604800.0 + s.tow_s) * 1.0e3 + bias_ms));
    const bool gap = clk.have && (rx - clk.rx_ms <= 0 || rx - clk.rx_ms > kStateGapMs);
    clk.have = true;
    clk.fix = fix;
    clk.rx_ms = rx;
    if (!fix)
    {
        if (gap && clk.carry)
        {
            clk.carry = false;
            clk.breaks++;
        }
        return false;
    }
    bool cut = false;
    if (clk.have_fix)
    {
        const int64_t dt_ms = rx - clk.fix_rx_ms;
        if (dt_ms <= 0 || dt_ms > kFixGapMs)
        {
            clk.restarts++;
            cut = true;
        }
        else
        {
            const double predicted = clk.bias_ms + clk.drift_ms_per_s * (dt_ms * 1.0e-3);
            if (std::fabs(bias_ms - predicted) > 0.25)   // a whole millisecond, give or take a quarter
            {
                clk.jumps++;
                cut = true;
            }
        }
    }
    clk.have_fix = true;
    clk.bias_ms = bias_ms;
    clk.drift_ms_per_s = drift;
    clk.fix_rx_ms = rx;
    clk.carry = true;
    return cut;
}

// The bias (ms) at an epoch rx_ms on the receiver's clock: the last fix's,
// carried by its drift; false without a fix within max_age_ms before it
// (before a fix the receiver's time of week is not GPS time), after a break
// in the 0xDF since it, or with no 0xDF within kStateGapMs before the epoch.
inline bool biasAt(const Px1105rClock& clk, int64_t rx_ms, double& dtr_ms, int64_t max_age_ms = kBiasCarryMaxMs)
{
    if (!clk.have_fix || !clk.carry)
        return false;
    const int64_t age = rx_ms - clk.fix_rx_ms;
    if (age < 0 || age > max_age_ms || rx_ms - clk.rx_ms > kStateGapMs)
        return false;
    dtr_ms = clk.bias_ms + clk.drift_ms_per_s * (age * 1.0e-3);
    return true;
}

// An epoch's GPS time, continuous seconds (week * 604800 + time of week),
// from its receiver time and the bias then (receiver minus GPS, ms). As
// mosaic::epochGpsS: the whole milliseconds as integers first.
inline double epochGpsS(uint16_t week, uint32_t tow_ms, double dtr_ms)
{
    return (static_cast<double>(rxMs(week, tow_ms)) - dtr_ms) * 1.0e-3;
}

// The cells of one 0xE5 as a RawEpoch (tow_s the receiver's own time of week:
// tow_s - pr / c is still each satellite's transmit time). Slip flags are
// noted into lk (they hold the satellite's lock from then on). Returns the
// cells kept; out.n = 0 for an epoch with none.
inline int rawEpochFromE5(const skytraq::RawE5& in, Px1105rLockState& lk, gnss_raw::RawEpoch& out)
{
    out.n = 0;
    out.tow_s = in.tow_ms * 1.0e-3;
    const int64_t now = rxMs(in.week, in.tow_ms);
    for (int i = 0; i < in.n && out.n < gnss_raw::RawEpoch::kMaxObs; ++i)
    {
        const skytraq::RawCell& c = in.cell[i];
        const int sys = filterSystem(c);
        if (sys < 0)
            continue;
        const bool has_pr = (c.ind & skytraq::kIndPr) && std::isfinite(c.pr_m) && c.pr_m > 0.0;
        const bool has_rate = (c.ind & skytraq::kIndDoppler) && std::isfinite(c.doppler_hz);
        const bool has_carrier = (c.ind & skytraq::kIndCarrier) && std::isfinite(c.carrier_cyc);
        if (!has_pr && !has_rate)
            continue;
        if (c.ind & skytraq::kIndSlip)
        {
            lk.slip_ms[sys][c.svid] = now;
            lk.slips++;
        }
        // the lock, held to the last slip and clock jump
        uint32_t lock = skytraq::lockTimeMinMs(c.lock_ind);
        int64_t since_cut = INT64_MAX;
        if (lk.slip_ms[sys][c.svid] != 0)
            since_cut = now - lk.slip_ms[sys][c.svid];
        if (lk.have_cut && now - lk.cut_ms < since_cut)
            since_cut = now - lk.cut_ms;
        if (since_cut < static_cast<int64_t>(lock))
            lock = since_cut > 0 ? static_cast<uint32_t>(since_cut) : 0U;

        const double lambda = kC / carrierHz(sys);
        gnss_raw::RawObs& o = out.obs[out.n++];
        o.sys = static_cast<uint8_t>(sys);
        o.prn = c.svid;
        o.cn0_dbhz = static_cast<float>(c.cn0);
        o.has_pr = has_pr;
        o.pr_m = has_pr ? c.pr_m : 0.0;
        o.has_rate = has_rate;
        o.rate_mps = has_rate ? -lambda * skytraq::dopplerHz(c.doppler_hz) : 0.0;
        o.lock_ms = lock;
        o.has_phase = has_pr && has_carrier && !(c.ind & skytraq::kIndSlip) && lock > 0;
        o.halfcyc = (c.ind & skytraq::kIndHalfCyc) != 0;
        o.phase_m = o.has_phase ? lambda * c.carrier_cyc : 0.0;
    }
    return out.n;
}

// Whether an epoch is on the fuse grid: its time of week a whole number of
// periods into the week (period 0: every epoch). The receiver's 50 ms epochs
// sit on whole milliseconds of its own clock, and the profile's periods
// (50, 100, 1000 ms) divide the week.
inline bool onFuseGrid(uint32_t tow_ms, uint32_t period_ms)
{
    return period_ms == 0 || tow_ms % period_ms == 0;
}

}  // namespace px1105r

#endif  // PX1105R_RAW_EPOCH_H
