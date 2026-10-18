// Host tests for the mosaic-G5's raw-epoch time tag across a lost and
// regained sky (TR_GNSSReceiverMosaic_Serial/MosaicTimeTag.h), through the
// driver's own pure pieces in the driver's order: MeasEpoch blocks
// (sbf_meas_build.h) and their arrivals into rawEpochFromMeasEpoch /
// noteMeasEpochClock, the 1PPS edges into PpsPairer (none while the link is
// backed up) and the PPS line (GnssTimeSync, exact_step_s as the mosaic
// image sets it, stepping later only onto edges on the chain), and each
// PVTGeodetic's RxClkBias.
//
// A simulated bench: an MCU crystal 16 ppm fast (the V9's); a receiver clock
// 0.27 ppm fast that steps whole milliseconds with a fix when its bias passes
// 0.5 ms (CumClkJumps); MeasEpoch and PVTGeodetic at 20 Hz, the fused grid at
// 100 ms; a 1PPS on the receiver's GPS time (free-running through a lost sky,
// as MaxSyncAge 60 s keeps it). Every epoch handed over timed is checked
// against its true MCU instant: the instant the receiver's clock read its
// TOW, on the MCU clock.
//
// The 2026-10-17 bench (image 2f724a00, the RF rig's pad sky): a transmitter
// underrun of 6.854 ms delayed the whole sky; the receiver lost it, came back
// 15 s later with its bias 6.85 ms larger, stepped its clock 7 ms and moved
// its 1PPS. With the driver's rule then, every whole second read -6855 us.
// With MosaicTimeTag the tag is right or the epoch untimed, and the tags come
// back within seconds. The review's cases after it (2026-10-17): the 1PPS
// late to the step by 1 to 10 s (the PPS check reads 0 there and the chain
// keeps the tags untimed), runs of edges stamped late the same each second
// (the line never steps later onto them), a sky back early by whole quarters
// of a second (a jump the count shows mod 256, placed by the arrivals), and a
// link backed up a second (no edge paired: a line labelled a second off would
// pass every check).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <set>
#include <vector>

#include "GnssTimeSync.h"
#include "MosaicRawEpoch.h"
#include "MosaicTimeTag.h"
#include "PpsPairer.h"
#include "SbfParser.h"
#include "SimReplay.h"
#include "sbf_meas_build.h"

using namespace mosaic;

namespace
{

using Why = MosaicTimeTag::Why;
using Break = MosaicTimeTag::Break;

constexpr int64_t kWeekMs = 604800000LL;

sbftest::MeasT1 gpsSat(uint8_t svid)
{
    sbftest::MeasT1 t;
    t.rx_channel = svid;
    t.svid = svid;
    t.sig(0);
    t.codeMm(22000000000LL + svid * 1000);
    t.doppler = -15000000 + svid;
    t.carrier(-2000 - svid);
    t.cn0 = 128;
    t.lock = 600;
    return t;
}

// The bench. Physical time t is seconds from the start, which is GPS time
// gps0_ms + t on a sky that runs true until it is delayed.
struct Bench
{
    // the scenario
    int64_t gps0_ms = int64_t(2440) * kWeekMs + 302400000;   // whole seconds
    double  mcu_ppm = 16.0;
    int64_t mcu0_us = 50000000;
    double  rx_ppm = 0.27;          // the receiver's clock against the sky
    double  rx_off0_ms = 0.21;      // its bias at t = 0
    double  sky_delay_ms = 0.0;     // the sky's delay from sky_delay_at_s on (an underrun)
    double  sky_delay_at_s = 1e9;
    double  lost_from_s = 1e9, lost_to_s = -1.0;   // no satellite tracked
    double  fix_after_s = 2.0;      // tracked again: the first fix this much later
    int     partial_sats = 0;       // tracked in the loss's last part without a fix (0: none)
    bool    pps_lags_a_second = false;   // the 1PPS moves to a new sky after the fix ...
    double  pps_lag_s = 1.0;            //   ... this long after it
    int64_t late_from_k = -1, late_to_k = -2;   // a run of 1PPS edges stamped late (the same hold-off each second)
    int64_t late_run_us = 0;            //   by this, plus up to 60 us
    double  backlog_from_s = 1e9, backlog_to_s = -1.0;   // the link backed up: every block this much later
    double  backlog_s = 0.0;
    bool    jump_with_first_fix = false; // the first fix's PVT states its bias after the step
    double  reset_at_s = 1e9;       // the receiver resets: no blocks for reset_gap_s, then its time anew
    double  reset_gap_s = 12.0;
    double  reset_bias_ms = 12.3;   // its bias after the reset, before the first fix steps it
    int64_t late_edge_k = -1;       // a 1PPS edge stamped late (a flash write)
    int64_t late_edge_us = 3000;
    uint8_t cum0 = 254;

    // the receiver's state
    double  jumps_ms = 0.0;         // its clock's steps so far
    uint8_t cum = 254;
    double  pending_jump_ms = 0.0;
    bool    fine = true;
    double  last_fix_s = -1e9;
    double  pps_delay_known_ms = 0.0;   // the sky delay its 1PPS is on
    double  reset_done_s = -1.0;
    bool    have_fixed_since_loss = false;

    double mcuUs(double t) const { return mcu0_us + t * 1.0e6 * (1.0 + mcu_ppm * 1e-6); }
    double skyDelayMs(double t) const { return t >= sky_delay_at_s ? sky_delay_ms : 0.0; }
    // the receiver's bias against the sky now, ms
    double biasMs(double t) const { return rx_off0_ms + rx_ppm * 1e-3 * t + jumps_ms + skyDelayMs(t); }
    // the physical time the receiver's clock reads rx_ms
    double tOf(int64_t rx_ms) const
    {
        return (static_cast<double>(rx_ms - gps0_ms) - rx_off0_ms - jumps_ms) * 1e-3 / (1.0 + rx_ppm * 1e-6);
    }
    bool lost(double t) const { return t >= lost_from_s && t < lost_to_s; }
    bool resetting(double t) const { return t >= reset_at_s && t < reset_at_s + reset_gap_s; }
};

// One epoch handed over, as the GNSS task gets it.
struct Out
{
    int64_t rx_ms = 0;
    int64_t mcu_us = 0;
    double  err_us = 0.0;     // against the true instant (timed only)
    double  t = 0.0;
    double  dtr_ms = 0.0;     // the bias the tag took off (TimedRawEpoch::tag_dtr_ms; timed only)
};

// The driver's pieces, in the driver's order (TR_GNSSReceiverMosaic_Serial.cpp
// rawMeasEpoch / pairPps / rawPvt), new rule (MosaicTimeTag) or the one it
// replaced (a PVT carried at most 1 s, dtr 0 without one, the line an
// envelope, no hold): legacy reproduces the 10/17 bench.
struct Driver
{
    bool legacy = false;
    double last_dtr_ms = 0.0;   // the last epoch's tag_dtr_ms: its bias if handed over timed, else 0
    lc86::GnssTimeSync sync;
    PpsPairer pairer;
    MosaicJumpState js;
    MosaicTimeTag tag;
    uint32_t period_ms = 100;
    // legacy state
    bool pvt_have = false;
    double pvt_bias = 0.0, pvt_drift = 0.0;
    int64_t pvt_rx = 0;
    int32_t pvt_j = 0;
    uint32_t pvt_rebases = 0;
    bool pair_have = false;
    int64_t pair_s = 0, pair_edge = 0;
    bool tag_have = false;
    int64_t tag_rx = 0, tag_mcu = 0;
    PpsTagStats legacy_pps;
    std::vector<int32_t> legacy_residuals;   // whole seconds judged, with their rx_ms
    std::vector<int64_t> legacy_residual_rx;

    explicit Driver(bool old) : legacy(old)
    {
        if (!legacy)
        {
            lc86::GnssTimeSync::Config c;
            c.exact_step_s = 100e-6;   // TR_GNSSReceiverMosaicSerial: kPpsOnLineUs
            sync.setConfig(c);
        }
    }

    void edge(int64_t mcu_us) { pairer.addEdge(mcu_us); }

    void receiverTime(bool fine) { tag.noteFineTime(fine); legacy_fine = fine; }
    bool legacy_fine = false;

    // a MeasEpoch; returns whether it was handed over, and its mcu_us
    bool measEpoch(const std::vector<uint8_t>& blk, uint32_t tow, int64_t now_us, int64_t& mcu_out)
    {
        mcu_out = 0;
        MosaicEpochInfo info;
        // the new rule's count places a jump past 256 ms by the arrival (MosaicJumpState)
        const int64_t arrival = legacy ? kMosaicNoArrival : now_us;
        if (!onFuseGrid(tow, period_ms))
        {
            (void)noteMeasEpochClock(blk.data(), blk.size(), js, info, arrival);
            return false;
        }
        gnss_raw::RawEpoch ep;
        const bool cells = rawEpochFromMeasEpoch(blk.data(), blk.size(), ep, js, info, arrival);
        if (info.status != SbfStatus::OK)
            return false;
        const int64_t rx_ms = static_cast<int64_t>(info.wnc) * kWeekMs + info.tow_ms;
        if (legacy)
        {
            if (!cells)
                return false;
            if (sync.valid() && legacy_fine && pair_have && now_us - pair_edge < 120000000)
            {
                double dtr = 0.0;
                if (pvt_have && rx_ms >= pvt_rx && rx_ms - pvt_rx <= 1000 && js.rebases == pvt_rebases)
                    dtr = dtrMs(pvt_bias, pvt_drift, (rx_ms - pvt_rx) * 1e-3, info.j_ms - pvt_j);
                int64_t mcu = 0;
                if (sync.mcuAt(epochGpsS(info.wnc, info.tow_ms, dtr), mcu))
                    mcu_out = mcu;
            }
            if (mcu_out != 0 && info.tow_ms % 1000 == 0)
            {
                tag_have = true;
                tag_rx = rx_ms;
                tag_mcu = mcu_out;
            }
            return true;
        }
        double dtr = 0.0;
        const bool carried = tag.carry(info, js, dtr);
        int64_t mcu = 0;
        const bool placed = carried && sync.mcuAt(epochGpsS(info.wnc, info.tow_ms, dtr), mcu);
        const Why why = tag.judge(sync.valid(), carried, now_us);
        const bool timed = placed && why == Why::TIMED;
        last_dtr_ms = 0.0;
        if (cells)
        {
            tag.countEpoch(timed ? Why::TIMED : why);
            if (timed)
            {
                mcu_out = mcu;
                last_dtr_ms = dtr;
            }
        }
        if (info.tow_ms % 1000 == 0 && placed)
            tag.noteWholeSecond(rx_ms, mcu, cells && timed);
        return cells;
    }

    // a PVTGeodetic arriving at arrival_us
    void pvt(const PvtGeodetic& p, int64_t arrival_us)
    {
        if (p.tow_ms % 1000 == 0 && (legacy || !pair_guard || !(js.arr_have && js.late_us > 300000)))
        {
            const double gs = p.wnc * 604800.0 + p.tow_ms / 1000;
            int64_t e = 0;
            if (pairer.pair(gs, arrival_us, e))
            {
                // the line steps later onto an edge only where the PVTs moved by as much
                sync.addSample(gs, e, legacy || tag.edgeOnChain(std::llround(gs), e));
                pair_have = true;
                pair_s = std::llround(gs);
                pair_edge = e;
                int64_t at = 0;
                const bool on = sync.mcuAt(gs, at);
                tag.notePair(std::llround(gs), e, on, at, sync.driftPpm() * 1e-6, sync.restarts());
            }
        }
        const int64_t rx_ms = static_cast<int64_t>(p.wnc) * kWeekMs + p.tow_ms;
        if (legacy)
        {
            if (!p.has_solution)
                return;
            if (pvtAnchorsJumps(js, p.wnc, p.tow_ms))
            {
                pvt_have = true;
                pvt_bias = p.rx_clk_bias_ms;
                pvt_drift = p.rx_clk_drift_ppm;
                pvt_rx = rx_ms;
                pvt_j = js.j_ms;
                pvt_rebases = js.rebases;
            }
            if (tag_have && tag_rx == rx_ms)
            {
                tag_have = false;
                if (pair_have && pair_s * 1000 == rx_ms)
                {
                    const int32_t r = notePpsTag(legacy_pps, pair_s, tag_mcu, pair_edge, 1000016,
                                                 sync.driftFitted(), p.rx_clk_bias_ms, 200);
                    if (sync.driftFitted())
                    {
                        legacy_residuals.push_back(r);
                        legacy_residual_rx.push_back(rx_ms);
                    }
                }
            }
            return;
        }
        int64_t second_us = 1000000;
        if (p.tow_ms % 1000 == 0)
        {
            const double gs = p.wnc * 604800.0 + p.tow_ms / 1000;
            int64_t a = 0, b = 0;
            if (sync.mcuAt(gs, a) && sync.mcuAt(gs - 1.0, b))
                second_us = a - b;
        }
        tag.notePvt(p, js, second_us, sync.driftFitted());
        if (trace && p.tow_ms % 1000 == 0)
        {
            const auto& c = tag.lastCheck();
            std::printf("  [trace] tow %u fix %d bias %+.4f | held %d (%s) passes %u edge_on %d (%+lld us) | chain "
                        "%+.3f ms edge %+.3f ms (%s) | check %+d (%s) | line steps %u restarts %u | jumps resolved %u "
                        "resets %u\n", p.tow_ms / 1000, p.has_solution, p.rx_clk_bias_ms, tag.held(),
                        MosaicTimeTag::breakName(tag.holdBreak()), tag.passes(), tag.edgeOnLine(),
                        (long long)tag.edgeOffLineUs(), tag.chainMs(), tag.edgeRefUs() * 1e-3,
                        !tag.refHave() ? "no ref" : (tag.onChain() ? "on" : "OFF"),
                        c.rx_ms == rx_ms ? c.residual_us : 0, c.rx_ms == rx_ms ? (c.published ? "out" : "held") : "-",
                        sync.steps(), sync.restarts(), js.resolved, js.resets);
        }
    }
    bool trace = false;
    bool pair_guard = true;   // the driver's: no pairing while the link is backed up (kPairMaxLateUs)
};

struct BenchRun
{
    std::vector<Out> timed;      // handed over timed
    int handed = 0;              // handed over at all (with cells)
    double worst_us = 0.0;
    double worst_t = 0.0;
    int64_t first_timed_after_rx = -1;   // rx_ms
};

// Run the bench for `seconds` through the driver; on_handed sees every epoch
// handed over, timed or not (mcu_us 0), as the GNSS task drains it.
BenchRun run(Bench b, Driver& d, double seconds, double after_s = 1e9, double* first_timed_after = nullptr,
             const std::function<void(const Out&)>& on_handed = {})
{
    BenchRun out;
    b.cum = b.cum0;
    // receiver epochs every 50 ms of its own time from the first whole second
    int64_t rx_ms = b.gps0_ms + 1000;
    int64_t next_pps = b.gps0_ms / 1000 + 1;   // GPS second of the next 1PPS edge
    bool resumed = false;
    double first_after = -1.0;
    int64_t jitter = 0;
    double t_prev = -1e9;
    double last_edge_t = -1e9;   // a pulse moved back is never sent into the past: the next one comes on time
    while (true)
    {
        double t = b.tOf(rx_ms);
        if (t > seconds)
            break;
        // a receiver reset: nothing for the gap, then its clock anew
        if (b.resetting(t))
        {
            if (!resumed)
            {
                // its time comes back within 20 ms of the sky, its jump count at 0, not FINETIME
                b.jumps_ms = b.reset_bias_ms - (b.rx_off0_ms + b.rx_ppm * 1e-3 * (b.reset_at_s + b.reset_gap_s) +
                                                b.skyDelayMs(b.reset_at_s + b.reset_gap_s));
                b.cum = 0;
                b.fine = false;
                b.reset_done_s = b.reset_at_s + b.reset_gap_s;
                b.last_fix_s = -1e9;
                resumed = true;
            }
            // the first label after the gap at the receiver's new time
            const double t_end = b.reset_at_s + b.reset_gap_s;
            const double rx_end_ms = b.gps0_ms + t_end * 1e3 + b.biasMs(t_end) - b.skyDelayMs(t_end);
            rx_ms = static_cast<int64_t>(std::ceil(rx_end_ms / 50.0)) * 50;
            continue;
        }
        // a step decided at the last fix applies from this epoch; a step
        // forward skips the epochs its clock jumped past (they stay on its
        // own 50 ms grid)
        if (b.pending_jump_ms != 0.0)
        {
            b.jumps_ms += b.pending_jump_ms;
            b.cum = static_cast<uint8_t>(b.cum + static_cast<int>(b.pending_jump_ms));
            b.pending_jump_ms = 0.0;
            t = b.tOf(rx_ms);
            while (t <= t_prev)
            {
                rx_ms += 50;
                t = b.tOf(rx_ms);
            }
        }
        const bool lost = b.lost(t);
        const bool after_loss = t >= b.lost_to_s && b.lost_to_s > 0;
        const bool after_reset = b.reset_done_s > 0 && t >= b.reset_done_s;
        const double since = after_loss ? t - b.lost_to_s : (after_reset ? t - b.reset_done_s : 1e9);
        const bool fix = !lost && !(since < b.fix_after_s);
        const bool partial = !lost && !fix;
        if (fix && b.jump_with_first_fix && std::fabs(b.biasMs(t)) > 0.5)
        {
            // the step taken at this very epoch: its MeasEpoch shows the new
            // count and its PVT the bias after it
            const double j = -std::round(b.biasMs(t));
            b.jumps_ms += j;
            b.cum = static_cast<uint8_t>(b.cum + static_cast<int>(j));
            t = b.tOf(rx_ms);
            while (t <= t_prev)
            {
                rx_ms += 50;
                t = b.tOf(rx_ms);
            }
        }
        t_prev = t;
        if (fix)
        {
            b.fine = true;   // FINETIME from the first fix (after a reset: cleared until then)
            b.last_fix_s = t;
        }
        // the receiver's 1PPS: on its GPS time, as it knew the sky at its last fix
        const double pps_known_ms =
            (b.pps_lags_a_second && fix && t - (after_loss ? b.lost_to_s + b.fix_after_s : 0.0) < b.pps_lag_s + 0.5)
                ? b.pps_delay_known_ms
                : (fix ? b.skyDelayMs(t) : b.pps_delay_known_ms);
        b.pps_delay_known_ms = pps_known_ms;
        const bool pps_on = b.last_fix_s > -1e8 && t - b.last_fix_s < 60.0 && !(after_reset && b.last_fix_s < b.reset_done_s);
        const double backlog = t >= b.backlog_from_s && t < b.backlog_to_s ? b.backlog_s : 0.0;
        const int64_t meas_arrival = static_cast<int64_t>(std::llround(b.mcuUs(t + 0.013 + backlog)));
        while (true)
        {
            const double t_edge = (next_pps * 1000 - b.gps0_ms) * 1e-3 + b.pps_delay_known_ms * 1e-3;
            if (t_edge > t + 0.013 + backlog)   // the edges up to when the blocks are taken
                break;
            if (pps_on && !b.resetting(t_edge) && t_edge > last_edge_t + 0.5)
            {
                last_edge_t = t_edge;
                int64_t e = static_cast<int64_t>(std::llround(b.mcuUs(t_edge))) + (jitter++ * 7919) % 4;
                if (next_pps == b.late_edge_k + b.gps0_ms / 1000)
                    e += b.late_edge_us;
                const int64_t k = next_pps - b.gps0_ms / 1000;
                if (k >= b.late_from_k && k <= b.late_to_k)
                    e += b.late_run_us + (k * 37) % 61;
                d.edge(e);
            }
            next_pps++;
        }
        // ReceiverTime once a second
        if (rx_ms % 1000 == 0)
            d.receiverTime(b.fine);

        // MeasEpoch
        sbftest::MeasEpochSpec s;
        s.wnc = static_cast<uint16_t>(rx_ms / kWeekMs);
        s.tow = static_cast<uint32_t>(rx_ms % kWeekMs);
        s.cum = b.cum;
        if (fix)
            s.t1 = {gpsSat(3), gpsSat(7), gpsSat(11), gpsSat(19), gpsSat(24)};
        else if (partial && b.partial_sats > 0)
            for (int k = 0; k < b.partial_sats; ++k)
                s.t1.push_back(gpsSat(static_cast<uint8_t>(3 + 4 * k)));
        const auto blk = sbftest::measEpoch(s);
        int64_t mcu = 0;
        const bool handed = d.measEpoch(blk, s.tow, meas_arrival, mcu);
        if (handed)
        {
            out.handed++;
            if (on_handed)
            {
                Out h;
                h.rx_ms = rx_ms;
                h.mcu_us = mcu;
                h.t = t;
                h.dtr_ms = d.last_dtr_ms;
                on_handed(h);
            }
            if (mcu != 0)
            {
                Out o;
                o.rx_ms = rx_ms;
                o.mcu_us = mcu;
                o.t = t;
                o.dtr_ms = d.last_dtr_ms;
                o.err_us = static_cast<double>(mcu) - b.mcuUs(t);
                if (std::fabs(o.err_us) > out.worst_us)
                {
                    out.worst_us = std::fabs(o.err_us);
                    out.worst_t = t;
                }
                out.timed.push_back(o);
                if (t >= after_s && first_after < 0)
                    first_after = t;
            }
        }

        // PVTGeodetic: the bias as the receiver states it at this epoch
        PvtGeodetic p;
        p.wnc = s.wnc;
        p.tow_ms = s.tow;
        p.has_solution = fix;
        const double bias = b.biasMs(t);
        if (fix && std::fabs(bias) > 0.5)
            b.pending_jump_ms = -std::round(bias);   // the clock steps at the next epoch
        p.rx_clk_bias_ms = fix ? bias : -2e10;
        p.rx_clk_drift_ppm = static_cast<float>(b.rx_ppm);
        d.pvt(p, static_cast<int64_t>(std::llround(b.mcuUs(t + 0.016 + backlog))));
        rx_ms += 50;
    }
    if (first_timed_after != nullptr)
        *first_timed_after = first_after;
    return out;
}

// The 10/17 sky: 160 s of sky, a 6.854 ms underrun at 160 s, 15 s lost, the
// first fix 2 s after it is tracked again, 160 s more.
Bench underrun()
{
    Bench b;
    b.sky_delay_ms = 6.854;
    b.sky_delay_at_s = 160.0;
    b.lost_from_s = 160.0;
    b.lost_to_s = 175.0;
    b.fix_after_s = 2.0;
    return b;
}

}  // namespace

// The bug as the bench saw it: the driver's rule before (the bias of a PVT at
// most 1 s old, dtr 0 without one, the line an envelope) reads about -6855 us
// at every whole second after the receiver re-acquires.
TEST(MosaicTimeTag, ReproducesThe1017BenchWithTheRuleBefore)
{
    Driver d(true);
    const BenchRun r = run(underrun(), d, 330.0);
    int before = 0, before_bad = 0, after = 0, after_ms = 0;
    for (size_t i = 0; i < d.legacy_residuals.size(); ++i)
    {
        const double t = (d.legacy_residual_rx[i] - Bench{}.gps0_ms) * 1e-3;
        const int32_t res = d.legacy_residuals[i];
        if (t < 160.0)
        {
            if (t < 40.0)
                continue;   // the second the blocks first fit the drift: a tag made on the flat line
            before++;
            if (std::abs(res) > 200)
            {
                before_bad++;
                std::printf("[before] over the bar before the underrun: %d us at %.0f s\n", res, t);
            }
        }
        else
        {
            after++;
            if (res < -6500 && res > -7200) after_ms++;
        }
    }
    int wrong = 0;
    for (const Out& o : r.timed)
        if (std::fabs(o.err_us) > 1000.0) wrong++;
    std::printf("[before] whole seconds before the underrun %d (%d over 200 us), after %d (%d at -6.5..-7.2 ms); "
                "epochs timed wrong by over 1 ms: %d of %zu; worst %.0f us\n",
                before, before_bad, after, after_ms, wrong, r.timed.size(), r.worst_us);
    EXPECT_GE(before, 100);   // the bench: |residual| max 39 us before the dropout
    EXPECT_EQ(before_bad, 0);
    EXPECT_GE(after_ms, 20);          // the bench: 24 of 24 judged
    EXPECT_GT(wrong, 200);
    EXPECT_NEAR(r.worst_us, 6854.0, 600.0);
}

// The same bench with MosaicTimeTag and the line that follows its edges:
// every epoch handed over timed is within 20 us of its true instant (the
// edges' own few us of interrupt latency; the decisions' bar is 100 us), and
// the tags are back within 12 s of the first fix.
TEST(MosaicTimeTag, TheUnderrunLeavesEveryTagRightOrUntimed)
{
    Driver d(false);
    double back = -1.0;
    const BenchRun r = run(underrun(), d, 330.0, 177.0, &back);
    const auto& st = d.tag.stats();
    int timed_before = 0, timed_after = 0;
    for (const Out& o : r.timed)
        (o.t < 160.0 ? timed_before : timed_after)++;
    std::printf("[after] timed %zu of %d handed over (worst %.1f us); before the underrun %d, after it %d, back "
                "%.1f s after the first fix; untimed: no line %u, FINETIME %u, no edge %u, no bias %u, edge off %u, "
                "held %u; breaks: first %u rebase %u expired %u bias step %u (%.3f ms) check %u; released %u; "
                "line steps %u (%.3f ms); PPS judged %u max %d us over %u, held %u max %d us\n",
                r.timed.size(), r.handed, r.worst_us, timed_before, timed_after, back - 177.0,
                st.why[(int)Why::NO_LINE], st.why[(int)Why::NOT_FINETIME], st.why[(int)Why::PPS_STALE],
                st.why[(int)Why::NO_BIAS], st.why[(int)Why::EDGE_OFF], st.why[(int)Why::HELD],
                st.breaks[(int)Break::FIRST], st.breaks[(int)Break::REBASE], st.breaks[(int)Break::EXPIRED],
                st.breaks[(int)Break::BIAS_STEP], st.last_bias_step_ms, st.breaks[(int)Break::CHECK_FAILED],
                st.confirmed, d.sync.steps(), d.sync.lastStepMs(), st.pps.checks, st.pps.max_us, st.pps.over,
                st.pps.held, st.pps.held_max_us);
    EXPECT_LT(r.worst_us, 20.0);
    EXPECT_EQ(st.pps.over, 0u);
    EXPECT_LE(st.pps.max_us, 20);
    EXPECT_EQ(st.breaks[(int)Break::BIAS_STEP], 1u);
    EXPECT_NEAR(st.last_bias_step_ms, 6.854, 0.01);
    EXPECT_EQ(d.sync.steps(), 1u);
    EXPECT_NEAR(d.sync.lastStepMs(), 6.854, 0.02);
    EXPECT_GT(back, 0.0);
    EXPECT_LT(back - 177.0, 12.0);
    EXPECT_GE(timed_after, 1400);     // of ~1530 epochs after the first fix
    EXPECT_GE(timed_before, 1000);
    EXPECT_FALSE(d.tag.held());
}

// Variants of how the receiver might come back: its first fix's PVT already
// after the step (a restart of the jump count, no bias step to see), its
// 1PPS a second behind its fix, and a few satellites tracked before the fix.
TEST(MosaicTimeTag, EveryWayBackFromTheUnderrunStaysRightOrUntimed)
{
    struct V
    {
        const char* name;
        bool jump_with_fix, pps_lag;
        int partial;
    };
    for (const V v : {V{"step with the first fix", true, false, 0}, V{"1PPS a second late", false, true, 0},
                      V{"both", true, true, 0}, V{"3 satellites before the fix", false, false, 3},
                      V{"all", true, true, 3}})
    {
        Bench b = underrun();
        b.jump_with_first_fix = v.jump_with_fix;
        b.pps_lags_a_second = v.pps_lag;
        b.partial_sats = v.partial;
        Driver d(false);
        d.trace = std::getenv("TAG_TRACE") != nullptr;
        if (d.trace) std::printf("[%s]\n", v.name);
        double back = -1.0;
        const BenchRun r = run(b, d, 330.0, 177.0, &back);
        int timed_after = 0;
        for (const Out& o : r.timed)
            if (o.t >= 160.0) timed_after++;
        const auto& st = d.tag.stats();
        std::printf("[%s] timed %zu, worst %.1f us, back %.1f s after the first fix, %d after the underrun, PPS "
                    "judged max %d us; breaks rebase %u bias step %u check %u; line steps %u\n", v.name,
                    r.timed.size(), r.worst_us, back - 177.0, timed_after, st.pps.max_us,
                    st.breaks[(int)Break::REBASE], st.breaks[(int)Break::BIAS_STEP],
                    st.breaks[(int)Break::CHECK_FAILED], d.sync.steps());
        EXPECT_LT(r.worst_us, 20.0) << v.name;
        EXPECT_EQ(d.tag.stats().pps.over, 0u) << v.name;
        EXPECT_GT(back, 0.0) << v.name;
        EXPECT_LT(back - 177.0, 15.0) << v.name;
        EXPECT_GE(timed_after, 1350) << v.name;
    }
}

// The real sky lost and found again (a boost dropout): GPS time does not
// step, so neither do the 1PPS nor the receiver's time against its clock. The
// bias is carried across the loss, the epochs with a few satellites before
// the fix are timed and right, and nothing is held when the fix comes back,
// even with the clock stepping 1 ms at it.
TEST(MosaicTimeTag, ARealSkyLostAndFoundKeepsTheTag)
{
    Bench b;
    b.rx_off0_ms = 0.47;          // the bias passes 0.5 ms during the loss: a -1 ms step at the first fix
    b.rx_ppm = 1.2;
    b.lost_from_s = 120.0;
    b.lost_to_s = 150.0;
    b.fix_after_s = 3.0;
    b.partial_sats = 2;
    Driver d(false);
    double back = -1.0;
    const BenchRun r = run(b, d, 220.0, 150.0, &back);
    const auto& st = d.tag.stats();
    int partial_timed = 0;
    for (const Out& o : r.timed)
        if (o.t >= 150.0 && o.t < 153.0) partial_timed++;
    std::printf("[real sky] timed %zu, worst %.1f us, %d of the 30 epochs with 2 satellites timed, back %.2f s after "
                "the sky; breaks after the first: rebase %u bias step %u expired %u check %u; jumps %u\n",
                r.timed.size(), r.worst_us, partial_timed, back - 150.0, st.breaks[(int)Break::REBASE],
                st.breaks[(int)Break::BIAS_STEP], st.breaks[(int)Break::EXPIRED], st.breaks[(int)Break::CHECK_FAILED],
                d.js.jumps);
    EXPECT_LT(r.worst_us, 20.0);
    EXPECT_GE(partial_timed, 29);
    EXPECT_LT(back - 150.0, 0.2);
    EXPECT_EQ(st.breaks[(int)Break::REBASE] + st.breaks[(int)Break::BIAS_STEP] + st.breaks[(int)Break::EXPIRED], 0u);
    EXPECT_GE(d.js.jumps, 1u);
}

// A receiver reset: 12 s of nothing, its jump count back at 0 from 254 (a
// "+2 ms jump" to a count that only follows CumClkJumps), its time 12.3 ms
// off until its first fix steps it, its 1PPS off until then, and FINETIME
// cleared. The gap restarts the count; no epoch is timed until the new
// chain is confirmed, and every one timed is right.
TEST(MosaicTimeTag, AReceiverResetIsNeverTimedWrong)
{
    Bench b;
    b.reset_at_s = 140.0;
    b.reset_gap_s = 12.0;
    b.fix_after_s = 2.0;
    b.partial_sats = 4;
    b.cum0 = 254;
    Driver d(false);
    double back = -1.0;
    const BenchRun r = run(b, d, 240.0, 152.0, &back);
    const auto& st = d.tag.stats();
    std::printf("[reset] timed %zu, worst %.1f us, back %.1f s after the reset ended; rebases %u, breaks rebase %u "
                "bias step %u; untimed not FINETIME %u, no bias %u, held %u\n",
                r.timed.size(), r.worst_us, back - 152.0, d.js.rebases, st.breaks[(int)Break::REBASE],
                st.breaks[(int)Break::BIAS_STEP], st.why[(int)Why::NOT_FINETIME], st.why[(int)Why::NO_BIAS],
                st.why[(int)Why::HELD]);
    EXPECT_LT(r.worst_us, 20.0);
    EXPECT_GE(d.js.rebases, 1u);
    EXPECT_GT(back, 0.0);
    EXPECT_LT(back - 152.0, 12.0);
}

// A 1PPS edge stamped 3 ms late (a flash write held the interrupt off): the
// epochs of that second wait for the next edge, nothing is held, the line
// does not move, and no tag is wrong.
TEST(MosaicTimeTag, ALateEdgeCostsASecondAndNoTag)
{
    Bench b;
    b.late_edge_k = 150;
    Driver d(false);
    const BenchRun r = run(b, d, 200.0);
    const auto& st = d.tag.stats();
    int timed_150_152 = 0;
    for (const Out& o : r.timed)
        if (o.t >= 150.0 && o.t < 152.0) timed_150_152++;
    std::printf("[late edge] timed %zu, worst %.1f us; in 150-152 s %d of 20; edges off %u; holds after the first %u; "
                "PPS late %u judged max %d\n", r.timed.size(), r.worst_us, timed_150_152, st.edges_off,
                st.breaks[(int)Break::CHECK_FAILED] + st.breaks[(int)Break::REBASE] + st.breaks[(int)Break::BIAS_STEP],
                st.pps.late, st.pps.max_us);
    EXPECT_LT(r.worst_us, 20.0);
    EXPECT_EQ(st.edges_off, 1u);
    EXPECT_EQ(d.sync.steps(), 0u);
    EXPECT_EQ(st.breaks[(int)Break::CHECK_FAILED], 0u);
    EXPECT_EQ(st.pps.late, 1u);
    EXPECT_GE(timed_150_152, 10);
}

// From the boot: the line is valid after 10 s of edges, before its blocks
// have fitted the crystal's drift (16 ppm: a flat line would part from the
// edges by 16 us a second, 0.15 ms by then). On the provisional drift of its
// edges its tags are right from the first, and the PPS check holds them back
// until two whole seconds confirm the first chain.
TEST(MosaicTimeTag, TheBootsFlatLineIsNeverHandedOverOffTheEdges)
{
    Bench b;
    Driver d(false);
    const BenchRun r = run(b, d, 90.0);
    const auto& st = d.tag.stats();
    double first = 1e9, last_gap_start = -1, last_gap_end = -1, prev = -1;
    for (const Out& o : r.timed)
    {
        first = std::min(first, o.t);
        if (prev > 0 && o.t - prev > 1.0)
        {
            last_gap_start = prev;
            last_gap_end = o.t;
        }
        prev = o.t;
    }
    std::printf("[boot] timed %zu of %d, worst %.1f us, first at %.1f s, a gap %.1f-%.1f s; untimed: no line %u, "
                "edge off %u, held %u; PPS settling %u (max %d), judged %u (max %d)\n",
                r.timed.size(), r.handed, r.worst_us, first, last_gap_start, last_gap_end,
                st.why[(int)Why::NO_LINE], st.why[(int)Why::EDGE_OFF], st.why[(int)Why::HELD], st.pps.settling,
                st.pps.settling_max_us, st.pps.checks, st.pps.max_us);
    EXPECT_LT(r.worst_us, 20.0);
    EXPECT_LE(st.pps.settling_max_us, 20);
    EXPECT_EQ(st.pps.over, 0u);
    EXPECT_LT(first, 15.0);
    EXPECT_TRUE(d.sync.driftFitted());
    EXPECT_GE(r.timed.size(), 400u);
}

// The 1PPS late to the step (review, 2026-10-17): a receiver that re-aligns
// its pulse to the new sky only some seconds after its first fix leaves the
// edges, and the line on them, on the old sky while its PVTs state the new
// one. The PPS check reads 0 there (the line on its edges, the carry on its
// PVT) while every tag is off by the whole step; the chain's displacement
// (6.854 ms) against the edges' (0) keeps them untimed until the pulse moves
// and the line follows it. However long it lags, with the step at the first
// fix or after it.
TEST(MosaicTimeTag, AOnePpsLateToTheStepNeverConfirmsIt)
{
    for (double lag : {1.0, 2.0, 3.0, 5.0, 10.0})
        for (bool jwf : {false, true})
        {
            Bench b = underrun();
            b.pps_lags_a_second = true;
            b.pps_lag_s = lag;
            b.jump_with_first_fix = jwf;
            Driver d(false);
            d.trace = std::getenv("TAG_TRACE") != nullptr;
            double back = -1.0;
            const BenchRun r = run(b, d, 330.0, 177.0, &back);
            const auto& st = d.tag.stats();
            std::printf("[pps %.0f s late, step %s] timed %zu, worst %.1f us, back %.1f s after the first fix; PPS "
                        "judged max %d us over %u; untimed off the chain %u; line steps %u, later set aside %u\n",
                        lag, jwf ? "with the fix" : "after it", r.timed.size(), r.worst_us, back - 177.0,
                        st.pps.max_us, st.pps.over, st.why[(int)Why::OFF_CHAIN], d.sync.steps(), d.sync.laterRefused());
            EXPECT_LT(r.worst_us, 20.0) << lag << " " << jwf;
            EXPECT_EQ(st.pps.over, 0u);
            EXPECT_GT(st.why[(int)Why::OFF_CHAIN], 0u);
            EXPECT_EQ(d.sync.steps(), 1u);
            EXPECT_GT(back, 0.0);
            EXPECT_LT(back - 177.0, lag + 10.0);
        }
}

// A run of 1PPS edges stamped late by the same few hundred us or ms each
// second (an interrupt held off by something on the pulse's phase; review,
// 2026-10-17): with no step in the PVTs there is no reason for the line to
// step later, so it never does; those seconds are untimed and no tag is
// wrong, however long the run.
TEST(MosaicTimeTag, ARunOfLateEdgesNeverMovesTheLine)
{
    for (int n : {2, 3, 5, 10, 30})
        for (int64_t late : {300, 2000})
        {
            Bench b;
            b.late_from_k = 150;
            b.late_to_k = 150 + n - 1;
            b.late_run_us = late;
            Driver d(false);
            double back = -1.0;
            const BenchRun r = run(b, d, 260.0, 150.0 + n, &back);
            const auto& st = d.tag.stats();
            std::printf("[late run %d x %lld us] timed %zu, worst %.1f us, back %.1f s after it; line steps %u, later "
                        "set aside %u; holds check %u off chain %u; PPS judged over %u\n",
                        n, (long long)late, r.timed.size(), r.worst_us, back - 150.0 - n, d.sync.steps(),
                        d.sync.laterRefused(), st.breaks[(int)Break::CHECK_FAILED], st.breaks[(int)Break::OFF_CHAIN],
                        st.pps.over);
            EXPECT_LT(r.worst_us, 20.0) << n << " x " << late;
            EXPECT_EQ(d.sync.steps(), 0u);
            EXPECT_EQ(st.pps.over, 0u);
            EXPECT_GE(d.sync.laterRefused(), static_cast<uint32_t>(n));
            EXPECT_LT(back - 150.0 - n, 1.5) << n << " x " << late;
        }
}

// A sky come back early by whole quarters of a second (review, 2026-10-17):
// the receiver steps its clock forward by about k * 256 ms, which its jump
// count (mod 256) shows as 0 or -1 ms, and its epochs skip ahead. The
// arrivals place the jump (MosaicJumpState), so the chain carries the bias
// across it and sees the sky's step whether the first fix states it before
// the clock steps or after; a pre-step bias of a second or more anchors too.
// Nothing is timed wrong; the 1PPS moved more than half a second restarts
// the line (10 s to valid).
TEST(MosaicTimeTag, ASkyBackEarlyByWholeQuartersOfASecondIsSeen)
{
    for (double adv : {100.0, 256.0, 512.0, 1023.0, 1280.0})
        for (bool jwf : {false, true})
        {
            Bench b;
            b.sky_delay_ms = -adv;
            b.sky_delay_at_s = 160.0;
            b.lost_from_s = 160.0;
            b.lost_to_s = 175.0;
            b.fix_after_s = 2.0;
            b.jump_with_first_fix = jwf;
            Driver d(false);
            d.trace = std::getenv("TAG_TRACE") != nullptr;
            double back = -1.0;
            const BenchRun r = run(b, d, 260.0, 177.0, &back);
            const auto& st = d.tag.stats();
            std::printf("[sky %.0f ms early, step %s] timed %zu, worst %.1f us, back %.1f s after the first fix; "
                        "jumps placed %u, resets %u; breaks rebase %u bias step %u (%+.3f ms) line restart %u; line "
                        "steps %u restarts %u\n",
                        adv, jwf ? "with the fix" : "after it", r.timed.size(), r.worst_us, back - 177.0, d.js.resolved,
                        d.js.resets, st.breaks[(int)Break::REBASE], st.breaks[(int)Break::BIAS_STEP],
                        st.last_bias_step_ms, st.breaks[(int)Break::LINE_RESTART], d.sync.steps(), d.sync.restarts());
            EXPECT_LT(r.worst_us, 20.0) << adv << " " << jwf;
            EXPECT_EQ(st.pps.over, 0u);
            EXPECT_EQ(d.js.resets, 0u);
            EXPECT_GT(back, 0.0);
            EXPECT_LT(back - 177.0, 25.0) << adv << " " << jwf;
        }
}

// A sky come back late by a tenth to most of a second (the rig's run 2 started
// a little before the sky's own time: the later step pc1 had, larger): the
// receiver steps its clock back by that, its epochs going on on their grid,
// and its count shows the step mod 256 (-200 ms reads +56). The arrivals
// place it, so the chain sees the sky's step; the 1PPS moves later by as
// much, onto the chain, and the line steps later onto it (a step over half a
// second restarts it instead). Nothing is timed wrong. A step of exactly
// k * 256 ms (a count of 0) is not placed: a late stamp looks the same.
TEST(MosaicTimeTag, ASkyBackLateIsSeen)
{
    for (double late : {6.854, 100.0, 200.0, 300.0, 450.0, 600.0})
        for (bool jwf : {false, true})
        {
            Bench b;
            b.sky_delay_ms = late;
            b.sky_delay_at_s = 160.0;
            b.lost_from_s = 160.0;
            b.lost_to_s = 175.0;
            b.fix_after_s = 2.0;
            b.jump_with_first_fix = jwf;
            Driver d(false);
            d.trace = std::getenv("TAG_TRACE") != nullptr;
            double back = -1.0;
            const BenchRun r = run(b, d, 260.0, 177.0, &back);
            const auto& st = d.tag.stats();
            std::printf("[sky %.3f ms late, step %s] timed %zu, worst %.1f us, back %.1f s after the first fix; "
                        "jumps placed %u, resets %u; breaks rebase %u bias step %u (%+.3f ms) line restart %u; line "
                        "steps %u (%+.3f ms) restarts %u\n",
                        late, jwf ? "with the fix" : "after it", r.timed.size(), r.worst_us, back - 177.0,
                        d.js.resolved, d.js.resets, st.breaks[(int)Break::REBASE], st.breaks[(int)Break::BIAS_STEP],
                        st.last_bias_step_ms, st.breaks[(int)Break::LINE_RESTART], d.sync.steps(), d.sync.lastStepMs(),
                        d.sync.restarts());
            EXPECT_LT(r.worst_us, 20.0) << late << " " << jwf;
            EXPECT_EQ(st.pps.over, 0u);
            EXPECT_EQ(d.js.resets, 0u);
            EXPECT_GT(back, 0.0);
            EXPECT_LT(back - 177.0, 25.0) << late << " " << jwf;
            if (late < 500.0)
                EXPECT_EQ(d.sync.steps(), 1u) << late << " " << jwf;
        }
}

// A link backed up a second for two and a half minutes (a relink at a low
// baud rate with the 20 Hz streams queued): a block that comes a second late
// pairs the next second's edge, and a line on those labels passes every PPS
// check (its tags and its edges carry the same wrong second). The driver
// pairs nothing while its MeasEpochs come that late against the earliest
// lately. Without that, once the reference is too old to object, the
// mislabelled line is confirmed and its tags go out a second wrong.
TEST(MosaicTimeTag, ABackedUpLinkPairsNoEdge)
{
    for (bool guard : {true, false})
    {
        Bench b;
        b.backlog_from_s = 150.0;
        b.backlog_to_s = 300.0;
        b.backlog_s = 1.0;
        Driver d(false);
        d.pair_guard = guard;
        double back = -1.0;
        const BenchRun r = run(b, d, 360.0, 300.0, &back);
        const auto& st = d.tag.stats();
        std::printf("[backlog 1 s for 150 s, guard %d] timed %zu, worst %.0f us, back %.1f s after it; line restarts "
                    "%u; holds line restart %u off chain %u; references dropped %u\n",
                    guard, r.timed.size(), r.worst_us, back - 300.0, d.sync.restarts(),
                    st.breaks[(int)Break::LINE_RESTART], st.breaks[(int)Break::OFF_CHAIN], st.refs_dropped);
        if (guard)
        {
            EXPECT_LT(r.worst_us, 20.0);
            EXPECT_EQ(d.sync.restarts(), 0u);
            EXPECT_GT(back, 0.0);
            EXPECT_LT(back - 300.0, 6.0);
        }
        else
        {
            EXPECT_GT(r.worst_us, 9.0e5);   // what the guard is for
        }
    }
}

// ---- the rule piece by piece

namespace
{

PvtGeodetic pvtAt(int64_t rx_ms, double bias_ms, double drift_ppm = 0.3, bool fix = true)
{
    PvtGeodetic p;
    p.wnc = static_cast<uint16_t>(rx_ms / kWeekMs);
    p.tow_ms = static_cast<uint32_t>(rx_ms % kWeekMs);
    p.has_solution = fix;
    p.rx_clk_bias_ms = bias_ms;
    p.rx_clk_drift_ppm = static_cast<float>(drift_ppm);
    return p;
}

// note a MeasEpoch header at rx_ms with jump count cum
MosaicEpochInfo noteAt(MosaicJumpState& js, int64_t rx_ms, uint8_t cum)
{
    sbftest::MeasEpochSpec s;
    s.wnc = static_cast<uint16_t>(rx_ms / kWeekMs);
    s.tow = static_cast<uint32_t>(rx_ms % kWeekMs);
    s.cum = cum;
    s.t1 = {gpsSat(3)};
    const auto blk = sbftest::measEpoch(s);
    MosaicEpochInfo info;
    EXPECT_TRUE(noteMeasEpochClock(blk.data(), blk.size(), js, info));
    return info;
}

}  // namespace

// The bias is carried up to carry_ms across the jumps since, never past a
// restart of the count, and an epoch without an anchor is untimed, not dtr 0.
TEST(MosaicTimeTag, CarriesTheBiasTwoMinutesAndNeverGuesses)
{
    MosaicTimeTag tag;
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    double dtr = 0.0;
    MosaicEpochInfo i0 = noteAt(js, t0, 10);
    EXPECT_FALSE(tag.carry(i0, js, dtr));   // no PVT yet: untimed
    tag.notePvt(pvtAt(t0, 0.31, 0.5), js, 1000000, true);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::FIRST);
    // 90 s later, one -1 ms step since
    for (int64_t ms = 50; ms <= 90000; ms += 50)
    {
        const MosaicEpochInfo i = noteAt(js, t0 + ms, ms >= 60000 ? 9 : 10);
        if (ms == 90000)
        {
            ASSERT_TRUE(tag.carry(i, js, dtr));
            EXPECT_NEAR(dtr, 0.31 + 0.5e-3 * 90.0 - 1.0, 1e-9);
        }
    }
    // past two minutes: nothing to carry
    const MosaicEpochInfo late = noteAt(js, t0 + 120050, 9);
    EXPECT_FALSE(tag.carry(late, js, dtr));
    // the count restarts (time backwards): nothing carried across it
    MosaicTimeTag tag2;
    MosaicJumpState js2;
    noteAt(js2, t0, 10);
    tag2.notePvt(pvtAt(t0, 0.31), js2, 1000000, true);
    const MosaicEpochInfo back = noteAt(js2, t0 - 5000, 10);
    EXPECT_EQ(js2.rebases, 1u);
    EXPECT_FALSE(tag2.carry(back, js2, dtr));
}

// A gap of MeasEpochs longer than kMosaicMeasGapMs restarts the jump count:
// a reset whose CumClkJumps lands within 2 ms of the last would otherwise
// read as a jump.
TEST(MosaicTimeTag, AGapInTheMeasEpochsRestartsTheJumpCount)
{
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    noteAt(js, t0, 254);
    noteAt(js, t0 + 50, 254);
    MosaicEpochInfo i = noteAt(js, t0 + 50 + kMosaicMeasGapMs, 254);   // at the gap's limit: no restart
    EXPECT_FALSE(i.rebased);
    EXPECT_EQ(js.rebases, 0u);
    i = noteAt(js, t0 + 12000, 0);                                      // 254 -> 0 after 10 s: a reset
    EXPECT_TRUE(i.rebased);
    EXPECT_EQ(js.rebases, 1u);
    EXPECT_EQ(js.j_ms, 0);
}

// A PVT off the bias carried to it breaks the chain; within bias_step_ms it
// does not; across a counted jump the carried bias steps with it.
TEST(MosaicTimeTag, ABiasStepBreaksTheChain)
{
    MosaicTimeTag tag;
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    noteAt(js, t0, 10);
    tag.notePvt(pvtAt(t0, 0.49, 0.3), js, 1000000, true);
    // confirm the chain
    for (int k = 1; k <= 2; ++k)
    {
        const int64_t rx = t0 + 1000 * k;
        noteAt(js, rx, 10);
        tag.notePair(rx / 1000, 777000000 + k, true, 777000000 + k, 0.0, 0u);
        tag.noteWholeSecond(rx, 777000000 + k - 490, false);
        tag.notePvt(pvtAt(rx, 0.49 + 0.3e-3 * k), js, 1000000, true);
    }
    EXPECT_FALSE(tag.held());
    EXPECT_EQ(tag.stats().confirmed, 1u);
    // a -1 ms jump: carried, no break
    noteAt(js, t0 + 2050, 9);
    tag.notePvt(pvtAt(t0 + 2050, 0.49 + 0.3e-3 * 2.05 - 1.0), js, 1000000, true);
    EXPECT_FALSE(tag.held());
    // 0.09 ms off: within the step
    noteAt(js, t0 + 2100, 9);
    tag.notePvt(pvtAt(t0 + 2100, 0.49 + 0.3e-3 * 2.1 - 1.0 + 0.09), js, 1000000, true);
    EXPECT_FALSE(tag.held());
    // 6.85 ms off: a step in the receiver's GPS time against its clock
    noteAt(js, t0 + 2150, 9);
    tag.notePvt(pvtAt(t0 + 2150, 0.49 + 0.3e-3 * 2.15 - 1.0 + 0.09 + 6.85), js, 1000000, true);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::BIAS_STEP);
    EXPECT_NEAR(tag.stats().last_bias_step_ms, 6.85, 1e-6);
    // a PVT without a solution, or whose MeasEpoch was not the last, anchors nothing
    tag.notePvt(pvtAt(t0 + 2200, 0.0, 0.3, false), js, 1000000, true);
    tag.notePvt(pvtAt(t0 + 2200, 0.0), js, 1000000, true);
    EXPECT_EQ(tag.stats().pvt_passed, 1u);
}

// The hold: released by confirm passing checks in a row of tags placed in
// the chain; a failed one starts the count over; a check of a tag placed
// before the break says nothing; an edge off the line stops the tags until
// one is back on it; a failed check of a tag handed over breaks the chain.
TEST(MosaicTimeTag, TheHoldWantsTwoChecksInARow)
{
    MosaicTimeTag tag;
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    tag.noteFineTime(true);
    auto second = [&](int k, int32_t tag_off_us, bool published, bool edge_on = true) {
        const int64_t rx = t0 + 1000 * k;
        noteAt(js, rx, 10);
        const int64_t edge = 900000000 + 1000000LL * k;
        tag.notePair(rx / 1000, edge, true, edge_on ? edge - 3 : edge - 500, 0.0, 0u);
        tag.noteWholeSecond(rx, edge - 250 + tag_off_us, published);
        tag.notePvt(pvtAt(rx, 0.25), js, 1000000, true);
    };
    second(0, 0, false);                       // the first anchor: held
    EXPECT_EQ(tag.judge(true, true, 900000000), Why::HELD);
    second(1, 0, false);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.passes(), 1u);
    second(2, 150, false);                     // 150 us off: the count starts over
    EXPECT_EQ(tag.passes(), 0u);
    second(3, 0, false);
    second(4, 0, false);
    EXPECT_FALSE(tag.held());
    EXPECT_EQ(tag.judge(true, true, 904000000), Why::TIMED);
    // an edge off the line: no tag until one is back on it
    second(5, 0, true, false);
    EXPECT_EQ(tag.judge(true, true, 905000000), Why::EDGE_OFF);
    EXPECT_FALSE(tag.held());
    second(6, 0, true);
    EXPECT_EQ(tag.judge(true, true, 906000000), Why::TIMED);
    // a tag handed over that fails the check breaks the chain
    second(7, 300, true);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::CHECK_FAILED);
    EXPECT_EQ(tag.stats().pps.over, 1u);
    // the other reasons, in order
    EXPECT_EQ(tag.judge(false, true, 907000000), Why::NO_LINE);
    tag.noteFineTime(false);
    EXPECT_EQ(tag.judge(true, true, 907000000), Why::NOT_FINETIME);
    tag.noteFineTime(true);
    EXPECT_EQ(tag.judge(true, true, 907000000 + 120000000), Why::PPS_STALE);
    EXPECT_EQ(tag.judge(true, false, 907000000), Why::NO_BIAS);
}

// A whole second's tag placed before a break, checked at the PVT that breaks
// it, does not count toward confirming the new chain.
TEST(MosaicTimeTag, ACheckFromBeforeABreakDoesNotConfirm)
{
    MosaicTimeTag::Config c;
    c.confirm = 1;
    MosaicTimeTag tag(c);
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    noteAt(js, t0, 10);
    tag.notePvt(pvtAt(t0, 0.25), js, 1000000, true);   // FIRST: held
    // a tag placed under the first chain, its PVT stating a bias 2 ms off:
    // the check passes against the old bias's tag (its edge sits where it
    // was), and the bias step breaks the chain after it
    const int64_t rx = t0 + 1000;
    noteAt(js, rx, 10);
    tag.notePair(rx / 1000, 5000000, true, 5000000, 0.0, 0u);
    tag.noteWholeSecond(rx, 5000000 - 2250, false);   // right for the new bias, so the check passes...
    tag.notePvt(pvtAt(rx, 2.25), js, 1000000, true);
    EXPECT_FALSE(tag.held() && tag.holdBreak() == Break::FIRST);   // ...and releases the first chain
    EXPECT_TRUE(tag.held());                                       // then the step holds the next
    EXPECT_EQ(tag.holdBreak(), Break::BIAS_STEP);
    EXPECT_EQ(tag.passes(), 0u);
}

// The edges against the chain, piece by piece: a release sets the reference;
// an edge that moved while the PVTs did not is off the chain (untimed, and on
// the line it breaks the chain); one that moved with them is on it and lets
// the line step later onto it (edgeOnChain); a restart of the line breaks the
// chain; a reference left two minutes is dropped and the next chain is the
// 1PPS's alone; a bias carried across a big jump (a start-over that kept the
// clock) shows the sky's step, and across a lost clock nothing is carried.
TEST(MosaicTimeTag, TheEdgesAgainstTheChain)
{
    MosaicTimeTag tag;
    MosaicJumpState js;
    const int64_t t0 = int64_t(2440) * kWeekMs + 100000000;
    tag.noteFineTime(true);
    int64_t moved_us = 0;     // the 1PPS against the reference
    uint32_t restarts = 0;
    auto second = [&](int k, double bias_ms, int64_t line_off_us = 0) {
        const int64_t rx = t0 + 1000 * k;
        noteAt(js, rx, 10);
        const int64_t edge = 900000000 + 1000000LL * k + moved_us;
        tag.notePair(rx / 1000, edge, true, edge - 2 - line_off_us, 0.0, restarts);
        tag.noteWholeSecond(rx, edge - 2 - line_off_us - static_cast<int64_t>(std::llround(bias_ms * 1000.0)),
                            false);
        tag.notePvt(pvtAt(rx, bias_ms, 0.0), js, 1000000, true);
    };
    second(0, 0.25);
    EXPECT_FALSE(tag.refHave());          // the first anchor: no chain before it
    second(1, 0.25);
    second(2, 0.25);
    ASSERT_FALSE(tag.held());
    EXPECT_TRUE(tag.refHave());           // the release set it
    EXPECT_EQ(tag.judge(true, true, 902000000), Why::TIMED);

    // the edge moves 3 ms later, the PVTs do not: off the line and the chain, no hold
    moved_us = 3000;
    EXPECT_FALSE(tag.edgeOnChain((t0 + 3000) / 1000, 903000000 + 3000));
    second(3, 0.25, 3000);   // the line still where it was
    EXPECT_FALSE(tag.onChain());
    EXPECT_EQ(tag.judge(true, true, 903000000), Why::EDGE_OFF);
    EXPECT_FALSE(tag.held());
    // the line follows it anyway: on the line, off the chain -> held
    second(4, 0.25);
    EXPECT_EQ(tag.judge(true, true, 904000000), Why::OFF_CHAIN);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::OFF_CHAIN);
    // its checks pass (the line on its edges, the carry on its PVT) and confirm nothing
    second(5, 0.25);
    second(6, 0.25);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.passes(), 0u);
    // the PVTs move 3 ms too: on the chain, and the line may step later onto such an edge
    second(7, 3.25);
    EXPECT_NEAR(tag.chainMs(), 3.0, 1e-9);
    EXPECT_TRUE(tag.edgeOnChain((t0 + 8000) / 1000, 908000000 + 3000));
    EXPECT_FALSE(tag.edgeOnChain((t0 + 8000) / 1000, 908000000));
    second(8, 3.25);
    EXPECT_TRUE(tag.onChain());
    second(9, 3.25);
    EXPECT_FALSE(tag.held());
    EXPECT_NEAR(tag.chainMs(), 0.0, 1e-9);   // the release moved the reference up
    EXPECT_EQ(tag.edgeRefUs(), 2);

    // a restart of the line: held until confirmed anew
    restarts = 1;
    second(10, 3.25);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::LINE_RESTART);
    second(11, 3.25);
    second(12, 3.25);
    EXPECT_FALSE(tag.held());

    // two minutes and more with no trusted edge (edges off the line all along, no whole second's epoch to
    // check): the reference is dropped, the chain held, and the 1PPS alone confirms the next one
    for (int k = 13; k < 140; ++k)
    {
        const int64_t rx = t0 + 1000 * k;
        noteAt(js, rx, 10);
        const int64_t edge = 900000000 + 1000000LL * k + moved_us;
        tag.notePair(rx / 1000, edge, true, edge - 500, 0.0, restarts);
        EXPECT_EQ(tag.judge(true, true, edge), Why::EDGE_OFF);
        tag.notePvt(pvtAt(rx, 3.25, 0.0), js, 1000000, true);
    }
    EXPECT_FALSE(tag.refHave());
    EXPECT_EQ(tag.stats().refs_dropped, 1u);
    EXPECT_TRUE(tag.held());
    EXPECT_EQ(tag.holdBreak(), Break::OFF_CHAIN);
    second(140, 3.25);
    second(141, 3.25);
    EXPECT_FALSE(tag.held());
    EXPECT_TRUE(tag.refHave());

    // a big jump the count starts over at (a -7 ms step: the clock kept) with the sky 6.854 ms later: the
    // chain carries across it and sees the step
    MosaicTimeTag t2;
    MosaicJumpState j2;
    noteAt(j2, t0, 20);
    t2.notePvt(pvtAt(t0, 0.30, 0.0), j2, 1000000, true);
    noteAt(j2, t0 + 50, 13);
    EXPECT_EQ(j2.rebases, 1u);
    EXPECT_EQ(j2.resets, 0u);
    EXPECT_EQ(j2.jsum_ms, -7);
    t2.notePvt(pvtAt(t0 + 50, 0.30 + 6.854 - 7.0, 0.0), j2, 1000000, true);
    EXPECT_EQ(t2.holdBreak(), Break::REBASE);
    EXPECT_NEAR(t2.chainMs(), 6.854, 1e-9);
    // a reset (time going back): its new bias says nothing of the sky
    noteAt(j2, t0 - 4000, 0);
    EXPECT_EQ(j2.resets, 1u);
    t2.notePvt(pvtAt(t0 - 4000, 12.3, 0.0), j2, 1000000, true);
    EXPECT_EQ(t2.holdBreak(), Break::RESET);
    EXPECT_NEAR(t2.chainMs(), 6.854, 1e-9);   // the sky taken not to have moved since
}

// ---- the rig replay's clock on the tag (#1600 on the V9 mosaic image)
//
// A rig replay build sets the replay's clock from the epochs the driver hands
// over (TR_Sensor_Collector_Sim replayNoteEpoch, ReplayClock::onEpoch): a
// timed one at its tag's GPS instant (the receiver's time of week less the
// bias the tag took off, TimedRawEpoch::tag_dtr_ms), an untimed or held one
// (mcu_us and tag_dtr_ms 0) not at all. The freeze's 1PPS check takes the
// driver's last residual with its own second (PpsTagStats::last_s): only a
// second handed over timed is judged there, a held one's check is the
// driver's alone. The 10/17 underrun on the rig's pad: the sky 6.854 ms late
// at 160 s, lost 15 s, the tag held after the first fix until the 1PPS
// confirms the new chain.

namespace
{

struct RigReplayRun
{
    sim_replay::ReplayClock clock;
    int handed = 0, timed = 0, untimed = 0;
    int untimed_counted = 0;        // untimed epochs the clock counted: never
    int checks = 0;                 // 1PPS checks handed to the clock
    int checks_not_timed = 0;       //   of a second not handed over timed: never
    double freeze_t = -1.0;         // the bench time of the epoch that froze the clock
    double freeze_at_t = 0.0;       //   its replay time, s from ignition: the console's "frozen for the
                                    //   flight (the receiver's timed epoch at t X s)"
    double freeze_step_us = 0.0;    //   how far it moved the clock at its own MCU instant (us)
    double freeze_sky_step_us = 0.0;   //   the sky's: its time less the MCU's since the last epoch counted
    double ignition_gps_s = 0.0;
};

// The bench's sky GPS time (s) at bench time t: the receiver's time less its bias
double skyGpsS(const Bench& b, double t) { return b.gps0_ms * 1e-3 + t - b.skyDelayMs(t) * 1e-3; }

// The underrun bench through the driver, its epochs drained as a replay
// build's GNSS task drains them (replayNoteEpoch), ignition at bench time
// ignition_t on the sky's time.
void rigReplay(double ignition_t, Driver& d, RigReplayRun& rr)
{
    const Bench b = underrun();
    rr.ignition_gps_s = skyGpsS(b, ignition_t);
    sim_replay::Header h;
    h.ignition_gps_s = rr.ignition_gps_s;
    h.scen_start_s = -ignition_t;          // the rig file spans the whole bench
    h.scen_end_s = 330.0 - ignition_t;
    rr.clock.reset(h);
    std::set<int64_t> timed_seconds;
    uint32_t seen = 0;
    double last_counted_t = -1.0;
    run(b, d, 330.0, 1e9, nullptr, [&](const Out& o) {
        rr.handed++;
        // the driver's checks so far, before this epoch's own PVT, read as the
        // sim reads them as it drains the epoch (replayNoteEpoch: two reads,
        // taken when they agree; here nothing writes between the two)
        const sim_replay::TagCheckRead chk = sim_replay::tagCheckRead(d.tag.stats().pps);
        const bool new_check = sim_replay::newTagCheck(chk, sim_replay::tagCheckRead(d.tag.stats().pps), seen);
        const auto before = rr.clock.state();
        const uint32_t fixes = rr.clock.fixes();
        double t_before = 0.0;
        const bool had_t = rr.clock.tAt(o.mcu_us, t_before);
        rr.clock.onEpoch(static_cast<uint16_t>(o.rx_ms / kWeekMs), static_cast<double>(o.rx_ms % kWeekMs) * 1e-3,
                         o.mcu_us, o.dtr_ms * 1e-3);
        if (o.mcu_us == 0)
        {
            rr.untimed++;
            EXPECT_EQ(o.dtr_ms, 0.0);
            if (rr.clock.fixes() != fixes)
                rr.untimed_counted++;
        }
        else
        {
            rr.timed++;
            if (o.rx_ms % 1000 == 0)
                timed_seconds.insert(o.rx_ms / 1000);
        }
        if (before != sim_replay::ReplayClock::kFrozen && rr.clock.state() == sim_replay::ReplayClock::kFrozen)
        {
            rr.freeze_t = o.t;
            rr.freeze_at_t = rr.clock.lastFixT();
            double t_after = 0.0;
            if (had_t && rr.clock.tAt(o.mcu_us, t_after))
                rr.freeze_step_us = (t_after - t_before) * 1e6;
            if (last_counted_t >= 0.0)
                rr.freeze_sky_step_us = (skyGpsS(b, o.t) - skyGpsS(b, last_counted_t)) * 1e6 -
                                        (b.mcuUs(o.t) - b.mcuUs(last_counted_t));
        }
        if (rr.clock.fixes() != fixes)
            last_counted_t = o.t;
        if (new_check)
        {
            rr.checks++;
            if (timed_seconds.count(chk.sec) == 0)
                rr.checks_not_timed++;
            rr.clock.onTagCheck(static_cast<double>(chk.sec), chk.residual_us);
        }
    });
}

// The replayed flight's time at bench time t against the sky's, us, from the
// freeze on: the sky's time at the freezing epoch, carried on by the MCU's
// crystal (16 ppm fast here) as the frozen clock is by design (SimReplay.h:
// the samples run on the MCU's crystal from the freeze, as a real IMU's do).
double rigClockErrUs(const RigReplayRun& rr, double t)
{
    const Bench b = underrun();
    double tr = 0.0;
    if (!rr.clock.tAt(static_cast<int64_t>(std::llround(b.mcuUs(t))), tr))
        return 1e12;
    const double crystal_s = (b.mcuUs(t) - b.mcuUs(rr.freeze_t)) * 1e-6 - (skyGpsS(b, t) - skyGpsS(b, rr.freeze_t));
    return (tr - (skyGpsS(b, t) - rr.ignition_gps_s) - crystal_s) * 1e6;
}

}  // namespace

// The freeze 50 s after the underrun: the held epochs set nothing, the clock
// follows the timed ones back onto the late sky, and the freeze and its
// 1PPS check stand on the sky's time.
TEST(MosaicTimeTag, TheRigReplaysClockTakesOnlyTheTimedEpochs)
{
    Driver d(false);
    RigReplayRun rr;
    rigReplay(230.0, d, rr);
    const auto& pps = d.tag.stats().pps;
    std::printf("[rig] handed %d: timed %d, untimed %d (counted %d); clock fixes %u, outside %u, restarts %u; froze "
                "at %.2f s, check %d (t %+.1f s, residual %d us, move %.1f us); checks %d (%d not timed); driver: "
                "judged %u, held %u\n",
                rr.handed, rr.timed, rr.untimed, rr.untimed_counted, rr.clock.fixes(), rr.clock.outside(),
                rr.clock.restarts(), rr.freeze_t, (int)rr.clock.check(), rr.clock.checkT(),
                rr.clock.checkResidualUs(), rr.clock.checkMoveUs(), rr.checks, rr.checks_not_timed, pps.checks,
                pps.held);
    EXPECT_GT(rr.untimed, 20);             // the hold after the first fix
    EXPECT_GT(pps.held, 1u);               // whose seconds the driver checked, not judged
    EXPECT_EQ(rr.untimed_counted, 0);
    EXPECT_EQ(rr.clock.fixes(), static_cast<uint32_t>(rr.timed));
    EXPECT_EQ(rr.clock.outside(), 0u);
    EXPECT_EQ(rr.clock.restarts(), 0u);
    EXPECT_GT(rr.checks, 100);
    EXPECT_EQ(rr.checks_not_timed, 0);
    ASSERT_EQ(rr.clock.state(), sim_replay::ReplayClock::kFrozen);
    EXPECT_NEAR(rr.freeze_t, 225.0, 0.11);
    EXPECT_GE(rr.freeze_at_t, -5.0);         // the arming instant, as score_v9.py reads it: no mark
    EXPECT_LT(rr.freeze_at_t, -4.89);
    EXPECT_LT(std::fabs(rr.freeze_step_us), 25.0);   // on the sky it followed a tenth of a second before
    EXPECT_EQ(rr.clock.check(), sim_replay::ReplayClock::kKept);
    EXPECT_NEAR(rr.clock.checkT(), -5.0, 0.01);   // the freeze's own second: the receiver's, its bias off GPS
    EXPECT_LT(std::fabs(rr.clock.checkMoveUs()), 25.0);
    for (double t : {226.0, 230.0, 241.7, 300.0, 329.0})
        EXPECT_LT(std::fabs(rigClockErrUs(rr, t)), 25.0) << t;
}

// The freeze inside the hold (5 s before an ignition 6 s after the first
// fix): the clock follows the old sky through the hold, never a held epoch,
// and the first timed epoch after the release freezes it on the late sky's
// time -- the stale clock, 6.854 ms ahead and the crystal's drift since, is
// never frozen. The release comes
// after T-2 s, so the freeze goes unchecked (score_v9.py marks it), and the
// step onto the late sky is on the pad, where the replayed rocket stands
// still.
TEST(MosaicTimeTag, ARigReplayFreezeInsideTheHoldWaitsForATimedEpoch)
{
    Driver d(false);
    RigReplayRun rr;
    rigReplay(183.0, d, rr);
    std::printf("[rig] hold: handed %d: timed %d, untimed %d (counted %d); froze at %.2f s (t %+.2f, step %+.0f us), "
                "check %d (t %+.1f s, residual %d us, move %.1f us)\n",
                rr.handed, rr.timed, rr.untimed, rr.untimed_counted, rr.freeze_t, rr.freeze_at_t, rr.freeze_step_us,
                (int)rr.clock.check(), rr.clock.checkT(), rr.clock.checkResidualUs(), rr.clock.checkMoveUs());
    EXPECT_EQ(rr.untimed_counted, 0);
    EXPECT_EQ(rr.clock.restarts(), 0u);
    ASSERT_EQ(rr.clock.state(), sim_replay::ReplayClock::kFrozen);
    EXPECT_GT(rr.freeze_t, 178.0);          // not at the arming instant: its epochs were held
    EXPECT_GT(rr.freeze_at_t, -2.0);        // after the check's deadline...
    EXPECT_LT(rr.freeze_at_t, 0.0);         // ...and before ignition: on the pad
    // the step onto the late sky: the slip, and the crystal's 16 ppm over the
    // 22 s the clock ran on its own
    EXPECT_NEAR(rr.freeze_step_us, rr.freeze_sky_step_us, 25.0);
    EXPECT_LT(rr.freeze_step_us, -6854.0);
    EXPECT_GT(rr.freeze_step_us, -6854.0 - 16e-6 * 25e6);
    EXPECT_EQ(rr.clock.check(), sim_replay::ReplayClock::kUnchecked);
    for (double t : {rr.freeze_t + 0.05, 190.0, 200.0, 250.0})
        EXPECT_LT(std::fabs(rigClockErrUs(rr, t)), 25.0) << t;
}

// The hold across both T-5 s and ignition (the 10/17 underrun 18 s before
// an ignition at bench second 178: the sky lost from T-18 s, back at T-3 s,
// the first fix at T-1 s, the tag released at T+4.1 s): the clock follows
// the old sky through the hold, so the replayed flight starts on the stale
// clock, and the first timed epoch after the release freezes it in flight on
// the late sky, stepping the replayed IMU back by the slip and the crystal's
// drift since (7.2 ms) -- that much of the boost replayed twice. Nothing in the
// firmware stops it; the console's "frozen for the flight (the receiver's
// timed epoch at t X s)" with X >= 0 is what score_v9.py voids the run on.
TEST(MosaicTimeTag, ARigReplayHoldPastIgnitionFreezesInFlight)
{
    Driver d(false);
    RigReplayRun rr;
    rigReplay(178.0, d, rr);
    std::printf("[rig] hold past ignition: handed %d: timed %d, untimed %d (counted %d); froze at %.2f s (t %+.2f, "
                "step %+.0f us), check %d\n",
                rr.handed, rr.timed, rr.untimed, rr.untimed_counted, rr.freeze_t, rr.freeze_at_t, rr.freeze_step_us,
                (int)rr.clock.check());
    EXPECT_EQ(rr.untimed_counted, 0);
    ASSERT_EQ(rr.clock.state(), sim_replay::ReplayClock::kFrozen);
    EXPECT_GT(rr.freeze_at_t, 0.0);         // in flight: score_v9.py's VOID
    EXPECT_NEAR(rr.freeze_at_t, rr.freeze_t - 178.0, 0.01);
    EXPECT_NEAR(rr.freeze_step_us, rr.freeze_sky_step_us, 25.0);   // the slip and the crystal, as above
    EXPECT_LT(rr.freeze_step_us, -6854.0);
    EXPECT_EQ(rr.clock.check(), sim_replay::ReplayClock::kUnchecked);
}

// The replay's read of the driver's 1PPS check (sim_replay::tagCheckRead and
// newTagCheck, as replayNoteEpoch reads RawStats::tag.pps): a second handed
// over timed is a new check with its own second and residual; a held
// second's check moves prev_s and held only, never the count or last_s; two
// reads that differ (the poll task wrote between them) are no check.
TEST(MosaicTimeTag, TheRigReplayReadsTheDriversCheckOnlyWhenTwoReadsAgree)
{
    using sim_replay::newTagCheck;
    using sim_replay::tagCheckRead;
    PpsTagStats s;
    uint32_t seen = 0;
    const int64_t sec_us = 1000000;
    // a judged second
    notePpsTag(s, 1000, 5000000 + 30, 5000000, sec_us, true, 0.0, 200, true);
    sim_replay::TagCheckRead a = tagCheckRead(s);
    EXPECT_EQ(a.n, 1u);
    EXPECT_EQ(a.sec, 1000);
    EXPECT_EQ(a.residual_us, 30);
    EXPECT_TRUE(newTagCheck(a, tagCheckRead(s), seen));
    EXPECT_EQ(seen, 1u);
    EXPECT_FALSE(newTagCheck(a, tagCheckRead(s), seen));   // the same check again
    // a held second: the driver's alone
    notePpsTag(s, 1001, 6000000 + 7000, 6000000, sec_us, true, 0.0, 200, false);
    EXPECT_EQ(s.prev_s, 1001);
    EXPECT_EQ(s.held, 1u);
    a = tagCheckRead(s);
    EXPECT_EQ(a.n, 1u);
    EXPECT_EQ(a.sec, 1000);                  // last_s, not prev_s
    EXPECT_EQ(a.residual_us, 30);
    EXPECT_FALSE(newTagCheck(a, tagCheckRead(s), seen));
    // a check written between the two reads
    a = tagCheckRead(s);
    notePpsTag(s, 1002, 7000000 - 20, 7000000, sec_us, true, 0.0, 200, true);
    EXPECT_FALSE(newTagCheck(a, tagCheckRead(s), seen));
    EXPECT_EQ(seen, 1u);
    // a torn read of one check: the count agrees, the second or the residual does not
    sim_replay::TagCheckRead torn = tagCheckRead(s);
    torn.sec = 1001;
    EXPECT_FALSE(newTagCheck(torn, tagCheckRead(s), seen));
    torn = tagCheckRead(s);
    torn.residual_us = 30;
    EXPECT_FALSE(newTagCheck(torn, tagCheckRead(s), seen));
    // then two that agree: that check, its own second
    a = tagCheckRead(s);
    EXPECT_TRUE(newTagCheck(a, tagCheckRead(s), seen));
    EXPECT_EQ(seen, 2u);
    EXPECT_EQ(a.sec, 1002);
    EXPECT_EQ(a.residual_us, -20);
    // a settling second counts as a check handed over (not judged by the bar, but the clock's to pair)
    notePpsTag(s, 1003, 8000000 + 400, 8000000, sec_us, false, 0.0, 200, true);
    a = tagCheckRead(s);
    EXPECT_TRUE(newTagCheck(a, tagCheckRead(s), seen));
    EXPECT_EQ(a.n, 3u);
    EXPECT_EQ(a.sec, 1003);
}
