// SdrRawEpoch.h — the gnss-sdr-dev board's epochs as the raw-GNSS filter takes
// them (TR_GNSSRaw's RawEpoch). Pure and header-only, like Lc86RawEpoch.h and
// MosaicRawEpoch.h; tools/raw_replay (--receiver sdr) runs it on a log.
//
// Each of the board's epochs is its MSM7 messages (1077 GPS, 1097 Galileo,
// 1127 BeiDou, the multiple-message bit on all but the last) followed by
// TinkerRocket's 4001 sub-type 1 with the same time of week (SdrRtcm.h). The
// epoch is whole only with that record, which carries two things the MSM7
// cannot:
//   - The clock's steps. The board steps its clock onto GPS time (about
//     330 m, every ~7 s on the rig's TCXO), and every pseudorange and carrier
//     phase jumps with it, by minus the change in the record's step sum
//     (checked on the board's recordings of 2026-11-02: 60 steps in 7 min and
//     8 in a 52 s V10 log, each within a few cm of the sum's change). The
//     filter takes a common step only above 1 km (RawGnssEKFParams
//     clock_step_m), so here the sum goes back in: every pseudorange and phase
//     plus clk_step_m, and the time of week with it (plus clk_step_m / c). The
//     epoch is then on the board's own clock as if it had never stepped, its
//     ranges and its receive time consistent (each satellite's transmit time,
//     tow_s - pr / c, is the tag's), and the filter's clock bias is that
//     free-running clock's: continuous, metres a second of drift. The GPS
//     instant to put on the MCU clock is still the tag (gps_s), which the
//     record says is GPS time to within the clock's residual (< 1 us).
//   - The board's own marks (kMarkFix: the last fix's residual test left the
//     satellite's pseudorange out; kMarkNew: its channel has not settled):
//     what they cover is left out (Config).
// One signal a system: GPS L1 C/A (MSM signal 2), Galileo E1-C (2) and the
// BeiDou B1C pilot (31, at 1575.42 MHz: the receiver profile's BeiDou carrier
// has to say so, ReceiverProfile.h sdrDevProfile). QZSS and anything else is
// left out, as Lc86RawEpoch.h leaves it.
#ifndef SDR_RAW_EPOCH_H
#define SDR_RAW_EPOCH_H

#include <cmath>
#include <cstdint>

#include "RawMeas.h"     // TR_GNSSRaw
#include "Rtcm3Msm.h"    // TR_GNSSReceiverLC86_Serial: decodeMsm7
#include "SdrRtcm.h"
#include "gnss/types.h"

namespace sdr_rtcm
{

static constexpr double kC = 299792458.0;

// An epoch made whole by its 4001 sub-type 1.
struct SdrEpoch
{
    gnss_raw::RawEpoch epoch;     // tow_s and every range on the board's unstepped clock (see the top)
    uint16_t week = 0;            // the record's GPS week
    uint32_t tow_ms = 0;          // the MSMs' tag, GPS time of week
    double   gps_s = 0.0;         // week * 604800 + tow_ms / 1000: the GPS instant to place
    double   step_m = 0.0;        // the step sum put back (the record's clk_step_m)
    int64_t  arrival_us = 0;      // its record's last byte (the time sync's sample)
    uint8_t  mode = 0;            // the record's: 0 the fix's epochs, 1 the measurement mode's
    uint8_t  nobs = 0;            // observables the record says its MSMs carried
    uint8_t  ndrop = 0;           // ranges the board's checks left out of them
};

class SdrEpochAssembler
{
public:
    struct Config
    {
        uint32_t fuse_period_ms = 0;   // only the first epoch of each such period of GPS time (0: every one): the
                                       //   board's epochs sit at a millisecond phase of their own, not on the grid
        bool drop_fix_marked = true;   // kMarkFix: the pseudorange and its phase left out
        bool drop_new_marked = true;   // kMarkNew: the pseudorange, phase and Doppler left out
        bool restore_steps = true;     // the step sum put back (see the top); false: the board's ranges as sent
    };
    struct Counts
    {
        uint32_t msm = 0;              // MSM7 messages
        uint32_t epochs = 0;           // epochs made whole
        uint32_t off_grid = 0;         //   and left out as off the grid
        uint32_t no_record = 0;        // MSM epochs whose record never came (the next epoch began first)
        uint32_t lone_record = 0;      // records with no MSMs of their time
        uint32_t short_epochs = 0;     // epochs with fewer cells than the record's observables
        uint32_t cells = 0;            // cells kept
        uint32_t other_signal = 0;     // cells of another signal or system, left out
        uint32_t marked_pr = 0;        // pseudoranges left out by a mark
        uint32_t marked_rate = 0;      // Dopplers left out by a mark
        uint32_t full = 0;             // cells past RawEpoch::kMaxObs
    };

    SdrEpochAssembler() = default;
    explicit SdrEpochAssembler(const Config& cfg) : cfg_(cfg) {}
    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Counts& counts() const { return n_; }

    // A whole frame whose CRC checks (any message) and when its last byte
    // arrived (MCU us). True when it was the record that made an epoch whole
    // and that epoch is on the grid: `out` then holds it.
    bool onFrame(const uint8_t* frame, size_t len, int64_t arrival_us, SdrEpoch& out)
    {
        const uint16_t number = messageNumber(frame, len);
        if (number == 1077 || number == 1097 || number == 1117 || number == 1127)
        {
            onMsm(frame, len);
            return false;
        }
        TrEpoch rec;
        if (number != kTrMessage || !decodeTrEpoch(frame, len, rec))
            return false;
        if (!have_ || rec.tow_ms != tow_ms_)
        {
            n_.lone_record++;
            return false;
        }
        have_ = false;
        n_.epochs++;
        if (cfg_.fuse_period_ms != 0)
        {
            const int64_t k = static_cast<int64_t>(rec.week) * (604800000 / cfg_.fuse_period_ms) +
                              rec.tow_ms / cfg_.fuse_period_ms;
            if (k == last_period_)
            {
                n_.off_grid++;
                return false;
            }
            last_period_ = k;
        }
        if (raw_cells_ < rec.nobs)
            n_.short_epochs++;
        out.week = rec.week;
        out.tow_ms = rec.tow_ms;
        out.gps_s = rec.week * 604800.0 + rec.tow_ms * 1.0e-3;
        out.arrival_us = arrival_us;
        out.mode = rec.mode;
        out.nobs = rec.nobs;
        out.ndrop = rec.has_tail ? rec.ndrop : 0;
        gnss_raw::RawEpoch& e = out.epoch;
        e = ep_;
        const double step = cfg_.restore_steps ? rec.clk_step_m : 0.0;
        out.step_m = step;
        e.tow_s = rec.tow_ms * 1.0e-3 + step / kC;
        for (int i = 0; i < e.n; i++)
        {
            gnss_raw::RawObs& o = e.obs[i];
            if (o.has_pr) o.pr_m += step;
            if (o.has_phase) o.phase_m += step;
        }
        if (rec.has_tail)
        {
            for (int k = 0; k < rec.nmark; k++)
            {
                const TrMark& mk = rec.mark[k];
                const bool fix = cfg_.drop_fix_marked && (mk.flags & kMarkFix);
                const bool fresh = cfg_.drop_new_marked && (mk.flags & kMarkNew);
                if (!fix && !fresh) continue;
                for (int i = 0; i < e.n; i++)
                {
                    gnss_raw::RawObs& o = e.obs[i];
                    if (o.sys != mk.sys || o.prn != mk.prn) continue;
                    if (o.has_pr) n_.marked_pr++;
                    o.has_pr = false;
                    o.has_phase = false;
                    if (fresh)
                    {
                        if (o.has_rate) n_.marked_rate++;
                        o.has_rate = false;
                    }
                }
            }
        }
        return true;
    }

private:
    void onMsm(const uint8_t* frame, size_t len)
    {
        n_.msm++;
        rtcm3::MsmHeader h;
        rtcm3::MsmCell cells[RTCM_MSM_CAP];
        size_t dropped = 0;
        if (len < 6 || !rtcm3::decodeMsm7(frame + 3, len - 6, h, cells, RTCM_MSM_CAP, &dropped))
            return;
        if (h.gps_tow_ms == rtcm3::kInvalidTow)
            return;
        if (have_ && h.gps_tow_ms != tow_ms_)
        {
            n_.no_record++;          // the last epoch's record never came
            have_ = false;
        }
        if (!have_)
        {
            have_ = true;
            tow_ms_ = h.gps_tow_ms;
            ep_ = gnss_raw::RawEpoch();
            raw_cells_ = 0;
        }
        for (size_t i = 0; i < h.n_cells && i < RTCM_MSM_CAP; i++)
        {
            const rtcm3::MsmCell& c = cells[i];
            raw_cells_++;
            uint8_t sys;
            uint8_t want;
            switch (c.gnss_id)                       // UBX gnssId (Rtcm3Msm.h)
            {
                case 0: sys = GNSS_SYS_GPS; want = 2; break;    // L1 C/A
                case 2: sys = GNSS_SYS_GAL; want = 2; break;    // E1-C
                case 3: sys = GNSS_SYS_BDS; want = 31; break;   // B1C pilot
                default: n_.other_signal++; continue;
            }
            if (c.sig_id != want)
            {
                n_.other_signal++;
                continue;
            }
            if (ep_.n >= gnss_raw::RawEpoch::kMaxObs)
            {
                n_.full++;
                continue;
            }
            gnss_raw::RawObs& o = ep_.obs[ep_.n++];
            o.sys = sys;
            o.prn = c.sv_id;
            o.has_pr = std::isfinite(c.pseudorange_m);
            o.pr_m = o.has_pr ? c.pseudorange_m : 0.0;
            o.has_rate = std::isfinite(c.rate_mps);
            o.rate_mps = o.has_rate ? static_cast<double>(c.rate_mps) : 0.0;
            o.cn0_dbhz = c.cn0_dbhz;
            o.lock_ms = c.lock_ms;
            o.has_phase = std::isfinite(c.phase_range_m);
            o.halfcyc = c.half_cycle;
            o.phase_m = o.has_phase ? c.phase_range_m : 0.0;
            n_.cells++;
        }
    }

    // One MSM7 message's cells at most (RTCM's 64).
    static constexpr size_t RTCM_MSM_CAP = 64;

    Config   cfg_;
    Counts   n_;
    bool     have_ = false;    // MSM cells of tow_ms_ collected, their record not yet come
    uint32_t tow_ms_ = 0;
    int      raw_cells_ = 0;   // every cell of the epoch, kept or not
    int64_t  last_period_ = -1;   // the fused period the last epoch handed over fell in
    gnss_raw::RawEpoch ep_;
};

}  // namespace sdr_rtcm

#endif  // SDR_RAW_EPOCH_H
