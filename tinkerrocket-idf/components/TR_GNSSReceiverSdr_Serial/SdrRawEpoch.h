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
//
// The Dopplers between the fused epochs (SdrDopplerMean, below; off unless
// Config::mean asks): the board measures every 10 ms and the filter fuses
// every 100, so nine of every ten epochs' Dopplers would never reach it. Each
// grid epoch's Doppler can instead be the mean of its satellite's Dopplers
// over every epoch since the last grid epoch, itself included -- the mosaic's
// Dopplers in pairs (MosaicRawEpoch.h) widened from two epochs to the grid's
// whole period, and so to any fused rate: the window is always the epochs
// since the last one handed over. The filter takes such an epoch's Dopplers
// at the window's middle (RawGnssEKFParams::rr_pair_lag_s, half the window's
// span: 45 ms at 10 of 10 ms; ReceiverProfileApply.h sdrDopplerMeanLag).
//
// Everything the assembler keeps is a member (two epochs and one MSM7's cells,
// ~7 kB): the driver holds it off its poll task's stack.
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
    uint8_t  rr_mean = 0;         // 1: its Dopplers are their windows' means (SdrDopplerMean), at the window's middle
    uint16_t week = 0;            // the record's GPS week
    uint32_t tow_ms = 0;          // the MSMs' tag, GPS time of week
    double   gps_s = 0.0;         // week * 604800 + tow_ms / 1000: the GPS instant to place
    double   step_m = 0.0;        // the step sum put back (the record's clk_step_m)
    int64_t  arrival_us = 0;      // its record's last byte (the time sync's sample)
    uint8_t  mode = 0;            // the record's: 0 the fix's epochs, 1 the measurement mode's
    uint8_t  nobs = 0;            // observables the record says its MSMs carried
    uint8_t  ndrop = 0;           // ranges the board's checks left out of them
};


// ── The Dopplers between the fused epochs: their means ─────────────────
//
// A grid epoch's Doppler (valid, its C/N0 at or above its system's floor:
// one the builder would use) becomes the mean of its satellite's Dopplers
// over the window -- every epoch of the board's since the last grid epoch,
// this one included -- when:
//   - the window holds exactly n epochs, each period_ms after the one before
//     on the GPS-time tag (a lost epoch, a gap, a mode change or the start:
//     the whole epoch goes as it came);
//   - the satellite has a usable Doppler in every one of them (a board mark
//     that took it out of one, kMarkNew, counts as missing);
//   - its lock time never went down inside the window (a relock: the MSM7's
//     lock indicator, held as the cells carry it);
//   - no two consecutive Dopplers are more than max_step_mps apart: 5 m/s in
//     10 ms is 500 m/s^2, about 50 g along the line of sight, beyond any
//     boost these rockets fly (the hotshot's 35 g) and its 1 ppb/g TCXO step;
//     further apart is a slip or a false lock, which a mean would spread over
//     the window.
// The mean of range rates evenly spaced over the window is the range rate at
// its middle while the line-of-sight acceleration holds, off by the jerk
// times span^2 / 12 where it changes in it (a thrust step at the middle,
// about its size times 7.5 ms at 90 ms); the filter predicts it there. Where
// at least half of the grid epoch's usable Dopplers have a whole window,
// each takes its mean (the cell's C/N0, lock and sigma stay its own) and each
// one left without is dropped (has_rate cleared: its code and carrier stay);
// otherwise the epoch goes as it came, every Doppler at the epoch's instant.
// The clock's steps move no Doppler (only the code and carrier step), so a
// window across one is whole.
struct SdrDopplerMeanConfig
{
    uint32_t n = 0;                             // epochs a mean takes: the fused period over the board's (0, 1: off)
    uint32_t period_ms = 10;                    // the board's epochs this far apart (its measurement mode's)
    double   max_step_mps = 5.0;                // consecutive Dopplers at most this far apart
    float    cn0_min[GNSS_SYS_COUNT] = {};      // dB-Hz by gnss_sys_t: the profile's cn0_min_rr, at every epoch
};

// Counts since the assembler started: grid epochs handed over with means or
// as they came (and why), then their usable Dopplers, averaged, left at their
// epoch's instant with it, or dropped from an epoch of means by the first rule
// each one failed. dopplers = means + alone_epoch + the three drops.
struct SdrDopplerMeanStats
{
    uint32_t epochs = 0;          // grid epochs with a usable Doppler; handed over as they came:
    uint32_t epochs_short = 0;    //   their window not n consecutive epochs
    uint32_t epochs_few = 0;      //   fewer than half their Dopplers with a whole window; the rest with means
    uint32_t dopplers = 0;        // usable Dopplers in those epochs
    uint32_t means = 0;           //   averaged
    uint32_t alone_epoch = 0;     //   left as they came, with their epoch (the two above)
    uint32_t alone_missing = 0;   //   dropped: missing or unusable in an epoch of the window
    uint32_t alone_lock = 0;      //   dropped: a relock in the window
    uint32_t alone_step = 0;      //   dropped: two consecutive more than max_step_mps apart
    float    step_max_mps = 0.0f; // the largest step inside a window averaged
    float    spread_max_mps = 0.0f;   // the largest spread (max - min) of a window averaged
};

class SdrDopplerMean
{
public:
    void setConfig(const SdrDopplerMeanConfig& c)
    {
        cfg_ = c;
        restart();
    }
    const SdrDopplerMeanConfig& config() const { return cfg_; }
    bool on() const { return cfg_.n >= 2; }

    // Every epoch the assembler made whole, in order, its marks obeyed; t_ms
    // its GPS-time tag (week * 604800000 + time of week).
    void note(const gnss_raw::RawEpoch& e, int64_t t_ms)
    {
        if (!on())
            return;
        if (epochs_ > 0 && t_ms - last_t_ms_ != static_cast<int64_t>(cfg_.period_ms))
            broken_ = true;   // a lost epoch, a gap or a back-step: this window is not whole
        last_t_ms_ = t_ms;
        if (epochs_ < 0xFFFF)
            epochs_++;
        for (int i = 0; i < e.n; ++i)
        {
            const gnss_raw::RawObs& o = e.obs[i];
            if (!usable(o))
                continue;
            Acc* a = find(o.sys, o.prn);
            if (a == nullptr)
            {
                if (nacc_ >= gnss_raw::RawEpoch::kMaxObs)
                    continue;
                a = &acc_[nacc_++];
                *a = Acc{};
                a->sys = o.sys;
                a->prn = o.prn;
                a->lo = a->hi = static_cast<float>(o.rate_mps);
            }
            else
            {
                const double step = std::fabs(o.rate_mps - a->last);
                if (a->why == kOk && o.lock_ms < a->lock)
                    a->why = kLock;
                else if (a->why == kOk && !(step <= cfg_.max_step_mps))
                    a->why = kStep;
                if (static_cast<float>(step) > a->step_max)
                    a->step_max = static_cast<float>(step);
            }
            a->n++;
            a->sum += o.rate_mps;
            a->last = o.rate_mps;
            a->lock = o.lock_ms;
            const float r = static_cast<float>(o.rate_mps);
            if (r < a->lo) a->lo = r;
            if (r > a->hi) a->hi = r;
        }
    }

    // The grid epoch, noted last: its Dopplers become their windows' means by
    // the rule above, in place. Returns how many were averaged (the epoch then
    // goes with means: SdrEpoch::rr_mean). The window starts over after it.
    int close(gnss_raw::RawEpoch& e, SdrDopplerMeanStats& st)
    {
        if (!on())
            return 0;
        int usable_n = 0;
        for (int i = 0; i < e.n; ++i)
            if (usable(e.obs[i]))
                usable_n++;
        int n = 0;
        if (usable_n > 0)
        {
            st.epochs++;
            st.dopplers += static_cast<uint32_t>(usable_n);
            if (broken_ || epochs_ != cfg_.n)
            {
                st.epochs_short++;
                st.alone_epoch += static_cast<uint32_t>(usable_n);
            }
            else
            {
                for (int i = 0; i < e.n; ++i)
                    if (usable(e.obs[i]) && whyOf(e.obs[i]) == kOk)
                        n++;
                if (2 * n < usable_n)
                {
                    st.epochs_few++;
                    st.alone_epoch += static_cast<uint32_t>(usable_n);
                    n = 0;
                }
                else
                {
                    for (int i = 0; i < e.n; ++i)
                    {
                        gnss_raw::RawObs& o = e.obs[i];
                        if (!usable(o))
                            continue;
                        const Acc* a = find(o.sys, o.prn);
                        switch (whyOf(o))
                        {
                        case kOk:
                            o.rate_mps = a->sum / static_cast<double>(a->n);
                            st.means++;
                            if (a->step_max > st.step_max_mps) st.step_max_mps = a->step_max;
                            if (a->hi - a->lo > st.spread_max_mps) st.spread_max_mps = a->hi - a->lo;
                            continue;
                        case kLock: st.alone_lock++; break;
                        case kStep: st.alone_step++; break;
                        default: st.alone_missing++; break;
                        }
                        o.has_rate = false;   // its code and carrier stay
                    }
                }
            }
        }
        restart();
        return n;
    }

    // The window starts over (the epochs after a grid epoch, or after the
    // stream broke off).
    void restart()
    {
        epochs_ = 0;
        nacc_ = 0;
        broken_ = false;
    }

private:
    enum : uint8_t { kOk, kMissing, kLock, kStep };
    struct Acc
    {
        uint8_t  sys = 0, prn = 0;
        uint8_t  why = kOk;     // the first rule it failed in this window
        uint16_t n = 0;         // epochs it had a usable Doppler in
        uint32_t lock = 0;      // its last lock time
        double   sum = 0.0;     // of its Dopplers
        double   last = 0.0;    // its last Doppler
        float    lo = 0.0f, hi = 0.0f;   // their range
        float    step_max = 0.0f;
    };
    bool usable(const gnss_raw::RawObs& o) const
    {
        return o.has_rate && std::isfinite(o.rate_mps) &&
               !(o.sys < GNSS_SYS_COUNT && o.cn0_dbhz < cfg_.cn0_min[o.sys]);
    }
    Acc* find(uint8_t sys, uint8_t prn)
    {
        for (int i = 0; i < nacc_; ++i)
            if (acc_[i].sys == sys && acc_[i].prn == prn)
                return &acc_[i];
        return nullptr;
    }
    uint8_t whyOf(const gnss_raw::RawObs& o)
    {
        const Acc* a = find(o.sys, o.prn);
        if (a == nullptr || a->n != epochs_)
            return kMissing;
        return a->why;
    }

    SdrDopplerMeanConfig cfg_;
    uint16_t epochs_ = 0;      // epochs noted in this window
    bool     broken_ = false;  // two of them not period_ms apart
    int64_t  last_t_ms_ = 0;
    int      nacc_ = 0;
    Acc      acc_[gnss_raw::RawEpoch::kMaxObs];
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
        SdrDopplerMeanConfig mean;     // the Dopplers between the grid epochs averaged in (n 0: off)
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
        uint32_t marked_pr = 0;        // pseudoranges left out by a mark (every epoch, on the grid or not)
        uint32_t marked_rate = 0;      // Dopplers left out by a mark (the same)
        uint32_t full = 0;             // cells past RawEpoch::kMaxObs
    };

    SdrEpochAssembler() = default;
    explicit SdrEpochAssembler(const Config& cfg) { setConfig(cfg); }
    void setConfig(const Config& cfg)
    {
        cfg_ = cfg;
        mean_.setConfig(cfg.mean);
    }
    const Config& config() const { return cfg_; }
    const Counts& counts() const { return n_; }
    const SdrDopplerMeanStats& meanStats() const { return mean_st_; }

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
        if (raw_cells_ < rec.nobs)
            n_.short_epochs++;
        // the board's marks, on every epoch: what they take out of one is
        // missing from a Doppler window too
        if (rec.has_tail)
            applyMarks(rec, ep_);
        const int64_t t_ms = static_cast<int64_t>(rec.week) * 604800000LL + rec.tow_ms;
        mean_.note(ep_, t_ms);
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
        // the Dopplers since the last grid epoch (SdrDopplerMean; none off)
        out.rr_mean = mean_.close(e, mean_st_) > 0 ? 1 : 0;
        return true;
    }

private:
    void applyMarks(const TrEpoch& rec, gnss_raw::RawEpoch& e)
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

    void onMsm(const uint8_t* frame, size_t len)
    {
        n_.msm++;
        rtcm3::MsmHeader h;
        rtcm3::MsmCell* const cells = cells_;
        size_t dropped = 0;
        if (len < 6 || !rtcm3::decodeMsm7(frame + 3, len - 6, h, cells, RTCM_MSM_CAP, &dropped))
            return;
        if (h.gps_tow_ms == rtcm3::kInvalidTow)
            return;
        if (have_ && h.gps_tow_ms != tow_ms_)
        {
            n_.no_record++;          // the last epoch's record never came
            have_ = false;
            mean_.restart();         // and its Dopplers are missing from the window: it starts over
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
    rtcm3::MsmCell cells_[RTCM_MSM_CAP];   // one MSM7's (2 kB): a member, off the caller's stack
    SdrDopplerMean mean_;
    SdrDopplerMeanStats mean_st_;
};

}  // namespace sdr_rtcm

#endif  // SDR_RAW_EPOCH_H
