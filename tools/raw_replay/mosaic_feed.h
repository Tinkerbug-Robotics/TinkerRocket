// mosaic_feed.h -- the mosaic-G5's SBF blocks, offline, as the V8/V9 raw-GNSS
// filter is handed them (raw_replay --receiver mosaic).
//
// What the mosaic driver and the GNSS task do with the receiver's stream on
// the mosaic-only build, with the firmware's own pure decoders
// (TR_GNSSReceiverMosaic_Serial: SbfMeas, SbfNav, MosaicRawEpoch, SbfParser),
// so a replay runs the flight code from the bytes the OC logged (0x95):
//   - MeasEpoch: every one's CumClkJumps followed (MosaicJumpState); those on
//     the fused grid (onFuseGrid at the profile's fuse_epoch_period_s) made a
//     RawEpoch (rawEpochFromMeasEpoch), an empty one not handed over. With the
//     Dopplers in pairs (Config::pair, the profile's rr_pair_avg), each one off
//     the grid read for its Dopplers (noteMeasEpochDopplers) and each grid
//     epoch's averaged with them (averageDopplerPairs) and marked in pairs or
//     not (TimedRawEpoch::rr_paired), as the driver does. Offline only
//     (Config::mb.on, raw_replay --rr-multiband 1): each satellite's other
//     bands combined into its cells' Dopplers at both ends of a pair
//     (MosaicMultiband.h rawEpochFromMeasEpochMb, noteMeasEpochDopplersMb),
//     each row's sigma ratio and wind-up wavelength kept for the harness
//     (mbRowRatio, mbRowLam), the gyro's wind-up given by the harness where
//     the referral may take it (setMbWindup); off, exactly the image's
//     cells. Its GPS
//     time is its receiver time less the receiver's clock bias: the last
//     PVTGeodetic that anchors the jump count (pvtAnchorsJumps), carried by
//     its drift and the jumps since (dtrMs), while it is no more than two
//     minutes old and the count has not started over; otherwise the epoch is
//     untimed and counted (MosaicTimeTag.h, the driver's own rule).
//   - GPSNav, GALNav, BDSNav: kept as the driver keeps them (sbfNavKeep: no
//     Galileo F/NAV, no BeiDou GEO), decoded (decodeSbfEphemeris) for the
//     stage's addEphemeris.
//   - GPSIon: the 8 Klobuchar bytes, applied by the GNSS task's one rule:
//     only a set of a day at least the current one's, never the same bytes
//     again, the receiver's winning a tie with the app's (IonoRule: the
//     firmware's raw_iono_rule.h where the tree has it).
//   - PVTGeodetic: the fix the flight loop gets (gnssDataFromPvt, with the
//     leap seconds of ReceiverTime and the PDOP of DOP), for the GPS apogee
//     voter and GnssAscentGate; and its geoid separation.
//   - ReceiverTime: the leap seconds and FINETIME.
// The driver's PPS pairing and the clock line are the log's own (0x9D, the
// harness's K lines: each is a pairing): the caller turns the GPS time into
// MCU time, where TimeTagGate (below) says the driver would have tagged it.
// The chain of anchors and its hold are the driver's MosaicTimeTag: the
// caller hands it each K line's edge against that line (notePair) and each
// whole second's placed tag (noteWholeSecond), and each PVT checks the
// second's tag against its edge. A log begins long after the boot, so its
// first chain stands for one confirmed before it (assumeConfirmedAtStart).
// A K line is stamped with its edge, but the driver took it when the block
// that paired the edge arrived (the first xPPSOffset or whole-second
// PVTGeodetic naming its second: pairingSecond), a few ms later and maybe
// after that second's MeasEpoch was placed: the caller holds each K line
// until that block, so the epochs between go on the line before it, as they
// did in flight.
//
// The glue mirrors the driver's (TR_GNSSReceiverMosaic_Serial.cpp,
// rawMeasEpoch/rawPvt/pairPps: the two-minute carry, the jump count with its
// arrivals, the PVT's clock sanity, the chain's breaks, its reference and the
// 1PPS's confirmation, the tag's FINETIME and 120 s pairing age) and the GNSS
// task's (main.cpp, rawGnssMeasTask). Not mirrored: a whole second whose
// epoch has no cell is not checked here (the driver checks it too), and the
// line's permit to step later (GnssTimeSync's may_step_later) is in the K
// lines already.
// Header-only and pure (tests_cpp/test_raw_replay_mosaic.cpp).
#ifndef RAW_REPLAY_MOSAIC_FEED_H
#define RAW_REPLAY_MOSAIC_FEED_H

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include <RocketComputerTypes.h>

#include "MosaicRawEpoch.h"
#include "MosaicTimeTag.h"
#include "RawMeas.h"
#include "SbfMeas.h"
#include "SbfNav.h"
#include "SbfParser.h"
#include "ShadowLink.h"
#include "gnss/eph.h"

// A tree with the Dopplers in pairs (MosaicRawEpoch.h averageDopplerPairs); raw_replay's RAW_REPLAY_DOPAVG says
#ifndef RAW_REPLAY_DOPAVG
#define RAW_REPLAY_DOPAVG 1
#endif
// A tree with the multi-band combiner (MosaicMultiband.h, harness only); raw_replay's RAW_REPLAY_MULTIBAND says
#ifndef RAW_REPLAY_MULTIBAND
#if RAW_REPLAY_DOPAVG && __has_include("MosaicMultiband.h")
#define RAW_REPLAY_MULTIBAND 1
#else
#define RAW_REPLAY_MULTIBAND 0
#endif
#endif
#if RAW_REPLAY_MULTIBAND
#include "MosaicMultiband.h"
#endif

#if __has_include("raw_iono_rule.h")
#include "raw_iono_rule.h"
#define RR_FIRMWARE_IONO_RULE 1
#else
#define RR_FIRMWARE_IONO_RULE 0
#endif

namespace rr
{

// The GNSS task's one ionosphere rule (plan section 2): a set is applied only
// when its day is at least the current one's and its bytes differ, and at the
// same day the receiver's wins over the app's. Where the tree has the
// firmware's own (flight_computer/main/raw_iono_rule.h, D1a), it is that one.
class IonoRule
{
public:
    // True if io should be applied now (and it becomes the current set).
    bool offer(const gnss_raw::ShadowIono& io, bool from_receiver)
    {
#if RR_FIRMWARE_IONO_RULE
        if (!RawIonoRule::offer(st_, io.raw, io.gps_day, from_receiver))
            return false;
#else
        if (have_)
        {
            if (std::memcmp(io.raw, cur_.raw, sizeof(io.raw)) == 0)
                return false;
            if (io.gps_day < cur_.gps_day)
                return false;
            if (io.gps_day == cur_.gps_day && cur_receiver_ && !from_receiver)
                return false;
        }
        have_ = true;
        cur_ = io;
        cur_receiver_ = from_receiver;
#endif
        applied_++;
        return true;
    }
    uint32_t applied() const { return applied_; }

private:
#if RR_FIRMWARE_IONO_RULE
    RawIonoRule::State st_;
#else
    bool have_ = false;
    bool cur_receiver_ = false;
    gnss_raw::ShadowIono cur_ = {};
#endif
    uint32_t applied_ = 0;
};

// The driver's time tag (rawTimeTagOk): an epoch gets its MCU time only with
// the clock line valid, the receiver's time FINETIME (its last ReceiverTime)
// and a 1PPS edge paired with its second within 120 s (the line drifts with
// the MCU crystal once the edges stop). A log begins long after the receiver
// did, so until its first ReceiverTime and its first pairing those first ones
// stand for the earlier, as the harness's clock line does; from_boot: nothing
// before them, as after a boot. A log with no ReceiverTime at all is left
// untimed, as the driver would leave it (assume_finetime: timed).
struct TimeTagGate
{
    static constexpr int64_t kPpsPairMaxAgeUs = 120000000;   // TR_GNSSReceiverMosaic_Serial.cpp
    enum class Why : uint8_t
    {
        TIMED,
        NOT_FINETIME,   // the receiver's time is not FINETIME (or not known to be)
        PPS_STALE,      // no 1PPS edge paired within 120 s
        NO_LINE,        // the clock line is not valid
        NO_BIAS,        // no PVT's bias carries to it (MosaicTimeTag)
        EDGE_OFF,       // the last edge sat off the line
        OFF_CHAIN,      // the last edge sat off the chain: the 1PPS and the PVTs state different GPS times
        HELD,           // the chain of PVT biases is not confirmed by the 1PPS yet
    };
    int first_rt = -1;              // the log's first ReceiverTime: -1 none, else its FINETIME (0 or 1)
    bool from_boot = false;
    bool assume_finetime = false;
    int64_t pair_us = INT64_MIN;    // the last pairing (MCU us); before the log's first, that first (unless from_boot)

    // have_time/finetime: the feed's last ReceiverTime so far (MosaicFeed::haveTime, finetime); anchored,
    // edge_on_line, held, on_chain: the feed's MosaicTimeTag (MosaicFeed::anchored, timeTag())
    Why judge(bool have_time, bool finetime, bool line_valid, int64_t now_us, bool anchored = true,
              bool edge_on_line = true, bool held = false, bool on_chain = true) const
    {
        const bool fine = have_time ? finetime : (first_rt < 0 ? assume_finetime : (!from_boot && first_rt == 1));
        if (!fine)
            return Why::NOT_FINETIME;
        if (pair_us == INT64_MIN || now_us - pair_us >= kPpsPairMaxAgeUs)
            return Why::PPS_STALE;
        if (!line_valid)
            return Why::NO_LINE;
        if (!anchored)
            return Why::NO_BIAS;
        if (!edge_on_line)
            return Why::EDGE_OFF;
        if (!on_chain)
            return Why::OFF_CHAIN;
        if (held)
            return Why::HELD;
        return Why::TIMED;
    }
};

class MosaicFeed
{
public:
    struct Config
    {
        uint32_t fuse_period_ms = 100;   // the profile's fuse_epoch_period_s: epochs on this GPS-time grid
        double   pvt_max_age_s = 120.0;  // a PVT's clock bias is carried at most this long (the driver's kPvtCarryMs)
#if RAW_REPLAY_DOPAVG
        bool     pair = false;           // the Dopplers in pairs (the driver's setRawDopplerPairing)
        mosaic::MosaicDopplerPairConfig pair_cfg;
#endif
#if RAW_REPLAY_MULTIBAND
        // each satellite's other bands combined into its row's Doppler (MosaicMultiband.h; off: as the image)
        mosaic::MosaicMultibandConfig mb;
#endif
    };

    enum class Kind : uint8_t
    {
        OTHER,
        EPOCH,           // te holds a fused-grid epoch: tag it with gpsS()
        EPOCH_SKIPPED,   // off the grid, empty, or refused: its clock jumps noted only
        NAV,             // eph() holds a decoded ephemeris
        NAV_SKIPPED,     // a nav block the driver does not keep (F/NAV, GEO) or that did not decode
        ION,             // iono() holds a set the rule applies
        PVT,             // gnss() holds the fix, pvt() the block
    };

    struct Counts
    {
        uint32_t meas = 0, meas_bad = 0, off_grid = 0, empty = 0, epochs = 0;
        uint32_t dtr_anchored = 0, dtr_zero = 0;   // epochs with a PVT's bias to carry, or with none (untimed)
        uint32_t jumps = 0, rebases = 0;
        uint32_t resets = 0, resolved = 0;   // of the rebases, the clock lost; jumps the arrivals placed past 256 ms
        uint32_t pvt = 0, pvt_fix = 0, pvt_anchor = 0;
        uint32_t nav = 0, nav_ok = 0, nav_dropped = 0, nav_bad = 0;
        uint32_t ion = 0, ion_applied = 0;
    };

    MosaicFeed() : MosaicFeed(Config{}) {}
    explicit MosaicFeed(const Config& c) : cfg_(c), tag_(tagConfig(c)) {}

    // The driver's MosaicTimeTag: its chain of anchors, the edges against the
    // line, the whole seconds' checks (see the top of this file).
    const mosaic::MosaicTimeTag& timeTag() const { return tag_; }
    void assumeConfirmedAtStart() { tag_.assumeConfirmedAtStart(); }
    // A 1PPS edge paired with GPS second gps_s (a K line: its time and the
    // line it moved), against that line's place for the second; line_drift
    // the line's d(MCU - GPS)/d(GPS), line_restarts its restarts so far.
    void notePair(int64_t gps_s, int64_t edge_us, bool line_valid, int64_t line_mcu_us, double line_drift,
                  uint32_t line_restarts)
    {
        tag_.notePair(gps_s, edge_us, line_valid, line_mcu_us, line_drift, line_restarts);
    }

    // The GPS second a block would pair with a 1PPS edge, as the driver's
    // onBlock names it: a whole-second PVTGeodetic, or an xPPSOffset on GPS
    // time (its TOW the pulse's, rounded). False for any other block.
    static bool pairingSecond(const uint8_t* b, size_t len, int64_t& gps_s)
    {
        mosaic::SbfHeader h;
        if (!mosaic::sbfHeader(b, len, h))
            return false;
        if (h.number == mosaic::SBF_PVT_GEODETIC)
        {
            if (h.tow_ms == mosaic::kTowDnu || h.wnc == mosaic::kWncDnu || h.tow_ms % 1000U != 0)
                return false;
            gps_s = static_cast<int64_t>(h.wnc) * 604800 + h.tow_ms / 1000U;
            return true;
        }
        if (h.number == mosaic::SBF_XPPS_OFFSET)
        {
            mosaic::XppsOffset x;
            if (!mosaic::decodeXppsOffset(b, len, x) || x.time_scale != 1 || x.tow_ms == mosaic::kTowDnu ||
                x.wnc == mosaic::kWncDnu)
                return false;
            gps_s = static_cast<int64_t>(x.wnc) * 604800 + (x.tow_ms + 500U) / 1000U;
            return true;
        }
        return false;
    }
    // The last EPOCH, if a whole second, placed at mcu_us (handed over timed
    // or not): its PPS check comes with its PVT.
    void noteWholeSecond(int64_t mcu_us, bool timed)
    {
        if (info_.tow_ms % 1000U == 0)
            tag_.noteWholeSecond(rxMs(), mcu_us, timed);
    }
    // The clock line's MCU length of a second (the check's late-edge test).
    void setLineSecondUs(int64_t us) { second_us_ = us; }

    // One CRC-checked SBF block (len its Length), arrived at t_arr_us.
    Kind onBlock(const uint8_t* b, size_t len, int64_t t_arr_us, gnss_raw::TimedRawEpoch& te)
    {
        mosaic::SbfHeader h;
        if (!mosaic::sbfHeader(b, len, h))
            return Kind::OTHER;
        switch (h.number)
        {
            case mosaic::SBF_MEAS_EPOCH: return onMeasEpoch(b, len, t_arr_us, te);
            case mosaic::SBF_PVT_GEODETIC: return onPvt(b, len, t_arr_us);
            case mosaic::SBF_RECEIVER_TIME:
            {
                mosaic::ReceiverTime rt;
                if (mosaic::decodeReceiverTime(b, len, rt))
                {
                    if (rt.delta_ls != -128)
                        leap_s_ = rt.delta_ls;
                    finetime_ = (rt.sync_level & 0x04) != 0;
                    have_time_ = true;
                    tag_.noteFineTime(finetime_);
                }
                return Kind::OTHER;
            }
            case mosaic::SBF_DOP:
            {
                mosaic::Dop d;
                if (mosaic::decodeDop(b, len, d))
                    pdop_x100_ = d.pdop_x100;
                return Kind::OTHER;
            }
            case mosaic::kSbfGpsIon:
            {
                n_.ion++;
                gnss_raw::ShadowIono io{};
                if (!mosaic::decodeGpsIon(b, len, io.raw, io.gps_day))
                    return Kind::OTHER;
                if (!iono_rule_.offer(io, true))
                    return Kind::OTHER;
                iono_ = io;
                n_.ion_applied++;
                return Kind::ION;
            }
            default:
                break;
        }
        if (mosaic::isSbfEphemerisBlock(h.number))
        {
            n_.nav++;
            // the driver's check before it queues a block, then the GNSS task's decode
            if (!mosaic::sbfNavKeep(b, len) || len > mosaic::kSbfNavMaxLen)
            {
                n_.nav_dropped++;
                return Kind::NAV_SKIPPED;
            }
            if (mosaic::decodeSbfEphemeris(b, len, last_wnc_, eph_) != mosaic::SbfEphStatus::OK)
            {
                n_.nav_bad++;
                return Kind::NAV_SKIPPED;
            }
            n_.nav_ok++;
            return Kind::NAV;
        }
        return Kind::OTHER;
    }

    // An ionosphere set from elsewhere (the app's: the pad context), through the same rule.
    bool offerAppIono(const gnss_raw::ShadowIono& io) { return iono_rule_.offer(io, false); }

    // The last EPOCH's GPS time (continuous s: week * 604800 + time of week),
    // its receiver clock bias (ms) and whether a PVT's bias placed it.
    double gpsS() const { return gps_s_; }
    double dtrMs() const { return dtr_ms_; }
    bool anchored() const { return anchored_; }
    int64_t rxMs() const { return static_cast<int64_t>(info_.wnc) * 604800000LL + info_.tow_ms; }
    const mosaic::MosaicEpochInfo& info() const { return info_; }
    const gps_eph_t& eph() const { return eph_; }
    const gnss_raw::ShadowIono& iono() const { return iono_; }
    const mosaic::PvtGeodetic& pvt() const { return pvt_; }
    const GNSSData& gnss() const { return gnss_; }
    // ReceiverTime: FINETIME (false until one is seen: haveTime())
    bool haveTime() const { return have_time_; }
    bool finetime() const { return finetime_; }
    uint16_t lastWnc() const { return last_wnc_; }
    const Counts& counts() const { return n_; }
    const Config& config() const { return cfg_; }
#if RAW_REPLAY_DOPAVG
    // the Dopplers in pairs: the counts so far, and which cells of the last EPOCH were averaged (bit i: cell i)
    const mosaic::MosaicDopplerPairStats& pairStats() const { return pair_st_; }
    uint64_t lastPaired() const { return pair_mask_; }
#endif
#if RAW_REPLAY_MULTIBAND
    // the multi-band combination (cfg.mb.on): its counts; the last combination's side outputs (mbLast: a grid
    // epoch's when mbLastGrid, else an epoch between's) and whether one ran since mbTake() last cleared it; and the
    // last EPOCH's rows as handed over, each its sigma ratio and wind-up wavelength with its pair's other end in
    // (mosaic::mbPairedRow)
    const mosaic::MosaicMultibandStats& mbStats() const { return mb_st_; }
    const mosaic::MosaicMbOut& mbLast() const { return mb_grid_last_ ? mb_out_ : mb_out_prev_; }
    bool mbLastGrid() const { return mb_grid_last_; }
    bool mbTake()
    {
        const bool f = mb_fresh_;
        mb_fresh_ = false;
        return f;
    }
    const mosaic::MosaicMbOut& mbGrid() const { return mb_out_; }
    // the gyro's wind-up for the next MeasEpoch (cfg.mb.w_src GYRO, AUTO), as the driver would read the IMU path's
    // last roll rate when it decodes one; it stands until set again
    void setMbWindup(const mosaic::MosaicMbWindupIn& w) { mb_wext_ = w; }
    double mbRowRatio(int i) const { return i >= 0 && i < gnss_raw::RawEpoch::kMaxObs ? mb_ratio_[i] : 1.0; }
    double mbRowLam(int i) const { return i >= 0 && i < gnss_raw::RawEpoch::kMaxObs ? mb_lam_[i] : 0.0; }
#endif

private:
    Kind onMeasEpoch(const uint8_t* b, size_t len, int64_t t_arr_us, gnss_raw::TimedRawEpoch& te)
    {
        n_.meas++;
        const uint32_t j0 = js_.jumps, r0 = js_.rebases;
        mosaic::SbfMeasReader rd;
        const bool header_ok = rd.begin(b, len) == mosaic::SbfStatus::OK;
        const uint32_t tow = rd.header().tow_ms;
        Kind k;
        if (header_ok && mosaic::onFuseGrid(tow, cfg_.fuse_period_ms))
        {
            te = gnss_raw::TimedRawEpoch();
#if RAW_REPLAY_MULTIBAND
            // with the bands combined (MosaicMultiband.h), else exactly the image's cells
            bool built;
            if (cfg_.mb.on)
            {
                built = mosaic::rawEpochFromMeasEpochMb(b, len, te.epoch, js_, info_, cfg_.mb, *mb_scr_, mb_st_,
                                                        mb_out_, t_arr_us, mb_wext_);
                mb_fresh_ = info_.status == mosaic::SbfStatus::OK;
                mb_grid_last_ = true;
            }
            else
            {
                built = mosaic::rawEpochFromMeasEpoch(b, len, te.epoch, js_, info_, t_arr_us);
            }
#else
            const bool built = mosaic::rawEpochFromMeasEpoch(b, len, te.epoch, js_, info_, t_arr_us);
#endif
#if RAW_REPLAY_DOPAVG
#if RAW_REPLAY_MULTIBAND
            // the epoch between's side outputs, before the pairing spends it
            const bool mb_prev_valid = cfg_.mb.on && pair_prev_.valid;
#endif
            if (cfg_.pair)
            {
                // as the driver's rawMeasEpoch: with the epoch before, then that one is spent; the epoch
                // handed over in pairs or as it came, and marked so
                pair_mask_ = 0;
                if (built && info_.status == mosaic::SbfStatus::OK)
                    te.rr_paired = mosaic::averageDopplerPairs(te.epoch, rxMs(), info_.jumped || info_.rebased,
                                                               pair_prev_, cfg_.pair_cfg, pair_st_, &pair_mask_) > 0
                                       ? 1 : 0;
                pair_prev_.valid = false;
            }
#if RAW_REPLAY_MULTIBAND
            if (cfg_.mb.on && built)
                mbRows(te.epoch, mb_prev_valid);
#endif
#elif RAW_REPLAY_MULTIBAND
            if (cfg_.mb.on && built)
                mbRows(te.epoch, false);
#endif
            if (built)
            {
                te.gps_week = info_.wnc;
                te.arrival_us = t_arr_us;
                tag();
                n_.epochs++;
                k = Kind::EPOCH;
            }
            else
            {
                if (info_.status == mosaic::SbfStatus::OK)
                    n_.empty++;
                else
                    n_.meas_bad++;
                k = Kind::EPOCH_SKIPPED;
            }
        }
        else
        {
#if RAW_REPLAY_MULTIBAND
            bool noted;
            if (cfg_.pair && cfg_.mb.on)
            {
                noted = mosaic::noteMeasEpochDopplersMb(b, len, js_, info_, pair_prev_, cfg_.mb, *mb_scr_, mb_st_,
                                                        mb_out_prev_, t_arr_us, mb_wext_);
                mb_fresh_ = noted;
                mb_grid_last_ = false;
            }
            else
            {
                noted = cfg_.pair ? mosaic::noteMeasEpochDopplers(b, len, js_, info_, pair_prev_, t_arr_us)
                                  : mosaic::noteMeasEpochClock(b, len, js_, info_, t_arr_us);
            }
#elif RAW_REPLAY_DOPAVG
            const bool noted = cfg_.pair ? mosaic::noteMeasEpochDopplers(b, len, js_, info_, pair_prev_, t_arr_us)
                                         : mosaic::noteMeasEpochClock(b, len, js_, info_, t_arr_us);
#else
            const bool noted = mosaic::noteMeasEpochClock(b, len, js_, info_, t_arr_us);
#endif
            if (!noted)
                n_.meas_bad++;
            else
                n_.off_grid++;
            k = Kind::EPOCH_SKIPPED;
        }
        if (info_.status == mosaic::SbfStatus::OK && info_.wnc != mosaic::kWncDnu)
            last_wnc_ = info_.wnc;
        n_.jumps += js_.jumps - j0;
        n_.rebases += js_.rebases - r0;
        n_.resets = js_.resets;
        n_.resolved = js_.resolved;
        return k;
    }

#if RAW_REPLAY_MULTIBAND
    // each row of the grid epoch as handed over: its sigma ratio and wind-up wavelength, with the epoch between's
    // end where the row was averaged with it (pair_mask_)
    void mbRows(const gnss_raw::RawEpoch& e, bool prev_valid)
    {
        for (int i = 0; i < e.n && i < gnss_raw::RawEpoch::kMaxObs; ++i)
        {
            int j = -1;
#if RAW_REPLAY_DOPAVG
            if (prev_valid && ((pair_mask_ >> i) & 1u))
                for (int k = 0; k < mb_out_prev_.n && j < 0; ++k)
                    if (mb_out_prev_.sys[k] == e.obs[i].sys && mb_out_prev_.prn[k] == e.obs[i].prn)
                        j = k;
#else
            (void)prev_valid;
#endif
            mosaic::mbPairedRow(mb_out_, i, mb_out_prev_, j, mb_ratio_[i], mb_lam_[i]);
        }
    }
#endif

    static mosaic::MosaicTimeTag::Config tagConfig(const Config& c)
    {
        mosaic::MosaicTimeTag::Config t;
        t.carry_ms = static_cast<int64_t>(std::llround(c.pvt_max_age_s * 1000.0));
        return t;
    }

    // The epoch's GPS time: its receiver time less the bias of the last
    // anchoring PVT, carried by its drift and the jumps since; none (the
    // epoch untimed) without one.
    void tag()
    {
        dtr_ms_ = 0.0;
        anchored_ = tag_.carry(info_, js_, dtr_ms_);
        if (anchored_)
            n_.dtr_anchored++;
        else
            n_.dtr_zero++;
        gps_s_ = mosaic::epochGpsS(info_.wnc, info_.tow_ms, dtr_ms_);
    }

    Kind onPvt(const uint8_t* b, size_t len, int64_t t_arr_us)
    {
        if (!mosaic::decodePvtGeodetic(b, len, pvt_))
            return Kind::OTHER;
        n_.pvt++;
        mosaic::gnssDataFromPvt(pvt_, leap_s_, pdop_x100_, gnss_);
        gnss_.time_us = static_cast<uint32_t>(t_arr_us);   // stamped on arrival, as the driver does
        if (pvt_.has_solution)
            n_.pvt_fix++;
        // a solution's clock, as the driver takes it: RxClkBias is not in has_solution's do-not-use test; only a
        // PVT whose own MeasEpoch was the last noted holds the jump count it states its bias at (counted here; the
        // driver's MosaicTimeTag decides, and checks its second's tag against the edge first)
        if (pvt_.has_solution && std::isfinite(pvt_.rx_clk_bias_ms) &&
            std::fabs(pvt_.rx_clk_bias_ms) <= mosaic::MosaicTimeTag::kMaxBiasMs &&
            std::isfinite(pvt_.rx_clk_drift_ppm) && std::fabs(pvt_.rx_clk_drift_ppm) <= 1000.0f &&
            mosaic::pvtAnchorsJumps(js_, pvt_.wnc, pvt_.tow_ms))
            n_.pvt_anchor++;
        tag_.notePvt(pvt_, js_, second_us_, true);
        return Kind::PVT;
    }

    Config cfg_;
    Counts n_;
    mosaic::MosaicJumpState js_;
    mosaic::MosaicEpochInfo info_;
    uint16_t last_wnc_ = mosaic::kWncDnu;
    // the anchoring PVT, its chain, the edges and the checks (the driver's)
    mosaic::MosaicTimeTag tag_;
    int64_t second_us_ = 1000000;
    // the last epoch's tag
    double gps_s_ = 0.0, dtr_ms_ = 0.0;
    bool anchored_ = false;
    // the other blocks
    gps_eph_t eph_ = {};
    gnss_raw::ShadowIono iono_ = {};
    IonoRule iono_rule_;
    mosaic::PvtGeodetic pvt_;
    GNSSData gnss_ = {};
    int leap_s_ = -128;
    uint16_t pdop_x100_ = 0;
    bool have_time_ = false, finetime_ = false;
#if RAW_REPLAY_DOPAVG
    // the Dopplers in pairs: the last off-grid epoch's, the counts, the last EPOCH's averaged cells
    mosaic::MosaicDopplerSet pair_prev_;
    mosaic::MosaicDopplerPairStats pair_st_;
    uint64_t pair_mask_ = 0;
#endif
#if RAW_REPLAY_MULTIBAND
    // the multi-band combination: its scratch (on the heap), counts and the two ends' side outputs
    std::unique_ptr<mosaic::MosaicMbScratch> mb_scr_{new mosaic::MosaicMbScratch()};
    mosaic::MosaicMultibandStats mb_st_;
    mosaic::MosaicMbOut mb_out_, mb_out_prev_;
    bool mb_fresh_ = false, mb_grid_last_ = false;
    mosaic::MosaicMbWindupIn mb_wext_;
    double mb_ratio_[gnss_raw::RawEpoch::kMaxObs] = {};
    double mb_lam_[gnss_raw::RawEpoch::kMaxObs] = {};
#endif
};

}  // namespace rr

#endif  // RAW_REPLAY_MOSAIC_FEED_H
