// mosaic_feed.h -- the mosaic-G5's SBF blocks, offline, as the V8/V9 raw-GNSS
// filter is handed them (raw_replay --receiver mosaic).
//
// What the mosaic driver and the GNSS task do with the receiver's stream on
// the mosaic-only build, with the firmware's own pure decoders
// (TR_GNSSReceiverMosaic_Serial: SbfMeas, SbfNav, MosaicRawEpoch, SbfParser),
// so a replay runs the flight code from the bytes the OC logged (0x95):
//   - MeasEpoch: every one's CumClkJumps followed (MosaicJumpState); those on
//     the fused grid (onFuseGrid at the profile's fuse_epoch_period_s) made a
//     RawEpoch (rawEpochFromMeasEpoch), an empty one not handed over. Its GPS
//     time is its receiver time less the receiver's clock bias: the last
//     PVTGeodetic that anchors the jump count (pvtAnchorsJumps), carried by
//     its drift and the jumps since (dtrMs), while it is no more than a
//     second old and the count has not started over; otherwise the bias is
//     taken as 0 (up to 0.5 ms off) and counted.
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
//
// The glue mirrors the driver's (TR_GNSSReceiverMosaic_Serial.cpp,
// rawMeasEpoch/rawPvt/rawTimeTagOk: the 1 s carry, the start-over check, the
// PVT's clock sanity, the tag's FINETIME and 120 s pairing age) and the GNSS
// task's (main.cpp, rawGnssMeasTask).
// Header-only and pure (tests_cpp/test_raw_replay_mosaic.cpp).
#ifndef RAW_REPLAY_MOSAIC_FEED_H
#define RAW_REPLAY_MOSAIC_FEED_H

#include <cmath>
#include <cstdint>
#include <cstring>

#include <RocketComputerTypes.h>

#include "MosaicRawEpoch.h"
#include "RawMeas.h"
#include "SbfMeas.h"
#include "SbfNav.h"
#include "SbfParser.h"
#include "ShadowLink.h"
#include "gnss/eph.h"

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
    };
    int first_rt = -1;              // the log's first ReceiverTime: -1 none, else its FINETIME (0 or 1)
    bool from_boot = false;
    bool assume_finetime = false;
    int64_t pair_us = INT64_MIN;    // the last pairing (MCU us); before the log's first, that first (unless from_boot)

    // have_time/finetime: the feed's last ReceiverTime so far (MosaicFeed::haveTime, finetime)
    Why judge(bool have_time, bool finetime, bool line_valid, int64_t now_us) const
    {
        const bool fine = have_time ? finetime : (first_rt < 0 ? assume_finetime : (!from_boot && first_rt == 1));
        if (!fine)
            return Why::NOT_FINETIME;
        if (pair_us == INT64_MIN || now_us - pair_us >= kPpsPairMaxAgeUs)
            return Why::PPS_STALE;
        if (!line_valid)
            return Why::NO_LINE;
        return Why::TIMED;
    }
};

class MosaicFeed
{
public:
    struct Config
    {
        uint32_t fuse_period_ms = 100;   // the profile's fuse_epoch_period_s: epochs on this GPS-time grid
        double   pvt_max_age_s = 1.0;    // a PVT's clock bias is carried at most this long
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
        uint32_t dtr_anchored = 0, dtr_zero = 0;   // epochs tagged with a PVT's bias, or with none
        uint32_t jumps = 0, rebases = 0;
        uint32_t pvt = 0, pvt_fix = 0, pvt_anchor = 0;
        uint32_t nav = 0, nav_ok = 0, nav_dropped = 0, nav_bad = 0;
        uint32_t ion = 0, ion_applied = 0;
    };

    MosaicFeed() : MosaicFeed(Config{}) {}
    explicit MosaicFeed(const Config& c) : cfg_(c) {}

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
            if (mosaic::rawEpochFromMeasEpoch(b, len, te.epoch, js_, info_))
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
            if (!mosaic::noteMeasEpochClock(b, len, js_, info_))
                n_.meas_bad++;
            else
                n_.off_grid++;
            k = Kind::EPOCH_SKIPPED;
        }
        if (info_.status == mosaic::SbfStatus::OK && info_.wnc != mosaic::kWncDnu)
            last_wnc_ = info_.wnc;
        n_.jumps += js_.jumps - j0;
        n_.rebases += js_.rebases - r0;
        return k;
    }

    // The epoch's GPS time: its receiver time less the bias of the last
    // anchoring PVT, carried by its drift and the jumps since.
    void tag()
    {
        const int64_t rx_ms = static_cast<int64_t>(info_.wnc) * 604800000LL + info_.tow_ms;
        const double dt_s = (rx_ms - anc_rx_ms_) * 1.0e-3;
        anchored_ = anc_valid_ && js_.rebases == anc_rebases_ && dt_s >= 0.0 && dt_s <= cfg_.pvt_max_age_s;
        dtr_ms_ = anchored_ ? mosaic::dtrMs(anc_bias_ms_, anc_drift_ppm_, dt_s, info_.j_ms - anc_j_ms_) : 0.0;
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
        // a solution's clock, as the driver takes it: RxClkBias is not in has_solution's do-not-use test
        if (pvt_.has_solution && std::isfinite(pvt_.rx_clk_bias_ms) && std::fabs(pvt_.rx_clk_bias_ms) <= 1000.0 &&
            std::isfinite(pvt_.rx_clk_drift_ppm) && std::fabs(pvt_.rx_clk_drift_ppm) <= 1000.0f)
        {
            // only a PVT whose own MeasEpoch was the last noted holds the jump count it states its bias at
            if (mosaic::pvtAnchorsJumps(js_, pvt_.wnc, pvt_.tow_ms))
            {
                anc_valid_ = true;
                anc_bias_ms_ = pvt_.rx_clk_bias_ms;
                anc_drift_ppm_ = pvt_.rx_clk_drift_ppm;
                anc_rx_ms_ = static_cast<int64_t>(pvt_.wnc) * 604800000LL + pvt_.tow_ms;
                anc_j_ms_ = js_.j_ms;
                anc_rebases_ = js_.rebases;
                n_.pvt_anchor++;
            }
        }
        return Kind::PVT;
    }

    Config cfg_;
    Counts n_;
    mosaic::MosaicJumpState js_;
    mosaic::MosaicEpochInfo info_;
    uint16_t last_wnc_ = mosaic::kWncDnu;
    // the anchoring PVT
    bool anc_valid_ = false;
    double anc_bias_ms_ = 0.0;
    double anc_drift_ppm_ = 0.0;
    int64_t anc_rx_ms_ = 0;
    int32_t anc_j_ms_ = 0;
    uint32_t anc_rebases_ = 0;
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
};

}  // namespace rr

#endif  // RAW_REPLAY_MOSAIC_FEED_H
