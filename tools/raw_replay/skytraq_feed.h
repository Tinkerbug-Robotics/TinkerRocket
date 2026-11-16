// skytraq_feed.h -- the SkyTraq PX1105R's binary stream, offline, as the V9
// PX1105R image's raw-GNSS filter is handed it (raw_replay --receiver px1105r).
//
// What the PX1105R driver (TR_GNSSReceiverPX1105R_Serial.cpp: onFrame, rawE5,
// rawDf, rawNav) and the GNSS task (main.cpp rawGnssMeasTask, rawIonoOffer) do
// with each checksum-valid body, with the firmware's own pure code unchanged
// (TR_GNSSReceiverPX1105R_Serial: SkyTraqBinary, SkyTraqNav, Px1105rRawEpoch.h;
// the LC86G driver's GnssTimeSync; the mosaic's Doppler pairs, MosaicRawEpoch.h):
//   - 0xE5: decoded (decodeRawE5); its GPS instant the receiver's time of week
//     less the last 0xDF's clock bias carried by its drift (biasAt,
//     epochGpsS), which with the frame's arrival is a sample of the MCU <-> GPS
//     line (lc86::GnssTimeSync at the profile's epoch_latency_s: no 1PPS on
//     this carrier). On the fused grid (TOW % fuse period == 0) the cells kept
//     (rawEpochFromE5: GPS L1 C/A, Galileo E1, BeiDou B1I MEO/IGSO) are an
//     epoch, its Dopplers averaged with the off-grid epoch's before it
//     (averageDopplerPairs, none across a clock cut), tagged on the line;
//     off the grid only its Dopplers are kept (dopplerSetFromRawEpoch).
//   - 0xDF: the receiver's clock (noteRcvState: a step cuts every lock there)
//     and its fix (GNSSData as gnssDataFromRcvState builds it: the height
//     above the ellipsoid, no geoid).
//   - 0xE7: the satellites its fix used.
//   - 0xE0 / 0xE2 / 0xE6: filed (NavAssembler::add with the receiver's week
//     from its clock, 0 until a 0xDF), each completed ephemeris handed over,
//     GPS ones held for the week (takePending) once it is known; the GPS
//     Klobuchar page offered to the GNSS task's one ionosphere rule as the
//     receiver's (mosaic_feed.h IonoRule: the firmware's raw_iono_rule.h).
// Header-only and pure; the caller turns EPOCH into the GNSS task's work, NAV
// into the stage's addEphemeris, ION into the stage's and the filter's
// ionosphere, FIX into the flight loop's fix.
#ifndef RAW_REPLAY_SKYTRAQ_FEED_H
#define RAW_REPLAY_SKYTRAQ_FEED_H

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <RocketComputerTypes.h>

#include "GnssTimeSync.h"
#include "MosaicRawEpoch.h"
#include "Px1105rRawEpoch.h"
#include "RawMeas.h"
#include "ShadowLink.h"
#include "SkyTraqBinary.h"
#include "SkyTraqNav.h"
#include "gnss/eph.h"
#include "mosaic_feed.h"

namespace rr
{

class SkyTraqFeed
{
public:
    struct Config
    {
        uint32_t fuse_period_ms = 100;   // the profile's fuse_epoch_period_s
        double   epoch_latency_s = 0.0;  // the profile's epoch_latency_s (the driver's setEpochLatencyS)
        bool     pair = false;           // the Dopplers in pairs (the profile's rr_pair_avg)
        mosaic::MosaicDopplerPairConfig pair_cfg;
    };

    enum class Kind : uint8_t
    {
        OTHER,
        EPOCH,           // te holds a fused-grid epoch, tagged (mcu_us) where the driver would tag it
        EPOCH_SKIPPED,   // off the grid (its Dopplers kept), empty, or not decoded
        NAV,             // ephs() holds one or more completed ephemerides
        ION,             // iono() holds a set the rule applies (ephs() may hold ephemerides too)
        FIX,             // fix() / gnss() hold the 0xDF just decoded (any navigation state)
    };

    struct Counts
    {
        uint32_t frames = 0, e5 = 0, e5_bad = 0, epochs = 0, decimated = 0, empty = 0;
        uint32_t timed = 0, untimed_nobias = 0, untimed_noline = 0;
        uint32_t df = 0, df_fix = 0, e7 = 0;
        uint32_t nav = 0, eph = 0, eph_pending = 0, nav_refused = 0, ion_pages = 0, ion_applied = 0;
        uint32_t cells_kept = 0, cells_listed = 0;
    };

    SkyTraqFeed() : SkyTraqFeed(Config{}) {}
    explicit SkyTraqFeed(const Config& c) : cfg_(c)
    {
        lc86::GnssTimeSync::Config ts = time_sync_.config();
        ts.latency_s = c.epoch_latency_s;
        time_sync_.setConfig(ts);
        nav_.reset();
    }

    // One checksum-valid body (from its message ID on) whose frame ended at end_us.
    Kind onBody(const uint8_t* body, size_t len, int64_t end_us, gnss_raw::TimedRawEpoch& te)
    {
        if (len < 1)
            return Kind::OTHER;
        n_.frames++;
        ephs_.clear();
        switch (body[0])
        {
        case skytraq::MSG_EXT_RAW:
            n_.e5++;
            return onE5(body, len, end_us, te);
        case skytraq::MSG_RCV_STATE:
        {
            if (!skytraq::decodeRcvState(body, len, rs_))
                return Kind::OTHER;
            n_.df++;
            const bool cut = px1105r::noteRcvState(rs_, clk_);
            if (cut)
            {
                lock_.have_cut = true;
                lock_.cut_ms = clk_.rx_ms;
            }
            if (rs_.nav_state >= skytraq::kNavFix2d)
                n_.df_fix++;
            gnssFromRcvState(rs_, have_channels_ ? channels_.used : 0, gnss_);
            gnss_.time_us = static_cast<uint32_t>(end_us);
            // the fix's own GPS instant on the line (rawDf's TimedFix), for the caller
            fix_gps_s_ = rs_.week * 604800.0 + rs_.tow_s;
            int64_t mcu = 0;
            fix_mcu_us_ = time_sync_.valid() && time_sync_.mcuAt(fix_gps_s_, mcu) ? mcu : 0;
            return Kind::FIX;
        }
        case skytraq::MSG_SV_CH_STATUS:
            if (skytraq::decodeSvChStatus(body, len, channels_))
            {
                have_channels_ = true;
                n_.e7++;
            }
            return Kind::OTHER;
        case skytraq::MSG_GPS_SUBFRAME:
        case skytraq::MSG_BDS_D1:
        case skytraq::MSG_GAL_PAGE:
            return onNav(body, len);
        default:
            return Kind::OTHER;
        }
    }

    // An ionosphere set from elsewhere (the app's), through the same rule.
    bool offerAppIono(const gnss_raw::ShadowIono& io) { return iono_rule_.offer(io, false); }

    const std::vector<gps_eph_t>& ephs() const { return ephs_; }
    const gnss_raw::ShadowIono& iono() const { return iono_; }
    const skytraq::RcvState& fix() const { return rs_; }
    const GNSSData& gnss() const { return gnss_; }
    double fixGpsS() const { return fix_gps_s_; }
    int64_t fixMcuUs() const { return fix_mcu_us_; }
    // the last EPOCH's GPS instant (continuous s), the bias taken off it (ms), whether a 0xDF's bias placed it
    double gpsS() const { return gps_s_; }
    double dtrMs() const { return dtr_ms_; }
    bool biased() const { return biased_; }
    const skytraq::RawE5& e5() const { return e5_; }
    const px1105r::Px1105rClock& clock() const { return clk_; }
    const px1105r::Px1105rLockState& lock() const { return lock_; }
    const lc86::GnssTimeSync& timeSync() const { return time_sync_; }
    const skytraq::NavAssembler& nav() const { return nav_; }
    const Counts& counts() const { return n_; }
    const Config& config() const { return cfg_; }
    const mosaic::MosaicDopplerPairStats& pairStats() const { return pair_st_; }
    uint64_t lastPaired() const { return pair_mask_; }

private:
    // rawE5: the time sync's sample, then the grid's cut, the pairs and the tag
    Kind onE5(const uint8_t* body, size_t len, int64_t end_us, gnss_raw::TimedRawEpoch& te)
    {
        if (!skytraq::decodeRawE5(body, len, e5_))
        {
            n_.e5_bad++;
            return Kind::EPOCH_SKIPPED;
        }
        const int64_t rx_ms = px1105r::rxMs(e5_.week, e5_.tow_ms);
        double dtr = 0.0;
        const bool biased = px1105r::biasAt(clk_, rx_ms, dtr);
        const double gps_s = px1105r::epochGpsS(e5_.week, e5_.tow_ms, dtr);
        if (biased)
            time_sync_.addSample(gps_s, end_us);
        const bool on_grid = px1105r::onFuseGrid(e5_.tow_ms, cfg_.fuse_period_ms);
        te = gnss_raw::TimedRawEpoch();
        const int cells = px1105r::rawEpochFromE5(e5_, lock_, te.epoch);
        n_.cells_listed += e5_.n_listed;
        if (!on_grid)
        {
            if (cfg_.pair)
                mosaic::dopplerSetFromRawEpoch(te.epoch, rx_ms, prev_);
            n_.decimated++;
            return Kind::EPOCH_SKIPPED;
        }
        te.rr_paired = 0;
        pair_mask_ = 0;
        if (cfg_.pair)
        {
            // a clock jump between the two: no pairs (MosaicRawEpoch.h's rule)
            const bool cut = lock_.have_cut && lock_.cut_ms > rx_ms - static_cast<int64_t>(cfg_.pair_cfg.dt_ms) &&
                             lock_.cut_ms <= rx_ms;
            if (cells > 0)
            {
                const int n = mosaic::averageDopplerPairs(te.epoch, rx_ms, cut, prev_, cfg_.pair_cfg, pair_st_,
                                                          &pair_mask_);
                te.rr_paired = n > 0 ? 1 : 0;
            }
            prev_.valid = false;   // one grid epoch's mate only
        }
        if (cells == 0)
        {
            n_.empty++;
            return Kind::EPOCH_SKIPPED;
        }
        n_.cells_kept += static_cast<uint32_t>(cells);
        int64_t mcu = 0;
        const bool line_ok = time_sync_.valid();
        const bool timed = biased && line_ok && time_sync_.mcuAt(gps_s, mcu);
        if (!biased)
            n_.untimed_nobias++;
        else if (!timed)
            n_.untimed_noline++;
        else
            n_.timed++;
        te.arrival_us = end_us;
        te.gps_week = e5_.week;
        te.mcu_us = timed ? mcu : 0;
        te.tag_dtr_ms = timed ? static_cast<float>(dtr) : 0.0f;
        gps_s_ = gps_s;
        dtr_ms_ = dtr;
        biased_ = biased;
        n_.epochs++;
        return Kind::EPOCH;
    }

    // rawNav, then the GNSS task's ionosphere rule
    Kind onNav(const uint8_t* body, size_t len)
    {
        if (!skytraq::isNavMessage(body, len))
            return Kind::OTHER;
        n_.nav++;
        gps_eph_t eph;
        int8_t io[8];
        const int week = clk_.have ? static_cast<int>(clk_.rx_ms / 604800000LL) : 0;
        const skytraq::NavStatus st = nav_.add(body, len, week, eph, io);
        Kind k = Kind::OTHER;
        switch (st)
        {
        case skytraq::NavStatus::EPHEMERIS:
            ephs_.push_back(eph);
            n_.eph++;
            k = Kind::NAV;
            break;
        case skytraq::NavStatus::IONO:
        {
            n_.ion_pages++;
            const uint16_t day = nav_.ionoGpsDay();
            if (day != 0)
            {
                gnss_raw::ShadowIono cand{};
                std::memcpy(cand.raw, io, sizeof cand.raw);
                cand.gps_day = day;
                if (iono_rule_.offer(cand, true))
                {
                    iono_ = cand;
                    n_.ion_applied++;
                    k = Kind::ION;
                }
            }
            break;
        }
        case skytraq::NavStatus::PART:
        case skytraq::NavStatus::UNUSED:
        case skytraq::NavStatus::NO_WEEK:
            break;
        default:
            n_.nav_refused++;
            break;
        }
        if (week != 0)
        {
            while (nav_.takePending(week, eph) == skytraq::NavStatus::EPHEMERIS)
            {
                ephs_.push_back(eph);
                n_.eph++;
                n_.eph_pending++;
                if (k == Kind::OTHER)
                    k = Kind::NAV;
            }
        }
        return k;
    }

    // gnssDataFromRcvState (TR_GNSSReceiverPX1105R_Serial.cpp): the fix the flight loop gets; the height above the
    // ellipsoid. The UTC fields are left out (the harness does not read them).
    static void gnssFromRcvState(const skytraq::RcvState& s, uint8_t used, GNSSData& out)
    {
        std::memset(&out, 0, sizeof out);
        out.num_sats = used;
        out.h_acc_m = 255;
        out.v_acc_m = 255;
        switch (s.nav_state)
        {
        case skytraq::kNavFix2d: out.fix_mode = 2; break;
        case skytraq::kNavFix3d:
        case skytraq::kNavDiff: out.fix_mode = 3; break;
        case skytraq::kNavPrediction: out.fix_mode = 1; break;
        default: out.fix_mode = 0; break;
        }
        if (out.fix_mode < 2)
            return;
        double lat, lon, h;
        ecefToLla(s.ecef_m, lat, lon, h);
        out.lat_e7 = static_cast<int32_t>(std::lround(lat * (180.0 / M_PI) * 1.0e7));
        out.lon_e7 = static_cast<int32_t>(std::lround(lon * (180.0 / M_PI) * 1.0e7));
        out.alt_mm = static_cast<int32_t>(std::lround(h * 1000.0));
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const double vx = s.vel_mps[0], vy = s.vel_mps[1], vz = s.vel_mps[2];
        const double ve = -so * vx + co * vy;
        const double vn = -sl * co * vx - sl * so * vy + cl * vz;
        const double vu = cl * co * vx + cl * so * vy + sl * vz;
        out.vel_e_mmps = static_cast<int32_t>(std::lround(ve * 1000.0));
        out.vel_n_mmps = static_cast<int32_t>(std::lround(vn * 1000.0));
        out.vel_u_mmps = static_cast<int32_t>(std::lround(vu * 1000.0));
    }

public:
    // WGS-84 ECEF -> geodetic (rad, rad, m above the ellipsoid): the driver's ecefToLla (Bowring)
    static void ecefToLla(const double x[3], double& lat, double& lon, double& h)
    {
        constexpr double a = 6378137.0;
        constexpr double f = 1.0 / 298.257223563;
        constexpr double b = a * (1.0 - f);
        constexpr double e2 = f * (2.0 - f);
        constexpr double ep2 = e2 / (1.0 - e2);
        const double p = std::sqrt(x[0] * x[0] + x[1] * x[1]);
        lon = std::atan2(x[1], x[0]);
        const double th = std::atan2(x[2] * a, p * b);
        const double st = std::sin(th), ct = std::cos(th);
        lat = std::atan2(x[2] + ep2 * b * st * st * st, p - e2 * a * ct * ct * ct);
        const double sl = std::sin(lat);
        const double n = a / std::sqrt(1.0 - e2 * sl * sl);
        h = (std::fabs(std::cos(lat)) > 1e-9) ? p / std::cos(lat) - n : std::fabs(x[2]) - b;
    }

private:
    Config cfg_;
    Counts n_;
    skytraq::RawE5 e5_;
    skytraq::RcvState rs_;
    skytraq::ChannelSummary channels_;
    bool have_channels_ = false;
    px1105r::Px1105rLockState lock_;
    px1105r::Px1105rClock clk_;
    lc86::GnssTimeSync time_sync_;
    skytraq::NavAssembler nav_;
    IonoRule iono_rule_;
    gnss_raw::ShadowIono iono_ = {};
    std::vector<gps_eph_t> ephs_;
    GNSSData gnss_ = {};
    double fix_gps_s_ = 0.0;
    int64_t fix_mcu_us_ = 0;
    double gps_s_ = 0.0, dtr_ms_ = 0.0;
    bool biased_ = false;
    mosaic::MosaicDopplerSet prev_;
    mosaic::MosaicDopplerPairStats pair_st_;
    uint64_t pair_mask_ = 0;
};

}  // namespace rr

#endif  // RAW_REPLAY_SKYTRAQ_FEED_H
