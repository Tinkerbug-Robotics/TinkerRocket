#pragma once

#include <cmath>
#include <stddef.h>
#include <stdint.h>

#include <SdrAid.h>                // TR_GNSSReceiverSdr_Serial: TinkerRocket's message 4002
#include "sdr_phase_policy.h"      // the board's clock line, held by the phase sender

// What a flight image sends the gnss-sdr-dev board on J1 beside its flight
// phase (sdr_phase_policy.h): its filter's attitude (TinkerRocket's message
// 4002, sub-type 1) ten times a second, and its rests (sub-type 5), at the
// start and every 5 s while it stays still. Not the navigation state
// (sub-type 2) nor the geometry (sub-type 4): no configuration of where the
// board sits in the vehicle or of its antenna's lever arm exists yet.
//
// Why the rests: the board finds how it is mounted in the vehicle's body by
// pairing each one (this side's mean specific force over a still window,
// body FRD axes) with its own IMU's mean over the same window, which it
// places by the window's stamp (its middle) and length (dur_ms). The test of
// stillness is this side's own (RestDetector below), so the board needs no
// test of its own: a window is still when the specific force's mean is 1 g
// within 0.5 m/s^2, its samples' spread about that mean under 0.05 m/s^2 a
// axis (1-sigma) and the gyro under 3 deg/s throughout (raw: a bias of
// 0.4 deg/s is normal). Windows are back to back, 1 s each by the filter's
// IMU clock; the first still one goes, then every fifth still one in a row
// (every 5 s) while nothing moves, and the first still one after any window
// that was not (a new pose) at once. A window not still sends nothing, and a
// gap in the samples (a filter stopped: MAG_CALIBRATION, a reinit) counts as
// one, since it could hide a move.
//
// The attitude: body (FRD) to ECEF, q_en (x) q_nb, the filter's body-to-NED
// quaternion turned into ECEF at a place; the body's rate in body axes, the
// gyro less the filter's bias, rad/s; no 1-sigmas (the filters give tilt and
// heading ones only through their body-axis covariance: unknown here). The
// place is the board's own last valid fix (its 4001 sub-type 2), and only
// failing one a fix the filter navigates on: the board turns the attitude
// back into its own NED with its own fix (rx_fc.c: C_bn = C_ne C_eb), so the
// board's fix makes that round trip exact, and with the board on J1 the
// filter navigates on no fix at all. It flies GNSS-absent (gnssTypeGivesFix)
// from a degraded start at 0 N 0 E, a pad-relative origin, where q_en would
// put the attitude tens of degrees off. With no place the attitude goes as
// not valid (its rate still does). The stamp is the filter's state's: the
// IMU sample of the update the quaternion came from. A filter that stopped
// updating (none for kAttFreshUs: MAG_CALIBRATION, a reinit, before its
// start) sends not valid, stamped at the pass, so the board drops what it
// holds at once rather than after its own 350 ms hold.
//
// When: the attitude every 100 ms (sdr_aid::AidSchedule with the attitude
// alone: each due time steps by the period, one more than a period late
// restarts from now); a rest as soon as its window closes, after any
// attitude due on that pass. Nothing goes until the board's clock line is
// valid, the phase sender's held line (sdr_phase::Sender::lineUsable: the
// last valid one, for up to 600 s after it last moved), and each stamp's tq
// is that line's (5, 6 once it has not moved for 100 s). A rest that found no
// valid line or no room in the UART is given again on the next pass; one
// more than kRestDropUs past its window's end is dropped. src kSrcOwn (the
// filter runs on the vehicle's own IMU, barometer and magnetometer), test
// while a sim or a rig replay flies, as the phase.
//
// Pure: the caller (main.cpp sdrAidNoteUpdate and sdrAidStep) checks that
// the receiver on J1 is the board, hands RestDetector and Sender each filter
// update's input and state, hands Sender the board's valid fixes, and writes
// the frame Sender gives it (never waiting: one that found no room goes on a
// later pass), one 4002 frame on the line at a time with the phase's.
// Host-tested in tests_cpp/test_sdr_aid.cpp.

namespace sdr_aid_fc
{

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kDegToRad = kPi / 180.0;

// Hamilton product a (x) b, scalar first.
inline void quatMul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// NED to ECEF at geodetic lat, lon (rad), scalar first, v_ECEF = q v_NED q*:
// its matrix's columns are north (-sin lat cos lon, -sin lat sin lon,
// cos lat), east (-sin lon, cos lon, 0) and down (-cos lat cos lon,
// -cos lat sin lon, -sin lat) in ECEF. A turn of -(lat + 90 deg) about y
// (at 0 N 0 E north is +z, east +y, down -x), then of lon about z.
inline void nedToEcef(double lat, double lon, double q[4])
{
    const double a = 0.5 * lon;
    const double b = -0.5 * (lat + 0.5 * kPi);
    const double qz[4] = {std::cos(a), 0.0, 0.0, std::sin(a)};
    const double qy[4] = {std::cos(b), 0.0, std::sin(b), 0.0};
    quatMul(qz, qy, q);
}

// Body (FRD) to ECEF from body to NED at lat, lon: q_en (x) q_nb.
inline void bodyToEcef(const double q_nb[4], double lat, double lon, double q_eb[4])
{
    double q_en[4];
    nedToEcef(lat, lon, q_en);
    quatMul(q_en, q_nb, q_eb);
}

// The geodetic (WGS-84) latitude and longitude of an ECEF position, rad
// (Bowring's formula: well under 1e-9 rad within 10 km of the ellipsoid).
// False for a position that is not within 400 km of the Earth's surface, or
// not a number.
inline bool latLonOf(const double p[3], double& lat, double& lon)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = f * (2.0 - f);
    const double ep2 = e2 / (1.0 - e2);
    const double r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    if (!(r > 6.0e6 && r < 6.8e6))
        return false;
    const double s = std::sqrt(p[0] * p[0] + p[1] * p[1]);
    const double th = std::atan2(p[2] * a, s * b);
    const double st = std::sin(th);
    const double ct = std::cos(th);
    lat = std::atan2(p[2] + ep2 * b * st * st * st, s - e2 * a * ct * ct * ct);
    lon = std::atan2(p[1], p[0]);
    return true;
}

// A 32-bit MCU micros (time_us(), an IMU sample's time_us) as esp_timer's 64
// bits, given esp_timer now: the instant within +-35 min of now that it
// names (the clock line takes 64-bit MCU time).
inline int64_t widenUs(uint32_t t_us, int64_t now_us)
{
    return now_us - (int64_t)(int32_t)((uint32_t)now_us - t_us);
}

// ── At rest ───────────────────────────────────────────────────────────────

// A closed window. Each sample stands for the time since the one before it,
// so a window runs from the sample before its first (t0_us) to its last
// (t1_us), and the next starts where it ends.
struct RestWindow
{
    int64_t  t0_us = 0;            // esp_timer, us
    int64_t  t1_us = 0;
    uint32_t n = 0;                // samples in it
    double   f[3] = {};            // the mean specific force, body FRD, m/s^2
    double   sd_f = 0.0;           // rms of |f - mean| over its samples / sqrt(3): a 1-sigma a axis, m/s^2
    double   gyro_max_dps = 0.0;   // the largest |gyro| in it, deg/s (NaN if any was)
    bool     still = false;
};

class RestDetector
{
public:
    static constexpr int64_t  kWindowUs = 1000000;  // a window closes on its first sample this long after its start
    static constexpr int64_t  kGapUs = 100000;      // a gap longer than this between two samples breaks the run
    static constexpr double   kG = 9.80665;         // m/s^2
    static constexpr double   kGTol = 0.5;          // | |mean f| - g | under this, m/s^2
    static constexpr double   kSdMax = 0.05;        // sd_f under this, m/s^2
    static constexpr double   kGyroMaxDps = 3.0;    // every |gyro| under this, deg/s
    static constexpr uint32_t kEvery = 5;           // a still window in a row this many after the last sent goes

    // Whether a window's values are still.
    static bool stillOf(const RestWindow& w)
    {
        const double m = std::sqrt(w.f[0] * w.f[0] + w.f[1] * w.f[1] + w.f[2] * w.f[2]);
        return std::fabs(m - kG) < kGTol && w.sd_f < kSdMax && w.gyro_max_dps < kGyroMaxDps;
    }

    // One filter IMU sample at t_us (esp_timer): the specific force in body
    // FRD axes (m/s^2) and the raw gyro in the same axes (deg/s). True when
    // it closed a window that is to be sent (window()); a sample not after
    // the last one is passed over.
    bool feed(int64_t t_us, const double f[3], const double gyro_dps[3])
    {
        if (!have_last_ || t_us - last_us_ > kGapUs)
        {
            // The first sample, or the first after a gap: it starts a window
            // (and stands for nothing before it); a gap ends the run
            if (have_last_) run_ = 0;
            have_last_ = true;
            last_us_ = t_us;
            restart(t_us);
            return false;
        }
        if (t_us <= last_us_)
            return false;
        last_us_ = t_us;
        if (n_ == 0)
            for (int k = 0; k < 3; k++) ref_[k] = f[k];
        double g2 = 0.0;
        for (int k = 0; k < 3; k++)
        {
            const double d = f[k] - ref_[k];
            sum_[k] += d;
            sum2_[k] += d * d;
            g2 += gyro_dps[k] * gyro_dps[k];
        }
        const double g = std::sqrt(g2);
        if (!(g <= gmax_)) gmax_ = g;   // a NaN stays
        n_++;
        if (t_us - t0_us_ < kWindowUs)
            return false;

        RestWindow& w = win_;
        w.t0_us = t0_us_;
        w.t1_us = t_us;
        w.n = n_;
        double var = 0.0;
        for (int k = 0; k < 3; k++)
        {
            const double m = sum_[k] / n_;
            w.f[k] = ref_[k] + m;
            var += sum2_[k] / n_ - m * m;
        }
        w.sd_f = std::sqrt((var > 0.0 ? var : 0.0) / 3.0);
        w.gyro_max_dps = gmax_;
        w.still = stillOf(w);
        windows_++;
        restart(t_us);
        if (!w.still)
        {
            run_ = 0;
            return false;
        }
        run_++;
        return (run_ - 1) % kEvery == 0;
    }

    const RestWindow& window() const { return win_; }   // the last window closed
    uint32_t run() const { return run_; }               // still windows in a row, the last included
    uint32_t windows() const { return windows_; }       // windows closed

private:
    void restart(int64_t t_us)
    {
        t0_us_ = t_us;
        n_ = 0;
        gmax_ = 0.0;
        for (int k = 0; k < 3; k++) sum_[k] = sum2_[k] = 0.0;
    }

    bool       have_last_ = false;
    int64_t    last_us_ = 0;     // the last sample taken
    int64_t    t0_us_ = 0;       // the open window's start
    uint32_t   n_ = 0;
    double     ref_[3] = {};     // its first sample: sums about it, so the spread loses nothing to 1 g
    double     sum_[3] = {};
    double     sum2_[3] = {};
    double     gmax_ = 0.0;
    uint32_t   run_ = 0;
    uint32_t   windows_ = 0;
    RestWindow win_ = {};
};

// The rest a window is, stamped at its middle; seq, src, test and tq left to
// the sender.
inline sdr_aid::Rest restOf(const RestWindow& w)
{
    sdr_aid::Rest r;
    r.valid = w.still;
    const int64_t dur = (w.t1_us - w.t0_us + 500) / 1000;
    r.dur_ms = (uint16_t)(dur > 65535 ? 65535 : (dur < 0 ? 0 : dur));   // the codec saturates at 4095
    for (int k = 0; k < 3; k++) r.f[k] = w.f[k];
    r.sd_f = w.sd_f;
    return r;
}

// ── The attitude ──────────────────────────────────────────────────────────

// The filter's state right after an update.
struct FilterState
{
    int64_t t_us = 0;                       // the update's IMU sample (esp_timer, us)
    double  q_nb[4] = {1.0, 0.0, 0.0, 0.0};  // body (FRD) to NED, scalar first (getQuaternion)
    double  w[3] = {};                      // the body's rate, body axes, the gyro less the filter's bias, rad/s
    bool    fix_place = false;              // lat, lon from a fix the filter navigates on (not a GNSS-absent
                                            // start's pad-relative origin)
    double  lat = 0.0;                      // rad (getPosEst)
    double  lon = 0.0;
};

// An update's state from its IMU input (gyro deg/s) and the filter's gyro
// bias (rad/s, both filters' getRotRateBias).
inline void rateOf(const double gyro_dps[3], const float bias_rps[3], double w[3])
{
    for (int k = 0; k < 3; k++) w[k] = gyro_dps[k] * kDegToRad - (double)bias_rps[k];
}

class Sender
{
public:
    static constexpr int64_t kAttFreshUs = 100000;    // a filter state older than this at a pass: stopped
    static constexpr int64_t kRestDropUs = 3000000;   // a rest unsent this long after its window's end: dropped

    // A valid fix of the board's (its antenna, ECEF, m): the place from now
    // on. False (and nothing kept) for a position off the Earth.
    bool noteBoardFix(const double ecef[3])
    {
        double lat = 0.0, lon = 0.0;
        if (!latLonOf(ecef, lat, lon))
            return false;
        board_lat_ = lat;
        board_lon_ = lon;
        have_board_ = true;
        return true;
    }
    bool haveBoardPlace() const { return have_board_; }
    // That place, rad: false while there is none.
    bool boardPlace(double& lat, double& lon) const
    {
        lat = board_lat_;
        lon = board_lon_;
        return have_board_;
    }

    // Each update's state, right after it.
    void noteFilter(const FilterState& s)
    {
        filt_ = s;
        have_filt_ = true;
    }

    // A window the detector says to send (a newer one replaces one not sent).
    void noteRest(const RestWindow& w)
    {
        rest_ = w;
        rest_pending_ = true;
    }
    bool restPending() const { return rest_pending_; }

    // The attitude a pass at now_us sends, without its stamp's time fields
    // and seq: the filter's, or not valid (see the top). stamp_us: the MCU
    // instant it holds at.
    sdr_aid::Attitude attitude(int64_t now_us, bool test, uint8_t tq, int64_t& stamp_us) const
    {
        sdr_aid::Attitude a;
        a.h.src = sdr_aid::kSrcOwn;
        a.h.test = test;
        a.h.tq = tq;
        a.frame = 0;
        stamp_us = now_us;
        if (!have_filt_ || now_us - filt_.t_us > kAttFreshUs)
            return a;   // not valid, nor its rate
        stamp_us = filt_.t_us;
        a.rate_valid = std::isfinite(filt_.w[0]) && std::isfinite(filt_.w[1]) && std::isfinite(filt_.w[2]);
        for (int k = 0; k < 3; k++) a.w[k] = filt_.w[k];
        double lat = 0.0, lon = 0.0;
        if (have_board_)
        {
            lat = board_lat_;
            lon = board_lon_;
        }
        else if (filt_.fix_place)
        {
            lat = filt_.lat;
            lon = filt_.lon;
        }
        else
        {
            return a;   // no place: no attitude in ECEF
        }
        bodyToEcef(filt_.q_nb, lat, lon, a.q);
        a.att_valid = std::isfinite(a.q[0]) && std::isfinite(a.q[1]) && std::isfinite(a.q[2]) && std::isfinite(a.q[3]);
        return a;
    }

    // The frame to send now into out (cap at least sdr_aid::kMaxFrame): its
    // length, 0 for none. line: the phase sender, whose held line stamps it.
    size_t frame(int64_t now_us, const sdr_phase::Sender& line, bool test, uint8_t* out, size_t cap)
    {
        last_sub_ = 0;
        if (rest_pending_ && now_us - rest_.t1_us > kRestDropUs)
        {
            rest_pending_ = false;
            rests_dropped_++;
        }
        if (!line.lineUsable(now_us))
            return 0;
        const uint8_t tq = line.lineTq(now_us);
        double gps_s = 0.0;
        if (sched_.due(now_us, 0) == sdr_aid::kAttitude)
        {
            int64_t stamp_us = 0;
            sdr_aid::Attitude a = attitude(now_us, test, tq, stamp_us);
            if (!line.gpsAt(now_us, stamp_us, gps_s))
                return 0;
            stamp(a.h, gps_s, att_seq_);
            last_sub_ = sdr_aid::kAttitude;
            return sdr_aid::encode(a, out, cap);
        }
        if (rest_pending_)
        {
            sdr_aid::Rest r = restOf(rest_);
            r.h.src = sdr_aid::kSrcOwn;
            r.h.test = test;
            r.h.tq = tq;
            if (!line.gpsAt(now_us, rest_.t0_us + (rest_.t1_us - rest_.t0_us) / 2, gps_s))
                return 0;
            stamp(r.h, gps_s, rest_seq_);
            last_sub_ = sdr_aid::kRest;
            return sdr_aid::encode(r, out, cap);
        }
        return 0;
    }

    // The frame frame() last gave went out at now_us.
    void sent(int64_t now_us)
    {
        if (last_sub_ == sdr_aid::kAttitude)
        {
            att_seq_++;
            sched_.sent(sdr_aid::kAttitude, now_us, 0);
        }
        else if (last_sub_ == sdr_aid::kRest)
        {
            rest_seq_++;
            rest_pending_ = false;
        }
        last_sub_ = 0;
    }

    uint8_t  attSeq() const { return att_seq_; }     // the next frame's of each
    uint8_t  restSeq() const { return rest_seq_; }
    uint32_t restsDropped() const { return rests_dropped_; }

private:
    static void stamp(sdr_aid::Header& h, double gps_s, uint8_t seq)
    {
        const sdr_aid::Header t = sdr_aid::gpsStamp(gps_s);
        h.week = t.week;
        h.tow_ms = t.tow_ms;
        h.sub_ns = t.sub_ns;
        h.seq = seq;
    }

    sdr_aid::AidSchedule sched_{1u << sdr_aid::kAttitude};
    FilterState filt_ = {};
    bool        have_filt_ = false;
    bool        have_board_ = false;
    double      board_lat_ = 0.0;
    double      board_lon_ = 0.0;
    RestWindow  rest_ = {};
    bool        rest_pending_ = false;
    uint8_t     att_seq_ = 0;
    uint8_t     rest_seq_ = 0;
    uint8_t     last_sub_ = 0;        // the sub-type of the frame frame() last gave
    uint32_t    rests_dropped_ = 0;
};

}  // namespace sdr_aid_fc
