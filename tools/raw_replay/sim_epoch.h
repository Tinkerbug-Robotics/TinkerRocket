// sim_epoch.h -- a simulated receiver's epoch through the GNSS task's stage,
// offline (raw_replay's 'X' events: Grid B, sim_events.py).
//
// A sim has no ephemerides and no SBF: its receiver model hands each
// satellite already placed (the satellite at the transmit time, its velocity
// and acceleration) with a corrected pseudorange, a range rate and a C/N0.
// What the GNSS task's stage does with an epoch (RawGnssMeasStage::offer)
// still applies, and this does it with the stage's own code wherever the
// stage has it:
//   - see() on the whole epoch (tracked, tracked_no_eph), the untimed and
//     idle refusals, every satellite's lock age noted on the whole sky
//     (RawMeasBuilder::noteEpoch: a satellite missing longer than the
//     profile's relock_gap_s has relocked);
//   - the satellite cap (capEpoch, capBuilt: four GPS first, then by C/N0
//     with the 3 dB hold, a pick dropped under the mask picked last), on a
//     stand-in ephemeris for each satellite the sim names, so the cap's
//     "has an ephemeris" holds and nothing is ever built from it;
//   - the ionosphere's choice (chooseIono: broadcast, as the mosaic's GPSIon
//     sets it on the pad).
// What the builder does to each pick is mirrored here, from its config (the
// receiver profile): the C/N0 floors of the code and the Doppler, the
// elevation mask at the filter's position, and the noise model --
// sigma^2 = a^2 + b^2 10^(-(C/N0 - 35)/10) for the white and correlated code
// and the Doppler, the correlated code's relock term decaying with the
// satellite's lock age, the Doppler's relock term, rr_scale -- with no
// stress term (the stage builds with g_peak 0; the filter adds it from the
// IMU). A sim's own extra Doppler sigma (rr_sig_add) is added in
// quadrature -- for experiments only: the firmware has no path that tells
// the filter one satellite's Doppler is worse (the builder's relock term
// comes from a gap it sees, and the mosaic's own Doppler variance is decoded
// but not used), so sim_events.py's patterns leave it 0.
// The carrier is off on the mosaic, so has_cr is never set.
// With the cheaper build (the profile's build_lite, the builder's Config::lite)
// the mirror is that build's: the noise from RawMeasBuilder::liteSigmas, the
// filter's geodetic position as given, and each satellite's elevation kept
// for its atmo_period_s (a SimLite's, the caller's). A sim has no orbit to
// solve and no atmosphere, so the rest of that build never runs here.
//
// The stage's private members are the caller's to open: include this after
// RawGnssMeasStage.h is read with private and protected public, as
// raw_replay.cpp and its test do.
// Header-only and pure (tests_cpp/test_raw_replay_sim.cpp).
#ifndef RAW_REPLAY_SIM_EPOCH_H
#define RAW_REPLAY_SIM_EPOCH_H

#include <cmath>
#include <cstdint>
#include <cstring>

#include "RawGnssMeasStage.h"
#include "RawMeas.h"
#include "ShadowLink.h"
#include "gnss/atmo.h"
#include "gnss/eph.h"
#include "gnss/types.h"

namespace rr
{

// One satellite of a sim's epoch, as its receiver model gives it.
struct SimMeas
{
    uint8_t sys = 0, prn = 0;      // gnss_sys_t, PRN (1-63; BeiDou MEO/IGSO)
    bool    has_pr = false, has_rr = false;
    double  sat_pos[3] = {}, sat_vel[3] = {}, sat_acc[3] = {};   // ECEF of the transmit time
    double  pr = 0.0;              // m, corrected, the receiver clock in it
    double  rr = 0.0;              // m/s, the receiver's drift in it
    float   cn0_dbhz = 0.0f;
    float   rr_sig_add = 0.0f;     // m/s more on the Doppler's sigma (in quadrature)
};

// One epoch of it: its GPS instant on the MCU clock (0: untimed), time of week
// and week.
struct SimEpoch
{
    int64_t  mcu_us = 0;
    double   tow_s = 0.0;
    uint16_t week = 0;
    uint8_t  rr_paired = 0;   // its Dopplers pairs' means (the harness's pairs: TimedRawEpoch::rr_paired)
    int      n = 0;
    SimMeas  meas[gnss_raw::RawEpoch::kMaxObs];
};

// What the sim path did, beside the stage's own counts.
struct SimCounts
{
    uint32_t epochs = 0;           // offered
    uint32_t built = 0;            // handed to the filter
    uint32_t eph_standins = 0;     // stand-in ephemerides filed
    uint32_t below_cn0 = 0;        // picks under the code's C/N0 floor
    uint32_t below_mask = 0;       // picks under the elevation mask
    uint32_t meas_built = 0;       // measurements built
};

// The cheaper build's mirror, where the tree has that build (raw_replay's
// RAW_REPLAY_BUILD_LITE; a test, built with its own tree, has it).
#ifndef RAW_REPLAY_BUILD_LITE
#define RAW_REPLAY_BUILD_LITE 1
#endif
#if RAW_REPLAY_BUILD_LITE
inline bool simLiteOn(const gnss_raw::RawMeasBuilder::Config& c) { return c.lite; }
inline double simLitePeriod(const gnss_raw::RawMeasBuilder::Config& c) { return c.atmo_period_s; }
inline int64_t simLitePeriodOf(double tow_s, double period_s)
{
    return gnss_raw::RawMeasBuilder::litePeriodOf(tow_s, period_s);
}
inline void simLiteSigmas(const gnss_raw::RawMeasBuilder::Config& c, double cn0, double age_s, double carrier_hz,
                          gnss_raw::RawMeas& m)
{
    gnss_raw::RawMeasBuilder::liteSigmas(c, cn0, age_s, carrier_hz, 0.0, m.sigma_pr, m.sigma_pr_corr, m.sigma_rr);
    m.tau_pr = static_cast<float>(c.pr_tau_s);
    m.tau_rr = static_cast<float>(c.rr_tau_s);
}
#else
inline bool simLiteOn(const gnss_raw::RawMeasBuilder::Config&) { return false; }
inline double simLitePeriod(const gnss_raw::RawMeasBuilder::Config&) { return 0.0; }
inline int64_t simLitePeriodOf(double, double) { return INT64_MIN; }
inline void simLiteSigmas(const gnss_raw::RawMeasBuilder::Config&, double, double, double, gnss_raw::RawMeas&) {}
#endif

// The cheaper build's elevations for a sim's epochs: each satellite's as
// worked out on the first epoch of each atmo_period_s it was built in.
struct SimLite
{
    int64_t k[3][64];
    double  el[3][64];
    SimLite()
    {
        for (int s = 0; s < 3; ++s)
            for (int p = 0; p < 64; ++p)
            {
                k[s][p] = INT64_MIN;
                el[s][p] = 0.0;
            }
    }
};

// The noise the builder gives a measurement at C/N0 cn0 and lock age age_s
// (RawMeasBuilder::next, g_peak 0): sigma_pr, sigma_pr_corr, tau_pr,
// sigma_rr, tau_rr.
inline void simSigmas(const gnss_raw::RawMeasBuilder::Config& c, double cn0, double age_s, double carrier_hz,
                      gnss_raw::RawMeas& m)
{
    auto cn0Sigma = [](double a, double b, double x) { return std::sqrt(a * a + b * b * std::pow(10.0, -(x - 35.0) / 10.0)); };
    const double lam = GNSS_C / carrier_hz;
    const double s_rr = std::hypot(cn0Sigma(c.rr_white_a, c.rr_white_b, cn0), lam * c.doppler_step_hz / std::sqrt(12.0));
    double s_corr = cn0Sigma(c.pr_corr_a, c.pr_corr_b, cn0) * c.pr_corr_scale;
    const double fresh = std::exp(-age_s / c.relock_tau_s);
    if (c.relock_sigma_m > 0.0)
        s_corr = std::hypot(s_corr, c.relock_sigma_m * fresh);
    const double s_rel = c.rr_relock_sigma * fresh;
    const double s_rr2 = s_rr * c.rr_scale;
    m.sigma_pr = static_cast<float>(cn0Sigma(c.pr_white_a, c.pr_white_b, cn0));
    m.sigma_pr_corr = static_cast<float>(s_corr);
    m.tau_pr = static_cast<float>(c.pr_tau_s);
    m.sigma_rr = static_cast<float>(std::sqrt(s_rr2 * s_rr2 + s_rel * s_rel));
    m.tau_rr = static_cast<float>(c.rr_tau_s);
}

// The GNSS task's turn for a sim epoch (RawGnssMeasStage::offer): true, and
// out is the epoch to hand the filter's task, if it is timed and the filter
// runs (nav: its state; the build's position is its). first: the first
// epoch this stage builds (its ionosphere then chosen). lite: the cheaper
// build's kept elevations (nullptr: each worked out every epoch).
inline bool simOffer(gnss_raw::RawGnssMeasStage& st, const SimEpoch& x, const gnss_raw::ShadowNav& nav,
                     gnss_raw::BuiltEpoch& out, SimCounts& c, SimLite* lite = nullptr)
{
    using namespace gnss_raw;
    c.epochs++;
    TimedRawEpoch te;
    te.mcu_us = x.mcu_us;
    te.arrival_us = x.mcu_us;
    te.gps_week = x.week;
    te.epoch.tow_s = x.tow_s;
    const int n = x.n < RawEpoch::kMaxObs ? x.n : RawEpoch::kMaxObs;
    te.epoch.n = n;
    for (int i = 0; i < n; ++i)
    {
        const SimMeas& s = x.meas[i];
        RawObs& o = te.epoch.obs[i];
        o.sys = s.sys;
        o.prn = s.prn;
        o.has_pr = s.has_pr;
        o.has_rate = s.has_rr;
        o.cn0_dbhz = s.cn0_dbhz;
        o.pr_m = s.pr;
        o.rate_mps = s.rr;
        // each satellite the sim names has its orbit: a stand-in ephemeris (never built from) so the stage's
        // ephemeris tests hold, filed as the receiver would have filed the real one before the epoch
        if (s.sys <= GNSS_SYS_BDS && s.prn >= 1 && s.prn < 64 && !st.hasEphemeris(s.sys, s.prn, x.tow_s))
        {
            gps_eph_t e;
            std::memset(&e, 0, sizeof(e));
            e.valid = 1;
            e.sys = s.sys;
            e.prn = s.prn;
            e.week = x.week;
            e.iode = e.iodc = 1;
            e.toe = e.toc = x.tow_s + (s.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
            e.sqrt_a = 5153.7;
            if (st.addEphemeris(e))
                c.eph_standins++;
        }
    }
    st.see(te);
    if (te.mcu_us == 0)
    {
        st.st_.untimed++;
        st.builder_.noteEpoch(te.epoch);
        return false;
    }
    if (!nav.running)
    {
        st.st_.idle++;
        st.builder_.noteEpoch(te.epoch);
        return false;
    }
    double r[3];
    geo_to_ecef(nav.lla[0], nav.lla[1], nav.lla[2], r);
    // every satellite's lock age from the whole sky, then the cap's picks (as offer: begin() would note the
    // picks again at the same time, no gap)
    st.builder_.noteEpoch(te.epoch);
    const TimedRawEpoch* use = &te;
    if (st.cfg_.sat_cap > 0)
    {
        st.capEpoch(te);
        use = &st.sel_;
    }
    st.chooseIono(nav.in_flight != 0, out);
    out.mcu_us = te.mcu_us;
    out.tow_s = te.epoch.tow_s;
    out.gps_week = te.gps_week;
    out.rr_paired = x.rr_paired;
    for (int k = 0; k < 3; ++k)
        out.r_approx[k] = r[k];
    out.build_us = 0;
    out.n = 0;
    const RawMeasBuilder::Config& bc = st.builder_.config();
    const bool have_pos = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) > 6.0e6;
    double lat = 0.0, lon = 0.0, hgt = 0.0;
    if (have_pos)
    {
        if (simLiteOn(bc))
        {
            lat = nav.lla[0];             // as the stage hands the cheaper build its position
            lon = nav.lla[1];
            hgt = nav.lla[2];
        }
        else
        {
            ecef_to_geo(r, &lat, &lon, &hgt);
        }
    }
    const double el_mask = bc.el_mask_deg * M_PI / 180.0;
    const bool keep_el = simLiteOn(bc) && lite != nullptr && simLitePeriod(bc) > 0.0;
    const int64_t el_k = keep_el ? simLitePeriodOf(te.epoch.tow_s, simLitePeriod(bc)) : INT64_MIN;
    for (int i = 0; i < use->epoch.n; ++i)
    {
        const RawObs& o = use->epoch.obs[i];
        if (!o.has_pr || o.sys > GNSS_SYS_BDS || o.prn >= 64)
            continue;
        const GnssSignalProfile& sig = bc.sig[o.sys];
        if (o.cn0_dbhz < sig.cn0_min_pr)
        {
            c.below_cn0++;
            continue;
        }
        const SimMeas* s = nullptr;
        for (int j = 0; j < n && s == nullptr; ++j)
            if (x.meas[j].sys == o.sys && x.meas[j].prn == o.prn)
                s = &x.meas[j];
        if (s == nullptr)
            continue;
        if (have_pos)
        {
            double az, el;
            if (keep_el && lite->k[o.sys][o.prn] == el_k)
            {
                el = lite->el[o.sys][o.prn];
            }
            else
            {
                const double d[3] = {s->sat_pos[0] - r[0], s->sat_pos[1] - r[1], s->sat_pos[2] - r[2]};
                gnss_az_el(lat, lon, d, &az, &el);
                if (keep_el)
                {
                    lite->k[o.sys][o.prn] = el_k;
                    lite->el[o.sys][o.prn] = el;
                }
            }
            if (el < el_mask)
            {
                c.below_mask++;
                continue;
            }
        }
        RawMeas& m = out.meas[out.n];
        std::memset(&m, 0, sizeof(m));
        m.sys = o.sys;
        m.prn = o.prn;
        for (int k = 0; k < 3; ++k)
        {
            m.sat_pos[k] = s->sat_pos[k];
            m.sat_vel[k] = s->sat_vel[k];
            m.sat_acc[k] = s->sat_acc[k];
        }
        m.has_pr = true;
        m.pr = s->pr;
        m.has_rr = o.has_rate && !(o.cn0_dbhz < sig.cn0_min_rr);
        m.rr = m.has_rr ? s->rr : 0.0;
        const double age = st.builder_.relockAge(o.sys, o.prn, te.epoch.tow_s);
        if (simLiteOn(bc))
            simLiteSigmas(bc, static_cast<double>(o.cn0_dbhz), age, sig.carrier_hz, m);
        else
        {
            simSigmas(bc, static_cast<double>(o.cn0_dbhz), age, sig.carrier_hz, m);
        }
        if (s->rr_sig_add > 0.0f)
            m.sigma_rr = std::hypot(m.sigma_rr, s->rr_sig_add);
        m.has_cr = false;
        out.lock_age_s[out.n] = age;
        out.n++;
        c.meas_built++;
    }
    if (st.cfg_.sat_cap > 0)
        st.capBuilt(out);
    st.st_.built++;
    c.built++;
    return true;
}

}  // namespace rr

#endif  // RAW_REPLAY_SIM_EPOCH_H
