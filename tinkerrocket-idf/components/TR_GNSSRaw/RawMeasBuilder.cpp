// RawMeasBuilder.cpp — see RawMeasBuilder.h
#include "RawMeasBuilder.h"

#include <cmath>

#include "gnss/atmo.h"
#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

constexpr double kBdtMinusGpst = BDT_MINUS_GPST;   // s
constexpr double kPi = 3.14159265358979323846;

double cn0Sigma(double a, double b, double cn0)
{
    return std::sqrt(a * a + b * b * std::pow(10.0, -(cn0 - 35.0) / 10.0));
}

}  // namespace

void RawMeasBuilder::reset()
{
    for (int i = 0; i < kSatCache; ++i)
        cache_[i].valid = false;
    cr_n_[0] = cr_n_[1] = 0;
    cr_done_ = true;
    for (int s = 0; s < kSys; ++s)
        for (int p = 0; p < kPrn; ++p)
        {
            first_[s][p] = 0.0;
            last_[s][p] = 0.0;
            seen_[s][p] = false;
        }
}

void RawMeasBuilder::noteEpoch(const RawEpoch& ep)
{
    for (int i = 0; i < ep.n; ++i)
    {
        const RawObs& o = ep.obs[i];
        if (o.sys >= kSys || o.prn >= kPrn)
            continue;
        if (!seen_[o.sys][o.prn] || gps_time_diff(ep.tow_s, last_[o.sys][o.prn]) > cfg_.relock_gap_s)
            first_[o.sys][o.prn] = ep.tow_s;
        last_[o.sys][o.prn] = ep.tow_s;
        seen_[o.sys][o.prn] = true;
    }
}

// The satellite's state near t_sv: kept from an earlier epoch if it is the
// same ephemeris's and within sat_refresh_s, else solved exactly here -- the
// position at the transmit time and half a second either side (velocity and
// acceleration from them, as the bench prototype takes them), the clock
// likewise.
const RawMeasBuilder::SatState& RawMeasBuilder::satState(const gps_eph_t& e, int sys, int prn, double t_sv)
{
    cache_tick_++;
    SatState* slot = nullptr;
    for (int i = 0; i < kSatCache; ++i)
    {
        SatState& c = cache_[i];
        if (c.valid && c.sys == sys && c.prn == prn)
        {
            if (c.iode == e.iode && c.toe == e.toe && cfg_.sat_refresh_s > 0.0
                && std::fabs(gps_time_diff(t_sv, c.t_ref)) <= cfg_.sat_refresh_s)
            {
                c.used = cache_tick_;
                return c;
            }
            slot = &c;                    // stale: solve it again in place
            break;
        }
    }
    if (slot == nullptr)
    {
        slot = &cache_[0];
        for (int i = 0; i < kSatCache; ++i)
        {
            if (!cache_[i].valid)
            {
                slot = &cache_[i];
                break;
            }
            if (cache_[i].used < slot->used)
                slot = &cache_[i];
        }
    }
    const double t_tx = t_sv - gps_sat_clock(&e, t_sv);
    double rs[3], rs1[3], rs2[3], dts, dts1, dts2;
    gps_sat_pos(&e, t_tx, rs, nullptr, &dts);
    gps_sat_pos(&e, t_tx + 0.5, rs2, nullptr, &dts2);
    gps_sat_pos(&e, t_tx - 0.5, rs1, nullptr, &dts1);
    SatState& c = *slot;
    c.sys = static_cast<uint8_t>(sys);
    c.prn = static_cast<uint8_t>(prn);
    c.valid = true;
    c.iode = e.iode;
    c.toe = e.toe;
    c.t_ref = t_tx;
    for (int k = 0; k < 3; ++k)
    {
        c.pos[k] = rs[k];
        c.vel[k] = rs2[k] - rs1[k];
        c.acc[k] = (rs2[k] - 2.0 * rs[k] + rs1[k]) / 0.25;
    }
    c.clk = dts;
    c.clk_rate = dts2 - dts1;
    c.used = cache_tick_;
    return c;
}

void RawMeasBuilder::markRelock(int sys, int prn, double tow)
{
    if (sys < 0 || sys >= kSys || prn < 0 || prn >= kPrn)
        return;
    first_[sys][prn] = tow;
    last_[sys][prn] = tow;
    seen_[sys][prn] = true;
}

double RawMeasBuilder::relockAge(int sys, int prn, double tow) const
{
    if (sys < 0 || sys >= kSys || prn < 0 || prn >= kPrn || !seen_[sys][prn])
        return 0.0;
    return gps_time_diff(tow, first_[sys][prn]);
}

int RawMeasBuilder::build(const RawEpoch& ep, const EphemerisStore& eph, const double* r_approx,
                          double g_peak, RawMeas* out, int cap)
{
    begin(ep, r_approx, g_peak);
    return next(ep, eph, out, 0, cap, ep.n);
}

void RawMeasBuilder::begin(const RawEpoch& ep, const double* r_approx, double g_peak)
{
    noteEpoch(ep);

    have_pos_ = r_approx != nullptr
        && std::sqrt(r_approx[0] * r_approx[0] + r_approx[1] * r_approx[1] + r_approx[2] * r_approx[2]) > 6.0e6;
    lat_ = lon_ = hgt_ = 0.0;
    if (have_pos_)
    {
        for (int k = 0; k < 3; ++k)
            r_approx_[k] = r_approx[k];
        ecef_to_geo(r_approx_, &lat_, &lon_, &hgt_);
    }
    el_mask_ = cfg_.el_mask_deg * kPi / 180.0;
    g_over_ = g_peak > 0.0 ? g_peak : 0.0;
    next_obs_ = 0;
    // this epoch's carrier ends, from none: the last epoch's are kept until
    // it is done (an epoch begun and not finished leaves them as they were)
    cr_n_[cr_side_ ^ 1] = 0;
    cr_done_ = false;
    cr_model_ = have_pos_ ? static_cast<uint8_t>(1u | (static_cast<unsigned>(iono_gen_) << 1)) : 0;
    if (cfg_.lite)
        beginLite(ep);                    // the cheaper build's epoch (RawMeasBuilderLite.cpp)
}

int RawMeasBuilder::next(const RawEpoch& ep, const EphemerisStore& eph, RawMeas* out, int n, int cap, int max_obs)
{
    if (cfg_.lite)
        return nextLite(ep, eph, nullptr, out, n, cap, max_obs);   // the cheaper build (RawMeasBuilderLite.cpp)
    for (int looked = 0; looked < max_obs && next_obs_ < ep.n && n < cap; ++looked)
    {
        const int i = next_obs_++;
        const RawObs& o = ep.obs[i];
        if (!o.has_pr || o.sys >= kSys || o.prn >= kPrn)
            continue;
        const GnssSignalProfile& sig = cfg_.sig[o.sys];   // the receiver's signal of that system
        if (o.cn0_dbhz < sig.cn0_min_pr)
            continue;
        // each ephemeris keeps its own system's time: BeiDou runs 14 s behind GPS
        const double to_sys = o.sys == GNSS_SYS_BDS ? kBdtMinusGpst : 0.0;
        const gps_eph_t* e = eph.select(o.sys, o.prn, ep.tow_s + to_sys);
        if (e == nullptr)
            continue;

        // transmit time: the travel time, then the satellite's own clock
        const double t_sv = ep.tow_s - o.pr_m / GNSS_C + to_sys;
        const SatState& c = satState(*e, o.sys, o.prn, t_sv);
        const double t_tx = t_sv - (c.clk + c.clk_rate * gps_time_diff(t_sv, c.t_ref));
        const double dt = gps_time_diff(t_tx, c.t_ref);
        double rs[3];
        for (int k = 0; k < 3; ++k)
            rs[k] = c.pos[k] + (c.vel[k] + 0.5 * c.acc[k] * dt) * dt;
        const double dts = c.clk + c.clk_rate * dt;

        double corr = GNSS_C * dts;   // the clock with relativity, less the group delay
        double iono = 0.0, trop = 0.0;
        if (have_pos_)
        {
            const double d[3] = {rs[0] - r_approx_[0], rs[1] - r_approx_[1], rs[2] - r_approx_[2]};
            double az, el;
            gnss_az_el(lat_, lon_, d, &az, &el);
            if (el < el_mask_)
                continue;
            const double fr = GNSS_FREQ_L1_HZ / sig.carrier_hz;
            iono = iono_klobuchar(&iono_, lat_, lon_, az, el, ep.tow_s) * fr * fr;
            trop = cfg_.troposphere ? tropo_saastamoinen(lat_, hgt_, el) : 0.0;
            corr -= iono + trop;
        }

        RawMeas& m = out[n++];
        m.sys = o.sys;
        m.prn = o.prn;
        for (int k = 0; k < 3; ++k)
        {
            m.sat_pos[k] = rs[k];
            m.sat_vel[k] = c.vel[k] + c.acc[k] * dt;
            m.sat_acc[k] = c.acc[k];
        }
        m.has_pr = true;
        m.pr = o.pr_m + corr;
        m.has_rr = o.has_rate && !(o.cn0_dbhz < sig.cn0_min_rr);
        m.rr = m.has_rr ? o.rate_mps + GNSS_C * c.clk_rate : 0.0;

        const double cn0 = static_cast<double>(o.cn0_dbhz);
        const double lam = GNSS_C / sig.carrier_hz;
        const double s_rr = std::hypot(cn0Sigma(cfg_.rr_white_a, cfg_.rr_white_b, cn0),
                                       lam * cfg_.doppler_step_hz / std::sqrt(12.0));
        double s_corr = cn0Sigma(cfg_.pr_corr_a, cfg_.pr_corr_b, cn0) * cfg_.pr_corr_scale;
        const double age = relockAge(o.sys, o.prn, ep.tow_s);
        const double fresh = std::exp(-age / cfg_.relock_tau_s);
        if (cfg_.relock_sigma_m > 0.0)
            s_corr = std::hypot(s_corr, cfg_.relock_sigma_m * fresh);
        const double s_dyn = cfg_.rr_dyn_sigma * g_over_;
        const double s_rel = cfg_.rr_relock_sigma * fresh;
        const double s_rr2 = s_rr * cfg_.rr_scale;
        m.sigma_pr = static_cast<float>(cn0Sigma(cfg_.pr_white_a, cfg_.pr_white_b, cn0));
        m.sigma_pr_corr = static_cast<float>(s_corr);
        m.tau_pr = static_cast<float>(cfg_.pr_tau_s);
        m.sigma_rr = static_cast<float>(std::sqrt(s_rr2 * s_rr2 + s_dyn * s_dyn + s_rel * s_rel));
        m.tau_rr = static_cast<float>(cfg_.rr_tau_s);

        // the carrier range rate (RawMeasCarrier.cpp)
        carrierRate(o, sig, ep.tow_s, dts, iono, trop, rs, m);
    }
    if (!cr_done_ && next_obs_ >= ep.n)
    {
        cr_side_ ^= 1;                    // the epoch is done: its ends are the last epoch's now
        cr_done_ = true;
    }
    return n;
}

}  // namespace gnss_raw
