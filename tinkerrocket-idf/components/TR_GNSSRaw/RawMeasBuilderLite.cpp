// RawMeasBuilderLite.cpp — RawMeasBuilder's cheaper build (Config::lite, see
// RawMeasBuilder.h): next()'s measurements with each satellite's atmosphere
// kept for a period, its orbit solved on a phase of its own, and the noise
// with one exp. Apart from the rest of the builder so that on the Beetle,
// where the builder runs from internal RAM (raw_gnss_iram.lf) and lite is
// off, this stays in flash.
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
constexpr double kNegLn10Over10 = -0.23025850929940458;   // 10^(-x/10) = exp(kNegLn10Over10 * x)
// A cache slot's phase of sat_refresh_s: the golden ratio's multiples, so the
// slots a cold start fills in turn spread over the period evenly
constexpr double kGolden = 0.6180339887498949;

}  // namespace

// As RawGnssShadow's periodIndex (on the millisecond the receivers tag their
// epochs on).
int64_t RawMeasBuilder::litePeriodOf(double tow_s, double period_s)
{
    const long long p = std::llround(period_s * 1000.0);
    const long long ms = std::llround(tow_s * 1000.0);
    if (p <= 0)
        return ms;
    return ms >= 0 ? ms / p : -((p - 1 - ms) / p);
}

// The C/N0's part of the noise: sigma_pr, and the correlated code's and the
// Doppler's before their relock and stress terms.
void RawMeasBuilder::liteNoise(const Config& c, double cn0, double carrier_hz, float& sigma_pr, double& s_corr,
                               double& s_rr2)
{
    // the three sigma^2 = a^2 + b^2 * 10^(-(C/N0 - 35)/10) on one exp
    const double q = std::exp(kNegLn10Over10 * (cn0 - 35.0));
    double s_rr = std::sqrt(c.rr_white_a * c.rr_white_a + c.rr_white_b * c.rr_white_b * q);
    if (c.doppler_step_hz != 0.0)
        s_rr = std::hypot(s_rr, GNSS_C / carrier_hz * c.doppler_step_hz / std::sqrt(12.0));
    s_corr = std::sqrt(c.pr_corr_a * c.pr_corr_a + c.pr_corr_b * c.pr_corr_b * q) * c.pr_corr_scale;
    s_rr2 = s_rr * c.rr_scale;
    sigma_pr = static_cast<float>(std::sqrt(c.pr_white_a * c.pr_white_a + c.pr_white_b * c.pr_white_b * q));
}

// The relock and stress terms on liteNoise's: sigma_pr_corr and sigma_rr.
void RawMeasBuilder::liteRelock(const Config& c, double age_s, double g_over, double s_corr, double s_rr2,
                                float& sigma_pr_corr, float& sigma_rr)
{
    // none once the lock is kLiteFreshMax relock_tau_s old
    const double x = age_s / c.relock_tau_s;
    const double fresh = x < kLiteFreshMax ? std::exp(-x) : 0.0;
    if (c.relock_sigma_m > 0.0 && fresh > 0.0)
        s_corr = std::hypot(s_corr, c.relock_sigma_m * fresh);
    const double s_dyn = c.rr_dyn_sigma * g_over;
    const double s_rel = c.rr_relock_sigma * fresh;
    sigma_pr_corr = static_cast<float>(s_corr);
    sigma_rr = static_cast<float>(s_dyn == 0.0 && s_rel == 0.0
                                      ? s_rr2
                                      : std::sqrt(s_rr2 * s_rr2 + s_dyn * s_dyn + s_rel * s_rel));
}

void RawMeasBuilder::liteSigmas(const Config& c, double cn0, double age_s, double carrier_hz, double g_over,
                                float& sigma_pr, float& sigma_pr_corr, float& sigma_rr)
{
    double s_corr, s_rr2;
    liteNoise(c, cn0, carrier_hz, sigma_pr, s_corr, s_rr2);
    liteRelock(c, age_s, g_over, s_corr, s_rr2, sigma_pr_corr, sigma_rr);
}

void RawMeasBuilder::beginWith(const RawEpoch& ep, const double* r_approx, const double* lla, double g_peak, bool noted)
{
    if (!cfg_.lite)
    {
        begin(ep, r_approx, g_peak);
        return;
    }
    // begin(), with the caller's geodetic position and lock ages
    if (!noted)
        noteEpoch(ep);
    have_pos_ = r_approx != nullptr
        && std::sqrt(r_approx[0] * r_approx[0] + r_approx[1] * r_approx[1] + r_approx[2] * r_approx[2]) > 6.0e6;
    lat_ = lon_ = hgt_ = 0.0;
    if (have_pos_)
    {
        for (int k = 0; k < 3; ++k)
            r_approx_[k] = r_approx[k];
        if (lla != nullptr)
        {
            lat_ = lla[0];
            lon_ = lla[1];
            hgt_ = lla[2];
        }
        else
        {
            ecef_to_geo(r_approx_, &lat_, &lon_, &hgt_);
        }
    }
    el_mask_ = cfg_.el_mask_deg * kPi / 180.0;
    g_over_ = g_peak > 0.0 ? g_peak : 0.0;
    next_obs_ = 0;
    cr_n_[cr_side_ ^ 1] = 0;
    cr_done_ = false;
    cr_model_ = have_pos_ ? static_cast<uint8_t>(1u | (static_cast<unsigned>(iono_gen_) << 1)) : 0;
    beginLite(ep);
}

int RawMeasBuilder::nextWith(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of,
                             RawMeas* out, int n, int cap, int max_obs)
{
    if (!cfg_.lite)
        return next(ep, eph, out, n, cap, max_obs);
    return nextLite(ep, eph, eph_of, out, n, cap, max_obs);
}

// The epoch's part: its atmosphere period, and nothing of the receiver's
// trigonometry or troposphere worked out yet (atmoLite and zenithLite, when a
// satellite first needs them).
void RawMeasBuilder::beginLite(const RawEpoch& ep)
{
    lite_k_ = cfg_.atmo_period_s > 0.0 ? litePeriodOf(ep.tow_s, cfg_.atmo_period_s) : -1;
    lite_geo_ = false;
    lite_zen_ = false;
}

// satState() with each slot's solve on a phase of its own: kept while it is
// the same ephemeris's, within sat_refresh_s, and short of its slot's next
// grid time (t_next); solved again in place otherwise (its atmosphere kept: a
// new solve or ephemeris moves the satellite by metres at most). A slot taken
// for a new satellite works its atmosphere out anew.
int RawMeasBuilder::satSlotLite(const gps_eph_t& e, int sys, int prn, double t_sv)
{
    cache_tick_++;
    int slot = -1;
    for (int i = 0; i < kSatCache; ++i)
    {
        SatState& c = cache_[i];
        if (c.valid && c.sys == sys && c.prn == prn)
        {
            if (c.iode == e.iode && c.toe == e.toe && cfg_.sat_refresh_s > 0.0
                && std::fabs(gps_time_diff(t_sv, c.t_ref)) <= cfg_.sat_refresh_s
                && gps_time_diff(t_sv, lite_[i].t_next) < 0.0)
            {
                c.used = cache_tick_;
                return i;
            }
            slot = i;                     // stale, or its phase came round: solved again in place
            break;
        }
    }
    if (slot < 0)
    {
        slot = 0;
        for (int i = 0; i < kSatCache; ++i)
        {
            if (!cache_[i].valid)
            {
                slot = i;
                break;
            }
            if (cache_[i].used < cache_[slot].used)
                slot = i;
        }
        lite_[slot].atmo_k = -1;
        lite_[slot].sig_cn0 = -1.0f;
    }
    // as satState() solves it
    const double t_tx = t_sv - gps_sat_clock(&e, t_sv);
    double rs[3], rs1[3], rs2[3], dts, dts1, dts2;
    gps_sat_pos(&e, t_tx, rs, nullptr, &dts);
    gps_sat_pos(&e, t_tx + 0.5, rs2, nullptr, &dts2);
    gps_sat_pos(&e, t_tx - 0.5, rs1, nullptr, &dts1);
    SatState& c = cache_[slot];
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
    // its next solve: the first time after t_sv on the slot's own grid of
    // sat_refresh_s, so at most sat_refresh_s from now and on no other
    // slot's phase
    const double R = cfg_.sat_refresh_s;
    if (R > 0.0)
    {
        const double g = static_cast<double>(slot) * kGolden;
        const double ph = R * (g - std::floor(g));
        lite_[slot].t_next = (std::floor((t_sv + ph) / R) + 1.0) * R - ph;
    }
    return slot;
}

// The troposphere at the zenith over this epoch's receiver
// (tropo_saastamoinen's receiver part, the same arithmetic): once an epoch,
// when a satellite first needs it.
void RawMeasBuilder::zenithLite()
{
    const double h = hgt_;
    lite_trop_ok_ = cfg_.troposphere && !(h < -100.0 || h > 4e4);
    if (lite_trop_ok_)
    {
        const double hh = h < 0.0 ? 0.0 : h;
        const double p = 1013.25 * std::pow(1.0 - 2.2557e-5 * hh, 5.2568);
        const double tk = 15.0 - 6.5e-3 * hh + 273.16;
        double tw = tk, pw = 1.0;
        if (hh > 11000.0)
        {
            tw = 15.0 - 6.5e-3 * 11000.0 + 273.16;
            pw = p / (1013.25 * std::pow(1.0 - 2.2557e-5 * 11000.0, 5.2568));
        }
        const double ew = 6.108 * 0.7 * std::exp((17.15 * tw - 4684.0) / (tw - 38.45)) * pw;
        lite_dry_z_ = 0.0022768 * p / (1.0 - 0.00266 * std::cos(2.0 * lat_) - 0.00028 * hh / 1e3);
        lite_wet_z_ = 0.002277 * (1255.0 / tw + 0.05) * ew;
    }
    lite_zen_ = true;
}

// The satellite's elevation and, above the mask, its ionosphere (on its
// signal's carrier) and the cosine of its zenith angle, which maps the
// troposphere, at the epoch's receiver, into a -- as next() works them out
// (gnss_az_el and tropo_saastamoinen in the same arithmetic, iono_klobuchar
// as klobucharLite), with the receiver's trigonometry once an epoch.
void RawMeasBuilder::atmoLite(const GnssSignalProfile& sig, const double rs[3], double tow, LiteSat& a)
{
    if (!lite_geo_)
    {
        lite_sl_ = std::sin(lat_);
        lite_cl_ = std::cos(lat_);
        lite_so_ = std::sin(lon_);
        lite_co_ = std::cos(lon_);
        lite_geo_ = true;
    }
    const double d[3] = {rs[0] - r_approx_[0], rs[1] - r_approx_[1], rs[2] - r_approx_[2]};
    const double e = -lite_so_ * d[0] + lite_co_ * d[1];
    const double n = -lite_sl_ * lite_co_ * d[0] - lite_sl_ * lite_so_ * d[1] + lite_cl_ * d[2];
    const double u = lite_cl_ * lite_co_ * d[0] + lite_cl_ * lite_so_ * d[1] + lite_sl_ * d[2];
    a.el = std::asin(u / std::sqrt(e * e + n * n + u * u));
    a.atmo_k = lite_k_;
    a.atmo_gen = iono_gen_;
    a.iono = a.cz = 0.0;
    if (a.el < el_mask_)
        return;                           // under the mask: built from neither
    // the azimuth's cosine and sine from the direction's north and east, for
    // the ionosphere (iono_klobuchar takes the azimuth and turns it back)
    const double hz = std::sqrt(e * e + n * n);
    const double ca = hz > 0.0 ? n / hz : 1.0, sa = hz > 0.0 ? e / hz : 0.0;
    const double fr = GNSS_FREQ_L1_HZ / sig.carrier_hz;
    a.iono = klobucharLite(ca, sa, a.el, tow) * fr * fr;
    if (a.el > 0.0)
        a.cz = std::cos(kPi / 2.0 - a.el);
}

// iono_klobuchar (atmo.c: IS-GPS-200 20.3.3.5.2.5) at the epoch's receiver,
// from the azimuth's cosine and sine, with the obliquity's cube by
// multiplication: the same delay to a few parts in 10^16.
double RawMeasBuilder::klobucharLite(double ca, double sa, double el, double t) const
{
    const double phi_u = lat_ / kPi, lam_u = lon_ / kPi, e = el / kPi;
    const double psi = 0.0137 / (e + 0.11) - 0.022;
    double phi_i = phi_u + psi * ca;
    if (phi_i > 0.416)
        phi_i = 0.416;
    else if (phi_i < -0.416)
        phi_i = -0.416;
    const double lam_i = lam_u + psi * sa / std::cos(phi_i * kPi);
    const double phi_m = phi_i + 0.064 * std::cos((lam_i - 1.617) * kPi);
    double tl = std::fmod(4.32e4 * lam_i + t, 86400.0);
    if (tl < 0.0)
        tl += 86400.0;
    const double w = 0.53 - e;
    const double f = 1.0 + 16.0 * (w * w * w);
    const gps_iono_t& io = iono_;
    double amp = io.alpha[0] + phi_m * (io.alpha[1] + phi_m * (io.alpha[2] + phi_m * io.alpha[3]));
    double per = io.beta[0] + phi_m * (io.beta[1] + phi_m * (io.beta[2] + phi_m * io.beta[3]));
    if (amp < 0.0)
        amp = 0.0;
    if (per < 72000.0)
        per = 72000.0;
    const double x = 2.0 * kPi * (tl - 50400.0) / per;
    const double delay =
        (std::fabs(x) < 1.57) ? f * (5e-9 + amp * (1.0 - x * x / 2.0 + x * x * x * x / 24.0)) : f * 5e-9;
    return GNSS_C * delay;
}

// next() for the cheaper build (RawMeasBuilder.h).
int RawMeasBuilder::nextLite(const RawEpoch& ep, const EphemerisStore& eph, const gps_eph_t* const* eph_of,
                             RawMeas* out, int n, int cap, int max_obs)
{
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
        const gps_eph_t* e = eph_of != nullptr ? eph_of[i] : eph.select(o.sys, o.prn, ep.tow_s + to_sys);
        if (e == nullptr)
            continue;

        // transmit time: the travel time, then the satellite's own clock
        const double t_sv = ep.tow_s - o.pr_m / GNSS_C + to_sys;
        const int slot = satSlotLite(*e, o.sys, o.prn, t_sv);
        const SatState& c = cache_[slot];
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
            // the satellite's elevation and ionosphere as worked out in this
            // period, unless the ionosphere changed since or its carrier
            // phase is built on; the troposphere this epoch's receiver's,
            // mapped by the kept elevation (a climb changes it fastest)
            LiteSat& a = lite_[slot];
            if (!(cfg_.atmo_period_s > 0.0 && a.atmo_k == lite_k_ && a.atmo_gen == iono_gen_
                  && !(cfg_.carrier && o.has_phase)))
                atmoLite(sig, rs, ep.tow_s, a);
            if (a.el < el_mask_)
                continue;
            if (!lite_zen_)
                zenithLite();
            iono = a.iono;
            trop = lite_trop_ok_ && a.cz > 0.0 ? lite_dry_z_ / a.cz + lite_wet_z_ / a.cz : 0.0;
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
        // the noise: its C/N0 part as the satellite's last epoch had it where
        // its C/N0 is the same (three times in four at 10 Hz)
        LiteSat& a = lite_[slot];
        if (!(a.sig_cn0 == o.cn0_dbhz))
        {
            liteNoise(cfg_, static_cast<double>(o.cn0_dbhz), sig.carrier_hz, a.sig_pr, a.sig_corr, a.sig_rr2);
            a.sig_cn0 = o.cn0_dbhz;
        }
        m.sigma_pr = a.sig_pr;
        liteRelock(cfg_, relockAge(o.sys, o.prn, ep.tow_s), g_over_, a.sig_corr, a.sig_rr2, m.sigma_pr_corr,
                   m.sigma_rr);
        m.tau_pr = static_cast<float>(cfg_.pr_tau_s);
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
