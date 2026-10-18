// RawGnssEKFUpdate.h -- TR_RawGnssEKF's epoch units, beginGnssRaw and
// updateGnssRawPart (TR_RawGnssEKF.h), the one source of both their forms:
// kShare false, today's, instantiated by TR_RawGnssEKF.cpp (each update, in
// internal RAM on the Beetle), and kShare true, with the epoch's geometry
// shared by its units (RawGnssEKFParams::share_los), instantiated by
// TR_RawGnssCarrier.cpp with the rest of that geometry: in flash, so the
// Beetle, which never turns share_los on, keeps none of it in internal RAM
// (raw_gnss_iram.lf). The filter's own beginGnssRaw and updateGnssRawPart
// take one or the other as share_los stands. Internal: not part of the
// filter's interface; those two sources include it, each instantiating its
// own form (TR_RawGnssEKF.cpp declares the other extern).
#ifndef TR_RAW_GNSS_EKF_UPDATE_H
#define TR_RAW_GNSS_EKF_UPDATE_H

#include <cmath>

#include "RawGnssEKFGeometry.h"
#include "TR_RawGnssEKF.h"
#include "gnss/types.h"

namespace gnss_raw
{
namespace ekf_update
{
namespace
{

// numpy's median: the mean of the two middle values for an even count.
double median(double* v, int n)
{
    for (int i = 1; i < n; ++i)
    {
        const double x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x)
        {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = x;
    }
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace
}  // namespace ekf_update

template <typename T>
template <bool kShare>
bool RawGnssEKF<T>::beginGnssRawAs(const RawMeas* m, int n, double age_s, RawGnssStats& st)
{
    if (age_s < 0.0)
        age_s = 0.0;
    if (!clk_ready_ && !initClockFrom(m, n, age_s))
        return false;

    // the receiver's history back to the epoch
    T dp_e[3], dv_e[3];
    historyChange(age_s, dv_e, dp_e);

    // Receiver clock steps: every satellite's innovation agrees on one
    {
        double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
        if constexpr (kShare)
            frameOf(r, v, Tm);
        else
            receiverState(r, v, Tm);
        ekf_geom::backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
        double inn[RawEpoch::kMaxObs], inn_rr[RawEpoch::kMaxObs], dev[RawEpoch::kMaxObs];
        int ni = 0, nr = 0;
        const double b_e = clk_b_ - static_cast<double>(clk_d_) * age_s;
        for (int i = 0; i < n; ++i)
        {
            if (m[i].sys > GNSS_SYS_BDS)
                continue;
            ekf_geom::Los L;
            if constexpr (kShare)
            {
                // the epoch's (TR_RawGnssEKF.h share_los), the rate as lineOfSight forms it
                L.rho = losOf(m, n, i, t_ - age_s, r_e, L.u, L.vs);
                L.rate = L.u[0] * (L.vs[0] - v_e[0]) + L.u[1] * (L.vs[1] - v_e[1]) + L.u[2] * (L.vs[2] - v_e[2]);
            }
            else
                ekf_geom::lineOfSight(m[i], r_e, v_e, L);
            if (m[i].has_pr && ni < RawEpoch::kMaxObs)
                inn[ni++] = m[i].pr - (L.rho + b_e + static_cast<double>(isb(m[i].sys)));
            if (m[i].has_rr && nr < RawEpoch::kMaxObs)
                inn_rr[nr++] = m[i].rr - (L.rate + static_cast<double>(clk_d_));
        }
        if (ni > 0)
        {
            for (int i = 0; i < ni; ++i)
                dev[i] = inn[i];
            const double med = ekf_update::median(dev, ni);
            const double ms = ekf_geom::kC * 1e-3;
            const long k = std::lround(med / ms);
            if (k != 0 && std::fabs(med - static_cast<double>(k) * ms) < 0.1 * ms)
            {
                clk_b_ += static_cast<double>(k) * ms;
                st.clock_jump_ms = static_cast<int>(k);
            }
            else if (ni >= 4 && std::fabs(med) > p_.clock_step_m)
            {
                for (int i = 0; i < ni; ++i)
                    dev[i] = std::fabs(inn[i] - med);
                if (ekf_update::median(dev, ni) < p_.clock_step_spread_m)
                {
                    clk_b_ += med;
                    st.clock_step_m = med;
                }
            }
        }
        if (nr >= 4)
        {
            for (int i = 0; i < nr; ++i)
                dev[i] = inn_rr[i];
            const double med = ekf_update::median(dev, nr);
            if (std::fabs(med) > p_.drift_step_mps)
            {
                for (int i = 0; i < nr; ++i)
                    dev[i] = std::fabs(inn_rr[i] - med);
                if (ekf_update::median(dev, nr) < p_.drift_step_spread_mps)
                {
                    clk_d_ += T(med);
                    st.drift_step_mps = med;
                }
            }
        }
    }
    // The carrier's receiver at this epoch: back-dated as the pseudoranges
    // take it, before they correct it (each correction from here moves it)
    if (p_.cr_mode != kCarrierOff)
        crSnapshot(age_s, dp_e);
    // hdg_obs_mode's share of the heading, once for the whole epoch: its parts
    // fused over later steps take the same
    if (p_.hdg_obs_mode != kHdgObsOff)
        hdg_obs_s_ = hdgObsScaleNow(age_s);
    return true;
}

template <typename T>
template <bool kShare>
void RawGnssEKF<T>::updateGnssRawPartAs(const RawMeas* m, int i0, int i1, double age_s, RawGnssStats& st)
{
    if (age_s < 0.0)
        age_s = 0.0;
    // the receiver's history back to the epoch, and to its Doppler's instant,
    // as it stands now: a part fused later takes the epoch back further (the
    // instant: rr_lag_s before, and on an epoch in pairs rr_pair_lag_s more)
    const double tau = rrLag();
    // rr_float's Doppler prediction (TR_RawGnssEKF.h): share_los's form only,
    // and only where the Doppler has no lag (an epoch in pairs has: it takes
    // the double prediction, the pair's lag and pairAccel, as with rr_float off)
    const bool rr_float = kShare && p_.rr_float && tau <= 0.0;
    T dp_e[3], dv_e[3], dp_l[3], dv_l[3];
    historyChange(age_s, dv_e, dp_e);
    historyChange(age_s + tau, dv_l, dp_l);
    // lever_on: the antenna, not the IMU, at the epoch and the Doppler's instant
    LeverEpoch lv;
    if (p_.lever_on)
        leverBack(age_s, tau, dp_e, dv_e, dv_l, lv);

    const T att = attGate();
    T vert[3];                            // the vertical in the body frame (att_gate_vertical)
    const T* axis = gnssAttAxis(vert);
    // att_gnss_off: the attitude and the gyro bias kept out of every GNSS row
    // here, and only here (scalarUpdate's other callers -- the levelling, the
    // height, zero velocity, the magnetometer -- keep theirs). The
    // accelerometer bias keeps its gain (RawGnssEKFParams::att_gnss_off says
    // what that leaves). The hold (setAttHold) is the same mask for a while.
    const uint32_t kmask =
        p_.att_gnss_off || att_hold_ ? ~((0x7u << kAtt) | (0x7u << kGyroBias)) : 0xFFFFFFFFu;
    // hdg_obs_mode: the rows' heading part -- the attitude about the vertical,
    // the gyro bias along it -- scaled by the epoch's s (beginGnssRaw); none
    // to scale with att_gnss_off, and none with s 1 (the mode off: today's)
    const bool hdg_scaled = p_.hdg_obs_mode != kHdgObsOff && !p_.att_gnss_off && hdg_obs_s_ < T(1);
    T hvert[3];
    if (hdg_scaled)
        gnssAttAxis(hvert);
    const T* hdg_axis = hdg_scaled ? hvert : nullptr;
    for (int i = i0; i < i1; ++i)
    {
        const RawMeas& mi = m[i];
        if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64)
            continue;
        if ((!mi.has_pr || !fuse_pr_) && !mi.has_rr)
            continue;
        // The geometry once per satellite, at the state every satellite
        // before it left (the prototype redoes it per measurement too, but
        // this satellite's own pseudorange update moves the receiver by
        // metres at most, turning its line of sight by ~1e-7 rad).
        double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
        ekf_geom::Los L;
        if constexpr (kShare)
        {
            // the epoch's line of sight (TR_RawGnssEKF.h share_los) at the
            // receiver back-dated as below, the frame kept: no velocity here
            // (its rate is the Doppler's own, below)
            frameOf(r, nullptr, Tm);
            for (int a = 0; a < 3; ++a)
            {
                double sp = 0.0;
                for (int k = 0; k < 3; ++k)
                    sp += Tm[k][a] * static_cast<double>(dp_e[k]);
                r_e[a] = r[a] - sp;
            }
            L.rho = losOf(m, 0, i, t_ - age_s, r_e, L.u, L.vs);
            L.rate = 0.0;
        }
        else
        {
            receiverState(r, v, Tm);
            ekf_geom::backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
            ekf_geom::lineOfSight(mi, r_e, v_e, L);
        }
        T u_ned[3];
        for (int a = 0; a < 3; ++a)
            u_ned[a] = T(Tm[a][0] * L.u[0] + Tm[a][1] * L.u[1] + Tm[a][2] * L.u[2]);
        for (int kind = 0; kind < 2; ++kind)
        {
            if (kind == 0 ? (!mi.has_pr || !fuse_pr_) : !mi.has_rr)
                continue;
            double pred_rate = 0.0;
            if constexpr (kShare)
            {
                if (kind == 1 && rr_float)
                {
                    // rr_float (TR_RawGnssEKF.h): the receiver's part in NED, float
                    T vl[3];
                    for (int a = 0; a < 3; ++a)
                        vl[a] = v_[a] - dv_e[a];
                    pred_rate = rrFloat(L.u[0] * L.vs[0] + L.u[1] * L.vs[1] + L.u[2] * L.vs[2], u_ned, vl);
                }
            }
            if (kind == 1 && !rr_float)
            {
                // the receiver's velocity as the pseudorange update left it,
                // taken back to the Doppler's instant (a late Doppler: tau
                // before the epoch, the satellite's own range acceleration
                // backed out -- on an epoch in pairs as its first consumer
                // took it, pairAccel)
                double vr[3], v_l[3];
                for (int a = 0; a < 3; ++a)
                    vr[a] = Tm[0][a] * static_cast<double>(v_[0]) + Tm[1][a] * static_cast<double>(v_[1])
                          + Tm[2][a] * static_cast<double>(v_[2]);
                const T* dv = tau > 0.0 ? dv_l : dv_e;
                for (int a = 0; a < 3; ++a)
                    v_l[a] = vr[a] - (Tm[0][a] * static_cast<double>(dv[0]) + Tm[1][a] * static_cast<double>(dv[1])
                                      + Tm[2][a] * static_cast<double>(dv[2]));
                ekf_geom::Los Lr = L;
                Lr.rate = 0.0;
                for (int a = 0; a < 3; ++a)
                    Lr.rate += L.u[a] * (L.vs[a] - v_l[a]);
                pred_rate = tau > 0.0 ? Lr.rate - tau * (rr_paired_ ? pairAccel(i, mi, Lr, v_l)
                                                                    : ekf_geom::satRangeAccel(mi, Lr, v_l))
                                      : Lr.rate;
            }

            int hidx[10];
            T hval[10];
            int hn = 0;
            T y, R;
            const double dt_i = t_ - last_upd_[mi.sys][mi.prn][kind];
            if (kind == 0)
            {
                for (int a = 0; a < 3; ++a)
                {
                    hidx[hn] = kPos + a;
                    hval[hn++] = -u_ned[a];
                }
                hidx[hn] = kClkBias;
                hval[hn++] = T(1);
                if (mi.sys == GNSS_SYS_GAL || mi.sys == GNSS_SYS_BDS)
                {
                    hidx[hn] = mi.sys == GNSS_SYS_GAL ? kIsbGal : kIsbBds;
                    hval[hn++] = T(1);
                }
                if (age_s > 0.0)
                {
                    // the position and clock bias then: now less velocity and drift times age
                    for (int a = 0; a < 3; ++a)
                    {
                        hidx[hn] = kVel + a;
                        hval[hn++] = T(age_s) * u_ned[a];
                    }
                    hidx[hn] = kClkDrift;
                    hval[hn++] = T(-age_s);
                }
                const double b_e = clk_b_ - static_cast<double>(clk_d_) * age_s;
                y = T(mi.pr - (L.rho + b_e + static_cast<double>(isb(mi.sys))));
                const double s_w = static_cast<double>(mi.sigma_pr), s_c = static_cast<double>(mi.sigma_pr_corr);
                R = T(s_w * s_w + s_c * s_c * ekf_geom::corrInflation(dt_i, static_cast<double>(mi.tau_pr)));
            }
            else
            {
                for (int a = 0; a < 3; ++a)
                {
                    hidx[hn] = kVel + a;
                    hval[hn++] = -u_ned[a];
                }
                hidx[hn] = kClkDrift;
                hval[hn++] = T(1);
                y = T(mi.rr - (pred_rate + static_cast<double>(clk_d_)));
                const double s_r = static_cast<double>(mi.sigma_rr);
                R = T(s_r * s_r * ekf_geom::corrInflation(dt_i, static_cast<double>(mi.tau_rr)));
                if (p_.lever_on)
                    hn = leverRows(u_ned, lv, hidx, hval, hn, R);
            }

            uint8_t& rej = rejects_[mi.sys][mi.prn][kind];
            T gate = T(p_.gate_chi2);
            if (rej >= rejectPersist(kind))
            {
                // persistent disagreement: the filter is the suspect, so let
                // it in, its variance scaled by its NIS over the gate (the
                // prototype's rule: on R alone, it would just have passed)
                gate = T(0);
                const T nis0 = y * y / (hph(hidx, hval, hn) + R);
                if (nis0 > T(p_.gate_chi2))
                    R *= nis0 / T(p_.gate_chi2);
            }
            T dx[N];
            T nis = T(0);
            const bool ok =
                scalarUpdate(hidx, hval, hn, y, R, gate, att, dx, &nis, kmask, axis, hdg_axis, hdg_obs_s_) == 1;
            if (ok)
            {
                inject(dx);
                stabilize();
                last_upd_[mi.sys][mi.prn][kind] = t_;
                rej = 0;
                if (static_cast<float>(nis) > st.nis_max)
                    st.nis_max = static_cast<float>(nis);
                if (kind == 0)
                    st.used_pr++;
                else
                    st.used_rr++;
            }
            else
            {
                if (rej < 255)
                    rej++;
                if (kind == 0)
                    st.rejected_pr++;
                else
                    st.rejected_rr++;
            }
        }
    }
}

}  // namespace gnss_raw

#endif  // TR_RAW_GNSS_EKF_UPDATE_H
