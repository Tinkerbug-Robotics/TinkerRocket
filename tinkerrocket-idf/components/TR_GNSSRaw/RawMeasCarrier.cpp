// RawMeasCarrier.cpp — RawMeasBuilder's carrier range rate (see
// RawMeasBuilder.h): each satellite's end of the epoch kept, and its rate
// since the last epoch's end when the receiver profile's gates pass at both.
// Apart from the rest of the builder so that on the Beetle, where the
// builder runs from internal RAM (raw_gnss_iram.lf), this stays in flash.
#include "RawMeasBuilder.h"

#include <cmath>

#include "gnss/atmo.h"

namespace gnss_raw
{

// The satellite's end of this epoch kept, and its carrier range rate since
// the last epoch's end into m when the gates pass at both (RawMeasBuilder.h).
void RawMeasBuilder::carrierRate(const RawObs& o, const GnssSignalProfile& sig, double tow, double dts, double iono,
                                 double trop, const double rs[3], RawMeas& m)
{
    m.has_cr = false;
    m.sigma_cr = 0.0f;
    m.cr_dt = 0.0f;
    m.cr_dphi_m = 0.0;
    for (int k = 0; k < 3; ++k)
        m.sat_pos_prev[k] = 0.0;
    if (!cfg_.carrier || !o.has_phase)
        return;
    // the phase range corrected as the code is, but for the ionosphere's sign
    // (it advances the phase)
    const double phase = o.phase_m + (GNSS_C * dts + iono - trop);
    const float sigma = phaseSigmaM(sig, o.cn0_dbhz);
    int& ncur = cr_n_[cr_side_ ^ 1];
    if (ncur < RawEpoch::kMaxObs)
    {
        CarrierEnd& e = cr_end_[cr_side_ ^ 1][ncur++];
        e.sys = o.sys;
        e.prn = o.prn;
        e.halfcyc = o.halfcyc;
        e.model = cr_model_;
        e.cn0 = o.cn0_dbhz;
        e.sigma_m = sigma;
        e.lock_ms = o.lock_ms;
        e.tow = tow;
        e.phase = phase;
        for (int k = 0; k < 3; ++k)
            e.sat_pos[k] = rs[k];
    }
    const CarrierEnd* p = nullptr;
    for (int i = 0; i < cr_n_[cr_side_]; ++i)
    {
        const CarrierEnd& e = cr_end_[cr_side_][i];
        if (e.sys == o.sys && e.prn == o.prn)
        {
            p = &e;
            break;
        }
    }
    if (p == nullptr)
        return;                           // not in the last epoch: it starts over
    const double dt = gps_time_diff(tow, p->tow);
    if (!(dt > 0.5 * cfg_.epoch_period_s && dt < cfg_.cr_max_interval_s))
        return;                           // not the epoch before
    if (o.halfcyc || p->halfcyc || p->model != cr_model_)
        return;
    if (o.cn0_dbhz < sig.cn0_min_cr || p->cn0 < sig.cn0_min_cr)
        return;
    // the lock: not gone back, held long enough at the earlier end, and the
    // later end's older than the interval (no relock between)
    if (o.lock_ms < p->lock_ms || p->lock_ms < cfg_.cr_min_lock_ms || static_cast<double>(o.lock_ms) < dt * 1000.0)
        return;
    m.has_cr = true;
    m.cr_dt = static_cast<float>(dt);
    m.cr_dphi_m = phase - p->phase;
    for (int k = 0; k < 3; ++k)
        m.sat_pos_prev[k] = p->sat_pos[k];
    const double sa = static_cast<double>(sigma), sb = static_cast<double>(p->sigma_m);
    const double s = std::sqrt(sa * sa + sb * sb) / dt;
    m.sigma_cr = static_cast<float>(s > cfg_.cr_sigma_floor_mps ? s : cfg_.cr_sigma_floor_mps);
}

}  // namespace gnss_raw
