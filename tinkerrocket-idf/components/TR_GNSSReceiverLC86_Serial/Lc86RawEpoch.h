// Lc86RawEpoch.h — the LC86G's MSM7 epoch as the receiver-neutral
// observations TR_GNSSRaw builds a raw-GNSS filter's measurements from (#1600)
//
// One cell per satellite on this module: GPS L1 C/A, Galileo E1 and BeiDou
// B1I (MSM signal slot 2) are kept; GLONASS and QZSS are left out, as the
// bench prototype leaves them (gnss/eph.h has no GLONASS orbit integration
// and no QZSS constants). The phase-range rate is the receiver's range rate
// with its own clock drift already taken out (Rtcm3Msm.h); the filter's
// clock-drift state then sits near zero. The carrier phase is the MSM7
// phase range (DF397 + DF398 + DF406), with its half-cycle flag (DF420) and
// the cell's lock time. Header-only, pure.
#ifndef LC86_RAW_EPOCH_H
#define LC86_RAW_EPOCH_H

#include <cmath>

#include "RawMeas.h"
#include "Rtcm3Msm.h"
#include "gnss/types.h"

namespace lc86
{

// False, with out.n = 0, if the epoch carries no GPS time (GLONASS alone).
inline bool rawEpochFromMsm(const rtcm3::MsmEpoch& in, gnss_raw::RawEpoch& out)
{
    out.n = 0;
    if (in.gps_tow_ms == rtcm3::kInvalidTow)
        return false;
    out.tow_s = in.gps_tow_ms * 1.0e-3;
    for (int i = 0; i < in.n_cells && out.n < gnss_raw::RawEpoch::kMaxObs; ++i)
    {
        const rtcm3::MsmCell& c = in.cells[i];
        uint8_t sys;
        switch (c.gnss_id)                       // UBX gnssId (Rtcm3Msm.h)
        {
            case 0: sys = GNSS_SYS_GPS; break;
            case 2: sys = GNSS_SYS_GAL; break;
            case 3: sys = GNSS_SYS_BDS; break;
            default: continue;
        }
        if (c.sig_id != 2)
            continue;
        gnss_raw::RawObs& o = out.obs[out.n++];
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
    }
    return true;
}

// An MSM7 epoch's continuous GPS time (s: week * 604800 + time of week).
// MSM7 states only the time of week; the week is the one of the nearest
// $PQTMPVT (the parser's pvtGpsWeek/pvtTowMs), moved across the week's end
// when the two times of week straddle it.
inline double epochGpsSeconds(uint32_t tow_ms, uint16_t pvt_week, uint32_t pvt_tow_ms)
{
    double t = pvt_week * 604800.0 + tow_ms * 1.0e-3;
    const double ref = pvt_week * 604800.0 + pvt_tow_ms * 1.0e-3;
    if (t - ref > 302400.0)
        t -= 604800.0;
    else if (t - ref < -302400.0)
        t += 604800.0;
    return t;
}

}  // namespace lc86

#endif  // LC86_RAW_EPOCH_H
