// EphemerisStore.cpp — see EphemerisStore.h
#include "EphemerisStore.h"

#include <cmath>
#include <cstring>

namespace gnss_raw
{

void EphemerisStore::clear()
{
    std::memset(pool_, 0, sizeof(pool_));
    used_ = 0;
}

bool EphemerisStore::add(const gps_eph_t& e)
{
    if (!e.valid)
        return false;
    size_t mine[kCapacity];
    size_t n_mine = 0;
    for (size_t i = 0; i < used_; ++i)
    {
        const gps_eph_t& h = pool_[i];
        if (h.sys != e.sys || h.prn != e.prn)
            continue;
        if (h.iode == e.iode && h.toe == e.toe)
            return false;                                  // already held
        mine[n_mine++] = i;
    }
    size_t slot = used_;
    if (n_mine >= static_cast<size_t>(kPerSat))
    {
        // replace this satellite's oldest (by toe, across the week boundary)
        slot = mine[0];
        for (size_t k = 1; k < n_mine; ++k)
            if (gps_time_diff(pool_[mine[k]].toe, pool_[slot].toe) < 0.0)
                slot = mine[k];
        if (gps_time_diff(e.toe, pool_[slot].toe) < 0.0)
            return false;                                  // older than both held
    }
    else if (used_ == kCapacity)
    {
        // pool full: drop the ephemeris whose toe is furthest from this one's
        slot = 0;
        double worst = -1.0;
        for (size_t i = 0; i < used_; ++i)
        {
            const double d = std::fabs(gps_time_diff(e.toe, pool_[i].toe));
            if (d > worst)
            {
                worst = d;
                slot = i;
            }
        }
    }
    else
    {
        ++used_;
    }
    pool_[slot] = e;
    return true;
}

const gps_eph_t* EphemerisStore::select(int sys, int prn, double t) const
{
    const gps_eph_t* best = nullptr;
    double best_dt = kMaxAgeS;
    for (size_t i = 0; i < used_; ++i)
    {
        const gps_eph_t& h = pool_[i];
        if (h.sys != sys || h.prn != prn)
            continue;
        const double dt = std::fabs(gps_time_diff(t, h.toe));
        if (dt < best_dt)
        {
            best_dt = dt;
            best = &h;
        }
    }
    return (best != nullptr && best->health == 0) ? best : nullptr;
}

int EphemerisStore::satellites(int sys) const
{
    int n = 0;
    for (size_t i = 0; i < used_; ++i)
    {
        if (pool_[i].sys != sys)
            continue;
        bool first = true;
        for (size_t j = 0; j < i; ++j)
            if (pool_[j].sys == sys && pool_[j].prn == pool_[i].prn)
                first = false;
        n += first ? 1 : 0;
    }
    return n;
}

}  // namespace gnss_raw
