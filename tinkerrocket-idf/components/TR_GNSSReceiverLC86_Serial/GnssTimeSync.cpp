// GnssTimeSync.cpp — see GnssTimeSync.h
#include "GnssTimeSync.h"

#include <algorithm>
#include <cmath>

namespace lc86
{

void GnssTimeSync::reset()
{
    have_ = false;
    head_ = 0;
    n_ = 0;
    bhead_ = 0;
    bn_ = 0;
    a_ = 0.0;
    drift_ = 0.0;
    t_ref_ = 0.0;
    env_rms_ms_ = 0.0;
    off_run_ = 0;
    late_n_ = 0;
    late_head_ = 0;
}

void GnssTimeSync::start(double gps_s, int64_t mcu_us)
{
    reset();
    gps0_ = std::floor(gps_s);
    mcu0_ = mcu_us;
    have_ = true;
}

bool GnssTimeSync::addSample(double gps_s, int64_t mcu_us)
{
    if (!have_)
        start(gps_s, mcu_us);
    double t = gps_s - gps0_;
    double d = static_cast<double>(mcu_us - mcu0_) * 1.0e-6 - t;

    // A sample far from the line: an outlier, or (several in a row, or time
    // running backwards) a step in either clock that restarts the history.
    const bool backwards = n_ > 0 && t < bins_[(head_ + kBins - 1) % kBins].t - 1.0;
    const bool off = valid() && std::fabs(d - line(t)) > cfg_.max_step_s;
    if (off && ++off_run_ < kStepCount && !backwards)
        return false;                                     // set aside
    bool kept = true;
    if (off || backwards)
    {
        restarts_++;
        start(gps_s, mcu_us);
        t = gps_s - gps0_;
        d = -t;
        kept = false;
    }
    else
    {
        off_run_ = 0;
    }
    samples_++;

    if (valid())
    {
        late_ms_[late_head_] = static_cast<float>((d - line(t)) * 1.0e3);
        late_head_ = (late_head_ + 1) % 32;
        late_n_ = std::min(late_n_ + 1, 32);
    }

    // the drift's 10 s blocks: a block completes when the next one starts
    const int64_t blk = static_cast<int64_t>(std::floor(t / kBlockS));
    Block* nb = bn_ > 0 ? &blocks_[(bhead_ + kBlocks - 1) % kBlocks] : nullptr;
    if (nb != nullptr && nb->blk == blk)
    {
        if (d < nb->d)
        {
            nb->d = d;
            nb->t = t;
        }
    }
    else
    {
        if (nb != nullptr)
            refitDrift();
        blocks_[bhead_] = Block{blk, t, d};
        bhead_ = (bhead_ + 1) % kBlocks;
        bn_ = std::min(bn_ + 1, kBlocks);
    }

    // the envelope's 1 s bins: a second completes when the next one starts
    const int64_t sec = static_cast<int64_t>(std::floor(t));
    Bin* newest = n_ > 0 ? &bins_[(head_ + kBins - 1) % kBins] : nullptr;
    if (newest != nullptr && newest->sec == sec)
    {
        if (d < newest->d)
        {
            newest->d = d;
            newest->t = t;
        }
        return kept;
    }
    if (newest != nullptr)
        refit();
    bins_[head_] = Bin{sec, t, d};
    head_ = (head_ + 1) % kBins;
    n_ = std::min(n_ + 1, kBins);
    if (n_ == 1)
    {
        a_ = d;
        t_ref_ = t;
    }
    return kept;
}

void GnssTimeSync::refitDrift()
{
    // Called as a new block starts, before it is filed: every block held is complete.
    const int m = bn_;
    if (m < 3)
        return;
    const int first = (bhead_ + kBlocks - bn_) % kBlocks;
    auto at = [&](int k) -> const Block& { return blocks_[(first + k) % kBlocks]; };
    const double span = at(m - 1).t - at(0).t;
    if (span < 2.0 * kBlockS)
        return;

    // A robust slope: the median of the slopes from each block of the first
    // half to its partner in the second half (one corrupt block spoils one).
    double sl[kBlocks / 2 + 1];
    int ns = 0;
    const int half = m / 2;
    for (int k = 0; k + half < m && ns < kBlocks / 2 + 1; ++k)
    {
        const double dt = at(k + half).t - at(k).t;
        if (dt > 0.5 * kBlockS)
            sl[ns++] = (at(k + half).d - at(k).d) / dt;
    }
    if (ns == 0)
        return;
    std::nth_element(sl, sl + ns / 2, sl + ns);
    double b = sl[ns / 2];

    // Least squares over the blocks within 3 sigma of it (MAD), then shrink
    // toward zero by the slope's own uncertainty against the crystal prior.
    double r[kBlocks];
    const double tm = 0.5 * (at(0).t + at(m - 1).t);
    for (int k = 0; k < m; ++k)
        r[k] = at(k).d - b * (at(k).t - tm);
    double rs[kBlocks];
    std::copy(r, r + m, rs);
    std::nth_element(rs, rs + m / 2, rs + m);
    const double med = rs[m / 2];
    for (int k = 0; k < m; ++k)
        rs[k] = std::fabs(r[k] - med);
    std::nth_element(rs, rs + m / 2, rs + m);
    const double sigma = std::max(5.0e-5, 1.4826 * rs[m / 2]);   // >= 50 us: no block is that exact
    double st = 0, sd = 0, stt = 0, std_ = 0;
    int q = 0;
    for (int k = 0; k < m; ++k)
    {
        if (std::fabs(r[k] - med) > 3.0 * sigma)
            continue;
        const double x = at(k).t - tm;
        st += x;
        sd += at(k).d;
        stt += x * x;
        std_ += x * at(k).d;
        q++;
    }
    if (q >= 3)
    {
        const double den = q * stt - st * st;
        if (den > 0.0)
            b = (q * std_ - st * sd) / den;
    }
    const double sigma_b = sigma * std::sqrt(12.0 / q) / span;
    const double w = kDriftPrior * kDriftPrior / (kDriftPrior * kDriftPrior + sigma_b * sigma_b);
    drift_ = std::max(-kMaxDrift, std::min(kMaxDrift, w * b));
}

void GnssTimeSync::refit()
{
    if (n_ < 1)
        return;
    const int first = (head_ + kBins - n_) % kBins;
    auto at = [&](int k) -> const Bin& { return bins_[(first + k) % kBins]; };
    const double t_new = at(n_ - 1).t;

    // The anchor: the kAnchorRank-th lowest second about the drift's slope.
    double r[kBins];
    for (int k = 0; k < n_; ++k)
        r[k] = at(k).d - drift_ * (at(k).t - t_new);
    const int rank = std::min(kAnchorRank, n_) - 1;
    std::nth_element(r, r + rank, r + n_);
    const double a = r[rank];

    // How tightly the seconds near the envelope sit about it.
    double ss = 0.0;
    int q = 0;
    for (int k = 0; k < n_; ++k)
    {
        const double e = at(k).d - (a + drift_ * (at(k).t - t_new));
        if (std::fabs(e) <= kNearS)
        {
            ss += e * e;
            q++;
        }
    }
    env_rms_ms_ = q > 0 ? std::sqrt(ss / q) * 1.0e3 : 0.0;

    a_ = a;
    t_ref_ = t_new;
}

bool GnssTimeSync::valid() const
{
    if (!have_ || n_ < 2)
        return false;
    const int first = (head_ + kBins - n_) % kBins;
    const int last = (head_ + kBins - 1) % kBins;
    return bins_[last].t - bins_[first].t >= kMinSpanS;
}

bool GnssTimeSync::mcuAt(double gps_s, int64_t& mcu_us) const
{
    if (!valid())
        return false;
    const double t = gps_s - gps0_;
    const double rel = t + line(t) - cfg_.latency_s;      // MCU s from mcu0_
    mcu_us = mcu0_ + static_cast<int64_t>(std::llround(rel * 1.0e6));
    return true;
}

bool GnssTimeSync::gpsAt(int64_t mcu_us, double& gps_s) const
{
    if (!valid())
        return false;
    // mcu_rel = t + a + drift (t - t_ref) - latency  ->  t
    const double mcu_rel = static_cast<double>(mcu_us - mcu0_) * 1.0e-6;
    const double t = (mcu_rel - a_ + drift_ * t_ref_ + cfg_.latency_s) / (1.0 + drift_);
    gps_s = gps0_ + t;
    return true;
}

double GnssTimeSync::lateness90Ms() const
{
    if (late_n_ == 0)
        return 0.0;
    float v[32];
    for (int k = 0; k < late_n_; ++k)
        v[k] = late_ms_[k];
    const int k90 = std::min(late_n_ - 1, (late_n_ * 9) / 10);
    std::nth_element(v, v + k90, v + late_n_);
    return v[k90];
}

}  // namespace lc86
