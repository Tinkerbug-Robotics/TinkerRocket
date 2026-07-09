// Pure decision logic for admitting barometer samples to the EKF measurement
// update (issue #450).  No ESP-IDF dependencies so it is host-testable.
//
// Solves two problems with the previous inline gate in main.cpp:
//
// 1. Sample drop (#450): the old gate keyed on the loop-lifetime flag
//    bmp_new_for_kf, which is cleared at the end of EVERY loop iteration while
//    the EKF only runs every EKF_DECIMATION-th iteration.  A sample read on an
//    EKF-off iteration was wiped before any EKF tick saw it.  With the BMP ODR
//    (487 Hz) beating against the decimated EKF rate (959/2 = 479.5 Hz) the
//    arrival parity drifts continuously, so the EKF baro path alternated
//    ~66 ms dead / ~66 ms live for the whole flight — the EKF silently lost
//    ~50% of baro measurements.  This policy instead tracks the last CONSUMED
//    sample timestamp, so freshness survives EKF-off iterations and every
//    accepted BMP sample reaches exactly one EKF tick (only samples overwritten
//    in bmp_latest_si between two EKF ticks are lost: ~1.5% at these rates).
//
// 2. Rate-blind spike test: the old test rejected any |delta| > 5 m between
//    consecutive compared samples regardless of how far apart in time they
//    were, so a legitimate gap (the very dead windows above, or a future
//    lockout/outage) at >75 m/s read as a "spike" and the recovery sample was
//    discarded too.  A spike now additionally requires the APPARENT RATE
//    (delta / actual pair spacing) to exceed a limit far above any subsonic
//    flight profile.  Glitches remain caught: an in-band corrupted sample or
//    ejection-charge transient at ~2 ms spacing shows km/s-scale apparent rate.
#pragma once

#include <cmath>
#include <cstdint>

class BaroGatePolicy
{
public:
    struct Decision
    {
        bool  fresh  = false;  // sample not yet seen by the EKF path
        bool  spike  = false;  // fresh but rejected as a spike
        bool  accept = false;  // fresh && !spike && !locked -> run baroMeasUpdate
        float delta_m = 0.0f;             // |alt - prev alt| (fresh only)
        float apparent_rate_mps = 0.0f;   // delta / actual pair spacing
    };

    // Evaluate the latest available sample on an EKF tick.  Call with the same
    // sample repeatedly (EKF ticks between BMP arrivals) is fine: consume-on-use
    // makes the repeats !fresh no-ops.  `locked` is the transonic lockout: the
    // sample still updates the internal reference (so unlock does not produce a
    // false delta) but is not accepted for the measurement update.
    //
    // `min_fuse_interval_us` throttles how often a sample is ACCEPTED for the
    // EKF measurement update (0 disables).  Fusing baro at the full ~487 Hz ODR
    // buys nothing — altitude dynamics are slow against the sample rate — but
    // each fusion pays the EKF's covariance update, which tripled the per-tick
    // EKF cost (554 us -> ~1650 us) and cut the flight loop from ~960/s to
    // ~715/s when the #450 fix restored every-sample fusion.  Spike/freshness
    // tracking still sees EVERY sample (references advance per sample, so the
    // spike compare stays a consecutive-sample test); only the accept output
    // is rate-limited, and the interval restarts from the last FUSED sample.
    Decision evaluate(uint32_t sample_time_us,
                      float    altitude_m,
                      bool     locked,
                      float    spike_thresh_m,
                      float    spike_rate_mps,
                      uint32_t min_fuse_interval_us)
    {
        Decision d;
        d.fresh = !have_consumed_ || (sample_time_us != last_time_us_);
        if (!d.fresh)
        {
            return d;
        }

        if (have_consumed_)
        {
            d.delta_m = fabsf(altitude_m - prev_alt_m_);
            // Pair spacing from the sample's own timestamps; unsigned
            // subtraction is wrap-safe.  Floor keeps the rate finite on a
            // duplicate/garbage timestamp.
            uint32_t dt_us = sample_time_us - last_time_us_;
            if (dt_us < kMinPairDtUs)
            {
                dt_us = kMinPairDtUs;
            }
            d.apparent_rate_mps = d.delta_m / ((float)dt_us * 1e-6f);
            d.spike = (d.delta_m > spike_thresh_m) &&
                      (d.apparent_rate_mps > spike_rate_mps);
        }

        // Always advance the reference — even on spike, lockout or throttle —
        // so one bad sample (or a lockout window) cannot poison every
        // following compare.
        last_time_us_  = sample_time_us;
        prev_alt_m_    = altitude_m;
        have_consumed_ = true;

        // Fusion throttle: wrap-safe elapsed-time check against the last
        // ACCEPTED sample.  The first-ever sample always passes.
        const bool throttled =
            (min_fuse_interval_us > 0u) && have_fused_ &&
            (sample_time_us - last_fused_time_us_) < min_fuse_interval_us;

        d.accept = !d.spike && !locked && !throttled;
        if (d.accept)
        {
            last_fused_time_us_ = sample_time_us;
            have_fused_ = true;
        }
        return d;
    }

private:
    static constexpr uint32_t kMinPairDtUs = 100u;

    uint32_t last_time_us_       = 0;
    float    prev_alt_m_         = 0.0f;
    bool     have_consumed_      = false;
    uint32_t last_fused_time_us_ = 0;
    bool     have_fused_         = false;
};
