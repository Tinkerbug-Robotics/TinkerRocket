// CapGovernor.h — the satellite cap that follows the filter's slack
// (exp/v9-cap-adapt): how many satellites the GNSS task's stage builds an
// epoch (RawGnssMeasStage's cap), between a floor and a ceiling, by how far
// the navigation filter falls behind its epochs. Off unless a ceiling above
// the floor is given (the receiver profile's sat_cap_max, 0 on every
// profile): the cap is then the floor, fixed, as it always was.
//
// What it judges. Each epoch the filter closes, its age at the close: the
// time from its GPS instant to the end of its fusion, which takes in the
// receiver's delay, the build in the GNSS task, the wait for the IMU and the
// filter's slices (RawGnssShadow's epoch_slice_us a step). An epoch that
// reaches hard_age_s (the profile's epoch_job_max_s, 0.2 s on the mosaic)
// is worked at once, in one slice that holds the published state up. On the
// open-sky bench (boost screen on every epoch) 16 satellites closed at a
// median 109 ms and at most 149, 24 at 146 and 190: about 5 ms more a
// satellite, and 24 within 10 ms of working an epoch at once.
//
// The rules, each epoch closed:
//   - an epoch worked at once, closed hard_age_s old or more, or lost on the
//     way (dropped as stale, or a gap in the epochs the filter was handed:
//     the handoff or the driver's queue full) puts the cap back to the floor;
//   - cut_count of the last cut_window closes (two of four) at cut_frac of
//     hard_age_s old or more (0.16 s) cut it by cut_step: a lag that stays,
//     not one epoch's spike (the 5-s orbit refresh builds every satellite's
//     orbit in one epoch, 15-25 ms more; a spike that reaches hard_age_s is
//     the rule above's);
//   - raise_epochs closed in a row at most raise_frac of it old (0.12 s, a
//     second of epochs at 10 Hz) raise it by one.
// The band between the raise and the cut (40 ms, some eight satellites'
// worth) is the hysteresis; and a cap that was cut from is not raised into
// again for backoff_s, doubled each time a retry of it is cut (up to
// backoff_max_s), so a sky whose cost sits at a cap's edge probes it now and
// then, not every second. The settle_epochs closes after a change were built
// before it: they neither raise nor cut softly (a hard one still floors).
//
// The boost. From the boost's first sign -- the launch, the IMU past the
// boost screen's threshold, an epoch the boost screen judges -- to
// boost_hold_s after its last, the cap is the floor: that is when the CPU is
// tightest (the screen's scratch copy doubles the fusion) and the boost's
// Dopplers are the least trusted. The climb back starts after it, a satellite
// a raise_epochs.
//
// Only the count changes: the stage's picks keep their 3 dB hold
// (RawGnssMeasStage::capEpoch), so a raise adds the strongest satellite left
// out and a cut drops the weakest picked, and the Doppler change check keeps
// every other satellite in both epochs.
//
// Pure, header-only, no clock of its own (the filter's time, s); the
// filter's task runs it (RawGnssShadow) and the GNSS task's stage takes the
// cap with the navigation state it builds at (ShadowNav::sat_cap).
// tests_cpp/test_cap_governor.cpp.
#ifndef RAW_GNSS_CAP_GOVERNOR_H
#define RAW_GNSS_CAP_GOVERNOR_H

#include <cstdint>

namespace gnss_raw
{

class CapGovernor
{
public:
    struct Config
    {
        int    floor = 0;              // the cap at a start, in the boost, after a hard cut (the profile's sat_cap)
        int    ceiling = 0;            // the most it is raised to (sat_cap_max); at or under the floor: off
        double hard_age_s = 0.2;       // an epoch closed this old was worked at once (epoch_job_max_s)
        double cut_frac = 0.8;         // cut_count of the last cut_window closed this share of hard_age_s
        int    cut_count = 2;          //   old or more: cut_step off
        int    cut_window = 4;
        int    cut_step = 2;
        double raise_frac = 0.6;       // raise_epochs in a row at most this share of it old: one more
        int    raise_epochs = 10;
        int    settle_epochs = 2;      // closes after a change that were built before it (the pipeline)
        double backoff_s = 10.0;       // a cap cut from is not raised into again for this long,
        double backoff_max_s = 80.0;   //   doubled each time a retry of it is cut, up to this
        double boost_hold_s = 2.0;     // the floor from the boost's first sign to this long after its last
        double gap_s = 0.15;           // epochs handed over this far apart: one between was lost (1.5 periods)
    };

    struct Stats
    {
        uint32_t closes = 0;           // epochs judged
        uint32_t raises = 0;
        uint32_t cuts = 0;             // cut_step off: cut_count of cut_window closed cut_frac old
        uint32_t floors = 0;           // back to the floor from above it: worked at once, too old or lost
        uint32_t late = 0;             // epochs worked at once or closed hard_age_s old (at the floor too)
        uint32_t lost = 0;             // epochs lost on the way (at the floor too)
        uint32_t boosts = 0;           // boost holds begun
        double   last_age_s = 0.0;     // the last epoch's age at its close
        double   max_age_s = 0.0;     // the oldest since takeWindow
        int      win_min = 0;          // the cap's least and most since takeWindow
        int      win_max = 0;
    };

    CapGovernor() : CapGovernor(Config()) {}
    explicit CapGovernor(const Config& c) : cfg_(c) { reset(); }

    // A ceiling above a floor: the cap moves. Otherwise it is the floor.
    bool on() const { return cfg_.floor > 0 && cfg_.ceiling > cfg_.floor; }
    int cap() const { return cap_; }
    const Config& config() const { return cfg_; }
    const Stats& stats() const { return st_; }
    double backoff() const { return backoff_; }
    double raiseAgeS() const { return cfg_.raise_frac * cfg_.hard_age_s; }
    double cutAgeS() const { return cfg_.cut_frac * cfg_.hard_age_s; }

    // A start (the filter started, or started again): the floor, nothing held.
    void reset()
    {
        cap_ = cfg_.floor;
        clean_ = 0;
        over_ = 0;
        settle_ = 0;
        block_cap_ = 0;
        block_until_ = -1.0e9;
        last_cut_from_ = 0;
        backoff_ = cfg_.backoff_s;
        boost_until_ = -1.0e9;
        st_.win_min = st_.win_max = cap_;
    }

    // An epoch closed at t, age_s after its GPS instant; late: some of its
    // work was done at once (its age past hard_age_s at a slice's start).
    void onClose(double age_s, bool late, double t)
    {
        if (!on())
            return;
        st_.closes++;
        st_.last_age_s = age_s;
        if (age_s > st_.max_age_s)
            st_.max_age_s = age_s;
        if (late || age_s >= cfg_.hard_age_s)
        {
            st_.late++;
            toFloor(t);
            return;
        }
        if (settle_ > 0)
        {
            settle_--;
            return;
        }
        if (age_s >= cutAgeS())
        {
            clean_ = 0;
            over_ = ((over_ << 1) | 1u) & windowMask();
            if (cap_ > cfg_.floor && bits(over_) >= cfg_.cut_count)
            {
                st_.cuts++;
                cutTo(cap_ - cfg_.cut_step, t);
            }
            return;
        }
        over_ = (over_ << 1) & windowMask();
        if (age_s > raiseAgeS() || t < boost_until_)
        {
            clean_ = 0;
            return;
        }
        if (clean_ < cfg_.raise_epochs)
            clean_++;
        if (clean_ < cfg_.raise_epochs || cap_ >= cfg_.ceiling)
            return;
        if (cap_ + 1 >= block_cap_ && t < block_until_)
            return;                       // the run waits for the backoff: the next clean close raises
        cap_++;
        st_.raises++;
        clean_ = 0;
        over_ = 0;
        settle_ = cfg_.settle_epochs;
        note();
    }

    // An epoch lost on the way at t: dropped as stale, or never handed over.
    void onLoss(double t)
    {
        if (!on())
            return;
        st_.lost++;
        toFloor(t);
    }

    // A sign of the boost at t (the launch, the IMU past the screen's
    // threshold, an epoch the screen judges): the floor until boost_hold_s on.
    void onBoost(double t)
    {
        if (!on())
            return;
        if (t >= boost_until_)
            st_.boosts++;
        if (t + cfg_.boost_hold_s > boost_until_)
            boost_until_ = t + cfg_.boost_hold_s;
        clean_ = 0;
        if (cap_ > cfg_.floor)
        {
            cap_ = cfg_.floor;
            over_ = 0;
            settle_ = cfg_.settle_epochs;
            note();
        }
    }

    bool boostHeld(double t) const { return t < boost_until_; }

    // The console's window: the cap's least and most, and the oldest close,
    // since the last call (then started again).
    void takeWindow(int& cap_min, int& cap_max, double& max_age_s)
    {
        cap_min = st_.win_min;
        cap_max = st_.win_max;
        max_age_s = st_.max_age_s;
        st_.win_min = st_.win_max = cap_;
        st_.max_age_s = 0.0;
    }

private:
    uint32_t windowMask() const
    {
        const int w = cfg_.cut_window < 1 ? 1 : (cfg_.cut_window > 31 ? 31 : cfg_.cut_window);
        return (1u << w) - 1u;
    }

    static int bits(uint32_t v)
    {
        int n = 0;
        for (; v != 0; v &= v - 1)
            n++;
        return n;
    }

    void note()
    {
        if (cap_ < st_.win_min)
            st_.win_min = cap_;
        if (cap_ > st_.win_max)
            st_.win_max = cap_;
    }

    void toFloor(double t)
    {
        clean_ = 0;
        if (cap_ <= cfg_.floor)
            return;
        st_.floors++;
        cutTo(cfg_.floor, t);
    }

    // Down to c (not under the floor) at t, the cap cut from held off for the
    // backoff: doubled when it is the same cap as the last cut's (a retry of
    // it failed), back to backoff_s for another.
    void cutTo(int c, double t)
    {
        const int from = cap_;
        backoff_ = from == last_cut_from_ ? (2.0 * backoff_ < cfg_.backoff_max_s ? 2.0 * backoff_ : cfg_.backoff_max_s)
                                          : cfg_.backoff_s;
        last_cut_from_ = from;
        block_cap_ = from;
        block_until_ = t + backoff_;
        cap_ = c < cfg_.floor ? cfg_.floor : c;
        over_ = 0;
        settle_ = cfg_.settle_epochs;
        note();
    }

    Config cfg_;
    Stats  st_;
    int    cap_ = 0;
    int    clean_ = 0;            // clean closes in a row at this cap
    uint32_t over_ = 0;           // the last cut_window closes judged: at the cut age or over (bit 0 the last)
    int    settle_ = 0;           // closes still to pass unjudged (built before the last change)
    int    block_cap_ = 0;        // the cap last cut from: not raised into before block_until_
    double block_until_ = -1.0e9;
    int    last_cut_from_ = 0;
    double backoff_ = 0.0;
    double boost_until_ = -1.0e9;
};

}  // namespace gnss_raw

#endif  // RAW_GNSS_CAP_GOVERNOR_H
