// PadSurvey.h — the GNSS pad survey (#1600 phase 5): each satellite's
// pseudorange against the pad's known position, while the rocket waits on it.
//
// The pad is the flight computer's launch-site average of the receiver's own
// fix. On each raw epoch there, a satellite's corrected pseudorange
// (RawMeasBuilder's: its orbit and clock, the troposphere, and the ionosphere
// when the flight computer has the broadcast parameters) less its range from
// the pad is its residual: the receiver clock, the system's time offset, and
// what the corrections left. Without the parameters that is the ionosphere
// above all; with them, the broadcast orbit and clock and the pad's multipath.
// Less the clock and offset, and smoothed over tau_s, it is the satellite's
// pad correction. It is kept fresh until liftoff and frozen there.
//
// The backup ionosphere. With no current broadcast parameters the shadow
// takes each satellite's correction out of its pseudorange (correct()): the
// filter then sits on the pad's position, and the ionosphere above the pad
// is gone with the rest. A satellite with no correction (risen since, or
// relocked) is still used, its sigma widened by the ionosphere it keeps.
//
// The clock. A correction is defined up to a constant per system, which the
// filter's clock and system offsets take up. Each epoch the clock is the
// median of the settled satellites' residuals less their corrections, so a
// satellite coming or going does not move the others'.
//
// The pad, checked. The receiver's own fix can be wrong while its raw
// measurements are not: on 2026-10-13 the Beetle's LC86G put itself 312 m below
// sea level, 3-D fix, "1 m" vertical accuracy, while its pseudoranges agreed
// (to 0.2-1.7 m per constellation) on a point near sea level, as the barometer
// did. Corrections against such a pad would drag the filter onto it. So each
// epoch the corrections in use are fitted as a position offset and a clock per
// system: where the raw measurements put the pad. Further than
// max_pad_offset_m from the receiver's average (the uncorrected ionosphere is
// some metres of it), the corrections are not used until it agrees again.
//
// Relocks. A pseudorange starts biased after a relock (RawMeasBuilder: 40 m of
// sigma decaying over 10 s), so a satellite's residuals count only once it
// has been locked settle_s; the caller passes each one's lock age.
//
// Pure C++: RawGnssShadow drives it, tests_cpp/test_pad_survey.cpp tests it.
#ifndef PAD_SURVEY_H
#define PAD_SURVEY_H

#include <cstdint>

#include "RawMeas.h"

namespace gnss_raw
{

class PadSurvey
{
public:
    struct Config
    {
        double tau_s        = 120.0;  // each correction's smoothing time
        double settle_s     = 50.0;   // a satellite's residuals count once it has been locked this long
        double min_span_s   = 30.0;   // a correction is used once it has this much behind it
        double gap_s        = 2.5;    // missing longer than this, a satellite starts over (a relock)
        double max_resid_m  = 30.0;   // a settled satellite's residual this far from its correction is refused
        double pad_move_m   = 20.0;   // the pad moving this far between epochs starts the survey over
        double max_pad_offset_m = 30.0; // the raw measurements putting the pad further than this from
                                        // the receiver's average: its corrections are not used
        float  iono_zenith_m = 5.0f;  // what a satellite without a correction keeps (zenith, m; times
                                      // Klobuchar's obliquity factor), added to its correlated sigma
    };

    struct Summary
    {
        uint8_t tracked = 0;      // satellites with a pseudorange in the last epoch
        uint8_t settling = 0;     //   locked less than settle_s
        uint8_t corrected = 0;    //   with a correction in use
        float   rms_m = 0.0f;     // the RMS of those corrections
        bool    pad_ok = false;   // the raw measurements put the pad within max_pad_offset_m of the
                                  // receiver's average (false too until enough satellites tell)
        float   pad_offset_m = 0.0f;  // how far they put it, m
        double  last_tow_s = -1;  // the last epoch observed (time of week); -1: none
    };

    PadSurvey() { reset(); }
    explicit PadSurvey(const Config& cfg) : cfg_(cfg) { reset(); }
    const Config& config() const { return cfg_; }

    void reset();

    // One pad epoch: its measurements as built (before correct()), each
    // one's lock age (s, RawMeasBuilder::relockAge), and the pad (ECEF, m).
    void observe(const RawMeas* m, int n, const double* lock_age_s, double tow_s, const double pad[3]);

    // Take each satellite's correction out of its pseudorange; without one,
    // widen its correlated sigma by the ionosphere it keeps (r: the receiver,
    // ECEF, for the elevation). Returns how many were corrected. With none
    // in use, or the pad not vouched for (summary().pad_ok), nothing is changed.
    int correct(RawMeas* m, int n, const double r[3]) const;

    // The satellite's correction (m), if one is in use.
    bool correction(int sys, int prn, double& corr_m) const;

    const Summary& summary() const { return sum_; }

private:
    static constexpr int kSys = 3;
    static constexpr int kPrn = 64;

    struct Sat
    {
        double corr = 0.0;      // m
        double first = 0.0;     // tow of its first residual since it (re)started
        double last_obs = 0.0;  // tow it was last in an epoch
        double last_upd = 0.0;  // tow of its last residual
        uint32_t n = 0;         // residuals taken
        bool   seen = false;
    };

    bool usable(const Sat& s) const { return s.n > 0 && s.last_upd - s.first >= cfg_.min_span_s; }
    static double towDiff(double a, double b);

    Config  cfg_;
    Sat     sat_[kSys][kPrn];
    bool    have_pad_ = false;
    double  pad_[3] = {};
    Summary sum_;
};

}  // namespace gnss_raw

#endif  // PAD_SURVEY_H
