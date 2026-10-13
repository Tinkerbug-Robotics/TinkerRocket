// SatJumpCheck.h — each satellite's pseudorange against the IMU, epoch to
// epoch (#1508): a measurement that moved in a way the vehicle did not.
//
// Between two raw epochs the filter carries the receiver on the IMU alone;
// its prior at the new epoch is the IMU's propagation from the last update.
// For each satellite in both epochs, the pseudorange's change should then be
// the change in its range from those two receiver positions, plus the receiver
// clock's change, which every satellite shares (taken as their median, so a
// clock step of any size drops out with it). What is left is the satellite's
// own: noise of a metre or two, and a jump where its code jumped -- a relock,
// a reflection taking over, a tracking loop slipping. Being a difference, the
// test does not see an error that stays the same from epoch to epoch, which is
// the absolute check's business (the filter's innovation gate); it sees a
// jump at once, however the filter's position has been led.
//
// The shadow takes a flagged satellite as relocked (RawMeasBuilder's relock
// sigma, decaying over 10 s), so the filter de-weights it until it settles.
//
// Pure C++: RawGnssShadow drives it, tests_cpp/test_gnss_checks.cpp tests it.
#ifndef SAT_JUMP_CHECK_H
#define SAT_JUMP_CHECK_H

#include <cstdint>

#include "RawMeas.h"

namespace gnss_raw
{

class SatJumpCheck
{
public:
    struct Config
    {
        double jump_m   = 20.0;   // a satellite's change this far from the common clock's is a jump
        int    min_sats = 5;      // fewer satellites in both epochs: no judgement
        double max_dt_s = 1.6;    // the epochs this far apart at most (the LC86G's is 1 s)
    };

    struct Summary
    {
        int    judged = 0;        // satellites compared, last epoch
        int    flagged = 0;       //   of them, jumps
        double clock_m = 0.0;     // the common clock's change
        double worst_m = 0.0;     // the largest change off it
    };

    SatJumpCheck() { reset(); }
    explicit SatJumpCheck(const Config& cfg) : cfg_(cfg) { reset(); }

    void reset();

    // Before the update: the epoch's measurements as built, its time of week,
    // and the receiver there as the filter's prior has it (ECEF). Sets
    // flagged[i] for each measurement that jumped; returns how many.
    int check(const RawMeas* m, int n, double tow_s, const double r_prior[3], bool* flagged);

    // After the update: the receiver at that epoch as the filter now has it
    // (ECEF), which the next epoch's changes are taken from. The pseudoranges
    // kept are the ones check() saw, as built: corrections applied in between
    // (the pad survey's) would otherwise read as a jump when they start.
    void commit(const RawMeas* m, int n, double tow_s, const double r_post[3]);

    const Summary& summary() const { return sum_; }

private:
    static constexpr int kSys = 3;
    static constexpr int kPrn = 64;

    struct Sat
    {
        double pr = 0.0;      // the pseudorange as built, m
        double range = 0.0;   // its range from the receiver after the update, m
        double tow = -1.0;    // the epoch they are from
        double pr_new = 0.0;  // this epoch's, as check() saw it
        double tow_new = -1.0;
    };

    Config  cfg_;
    Sat     sat_[kSys][kPrn];
    Summary sum_;
};

}  // namespace gnss_raw

#endif  // SAT_JUMP_CHECK_H
