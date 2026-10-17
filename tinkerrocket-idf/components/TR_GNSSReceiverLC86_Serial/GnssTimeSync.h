// GnssTimeSync.h — the MCU clock against GPS time, from the receiver's output alone (no 1PPS)
//
// A raw-measurement filter applies each GNSS epoch at its own instant on the
// IMU's clock (esp_timer), so it has to know which MCU microsecond a GPS time
// was (#1600). The Beetle M1 has no 1PPS line to the MCU (it only drives an
// LED), so the evidence is when the receiver's sentences arrive. Every
// $PQTMPVT carries its epoch's GPS time; the driver stamps when the
// sentence's last byte arrived, backdated from the moment it read the bytes
// by the bytes that came after it. That stamp minus the GPS time is
//
//     clock offset  +  the receiver's own delay  +  a lateness of 0..one poll
//
// The lateness is never negative: a poll that runs after the burst has ended
// has no bytes behind the sentence to backdate by. So the lower envelope of
// those differences, tracked against the MCU crystal's drift, is the clock
// offset plus the receiver's delay. That delay, from a GPS epoch to the last
// byte of its $PQTMPVT, is a constant of the receiver (Config::latency_s), to
// be measured once against a 1PPS (Beetle V2) or a scope trace; until then
// every epoch is placed late by it, the same for all of them.
//
// How the envelope is tracked: each GPS second keeps its lowest MCU - GPS
// (ten samples at 10 Hz), and the line sits on the kAnchorRank-th lowest of
// the last ~minute of them rather than the lowest, so one or two corrupt
// samples cannot drag it down. Its slope, the MCU crystal's drift, comes from
// a longer baseline: the lowest MCU - GPS of each 10 s block over up to 10
// minutes, through a robust line, shrunk toward zero by its own scatter
// against a +-20 ppm crystal. A minute is too short for it: on four Beetle
// logs (2026-10-12) a one-minute slope swung +-80 ppm between halves of the
// same log, and placing each next second with it did worse than no drift at
// all. Five samples in a row far from the line (a step in the receiver's
// time, a reset) restart it.
//
// Pure and host-testable (tests_cpp/test_gnss_time_sync.cpp).
#ifndef GNSS_TIME_SYNC_H
#define GNSS_TIME_SYNC_H

#include <cstddef>
#include <cstdint>

namespace lc86
{

class GnssTimeSync
{
public:
    struct Config
    {
        // The receiver's delay from a GPS epoch to the last byte of that
        // epoch's sentence, s. Uncalibrated: 0.
        double latency_s = 0.0;
        // A sample this far from the line counts toward a restart, s.
        double max_step_s = 0.5;
    };

    static constexpr int    kBins       = 64;     // GPS seconds with samples: about a minute
    static constexpr int    kAnchorRank = 3;      // the envelope sits on the 3rd-lowest second
    static constexpr int    kStepCount  = 5;      // samples in a row off the line to restart
    static constexpr double kMinSpanS   = 10.0;   // history before answering, s
    static constexpr double kNearS      = 1.0e-3; // seconds this close to the line count as on it
    static constexpr int    kBlocks     = 60;     // drift baseline: 10 minutes of ...
    static constexpr double kBlockS     = 10.0;   // ... 10 s blocks
    static constexpr double kDriftPrior = 20e-6;  // a crystal's drift, 1 sigma
    static constexpr double kMaxDrift   = 1.0e-4; // 100 ppm: a crystal that far off is not one

    GnssTimeSync() { reset(); }
    explicit GnssTimeSync(const Config& cfg) : cfg_(cfg) { reset(); }

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }
    void reset();

    // One epoch: its GPS time (continuous seconds, week * 604800 + time of
    // week) and the MCU time (esp_timer, us) its sentence ended. False if the
    // sample was set aside as an outlier or restarted the history.
    bool addSample(double gps_s, int64_t mcu_us);

    // Enough history to answer: seconds spanning kMinSpanS.
    bool valid() const;

    // The MCU time (us) of a GPS instant; the GPS time of an MCU instant.
    bool mcuAt(double gps_s, int64_t& mcu_us) const;
    bool gpsAt(int64_t mcu_us, double& gps_s) const;

    // The MCU crystal against GPS, ppm (positive: the MCU clock runs fast).
    double driftPpm() const { return drift_ * 1.0e6; }
    // Whether that drift has been fitted yet (three 10 s blocks spanning
    // 20 s, about 30 s after the first sample). Before, the line runs flat:
    // valid, but off the MCU clock by the crystal's ppm each second since its
    // anchor.
    bool driftFitted() const { return drift_fitted_; }
    // How far above the envelope 90 % of the recent samples sit, ms: the
    // poll's quantisation, what the envelope removes. (Polls that land while
    // the receiver's burst is still arriving backdate exactly, so the median
    // is often 0.)
    double lateness90Ms() const;
    // RMS of the near-envelope seconds about the line, ms: how well the
    // envelope is pinned.
    double envelopeRmsMs() const { return env_rms_ms_; }
    uint32_t samples() const { return samples_; }
    uint32_t restarts() const { return restarts_; }

    // The line as mcuAt applies it, for a log a replay can place epochs by
    // exactly as the flight did: with t = gps_s - gps0_s,
    //   mcu_us = mcu0_us + llround((t + a_s + drift * (t - t_ref_s) - latency_s) * 1e6)
    // (valid only: before that mcuAt answers nothing).
    struct Snapshot
    {
        double   gps0_s = 0.0;      // whole GPS seconds (week * 604800 + time of week)
        int64_t  mcu0_us = 0;       // esp_timer, us
        double   a_s = 0.0;         // the envelope's MCU - GPS at t_ref_s, s from the references
        double   t_ref_s = 0.0;     // s after gps0_s
        double   drift = 0.0;       // d(MCU - GPS) / d(GPS)
        double   latency_s = 0.0;   // Config::latency_s
        double   env_rms_ms = 0.0;  // envelopeRmsMs()
        double   late90_ms = 0.0;   // lateness90Ms()
        uint32_t samples = 0;
        uint32_t restarts = 0;
        uint32_t seq = 0;           // lineSeq()
        int      bins = 0;          // GPS seconds held (0..kBins)
        bool     started = false;   // a sample has started the history
        bool     valid = false;     // valid()
    };
    void snapshot(Snapshot& out) const;
    // Counts every change of the line: each GPS second filed (the refit), the
    // first one, a restart. Never reset, so a log's gaps show lost records.
    uint32_t lineSeq() const { return seq_; }

private:
    struct Bin
    {
        int64_t sec;   // whole GPS second, from gps0_
        double  t;     // GPS time of that second's lowest sample, s from gps0_
        double  d;     // its MCU - GPS, s, both from their references
    };

    struct Block
    {
        int64_t blk;   // whole kBlockS block, from gps0_
        double  t;     // GPS time of its lowest sample, s from gps0_
        double  d;     // its MCU - GPS, s
    };

    double line(double t) const { return a_ + drift_ * (t - t_ref_); }
    void   refit();
    void   refitDrift();
    void   start(double gps_s, int64_t mcu_us);

    Config   cfg_;
    double   gps0_ = 0.0;   // references that keep the doubles small
    int64_t  mcu0_ = 0;
    Bin      bins_[kBins] = {};
    int      head_ = 0;     // next slot
    int      n_ = 0;
    Block    blocks_[kBlocks] = {};
    int      bhead_ = 0;
    int      bn_ = 0;
    double   a_ = 0.0;      // the envelope at t_ref_, s
    double   drift_ = 0.0;  // d(MCU - GPS) / d(GPS)
    bool     drift_fitted_ = false;
    double   t_ref_ = 0.0;
    double   env_rms_ms_ = 0.0;
    bool     have_ = false;
    int      off_run_ = 0;
    uint32_t samples_ = 0;
    uint32_t restarts_ = 0;
    uint32_t seq_ = 0;          // lineSeq(): not reset
    float    late_ms_[32] = {};
    int      late_n_ = 0;
    int      late_head_ = 0;
};

}  // namespace lc86

#endif  // GNSS_TIME_SYNC_H
