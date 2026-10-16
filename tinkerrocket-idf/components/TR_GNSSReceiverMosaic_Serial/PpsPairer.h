// PpsPairer.h — which GPS second each 1PPS edge was (pure, header-only, host-tested)
//
// The edge interrupt knows only when (MCU time); the receiver's blocks know
// only which second (GPS time). xPPSOffset goes out just after each pulse and
// PVTGeodetic stamps every whole second, both a few to tens of ms after the
// edge they belong to, so a block names the second of the latest edge before
// it, within a window far shorter than the second between edges. Both name
// each second; the first to arrive pairs it, the second finds it taken.
// Tested in tests_cpp/test_mosaic_link.cpp.
#ifndef PPSPAIRER_H
#define PPSPAIRER_H

#include <cstddef>
#include <cstdint>

namespace mosaic
{

class PpsPairer
{
public:
    static constexpr size_t kEdges = 4;   // edges waiting for their second

    explicit PpsPairer(int64_t window_us = 500000) : window_us_(window_us) {}

    // An edge, in time order.
    void addEdge(int64_t mcu_us)
    {
        if (n_ == kEdges)
        {
            for (size_t i = 1; i < n_; i++) edges_[i - 1] = edges_[i];
            n_--;
        }
        edges_[n_++] = mcu_us;
    }

    // A block naming GPS second `gps_s` arrived at `arrival_us`. True with
    // the edge that was that second; the edge and any older are then spent.
    bool pair(double gps_s, int64_t arrival_us, int64_t& edge_us)
    {
        if (paired_any_ && gps_s == last_s_) return false;
        int best = -1;
        for (size_t i = 0; i < n_; i++)
        {
            if (edges_[i] <= arrival_us && arrival_us - edges_[i] < window_us_) best = static_cast<int>(i);
        }
        if (best < 0) return false;
        edge_us = edges_[best];
        const size_t keep = n_ - static_cast<size_t>(best) - 1;
        for (size_t i = 0; i < keep; i++) edges_[i] = edges_[best + 1 + i];
        n_ = keep;
        last_s_ = gps_s;
        paired_any_ = true;
        pairs_++;
        return true;
    }

    uint32_t pairs() const { return pairs_; }
    size_t   waiting() const { return n_; }

private:
    int64_t  window_us_;
    int64_t  edges_[kEdges] = {};
    size_t   n_ = 0;
    double   last_s_ = 0.0;
    bool     paired_any_ = false;
    uint32_t pairs_ = 0;
};

}  // namespace mosaic

#endif  // PPSPAIRER_H
