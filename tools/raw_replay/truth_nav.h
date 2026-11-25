// truth_nav.h -- a sim's truth as the navigation filter the flight loop
// reads (raw_replay --truth-nav: Grid B's reference call).
//
// The apogee quorum (flight_votes.h) reads the filter through RawNavAdapter.
// Publishing the truth on a ShadowLink of its own, each pass, gives a second
// quorum a perfect filter -- its vertical velocity, its pitch and its height
// the truth's -- with the run's own barometer, receiver fixes and IMU ticks,
// all through the firmware's own voters and debounces. Its call is the one
// a filter's call is measured against: what the counters' latency alone
// costs, not a guess at it.
//
// The file is sim_events.py's truth/<flight>.nav: '#' comments, then a line
// a row -- the MCU time (us), the NED velocity (m/s), the attitude (body FRD
// -> NED, scalar first) and the height on the receiver fix's datum (m) --
// in time order.
//
// Pure (tests_cpp/test_raw_replay_votes.cpp).
#ifndef RAW_REPLAY_TRUTH_NAV_H
#define RAW_REPLAY_TRUTH_NAV_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ShadowLink.h"

namespace rr
{

class TruthNav
{
public:
    struct Row
    {
        int64_t t_us = 0;
        float   v[3] = {};
        float   q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        double  h = 0.0;
    };

    // Reads the file; false (and err says why) on a missing file, a bad row
    // or time running back.
    bool load(const std::string& path, std::string* err = nullptr)
    {
        rows_.clear();
        at_ = 0;
        std::ifstream in(path);
        if (!in)
            return fail(err, "cannot open " + path);
        std::string line;
        int n = 0;
        while (std::getline(in, line))
        {
            n++;
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream ss(line);
            Row r;
            long long t = 0;
            ss >> t >> r.v[0] >> r.v[1] >> r.v[2] >> r.q[0] >> r.q[1] >> r.q[2] >> r.q[3] >> r.h;
            if (!ss)
                return fail(err, path + ": a bad row at line " + std::to_string(n));
            r.t_us = t;
            if (!rows_.empty() && r.t_us <= rows_.back().t_us)
                return fail(err, path + ": time runs back at line " + std::to_string(n));
            rows_.push_back(r);
        }
        if (rows_.empty())
            return fail(err, path + ": no rows");
        return true;
    }

    // The truth at t_us (the row at or before it) published on link as the
    // filter's state: running, GNSS- and barometer-aided, at that row's time.
    // Calls come in time order. False before the first row (nothing published).
    bool publish(int64_t t_us, gnss_raw::ShadowLink& link)
    {
        if (rows_.empty() || t_us < rows_.front().t_us)
            return false;
        while (at_ + 1 < rows_.size() && rows_[at_ + 1].t_us <= t_us)
            at_++;
        const Row& r = rows_[at_];
        gnss_raw::ShadowNav n{};
        n.time_us = static_cast<uint32_t>(r.t_us);
        n.lla[0] = 0.0;   // the votes read the height only
        n.lla[1] = 0.0;
        n.lla[2] = r.h;
        for (int i = 0; i < 3; ++i)
            n.vel_ned[i] = r.v[i];
        for (int i = 0; i < 4; ++i)
            n.quat[i] = r.q[i];
        for (float& v : n.var)
            v = 1e-4f;
        n.running = 1;
        n.gnss_aided = 1;
        n.baro_aided = 1;
        link.nav.write(n);
        return true;
    }

    size_t size() const { return rows_.size(); }
    const Row& row(size_t i) const { return rows_[i]; }

private:
    // nothing kept from a file that did not read
    bool fail(std::string* err, const std::string& why)
    {
        rows_.clear();
        if (err != nullptr)
            *err = why;
        return false;
    }

    std::vector<Row> rows_;
    size_t at_ = 0;
};

}  // namespace rr

#endif  // RAW_REPLAY_TRUTH_NAV_H
