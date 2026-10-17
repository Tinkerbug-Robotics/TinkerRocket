// The heading fit's bench on the host (TR_RawGnssShadow/HeadingFitBench.h):
// the measurements the flight computer prints at boot with
// -DTR_HEADING_FIT_BENCH=1, here, to set the two side by side. Not a test:
// built with the tests, never run by ctest.
#include <chrono>
#include <cstdio>

#include "HeadingFitBench.h"

namespace
{

int64_t nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void out(const char* line)
{
    std::puts(line);
}

}  // namespace

int main()
{
    static gnss_raw::HeadingFit fit;
    static gnss_raw::RawMeas m[gnss_raw::RawEpoch::kMaxObs];
    gnss_raw::bench::run(fit, m, nowNs, out, 200, 1000.0);
    return 0;
}
