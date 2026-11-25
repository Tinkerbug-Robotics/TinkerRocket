// test_sim_replay_sdr.cpp — the rig replay's clock from the gnss-sdr-dev
// board's clock line (#1600 rig replay on the V10's auto image, the P1 rig's
// boost through the V10).
//
// The board's driver hands no timed fix: the flight loop drains the
// GPS-to-MCU line its RTCM epochs build (GnssClockData, about once a second
// once it is valid, 10 s of epochs after the board's first fix), and each
// record puts the drain's own instant in GPS time (lc86::gnssClockGpsAt),
// which SensorCollectorSim::replayNoteTime hands the clock as a fix. These pin
// that those records follow the pad and freeze the clock as the LC86G's fixes
// do, that the frozen clock places an MCU instant on the flight's time as the
// line does, and that a line first valid after the freeze leaves the replay on
// the pad.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include <GnssClockRecord.h>

#include "SimReplay.h"

using sim_replay::Header;
using sim_replay::ReplayClock;

namespace
{

// The P1 rig's hotshot scene: SignalSim's 2026-08-18 08:37:42 UTC start, a
// 120.1 s pad, ignition at GPS week 2432 TOW 204000.1, the flight to T+120
constexpr double kIgnitionGps = 2432 * 604800.0 + 204000.1;

Header hotshotHeader()
{
    Header h;
    h.ignition_gps_s = kIgnitionGps;
    h.scen_start_s = -120.1;
    h.scen_end_s = 119.9;
    return h;
}

// The board's line: GPS time gps0_s at MCU instant mcu0_us, the MCU's crystal
// `drift` fast (17 ppm measured on the rig V10)
GnssClockData boardLine(double gps0_s, int64_t mcu0_us, double drift, bool valid = true)
{
    GnssClockData c{};
    c.gps0_s = static_cast<uint32_t>(gps0_s);
    // (the line's whole second: the fraction goes onto the MCU side)
    c.mcu0_us = mcu0_us - static_cast<int64_t>(std::llround((gps0_s - std::floor(gps0_s)) * (1.0 + drift) * 1e6));
    c.drift = drift;
    c.flags = valid ? GCL_VALID : 0;
    return c;
}

// The flight loop's drain at MCU instant now_us: the record's GPS time there,
// handed to the clock (main.cpp's clock-line drain, replayNoteTime)
bool drain(ReplayClock& clk, const GnssClockData& line, int64_t now_us)
{
    double gps_s = 0.0;
    if (!lc86::gnssClockGpsAt(line, now_us, gps_s))
        return false;
    clk.onFix(gps_s, now_us);
    return true;
}

}  // namespace

TEST(ReplayClockSdrLine, ABoardLineFollowsThePadAndFreezesBeforeIgnition)
{
    ReplayClock clk;
    clk.reset(hotshotHeader());
    // The board's first fix 48 s into the scene, its line valid 10 s later
    // (T-62 s); the V10 up 30 s before the scene, its crystal 17 ppm fast
    const double drift = 17e-6;
    const int64_t mcu_ign_us = 180'000'000;   // ignition on the MCU, as the line puts it
    const GnssClockData line = boardLine(kIgnitionGps, mcu_ign_us, drift);
    // a record a second, drained 3-40 ms after the line moved
    for (double t = -62.0; t < -5.0; t += 1.0)
    {
        const int64_t now = mcu_ign_us + static_cast<int64_t>(std::llround(t * (1.0 + drift) * 1e6)) +
                            3000 + static_cast<int64_t>(std::fmod(-t * 7.0, 37.0) * 1000.0);
        ASSERT_TRUE(drain(clk, line, now));
        ASSERT_EQ(clk.state(), ReplayClock::kFollowing) << "t " << t;
    }
    EXPECT_EQ(clk.restarts(), 0u);
    // the first record at or after -5 s freezes it
    ASSERT_TRUE(drain(clk, line, mcu_ign_us + static_cast<int64_t>(std::llround(-4.99 * (1.0 + drift) * 1e6))));
    EXPECT_EQ(clk.state(), ReplayClock::kFrozen);
    // from there the MCU's crystal: ignition and burnout land where the line
    // puts them, within what 17 ppm runs off in the 9 s since the freeze
    double t = 0.0;
    ASSERT_TRUE(clk.tAt(mcu_ign_us, t));
    EXPECT_NEAR(t, 0.0, 2e-4);
    ASSERT_TRUE(clk.tAt(mcu_ign_us + static_cast<int64_t>(std::llround(3.9 * (1.0 + drift) * 1e6)), t));
    EXPECT_NEAR(t, 3.9, 2e-4);
}

TEST(ReplayClockSdrLine, ALineNotValidYetSetsNothing)
{
    ReplayClock clk;
    clk.reset(hotshotHeader());
    const GnssClockData line = boardLine(kIgnitionGps, 180'000'000, 0.0, /*valid=*/false);
    EXPECT_FALSE(drain(clk, line, 120'000'000));
    EXPECT_EQ(clk.state(), ReplayClock::kWaiting);
    double t = 0.0;
    EXPECT_FALSE(clk.tAt(150'000'000, t));   // the pad, looped
}

TEST(ReplayClockSdrLine, ALineFirstValidAfterTheFreezeLeavesThePad)
{
    // a board started too late: its line valid only at T-2 s. The replay must
    // not begin mid-boost: the pad for good (the run is void, as the console says)
    ReplayClock clk;
    clk.reset(hotshotHeader());
    const int64_t mcu_ign_us = 180'000'000;
    const GnssClockData line = boardLine(kIgnitionGps, mcu_ign_us, 0.0);
    ASSERT_TRUE(drain(clk, line, mcu_ign_us - 2'000'000));
    EXPECT_EQ(clk.state(), ReplayClock::kMissed);
    double t = 0.0;
    EXPECT_FALSE(clk.tAt(mcu_ign_us + 1'000'000, t));
}

TEST(ReplayClockSdrLine, ALineFromAnotherSkyIsNotTheRigs)
{
    // a line from the real sky (today, not the scene's day) is outside the
    // scene's span: it neither arms nor counts as the rig's
    ReplayClock clk;
    clk.reset(hotshotHeader());
    const GnssClockData line = boardLine(kIgnitionGps + 86.0 * 86400.0, 180'000'000, 0.0);
    ASSERT_TRUE(drain(clk, line, 100'000'000));
    EXPECT_EQ(clk.state(), ReplayClock::kWaiting);
    EXPECT_EQ(clk.fixes(), 0u);
}
