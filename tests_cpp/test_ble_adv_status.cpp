// Host tests for BleAdvStatus.h — the once-a-minute BLE status line.
//
// The line exists to catch one state live: nobody connected and nothing
// advertising, which makes the rocket impossible to find. These tests pin the
// verdicts (above all FAST_OVERDUE, the fast phase ending with no slow phase
// after it), their wrap-safety across a millis() rollover, and the line itself.

#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "BleAdvStatus.h"

using tr_ble::AdvStatus;
using tr_ble::AdvVerdict;
using tr_ble::classify;
using tr_ble::formatAge;
using tr_ble::formatStatus;
using tr_ble::isAlarm;
using tr_ble::kOverdueSlackMs;

namespace
{

std::string line(const AdvStatus& s, uint32_t now_ms)
{
    char buf[256];
    formatStatus(s, now_ms, buf, sizeof buf);
    return buf;
}

std::string age(uint32_t ms)
{
    char buf[24];
    formatAge(ms, buf, sizeof buf);
    return buf;
}

// Fast advertising started at t0 for the #541 30 s window.
AdvStatus fastFrom(uint32_t t0)
{
    AdvStatus s;
    s.adv_active    = true;
    s.adv_fast      = true;
    s.adv_since_ms  = t0;
    s.fast_until_ms = t0 + 30000;
    s.adv_starts    = 1;
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// Verdicts
// ---------------------------------------------------------------------------

TEST(BleAdvStatus, ConnectedWinsWhateverTheAdvertisingState)
{
    AdvStatus s = fastFrom(1000);
    s.connected          = true;
    s.connected_since_ms = 2000;
    // Advertising is off by design while connected, so an old fast deadline
    // far in the past must not raise the alarm.
    EXPECT_EQ(classify(s, 10'000'000), AdvVerdict::CONNECTED);
    EXPECT_FALSE(isAlarm(classify(s, 10'000'000)));
}

TEST(BleAdvStatus, FastPhaseInsideItsWindowIsAdvertising)
{
    const AdvStatus s = fastFrom(1000);
    EXPECT_EQ(classify(s, 1000), AdvVerdict::ADVERTISING);
    EXPECT_EQ(classify(s, 31000), AdvVerdict::ADVERTISING);  // deadline itself
}

TEST(BleAdvStatus, FastPhaseJustPastItsDeadlineIsNotYetOverdue)
{
    // ADV_COMPLETE lands milliseconds after the duration ends; the slack keeps
    // a status line that happens to fall in that gap from crying wolf.
    const AdvStatus s = fastFrom(1000);
    EXPECT_EQ(classify(s, 31000 + kOverdueSlackMs - 1), AdvVerdict::ADVERTISING);
}

TEST(BleAdvStatus, FastPhaseWithNoSlowPhaseAfterItIsOverdue)
{
    // The suspect: the controller stopped advertising at the end of the fast
    // window, and the host never handled the event that starts the slow phase.
    const AdvStatus s = fastFrom(1000);
    EXPECT_EQ(classify(s, 31000 + kOverdueSlackMs), AdvVerdict::FAST_OVERDUE);
    EXPECT_TRUE(isAlarm(AdvVerdict::FAST_OVERDUE));
}

TEST(BleAdvStatus, SlowPhaseNeverGoesOverdue)
{
    AdvStatus s = fastFrom(1000);
    s.adv_fast      = false;  // the slow phase has no duration
    s.fast_until_ms = 0;
    EXPECT_EQ(classify(s, 50'000'000), AdvVerdict::ADVERTISING);
}

TEST(BleAdvStatus, NothingRunningAndNobodyConnectedIsAnAlarm)
{
    AdvStatus s;  // e.g. a host reset stopped advertising and no start followed
    s.host_resets = 1;
    EXPECT_EQ(classify(s, 5000), AdvVerdict::NOT_ADVERTISING);
    EXPECT_TRUE(isAlarm(AdvVerdict::NOT_ADVERTISING));
}

TEST(BleAdvStatus, OverdueIsJudgedAcrossTheMillisWrap)
{
    // Started 10 s before millis() wraps: the deadline lands after the wrap.
    const uint32_t t0 = 0xFFFFFFFFu - 10000u;
    const AdvStatus s = fastFrom(t0);            // fast_until = t0 + 30 s, wrapped
    const uint32_t deadline = s.fast_until_ms;   // ~19.999 s past the wrap
    EXPECT_LT(deadline, t0);                     // the wrap really happened
    EXPECT_EQ(classify(s, t0 + 5000), AdvVerdict::ADVERTISING);      // before the wrap
    EXPECT_EQ(classify(s, deadline), AdvVerdict::ADVERTISING);       // after it
    EXPECT_EQ(classify(s, deadline + kOverdueSlackMs), AdvVerdict::FAST_OVERDUE);
}

// ---------------------------------------------------------------------------
// The line
// ---------------------------------------------------------------------------

TEST(BleAdvStatus, AgesReadAsSecondsThenMinutes)
{
    EXPECT_EQ(age(0), "0 s");
    EXPECT_EQ(age(42'999), "42 s");
    EXPECT_EQ(age(119'999), "119 s");
    EXPECT_EQ(age(120'000), "2.0 min");
    EXPECT_EQ(age(750'000), "12.5 min");
}

TEST(BleAdvStatus, LineForEachVerdict)
{
    AdvStatus c;
    c.connected          = true;
    c.connected_since_ms = 1000;
    c.connects           = 4;
    c.disconnects        = 3;
    c.last_disconnect_reason = 531;
    c.adv_starts         = 9;
    EXPECT_EQ(line(c, 43'000),
              "[STATUS] connected 42 s | connects 4, disconnects 3 (last reason 531)"
              " | adv starts 9, fails 0");

    AdvStatus slow = fastFrom(0);
    slow.adv_fast = false;
    EXPECT_EQ(line(slow, 300'000).rfind("[STATUS] advertising SLOW (1000 ms) for 5.0 min |", 0), 0u);

    const AdvStatus fast = fastFrom(0);
    EXPECT_EQ(line(fast, 12'000).rfind("[STATUS] advertising FAST (152.5 ms) for 12 s |", 0), 0u);

    // Overdue: the age is measured from the end of the fast window.
    EXPECT_EQ(line(fast, 30'000 + 95'000).rfind(
                  "[STATUS] NOT ADVERTISING: the fast phase ended 95 s ago and the slow "
                  "phase never started (no ADV_COMPLETE) |", 0), 0u);

    AdvStatus failed;
    failed.last_adv_rc     = 6;
    failed.adv_start_fails = 1;
    EXPECT_EQ(line(failed, 1000).rfind("[STATUS] NOT ADVERTISING: the last start failed, rc=6 |", 0), 0u);
    EXPECT_NE(line(failed, 1000).find("adv starts 0, fails 1"), std::string::npos);
}

TEST(BleAdvStatus, HostResetsAppearOnlyOnceThereIsOne)
{
    AdvStatus s = fastFrom(0);
    EXPECT_EQ(line(s, 1000).find("host resets"), std::string::npos);

    AdvStatus r;
    r.host_resets            = 2;
    r.last_host_reset_reason = 19;
    const std::string l = line(r, 1000);
    EXPECT_EQ(l.rfind("[STATUS] NOT ADVERTISING and nobody connected |", 0), 0u);
    EXPECT_NE(l.find(" | host resets 2 (last reason 19)"), std::string::npos);
}

TEST(BleAdvStatus, ShortBufferTruncatesAndStaysTerminated)
{
    AdvStatus r;
    r.host_resets = 7;
    for (size_t len : {1u, 10u, 40u, 90u})
    {
        char buf[128];
        memset(buf, 'x', sizeof buf);
        formatStatus(r, 1000, buf, len);
        EXPECT_LT(strlen(buf), len);                 // terminated inside the buffer
        EXPECT_EQ(buf[len], 'x');                    // nothing written past it
    }
    char none[4] = {'x', 'x', 'x', 'x'};
    formatStatus(r, 1000, none, 0);                  // len 0 writes nothing
    EXPECT_EQ(none[0], 'x');
}
