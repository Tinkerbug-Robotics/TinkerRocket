#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// ============================================================================
// The once-a-minute BLE status line: is anyone connected, and if not, is the
// board advertising at all?
//
// The rocket has been going missing over BLE with no phone holding the link.
// On 2026-09-24 two 10 s connect scans from the Mac missed the mini outright,
// then seven reconnects in a row found it within seconds. The one live
// occurrence was lost to a reset. This line puts the BLE state in every console
// capture, so the next occurrence can be read as it happens.
//
// The state is what TR_BLE_To_APP records in its own callbacks: connect,
// disconnect, each advertising start and completion, host reset. The line
// does not call ble_gap_adv_active(), which takes the host lock. If the host
// task were stuck holding it, asking would stall the caller's loop as well.
//
// Verdicts, in the order they are checked:
//   CONNECTED        a central holds the link; advertising is off by design.
//   NOT_ADVERTISING  nobody is connected and nothing is advertising: the last
//                    advertising start failed, or a host reset stopped it and
//                    no start has succeeded since.
//   FAST_OVERDUE     the fast phase's duration ran out more than
//                    kOverdueSlackMs ago, and no BLE_GAP_EVENT_ADV_COMPLETE
//                    started the slow phase. The controller stops advertising
//                    when the duration ends, and the slow phase starts only
//                    from that event on the host task, so the board cannot be
//                    found until something else restarts advertising.
//   ADVERTISING      a live fast or slow phase.
//
// Pure: integers in, a verdict and a line out. Tested in
// tests_cpp/test_ble_adv_status.cpp.
// ============================================================================

namespace tr_ble
{

struct AdvStatus
{
    bool     connected          = false;
    uint32_t connected_since_ms = 0;  // millis() at the last connect
    bool     adv_active         = false;  // a start succeeded and has not completed
    bool     adv_fast           = false;  // that advertising's phase
    uint32_t adv_since_ms       = 0;  // millis() when it started
    uint32_t fast_until_ms      = 0;  // when its fast-phase duration runs out
    uint32_t connects           = 0;
    uint32_t disconnects        = 0;
    int      last_disconnect_reason = 0;
    uint32_t adv_starts         = 0;
    uint32_t adv_start_fails    = 0;
    int      last_adv_rc        = 0;  // of the last ble_gap_adv_start()
    uint32_t host_resets        = 0;
    int      last_host_reset_reason = 0;
};

enum class AdvVerdict : uint8_t
{
    CONNECTED,
    ADVERTISING,
    FAST_OVERDUE,
    NOT_ADVERTISING,
};

// NimBLE reports the end of a timed advertisement within milliseconds. Five
// seconds is far beyond any honest delay, and well inside one status period.
static constexpr uint32_t kOverdueSlackMs = 5000;

// Wrap-safe "now is at or past deadline" for millis() stamps.
inline bool reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

inline AdvVerdict classify(const AdvStatus& s, uint32_t now_ms)
{
    if (s.connected) return AdvVerdict::CONNECTED;
    if (!s.adv_active) return AdvVerdict::NOT_ADVERTISING;
    if (s.adv_fast && reached(now_ms, s.fast_until_ms + kOverdueSlackMs))
        return AdvVerdict::FAST_OVERDUE;
    return AdvVerdict::ADVERTISING;
}

inline bool isAlarm(AdvVerdict v)
{
    return v == AdvVerdict::FAST_OVERDUE || v == AdvVerdict::NOT_ADVERTISING;
}

// An elapsed time as "42 s" under two minutes and "12.5 min" after.
inline void formatAge(uint32_t ms, char* buf, size_t len)
{
    if (ms < 120000u)
        snprintf(buf, len, "%lu s", (unsigned long)(ms / 1000u));
    else
        snprintf(buf, len, "%lu.%lu min", (unsigned long)(ms / 60000u),
                 (unsigned long)((ms % 60000u) / 6000u));
}

// The whole line, without the log tag, truncated to len if it does not fit.
inline void formatStatus(const AdvStatus& s, uint32_t now_ms, char* buf, size_t len)
{
    if (len == 0) return;
    char age[24];
    int n = 0;
    switch (classify(s, now_ms))
    {
    case AdvVerdict::CONNECTED:
        formatAge(now_ms - s.connected_since_ms, age, sizeof age);
        n = snprintf(buf, len, "[STATUS] connected %s", age);
        break;
    case AdvVerdict::ADVERTISING:
        formatAge(now_ms - s.adv_since_ms, age, sizeof age);
        n = snprintf(buf, len, "[STATUS] advertising %s for %s",
                     s.adv_fast ? "FAST (152.5 ms)" : "SLOW (1000 ms)", age);
        break;
    case AdvVerdict::FAST_OVERDUE:
        formatAge(now_ms - s.fast_until_ms, age, sizeof age);
        n = snprintf(buf, len, "[STATUS] NOT ADVERTISING: the fast phase ended %s ago "
                               "and the slow phase never started (no ADV_COMPLETE)", age);
        break;
    case AdvVerdict::NOT_ADVERTISING:
        if (s.last_adv_rc != 0)
            n = snprintf(buf, len, "[STATUS] NOT ADVERTISING: the last start failed, rc=%d",
                         s.last_adv_rc);
        else
            n = snprintf(buf, len, "[STATUS] NOT ADVERTISING and nobody connected");
        break;
    }
    if (n < 0 || (size_t)n >= len) return;

    size_t used = (size_t)n;
    n = snprintf(buf + used, len - used,
                 " | connects %lu, disconnects %lu (last reason %d) | adv starts %lu, fails %lu",
                 (unsigned long)s.connects, (unsigned long)s.disconnects,
                 s.last_disconnect_reason, (unsigned long)s.adv_starts,
                 (unsigned long)s.adv_start_fails);
    if (n < 0 || used + (size_t)n >= len) return;

    used += (size_t)n;
    if (s.host_resets > 0)
    {
        snprintf(buf + used, len - used, " | host resets %lu (last reason %d)",
                 (unsigned long)s.host_resets, s.last_host_reset_reason);
    }
}

}  // namespace tr_ble
