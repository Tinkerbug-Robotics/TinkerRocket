#pragma once

#include <stdint.h>

// #1779: the two frames the FC sends the OC at LANDED.
//
// Two frames end a flight on the OC. The LANDED snapshot clear retires the
// in-flight recovery record the OC holds: its RAM cache and the cache's RTC
// copy (#1717). END_FLIGHT ends its log. Both go into the I2S transmit queue
// with no wait, and the queue can be full at LANDED entry (#141).
//
// END_FLIGHT has long been retried on every LANDED pass until its enqueue went
// through. The clear was sent once, from the one-shot LANDED actions, so a
// full queue lost it. The OC then kept the flight's last INFLIGHT frame and
// served it for 30 min, and an FC reset in that time restored INFLIGHT on the
// ground. Now:
//
//   * The clear is tried every kRetryMs until it is in the queue. Each try
//     builds a whole snapshot, so a queue that never drains (the TX channel
//     down) costs the loop one build every 20 ms, not one every pass.
//   * END_FLIGHT goes in once the clear has, so the OC always has the clear
//     first: the queue is FIFO, and the two share it, so waiting adds no way
//     for END_FLIGHT to be lost.
//   * Once queued, the clear is sent again every kRepeatMs for as long as the
//     FC stays in LANDED. Neither frame is acknowledged, and one gap on the
//     link can take both: an OC receive overrun, or an OC that resets across
//     the landing and seeds its cache with the flight from its RTC copy. The
//     next repeat reaches it within a second. The OC keeps every clear, and
//     only the first after the flight's frames moves its replay filter's
//     baseline (snapshot_replay_filter.h).
//
// The OC also marks a cache still reading INFLIGHT when END_FLIGHT arrives
// (out_computer/main/snapshot_end_flight_policy.h), which covers the clear
// lost on the link until the next repeat.
//
// Pure so the host tests can step it (tests_cpp/test_landed_clear.cpp).

namespace LandedSends {

inline constexpr uint32_t kRetryMs  = 20;     // a clear that did not go in, tried again
inline constexpr uint32_t kRepeatMs = 1000;   // a clear that did, sent again

struct State
{
    bool     clear_sent      = false;   // a LANDED snapshot clear is in the I2S queue
    bool     end_flight_sent = false;   // END_FLIGHT is in the I2S queue
    bool     clear_tried     = false;   // the clear was tried at clear_tried_ms
    uint32_t clear_tried_ms  = 0;
};

// One LANDED pass at now_ms. send_clear and send_end enqueue their frame and
// say whether the enqueue went through.
template <typename SendClear, typename SendEnd>
inline void service(State& s, uint32_t now_ms, SendClear&& send_clear, SendEnd&& send_end)
{
    const uint32_t gap = s.clear_sent ? kRepeatMs : kRetryMs;
    if (!s.clear_tried || (uint32_t)(now_ms - s.clear_tried_ms) >= gap)
    {
        s.clear_tried    = true;
        s.clear_tried_ms = now_ms;
        if (send_clear()) s.clear_sent = true;
    }
    if (s.clear_sent && !s.end_flight_sent) s.end_flight_sent = send_end();
}

}  // namespace LandedSends
