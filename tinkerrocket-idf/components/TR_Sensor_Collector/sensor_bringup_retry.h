#pragma once

#include <stdint.h>

// #1773: a sensor's bring-up, bounded.
//
// The MMC5983MA's bring-up used to retry its product-ID read forever, inside
// setup_fc, before the flight task exists and before an in-flight reboot's
// recovery: a part that did not answer (a cold joint, a missing or dead part,
// or one that stopped answering in flight before an FC reboot) kept the FC in
// setup for good, and the setup watchdog's restart only ran into it again.
// The barometer and the GNSS were already bounded, with a degraded mode after.
//
// retryUntil tries `attempt` until it succeeds or `timeout_ms` has passed
// since the first try, calling `recover` (a soft reset, the waits) between
// tries. The deadline is checked before each retry, never during one, so one
// full attempt always completes (signed compare: wrap-safe). `attempts`
// counts the tries made.
//
// Pure so it is host-testable (tests_cpp/test_sensor_bringup_retry.cpp).
namespace SensorBringup {

template <class Attempt, class Recover, class NowMs>
bool retryUntil(Attempt&& attempt, Recover&& recover, NowMs&& now_ms, uint32_t timeout_ms, uint32_t& attempts)
{
    const uint32_t deadline = now_ms() + timeout_ms;
    attempts = 0;
    for (;;)
    {
        attempts++;
        if (attempt())
            return true;
        if ((int32_t)(now_ms() - deadline) >= 0)
            return false;
        recover();
    }
}

}  // namespace SensorBringup
