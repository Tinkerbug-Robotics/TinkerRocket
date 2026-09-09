// #732 item 3b — the piezo must never be left DC-energised.
//
// LS1 is a ~16 ohm magnetic coil. A pin left high is ~200 mA / 0.7 W through
// it, indefinitely, which the hardware review (I51) says cooks the transducer
// and loads the sensor rail. These tests interleave the transitions the way
// two cores can and assert the pin always settles low.
#include <gtest/gtest.h>
#include "piezo_wave_policy.h"

using PiezoWavePolicy::PinAction;
using PiezoWavePolicy::State;
using PiezoWavePolicy::onStart;
using PiezoWavePolicy::onStop;
using PiezoWavePolicy::onTick;

namespace {

// Mirrors main.cpp's piezoApply(): None leaves the pin alone.
void apply(PinAction a, bool &pin)
{
    if (a == PinAction::DriveHigh) pin = true;
    else if (a == PinAction::DriveLow) pin = false;
}

constexpr int64_t kStart = 1'000'000;
constexpr uint32_t kDurMs = 100;

}   // namespace

TEST(PiezoWavePolicy, StartsLowAndTogglesASquareWave)
{
    State s;
    bool pin = false, stop = false;
    apply(onStart(s, kStart, kDurMs), pin);
    EXPECT_FALSE(pin) << "a wave must begin from a known-low pin";

    apply(onTick(s, kStart + 227, stop), pin);
    EXPECT_TRUE(pin);
    EXPECT_FALSE(stop);
    apply(onTick(s, kStart + 454, stop), pin);
    EXPECT_FALSE(pin);
    apply(onTick(s, kStart + 681, stop), pin);
    EXPECT_TRUE(pin);
}

TEST(PiezoWavePolicy, ExpiryDrivesLowAndAsksForTheTimerToStop)
{
    State s;
    bool pin = false, stop = false;
    apply(onStart(s, kStart, kDurMs), pin);
    apply(onTick(s, kStart + 227, stop), pin);
    ASSERT_TRUE(pin) << "precondition: expire from the HIGH half of the cycle";

    apply(onTick(s, kStart + (int64_t)kDurMs * 1000, stop), pin);
    EXPECT_FALSE(pin) << "the last edge of a beep must be low, not high";
    EXPECT_TRUE(stop);
    EXPECT_FALSE(s.active);
}

// The regression. Ordering: the callback is already inside its transition
// when the stopper runs. Under the old code the callback's toggle was the
// last write and left the coil energised with nothing armed to release it.
TEST(PiezoWavePolicy, TickThatLosesTheRaceWithStopCannotLatchTheCoilOn)
{
    State s;
    bool pin = false, stop = false;
    apply(onStart(s, kStart, kDurMs), pin);

    // Land on the LOW half, so the next toggle would drive HIGH — the exact
    // interleaving that latched the coil.
    apply(onTick(s, kStart + 227, stop), pin);
    apply(onTick(s, kStart + 454, stop), pin);
    ASSERT_FALSE(pin);

    apply(onStop(s), pin);
    ASSERT_FALSE(pin);

    // The straggling callback completes here, still well inside the duration.
    const PinAction late = onTick(s, kStart + 681, stop);
    EXPECT_EQ(late, PinAction::None) << "a tick after a stop must not write the pin";
    apply(late, pin);
    EXPECT_FALSE(pin) << "#732 item 3b: the coil would sit at 200 mA until reboot";
    EXPECT_FALSE(stop);
}

TEST(PiezoWavePolicy, NoTickSequenceAfterStopEverDrivesHigh)
{
    State s;
    bool pin = false, stop = false;
    apply(onStart(s, kStart, kDurMs), pin);
    apply(onTick(s, kStart + 227, stop), pin);
    ASSERT_TRUE(pin) << "stop from the HIGH half — the worst case";

    apply(onStop(s), pin);
    ASSERT_FALSE(pin);

    // Nothing re-arms the timer, so every one of these is a straggler.
    for (int i = 1; i <= 200; ++i)
    {
        const PinAction a = onTick(s, kStart + 227 * (int64_t)i, stop);
        ASSERT_EQ(a, PinAction::None) << "tick " << i;
        apply(a, pin);
        ASSERT_FALSE(pin) << "tick " << i;
    }
}

TEST(PiezoWavePolicy, StopIsIdempotentAndAlwaysAssertsTheSafeLevel)
{
    State s;
    bool pin = true;   // pin left high by something outside the state machine
    EXPECT_EQ(onStop(s), PinAction::DriveLow)
        << "a stop against an idle wave must still drive low";
    apply(onStop(s), pin);
    EXPECT_FALSE(pin);
    apply(onStop(s), pin);
    EXPECT_FALSE(pin);
}

TEST(PiezoWavePolicy, RestartAfterAStopRunsANormalWave)
{
    State s;
    bool pin = false, stop = false;
    apply(onStart(s, kStart, kDurMs), pin);
    apply(onTick(s, kStart + 227, stop), pin);
    apply(onStop(s), pin);

    const int64_t t2 = kStart + 5'000'000;
    apply(onStart(s, t2, kDurMs), pin);
    EXPECT_FALSE(pin);
    apply(onTick(s, t2 + 227, stop), pin);
    EXPECT_TRUE(pin) << "a stopped-then-restarted wave must sound again";
    EXPECT_FALSE(stop);
}

// #382 lives on the same path: the end time is 64-bit so a long-uptime beep
// does not terminate on its first toggle.
TEST(PiezoWavePolicy, EndTimeSurvivesPastTheThirtyTwoBitWrap)
{
    State s;
    bool pin = false, stop = false;
    const int64_t late = 4'300'000'000LL;   // ~71.7 min, just past the wrap
    apply(onStart(s, late, kDurMs), pin);
    apply(onTick(s, late + 227, stop), pin);
    EXPECT_TRUE(pin) << "#382: the beep must not die on its first toggle";
    EXPECT_FALSE(stop);
}
