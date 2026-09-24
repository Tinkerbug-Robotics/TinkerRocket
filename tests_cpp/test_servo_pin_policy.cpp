// servoPinsValid() read the rocket-computer-mini as having four servos.
//
// The board headers spell an absent pin two ways. V7/V8/V9 declare the servo
// pins as uint8_t GPIO numbers (255 = unmapped); M1, which has no servos,
// declares all four as `static constexpr int ... = -1`. main.cpp compared each
// pin with `!= 255U`, and an int -1 converts to 0xFFFFFFFF under the usual
// arithmetic conversions, so the mini passed: setup_fc ran
// servo_control.begin() with every pin narrowed to uint8_t 255, all four LEDC
// channel configs failed, and servo_enabled was never forced off.

#include <gtest/gtest.h>

#include <climits>
#include <stdint.h>

#include "servo_pin_policy.h"

using ServoPinPolicy::allPinsMapped;
using ServoPinPolicy::pinMapped;

namespace {

// Both spellings, declared exactly as the board headers declare them.
struct V9Pins {   // board/board_v9.h (V8 is the same map)
    static constexpr uint8_t SERVO_PIN_1 = 45;
    static constexpr uint8_t SERVO_PIN_2 = 44;
    static constexpr uint8_t SERVO_PIN_3 = 43;
    static constexpr uint8_t SERVO_PIN_4 = 54;
};
struct M1Pins {   // board/board_m1.h — no servos on this board
    static constexpr int SERVO_PIN_1 = -1;
    static constexpr int SERVO_PIN_2 = -1;
    static constexpr int SERVO_PIN_3 = -1;
    static constexpr int SERVO_PIN_4 = -1;
};

// main.cpp's servoPinsValid(), over one board's pins.
template <typename Board>
bool servoPinsValid()
{
    return allPinsMapped(Board::SERVO_PIN_1, Board::SERVO_PIN_2,
                         Board::SERVO_PIN_3, Board::SERVO_PIN_4);
}

}  // namespace

TEST(ServoPinPolicy, V9Uint8PinsAreMapped) {
    EXPECT_TRUE(servoPinsValid<V9Pins>());
}

TEST(ServoPinPolicy, M1IntMinusOnePinsAreUnmapped) {
    // THE BUG. The old `SERVO_PIN_1 != 255U` compared 0xFFFFFFFF with 255 and
    // said yes; begin() then ran against pin 255 on all four channels.
    EXPECT_FALSE(servoPinsValid<M1Pins>());
    EXPECT_FALSE(pinMapped(M1Pins::SERVO_PIN_1));
}

TEST(ServoPinPolicy, TheUint8SentinelStillMeansUnmapped) {
    // 255 is what the old comparison was written for; it keeps its meaning.
    constexpr uint8_t kUnmapped = 255;
    EXPECT_FALSE(pinMapped(kUnmapped));
    EXPECT_FALSE(allPinsMapped(V9Pins::SERVO_PIN_1, V9Pins::SERVO_PIN_2,
                               V9Pins::SERVO_PIN_3, kUnmapped));
}

TEST(ServoPinPolicy, AnyNegativePinIsUnmapped) {
    EXPECT_FALSE(pinMapped(-1));
    EXPECT_FALSE(pinMapped(-2));
    EXPECT_FALSE(pinMapped(INT_MIN));
}

TEST(ServoPinPolicy, EveryPinBelowTheSentinelIsMapped) {
    // GPIO0 included: `>= 0`, not `> 0`.
    for (int pin = 0; pin < 255; ++pin)
    {
        EXPECT_TRUE(pinMapped(pin)) << "GPIO" << pin;
    }
}

TEST(ServoPinPolicy, OneUnmappedPinDisablesAllFour) {
    // Unchanged from the old predicate: the four outputs are one set.
    EXPECT_FALSE(allPinsMapped(-1, 44, 43, 54));
    EXPECT_FALSE(allPinsMapped(45, -1, 43, 54));
    EXPECT_FALSE(allPinsMapped(45, 44, -1, 54));
    EXPECT_FALSE(allPinsMapped(45, 44, 43, -1));
    EXPECT_TRUE(allPinsMapped(45, 44, 43, 54));
}
