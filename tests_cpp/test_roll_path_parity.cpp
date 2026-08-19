// Parity between the FC's TWO roll-only control paths.
//
// The ground test is the flight-safety gate — if it can pass while the flying
// path does something different, it is not a gate at all.  But roll-only
// control is mixed in two different places:
//
//   GROUND TEST (main.cpp, ground_test_active, control mode "Roll Only")
//     roll_rate_pid_standalone -> TR_ControlMixer::mixToFins(roll,0,0,..)
//                              -> TR_ServoControl::setServoAngles()
//
//   FLIGHT (main.cpp, roll-only branch: powered flight / guidance off)
//     TR_ServoControl's internal PID -> its own roll-only drive
//
// They diverged: "Reverse roll" (FinConfigData::roll_reverse_mask) reached the
// mixer only, so the ground test honoured the toggle and the flight ignored
// it — a rocket could pass a bench roll check and then roll the wrong way.
//
// These tests run BOTH paths end-to-end into their own TR_ServoControl and
// compare the pulses actually written to the four channels.  Anything that
// makes one path mix differently from the other — a sign, a mask, a clamp, a
// bias, the deg->us calibration — breaks them.  #268 already keeps the two
// PIDs' gains and limits in lockstep; this covers the mixing that follows.

#include <gtest/gtest.h>
#include <Arduino.h>  // host shim: setMockMicros()
#include "TR_ServoControl_ledc_mult.h"
#include "TR_ControlMixer.h"
#include "TR_PID.h"

namespace {

constexpr float KP      = 1.0f;   // P-only: roll command == rate error
constexpr float KI      = 0.0f;
constexpr float KD      = 0.0f;
constexpr float MIN_CMD = -60.0f;
constexpr float MAX_CMD =  60.0f;
// Wider than any command the P-only loop produces below, so the mixer's clamp
// never fires and the comparison is of the MIX, not of a saturation limit.
constexpr float MAX_FIN = 60.0f;

constexpr uint32_t TICK_US = 2000;  // 500 Hz control loop

// Both paths drive a servo block configured identically to the airframe's —
// unequal biases included, so a path that dropped the per-servo bias shows up.
TR_ServoControl makeServos() {
    return TR_ServoControl(1, 2, 3, 4,        // pins
                           0, -15, 7, 22,     // bias us (deliberately unequal)
                           50, 1000, 2000,    // hz, min/max us
                           KP, KI, KD, MIN_CMD, MAX_CMD);
}

class RollPathParityTest : public ::testing::Test {
protected:
    // Ground-test path: standalone PID + control_mixer, driven onto its servos.
    TR_ServoControl ground_servos = makeServos();
    TR_PID          standalone{KP, KI, KD, MAX_CMD, MIN_CMD};
    TR_ControlMixer mixer;

    // Flight path: TR_ServoControl's internal PID and its own roll-only drive.
    TR_ServoControl flight_servos = makeServos();

    uint32_t now_us_ = 0;

    void SetUp() override {
        setMockMicros(0);
        ground_servos.begin();
        flight_servos.begin();
        // TR_PID returns 0 on its first call (dt bootstrap) — burn one tick on
        // every controller so the comparisons are all steady-state.
        tick();
        standalone.computePID(0.0f, 0.0f);
        flight_servos.control(0.0f);
    }

    void tick() {
        now_us_ += TICK_US;
        setMockMicros(now_us_);
    }

    // Apply the same fin layout to both paths, exactly as main.cpp does at boot
    // and on every FIN_CONFIG frame.
    void setLayout(const float azimuth_deg[4], uint8_t tilt_mask, uint8_t roll_mask) {
        mixer.setFinLayout(azimuth_deg, tilt_mask, roll_mask);
        flight_servos.setRollReverseMask(roll_mask);
    }

    // One control tick down both paths at the same measured roll rate, then
    // compare what reached the four channels.
    void expectSamePulses(float roll_rate_dps) {
        tick();

        // GROUND TEST: standalone PID -> mixer -> servos.  Pure roll, so the
        // pitch/yaw fin commands are zero (main.cpp leaves them 0 in Roll Only).
        float roll_fin_cmd = standalone.computePID(0.0f, -roll_rate_dps);
        float deflections[4];
        mixer.mixToFins(roll_fin_cmd, 0.0f, 0.0f, MAX_FIN, deflections);
        ground_servos.setServoAngles(deflections);

        // FLIGHT: internal PID -> roll-only drive.
        flight_servos.control(-roll_rate_dps);

        ASSERT_NEAR(flight_servos.getRollCmdDeg(), roll_fin_cmd, 1e-4f)
            << "the two PIDs disagree before mixing — parity test is invalid";
        for (int i = 0; i < 4; ++i) {
            EXPECT_EQ(flight_servos.getServoPulseUs(i), ground_servos.getServoPulseUs(i))
                << "servo " << (i + 1) << " differs between the ground-test and flight "
                << "roll paths at roll_rate=" << roll_rate_dps << " dps";
        }
    }
};

// The default airframe: "+" ring, nothing reversed.
TEST_F(RollPathParityTest, MatchesWithNoReverses) {
    const float az[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    setLayout(az, 0, 0);
    expectSamePulses( 25.0f);
    expectSamePulses(-25.0f);
}

// The case that was broken: roll reversed on the whole airframe.  Pre-fix the
// ground test flipped and the flight did not.
TEST_F(RollPathParityTest, MatchesWithRollReversedOnAllFins) {
    const float az[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    setLayout(az, 0, 0b1111);
    expectSamePulses( 25.0f);
    expectSamePulses(-25.0f);
}

// Every per-servo roll-reverse combination, on both ring orientations.  A
// single mis-linked servo has to read the same on the bench as in the air too.
TEST_F(RollPathParityTest, MatchesForEveryRollReverseCombination) {
    const float plus_ring[4]  = {0.0f, 90.0f, 180.0f, 270.0f};
    const float cross_ring[4] = {45.0f, 135.0f, 225.0f, 315.0f};

    for (int ring = 0; ring < 2; ++ring) {
        for (uint8_t roll_mask = 0; roll_mask < 16; ++roll_mask) {
            // Tilt reverses must not leak into the roll mix, so sweep them too.
            for (uint8_t tilt_mask : {uint8_t(0), uint8_t(0b0101), uint8_t(0b1111)}) {
                setLayout(ring == 0 ? plus_ring : cross_ring, tilt_mask, roll_mask);
                SCOPED_TRACE(testing::Message()
                             << "ring=" << ring << " roll_mask=" << (int)roll_mask
                             << " tilt_mask=" << (int)tilt_mask);
                expectSamePulses(18.0f);
            }
        }
    }
}

// The fin calibration (#267) rescales deg->us.  Both paths must pick it up, or
// a calibrated airframe's bench deflection would not be its flight deflection.
TEST_F(RollPathParityTest, MatchesUnderANarrowFinCalibration) {
    const float az[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    setLayout(az, 0, 0b1010);
    ground_servos.setFinCalibration(-12.0f, 12.0f);
    flight_servos.setFinCalibration(-12.0f, 12.0f);
    expectSamePulses(20.0f);
    // Past the calibrated travel: both paths must clamp to the same endpoint
    // rather than one saturating on pulse and the other on angle.
    expectSamePulses(55.0f);
}

}  // namespace
