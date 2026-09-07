// Host-side test for the FC's one-shot command session gate (#1105).
//
// The OC repeats a served command for CMD_REPEAT_LIMIT polls and advances its
// serving slot only on polls it receives, so an FC reset inside that window
// freezes the slot; the rebooted FC's dedup key boots to zero, so it read the
// frozen PYRO_FIRE_TEST back as new and fired the channel a second time. The
// gate admits a one-shot actuating command only on a 0 -> cmd edge observed
// this boot — the OC always serves an idle poll between commands, so the only
// command that can arrive before any idle is one that predates the session.
//
// Each test drives the gate with the exact read sequence the FC sees:
// observe() per successful status read (setup_fc boot read, main-loop reads),
// nothing for a failed read, admits() at dispatch.

#include <gtest/gtest.h>

#include <cstdint>

#include "oc_cmd_session_gate.h"

namespace
{

TEST(OcCmdSessionGate, FrozenFireTestAtBootIsRefusedUntilAnIdlePoll)
{
    // The issue's scenario: the FC reset during the repeat window, so the
    // first thing this boot reads is the OC still serving PYRO_FIRE_TEST.
    OcCmdSessionGate g;
    g.observe(PYRO_FIRE_TEST);                 // setup_fc boot read
    EXPECT_FALSE(g.admits(PYRO_FIRE_TEST));
    g.observe(PYRO_FIRE_TEST);                 // main-loop repeats
    g.observe(PYRO_FIRE_TEST);
    EXPECT_FALSE(g.admits(PYRO_FIRE_TEST));
    // The OC clears the slot and serves its idle poll: the session now has
    // an edge, and a FRESH fire test (a new tap) is admitted.
    g.observe(0);
    g.observe(PYRO_FIRE_TEST);
    EXPECT_TRUE(g.admits(PYRO_FIRE_TEST));
}

TEST(OcCmdSessionGate, FailedBootReadDoesNotOpenTheGate)
{
    // The boot read timed out or unpacked garbage (an FC reset mid-transfer
    // can leave the slave TX path desynced): nothing was observed, so the
    // first successful main-loop read of the frozen command is still refused.
    OcCmdSessionGate g;
    g.observe(PYRO_FIRE_TEST);                 // first successful loop read
    EXPECT_FALSE(g.admits(PYRO_FIRE_TEST));
}

TEST(OcCmdSessionGate, IdleAtBootThenFreshFireIsAdmitted)
{
    // Normal life: the FC comes up to an idle OC, the operator taps fire later.
    OcCmdSessionGate g;
    g.observe(0);
    EXPECT_TRUE(g.admits(PYRO_FIRE_TEST));
    g.observe(PYRO_FIRE_TEST);
    EXPECT_TRUE(g.admits(PYRO_FIRE_TEST));
}

TEST(OcCmdSessionGate, ConfigServedAtBootIsAdmitted)
{
    // The connect-before-power-on sync: the OC's I2C pre-fill pops the first
    // config command before the FC has booted, so the FC's first read of the
    // session is a config with no idle poll in front of it. That must still
    // apply — the gate is for one-shots only.
    OcCmdSessionGate g;
    g.observe(SERVO_CONFIG_PENDING);
    EXPECT_TRUE(g.admits(SERVO_CONFIG_PENDING));
    g.observe(PID_CONFIG_PENDING);
    EXPECT_TRUE(g.admits(PID_CONFIG_PENDING));
    // ...and the sync's idle gaps open the gate for anything after them.
    g.observe(0);
    EXPECT_TRUE(g.admits(SERVO_TEST_PENDING));
}

TEST(OcCmdSessionGate, SimPairQueuedAcrossPowerOnStillStarts)
{
    // SIM_CONFIG_PENDING then SIM_START_CMD, queued while the rail was off:
    // the config is served first (admitted, not a one-shot), the OC's idle
    // gap follows, and the start rides the resulting edge.
    OcCmdSessionGate g;
    g.observe(SIM_CONFIG_PENDING);
    EXPECT_TRUE(g.admits(SIM_CONFIG_PENDING));
    g.observe(SIM_CONFIG_PENDING);
    g.observe(SIM_CONFIG_PENDING);
    g.observe(0);
    g.observe(SIM_START_CMD);
    EXPECT_TRUE(g.admits(SIM_START_CMD));
}

TEST(OcCmdSessionGate, StopsAreNeverRefused)
{
    OcCmdSessionGate g;   // no idle seen yet
    EXPECT_TRUE(g.admits(SERVO_TEST_STOP));
    EXPECT_TRUE(g.admits(SERVO_REPLAY_STOP));
    EXPECT_TRUE(g.admits(GROUND_TEST_STOP));
    EXPECT_TRUE(g.admits(SIM_STOP_CMD));
    EXPECT_TRUE(g.admits(RECOVERY_END_PENDING));
}

TEST(OcCmdSessionGate, EveryOneShotIsRefusedBeforeAnIdlePoll)
{
    const uint8_t one_shots[] = {PYRO_FIRE_TEST, PYRO_CONT_TEST, SERVO_TEST_PENDING,
                                 SERVO_REPLAY_PENDING, GROUND_TEST_START, SIM_START_CMD};
    for (uint8_t c : one_shots)
    {
        OcCmdSessionGate g;
        g.observe(c);
        EXPECT_FALSE(g.admits(c)) << "cmd 0x" << std::hex << (unsigned)c;
        g.observe(0);
        EXPECT_TRUE(g.admits(c)) << "cmd 0x" << std::hex << (unsigned)c;
    }
}

TEST(OcCmdSessionGate, IdleIsStickyAcrossAMissedIdlePoll)
{
    // A later I2C read failure that lands on the OC's single idle poll must
    // not refuse the command behind it: the gate keys on "an idle has been
    // seen this boot", not on the previous read.
    OcCmdSessionGate g;
    g.observe(0);
    g.observe(SERVO_CONFIG_PENDING);
    g.observe(SERVO_CONFIG_PENDING);
    g.observe(SERVO_CONFIG_PENDING);
    // (idle poll lost to a read failure — no observe())
    g.observe(PYRO_CONT_TEST);
    EXPECT_TRUE(g.admits(PYRO_CONT_TEST));
}

}  // namespace
