#include <gtest/gtest.h>
#include "modem_rail_sequence.h"

using modem_rail::RailModel;
using modem_rail::Step;

// #700 — the host drove the LoRa daughterboard's UART while its rail was gated
// off, and nothing said it must not.
//
// Every daughterboard branch on V8 and V9 gates the RAIL, never the signals,
// and there is no series resistance on any of these lines. A host TX pad still
// configured with the rail down idles HIGH at 3.3 V and feeds the module
// through its RX ESD diode — on V8 the module's ground floats (low-side N-FET
// in the return), on V9 the ground is common and the rail is simply dead
// (high-side TPS22810). Different mechanism, identical outcome: the host powers
// a module it believes it switched off, unbounded, for as long as the rail is
// down.
//
// UartModemBackend::failClosed() dropped act_pin and left the UART installed —
// and TR_UART_Link has no end(), so it stayed installed for the rest of the
// boot. The same hole was in the re-attach failure path. The base station runs
// the same backend and only recently gained a real LORA_ACT (da6f876), so it
// inherited the hole the moment its gate became real.
//
// The rules are trivial, which is the point: they were violated because nothing
// wrote them down. UartModemBackend now executes these sequences rather than
// open-coding an order per call site.

TEST(ModemRailSequence, PowerDownStopsDrivingBeforeTheRailGoes) {
    EXPECT_EQ(modem_rail::kPowerDownLen, 2u);
    EXPECT_EQ(modem_rail::kPowerDown[0], Step::ParkUart);
    EXPECT_EQ(modem_rail::kPowerDown[1], Step::RailDown);
}

TEST(ModemRailSequence, PowerUpRaisesTheRailBeforeDriving) {
    EXPECT_EQ(modem_rail::kPowerUpLen, 2u);
    EXPECT_EQ(modem_rail::kPowerUp[0], Step::RailUp);
    EXPECT_EQ(modem_rail::kPowerUp[1], Step::AttachUart);
}

// THE invariant, asserted rather than eyeballed.
TEST(ModemRailSequence, TheShippedSequencesNeverDriveADeadRail) {
    RailModel m;
    for (auto s : modem_rail::kPowerUp)   m.step(s);
    for (auto s : modem_rail::kPowerDown) m.step(s);
    for (auto s : modem_rail::kPowerUp)   m.step(s);   // the re-attach case
    for (auto s : modem_rail::kPowerDown) m.step(s);   // re-attach failed
    EXPECT_FALSE(m.violated);
    EXPECT_FALSE(m.uart_driving) << "left driving after a power-down";
}

// The OLD behaviour, so the test demonstrates the defect rather than asserting
// a preference: failClosed dropped the rail with the UART still attached.
TEST(ModemRailSequence, TheOldOrderingIsDetectedAsAViolation) {
    RailModel m;
    m.step(Step::RailUp);
    m.step(Step::AttachUart);
    m.step(Step::RailDown);          // <- what failClosed() used to do
    EXPECT_TRUE(m.violated) << "the model cannot see the bug it exists to catch";
    EXPECT_TRUE(m.uart_driving);
}

// Parking after the fact does not undo it: the feed already happened, and a
// latched violation says so.
TEST(ModemRailSequence, ParkingAfterTheRailDropsIsStillAViolation) {
    RailModel m;
    m.step(Step::RailUp);
    m.step(Step::AttachUart);
    m.step(Step::RailDown);
    m.step(Step::ParkUart);
    EXPECT_TRUE(m.violated);
    EXPECT_FALSE(m.uart_driving);   // quiet NOW, but it fed the module first
}

// A power-down from the parked state (begin() never opened the link) is fine.
TEST(ModemRailSequence, PowerDownFromAlreadyParkedIsClean) {
    RailModel m;
    for (auto s : modem_rail::kPowerDown) m.step(s);
    EXPECT_FALSE(m.violated);
}
