#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "BleCommandRing.h"

using tr_ble::CommandRing;
using tr_ble::PendingCommand;

// The #517 fix: BLE command intake was a SINGLE latch. The NimBLE write callback
// overwrote it unconditionally and the loop task drained it once per iteration,
// so two commands arriving between two polls lost the first outright. The app's
// connect-time sync fires ~13 commands back-to-back with no pacing (#366), and
// any loop stall longer than their ~60-90 ms spacing dropped one.
//
// These tests pin the ring that replaces it: commands are delivered in order,
// each keeps the payload/filename/page that arrived with it, and a full ring
// drops the newest rather than rotating out older commands the app is waiting on.

namespace {

// Command ids, from onCommandWrite's dispatch chain.
constexpr uint8_t CMD_FILE_LIST = 2;
constexpr uint8_t CMD_DELETE    = 3;
constexpr uint8_t CMD_DOWNLOAD  = 4;
constexpr uint8_t CMD_SIM_CFG   = 5;
constexpr uint8_t CMD_SIM_START = 6;

PendingCommand plain(uint8_t cmd) {
    PendingCommand e;
    e.cmd = cmd;
    return e;
}

PendingCommand withPayload(uint8_t cmd, const uint8_t* p, size_t n) {
    PendingCommand e;
    e.cmd = cmd;
    memcpy(e.payload, p, n);
    e.payload_len = n;
    return e;
}

PendingCommand withDelete(const char* name) {
    PendingCommand e;
    e.cmd = CMD_DELETE;
    snprintf(e.delete_name, sizeof(e.delete_name), "%s", name);
    return e;
}

PendingCommand withDownload(const char* name) {
    PendingCommand e;
    e.cmd = CMD_DOWNLOAD;
    snprintf(e.download_name, sizeof(e.download_name), "%s", name);
    return e;
}

}  // namespace

// ---- The regression: a second command must not evict the first ----

// Before the fix, a sim-config (cmd 5) followed by a sim-start (cmd 6) before
// the loop polled left ONLY cmd 6 — the rocket then flew with stale config.
TEST(BleCommandRing, SecondCommandDoesNotEvictTheFirst) {
    CommandRing ring;
    const uint8_t cfg[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    ASSERT_TRUE(ring.push(withPayload(CMD_SIM_CFG, cfg, sizeof(cfg))));
    ASSERT_TRUE(ring.push(plain(CMD_SIM_START)));
    EXPECT_EQ(ring.size(), 2u);

    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_SIM_CFG) << "the config must survive the start that followed it";
    EXPECT_EQ(out.payload_len, sizeof(cfg));
    EXPECT_EQ(memcmp(out.payload, cfg, sizeof(cfg)), 0);

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_SIM_START);

    EXPECT_TRUE(ring.empty());
}

// The #366 pattern: ActiveRocketSyncer fires ~13 config commands back-to-back
// with no pacing. All of them must survive a loop that never polls in between.
TEST(BleCommandRing, FullConnectTimeSyncBurstSurvivesAnUndrainedLoop) {
    CommandRing ring;
    constexpr int kBurst = 13;
    for (int i = 0; i < kBurst; i++) {
        ASSERT_TRUE(ring.push(plain((uint8_t)(20 + i)))) << "command " << i << " of the sync burst";
    }
    EXPECT_EQ(ring.size(), (size_t)kBurst);

    for (int i = 0; i < kBurst; i++) {
        PendingCommand out;
        ASSERT_TRUE(ring.pop(out));
        EXPECT_EQ(out.cmd, (uint8_t)(20 + i)) << "burst must drain in order";
    }
    EXPECT_TRUE(ring.empty());
}

// ---- Ordering / wrap ----

TEST(BleCommandRing, DrainsInFifoOrder) {
    CommandRing ring;
    for (uint8_t c = 1; c <= 5; c++) ASSERT_TRUE(ring.push(plain(c)));

    for (uint8_t expected = 1; expected <= 5; expected++) {
        PendingCommand out;
        ASSERT_TRUE(ring.pop(out));
        EXPECT_EQ(out.cmd, expected);
    }
}

TEST(BleCommandRing, RingWrapsWithoutLosingOrder) {
    CommandRing ring;
    for (int round = 0; round < 3; round++) {
        for (size_t i = 0; i < tr_ble::kRingDepth; i++) {
            ASSERT_TRUE(ring.push(plain((uint8_t)(i + 1))));
        }
        ASSERT_TRUE(ring.full());
        for (size_t i = 0; i < tr_ble::kRingDepth; i++) {
            PendingCommand out;
            ASSERT_TRUE(ring.pop(out));
            EXPECT_EQ(out.cmd, (uint8_t)(i + 1)) << "round " << round;
        }
        EXPECT_TRUE(ring.empty());
    }
}

// ---- Full ring: drop the newest, keep what the app is waiting on ----

TEST(BleCommandRing, FullRingRejectsNewestAndKeepsOlderCommands) {
    CommandRing ring;
    for (size_t i = 0; i < tr_ble::kRingDepth; i++) {
        ASSERT_TRUE(ring.push(plain((uint8_t)(i + 1))));
    }
    ASSERT_TRUE(ring.full());

    EXPECT_FALSE(ring.push(plain(99)));

    // The rejection must not have displaced anything.
    EXPECT_EQ(ring.size(), tr_ble::kRingDepth);
    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, 1) << "the oldest command must still be first in line";
}

// ---- Each command keeps the data that arrived with it ----

// The single latch shared one filename buffer across commands, so a delete name
// could still be live when an unrelated command was consumed. The OC reads
// getDownloadFilename() on EVERY command, so a leaked name would kick off a
// spurious multi-second file transfer.
TEST(BleCommandRing, DeleteAndDownloadNamesDoNotLeakAcrossCommands) {
    CommandRing ring;
    ASSERT_TRUE(ring.push(withDelete("old_flight.bin")));
    ASSERT_TRUE(ring.push(plain(CMD_SIM_START)));
    ASSERT_TRUE(ring.push(withDownload("flight_42.bin")));

    PendingCommand out;

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_DELETE);
    EXPECT_STREQ(out.delete_name, "old_flight.bin");
    EXPECT_STREQ(out.download_name, "") << "a delete must not carry a download target";

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_SIM_START);
    EXPECT_STREQ(out.delete_name, "") << "the delete name must not leak onto the next command";
    EXPECT_STREQ(out.download_name, "");

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_DOWNLOAD);
    EXPECT_STREQ(out.download_name, "flight_42.bin");
    EXPECT_STREQ(out.delete_name, "");
}

TEST(BleCommandRing, PayloadsDoNotLeakAcrossCommands) {
    CommandRing ring;
    const uint8_t cfg[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    ASSERT_TRUE(ring.push(withPayload(CMD_SIM_CFG, cfg, sizeof(cfg))));
    ASSERT_TRUE(ring.push(plain(CMD_SIM_START)));   // carries no payload

    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.payload_len, 4u);

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.cmd, CMD_SIM_START);
    EXPECT_EQ(out.payload_len, 0u) << "sim-start must not inherit the config's payload";
}

TEST(BleCommandRing, FileListPageTravelsWithItsCommand) {
    CommandRing ring;
    PendingCommand e = plain(CMD_FILE_LIST);
    e.file_list_page = 3;
    ASSERT_TRUE(ring.push(e));
    ASSERT_TRUE(ring.push(plain(CMD_SIM_START)));

    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.file_list_page, 3);

    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.file_list_page, 0) << "page must not leak onto the next command";
}

// A reused slot must not resurrect the previous occupant's data.
TEST(BleCommandRing, ReusedSlotDoesNotResurrectStaleData) {
    CommandRing ring;
    ASSERT_TRUE(ring.push(withDownload("stale.bin")));
    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    ASSERT_STREQ(out.download_name, "stale.bin");

    // Fill and drain enough to come back around to that same slot.
    for (size_t i = 0; i < tr_ble::kRingDepth; i++) {
        ASSERT_TRUE(ring.push(plain(CMD_SIM_START)));
    }
    for (size_t i = 0; i < tr_ble::kRingDepth; i++) {
        ASSERT_TRUE(ring.pop(out));
        EXPECT_EQ(out.cmd, CMD_SIM_START);
        EXPECT_STREQ(out.download_name, "") << "slot " << i << " resurrected a stale filename";
        EXPECT_EQ(out.payload_len, 0u);
    }
}

// Max-length payload (RollProfileData is the largest at 76 B) round-trips intact.
TEST(BleCommandRing, MaxPayloadRoundTripsIntact) {
    CommandRing ring;
    uint8_t big[tr_ble::kMaxPayload];
    for (size_t i = 0; i < sizeof(big); i++) big[i] = (uint8_t)(i * 7 + 1);

    ASSERT_TRUE(ring.push(withPayload(66, big, sizeof(big))));
    PendingCommand out;
    ASSERT_TRUE(ring.pop(out));
    EXPECT_EQ(out.payload_len, tr_ble::kMaxPayload);
    EXPECT_EQ(memcmp(out.payload, big, sizeof(big)), 0);
}

// ---- Empty-ring safety ----

TEST(BleCommandRing, PopOnEmptyReturnsFalseAndLeavesOutputUntouched) {
    CommandRing ring;
    PendingCommand out = plain(0x7E);   // sentinel

    EXPECT_FALSE(ring.pop(out));
    EXPECT_EQ(out.cmd, 0x7E) << "pop() must not clobber the caller's buffer when empty";
    EXPECT_TRUE(ring.empty());
    EXPECT_EQ(ring.size(), 0u);
}

TEST(BleCommandRing, StartsEmpty) {
    CommandRing ring;
    EXPECT_TRUE(ring.empty());
    EXPECT_FALSE(ring.full());
    EXPECT_EQ(ring.size(), 0u);
}
