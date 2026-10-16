// Host tests for finding and setting up the mosaic-G5 (MosaicLink.cpp) and
// for pairing 1PPS edges with GPS seconds (PpsPairer.h), against a scripted
// receiver: it answers only at its own baud and once booted, replies
// "$R: <command>" or "$R? ..." then its prompt (Reference Guide 3.1.3), takes
// a new baud after replying to scs, streams once its 20 Hz stream is set, and
// remembers what eccf saved across a reset.
#include <gtest/gtest.h>

#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "MosaicLink.h"
#include "PpsPairer.h"

using namespace mosaic;

namespace {

const SetupCommand kCmds[] = {
    {"sdio, {cd}, CMD, SBF", true},
    {"srd, High, Unlimited", true},
    {"sso, Stream7, {cd}, MeasEpoch+MeasExtra+PVTGeodetic, msec50", true},
    {"spps, sec1, Low2High, 0.00, GPS, 60, 5.000", true},
    {"sfn, DSK1, Incremental, trfc", false},
};
constexpr size_t kNCmds = sizeof(kCmds) / sizeof(kCmds[0]);

bool startsWith(const std::string& s, const std::string& p) { return s.compare(0, p.size(), p) == 0; }

struct FakeReceiver
{
    uint32_t baud = 115200;
    uint32_t boots_at_ms = 0;
    bool cmd_input = true;            // COM1 takes commands
    bool streaming = false;
    bool refuse_scs = false;
    std::set<std::string> refuse;     // command prefixes answered "$R?"
    // What eccf saved, applied at the next boot.
    bool saved = false;
    uint32_t saved_baud = 115200;
    bool saved_streaming = false;
    int saves = 0;
    std::vector<std::string> received;

    void reboot(uint32_t at_ms)
    {
        boots_at_ms = at_ms;
        baud = saved ? saved_baud : 115200;
        streaming = saved && saved_streaming;
        cmd_input = true;
    }
};

struct Harness
{
    MosaicLink link;
    FakeReceiver rx;
    uint32_t host_baud = 0;
    uint32_t now = 0;
    std::vector<std::string> sent;   // everything the link sent, CR stripped

    void start(LinkConfig cfg = LinkConfig()) { link.start(cfg, kCmds, kNCmds, now); }

    bool booted() const { return now >= rx.boots_at_ms; }

    void run(uint32_t ms, uint32_t step = 5)
    {
        const uint32_t end = now + ms;
        for (; now < end; now += step)
        {
            if (booted() && rx.streaming && host_baud == rx.baud && now % 50 == 0) link.onBlock(now);
            for (;;)
            {
                const LinkAction a = link.poll(now);
                if (a.kind == LinkAction::NONE) break;
                if (a.kind == LinkAction::SET_BAUD)
                {
                    host_baud = a.baud;
                    continue;
                }
                std::string t(a.text);
                ASSERT_FALSE(t.empty());
                ASSERT_EQ(t.back(), '\r');
                t.pop_back();
                sent.push_back(t);
                deliver(t);
            }
        }
    }

    void reply(const std::string& line) { link.onLine(line.c_str(), now); }
    void prompt() { link.onPrompt("COM1", now); }

    void deliver(const std::string& t)
    {
        if (!booted() || host_baud != rx.baud) return;   // lost on the wire
        if (t == "SSSSSSSSSS")
        {
            rx.cmd_input = true;
            reply("$R? SSSSSSSSSS: Invalid command!");
            prompt();
            return;
        }
        if (!rx.cmd_input) return;
        if (t.empty())
        {
            prompt();
            return;
        }
        rx.received.push_back(t);
        for (const auto& r : rx.refuse)
        {
            if (startsWith(t, r))
            {
                reply("$R? " + t + ": Invalid argument!");
                prompt();
                return;
            }
        }
        if (startsWith(t, "lif, Identification"))
        {
            reply("$R; lif, Identification");
            reply("---->");
            reply("$-- BLOCK 2 / 5");
            reply("    <hwplatform product=\"mosaic-G5 P3\"");
            reply("$-- BLOCK 3 / 5");
            reply("    <firmware version=\"1.0.0\" date=\"250731\" rev=\"g794c61\">");
            reply("        <bootloader version=\"2025.04.1-g2b4d387\" date=\"250730\"/>");
            prompt();
            return;
        }
        if (startsWith(t, "scs, COM1, baud"))
        {
            if (rx.refuse_scs)
            {
                reply("$R? " + t + ": Argument 'Rate' is invalid!");
                prompt();
                return;
            }
            reply("$R: " + t);
            prompt();
            rx.baud = static_cast<uint32_t>(std::stoul(t.substr(std::strlen("scs, COM1, baud"))));
            return;
        }
        if (startsWith(t, "sso, Stream7, COM1")) rx.streaming = true;
        if (startsWith(t, "eccf, Current, Boot"))
        {
            rx.saved = true;
            rx.saved_baud = rx.baud;
            rx.saved_streaming = rx.streaming;
            rx.saves++;
        }
        reply("$R: " + t);
        reply("  echo");
        prompt();
    }

    size_t count(const std::string& prefix) const
    {
        size_t n = 0;
        for (const auto& s : rx.received) n += startsWith(s, prefix) ? 1 : 0;
        return n;
    }
};

}  // namespace

TEST(MosaicLink, FactoryReceiverIsFoundRebaudedSetUpAndSaved)
{
    Harness h;
    h.start();
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.link.baud(), 2000000u);
    EXPECT_EQ(h.rx.baud, 2000000u);
    EXPECT_STREQ(h.link.product(), "mosaic-G5 P3");
    EXPECT_STREQ(h.link.firmware(), "1.0.0");
    EXPECT_STREQ(h.link.port(), "COM1");
    EXPECT_EQ(h.link.refused(), 0u);
    EXPECT_TRUE(h.link.saved());
    EXPECT_EQ(h.rx.saves, 1);
    // In order, "{cd}" expanded: identify, baud, the setup, the save.
    const std::vector<std::string> want = {
        "lif, Identification",
        "scs, COM1, baud2000000, bits8, No, bit1, none",
        "sdio, COM1, CMD, SBF",
        "srd, High, Unlimited",
        "sso, Stream7, COM1, MeasEpoch+MeasExtra+PVTGeodetic, msec50",
        "spps, sec1, Low2High, 0.00, GPS, 60, 5.000",
        "sfn, DSK1, Incremental, trfc",
        "eccf, Current, Boot",
    };
    EXPECT_EQ(h.rx.received, want);
}

TEST(MosaicLink, ASavedReceiverIsSetUpAtTheTargetBaudWithNoBaudChange)
{
    Harness h;
    h.rx.baud = 2000000;
    h.rx.streaming = true;
    h.start();
    h.run(3000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.count("scs"), 0u);
    EXPECT_EQ(h.count("eccf"), 1u);   // once per start(), whatever was saved before
    EXPECT_EQ(h.link.baud(), 2000000u);
}

TEST(MosaicLink, AReceiverThatBootsLateIsFound)
{
    Harness h;
    h.rx.boots_at_ms = 7000;
    h.start();
    h.run(6900);
    EXPECT_FALSE(h.link.running());
    EXPECT_TRUE(h.rx.received.empty());
    h.run(3000);
    EXPECT_TRUE(h.link.running());
    EXPECT_EQ(h.rx.baud, 2000000u);
}

TEST(MosaicLink, ARefusedCommandIsCountedAndTheSetupGoesOn)
{
    Harness h;
    h.rx.refuse = {"sfn"};
    h.start();
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.link.refused(), 1u);
    EXPECT_EQ(h.link.requiredFailed(), 0u);
    EXPECT_STREQ(h.link.lastFailure(), "sfn, DSK1, Incremental, trfc");
    EXPECT_EQ(h.count("sfn"), 1u);    // a refusal is not retried

    Harness r;
    r.rx.refuse = {"srd"};
    r.start();
    r.run(5000);
    ASSERT_TRUE(r.link.running());
    EXPECT_EQ(r.link.requiredFailed(), 1u);
}

TEST(MosaicLink, ARefusedBaudLeavesTheLinkAtTheFactoryBaud)
{
    Harness h;
    h.rx.refuse_scs = true;
    h.start();
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.link.baud(), 115200u);
    EXPECT_EQ(h.rx.baud, 115200u);
    EXPECT_GE(h.link.requiredFailed(), 1u);
    EXPECT_EQ(h.count("sso, Stream7"), 1u);   // still set up, at the baud it has
}

TEST(MosaicLink, SilenceInFlightFindsTheReceiverAgainWithoutSavingTwice)
{
    Harness h;
    h.start();
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    ASSERT_TRUE(h.rx.saved);
    // The receiver resets: dark for 3 s, then back on its saved setup.
    h.rx.reboot(h.now + 3000);
    h.run(10000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.link.relinks(), 1u);
    EXPECT_EQ(h.link.setups(), 2u);
    EXPECT_EQ(h.rx.saves, 1);          // saved once per start()
    EXPECT_EQ(h.count("scs"), 1u);     // it came back at the saved 2 Mbaud
    EXPECT_EQ(h.count("sso, Stream7"), 2u);
}

TEST(MosaicLink, APortNotTakingCommandsIsReopened)
{
    Harness h;
    h.rx.baud = 2000000;
    h.rx.streaming = true;
    h.rx.cmd_input = false;
    h.start();
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    bool escaped = false;
    for (const auto& s : h.sent) escaped |= (s == "SSSSSSSSSS");
    EXPECT_TRUE(escaped);
}

TEST(MosaicLink, AOneOffCommandGoesOnceWhileRunning)
{
    Harness h;
    h.start();
    EXPECT_TRUE(h.link.queueCommand("esoc, {cd}, GPSNav+GALNav"));
    EXPECT_FALSE(h.link.queueCommand("esoc, {cd}, GPSNav"));   // one at a time
    h.run(5000);
    ASSERT_TRUE(h.link.running());
    EXPECT_EQ(h.count("esoc, COM1, GPSNav+GALNav"), 1u);
    EXPECT_EQ(h.link.oneOffsSent(), 1u);
    EXPECT_EQ(h.link.oneOffsRefused(), 0u);
    // ... and only after the setup, never in the middle of it.
    EXPECT_EQ(h.rx.received.back(), "esoc, COM1, GPSNav+GALNav");
    EXPECT_TRUE(h.link.queueCommand("esoc, {cd}, BDSNav"));
    h.run(100);
    EXPECT_EQ(h.count("esoc, COM1, BDSNav"), 1u);
}

TEST(MosaicLink, NoReceiverMeansNoSetup)
{
    Harness h;
    h.rx.boots_at_ms = 1u << 30;
    h.start();
    h.run(30000);
    EXPECT_FALSE(h.link.running());
    EXPECT_EQ(h.link.state(), LinkState::PROBE);
    // It kept trying both bauds.
    size_t probes = 0;
    for (const auto& s : h.sent) probes += s.empty() ? 1 : 0;
    EXPECT_GT(probes, 50u);
}

// ── PPS pairing ─────────────────────────────────────────────────────────

TEST(PpsPairer, EachBlockNamesTheEdgeBeforeIt)
{
    PpsPairer p;
    int64_t e = 0;
    const int64_t edges[] = {1000000, 2000003, 2999998};
    for (int k = 0; k < 3; k++)
    {
        p.addEdge(edges[k]);
        ASSERT_TRUE(p.pair(100.0 + k, edges[k] + 20000, e));   // the block 20 ms later
        EXPECT_EQ(e, edges[k]);
    }
    EXPECT_EQ(p.pairs(), 3u);
}

TEST(PpsPairer, TwoBlocksNamingOneSecondPairItOnce)
{
    PpsPairer p;
    int64_t e = 0;
    p.addEdge(1000000);
    ASSERT_TRUE(p.pair(100.0, 1003000, e));    // xPPSOffset, 3 ms after
    EXPECT_FALSE(p.pair(100.0, 1021000, e));   // the whole-second PVTGeodetic
    EXPECT_EQ(p.pairs(), 1u);
}

TEST(PpsPairer, AMissingEdgeLeavesItsSecondUnpaired)
{
    PpsPairer p;
    int64_t e = 0;
    p.addEdge(1000000);
    ASSERT_TRUE(p.pair(100.0, 1020000, e));
    // No edge at 2 s: the block for 101 finds only stale or no edges.
    EXPECT_FALSE(p.pair(101.0, 2020000, e));
    p.addEdge(3000001);
    ASSERT_TRUE(p.pair(102.0, 3020000, e));
    EXPECT_EQ(e, 3000001);
}

TEST(PpsPairer, AnEdgeAfterTheBlockOrTooLongBeforeIsNotIt)
{
    PpsPairer p;
    int64_t e = 0;
    p.addEdge(1000000);
    EXPECT_FALSE(p.pair(100.0, 1600000, e));   // 600 ms later: outside the window
    p.addEdge(2000000);
    EXPECT_FALSE(p.pair(101.0, 1999000, e));   // the edge came after the block
    ASSERT_TRUE(p.pair(101.0, 2010000, e));
    EXPECT_EQ(e, 2000000);
}
