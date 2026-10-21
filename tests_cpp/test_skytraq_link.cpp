// Finding and setting up the PX1105R (TR_GNSSReceiverPX1105R_Serial/SkyTraqLink.h)
// against a simulated receiver: at its factory 115200 baud talking NMEA, or
// already set up at 921600 (the FC rebooted alone), refusing a command, or
// resetting while the link runs. The simulated receiver answers each command
// the way the rig's tools saw the PX1105R answer (ACK per command, the
// readbacks' replies), 300 ms late, and streams 0xE5 at its position rate once
// it is an RTK base with the raw output on.
#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <vector>

#include "SkyTraqBinary.h"
#include "SkyTraqLink.h"

using namespace skytraq;

namespace
{

struct FakeRx
{
    uint32_t baud = 115200;
    bool     base = false;      // RTK base (raw output possible)
    uint8_t  rate_code = 0;     // 0x1E's
    bool     ext_raw = false;
    uint8_t  subframes = 0;
    uint8_t  pos_rate = 1;
    uint8_t  nav_mode = 0;
    uint8_t  power = 1;         // ships in power save
    bool     nack_nav = false;  // refuse 0x64/0x17
    bool     dead = false;
    uint32_t ack_delay_ms = 300;
    struct Out { uint32_t at; std::vector<uint8_t> body; };
    std::deque<Out> out;
    uint32_t pending_baud = 0, pending_at = 0;
    uint32_t next_nmea = 0, next_epoch = 0;

    void reply(uint32_t now, std::initializer_list<uint8_t> b) { out.push_back({now + ack_delay_ms, std::vector<uint8_t>(b)}); }
    void ack(uint32_t now, uint8_t id, int sub = -1)
    {
        if (sub >= 0) reply(now, {MSG_ACK, id, static_cast<uint8_t>(sub)});
        else reply(now, {MSG_ACK, id});
    }

    void command(const uint8_t* f, size_t n, uint32_t now)
    {
        if (dead || n < 7 || f[0] != 0xA0 || f[1] != 0xA1) return;
        const uint8_t* b = f + 4;
        const size_t len = (static_cast<size_t>(f[2]) << 8) | f[3];
        switch (b[0])
        {
        case 0x02:
            ack(now, 0x02);
            reply(now, {MSG_SW_VERSION, 1, 0, 3, 6, 0, 0, 1, 7, 33, 0, 23, 8, 31});
            break;
        case 0x05:
            ack(now, 0x05);
            pending_baud = b[2] == 8 ? 921600 : (b[2] == 5 ? 115200 : 0);
            pending_at = now + 100;
            break;
        case 0x6A:
            if (len >= 2 && b[1] == 0x06) { base = b[2] == 1; ack(now, 0x6A, 0x06); }
            break;
        case 0x1E:
            rate_code = b[1]; ext_raw = b[7] == 1; subframes = b[6]; ack(now, 0x1E);
            break;
        case 0x0E: pos_rate = b[1]; ack(now, 0x0E); break;
        case 0x64:
            if (b[1] == 0x17)
            {
                if (nack_nav) reply(now, {MSG_NACK, 0x64, 0x17});
                else { nav_mode = b[2]; ack(now, 0x64, 0x17); }
            }
            else if (b[1] == 0x18) { ack(now, 0x64, 0x18); reply(now, {0x64, 0x8B, nav_mode}); }
            break;
        case 0x0C: power = b[1]; ack(now, 0x0C); break;
        case 0x2B: ack(now, 0x2B); break;
        case 0x10: ack(now, 0x10); reply(now, {MSG_POS_RATE, pos_rate}); break;
        case 0x1F: ack(now, 0x1F); reply(now, {MSG_BIN_MEAS_CFG, rate_code, 0, 0, 1, 1, subframes, static_cast<uint8_t>(ext_raw)}); break;
        case 0x15: ack(now, 0x15); reply(now, {MSG_POWER_MODE, power}); break;
        default: reply(now, {MSG_NACK, b[0]}); break;
        }
    }

    // a reset: its flash defaults (SRAM settings gone)
    void reset()
    {
        *this = FakeRx{};
    }
};

struct Bench
{
    FakeRx rx;
    SkyTraqLink link;
    uint32_t host_baud = 0;
    uint32_t now = 1000;

    void step()
    {
        for (int i = 0; i < 8; ++i)
        {
            const LinkAction a = link.poll(now);
            if (a.kind == LinkAction::NONE) break;
            if (a.kind == LinkAction::SET_BAUD) host_baud = a.baud;
            else if (a.kind == LinkAction::SEND && host_baud == rx.baud) rx.command(a.bytes, a.len, now);
        }
        if (rx.pending_baud && now >= rx.pending_at) { rx.baud = rx.pending_baud; rx.pending_baud = 0; }
        const bool hears = host_baud == rx.baud && !rx.dead;
        while (!rx.out.empty() && rx.out.front().at <= now)
        {
            if (hears) link.onFrame(rx.out.front().body.data(), rx.out.front().body.size(), now);
            rx.out.pop_front();
        }
        if (rx.dead) return;
        if (!rx.base && now >= rx.next_nmea)
        {
            rx.next_nmea = now + 1000;
            if (hears) { link.onNmea(now); link.onNmea(now); }
        }
        const uint32_t period = 1000 / (rx.pos_rate ? rx.pos_rate : 1);
        if (rx.base && now >= rx.next_epoch)
        {
            rx.next_epoch = now + period;
            if (hears)
            {
                uint8_t df[81] = {MSG_RCV_STATE};
                link.onFrame(df, sizeof df, now);
                if (rx.ext_raw)
                {
                    uint8_t e5[14] = {MSG_EXT_RAW, 1, 0, 0x09, 0x85, 0, 0, 0, 0,
                                      static_cast<uint8_t>(period >> 8), static_cast<uint8_t>(period & 0xFF), 0, 0, 0};
                    link.onFrame(e5, sizeof e5, now);
                }
            }
        }
    }
    bool runUntilRunning(uint32_t limit_ms)
    {
        const uint32_t end = now + limit_ms;
        while (now < end)
        {
            step();
            if (link.running()) return true;
            now += 10;
        }
        return false;
    }
    void runFor(uint32_t ms)
    {
        const uint32_t end = now + ms;
        while (now < end) { step(); now += 10; }
    }
};

}  // namespace

TEST(SkyTraqLink, SetsUpAFactoryReceiverTalkingNmea)
{
    Bench b;
    b.link.start(LinkConfig{}, b.now);
    ASSERT_TRUE(b.runUntilRunning(30000)) << "state " << int(b.link.state()) << " " << b.link.lastFailure();
    EXPECT_EQ(b.link.baud(), 921600u);
    EXPECT_EQ(b.rx.baud, 921600u);
    EXPECT_EQ(b.link.foundBaud(), 115200u);
    EXPECT_TRUE(b.link.foundNmea());
    EXPECT_TRUE(b.rx.base);
    EXPECT_EQ(b.rx.rate_code, 5);       // 20 Hz
    EXPECT_EQ(b.rx.pos_rate, 20);
    EXPECT_EQ(b.rx.subframes, 0x0D);    // GPS, Galileo, BeiDou
    EXPECT_TRUE(b.rx.ext_raw);
    EXPECT_EQ(b.rx.nav_mode, 5);        // airborne
    EXPECT_EQ(b.rx.power, 0);           // normal
    EXPECT_EQ(b.link.refused(), 0u);
    EXPECT_EQ(b.link.checksFailed(), 0u);
    EXPECT_FALSE(b.link.degraded());
    EXPECT_EQ(b.link.setups(), 1u);
    EXPECT_EQ(b.link.rawPeriodMs(), 50);
    EXPECT_STREQ(b.link.version(), "3.6.0 1.7.33 230831");
}

TEST(SkyTraqLink, FindsAReceiverAlreadySetUpAtTheTargetBaud)
{
    Bench b;
    b.rx.baud = 921600;
    b.rx.base = true;
    b.rx.ext_raw = true;
    b.rx.rate_code = 5;
    b.rx.pos_rate = 20;
    b.link.start(LinkConfig{}, b.now);
    const uint32_t t0 = b.now;
    ASSERT_TRUE(b.runUntilRunning(30000));
    EXPECT_EQ(b.link.foundBaud(), 921600u);
    EXPECT_FALSE(b.link.foundNmea());
    EXPECT_LT(b.now - t0, 8000u);   // no baud search: the setup and its readbacks only
}

TEST(SkyTraqLink, ARefusedCommandRunsDegraded)
{
    Bench b;
    b.rx.nack_nav = true;
    b.link.start(LinkConfig{}, b.now);
    ASSERT_TRUE(b.runUntilRunning(60000));
    EXPECT_TRUE(b.link.degraded());
    EXPECT_GE(b.link.refused(), 1u);
    EXPECT_STREQ(b.link.lastFailure(), "navigation mode");
    EXPECT_TRUE(b.rx.ext_raw);   // the rest of the setup still went in
}

TEST(SkyTraqLink, AReceiverResetWhileRunningIsSetUpAgain)
{
    Bench b;
    b.link.start(LinkConfig{}, b.now);
    ASSERT_TRUE(b.runUntilRunning(30000));
    b.runFor(2000);
    b.rx.reset();   // back to 115200, NMEA, power save
    b.rx.next_nmea = b.now;
    b.runFor(4000);
    EXPECT_FALSE(b.link.running());
    EXPECT_EQ(b.link.relinks(), 1u);
    ASSERT_TRUE(b.runUntilRunning(30000));
    EXPECT_EQ(b.link.setups(), 2u);
    EXPECT_EQ(b.rx.baud, 921600u);
    EXPECT_EQ(b.rx.nav_mode, 5);
}

TEST(SkyTraqLink, AReceiverThatLosesItsRawOutputIsSetUpAgain)
{
    Bench b;
    b.link.start(LinkConfig{}, b.now);
    ASSERT_TRUE(b.runUntilRunning(30000));
    b.rx.ext_raw = false;   // 0xDF still comes, 0xE5 not
    b.runFor(5000);
    EXPECT_EQ(b.link.reconfigs(), 1u);
    ASSERT_TRUE(b.runUntilRunning(30000));
    EXPECT_TRUE(b.rx.ext_raw);
}

TEST(SkyTraqLink, ADeadReceiverKeepsTheLinkSearching)
{
    Bench b;
    b.rx.dead = true;
    b.link.start(LinkConfig{}, b.now);
    EXPECT_FALSE(b.runUntilRunning(20000));
    EXPECT_EQ(b.link.state(), LinkState::PROBE);
}
