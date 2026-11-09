// The RunCam readiness probe without waiting on the camera (CENJARS 2026-11-01,
// GTV Blue: a silent camera stopped the FC loop 60 ms in every 250 on the pad).
// Pins the CRC, the reply parser fed in arbitrary pieces, the per-pass decision,
// and the probe cadence a silent camera gets over a whole boot window.

#include <gtest/gtest.h>

#include <vector>

#include "runcam_probe_policy.h"

using RunCamProbePolicy::ReplyParser;
using RunCamProbePolicy::Step;
using RunCamProbePolicy::crc8;
using RunCamProbePolicy::step;
using State = ReplyParser::State;

namespace {

constexpr uint32_t kWindowMs   = 60;   // config::RUNCAM_PROBE_READ_MS
constexpr uint32_t kIntervalMs = 250;  // config::RUNCAM_PROBE_INTERVAL_MS

// The bench unit's GET_DEVICE_INFO reply (protocol v1, features 0x0077), CRC'd.
std::vector<uint8_t> benchReply()
{
    std::vector<uint8_t> r = {0xCC, 0x01, 0x77, 0x00, 0x00};
    r[4] = crc8(r.data(), 4);
    return r;
}

}  // namespace

TEST(RunCamProbePolicy, Crc8MatchesTheProtocolFrames)
{
    const uint8_t pwr[] = {0xCC, 0x01, 0x01};   // RUNCAM_PWR_CMD = CC 01 01 E7
    EXPECT_EQ(crc8(pwr, sizeof(pwr)), 0xE7);
    const uint8_t info[] = {0xCC, 0x00};        // RUNCAM_DEVICE_INFO_CMD = CC 00 60
    EXPECT_EQ(crc8(info, sizeof(info)), 0x60);
}

TEST(RunCamProbePolicy, WholeReplyInOneReadIsValid)
{
    ReplyParser p;
    const auto r = benchReply();
    EXPECT_EQ(p.feed(r.data(), r.size()), State::Valid);
    EXPECT_EQ(p.protocolVersion(), 1);
    EXPECT_EQ(p.features(), 0x0077);
}

TEST(RunCamProbePolicy, ReplyByteByByteStaysPendingUntilTheLastByte)
{
    ReplyParser p;
    const auto r = benchReply();
    for (size_t i = 0; i + 1 < r.size(); ++i)
        EXPECT_EQ(p.feed(&r[i], 1), State::Pending) << "byte " << i;
    EXPECT_EQ(p.feed(&r.back(), 1), State::Valid);
    EXPECT_EQ(p.features(), 0x0077);
}

TEST(RunCamProbePolicy, EmptyReadsChangeNothing)
{
    ReplyParser p;
    EXPECT_EQ(p.feed(nullptr, 0), State::Pending);
    const auto r = benchReply();
    p.feed(r.data(), 2);
    EXPECT_EQ(p.feed(nullptr, 0), State::Pending);
    EXPECT_EQ(p.feed(r.data() + 2, 3), State::Valid);
}

TEST(RunCamProbePolicy, BootChatterBeforeTheHeaderIsSkipped)
{
    // The blocking read took the first 5 bytes as the reply and failed on any
    // chatter; the parser looks for the header instead.
    ReplyParser p;
    std::vector<uint8_t> bytes = {0x00, 0x7F, 0x13};
    const auto r = benchReply();
    bytes.insert(bytes.end(), r.begin(), r.end());
    EXPECT_EQ(p.feed(bytes.data(), bytes.size()), State::Valid);
    EXPECT_EQ(p.features(), 0x0077);
}

TEST(RunCamProbePolicy, BadCrcIsInvalidAndStaysInvalid)
{
    ReplyParser p;
    auto r = benchReply();
    r[4] ^= 0x01;
    EXPECT_EQ(p.feed(r.data(), r.size()), State::Invalid);
    // A good reply arriving after the verdict does not overturn it: the probe
    // it answered is over, and the next probe resets the parser.
    const auto good = benchReply();
    EXPECT_EQ(p.feed(good.data(), good.size()), State::Invalid);
    p.reset();
    EXPECT_EQ(p.state(), State::Pending);
    EXPECT_EQ(p.feed(good.data(), good.size()), State::Valid);
}

TEST(RunCamProbePolicy, StepTable)
{
    const uint32_t sent = 1000, deadline = 5000;
    // Nothing out: send, whatever the clock says.
    EXPECT_EQ(step(false, State::Pending, sent, 900, deadline, kWindowMs), Step::Send);
    EXPECT_EQ(step(false, State::Pending, sent, 9000, deadline, kWindowMs), Step::Send);
    // Out, window open, nothing yet: keep reading.
    EXPECT_EQ(step(true, State::Pending, sent, sent, deadline, kWindowMs), Step::Wait);
    EXPECT_EQ(step(true, State::Pending, sent, sent + kWindowMs - 1, deadline, kWindowMs), Step::Wait);
    // Window closed empty before the deadline: probe again later.
    EXPECT_EQ(step(true, State::Pending, sent, sent + kWindowMs, deadline, kWindowMs), Step::NextProbe);
    // A bad reply ends the probe at once, like the blocking read did.
    EXPECT_EQ(step(true, State::Invalid, sent, sent + 2, deadline, kWindowMs), Step::NextProbe);
    // A good reply wins, inside the window or found on the pass that closes it.
    EXPECT_EQ(step(true, State::Valid, sent, sent + 3, deadline, kWindowMs), Step::Alive);
    EXPECT_EQ(step(true, State::Valid, sent, sent + kWindowMs + 5, deadline, kWindowMs), Step::Alive);
}

TEST(RunCamProbePolicy, DeadlineIsJudgedOnlyWhenAWindowCloses)
{
    const uint32_t deadline = 5000;
    // A probe sent just before the deadline keeps its whole window.
    EXPECT_EQ(step(true, State::Pending, 4990, 5030, deadline, kWindowMs), Step::Wait);
    EXPECT_EQ(step(true, State::Valid, 4990, 5030, deadline, kWindowMs), Step::Alive);
    // Its window closing at or past the deadline ends the boot window.
    EXPECT_EQ(step(true, State::Pending, 4990, 5050, deadline, kWindowMs), Step::Silent);
    EXPECT_EQ(step(true, State::Invalid, 4990, 5000, deadline, kWindowMs), Step::Silent);
}

TEST(RunCamProbePolicy, MillisWraparound)
{
    const uint32_t sent = 0xFFFFFFF0u;  // 16 ms before the wrap
    const uint32_t deadline = sent + 6000u;  // wraps
    EXPECT_EQ(step(true, State::Pending, sent, sent + 20u, deadline, kWindowMs), Step::Wait);
    EXPECT_EQ(step(true, State::Pending, sent, sent + kWindowMs, deadline, kWindowMs), Step::NextProbe);
    EXPECT_EQ(step(true, State::Pending, sent, deadline + 10u, deadline, kWindowMs), Step::Silent);
}

// The serviceCameraStart Probing arm, as a loop model: one pass every 2 ms, a
// pass due again at once on Send/Wait, at sent + interval on NextProbe.
namespace {

struct ProbeRun {
    std::vector<uint32_t> sends;
    uint32_t end_ms = 0;
    Step end = Step::Wait;
};

// reply_at_ms / reply_after_send: the camera answers the first probe sent at or
// after reply_at_ms, in two pieces, reply_after_send ms and +1 ms after the send.
ProbeRun simulate(uint32_t start_ms, uint32_t deadline_ms, int64_t reply_at_ms = -1,
             uint32_t reply_after_send = 3)
{
    ProbeRun run;
    ReplyParser parser;
    bool out = false;
    uint32_t sent = 0, due = start_ms;
    bool answering = false;
    const auto reply = benchReply();
    for (uint32_t now = start_ms; now < start_ms + 20000; now += 2)
    {
        if ((int32_t)(now - due) < 0) continue;
        if (out && answering)
        {
            if (now - sent >= reply_after_send && parser.state() == State::Pending)
                parser.feed(reply.data(), 2);
            if (now - sent >= reply_after_send + 1)
                parser.feed(reply.data() + 2, 3);
        }
        const Step s = step(out, out ? parser.state() : State::Pending, sent, now,
                            deadline_ms, kWindowMs);
        switch (s)
        {
            case Step::Send:
                parser.reset();
                out = true;
                sent = now;
                due = now;
                run.sends.push_back(now);
                answering = reply_at_ms >= 0 && (int64_t)now >= reply_at_ms;
                break;
            case Step::Wait:
                due = now;
                break;
            case Step::NextProbe:
                out = false;
                due = sent + kIntervalMs;
                break;
            case Step::Alive:
            case Step::Silent:
                run.end_ms = now;
                run.end = s;
                return run;
        }
    }
    return run;
}

}  // namespace

TEST(RunCamProbePolicy, SilentCameraKeepsThe250msCadenceAndEndsAtTheDeadline)
{
    // Boot grace over at 2000, deadline 8000 (RUNCAM_BOOT_MIN/MAX_MS): probes at
    // 2000, 2250, ... 8000, as the blocking probe sent them, and the window of
    // the 8000 probe ends the boot window 60 ms later.
    const ProbeRun run = simulate(2000, 8000);
    ASSERT_EQ(run.sends.size(), 25u);
    for (size_t i = 0; i < run.sends.size(); ++i)
        EXPECT_EQ(run.sends[i], 2000u + 250u * i) << "probe " << i;
    EXPECT_EQ(run.end, Step::Silent);
    EXPECT_EQ(run.end_ms, 8060u);
}

TEST(RunCamProbePolicy, ReplyEndsProbingOnThePassItCompletes)
{
    // The camera comes up between probes and answers the 2750 probe in two
    // pieces, 3 and 4 ms after it went out.
    const ProbeRun run = simulate(2000, 8000, 2700, 3);
    ASSERT_EQ(run.sends.size(), 4u);
    EXPECT_EQ(run.sends.back(), 2750u);
    EXPECT_EQ(run.end, Step::Alive);
    EXPECT_EQ(run.end_ms, 2754u);
}
