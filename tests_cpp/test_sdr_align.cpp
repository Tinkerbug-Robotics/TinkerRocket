// Host tests for the gnss-sdr-dev board IMU's mounting between the apps and
// the board: its 4001 sub-type 5 report as the flight computer reads it
// (TR_GNSSReceiverSdr_Serial/SdrRtcm.cpp decodeTrAlign), the 4002 sub-type 6
// command it sends (SdrAid.cpp), and the relay between them (flight_computer/
// main/sdr_align_policy.h): the command again until the board echoes it, its
// own board id, refusals, the give-up, and the status the apps get, turned
// into this IMU's frame. The golden bytes come from the reference codec
// (v10link_codec.py's enc_align/enc_cmd through align_codec.py, the
// alignment protocol's rev 2); the board's codec tests pin the same ones.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "GnssClockRecord.h"
#include "RocketComputerTypes.h"
#include "Rtcm3Msm.h"
#include "SdrAid.h"
#include "SdrRtcm.h"
#include "sdr_phase_policy.h"   // projects/flight_computer/main: the board's clock line
#include "sdr_align_policy.h"   // and the relay

namespace {

using Bytes = std::vector<uint8_t>;

Bytes hex(const std::string& s)
{
    Bytes b;
    for (size_t i = 0; i + 1 < s.size(); i += 3) b.push_back(static_cast<uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
    return b;
}

// ── Goldens (align_codec.py) ───────────────────────────────────────────────
// 4002 sub-type 6
const char* kC1 = "D3 00 0E FA 26 09 80 30 A3 EC EA C6 6D 01 D5 12 A0 CA 9F 10";   // save, id 42, on the line
const char* kC2 = "D3 00 0E FA 26 00 00 00 00 00 00 00 00 00 10 30 10 1C 89 04";   // restart, id 1, no line
const char* kC3 = "D3 00 0E FA 26 09 80 30 A3 EC EA C6 6D 3F D5 2F F0 F0 02 27";   // clear, id 255, seq 255
// 4001 sub-type 5
const char* kA1 =   // watching, no time, nothing yet
    "D3 00 23 FA 15 00 00 40 60 30 18 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 00 00 07 19 6A";
const char* kA2 =   // six faces, whole, U10 turned 90 deg about z on this IMU, every axis calibrated
    "D3 00 3B FA 15 C9 11 B3 6D C9 60 69 53 2B 40 E1 85 A8 20 00 00 00 05 A8 20 96 1F 5D 80 25 80 BF 5E AD 40 "
    "0B 20 0F E0 00 00 50 20 00 00 60 0F E0 00 40 10 20 00 20 00 0F E0 80 00 10 3F E0 2A AF F6";
const char* kA3 =   // one pose on a saved rotation (from 3), against the body; moved 1.234 deg; saturated residual and fit
    "D3 00 27 FA 15 05 01 6E 82 10 48 20 19 40 1F FF E7 FE 00 59 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
    "00 07 D0 20 1F 20 80 9F B4 90";

sdr_aid::Header lineHeader(uint8_t seq)
{
    sdr_aid::Header h = sdr_aid::stampAt(2432, 204012.3456789);
    h.seq = seq;
    h.src = sdr_aid::kSrcOwn;
    h.test = false;
    h.tq = 5;
    return h;
}

TEST(SdrAlignCodec, CommandsAreTheGoldenFrames)
{
    uint8_t out[sdr_aid::kCommandFrame];
    sdr_aid::Command c;
    c.h = lineHeader(7);
    c.op = sdr_aid::kOpSave;
    c.id = 42;
    ASSERT_EQ(sdr_aid::encode(c, out, sizeof(out)), 20u);
    EXPECT_EQ(Bytes(out, out + 20), hex(kC1));

    c = sdr_aid::Command{};
    c.h.src = sdr_aid::kSrcOwn;
    c.h.test = false;
    c.op = sdr_aid::kOpRestart;
    c.id = 1;
    ASSERT_EQ(sdr_aid::encode(c, out, sizeof(out)), 20u);
    EXPECT_EQ(Bytes(out, out + 20), hex(kC2));

    c.h = lineHeader(255);
    c.op = sdr_aid::kOpClear;
    c.id = 255;
    ASSERT_EQ(sdr_aid::encode(c, out, sizeof(out)), 20u);
    EXPECT_EQ(Bytes(out, out + 20), hex(kC3));

    EXPECT_EQ(sdr_aid::encode(c, out, 19), 0u);   // short of a frame
}

TEST(SdrAlignCodec, CommandsDecodeBack)
{
    const Bytes f = hex(kC1);
    EXPECT_EQ(sdr_aid::fcSubType(f.data(), f.size()), sdr_aid::kCommand);
    sdr_aid::Command c;
    ASSERT_TRUE(sdr_aid::decode(f.data(), f.size(), c));
    EXPECT_EQ(c.op, sdr_aid::kOpSave);
    EXPECT_EQ(c.id, 42);
    EXPECT_EQ(c.h.seq, 7);
    EXPECT_EQ(c.h.week, 2432);
    EXPECT_EQ(c.h.tow_ms, 204012346u);
    // not a rest, nor a command of another length
    sdr_aid::Rest r;
    EXPECT_FALSE(sdr_aid::decode(f.data(), f.size(), r));
    EXPECT_FALSE(sdr_aid::decode(f.data(), f.size() - 1, c));
}

TEST(SdrAlignCodec, TheBoardsReportDecodes)
{
    sdr_rtcm::TrAlign a;
    Bytes f = hex(kA1);
    EXPECT_EQ(sdr_rtcm::trSubType(f.data(), f.size()), sdr_rtcm::kTrAlign);
    ASSERT_TRUE(sdr_rtcm::decodeTrAlign(f.data(), f.size(), a));
    EXPECT_EQ(a.imu, 1);
    EXPECT_FALSE(a.time);
    EXPECT_EQ(a.have, sdr_rtcm::TrAlign::kHaveNone);
    EXPECT_TRUE(a.autosave);
    EXPECT_EQ(a.frame, sdr_rtcm::TrAlign::kFrameImu);
    EXPECT_EQ(a.n_pose, 0);
    EXPECT_EQ(a.last_rest, 6);   // no time to place it
    EXPECT_EQ(a.n_rest, 3);
    EXPECT_EQ(a.q[0], 0.0);      // none: sent as 0

    f = hex(kA2);
    ASSERT_TRUE(sdr_rtcm::decodeTrAlign(f.data(), f.size(), a));
    EXPECT_EQ(a.seq, 201);
    EXPECT_EQ(a.gen, 17);
    EXPECT_EQ(a.imu, 2);
    EXPECT_TRUE(a.time);
    EXPECT_EQ(a.have, sdr_rtcm::TrAlign::kHaveWhole);
    EXPECT_EQ(a.from, 1);
    EXPECT_EQ(a.saved, 2);
    EXPECT_EQ(a.n_pose, 6);
    EXPECT_EQ(a.cal_axes, 7);
    EXPECT_EQ(a.last_rest, 1);
    EXPECT_EQ(a.n_rest, 300 % 256);
    EXPECT_EQ(a.n_pair, 13);
    EXPECT_EQ(a.cmd_id, 42);
    EXPECT_EQ(a.cmd_op, sdr_aid::kOpRestart);
    EXPECT_EQ(a.cmd_res, 1);
    EXPECT_EQ(a.spread_deg, 90.0);
    EXPECT_NEAR(a.resid_deg, 0.14, 1e-12);
    EXPECT_NEAR(a.cal_rms, 0.012, 1e-12);
    const double s = std::sqrt(0.5);
    EXPECT_NEAR(a.q[0], s, 1e-4);   // q0 >= 0 on the wire
    EXPECT_NEAR(a.q[3], s, 1e-4);
    EXPECT_NEAR(a.scale[0], 1.012, 1e-9);
    EXPECT_NEAR(a.scale[1], 0.987, 1e-9);
    EXPECT_NEAR(a.bias[1], -0.271, 1e-9);
    EXPECT_NEAR(a.bias[2], 0.0089, 1e-9);
    EXPECT_EQ(a.pose[1].dir[0], -1.0);
    EXPECT_EQ(a.pose[4].dir[2], 1.0);
    EXPECT_EQ(a.pose[4].pairs, 4);
    EXPECT_EQ(a.pose[5].pairs, 255);   // that or more
    EXPECT_EQ(a.moved_deg, 0.0);       // from 1: moved is from 3's

    f = hex(kA3);
    ASSERT_TRUE(sdr_rtcm::decodeTrAlign(f.data(), f.size(), a));
    EXPECT_EQ(a.have, sdr_rtcm::TrAlign::kHaveTilt);
    EXPECT_EQ(a.frame, sdr_rtcm::TrAlign::kFrameBody);
    EXPECT_EQ(a.from, 3);
    EXPECT_EQ(a.cmd_res, 2);
    EXPECT_NEAR(a.resid_deg, 5.1, 1e-12);   // that or more
    EXPECT_NEAR(a.cal_rms, 0.255, 1e-12);
    EXPECT_EQ(a.cal_axes, 0);
    EXPECT_NEAR(a.moved_deg, 1.24, 1e-12);   // 1.234 deg to the nearest 0.02
    EXPECT_EQ(a.scale[0], 1.0);
    EXPECT_NEAR(a.pose[0].dir[0], -1.0, 1e-12);
    EXPECT_NEAR(a.pose[0].dir[2], -7.0 / 127.0, 1e-12);
}

TEST(SdrAlignCodec, AReportShortOfItsPosesIsNotRead)
{
    sdr_rtcm::TrAlign a;
    Bytes f = hex(kA2);
    // Cut the last pose off and re-frame it: the count says six, five are there
    Bytes m(f.begin() + 3, f.end() - 3);
    m.resize(m.size() - 4);
    Bytes g = {0xD3, 0, static_cast<uint8_t>(m.size())};
    g.insert(g.end(), m.begin(), m.end());
    const uint32_t crc = rtcm3::crc24q(g.data(), g.size());
    g.push_back(static_cast<uint8_t>(crc >> 16));
    g.push_back(static_cast<uint8_t>(crc >> 8));
    g.push_back(static_cast<uint8_t>(crc));
    EXPECT_FALSE(sdr_rtcm::decodeTrAlign(g.data(), g.size(), a));
    // Another sub-type is not one
    const Bytes c = hex(kC1);
    EXPECT_FALSE(sdr_rtcm::decodeTrAlign(c.data(), c.size(), a));
}

// ── The relay ─────────────────────────────────────────────────────────────

GnssClockData validLine()
{
    GnssClockData c{};
    c.flags = GCL_VALID | GCL_STARTED;
    c.gps0_s = 2432u * 604800u + 203000u;
    c.mcu0_us = 4123456;
    c.a_s = 0.0021;
    c.t_ref_s = 31.0;
    c.drift = 17.4e-6;
    c.latency_s = 0.0;
    return c;
}

const float kI[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

sdr_rtcm::TrAlign report(const char* golden)
{
    sdr_rtcm::TrAlign a;
    const Bytes f = hex(golden);
    EXPECT_TRUE(sdr_rtcm::decodeTrAlign(f.data(), f.size(), a));
    return a;
}

sdr_aid::Command sentCommand(sdr_align::Relay& r, int64_t now, const sdr_phase::Sender& line)
{
    uint8_t out[sdr_aid::kCommandFrame];
    sdr_aid::Command c;
    const size_t len = r.frame(now, line, out, sizeof(out));
    EXPECT_EQ(len, sdr_aid::kCommandFrame);
    EXPECT_TRUE(sdr_aid::decode(out, len, c));
    r.sent(now);
    return c;
}

TEST(SdrAlignRelay, ACommandGoesAgainUntilTheBoardEchoesIt)
{
    sdr_align::Relay r;
    sdr_phase::Sender line;
    const int64_t t = 50000000;
    line.noteLine(validLine(), t);
    r.noteReport(report(kA1), t);   // the board's echo is id 0: none yet
    r.command(SDR_ALIGN_OP_SAVE, 9, true, t);
    EXPECT_EQ(r.state(), SDRA_CMD_SENDING);

    sdr_aid::Command c = sentCommand(r, t, line);
    EXPECT_EQ(c.op, SDR_ALIGN_OP_SAVE);
    EXPECT_EQ(c.id, 1);           // its own board id, not the app's 9
    EXPECT_EQ(c.h.src, sdr_aid::kSrcOwn);
    EXPECT_FALSE(c.h.test);
    EXPECT_NE(c.h.week, 0);       // stamped off the line
    EXPECT_EQ(c.h.tq, sdr_aid::kLineTq);
    uint8_t out[sdr_aid::kCommandFrame];
    EXPECT_EQ(r.frame(t + sdr_align::Relay::kRepeatUs - 1, line, out, sizeof(out)), 0u);   // not again yet
    c = sentCommand(r, t + sdr_align::Relay::kRepeatUs, line);
    EXPECT_EQ(c.id, 1);
    EXPECT_EQ(c.h.seq, 1);

    // The board echoes another command's id: still going
    sdr_rtcm::TrAlign a = report(kA2);
    a.cmd_id = 7;
    a.cmd_op = SDR_ALIGN_OP_SAVE;
    r.noteReport(a, t + 300000);
    EXPECT_EQ(r.state(), SDRA_CMD_SENDING);
    // Its own: echoed, and nothing more goes
    a.cmd_id = 1;
    r.noteReport(a, t + 400000);
    EXPECT_EQ(r.state(), SDRA_CMD_ECHOED);
    EXPECT_EQ(r.frame(t + 600000, line, out, sizeof(out)), 0u);
    const SdrAlignStatusData s = r.status(t + 600000, kI, true, true, false);
    EXPECT_EQ(s.fc_cmd_state, SDRA_CMD_ECHOED);
    EXPECT_EQ(s.fc_cmd_id, 9);    // the app's id, for the app
    EXPECT_EQ(s.fc_cmd_op, SDR_ALIGN_OP_SAVE);
    EXPECT_EQ(s.cmd_id, 1);       // the board's
}

TEST(SdrAlignRelay, ANewIdNeverTheOneTheBoardEchoesNow)
{
    sdr_align::Relay r;
    sdr_phase::Sender line;
    const int64_t t = 50000000;
    sdr_rtcm::TrAlign a = report(kA1);
    a.cmd_id = 1;   // a board that acted on id 1 before this flight computer started
    r.noteReport(a, t);
    r.command(SDR_ALIGN_OP_RESTART, 1, true, t);
    EXPECT_EQ(r.boardId(), 2);
    // ids go round 255 -> 1, skipping 0 and the echoed one
    a.cmd_id = 2;
    r.noteReport(a, t);
    for (int k = 0; k < 253; k++) r.command(SDR_ALIGN_OP_REPORT, 3, true, t);
    EXPECT_EQ(r.boardId(), 255);
    r.command(SDR_ALIGN_OP_REPORT, 3, true, t);
    EXPECT_EQ(r.boardId(), 1);
    r.command(SDR_ALIGN_OP_REPORT, 3, true, t);
    EXPECT_EQ(r.boardId(), 3);    // 2 is what the board echoes
}

TEST(SdrAlignRelay, WithNoLineACommandStillGoesUnstamped)
{
    sdr_align::Relay r;
    sdr_phase::Sender line;   // no line noted
    r.command(SDR_ALIGN_OP_CLEAR, 4, true, 1000);
    const sdr_aid::Command c = sentCommand(r, 1000, line);
    EXPECT_EQ(c.op, SDR_ALIGN_OP_CLEAR);
    EXPECT_EQ(c.h.week, 0);
    EXPECT_EQ(c.h.tow_ms, 0u);
    EXPECT_EQ(c.h.tq, 0);
}

TEST(SdrAlignRelay, NoEchoIsGivenUpOn)
{
    sdr_align::Relay r;
    sdr_phase::Sender line;
    r.command(SDR_ALIGN_OP_SAVE, 4, true, 0);
    int sent = 0;
    uint8_t out[sdr_aid::kCommandFrame];
    for (int64_t t = 0; t <= sdr_align::Relay::kGiveUpUs + 100000; t += 10000)
    {
        if (r.frame(t, line, out, sizeof(out)) != 0)
        {
            r.sent(t);
            sent++;
        }
    }
    EXPECT_EQ(sent, 16);   // at 0, 0.2, ... 3.0 s
    EXPECT_EQ(r.state(), SDRA_CMD_NO_ANSWER);
    EXPECT_TRUE(r.publishDue(sdr_align::Relay::kGiveUpUs + 100000));
}

TEST(SdrAlignRelay, ARefusedCommandSendsNothing)
{
    sdr_align::Relay r;
    sdr_phase::Sender line;
    uint8_t out[sdr_aid::kCommandFrame];
    r.command(SDR_ALIGN_OP_SAVE, 5, false, 0);   // in flight, after one, or no board
    EXPECT_EQ(r.state(), SDRA_CMD_REFUSED);
    EXPECT_EQ(r.frame(0, line, out, sizeof(out)), 0u);
    r.command(0, 6, true, 0);                     // no such op
    EXPECT_EQ(r.state(), SDRA_CMD_REFUSED);
    r.command(5, 6, true, 0);
    EXPECT_EQ(r.state(), SDRA_CMD_REFUSED);
    const SdrAlignStatusData s = r.status(0, kI, false, false, false);
    EXPECT_EQ(s.fc_cmd_id, 6);
    EXPECT_EQ(s.flags & SDRA_BOARD, 0);
}

TEST(SdrAlignRelay, TheStatusInThisImusFrame)
{
    sdr_align::Relay r;
    const int64_t t = 10000000;
    r.noteRestSent(t - 2500000);
    r.noteReport(report(kA2), t - 400000);
    const SdrAlignStatusData s = r.status(t, kI, true, true, true);
    EXPECT_EQ(s.ver, SDR_ALIGN_STATUS_VER);
    EXPECT_EQ(s.flags, SDRA_BOARD | SDRA_REPORT | SDRA_TIME | SDRA_LINE | SDRA_AUTOSAVE | SDRA_STILL);
    EXPECT_EQ(s.imu, 2);
    EXPECT_EQ(s.have, SDRA_HAVE_WHOLE);
    EXPECT_EQ(s.from, SDRA_FROM_RESTS);
    EXPECT_EQ(s.saved, SDRA_SAVED_DIFFERS);
    EXPECT_EQ(s.n_pose, 6);
    EXPECT_EQ(s.cal_axes, 7);
    EXPECT_EQ(s.last_rest, SDRA_REST_PAIRED);
    EXPECT_EQ(s.rests_sent, 1);
    EXPECT_EQ(s.rest_age_ds, 25);
    EXPECT_EQ(s.board_age_ds, 4);
    EXPECT_FLOAT_EQ(s.spread_deg, 90.0f);
    EXPECT_NEAR(s.q[0], std::sqrt(0.5), 1e-4);   // passed as it is: the board solved against this IMU
    EXPECT_NEAR(s.q[3], std::sqrt(0.5), 1e-4);
    EXPECT_NEAR(s.scale[0], 1.012f, 1e-6);
    EXPECT_NEAR(s.bias_mps2[0], 0.153f, 1e-6);
    EXPECT_EQ(s.pose_dir[0][0], 127);
    EXPECT_EQ(s.pose_dir[1][0], -127);
    EXPECT_EQ(s.pose_dir[5][2], -127);
    EXPECT_EQ(s.pose_pairs[5], 255);
    EXPECT_EQ(s.pose_pairs[6], 0);   // past n_pose: zero

    // Three seconds on with no report: no longer held, the rest kept as it was
    const SdrAlignStatusData old = r.status(t - 400000 + sdr_align::Relay::kReportFreshUs + 1, kI, true, true, false);
    EXPECT_EQ(old.flags & SDRA_REPORT, 0);
    EXPECT_EQ(old.have, SDRA_HAVE_WHOLE);
}

TEST(SdrAlignRelay, ABodyFrameReportIsTurnedBackThroughTheMounting)
{
    // This IMU mounted with its +Z toward the nose: body x = imu z, body y =
    // imu y, body z = -imu x (v_body = b2r v_imu).
    const float b2r[9] = {0, 0, 1, 0, 1, 0, -1, 0, 0};
    sdr_rtcm::TrAlign a = report(kA3);   // body frame, one pose: the nose down (body -x up... f along -x)
    a.q[0] = 1.0;                        // U10 = body
    a.q[1] = a.q[2] = a.q[3] = 0.0;
    a.pose[0].dir[0] = -1.0;
    a.pose[0].dir[1] = 0.0;
    a.pose[0].dir[2] = 0.0;
    sdr_align::Relay r;
    r.noteReport(a, 0);
    const SdrAlignStatusData s = r.status(0, b2r, true, true, false);
    EXPECT_TRUE(s.flags & SDRA_BODY);
    EXPECT_EQ(s.from, SDRA_FROM_SAVED_RESTS);
    EXPECT_NEAR(s.moved_deg, 1.24f, 1e-6);   // the rail pose off the saved rotation, relayed
    // body -x is imu -z
    EXPECT_EQ(s.pose_dir[0][0], 0);
    EXPECT_EQ(s.pose_dir[0][1], 0);
    EXPECT_EQ(s.pose_dir[0][2], -127);
    // U10 -> imu = b2r^T: U10's x is imu's -z... check by turning U10's x
    const double q[4] = {s.q[0], s.q[1], s.q[2], s.q[3]};
    auto turn = [&](const double v[3], double o[3]) {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        const double R[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                             2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                             2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
        for (int i = 0; i < 3; i++) o[i] = R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2];
    };
    const double ux[3] = {1, 0, 0};
    double o[3];
    turn(ux, o);
    EXPECT_NEAR(o[0], 0.0, 1e-6);
    EXPECT_NEAR(o[2], 1.0, 1e-6);   // body x (U10's x) is imu z
    EXPECT_GE(s.q[0], 0.0f);
}

TEST(SdrAlignRelay, PublishedOnAChangeAndOnceASecond)
{
    sdr_align::Relay r;
    EXPECT_TRUE(r.publishDue(0));   // the first
    r.published(0);
    EXPECT_FALSE(r.publishDue(500000));
    EXPECT_TRUE(r.publishDue(sdr_align::Relay::kPublishUs));
    r.noteReport(report(kA1), 600000);   // the first report: a change
    EXPECT_TRUE(r.publishDue(600000));
    r.published(600000);
    sdr_rtcm::TrAlign a = report(kA1);
    a.seq = 9;   // its seq alone: nothing new
    r.noteReport(a, 700000);
    EXPECT_FALSE(r.publishDue(700000));
    a.n_rest = 4;   // a rest received
    r.noteReport(a, 800000);
    EXPECT_TRUE(r.publishDue(800000));
    r.published(800000);
    r.command(SDR_ALIGN_OP_REPORT, 1, true, 900000);
    EXPECT_TRUE(r.publishDue(900000));
}

TEST(SdrAlignRelay, NoReportYetIsAnEmptyStatus)
{
    sdr_align::Relay r;
    const SdrAlignStatusData s = r.status(0, kI, true, false, false);
    EXPECT_EQ(s.flags, SDRA_BOARD);
    EXPECT_EQ(s.have, SDRA_HAVE_NONE);
    EXPECT_EQ(s.q[0], 1.0f);
    EXPECT_EQ(s.scale[2], 1.0f);
    EXPECT_EQ(s.rest_age_ds, 65535);
    EXPECT_EQ(s.board_age_ds, 65535);
}

}  // namespace
