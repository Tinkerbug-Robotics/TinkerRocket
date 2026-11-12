#pragma once

#include <cmath>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <RocketComputerTypes.h>   // SdrAlignStatusData, SDRA_*, SDR_ALIGN_OP_*
#include <SdrAid.h>                // TR_GNSSReceiverSdr_Serial: 4002 sub-type 6, the command
#include <SdrRtcm.h>               // and 4001 sub-type 5, the board's report
#include "sdr_phase_policy.h"      // the board's clock line, held by the phase sender
#include "sdr_aid_policy.h"        // quatMul, quatOfDcm: the mounting's quaternion

// The gnss-sdr-dev board's IMU on its flight board, between the apps and the
// board: the board finds how its IMU (U10) is mounted, and calibrates its
// accelerometer, from the rests this flight computer sends it (sub-type 5,
// sdr_aid_policy.h), keeps the result in its own NVS, and says what it holds
// in its 4001 sub-type 5 (once a second, and within 100 ms of a change). This
// relays that report to the out computer (SDR_ALIGN_STATUS_MSG, the apps'
// 0xD2) and sends the board the apps' commands (BLE 83: save, clear,
// restart, report) as 4002 sub-type 6.
//
// The report goes on, turned into this flight computer's IMU frame (the
// six-face calibration's board frame: after the chip's Z rotation, before
// the mounting), so the apps match the board's poses to the same faces they
// measure this IMU on. A board that solved against the body (the rests' FRD
// axes, the mounting taken out) has its mounting and its poses turned back
// through this side's mounting as it is now (SDRA_BODY says so: a board
// paired with a V10 that sent the body's rests); one that solved against
// this IMU's own axes (frame 1, what this image sends) is passed as it is.
//
// A command: the apps' op and id (the id only for them to match the
// answer). The board acts once per id and echoes the last it acted on, so
// this side gives each command its own board id, never the one the board
// echoes now (a board that acted on id 7 before would only acknowledge
// another 7). It goes at once and again every kRepeatUs until the board's
// report echoes it (SDRA_CMD_ECHOED: the report's cmd_res says what the
// board did), for kGiveUpUs at most (SDRA_CMD_NO_ANSWER). Stamped off the
// board's clock line when it serves; else week 0, tow 0, tq 0, which the
// board acts on all the same. Refused here (SDRA_CMD_REFUSED, nothing sent)
// when the caller says no: in flight, after a flight, or no board on J1. A
// new command replaces one still going.
//
// The status: once a second while the caller publishes it (on the ground,
// the board on J1), and at once after a change -- a report whose state
// moved, a command's start, echo, refusal or give-up.
//
// Pure: the caller (main.cpp sdrAlignStep, the pending-command handler and
// the board-frame drain) hands it the board's reports, the apps' commands
// and this side's rests, writes the frame it gives (one 4002 frame on the
// line at a time, as the other senders), and sends the status it builds.
// Host-tested in tests_cpp/test_sdr_align.cpp.

namespace sdr_align
{

// 0.1 s steps since an instant, 65535 for none or that long.
inline uint16_t ageDs(bool have, int64_t since_us, int64_t now_us)
{
    if (!have)
        return 65535;
    const int64_t ds = (now_us - since_us) / 100000;
    return (uint16_t)(ds < 0 ? 0 : (ds > 65534 ? 65534 : ds));
}

class Relay
{
public:
    static constexpr int64_t kRepeatUs = 200000;        // a command goes again this often until echoed
    static constexpr int64_t kGiveUpUs = 3000000;       // and for this long at most
    static constexpr int64_t kReportFreshUs = 3000000;  // a report older than this is no longer held
    static constexpr int64_t kPublishUs = 1000000;      // the status at least this often

    // The board's report (a decoded 4001 sub-type 5), arrived at now_us.
    void noteReport(const sdr_rtcm::TrAlign& a, int64_t now_us)
    {
        if (!have_report_ || moved(report_, a))
            changed_ = true;
        report_ = a;
        have_report_ = true;
        report_us_ = now_us;
        if (state_ == SDRA_CMD_SENDING && a.cmd_id == board_id_ && a.cmd_op == op_)
        {
            state_ = SDRA_CMD_ECHOED;
            changed_ = true;
        }
    }

    // The apps' command at now_us: op SDR_ALIGN_OP_*, their id for it.
    // allowed: false in flight, after a flight, or with no board on J1.
    void command(uint8_t op, uint8_t app_id, bool allowed, int64_t now_us)
    {
        op_ = op;
        app_id_ = app_id;
        changed_ = true;
        if (!allowed || op < SDR_ALIGN_OP_SAVE || op > SDR_ALIGN_OP_REPORT)
        {
            state_ = SDRA_CMD_REFUSED;
            return;
        }
        // Its own board id: the next after the last one sent, never 0 nor the
        // one the board echoes now.
        do
            last_board_id_ = (uint8_t)(last_board_id_ % 255u + 1u);
        while (have_report_ && last_board_id_ == report_.cmd_id);
        board_id_ = last_board_id_;
        state_ = SDRA_CMD_SENDING;
        start_us_ = now_us;
        due_us_ = now_us;
    }

    // The command frame to send now into out (cap at least
    // sdr_aid::kCommandFrame): its length, 0 for none.
    size_t frame(int64_t now_us, const sdr_phase::Sender& line, uint8_t* out, size_t cap)
    {
        if (state_ != SDRA_CMD_SENDING)
            return 0;
        if (now_us - start_us_ > kGiveUpUs)
        {
            state_ = SDRA_CMD_NO_ANSWER;
            changed_ = true;
            return 0;
        }
        if (now_us < due_us_)
            return 0;
        sdr_aid::Command c;
        double gps_s = 0.0;
        if (line.gpsAt(now_us, now_us, gps_s))
        {
            c.h = sdr_aid::gpsStamp(gps_s);
            c.h.tq = line.lineTq(now_us);
        }
        else
        {
            c.h = sdr_aid::Header{};   // week 0, tow 0, tq 0: acted on all the same
        }
        c.h.seq = seq_;
        c.h.src = sdr_aid::kSrcOwn;
        c.h.test = false;
        c.op = op_;
        c.id = board_id_;
        return sdr_aid::encode(c, out, cap);
    }

    // The frame frame() last gave went out at now_us.
    void sent(int64_t now_us)
    {
        seq_++;
        due_us_ = now_us + kRepeatUs;
    }

    // A rest of this side's went to the board at now_us.
    void noteRestSent(int64_t now_us)
    {
        rests_sent_++;
        rest_us_ = now_us;
        have_rest_ = true;
    }

    bool reportHeld(int64_t now_us) const { return have_report_ && now_us - report_us_ <= kReportFreshUs; }
    uint8_t state() const { return state_; }
    uint8_t boardId() const { return board_id_; }

    // The status at now_us. b2r: this side's mounting as it is now, row-major,
    // v_body = b2r v_imu (b2r_active_R). board: the receiver on J1 is the
    // board; line: its clock line serves (the rests go); still: this side's
    // last 1 s window was still.
    SdrAlignStatusData status(int64_t now_us, const float b2r[9], bool board, bool line, bool still) const
    {
        SdrAlignStatusData s;
        memset(&s, 0, sizeof(s));
        s.ver = SDR_ALIGN_STATUS_VER;
        const bool held = reportHeld(now_us);
        s.flags = (uint8_t)((board ? SDRA_BOARD : 0u) | (held ? SDRA_REPORT : 0u) | (line ? SDRA_LINE : 0u) |
                            (still ? SDRA_STILL : 0u));
        s.fc_cmd_state = state_;
        s.fc_cmd_op = op_;
        s.fc_cmd_id = app_id_;
        s.rests_sent = rests_sent_;
        s.rest_age_ds = ageDs(have_rest_, rest_us_, now_us);
        s.board_age_ds = ageDs(have_report_, report_us_, now_us);
        s.q[0] = 1.0f;
        for (int k = 0; k < 3; k++) s.scale[k] = 1.0f;
        if (!have_report_)
            return s;
        // The last report, held or not (board_age_ds and SDRA_REPORT say how old).
        const sdr_rtcm::TrAlign& a = report_;
        const bool body = a.frame == sdr_rtcm::TrAlign::kFrameBody;
        s.flags |= (uint8_t)((a.time ? SDRA_TIME : 0u) | (a.autosave ? SDRA_AUTOSAVE : 0u) | (body ? SDRA_BODY : 0u));
        s.imu = a.imu;
        s.have = a.have;
        s.from = a.from;
        s.saved = a.saved;
        s.n_pose = a.n_pose > SDR_ALIGN_POSES ? SDR_ALIGN_POSES : a.n_pose;
        s.cal_axes = a.cal_axes;
        s.last_rest = a.last_rest;
        s.gen = a.gen;
        s.n_rest = a.n_rest;
        s.n_pair = a.n_pair;
        s.cmd_id = a.cmd_id;
        s.cmd_op = a.cmd_op;
        s.cmd_res = a.cmd_res;
        s.spread_deg = (float)a.spread_deg;
        s.resid_deg = (float)a.resid_deg;
        s.cal_rms_mps2 = (float)a.cal_rms;
        s.moved_deg = (float)a.moved_deg;
        // The mounting into this IMU's frame: q_imu<-U10 = conj(q_b2r) (x) q_body<-U10.
        double q[4] = {a.q[0], a.q[1], a.q[2], a.q[3]};
        if (body)
        {
            double R[9], qb[4];
            for (int k = 0; k < 9; k++) R[k] = b2r[k];
            sdr_aid_fc::quatOfDcm(R, qb);
            const double qbc[4] = {qb[0], -qb[1], -qb[2], -qb[3]};
            sdr_aid_fc::quatMul(qbc, a.q, q);
        }
        if (q[0] < 0.0)
            for (int k = 0; k < 4; k++) q[k] = -q[k];
        for (int k = 0; k < 4; k++) s.q[k] = (float)q[k];
        for (int k = 0; k < 3; k++)
        {
            s.scale[k] = (float)a.scale[k];
            s.bias_mps2[k] = (float)a.bias[k];
        }
        for (int i = 0; i < s.n_pose; i++)
        {
            double d[3] = {a.pose[i].dir[0], a.pose[i].dir[1], a.pose[i].dir[2]};
            if (body)
            {
                // v_imu = b2r^T v_body
                const double b[3] = {d[0], d[1], d[2]};
                for (int k = 0; k < 3; k++) d[k] = b2r[k] * b[0] + b2r[3 + k] * b[1] + b2r[6 + k] * b[2];
            }
            for (int k = 0; k < 3; k++)
            {
                const double c = std::round(d[k] * 127.0);
                s.pose_dir[i][k] = (int8_t)(c > 127.0 ? 127 : (c < -127.0 ? -127 : c));
            }
            s.pose_pairs[i] = a.pose[i].pairs;
        }
        return s;
    }

    // Whether the status is due at now_us: a change since the last, or a
    // second gone.
    bool publishDue(int64_t now_us) const { return changed_ || !published_ || now_us - pub_us_ >= kPublishUs; }
    void published(int64_t now_us)
    {
        changed_ = false;
        published_ = true;
        pub_us_ = now_us;
    }

private:
    // A report that says something new: not its seq, nor the same state again.
    static bool moved(const sdr_rtcm::TrAlign& o, const sdr_rtcm::TrAlign& a)
    {
        return o.gen != a.gen || o.imu != a.imu || o.time != a.time || o.have != a.have || o.from != a.from ||
               o.saved != a.saved || o.autosave != a.autosave || o.n_pose != a.n_pose || o.last_rest != a.last_rest ||
               o.n_rest != a.n_rest || o.n_pair != a.n_pair || o.cmd_id != a.cmd_id || o.cmd_res != a.cmd_res;
    }

    sdr_rtcm::TrAlign report_ = {};
    bool     have_report_ = false;
    int64_t  report_us_ = 0;
    uint8_t  state_ = SDRA_CMD_IDLE;
    uint8_t  op_ = 0;
    uint8_t  app_id_ = 0;
    uint8_t  board_id_ = 0;
    uint8_t  last_board_id_ = 0;
    int64_t  start_us_ = 0;
    int64_t  due_us_ = 0;
    uint8_t  seq_ = 0;
    uint8_t  rests_sent_ = 0;
    int64_t  rest_us_ = 0;
    bool     have_rest_ = false;
    bool     changed_ = false;
    bool     published_ = false;
    int64_t  pub_us_ = 0;
};

}  // namespace sdr_align
