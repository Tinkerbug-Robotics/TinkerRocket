// SkyTraqLink.cpp — see SkyTraqLink.h
#include "SkyTraqLink.h"

#include <cstdio>
#include <cstring>

#include "SkyTraqBinary.h"

namespace skytraq
{

namespace
{

// Where to listen, in order: the target first (the FC rebooted alone, the
// receiver still set up), then the factory 115200, then the rest of 0x05's.
constexpr uint32_t kProbeBauds[] = {921600, 115200, 460800, 230400, 38400, 9600};
constexpr uint32_t kNumProbeBauds = sizeof(kProbeBauds) / sizeof(kProbeBauds[0]);

const uint8_t kQueryVersion[] = {0x02, 0x01};

// The readbacks (CHECK), in order: the body, and the reply's ID and sub-ID.
struct Query
{
    uint8_t     body[2];
    uint8_t     len;
    uint8_t     reply_id;
    int16_t     reply_sub;
    const char* name;
};
constexpr Query kQueries[] = {
    {{0x10, 0x00}, 1, MSG_POS_RATE, -1, "position rate"},
    {{0x1F, 0x00}, 1, MSG_BIN_MEAS_CFG, -1, "binary output"},
    {{0x64, 0x18}, 2, MSG_SUB_64, 0x8B, "navigation mode"},
    {{0x15, 0x00}, 1, MSG_POWER_MODE, -1, "power mode"},
};
constexpr int kNumQueries = sizeof(kQueries) / sizeof(kQueries[0]);

void putBe32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

}  // namespace

int baudCode(uint32_t baud)
{
    switch (baud)
    {
    case 4800: return 0;
    case 9600: return 1;
    case 19200: return 2;
    case 38400: return 3;
    case 57600: return 4;
    case 115200: return 5;
    case 230400: return 6;
    case 460800: return 7;
    case 921600: return 8;
    default: return -1;
    }
}

int rateCode(uint8_t hz)
{
    switch (hz)
    {
    case 1: return 0;
    case 2: return 1;
    case 4: return 2;
    case 5: return 3;
    case 10: return 4;
    case 20: return 5;
    case 8: return 6;
    default: return -1;
    }
}

void SkyTraqLink::push(LinkAction::Kind kind, uint32_t baud, const uint8_t* body, size_t len)
{
    if (q_n_ >= kQueue)
        return;   // a state queues at most two; never reached
    LinkAction& a = q_[(q_head_ + q_n_) % kQueue];
    a = LinkAction{};
    a.kind = kind;
    a.baud = baud;
    if (kind == LinkAction::SEND)
    {
        uint8_t* buf = tx_[tx_next_];
        tx_next_ = (tx_next_ + 1) % kQueue;
        a.len = buildFrame(body, len, buf, kMaxCommand + 8);
        a.bytes = buf;
        if (a.len == 0)
            return;
    }
    else
    {
        baud_ = baud;
    }
    q_n_++;
}

void SkyTraqLink::noteFailure(const char* what)
{
    std::snprintf(last_failure_, sizeof last_failure_, "%s", what);
}

void SkyTraqLink::buildSetup()
{
    const uint8_t attr = cfg_.flash ? 1 : 0;
    n_cmds_ = 0;
    auto add = [&](const uint8_t* b, size_t n, int16_t sub, bool required, const char* name) {
        Cmd& c = cmds_[n_cmds_++];
        std::memcpy(c.body, b, n);
        c.len = static_cast<uint8_t>(n);
        c.ack_sub = sub;
        c.required = required;
        c.name = name;
    };
    {
        // AN0037 0x6A/0x06: base mode (1), kinematic function (0), survey
        // length 2000 and standard deviation 30 (unused by the kinematic
        // function), no saved position, then the attribute: 37 B, as
        // skytraq_raw.py --kinematic-base sends it
        uint8_t b[37] = {};
        b[0] = 0x6A;
        b[1] = 0x06;
        b[2] = 0x01;
        b[3] = 0x00;
        putBe32(b + 4, 2000);
        putBe32(b + 8, 30);
        // b[12..35]: latitude, longitude (float64), height, antenna height (float32): 0
        b[36] = attr;
        add(b, sizeof b, 0x06, true, "RTK kinematic base");
    }
    {
        const int rc = rateCode(cfg_.rate_hz);
        const uint8_t b[] = {0x1E, static_cast<uint8_t>(rc < 0 ? 5 : rc), 0x00, 0x00, 0x01, 0x01, cfg_.subframes,
                             0x01, attr};
        add(b, sizeof b, -1, true, "binary output");
    }
    {
        const uint8_t b[] = {0x0E, cfg_.rate_hz, attr};
        add(b, sizeof b, -1, true, "position rate");
    }
    {
        const uint8_t b[] = {0x64, 0x17, cfg_.nav_mode, attr};
        add(b, sizeof b, 0x17, true, "navigation mode");
    }
    {
        const uint8_t b[] = {0x0C, 0x00, attr};
        add(b, sizeof b, -1, true, "power normal");
    }
    {
        const uint8_t b[] = {0x2B, 0x01, cfg_.elev_mask_deg, 0x00, attr};
        add(b, sizeof b, -1, false, "elevation mask");
    }
}

void SkyTraqLink::start(const LinkConfig& cfg, uint32_t now_ms)
{
    cfg_ = cfg;
    q_head_ = q_n_ = 0;
    setups_in_row_ = 0;
    degraded_ = false;
    last_frame_ms_ = now_ms;
    last_raw_ms_ = now_ms;
    buildSetup();
    enterProbe(0, now_ms);
}

void SkyTraqLink::enterProbe(uint32_t idx, uint32_t now_ms)
{
    state_ = LinkState::PROBE;
    probe_idx_ = idx % kNumProbeBauds;
    t0_ = now_ms;
    heard_ = false;
    nmea_seen_ = 0;
    queried_ = false;
    push(LinkAction::SET_BAUD, kProbeBauds[probe_idx_], nullptr, 0);
    push(LinkAction::SEND, 0, kQueryVersion, sizeof kQueryVersion);
}

void SkyTraqLink::enterSwitch(uint32_t now_ms)
{
    state_ = LinkState::SWITCH_BAUD;
    t0_ = now_ms;
    const int code = baudCode(cfg_.target_baud);
    const uint8_t b[] = {0x05, 0x00, static_cast<uint8_t>(code < 0 ? 8 : code), static_cast<uint8_t>(cfg_.flash ? 1 : 0)};
    push(LinkAction::SEND, 0, b, sizeof b);
}

void SkyTraqLink::enterVerify(uint32_t now_ms)
{
    state_ = LinkState::VERIFY_BAUD;
    t0_ = now_ms;
    heard_ = false;
    push(LinkAction::SET_BAUD, cfg_.target_baud, nullptr, 0);
    push(LinkAction::SEND, 0, kQueryVersion, sizeof kQueryVersion);
}

void SkyTraqLink::enterConfigure(uint32_t now_ms)
{
    state_ = LinkState::CONFIGURE;
    refused_ = 0;
    checks_failed_ = 0;
    last_frame_ms_ = now_ms;
    cmd_ = 0;
    sendCommand(now_ms);
}

void SkyTraqLink::sendCommand(uint32_t now_ms)
{
    attempt_ = 0;
    got_ack_ = got_nack_ = false;
    t0_ = now_ms;
    push(LinkAction::SEND, 0, cmds_[cmd_].body, cmds_[cmd_].len);
}

void SkyTraqLink::nextCommand(uint32_t now_ms)
{
    if (++cmd_ < n_cmds_)
        sendCommand(now_ms);
    else
        enterCheck(now_ms);
}

void SkyTraqLink::enterCheck(uint32_t now_ms)
{
    state_ = LinkState::CHECK;
    query_ = 0;
    sendQuery(now_ms);
}

void SkyTraqLink::sendQuery(uint32_t now_ms)
{
    attempt_ = 0;
    got_reply_ = false;
    reply_ok_ = false;
    t0_ = now_ms;
    push(LinkAction::SEND, 0, kQueries[query_].body, kQueries[query_].len);
}

void SkyTraqLink::enterStream(uint32_t now_ms)
{
    state_ = LinkState::STREAM;
    t0_ = now_ms;
    raw_ok_in_row_ = 0;
}

void SkyTraqLink::enterRunning(uint32_t now_ms)
{
    state_ = LinkState::RUNNING;
    setups_++;
    last_frame_ms_ = now_ms;
    last_raw_ms_ = now_ms;
}

bool SkyTraqLink::checkReply(const uint8_t* b, size_t len)
{
    const int rc = rateCode(cfg_.rate_hz);
    switch (query_)
    {
    case 0:   // 0x86: [1] the rate, Hz
        return len >= 2 && b[1] == cfg_.rate_hz;
    case 1:   // 0x89: [1] rate code [2] meas time [3] raw [4] channel status [5] state [6] subframes [7] 0xE5
        return len >= 8 && b[1] == rc && b[4] == 1 && b[5] == 1 && b[6] == cfg_.subframes && b[7] == 1;
    case 2:   // 0x64/0x8B: [2] the mode
        return len >= 3 && b[2] == cfg_.nav_mode;
    case 3:   // 0xB9: [1] 0 normal, 1 save
        return len >= 2 && b[1] == 0;
    default:
        return true;
    }
}

void SkyTraqLink::onNmea(uint32_t now_ms)
{
    last_frame_ms_ = now_ms;
    if (state_ == LinkState::PROBE && nmea_seen_ < 255 && ++nmea_seen_ >= 2)
    {
        heard_ = true;
        found_nmea_ = true;
    }
}

void SkyTraqLink::onFrame(const uint8_t* b, size_t len, uint32_t now_ms)
{
    if (len == 0)
        return;
    last_frame_ms_ = now_ms;
    if (state_ == LinkState::PROBE && !heard_)
    {
        heard_ = true;
        found_nmea_ = false;
    }
    else if (state_ == LinkState::VERIFY_BAUD)
    {
        heard_ = true;
    }

    if (b[0] == MSG_EXT_RAW && len >= 11)
    {
        raw_period_ms_ = be16(b + 9);
        last_raw_ms_ = now_ms;
        if (state_ == LinkState::STREAM)
        {
            const uint16_t want = static_cast<uint16_t>(1000 / (cfg_.rate_hz ? cfg_.rate_hz : 1));
            raw_ok_in_row_ = raw_period_ms_ == want ? static_cast<uint8_t>(raw_ok_in_row_ + 1) : 0;
        }
        return;
    }
    if (b[0] == MSG_SW_VERSION && len >= 14)
    {
        std::snprintf(version_, sizeof version_, "%u.%u.%u %u.%u.%u %02u%02u%02u", b[3], b[4], b[5], b[7], b[8], b[9],
                      b[11], b[12], b[13]);
        return;
    }
    bool acked = false;
    if (state_ == LinkState::CONFIGURE && cmd_ < n_cmds_ &&
        isAckOf(b, len, cmds_[cmd_].body[0], cmds_[cmd_].ack_sub, acked))
    {
        if (acked)
            got_ack_ = true;
        else
            got_nack_ = true;
        return;
    }
    if (state_ == LinkState::CHECK && query_ < kNumQueries)
    {
        const Query& q = kQueries[query_];
        if (isAckOf(b, len, q.body[0], q.len > 1 ? q.body[1] : -1, acked))
        {
            if (!acked)
            {
                // a query this receiver does not answer: nothing to check
                got_reply_ = true;
                reply_ok_ = true;
            }
            return;
        }
        if (b[0] == q.reply_id && (q.reply_sub < 0 || (len >= 2 && b[1] == q.reply_sub)))
        {
            got_reply_ = true;
            reply_ok_ = checkReply(b, len);
        }
    }
}

LinkAction SkyTraqLink::poll(uint32_t now_ms)
{
    if (q_n_ == 0)
    {
        const uint32_t dt = now_ms - t0_;
        switch (state_)
        {
        case LinkState::PROBE:
            if (heard_)
            {
                found_baud_ = baud_;
                if (baud_ == cfg_.target_baud)
                    enterConfigure(now_ms);
                else
                    enterSwitch(now_ms);
            }
            else if (!queried_ && dt >= cfg_.probe_ms / 2)
            {
                // a receiver set up for binary with nothing streaming answers this
                queried_ = true;
                push(LinkAction::SEND, 0, kQueryVersion, sizeof kQueryVersion);
            }
            else if (dt >= cfg_.probe_ms)
            {
                enterProbe(probe_idx_ + 1, now_ms);
            }
            break;
        case LinkState::SWITCH_BAUD:
            if (dt >= cfg_.switch_ms)
                enterVerify(now_ms);
            break;
        case LinkState::VERIFY_BAUD:
            if (heard_)
                enterConfigure(now_ms);
            else if (dt >= cfg_.reply_ms)
            {
                noteFailure("no answer at the target baud");
                enterProbe(0, now_ms);
            }
            break;
        case LinkState::CONFIGURE:
            if (got_ack_)
                nextCommand(now_ms);
            else if (got_nack_)
            {
                refused_++;
                noteFailure(cmds_[cmd_].name);
                if (cmds_[cmd_].required)
                    degraded_ = true;
                nextCommand(now_ms);
            }
            else if (dt >= cfg_.reply_ms)
            {
                if (now_ms - last_frame_ms_ >= cfg_.silence_ms)
                {
                    // the receiver went quiet (a reset back to its factory baud)
                    noteFailure("silent during setup");
                    enterProbe(0, now_ms);
                }
                else if (attempt_ < cfg_.retries)
                {
                    attempt_++;
                    t0_ = now_ms;
                    push(LinkAction::SEND, 0, cmds_[cmd_].body, cmds_[cmd_].len);
                }
                else
                {
                    refused_++;
                    noteFailure(cmds_[cmd_].name);
                    nextCommand(now_ms);
                }
            }
            break;
        case LinkState::CHECK:
            if (got_reply_ || dt >= cfg_.reply_ms)
            {
                if (!got_reply_ && attempt_ < cfg_.retries)
                {
                    attempt_++;
                    t0_ = now_ms;
                    push(LinkAction::SEND, 0, kQueries[query_].body, kQueries[query_].len);
                    break;
                }
                if (!got_reply_ || !reply_ok_)
                {
                    checks_failed_++;
                    noteFailure(kQueries[query_].name);
                }
                if (++query_ < kNumQueries)
                {
                    sendQuery(now_ms);
                }
                else if (checks_failed_ > 0 && ++setups_in_row_ < cfg_.max_setups)
                {
                    enterConfigure(now_ms);
                }
                else
                {
                    if (checks_failed_ > 0)
                        degraded_ = true;
                    enterStream(now_ms);
                }
            }
            break;
        case LinkState::STREAM:
            if (raw_ok_in_row_ >= 3)
            {
                setups_in_row_ = 0;
                enterRunning(now_ms);
            }
            else if (dt >= cfg_.stream_ms)
            {
                noteFailure("no 0xE5 at the rate");
                if (++setups_in_row_ < cfg_.max_setups)
                {
                    enterConfigure(now_ms);
                }
                else
                {
                    degraded_ = true;
                    setups_in_row_ = 0;
                    enterRunning(now_ms);
                }
            }
            break;
        case LinkState::RUNNING:
            if (now_ms - last_frame_ms_ >= cfg_.silence_ms)
            {
                relinks_++;
                degraded_ = false;
                noteFailure("silent while running");
                enterProbe(0, now_ms);
            }
            else if (now_ms - last_raw_ms_ >= cfg_.no_raw_ms)
            {
                reconfigs_++;
                degraded_ = false;
                noteFailure("0xE5 stopped while running");
                enterConfigure(now_ms);
            }
            break;
        }
    }
    if (q_n_ == 0)
        return LinkAction{};
    const LinkAction a = q_[q_head_];
    q_head_ = (q_head_ + 1) % kQueue;
    q_n_--;
    return a;
}

}  // namespace skytraq
