// SkyTraqBinary.cpp — see SkyTraqBinary.h
#include "SkyTraqBinary.h"

#include <cmath>
#include <cstring>

namespace skytraq
{

uint16_t be16(const uint8_t* p)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t be32(const uint8_t* p)
{
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

float beF32(const uint8_t* p)
{
    const uint32_t u = be32(p);
    float f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}

double beF64(const uint8_t* p)
{
    const uint64_t u = (static_cast<uint64_t>(be32(p)) << 32) | be32(p + 4);
    double d;
    std::memcpy(&d, &u, sizeof d);
    return d;
}

size_t buildFrame(const uint8_t* body, size_t n, uint8_t* out, size_t cap)
{
    if (n == 0 || n > 0xFFFF || cap < n + 7)
        return 0;
    out[0] = 0xA0;
    out[1] = 0xA1;
    out[2] = static_cast<uint8_t>(n >> 8);
    out[3] = static_cast<uint8_t>(n & 0xFF);
    uint8_t cs = 0;
    for (size_t i = 0; i < n; ++i)
    {
        out[4 + i] = body[i];
        cs ^= body[i];
    }
    out[4 + n] = cs;
    out[5 + n] = 0x0D;
    out[6 + n] = 0x0A;
    return n + 7;
}

// ── FrameParser ─────────────────────────────────────────────────────────────

void FrameParser::reset()
{
    state_ = State::IDLE;
    raw_n_ = 0;
    want_ = 0;
    cs_ = 0;
    back_pos_ = 0;
    back_n_ = 0;
    text_n_ = 0;
    text_live_ = false;
}

void FrameParser::feed(const uint8_t* data, size_t n, FrameSink& sink)
{
    for (size_t i = 0; i < n; ++i)
    {
        index_ = i;
        step(data[i], sink);
        // what a false sync gave back, before the next new byte
        while (back_pos_ < back_n_)
            step(back_[back_pos_++], sink);
        back_pos_ = 0;
        back_n_ = 0;
    }
}

void FrameParser::textByte(uint8_t b)
{
    if (b == '$')
    {
        text_live_ = true;
        text_n_ = 0;
        return;
    }
    if (b == '\n' || b == '\r')
    {
        if (text_live_ && text_n_ >= 2)
        {
            text_lines_++;
            if (text_[0] == 'G' || text_[0] == 'B')
                nmea_lines_++;
        }
        text_live_ = false;
        text_n_ = 0;
        return;
    }
    if (text_live_ && text_n_ < sizeof text_)
        text_[text_n_] = static_cast<char>(b);
    if (text_live_ && text_n_ < 255)
        text_n_++;
}

void FrameParser::resync()
{
    // The candidate's bytes after its A0, from the next A0 on: none before it
    // can start a frame. They go back ahead of whatever is still to replay.
    size_t k = 1;
    while (k < raw_n_ && raw_[k] != 0xA0)
        ++k;
    const size_t head = raw_n_ - k;
    const size_t tail = back_n_ - back_pos_;
    std::memmove(back_ + head, back_ + back_pos_, tail);
    std::memcpy(back_, raw_ + k, head);
    back_pos_ = 0;
    back_n_ = head + tail;
    raw_n_ = 0;
    state_ = State::IDLE;
}

void FrameParser::step(uint8_t b, FrameSink& sink)
{
    switch (state_)
    {
    case State::IDLE:
        if (b == 0xA0)
        {
            raw_[0] = b;
            raw_n_ = 1;
            state_ = State::A1;
        }
        else
        {
            textByte(b);
        }
        return;
    case State::A1:
        if (b == 0xA1)
        {
            raw_[raw_n_++] = b;
            state_ = State::LEN_HI;
        }
        else
        {
            raw_n_ = 0;
            state_ = State::IDLE;
            step(b, sink);   // it may be the next A0, or text
        }
        return;
    case State::LEN_HI:
        raw_[raw_n_++] = b;
        want_ = static_cast<size_t>(b) << 8;
        state_ = State::LEN_LO;
        return;
    case State::LEN_LO:
        raw_[raw_n_++] = b;
        want_ |= b;
        if (want_ == 0 || want_ > kMaxBody)
        {
            bad_len_++;
            resync();
            return;
        }
        cs_ = 0;
        state_ = State::BODY;
        return;
    case State::BODY:
        raw_[raw_n_++] = b;
        cs_ ^= b;
        if (raw_n_ == 4 + want_)
            state_ = State::CS;
        return;
    case State::CS:
        raw_[raw_n_++] = b;
        if (b != cs_)
        {
            bad_cs_++;
            resync();
            return;
        }
        state_ = State::CR;
        return;
    case State::CR:
        raw_[raw_n_++] = b;
        if (b != 0x0D)
        {
            bad_cs_++;
            resync();
            return;
        }
        state_ = State::LF;
        return;
    case State::LF:
        raw_[raw_n_++] = b;
        if (b != 0x0A)
        {
            bad_cs_++;
            resync();
            return;
        }
        frames_++;
        state_ = State::IDLE;
        raw_n_ = 0;
        text_live_ = false;
        sink.onFrame(raw_ + 4, want_, index_);
        return;
    }
}

// ── 0xE5 ────────────────────────────────────────────────────────────────────

bool decodeRawE5(const uint8_t* b, size_t len, RawE5& out)
{
    if (len < 14 || b[0] != MSG_EXT_RAW || b[1] != 1)
        return false;
    const size_t listed = b[13];
    if (len < 14 + 31 * listed)
        return false;
    out.version = b[1];
    out.iod = b[2];
    out.week = be16(b + 3);
    out.tow_ms = be32(b + 5);
    out.period_ms = be16(b + 9);
    out.meas_ind = b[11];
    out.n_listed = static_cast<uint8_t>(listed);
    out.n = 0;
    for (size_t j = 0; j < listed && out.n < RawE5::kMaxCells; ++j)
    {
        const uint8_t* r = b + 14 + 31 * j;
        RawCell& c = out.cell[out.n++];
        c.gnss = r[0] & 0x0F;
        c.sig = static_cast<uint8_t>(r[0] >> 4);
        c.svid = r[1];
        c.lock_ind = static_cast<uint8_t>(r[2] >> 4);
        c.cn0 = r[3];
        c.pr_m = beF64(r + 4);
        c.carrier_cyc = beF64(r + 12);
        c.doppler_hz = beF32(r + 20);
        c.pr_std = r[24];
        c.carrier_std = r[25];
        c.doppler_std = r[26];
        c.ind = be16(r + 27);
    }
    return true;
}

uint32_t lockTimeMinMs(uint8_t lock_ind)
{
    if (lock_ind == 0)
        return 0;
    if (lock_ind > 15)
        lock_ind = 15;
    return 50U << lock_ind;
}

double dopplerHz(float sent)
{
    const double d = static_cast<double>(sent);
    if (d == 0.0 || d != std::floor(d))
        return d;
    return d + (d > 0.0 ? 0.5 : -0.5);
}

// ── 0xDF ────────────────────────────────────────────────────────────────────

bool decodeRcvState(const uint8_t* b, size_t len, RcvState& out)
{
    if (len < 81 || b[0] != MSG_RCV_STATE)
        return false;
    out.iod = b[1];
    out.nav_state = b[2];
    out.week = be16(b + 3);
    out.tow_s = beF64(b + 5);
    for (int k = 0; k < 3; ++k)
    {
        out.ecef_m[k] = beF64(b + 13 + 8 * k);
        out.vel_mps[k] = beF32(b + 37 + 4 * k);
    }
    out.clk_bias_m = beF64(b + 49);
    out.clk_drift_mps = beF32(b + 57);
    out.gdop = beF32(b + 61);
    out.pdop = beF32(b + 65);
    out.hdop = beF32(b + 69);
    out.vdop = beF32(b + 73);
    out.tdop = beF32(b + 77);
    return true;
}

// ── 0xE7 ────────────────────────────────────────────────────────────────────

bool decodeSvChStatus(const uint8_t* b, size_t len, ChannelSummary& out)
{
    if (len < 4 || b[0] != MSG_SV_CH_STATUS)
        return false;
    const size_t n = b[3];
    if (len < 4 + 7 * n)
        return false;
    out.iod = b[2];
    out.channels = static_cast<uint8_t>(n);
    out.used = 0;
    out.tracked = 0;
    // one count a satellite: a satellite's L1 and L5 channels are two records
    uint8_t seen_used[8][8] = {};      // [gnss][svid / 8] bit svid % 8 (svid < 64)
    uint8_t seen_tracked[8][8] = {};
    for (size_t i = 0; i < n; ++i)
    {
        const uint8_t* r = b + 4 + 7 * i;
        const uint8_t g = r[1] & 0x07;
        const uint8_t sv = r[2];
        if (sv >= 64)
            continue;
        const uint8_t bit = static_cast<uint8_t>(1U << (sv & 7));
        if ((r[6] & 0x10) && !(seen_used[g][sv >> 3] & bit))
        {
            seen_used[g][sv >> 3] |= bit;
            out.used++;
        }
        if (static_cast<int8_t>(r[5]) > 0 && !(seen_tracked[g][sv >> 3] & bit))
        {
            seen_tracked[g][sv >> 3] |= bit;
            out.tracked++;
        }
    }
    return true;
}

// ── ACK / NACK ──────────────────────────────────────────────────────────────

bool isAckOf(const uint8_t* b, size_t len, uint8_t id, int sub, bool& acked)
{
    if (len < 2 || (b[0] != MSG_ACK && b[0] != MSG_NACK) || b[1] != id)
        return false;
    if (sub >= 0 && len >= 3 && b[2] != static_cast<uint8_t>(sub))
        return false;
    acked = b[0] == MSG_ACK;
    return true;
}

}  // namespace skytraq
