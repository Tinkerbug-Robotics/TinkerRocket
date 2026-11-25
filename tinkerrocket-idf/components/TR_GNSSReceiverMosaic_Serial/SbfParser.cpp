// SbfParser.cpp — see SbfParser.h. Pure: no ESP-IDF, host-tested.
#include "SbfParser.h"

#include <cmath>
#include <cstring>

namespace mosaic
{

namespace
{

struct CrcTable
{
    uint16_t t[256];
    constexpr CrcTable() : t()
    {
        for (int i = 0; i < 256; i++)
        {
            uint16_t c = static_cast<uint16_t>(i << 8);
            for (int k = 0; k < 8; k++)
            {
                c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021) : static_cast<uint16_t>(c << 1);
            }
            t[i] = c;
        }
    }
};
constexpr CrcTable kCrc;

inline uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t u32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline float f32(const uint8_t* p)
{
    const uint32_t x = u32(p);
    float f;
    std::memcpy(&f, &x, sizeof(f));
    return f;
}
inline double f64(const uint8_t* p)
{
    const uint64_t x = static_cast<uint64_t>(u32(p)) | (static_cast<uint64_t>(u32(p + 4)) << 32);
    double d;
    std::memcpy(&d, &x, sizeof(d));
    return d;
}

// SBF's float do-not-use value is -2e10; anything that far down is one.
inline bool dnu(double x) { return !(x > -1.0e10); }

// A prompt is the whole line: 2-4 capitals and 1-2 digits ("COM1", "USB2",
// "IP10"), or "STOP", then '>'. `n` counts the '>'.
bool promptName(const char* s, size_t n, char* out, size_t cap)
{
    if (n < 3 || s[n - 1] != '>') return false;
    const size_t name_n = n - 1;
    if (name_n == 4 && std::memcmp(s, "STOP", 4) == 0)
    {
        if (cap < 5) return false;
        std::memcpy(out, s, 4);
        out[4] = '\0';
        return true;
    }
    size_t letters = 0;
    while (letters < name_n && s[letters] >= 'A' && s[letters] <= 'Z') letters++;
    const size_t digits = name_n - letters;
    if (letters < 2 || letters > 4 || digits < 1 || digits > 2) return false;
    for (size_t i = letters; i < name_n; i++)
    {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    if (cap <= name_n) return false;
    std::memcpy(out, s, name_n);
    out[name_n] = '\0';
    return true;
}

inline int32_t roundToI32(double x)
{
    if (x >= 2147483647.0) return 2147483647;
    if (x <= -2147483648.0) return static_cast<int32_t>(-2147483647 - 1);
    return static_cast<int32_t>(std::lround(x));
}

// The u-blox driver rounds its accuracy to whole meters and saturates at 255;
// the receiver's figure is 95 %, which this halves to that driver's
// one-sigma scale first.
inline uint8_t accuracyM(uint16_t acc95_cm)
{
    if (acc95_cm == 65535) return 255;
    const uint32_t m = (static_cast<uint32_t>(acc95_cm) + 100U) / 200U;
    return static_cast<uint8_t>(m > 255U ? 255U : m);
}

// Days from 1970-01-01 to a civil date's inverse (H. Hinnant's algorithm).
void civilFromDays(int64_t z, int& y, unsigned& m, unsigned& d)
{
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = static_cast<int>(yy + (m <= 2 ? 1 : 0));
}

constexpr int64_t kGpsEpochDays = 3657;   // 1980-01-06 counted from 1970-01-01
constexpr int     kLeapSecondsFallback = 18;   // GPS - UTC since 2017-01-01

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc)
{
    for (size_t i = 0; i < len; i++)
    {
        crc = static_cast<uint16_t>((crc << 8) ^ kCrc.t[((crc >> 8) ^ data[i]) & 0xFF]);
    }
    return crc;
}

// ── Stream ──────────────────────────────────────────────────────────────

void SbfParser::feed(const uint8_t* data, size_t n, SbfSink& sink)
{
    for (size_t i = 0; i < n; i++)
    {
        index_ = i;
        step(data[i], sink);
        // What a false sync gave back is read before the next new byte.
        while (back_n_ > 0)
        {
            const uint8_t b = back_[back_head_];
            back_head_ = (back_head_ + 1) % kMaxBlock;
            back_n_--;
            step(b, sink);
        }
    }
}

void SbfParser::reset()
{
    state_ = State::TEXT;
    n_ = 0;
    want_ = 0;
    line_n_ = 0;
    line_cut_ = false;
    back_head_ = 0;
    back_n_ = 0;
}

void SbfParser::step(uint8_t b, SbfSink& sink)
{
    switch (state_)
    {
        case State::TEXT:
            if (b == '$')
            {
                state_ = State::DOLLAR;
                return;
            }
            textByte(b, sink);
            return;

        case State::DOLLAR:
            // Every text item on this port starts its own line, so a '$'
            // ends whatever text came before it.
            flushLine(sink);
            if (b == '@')
            {
                buf_[0] = '$';
                buf_[1] = '@';
                n_ = 2;
                want_ = 0;
                state_ = State::BLOCK;
                return;
            }
            state_ = State::TEXT;
            textByte('$', sink);
            if (b == '$')
            {
                state_ = State::DOLLAR;
                return;
            }
            textByte(b, sink);
            return;

        case State::BLOCK:
            buf_[n_++] = b;
            if (n_ == 8)
            {
                const uint16_t len = u16(buf_ + 6);
                if (len < 8 || (len % 4) != 0 || len > kMaxBlock)
                {
                    bad_lengths_++;
                    resync();
                    return;
                }
                want_ = len;
            }
            if (want_ != 0 && n_ == want_)
            {
                const size_t len = n_;
                if (crc16Ccitt(buf_ + 4, len - 4) == u16(buf_ + 2))
                {
                    blocks_++;
                    state_ = State::TEXT;
                    n_ = 0;
                    want_ = 0;
                    sink.onBlock(buf_, len, index_);   // valid until the next byte
                }
                else
                {
                    crc_fails_++;
                    resync();
                }
            }
            return;
    }
}

void SbfParser::textByte(uint8_t b, SbfSink& sink)
{
    if (b == '\r' || b == '\n')
    {
        flushLine(sink);
        return;
    }
    // Not text: noise between items, or a block's bytes after a false sync.
    if (b < 0x20 || b > 0x7E) return;
    if (line_n_ < kMaxLine)
    {
        line_[line_n_++] = static_cast<char>(b);
    }
    else
    {
        line_cut_ = true;
    }
    if (b == '>' && !line_cut_)
    {
        char port[8];
        if (promptName(line_, line_n_, port, sizeof(port)))
        {
            line_n_ = 0;
            sink.onPrompt(port, index_);
        }
    }
}

void SbfParser::flushLine(SbfSink& sink)
{
    if (line_n_ == 0)
    {
        line_cut_ = false;
        return;
    }
    if (line_cut_) long_lines_++;
    line_[line_n_] = '\0';
    line_n_ = 0;
    line_cut_ = false;
    sink.onLine(line_, index_);
}

void SbfParser::resync()
{
    // The '$' that started it was not a block's. Everything after it gets a
    // second look, ahead of what is already waiting, in its original order.
    size_t m = n_ > 0 ? n_ - 1 : 0;
    if (back_n_ + m > kMaxBlock) m = kMaxBlock - back_n_;   // cannot happen; see back_
    back_head_ = (back_head_ + kMaxBlock - m) % kMaxBlock;
    for (size_t k = 0; k < m; k++)
    {
        back_[(back_head_ + k) % kMaxBlock] = buf_[1 + k];
    }
    back_n_ += m;
    n_ = 0;
    want_ = 0;
    state_ = State::TEXT;
}

// ── Decoders ────────────────────────────────────────────────────────────

bool sbfHeader(const uint8_t* b, size_t len, SbfHeader& out)
{
    if (len < 14 || b[0] != '$' || b[1] != '@' || u16(b + 6) != len) return false;
    const uint16_t id = u16(b + 4);
    out.number = static_cast<uint16_t>(id & 0x1FFF);
    out.revision = static_cast<uint8_t>(id >> 13);
    out.length = static_cast<uint16_t>(len);
    out.tow_ms = u32(b + 8);
    out.wnc = u16(b + 12);
    return true;
}

bool decodePvtGeodetic(const uint8_t* b, size_t len, PvtGeodetic& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_PVT_GEODETIC || len < 75) return false;
    out = PvtGeodetic{};
    out.tow_ms = h.tow_ms;
    out.wnc = h.wnc;
    out.mode = b[14];
    out.error = b[15];
    out.lat_rad = f64(b + 16);
    out.lon_rad = f64(b + 24);
    out.height_m = f64(b + 32);
    out.undulation_m = f32(b + 40);
    out.vn = f32(b + 44);
    out.ve = f32(b + 48);
    out.vu = f32(b + 52);
    out.rx_clk_bias_ms = f64(b + 60);
    out.rx_clk_drift_ppm = f32(b + 68);
    out.time_system = b[72];
    out.nr_sv = b[74];
    if (len >= 94)
    {
        out.latency_100us = u16(b + 88);
        out.h_acc_cm = u16(b + 90);
        out.v_acc_cm = u16(b + 92);
    }
    out.has_solution = (out.mode & 0x0F) != 0 && out.error == 0 && h.tow_ms != kTowDnu &&
                       h.wnc != kWncDnu && !dnu(out.lat_rad) && !dnu(out.lon_rad) &&
                       !dnu(out.height_m) && !dnu(out.vn) && !dnu(out.ve) && !dnu(out.vu);
    return true;
}

bool decodeReceiverTime(const uint8_t* b, size_t len, ReceiverTime& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_RECEIVER_TIME || len < 22) return false;
    out.delta_ls = static_cast<int8_t>(b[20]);
    out.sync_level = b[21];
    return true;
}

bool decodeDop(const uint8_t* b, size_t len, Dop& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_DOP || len < 18) return false;
    out.nr_sv = b[14];
    out.pdop_x100 = u16(b + 16);
    return true;
}

bool decodeReceiverStatus(const uint8_t* b, size_t len, ReceiverStatus& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_RECEIVER_STATUS || len < 28) return false;
    out = ReceiverStatus{};
    out.cpu_load = b[14];
    out.ext_error = b[15];
    out.uptime_s = u32(b + 16);
    out.rx_state = u32(b + 20);
    out.rx_error = u32(b + 24);
    if (h.revision >= 1 && len >= 32 && b[31] != 255) out.temp_c = static_cast<int16_t>(b[31] - 100);
    return true;
}

bool decodeXppsOffset(const uint8_t* b, size_t len, XppsOffset& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_XPPS_OFFSET || len < 20) return false;
    out.tow_ms = h.tow_ms;
    out.wnc = h.wnc;
    out.sync_age_s = b[14];
    out.time_scale = b[15];
    out.offset_ns = f32(b + 16);
    return true;
}

bool decodeDiskStatus(const uint8_t* b, size_t len, DiskStatus& out)
{
    SbfHeader h;
    if (!sbfHeader(b, len, h) || h.number != SBF_DISK_STATUS || len < 20) return false;
    out = DiskStatus{};
    const uint8_t n = b[14];
    const uint8_t sbl = b[15];
    if (sbl < 12) return true;   // no sub-block we can read: no card reported
    for (uint8_t k = 0; k < n; k++)
    {
        const size_t at = 20 + static_cast<size_t>(k) * sbl;
        if (at + 12 > len) break;
        if (b[at] != 1) continue;   // DiskID 1 = the internal SD card
        out.present = true;
        out.status = b[at + 1];
        out.used_bytes = (static_cast<uint64_t>(u16(b + at + 2)) << 32) | u32(b + at + 4);
        out.size_mb = u32(b + at + 8);
        out.error = (h.revision >= 1 && at + 14 <= len) ? b[at + 13] : 0;
        break;
    }
    return true;
}

void gpsToUtc(uint16_t wnc, uint32_t tow_ms, int leap_s, UtcTime& out)
{
    const int64_t s = static_cast<int64_t>(wnc) * 604800 + tow_ms / 1000 - leap_s;
    const int64_t days = kGpsEpochDays + (s >= 0 ? s / 86400 : (s - 86399) / 86400);
    const int64_t sod = s - (days - kGpsEpochDays) * 86400;
    int y;
    unsigned m, d;
    civilFromDays(days, y, m, d);
    out.year = static_cast<uint16_t>(y);
    out.month = static_cast<uint8_t>(m);
    out.day = static_cast<uint8_t>(d);
    out.hour = static_cast<uint8_t>(sod / 3600);
    out.minute = static_cast<uint8_t>((sod % 3600) / 60);
    out.second = static_cast<uint8_t>(sod % 60);
    out.millisecond = static_cast<uint16_t>(tow_ms % 1000);
}

void gnssDataFromPvt(const PvtGeodetic& pvt, int leap_s, uint16_t pdop_x100, GNSSData& out)
{
    std::memset(&out, 0, sizeof(out));
    if (pvt.tow_ms != kTowDnu && pvt.wnc != kWncDnu)
    {
        UtcTime utc;
        gpsToUtc(pvt.wnc, pvt.tow_ms, (leap_s == -128) ? kLeapSecondsFallback : leap_s, utc);
        out.year = utc.year;
        out.month = utc.month;
        out.day = utc.day;
        out.hour = utc.hour;
        out.minute = utc.minute;
        out.second = utc.second;
        out.milli_second = utc.millisecond;
    }
    out.num_sats = pvt.nr_sv == 255 ? 0 : pvt.nr_sv;
    const uint32_t pdop_x10 = (static_cast<uint32_t>(pdop_x100) + 5U) / 10U;
    out.pdop_x10 = static_cast<uint8_t>(pdop_x10 > 255U ? 255U : pdop_x10);
    out.h_acc_m = 255;
    out.v_acc_m = 255;
    if (!pvt.has_solution) return;   // fix_mode 0: the position fields stay 0

    static constexpr double kRad2Deg = 180.0 / M_PI;
    out.fix_mode = (pvt.mode & 0x80) ? 2 : 3;
    out.lat_e7 = roundToI32(pvt.lat_rad * kRad2Deg * 1.0e7);
    out.lon_e7 = roundToI32(pvt.lon_rad * kRad2Deg * 1.0e7);
    const double sep = dnu(pvt.undulation_m) ? 0.0 : static_cast<double>(pvt.undulation_m);
    out.alt_mm = roundToI32((pvt.height_m - sep) * 1000.0);
    out.vel_e_mmps = roundToI32(static_cast<double>(pvt.ve) * 1000.0);
    out.vel_n_mmps = roundToI32(static_cast<double>(pvt.vn) * 1000.0);
    out.vel_u_mmps = roundToI32(static_cast<double>(pvt.vu) * 1000.0);
    out.h_acc_m = accuracyM(pvt.h_acc_cm);
    out.v_acc_m = accuracyM(pvt.v_acc_cm);
}

}  // namespace mosaic
