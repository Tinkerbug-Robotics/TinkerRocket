// Lc86Parser.cpp — see Lc86Parser.h. NO ESP-IDF includes in this file:
// it must compile on the host for tests_cpp/test_lc86_parser.cpp.
#include "Lc86Parser.h"

#include <cstring>
#include <cstdlib>
#include <cmath>

namespace lc86
{

// ── Field helpers ───────────────────────────────────────────────────────
// A fixless module routinely emits EMPTY numeric fields; empty parses as 0
// (and the Quality gate in parsePqtmPvt keeps such epochs untrusted).

static double fieldDouble(const char* s)
{
    if (s == nullptr || *s == '\0') return 0.0;
    return strtod(s, nullptr);
}

static long fieldLong(const char* s)
{
    if (s == nullptr || *s == '\0') return 0;
    return strtol(s, nullptr, 10);
}

// v*scale, rounded, saturated to int32 — garbage input must clamp, never wrap.
static int32_t scaleI32(double v, double scale)
{
    const double x = v * scale;
    if (!(x == x)) return 0;  // NaN guard
    if (x >= 2147483647.0) return INT32_MAX;
    if (x <= -2147483648.0) return INT32_MIN;
    return (int32_t)llround(x);
}

// Round to whole units, saturate to uint8 (255 = worst/unknown). NaN and
// negatives → 0, matching the u-blox driver's (x + 500)/1000 truncation
// behavior for sub-half-unit values.
static uint8_t satRoundU8(double v)
{
    if (!(v > 0.0)) return 0;
    if (v >= 254.5) return 255;
    return (uint8_t)lround(v);
}

// "hhmmss.sss" → fields. Anything malformed leaves zeros (never trusted
// anyway unless the epoch's Quality gate passes).
static void parseHmsMs(const char* s, uint8_t& h, uint8_t& m, uint8_t& sec,
                       uint16_t& ms)
{
    h = m = sec = 0;
    ms = 0;
    if (s == nullptr) return;
    for (int i = 0; i < 6; i++)
    {
        // A short string hits its NUL here and bails before any overrun.
        if (s[i] < '0' || s[i] > '9') return;
    }
    h   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    m   = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    sec = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
    if (s[6] != '.') return;
    uint16_t frac = 0;
    int digits = 0;
    for (const char* q = s + 7; *q >= '0' && *q <= '9' && digits < 3; q++)
    {
        frac = (uint16_t)(frac * 10 + (uint16_t)(*q - '0'));
        digits++;
    }
    while (digits > 0 && digits < 3)  // ".5" means 500 ms, ".25" means 250 ms
    {
        frac = (uint16_t)(frac * 10);
        digits++;
    }
    ms = frac;
}

static int hexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// ── Public free functions ───────────────────────────────────────────────

uint8_t checksum(const char* data, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++)
    {
        cs ^= (uint8_t)data[i];
    }
    return cs;
}

size_t buildSentence(const char* body, char* out, size_t out_cap)
{
    static const char kHex[] = "0123456789ABCDEF";
    const size_t body_len  = strlen(body);
    const size_t frame_len = 1 + body_len + 3 + 2;  // '$' body "*HH" "\r\n"
    if (out_cap < frame_len + 1) return 0;
    const uint8_t cs = checksum(body, body_len);
    out[0] = '$';
    memcpy(out + 1, body, body_len);
    out[1 + body_len] = '*';
    out[2 + body_len] = kHex[cs >> 4];
    out[3 + body_len] = kHex[cs & 0x0F];
    out[4 + body_len] = '\r';
    out[5 + body_len] = '\n';
    out[frame_len]    = '\0';
    return frame_len;
}

// ── Stream assembly ─────────────────────────────────────────────────────

void Lc86Parser::resetLine()
{
    in_line_  = false;
    line_len_ = 0;
}

Event Lc86Parser::feed(uint8_t byte)
{
    const char c = (char)byte;

    if (c == '$')
    {
        // Sentence start always resynchronizes: '$' can never appear inside a
        // valid sentence body, so one mid-line means the previous line was
        // truncated (power glitch, baud change, RX ring overflow).
        if (in_line_ && line_len_ > 1) truncated_lines_++;
        in_line_  = true;
        line_len_ = 0;
        line_[line_len_++] = '$';
        return Event::NONE;
    }

    if (!in_line_) return Event::NONE;  // scanning for '$' between lines

    if (c == '\r' || c == '\n')
    {
        if (line_len_ <= 1)
        {
            resetLine();
            return Event::NONE;
        }
        line_[line_len_] = '\0';
        const Event ev = parseLine();
        resetLine();
        return ev;
    }

    if (line_len_ >= kMaxLine)
    {
        overlong_lines_++;
        resetLine();
        return Event::BAD;
    }

    line_[line_len_++] = c;
    return Event::NONE;
}

Event Lc86Parser::parseLine()
{
    // line_ = "$<payload>*HH", NUL-terminated, CR/LF already stripped.
    if (line_len_ < 5 || line_[line_len_ - 3] != '*')
    {
        bad_lines_++;
        return Event::BAD;
    }
    const int hi = hexVal(line_[line_len_ - 2]);
    const int lo = hexVal(line_[line_len_ - 1]);
    if (hi < 0 || lo < 0)
    {
        bad_lines_++;
        return Event::BAD;
    }

    const size_t payload_len = line_len_ - 4;  // span between '$' and '*'
    const uint8_t want = (uint8_t)((hi << 4) | lo);
    if (checksum(line_ + 1, payload_len) != want)
    {
        bad_lines_++;
        return Event::BAD;
    }

    // Tokenize the payload in place (',' → NUL). Empty fields stay as valid
    // zero-length strings.
    line_[1 + payload_len] = '\0';  // cut off "*HH"
    const char* fields[kMaxFields];
    size_t nfields = 0;
    char* p = line_ + 1;
    fields[nfields++] = p;
    while (*p != '\0' && nfields < kMaxFields)
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[nfields++] = p + 1;
        }
        p++;
    }

    const char* addr = fields[0];
    if (strcmp(addr, "PQTMPVT") == 0)        return parsePqtmPvt(fields, nfields);
    if (strcmp(addr, "PQTMEPE") == 0)        return parsePqtmEpe(fields, nfields);
    if (strcmp(addr, "PAIR001") == 0)        return parsePairAck(fields, nfields);
    if (strcmp(addr, "PQTMCFGMSGRATE") == 0) return parseQtmCfgMsgRate(fields, nfields);
    if (strlen(addr) == 5 && strcmp(addr + 2, "GGA") == 0)
    {
        return parseGga(fields, nfields);  // GPGGA/GNGGA/GAGGA/GBGGA...
    }
    return Event::OTHER_VALID;
}

// ── Sentence parsers ────────────────────────────────────────────────────

Event Lc86Parser::parsePqtmPvt(const char* const* f, size_t n)
{
    // $PQTMPVT,<MsgVer=1>,<TOW>,<Date>,<Time>,<Quality>,<FixMode>,
    //   <NumSatUsed>,<LeapS>,<Lat>,<Lon>,<Alt>,<Sep>,<VelN>,<VelE>,<VelD>,
    //   <Spd>,<Heading>,<HDOP>,<PDOP>          (protocol spec §2.3.8)
    // 19 data fields; MsgVer is documented "always 1" — an unknown version
    // means the field positions can't be trusted, so reject the line rather
    // than guess.
    if (n < 20 || strcmp(f[1], "1") != 0)
    {
        bad_lines_++;
        return Event::BAD;
    }

    GNSSData d = {};  // time_us stays 0 — the owner stamps MCU time

    const long date = fieldLong(f[3]);  // YYYYMMDD
    d.year  = (uint16_t)(date / 10000);
    d.month = (uint8_t)((date / 100) % 100);
    d.day   = (uint8_t)(date % 100);
    // Locals, not d.* directly: GNSSData is packed, and Xtensa GCC rejects
    // binding a reference to a packed uint16_t field.
    uint8_t hh = 0, mm = 0, ss = 0;
    uint16_t ms = 0;
    parseHmsMs(f[4], hh, mm, ss, ms);
    d.hour = hh;
    d.minute = mm;
    d.second = ss;
    d.milli_second = ms;

    const long quality  = fieldLong(f[5]);
    const long fix_mode = fieldLong(f[6]);
    uint8_t fm = (fix_mode >= 0 && fix_mode <= 5) ? (uint8_t)fix_mode : 0;
    // Trust gate (the #562 gnssFixOK analogue): <Quality> is the GGA-style
    // fix indicator and is the LC86G's own word on solution validity —
    // 0 = invalid regardless of FixMode. Zero fix_mode so every downstream
    // `fix_mode >= 2/3` consumer treats the epoch as no-fix.
    if (quality == 0) fm = 0;
    d.fix_mode = fm;

    const long sats = fieldLong(f[7]);
    d.num_sats = (uint8_t)((sats < 0) ? 0 : ((sats > 255) ? 255 : sats));

    d.lat_e7 = scaleI32(fieldDouble(f[9]), 1e7);
    d.lon_e7 = scaleI32(fieldDouble(f[10]), 1e7);
    d.alt_mm = scaleI32(fieldDouble(f[11]), 1000.0);  // <Alt> is MSL, m → mm

    d.vel_n_mmps = scaleI32(fieldDouble(f[13]), 1000.0);
    d.vel_e_mmps = scaleI32(fieldDouble(f[14]), 1000.0);
    // NED→ENU vertical: vel_u = -VelD. THE one sign in this file that draws
    // blood if flipped — the EKF hard-fuses velD (sigma 0.3 m/s), so a wrong
    // sign reads a climbing rocket as diving at twice the rate.
    d.vel_u_mmps = scaleI32(-fieldDouble(f[15]), 1000.0);

    // PDOP ×10, saturating; the documented "invalid" marker 99.99 → 999.9
    // saturates to 255 naturally.
    d.pdop_x10 = satRoundU8(fieldDouble(f[19]) * 10.0);

    // Merge the latest $PQTMEPE accuracies — PQTMPVT itself carries none.
    d.h_acc_m = epe_h_acc_m_;
    d.v_acc_m = epe_v_acc_m_;

    pvt_ = d;
    return Event::PVT;
}

Event Lc86Parser::parsePqtmEpe(const char* const* f, size_t n)
{
    // $PQTMEPE,<MsgVer=2>,<EPE_North>,<EPE_East>,<EPE_Down>,<EPE_2D>,<EPE_3D>
    // Metres. Cached only; published merged into the NEXT PVT so the pair
    // always travels together (EPE carries no epoch marker of its own — a
    // stale cache going slightly stale is fine, accuracy drifts slowly).
    if (n < 7)
    {
        bad_lines_++;
        return Event::BAD;
    }
    // Any MsgVer accepted: the fields are positional and begin() controls
    // which version was enabled (2 per the doc, 1 as its fallback).
    //
    // Validity guard: a fixless module emits empty (or all-zero) numeric
    // fields, and fieldDouble maps empty to 0.0 — caching that would
    // advertise a perfect 0 m accuracy estimate with no fix at all. Only a
    // strictly positive estimate updates the cache; anything else leaves
    // the previous value (or the 255 "unknown" the cache starts at).
    const double epe_2d   = fieldDouble(f[5]);
    const double epe_down = fieldDouble(f[4]);
    if (f[5][0] != '\0' && epe_2d > 0.0)
    {
        epe_h_acc_m_ = satRoundU8(epe_2d);    // EPE_2D   → h_acc_m
    }
    if (f[4][0] != '\0' && epe_down > 0.0)
    {
        epe_v_acc_m_ = satRoundU8(epe_down);  // EPE_Down → v_acc_m
    }
    return Event::EPE;
}

Event Lc86Parser::parsePairAck(const char* const* f, size_t n)
{
    // $PAIR001,<CommandID>,<Result>  (protocol spec §2.4.1)
    if (n < 3)
    {
        bad_lines_++;
        return Event::BAD;
    }
    const long cmd = fieldLong(f[1]);
    const long res = fieldLong(f[2]);
    ack_cmd_    = (uint16_t)((cmd < 0 || cmd > 999) ? 0 : cmd);
    ack_result_ = (uint8_t)((res < 0 || res > 255) ? 255 : res);
    return Event::PAIR_ACK;
}

Event Lc86Parser::parseQtmCfgMsgRate(const char* const* f, size_t n)
{
    // Write acks: $PQTMCFGMSGRATE,OK / $PQTMCFGMSGRATE,ERROR,<ErrCode>.
    // (Read responses carry the message name instead — we never send R.)
    if (n >= 2 && strcmp(f[1], "OK") == 0) return Event::QTM_OK;
    if (n >= 2 && strcmp(f[1], "ERROR") == 0)
    {
        qtm_err_ = (n >= 3) ? (int)fieldLong(f[2]) : -1;
        return Event::QTM_ERROR;
    }
    return Event::OTHER_VALID;
}

Event Lc86Parser::parseGga(const char* const* f, size_t n)
{
    // $xxGGA,<Time>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,<HDOP>,...
    // Liveness/debug fields only — never a fix source (no vertical velocity).
    if (n < 8) return Event::OTHER_VALID;
    const long q = fieldLong(f[6]);
    const long s = fieldLong(f[7]);
    gga_quality_  = (uint8_t)((q < 0 || q > 8) ? 0 : q);
    gga_num_sats_ = (uint8_t)((s < 0) ? 0 : ((s > 255) ? 255 : s));
    return Event::GGA;
}

}  // namespace lc86
