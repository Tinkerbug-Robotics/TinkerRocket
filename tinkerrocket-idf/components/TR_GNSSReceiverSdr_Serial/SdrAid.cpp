// SdrAid.cpp — see SdrAid.h. Pure: no ESP-IDF, host-tested. Built with
// -ffp-contract=off, here and in tests_cpp: a fused multiply-add would round
// differently from the board's codec and the reference one.
#include "SdrAid.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "Rtcm3Msm.h"   // TR_GNSSReceiverLC86_Serial: rtcm3::crc24q
#include "SdrRtcm.h"

namespace sdr_aid
{

namespace
{

constexpr int64_t kWeekMs = 604800000;

// `n` bits (at most 64) of `v` at bit `pos` of `p`, most significant first,
// as RTCM 3 packs them.
void put(uint8_t* p, size_t pos, int n, uint64_t v)
{
    for (int i = n - 1; i >= 0; i--, pos++)
    {
        const uint8_t m = static_cast<uint8_t>(0x80U >> (pos % 8));
        if ((v >> i) & 1U)
            p[pos / 8] |= m;
        else
            p[pos / 8] &= static_cast<uint8_t>(~m);
    }
}

// A signed field: its two's complement in n bits.
void putS(uint8_t* p, size_t pos, int n, int64_t v)
{
    put(p, pos, n, static_cast<uint64_t>(v) & ((uint64_t{1} << n) - 1));
}

uint64_t get(const uint8_t* p, size_t pos, int n)
{
    uint64_t v = 0;
    for (int i = 0; i < n; i++, pos++) v = (v << 1) | ((p[pos / 8] >> (7 - pos % 8)) & 1U);
    return v;
}

int64_t getS(const uint8_t* p, size_t pos, int n)
{
    const uint64_t v = get(p, pos, n);
    return (v >> (n - 1)) & 1U ? static_cast<int64_t>(v) - (int64_t{1} << n) : static_cast<int64_t>(v);
}

// The board's clamp_i (receiver/core/nav/rtcm_msm.c): saturated at
// +-(2^(n-1) - 1), else rounded to nearest, halves away from zero. Never
// handed a NaN: a group with one goes out as not valid.
int64_t clampI(double v, int n)
{
    const double lim = static_cast<double>((int64_t{1} << (n - 1)) - 1);
    if (v > lim) return static_cast<int64_t>(lim);
    if (v < -lim) return -static_cast<int64_t>(lim);
    return static_cast<int64_t>(std::llround(v));
}

// A 1-sigma already times its scale, rounded up to its LSB, forgiving a
// thousandth of one and never 0; all ones when unknown or past the field.
uint64_t sigCode(double x, int n)
{
    const double top = static_cast<double>((uint64_t{1} << n) - 1);
    if (!(x >= 0.0) || std::isinf(x)) return static_cast<uint64_t>(top);
    double c = std::ceil(x - 1e-3);
    if (c < 1.0) c = 1.0;
    return static_cast<uint64_t>(c > top ? top : c);
}

// A sigma's code back in its units; all ones reads unknown.
double sigValue(uint64_t code, int n, double lsb)
{
    return code == (uint64_t{1} << n) - 1 ? kUnknown : static_cast<double>(code) * lsb;
}

uint64_t usat(uint64_t v, int n)
{
    const uint64_t top = (uint64_t{1} << n) - 1;
    return v > top ? top : v;
}

bool noNaN(const double* v, int n)
{
    for (int k = 0; k < n; k++)
        if (std::isnan(v[k])) return false;
    return true;
}

void putHeader(uint8_t* m, uint8_t sub, const Header& h)
{
    put(m, 0, 12, kFcMessage);
    put(m, 12, 4, sub);
    put(m, 16, 16, h.week);
    put(m, 32, 30, h.tow_ms & 0x3FFFFFFFU);
    putS(m, 62, 20, clampI(static_cast<double>(h.sub_ns), 20));
    put(m, 82, 8, h.seq);
    put(m, 90, 2, h.src & 3U);
    put(m, 92, 1, h.test ? 1 : 0);
    put(m, 93, 3, h.tq & 7U);
}

Header getHeader(const uint8_t* m)
{
    Header h;
    h.week = static_cast<uint16_t>(get(m, 16, 16));
    h.tow_ms = static_cast<uint32_t>(get(m, 32, 30));
    h.sub_ns = static_cast<int32_t>(getS(m, 62, 20));
    h.seq = static_cast<uint8_t>(get(m, 82, 8));
    h.src = static_cast<uint8_t>(get(m, 90, 2));
    h.test = get(m, 92, 1) != 0;
    h.tq = static_cast<uint8_t>(get(m, 93, 3));
    return h;
}

// The frame around the message already at out + 3: 0xD3, six zero bits and
// the 10-bit length, the CRC-24Q after it.
size_t close(uint8_t* out, size_t bytes)
{
    out[0] = rtcm3::kPreamble;
    out[1] = static_cast<uint8_t>((bytes >> 8) & 0x03);
    out[2] = static_cast<uint8_t>(bytes & 0xFF);
    const uint32_t crc = rtcm3::crc24q(out, bytes + 3);
    out[bytes + 3] = static_cast<uint8_t>(crc >> 16);
    out[bytes + 4] = static_cast<uint8_t>(crc >> 8);
    out[bytes + 5] = static_cast<uint8_t>(crc);
    return bytes + 6;
}

// The message of a 4002 frame of sub-type `sub` long enough for its bytes.
const uint8_t* fcPayload(const uint8_t* frame, size_t len, uint8_t sub, size_t bytes)
{
    if (fcSubType(frame, len) != sub || len - 6 < bytes) return nullptr;
    return frame + 3;
}

// ── The script's constants: literals, used as they are on both sides (none
// is recomputed). The RF rig scene's pad at 0 N, 119 W, 1200 m (WGS-84), its
// east, and the attitude at roll 0: the pad's NED to ECEF, then yaw 30 deg
// and pitch 80 deg (body FRD to NED).
constexpr double kR2 = 0.7071067811865476;
constexpr double kPad[3] = {-3092763.9484, -5579493.8587, 0.0};
constexpr double kEast[3] = {0.8746197071393959, -0.484809620246337, 0.0};
constexpr double kWRoll = 0.10471975511965977;          // rad/s: one roll a minute
constexpr double kWRollPerMs = 1.0471975511965977e-4;   // rad/ms
constexpr double kNavWPerMs = 6.283185307179586e-4;     // rad/ms: once round in 10 s
constexpr double kNavVAmp = 1.2566370614359172;         // m/s: 2 m x 2 pi / 10 s
constexpr double kQ0[4] = {0.507815141369057, -0.20339782543985335, 0.17942981533211608, -0.8176539904950908};
constexpr int64_t kSceneIgnMs = 204000100;              // the hotshot scene's ignition, GPS week 2432

struct PhaseAt
{
    uint8_t  phase, loops;
    bool     canopy;
    uint32_t since_ms;
};

// The hotshot scene's timeline from two minutes before its ignition to five
// after (the V10's launch flag 60 ms late; burnout at 3.9 s, seen 0.25 s
// later; the drogue at 73.1 s, a canopy from 80 s); a 120 s cycle through
// every phase anywhere else (the real sky on the bench).
PhaseAt scriptPhaseAt(uint32_t tow_ms)
{
    int64_t d = static_cast<int64_t>(tow_ms) - kSceneIgnMs;
    if (d < -kWeekMs / 2)
        d += kWeekMs;
    else if (d >= kWeekMs / 2)
        d -= kWeekMs;
    if (d >= -120000 && d < 300000)
    {
        if (d < 60) return {kPhPad, kLoopsNone, false, 65535};
        if (d < 4150) return {kPhBoost, kLoopsBoost, false, static_cast<uint32_t>(d - 60)};
        if (d < 73100) return {kPhCoast, kLoopsQuiet, false, static_cast<uint32_t>(std::min<int64_t>(d - 4150, 65535))};
        return {kPhDescent, kLoopsNone, d >= 80000, static_cast<uint32_t>(std::min<int64_t>(d - 73100, 65535))};
    }
    const uint32_t c = tow_ms % 120000;
    if (c < 20000) return {kPhPad, kLoopsNone, false, 65535};
    if (c < 24000) return {kPhBoost, kLoopsBoost, false, c - 20000};
    if (c < 60000) return {kPhCoast, kLoopsQuiet, false, c - 24000};
    if (c < 110000) return {kPhDescent, kLoopsNone, c >= 70000, c - 60000};
    return {kPhLanded, kLoopsNone, false, c - 110000};
}

}  // namespace

// ── Frames ──────────────────────────────────────────────────────────────

size_t encode(const Attitude& a, uint8_t* out, size_t cap)
{
    if (out == nullptr || cap < kAttitudeFrame) return 0;
    std::memset(out, 0, kAttitudeFrame);
    uint8_t* m = out + 3;
    putHeader(m, kAttitude, a.h);
    const bool av = a.att_valid && noNaN(a.q, 4);
    const bool rv = a.rate_valid && noNaN(a.w, 3);
    put(m, 96, 1, av ? 1 : 0);
    put(m, 97, 1, rv ? 1 : 0);
    put(m, 98, 2, a.frame & 3U);
    const bool neg = av && a.q[0] < 0.0;   // q and -q are one rotation: q0 >= 0 on the wire
    for (int k = 0; k < 4; k++) putS(m, 100 + 16 * k, 16, av ? clampI((neg ? -a.q[k] : a.q[k]) * 16384.0, 16) : 0);
    for (int k = 0; k < 3; k++) putS(m, 164 + 18 * k, 18, rv ? clampI(a.w[k] * 1024.0, 18) : 0);
    put(m, 218, 8, av ? sigCode(a.sig_tilt_deg * 10.0, 8) : 0);
    put(m, 226, 8, av ? sigCode(a.sig_hdg_deg * 2.0, 8) : 0);
    return close(out, kAttitudeBytes);
}

size_t encode(const NavState& a, uint8_t* out, size_t cap)
{
    if (out == nullptr || cap < kNavStateFrame) return 0;
    std::memset(out, 0, kNavStateFrame);
    uint8_t* m = out + 3;
    putHeader(m, kNavState, a.h);
    const bool pv = a.pos_valid && noNaN(a.pos, 3);
    const bool vv = a.vel_valid && noNaN(a.vel, 3);
    put(m, 96, 1, pv ? 1 : 0);
    put(m, 97, 1, vv ? 1 : 0);
    put(m, 98, 2, a.point & 3U);
    for (int k = 0; k < 3; k++) putS(m, 100 + 38 * k, 38, pv ? clampI(a.pos[k] * 1e4, 38) : 0);   // 0.1 mm
    for (int k = 0; k < 3; k++) putS(m, 214 + 28 * k, 28, vv ? clampI(a.vel[k] * 1e4, 28) : 0);   // 0.1 mm/s
    put(m, 298, 16, pv ? sigCode(a.sig[0] * 100.0, 16) : 0);    // cm
    put(m, 314, 16, pv ? sigCode(a.sig[1] * 100.0, 16) : 0);
    put(m, 330, 16, vv ? sigCode(a.sig[2] * 1000.0, 16) : 0);   // mm/s
    put(m, 346, 16, vv ? sigCode(a.sig[3] * 1000.0, 16) : 0);
    return close(out, kNavStateBytes);
}

size_t encode(const PhaseMsg& a, uint8_t* out, size_t cap)
{
    if (out == nullptr || cap < kPhaseFrame) return 0;
    std::memset(out, 0, kPhaseFrame);
    uint8_t* m = out + 3;
    putHeader(m, kPhase, a.h);
    put(m, 96, 3, a.phase & 7U);
    put(m, 99, 2, a.loops & 3U);
    put(m, 101, 1, a.canopy ? 1 : 0);
    put(m, 102, 16, usat(a.since_ms, 16));
    put(m, 118, 8, usat((uint64_t{a.hold_ms} + 10) / 20, 8));   // 20 ms steps, to the nearest
    return close(out, kPhaseBytes);
}

size_t encode(const Geometry& a, uint8_t* out, size_t cap)
{
    if (out == nullptr || cap < kGeometryFrame) return 0;
    std::memset(out, 0, kGeometryFrame);
    uint8_t* m = out + 3;
    putHeader(m, kGeometry, a.h);
    const bool lv = a.lever_valid && noNaN(a.lever_m, 3);
    const bool rv = a.rot_valid && noNaN(a.rot, 4);
    put(m, 96, 1, lv ? 1 : 0);
    put(m, 97, 1, rv ? 1 : 0);
    put(m, 98, 4, a.ant_type & 15U);
    put(m, 102, 3, a.ant_axis & 7U);
    for (int k = 0; k < 3; k++) putS(m, 112 + 16 * k, 16, lv ? clampI(a.lever_m[k] * 1000.0, 16) : 0);   // mm
    const bool neg = rv && a.rot[0] < 0.0;
    for (int k = 0; k < 4; k++) putS(m, 160 + 16 * k, 16, rv ? clampI((neg ? -a.rot[k] : a.rot[k]) * 16384.0, 16) : 0);
    return close(out, kGeometryBytes);
}

size_t encode(const Rest& a, uint8_t* out, size_t cap)
{
    if (out == nullptr || cap < kRestFrame) return 0;
    std::memset(out, 0, kRestFrame);
    uint8_t* m = out + 3;
    putHeader(m, kRest, a.h);
    const bool v = a.valid && noNaN(a.f, 3);
    put(m, 96, 1, v ? 1 : 0);
    put(m, 97, 1, a.frame & 1U);                                                               // 2 bits reserved
    put(m, 100, 12, usat(a.dur_ms, 12));
    for (int k = 0; k < 3; k++) putS(m, 112 + 20 * k, 20, v ? clampI(a.f[k] * 10000.0, 20) : 0);   // 1e-4 m/s^2
    put(m, 172, 8, sigCode(a.sd_f * 1000.0, 8));   // 1e-3 m/s^2, valid or not; 4 bits reserved
    return close(out, kRestBytes);
}

uint8_t fcSubType(const uint8_t* frame, size_t len)
{
    return sdr_rtcm::messageNumber(frame, len) == kFcMessage ? static_cast<uint8_t>(frame[4] & 0x0F) : 0;
}

bool decode(const uint8_t* frame, size_t len, Attitude& out)
{
    const uint8_t* m = fcPayload(frame, len, kAttitude, kAttitudeBytes);
    if (m == nullptr) return false;
    out.h = getHeader(m);
    out.att_valid = get(m, 96, 1) != 0;
    out.rate_valid = get(m, 97, 1) != 0;
    out.frame = static_cast<uint8_t>(get(m, 98, 2));
    for (int k = 0; k < 4; k++) out.q[k] = static_cast<double>(getS(m, 100 + 16 * k, 16)) / 16384.0;
    for (int k = 0; k < 3; k++) out.w[k] = static_cast<double>(getS(m, 164 + 18 * k, 18)) / 1024.0;
    out.sig_tilt_deg = sigValue(get(m, 218, 8), 8, 0.1);
    out.sig_hdg_deg = sigValue(get(m, 226, 8), 8, 0.5);
    return true;
}

bool decode(const uint8_t* frame, size_t len, NavState& out)
{
    const uint8_t* m = fcPayload(frame, len, kNavState, kNavStateBytes);
    if (m == nullptr) return false;
    out.h = getHeader(m);
    out.pos_valid = get(m, 96, 1) != 0;
    out.vel_valid = get(m, 97, 1) != 0;
    out.point = static_cast<uint8_t>(get(m, 98, 2));
    for (int k = 0; k < 3; k++) out.pos[k] = static_cast<double>(getS(m, 100 + 38 * k, 38)) / 1e4;
    for (int k = 0; k < 3; k++) out.vel[k] = static_cast<double>(getS(m, 214 + 28 * k, 28)) / 1e4;
    out.sig[0] = sigValue(get(m, 298, 16), 16, 0.01);
    out.sig[1] = sigValue(get(m, 314, 16), 16, 0.01);
    out.sig[2] = sigValue(get(m, 330, 16), 16, 0.001);
    out.sig[3] = sigValue(get(m, 346, 16), 16, 0.001);
    return true;
}

bool decode(const uint8_t* frame, size_t len, PhaseMsg& out)
{
    const uint8_t* m = fcPayload(frame, len, kPhase, kPhaseBytes);
    if (m == nullptr) return false;
    out.h = getHeader(m);
    out.phase = static_cast<uint8_t>(get(m, 96, 3));
    out.loops = static_cast<uint8_t>(get(m, 99, 2));
    out.canopy = get(m, 101, 1) != 0;
    out.since_ms = static_cast<uint32_t>(get(m, 102, 16));
    out.hold_ms = static_cast<uint32_t>(get(m, 118, 8)) * 20U;
    return true;
}

bool decode(const uint8_t* frame, size_t len, Geometry& out)
{
    const uint8_t* m = fcPayload(frame, len, kGeometry, kGeometryBytes);
    if (m == nullptr) return false;
    out.h = getHeader(m);
    out.lever_valid = get(m, 96, 1) != 0;
    out.rot_valid = get(m, 97, 1) != 0;
    out.ant_type = static_cast<uint8_t>(get(m, 98, 4));
    out.ant_axis = static_cast<uint8_t>(get(m, 102, 3));
    for (int k = 0; k < 3; k++) out.lever_m[k] = static_cast<double>(getS(m, 112 + 16 * k, 16)) / 1000.0;
    for (int k = 0; k < 4; k++) out.rot[k] = static_cast<double>(getS(m, 160 + 16 * k, 16)) / 16384.0;
    return true;
}

bool decode(const uint8_t* frame, size_t len, Rest& out)
{
    const uint8_t* m = fcPayload(frame, len, kRest, kRestBytes);
    if (m == nullptr) return false;
    out.h = getHeader(m);
    out.valid = get(m, 96, 1) != 0;
    out.frame = static_cast<uint8_t>(get(m, 97, 1));
    out.dur_ms = static_cast<uint16_t>(get(m, 100, 12));
    for (int k = 0; k < 3; k++) out.f[k] = static_cast<double>(getS(m, 112 + 20 * k, 20)) * 1e-4;
    out.sd_f = sigValue(get(m, 172, 8), 8, 1e-3);
    return true;
}

Header stampAt(int week, double tow_s)
{
    int64_t ms = std::llround(tow_s * 1000.0);
    if (ms >= kWeekMs)
    {
        week++;
        ms -= kWeekMs;
        tow_s -= 604800.0;
    }
    Header h;
    h.week = static_cast<uint16_t>(week);
    h.tow_ms = static_cast<uint32_t>(ms);
    h.sub_ns = static_cast<int32_t>(std::llround((tow_s - static_cast<double>(ms) * 1e-3) * 1e9));
    return h;
}

Header gpsStamp(double gps_s)
{
    const int week = static_cast<int>(std::floor(gps_s / 604800.0));
    return stampAt(week, gps_s - static_cast<double>(week) * 604800.0);
}

uint32_t phaseKey(const PhaseMsg& p)
{
    return static_cast<uint32_t>(p.phase & 7U) | static_cast<uint32_t>(p.loops & 3U) << 3 |
           static_cast<uint32_t>(p.canopy ? 1 : 0) << 5 | static_cast<uint32_t>(p.h.test ? 1 : 0) << 6;
}

// ── The script ──────────────────────────────────────────────────────────

Header scriptStamp(double gps_s)
{
    Header h = gpsStamp(gps_s);
    h.sub_ns = 0;
    h.src = kSrcScript;
    h.test = true;
    h.tq = kScriptTq;
    return h;
}

Attitude scriptAttitude(const Header& h)
{
    // A roll about the body's x axis, -60 to +60 deg and back every 20 s: Q0
    // times the roll's quaternion. Yaw 30, pitch 80 at the pad throughout.
    const int64_t k = static_cast<int64_t>(h.tow_ms % 20000) - 10000;
    const double phi = kWRollPerMs * static_cast<double>(k);
    const double half = 0.5 * phi;
    const double c = std::cos(half);
    const double s = std::sin(half);
    Attitude a;
    a.h = h;
    a.att_valid = true;
    a.rate_valid = true;
    a.frame = 0;
    a.q[0] = kQ0[0] * c - kQ0[1] * s;
    a.q[1] = kQ0[0] * s + kQ0[1] * c;
    a.q[2] = kQ0[2] * c + kQ0[3] * s;
    a.q[3] = kQ0[3] * c - kQ0[2] * s;
    a.w[0] = kWRoll;
    a.w[1] = 0.0;
    a.w[2] = 0.0;
    a.sig_tilt_deg = 0.5;
    a.sig_hdg_deg = 30.0;   // the flight computer's heading seed without GNSS
    return a;
}

NavState scriptNav(const Header& h)
{
    // The pad swinging 2 m east and west every 10 s; the velocity invalid 10 s
    // of every minute.
    const double u = kNavWPerMs * static_cast<double>(h.tow_ms % 10000);
    const double su = std::sin(u);
    const double cu = std::cos(u);
    const bool vv = h.tow_ms % 60000 < 50000;
    NavState n;
    n.h = h;
    n.pos_valid = true;
    n.vel_valid = vv;
    n.point = 0;
    for (int j = 0; j < 3; j++)
    {
        n.pos[j] = kPad[j] + (2.0 * su) * kEast[j];
        n.vel[j] = vv ? (kNavVAmp * cu) * kEast[j] : 0.0;
    }
    n.sig[0] = 3.0;
    n.sig[1] = 5.0;
    n.sig[2] = 0.2;
    n.sig[3] = 0.3;
    return n;
}

PhaseMsg scriptPhase(const Header& h)
{
    const PhaseAt at = scriptPhaseAt(h.tow_ms);
    PhaseMsg p;
    p.h = h;
    p.phase = at.phase;
    p.loops = at.loops;
    p.canopy = at.canopy;
    p.since_ms = at.since_ms;
    p.hold_ms = kScriptHoldMs;
    return p;
}

Geometry scriptGeometry(const Header& h)
{
    // A patch along +z, its lever arm, the board turned 90 deg about +z.
    Geometry g;
    g.h = h;
    g.lever_valid = true;
    g.rot_valid = true;
    g.ant_type = 1;
    g.ant_axis = 5;
    g.lever_m[0] = 0.012;
    g.lever_m[1] = -0.034;
    g.lever_m[2] = 0.156;
    g.rot[0] = kR2;
    g.rot[1] = 0.0;
    g.rot[2] = 0.0;
    g.rot[3] = kR2;
    return g;
}

uint32_t scriptPhaseKey(uint32_t tow_ms)
{
    const PhaseAt at = scriptPhaseAt(tow_ms);
    return static_cast<uint32_t>(at.phase) | static_cast<uint32_t>(at.loops) << 3 |
           static_cast<uint32_t>(at.canopy ? 1 : 0) << 5;
}

// ── The board's aiding block ────────────────────────────────────────────

bool decodeEpochAid(const uint8_t* frame, size_t len, EpochAid& out)
{
    // As decodeTrEpoch: number, sub-type 1 and the 202 bits through the sample.
    out = EpochAid{};
    if (sdr_rtcm::trSubType(frame, len) != sdr_rtcm::kTrEpoch || (len - 6) * 8 < 202) return false;
    const uint8_t* m = frame + 3;
    const size_t bits = (len - 6) * 8;
    out.shift_ns = static_cast<int32_t>(getS(m, 62, 20));
    // The block sits after the marks the mark count says (not the marks a
    // reader takes: the board's own decoder stops at 32), and only a frame
    // that holds all its 32 bits has one: an older frame's padding after its
    // marks is at most 7 zero bits.
    if (bits < 214) return true;
    const size_t at = 214 + 10 * static_cast<size_t>(get(m, 208, 6));
    if (bits < at + 32 || get(m, at, 4) == 0) return true;
    out.present = true;
    out.ver = static_cast<uint8_t>(get(m, at, 4));
    out.src = static_cast<uint8_t>(get(m, at + 4, 2));
    out.check = static_cast<uint8_t>(get(m, at + 6, 2));
    out.boost = get(m, at + 8, 1) != 0;
    out.wide = get(m, at + 9, 1) != 0;
    out.by = static_cast<uint8_t>(get(m, at + 10, 2));
    out.planned = get(m, at + 12, 1) != 0;
    out.phase = static_cast<uint8_t>(get(m, at + 13, 3));
    out.att = get(m, at + 16, 1) != 0;
    out.nav = get(m, at + 17, 1) != 0;
    out.late = get(m, at + 18, 1) != 0;
    out.fc_frames = static_cast<uint8_t>(get(m, at + 24, 8));
    return true;
}

bool latchGps(const sdr_rtcm::TrEpoch& e, const EpochAid& aid, double& gps_s)
{
    if (aid.present && aid.late) return false;
    gps_s = e.week * 604800.0 + e.tow_ms * 1e-3 - aid.shift_ns * 1e-9;
    return true;
}

// ── EchoCount ───────────────────────────────────────────────────────────

void EchoCount::add(uint8_t fc_frames, uint32_t sent, int64_t at_us)
{
    const uint32_t out = sent - sent_;                                     // sent since the last block
    const uint32_t took = static_cast<uint8_t>(fc_frames - fc_frames_);   // the board's count since, mod 256
    if (have_ && at_us - at_us_ <= kGapUs && out + kSlack <= 0xFF && took <= out + kSlack)
    {
        took_ += took;
        of_ += out;
    }
    else
    {
        aside_ += out;
    }
    have_ = true;
    fc_frames_ = fc_frames;
    sent_ = sent;
    at_us_ = at_us;
}

// ── AidSchedule ─────────────────────────────────────────────────────────

namespace
{

constexpr int64_t kAttitudeUs = 100000;
constexpr int64_t kNavStateUs = 200000;
constexpr int64_t kPhaseFastUs = 100000;     // after a change ...
constexpr int64_t kPhaseFastForUs = 300000;  // ... for this long: three repeats
constexpr int64_t kPhaseUs = 1000000;
constexpr int64_t kGeometryUs = 10000000;
constexpr int64_t kNavStateOffsetUs = 5000;  // each one's offset from the attitude's passes
constexpr int64_t kPhaseOffsetUs = 3000;
constexpr int64_t kGeometryOffsetUs = 7000;

}  // namespace

uint8_t AidSchedule::due(int64_t now_us, uint32_t phase_key)
{
    if (!started_)
    {
        // The phase's first frame is a change (no key sent yet), which
        // times its repeats.
        started_ = true;
        next_us_[kAttitude] = now_us;
        next_us_[kNavState] = now_us + kNavStateOffsetUs;
        next_us_[kGeometry] = now_us + kGeometryOffsetUs;
    }
    if (any_sent_ && now_us - last_us_ < kSpacingUs) return 0;
    // Only the sub-types this schedule gives (all four on the bench, the
    // phase alone in a flight image).
    if ((subs_ >> kPhase & 1U) != 0 && (!have_key_ || phase_key != key_)) return kPhase;
    for (uint8_t sub : {kAttitude, kNavState, kPhase, kGeometry})
        if ((subs_ >> sub & 1U) != 0 && now_us >= next_us_[sub]) return sub;
    return 0;
}

bool AidSchedule::anyDue(int64_t now_us)
{
    return due(now_us, key_) != 0;
}

void AidSchedule::sent(uint8_t sub, int64_t now_us, uint32_t phase_key)
{
    if (sub < kAttitude || sub > kGeometry) return;
    last_us_ = now_us;
    any_sent_ = true;
    if (sub == kPhase && (!have_key_ || phase_key != key_))
    {
        // A change: its repeats every 100 ms step from its pass, the phase's
        // offset after it.
        have_key_ = true;
        key_ = phase_key;
        repeat_us_ = now_us + kPhaseOffsetUs;
        next_us_[kPhase] = repeat_us_ + kPhaseFastUs;
        return;
    }
    int64_t period = kGeometryUs;
    if (sub == kAttitude)
        period = kAttitudeUs;
    else if (sub == kNavState)
        period = kNavStateUs;
    else if (sub == kPhase)
        period = next_us_[kPhase] - repeat_us_ < kPhaseFastForUs ? kPhaseFastUs : kPhaseUs;
    int64_t& next = next_us_[sub];
    next = now_us - next > period ? now_us + period : next + period;
}

}  // namespace sdr_aid
