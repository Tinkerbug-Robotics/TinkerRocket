// Host tests for finding which GNSS receiver is on the V9's J1
// (TR_Sensor_Collector/gnss_detect.cpp): the u-blox SAM-M10Q carrier or a
// Septentrio mosaic-G5, each in the states a boot finds it in, against a
// serial line simulated bit by bit. A receiver heard at the wrong baud then
// gives the host what a real UART gives it: the receiver's bytes cut up by
// start bits found at the wrong times.
//
// The mosaic's stream is recorded (fixtures/gnss_detect_sbf.h, from the
// 2026-10-15 spin-rig capture). There is no recording of the u-blox on this
// bench, so its streams are built here to its protocol: NMEA once a second
// from the factory, NAV-PVT and NAV-SAT at 18 Hz once a flight computer has
// set it up, and a MON-VER reply to the probe's poll.
#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "BoardIdentity.h"
#include "RocketComputerTypes.h"
#include "gnss_detect.h"
#include "fixtures/gnss_detect_sbf.h"

using namespace gnss_detect;

namespace {

// ── The line ────────────────────────────────────────────────────────────

// One direction of J1's serial line. 8N1 frames, idle high: a start bit (0),
// eight data bits LSB first, a stop bit (1).
class Line
{
public:
    // Bytes onto the wire at `baud`, back to back, from t_s or the end of
    // whatever is still going out.
    void send(const uint8_t* d, size_t n, uint32_t baud, double t_s)
    {
        const double bit = 1.0 / static_cast<double>(baud);
        double t = std::max(t_s, busy_until_);
        for (size_t i = 0; i < n; i++)
        {
            frames_.push_back({t, bit, static_cast<uint16_t>((d[i] << 1) | 0x200)});
            t += 10.0 * bit;
        }
        busy_until_ = t;
    }
    void send(const std::vector<uint8_t>& v, uint32_t baud, double t_s) { send(v.data(), v.size(), baud, t_s); }
    void send(const std::string& s, uint32_t baud, double t_s)
    {
        send(reinterpret_cast<const uint8_t*>(s.data()), s.size(), baud, t_s);
    }

    int level(double t) const
    {
        const size_t i = firstEndingAfter(t);
        if (i >= frames_.size() || frames_[i].t0 > t) return 1;
        const Frame& f = frames_[i];
        const int k = static_cast<int>((t - f.t0) / f.bit);
        return (k >= 0 && k < 10) ? ((f.bits >> k) & 1) : 1;
    }

    // The first falling edge at or after t; infinity when none.
    double nextFall(double t) const
    {
        for (size_t i = firstEndingAfter(t); i < frames_.size(); i++)
        {
            const Frame& f = frames_[i];
            if (f.t0 >= t) return f.t0;   // a start bit, after a stop bit or idle
            for (int k = 1; k < 10; k++)
            {
                const double edge = f.t0 + k * f.bit;
                if (edge < t) continue;
                if (((f.bits >> (k - 1)) & 1) == 1 && ((f.bits >> k) & 1) == 0) return edge;
            }
        }
        return std::numeric_limits<double>::infinity();
    }

private:
    struct Frame
    {
        double   t0;
        double   bit;
        uint16_t bits;
    };
    std::vector<Frame> frames_;
    double busy_until_ = 0.0;

    size_t firstEndingAfter(double t) const
    {
        size_t lo = 0, hi = frames_.size();
        while (lo < hi)
        {
            const size_t mid = (lo + hi) / 2;
            if (frames_[mid].t0 + 10.0 * frames_[mid].bit <= t)
                lo = mid + 1;
            else
                hi = mid;
        }
        return lo;
    }
};

// A UART listening to a Line at its own baud: a falling edge starts a byte if
// the line is still low half a bit later, each bit is sampled mid-bit, and a
// byte whose stop bit reads low is delivered all the same (a framing error),
// as the ESP32's UART delivers it.
class UartRx
{
public:
    void listen(uint32_t baud, double from_s)
    {
        baud_ = baud;
        search_ = from_s;
    }

    void pump(const Line& line, double now_s, std::vector<uint8_t>& out)
    {
        if (baud_ == 0) return;
        const double bit = 1.0 / static_cast<double>(baud_);
        for (;;)
        {
            const double ts = line.nextFall(search_);
            if (!(ts + 9.5 * bit <= now_s)) return;
            if (line.level(ts + 0.5 * bit) != 0)
            {
                search_ = ts + 0.5 * bit;   // a glitch, not a start bit
                continue;
            }
            uint8_t b = 0;
            for (int i = 0; i < 8; i++) b = static_cast<uint8_t>(b | (line.level(ts + (1.5 + i) * bit) << i));
            out.push_back(b);
            search_ = ts + 9.5 * bit;
        }
    }

private:
    uint32_t baud_ = 0;
    double   search_ = 0.0;
};

// ── The u-blox's messages ───────────────────────────────────────────────

std::vector<uint8_t> ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f = {0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF),
                              static_cast<uint8_t>(payload.size() >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < f.size(); i++)
    {
        a = static_cast<uint8_t>(a + f[i]);
        b = static_cast<uint8_t>(b + a);
    }
    f.push_back(a);
    f.push_back(b);
    return f;
}

std::string nmea(const std::string& body)
{
    uint8_t sum = 0;
    for (char c : body) sum = static_cast<uint8_t>(sum ^ static_cast<uint8_t>(c));
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
    return "$" + body + tail;
}

std::vector<uint8_t> field(const char* s, size_t width)
{
    std::vector<uint8_t> v(width, 0);
    std::memcpy(v.data(), s, std::min(width, std::strlen(s)));
    return v;
}

// MON-VER as a SAM-M10Q answers it: software and hardware versions, then
// the 30-byte extension strings.
std::vector<uint8_t> monVerReply()
{
    std::vector<uint8_t> p;
    for (const auto& part : {field("ROM SPG 5.10 (7b202e)", 30), field("000A0000", 10),
                             field("FWVER=SPG 5.10", 30), field("PROTVER=34.10", 30),
                             field("MOD=SAM-M10Q", 30), field("GPS;GLO;GAL;BDS", 30),
                             field("SBAS;QZSS", 30)})
    {
        p.insert(p.end(), part.begin(), part.end());
    }
    return ubx(0x0A, 0x04, p);
}

std::vector<uint8_t> navPvt(uint32_t itow_ms)
{
    std::vector<uint8_t> p(92, 0);
    std::memcpy(p.data(), &itow_ms, 4);
    p[20] = 3;      // fixType 3D
    p[23] = 14;     // numSV
    return ubx(0x01, 0x07, p);
}

std::vector<uint8_t> navSat(uint32_t itow_ms, uint8_t n)
{
    std::vector<uint8_t> p(8 + 12 * n, 0);
    std::memcpy(p.data(), &itow_ms, 4);
    p[4] = 1;
    p[5] = n;
    for (uint8_t i = 0; i < n; i++)
    {
        p[8 + 12 * i + 1] = static_cast<uint8_t>(i + 1);   // svId
        p[8 + 12 * i + 2] = 38;                             // C/N0
    }
    return ubx(0x01, 0x35, p);
}

// A factory SAM-M10Q's once-a-second NMEA, no fix yet.
std::string ubloxNmeaSecond()
{
    return nmea("GNRMC,,V,,,,,,,,,,N,V") + nmea("GNVTG,,,,,,,,,N") +
           nmea("GNGGA,,,,,,0,00,99.99,,,,,,") + nmea("GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1") +
           nmea("GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,3") + nmea("GPGSV,1,1,03,05,,,31,13,,,28,15,,,24,1") +
           nmea("GLGSV,1,1,00,1") + nmea("GAGSV,1,1,00,7") + nmea("GBGSV,1,1,00,1") +
           nmea("GNGLL,,,,,,V,N");
}

// ── The receivers ───────────────────────────────────────────────────────

double secs(uint32_t ms) { return static_cast<double>(ms) / 1000.0; }

struct Receiver
{
    virtual ~Receiver() = default;
    // Let 1 ms pass: hear what the host sent, answer, stream.
    virtual void tick(uint32_t now_ms, const Line& from_host, Line& to_host) = 0;
};

struct FakeUblox : Receiver
{
    uint32_t baud = 38400;
    uint32_t boots_at_ms = 0;
    bool     set_up = false;   // UBX at 18 Hz, as a flight computer leaves it; else factory NMEA at 1 Hz
    int      polls = 0;        // MON-VER polls it answered
    int      other_ubx = 0;    // UBX frames it would act on
    int      nmea_in = 0;      // NMEA sentences it would act on ($PUBX,41 sets its baud)

    void tick(uint32_t now_ms, const Line& from_host, Line& to_host) override
    {
        if (now_ms < boots_at_ms) return;
        const double t = secs(now_ms);
        if (!up_)
        {
            up_ = true;
            rx_.listen(baud, t);
            next_ms_ = now_ms + 300;   // the first output a little after it boots
        }
        std::vector<uint8_t> heard;
        rx_.pump(from_host, t, heard);
        for (uint8_t b : heard)
        {
            if (scan_.step(b))
            {
                if (scan_.lastClass() == 0x0A && scan_.lastId() == 0x04 && scan_.lastLength() == 0)
                {
                    polls++;
                    to_host.send(monVerReply(), baud, t + 0.005);
                }
                else
                {
                    other_ubx++;
                }
            }
            if (b == '\r' || b == '\n')
            {
                if (isNmeaSentence(line_.c_str())) nmea_in++;
                line_.clear();
            }
            else if (b >= 0x20 && b <= 0x7E && line_.size() < 120)
            {
                line_ += static_cast<char>(b);
            }
        }
        if (now_ms >= next_ms_)
        {
            if (set_up)
            {
                to_host.send(navPvt(now_ms), baud, t);
                to_host.send(navSat(now_ms, 24), baud, t);
                next_ms_ += 56;   // 18 Hz
            }
            else
            {
                to_host.send(ubloxNmeaSecond(), baud, t);
                next_ms_ += 1000;
            }
        }
    }

private:
    bool        up_ = false;
    UartRx      rx_;
    UbxScanner  scan_;
    std::string line_;
    uint32_t    next_ms_ = 0;
};

struct FakeMosaic : Receiver
{
    uint32_t baud = kMosaicTargetBaud;
    uint32_t boots_at_ms = 0;
    bool     set_up = true;   // streams the set of blocks saved into it; else silent until spoken to
    int      prompts = 0;     // empty commands it answered with its prompt
    std::vector<std::string> commands;   // anything else it took for a command

    void tick(uint32_t now_ms, const Line& from_host, Line& to_host) override
    {
        if (now_ms < boots_at_ms) return;
        const double t = secs(now_ms);
        if (!up_)
        {
            up_ = true;
            rx_.listen(baud, t);
            next_epoch_ms_ = now_ms;
            next_second_ms_ = now_ms;
        }
        std::vector<uint8_t> heard;
        rx_.pump(from_host, t, heard);
        for (uint8_t b : heard)
        {
            if (b == '\r')
            {
                if (cmd_.empty())
                {
                    prompts++;
                    to_host.send(std::string("\r\nCOM1>"), baud, t + 0.002);
                }
                else
                {
                    commands.push_back(cmd_);
                    to_host.send("$R? " + cmd_ + ": Invalid command!\r\n\r\nCOM1>", baud, t + 0.002);
                }
                cmd_.clear();
            }
            else if (b >= 0x20 && b <= 0x7E && cmd_.size() < 200)
            {
                cmd_ += static_cast<char>(b);
            }
        }
        if (!set_up) return;
        if (now_ms >= next_epoch_ms_)
        {
            to_host.send(kMosaicEpoch, sizeof(kMosaicEpoch), baud, t);
            next_epoch_ms_ += 50;
        }
        if (now_ms >= next_second_ms_)
        {
            to_host.send(kMosaicSecond, sizeof(kMosaicSecond), baud, t);
            next_second_ms_ += 1000;
        }
    }

private:
    bool        up_ = false;
    UartRx      rx_;
    std::string cmd_;
    uint32_t    next_epoch_ms_ = 0;
    uint32_t    next_second_ms_ = 0;
};

// ── Running a detection ─────────────────────────────────────────────────

struct Result
{
    uint8_t  found = GNSS_TYPE_NONE;
    Proof    proof = Proof::NONE;
    uint32_t baud = 0;
    uint32_t ms = 0;        // start to verdict, by the detector's clock
    uint32_t probes = 0;
    bool     done = false;
    size_t   heard = 0;     // bytes the host's UART delivered, all probes
};

// The receiver runs from t = 0; the detector starts at start_ms (a boot
// reaches its GNSS bring-up some seconds after power-up).
Result detect(Receiver* rx, const Plan& plan, uint32_t start_ms = 4000, uint32_t limit_ms = 60000)
{
    Detector d;
    Line to_rx, to_host;
    UartRx host;
    uint32_t host_baud = 0;
    bool started = false;
    size_t heard = 0;
    for (uint32_t now = 0; now < start_ms + limit_ms; now++)
    {
        const double t = secs(now);
        if (rx != nullptr) rx->tick(now, to_rx, to_host);
        if (now < start_ms) continue;
        if (!started)
        {
            d.start(plan, now);
            started = true;
        }
        std::vector<uint8_t> got;
        host.pump(to_host, t, got);
        heard += got.size();
        if (!got.empty()) d.feed(got.data(), got.size(), now);
        for (;;)
        {
            const Action a = d.poll(now);
            if (a.kind == Action::NONE) break;
            if (a.kind == Action::SET_BAUD)
            {
                host_baud = a.baud;
                host.listen(a.baud, t);   // what the UART held is dropped with the old baud
            }
            else
            {
                to_rx.send(a.data, a.len, host_baud, t);
            }
        }
        if (d.done()) break;
    }
    return {d.found(), d.proof(), d.foundBaud(), d.elapsedMs(), d.probesRun(), d.done(), heard};
}

}  // namespace

// ── The simulated line itself ───────────────────────────────────────────

std::vector<uint8_t> overTheLine(const std::vector<uint8_t>& sent, uint32_t tx_baud, uint32_t rx_baud)
{
    Line line;
    line.send(sent, tx_baud, 0.001);
    UartRx rx;
    rx.listen(rx_baud, 0.0);
    std::vector<uint8_t> got;
    rx.pump(line, 10.0, got);
    return got;
}

TEST(GnssDetectLine, TheSameBaudGivesTheBytesBack)
{
    const std::vector<uint8_t> epoch(kMosaicEpoch, kMosaicEpoch + sizeof(kMosaicEpoch));
    for (uint32_t b : {9600u, 38400u, 115200u, 460800u, 2000000u}) EXPECT_EQ(overTheLine(epoch, b, b), epoch) << b;
    const std::vector<uint8_t> pvt = navPvt(123456);
    EXPECT_EQ(overTheLine(pvt, 460800, 460800), pvt);
}

TEST(GnssDetectLine, AnotherBaudGivesNoise)
{
    const std::vector<uint8_t> epoch(kMosaicEpoch, kMosaicEpoch + sizeof(kMosaicEpoch));
    for (uint32_t b : {9600u, 38400u, 115200u, 460800u})
    {
        const std::vector<uint8_t> got = overTheLine(epoch, 2000000, b);
        EXPECT_FALSE(got.empty()) << b;
        EXPECT_NE(got, epoch) << b;
    }
    // A slow stream heard fast: long runs of one level, framing errors.
    const std::string s = ubloxNmeaSecond();
    const std::vector<uint8_t> nm(s.begin(), s.end());
    const std::vector<uint8_t> got = overTheLine(nm, 9600, 2000000);
    EXPECT_FALSE(got.empty());
    EXPECT_NE(got, nm);
    // A fast stream heard slow delivers fewer bytes than were sent, never more
    // than the slow baud can carry.
    const std::vector<uint8_t> slow = overTheLine(epoch, 2000000, 9600);
    EXPECT_LE(slow.size(), static_cast<size_t>(sizeof(kMosaicEpoch) * 10 / 2000000.0 * 9600 / 10 + 2));
}

// ── The receivers as a boot finds them ──────────────────────────────────

TEST(GnssDetect, FactoryUbloxAt38400)
{
    FakeUblox u;
    u.baud = 38400;
    const Result r = detect(&u, planFor(GNSS_TYPE_NONE, 0, false));
    EXPECT_EQ(r.found, GNSS_TYPE_UBLOX);
    EXPECT_EQ(r.proof, Proof::UBX_FRAME);
    EXPECT_EQ(r.baud, 38400u);
    EXPECT_LT(r.ms, 400u);   // the first probe: no hint, so 38400 leads
    EXPECT_EQ(u.polls, 1);
}

TEST(GnssDetect, FactoryUbloxAt9600WithItsHint)
{
    FakeUblox u;
    u.baud = 9600;
    const Result r = detect(&u, planFor(GNSS_TYPE_UBLOX, 9600, false));
    EXPECT_EQ(r.found, GNSS_TYPE_UBLOX);
    EXPECT_EQ(r.baud, 9600u);
    EXPECT_EQ(r.probes, 1u);   // where its driver found it last time
    EXPECT_LT(r.ms, ubloxListenMs(9600));
}

TEST(GnssDetect, SetUpUbloxAt460800AfterASoftwareReset)
{
    FakeUblox u;
    u.baud = 460800;
    u.set_up = true;
    const Result r = detect(&u, planFor(GNSS_TYPE_UBLOX, 9600, true));
    EXPECT_EQ(r.found, GNSS_TYPE_UBLOX);
    EXPECT_EQ(r.baud, 460800u);
    EXPECT_EQ(r.probes, 1u);
    EXPECT_LT(r.ms, 100u);
}

TEST(GnssDetect, SetUpUbloxOnABoardThatHadAMosaic)
{
    // The mosaic's probes come first and hear the u-blox's 460800 stream as
    // noise at 2 Mbaud and 115200: no mosaic proof in it.
    FakeUblox u;
    u.baud = 460800;
    u.set_up = true;
    const Result r = detect(&u, planFor(GNSS_TYPE_MOSAIC, 0, true));
    EXPECT_EQ(r.found, GNSS_TYPE_UBLOX);
    EXPECT_EQ(r.baud, 460800u);
    EXPECT_EQ(r.probes, 3u);
    EXPECT_EQ(u.other_ubx, 0);
    EXPECT_EQ(u.nmea_in, 0);
}

TEST(GnssDetect, FactoryUbloxOnABoardThatHadAMosaic)
{
    FakeUblox u;
    u.baud = 9600;
    const Result r = detect(&u, planFor(GNSS_TYPE_MOSAIC, 0, false));
    EXPECT_EQ(r.found, GNSS_TYPE_UBLOX);
    EXPECT_EQ(r.baud, 9600u);
}

TEST(GnssDetect, SetUpMosaicAt2MbaudWithItsHint)
{
    FakeMosaic m;
    const Result r = detect(&m, planFor(GNSS_TYPE_MOSAIC, 0, true));
    EXPECT_EQ(r.found, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(r.baud, kMosaicTargetBaud);
    EXPECT_EQ(r.probes, 1u);
    EXPECT_LT(r.ms, 20u);   // the prompt, or the next block
}

TEST(GnssDetect, SetUpMosaicAt2MbaudWithNothingKnown)
{
    // The u-blox probes go first and hear 2 Mbaud SBF as noise at 460800,
    // 38400 and 9600: it never frames as UBX or NMEA.
    FakeMosaic m;
    const Result r = detect(&m, planFor(GNSS_TYPE_NONE, 0, true));
    EXPECT_EQ(r.found, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(r.baud, kMosaicTargetBaud);
    EXPECT_EQ(r.probes, 4u);
    EXPECT_LT(r.ms, 1000u);
}

TEST(GnssDetect, SetUpMosaicIsFoundByItsBlocksAlone)
{
    // A port set to take no commands still streams: its blocks prove it.
    struct DeafMosaic : FakeMosaic
    {
        void tick(uint32_t now_ms, const Line& from_host, Line& to_host) override
        {
            Line nothing;
            FakeMosaic::tick(now_ms, nothing, to_host);
            (void)from_host;
        }
    } m;
    const Result r = detect(&m, planFor(GNSS_TYPE_MOSAIC, 0, true));
    EXPECT_EQ(r.found, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(r.proof, Proof::SBF_BLOCK);
}

TEST(GnssDetect, FactoryMosaicAt115200)
{
    FakeMosaic m;
    m.baud = kMosaicFactoryBaud;
    m.set_up = false;
    const Result r = detect(&m, planFor(GNSS_TYPE_NONE, 0, false));
    EXPECT_EQ(r.found, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(r.proof, Proof::PROMPT);
    EXPECT_EQ(r.baud, kMosaicFactoryBaud);
    EXPECT_GE(m.prompts, 1);
}

TEST(GnssDetect, MosaicStillBootingIsFoundOnceUp)
{
    FakeMosaic m;
    m.boots_at_ms = 7000;   // 3 s after the detection starts
    const Result r = detect(&m, planFor(GNSS_TYPE_NONE, 0, false));
    EXPECT_EQ(r.found, GNSS_TYPE_MOSAIC);
    EXPECT_GE(r.ms, 3000u);
    EXPECT_LT(r.ms, kBudgetNoneMs);
}

TEST(GnssDetect, MosaicBootingPastTheBudgetIsLeftToTheSecondLook)
{
    FakeMosaic m;
    m.boots_at_ms = 4000 + kBudgetNoneMs + 2000;
    const Result r = detect(&m, planFor(GNSS_TYPE_NONE, 0, false));
    EXPECT_TRUE(r.done);
    EXPECT_EQ(r.found, GNSS_TYPE_NONE);
    // ...and a minute later it answers the second look at once.
    FakeMosaic later;
    const Result r2 = detect(&later, secondLookPlan(), 10000);
    EXPECT_EQ(r2.found, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(r2.probes, 1u);
}

TEST(GnssDetect, SilenceRunsOutTheBudget)
{
    const Result none = detect(nullptr, planFor(GNSS_TYPE_NONE, 0, false));
    EXPECT_TRUE(none.done);
    EXPECT_EQ(none.found, GNSS_TYPE_NONE);
    EXPECT_GE(none.ms, kBudgetNoneMs);
    EXPECT_LT(none.ms, kBudgetNoneMs + 1500);

    const Result ublox = detect(nullptr, planFor(GNSS_TYPE_UBLOX, 38400, false));
    EXPECT_TRUE(ublox.done);
    EXPECT_EQ(ublox.found, GNSS_TYPE_NONE);
    EXPECT_EQ(ublox.probes, 5u);   // once round: 38400, 9600, 460800, 2M, 115200
    EXPECT_LT(ublox.ms, 1500u);

    const Result mosaic = detect(nullptr, planFor(GNSS_TYPE_MOSAIC, 0, false), 4000, 40000);
    EXPECT_TRUE(mosaic.done);
    EXPECT_EQ(mosaic.found, GNSS_TYPE_NONE);
    EXPECT_GE(mosaic.ms, kBudgetMosaicMs);
    EXPECT_LT(mosaic.ms, kBudgetMosaicMs + 1500);
}

TEST(GnssDetect, NeitherProbeGivesTheOtherReceiverACommand)
{
    // Every probe, round and round for 30 s, at a set-up mosaic (whose own
    // probes it answers) and at a u-blox that answers only at 460800.
    FakeMosaic m;
    Plan p = planFor(GNSS_TYPE_UBLOX, 0, true);
    for (size_t i = 0; i < p.n; i++)
        if (p.probes[i].asks == GNSS_TYPE_MOSAIC) p.probes[i].baud = 921600;   // never its baud
    p.budget_ms = 30000;
    const Result r = detect(&m, p, 4000, 40000);
    EXPECT_EQ(r.found, GNSS_TYPE_NONE);
    // Noise it took for commands is answered "$R?", but nothing it heard
    // starts like a command it knows, nor is the escape that reopens a port.
    const char* const kWords[] = {"set", "get", "exe", "lst", "sso", "sdio", "scs", "eccf",
                                  "spps", "srd", "sfn", "esoc", "lif", "SSSSSSSSSS"};
    for (const std::string& c : m.commands)
    {
        std::string lower;
        for (char ch : c) lower += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        for (const char* w : kWords)
        {
            std::string lw;
            for (const char* q = w; *q; q++) lw += static_cast<char>(std::tolower(static_cast<unsigned char>(*q)));
            EXPECT_NE(lower.compare(0, lw.size(), lw), 0) << "the mosaic heard \"" << c << "\"";
        }
    }

    FakeUblox u;
    u.baud = 460800;
    u.set_up = true;
    Plan q = planFor(GNSS_TYPE_MOSAIC, 0, true);
    q.n = 2;   // the mosaic's probes alone, at 2 Mbaud and 115200
    const Result r2 = detect(&u, q, 4000, 35000);
    EXPECT_EQ(r2.found, GNSS_TYPE_NONE);
    EXPECT_EQ(u.other_ubx, 0);
    EXPECT_EQ(u.nmea_in, 0);
}

TEST(GnssDetect, ASetUpMosaicHeardAtEveryUbloxBaudIsNeverAUblox)
{
    for (uint32_t baud : {9600u, 38400u, 57600u, 115200u, 230400u, 460800u})
    {
        FakeMosaic m;
        Plan p;
        p.probes[p.n++] = Probe{baud, GNSS_TYPE_UBLOX, 400};
        p.budget_ms = 10000;
        const Result r = detect(&m, p, 2000, 12000);
        EXPECT_NE(r.found, GNSS_TYPE_UBLOX) << "at " << baud;
        EXPECT_GT(r.heard, 100u) << "at " << baud;   // noise, not silence
    }
}

TEST(GnssDetect, AUbloxHeardAtTheMosaicsBaudsIsNeverAMosaic)
{
    for (uint32_t ublox_baud : {9600u, 38400u, 460800u})
    {
        for (bool set_up : {false, true})
        {
            FakeUblox u;
            u.baud = ublox_baud;
            u.set_up = set_up;
            Plan p = planFor(GNSS_TYPE_MOSAIC, 0, true);
            p.n = 2;   // 2 Mbaud and 115200 only
            p.budget_ms = 10000;
            const Result r = detect(&u, p, 2000, 12000);
            EXPECT_EQ(r.found, GNSS_TYPE_NONE) << ublox_baud << (set_up ? " set up" : " factory");
            EXPECT_GT(r.heard, 20u) << ublox_baud << (set_up ? " set up" : " factory");
        }
    }
}

// ── Pieces ──────────────────────────────────────────────────────────────

TEST(GnssDetectUbx, ChecksumAndSlicing)
{
    const std::vector<uint8_t> f = monVerReply();
    UbxScanner s;
    int frames = 0;
    for (uint8_t b : f) frames += s.step(b) ? 1 : 0;
    EXPECT_EQ(frames, 1);
    EXPECT_EQ(s.lastClass(), 0x0A);
    EXPECT_EQ(s.lastId(), 0x04);

    std::vector<uint8_t> bad = f;
    bad[20] ^= 0x01;
    UbxScanner s2;
    frames = 0;
    for (uint8_t b : bad) frames += s2.step(b) ? 1 : 0;
    EXPECT_EQ(frames, 0);

    // A frame right after a false sync is still found.
    std::vector<uint8_t> v = {0xB5, 0xB5, 0x62};
    v.insert(v.end(), f.begin() + 2, f.end());
    UbxScanner s3;
    frames = 0;
    for (uint8_t b : v) frames += s3.step(b) ? 1 : 0;
    EXPECT_EQ(frames, 1);

    // The probe's own poll is a valid frame.
    UbxScanner s4;
    frames = 0;
    for (uint8_t b : kUbxMonVerPoll) frames += s4.step(b) ? 1 : 0;
    EXPECT_EQ(frames, 1);
    EXPECT_EQ(s4.lastLength(), 0);
}

TEST(GnssDetectUbx, ImpossibleLengthIsAFalseSync)
{
    UbxScanner s;
    const uint8_t huge[] = {0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF};
    for (uint8_t b : huge) EXPECT_FALSE(s.step(b));
    int frames = 0;
    for (uint8_t b : navPvt(1000)) frames += s.step(b) ? 1 : 0;
    EXPECT_EQ(frames, 1);
}

TEST(GnssDetectNmea, Sentences)
{
    auto line = [](const std::string& s) { return s.substr(0, s.find('\r')); };
    EXPECT_TRUE(isNmeaSentence(line(nmea("GNRMC,,V,,,,,,,,,,N,V")).c_str()));
    EXPECT_TRUE(isNmeaSentence(line(nmea("PUBX,00,000000.00,0000.00000,N")).c_str()));
    EXPECT_TRUE(isNmeaSentence(line(nmea("GNTXT,01,01,02,u-blox AG - www.u-blox.com")).c_str()));
    std::string wrong = line(nmea("GNRMC,,V,,,,,,,,,,N,V"));
    wrong[wrong.size() - 1] = (wrong.back() == '0') ? '1' : '0';
    EXPECT_FALSE(isNmeaSentence(wrong.c_str()));
    EXPECT_FALSE(isNmeaSentence("$R: sdio, COM1, CMD, SBF"));
    EXPECT_FALSE(isNmeaSentence("COM1>"));
    EXPECT_FALSE(isNmeaSentence("$GP*00"));
    EXPECT_FALSE(isNmeaSentence(nullptr));
}

TEST(GnssDetectPlan, TheHintLeads)
{
    const Plan m = planFor(GNSS_TYPE_MOSAIC, 9600, false);
    ASSERT_GE(m.n, 2u);
    EXPECT_EQ(m.probes[0].asks, GNSS_TYPE_MOSAIC);
    EXPECT_EQ(m.probes[0].baud, kMosaicTargetBaud);
    EXPECT_EQ(m.probes[1].baud, kMosaicFactoryBaud);
    EXPECT_EQ(m.fallback, GNSS_TYPE_NONE);
    EXPECT_EQ(m.budget_ms, kBudgetMosaicMs);

    const Plan u = planFor(GNSS_TYPE_UBLOX, 9600, false);
    EXPECT_EQ(u.probes[0].asks, GNSS_TYPE_UBLOX);
    EXPECT_EQ(u.probes[0].baud, 9600u);   // where its driver found it last
    EXPECT_EQ(u.fallback, GNSS_TYPE_UBLOX);
    EXPECT_EQ(u.budget_ms, kBudgetUbloxMs);

    const Plan warm = planFor(GNSS_TYPE_UBLOX, 9600, true);
    EXPECT_EQ(warm.probes[0].baud, kUbloxRunBaud);   // a receiver that kept its power is still set up
    EXPECT_EQ(warm.probes[1].baud, 9600u);

    // Every receiver is asked at every baud its driver can find it at, once.
    for (const Plan& p : {m, u, warm, planFor(GNSS_TYPE_NONE, 0, false)})
    {
        for (uint32_t b : {kMosaicTargetBaud, kMosaicFactoryBaud})
        {
            int n = 0;
            for (size_t i = 0; i < p.n; i++) n += (p.probes[i].asks == GNSS_TYPE_MOSAIC && p.probes[i].baud == b);
            EXPECT_EQ(n, 1);
        }
        for (uint32_t b : {kUbloxRunBaud, 38400u, 9600u})
        {
            int n = 0;
            for (size_t i = 0; i < p.n; i++) n += (p.probes[i].asks == GNSS_TYPE_UBLOX && p.probes[i].baud == b);
            EXPECT_EQ(n, 1);
        }
    }

    // A remembered baud the u-blox driver would never use is ignored.
    const Plan odd = planFor(GNSS_TYPE_UBLOX, 12345, false);
    EXPECT_EQ(odd.probes[0].baud, 38400u);
}

// ── The boot's order of things ──────────────────────────────────────────

using Step = BringUp::Step;

TEST(GnssBringUp, TheProvenReceiverGetsItsDriver)
{
    BringUp b;
    b.start(GNSS_TYPE_MOSAIC, 0, true);
    EXPECT_EQ(b.step(), Step::DETECT);
    b.detected(GNSS_TYPE_MOSAIC);
    EXPECT_EQ(b.step(), Step::BEGIN_MOSAIC);
    EXPECT_EQ(b.mosaicTimeoutMs(), BringUp::kMosaicTimeoutMs);
    b.driverResult(true);
    EXPECT_EQ(b.step(), Step::DONE);
    EXPECT_EQ(b.active(), GNSS_TYPE_MOSAIC);
    EXPECT_FALSE(b.hintChanged());

    BringUp c;
    c.start(GNSS_TYPE_NONE, 0, false);
    c.detected(GNSS_TYPE_UBLOX);
    EXPECT_EQ(c.step(), Step::BEGIN_UBLOX);
    c.driverResult(true);
    EXPECT_EQ(c.active(), GNSS_TYPE_UBLOX);
    EXPECT_TRUE(c.hintChanged());
    EXPECT_EQ(c.hint(), GNSS_TYPE_UBLOX);
}

TEST(GnssBringUp, NothingProvenRunsTheUbloxDriverAsBefore)
{
    BringUp b;
    b.start(GNSS_TYPE_UBLOX, 9600, false);
    b.detected(GNSS_TYPE_NONE);
    EXPECT_EQ(b.step(), Step::BEGIN_UBLOX);
    b.driverResult(true);   // its own search found it (another baud, swapped wires)
    EXPECT_EQ(b.active(), GNSS_TYPE_UBLOX);
    EXPECT_FALSE(b.hintChanged());
    EXPECT_FALSE(b.secondLookRan());
}

TEST(GnssBringUp, AMosaicMissedWhileBootingIsFoundAfterTheUbloxDriver)
{
    BringUp b;
    b.start(GNSS_TYPE_UBLOX, 9600, false);
    b.detected(GNSS_TYPE_NONE);
    b.driverResult(false);   // the u-blox driver's 60 s
    EXPECT_EQ(b.step(), Step::SECOND_LOOK);
    EXPECT_EQ(b.plan().fallback, GNSS_TYPE_NONE);
    for (size_t i = 0; i < b.plan().n; i++) EXPECT_EQ(b.plan().probes[i].asks, GNSS_TYPE_MOSAIC);
    b.detected(GNSS_TYPE_MOSAIC);
    EXPECT_EQ(b.step(), Step::BEGIN_MOSAIC);
    EXPECT_EQ(b.mosaicTimeoutMs(), BringUp::kMosaicLateTimeoutMs);
    b.driverResult(true);
    EXPECT_EQ(b.active(), GNSS_TYPE_MOSAIC);
    EXPECT_TRUE(b.hintChanged());
    EXPECT_EQ(b.hint(), GNSS_TYPE_MOSAIC);
}

TEST(GnssBringUp, AProvenUbloxThatFailsIsNotFollowedByASecondLook)
{
    BringUp b;
    b.start(GNSS_TYPE_UBLOX, 0, false);
    b.detected(GNSS_TYPE_UBLOX);
    b.driverResult(false);
    EXPECT_EQ(b.step(), Step::DONE);
    EXPECT_EQ(b.active(), GNSS_TYPE_NONE);
    EXPECT_FALSE(b.secondLookRan());
}

TEST(GnssBringUp, AMosaicBoardThatAnswersNothingIsForgotten)
{
    BringUp b;
    b.start(GNSS_TYPE_MOSAIC, 0, false);
    b.detected(GNSS_TYPE_NONE);
    EXPECT_EQ(b.step(), Step::DONE);
    EXPECT_EQ(b.active(), GNSS_TYPE_NONE);
    EXPECT_TRUE(b.hintChanged());
    EXPECT_EQ(b.hint(), GNSS_TYPE_NONE);
}

TEST(GnssBringUp, DeadReceiverLeavesTheHintAlone)
{
    BringUp b;
    b.start(GNSS_TYPE_UBLOX, 9600, false);
    b.detected(GNSS_TYPE_NONE);
    b.driverResult(false);
    b.detected(GNSS_TYPE_NONE);
    EXPECT_EQ(b.step(), Step::DONE);
    EXPECT_EQ(b.active(), GNSS_TYPE_NONE);
    EXPECT_FALSE(b.hintChanged());
}

TEST(GnssBringUp, SwappedWiresGoStraightToTheUbloxDriver)
{
    // Its driver found it with RX and TX the other way round: the probes
    // would drive TX against the receiver's output and hear nothing.
    BringUp b;
    b.start(GNSS_TYPE_UBLOX, 9600, false, true);
    EXPECT_EQ(b.plan().n, 0u);
    Detector d;
    d.start(b.plan(), 0);
    EXPECT_TRUE(d.done());
    EXPECT_EQ(d.poll(0).kind, Action::NONE);   // nothing sent at all
    b.detected(d.found());
    EXPECT_EQ(b.step(), Step::BEGIN_UBLOX);
    b.driverResult(false);
    EXPECT_EQ(b.step(), Step::SECOND_LOOK);    // a mosaic moved onto it is still found

    // A first boot of the image on such a board: the same.
    BringUp c;
    c.start(GNSS_TYPE_NONE, 9600, false, true);
    EXPECT_EQ(c.plan().n, 0u);

    // A mosaic found since: the record is old, the mosaic's plan runs.
    BringUp m;
    m.start(GNSS_TYPE_MOSAIC, 9600, false, true);
    EXPECT_GT(m.plan().n, 0u);
    EXPECT_EQ(m.plan().probes[0].asks, GNSS_TYPE_MOSAIC);
}

TEST(GnssBringUp, ARecordThatIsNoReceiverReadsAsNothingKnown)
{
    BringUp b;
    b.start(0xFF, 0, false);
    EXPECT_EQ(b.plan().budget_ms, kBudgetNoneMs);
    EXPECT_EQ(b.hint(), GNSS_TYPE_NONE);
    b.start(GNSS_TYPE_LC86G, 0, false);   // not a J1 receiver
    EXPECT_EQ(b.plan().budget_ms, kBudgetNoneMs);
}

TEST(GnssBringUp, EveryPathFinishesWellInsideTheSetupWatchdog)
{
    // The GNSS part of a boot, from each path's waits: a round of probes, the
    // u-blox driver's 60 s search for nothing, the mosaic driver's own search.
    // fc_rescue_policy.h's setup watchdog is 120 s for all of setup_fc(),
    // which takes ~25 s besides; the u-blox image's dead-receiver boot spent
    // 60 s here.
    const uint32_t kUbloxDriverFailMs = 60000 + 3000;   // its deadline, plus a baud probe past it
    auto worst = [&](uint8_t hint) {
        uint32_t ms = 0;
        BringUp b;
        b.start(hint, 0, false);
        const Plan first = b.plan();
        uint32_t round = 0;
        for (size_t i = 0; i < first.n; i++) round += first.probes[i].listen_ms;
        ms += first.budget_ms + round;   // the budget, then the round it ends inside
        b.detected(GNSS_TYPE_NONE);
        if (b.step() == Step::BEGIN_UBLOX)
        {
            ms += kUbloxDriverFailMs;
            b.driverResult(false);
            if (b.step() == Step::SECOND_LOOK)
            {
                const Plan second = b.plan();
                uint32_t r2 = 0;
                for (size_t i = 0; i < second.n; i++) r2 += second.probes[i].listen_ms;
                ms += second.budget_ms + r2;
                b.detected(GNSS_TYPE_MOSAIC);   // and then its own search fails too
                ms += b.mosaicTimeoutMs();
                b.driverResult(false);
            }
        }
        EXPECT_EQ(b.step(), Step::DONE);
        return ms;
    };
    EXPECT_LT(worst(GNSS_TYPE_MOSAIC), 35000u);
    EXPECT_LT(worst(GNSS_TYPE_UBLOX), 90000u);
    EXPECT_LT(worst(GNSS_TYPE_NONE), 95000u);
}

// ── Naming the receiver, and what it does to the IMU rate ───────────────

TEST(GnssType, NamesRoundTrip)
{
    for (uint8_t t : {GNSS_TYPE_UBLOX, GNSS_TYPE_MOSAIC, GNSS_TYPE_LC86G})
    {
        EXPECT_STRNE(gnssTypeName(t), "");
        EXPECT_EQ(gnssTypeFromName(gnssTypeName(t)), t);
    }
    EXPECT_STREQ(gnssTypeName(GNSS_TYPE_NONE), "");
    EXPECT_EQ(gnssTypeFromName(""), GNSS_TYPE_NONE);
    EXPECT_EQ(gnssTypeFromName(nullptr), GNSS_TYPE_NONE);
    EXPECT_EQ(gnssTypeFromName("mosaic-G5 P3"), GNSS_TYPE_NONE);   // only exact names
    EXPECT_EQ(gnssTypeFromName("u-blo"), GNSS_TYPE_NONE);
    EXPECT_EQ(gnssTypeFromName("F9P"), GNSS_TYPE_NONE);
}

TEST(GnssType, TheMosaicCapsTheImuAt3840)
{
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_MOSAIC), 3840);
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_UBLOX), 7680);
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_NONE), 7680);
    EXPECT_EQ(imuRateMaxForGnss(7680, GNSS_TYPE_LC86G), 7680);
    EXPECT_EQ(imuRateMaxForGnss(3840, GNSS_TYPE_MOSAIC), 3840);
    EXPECT_TRUE(imuRateValid(IMU_RATE_MAX_HZ_MOSAIC));
}

TEST(GnssType, ASettingIsBroughtWithinTheCap)
{
    EXPECT_EQ(imuRateSettingClamp(IMU_RATE_DYNAMIC_8K, 3840), IMU_RATE_DYNAMIC);
    EXPECT_EQ(imuRateSettingClamp(7680, 3840), 3840);
    EXPECT_EQ(imuRateSettingClamp(IMU_RATE_DYNAMIC, 3840), IMU_RATE_DYNAMIC);
    EXPECT_EQ(imuRateSettingClamp(3840, 3840), 3840);
    EXPECT_EQ(imuRateSettingClamp(1920, 3840), 1920);
    EXPECT_EQ(imuRateSettingClamp(960, 3840), 960);
    // Within the board's own limit nothing changes.
    EXPECT_EQ(imuRateSettingClamp(IMU_RATE_DYNAMIC_8K, 7680), IMU_RATE_DYNAMIC_8K);
    EXPECT_EQ(imuRateSettingClamp(7680, 7680), 7680);
    // What is no setting at all comes back for the caller's own check.
    EXPECT_EQ(imuRateSettingClamp(5000, 3840), 5000);
    for (uint16_t s : {IMU_RATE_DYNAMIC, IMU_RATE_DYNAMIC_8K, (uint16_t)960, (uint16_t)1920, (uint16_t)3840, (uint16_t)7680})
    {
        EXPECT_TRUE(imuRateSettingValid(imuRateSettingClamp(s, 3840), 3840)) << s;
    }
}

TEST(GnssIdentity, TheReceiverRidesTheIdentityPush)
{
    char buf[64];
    const size_t n = board_identity::encodePayload("393fed6c-v9+20261017-0017", "V9", buf, sizeof(buf),
                                                   gnssTypeName(GNSS_TYPE_MOSAIC));
    char ver[40], rev[8], gnss[16];
    board_identity::decodePayload(buf, n, ver, sizeof(ver), rev, sizeof(rev));
    EXPECT_STREQ(ver, "393fed6c-v9+20261017-0017");
    EXPECT_STREQ(rev, "V9");   // what every OC reads, this field or not
    board_identity::decodeGnss(buf, n, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "mosaic-G5");
    EXPECT_EQ(gnssTypeFromName(gnss), GNSS_TYPE_MOSAIC);
}

TEST(GnssIdentity, AnUnprovisionedBoardKeepsAnEmptyRevision)
{
    char buf[64];
    const size_t n = board_identity::encodePayload("abc-v9", "", buf, sizeof(buf), "u-blox");
    char ver[40], rev[8], gnss[16];
    board_identity::decodePayload(buf, n, ver, sizeof(ver), rev, sizeof(rev));
    EXPECT_STREQ(rev, "");   // not "UBLOX": the NUL that ends the empty revision is always sent
    board_identity::decodeGnss(buf, n, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "u-blox");
}

TEST(GnssIdentity, NoReceiverOrAnOldFcSendsTwoFields)
{
    char a[64], b[64];
    const size_t na = board_identity::encodePayload("abc-v9", "V9", a, sizeof(a));
    const size_t nb = board_identity::encodePayload("abc-v9", "V9", b, sizeof(b), "");
    ASSERT_EQ(na, nb);
    EXPECT_EQ(std::memcmp(a, b, na), 0);
    char gnss[16] = "x";
    board_identity::decodeGnss(a, na, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "");
    // An FC that predates the revision too: the version alone.
    board_identity::decodeGnss("abc-v9", 6, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "");
}

TEST(GnssIdentity, AFieldThatIsNotANameIsEmpty)
{
    const char weird[] = "v\0V9\0mosaic\"G5";
    char gnss[16] = "x";
    board_identity::decodeGnss(weird, sizeof(weird) - 1, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "");
    const char longname[] = "v\0V9\0a-receiver-name-too-long";
    board_identity::decodeGnss(longname, sizeof(longname) - 1, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "");   // never cut into another name
    // A payload with no room for the whole name leaves the field out.
    char tiny[12];
    const size_t n = board_identity::encodePayload("abcdef", "V9", tiny, sizeof(tiny), "mosaic-G5");
    board_identity::decodeGnss(tiny, n, gnss, sizeof(gnss));
    EXPECT_STREQ(gnss, "");
}
