// rinex_nav.cpp -- see rinex_nav.h.
#include "rinex_nav.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include "gnss/types.h"
#include "SbfNav.h"   // kGalHealthNoStatus, kGalHealthNoSisa: the same marks the SBF decoder sets

namespace rinex
{

namespace
{

constexpr int kBdtWeek0 = 1356;   // BDT week 0 = GPS week 1356 (2006-01-01)

// One field of a record: 19 characters, a FORTRAN D or E exponent, blank = 0.
double field(const std::string& line, size_t at, bool& ok)
{
    if (line.size() <= at)
        return 0.0;
    std::string s = line.substr(at, 19);
    bool blank = true;
    for (char& c : s)
    {
        if (c == 'D' || c == 'd')
            c = 'E';
        if (c != ' ')
            blank = false;
    }
    if (blank)
        return 0.0;
    char* end = nullptr;
    const double x = std::strtod(s.c_str(), &end);
    if (end == s.c_str())
        ok = false;
    return x;
}

// The four fields of a broadcast-orbit line (4X, 4D19.12).
void orbit(const std::string& line, double f[4], bool& ok)
{
    for (int k = 0; k < 4; ++k)
        f[k] = field(line, 4 + 19 * k, ok);
}

int intOf(const std::string& s, size_t at, size_t n, bool& ok)
{
    if (s.size() < at + n)
    {
        ok = false;
        return 0;
    }
    const std::string t = s.substr(at, n);
    char* end = nullptr;
    const long v = std::strtol(t.c_str(), &end, 10);
    if (end == t.c_str())
        ok = false;
    return static_cast<int>(v);
}

// One record: its first line and its broadcast-orbit lines.
enum class Took { OK, SYSTEM, FNAV, GEO, BAD };

Took parseRecord(const std::vector<std::string>& L, NavRecord& r)
{
    const std::string& h = L[0];
    const char sys = h[0];
    if (sys != 'G' && sys != 'E' && sys != 'C')
        return Took::SYSTEM;
    if (L.size() < 8)
        return Took::BAD;
    bool ok = true;
    const int prn = intOf(h, 1, 2, ok);
    const int year = intOf(h, 4, 4, ok), month = intOf(h, 9, 2, ok), day = intOf(h, 12, 2, ok);
    const int hour = intOf(h, 15, 2, ok), minute = intOf(h, 18, 2, ok), second = intOf(h, 21, 2, ok);
    const double af0 = field(h, 23, ok), af1 = field(h, 42, ok), af2 = field(h, 61, ok);
    double o[7][4];
    for (int k = 0; k < 7; ++k)
        orbit(L[1 + k], o[k], ok);
    if (!ok)
        return Took::BAD;

    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    e.prn = prn;
    int toc_week = 0;
    double toc = 0.0;
    calendarToWeek(year, month, day, hour, minute, static_cast<double>(second), toc_week, toc);
    e.toc = toc;
    e.af0 = af0;
    e.af1 = af1;
    e.af2 = af2;
    // the Keplerian set, laid out alike by all three systems
    e.crs = o[0][1];
    e.delta_n = o[0][2];
    e.m0 = o[0][3];
    e.cuc = o[1][0];
    e.e = o[1][1];
    e.cus = o[1][2];
    e.sqrt_a = o[1][3];
    e.toe = o[2][0];
    e.cic = o[2][1];
    e.omega0 = o[2][2];
    e.cis = o[2][3];
    e.i0 = o[3][0];
    e.crc = o[3][1];
    e.omega = o[3][2];
    e.omega_dot = o[3][3];
    e.idot = o[4][0];

    if (sys == 'G')
    {
        if (prn < 1 || prn > 37)
            return Took::BAD;
        e.sys = GNSS_SYS_GPS;
        e.iode = static_cast<int>(std::lround(o[0][0]));
        e.week = static_cast<int>(std::lround(o[4][2]));   // continuous, not mod 1024
        e.ura = uraIndex(o[5][0]);
        e.health = static_cast<int>(std::lround(o[5][1])) & 0x3F;
        e.tgd = o[5][2];
        e.iodc = static_cast<int>(std::lround(o[5][3]));
    }
    else if (sys == 'E')
    {
        if (prn < 1 || prn > 36)
            return Took::BAD;
        const long src = std::lround(o[4][1]);
        // I/NAV (E1-B or E5b) with the (E5b,E1) clock, as GALNav Source 2;
        // F/NAV repeats IODnav and toe with the (E5a,E1) clock
        if ((src & 0x02) || !(src & 0x200) || !(src & 0x05))
            return Took::FNAV;
        e.sys = GNSS_SYS_GAL;
        e.iode = static_cast<int>(std::lround(o[0][0])) & 0x3FF;   // IODnav
        e.iodc = e.iode;
        e.week = static_cast<int>(std::lround(o[4][2]));            // a GPS week in RINEX 3
        const double sisa = o[5][0];
        e.ura = sisaIndex(sisa);
        const long hv = std::lround(o[5][1]);
        // E1-B DVS (bit 0) and HS (bits 1-2): SbfNav's (HS << 1) | DVS; none
        // without the E1-B page
        e.health = (src & 0x01) ? static_cast<int>(hv & 0x07) : mosaic::kGalHealthNoStatus;
        if (e.ura == 255)
            e.health |= mosaic::kGalHealthNoSisa;
        e.tgd = o[5][3];                                             // BGD(E5b,E1)
    }
    else
    {
        if (prn < 1 || prn > 63)
            return Took::BAD;
        if (prn <= 5 || prn >= 59)
            return Took::GEO;
        e.sys = GNSS_SYS_BDS;
        e.iode = static_cast<int>(std::lround(o[0][0]));             // AODE
        e.week = static_cast<int>(std::lround(o[4][2])) + kBdtWeek0; // BDT week
        e.ura = uraIndex(o[5][0]);
        e.health = static_cast<int>(std::lround(o[5][1]));           // SatH1
        e.tgd = o[5][2];                                             // TGD1, B1I
        e.iodc = static_cast<int>(std::lround(o[6][1]));             // AODC
    }
    if (!std::isfinite(e.sqrt_a) || e.sqrt_a <= 0.0 || !std::isfinite(e.toe))
        return Took::BAD;
    e.valid = 1;
    r.eph = e;
    r.toe_gps_s = toeGpsS(e);
    r.avail_gps_s = availGpsS(e);
    return Took::OK;
}

}  // namespace

double toeGpsS(const gps_eph_t& e)
{
    return e.week * 604800.0 + e.toe - (e.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
}

double availGpsS(const gps_eph_t& e)
{
    const double lag = e.sys == GNSS_SYS_GAL ? kGalLagS : (e.sys == GNSS_SYS_BDS ? kBdsLagS : kGpsLeadS);
    return toeGpsS(e) + lag;
}

long daysFromGpsEpoch(int year, int month, int day)
{
    // days from 1970-01-01 (H. Hinnant's days_from_civil), less 1980-01-06's
    const int y = year - (month <= 2 ? 1 : 0);
    const long era = (y >= 0 ? y : y - 399) / 400;
    const long yoe = y - era * 400;
    const long doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long days1970 = era * 146097 + doe - 719468;
    return days1970 - 3657;
}

void calendarToWeek(int year, int month, int day, int hour, int minute, double second, int& week, double& sow)
{
    const long d = daysFromGpsEpoch(year, month, day);
    week = static_cast<int>(d >= 0 ? d / 7 : -((-d + 6) / 7));
    const long dow = d - static_cast<long>(week) * 7;
    sow = dow * 86400.0 + hour * 3600.0 + minute * 60.0 + second;
}

int uraIndex(double m)
{
    static const double kBound[15] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
                                      96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
    if (!(m >= 0.0))
        return 15;
    for (int i = 0; i < 15; ++i)
        if (m <= kBound[i])
            return i;
    return 15;
}

int sisaIndex(double m)
{
    if (!(m >= 0.0))
        return 255;
    if (m < 0.495)
        return static_cast<int>(std::lround(m * 100.0));
    if (m < 0.99)
        return 50 + static_cast<int>(std::lround((m - 0.5) / 0.02));
    if (m < 1.98)
        return 75 + static_cast<int>(std::lround((m - 1.0) / 0.04));
    if (m <= 6.0)
        return 100 + static_cast<int>(std::lround((m - 2.0) / 0.16));
    return 255;
}

bool parseNav(const std::string& text, NavFile& out, std::string& err)
{
    out = NavFile{};
    std::istringstream in(text);
    std::string line;
    bool header = true;
    std::vector<std::string> rec;
    auto flush = [&]() {
        if (rec.empty())
            return;
        NavRecord r;
        switch (parseRecord(rec, r))
        {
            case Took::OK: out.recs.push_back(r); break;
            case Took::SYSTEM: out.skipped_system++; break;
            case Took::FNAV: out.skipped_fnav++; break;
            case Took::GEO: out.skipped_geo++; break;
            default: out.bad++; break;
        }
        rec.clear();
    };
    bool first = true;
    while (std::getline(in, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (header)
        {
            if (first)
            {
                first = false;
                // "     3.05           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE"
                if (line.find("RINEX VERSION / TYPE") == std::string::npos || line.size() < 21 || line[20] != 'N')
                {
                    err = "not a RINEX navigation file";
                    return false;
                }
                out.version_major = std::atoi(line.substr(0, 9).c_str());
                if (out.version_major != 3)
                {
                    err = "RINEX version " + std::to_string(out.version_major) + ", not 3";
                    return false;
                }
            }
            if (line.find("END OF HEADER") != std::string::npos)
                header = false;
            continue;
        }
        if (line.empty())
            continue;
        if (line[0] != ' ')
            flush();   // a new record starts with its system's letter
        rec.push_back(line);
    }
    flush();
    if (header)
    {
        err = "no END OF HEADER";
        return false;
    }
    return true;
}

bool readNavFile(const std::string& path, NavFile& out, std::string& err)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
    {
        err = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parseNav(ss.str(), out, err);
}

}  // namespace rinex
