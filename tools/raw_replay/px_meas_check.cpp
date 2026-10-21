// px_meas_check.cpp -- the PX1105R's measurements through the PX1105R image's GNSS task, against a known receiver
// (raw_replay's --receiver px1105r, its measurement stage alone).
//
// Each SkyTraq frame of a capture ('<host_t> B <hex body>') goes through the driver's chain (skytraq_feed.h: the
// 0xE5 on the fused grid, its Dopplers in pairs, the 0xDF's clock bias, the subframes' ephemerides and ionosphere),
// and each fused-grid epoch is built (RawGnssMeasStage::build, the image's profile: the code once a second is the
// filter's, every code is built here) at a receiver the caller knows: a fixed one (--fixed lat,lon,h: a static
// capture) or a trajectory (--truth FILE: 't_gps lat_deg lon_deg h_m' rows, ellipsoidal, the epochs placed at their
// GPS instant). Each measurement is written with what its residual needs: the corrected pseudorange and range rate
// (receiver clock and drift still in them), the satellite at its transmit time turned into the receive instant's
// ECEF, the unit line of sight, and the satellite's C/N0 and lock; the caller removes the clock and compares the
// range rate with the receiver's velocity at any instant (px_meas.py).
//
//   px_meas_check --capture CAPTURE --out CSV (--fixed lat,lon,h | --truth FILE) [--pairs 0|1]
//                 [--iono "a0 a1 a2 a3 b0 b1 b2 b3 day"] [--no-iono]
//
// --x-out FILE --x-gps-ign S [--x-tofs 200] [--x-seed N] (with --truth): a mosaic-grade receiver on the same
// satellites, as raw_replay's sim epochs ('X' lines, sim_epoch.h): every 0xE5 (20 Hz, no grid cut, no pairs: the
// harness cuts and pairs a sim's epochs itself), each satellite the PX1105R tracked placed by its broadcast ephemeris
// as the image builds it, its pseudorange and range rate the truth's (the receiver on the trajectory, its clock 0)
// plus the mosaic-G5 profile's noise for the PX1105R's C/N0 (the code white 0.10/0.30 and correlated 1.94/3.5 x 0.5,
// 15 s; the Doppler white 0.01/0.075, no lag): the PX1105R's own geometry and losses with a mosaic's measurements,
// on the MCU clock px_events.py writes (MCU s = GPS s - S + T_OFS), released 30 ms after the epoch.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "RawGnssMeasStage.h"
#include "RawGnssShadow.h"
#include "ReceiverProfileApply.h"
#include "SkyTraqBinary.h"
#include "skytraq_feed.h"

using namespace gnss_raw;

namespace
{

constexpr double kD2R = M_PI / 180.0;

void llaToEcef(double lat, double lon, double h, double x[3])
{
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    const double s = std::sin(lat), c = std::cos(lat);
    const double n = a / std::sqrt(1.0 - e2 * s * s);
    x[0] = (n + h) * c * std::cos(lon);
    x[1] = (n + h) * c * std::sin(lon);
    x[2] = (n * (1.0 - e2) + h) * s;
}

struct Truth
{
    std::vector<double> t, lat, lon, h;
    bool at(double ts, double& la, double& lo, double& hh) const
    {
        if (t.empty() || ts < t.front() || ts > t.back())
            return false;
        size_t i = std::lower_bound(t.begin(), t.end(), ts) - t.begin();
        if (i == 0)
            i = 1;
        const double f = (ts - t[i - 1]) / (t[i] - t[i - 1]);
        la = lat[i - 1] + f * (lat[i] - lat[i - 1]);
        lo = lon[i - 1] + f * (lon[i] - lon[i - 1]);
        hh = h[i - 1] + f * (h[i] - h[i - 1]);
        return true;
    }
};

std::vector<uint8_t> unhex(const std::string& s)
{
    std::vector<uint8_t> out(s.size() / 2);
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = static_cast<uint8_t>(std::stoi(s.substr(2 * i, 2), nullptr, 16));
    return out;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string cap, out, truth_path, iono_s, eph_out, x_out;
    double x_gps_ign = 0.0, x_tofs = 200.0;
    int x_seed = 1;
    double fixed[3] = {0, 0, 0};
    bool have_fixed = false, no_iono = false;
    int pairs = 1;
    for (int i = 1; i < argc; ++i)
    {
        const std::string k = argv[i];
        auto val = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (k == "--capture") cap = val();
        else if (k == "--out") out = val();
        else if (k == "--truth") truth_path = val();
        else if (k == "--pairs") pairs = std::atoi(val().c_str());
        else if (k == "--iono") iono_s = val();
        else if (k == "--no-iono") no_iono = true;
        else if (k == "--eph-out") eph_out = val();
        else if (k == "--x-out") x_out = val();
        else if (k == "--x-gps-ign") x_gps_ign = std::atof(val().c_str());
        else if (k == "--x-tofs") x_tofs = std::atof(val().c_str());
        else if (k == "--x-seed") x_seed = std::atoi(val().c_str());
        else if (k == "--fixed")
        {
            have_fixed = std::sscanf(val().c_str(), "%lf,%lf,%lf", &fixed[0], &fixed[1], &fixed[2]) == 3;
        }
        else
        {
            std::fprintf(stderr, "unknown argument %s\n", k.c_str());
            return 2;
        }
    }
    if (cap.empty() || out.empty() || (!have_fixed && truth_path.empty()))
    {
        std::fprintf(stderr, "px_meas_check --capture F --out CSV (--fixed lat,lon,h | --truth FILE) [--pairs 0|1] "
                             "[--iono '...'] [--no-iono]\n");
        return 2;
    }
    Truth tr;
    if (!truth_path.empty())
    {
        std::ifstream f(truth_path);
        double a, b, c, d;
        while (f >> a >> b >> c >> d)
        {
            tr.t.push_back(a);
            tr.lat.push_back(b * kD2R);
            tr.lon.push_back(c * kD2R);
            tr.h.push_back(d);
        }
    }

    // the PX1105R image's configuration (main.cpp startRawShadow under TR_GNSS_DRIVER_PX1105R)
    GnssReceiverProfile rx = px1105rProfile();
    applyPx1105rFlightProfile(rx);
    if (pairs == 0)
        rx.rr_pair_avg = false;
    RawGnssShadow::Config cfg;
    applyReceiverProfile(rx, cfg);
    applyPx1105rFlightSafeguards(cfg);
    std::unique_ptr<RawGnssMeasStage> stp(new RawGnssMeasStage(RawGnssShadow::stageConfig(cfg)));
    RawGnssMeasStage& stage = *stp;

    const bool xmode = !x_out.empty();
    if (xmode && (truth_path.empty() || x_gps_ign <= 0.0))
    {
        std::fprintf(stderr, "--x-out needs --truth and --x-gps-ign\n");
        return 2;
    }
    if (xmode)
        pairs = 0;
    rr::SkyTraqFeed::Config pc;
    pc.fuse_period_ms = xmode ? 50u : static_cast<uint32_t>(std::llround(rx.fuse_epoch_period_s * 1000.0));
    pc.epoch_latency_s = rx.epoch_latency_s;
    pc.pair = xmode ? false : rx.rr_pair_avg;
    pc.pair_cfg.dt_ms = static_cast<uint32_t>(std::llround(rx.raw_epoch_period_s * 1000.0));
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        pc.pair_cfg.cn0_min[k] = rx.sig[k].cn0_min_rr;
    pc.pair_cfg.max_diff_mps = rx.rr_pair_max_diff_mps;
    std::unique_ptr<rr::SkyTraqFeed> feedp(new rr::SkyTraqFeed(pc));
    rr::SkyTraqFeed& feed = *feedp;
    if (!iono_s.empty() && !no_iono)
    {
        int v[9];
        if (std::sscanf(iono_s.c_str(), "%d %d %d %d %d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6],
                        &v[7], &v[8]) == 9)
        {
            ShadowIono io{};
            for (int i = 0; i < 8; ++i)
                io.raw[i] = static_cast<int8_t>(v[i]);
            io.gps_day = static_cast<uint16_t>(v[8]);
            if (feed.offerAppIono(io))
                stage.setIono(io);
        }
    }

    FILE* fo = std::fopen(out.c_str(), "w");
    if (!fo)
        return 1;
    // --eph-out: every ephemeris the chain completed (repeats included), for the group delays' check
    FILE* fe = eph_out.empty() ? nullptr : std::fopen(eph_out.c_str(), "w");
    if (fe)
        std::fprintf(fe, "sys,prn,week,iode,iodc,toe,health,ura,tgd_s,af0_s,filed\n");
    std::fprintf(fo, "gps_s,sys,prn,cn0,lock_ms,paired,use_iono,has_pr,pr,rho,has_rr,rr,ux,uy,uz,vsx,vsy,vsz,"
                     "rx,ry,rz,sigma_pr,sigma_rr,dtr_ms\n");
    FILE* fx = xmode ? std::fopen(x_out.c_str(), "w") : nullptr;
    uint32_t rng = 2463534242u + static_cast<uint32_t>(x_seed) * 7919u;
    auto unif = [&rng]() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return ((rng & 0xFFFFFF) + 0.5) / 16777216.0;
    };
    auto gauss = [&]() { return std::sqrt(-2.0 * std::log(unif())) * std::cos(2.0 * M_PI * unif()); };
    double gm[GNSS_SYS_COUNT][64] = {};   // each satellite's correlated code error
    double gm_t[GNSS_SYS_COUNT][64] = {};
    int x_lines = 0;
    std::ifstream f(cap);
    std::string line;
    skytraq::FrameParser parser;   // unused: the capture holds bodies
    (void)parser;
    std::unique_ptr<TimedRawEpoch> tep(new TimedRawEpoch());
    std::unique_ptr<BuiltEpoch> bp(new BuiltEpoch());
    int epochs = 0, built = 0, rows = 0, ephs = 0, ions = 0;
    while (std::getline(f, line))
    {
        std::istringstream ss(line);
        double ht = 0.0;
        std::string tag, hex;
        if (!(ss >> ht >> tag) || tag != "B" || !(ss >> hex))
            continue;
        const std::vector<uint8_t> body = unhex(hex);
        TimedRawEpoch& te = *tep;
        const rr::SkyTraqFeed::Kind kind =
            feed.onBody(body.data(), body.size(), static_cast<int64_t>(std::llround(ht * 1.0e6)), te);
        for (const gps_eph_t& e : feed.ephs())
        {
            const bool filed = stage.addEphemeris(e);
            if (filed)
                ephs++;
            if (fe)
                std::fprintf(fe, "%d,%d,%d,%d,%d,%.0f,%d,%d,%.12e,%.12e,%d\n", e.sys, e.prn, e.week, e.iode, e.iodc,
                             e.toe, e.health, e.ura, e.tgd, e.af0, filed ? 1 : 0);
        }
        if (kind == rr::SkyTraqFeed::Kind::ION && !no_iono)
        {
            stage.setIono(feed.iono());
            ions++;
        }
        if (kind != rr::SkyTraqFeed::Kind::EPOCH || !feed.biased())
            continue;
        epochs++;
        const double gps_s = feed.gpsS();
        double lat = fixed[0] * kD2R, lon = fixed[1] * kD2R, h = fixed[2];
        if (!have_fixed && !tr.at(gps_s, lat, lon, h))
        {
            stage.see(te);
            stage.noteEpoch(te.epoch);
            continue;
        }
        double r[3];
        llaToEcef(lat, lon, h, r);
        te.mcu_us = static_cast<int64_t>(std::llround(gps_s * 1.0e6)) - 1400000000000000LL;
        stage.see(te);
        BuiltEpoch& b = *bp;
        stage.build(te, r, false, b);
        built++;
        if (xmode)
        {
            // the receiver's velocity: the trajectory's, central difference over 10 ms
            double lat0, lon0, h0, lat1, lon1, h1, r0[3], r1[3], v[3] = {0, 0, 0};
            if (tr.at(gps_s - 0.005, lat0, lon0, h0) && tr.at(gps_s + 0.005, lat1, lon1, h1))
            {
                llaToEcef(lat0, lon0, h0, r0);
                llaToEcef(lat1, lon1, h1, r1);
                for (int k = 0; k < 3; ++k)
                    v[k] = (r1[k] - r0[k]) / 0.01;
            }
            std::string parts;
            int nx = 0;
            char buf[512];
            for (int i = 0; i < b.n; ++i)
            {
                const RawMeas& m = b.meas[i];
                if (m.sys >= GNSS_SYS_COUNT || m.prn >= 64)
                    continue;
                const double kC = 299792458.0, kOmegaE = 7.2921151467e-5;
                double d0 = m.sat_pos[0] - r[0], d1 = m.sat_pos[1] - r[1], d2 = m.sat_pos[2] - r[2];
                double rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2), c = 1.0, s = 0.0;
                for (int it = 0; it < 2; ++it)
                {
                    const double th = kOmegaE * rho / kC;
                    c = std::cos(th);
                    s = std::sin(th);
                    const double x0 = c * m.sat_pos[0] + s * m.sat_pos[1], y0 = -s * m.sat_pos[0] + c * m.sat_pos[1];
                    d0 = x0 - r[0];
                    d1 = y0 - r[1];
                    d2 = m.sat_pos[2] - r[2];
                    rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
                }
                const double vs[3] = {c * m.sat_vel[0] + s * m.sat_vel[1], -s * m.sat_vel[0] + c * m.sat_vel[1],
                                      m.sat_vel[2]};
                const double rate = (d0 * (vs[0] - v[0]) + d1 * (vs[1] - v[1]) + d2 * (vs[2] - v[2])) / rho;
                float cn0 = 0.0f;
                for (int j = 0; j < te.epoch.n; ++j)
                    if (te.epoch.obs[j].sys == m.sys && te.epoch.obs[j].prn == m.prn)
                        cn0 = te.epoch.obs[j].cn0_dbhz;
                const double q = std::pow(10.0, -(cn0 - 35.0) / 10.0);
                const double s_pw = std::sqrt(0.10 * 0.10 + 0.30 * 0.30 * q);
                const double s_pc = 0.5 * std::sqrt(1.94 * 1.94 + 3.5 * 3.5 * q);
                const double s_rr = std::sqrt(0.01 * 0.01 + 0.075 * 0.075 * q);
                double& g = gm[m.sys][m.prn];
                double& gt = gm_t[m.sys][m.prn];
                const double dt = gt > 0.0 ? gps_s - gt : 1.0e9;
                const double phi = dt < 1.0e8 ? std::exp(-dt / 15.0) : 0.0;
                g = phi * g + std::sqrt(1.0 - phi * phi) * s_pc * gauss();
                gt = gps_s;
                const double pr = rho + g + s_pw * gauss();
                const double rrm = rate + s_rr * gauss();
                std::snprintf(buf, sizeof buf, " %u %u 1 1 %.3f %.3f %.3f %.4f %.4f %.4f %.5f %.5f %.5f %.4f %.5f %.2f 0.000",
                              m.sys, m.prn, m.sat_pos[0], m.sat_pos[1], m.sat_pos[2], m.sat_vel[0], m.sat_vel[1],
                              m.sat_vel[2], m.sat_acc[0], m.sat_acc[1], m.sat_acc[2], pr, rrm, cn0);
                parts += buf;
                nx++;
            }
            if (nx > 0 && fx)
            {
                const long long mcu = std::llround((gps_s - x_gps_ign + x_tofs) * 1.0e6);
                const int week = static_cast<int>(std::floor(gps_s / 604800.0));
                std::fprintf(fx, "X %lld %lld %.3f %d %d%s\n", mcu + 30000, mcu, gps_s - week * 604800.0, week, nx,
                             parts.c_str());
                x_lines++;
            }
        }
        for (int i = 0; i < b.n; ++i)
        {
            const RawMeas& m = b.meas[i];
            // the satellite at transmit time into the receive instant's ECEF (TR_RawGnssEKF.cpp's line of sight)
            const double kC = 299792458.0, kOmegaE = 7.2921151467e-5;
            double d0 = m.sat_pos[0] - r[0], d1 = m.sat_pos[1] - r[1], d2 = m.sat_pos[2] - r[2];
            double rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
            double c = 1.0, s = 0.0, rr_[3] = {m.sat_pos[0], m.sat_pos[1], m.sat_pos[2]};
            for (int it = 0; it < 2; ++it)
            {
                const double th = kOmegaE * rho / kC;
                c = std::cos(th);
                s = std::sin(th);
                rr_[0] = c * m.sat_pos[0] + s * m.sat_pos[1];
                rr_[1] = -s * m.sat_pos[0] + c * m.sat_pos[1];
                rr_[2] = m.sat_pos[2];
                d0 = rr_[0] - r[0];
                d1 = rr_[1] - r[1];
                d2 = rr_[2] - r[2];
                rho = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
            }
            const double vs[3] = {c * m.sat_vel[0] + s * m.sat_vel[1], -s * m.sat_vel[0] + c * m.sat_vel[1],
                                  m.sat_vel[2]};
            float cn0 = 0.0f;
            uint32_t lock = 0;
            for (int j = 0; j < te.epoch.n; ++j)
                if (te.epoch.obs[j].sys == m.sys && te.epoch.obs[j].prn == m.prn)
                {
                    cn0 = te.epoch.obs[j].cn0_dbhz;
                    lock = te.epoch.obs[j].lock_ms;
                }
            std::fprintf(fo, "%.4f,%u,%u,%.0f,%u,%u,%u,%d,%.4f,%.4f,%d,%.5f,%.8f,%.8f,%.8f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,"
                             "%.3f,%.4f,%.6f\n",
                         gps_s, m.sys, m.prn, cn0, lock, te.rr_paired, b.use_broadcast, m.has_pr ? 1 : 0, m.pr, rho,
                         m.has_rr ? 1 : 0, m.rr, d0 / rho, d1 / rho, d2 / rho, vs[0], vs[1], vs[2], r[0], r[1], r[2],
                         m.sigma_pr, m.sigma_rr, feed.dtrMs());
            rows++;
        }
    }
    std::fclose(fo);
    if (fe)
        std::fclose(fe);
    if (fx)
    {
        std::fclose(fx);
        std::fprintf(stderr, "  sim epochs (X) written: %d\n", x_lines);
    }
    const rr::SkyTraqFeed::Counts& n = feed.counts();
    std::fprintf(stderr, "%s: epochs %d (timed by a 0xDF bias), built %d, rows %d; ephemerides filed %d, ionosphere "
                         "sets %d; 0xE5 %u, 0xDF %u, nav refused %u, slips %u, clock jumps %u\n",
                 cap.c_str(), epochs, built, rows, ephs, ions, n.e5, n.df, n.nav_refused, feed.lock().slips,
                 feed.clock().jumps);
    const skytraq::NavCounters& nc = feed.nav().counters();
    std::fprintf(stderr, "  nav: frames GPS %u GAL %u BDS %u, ephemerides %u/%u/%u (new issues %u/%u/%u), iono pages %u; by "
                         "status:", nc.frames[0], nc.frames[1], nc.frames[2], nc.ephemerides[0], nc.ephemerides[1],
                 nc.ephemerides[2], nc.issues[0], nc.issues[1], nc.issues[2], nc.iono);
    for (size_t k = 0; k < static_cast<size_t>(skytraq::NavStatus::COUNT); ++k)
        if (nc.status[k] != 0)
            std::fprintf(stderr, " %s %u", skytraq::navStatusName(static_cast<skytraq::NavStatus>(k)), nc.status[k]);
    std::fprintf(stderr, "\n");
    return 0;
}
