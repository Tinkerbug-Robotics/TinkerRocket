// CodeBridge.h — an L5-band pseudorange moved onto its satellite's L1-band
// code, so the filter can fuse it where the L1 band's is gone (header-only,
// pure)
//
// A receiver that tracks both bands (the PX1105R: GPS L5, Galileo E5a and
// BeiDou B2a beside L1 C/A, E1 and B1I) gives each satellite two codes. The
// filter's pseudorange row is the L1 band's: its receiver clock bias and its
// Galileo and BeiDou offsets are the L1 band's, and so are the group delay and
// ionosphere the builder takes off. An L5-band code differs from its
// satellite's L1-band one by the receiver's own inter-band delay (on the RF
// rig, 2026-11-24, -5..+5 m on one run and +22..+35 m on another: it moves from
// one power-up to the next), the satellite's group delays (BeiDou B2a's comes in
// no message the PX1105R forwards) and the ionosphere's extra 0.79 of the L1
// delay -- and by noise. None of it changes much in a flight's minutes: on the
// rig each satellite's difference moved 2-5 m from the pad to the fall.
//
// So, per satellite: while both bands give code, the difference L5 - L1 (each
// as the receiver gave it: the receiver's clock and any whole-millisecond jump
// of it are in both, and cancel) kept as a mean that forgets over tau_s; and
// where an epoch has the satellite's L5-band code but not its L1-band one, that
// code less the kept difference -- the L1 band's code as the L5 band's
// measures it -- with RawObs::pr_l1 set. The builder then corrects it as the L1
// band's code and fuses it as that (RawMeasBuilder.h, bridge_sigma_m more). A
// satellite's difference is used once min_n epochs made it, until max_age_s
// after the last of them; a step of more than max_step_m in it (one band's
// code slipped, a relock) starts it over. An L5-band code whose satellite has
// the L1 band's in the epoch is left as it came, never fused.
//
// Only GPS, Galileo and BeiDou (the L5 band's systems on this receiver). The
// epoch's cells are looked at as they are: an L1-band one with code anywhere
// in it counts, whichever order the reader put them in.
#ifndef GNSS_RAW_CODE_BRIDGE_H
#define GNSS_RAW_CODE_BRIDGE_H

#include <cmath>
#include <cstdint>

#include "RawMeas.h"
#include "gnss/types.h"

namespace gnss_raw
{

struct CodeBridgeConfig
{
    double   tau_s = 10.0;         // the difference's mean forgets over this long
    double   max_age_s = 60.0;     // and is used at most this long after the last epoch with both codes
    uint32_t min_n = 20;           // from this many such epochs on (1 s at 20 Hz)
    double   max_step_m = 15.0;    // a difference further than this from the mean starts it over
};

struct CodeBridgeStats
{
    uint32_t moved = 0;            // L5-band codes moved onto the L1 band's
    uint32_t too_new = 0;          //   left as they came: their satellite's difference under min_n epochs
    uint32_t too_old = 0;          //     or older than max_age_s
    uint32_t none = 0;             //     or never made
    uint32_t restarts = 0;         // differences started over by a step
};

class CodeBridge
{
public:
    static constexpr int kSys = GNSS_SYS_BDS + 1;
    static constexpr int kPrn = 64;

    explicit CodeBridge(const CodeBridgeConfig& c = CodeBridgeConfig()) : cfg_(c) { reset(); }

    void setConfig(const CodeBridgeConfig& c) { cfg_ = c; }
    const CodeBridgeConfig& config() const { return cfg_; }
    const CodeBridgeStats& stats() const { return st_; }

    void reset()
    {
        for (int s = 0; s < kSys; ++s)
            for (int p = 0; p < kPrn; ++p)
                sat_[s][p] = Sat{};
    }

    // One epoch: each satellite's difference noted where it has both codes,
    // then each L5-band code whose satellite has no L1-band one moved (or left).
    // Returns the codes moved.
    int apply(RawEpoch& e)
    {
        // each satellite's L1-band code in the epoch, a bit a PRN by system
        uint64_t l1[kSys] = {};
        for (int i = 0; i < e.n; ++i)
        {
            const RawObs& o = e.obs[i];
            if (o.band == kBandL1 && o.has_pr && o.sys < kSys && o.prn < kPrn)
                l1[o.sys] |= uint64_t{1} << o.prn;
        }
        int moved = 0;
        for (int i = 0; i < e.n; ++i)
        {
            RawObs& o = e.obs[i];
            if (o.band != kBandL5 || !o.has_pr || o.sys >= kSys || o.prn >= kPrn)
                continue;
            o.pr_l1 = false;
            Sat& s = sat_[o.sys][o.prn];
            if (((l1[o.sys] >> o.prn) & 1u) != 0)
            {
                // both codes: the difference noted
                const double pr1 = l1Code(e, o.sys, o.prn);
                note(s, o.pr_m - pr1, e.tow_s);
                continue;
            }
            // the L5 band's code alone: moved where the satellite's difference is good
            if (s.n == 0)
            {
                st_.none++;
                continue;
            }
            if (s.n < cfg_.min_n)
            {
                st_.too_new++;
                continue;
            }
            if (std::fabs(towDiff(e.tow_s, s.last_tow)) > cfg_.max_age_s)
            {
                st_.too_old++;
                continue;
            }
            o.pr_m -= s.off_m;
            o.pr_l1 = true;
            st_.moved++;
            moved++;
        }
        return moved;
    }

    // a satellite's kept difference (m) and how many epochs made it (tests, the console)
    bool offsetOf(int sys, int prn, double& off_m, uint32_t& n) const
    {
        if (sys < 0 || sys >= kSys || prn < 0 || prn >= kPrn || sat_[sys][prn].n == 0)
            return false;
        off_m = sat_[sys][prn].off_m;
        n = sat_[sys][prn].n;
        return true;
    }

private:
    struct Sat
    {
        double   off_m = 0.0;      // the mean L5 - L1 code difference
        double   last_tow = 0.0;   // the last epoch with both codes (GPS time of week, s)
        uint32_t n = 0;            // epochs since it (re)started; 0: none
    };

    static double towDiff(double a, double b)
    {
        double d = a - b;
        if (d > 302400.0)
            d -= 604800.0;
        else if (d < -302400.0)
            d += 604800.0;
        return d;
    }

    static double l1Code(const RawEpoch& e, int sys, int prn)
    {
        for (int j = 0; j < e.n; ++j)
        {
            const RawObs& p = e.obs[j];
            if (p.band == kBandL1 && p.has_pr && p.sys == sys && p.prn == prn)
                return p.pr_m;
        }
        return 0.0;
    }

    void note(Sat& s, double d, double tow)
    {
        const double dt = s.n > 0 ? std::fabs(towDiff(tow, s.last_tow)) : 0.0;
        if (s.n == 0 || dt > cfg_.max_age_s)
        {
            s.off_m = d;
            s.n = 1;
        }
        else if (std::fabs(d - s.off_m) > cfg_.max_step_m)
        {
            s.off_m = d;
            s.n = 1;
            st_.restarts++;
        }
        else
        {
            // a mean that forgets over tau_s: equal weights until it has that many epochs' worth
            const double a_time = cfg_.tau_s > 0.0 ? dt / cfg_.tau_s : 1.0;
            const double a_count = 1.0 / static_cast<double>(s.n + 1);
            const double a = a_count > a_time ? a_count : a_time;
            s.off_m += (a < 1.0 ? a : 1.0) * (d - s.off_m);
            if (s.n < 0xFFFFFFFFu)
                s.n++;
        }
        s.last_tow = tow;
    }

    CodeBridgeConfig cfg_;
    CodeBridgeStats  st_;
    Sat              sat_[kSys][kPrn];
};

}  // namespace gnss_raw

#endif  // GNSS_RAW_CODE_BRIDGE_H
