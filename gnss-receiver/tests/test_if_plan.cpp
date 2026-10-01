// The provisional frequency plan (fpga/model/fe_format.h) through the correlator's carrier mixer.

extern "C" {
#include "corr_model.h"
#include "fe_format.h"
}

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

namespace {

struct Eff {
    double mean_db, min_db, max_db;
    double dc_db;  // worst DC leakage: |mean(replica)| against the signal gain, dB
};

// What the carrier table keeps of the SNR against white noise over one 1 ms period:
// |mean(replica * carrier)|^2 / mean(|replica|^2), for the NCO word nearest f, from
// `starts` evenly spaced start phases. An exact mixer keeps 1 (0 dB).
Eff efficiency(const corr_model_cfg_t &c, double f, int starts)
{
    const int n = int(FE_FS_CORR_HZ / 1000.0);
    const auto w = uint32_t(int32_t(std::llround(f / FE_FS_CORR_HZ * 4294967296.0)));
    const int shift = 32 - c.lut_bits;
    const std::complex<double> step = std::polar(1.0, 2.0 * M_PI * double(int32_t(w)) / 4294967296.0);
    std::vector<double> db;
    double dc = 0.0;
    for (int s = 0; s < starts; s++) {
        uint32_t ph = uint32_t((uint64_t(s) << 32) / uint64_t(starts)) + 0x01234567u;  // off the sector edges
        std::complex<double> z = std::polar(1.0, 2.0 * M_PI * double(ph) / 4294967296.0), g = 0.0, r = 0.0;
        double p = 0.0;
        for (int k = 0; k < n; k++) {
            unsigned j = ph >> shift;
            double co = c.cos_lut[j], si = c.sin_lut[j];
            g += std::complex<double>(co, -si) * z;  // the model mixes with (cos - j sin)
            r += std::complex<double>(co, -si);       // what a DC offset in the samples becomes
            p += co * co + si * si;
            ph += w;
            z *= step;
        }
        db.push_back(10.0 * std::log10(std::norm(g) / (p * n)));
        dc = std::max(dc, std::abs(r) / std::abs(g));
    }
    double sum = 0.0;
    for (double v : db) {
        sum += v;
    }
    return {sum / double(db.size()), *std::min_element(db.begin(), db.end()), *std::max_element(db.begin(), db.end()),
            20.0 * std::log10(std::max(dc, 1e-12))};
}

}  // namespace

TEST(IfPlan, TheHardwareSessionsNumbers)
{
    EXPECT_NEAR(FE_LO_HZ, 1571.328052e6, 1.0);
    EXPECT_NEAR(FE_IF_ADC_HZ, 4.091948e6, 1.0);
    EXPECT_NEAR(FE_IF_HZ, -2.658052e6, 1.0);
}

// +-100 kHz around the IF, against white noise and against a DC offset in the samples. A table
// harmonic n of the NCO frequency f moves DC to n f; where that folds onto 0 Hz (f a simple ratio of
// fs) a DC offset reaches the correlators unspread. The 8-sector table's harmonics are n = 1 mod 4
// (1, -3, 5, -7, 9, ...), so 2 fs / 5, 42 kHz from the IF, folds DC by its 15th. Satellites sit
// inside +-15 kHz of the IF (Doppler, vehicle velocity and the reference's +-0.5 ppm together).
TEST(IfPlan, MixerAroundTheIf)
{
    corr_model_cfg_t c;
    corr_model_default_cfg(&c);
    const Eff ref = efficiency(c, FE_IF_HZ, 64);
    std::printf("  offset kHz   SNR mean dB   min dB   max dB   DC leak dB\n");
    double worst_in = 0.0, dc_in = -300.0, dc_out = -300.0, dc_out_at = 0.0;
    for (int k = -1000; k <= 1000; k++) {  // 100 Hz steps
        const double off = 100.0 * k;
        const Eff e = efficiency(c, FE_IF_HZ + off, k % 100 == 0 ? 64 : 8);
        const double dev = std::max(std::abs(e.min_db - ref.mean_db), std::abs(e.max_db - ref.mean_db));
        if (std::abs(off) <= 15e3) {
            worst_in = std::max(worst_in, dev);
            dc_in = std::max(dc_in, e.dc_db);
        } else if (e.dc_db > dc_out) {
            dc_out = e.dc_db;
            dc_out_at = off;
        }
        if (k % 100 == 0) {
            std::printf("  %+6.0f       %7.3f     %7.3f  %7.3f   %7.1f\n", off / 1e3, e.mean_db, e.min_db, e.max_db, e.dc_db);
        }
    }
    const double ratio = -0.4 * FE_FS_CORR_HZ;  // 2 fs / 5, folded: -2.7 MHz
    std::printf("  2 fs / 5 (%+.3f kHz from the IF), by offset from it:\n", (ratio - FE_IF_HZ) / 1e3);
    for (double d : {-1000.0, -100.0, -10.0, 0.0, 10.0, 100.0, 1000.0}) {
        const Eff e = efficiency(c, ratio + d, 64);
        std::printf("    %+6.0f Hz   %7.3f     %7.3f  %7.3f   %7.1f\n", d, e.mean_db, e.min_db, e.max_db, e.dc_db);
    }
    std::printf("  within +-15 kHz: SNR flat to %.4f dB, DC leakage <= %.1f dB; beyond, the worst DC leakage "
                "is %.1f dB at %+.1f kHz\n", worst_in, dc_in, dc_out, dc_out_at / 1e3);
    EXPECT_NEAR(ref.mean_db, -0.246, 0.002);  // the 8-sector table's loss against white noise
    EXPECT_LT(worst_in, 0.01);
    EXPECT_LT(dc_in, -40.0);
}
