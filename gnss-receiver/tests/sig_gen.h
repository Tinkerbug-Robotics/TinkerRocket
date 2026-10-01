// Test helper: synthetic L1-band signals in white noise: GPS L1 C/A; Galileo E1 (E1-B data and
// E1-C pilot with CS25, BOC(1,1), each at half power, pilot negated as in the ICD); BeiDou B1C as
// SignalSim makes it (pilot on I at sqrt(29/44) with its secondary code, data on -Q at 1/2, both
// sine BOC(1,1)).
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

extern "C" {
#include "gnss/sig.h"
#include "rng.h"
}

namespace siggen {

struct Sat {
    gnss_sig_t sig = GNSS_SIG_GPS_L1CA;  // or GNSS_SIG_GAL_E1C (E1-B + E1-C), GNSS_SIG_BDS_B1CP (B1C)
    int prn = 1;
    double dop = 0.0;          // Hz, carrier = if + dop
    double code_phase = 0.0;   // chips at sample 0
    double phase = 0.0;        // carrier phase at sample 0, rad
    double amp = 1.0;          // complex amplitude
    int data_ms = 0;           // >0: flip the data sign every data_ms code periods (bit edges at code epochs)
};

struct Codes {
    std::vector<uint8_t> main, data, sec;  // 0/1 chips: tracked code, data code, secondary code
    int len = 0;
    bool boc = false;
};

inline Codes codes_for(const Sat &s)
{
    Codes c;
    if (s.sig == GNSS_SIG_GAL_E1C) {
        c.len = GAL_E1_LEN;
        c.main.resize(c.len);
        c.data.resize(c.len);
        c.sec.resize(GAL_E1C_SEC_LEN);
        gal_e1_code(s.prn, GNSS_SIG_GAL_E1C, c.main.data());
        gal_e1_code(s.prn, GNSS_SIG_GAL_E1B, c.data.data());
        gal_e1c_secondary(c.sec.data());
        c.boc = true;
    } else if (s.sig == GNSS_SIG_BDS_B1CP) {
        c.len = BDS_B1C_LEN;
        c.main.resize(c.len);
        c.data.resize(c.len);
        c.sec.resize(BDS_B1C_SEC_LEN);
        bds_b1c_code(s.prn, GNSS_SIG_BDS_B1CP, c.main.data());
        bds_b1c_code(s.prn, GNSS_SIG_BDS_B1CD, c.data.data());
        bds_b1c_secondary(s.prn, c.sec.data());
        c.boc = true;
    } else {
        c.len = GPS_CA_LEN;
        c.main.resize(c.len);
        gps_ca_code(s.prn, c.main.data());
    }
    return c;
}

// n samples at fs, L1 at if_hz; noise sigma per component (0 = none).
inline std::vector<float> make(const std::vector<Sat> &sats, double fs, double if_hz, size_t n, double sigma,
                               uint64_t seed = 1)
{
    std::vector<float> v(2 * n, 0.0f);
    for (const Sat &s : sats) {
        const Codes cd = codes_for(s);
        const int len = cd.len;
        double rate = 1.023e6 * (1.0 + s.dop / GNSS_FREQ_L1_HZ);
        for (size_t k = 0; k < n; k++) {
            double t = double(k) / fs;
            double ph = s.code_phase + rate * t;
            double ep = std::floor(ph / len);  // whole code periods since the reference
            double frac = ph - ep * len;
            int chip = int(frac);
            double sc = (cd.boc && frac - chip >= 0.5) ? -1.0 : 1.0;  // sine BOC(1,1)
            double bit = (s.data_ms > 0 && (int64_t(ep) / s.data_ms) % 2 != 0) ? -1.0 : 1.0;
            double re, im = 0.0;
            if (s.sig == GNSS_SIG_GAL_E1C) {
                double sec = cd.sec[size_t(int64_t(ep) % GAL_E1C_SEC_LEN)] ? -1.0 : 1.0;
                double b = cd.data[chip] ? -1.0 : 1.0, c = cd.main[chip] ? -1.0 : 1.0;
                re = sc * (b * bit - c * sec) / std::sqrt(2.0);
            } else if (s.sig == GNSS_SIG_BDS_B1CP) {
                double sec = cd.sec[size_t(int64_t(ep) % BDS_B1C_SEC_LEN)] ? -1.0 : 1.0;
                double b = cd.data[chip] ? -1.0 : 1.0, c = cd.main[chip] ? -1.0 : 1.0;
                re = sc * std::sqrt(29.0 / 44.0) * c * sec;
                im = -sc * 0.5 * b * bit;
            } else {
                re = (cd.main[chip] ? -1.0 : 1.0) * bit;
            }
            double p = 2.0 * M_PI * (if_hz + s.dop) * t + s.phase;
            // (re + j im) * exp(j p)
            v[2 * k] += float(s.amp * (re * std::cos(p) - im * std::sin(p)));
            v[2 * k + 1] += float(s.amp * (re * std::sin(p) + im * std::cos(p)));
        }
    }
    if (sigma > 0) {
        rng_t r;
        rng_seed(&r, seed);
        rng_add_noise(&r, v.data(), n, sigma);
    }
    return v;
}

// Per-component sigma that puts a unit-amplitude signal at cn0 dB-Hz.
inline double sigma_for(double cn0, double fs, double amp = 1.0)
{
    return std::sqrt(amp * amp / std::pow(10.0, cn0 / 10.0) * fs / 2.0);
}

}  // namespace siggen
