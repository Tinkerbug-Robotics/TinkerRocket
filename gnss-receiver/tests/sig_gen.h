// Test helper: a synthetic GPS L1 C/A signal in white noise.
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
    int prn = 1;
    double dop = 0.0;          // Hz, carrier = if + dop
    double code_phase = 0.0;   // chips at sample 0
    double phase = 0.0;        // carrier phase at sample 0, rad
    double amp = 1.0;          // complex amplitude
    int data_ms = 0;           // >0: flip the sign every data_ms code periods (bit edges at code epochs)
};

// n samples at fs, L1 at if_hz; noise sigma per component (0 = none).
inline std::vector<float> make(const std::vector<Sat> &sats, double fs, double if_hz, size_t n, double sigma,
                               uint64_t seed = 1)
{
    std::vector<float> v(2 * n, 0.0f);
    for (const Sat &s : sats) {
        uint8_t chips[GPS_CA_LEN];
        gps_ca_code(s.prn, chips);
        double rate = 1.023e6 * (1.0 + s.dop / GNSS_FREQ_L1_HZ);
        for (size_t k = 0; k < n; k++) {
            double t = double(k) / fs;
            double ph = s.code_phase + rate * t;
            double ep = std::floor(ph / GPS_CA_LEN);        // whole code periods since the reference
            int chip = int(ph - ep * GPS_CA_LEN);
            double c = chips[chip] ? -1.0 : 1.0;
            if (s.data_ms > 0 && (int64_t(ep) / s.data_ms) % 2 != 0) {
                c = -c;
            }
            double p = 2.0 * M_PI * (if_hz + s.dop) * t + s.phase;
            v[2 * k] += float(s.amp * c * std::cos(p));
            v[2 * k + 1] += float(s.amp * c * std::sin(p));
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
