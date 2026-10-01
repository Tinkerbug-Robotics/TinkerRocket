// Test helpers: complex tones and how well a stream matches one.
#pragma once

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

namespace tone {

constexpr double kTwoPi = 6.283185307179586476925286766559;

// n interleaved I,Q samples of a*exp(j*(2*pi*f/fs*(k + k0) + ph)).
inline std::vector<float> make(double f, double fs, size_t n, double a = 1.0, double ph = 0.0, double k0 = 0.0)
{
    std::vector<float> v(2 * n);
    for (size_t k = 0; k < n; k++) {
        double p = kTwoPi * f / fs * (double(k) + k0) + ph;
        v[2 * k] = float(a * std::cos(p));
        v[2 * k + 1] = float(a * std::sin(p));
    }
    return v;
}

struct Fit {
    std::complex<double> amp;  // least-squares complex amplitude of the tone
    double resid_db;           // residual power relative to the tone, dB
};

// Fits x[k] ~ amp * exp(j*2*pi*f/fs*(k + k0)) over samples [from, to).
inline Fit fit(const float *x, size_t from, size_t to, double f, double fs, double k0 = 0.0)
{
    std::complex<double> acc = 0.0;
    for (size_t k = from; k < to; k++) {
        double p = kTwoPi * f / fs * (double(k) + k0);
        acc += std::complex<double>(x[2 * k], x[2 * k + 1]) * std::polar(1.0, -p);
    }
    std::complex<double> amp = acc / double(to - from);
    double res = 0.0;
    for (size_t k = from; k < to; k++) {
        double p = kTwoPi * f / fs * (double(k) + k0);
        std::complex<double> e = std::complex<double>(x[2 * k], x[2 * k + 1]) - amp * std::polar(1.0, p);
        res += std::norm(e);
    }
    res /= double(to - from);
    return {amp, 10.0 * std::log10(res / std::norm(amp) + 1e-30)};
}

}  // namespace tone
