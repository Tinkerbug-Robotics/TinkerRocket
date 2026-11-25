// HeadingFit.h — the heading from thrust against GNSS velocity, as a batch
// fit, on the LC86G's raw Doppler.
//
// Near vertical, nothing the raw-GNSS filter fuses sees its heading: a GNSS
// velocity shows a tilt through the thrust but never a turn about it, and the
// magnetometer that gave the pad its heading is not used once the rocket has
// left. A heading error does show once the flight leans: the IMU's thrust,
// turned the wrong way about the vertical, sends the rocket somewhere its
// GNSS velocity says it did not go. This fits that turn.
//
// The model. The IMU's attitude runs on the gyro alone from a pad seed (the
// filter's attitude and gyro bias at the last still step before launch). Its
// specific force, integrated in that frame, is mapped onto the GNSS
// horizontal velocity by one rotation about vertical (the seed's heading
// error) and the accelerometer's two lateral biases, which tilt the pad's
// "up" through the seed and then add their own velocity in flight. For a
// launch from rest at t0, with C0 the seed's body-to-NED, dv = integral of
// C f dt and B = integral of C dt in the gyro-only frame, b the lateral
// biases and R(psi) a rotation about vertical:
//
//     v_h(t) = R(psi) [dv_h - (dv_D / g) (C0 b)_h - B_h b]
//
// The middle term is the tilt the bias put into the seed; on a still pad it
// cancels the last, as it must. psi is how far the seed's heading must turn
// to be right (the seed's heading error is -psi).
//
// The fit. For any psi the biases come in closed form from running sums (kept
// as the epochs come; an epoch cut is taken back out of them), so the cost over
// psi is a trigonometric polynomial of degree two: five numbers per fit,
// whatever the epochs. It is profiled on a 0.25 deg grid -- a coarse pass in
// float on the polynomial's own terms, then the grid around the best in
// double -- and refined by a parabola; its
// 1-sigma is where the profile rises by one (scaled up by the fit's own
// scatter when that is worse), none if that is not within 90 deg. The
// profile can have a second valley (the biases taking up a turn): its rise
// is the fit's rival, the margin by which the data prefer this heading to
// another one far off -- which the sigma, a curvature, does not say. Epochs out
// of a 2-dof gate are cut, worst first and a quarter at most, refitting each
// time. An epoch counts once the horizontal speed reaches min_speed_mps (below
// it the velocity's direction is noise: a vertical flight has no heading to
// give), and epochs closer than tau_s share one epoch's weight (the IMU's
// errors are the same from one to the next).
//
// GNSS here is the raw Doppler: each epoch's velocity a weighted least-squares
// solve of its range rates with one clock drift (dopplerVelocity), placed at
// the instant the Doppler describes. It flows through the boost, where the
// receiver withholds its own fix.
//
// A port of the bench prototype (gnss repo sim/tinkerrocket_gnss/heading_fit.py:
// solveGrid() is its 1,440-point profile on sums taken afresh, all double --
// the check on solve()). Its integrator is float, as the filter's; its sums
// and the profile near its minimum are double; the Doppler velocity is float
// after one double difference. On the ESP32-S3, where double arithmetic is
// software, that is what keeps an epoch's part under a slice (HeadingFitBench.h).
// Pure C++ (tests_cpp/test_heading_fit.cpp).
#ifndef HEADING_FIT_H
#define HEADING_FIT_H

#include <cstdint>

#include "RawMeas.h"

namespace gnss_raw
{

class HeadingFit
{
public:
    struct Config
    {
        double bias_sigma_mps2 = 0.09807;   // the prior on each lateral accelerometer bias (10 mg)
        double drift_mps2      = 0.02;      // unmodelled IMU error: an epoch's sigma grows by drift * (t - t0)
        double cut_chi2        = 13.8;      // an epoch beyond this (2 dof, p = 0.001) is cut
        double min_speed_mps   = 1.0;       // an epoch counts from this horizontal speed on
        double tau_s           = 1.0;       // epochs closer than this share one epoch's weight
    };

    struct Result
    {
        bool   ok = false;       // two epochs or more
        double psi = 0.0;        // rad, in [-pi, pi): the turn about vertical that rights the seed
        double sigma = -1.0;     // its 1-sigma, rad (a whole number of grid steps); < 0: not within 90 deg
        double rival = -1.0;     // the profile's other valley, if it has one: how far its floor rises
                                 //   above this one, as the sigma's (one is one sigma's); < 0: none
        double bias[2] = {};     // the lateral accelerometer biases, body y and z, m/s^2
        int    n = 0;            // epochs used
        int    n_cut = 0;        //   and cut
        double rms = 0.0;        // the residuals' weighted RMS per axis
        int    evals = 0;        // profile points evaluated, refits included (the cost)
    };

    static constexpr int    kMaxEpochs = 64;      // more are not taken
    static constexpr int    kHist      = 256;     // integrals kept: 2.56 s at the navigation filter's 100 Hz
    static constexpr int    kGrid      = 1440;    // 0.25 deg
    static constexpr double kG         = 9.807;   // the flight filter's G_NOMINAL

    HeadingFit() : HeadingFit(Config()) {}
    explicit HeadingFit(const Config& c) : cfg_(c) {}
    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // Start (again) at rest at time t0 (s), from the seed's attitude (body FRD
    // -> NED, scalar first: Quat2DCM(q) is NED -> body) and the still gyro's
    // mean (rad/s). Every epoch taken so far is forgotten.
    void start(const float q_seed[4], const float gyro_bias[3], double t0);
    void reset();
    bool started() const { return started_; }
    double t0() const { return t0_; }
    double time() const { return t_; }

    // One IMU step of dt (s) ending now: the specific force (m/s^2) and rate
    // (rad/s), body FRD.
    void imu(const float acc[3], const float gyro[3], float dt);

    // One epoch: the horizontal GNSS velocity (north, east, m/s) at time t,
    // var_h its per-axis variance ((m/s)^2). False if it does not count:
    // before the start or after the last step, older than the history, too
    // slow, or the store full.
    bool gnss(double t, const double v_ne[2], double var_h);
    int epochs() const { return nobs_; }

    // The fit on the epochs so far (ok false with fewer than two).
    Result solve() const { return solveWith(false); }
    // The same profiled on all 1,440 grid points, as the prototype.
    Result solveGrid() const { return solveWith(true); }

    // The fit's attitude now: the gyro-only attitude turned psi about the vertical.
    void attitude(double psi, float q[4]) const;
    const float* gyroOnlyQuat() const { return q_; }

private:
    struct Obs
    {
        double t;
        float  y[2];       // the GNSS horizontal velocity
        float  x[2];       // dv_h at the epoch
        float  M[2][2];    // the biases' map: -(dv_D / g) C0_hL - B_hL
        float  w;          // the epoch's weight, shared with its neighbours
        float  w_raw;      //   its own (the cut's)
    };
    struct Hist
    {
        double t;
        float  dv[3];
        float  BL[2][2];   // B's north and east rows at the lateral (body y, z) columns
    };
    // cost(psi) = a0 + a1 c + a2 s + a3 c^2 + a4 s^2 + a5 c s, c = cos psi, s = sin psi
    // the sums over the epochs (the biases' prior apart)
    struct Sums
    {
        double A[2][2];
        double Su[2], Sp[2], Sx[2];
        double uu, ux, px, xx;
        int    n;
    };
    struct Profile
    {
        double a[6];
        double Ai[2][2];
        double Su[2], Sp[2], Sx[2];
        double eval(double c, double s) const
        {
            return a[0] + a[1] * c + a[2] * s + a[3] * c * c + a[4] * s * s + a[5] * c * s;
        }
    };

    bool   at(double t, float dv[3], float BL[2][2]) const;
    static void accumulate(Sums& s, const Obs& o, double sign);
    void   sumsOf(const bool* keep, Sums& s) const;
    void   profile(const Sums& s, Profile& p) const;
    bool   fit(const Sums& s, bool full_grid, Result& r, double cs[2]) const;   // cs: cos, sin of r.psi
    double chi2(const Obs& o, const Result& r, double c, double s) const;
    Result solveWith(bool full_grid) const;

    Config cfg_;
    bool   started_ = false;
    double t0_ = 0.0;
    double t_ = 0.0;
    float  q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float  wb_[3] = {};
    float  C0L_[2][2] = {};    // the seed's body-to-NED: north and east rows, lateral columns
    float  dv_[3] = {};
    float  BL_[2][2] = {};
    Hist   hist_[kHist] = {};
    int    hhead_ = 0;         // the next slot
    int    hn_ = 0;
    Obs    obs_[kMaxEpochs] = {};
    int    nobs_ = 0;
    Sums   sum_ = {};              // over every epoch taken
    bool   have_last_ = false;
    double t_last_ = 0.0;
};

// One epoch's receiver velocity (ECEF, m/s) from its range rates, weighted by
// each one's sigma, with one clock drift: the worst satellite is dropped while
// its normalized residual exceeds cut. r: the receiver (ECEF, m). False with
// fewer than min_sats left; else v, its covariance and the satellites used.
bool dopplerVelocity(const RawMeas* m, int n, const double r[3], double v[3], double cov[3][3], int& used,
                     int min_sats = 5, double cut = 4.0);

// The rotation about NED down (rad, in [-pi, pi)) that takes attitude b to
// attitude a -- a = Rz b when they differ only in heading: a's heading less
// b's, whatever their tilts (the twist of a b^-1 about down). Attitudes body
// -> NED, scalar first.
double headingBetween(const float a[4], const float b[4]);

// The same turned into NED at lat, lon (rad): the velocity, and the mean of
// its north and east variances (the fit's var_h).
bool dopplerVelocityNed(const RawMeas* m, int n, const double r[3], double lat, double lon, double v_ned[3],
                        double& var_h, int& used, int min_sats = 5, double cut = 4.0);

}  // namespace gnss_raw

#endif  // HEADING_FIT_H
