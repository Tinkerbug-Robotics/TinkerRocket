// TR_RawGnssEKF.h — a tightly coupled GNSS/INS filter: the flight filter's
// error states plus the receiver clock, updated by each satellite's
// pseudorange and range rate instead of the receiver's fix (#1600)
//
// The LC86G withholds its own fix from about T-1 s to several seconds after
// ignition and comes back tens of metres off; its raw measurements (RTCM
// MSM7) keep coming throughout. On the COCOM rig (2026-10-12, four flights)
// this filter held altitude to 0.3-3 m and vertical velocity to 0.01-0.4 m/s
// in every phase, where the receiver's fix was 3-91 m and 9-53 m/s off
// (docs/plans/1600-realtime-raw-gnss.md).
//
// A port of the bench prototype that scored it (gnss repo
// sim/tinkerrocket_gnss/tc_ekf.py, TcEkf), with its states, mechanization
// and update rules, in the flight filter's numerical form (TR_GpsInsEKF):
//
//   error state (19)
//     0-2   position, NED (m)
//     3-5   velocity, NED (m/s)
//     6-8   attitude, quaternion vector part (half-angle)
//     9-11  accelerometer bias (m/s^2)
//     12-14 gyro bias (rad/s)
//     15    receiver clock bias (m)
//     16    receiver clock drift (m/s)
//     17    Galileo - GPS time offset (m)
//     18    BeiDou - GPS time offset (m)
//
// Body FRD, world NED, q scalar-first with Quat2DCM(q) = NED->body, as
// TR_GpsInsEKF. The prototype's other states (the clock's g-sensitivity, the
// Doppler's clock-rate offset and lag, the carrier-phase clone) changed
// nothing on the LC86G's flights and are left out; its Doppler lag is a
// constant (rr_lag_s).
//
// Each pseudorange and range rate is a scalar update, gated on its own
// innovation (chi-square, 1 dof), so one bad satellite is dropped without
// losing the rest; a satellite refused reject_persist epochs running is let
// back in with its variance inflated to the gate. A time-correlated
// pseudorange error is fused with its variance scaled by (1+rho)/(1-rho),
// rho = exp(-dt/tau) since that satellite's last update. Before the
// updates, a step every satellite agrees on is taken by the clock: whole
// milliseconds (u-blox, SkyTraq) or not (the LC86G steers its clock to its
// first fix in one step: -2629 km and -71 m/s on the rig). GNSS corrections
// reach the attitude scaled by cos^4 pitch, as the flight filter's.
//
// An epoch can be applied after its time (it reaches the flight computer
// 0.1-0.25 s late): age_s says how long ago it was, and the receiver is
// taken back by the IMU's own velocity increments since (a ring of them,
// ~0.5 s). The Doppler is the range rate rr_lag_s before the epoch's tag
// (55.5 ms on the LC86G), predicted the same way.
//
// T is the covariance and attitude/velocity scalar: float on the flight
// computer (single-precision FPU), double for checking the port against the
// prototype. Position (lat/lon/height) and the clock bias stay double.
// Measurement prediction runs in double once per satellite per epoch.
// Pure and host-compilable (tests_cpp/test_raw_gnss_ekf.cpp).
#ifndef TR_RAW_GNSS_EKF_H
#define TR_RAW_GNSS_EKF_H

#include <cstdint>

#include "RawMeas.h"

namespace gnss_raw
{

struct RawGnssEKFParams
{
    // IMU noise model: the flight filter's defaults (TR_GpsInsEKF.h)
    double a_noise        = 0.20;      // m/s^2
    double a_markov_sigma = 0.01;
    double a_markov_tau   = 100.0;     // s
    double w_noise        = 0.00175;   // rad/s
    double w_markov_sigma = 0.00025;
    double w_markov_tau   = 50.0;
    double accel_var      = 0.25;      // pad levelling, (m/s^2)^2
    // The magnetometer as a heading (TR_GpsInsEKF's, #1578/#1304)
    double mag_sigma_ut   = 0.5;       // the field's noise per axis, uT
    double mag_tilt_sigma_mps2 = 0.05; // the accelerometer's, as a tilt reference
    double mag_r_floor    = 0.0027;    // the heading variance at least (calibration residuals), rad^2
    double mag_r_ceiling  = 0.06;      // and at most, before the magnitude penalty
    double mag_bias_nis   = 9.0;       // an innovation past this does not move the gyro bias
    // Receiver clock: white frequency (bias) and random-walk frequency
    // (drift) noise. The LC86G steers its clock with its own solution, and
    // its measurements carry drift steps of tens of m/s when that solution
    // breaks (+124 m/s within seconds of a 15 g ignition): 100, not a free
    // TCXO's 0.036.
    double clk_bias_psd   = 0.009;     // m^2/s
    double clk_drift_psd  = 100.0;     // m^2/s^3
    double isb_psd        = 1.0e-4;    // m^2/s: nearly constant
    // Raw measurement gating
    double gate_chi2      = 10.83;     // p = 0.001, 1 dof
    int    reject_persist = 3;         // refused this many epochs running: let back in
    // A step in every pseudorange (or range rate) at once is the receiver's clock
    double clock_step_m          = 1000.0;
    double clock_step_spread_m   = 200.0;
    double drift_step_mps        = 10.0;
    double drift_step_spread_mps = 2.0;
    // The Doppler is the range rate this long before the epoch's tag (LC86G
    // MSM7, fitted against the rig's truth in boost and coast, 2026-10-12)
    double rr_lag_s       = 0.0555;
    bool   att_gate_cos4  = true;      // false: GNSS corrections reach the attitude in full
    // GNSS corrections to the attitude about the vertical only (wins over
    // att_gate_cos4): the heading, and the gyro biases only along the
    // vertical -- across it they would tilt the attitude later. The tilt
    // stays the gyro's and the levelling's. For a filter that learns its
    // heading in flight (the heading fit's hand-off, RawGnssShadow).
    bool   att_gate_vertical = false;
    // Initial sigmas, in the state's units (the attitude's: a half-angle).
    // p0_att: the tilt's, about north and east; p0_hdg: the heading's, about
    // the vertical of init's attitude (#1626 N10). p0_hdg was pi ("any
    // heading", uNavINS's), on body z. About a tilted vertical, pi^2 beside
    // what the levelling pins (the tilt with the accelerometer bias, to 4e-6)
    // is past float's precision: on a still pad with no magnetometer the
    // attitude block went indefinite within two minutes up the rail and the
    // heading's sigma restarted at 0.2 deg. 0.5 (a heading 1-sigma of
    // 57 deg) held 30 minutes at every rail attitude tried, 75-90 deg
    // (UpTheRailTheLevelledStartKeepsItsHeadingFreeInFloat). At 10-60 deg a
    // still pad gives float out after 4-7 minutes with either start: there the
    // levelled tilt itself nears float's floor. The flight filter takes 30 deg
    // about the vertical on the pad (PAD_HEADING_SIGMA_DEG).
    double p0_pos = 10.0, p0_vel = 1.0, p0_att = 0.34906, p0_hdg = 0.5;
    double p0_abias = 0.981, p0_wbias = 0.01745;
    double p0_clk_bias = 30.0, p0_clk_drift = 5.0, p0_isb = 30.0;
    // Gravity: the flight filter's constant 9.807, or WGS84 normal gravity
    bool   gravity_wgs84  = false;
    // The velocity-increment history keeps an entry per this much time, s
    // (128 entries: 0.5 s at 4 ms; at a 200 Hz IMU, every sample)
    double hist_spacing_s = 0.004;
};

struct RawGnssStats
{
    int    used_pr = 0, used_rr = 0;
    int    rejected_pr = 0, rejected_rr = 0;
    int    clock_jump_ms = 0;          // a whole-millisecond clock step taken
    double clock_step_m = 0.0;         // a common pseudorange step that was not
    double drift_step_mps = 0.0;       // a common range-rate step
    float  nis_max = 0.0f;             // largest NIS among the updates applied
};

template <typename T>
class RawGnssEKF
{
public:
    static constexpr int N = 19;
    enum : int
    {
        kPos = 0, kVel = 3, kAtt = 6, kAccBias = 9, kGyroBias = 12,
        kClkBias = 15, kClkDrift = 16, kIsbGal = 17, kIsbBds = 18
    };
    static constexpr int kHist = 128;

    explicit RawGnssEKF(const RawGnssEKFParams& p = RawGnssEKFParams());
    void setParams(const RawGnssEKFParams& p);
    const RawGnssEKFParams& params() const { return p_; }
    // att_gate_vertical, switched in flight (the heading fit's act mode: from
    // launch -- on the pad nothing sees the heading for GNSS to turn)
    void setAttGateVertical(bool on) { p_.att_gate_vertical = on; }

    // Start at a position (lat, lon rad; ellipsoidal height m), NED velocity
    // and attitude, at filter time t0 (s; any clock the IMU steps share).
    // The clock is set by the first raw epoch (initClockFrom). The attitude's
    // covariance: p0_att about the two horizontal axes, p0_hdg about the
    // vertical of q (#1626 N10) -- so q is the best attitude there is (on the
    // pad, the accelerometer's tilt), not a placeholder.
    void init(const double lla[3], const T v_ned[3], const T q[4], double t0);
    // Take the attitude as known (the pad seed): its variance to 1e-6 and its
    // cross-covariances to zero, as GpsInsEKF::setQuaternion.
    void setQuaternion(const T q[4]);
    // The gyro bias from a still gyro's mean (the pad seed).
    void seedGyroBias(const T gyro_mean[3]);
    // The heading's own 1-sigma (rad, a rotation about the vertical), as a
    // magnetometer or the heading fit hands it over: the attitude's variance
    // about the vertical set to it, its cross-covariances dropped, the tilt's
    // kept. (setQuaternion takes the whole attitude as known: right for the
    // tilt the pad levels, not for a heading a filter must learn in flight.)
    void setHeadingSigma(T sigma_rad);
    // The tilt's 1-sigma about north and about east (rad, a rotation) at
    // least sigma_rad: each one's deficit added along it, so the part about
    // the vertical, the north-east covariance and every cross-covariance are
    // kept, P symmetric and positive definite. (An axis between north and
    // east can stay under the floor by that covariance: little, once the
    // floor is past the tilt the pad left.) For a tilt the pad could not tell
    // from an accelerometer bias (#1626 K1), let go at launch (RawGnssShadow's
    // heading_launch_tilt_sigma_deg).
    void setTiltSigmaFloor(T sigma_rad);
    // Turn the attitude about the vertical (NED down) by dpsi (rad): the
    // heading grows by dpsi, the tilt is kept.
    void turnHeading(T dpsi);
    // The IMU biases and their 6x6 covariance (accelerometer then gyro), in
    // place of the filter's own, their cross-covariances dropped: a heading
    // hand-off puts back what the pad left, before a wrong heading leaned on
    // them in flight.
    void setBiases(const T ab[3], const T wb[3], const T Pb[6][6]);
    void biasCovariance(T Pb[6][6]) const;
    // Let the velocity go: each axis's variance at least sigma^2 (m/s), its
    // cross-covariances dropped, its value kept.
    void releaseVelocity(T sigma);
    // Forget which satellites were refused (reject_persist's count): ones
    // refused under a wrong heading come back on their own merits.
    void clearRejects();

    // IMU time update: specific force (m/s^2) and rate (rad/s), body FRD.
    void propagate(const T acc[3], const T gyro[3], T dt);
    // Gravity-reference update on the pad (GpsInsEKF::accelMeasUpdate).
    void updateAccelLevel(const T acc[3]);
    // A height above the ellipsoid (m) measured with variance var (m^2): the
    // barometer, referred to the filter's own pad. No innovation gate: a
    // gate refuses the very barometer that would bring a drifted altitude
    // back (GpsInsEKF::baroMeasUpdate, #1579). It moves the vertical position
    // and velocity and the receiver clock only: a height says nothing about
    // the attitude, the body's biases or the horizontal.
    void updateHeight(double h_m, T var);
    // Zero velocity, sigma (m/s) per axis, into position and velocity only:
    // a velocity measurement cannot tell an attitude error from an
    // accelerometer bias (GpsInsEKF::zeroVelocityUpdate, #1418).
    void updateZeroVelocity(T sigma);
    // The magnetometer (body FRD, uT) as a heading, at any attitude: its
    // field carried into NED on the state's attitude turned onto the tilt
    // reference by the shortest arc -- the accelerometer's down when it
    // measures gravity (accel_is_gravity: the pad, the ground), else the
    // state's own -- and that azimuth set against magnetic north
    // (declination_rad, true north east of magnetic). field_err_ut: |B| less
    // the model's, a hard iron that size can turn the heading (#1304); < 0
    // unknown. False if the field's horizontal part is too weak to say
    // (TR_GpsInsEKF::magMeasUpdate, #1578: on a vertical rail too).
    bool updateMagHeading(const T acc[3], const T mag[3], bool accel_is_gravity, T declination_rad,
                          T field_err_ut);

    // Clock bias and drift from an epoch's medians at the current position;
    // false with no pseudorange. updateGnssRaw calls it until it succeeds.
    bool initClockFrom(const RawMeas* m, int n, double age_s = 0.0);
    // Every satellite's pseudorange and range rate of one epoch, which was
    // age_s ago.
    RawGnssStats updateGnssRaw(const RawMeas* m, int n, double age_s = 0.0);
    // The same in parts, for a filter that must keep stepping while it fuses
    // a sky: beginGnssRaw takes the epoch's receiver clock (its first value, a
    // step every satellite agrees on; false with no clock yet), then
    // updateGnssRawPart the satellites [i0, i1), each part at the age the
    // epoch has when it is fused. updateGnssRaw is the two at one age.
    bool beginGnssRaw(const RawMeas* m, int n, double age_s, RawGnssStats& st);
    void updateGnssRawPart(const RawMeas* m, int i0, int i1, double age_s, RawGnssStats& st);

    double time() const { return t_; }
    const double* lla() const { return lla_; }
    const T* velNed() const { return v_; }
    const T* quat() const { return q_; }
    const T* accelBias() const { return ab_; }
    const T* gyroBias() const { return wb_; }
    double clockBias() const { return clk_b_; }
    T clockDrift() const { return clk_d_; }
    T isb(int sys) const;                       // gnss_sys_t; GPS is the reference: 0
    bool clockReady() const { return clk_ready_; }
    T P(int i, int j) const { return P_[i][j]; }
    T sigma(int i) const;
    // The attitude's 1-sigma (rad) as rotations about north, east and down
    // (down: the heading): the attitude block, a body-frame half-angle,
    // doubled and carried into NED.
    void attSigmaNed(T out[3]) const;
    // |specific force| / g - 1 at the last IMU step, and its largest since
    // the last take (floored at 0): what RawMeasBuilder::build takes.
    T gExcess() const { return g_excess_; }
    T takeGPeak();
    // The receiver in ECEF (m, m/s).
    void receiverEcef(double r[3], double v[3]) const;
    // The receiver age_s ago (s), ECEF: now taken back by the history, as an
    // epoch that age is applied (#1508's satellite check: before an update,
    // the IMU's propagation from the last one).
    void receiverEcefAt(double age_s, double r[3], double v[3]) const;
    // What the IMU alone did between filter times t1 <= t2 (s), NED: its
    // velocity change (specific force and gravity, no filter corrections) and
    // the displacement that change adds, the integral of v(t) - v(t1). From a
    // state at t1, the IMU predicts p(t2) = p(t1) + v(t1) (t2 - t1) + dp.
    // False, with both zero, unless the history covers [t1, t2].
    bool imuMotion(double t1, double t2, T dv[3], T dp[3]) const;

private:
    struct Hist
    {
        double t;      // end of the entry's span, filter time
        T      dv[3];  // NED velocity increment over the span
        T      dp[3];  // NED displacement over it
    };

    void   gravityNed(T g[3]) const;
    // f_b: the specific force less the accelerometer bias, body (the tilt's
    // Jacobian is its, not the kinematic acceleration's: #1626 E1)
    void   propagateCovariance(const T Tb2n[3][3], const T f_b[3], const T w_b[3], T g_d, T dt);
    void   stabilize();
    void   symmetrize();
    T      hph(const int* hidx, const T* hval, int hn) const;
    // kmask: the states the gain may move (bit i, state i); the rest keep
    // their estimate and, Joseph form, a consistent covariance. att_axis: the
    // gain's attitude and gyro-bias rows projected onto this body-frame unit
    // vector (att_gate_vertical: the vertical).
    int    scalarUpdate(const int* hidx, const T* hval, int hn, T y, T R, T gate, T att,
                        T dx[N], T* nis, uint32_t kmask = 0xFFFFFFFFu, const T* att_axis = nullptr);
    void   inject(const T dx[N]);
    T      attGate() const;
    void   pushHistory(T dt, const T a_ned[3]);
    bool   historyChange(double tau, T dv[3], T dp[3]) const;
    void   receiverState(double r[3], double v[3], double Tm[3][3]) const;

    RawGnssEKFParams p_;
    // process noise, from p_
    T      q_acc_, q_att_, q_ab_, q_wb_, f_ab_, f_wb_, q_clkb_, q_clkd_, q_isb_;

    double lla_[3] = {0.0, 0.0, 0.0};
    T      v_[3] = {};
    T      q_[4] = {1, 0, 0, 0};
    T      ab_[3] = {};
    T      wb_[3] = {};
    double clk_b_ = 0.0;
    T      clk_d_ = 0;
    T      isb_[2] = {};
    T      P_[N][N] = {};
    double t_ = 0.0;
    bool   clk_ready_ = false;
    T      a_ned_[3] = {};
    T      g_excess_ = 0;
    T      g_peak_ = 0;

    Hist   hist_[kHist] = {};
    int    hhead_ = 0;        // next slot
    int    hn_ = 0;
    double hopen_ = 0.0;      // start of the newest entry's span

    // per satellite (gnss_sys_t x PRN) and kind (0 pseudorange, 1 range rate)
    uint8_t rejects_[3][64][2] = {};
    double  last_upd_[3][64][2] = {};
};

extern template class RawGnssEKF<float>;
extern template class RawGnssEKF<double>;

// The range from r (ECEF, m) to m's satellite, turned by the Earth's rotation
// over the signal's travel: what the pseudorange update predicts, less the
// receiver's clock (the pad survey's residuals, PadSurvey.h).
double sagnacRange(const RawMeas& m, const double r[3]);
// The range rate a receiver still at r (ECEF, m) would see from m's satellite
// -- its Sagnac-turned velocity along the line of sight, m/s -- and that line
// of sight (unit, ECEF, receiver to satellite), in double: what a Doppler
// velocity solve predicts (HeadingFit.h's own, in float, is held to it), and
// the synthetic skies' Doppler in its tests and bench.
double stillRangeRate(const RawMeas& m, const double r[3], double u[3]);

}  // namespace gnss_raw

#endif  // TR_RAW_GNSS_EKF_H
