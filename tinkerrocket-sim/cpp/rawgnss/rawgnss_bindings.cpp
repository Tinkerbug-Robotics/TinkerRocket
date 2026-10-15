// Python bindings for the raw-GNSS (tightly coupled) filter of #1600: the
// LC86G's RTCM through the flight code's own parser (Lc86Parser), ephemeris
// store and measurement builder (TR_GNSSRaw), and the filter (TR_RawGnssEKF)
// in its float (flight) and double instantiations. The gnss repo's bench
// scorer (sim/tc_ekf_cocom.py, mode ins-cpp) runs it beside the prototype it
// was ported from. And the heading from thrust (TR_RawGnssShadow's
// HeadingFit, with the filter's heading gate and hand-off), for the gnss
// repo's heading runs (sim/heading_seed_sim.py) to fly the port.
//
// The receivers' profiles (ReceiverProfile.h) and their application to a
// builder or a filter; the carrier phase in an observation (phase_m,
// halfcyc), the carrier range rate in a measurement (cr_dphi_m, cr_dt,
// sat_pos_prev, sigma_cr), the filter's carrier update in its parts and its
// stats -- so a log replays offline through the flight code with the carrier
// off, observed or fused.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "EphemerisStore.h"
#include "HeadingFit.h"
#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
#include "ReceiverProfile.h"
#include "ReceiverProfileApply.h"
#include "Rtcm3Msm.h"
#include "RtcmEphemeris.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"

namespace py = pybind11;
using namespace gnss_raw;

namespace
{

using Vec3 = std::array<double, 3>;

Vec3 vec3(const double* p) { return {p[0], p[1], p[2]}; }

// The LC86G's byte stream as the flight computer reads it: RTCM3 frames
// through Lc86Parser, MSM7 epochs out, ephemeris into a store.
struct Lc86FrontEnd
{
    lc86::Lc86Parser parser;
    EphemerisStore store;
    int near_week = 2440;
    int eph_frames = 0, eph_added = 0;

    std::vector<RawEpoch> feed(const uint8_t* data, size_t n)
    {
        std::vector<RawEpoch> out;
        for (size_t i = 0; i < n; ++i)
        {
            const lc86::Event ev = parser.feed(data[i]);
            if (ev == lc86::Event::RTCM)
            {
                const uint8_t* payload = parser.rtcmFrame() + 3;
                const size_t len = parser.rtcmFrameLen() - rtcm3::kFrameOverhead;
                if (rtcm3::isEphemeris(rtcm3::messageType(payload, len)))
                {
                    eph_frames++;
                    gps_eph_t e;
                    if (decodeEphemeris(payload, len, near_week, e) && store.add(e))
                        eph_added++;
                }
            }
            else if (ev == lc86::Event::MSM_EPOCH)
            {
                rtcm3::MsmEpoch me;
                if (parser.takeMsmEpoch(me))
                {
                    RawEpoch ep;
                    if (lc86::rawEpochFromMsm(me, ep))
                        out.push_back(ep);
                }
            }
        }
        return out;
    }

    // One RTCM3 payload, framed as the module sends it.
    std::vector<RawEpoch> feedPayload(const py::bytes& b)
    {
        const std::string s = b;
        std::vector<uint8_t> f(s.size() + rtcm3::kFrameOverhead);
        f[0] = rtcm3::kPreamble;
        f[1] = static_cast<uint8_t>((s.size() >> 8) & 0x03);
        f[2] = static_cast<uint8_t>(s.size() & 0xFF);
        std::copy(s.begin(), s.end(), f.begin() + 3);
        const uint32_t crc = rtcm3::crc24q(f.data(), s.size() + 3);
        f[s.size() + 3] = static_cast<uint8_t>(crc >> 16);
        f[s.size() + 4] = static_cast<uint8_t>(crc >> 8);
        f[s.size() + 5] = static_cast<uint8_t>(crc);
        return feed(f.data(), f.size());
    }
};

// Each observation's carrier phase: (phase_m | None, halfcyc).
py::list obsCarrierList(const RawEpoch& ep)
{
    py::list l;
    for (int i = 0; i < ep.n; ++i)
    {
        const RawObs& o = ep.obs[i];
        l.append(py::make_tuple(o.has_phase ? py::object(py::float_(o.phase_m)) : py::object(py::none()),
                                o.halfcyc));
    }
    return l;
}

py::list obsList(const RawEpoch& ep)
{
    py::list l;
    for (int i = 0; i < ep.n; ++i)
    {
        const RawObs& o = ep.obs[i];
        l.append(py::make_tuple(o.sys, o.prn, o.has_pr ? py::object(py::float_(o.pr_m)) : py::object(py::none()),
                                o.has_rate ? py::object(py::float_(o.rate_mps)) : py::object(py::none()),
                                o.cn0_dbhz, o.lock_ms));
    }
    return l;
}

std::vector<RawMeas> toVector(const py::list& l)
{
    std::vector<RawMeas> v;
    for (auto h : l)
        v.push_back(h.cast<RawMeas>());
    return v;
}

py::dict statsDict(const RawGnssStats& s)
{
    py::dict d;
    d["used_pr"] = s.used_pr;
    d["used_rr"] = s.used_rr;
    d["rejected_pr"] = s.rejected_pr;
    d["rejected_rr"] = s.rejected_rr;
    d["clock_jump_ms"] = s.clock_jump_ms;
    d["clock_step_m"] = s.clock_step_m;
    d["drift_step_mps"] = s.drift_step_mps;
    d["nis_max"] = s.nis_max;
    d["used_cr"] = s.used_cr;
    d["cr_built"] = s.cr_built;
    d["cr_unmatched"] = s.cr_unmatched;
    d["cr_in"] = s.cr_in;
    d["cr_half"] = s.cr_half;
    d["cr_repaired"] = s.cr_repaired;
    d["cr_other"] = s.cr_other;
    d["cr_too_few"] = s.cr_too_few;
    d["cr_rms_mps"] = s.cr_rms_mps;
    d["cr_fit_rms_mps"] = s.cr_fit_rms_mps;
    d["cr_dv_mps"] = s.cr_dv_mps;
    d["cr_roll_rev_s"] = s.cr_roll_rev_s;
    d["cr_common_mps"] = s.cr_common_mps;
    return d;
}

py::list carrierInfoList(const CarrierInfo* ci, int n)
{
    static const char* kVerdict[] = {"in", "repaired", "half_cycle", "other", "unmatched", "too_few"};
    py::list l;
    for (int i = 0; i < n; ++i)
    {
        py::dict d;
        d["idx"] = ci[i].idx;
        d["sys"] = ci[i].sys;
        d["prn"] = ci[i].prn;
        d["verdict"] = (ci[i].verdict >= 0 && ci[i].verdict <= 5) ? kVerdict[ci[i].verdict] : "?";
        d["half"] = static_cast<int>(ci[i].half);
        d["sigma"] = ci[i].sigma;
        d["y"] = ci[i].y;
        l.append(d);
    }
    return l;
}

template <typename T, size_t K>
std::vector<T> arrayList(const T (&a)[K])
{
    return std::vector<T>(a, a + K);
}

template <typename T, size_t K>
void arraySet(T (&a)[K], const std::vector<T>& v)
{
    if (v.size() != K)
        throw py::value_error("expected " + std::to_string(K) + " values (one per gnss_sys_t)");
    for (size_t k = 0; k < K; ++k)
        a[k] = v[k];
}

template <typename T>
void bindEkf(py::module_& m, const char* name)
{
    using E = RawGnssEKF<T>;
    using A3 = std::array<double, 3>;
    using A4 = std::array<double, 4>;
    py::class_<E>(m, name)
        .def(py::init<const RawGnssEKFParams&>(), py::arg("params") = RawGnssEKFParams())
        .def_property("params", &E::params, &E::setParams)
        .def("init", [](E& e, const A3& lla, const A3& v, const A4& q, double t0) {
            const T vv[3] = {T(v[0]), T(v[1]), T(v[2])};
            const T qq[4] = {T(q[0]), T(q[1]), T(q[2]), T(q[3])};
            e.init(lla.data(), vv, qq, t0);
        }, py::arg("lla"), py::arg("v_ned"), py::arg("q"), py::arg("t0"))
        .def("set_quaternion", [](E& e, const A4& q) {
            const T qq[4] = {T(q[0]), T(q[1]), T(q[2]), T(q[3])};
            e.setQuaternion(qq);
        })
        .def("seed_gyro_bias", [](E& e, const A3& g) {
            const T gg[3] = {T(g[0]), T(g[1]), T(g[2])};
            e.seedGyroBias(gg);
        })
        .def("propagate", [](E& e, const A3& acc, const A3& gyro, double dt) {
            const T a[3] = {T(acc[0]), T(acc[1]), T(acc[2])};
            const T w[3] = {T(gyro[0]), T(gyro[1]), T(gyro[2])};
            e.propagate(a, w, T(dt));
        })
        .def("update_accel_level", [](E& e, const A3& acc) {
            const T a[3] = {T(acc[0]), T(acc[1]), T(acc[2])};
            e.updateAccelLevel(a);
        })
        .def("init_clock_from", [](E& e, const py::list& meas, double age) {
            auto v = toVector(meas);
            return e.initClockFrom(v.data(), static_cast<int>(v.size()), age);
        }, py::arg("meas"), py::arg("age_s") = 0.0)
        .def("update_gnss_raw", [](E& e, const py::list& meas, double age) {
            auto v = toVector(meas);
            return statsDict(e.updateGnssRaw(v.data(), static_cast<int>(v.size()), age));
        }, py::arg("meas"), py::arg("age_s") = 0.0,
           "The whole epoch: its clock, every pseudorange and range rate, and its carrier range rates in the "
           "params' cr_mode (off, observe, fuse); the epoch's stats")
        // an epoch in parts, as the flight computer works it (the stats carried between)
        .def("begin_gnss_raw", [](E& e, const py::list& meas, double age, RawGnssStats& st) {
            auto v = toVector(meas);
            return e.beginGnssRaw(v.data(), static_cast<int>(v.size()), age, st);
        }, py::arg("meas"), py::arg("age_s"), py::arg("stats"))
        .def("update_gnss_raw_part", [](E& e, const py::list& meas, int i0, int i1, double age, RawGnssStats& st) {
            auto v = toVector(meas);
            e.updateGnssRawPart(v.data(), i0, i1, age, st);
        }, py::arg("meas"), py::arg("i0"), py::arg("i1"), py::arg("age_s"), py::arg("stats"))
        .def("begin_carrier", [](E& e, const py::list& meas, double age, RawGnssStats& st) {
            auto v = toVector(meas);
            return e.beginCarrier(v.data(), static_cast<int>(v.size()), age, st);
        }, py::arg("meas"), py::arg("age_s"), py::arg("stats"),
           "The carrier rates screened at the prior (observe: all there is); how many fuse_carrier will fuse")
        .def("fuse_carrier", [](E& e, const py::list& meas, RawGnssStats& st) {
            auto v = toVector(meas);
            e.fuseCarrier(v.data(), st);
        }, py::arg("meas"), py::arg("stats"), "The same measurements begin_carrier screened")
        .def("end_carrier", &E::endCarrier, py::arg("stats"), "Once an epoch: the common rate dropped")
        .def("carrier_info", [](const E& e) {
            int n = 0;
            const CarrierInfo* ci = e.carrierInfo(n);
            return carrierInfoList(ci, n);
        }, "The last screen's rates: idx, sys, prn, verdict, half, sigma, y (m/s, before the common rate)")
        .def("update_height", [](E& e, double h, double var) { e.updateHeight(h, T(var)); }, py::arg("h_m"),
             py::arg("var_m2"))
        .def("update_zero_velocity", [](E& e, double s) { e.updateZeroVelocity(T(s)); }, py::arg("sigma_mps"))
        .def_property_readonly("t", &E::time)
        .def_property_readonly("lla", [](const E& e) { return vec3(e.lla()); })
        .def_property_readonly("v", [](const E& e) {
            return A3{double(e.velNed()[0]), double(e.velNed()[1]), double(e.velNed()[2])};
        })
        .def_property_readonly("q", [](const E& e) {
            const T* q = e.quat();
            return A4{double(q[0]), double(q[1]), double(q[2]), double(q[3])};
        })
        .def_property_readonly("ab", [](const E& e) {
            return A3{double(e.accelBias()[0]), double(e.accelBias()[1]), double(e.accelBias()[2])};
        })
        .def_property_readonly("wb", [](const E& e) {
            return A3{double(e.gyroBias()[0]), double(e.gyroBias()[1]), double(e.gyroBias()[2])};
        })
        .def_property_readonly("clk_bias", &E::clockBias)
        .def_property_readonly("clk_drift", [](const E& e) { return double(e.clockDrift()); })
        .def_property_readonly("isb_gal", [](const E& e) { return double(e.isb(GNSS_SYS_GAL)); })
        .def_property_readonly("isb_bds", [](const E& e) { return double(e.isb(GNSS_SYS_BDS)); })
        .def_property_readonly("clock_ready", &E::clockReady)
        .def_property_readonly("g_excess", [](const E& e) { return double(e.gExcess()); })
        .def("take_g_peak", [](E& e) { return double(e.takeGPeak()); })
        .def_property_readonly("P", [](const E& e) {
            py::array_t<double> a({E::N, E::N});
            auto r = a.template mutable_unchecked<2>();
            for (int i = 0; i < E::N; ++i)
                for (int j = 0; j < E::N; ++j)
                    r(i, j) = double(e.P(i, j));
            return a;
        })
        .def("receiver_ecef", [](const E& e) {
            double r[3], v[3];
            e.receiverEcef(r, v);
            return py::make_tuple(vec3(r), vec3(v));
        })
        // the heading fit's hand-off (RawGnssShadow::handOverHeading)
        .def("set_heading_sigma", [](E& e, double s) { e.setHeadingSigma(T(s)); }, py::arg("sigma_rad"))
        .def("turn_heading", [](E& e, double d) { e.turnHeading(T(d)); }, py::arg("dpsi_rad"))
        .def("set_att_gate_vertical", &E::setAttGateVertical)
        .def("set_att_gate_cos4", &E::setAttGateCos4)
        .def("set_tilt_sigma_floor", [](E& e, double s) { e.setTiltSigmaFloor(T(s)); }, py::arg("sigma_rad"))
        .def("set_biases", [](E& e, const A3& ab, const A3& wb, py::array_t<double> Pb) {
            auto r = Pb.template unchecked<2>();
            T P[6][6];
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j)
                    P[i][j] = T(r(i, j));
            const T a[3] = {T(ab[0]), T(ab[1]), T(ab[2])}, w[3] = {T(wb[0]), T(wb[1]), T(wb[2])};
            e.setBiases(a, w, P);
        })
        .def("bias_covariance", [](const E& e) {
            T P[6][6];
            e.biasCovariance(P);
            py::array_t<double> a({6, 6});
            auto r = a.template mutable_unchecked<2>();
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j)
                    r(i, j) = double(P[i][j]);
            return a;
        })
        .def("release_velocity", [](E& e, double s) { e.releaseVelocity(T(s)); }, py::arg("sigma_mps"))
        .def("clear_rejects", &E::clearRejects)
        .def("att_sigma_ned", [](const E& e) {
            T s[3];
            e.attSigmaNed(s);
            return A3{double(s[0]), double(s[1]), double(s[2])};
        });
}

py::dict fitDict(const HeadingFit::Result& r)
{
    py::dict d;
    d["ok"] = r.ok;
    d["psi"] = r.psi;
    d["sigma"] = r.sigma > 0.0 ? py::object(py::float_(r.sigma)) : py::object(py::none());
    d["rival"] = r.rival >= 0.0 ? py::object(py::float_(r.rival)) : py::object(py::none());
    d["bias"] = py::make_tuple(r.bias[0], r.bias[1]);
    d["n"] = r.n;
    d["n_cut"] = r.n_cut;
    d["rms"] = r.rms;
    d["evals"] = r.evals;
    return d;
}

}  // namespace

PYBIND11_MODULE(_rawgnss, m)
{
    m.doc() = "Raw-GNSS tightly coupled filter (#1600): TR_GNSSRaw + TR_RawGnssEKF + the LC86G parser";

    py::class_<RawMeas>(m, "RawMeas")
        .def(py::init([](int sys, int prn, const Vec3& sat_pos, const Vec3& sat_vel, const Vec3& sat_acc,
                         py::object pr, py::object rr, float sigma_pr, float sigma_pr_corr, float tau_pr,
                         float sigma_rr, float tau_rr, py::object cr_dphi_m, float cr_dt, py::object sat_pos_prev,
                         float sigma_cr) {
                 RawMeas x{};
                 x.has_cr = !cr_dphi_m.is_none();
                 if (x.has_cr)
                 {
                     if (sat_pos_prev.is_none())
                         throw py::value_error("a carrier range rate needs sat_pos_prev");
                     x.cr_dphi_m = cr_dphi_m.cast<double>();
                     x.cr_dt = cr_dt;
                     x.sigma_cr = sigma_cr;
                     const Vec3 sp = sat_pos_prev.cast<Vec3>();
                     for (int k = 0; k < 3; ++k)
                         x.sat_pos_prev[k] = sp[k];
                 }
                 x.sys = static_cast<uint8_t>(sys);
                 x.prn = static_cast<uint8_t>(prn);
                 for (int k = 0; k < 3; ++k)
                 {
                     x.sat_pos[k] = sat_pos[k];
                     x.sat_vel[k] = sat_vel[k];
                     x.sat_acc[k] = sat_acc[k];
                 }
                 x.has_pr = !pr.is_none();
                 x.pr = x.has_pr ? pr.cast<double>() : 0.0;
                 x.has_rr = !rr.is_none();
                 x.rr = x.has_rr ? rr.cast<double>() : 0.0;
                 x.sigma_pr = sigma_pr;
                 x.sigma_pr_corr = sigma_pr_corr;
                 x.tau_pr = tau_pr;
                 x.sigma_rr = sigma_rr;
                 x.tau_rr = tau_rr;
                 return x;
             }),
             py::arg("sys"), py::arg("prn"), py::arg("sat_pos"), py::arg("sat_vel"), py::arg("sat_acc"),
             py::arg("pr"), py::arg("rr"), py::arg("sigma_pr"), py::arg("sigma_pr_corr"), py::arg("tau_pr"),
             py::arg("sigma_rr"), py::arg("tau_rr"), py::arg("cr_dphi_m") = py::none(), py::arg("cr_dt") = 1.0f,
             py::arg("sat_pos_prev") = py::none(), py::arg("sigma_cr") = 0.008f,
             "cr_dphi_m (with sat_pos_prev, the satellite at the earlier epoch): a carrier range rate over cr_dt")
        .def_readonly("sys", &RawMeas::sys)
        .def_readonly("prn", &RawMeas::prn)
        .def_property_readonly("sat_pos", [](const RawMeas& x) { return vec3(x.sat_pos); })
        .def_property_readonly("sat_vel", [](const RawMeas& x) { return vec3(x.sat_vel); })
        .def_property_readonly("sat_acc", [](const RawMeas& x) { return vec3(x.sat_acc); })
        .def_property_readonly("pr", [](const RawMeas& x) { return x.has_pr ? py::object(py::float_(x.pr)) : py::object(py::none()); })
        .def_property_readonly("rr", [](const RawMeas& x) { return x.has_rr ? py::object(py::float_(x.rr)) : py::object(py::none()); })
        .def_readonly("sigma_pr", &RawMeas::sigma_pr)
        .def_readonly("sigma_pr_corr", &RawMeas::sigma_pr_corr)
        .def_readonly("tau_pr", &RawMeas::tau_pr)
        .def_readonly("sigma_rr", &RawMeas::sigma_rr)
        .def_readonly("tau_rr", &RawMeas::tau_rr)
        .def_readonly("has_cr", &RawMeas::has_cr)
        .def_property_readonly("cr_dphi_m", [](const RawMeas& x) {
            return x.has_cr ? py::object(py::float_(x.cr_dphi_m)) : py::object(py::none());
        })
        .def_readonly("cr_dt", &RawMeas::cr_dt)
        .def_property_readonly("sat_pos_prev", [](const RawMeas& x) { return vec3(x.sat_pos_prev); })
        .def_readonly("sigma_cr", &RawMeas::sigma_cr);

    py::class_<RawEpoch>(m, "RawEpoch")
        .def(py::init([](double tow, const py::list& obs) {
                 RawEpoch ep;
                 ep.tow_s = tow;
                 for (auto h : obs)
                 {
                     if (ep.n >= RawEpoch::kMaxObs)
                         break;
                     auto t = h.cast<py::tuple>();
                     RawObs& o = ep.obs[ep.n++];
                     o.sys = t[0].cast<uint8_t>();
                     o.prn = t[1].cast<uint8_t>();
                     o.has_pr = !t[2].is_none();
                     o.pr_m = o.has_pr ? t[2].cast<double>() : 0.0;
                     o.has_rate = !t[3].is_none();
                     o.rate_mps = o.has_rate ? t[3].cast<double>() : 0.0;
                     o.cn0_dbhz = t[4].cast<float>();
                     o.lock_ms = t.size() > 5 ? t[5].cast<uint32_t>() : 0u;
                     o.has_phase = t.size() > 6 && !t[6].is_none();
                     o.phase_m = o.has_phase ? t[6].cast<double>() : 0.0;
                     o.halfcyc = t.size() > 7 ? t[7].cast<bool>() : false;
                 }
                 return ep;
             }),
             py::arg("tow_s"), py::arg("obs"),
             "obs: [(sys, prn, pr_m | None, rate_mps | None, cn0[, lock_ms[, phase_m | None[, halfcyc]]])], "
             "sys as gnss_sys_t")
        .def_readonly("tow_s", &RawEpoch::tow_s)
        .def_property_readonly("obs", &obsList)
        .def_property_readonly("obs_carrier", &obsCarrierList, "each observation's (phase_m | None, halfcyc)");

    py::class_<EphemerisStore>(m, "EphemerisStore")
        .def(py::init<>())
        .def("add_rtcm", [](EphemerisStore& s, const py::bytes& b, int near_week) {
            const std::string p = b;
            gps_eph_t e;
            return decodeEphemeris(reinterpret_cast<const uint8_t*>(p.data()), p.size(), near_week, e) && s.add(e);
        }, py::arg("payload"), py::arg("near_gps_week") = 2440)
        .def("size", &EphemerisStore::size)
        .def("satellites", &EphemerisStore::satellites);

    py::class_<Lc86FrontEnd>(m, "Lc86FrontEnd")
        .def(py::init<>())
        .def_readwrite("near_week", &Lc86FrontEnd::near_week)
        .def_readonly("eph_frames", &Lc86FrontEnd::eph_frames)
        .def_readonly("eph_added", &Lc86FrontEnd::eph_added)
        .def_property_readonly("store", [](Lc86FrontEnd& f) -> EphemerisStore& { return f.store; },
                               py::return_value_policy::reference_internal)
        .def("feed_payload", &Lc86FrontEnd::feedPayload,
             "One RTCM3 payload (framed here); returns the MSM7 epochs it closed")
        .def("feed_bytes", [](Lc86FrontEnd& f, const py::bytes& b) {
            const std::string s = b;
            return f.feed(reinterpret_cast<const uint8_t*>(s.data()), s.size());
        })
        .def_property_readonly("msm_epochs", [](const Lc86FrontEnd& f) { return f.parser.msmEpochs(); })
        .def_property_readonly("msm_epochs_incomplete",
                               [](const Lc86FrontEnd& f) { return f.parser.msmEpochsIncomplete(); });

    py::class_<RawMeasBuilder::Config>(m, "BuilderConfig")
        .def(py::init<>())
        .def_readwrite("el_mask_deg", &RawMeasBuilder::Config::el_mask_deg)
        .def_readwrite("troposphere", &RawMeasBuilder::Config::troposphere)
        .def_readwrite("doppler_step_hz", &RawMeasBuilder::Config::doppler_step_hz)
        .def_readwrite("pr_corr_scale", &RawMeasBuilder::Config::pr_corr_scale)
        .def_readwrite("rr_scale", &RawMeasBuilder::Config::rr_scale)
        .def_readwrite("relock_sigma_m", &RawMeasBuilder::Config::relock_sigma_m)
        .def_readwrite("relock_tau_s", &RawMeasBuilder::Config::relock_tau_s)
        .def_readwrite("relock_gap_s", &RawMeasBuilder::Config::relock_gap_s)
        .def_readwrite("rr_dyn_sigma", &RawMeasBuilder::Config::rr_dyn_sigma)
        .def_readwrite("rr_relock_sigma", &RawMeasBuilder::Config::rr_relock_sigma)
        .def_readwrite("sat_refresh_s", &RawMeasBuilder::Config::sat_refresh_s)
        .def_readwrite("pr_white_a", &RawMeasBuilder::Config::pr_white_a)
        .def_readwrite("pr_white_b", &RawMeasBuilder::Config::pr_white_b)
        .def_readwrite("pr_corr_a", &RawMeasBuilder::Config::pr_corr_a)
        .def_readwrite("pr_corr_b", &RawMeasBuilder::Config::pr_corr_b)
        .def_readwrite("pr_tau_s", &RawMeasBuilder::Config::pr_tau_s)
        .def_readwrite("rr_white_a", &RawMeasBuilder::Config::rr_white_a)
        .def_readwrite("rr_white_b", &RawMeasBuilder::Config::rr_white_b)
        .def_readwrite("rr_tau_s", &RawMeasBuilder::Config::rr_tau_s)
        .def_readwrite("carrier", &RawMeasBuilder::Config::carrier)
        .def_readwrite("cr_min_lock_ms", &RawMeasBuilder::Config::cr_min_lock_ms)
        .def_readwrite("epoch_period_s", &RawMeasBuilder::Config::epoch_period_s)
        .def_readwrite("cr_max_interval_s", &RawMeasBuilder::Config::cr_max_interval_s)
        .def_readwrite("cr_sigma_floor_mps", &RawMeasBuilder::Config::cr_sigma_floor_mps)
        .def_property("sig", [](const RawMeasBuilder::Config& c) { return arrayList(c.sig); },
                      [](RawMeasBuilder::Config& c, const std::vector<GnssSignalProfile>& v) { arraySet(c.sig, v); },
                      "each system's signal (gnss_sys_t: GPS, Galileo, BeiDou, QZSS), copies");

    py::class_<RawMeasBuilder>(m, "RawMeasBuilder")
        .def(py::init<>())
        .def(py::init<const RawMeasBuilder::Config&>())
        .def_property("config", &RawMeasBuilder::config, &RawMeasBuilder::setConfig)
        .def("set_iono", [](RawMeasBuilder& b, const std::array<double, 4>& alpha, const std::array<double, 4>& beta) {
            gps_iono_t io{};
            io.valid = 1;
            for (int k = 0; k < 4; ++k)
            {
                io.alpha[k] = alpha[k];
                io.beta[k] = beta[k];
            }
            b.setIono(io);
        })
        .def("reset", &RawMeasBuilder::reset)
        .def("note_epoch", &RawMeasBuilder::noteEpoch)
        .def("relock_age", &RawMeasBuilder::relockAge)
        .def("build", [](RawMeasBuilder& b, const RawEpoch& ep, const EphemerisStore& store, py::object r_approx,
                         double g_peak) {
            double r[3];
            const double* rp = nullptr;
            if (!r_approx.is_none())
            {
                auto a = r_approx.cast<Vec3>();
                r[0] = a[0];
                r[1] = a[1];
                r[2] = a[2];
                rp = r;
            }
            RawMeas out[RawEpoch::kMaxObs];
            const int n = b.build(ep, store, rp, g_peak, out, RawEpoch::kMaxObs);
            return std::vector<RawMeas>(out, out + n);
        }, py::arg("epoch"), py::arg("store"), py::arg("r_approx"), py::arg("g_peak") = 0.0);

    py::class_<RawGnssEKFParams>(m, "RawGnssEKFParams")
        .def(py::init<>())
        .def_readwrite("a_noise", &RawGnssEKFParams::a_noise)
        .def_readwrite("a_markov_sigma", &RawGnssEKFParams::a_markov_sigma)
        .def_readwrite("a_markov_tau", &RawGnssEKFParams::a_markov_tau)
        .def_readwrite("w_noise", &RawGnssEKFParams::w_noise)
        .def_readwrite("w_markov_sigma", &RawGnssEKFParams::w_markov_sigma)
        .def_readwrite("w_markov_tau", &RawGnssEKFParams::w_markov_tau)
        .def_readwrite("accel_var", &RawGnssEKFParams::accel_var)
        .def_readwrite("clk_bias_psd", &RawGnssEKFParams::clk_bias_psd)
        .def_readwrite("clk_drift_psd", &RawGnssEKFParams::clk_drift_psd)
        .def_readwrite("isb_psd", &RawGnssEKFParams::isb_psd)
        .def_readwrite("gate_chi2", &RawGnssEKFParams::gate_chi2)
        .def_readwrite("reject_persist", &RawGnssEKFParams::reject_persist)
        .def_readwrite("clock_step_m", &RawGnssEKFParams::clock_step_m)
        .def_readwrite("clock_step_spread_m", &RawGnssEKFParams::clock_step_spread_m)
        .def_readwrite("drift_step_mps", &RawGnssEKFParams::drift_step_mps)
        .def_readwrite("drift_step_spread_mps", &RawGnssEKFParams::drift_step_spread_mps)
        .def_readwrite("rr_lag_s", &RawGnssEKFParams::rr_lag_s)
        .def_readwrite("att_gate_cos4", &RawGnssEKFParams::att_gate_cos4)
        .def_readwrite("att_gate_vertical", &RawGnssEKFParams::att_gate_vertical)
        .def_readwrite("p0_pos", &RawGnssEKFParams::p0_pos)
        .def_readwrite("p0_vel", &RawGnssEKFParams::p0_vel)
        .def_readwrite("p0_att", &RawGnssEKFParams::p0_att)
        .def_readwrite("p0_hdg", &RawGnssEKFParams::p0_hdg)
        .def_readwrite("p0_abias", &RawGnssEKFParams::p0_abias)
        .def_readwrite("p0_wbias", &RawGnssEKFParams::p0_wbias)
        .def_readwrite("p0_clk_bias", &RawGnssEKFParams::p0_clk_bias)
        .def_readwrite("p0_clk_drift", &RawGnssEKFParams::p0_clk_drift)
        .def_readwrite("p0_isb", &RawGnssEKFParams::p0_isb)
        .def_readwrite("gravity_wgs84", &RawGnssEKFParams::gravity_wgs84)
        .def_readwrite("hist_spacing_s", &RawGnssEKFParams::hist_spacing_s)
        .def_readwrite("cr_mode", &RawGnssEKFParams::cr_mode)
        .def_readwrite("cr_sigma_per_rev_s", &RawGnssEKFParams::cr_sigma_per_rev_s)
        .def_readwrite("cr_common_sigma_mps", &RawGnssEKFParams::cr_common_sigma_mps)
        .def_readwrite("cr_screen_k", &RawGnssEKFParams::cr_screen_k)
        .def_readwrite("cr_screen_floor_mps", &RawGnssEKFParams::cr_screen_floor_mps)
        .def_readwrite("cr_half_cycle", &RawGnssEKFParams::cr_half_cycle)
        .def_property("cr_carrier_hz", [](const RawGnssEKFParams& p) { return arrayList(p.cr_carrier_hz); },
                      [](RawGnssEKFParams& p, const std::vector<double>& v) { arraySet(p.cr_carrier_hz, v); })
        .def_property("cr_half_cycle_unflagged",
                      [](const RawGnssEKFParams& p) { return arrayList(p.cr_half_cycle_unflagged); },
                      [](RawGnssEKFParams& p, const std::vector<bool>& v) {
                          if (v.size() != static_cast<size_t>(kProfileSys))
                              throw py::value_error("expected one per gnss_sys_t");
                          for (int k = 0; k < kProfileSys; ++k)
                              p.cr_half_cycle_unflagged[k] = v[k];
                      });

    // the per-epoch stats, carried between an epoch's parts (begin_gnss_raw ...)
    py::class_<RawGnssStats>(m, "RawGnssStats")
        .def(py::init<>())
        .def_readonly("used_pr", &RawGnssStats::used_pr)
        .def_readonly("used_rr", &RawGnssStats::used_rr)
        .def_readonly("rejected_pr", &RawGnssStats::rejected_pr)
        .def_readonly("rejected_rr", &RawGnssStats::rejected_rr)
        .def_readonly("used_cr", &RawGnssStats::used_cr)
        .def_readonly("cr_built", &RawGnssStats::cr_built)
        .def_readonly("cr_in", &RawGnssStats::cr_in)
        .def_readonly("cr_common_mps", &RawGnssStats::cr_common_mps)
        .def("as_dict", &statsDict);

    // the receivers' profiles (ReceiverProfile.h)
    py::class_<GnssSignalProfile>(m, "GnssSignalProfile")
        .def(py::init<>())
        .def_readwrite("carrier_hz", &GnssSignalProfile::carrier_hz)
        .def_readwrite("cn0_min_pr", &GnssSignalProfile::cn0_min_pr)
        .def_readwrite("cn0_min_rr", &GnssSignalProfile::cn0_min_rr)
        .def_readwrite("cn0_min_cr", &GnssSignalProfile::cn0_min_cr)
        .def_readwrite("phase_sigma35_m", &GnssSignalProfile::phase_sigma35_m)
        .def_readwrite("phase_sigma45_m", &GnssSignalProfile::phase_sigma45_m)
        .def_readwrite("half_cycle_unflagged", &GnssSignalProfile::half_cycle_unflagged)
        .def("phase_sigma", [](const GnssSignalProfile& g, float cn0) { return phaseSigmaM(g, cn0); },
             py::arg("cn0_dbhz"), "the carrier phase's 1-sigma (m) at that C/N0");
    py::class_<GnssReceiverProfile>(m, "GnssReceiverProfile")
        .def(py::init<>())
        .def_property_readonly("type", [](const GnssReceiverProfile& p) { return static_cast<int>(p.type); })
        .def_property_readonly("name", [](const GnssReceiverProfile& p) { return std::string(p.name); })
        .def_readwrite("measured", &GnssReceiverProfile::measured)
        .def_property("sig", [](const GnssReceiverProfile& p) { return arrayList(p.sig); },
                      [](GnssReceiverProfile& p, const std::vector<GnssSignalProfile>& v) { arraySet(p.sig, v); },
                      "each system's signal (gnss_sys_t), copies")
        .def_readwrite("rr_lag_s", &GnssReceiverProfile::rr_lag_s)
        .def_readwrite("epoch_latency_s", &GnssReceiverProfile::epoch_latency_s)
        .def_readwrite("raw_epoch_period_s", &GnssReceiverProfile::raw_epoch_period_s)
        .def_readwrite("pr_white_a", &GnssReceiverProfile::pr_white_a)
        .def_readwrite("pr_white_b", &GnssReceiverProfile::pr_white_b)
        .def_readwrite("pr_corr_a", &GnssReceiverProfile::pr_corr_a)
        .def_readwrite("pr_corr_b", &GnssReceiverProfile::pr_corr_b)
        .def_readwrite("pr_corr_scale", &GnssReceiverProfile::pr_corr_scale)
        .def_readwrite("pr_tau_s", &GnssReceiverProfile::pr_tau_s)
        .def_readwrite("rr_white_a", &GnssReceiverProfile::rr_white_a)
        .def_readwrite("rr_white_b", &GnssReceiverProfile::rr_white_b)
        .def_readwrite("rr_scale", &GnssReceiverProfile::rr_scale)
        .def_readwrite("rr_tau_s", &GnssReceiverProfile::rr_tau_s)
        .def_readwrite("doppler_step_hz", &GnssReceiverProfile::doppler_step_hz)
        .def_readwrite("relock_sigma_m", &GnssReceiverProfile::relock_sigma_m)
        .def_readwrite("relock_tau_s", &GnssReceiverProfile::relock_tau_s)
        .def_readwrite("relock_gap_s", &GnssReceiverProfile::relock_gap_s)
        .def_readwrite("rr_dyn_sigma", &GnssReceiverProfile::rr_dyn_sigma)
        .def_readwrite("rr_relock_sigma", &GnssReceiverProfile::rr_relock_sigma)
        .def_readwrite("clk_bias_psd", &GnssReceiverProfile::clk_bias_psd)
        .def_readwrite("clk_drift_psd", &GnssReceiverProfile::clk_drift_psd)
        .def_readwrite("isb_psd", &GnssReceiverProfile::isb_psd)
        .def_readwrite("clock_step_m", &GnssReceiverProfile::clock_step_m)
        .def_readwrite("clock_step_spread_m", &GnssReceiverProfile::clock_step_spread_m)
        .def_readwrite("drift_step_mps", &GnssReceiverProfile::drift_step_mps)
        .def_readwrite("drift_step_spread_mps", &GnssReceiverProfile::drift_step_spread_mps)
        .def_readwrite("p0_clk_bias", &GnssReceiverProfile::p0_clk_bias)
        .def_readwrite("p0_clk_drift", &GnssReceiverProfile::p0_clk_drift)
        .def_readwrite("p0_isb", &GnssReceiverProfile::p0_isb)
        .def_readwrite("cr_mode", &GnssReceiverProfile::cr_mode)
        .def_readwrite("cr_min_lock_ms", &GnssReceiverProfile::cr_min_lock_ms)
        .def_readwrite("cr_max_interval_s", &GnssReceiverProfile::cr_max_interval_s)
        .def_readwrite("cr_sigma_floor_mps", &GnssReceiverProfile::cr_sigma_floor_mps)
        .def_readwrite("cr_sigma_per_rev_s", &GnssReceiverProfile::cr_sigma_per_rev_s)
        .def_readwrite("cr_common_sigma_mps", &GnssReceiverProfile::cr_common_sigma_mps)
        .def_readwrite("cr_screen_k", &GnssReceiverProfile::cr_screen_k)
        .def_readwrite("cr_screen_floor_mps", &GnssReceiverProfile::cr_screen_floor_mps)
        .def_readwrite("cr_half_cycle", &GnssReceiverProfile::cr_half_cycle);
    m.def("lc86g_profile", &lc86gProfile, "the Quectel LC86G's (the Beetle's), measured");
    m.def("neo_m8t_profile", &neoM8tProfile, "u-blox NEO-M8T: UNMEASURED placeholders");
    m.def("px1105r_profile", &px1105rProfile, "SkyTraq PX1105R: UNMEASURED placeholders");
    m.def("mosaic_g5_profile", &mosaicG5Profile, "Septentrio mosaic-G5: UNMEASURED placeholders");
    m.def("generic_profile", &genericProfile, "a receiver with none: UNMEASURED placeholders");
    m.def("profile_for", [](int type) { return profileFor(static_cast<GnssReceiverType>(type)); }, py::arg("type"));
    m.def("apply_profile", [](const GnssReceiverProfile& p, RawMeasBuilder::Config& c) { applyReceiverProfile(p, c); },
          py::arg("profile"), py::arg("config"), "the profile into a builder's config, in place");
    m.def("apply_profile", [](const GnssReceiverProfile& p, RawGnssEKFParams& e) { applyReceiverProfile(p, e); },
          py::arg("profile"), py::arg("params"), "the profile into a filter's params, in place");
    m.attr("CARRIER_OFF") = static_cast<int>(kCarrierOff);
    m.attr("CARRIER_OBSERVE") = static_cast<int>(kCarrierObserve);
    m.attr("CARRIER_FUSE") = static_cast<int>(kCarrierFuse);
    m.attr("HALF_CYCLE_REJECT") = static_cast<int>(kHalfCycleReject);
    m.attr("HALF_CYCLE_REPAIR") = static_cast<int>(kHalfCycleRepair);
    m.attr("RX_GENERIC") = static_cast<int>(kRxGeneric);
    m.attr("RX_LC86G") = static_cast<int>(kRxLc86g);
    m.attr("RX_NEO_M8T") = static_cast<int>(kRxNeoM8t);
    m.attr("RX_PX1105R") = static_cast<int>(kRxPx1105r);
    m.attr("RX_MOSAIC_G5") = static_cast<int>(kRxMosaicG5);

    bindEkf<float>(m, "RawGnssEKF32");
    bindEkf<double>(m, "RawGnssEKF64");

    // the heading from thrust (TR_RawGnssShadow/HeadingFit.h)
    py::class_<HeadingFit::Config>(m, "HeadingFitConfig")
        .def(py::init<>())
        .def_readwrite("bias_sigma_mps2", &HeadingFit::Config::bias_sigma_mps2)
        .def_readwrite("drift_mps2", &HeadingFit::Config::drift_mps2)
        .def_readwrite("cut_chi2", &HeadingFit::Config::cut_chi2)
        .def_readwrite("min_speed_mps", &HeadingFit::Config::min_speed_mps)
        .def_readwrite("tau_s", &HeadingFit::Config::tau_s);
    py::class_<HeadingFit>(m, "HeadingFit")
        .def(py::init<const HeadingFit::Config&>(), py::arg("config") = HeadingFit::Config())
        .def("start", [](HeadingFit& f, const std::array<double, 4>& q, const Vec3& wb, double t0) {
            const float qq[4] = {float(q[0]), float(q[1]), float(q[2]), float(q[3])};
            const float w[3] = {float(wb[0]), float(wb[1]), float(wb[2])};
            f.start(qq, w, t0);
        }, py::arg("q_seed"), py::arg("gyro_bias"), py::arg("t0"))
        .def("imu", [](HeadingFit& f, const Vec3& acc, const Vec3& gyro, double dt) {
            const float a[3] = {float(acc[0]), float(acc[1]), float(acc[2])};
            const float w[3] = {float(gyro[0]), float(gyro[1]), float(gyro[2])};
            f.imu(a, w, float(dt));
        })
        .def("gnss", [](HeadingFit& f, double t, const std::array<double, 2>& v_ne, double var_h) {
            return f.gnss(t, v_ne.data(), var_h);
        })
        .def("solve", [](const HeadingFit& f) { return fitDict(f.solve()); })
        .def("solve_grid", [](const HeadingFit& f) { return fitDict(f.solveGrid()); })
        .def("attitude", [](const HeadingFit& f, double psi) {
            float q[4];
            f.attitude(psi, q);
            return std::array<double, 4>{q[0], q[1], q[2], q[3]};
        })
        .def_property_readonly("q", [](const HeadingFit& f) {
            const float* q = f.gyroOnlyQuat();
            return std::array<double, 4>{q[0], q[1], q[2], q[3]};
        })
        .def_property_readonly("started", &HeadingFit::started)
        .def_property_readonly("t0", &HeadingFit::t0)
        .def_property_readonly("t", &HeadingFit::time)
        .def_property_readonly("epochs", &HeadingFit::epochs);
    m.def("doppler_velocity_ned", [](const py::list& meas, const Vec3& r, double lat, double lon) -> py::object {
        auto v = toVector(meas);
        double vn[3], var_h = 0.0;
        int used = 0;
        if (!dopplerVelocityNed(v.data(), static_cast<int>(v.size()), r.data(), lat, lon, vn, var_h, used))
            return py::none();
        return py::make_tuple(vec3(vn), var_h, used);
    }, py::arg("meas"), py::arg("r_ecef"), py::arg("lat"), py::arg("lon"));
    m.def("heading_between", [](const std::array<double, 4>& a, const std::array<double, 4>& b) {
        const float fa[4] = {float(a[0]), float(a[1]), float(a[2]), float(a[3])};
        const float fb[4] = {float(b[0]), float(b[1]), float(b[2]), float(b[3])};
        return headingBetween(fa, fb);
    });

    m.attr("SYS_GPS") = static_cast<int>(GNSS_SYS_GPS);
    m.attr("SYS_GAL") = static_cast<int>(GNSS_SYS_GAL);
    m.attr("SYS_BDS") = static_cast<int>(GNSS_SYS_BDS);
    m.attr("SYS_QZS") = static_cast<int>(GNSS_SYS_QZS);
}
