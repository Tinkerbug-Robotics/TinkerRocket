// Python bindings for the raw-GNSS (tightly coupled) filter of #1600: the
// LC86G's RTCM through the flight code's own parser (Lc86Parser), ephemeris
// store and measurement builder (TR_GNSSRaw), and the filter (TR_RawGnssEKF)
// in its float (flight) and double instantiations. The gnss repo's bench
// scorer (sim/tc_ekf_cocom.py, mode ins-cpp) runs it beside the prototype it
// was ported from.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "EphemerisStore.h"
#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawMeas.h"
#include "RawMeasBuilder.h"
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
    return d;
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
        }, py::arg("meas"), py::arg("age_s") = 0.0)
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
        });
}

}  // namespace

PYBIND11_MODULE(_rawgnss, m)
{
    m.doc() = "Raw-GNSS tightly coupled filter (#1600): TR_GNSSRaw + TR_RawGnssEKF + the LC86G parser";

    py::class_<RawMeas>(m, "RawMeas")
        .def(py::init([](int sys, int prn, const Vec3& sat_pos, const Vec3& sat_vel, const Vec3& sat_acc,
                         py::object pr, py::object rr, float sigma_pr, float sigma_pr_corr, float tau_pr,
                         float sigma_rr, float tau_rr) {
                 RawMeas x{};
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
             py::arg("sigma_rr"), py::arg("tau_rr"))
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
        .def_readonly("tau_rr", &RawMeas::tau_rr);

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
                 }
                 return ep;
             }),
             py::arg("tow_s"), py::arg("obs"),
             "obs: [(sys, prn, pr_m | None, rate_mps | None, cn0[, lock_ms])], sys as gnss_sys_t")
        .def_readonly("tow_s", &RawEpoch::tow_s)
        .def_property_readonly("obs", &obsList);

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
        .def_readwrite("rr_relock_sigma", &RawMeasBuilder::Config::rr_relock_sigma);

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
        .def_readwrite("hist_spacing_s", &RawGnssEKFParams::hist_spacing_s);

    bindEkf<float>(m, "RawGnssEKF32");
    bindEkf<double>(m, "RawGnssEKF64");

    m.attr("SYS_GPS") = static_cast<int>(GNSS_SYS_GPS);
    m.attr("SYS_GAL") = static_cast<int>(GNSS_SYS_GAL);
    m.attr("SYS_BDS") = static_cast<int>(GNSS_SYS_BDS);
}
