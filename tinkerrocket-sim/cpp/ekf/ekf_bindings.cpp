#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "TR_GpsInsEKF.h"

namespace py = pybind11;

PYBIND11_MODULE(_ekf, m) {
    m.doc() = "GPS/INS Fusion EKF (shared library: TR_GpsInsEKF)";

    // Expose under the old Python names for backward-compatibility
    py::class_<EkfIMUData>(m, "IMUData")
        .def(py::init<>())
        .def_readwrite("time_us", &EkfIMUData::time_us)
        .def_readwrite("acc_x", &EkfIMUData::acc_x)
        .def_readwrite("acc_y", &EkfIMUData::acc_y)
        .def_readwrite("acc_z", &EkfIMUData::acc_z)
        .def_readwrite("gyro_x", &EkfIMUData::gyro_x)
        .def_readwrite("gyro_y", &EkfIMUData::gyro_y)
        .def_readwrite("gyro_z", &EkfIMUData::gyro_z);

    py::class_<EkfGNSSData>(m, "GNSSData")
        .def(py::init<>())
        .def_readwrite("time_us", &EkfGNSSData::time_us)
        .def_readwrite("ecef_x", &EkfGNSSData::ecef_x)
        .def_readwrite("ecef_y", &EkfGNSSData::ecef_y)
        .def_readwrite("ecef_z", &EkfGNSSData::ecef_z)
        .def_readwrite("ecef_vx", &EkfGNSSData::ecef_vx)
        .def_readwrite("ecef_vy", &EkfGNSSData::ecef_vy)
        .def_readwrite("ecef_vz", &EkfGNSSData::ecef_vz);

    py::class_<EkfGNSSDataLLA>(m, "GNSSDataLLA")
        .def(py::init<>())
        .def_readwrite("time_us", &EkfGNSSDataLLA::time_us)
        .def_readwrite("lat_rad", &EkfGNSSDataLLA::lat_rad)
        .def_readwrite("lon_rad", &EkfGNSSDataLLA::lon_rad)
        .def_readwrite("alt_m", &EkfGNSSDataLLA::alt_m)
        .def_readwrite("vel_n_mps", &EkfGNSSDataLLA::vel_n_mps)
        .def_readwrite("vel_e_mps", &EkfGNSSDataLLA::vel_e_mps)
        .def_readwrite("vel_d_mps", &EkfGNSSDataLLA::vel_d_mps);

    py::class_<EkfMagData>(m, "MagData")
        .def(py::init<>())
        .def_readwrite("time_us", &EkfMagData::time_us)
        .def_readwrite("mag_x", &EkfMagData::mag_x)
        .def_readwrite("mag_y", &EkfMagData::mag_y)
        .def_readwrite("mag_z", &EkfMagData::mag_z);

    py::class_<EkfBaroData>(m, "BaroData")
        .def(py::init<>())
        .def_readwrite("time_us", &EkfBaroData::time_us)
        .def_readwrite("altitude_m", &EkfBaroData::altitude_m);

    py::class_<GpsInsEKF>(m, "GpsInsEKF")
        .def(py::init<>())
        // ECEF init (simulation path — backward compatible)
        .def("init", py::overload_cast<EkfIMUData, EkfGNSSData, EkfMagData>(&GpsInsEKF::init),
             py::arg("imu"), py::arg("gnss"), py::arg("mag"))
        // LLA init (flight computer path)
        .def("init_lla", py::overload_cast<EkfIMUData, EkfGNSSDataLLA, EkfMagData>(&GpsInsEKF::init),
             py::arg("imu"), py::arg("gnss"), py::arg("mag"))
        // ECEF update (simulation path — backward compatible)
        .def("update", py::overload_cast<bool, EkfIMUData, EkfGNSSData, EkfMagData>(&GpsInsEKF::update),
             py::arg("use_ahrs_acc"), py::arg("imu"), py::arg("gnss"), py::arg("mag"))
        // LLA update (flight computer path)
        .def("update_lla", py::overload_cast<bool, EkfIMUData, EkfGNSSDataLLA, EkfMagData>(&GpsInsEKF::update),
             py::arg("use_ahrs_acc"), py::arg("imu"), py::arg("gnss"), py::arg("mag"))
        .def("baro_meas_update", &GpsInsEKF::baroMeasUpdate,
             py::arg("baro"))
        .def("set_quaternion", [](GpsInsEKF& self, float q0, float q1, float q2, float q3) {
            self.setQuaternion(q0, q1, q2, q3);
        }, py::arg("q0"), py::arg("q1"), py::arg("q2"), py::arg("q3"))
        .def("set_position", &GpsInsEKF::setPosition,
             py::arg("lat_rad"), py::arg("lon_rad"), py::arg("alt_m"))
        .def("set_velocity", &GpsInsEKF::setVelocity,
             py::arg("vn"), py::arg("ve"), py::arg("vd"))
        .def("set_gps_noise_scale", &GpsInsEKF::setGpsNoiseScale,
             py::arg("scale"))
        .def("get_gps_noise_scale", &GpsInsEKF::getGpsNoiseScale)
        .def("get_orientation", [](const GpsInsEKF& self) {
            float r[3]; self.getOrientEst(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_position", [](const GpsInsEKF& self) {
            double r[3]; self.getPosEst(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_velocity", [](const GpsInsEKF& self) {
            float r[3]; self.getVelEst(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_accel_est", [](const GpsInsEKF& self) {
            float r[3]; self.getAccelEst(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_accel_bias", [](const GpsInsEKF& self) {
            float r[3]; self.getAccelBias(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_rot_rate_est", [](const GpsInsEKF& self) {
            float r[3]; self.getRotRateEst(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_rot_rate_bias", [](const GpsInsEKF& self) {
            float r[3]; self.getRotRateBias(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_quaternion", [](const GpsInsEKF& self) {
            float r[4]; self.getQuaternion(r);
            return py::make_tuple(r[0], r[1], r[2], r[3]);
        })
        .def("get_cov_pos", [](const GpsInsEKF& self) {
            float r[3]; self.getCovPos(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_cov_vel", [](const GpsInsEKF& self) {
            float r[3]; self.getCovVel(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_cov_orient", [](const GpsInsEKF& self) {
            float r[3]; self.getCovOrient(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_cov_accel_bias", [](const GpsInsEKF& self) {
            float r[3]; self.getCovAccelBias(r);
            return py::make_tuple(r[0], r[1], r[2]);
        })
        .def("get_cov_rot_rate_bias", [](const GpsInsEKF& self) {
            float r[3]; self.getCovRotRateBias(r);
            return py::make_tuple(r[0], r[1], r[2]);
        });
}
