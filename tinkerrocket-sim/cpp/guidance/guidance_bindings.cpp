#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "TR_GuidancePN.h"

namespace py = pybind11;

PYBIND11_MODULE(_guidance, m) {
    m.doc() = "Proportional Navigation guidance (shared library: TR_GuidancePN)";

    py::class_<TR_GuidancePN>(m, "GuidancePN")
        .def(py::init<>())
        .def("configure", &TR_GuidancePN::configure,
             py::arg("nav_gain"), py::arg("max_accel_mps2"),
             py::arg("target_alt_m"))
        .def("update", [](TR_GuidancePN& self,
                          std::array<float, 3> pos_enu,
                          std::array<float, 3> vel_ned,
                          float dt) {
            return self.update(pos_enu.data(), vel_ned.data(), dt);
        }, py::arg("pos_enu"), py::arg("vel_ned"), py::arg("dt"))
        .def("get_accel_east_cmd", &TR_GuidancePN::getAccelEastCmd)
        .def("get_accel_north_cmd", &TR_GuidancePN::getAccelNorthCmd)
        .def("get_accel_up_cmd", &TR_GuidancePN::getAccelUpCmd)
        .def("get_los_angle_deg", &TR_GuidancePN::getLosAngleDeg)
        .def("get_closing_velocity", &TR_GuidancePN::getClosingVelocity)
        .def("get_range", &TR_GuidancePN::getRange)
        .def("get_lateral_offset", &TR_GuidancePN::getLateralOffset)
        .def("is_active", &TR_GuidancePN::isActive)
        .def("is_cpa_reached", &TR_GuidancePN::isCpaReached)
        .def("reset", &TR_GuidancePN::reset);
}
