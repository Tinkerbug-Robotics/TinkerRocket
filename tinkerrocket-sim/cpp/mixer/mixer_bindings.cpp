#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "TR_ControlMixer.h"

namespace py = pybind11;

PYBIND11_MODULE(_mixer, m) {
    m.doc() = "3-axis fin mixing for cruciform configuration (shared library: TR_ControlMixer)";

    py::class_<TR_ControlMixer>(m, "ControlMixer")
        .def(py::init<>())
        .def("configure", &TR_ControlMixer::configure,
             py::arg("pitch_kp"), py::arg("pitch_ki"), py::arg("pitch_kd"),
             py::arg("yaw_kp"), py::arg("yaw_ki"), py::arg("yaw_kd"),
             py::arg("max_fin_deflection_deg"),
             py::arg("v_ref"), py::arg("v_min"))
        .def("update", &TR_ControlMixer::update,
             py::arg("pitch_angle_cmd_deg"), py::arg("yaw_angle_cmd_deg"),
             py::arg("pitch_angle_actual_deg"), py::arg("yaw_angle_actual_deg"),
             py::arg("pitch_rate_dps"), py::arg("yaw_rate_dps"),
             py::arg("roll_cmd_deg"), py::arg("speed_mps"),
             py::arg("kp_pitch_angle"), py::arg("kp_yaw_angle"),
             py::arg("dt"))
        .def("get_fin_deflections", [](const TR_ControlMixer& self) {
            float d[4];
            self.getFinDeflections(d);
            return std::array<float, 4>{d[0], d[1], d[2], d[3]};
        })
        .def("get_pitch_fin_cmd", &TR_ControlMixer::getPitchFinCmd)
        .def("get_yaw_fin_cmd", &TR_ControlMixer::getYawFinCmd)
        .def("enable_gain_schedule", &TR_ControlMixer::enableGainSchedule,
             py::arg("v_ref"), py::arg("v_min"))
        .def("disable_gain_schedule", &TR_ControlMixer::disableGainSchedule)
        .def("reset", &TR_ControlMixer::reset);
}
