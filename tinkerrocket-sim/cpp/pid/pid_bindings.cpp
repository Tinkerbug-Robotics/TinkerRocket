#include <pybind11/pybind11.h>
#include "pid_controller.h"

namespace py = pybind11;

PYBIND11_MODULE(_pid, m) {
    m.doc() = "PID controller (ported from TR_PID)";

    py::class_<PIDController>(m, "PIDController")
        .def(py::init<float, float, float, float, float>(),
             py::arg("kp") = 0.1f,
             py::arg("ki") = 0.0f,
             py::arg("kd") = 0.0f,
             py::arg("min_cmd") = -20.0f,
             py::arg("max_cmd") = 20.0f)
        .def("compute", &PIDController::compute,
             py::arg("setpoint"), py::arg("measurement"), py::arg("dt"),
             "Compute PID output. Returns 0 on first call.")
        .def("reset", &PIDController::reset)
        .def("set_kp", &PIDController::setKp)
        .def("set_ki", &PIDController::setKi)
        .def("set_kd", &PIDController::setKd)
        .def("set_min_cmd", &PIDController::setMinCmd)
        .def("set_max_cmd", &PIDController::setMaxCmd)
        .def_property_readonly("kp", &PIDController::getKp)
        .def_property_readonly("ki", &PIDController::getKi)
        .def_property_readonly("kd", &PIDController::getKd)
        .def_property_readonly("min_cmd", &PIDController::getMinCmd)
        .def_property_readonly("max_cmd", &PIDController::getMaxCmd)
        .def_property_readonly("cumulative_error", &PIDController::getCumulativeError)
        .def_property_readonly("last_error", &PIDController::getLastError);
}
