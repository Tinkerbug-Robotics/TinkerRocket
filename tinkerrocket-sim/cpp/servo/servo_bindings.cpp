// pybind11 binding for the REAL firmware roll controller.
//
// Compiles tinkerrocket-idf/components/TR_ServoControl_ledc_mult against the
// host shim (Arduino/compat + a stubbed driver/ledc.h) so the sim runs the
// exact controlAngle cascade, V^2 gain schedule, and KP_ANGLE rate cap that
// fly on the rocket — not a Python re-implementation.
//
// The controller's PID derives dt from micros(); on host that is the shim's
// mock clock. Call set_mock_micros(t_us) each control tick before control_*().
#include <pybind11/pybind11.h>
#include "TR_ServoControl_ledc_mult.h"

namespace py = pybind11;

PYBIND11_MODULE(_servo, m) {
    m.doc() = "Real firmware roll servo controller (TR_ServoControl_ledc_mult). "
              "LEDC PWM is stubbed on host; read roll_cmd_deg / roll_cmd_us.";

    // Drive the shim clock that the controller's PID reads via micros().
    m.def("set_mock_micros", &setMockMicros, py::arg("us"),
          "Set the host mock microsecond clock (PID dt source).");
    m.def("set_mock_millis", &setMockMillis, py::arg("ms"));

    py::class_<TR_ServoControl>(m, "ServoControl")
        .def(py::init<uint8_t, uint8_t, uint8_t, uint8_t,
                      int, int, int, int,
                      int, int, int,
                      float, float, float, float, float>(),
             py::arg("servo_pin_1") = 0, py::arg("servo_pin_2") = 1,
             py::arg("servo_pin_3") = 2, py::arg("servo_pin_4") = 3,
             py::arg("bias_us_1") = 0, py::arg("bias_us_2") = 0,
             py::arg("bias_us_3") = 0, py::arg("bias_us_4") = 0,
             py::arg("servo_hz") = 333, py::arg("min_us") = 1000, py::arg("max_us") = 2000,
             py::arg("kp") = 0.1f, py::arg("ki") = 0.0f, py::arg("kd") = 0.0f,
             py::arg("min_cmd") = -20.0f, py::arg("max_cmd") = 20.0f)
        // Cascaded angle -> rate controller (outer P + rate cap, inner gain-scheduled PID).
        .def("control_angle", &TR_ServoControl::controlAngle,
             py::arg("target_roll_deg"), py::arg("actual_roll_deg"),
             py::arg("roll_rate_dps"), py::arg("velocity_ms"),
             py::arg("kp_angle"), py::arg("rate_cap_dps"))
        // Rate control with V^2 gain scheduling.
        .def("control_with_gain_schedule", &TR_ServoControl::controlWithGainSchedule,
             py::arg("roll_rate"), py::arg("velocity_ms"))
        // Basic rate control (no scheduling).
        .def("control", &TR_ServoControl::control, py::arg("roll_rate"))
        .def("set_setpoint", &TR_ServoControl::setSetpoint, py::arg("setpoint"))
        .def("enable_gain_schedule", &TR_ServoControl::enableGainSchedule,
             py::arg("v_ref"), py::arg("v_min"))
        .def("disable_gain_schedule", &TR_ServoControl::disableGainSchedule)
        .def("reset_pid", &TR_ServoControl::resetPID)
        .def("set_pid_gains", &TR_ServoControl::setPIDGains,
             py::arg("kp"), py::arg("ki"), py::arg("kd"))
        .def("set_pid_limits", &TR_ServoControl::setPIDLimits,
             py::arg("min_cmd"), py::arg("max_cmd"))
        .def("set_pid_derivative_filter_cutoff_hz",
             &TR_ServoControl::setPIDDerivativeFilterCutoffHz, py::arg("fc_hz"))
        .def("set_pid_integral_separation_threshold",
             &TR_ServoControl::setPIDIntegralSeparationThreshold, py::arg("threshold"))
        .def("set_servo_timing", &TR_ServoControl::setServoTiming,
             py::arg("hz"), py::arg("min_us"), py::arg("max_us"))
        .def("set_angle_control_kp_angle", &TR_ServoControl::setAngleControlKpAngle,
             py::arg("kp"))
        .def("is_gain_schedule_enabled", &TR_ServoControl::isGainScheduleEnabled)
        // Computed outputs (what the sim feeds to the fin-torque model).
        .def_property_readonly("roll_cmd_deg",
             [](TR_ServoControl &s) { return s.getRollCmdDeg(); })
        .def_property_readonly("roll_cmd_us",
             [](TR_ServoControl &s) { return s.getRollCmdUs(); })
        // Base (pre-schedule) gains for snapshotting flown settings.
        .def_property_readonly("kp", [](const TR_ServoControl &s) { return s.getKp(); })
        .def_property_readonly("ki", [](const TR_ServoControl &s) { return s.getKi(); })
        .def_property_readonly("kd", [](const TR_ServoControl &s) { return s.getKd(); });
}
