"""Build script for C++ pybind11 extensions (PID controller, EKF, GuidancePN, ControlMixer).

Shared component sources live under ``tinkerrocket-idf/components/`` — the
canonical ESP-IDF component tree that is also flashed to the rocket. The
host build of the migrated components unconditionally includes <compat.h>,
so the pybind11 extensions add ``tests_cpp/host_shim`` (a small Arduino/
compat shim shared with the cpp unit tests) to their include path.

TR_GuidancePN is the lone exception: it is a separate git submodule whose
sources still live under libraries/TR_GuidancePN/ and still use
``#ifdef ARDUINO`` guards, so it builds on host without any shim. It will
migrate to tinkerrocket-idf/components/TR_GuidancePN once the submodule is
properly registered there — tracked alongside the libraries/ deletion PR.
"""
import os
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

_HERE = os.path.dirname(os.path.abspath(__file__))

# Canonical ESP-IDF component sources (used for TR_GpsInsEKF, TR_ControlMixer, TR_PID).
SHARED_LIB_DIR = os.path.relpath(
    os.path.join(_HERE, "..", "tinkerrocket-idf", "components"), _HERE
)
# Legacy Arduino library tree (only TR_GuidancePN still sourced from here).
LEGACY_LIB_DIR = os.path.relpath(os.path.join(_HERE, "..", "libraries"), _HERE)
# Host-side shim providing Arduino.h/compat.h for components that #include <compat.h>.
SHIM_DIR = os.path.relpath(
    os.path.join(_HERE, "..", "tests_cpp", "host_shim"), _HERE
)

ext_modules = [
    Pybind11Extension(
        "tinkerrocket_sim._pid",
        ["cpp/pid/pid_bindings.cpp"],
        include_dirs=["cpp/pid", "cpp/common"],
        cxx_std=17,
    ),
]

# EKF: build from shared TR_GpsInsEKF library
ekf_lib_dir = os.path.join(SHARED_LIB_DIR, "TR_GpsInsEKF")
if os.path.exists("cpp/ekf/ekf_bindings.cpp") and os.path.exists(ekf_lib_dir):
    ext_modules.append(
        Pybind11Extension(
            "tinkerrocket_sim._ekf",
            [
                os.path.join(ekf_lib_dir, "TR_GpsInsEKF.cpp"),
                "cpp/ekf/ekf_bindings.cpp",
            ],
            include_dirs=[SHIM_DIR, ekf_lib_dir, "cpp/ekf", "cpp/common"],
            cxx_std=17,
        ),
    )

# GuidancePN: TRANSITIONAL — still sourced from legacy libraries/ tree until
# tinkerrocket-idf/components/TR_GuidancePN is set up as a proper submodule.
guidance_lib_dir = os.path.join(LEGACY_LIB_DIR, "TR_GuidancePN")
if os.path.exists("cpp/guidance/guidance_bindings.cpp") and os.path.exists(guidance_lib_dir):
    ext_modules.append(
        Pybind11Extension(
            "tinkerrocket_sim._guidance",
            [
                os.path.join(guidance_lib_dir, "TR_GuidancePN.cpp"),
                "cpp/guidance/guidance_bindings.cpp",
            ],
            include_dirs=[guidance_lib_dir, "cpp/common"],
            cxx_std=17,
        ),
    )

# ControlMixer: build from shared TR_ControlMixer + TR_PID libraries
mixer_lib_dir = os.path.join(SHARED_LIB_DIR, "TR_ControlMixer")
pid_lib_dir = os.path.join(SHARED_LIB_DIR, "TR_PID")
if (os.path.exists("cpp/mixer/mixer_bindings.cpp") and
        os.path.exists(mixer_lib_dir) and os.path.exists(pid_lib_dir)):
    ext_modules.append(
        Pybind11Extension(
            "tinkerrocket_sim._mixer",
            [
                os.path.join(mixer_lib_dir, "TR_ControlMixer.cpp"),
                os.path.join(pid_lib_dir, "TR_PID.cpp"),
                "cpp/mixer/mixer_bindings.cpp",
            ],
            include_dirs=[SHIM_DIR, mixer_lib_dir, pid_lib_dir, "cpp/common"],
            cxx_std=17,
        ),
    )

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
