"""Build script for C++ pybind11 extensions (PID controller and EKF)."""
import os
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

# Path to shared Arduino libraries (one level up from TinkerRocket-Sim)
_HERE = os.path.dirname(os.path.abspath(__file__))
SHARED_LIB_DIR = os.path.relpath(os.path.join(_HERE, "..", "libraries"), _HERE)

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
            include_dirs=[ekf_lib_dir, "cpp/ekf", "cpp/common"],
            cxx_std=17,
        ),
    )

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
