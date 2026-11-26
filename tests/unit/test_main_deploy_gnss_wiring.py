"""#1761: main.cpp's GNSS inputs to MainDeployGate.

The gate's GNSS persistence (a run of consistent fixes before the backstop may fire) is host-tested in
tests_cpp/test_main_deploy_gate.cpp. It can only count fixes if main.cpp says which fix each tick carries and how old
it is; without that every tick looks like the same fix and the backstop never settles. main.cpp is not host-built,
so this test reads the source.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def test_the_gate_is_told_which_fix_and_how_old():
    src = open(MAIN_CPP).read()
    step = src.index("MainDeployGate::step(main_deploy_state, g);")
    block = src[src.rindex("MainDeployGate::Inputs& g = main_deploy_in;", 0, step):step]
    assert re.search(r"g\.gnss_fix_id\s*=\s*gnss_latest_si\.time_us;", block)
    assert re.search(r"g\.gnss_fix_age_ms\s*=\s*gnss_age_us / 1000u;", block)
    # the same age the propagation uses
    assert re.search(r"g\.gnss_agl_m = \(float\)\(gnss_latest_si\.alt - ref_alt_m\) \+\s*"
                     r"g\.gnss_vel_u_mps \* \(\(float\)gnss_age_us \* 1e-6f\);", block)
