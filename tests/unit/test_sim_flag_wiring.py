"""#1766: the FC reports NSF_SIM_ACTIVE for the whole of a sim's flight state, its held LANDED included.

The rule (sim_flight::reportsSim, TR_Sensor_Collector_Sim/sim_flight_policy.h) is host-tested in
tests_cpp/test_sim_flight_policy.cpp, and the OC's landed lockout fed the stream it makes in
tests_cpp/test_landed_lockout_policy.cpp. main.cpp is not host-built, so this reads the source:
- the flag is set from reportsSim() of the sim latch and the sim's activity, and nowhere else;
- the latch it reads is cleared only by the flight-state reset (a Stop, a Start or an early end), so the flag drops there.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def code():
    with open(MAIN_CPP) as fh:
        return "\n".join(line.split("//", 1)[0] for line in fh.read().splitlines())


def flat(text):
    return re.sub(r"\s+", " ", text)


def test_the_flag_is_reported_for_the_sims_whole_flight_state():
    src = flat(code())
    sets = re.findall(r"if \(([^;]*?)\) non_sensor_data\.flags \|= NSF_SIM_ACTIVE;", src)
    assert sets == ["sim_flight::reportsSim(sim_flight_latched, sensor_collector.isSimActive())"], sets
    assert src.count("NSF_SIM_ACTIVE") == 1, "set in one place"


def test_the_latch_is_cleared_only_by_the_flight_state_reset():
    src = code()
    clears = [m.start() for m in re.finditer(r"(?<!bool )\bsim_flight_latched\s*=\s*false\s*;", src)]   # not the declaration
    assert len(clears) == 1, clears
    reset = src.rindex("static void resetFlightStateForSim(", 0, clears[0])
    body_end = src.index("\n}\n", reset)
    assert reset < clears[0] < body_end, "the clear is in resetFlightStateForSim"
