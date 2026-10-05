// #1553: the FC's fire timing, for test_pyro_fault_trip_policy. Compiled
// against flight_computer/main/config.h in its own library: the OC's main/
// directory has a config.h of its own, so the two cannot share an include path.
#include <stdint.h>
#include "config.h"

uint32_t fcPyroArmSettleMs() { return config::PYRO_ARM_SETTLE_MS; }
uint32_t fcPyroFireDurationMs() { return config::PYRO_FIRE_DURATION_MS; }
