// TR_GuidancePN.h (stub)
//
// No-op stand-in for the real TR_GuidancePN submodule. Built ONLY when
// the real submodule at tinkerrocket-idf/components/TR_GuidancePN is not
// initialized (e.g. clones without access to the private upstream
// repository). Provides exactly the same public API as the real class;
// every method is a no-op that returns zero / false.
//
// This lets flight_computer/main.cpp compile and run every non-guidance
// feature (roll control, EKF, sensors, telemetry, logging, LoRa, BLE)
// without a single `#ifdef` sprinkled across 15+ call sites. A runtime
// `guidance_enabled` bool gates most of the public calls anyway, so the
// stubs rarely execute at all on a public build — and when they do they
// silently no-op.
//
// flight_computer/CMakeLists.txt picks between the real component and
// this stub at configure time based on whether the submodule is populated
// (see TR_GUIDANCE_AVAILABLE compile definition). When the real component
// is present, this stub is excluded from the build via EXCLUDE_COMPONENTS.

#ifndef TR_GUIDANCE_PN_H
#define TR_GUIDANCE_PN_H

#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class TR_GuidancePN {
public:
    TR_GuidancePN() = default;

    void configure(float /*nav_gain*/,
                   float /*max_accel_mps2*/,
                   float /*target_alt_m*/) {}

    bool update(const float /*pos_enu*/[3],
                const float /*vel_ned*/[3],
                float /*dt*/) { return false; }

    float getAccelEastCmd()    const { return 0.0f; }
    float getAccelNorthCmd()   const { return 0.0f; }
    float getAccelUpCmd()      const { return 0.0f; }
    float getLosAngleDeg()     const { return 0.0f; }
    float getClosingVelocity() const { return 0.0f; }
    float getRange()           const { return 0.0f; }
    float getLateralOffset()   const { return 0.0f; }
    bool  isActive()           const { return false; }
    bool  isCpaReached()       const { return false; }

    void reset() {}
};

#endif // TR_GUIDANCE_PN_H
