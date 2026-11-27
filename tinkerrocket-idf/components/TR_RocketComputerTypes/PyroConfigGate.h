#ifndef PYRO_CONFIG_GATE_H
#define PYRO_CONFIG_GATE_H

// #1775: the acceptance check for the pyro channel config (PyroConfigData:
// BLE command 34 on the OC, PYRO_CONFIG_PENDING on the FC, and both boot
// loads), as pure functions shared by both computers and host-tested
// (RollProfileGate.h precedent).
//
// Before, the config was applied and saved with only the frame CRC and the
// payload length checked. Three kinds of value leave an ENABLED channel
// unable to fire when it is needed:
//   - a mode other than 0 or 1 matches neither trigger branch, so the channel
//     stays Idle for the whole flight;
//   - a NaN value never satisfies `elapsed_s >= val` (time mode) or the deploy
//     gate's `<=` comparisons (altitude mode);
//   - an altitude of 0 m or less fires, on a trusted barometer, only once the
//     pad-referenced altitude reaches it while still descending: about
//     touchdown, or never on a landing site above the pad.
// The iOS dashboard's pyro sheet sent the last one for a cleared field
// (`Float(s) ?? 0`), and the pad health bits, built from "enabled" and
// continuity alone, read such a channel OK.
//
// The policy is REFUSE, never clamp (RollProfileGate.h): a refused frame
// leaves the previous config in place, the log names the failing channel and
// rule, and the config readback re-states what the rocket holds.
//
// Only ENABLED channels are checked. A disabled channel's mode and value are
// never read: the trigger loop, the flight timeout and the health bits all
// skip it, and both apps send whatever the profile last held for it.
// "Enabled" is any nonzero byte, as the FC's pyroChEnabled() and the OC's
// cache read it.

#include <cmath>
#include <stdint.h>

#include "RocketComputerTypes.h"

// Longest time-after-apogee delay accepted, in seconds.
//
// A policy bound, chosen for #1775, not a limit of the hardware. A deployment
// by time is a drogue or its backup, seconds after apogee; a main is set by
// altitude. The FC services the triggers only in INFLIGHT, so a delay that
// outlasts the descent never fires at all, and the rocket lands without it.
// 120 s leaves two minutes and refuses a millisecond value sent as seconds
// ("1500" for 1.5 s). An event wanted later than that on a high flight is set
// by altitude. A negative delay is refused too: it would fire at apogee,
// which 0 s already says.
inline constexpr float PYRO_TRIGGER_MAX_DELAY_S = 120.0f;

// Lowest altitude-on-descent accepted, in meters above the pad.
//
// At or below 0 m a channel fires at touchdown on a trusted barometer, or
// never on a landing site above the pad: that is what a cleared field sent.
// 30 m (98 ft) refuses those and the typos near them; a main set lower has
// little height left to open in. On the GNSS backstop the same setting fires
// 25 m higher (MainDeployGate's gnss_margin_m).
inline constexpr float PYRO_TRIGGER_MIN_ALTITUDE_M = 30.0f;

// Why a config was refused. The log line names it, and the apps' own copy of
// the rule (PyroTriggerRule) says the same range to the operator.
enum PyroConfigRc : uint8_t
{
    PYRO_CFG_OK           = 0,
    PYRO_CFG_REJ_MODE     = 1,  // an enabled channel's mode is not 0 or 1
    PYRO_CFG_REJ_VALUE    = 2,  // its value is NaN or infinite
    PYRO_CFG_REJ_DELAY    = 3,  // time mode, outside [0, PYRO_TRIGGER_MAX_DELAY_S]
    PYRO_CFG_REJ_ALTITUDE = 4,  // altitude mode, below PYRO_TRIGGER_MIN_ALTITUDE_M
};

// One enabled channel's trigger.
inline PyroConfigRc pyroTriggerRc(uint8_t mode, float value)
{
    if (mode != PYRO_TRIGGER_TIME_AFTER_APOGEE && mode != PYRO_TRIGGER_ALTITUDE_ON_DESCENT)
    {
        return PYRO_CFG_REJ_MODE;
    }
    if (!std::isfinite(value))
    {
        return PYRO_CFG_REJ_VALUE;
    }
    if (mode == PYRO_TRIGGER_TIME_AFTER_APOGEE)
    {
        return (value >= 0.0f && value <= PYRO_TRIGGER_MAX_DELAY_S) ? PYRO_CFG_OK
                                                                     : PYRO_CFG_REJ_DELAY;
    }
    return (value >= PYRO_TRIGGER_MIN_ALTITUDE_M) ? PYRO_CFG_OK : PYRO_CFG_REJ_ALTITUDE;
}

// One channel of the wire struct, by index 0-3.
struct PyroConfigChannel
{
    bool    enabled = false;
    uint8_t mode    = 0;
    float   value   = 0.0f;
};

inline PyroConfigChannel pyroConfigChannel(const PyroConfigData& c, int idx)
{
    switch (idx)
    {
        case 0: return { c.ch1_enabled != 0, c.ch1_trigger_mode, c.ch1_trigger_value };
        case 1: return { c.ch2_enabled != 0, c.ch2_trigger_mode, c.ch2_trigger_value };
        case 2: return { c.ch3_enabled != 0, c.ch3_trigger_mode, c.ch3_trigger_value };
        case 3: return { c.ch4_enabled != 0, c.ch4_trigger_mode, c.ch4_trigger_value };
    }
    return {};
}

// The verdict on a whole config: the first enabled channel that fails, as
// 1-4, and its rule. channel is 0 when every enabled channel passes.
struct PyroConfigVerdict
{
    PyroConfigRc rc      = PYRO_CFG_OK;
    uint8_t      channel = 0;
};

inline PyroConfigVerdict pyroConfigCheck(const PyroConfigData& c)
{
    for (int i = 0; i < 4; ++i)
    {
        const PyroConfigChannel ch = pyroConfigChannel(c, i);
        if (!ch.enabled)
        {
            continue;
        }
        const PyroConfigRc rc = pyroTriggerRc(ch.mode, ch.value);
        if (rc != PYRO_CFG_OK)
        {
            return { rc, (uint8_t)(i + 1) };
        }
    }
    return {};
}

// #1775: one channel's pad health bit (SH_PYRO), as the FC's scorecard builds
// it (#303). A channel not enabled is NA, so it is left out of the go/no-go.
// One whose trigger fails the check is BAD whatever its continuity, since it
// cannot fire when it is needed. Otherwise continuity decides: DEGRADED until
// it is read, then OK or BAD.
inline SensorHealthState pyroChannelHealth(bool enabled, uint8_t mode, float value,
                                           bool cont_known, bool cont)
{
    if (!enabled)
    {
        return SH_NA;
    }
    if (pyroTriggerRc(mode, value) != PYRO_CFG_OK)
    {
        return SH_BAD;
    }
    if (!cont_known)
    {
        return SH_DEGRADED;
    }
    return cont ? SH_OK : SH_BAD;
}

// Short tag for the log line naming the rule.
inline const char* pyroConfigRcName(PyroConfigRc rc)
{
    switch (rc)
    {
        case PYRO_CFG_OK:           return "ok";
        case PYRO_CFG_REJ_MODE:     return "trigger mode not 0 or 1";
        case PYRO_CFG_REJ_VALUE:    return "value not finite";
        case PYRO_CFG_REJ_DELAY:    return "delay outside 0-120 s";
        case PYRO_CFG_REJ_ALTITUDE: return "altitude below 30 m";
    }
    return "unknown";
}

#endif  // PYRO_CONFIG_GATE_H
