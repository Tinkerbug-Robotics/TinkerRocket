#pragma once
#include <math.h>
#include <stdint.h>
#include "RadioModemProtocol.h"

// #835 item 7 — should a SET_CONFIG ack be believed?
//
// Extracted as a pure function so the tri-state rule below is unit testable
// without a modem, a UART, or FreeRTOS (same pattern as MainDeployGate and
// bs_log_policy).
//
// The trap this exists to prevent: ModemStatusData::radio_enabled only says
// the radio is ALIVE.  TR_LoRa_Comms::reconfigure() rolls back to the previous
// modulation on any setBandwidth/setSpreadingFactor/setFrequency error and
// leaves the radio up, so a failed config still acks with radio_enabled=1.
// The host then cached a modulation that was never on the air and the OC wrote
// it to NVS; the next boot begin()s the illegal pair and kills the radio.
//
// WHY 0 IS NOT "REJECTED": config_ok was `reserved[0]` before this change, and
// older modem images zero-fill it.  Treating 0 as a rejection would make every
// config from an un-reflashed daughterboard fail.  Legacy images are covered
// by the on-air comparison instead, which catches a rollback either way
// because a rolled-back radio reports its PREVIOUS freq/SF.
namespace modem_config_ack
{

// Tolerance for the reported centre frequency, MHz.  STATUS carries a float
// that has round-tripped through the radio's own register math, so an exact
// compare would spuriously reject a config that did apply.
static constexpr float kFreqEpsilonMhz = 0.001f;

struct Ack
{
    uint8_t config_ok;          // ModemStatusData::config_ok (CfgAck)
    float   on_air_freq_mhz;    // ModemStatusData::current_freq_mhz
    uint8_t on_air_sf;          // ModemStatusData::current_sf
    uint8_t fail_reason = 0;    // ModemStatusData::config_fail_reason (#1173)
};

/// Why the modem said no, in words. "" when there is nothing to explain.
///
/// Pure and defaulted so every existing Ack{...} call site keeps compiling and
/// simply reports no reason — which is exactly what a legacy modem sends.
///
/// The case this exists for: CFG_FAIL_FRAME_PARAMS. There the requested
/// modulation IS on the air, so `accepted()`'s freq/SF comparison passes and
/// the operator's log reads "config NOT applied: asked 915.000 SF8, modem
/// reports 915.000 SF8" — a flat contradiction with no way to tell that the
/// preamble/CRC/gain/syncword are the ones that did not take.
static inline const char* reason_text(const Ack& ack)
{
    if (ack.config_ok != radio_modem::CFG_ACK_REJECTED || ack.fail_reason == 0)
    {
        return "";
    }
    // Radio-down first: it subsumes the others, and it is the only one that
    // leaves nothing at all on the air.
    if (ack.fail_reason & radio_modem::CFG_FAIL_RADIO_DOWN)
    {
        return "the radio failed to start - nothing is on the air";
    }
    if (ack.fail_reason & radio_modem::CFG_FAIL_MODULATION)
    {
        return "the modulation was refused - the radio rolled back to its "
               "previous one and is still up";
    }
    if (ack.fail_reason & radio_modem::CFG_FAIL_FRAME_PARAMS)
    {
        return "the modulation IS live, but the frame format "
               "(preamble/CRC/gain/syncword) was not applied - airtime "
               "accounting on this link is now wrong";
    }
    return "rejected for a reason this host does not recognise";
}

struct Want
{
    float   freq_mhz;
    uint8_t sf;
};

/// True when the modem's ack proves the requested modulation is on the air.
static inline bool accepted(const Ack& ack, const Want& want)
{
    // An explicit rejection is final — the modem told us it rolled back.
    if (ack.config_ok == radio_modem::CFG_ACK_REJECTED)
    {
        return false;
    }
    // CFG_ACK_APPLIED and CFG_ACK_UNKNOWN both fall through to the on-air
    // check.  APPLIED still gets verified because the modem's idea of success
    // and what its radio actually reports should agree; UNKNOWN has nothing
    // else to go on.
    if (ack.on_air_sf != want.sf)
    {
        return false;
    }
    return fabsf(ack.on_air_freq_mhz - want.freq_mhz) <= kFreqEpsilonMhz;
}

}  // namespace modem_config_ack
