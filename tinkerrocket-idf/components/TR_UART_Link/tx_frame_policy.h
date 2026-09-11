#pragma once
// Should this TX_FRAME be accepted, and if not, is it answerable? (#1152)
//
// Extracted as a pure function so the rule is unit testable without a modem, a
// UART or FreeRTOS — the same pattern as modem_config_ack.h, MainDeployGate and
// bs_log_policy.
//
// THE INVARIANT THIS PROTECTS, stated absolutely in RadioModemProtocol.h:
// "A TX frame is therefore never silently dropped: every accepted seq is
// answered." The host's credit window depends on it — a seq that is neither
// answered nor rejected is a credit the host never gets back, and eight of
// those stop it transmitting with no diagnostic at all.
//
// The old guard was `len < sizeof(TxFrameHeader) + 1`, i.e. len < 2, justified
// by the comment "no seq to answer". That reasoning is wrong for len == 1: the
// seq IS on the wire at payload[0], so the frame is answerable and was being
// dropped anyway. The only genuinely unanswerable frame is one with no seq
// byte at all.

#include <stddef.h>
#include <stdint.h>

#include "RadioModemProtocol.h"

namespace tx_frame_policy
{

enum class Verdict : uint8_t
{
    /// No seq on the wire (len == 0). The ONLY case that may be dropped in
    /// silence, because there is nothing to answer with.
    DropNoSeq,
    /// Answerable rejections: every one of these owes a TX_RESULT(seq, false).
    RejectEmptyAir,    ///< a seq, but zero air bytes — nothing to transmit
    RejectTooLong,     ///< air payload exceeds MAX_AIR_FRAME
    RejectRadioDown,   ///< modem alive, RF dead
    RejectQueueFull,   ///< host exceeded its credit window, or credits desynced
    Accept,
};

/// True when this verdict owes the host a TX_RESULT carrying the seq back.
inline bool owesResult(Verdict v)
{
    return v != Verdict::DropNoSeq && v != Verdict::Accept;
}

/// Decide, from the frame length and the modem's state, what happens to a
/// TX_FRAME. `len` is the whole payload: the seq byte plus the air bytes.
inline Verdict admit(size_t len, bool radio_up, uint8_t txq_used,
                     uint8_t capacity)
{
    if (len < sizeof(radio_modem::TxFrameHeader))
    {
        return Verdict::DropNoSeq;
    }
    const size_t air_len = len - sizeof(radio_modem::TxFrameHeader);
    // Checked before the radio state on purpose: an empty air frame is
    // malformed whatever the radio is doing, and saying so is more useful than
    // blaming a radio that is merely down.
    if (air_len == 0)
    {
        return Verdict::RejectEmptyAir;
    }
    if (air_len > radio_modem::MAX_AIR_FRAME)
    {
        return Verdict::RejectTooLong;
    }
    if (!radio_up)
    {
        return Verdict::RejectRadioDown;
    }
    if (txq_used >= capacity)
    {
        return Verdict::RejectQueueFull;
    }
    return Verdict::Accept;
}

}  // namespace tx_frame_policy
