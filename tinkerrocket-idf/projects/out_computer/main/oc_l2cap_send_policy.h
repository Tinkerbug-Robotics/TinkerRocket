#pragma once

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// #526: what to DO with an SDU mbuf after ble_l2cap_coc_send() returns.
//
// This is the crux of the whole feature's safety, and it is entirely about mbuf
// OWNERSHIP — get it wrong and you either double-free into the pool (a crash, or
// silent pool corruption at the exact moment RAM is tight) or leak an SDU every
// iteration until the pool is dry. The rules are NOT symmetric and NOT obvious;
// they were read straight out of ble_l2cap_coc.c in esp-idf-v6.0 and are pinned
// to the real BLE_HS_* codes by static_assert in the firmware .cpp:
//
//   rc == 0            Fully sent. The stack freed the SDU
//                      (ble_l2cap_coc.c:604 os_mbuf_free_chain). DO NOT free.
//   rc == ESTALLED     Out of peer credits mid-SDU. The stack KEEPS the SDU in
//                      tx->sdus[0] and will finish it on TX_UNSTALLED
//                      (:608-613). DO NOT free, DO NOT resend, DO NOT submit
//                      another SDU (the 1-deep slot is occupied).
//   rc == EBADDATA     SDU longer than the channel MTU. Returned BEFORE ownership
//                      is taken (:747-749). The CALLER still owns it -> free it.
//                      Must never happen if sduSize() clamps to peer_coc_mtu; if
//                      it does, it fails AND leaks unless we free.
//   rc == EBUSY        tx->sdus[0] already occupied (:751-756). Returned before
//                      ownership -> CALLER frees. Means we submitted while a
//                      previous SDU was still stalled: a producer bug, not a
//                      transport event.
//   anything else      The continue_tx `failed:` path already freed the SDU
//                      (:626-628, e.g. ENOMEM building a K-frame). DO NOT free.
//
// The one that bites: msys exhaustion is NOT ESTALLED and NOT a signal to wait.
// It surfaces as "anything else", the stack has already freed a half-transmitted
// SDU, and the peer's reassembler is now desynced -> a SILENTLY CORRUPT file.
// The producer must prevent it up front with the msys watermark gate below,
// never react to it after the fact. (This is #524's bug class in CoC form.)
//
// Pure and host-tested: test_oc_l2cap_send_policy.cpp walks every branch.
// ============================================================================

namespace oc_l2cap
{

// Mirror of the NimBLE return codes this policy classifies. Kept local so the
// header pulls in no NimBLE (it must stay host-linkable); the firmware .cpp
// static_asserts each against its BLE_HS_* macro so a future IDF renumbering is
// a compile error here, not a field corruption.
static constexpr int kRcOk       = 0;   // BLE_HS_ENONE-ish success
static constexpr int kRcENoMem   = 6;   // BLE_HS_ENOMEM
static constexpr int kRcEBadData = 10;  // BLE_HS_EBADDATA
static constexpr int kRcEBusy    = 15;  // BLE_HS_EBUSY
static constexpr int kRcEStalled = 31;  // BLE_HS_ESTALLED

enum class SendOutcome
{
    Sent,                 // rc == 0: complete, stack owns/freed. -> DONE
    Stalled,              // rc == ESTALLED: stack holds the SDU, wait for unstall
    MustFreeAndAbort,     // EBADDATA/EBUSY: caller still owns the mbuf, free + fail
    AbortStackAlreadyFreed,  // anything else: stack freed it, just fail
};

// Classify a ble_l2cap_coc_send() return value into an action.
inline SendOutcome classifySendRc(int rc)
{
    if (rc == kRcOk)        return SendOutcome::Sent;
    if (rc == kRcEStalled)  return SendOutcome::Stalled;
    if (rc == kRcEBadData)  return SendOutcome::MustFreeAndAbort;
    if (rc == kRcEBusy)     return SendOutcome::MustFreeAndAbort;
    return SendOutcome::AbortStackAlreadyFreed;
}

// True iff the caller must os_mbuf_free_chain() the SDU it passed in.
inline bool callerMustFree(SendOutcome o)
{
    return o == SendOutcome::MustFreeAndAbort;
}

// Is this outcome terminal for the transfer (anything but a clean send or a
// recoverable stall)?
inline bool isFatal(SendOutcome o)
{
    return o == SendOutcome::MustFreeAndAbort ||
           o == SendOutcome::AbortStackAlreadyFreed;
}

// ---------------------------------------------------------------------------
// SDU sizing. An SDU longer than the channel MTU returns EBADDATA *before*
// ownership is taken, i.e. it fails AND leaks every iteration — so the producer
// must clamp to what the peer advertised, never assume its own preferred size.
// peer_coc_mtu of 0 means "not connected / not learned yet" -> no SDU is legal.
// ---------------------------------------------------------------------------
static constexpr uint32_t kPreferredSduBytes = 1024;

inline uint32_t sduSize(uint32_t want, uint32_t peer_coc_mtu)
{
    if (peer_coc_mtu == 0) return 0;
    uint32_t n = (want < peer_coc_mtu) ? want : peer_coc_mtu;
    return n;
}

// ---------------------------------------------------------------------------
// msys watermark gate. Every K-frame the stack emits for our SDU is pulled from
// MSYS_1; if that pool runs dry mid-SDU the transfer corrupts (see above). The
// producer must therefore refuse to submit a new SDU unless the free count is
// comfortably above the fragments one SDU will consume. Telemetry notifications
// share the pool, so the margin is deliberately generous.
// ---------------------------------------------------------------------------
inline bool msysGateOk(int free_blocks, int watermark)
{
    return free_blocks >= watermark;
}

}  // namespace oc_l2cap
