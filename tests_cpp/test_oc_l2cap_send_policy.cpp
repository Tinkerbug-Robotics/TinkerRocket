// Host tests for oc_l2cap_send_policy.h — the mbuf-ownership decision after
// ble_l2cap_coc_send(). This is the module whose bugs are a double-free or a
// leak, so every return code is pinned to the action the NimBLE source dictates
// (verified against ble_l2cap_coc.c in esp-idf-v6.0).

#include <gtest/gtest.h>

#include "oc_l2cap_send_policy.h"

using namespace oc_l2cap;

// The five outcomes, straight from the ownership table in the header.
TEST(OcL2capSendPolicy, RcZeroIsSentAndStackOwns)
{
    auto o = classifySendRc(kRcOk);
    EXPECT_EQ(o, SendOutcome::Sent);
    EXPECT_FALSE(callerMustFree(o));   // stack freed it (coc.c:604)
    EXPECT_FALSE(isFatal(o));
}

TEST(OcL2capSendPolicy, StalledMeansStackHoldsItDoNotTouch)
{
    auto o = classifySendRc(kRcEStalled);
    EXPECT_EQ(o, SendOutcome::Stalled);
    EXPECT_FALSE(callerMustFree(o));   // stack keeps it in sdus[0] (coc.c:608-613)
    EXPECT_FALSE(isFatal(o));          // recoverable: TX_UNSTALLED will finish it
}

TEST(OcL2capSendPolicy, EBadDataCallerStillOwns)
{
    // Oversize SDU: returned before ownership taken (coc.c:747-749).
    auto o = classifySendRc(kRcEBadData);
    EXPECT_EQ(o, SendOutcome::MustFreeAndAbort);
    EXPECT_TRUE(callerMustFree(o));
    EXPECT_TRUE(isFatal(o));
}

TEST(OcL2capSendPolicy, EBusyCallerStillOwns)
{
    // Slot occupied: returned before ownership taken (coc.c:751-756).
    auto o = classifySendRc(kRcEBusy);
    EXPECT_EQ(o, SendOutcome::MustFreeAndAbort);
    EXPECT_TRUE(callerMustFree(o));
    EXPECT_TRUE(isFatal(o));
}

TEST(OcL2capSendPolicy, EnomemAlreadyFreedByStack)
{
    // The dangerous one: msys exhaustion takes the `failed:` label which ALREADY
    // freed the SDU (coc.c:626-628). Freeing again would be a double-free into
    // the mbuf pool at the worst possible moment.
    auto o = classifySendRc(kRcENoMem);
    EXPECT_EQ(o, SendOutcome::AbortStackAlreadyFreed);
    EXPECT_FALSE(callerMustFree(o));
    EXPECT_TRUE(isFatal(o));
}

TEST(OcL2capSendPolicy, UnknownErrorTreatedAsAlreadyFreed)
{
    // Any other rc reaches the same `failed:` path -> do not free.
    for (int rc : {2, 3, 5, 7, 12, 99, -1})
    {
        auto o = classifySendRc(rc);
        EXPECT_EQ(o, SendOutcome::AbortStackAlreadyFreed) << "rc=" << rc;
        EXPECT_FALSE(callerMustFree(o)) << "rc=" << rc;
    }
}

// The invariant that keeps this safe no matter how the table evolves: caller
// frees for EXACTLY the two before-ownership codes, and for nothing else.
TEST(OcL2capSendPolicy, CallerFreesOnlyBeforeOwnershipCodes)
{
    for (int rc = -5; rc <= 40; ++rc)
    {
        const bool must = callerMustFree(classifySendRc(rc));
        const bool expected = (rc == kRcEBadData || rc == kRcEBusy);
        EXPECT_EQ(must, expected) << "rc=" << rc;
    }
}

// ---------------------------------------------------------------------------
// SDU sizing: clamp to the peer's MTU, because an oversize SDU fails AND leaks.
// ---------------------------------------------------------------------------
TEST(OcL2capSendPolicy, SduSizeClampsToPeerMtu)
{
    EXPECT_EQ(sduSize(1024, 512), 512u);    // peer smaller -> clamp
    EXPECT_EQ(sduSize(1024, 2048), 1024u);  // want smaller -> keep want
    EXPECT_EQ(sduSize(1024, 1024), 1024u);  // equal
}

TEST(OcL2capSendPolicy, SduSizeZeroWhenPeerUnknown)
{
    // peer_coc_mtu == 0 means not connected / not learned -> no legal SDU.
    EXPECT_EQ(sduSize(1024, 0), 0u);
    EXPECT_EQ(sduSize(kPreferredSduBytes, 0), 0u);
}

// ---------------------------------------------------------------------------
// msys watermark gate — the backpressure that stops silent corruption.
// ---------------------------------------------------------------------------
TEST(OcL2capSendPolicy, MsysGate)
{
    EXPECT_TRUE(msysGateOk(48, 16));
    EXPECT_TRUE(msysGateOk(16, 16));    // exactly at the mark passes
    EXPECT_FALSE(msysGateOk(15, 16));   // one below blocks
    EXPECT_FALSE(msysGateOk(0, 16));
}
