/**
 * fake_write_sink.h — Stand-in for the TR_FlightLog write sink that
 *                     TR_LogToFlash drains into (issue #50 Stage 2c-3c).
 *
 * The real sink wraps each payload in a 16-byte PageHeader and programs one
 * NAND page, returning false when it could not place it — a full flight
 * region, or the page landing on a run of bad blocks.  Everything downstream
 * of TR_LogToFlash's accounting hangs off that boolean, so the fake's whole
 * job is to make it choosable and to record what was offered.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

struct FakeWriteSink
{
    struct Call
    {
        size_t len;
        bool   accepted;
    };

    /// Decides the fate of one page.  Called BEFORE the call is recorded, so
    /// `sink.calls.size()` inside the policy is the current call's 0-based
    /// index.  Unset means "accept everything".
    std::function<bool(const FakeWriteSink& sink, const uint8_t* payload, size_t len)> policy;

    std::vector<Call>    calls;
    std::vector<uint8_t> accepted_stream;   // concatenated accepted payloads
    uint64_t             accepted_bytes = 0;

    /// The C-style trampoline installed as cfg.write_sink; ctx is the sink.
    static bool trampoline(void* ctx, const uint8_t* payload, size_t len)
    {
        return static_cast<FakeWriteSink*>(ctx)->onWrite(payload, len);
    }

    bool onWrite(const uint8_t* payload, size_t len)
    {
        const bool ok = policy ? policy(*this, payload, len) : true;
        calls.push_back({len, ok});
        if (ok)
        {
            accepted_stream.insert(accepted_stream.end(), payload, payload + len);
            accepted_bytes += len;
        }
        return ok;
    }

    size_t callCount() const { return calls.size(); }

    /// Reject every page whose length is short of a full sink payload.  A
    /// session has exactly one such page — the tail flushed by
    /// closeLogSession — so this is the "the final partial page could not be
    /// placed" case and nothing else.
    static std::function<bool(const FakeWriteSink&, const uint8_t*, size_t)>
    rejectShortPages(size_t full_payload)
    {
        return [full_payload](const FakeWriteSink&, const uint8_t*, size_t len) {
            return len == full_payload;
        };
    }

    /// Reject the page at `index` and take every other one.
    static std::function<bool(const FakeWriteSink&, const uint8_t*, size_t)>
    rejectCall(size_t index)
    {
        return [index](const FakeWriteSink& s, const uint8_t*, size_t) {
            return s.calls.size() != index;
        };
    }
};
