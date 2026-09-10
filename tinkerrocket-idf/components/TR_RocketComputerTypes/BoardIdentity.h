#pragma once
// Board identity that survives a reflash (#773 step 2).
//
// THE PROBLEM. The only hardware signal on the wire today is the `-v9` suffix
// inside the running image's own version string, and TR_BOARD_REV_STR beside
// it. Both come from the image, so both are circular: a board flashed with the
// wrong build reports the wrong board forever, and reports it confidently.
//
// THE FIX is not to replace that value but to have a SECOND, independent one —
// written to NVS when the board is provisioned, surviving every OTA — and to
// treat a DISAGREEMENT between the two as the signal. "What this board is" and
// "what this image was built for" are different questions, and the interesting
// answer is when they differ.
//
// This header is the pure part: normalising a revision string, comparing two,
// and the FC->OC payload encoding. NVS and BLE live in the projects.
//
// The payload rides the EXISTING FC_IDENTITY message, which is already a
// variable-length string the OC copies with a bounded memcpy — no struct, no
// static_assert, no fixed-width wire field to keep in step on two ends. That
// matters: growing a packed struct across the FC<->OC link is the change class
// that killed the whole command path in #1257.

#include <compat.h>
#include <stddef.h>
#include <string.h>

namespace board_identity {

/// Longest canonical revision we store. "V10" and "M1" fit with room; the cap
/// exists so a corrupt NVS value cannot become an unbounded string on the wire.
static constexpr size_t kMaxRev = 7;

/// Canonicalise a revision string: upper-case, and cut at the first character
/// that is not a letter or digit.
///
/// This is what lets the compile-time constants be compared against a
/// provisioned value without either side having to agree on prose.
/// TR_BOARD_REV_STR is "V9/V10" on one build and "M1 (rocket-computer-mini)"
/// on another; both reduce to a token. "V9/V10" reduces to "V9", which is
/// deliberate — those two revisions share an image, so a V10 board provisioned
/// as "V10" is a genuine mismatch worth showing rather than hiding.
///
/// Returns the number of characters written (0 when there is nothing usable).
inline size_t normalizeRev(const char* in, char* out, size_t out_len)
{
    if (out == nullptr || out_len == 0) return 0;
    out[0] = '\0';
    if (in == nullptr) return 0;
    size_t n = 0;
    for (size_t i = 0; in[i] != '\0' && n < out_len - 1; ++i)
    {
        const char c = in[i];
        const bool is_alnum = (c >= '0' && c <= '9') ||
                              (c >= 'a' && c <= 'z') ||
                              (c >= 'A' && c <= 'Z');
        if (!is_alnum)
        {
            if (n > 0) break;      // token ended
            continue;              // leading junk: skip it
        }
        out[n++] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
    }
    out[n] = '\0';
    return n;
}

/// True when two revision strings name the same board, after normalising.
/// An empty side is NOT a match — "unprovisioned" must never read as agreement.
inline bool revsMatch(const char* a, const char* b)
{
    char na[kMaxRev + 1], nb[kMaxRev + 1];
    const size_t la = normalizeRev(a, na, sizeof(na));
    const size_t lb = normalizeRev(b, nb, sizeof(nb));
    if (la == 0 || lb == 0) return false;
    return strcmp(na, nb) == 0;
}

/// Encode the FC->OC identity payload: the version string, then a NUL, then the
/// provisioned revision. Returns the byte count written.
///
/// Old OC, new FC: the OC's bounded memcpy into a zeroed buffer yields the
/// version as a C string and simply never looks past the NUL. Nothing breaks.
/// New OC, old FC: no second field, so the revision reads empty, which is
/// exactly "unprovisioned" — the honest answer for a board that has not been.
inline size_t encodePayload(const char* version, const char* rev,
                            char* out, size_t out_len)
{
    if (out == nullptr || out_len == 0) return 0;
    size_t n = 0;
    if (version != nullptr)
        for (size_t i = 0; version[i] != '\0' && n < out_len - 1; ++i) out[n++] = version[i];
    if (n >= out_len - 1) { out[out_len - 1] = '\0'; return out_len - 1; }
    out[n++] = '\0';
    char norm[kMaxRev + 1];
    const size_t rn = normalizeRev(rev, norm, sizeof(norm));
    for (size_t i = 0; i < rn && n < out_len; ++i) out[n++] = norm[i];
    return n;
}

/// Split a received payload back into its two fields. `rev_out` is left empty
/// when the payload carries only a version, which is the pre-#773 shape.
inline void decodePayload(const char* payload, size_t len,
                          char* version_out, size_t version_len,
                          char* rev_out, size_t rev_len)
{
    if (version_out != nullptr && version_len > 0) version_out[0] = '\0';
    if (rev_out != nullptr && rev_len > 0) rev_out[0] = '\0';
    if (payload == nullptr || len == 0) return;

    size_t split = 0;
    while (split < len && payload[split] != '\0') ++split;

    if (version_out != nullptr && version_len > 0)
    {
        const size_t n = (split < version_len - 1) ? split : version_len - 1;
        memcpy(version_out, payload, n);
        version_out[n] = '\0';
    }
    if (rev_out == nullptr || rev_len == 0 || split >= len) return;

    const char* rev = payload + split + 1;
    size_t rlen = (split + 1 <= len) ? len - split - 1 : 0;
    // The tail may be NUL-padded by a fixed-size sender; stop at the first NUL.
    size_t r = 0;
    while (r < rlen && rev[r] != '\0') ++r;
    char raw[kMaxRev + 1];
    const size_t n = (r < sizeof(raw) - 1) ? r : sizeof(raw) - 1;
    memcpy(raw, rev, n);
    raw[n] = '\0';
    normalizeRev(raw, rev_out, rev_len);
}

}  // namespace board_identity
