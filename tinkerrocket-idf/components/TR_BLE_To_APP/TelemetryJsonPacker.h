#pragma once
// The telemetry JSON packer (#282, #283): one BLE notification's frame, built
// field by field in priority order and capped at the link's payload window
// (ATT MTU - 3). A field that does not fit is left out, the frame says so with
// "tr":1, and the app shows "Frame trimmed for link bandwidth" for 5 s.
//
// Each field is measured at the width it really prints. Until 2026-11-22 every
// number was charged 24 bytes, its widest possible value, though most print in
// 3-8, so a frame within ~30 B of the window lost its last fields even where
// they would have fit. A V10's direct frame with a fix (~483 B of a 509 B
// window) had ~5 B to spare at its last number; a base station's relay of one
// (its own battery and pack, the link, the rocket's name: ~545 B) must trim, and
// lost the dropped-frame count, all three high-g axes and the log filename,
// where now the high-g's Y and Z, the rocket's name and the filename go. A number is printed into
// a scratch buffer first and fit-checked at its length; one too long for that
// buffer (an absurd value, |x| > ~1e30) is left out like a field with no room,
// never written past it (the old code advanced by snprintf's would-be length).
//
// Pure (no NimBLE, no Arduino String), so the host tests run it:
// tests_cpp/test_telemetry_json_packer.cpp.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace tr_ble {

class TelemetryJsonPacker
{
public:
    // Held back from every fit check: the closing '}' and the NUL (2), and a
    // ',"tr":1' (7), so the trim flag always fits on a fully packed frame.
    static constexpr size_t kReserve = 2 + 7;

    // buf holds bufsize bytes (at least kReserve + 1). The finished frame, '}'
    // included, is at most max_notify bytes long, and never past the buffer.
    TelemetryJsonPacker(char* buf, size_t bufsize, size_t max_notify)
        : buf_(buf), cap_((max_notify + 1 < bufsize) ? (max_notify + 1) : bufsize)
    {
        buf_[pos_++] = '{';
    }

    void addInt(const char* key, int value)
    {
        char t[16];
        addNumber(key, t, sizeof(t), snprintf(t, sizeof(t), "%d", value));
    }

    void addUint(const char* key, uint32_t value)
    {
        char t[16];
        addNumber(key, t, sizeof(t), snprintf(t, sizeof(t), "%lu", (unsigned long)value));
    }

    // A NaN or an infinity is left out, unflagged: NaN is how a field says it
    // has no value (the omission saves bytes), and "inf" is not JSON, so it
    // would cost the app the whole frame.
    void addFloat(const char* key, float value, int decimals)
    {
        addDouble(key, (double)value, decimals);
    }

    void addDouble(const char* key, double value, int decimals)
    {
        if (!std::isfinite(value)) return;
        char t[40];
        addNumber(key, t, sizeof(t), snprintf(t, sizeof(t), "%.*f", decimals, value));
    }

    // Copied as given (the callers' strings need no escaping: state names,
    // firmware versions, file names, a unit name validated at cmd 40).
    void addString(const char* key, const char* value)
    {
        const size_t vlen = value ? strlen(value) : 0;
        if (!fits(keyBytes(key) + 2 + vlen)) { dropped_ = true; return; }
        appendKey(key);
        buf_[pos_++] = '"';
        if (vlen) memcpy(buf_ + pos_, value, vlen);
        pos_ += vlen;
        buf_[pos_++] = '"';
    }

    // The Tier 1 boundary. If anything above it was left out, nothing below it
    // goes in: the packer is best-fit (a field that does not fit is skipped and
    // a smaller one after it may still go in), so without this floor a tiny MTU
    // kept the least important field and lost every vital one (bench
    // 2026-08-03: {"af":"","tr":1}). Best-fit stays within Tiers 2 and 3.
    void endTier1() { tier1_incomplete_ = dropped_; }

    // Closes the frame: "tr":1 if anything was left out, then '}' and the NUL.
    // Returns its length (without the NUL).
    size_t finish()
    {
        if (dropped_)
        {
            appendKey("tr");
            buf_[pos_++] = '1';
        }
        buf_[pos_++] = '}';
        buf_[pos_] = '\0';
        return pos_;
    }

    bool dropped() const { return dropped_; }

private:
    // The key's bytes: a ',' unless it is the first field, its quotes, the ':'.
    size_t keyBytes(const char* key) const { return (first_ ? 0 : 1) + 2 + strlen(key) + 1; }

    bool fits(size_t n) const
    {
        if (tier1_incomplete_) return false;
        return pos_ + n + kReserve <= cap_;
    }

    void appendKey(const char* key)
    {
        if (!first_) buf_[pos_++] = ',';
        first_ = false;
        buf_[pos_++] = '"';
        const size_t klen = strlen(key);
        memcpy(buf_ + pos_, key, klen);
        pos_ += klen;
        buf_[pos_++] = '"';
        buf_[pos_++] = ':';
    }

    // text holds n printed bytes of a tcap-byte scratch buffer (snprintf's
    // return: negative on an error, tcap or more when it did not fit).
    void addNumber(const char* key, const char* text, size_t tcap, int n)
    {
        if (n < 0 || (size_t)n >= tcap || !fits(keyBytes(key) + (size_t)n))
        {
            dropped_ = true;
            return;
        }
        appendKey(key);
        memcpy(buf_ + pos_, text, (size_t)n);
        pos_ += (size_t)n;
    }

    char*  buf_;
    size_t cap_;
    size_t pos_ = 0;
    bool   first_ = true;
    bool   dropped_ = false;
    bool   tier1_incomplete_ = false;
};

}  // namespace tr_ble
