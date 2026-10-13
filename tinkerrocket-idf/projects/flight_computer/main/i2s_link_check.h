#pragma once

// The FC->OC I2S link as one view of it sees it, for the bench monitor
// (i2s_tx_monitor.h, TR_I2S_TX_MONITOR):
//   FrameStream    reassembles frames from consecutive DMA buffers the way
//                  the OC's ring parser does (SOF, length, CRC). A frame that
//                  straddles a buffer boundary is carried into the next one,
//                  and so is a tail too short to rule a SOF in or out:
//                  dropping that tail loses every frame that starts in the
//                  last 7 bytes of a buffer, ~2% of them at the 88200 link.
//   WriteLog       the sender's own record of every frame it wrote;
//   Matcher        one view's frames matched against it in write order:
//                  matched, late (seen after frames written later), or not in
//                  the log; writes skipped over and never seen are missing.
//   ImuGapTracker  ISM6 batch sample times as they arrive: a jump of more
//                  than three periods is a gap, the rule the log analysis uses.
//
// Header-only and IDF-free so the host tests pin it.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc16_fast.h"

namespace i2slink
{

// A frame's identity across views: its type, its length and the first bytes
// of its payload (every streamed type leads with its time_us).
inline uint32_t frameKey(uint8_t type, uint8_t len, const uint8_t *payload)
{
    uint32_t h = 2166136261u;
    h = (h ^ type) * 16777619u;
    h = (h ^ len) * 16777619u;
    const size_t n = len < 9 ? len : 9;
    for (size_t i = 0; i < n; ++i) h = (h ^ payload[i]) * 16777619u;
    return h | 1u;   // never 0, which callers may use for an empty slot
}

// A frame found in the stream: what it is, and where its first byte was.
struct FrameAt
{
    uint8_t type;
    uint8_t len;
    const uint8_t *payload;   // valid only inside the callback
    uint32_t buf_id;          // the feed() call its first byte came in
    size_t offset;            // and its offset in that buffer
};

// BUF_MAX: the largest buffer fed, at least one whole frame.
template <size_t BUF_MAX>
class FrameStream
{
public:
    static constexpr size_t MAX_FRAME = 4 + 1 + 1 + 255 + 2;

    // max_payload: longer lengths are a false SOF, skipped as the OC does.
    explicit FrameStream(size_t max_payload) : max_payload_(max_payload) {}

    // One DMA buffer, in the order the DMA moved them; on_frame(const FrameAt&)
    // for each frame whose CRC checks.
    template <class Fn>
    void feed(const uint8_t *buf, size_t len, uint32_t buf_id, Fn &&on_frame)
    {
        if (len > BUF_MAX) len = BUF_MAX;
        memcpy(win_ + carry_, buf, len);
        const size_t n = carry_ + len;
        size_t i = 0;
        size_t keep = n;   // first byte carried into the next buffer
        while (i < n)
        {
            const uint8_t *p = static_cast<const uint8_t *>(memchr(win_ + i, 0xAA, n - i));
            if (p == nullptr) break;
            i = (size_t)(p - win_);
            if (n - i < 6)
            {
                keep = i;   // too short to tell: decide with the next buffer
                break;
            }
            if (win_[i + 1] != 0x55 || win_[i + 2] != 0xAA || win_[i + 3] != 0x55)
            {
                ++i;
                continue;
            }
            const uint8_t type = win_[i + 4];
            const uint8_t plen = win_[i + 5];
            if (plen > max_payload_)
            {
                ++i;
                continue;
            }
            const size_t fl = 4 + 1 + 1 + (size_t)plen + 2;
            if (n - i < fl)
            {
                keep = i;   // the rest is in the next buffer
                break;
            }
            const uint16_t crc = (uint16_t)((win_[i + 6 + plen] << 8) | win_[i + 7 + plen]);
            if (crc16Frame(win_ + i + 4, (size_t)plen + 2) != crc)
            {
                ++crc_failures_;
                ++i;
                continue;
            }
            const bool in_carry = i < carry_;
            on_frame(FrameAt{ type, plen, win_ + i + 6,
                              in_carry ? carry_id_ : buf_id,
                              in_carry ? carry_off_ + i : i - carry_ });
            i += fl;
        }
        if (keep < n)
        {
            if (keep >= carry_)
            {
                carry_id_ = buf_id;
                carry_off_ = keep - carry_;
            }
            else
            {
                carry_off_ += keep;   // still in the old carry (a short buffer was fed)
            }
            carry_ = n - keep;
            memmove(win_, win_ + keep, carry_);
        }
        else
        {
            carry_ = 0;
        }
    }

    // The stream restarted (a new channel): forget a partial frame.
    void reset() { carry_ = 0; }

    uint32_t crcFailures() const { return crc_failures_; }

private:
    static_assert(BUF_MAX >= MAX_FRAME, "a frame must fit in two buffers");
    size_t max_payload_;
    uint8_t win_[BUF_MAX + MAX_FRAME];
    size_t carry_ = 0;
    uint32_t carry_id_ = 0;
    size_t carry_off_ = 0;
    uint32_t crc_failures_ = 0;
};

// One frame the sender wrote: when it began, and what it was.
struct Write
{
    uint32_t t_us;
    uint32_t key;
    uint8_t type;
};

// N, a power of two: the writes kept. One producer (the sender task); the
// monitor reads on the same core, so a plain index publishes each record.
template <uint32_t N>
class WriteLog
{
public:
    void add(uint8_t type, uint8_t len, const uint8_t *payload, uint32_t t_us)
    {
        const uint32_t h = head_;
        Write &w = ring_[h & (N - 1)];
        w.t_us = t_us;
        w.key = frameKey(type, len, payload);
        w.type = type;
        head_ = h + 1;
    }
    uint32_t head() const { return head_; }
    const Write &at(uint32_t i) const { return ring_[i & (N - 1)]; }

private:
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
    Write ring_[N];
    volatile uint32_t head_ = 0;
};

// One view's frames, matched in write order against a WriteLog.
template <uint32_t N>
class Matcher
{
public:
    enum class Kind : uint8_t
    {
        Matched,    // *w is its write
        Late,       // a write skipped over earlier, seen now
        NotInLog,   // nothing written matches (a replay, or another source)
    };

    static constexpr uint32_t WINDOW = 96;   // writes searched past the last match
    static constexpr uint32_t HOLD = 64;     // skipped writes kept for a late arrival

    Kind seen(const WriteLog<N> &log, uint8_t type, uint32_t key, uint32_t now_us, Write *w)
    {
        const uint32_t head = log.head();
        if (head - next_ > N) next_ = head - N;   // fell a whole log behind
        for (uint32_t j = next_; j != head && j - next_ < WINDOW; ++j)
        {
            const Write &c = log.at(j);
            if (c.key == key && c.type == type)
            {
                for (uint32_t k = next_; k != j; ++k) hold(log.at(k), now_us);
                next_ = j + 1;
                *w = c;
                return Kind::Matched;
            }
        }
        for (uint32_t k = 0; k < n_held_; ++k)
        {
            if (held_[k].w.key == key && held_[k].w.type == type)
            {
                held_[k] = held_[--n_held_];
                return Kind::Late;
            }
        }
        return Kind::NotInLog;
    }

    // Writes skipped over and still unseen after hold_us are missing:
    // on_missing(const Write&) once each.
    template <class Fn>
    void expire(uint32_t now_us, uint32_t hold_us, Fn &&on_missing)
    {
        for (uint32_t k = 0; k < n_held_;)
        {
            if ((uint32_t)(now_us - held_[k].since_us) > hold_us)
            {
                on_missing(held_[k].w);
                held_[k] = held_[--n_held_];
            }
            else
            {
                ++k;
            }
        }
    }

    // Writes skipped while the hold list was full: missing, not itemised.
    uint32_t untracked() const { return untracked_; }

private:
    struct Held
    {
        Write w;
        uint32_t since_us;
    };
    void hold(const Write &w, uint32_t now_us)
    {
        if (n_held_ < HOLD) held_[n_held_++] = Held{ w, now_us };
        else ++untracked_;
    }
    uint32_t next_ = 0;
    Held held_[HOLD];
    uint32_t n_held_ = 0;
    uint32_t untracked_ = 0;
};

// ISM6 batch sample times in arrival order.
class ImuGapTracker
{
public:
    // record_bytes: one ISM6HG256Data record, which starts with its time_us.
    explicit ImuGapTracker(size_t record_bytes) : rec_(record_bytes) {}

    // payload: an ISM6_BATCH_MSG payload, a count byte then the records.
    void batch(const uint8_t *payload, size_t len)
    {
        if (len < 1) return;
        const size_t n = payload[0];
        if (n == 0 || len < 1 + n * rec_) return;
        const uint32_t t0 = rd(payload + 1);
        const uint32_t tl = rd(payload + 1 + (n - 1) * rec_);
        if (n >= 2)
        {
            const uint32_t p = rd(payload + 1 + rec_) - t0;
            if (p > 0 && p < 100000) period_ = p;
        }
        if (have_ && period_ > 0)
        {
            const int32_t d = (int32_t)(t0 - last_);
            const int32_t per = (int32_t)period_;
            if (d > 3 * per)
            {
                ++gaps;
                lost += (uint32_t)((d + per / 2) / per - 1);
            }
        }
        // A late batch neither counts as a gap nor moves the clock back.
        if (!have_ || (int32_t)(tl - last_) > 0) last_ = tl;
        have_ = true;
    }

    uint32_t gaps = 0;
    uint32_t lost = 0;   // samples

private:
    static uint32_t rd(const uint8_t *p)
    {
        uint32_t v;
        memcpy(&v, p, sizeof v);
        return v;
    }
    size_t rec_;
    uint32_t period_ = 0;
    uint32_t last_ = 0;
    bool have_ = false;
};

}  // namespace i2slink
