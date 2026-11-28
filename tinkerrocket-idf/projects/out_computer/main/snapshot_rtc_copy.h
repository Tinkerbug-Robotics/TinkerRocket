// snapshot_rtc_copy.h — the OC's flight-snapshot cache, copied into RTC
// memory (#1717).
//
// The RAM cache (#846) holds the FC's newest snapshot, and it is what answers
// the FC's ask after an FC-only reset. When the OC resets too, the cache is
// gone. RTC memory survives a panic, a watchdog, a software restart and the
// OC's own brownout detector, which restarts the chip with RTC memory intact
// (#825's rail restore depends on that). It does not survive a power-on, and a
// sag deep enough to reset the chip itself is one: the chip reports it as a
// power-on and RTC memory is lost. So the OC also keeps the cache here, and a
// boot that resumes a flight seeds the RAM cache back from it before the NAND
// log is looked at.
//
// That copy is the newest frame the OC held. It includes the frame the FC
// sends the moment a pyro channel starts (#1760): the FC holds the pulse until
// that frame has left its I2S ring, plus a margin, so the OC has it at the
// pulse unless its own receive and parse lag behind. Restored from it, the FC
// treats that channel as spent and does not fire it again. The NAND log may
// not have that frame on its pages yet when the power goes.
//
// Being the newest, it also has the last word: on a boot resuming a flight, a
// copy that is intact but not servable (the FC's LANDED clear, or an
// in-flight frame older than the TTL) says the flight is over, and no older
// store may serve one (Verdict::Over).
//
// What makes it safe to serve:
//   - a CRC over the stamp and the frame, so a write torn by the reset, or the
//     garbage a power-on leaves, is never read as a frame;
//   - its age, from the RTC counter, which runs across the same resets the
//     memory survives: the stamp is the counter when the frame was stored, and
//     a counter below it means a power-on restarted it. The caller applies the
//     RAM cache's own serve TTL to that age, so a copy left by an abandoned
//     flight expires as the cache does;
//   - the caller seeds from it only on a boot that is resuming a flight (the
//     #825 rail restore or the #1176 token), and retires it on every other
//     boot. The frame's own checks (INFLIGHT, its flight time) are the
//     caller's snapshotServable(), as for every store.
//
// Pure and host-tested (tests_cpp/test_snapshot_reseed.cpp). The struct lives
// in RTC_NOINIT memory in main.cpp; only the parser task writes it (on every
// cache write) and only setup reads it, so it needs no lock.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <CRC.h>

namespace SnapshotRtcCopy {

inline constexpr uint32_t kMagic = 0x53524331;   // "SRC1"

template <size_t N>
struct Copy
{
    uint32_t magic;
    uint32_t crc;        // CRC32 over stamp_us and frame, which are contiguous
    uint64_t stamp_us;   // the RTC counter when the frame was stored
    uint8_t  frame[N];   // the snapshot wire frame, as the RAM cache holds it
};

template <size_t N>
inline uint32_t crcOf(const Copy<N>& c)
{
    static_assert(offsetof(Copy<N>, frame) == offsetof(Copy<N>, stamp_us) + sizeof(uint64_t),
                  "the CRC covers stamp_us and frame as one run");
    return calcCRC32(reinterpret_cast<const uint8_t*>(&c.stamp_us),
                     (crc_size_t)(sizeof(uint64_t) + N));
}

/// Store a frame, stamped with the RTC counter `now_us`.
template <size_t N>
inline void store(Copy<N>& c, const uint8_t* frame, uint64_t now_us)
{
    c.stamp_us = now_us;
    memcpy(c.frame, frame, N);
    c.crc   = crcOf(c);
    c.magic = kMagic;
}

/// Its magic and CRC hold: not torn, not a power-on's garbage.
template <size_t N>
inline bool intact(const Copy<N>& c)
{
    return c.magic == kMagic && c.crc == crcOf(c);
}

/// Its age in ms at the RTC counter `now_us`: false when it is not intact, or
/// the counter is below its stamp (a power-on restarted it).
template <size_t N>
inline bool ageMs(const Copy<N>& c, uint64_t now_us, uint32_t& age_ms)
{
    if (!intact(c) || now_us < c.stamp_us) return false;
    const uint64_t a = (now_us - c.stamp_us) / 1000u;
    age_ms = a > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)a;
    return true;
}

/// What the copy says on this boot.
enum class Verdict : uint8_t
{
    NotResuming,   // the boot is not resuming a flight: retire it, serve nothing
    Absent,        // not intact, or a power-on restarted the counter: the NAND
                   // log decides
    Seed,          // seed the RAM cache from it
    Over,          // intact but not servable: the flight is over, and no older
                   // store may serve one
};

/// `recovery_context`: the #825 rail restore or the #1176 token. `aged`:
/// ageMs() succeeded. `frame_servable`: the caller's frame checks (INFLIGHT,
/// its flight time) passed.
inline constexpr Verdict verdict(bool recovery_context, bool aged, uint32_t age_ms,
                                 uint32_t ttl_ms, bool frame_servable)
{
    if (!recovery_context) return Verdict::NotResuming;
    if (!aged) return Verdict::Absent;
    return (age_ms < ttl_ms && frame_servable) ? Verdict::Seed : Verdict::Over;
}

template <size_t N>
inline void retire(Copy<N>& c)
{
    c.magic = 0;
}

}  // namespace SnapshotRtcCopy
