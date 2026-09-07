#pragma once

#include <cstddef>
#include <cstdint>

#include "RocketComputerTypes.h"

// #1105: retire one-shot actuating commands at an FC session boundary.
//
// The relay queue and its serving slot outlive the FC. That is right for a
// config sync — the queue holds across a rail-off and drains at power-on by
// design (#366) — and wrong for a command that fires, arms or moves something:
// the operator issued it to the FC session that just ended, and delivering it
// to the next one is an actuation nobody asked for, after a gap bounded only
// by the FC's downtime. The serving slot is the acute case: the slot advances
// only on polls the OC receives, so an FC reset inside the CMD_REPEAT_LIMIT
// window freezes it with the command still served, and the rebooted FC reads
// it back as new. A queued-but-not-yet-served one-shot is the same hazard one
// step earlier, and the FC cannot see that one at all.
//
// The FC's boot-progress frames (FC_BOOT_STATUS_MSG) are the OC's positive
// evidence of a new session: emitted every 500 ms from ~250 ms into setup_fc
// until it completes, long before the boot-time status read that would
// otherwise be served the stale command. main.cpp latches one bit from them
// and acts on it at the top of queueOutStatusResponse() — the serving slot's
// only writer — so the first poll of the new session is served from a clean
// slot. The FC refuses such a command independently (oc_cmd_session_gate.h),
// which covers the case where every boot frame was missed.
//
// Pure: the ring compaction below is the whole policy, templated on the entry
// type so the host suite can drive it with a stand-in QueuedCommand and cover
// the wrap-around cases without the OC's locking.

// Should the command in the SERVING slot be retired when the FC reports a boot?
inline constexpr bool cmdQueueRetireServingOnFcBoot(uint8_t serving_cmd)
{
    return cmdIsOneShotActuating(serving_cmd);
}

// What was retired, for the caller to log OUTSIDE its critical section.
struct CmdQueueRetired
{
    uint8_t cmd;
    uint8_t sel;   // first payload byte (the pyro tests' channel); 0 if no payload
};

// Drop every one-shot actuating entry from the FIFO ring [head, head+count)
// mod N, preserving the order of the survivors; head is unchanged, count is
// updated. Entry must expose `cmd`, `cfg_len` and `cfg[]` (QueuedCommand does).
// Up to `out_cap` retired entries are recorded in `out`; the return value is
// the total retired, which can exceed out_cap. The caller owns the locking.
template <typename Entry, size_t N>
inline size_t cmdQueueRetireOneShots(Entry (&ring)[N], size_t head, size_t& count,
                                     CmdQueueRetired* out, size_t out_cap)
{
    size_t kept = 0;
    size_t retired = 0;
    for (size_t i = 0; i < count; i++)
    {
        const size_t src = (head + i) % N;
        if (cmdIsOneShotActuating(ring[src].cmd))
        {
            if (out != nullptr && retired < out_cap)
            {
                out[retired].cmd = ring[src].cmd;
                out[retired].sel = (ring[src].cfg_len > 0) ? ring[src].cfg[0] : 0U;
            }
            retired++;
            continue;
        }
        const size_t dst = (head + kept) % N;
        if (dst != src)
        {
            ring[dst] = ring[src];
        }
        kept++;
    }
    count = kept;
    return retired;
}
