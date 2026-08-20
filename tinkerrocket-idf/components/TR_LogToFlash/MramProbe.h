#pragma once

#include <stdint.h>

// #826: the verdict half of TR_LogToFlash::mramProbe(), split out so the
// property the probe depends on can be tested on the host.
//
// The probe issues WREN, reads the status register, issues WRDI, and reads it
// again. A healthy MR25H10 shows WEL (bit 1) set after WREN and clear after
// WRDI. The point of demanding a CHANGE rather than a value is that an absent
// part leaves MISO floating, and a floating bus returns a CONSTANT — 0x00 if
// it drifts low, 0xFF if it is pulled high, or whatever the last device on the
// bus left behind. Any constant fails one of the two checks.
//
// This is why a bare RDSR would not be enough: 0x00 is a perfectly plausible
// status byte (unprotected part, WEL clear), so a bus floating low reads as a
// healthy MRAM. The two-sample form has no such blind value.
namespace tr {

static constexpr uint8_t kMramSrWel = 0x02;

// sr_after_wren / sr_after_wrdi: status register bytes read after each command.
// Returns true only if WEL went set -> clear, i.e. the part followed both
// commands. Only WEL is examined; the other bits are block-protect and
// reserved, and vary with how the part was last configured.
constexpr bool mramProbeVerdict(uint8_t sr_after_wren, uint8_t sr_after_wrdi)
{
    return ((sr_after_wren & kMramSrWel) != 0) &&
           ((sr_after_wrdi & kMramSrWel) == 0);
}

}  // namespace tr
