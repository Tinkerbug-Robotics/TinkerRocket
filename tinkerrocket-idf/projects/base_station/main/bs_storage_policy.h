#pragma once

// Storage bring-up retry policy (#761).
//
// mountExternalFlashFat() used to be single-shot: any one of its four steps
// (spi_bus_initialize / spi_bus_add_device / spi_nand_flash_init_device /
// esp_vfs_fat_nand_mount) returned straight out to the caller, which then
// mounted the ~1.9 MB internal SPIFFS partition and logged there for the whole
// boot instead of the 512 MB NAND. A flight logged in that state truncates.
//
// Observed on one BaseStation an hour apart, same firmware: "Internal flash --
// Used 1.3 MB, Free 0.4 MB" after a flash, "External NAND -- Used 0.1 MB, Free
// 459.1 MB" after a plain reset. Ten scripted warm resets all came up healthy,
// so a warm boot is not the trigger; cold power-up, or the reset that follows a
// flash, is.
//
// The decisions the retry loop makes are small, and every one of them is an
// off-by-one away from being wrong in a way no bench test would catch (format
// on the wrong attempt destroys every stored flight; a fencepost on the last
// attempt silently drops the reduced-clock retry). So they live here, pure --
// no ESP-IDF / hardware dependencies -- and the host gtest drives them
// directly. Same pattern as bs_uplink_policy.h / bs_battery_soc.h.

#include <cstdint>

namespace bs_storage_policy {

// What one bring-up attempt should do.
struct Attempt
{
    uint32_t clock_hz;      // SPI clock for this attempt
    bool     allow_format;  // may esp_vfs_fat_nand_mount run f_mkfs on failure?
    bool     is_last;       // no further attempt follows
};

// Plan attempt `n` (1-based) of `attempts`.
//
// clock: every attempt but the last runs at the normal bring-up clock. The last
// halves it -- 20 MHz is already deliberately conservative for a part rated
// 166 MHz, so if a unit only ever comes up on the reduced clock the answer is
// signal integrity rather than timing, and that is worth being able to read off
// a field log. Half-speed NAND still beats a 1.9 MB partition.
//
// allow_format: ONLY the last attempt may format. A transient FTL or mount
// error that a retry would have fixed must not reach f_mkfs, because that
// trades a recoverable boot for every flight log on the chip. A genuinely blank
// NAND still gets formatted, just one attempt later.
inline constexpr Attempt planAttempt(uint8_t n, uint8_t attempts,
                                     uint32_t clock_hz, uint32_t fallback_clock_hz)
{
    const bool last = (n >= attempts);
    return Attempt{ last ? fallback_clock_hz : clock_hz, last, last };
}

// True when this board is fitted with primary storage (SPI NAND or an SD slot)
// and we ended up on internal SPIFFS anyway -- i.e. a demotion, not a
// configuration. Kept separate from "using internal flash" because on a board
// with no primary storage at all SPIFFS would be the intended backend, and
// alarming about it would be noise.
inline constexpr bool demoted(bool has_primary_storage, bool using_internal_flash)
{
    return has_primary_storage && using_internal_flash;
}

// True when primary storage IS up, but only after at least one attempt failed.
// Healthy for this boot and not worth alarming over -- but a unit that reports
// it every boot is on its way to demoting, which is worth surfacing quietly
// rather than discovering at the pad.
inline constexpr bool recovered(bool demoted_now, uint8_t attempts_used)
{
    return !demoted_now && attempts_used > 1;
}

}  // namespace bs_storage_policy
