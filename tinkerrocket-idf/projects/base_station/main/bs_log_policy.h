#pragma once

// Base-station LoRa CSV logging policy helpers (#137).
//
// Pure functions extracted from main.cpp so the host-side gtest can drive
// them without dragging in the LoRa radio, SD card, or BLE stack.  Anything
// in here must remain free of ESP-IDF / FreeRTOS / hardware dependencies.

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace bs_log_policy {

// Strict parser for the sequential lora_NNN.csv filenames used when the BS
// has no phone time-sync yet.  Returns true and sets `out_num` only when
// `name` exactly matches that pattern with the entire string consumed.
//
// Pre-fix, findNextFileNumber() used a bare `sscanf("lora_%hu.csv", &num)`,
// which sscanf will happily satisfy with partial matches:
//
//   sscanf("lora_20260509_164143.csv", "lora_%hu.csv", &num) -> 1, num=9885
//
// because `%hu` consumes the leading digits "20260509" (truncating to the
// low 16 bits = 9885), and the trailing format mismatch on '.' is not
// reflected in the return value.  The result was that any timestamped file
// on the SD inflated max_num past every real sequential, so the next
// no-time-sync boot produced silly names like `lora_9886.csv` even when the
// card had only ever held `lora_001.csv` next to timestamped flights (#137).
//
// Walks digits explicitly (rather than letting sscanf %hu accept signs /
// whitespace and wrap), so any non-digit between "lora_" and ".csv"
// disqualifies the match.
static inline bool parseSequentialFilename(const char* name, uint16_t& out_num)
{
    if (name == nullptr) return false;
    const size_t len = strlen(name);
    constexpr const char PREFIX[] = "lora_";
    constexpr const char SUFFIX[] = ".csv";
    constexpr size_t PREFIX_LEN = sizeof(PREFIX) - 1;
    constexpr size_t SUFFIX_LEN = sizeof(SUFFIX) - 1;
    if (len <= PREFIX_LEN + SUFFIX_LEN) return false;
    if (memcmp(name, PREFIX, PREFIX_LEN) != 0) return false;
    if (memcmp(name + len - SUFFIX_LEN, SUFFIX, SUFFIX_LEN) != 0) return false;
    const size_t digits = len - PREFIX_LEN - SUFFIX_LEN;
    if (digits == 0 || digits > 5) return false;
    uint32_t value = 0;
    for (size_t i = 0; i < digits; i++) {
        const char c = name[PREFIX_LEN + i];
        if (c < '0' || c > '9') return false;
        value = value * 10 + (uint32_t)(c - '0');
    }
    if (value > 0xFFFFu) return false;  // doesn't fit uint16
    out_num = (uint16_t)value;
    return true;
}

} // namespace bs_log_policy
