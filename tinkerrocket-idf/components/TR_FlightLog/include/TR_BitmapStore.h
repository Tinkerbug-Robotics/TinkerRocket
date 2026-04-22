#pragma once

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// Persistence hook for BlockStateBitmap. The real implementation on ESP-IDF
// delegates to NVS (namespace "map", same key as the legacy bad-block bitmap);
// tests use an in-memory variant.
//
// load() returns true if a valid-looking blob was read into `buf`. If no saved
// bitmap exists yet, load() should return false so the caller initializes a
// fresh all-FREE bitmap (wipe-on-first-boot behavior).
class TR_BitmapStore {
public:
    virtual ~TR_BitmapStore() = default;
    virtual bool load(uint8_t* buf, size_t len) = 0;
    virtual bool save(const uint8_t* buf, size_t len) = 0;
};

}  // namespace tr_flightlog
