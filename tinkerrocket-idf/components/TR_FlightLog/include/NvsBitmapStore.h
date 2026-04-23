#pragma once

#include "TR_BitmapStore.h"

namespace tr_flightlog {

// Concrete TR_BitmapStore backed by ESP-IDF NVS (Arduino Preferences wrapper).
// Persists the TR_FlightLog 3-state block bitmap across reboots.
//
// Uses a distinct NVS namespace from TR_LogToFlash's own bad-block map
// ("bblk"/"map"). The layouts are not compatible: TR_LogToFlash stores a
// 128-byte 1-bit-per-block map, TR_FlightLog stores a 256-byte 2-bit map.
// Sharing the namespace would cause silent corruption on upgrade.
//
// On host / in tests the implementation compiles to stubs that always return
// false — callers (TR_FlightLog::begin) treat that as "fresh chip" and seed
// from the backend's bad-block oracle, which is the correct behaviour for
// host tests using FakeNandBackend.
class NvsBitmapStore : public TR_BitmapStore {
public:
    explicit NvsBitmapStore(const char* namespace_name = "flightlog",
                            const char* key = "bitmap")
        : ns_(namespace_name), key_(key) {}

    bool load(uint8_t* buf, size_t len) override;
    bool save(const uint8_t* buf, size_t len) override;

private:
    const char* ns_;
    const char* key_;
};

}  // namespace tr_flightlog
