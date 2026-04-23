#include "NvsBitmapStore.h"

#ifdef ESP_PLATFORM
#include <TR_NVS.h>  // Arduino-compatible Preferences shim over ESP-IDF nvs_flash
#endif

namespace tr_flightlog {

bool NvsBitmapStore::load(uint8_t* buf, size_t len) {
#ifdef ESP_PLATFORM
    Preferences prefs;
    // Read-only open; fails cleanly when the namespace doesn't exist yet
    // (fresh chip / first boot of new firmware) — caller then seeds from
    // the NAND backend's bad-block oracle.
    if (!prefs.begin(ns_, /*readOnly=*/true)) return false;
    const size_t got = prefs.getBytes(key_, buf, len);
    prefs.end();
    return got == len;
#else
    (void)buf; (void)len;
    return false;
#endif
}

bool NvsBitmapStore::save(const uint8_t* buf, size_t len) {
#ifdef ESP_PLATFORM
    Preferences prefs;
    if (!prefs.begin(ns_, /*readOnly=*/false)) return false;
    prefs.putBytes(key_, buf, len);
    prefs.end();
    return true;
#else
    (void)buf; (void)len;
    return false;
#endif
}

}  // namespace tr_flightlog
