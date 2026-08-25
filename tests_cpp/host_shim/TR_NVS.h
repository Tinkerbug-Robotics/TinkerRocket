/**
 * TR_NVS.h — Host stub for the Preferences wrapper over ESP-IDF NVS.
 *
 * Backed by an in-process map rather than by "always fails to open": the
 * component's bad-block bookkeeping is a load/persist ROUND TRIP (#47's
 * bitmap, #511's "scanned" marker, #671's "gpage" geometry stamp), and a
 * store that drops every write would make begin() re-run the boot scan on
 * every construction and hide any regression in that gate.
 *
 * The store is process-global, exactly like the real NVS partition is
 * board-global.  Tests that stand up more than one TR_LogToFlash must call
 * _host_shim::nvsReset() between them, or the second begin() inherits the
 * first one's persisted map — which is realistic, but rarely what a test
 * meant.
 */
#pragma once

// host_shim/Arduino.h defines Arduino's `map(x, a, b, c, d)` macro, which
// mangles every declaration of std::map the moment <map> is parsed.  The
// component headers pull Arduino.h in first, so the macro is always live by
// the time this file is reached.  Hide it across the STL includes only,
// rather than dropping it for everyone.
#pragma push_macro("map")
#undef map
#include <map>
#include <string>
#include <vector>
#pragma pop_macro("map")

#include <cstdint>
#include <cstring>

namespace _host_shim {

using NvsNamespace = std::map<std::string, std::vector<uint8_t>>;

inline std::map<std::string, NvsNamespace>& nvsStore()
{
    static std::map<std::string, NvsNamespace> store;
    return store;
}

/// Wipe the simulated NVS partition — the host equivalent of erasing the
/// nvs partition between boots.
inline void nvsReset() { nvsStore().clear(); }

}  // namespace _host_shim

class Preferences {
public:
    ~Preferences() { end(); }

    bool begin(const char* ns, bool readOnly = false)
    {
        if (opened_) end();
        ns_ = ns;
        read_only_ = readOnly;
        // Mirror the real behaviour that shapes the caller's control flow: a
        // READ-ONLY open of a namespace nobody has written yet fails, and
        // loadBadBlocksFromNVS treats that as "first boot, start clean".
        if (readOnly && _host_shim::nvsStore().count(ns_) == 0) return false;
        opened_ = true;
        return true;
    }

    void end() { opened_ = false; }

    uint16_t getUShort(const char* key, uint16_t def = 0)
    {
        std::vector<uint8_t> v;
        if (!read(key, v) || v.size() != sizeof(uint16_t)) return def;
        uint16_t out;
        memcpy(&out, v.data(), sizeof(out));
        return out;
    }

    size_t getBytes(const char* key, void* buf, size_t maxLen)
    {
        std::vector<uint8_t> v;
        if (!read(key, v)) return 0;
        // Preferences::getBytes returns 0 when the stored blob is LARGER than
        // the caller's buffer; TR_LogToFlash's deferred length check (#671)
        // depends on that distinction.
        if (v.size() > maxLen) return 0;
        memcpy(buf, v.data(), v.size());
        return v.size();
    }

    void putUShort(const char* key, uint16_t val)
    {
        std::vector<uint8_t> v(sizeof(val));
        memcpy(v.data(), &val, sizeof(val));
        write(key, v);
    }

    size_t putBytes(const char* key, const void* buf, size_t len)
    {
        const uint8_t* p = static_cast<const uint8_t*>(buf);
        if (!write(key, std::vector<uint8_t>(p, p + len))) return 0;
        return len;
    }

private:
    bool read(const char* key, std::vector<uint8_t>& out) const
    {
        if (!opened_) return false;
        auto ns = _host_shim::nvsStore().find(ns_);
        if (ns == _host_shim::nvsStore().end()) return false;
        auto it = ns->second.find(key);
        if (it == ns->second.end()) return false;
        out = it->second;
        return true;
    }

    bool write(const char* key, std::vector<uint8_t> value)
    {
        if (!opened_ || read_only_) return false;
        _host_shim::nvsStore()[ns_][key] = std::move(value);
        return true;
    }

    std::string ns_;
    bool opened_ = false;
    bool read_only_ = false;
};
