#pragma once
#include <cstddef>
#include <cstdint>

// #1155 item 4: operator-supplied strings inside a readback JSON frame.
//
// The unit name reaches the config_identity readback through snprintf("%s"),
// and BLE cmd 40 accepts any 1..20 bytes as the name. A '"' or '\' in it
// terminated the JSON string early, both apps' strict parsers rejected the
// frame, and the device lost its uid/nid/rid/fw identification. Two guards,
// both used: cmd 40 refuses a name the readback would have to escape (so the
// BLE advertising name and the LoRa beacon stay plain too), and the readback
// escapes whatever is already in NVS. Host-tested in tests_cpp/test_json_escape.cpp.
namespace tr_json {

// True when the byte can sit inside a JSON string verbatim: anything printable
// (UTF-8 lead and continuation bytes included), never '"', '\' or a control byte.
inline bool isPlainStringByte(uint8_t c)
{
    return c >= 0x20 && c != 0x7F && c != '"' && c != '\\';
}

inline bool isPlainString(const char* s, size_t n)
{
    if (s == nullptr) return n == 0;
    for (size_t i = 0; i < n; ++i)
        if (!isPlainStringByte(static_cast<uint8_t>(s[i]))) return false;
    return true;
}

// Copies the NUL-terminated `in` into `out` (NUL-terminated), escaping '"' and
// '\' with a backslash and dropping control bytes. Returns false — and leaves
// `out` empty — when the escaped form does not fit: a caller must never emit a
// truncated string, which is the failure this exists to remove. Size `out` at
// 2 * strlen(in) + 1 for the worst case.
inline bool escapeInto(char* out, size_t out_size, const char* in)
{
    if (out == nullptr || out_size == 0) return false;
    size_t o = 0;
    for (const char* p = in; p != nullptr && *p != '\0'; ++p)
    {
        const uint8_t c = static_cast<uint8_t>(*p);
        if (c < 0x20 || c == 0x7F) continue;                 // control byte: dropped
        const size_t need = (c == '"' || c == '\\') ? 2u : 1u;
        if (o + need + 1u > out_size) { out[0] = '\0'; return false; }
        if (need == 2u) out[o++] = '\\';
        out[o++] = static_cast<char>(c);
    }
    out[o] = '\0';
    return true;
}

}  // namespace tr_json
