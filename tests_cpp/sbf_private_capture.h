#pragma once
// The private-capture runs of the raw-GNSS SBF tests (tests_cpp/v9raw/sbf*):
// real mosaic-G5 captures, such as the 10/16 V9 spin-rig logs, read through
// the flight's own framing (SbfParser). TR_SBF_PRIVATE_CAPTURE names one or
// more .sbf files, ':'-separated, which stay outside the repository (real
// pseudoranges with their ephemerides give the site away). Unset, as in CI,
// those tests skip. They print statistics only, never a position.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "SbfParser.h"

namespace sbftest {

inline std::vector<std::string> privateCapturePaths()
{
    std::vector<std::string> out;
    const char* env = std::getenv("TR_SBF_PRIVATE_CAPTURE");
    if (env == nullptr) return out;
    std::string s(env);
    size_t at = 0;
    while (at <= s.size())
    {
        const size_t end = s.find(':', at);
        const std::string p = s.substr(at, end == std::string::npos ? std::string::npos : end - at);
        if (!p.empty()) out.push_back(p);
        if (end == std::string::npos) break;
        at = end + 1;
    }
    return out;
}

// The file's name without its directories, for the printed statistics.
inline std::string captureName(const std::string& path)
{
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

struct CaptureScan
{
    uint64_t bytes = 0;
    uint32_t blocks = 0;
    uint32_t crc_fails = 0;
    uint32_t bad_lengths = 0;
};

// Every CRC-valid block of a capture, in order, through SbfParser as the
// flight computer frames them. False if the file cannot be read.
inline bool scanCapture(const std::string& path, const std::function<void(const uint8_t*, size_t)>& fn,
                        CaptureScan& out)
{
    struct Sink : mosaic::SbfSink
    {
        const std::function<void(const uint8_t*, size_t)>* fn;
        void onBlock(const uint8_t* b, size_t len, size_t) override { (*fn)(b, len); }
        void onLine(const char*, size_t) override {}
        void onPrompt(const char*, size_t) override {}
    } sink;
    sink.fn = &fn;
    FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) return false;
    auto parser = std::make_unique<mosaic::SbfParser>();
    std::vector<uint8_t> buf(1 << 16);
    out = CaptureScan{};
    for (;;)
    {
        const size_t n = std::fread(buf.data(), 1, buf.size(), f);
        if (n == 0) break;
        out.bytes += n;
        parser->feed(buf.data(), n, sink);
    }
    std::fclose(f);
    out.blocks = parser->blocks();
    out.crc_fails = parser->crcFails();
    out.bad_lengths = parser->badLengths();
    return true;
}

}  // namespace sbftest
