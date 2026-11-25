// SbfFields.h — little-endian field readers for SBF block bodies (pure, header-only)
//
// The raw-measurement and navigation decoders (SbfMeas.cpp, SbfNav.cpp) read
// fields at the Reference Guide's byte offsets with these. SbfParser.cpp keeps
// its own copies in its anonymous namespace; these are the same.
#ifndef SBFFIELDS_H
#define SBFFIELDS_H

#include <cstdint>
#include <cstring>

namespace mosaic
{
namespace sbf
{

inline uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline int16_t  i16(const uint8_t* p) { return static_cast<int16_t>(u16(p)); }
inline uint32_t u32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline int32_t i32(const uint8_t* p) { return static_cast<int32_t>(u32(p)); }
inline float f32(const uint8_t* p)
{
    const uint32_t x = u32(p);
    float f;
    std::memcpy(&f, &x, sizeof(f));
    return f;
}
inline double f64(const uint8_t* p)
{
    const uint64_t x = static_cast<uint64_t>(u32(p)) | (static_cast<uint64_t>(u32(p + 4)) << 32);
    double d;
    std::memcpy(&d, &x, sizeof(d));
    return d;
}

// SBF's float do-not-use value is -2e10; anything that far down (or NaN) is one.
inline bool dnu(double x) { return !(x > -1.0e10); }

}  // namespace sbf
}  // namespace mosaic

#endif  // SBFFIELDS_H
