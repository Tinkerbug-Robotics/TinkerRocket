#pragma once

// The WHO_AM_I values this driver accepts.
//
// ST's ISM6HGK256X (datasheet DS15340) is the ISM6HG256X (DS15034) with a
// redesigned gyroscope: same package and pinout, same register map, same full
// scales and sensitivities. Its WHO_AM_I reads 0x75 where the ISM6HG256X reads
// 0x73, and ST's own driver for it is the ISM6HG256X driver with only that ID
// changed. Either part can be fitted, and both run on this driver unchanged.
//
// Header-only and IDF-free, so the host tests pin it.

#include <stdint.h>

namespace ism6_whoami
{
constexpr uint8_t WHOAMI_ISM6HG256X = 0x73;
constexpr uint8_t WHOAMI_ISM6HGK256X = 0x75;

inline bool supported(uint8_t id)
{
    return id == WHOAMI_ISM6HG256X || id == WHOAMI_ISM6HGK256X;
}

// Which part answered, for the boot log.
inline const char *part_name(uint8_t id)
{
    switch (id)
    {
    case WHOAMI_ISM6HG256X:
        return "ISM6HG256X";
    case WHOAMI_ISM6HGK256X:
        return "ISM6HGK256X";
    default:
        return "unknown";
    }
}
} // namespace ism6_whoami
