// EphemerisFrames.cpp — see EphemerisFrames.h
#include "EphemerisFrames.h"

#include <cstring>

namespace gnss_raw
{

bool EphemerisFrames::add(const uint8_t* payload, size_t len)
{
    if (payload == nullptr || len < 3 || len > kMaxPayload)
        return false;
    // DF002, the message number (12 bits), then the satellite's ID: 4 bits
    // for QZSS's 1044 (DF429), 6 for the rest (DF009, DF038, DF488, DF252)
    const uint16_t msg = static_cast<uint16_t>((payload[0] << 4) | (payload[1] >> 4));
    const uint8_t sat = msg == 1044 ? static_cast<uint8_t>(payload[1] & 0x0F)
                                    : static_cast<uint8_t>(((payload[1] & 0x0F) << 2) | (payload[2] >> 6));
    int slot = -1;
    for (int i = 0; i < n_; ++i)
    {
        if (items_[i].msg == msg && items_[i].sat == sat)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0 && n_ < kMax)
        slot = n_++;
    if (slot < 0)
    {
        slot = 0;                                          // full: the oldest goes
        for (int i = 1; i < n_; ++i)
            if (items_[i].seq < items_[slot].seq)
                slot = i;
    }
    Item& it = items_[slot];
    it.msg = msg;
    it.sat = sat;
    it.len = static_cast<uint8_t>(len);
    it.seq = ++seq_;
    std::memcpy(it.bytes, payload, len);
    return true;
}

size_t EphemerisFrames::frame(int i, uint8_t* out, size_t cap) const
{
    if (i < 0 || i >= n_ || out == nullptr)
        return 0;
    const Item& it = items_[i];
    const size_t total = it.len + kFrameOverhead;
    if (total > cap)
        return 0;
    out[0] = 0xD3;
    out[1] = static_cast<uint8_t>((it.len >> 8) & 0x03);
    out[2] = static_cast<uint8_t>(it.len & 0xFF);
    std::memcpy(out + 3, it.bytes, it.len);
    const uint32_t crc = crc24q(out, it.len + 3u);
    out[it.len + 3] = static_cast<uint8_t>(crc >> 16);
    out[it.len + 4] = static_cast<uint8_t>(crc >> 8);
    out[it.len + 5] = static_cast<uint8_t>(crc);
    return total;
}

uint32_t EphemerisFrames::crc24q(const uint8_t* data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int b = 0; b < 8; ++b)
        {
            crc <<= 1;
            if (crc & 0x1000000u)
                crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

}  // namespace gnss_raw
