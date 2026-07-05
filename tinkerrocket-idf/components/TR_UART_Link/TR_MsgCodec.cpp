#include "TR_MsgCodec.h"

#include <CRC.h>

namespace tr_msg
{

bool pack(uint8_t type,
          const uint8_t* payload,
          size_t len,
          uint8_t* frame_out,
          size_t frame_out_capacity,
          size_t& frame_len_out)
{
    frame_len_out = 0;

    if (frame_out == nullptr)
    {
        return false;
    }
    if ((len > 0) && (payload == nullptr))
    {
        return false;
    }
    if (len > MAX_PAYLOAD)
    {
        return false;
    }

    const size_t frame_len = FRAME_OVERHEAD + len;
    if (frame_out_capacity < frame_len)
    {
        return false;
    }

    size_t idx = 0;
    frame_out[idx++] = SOF0;
    frame_out[idx++] = SOF1;
    frame_out[idx++] = SOF2;
    frame_out[idx++] = SOF3;

    frame_out[idx++] = type;
    frame_out[idx++] = static_cast<uint8_t>(len);

    for (size_t i = 0; i < len; i++)
    {
        frame_out[idx++] = payload[i];
    }

    // Same call as TR_I2C_Interface: CRC16 (library defaults) over
    // type + len + payload, stored big-endian.
    const uint16_t crc = calcCRC16(frame_out + 4, static_cast<int>(1 + 1 + len));
    frame_out[idx++] = static_cast<uint8_t>(crc >> 8);
    frame_out[idx++] = static_cast<uint8_t>(crc & 0xFF);

    frame_len_out = frame_len;
    return true;
}

void MsgDeframer::rehunt(uint8_t b)
{
    stats_.resync_bytes++;
    // SOF0 == SOF2 == 0xAA, so any stray 0xAA restarts the hunt one byte in.
    state_ = (b == SOF0) ? St::Hunt1 : St::Hunt0;
}

bool MsgDeframer::feed(uint8_t b)
{
    switch (state_)
    {
        case St::Hunt0:
            if (b == SOF0)
            {
                state_ = St::Hunt1;
            }
            else
            {
                stats_.resync_bytes++;
            }
            break;

        case St::Hunt1:
            if (b == SOF1)
            {
                state_ = St::Hunt2;
            }
            else if (b == SOF0)
            {
                // AA AA 55 ... — the second AA may start the real SOF.
                stats_.resync_bytes++;
            }
            else
            {
                rehunt(b);
            }
            break;

        case St::Hunt2:
            if (b == SOF2)
            {
                state_ = St::Hunt3;
            }
            else
            {
                rehunt(b);
            }
            break;

        case St::Hunt3:
            if (b == SOF3)
            {
                state_ = St::Type;
            }
            else
            {
                rehunt(b);
            }
            break;

        case St::Type:
            buf_[0] = b;
            state_ = St::Len;
            break;

        case St::Len:
            buf_[1] = b;
            payload_len_ = b;
            payload_got_ = 0;
            state_ = (payload_len_ > 0) ? St::Payload : St::Crc0;
            break;

        case St::Payload:
            buf_[2 + payload_got_++] = b;
            if (payload_got_ >= payload_len_)
            {
                state_ = St::Crc0;
            }
            break;

        case St::Crc0:
            crc_hi_ = b;
            state_ = St::Crc1;
            break;

        case St::Crc1:
        {
            const uint16_t crc_expected =
                static_cast<uint16_t>((crc_hi_ << 8) | b);
            const uint16_t crc_actual =
                calcCRC16(buf_, static_cast<int>(2 + payload_len_));
            state_ = St::Hunt0;
            if (crc_actual == crc_expected)
            {
                stats_.frames++;
                return true;
            }
            stats_.crc_fails++;
            break;
        }
    }
    return false;
}

}  // namespace tr_msg
