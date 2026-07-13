#include "NandBitmapStore.h"

#include "TR_FlightLog_types.h"  // NAND_PAGE_SIZE
#include "CRC.h"                 // calcCRC32 (same as FlightIndex)

#include <cstring>
#include <memory>
#include <new>

namespace tr_flightlog {

// CRC covers everything after the crc32 field: magic + sequence + length +
// payload (matches FlightIndex's compute_crc convention). Header is private, so
// this member helper keeps the span in one place.
size_t NandBitmapStore::crcSpan(size_t payload_len) {
    return sizeof(Header) - sizeof(uint32_t) + payload_len;
}

bool NandBitmapStore::readSlot(uint32_t block, uint32_t& seq_out,
                               uint8_t* out, size_t len) {
    seq_out = 0;
    if (!nand_) return false;

    std::unique_ptr<uint8_t[]> page(new (std::nothrow) uint8_t[NAND_PAGE_SIZE]);
    if (!page) return false;
    if (!nand_->readPage(block, 0, page.get())) return false;

    Header hdr;
    std::memcpy(&hdr, page.get(), sizeof(hdr));
    if (hdr.magic != kMagic) return false;
    if (hdr.length == 0 || hdr.length > NAND_PAGE_SIZE - sizeof(Header)) return false;

    const uint32_t got = calcCRC32(page.get() + sizeof(uint32_t), crcSpan(hdr.length));
    if (got != hdr.crc32) return false;

    seq_out = hdr.sequence;
    if (out) {
        const size_t n = (len < hdr.length) ? len : hdr.length;
        std::memcpy(out, page.get() + sizeof(Header), n);
        if (len > hdr.length) std::memset(out + hdr.length, 0, len - hdr.length);
    }
    return true;
}

bool NandBitmapStore::load(uint8_t* buf, size_t len) {
    if (!nand_ || !buf || len == 0) return false;

    uint32_t seq_a = 0, seq_b = 0;
    const bool a = readSlot(block_a_, seq_a, nullptr, 0);
    const bool b = readSlot(block_b_, seq_b, nullptr, 0);
    if (!a && !b) return false;

    // Prefer the higher sequence; on a tie or single-valid, pick that one.
    const uint32_t block = (a && (!b || seq_a >= seq_b)) ? block_a_ : block_b_;
    uint32_t seq = 0;
    if (!readSlot(block, seq, buf, len)) return false;
    last_sequence_ = seq;
    return true;
}

bool NandBitmapStore::save(const uint8_t* buf, size_t len) {
    if (!nand_ || !buf || len == 0 || len > NAND_PAGE_SIZE - sizeof(Header)) return false;

    // Target the older / invalid block so the newest valid snapshot survives a
    // power loss mid-erase or mid-program (power-safe dual copy).
    uint32_t seq_a = 0, seq_b = 0;
    const bool a = readSlot(block_a_, seq_a, nullptr, 0);
    const bool b = readSlot(block_b_, seq_b, nullptr, 0);
    uint32_t target;
    if (!a)      target = block_a_;
    else if (!b) target = block_b_;
    else         target = (seq_a <= seq_b) ? block_a_ : block_b_;

    const uint32_t newest   = (a || b) ? (seq_a > seq_b ? seq_a : seq_b) : 0;
    const uint32_t next_seq = newest + 1;

    std::unique_ptr<uint8_t[]> page(new (std::nothrow) uint8_t[NAND_PAGE_SIZE]);
    if (!page) return false;
    std::memset(page.get(), 0xFF, NAND_PAGE_SIZE);

    Header hdr;
    hdr.crc32    = 0;
    hdr.magic    = kMagic;
    hdr.sequence = next_seq;
    hdr.length   = static_cast<uint32_t>(len);
    std::memcpy(page.get(), &hdr, sizeof(hdr));
    std::memcpy(page.get() + sizeof(hdr), buf, len);

    const uint32_t crc = calcCRC32(page.get() + sizeof(uint32_t), crcSpan(len));
    std::memcpy(page.get(), &crc, sizeof(crc));

    if (!nand_->eraseBlock(target))       return false;
    if (!nand_->programPage(target, 0, page.get())) return false;
    last_sequence_ = next_seq;
    return true;
}

}  // namespace tr_flightlog
