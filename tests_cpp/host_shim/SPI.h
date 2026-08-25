/**
 * SPI.h — Host stub for the SPIClass / SPISettings surface that
 *         TR_Compat's compat.h provides on target.
 *
 * TR_LogToFlash::begin() takes an `SPIClass&`, and SPIClass is a concrete
 * type (not an interface), so the host build cannot inject a fake by
 * subclassing — the stub itself has to answer.  It answers as little as
 * possible:
 *
 *   RDID   (0x9F)  the canned chip ID set by setNandRdid(), so a test can
 *                  choose which entry of nand_geometry.h's part table
 *                  begin() resolves — that is what fixes the sink payload
 *                  quantum at 4080 B (legacy 4 KB page) or 2032 B (the 2 KB
 *                  GD5F parts on V9 and the mini).
 *   GETFEAT(0x0F)  0x00 — never busy, no program/erase failure.  A stub that
 *                  answered 0xFF here would set STAT_OIP and park
 *                  nandWaitReady() in its two-second poll.
 *   anything else  0xFF, i.e. erased flash.  That is what makes the #511
 *                  boot bad-block scan read every factory marker as good and
 *                  finish clean, instead of marking all 2048 blocks bad.
 *
 * This is a NAND *identity* responder, not a NAND model: nothing programmed
 * through it is stored, and nothing read back from it is real.  A test that
 * needs page contents to survive a write needs a different fake.
 */
#pragma once

#include <cstdint>
#include <cstring>

#ifndef SPI_MODE0
#define SPI_MODE0 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif
#ifndef LSBFIRST
#define LSBFIRST 0
#endif

struct SPISettings {
    uint32_t clock{1000000};
    uint8_t  bitOrder{MSBFIRST};
    uint8_t  dataMode{SPI_MODE0};

    SPISettings() = default;
    SPISettings(uint32_t clk, uint8_t order, uint8_t mode)
        : clock(clk), bitOrder(order), dataMode(mode) {}
};

class SPIClass {
public:
    /// Chip ID reported by the next RDID.  Defaults to 0x0000, which the
    /// component reads as a DEAD BUS: legacy geometry, and the boot
    /// bad-block scan skipped entirely.  Call this before begin() to stand
    /// in for a specific part instead.
    void setNandRdid(uint8_t mid, uint8_t did) { mid_ = mid; did_ = did; }

    void begin(int8_t = -1, int8_t = -1, int8_t = -1, int8_t = -1) {}
    void end() {}

    // Every NAND/MRAM operation in the component wraps itself in exactly one
    // beginTransaction/endTransaction pair, so the byte index within a
    // transaction identifies the opcode's operands.
    void beginTransaction(const SPISettings&) { idx_ = 0; cmd_ = 0; }
    void endTransaction() {}

    uint8_t transfer(uint8_t data)
    {
        const uint32_t i = idx_++;
        if (i == 0) { cmd_ = data; return 0xFF; }
        switch (cmd_)
        {
            case 0x9F:  // RDID: [cmd][dummy][MID][DID]
                if (i == 2) return mid_;
                if (i == 3) return did_;
                return 0xFF;
            case 0x0F:  // GETFEAT: [cmd][addr][value]
                return 0x00;
            default:
                return 0xFF;
        }
    }

    void transfer(void* buf, uint32_t size)
    {
        uint8_t* p = static_cast<uint8_t*>(buf);
        for (uint32_t i = 0; i < size; ++i) p[i] = transfer(p[i]);
    }

    void transferBytes(const uint8_t* tx, uint8_t* rx, uint32_t size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            const uint8_t out = transfer(tx ? tx[i] : 0x00);
            if (rx) rx[i] = out;
        }
    }

    void writeBytes(const uint8_t* data, uint32_t size)
    {
        for (uint32_t i = 0; i < size; ++i) (void)transfer(data ? data[i] : 0x00);
    }

private:
    uint32_t idx_ = 0;
    uint8_t  cmd_ = 0;
    uint8_t  mid_ = 0x00;
    uint8_t  did_ = 0x00;
};
