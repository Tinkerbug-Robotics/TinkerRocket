#pragma once

#include "TR_NandBackend.h"

#include <stdint.h>

// Forward declaration — the full header drags in lfs.h + FreeRTOS so it is
// only included in TR_NandBackend_esp.cpp under an ESP_PLATFORM guard.
class TR_LogToFlash;

namespace tr_flightlog {

// ESP-IDF implementation of TR_NandBackend, bridged to the SPI NAND driver
// currently owned by TR_LogToFlash.
//
// During Stage 2 the legacy LFS logger and TR_FlightLog coexist: they share
// the NAND, the SPI bus, and the NVS bad-block bitmap. This adapter forwards
// each primitive into TR_LogToFlash's public *NandPage / *NandBlock /
// *NandBlockBad surface so both layers stay consistent without duplication.
//
// Stage 3 removes LFS from the hot path; Stage 4 may retire TR_LogToFlash
// entirely, at which point the SPI driver moves into TR_FlightLog and this
// adapter becomes the owner rather than a bridge.
//
// Pass `nullptr` (or use the default ctor) on host / in tests — every
// operation returns a safe "no" without touching real hardware.
class TR_NandBackend_esp : public TR_NandBackend {
public:
    TR_NandBackend_esp() = default;
    explicit TR_NandBackend_esp(TR_LogToFlash* logger) : logger_(logger) {}

    bool readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) override;
    bool programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) override;
    bool eraseBlock(uint32_t block) override;
    bool isBlockBad(uint32_t block) override;
    bool markBlockBad(uint32_t block) override;

private:
    TR_LogToFlash* logger_ = nullptr;
};

}  // namespace tr_flightlog
