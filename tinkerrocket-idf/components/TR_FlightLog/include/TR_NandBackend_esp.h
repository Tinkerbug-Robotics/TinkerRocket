#pragma once

#include "TR_NandBackend.h"

#include <stdint.h>

namespace tr_flightlog {

// ESP-IDF implementation of TR_NandBackend, backed by the SPI NAND driver
// that currently lives in TR_LogToFlash.
//
// Stage 1 (this file): scaffold only — every operation returns false so the
// component compiles under ESP-IDF without pulling in an actual SPI stack.
// Stage 2 wiring will either:
//   * extract the nandRead/Program/EraseBlock helpers from TR_LogToFlash into
//     a shared low-level component (preferred), or
//   * add a narrow accessor interface on TR_LogToFlash that this class
//     delegates to.
//
// The choice is deferred because Stage 2 is where the flight_computer main.cpp
// flips between the legacy LFS path and the new TR_FlightLog path — we want
// the refactor boundary to fall along that flip.
class TR_NandBackend_esp : public TR_NandBackend {
public:
    TR_NandBackend_esp() = default;

    bool readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) override;
    bool programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) override;
    bool eraseBlock(uint32_t block) override;
    bool isBlockBad(uint32_t block) override;
    bool markBlockBad(uint32_t block) override;
};

}  // namespace tr_flightlog
