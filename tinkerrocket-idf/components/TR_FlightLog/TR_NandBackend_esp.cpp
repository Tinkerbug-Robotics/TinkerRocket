#include "TR_NandBackend_esp.h"

namespace tr_flightlog {

// Stage 1: every operation fails. This is intentional — the class exists so
// the TR_FlightLog component compiles under ESP-IDF; Stage 2 wires the
// methods to the real SPI NAND driver before anything on the flight side
// switches over.

bool TR_NandBackend_esp::readPage(uint32_t /*block*/, uint32_t /*page_in_block*/,
                                  uint8_t* /*out*/) {
    return false;
}

bool TR_NandBackend_esp::programPage(uint32_t /*block*/, uint32_t /*page_in_block*/,
                                      const uint8_t* /*data*/) {
    return false;
}

bool TR_NandBackend_esp::eraseBlock(uint32_t /*block*/) {
    return false;
}

bool TR_NandBackend_esp::isBlockBad(uint32_t /*block*/) {
    return true;  // conservative default — never allocate from this backend yet
}

bool TR_NandBackend_esp::markBlockBad(uint32_t /*block*/) {
    return false;
}

}  // namespace tr_flightlog
