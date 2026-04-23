#include "TR_NandBackend_esp.h"

#ifdef ESP_PLATFORM
#include "TR_LogToFlash.h"
#endif

namespace tr_flightlog {

// Host build (no ESP_PLATFORM): every method returns a safe "no" so the same
// stub surface the host tests see is preserved.
// ESP-IDF build: delegate to the logger's public NAND forwarders when a
// logger is attached; return "no" when it is not (handy during early boot
// before the logger's begin() has run).

bool TR_NandBackend_esp::readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) {
#ifdef ESP_PLATFORM
    return logger_ && logger_->readNandPage(block, page_in_block, out);
#else
    (void)block; (void)page_in_block; (void)out;
    return false;
#endif
}

bool TR_NandBackend_esp::programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) {
#ifdef ESP_PLATFORM
    return logger_ && logger_->programNandPage(block, page_in_block, data);
#else
    (void)block; (void)page_in_block; (void)data;
    return false;
#endif
}

bool TR_NandBackend_esp::eraseBlock(uint32_t block) {
#ifdef ESP_PLATFORM
    return logger_ && logger_->eraseNandBlock(block);
#else
    (void)block;
    return false;
#endif
}

bool TR_NandBackend_esp::isBlockBad(uint32_t block) {
#ifdef ESP_PLATFORM
    // Conservative default when no logger is attached: treat as bad so the
    // allocator will not pick the block. Matches the host stub behaviour.
    return !logger_ || logger_->isNandBlockBad(block);
#else
    (void)block;
    return true;
#endif
}

bool TR_NandBackend_esp::markBlockBad(uint32_t block) {
#ifdef ESP_PLATFORM
    return logger_ && logger_->markNandBlockBad(block);
#else
    (void)block;
    return false;
#endif
}

}  // namespace tr_flightlog
