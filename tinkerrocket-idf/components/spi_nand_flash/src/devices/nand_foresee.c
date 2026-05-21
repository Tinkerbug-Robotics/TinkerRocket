/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * FORESEE / Longsys SPI NAND support (manufacturer ID 0xCD).
 * Added downstream (TinkerRocket) -- not part of the upstream component.
 * Validated against F35SQB004G (4 Gbit): 4096-byte page, 64 pages/block,
 * 2048 blocks, on-die 8-bit ECC.
 */

#include <stdio.h>
#include <inttypes.h>
#include "esp_check.h"
#include "nand.h"
#include "spi_nand_oper.h"
#include "nand_flash_devices.h"

static const char *TAG = "nand_foresee";

esp_err_t spi_nand_foresee_init(spi_nand_flash_device_t *dev)
{
    uint8_t device_id = 0;
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_ERROR(spi_nand_read_device_id(dev, &device_id, sizeof(device_id)), TAG,
                        "%s, Failed to get the device ID %d", __func__, ret);
    dev->device_info.device_id = device_id;
    snprintf(dev->device_info.chip_name, sizeof(dev->device_info.chip_name),
             "foresee-0x%02" PRIx8, device_id);
    ESP_LOGD(TAG, "%s: device_id: %x\n", __func__, device_id);

    // On-die 8-bit ECC with a 3-bit status field (ECCS2-0).
    // NOTE: FORESEE's ECCS encoding is a linear bit-count (010=4 bits CORRECTED,
    // 111=uncorrectable), which differs from the Micron-style mapping the generic
    // 3-bit path in is_ecc_error() assumes. Fresh-chip reads (ECCS=000=OK) are
    // interpreted correctly; wear-time correction levels are NOT yet remapped --
    // see follow-up before relying on ECC reporting for data-integrity decisions.
    dev->chip.ecc_data.ecc_status_reg_len_in_bits = 3;
    dev->chip.read_page_delay_us    = 115;   // tRD  (datasheet max)
    dev->chip.program_page_delay_us = 450;   // tPROG (typ 350 us + margin)
    dev->chip.erase_block_delay_us  = 3000;  // tERS  (typ 2 ms + margin)
    dev->chip.has_quad_enable_bit   = 0;     // driven in SIO mode; QE not needed
    dev->chip.quad_enable_bit_pos   = 0;

    switch (device_id) {
    case FORESEE_DI_53:   // F35SQB004G: 4 Gbit = 4096 B/page x 64 pages x 2048 blocks
        dev->chip.log2_page_size = 12;       // 4096-byte page (override default 11)
        dev->chip.log2_ppb       = 6;        // 64 pages per block
        dev->chip.num_blocks     = 2048;
        break;
    default:
        ESP_LOGE(TAG, "%s: unrecognized FORESEE device id 0x%02x", __func__, device_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
