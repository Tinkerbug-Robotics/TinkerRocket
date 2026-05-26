#include <TR_MMC5983MA.h>
#include <cstring>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* MMC_TAG = "TR_MMC5983MA";

// ---------------- Ctor / Dtor ----------------

TR_MMC5983MA::TR_MMC5983MA(spi_host_device_t host, uint8_t cs_pin, uint32_t clock_hz)
    : host_(host),
      cs_pin_(cs_pin),
      clock_hz_(clock_hz)
{
}

TR_MMC5983MA::~TR_MMC5983MA()
{
    if (spi_dev_) {
        spi_bus_remove_device(spi_dev_);
        spi_dev_ = nullptr;
    }
}

// ---------------- IDF setup helpers ----------------

void TR_MMC5983MA::configureCsPin_()
{
    gpio_config_t io = {};
    io.pin_bit_mask  = 1ULL << cs_pin_;
    io.mode          = GPIO_MODE_OUTPUT;
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&io);
    csDeselect_();
}

bool TR_MMC5983MA::ensureSpiDevice_()
{
    if (spi_dev_) return true;

    // Configure CS as a GPIO output (idle HIGH) before the device is added.
    // Idempotent across the cold path (begin()) and any pre-begin I/O.
    configureCsPin_();

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = clock_hz_;
    devcfg.mode           = 0;          // SPI_MODE0 (MMC5983MA supports mode 0 and mode 3; we use 0)
    devcfg.spics_io_num   = -1;         // CS driven manually by this wrapper
    devcfg.queue_size     = 1;
    devcfg.command_bits   = 8;          // reg byte goes in t.cmd

    esp_err_t err = spi_bus_add_device(host_, &devcfg, &spi_dev_);
    if (err != ESP_OK) {
        ESP_LOGE(MMC_TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_dev_ = nullptr;
        return false;
    }
    return true;
}

// ---------------- Public API ----------------

bool TR_MMC5983MA::begin()
{
    if (!ensureSpiDevice_()) return false;
    return isConnected();
}

bool TR_MMC5983MA::isConnected()
{
    uint8_t id = 0;
    if (!readSingleByte(PROD_ID_REG, &id)) return false;
    return (id == PROD_ID);
}

bool TR_MMC5983MA::softReset()
{
    const bool ok = setShadowBit(INT_CTRL_1_REG, SW_RST);
    clearShadowBit(INT_CTRL_1_REG, SW_RST, false);
    vTaskDelay(pdMS_TO_TICKS(15));
    return ok;
}

bool TR_MMC5983MA::performSetOperation()
{
    const bool ok = setShadowBit(INT_CTRL_0_REG, SET_OPERATION);
    clearShadowBit(INT_CTRL_0_REG, SET_OPERATION, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ok;
}

bool TR_MMC5983MA::performResetOperation()
{
    const bool ok = setShadowBit(INT_CTRL_0_REG, RESET_OPERATION);
    clearShadowBit(INT_CTRL_0_REG, RESET_OPERATION, false);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ok;
}

bool TR_MMC5983MA::enableInterrupt()
{
    return setShadowBit(INT_CTRL_0_REG, INT_MEAS_DONE_EN);
}

bool TR_MMC5983MA::enableContinuousMode()
{
    return setShadowBit(INT_CTRL_2_REG, CMM_EN);
}

bool TR_MMC5983MA::setContinuousModeFrequency(uint16_t frequency_hz)
{
    bool success = false;
    switch (frequency_hz)
    {
        case 1:
            success = clearShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 10:
            success = clearShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 20:
            success = clearShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 50:
            success = setShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 100:
            success = setShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 200:
            success = setShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 1000:
            success = setShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= setShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        case 0:
            success = clearShadowBit(INT_CTRL_2_REG, CM_FREQ_2, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_1, false);
            success &= clearShadowBit(INT_CTRL_2_REG, CM_FREQ_0);
            break;
        default:
            success = false;
            break;
    }
    return success;
}

bool TR_MMC5983MA::setFilterBandwidth(uint16_t bandwidth_hz)
{
    bool success = false;
    switch (bandwidth_hz)
    {
        case 800:
            success = setShadowBit(INT_CTRL_1_REG, BW0, false);
            success &= setShadowBit(INT_CTRL_1_REG, BW1);
            break;
        case 400:
            success = clearShadowBit(INT_CTRL_1_REG, BW0, false);
            success &= setShadowBit(INT_CTRL_1_REG, BW1);
            break;
        case 200:
            success = setShadowBit(INT_CTRL_1_REG, BW0, false);
            success &= clearShadowBit(INT_CTRL_1_REG, BW1);
            break;
        case 100:
            success = clearShadowBit(INT_CTRL_1_REG, BW0, false);
            success &= clearShadowBit(INT_CTRL_1_REG, BW1);
            break;
        default:
            success = false;
            break;
    }
    return success;
}

bool TR_MMC5983MA::enableAutomaticSetReset()
{
    return setShadowBit(INT_CTRL_0_REG, AUTO_SR_EN);
}

bool TR_MMC5983MA::isMeasurementDone()
{
    uint8_t status = 0;
    if (!readSingleByte(STATUS_REG, &status)) return false;
    return ((status & MEAS_M_DONE) != 0U);
}

bool TR_MMC5983MA::readStatus(uint8_t *status)
{
    return readSingleByte(STATUS_REG, status);
}

bool TR_MMC5983MA::readControl0(uint8_t *ctrl0)
{
    return readSingleByte(INT_CTRL_0_REG, ctrl0);
}

bool TR_MMC5983MA::readControl1(uint8_t *ctrl1)
{
    return readSingleByte(INT_CTRL_1_REG, ctrl1);
}

bool TR_MMC5983MA::readControl2(uint8_t *ctrl2)
{
    return readSingleByte(INT_CTRL_2_REG, ctrl2);
}

bool TR_MMC5983MA::readFieldsXYZ(uint32_t *x, uint32_t *y, uint32_t *z)
{
    if (x == nullptr || y == nullptr || z == nullptr) return false;

    // Burst read all 7 output registers starting at 0x00
    uint8_t buf[7] = {};
    if (!readMultipleBytes(0x00, buf, 7)) return false;

    // Registers: Xout0(0), Xout1(1), Yout0(2), Yout1(3), Zout0(4), Zout1(5), XYZout2(6)
    // 18-bit values: [out0:8][out1:8][out2_bits:2] = 18 bits
    *x = (uint32_t)buf[0] << 10 | (uint32_t)buf[1] << 2 | (buf[6] >> 6);
    *y = (uint32_t)buf[2] << 10 | (uint32_t)buf[3] << 2 | ((buf[6] >> 4) & 0x03);
    *z = (uint32_t)buf[4] << 10 | (uint32_t)buf[5] << 2 | ((buf[6] >> 2) & 0x03);
    return true;
}

bool TR_MMC5983MA::clearMeasDoneInterrupt(uint8_t measMask)
{
    measMask &= (uint8_t)(MEAS_T_DONE | MEAS_M_DONE);
    return setRegisterBit(STATUS_REG, measMask);
}

bool TR_MMC5983MA::readSingleByte(uint8_t reg, uint8_t *value)
{
    if (value == nullptr) return false;
    if (!ensureSpiDevice_()) return false;

    spi_transaction_t t = {};
    t.cmd       = static_cast<uint16_t>(READ_REG(reg));  // bit7=1 for read
    t.length    = 8;
    t.flags     = SPI_TRANS_USE_RXDATA;

    csSelect_();
    esp_err_t err = spi_device_polling_transmit(spi_dev_, &t);
    csDeselect_();

    if (err != ESP_OK) return false;
    *value = t.rx_data[0];
    return true;
}

bool TR_MMC5983MA::writeSingleByte(uint8_t reg, uint8_t value)
{
    if (!ensureSpiDevice_()) return false;

    spi_transaction_t t = {};
    t.cmd        = static_cast<uint16_t>(reg & 0x7F);  // bit7=0 for write
    t.length     = 8;
    t.flags      = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = value;

    csSelect_();
    esp_err_t err = spi_device_polling_transmit(spi_dev_, &t);
    csDeselect_();

    return (err == ESP_OK);
}

bool TR_MMC5983MA::readMultipleBytes(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    if (buffer == nullptr || len == 0U) return false;
    if (!ensureSpiDevice_()) return false;

    spi_transaction_t t = {};
    t.cmd       = static_cast<uint16_t>(READ_REG(reg));  // bit7=1 for read (auto-increment)
    t.length    = len * 8;
    t.rx_buffer = buffer;

    csSelect_();
    esp_err_t err = spi_device_polling_transmit(spi_dev_, &t);
    csDeselect_();

    return (err == ESP_OK);
}

bool TR_MMC5983MA::setRegisterBit(uint8_t reg, uint8_t bitMask)
{
    uint8_t value = 0;
    if (!readSingleByte(reg, &value)) return false;
    value |= bitMask;
    return writeSingleByte(reg, value);
}

uint8_t *TR_MMC5983MA::shadowForRegister(uint8_t reg)
{
    switch (reg)
    {
        case INT_CTRL_0_REG: return &internal_ctrl_0;
        case INT_CTRL_1_REG: return &internal_ctrl_1;
        case INT_CTRL_2_REG: return &internal_ctrl_2;
        case INT_CTRL_3_REG: return &internal_ctrl_3;
        default: return nullptr;
    }
}

bool TR_MMC5983MA::setShadowBit(uint8_t reg, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadow = shadowForRegister(reg);
    if (shadow == nullptr) return false;
    *shadow |= bitMask;
    if (!doWrite) return true;
    return writeSingleByte(reg, *shadow);
}

bool TR_MMC5983MA::clearShadowBit(uint8_t reg, uint8_t bitMask, bool doWrite)
{
    uint8_t *shadow = shadowForRegister(reg);
    if (shadow == nullptr) return false;
    *shadow &= (uint8_t)~bitMask;
    if (!doWrite) return true;
    return writeSingleByte(reg, *shadow);
}
