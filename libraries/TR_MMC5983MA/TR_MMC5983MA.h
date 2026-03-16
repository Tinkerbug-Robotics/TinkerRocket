#ifndef TR_MMC5983MA_H
#define TR_MMC5983MA_H

#include <Arduino.h>
#include <SPI.h>

class TR_MMC5983MA
{
public:
    TR_MMC5983MA(SPIClass &spi, uint8_t cs_pin, SPISettings settings = SPISettings(2000000, MSBFIRST, SPI_MODE0));

    bool begin();
    bool isConnected();
    bool softReset();
    bool performSetOperation();
    bool performResetOperation();

    bool enableInterrupt();
    bool enableContinuousMode();
    bool setContinuousModeFrequency(uint16_t frequency_hz);
    bool setFilterBandwidth(uint16_t bandwidth_hz);
    bool enableAutomaticSetReset();
    bool isMeasurementDone();
    bool readStatus(uint8_t *status);
    bool readControl0(uint8_t *ctrl0);
    bool readControl1(uint8_t *ctrl1);
    bool readControl2(uint8_t *ctrl2);

    bool readFieldsXYZ(uint32_t *x, uint32_t *y, uint32_t *z);
    bool clearMeasDoneInterrupt(uint8_t measMask = 0x03);

private:
    // Register map
    static constexpr uint8_t X_OUT_0_REG = 0x00;
    static constexpr uint8_t STATUS_REG = 0x08;
    static constexpr uint8_t INT_CTRL_0_REG = 0x09;
    static constexpr uint8_t INT_CTRL_1_REG = 0x0A;
    static constexpr uint8_t INT_CTRL_2_REG = 0x0B;
    static constexpr uint8_t INT_CTRL_3_REG = 0x0C;
    static constexpr uint8_t PROD_ID_REG = 0x2F;
    static constexpr uint8_t PROD_ID = 0x30;

    // Bit definitions
    static constexpr uint8_t MEAS_M_DONE = (1U << 0);
    static constexpr uint8_t MEAS_T_DONE = (1U << 1);
    static constexpr uint8_t INT_MEAS_DONE_EN = (1U << 2);
    static constexpr uint8_t SET_OPERATION = (1U << 3);
    static constexpr uint8_t RESET_OPERATION = (1U << 4);
    static constexpr uint8_t AUTO_SR_EN = (1U << 5);
    static constexpr uint8_t BW0 = (1U << 0);
    static constexpr uint8_t BW1 = (1U << 1);
    static constexpr uint8_t CM_FREQ_0 = (1U << 0);
    static constexpr uint8_t CM_FREQ_1 = (1U << 1);
    static constexpr uint8_t CM_FREQ_2 = (1U << 2);
    static constexpr uint8_t CMM_EN = (1U << 3);
    static constexpr uint8_t SW_RST = (1U << 7);

    // Shadow registers (write-only on device)
    uint8_t internal_ctrl_0 = 0x00;
    uint8_t internal_ctrl_1 = 0x00;
    uint8_t internal_ctrl_2 = 0x00;
    uint8_t internal_ctrl_3 = 0x00;

    SPIClass &spi;
    uint8_t cs_pin;
    SPISettings spi_settings;

    static constexpr uint8_t READ_REG(uint8_t reg) { return (uint8_t)(0x80U | reg); }

    bool readSingleByte(uint8_t reg, uint8_t *value);
    bool writeSingleByte(uint8_t reg, uint8_t value);
    bool readMultipleBytes(uint8_t reg, uint8_t *buffer, uint8_t len);
    bool setRegisterBit(uint8_t reg, uint8_t bitMask);

    uint8_t *shadowForRegister(uint8_t reg);
    bool setShadowBit(uint8_t reg, uint8_t bitMask, bool doWrite = true);
    bool clearShadowBit(uint8_t reg, uint8_t bitMask, bool doWrite = true);
};

#endif
