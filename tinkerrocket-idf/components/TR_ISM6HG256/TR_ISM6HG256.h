#ifndef TR_ISM6HG256_H
#define TR_ISM6HG256_H

#include <cstdint>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "ism6hg256x_reg.h"

typedef enum {
    TR_ISM6HG256_OK = 0,
    TR_ISM6HG256_ERROR = -1
} TR_ISM6HG256Status;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} TR_ISM6HG256_AxesRaw_t;

class TR_ISM6HG256
{
public:
    // ESP-IDF native constructor.  The SPI bus identified by `host` must be
    // initialised (spi_bus_initialize) before begin() is called.  CS is
    // driven manually by the wrapper via the GPIO HAL.  SPI mode is fixed
    // to SPI_MODE0 (ST ISM6HG256X SPI convention).
    TR_ISM6HG256(spi_host_device_t host, uint8_t cs_pin, uint32_t clock_hz = 2000000);

    ~TR_ISM6HG256();

    TR_ISM6HG256Status begin();
    TR_ISM6HG256Status ReadWhoAmI(uint8_t *id);

    TR_ISM6HG256Status Enable_X();
    TR_ISM6HG256Status Enable_HG_X();
    TR_ISM6HG256Status Enable_G();
    TR_ISM6HG256Status Route_DRDY_To_INT1();

    TR_ISM6HG256Status Set_X_OutputDataRate(float Odr);
    TR_ISM6HG256Status Set_HG_X_OutputDataRate(float Odr);
    TR_ISM6HG256Status Set_G_OutputDataRate(float Odr);

    TR_ISM6HG256Status Set_X_FullScale(int32_t FullScale);
    TR_ISM6HG256Status Set_HG_X_FullScale(int32_t FullScale);
    TR_ISM6HG256Status Set_G_FullScale(int32_t FullScale);

    TR_ISM6HG256Status Get_X_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value);
    TR_ISM6HG256Status Get_HG_X_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value);
    TR_ISM6HG256Status Get_G_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value);

    // Bulk read: gyro + low-g accel + high-g accel in a single 24-byte SPI
    // transaction (0x22–0x39).  Bytes 12-17 (OIS/EIS gyro) are discarded.
    TR_ISM6HG256Status Get_AllAxesRaw(TR_ISM6HG256_AxesRaw_t *gyro,
                                       TR_ISM6HG256_AxesRaw_t *acc_lg,
                                       TR_ISM6HG256_AxesRaw_t *acc_hg);

    TR_ISM6HG256Status Get_DRDY_Status(ism6hg256x_data_ready_t *Status);
    TR_ISM6HG256Status Get_X_DRDY_Status(uint8_t *Status);
    TR_ISM6HG256Status HG_X_Get_DRDY_Status(uint8_t *Status);
    TR_ISM6HG256Status Get_G_DRDY_Status(uint8_t *Status);

private:
    // CS control (manual, IDF GPIO HAL).  spics_io_num is set to -1 in the
    // device config, so IDF does not drive CS — this wrapper does.
    inline void csSelect_()   { gpio_set_level(static_cast<gpio_num_t>(cs_pin_), 0); }
    inline void csDeselect_() { gpio_set_level(static_cast<gpio_num_t>(cs_pin_), 1); }

    // Lazily add the SPI device on first begin().  Returns false on
    // add_device failure.  Idempotent.
    bool ensureSpiDevice_();

    // Configure cs_pin_ as a GPIO output, idle HIGH.
    void configureCsPin_();

    spi_host_device_t   host_;
    uint8_t             cs_pin_;
    uint32_t            clock_hz_;
    spi_device_handle_t spi_dev_ = nullptr;

    uint8_t acc_is_enabled;
    uint8_t acc_hg_is_enabled;
    uint8_t gyro_is_enabled;

    ism6hg256x_data_rate_t acc_odr;
    ism6hg256x_hg_xl_data_rate_t acc_hg_odr;
    ism6hg256x_data_rate_t gyro_odr;

    ism6hg256x_ctx_t reg_ctx;

    TR_ISM6HG256Status Set_X_OutputDataRate_When_Enabled(float Odr);
    TR_ISM6HG256Status Set_X_OutputDataRate_When_Disabled(float Odr);
    TR_ISM6HG256Status Set_HG_X_OutputDataRate_When_Enabled(float Odr);
    TR_ISM6HG256Status Set_HG_X_OutputDataRate_When_Disabled(float Odr);
    TR_ISM6HG256Status Set_G_OutputDataRate_When_Enabled(float Odr);
    TR_ISM6HG256Status Set_G_OutputDataRate_When_Disabled(float Odr);

    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite);

    static int32_t io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
    static int32_t io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
};

#endif
