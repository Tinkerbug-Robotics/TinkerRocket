#ifndef TR_ISM6HG256_H
#define TR_ISM6HG256_H

#include <Arduino.h>
#include <SPI.h>
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
    TR_ISM6HG256(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);

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

    TR_ISM6HG256Status Get_DRDY_Status(ism6hg256x_data_ready_t *Status);
    TR_ISM6HG256Status Get_X_DRDY_Status(uint8_t *Status);
    TR_ISM6HG256Status HG_X_Get_DRDY_Status(uint8_t *Status);
    TR_ISM6HG256Status Get_G_DRDY_Status(uint8_t *Status);

private:
    SPIClass *dev_spi;
    int cs_pin;
    uint32_t spi_speed;

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
