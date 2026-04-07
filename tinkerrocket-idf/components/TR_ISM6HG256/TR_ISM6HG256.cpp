#include <TR_ISM6HG256.h>

TR_ISM6HG256::TR_ISM6HG256(SPIClass *spi, int cs_pin, uint32_t spi_speed)
    : dev_spi(spi),
      cs_pin(cs_pin),
      spi_speed(spi_speed),
      acc_is_enabled(0),
      acc_hg_is_enabled(0),
      gyro_is_enabled(0),
      acc_odr(ISM6HG256X_ODR_AT_120Hz),
      acc_hg_odr(ISM6HG256X_HG_XL_ODR_AT_480Hz),
      gyro_odr(ISM6HG256X_ODR_AT_120Hz)
{
    reg_ctx.write_reg = io_write;
    reg_ctx.read_reg = io_read;
    reg_ctx.handle = (void *)this;
}

TR_ISM6HG256Status TR_ISM6HG256::begin()
{
    uint8_t whoami = 0U;

    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    if (ReadWhoAmI(&whoami) != TR_ISM6HG256_OK) return TR_ISM6HG256_ERROR;
    if (whoami != ISM6HG256X_ID) return TR_ISM6HG256_ERROR;

    if (ism6hg256x_mem_bank_set(&reg_ctx, ISM6HG256X_MAIN_MEM_BANK) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_fifo_mode_set(&reg_ctx, ISM6HG256X_BYPASS_MODE) != 0) return TR_ISM6HG256_ERROR;

    acc_odr = ISM6HG256X_ODR_AT_120Hz;
    acc_hg_odr = ISM6HG256X_HG_XL_ODR_AT_480Hz;
    gyro_odr = ISM6HG256X_ODR_AT_120Hz;

    if (ism6hg256x_xl_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, ISM6HG256X_HG_XL_ODR_OFF, PROPERTY_ENABLE) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_gy_data_rate_set(&reg_ctx, ISM6HG256X_ODR_OFF) != 0) return TR_ISM6HG256_ERROR;

    if (ism6hg256x_xl_full_scale_set(&reg_ctx, ISM6HG256X_4g) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_hg_xl_full_scale_set(&reg_ctx, ISM6HG256X_32g) != 0) return TR_ISM6HG256_ERROR;
    if (ism6hg256x_gy_full_scale_set(&reg_ctx, ISM6HG256X_2000dps) != 0) return TR_ISM6HG256_ERROR;

    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::ReadWhoAmI(uint8_t *id)
{
    if (id == nullptr) return TR_ISM6HG256_ERROR;
    return (ism6hg256x_device_id_get(&reg_ctx, id) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Enable_X()
{
    if (acc_is_enabled == 1U) return TR_ISM6HG256_OK;
    if (ism6hg256x_xl_data_rate_set(&reg_ctx, acc_odr) != 0) return TR_ISM6HG256_ERROR;
    acc_is_enabled = 1;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Enable_HG_X()
{
    if (acc_hg_is_enabled == 1U) return TR_ISM6HG256_OK;
    if (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, acc_hg_odr, PROPERTY_ENABLE) != 0) return TR_ISM6HG256_ERROR;
    acc_hg_is_enabled = 1;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Enable_G()
{
    if (gyro_is_enabled == 1U) return TR_ISM6HG256_OK;
    if (ism6hg256x_gy_data_rate_set(&reg_ctx, gyro_odr) != 0) return TR_ISM6HG256_ERROR;
    gyro_is_enabled = 1;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Route_DRDY_To_INT1()
{
    ism6hg256x_pin_int_route_t route = {};
    route.drdy_xl = 1;
    route.drdy_g = 1;
    if (ism6hg256x_pin_int1_route_set(&reg_ctx, &route) != 0) return TR_ISM6HG256_ERROR;

    route.drdy_hg_xl = 1;
    if (ism6hg256x_pin_int1_route_hg_set(&reg_ctx, &route) != 0) return TR_ISM6HG256_ERROR;

    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_X_OutputDataRate(float Odr)
{
    return (acc_is_enabled == 1U) ? Set_X_OutputDataRate_When_Enabled(Odr)
                                  : Set_X_OutputDataRate_When_Disabled(Odr);
}

TR_ISM6HG256Status TR_ISM6HG256::Set_HG_X_OutputDataRate(float Odr)
{
    return (acc_hg_is_enabled == 1U) ? Set_HG_X_OutputDataRate_When_Enabled(Odr)
                                     : Set_HG_X_OutputDataRate_When_Disabled(Odr);
}

TR_ISM6HG256Status TR_ISM6HG256::Set_G_OutputDataRate(float Odr)
{
    return (gyro_is_enabled == 1U) ? Set_G_OutputDataRate_When_Enabled(Odr)
                                   : Set_G_OutputDataRate_When_Disabled(Odr);
}

TR_ISM6HG256Status TR_ISM6HG256::Set_X_FullScale(int32_t FullScale)
{
    ism6hg256x_xl_full_scale_t new_fs;
    new_fs = (FullScale <= 2) ? ISM6HG256X_2g
           : (FullScale <= 4) ? ISM6HG256X_4g
           : (FullScale <= 8) ? ISM6HG256X_8g
           :                    ISM6HG256X_16g;
    return (ism6hg256x_xl_full_scale_set(&reg_ctx, new_fs) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_HG_X_FullScale(int32_t FullScale)
{
    ism6hg256x_hg_xl_full_scale_t new_fs;
    new_fs = (FullScale <=  32) ? ISM6HG256X_32g
           : (FullScale <=  64) ? ISM6HG256X_64g
           : (FullScale <= 128) ? ISM6HG256X_128g
           :                      ISM6HG256X_256g;
    return (ism6hg256x_hg_xl_full_scale_set(&reg_ctx, new_fs) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_G_FullScale(int32_t FullScale)
{
    ism6hg256x_gy_full_scale_t new_fs;
    new_fs = (FullScale <=  250) ? ISM6HG256X_250dps
           : (FullScale <=  500) ? ISM6HG256X_500dps
           : (FullScale <= 1000) ? ISM6HG256X_1000dps
           : (FullScale <= 2000) ? ISM6HG256X_2000dps
           :                       ISM6HG256X_4000dps;
    return (ism6hg256x_gy_full_scale_set(&reg_ctx, new_fs) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_X_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value)
{
    int16_t raw[3] = {0, 0, 0};
    if (ism6hg256x_acceleration_raw_get(&reg_ctx, raw) != 0) return TR_ISM6HG256_ERROR;
    Value->x = raw[0]; Value->y = raw[1]; Value->z = raw[2];
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_HG_X_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value)
{
    int16_t raw[3] = {0, 0, 0};
    if (ism6hg256x_hg_acceleration_raw_get(&reg_ctx, raw) != 0) return TR_ISM6HG256_ERROR;
    Value->x = raw[0]; Value->y = raw[1]; Value->z = raw[2];
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_G_AxesRaw(TR_ISM6HG256_AxesRaw_t *Value)
{
    int16_t raw[3] = {0, 0, 0};
    if (ism6hg256x_angular_rate_raw_get(&reg_ctx, raw) != 0) return TR_ISM6HG256_ERROR;
    Value->x = raw[0]; Value->y = raw[1]; Value->z = raw[2];
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_AllAxesRaw(TR_ISM6HG256_AxesRaw_t *gyro,
                                                  TR_ISM6HG256_AxesRaw_t *acc_lg,
                                                  TR_ISM6HG256_AxesRaw_t *acc_hg)
{
    // Single 24-byte read: 0x22 (OUTX_L_G) through 0x39 (OUTZ_H_A_OIS_HG).
    // Layout: gyro[6] | lg_accel[6] | ois_eis_gyro[6](skip) | hg_accel[6]
    uint8_t buf[24];
    if (IO_Read(buf, ISM6HG256X_OUTX_L_G, 24) != 0) return TR_ISM6HG256_ERROR;

    // Gyro: bytes 0-5
    gyro->x = (int16_t)((uint16_t)buf[1]  << 8 | buf[0]);
    gyro->y = (int16_t)((uint16_t)buf[3]  << 8 | buf[2]);
    gyro->z = (int16_t)((uint16_t)buf[5]  << 8 | buf[4]);

    // Low-G accel: bytes 6-11
    acc_lg->x = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    acc_lg->y = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    acc_lg->z = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);

    // bytes 12-17: OIS/EIS gyro — skip

    // High-G accel: bytes 18-23
    acc_hg->x = (int16_t)((uint16_t)buf[19] << 8 | buf[18]);
    acc_hg->y = (int16_t)((uint16_t)buf[21] << 8 | buf[20]);
    acc_hg->z = (int16_t)((uint16_t)buf[23] << 8 | buf[22]);

    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_DRDY_Status(ism6hg256x_data_ready_t *Status)
{
    if (Status == nullptr) return TR_ISM6HG256_ERROR;
    return (ism6hg256x_flag_data_ready_get(&reg_ctx, Status) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_X_DRDY_Status(uint8_t *Status)
{
    ism6hg256x_data_ready_t val;
    if (Get_DRDY_Status(&val) != TR_ISM6HG256_OK) return TR_ISM6HG256_ERROR;
    *Status = val.drdy_xl;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::HG_X_Get_DRDY_Status(uint8_t *Status)
{
    ism6hg256x_data_ready_t val;
    if (Get_DRDY_Status(&val) != TR_ISM6HG256_OK) return TR_ISM6HG256_ERROR;
    *Status = val.drdy_hgxl;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Get_G_DRDY_Status(uint8_t *Status)
{
    ism6hg256x_data_ready_t val;
    if (Get_DRDY_Status(&val) != TR_ISM6HG256_OK) return TR_ISM6HG256_ERROR;
    *Status = val.drdy_gy;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_X_OutputDataRate_When_Enabled(float Odr)
{
    ism6hg256x_data_rate_t new_odr;
    new_odr = (Odr <=    1.875f) ? ISM6HG256X_ODR_AT_1Hz875
            : (Odr <=    7.5f)   ? ISM6HG256X_ODR_AT_7Hz5
            : (Odr <=   15.0f)   ? ISM6HG256X_ODR_AT_15Hz
            : (Odr <=   30.0f)   ? ISM6HG256X_ODR_AT_30Hz
            : (Odr <=   60.0f)   ? ISM6HG256X_ODR_AT_60Hz
            : (Odr <=  120.0f)   ? ISM6HG256X_ODR_AT_120Hz
            : (Odr <=  240.0f)   ? ISM6HG256X_ODR_AT_240Hz
            : (Odr <=  480.0f)   ? ISM6HG256X_ODR_AT_480Hz
            : (Odr <=  960.0f)   ? ISM6HG256X_ODR_AT_960Hz
            : (Odr <= 1920.0f)   ? ISM6HG256X_ODR_AT_1920Hz
            : (Odr <= 3840.0f)   ? ISM6HG256X_ODR_AT_3840Hz
            :                      ISM6HG256X_ODR_AT_7680Hz;
    return (ism6hg256x_xl_data_rate_set(&reg_ctx, new_odr) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_X_OutputDataRate_When_Disabled(float Odr)
{
    acc_odr = (Odr <=    1.875f) ? ISM6HG256X_ODR_AT_1Hz875
            : (Odr <=    7.5f)   ? ISM6HG256X_ODR_AT_7Hz5
            : (Odr <=   15.0f)   ? ISM6HG256X_ODR_AT_15Hz
            : (Odr <=   30.0f)   ? ISM6HG256X_ODR_AT_30Hz
            : (Odr <=   60.0f)   ? ISM6HG256X_ODR_AT_60Hz
            : (Odr <=  120.0f)   ? ISM6HG256X_ODR_AT_120Hz
            : (Odr <=  240.0f)   ? ISM6HG256X_ODR_AT_240Hz
            : (Odr <=  480.0f)   ? ISM6HG256X_ODR_AT_480Hz
            : (Odr <=  960.0f)   ? ISM6HG256X_ODR_AT_960Hz
            : (Odr <= 1920.0f)   ? ISM6HG256X_ODR_AT_1920Hz
            : (Odr <= 3840.0f)   ? ISM6HG256X_ODR_AT_3840Hz
            :                      ISM6HG256X_ODR_AT_7680Hz;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_HG_X_OutputDataRate_When_Enabled(float Odr)
{
    ism6hg256x_hg_xl_data_rate_t new_odr;
    new_odr = (Odr <=  480.0f) ? ISM6HG256X_HG_XL_ODR_AT_480Hz
            : (Odr <=  960.0f) ? ISM6HG256X_HG_XL_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? ISM6HG256X_HG_XL_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? ISM6HG256X_HG_XL_ODR_AT_3840Hz
            :                    ISM6HG256X_HG_XL_ODR_AT_7680Hz;
    return (ism6hg256x_hg_xl_data_rate_set(&reg_ctx, new_odr, PROPERTY_ENABLE) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_HG_X_OutputDataRate_When_Disabled(float Odr)
{
    acc_hg_odr = (Odr <=  480.0f) ? ISM6HG256X_HG_XL_ODR_AT_480Hz
              : (Odr <=  960.0f) ? ISM6HG256X_HG_XL_ODR_AT_960Hz
              : (Odr <= 1920.0f) ? ISM6HG256X_HG_XL_ODR_AT_1920Hz
              : (Odr <= 3840.0f) ? ISM6HG256X_HG_XL_ODR_AT_3840Hz
              :                    ISM6HG256X_HG_XL_ODR_AT_7680Hz;
    return TR_ISM6HG256_OK;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_G_OutputDataRate_When_Enabled(float Odr)
{
    ism6hg256x_data_rate_t new_odr;
    new_odr = (Odr <=    7.5f) ? ISM6HG256X_ODR_AT_7Hz5
            : (Odr <=   15.0f) ? ISM6HG256X_ODR_AT_15Hz
            : (Odr <=   30.0f) ? ISM6HG256X_ODR_AT_30Hz
            : (Odr <=   60.0f) ? ISM6HG256X_ODR_AT_60Hz
            : (Odr <=  120.0f) ? ISM6HG256X_ODR_AT_120Hz
            : (Odr <=  240.0f) ? ISM6HG256X_ODR_AT_240Hz
            : (Odr <=  480.0f) ? ISM6HG256X_ODR_AT_480Hz
            : (Odr <=  960.0f) ? ISM6HG256X_ODR_AT_960Hz
            : (Odr <= 1920.0f) ? ISM6HG256X_ODR_AT_1920Hz
            : (Odr <= 3840.0f) ? ISM6HG256X_ODR_AT_3840Hz
            :                    ISM6HG256X_ODR_AT_7680Hz;
    return (ism6hg256x_gy_data_rate_set(&reg_ctx, new_odr) == 0) ? TR_ISM6HG256_OK : TR_ISM6HG256_ERROR;
}

TR_ISM6HG256Status TR_ISM6HG256::Set_G_OutputDataRate_When_Disabled(float Odr)
{
    gyro_odr = (Odr <=    7.5f) ? ISM6HG256X_ODR_AT_7Hz5
             : (Odr <=   15.0f) ? ISM6HG256X_ODR_AT_15Hz
             : (Odr <=   30.0f) ? ISM6HG256X_ODR_AT_30Hz
             : (Odr <=   60.0f) ? ISM6HG256X_ODR_AT_60Hz
             : (Odr <=  120.0f) ? ISM6HG256X_ODR_AT_120Hz
             : (Odr <=  240.0f) ? ISM6HG256X_ODR_AT_240Hz
             : (Odr <=  480.0f) ? ISM6HG256X_ODR_AT_480Hz
             : (Odr <=  960.0f) ? ISM6HG256X_ODR_AT_960Hz
             : (Odr <= 1920.0f) ? ISM6HG256X_ODR_AT_1920Hz
             : (Odr <= 3840.0f) ? ISM6HG256X_ODR_AT_3840Hz
             :                    ISM6HG256X_ODR_AT_7680Hz;
    return TR_ISM6HG256_OK;
}

uint8_t TR_ISM6HG256::IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW);
    dev_spi->transfer(RegisterAddr | 0x80);
    // Bulk transfer: fill buffer with 0x00 (MOSI) and read MISO in-place.
    // ESP32 SPI driver can use DMA for this, much faster than byte-by-byte.
    memset(pBuffer, 0x00, NumByteToRead);
    dev_spi->transfer(pBuffer, NumByteToRead);
    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();
    return 0;
}

uint8_t TR_ISM6HG256::IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW);
    dev_spi->transfer(RegisterAddr);
    for (uint16_t i = 0; i < NumByteToWrite; i++) {
        dev_spi->transfer(pBuffer[i]);
    }
    digitalWrite(cs_pin, HIGH);
    dev_spi->endTransaction();
    return 0;
}

int32_t TR_ISM6HG256::io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    return ((TR_ISM6HG256 *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t TR_ISM6HG256::io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    return ((TR_ISM6HG256 *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
