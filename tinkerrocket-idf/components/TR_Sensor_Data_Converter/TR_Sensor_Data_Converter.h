#ifndef SENSORCONVERTER_H
#define SENSORCONVERTER_H

#include <compat.h>
#include <RocketComputerTypes.h>

// ISM6HG256X user-facing full-scale selections
// Supported ranges:
//  - Low-g accel: ±2/±4/±8/±16 g
//  - High-g accel: ±32/±64/±128/±256 g
//  - Gyro: ±250/±500/±1000/±2000/±4000 dps
enum class ISM6LowGFullScale : 
    uint8_t {FS_2G = 2,
             FS_4G = 4,
             FS_8G = 8,
             FS_16G = 16 };
enum class ISM6HighGFullScale : 
    uint16_t {FS_32G = 32,
              FS_64G = 64,
              FS_128G = 128,
              FS_256G = 256 };
enum class ISM6GyroFullScale : 
    uint16_t {DPS_250 = 250,
              DPS_500 = 500,
              DPS_1000 = 1000,
              DPS_2000 = 2000,
              DPS_4000 = 4000 };

// SensorConverter
// - Converts packed raw sensor data to SI/human-readable structs.
// - Provides pack/unpack for LoRa telemetry payload.
class SensorConverter 
{
public:


    SensorConverter();
    
    // Configure ISM6HG256: pass full-scale ranges for each
    // component of the sensor.
    void configureISM6HG256FullScale(ISM6LowGFullScale low_g_fs,
                                     ISM6HighGFullScale high_g_fs,
                                     ISM6GyroFullScale gyro_fs);
    void configureISM6HG256RotationZ(float rotation_z_deg);
    void configureMMC5983MARotationZ(float rotation_z_deg);
    void configureIIS2MDCRotationZ(float rotation_z_deg);

    // Set high-g accelerometer bias (m/s², in body frame).
    // Subtracted from ISM6HG256 high-g output during conversion.
    void setHighGBias(float bx, float by, float bz);

    // --- Data Conversion Functions ---
    // Convert from packed to human readable SI
    void convertGNSSData(const GNSSData& in, 
                         GNSSDataSI& out);
    void convertPowerData(const POWERData& in,
                          POWERDataSI& out);
    void convertBMP585Data(const BMP585Data& in,
                           BMP585DataSI& out);
    void convertISM6HG256Data(const ISM6HG256Data& in, 
                              ISM6HG256DataSI& out);
    void convertMMC5983MAData(const MMC5983MAData& in,
                              MMC5983MADataSI& out);
    void convertIIS2MDCData(const IIS2MDCData& in,
                            IIS2MDCDataSI& out);
    void convertNonSensorData(const NonSensorData& in, 
                              NonSensorDataSI& out);
    
    // --- SI -> Packed Power Data ---
    void packPowerData(const POWERDataSI& in, POWERData& out);

    // --- Pack and Unpack LoRa Data
    void packLoRa(const LoRaDataSI& in, LoRaData& out);
    void unpackLoRa(const LoRaData& in, LoRaDataSI& out);
    void packLoRaData(const LoRaDataSI& in, uint8_t out_bytes[SIZE_OF_LORA_DATA]);
    void unpackLoRa(const uint8_t in_bytes[SIZE_OF_LORA_DATA], LoRaDataSI& out);
                               
    
private:

private:
    float g_ms2 = 9.80665f;

    float acc_low_ms2_per_lsb  = 0.0f;
    float acc_high_ms2_per_lsb = 0.0f;
    float gyro_dps_per_lsb     = 0.0f;
    float ism6_rot_z_rad       = 0.0f;
    float mmc_rot_z_rad        = 0.0f;
    float iis2mdc_rot_z_rad    = 0.0f;

    // High-g accelerometer bias (m/s², body frame)
    float hg_bias_x_ = 0.0f, hg_bias_y_ = 0.0f, hg_bias_z_ = 0.0f;
  
    // --- Encode/Decode Helpers ---
    static float decodeVoltageFromInt(uint16_t raw);
    static float decodeCurrentFromInt(int16_t raw);
    static float decodeSOCFromInt(int16_t raw);

    static uint16_t encodeVoltage(float voltage_v);
    static int16_t  encodeCurrent(float current_ma);
    static int16_t  encodeSOC(float soc_pct);

    // --- ECEF to Geo ---
    static void ecefToLla(double x, 
                        double y,
                        double z,
                        double &lat_deg,
                        double &lon_deg,
                        double &alt_m);

    // ---- Generic helpers ----
    static inline float  clampf(float v, float lo, float hi)
        { return v < lo ? lo : (v > hi ? hi : v); }
    static inline double clampd(double v, double lo, double hi)
        { return v < lo ? lo : (v > hi ? hi : v); }
    static inline int32_t lroundi32(double x)
        { return (int32_t)llround(x); }

    // Little-endian i16 helpers
    static inline void put_i16(uint8_t* b, int16_t v)
    {
        b[0] = (uint8_t)(v & 0xFF);
        b[1] = (uint8_t)((v >> 8) & 0xFF);
    }
    static inline int16_t get_i16(const uint8_t* b)
    {
        return (int16_t)((int16_t)b[0] | ((int16_t)b[1] << 8));
    }

    // Little-endian signed i24 helpers (two's complement)
    static inline void put_i24(uint8_t* b, int32_t v)
    {
        b[0] = (uint8_t)(v & 0xFF);
        b[1] = (uint8_t)((v >> 8) & 0xFF);
        b[2] = (uint8_t)((v >> 16) & 0xFF);
    }
    static inline int32_t get_i24(const uint8_t* b)
    {
        int32_t v = (int32_t)b[0] | ((int32_t)b[1] << 8) | ((int32_t)b[2] << 16);
        if (v & 0x00800000) v |= 0xFF000000; // sign extend
        return v;
    }

    static inline i24le_t i24_from_i32(int32_t v)
    {
        // clamp to signed 24-bit
        if (v >  8388607) v =  8388607;
        if (v < -8388608) v = -8388608;
        i24le_t out;
        uint8_t* p = (uint8_t*)&out;
        put_i24(p, v);
        return out;
    }
    static inline int32_t i24_to_i32(i24le_t v)
    {
        const uint8_t* p = (const uint8_t*)&v;
        return get_i24(p);
    }

    // 0.01 V resolution, 2.00–10.00 V → 0…255 (uint8_t) used in LoRaData
    static inline uint8_t encodeVoltage_2_10_01(float v)
    {
        const float VMIN = 2.0f, VMAX = 10.0f;
        if (v < VMIN) v = VMIN;
        if (v > VMAX) v = VMAX;
        int raw = (int)lroundf((v - VMIN) * (255.0f / (VMAX - VMIN)));
        if (raw < 0) raw = 0;
        if (raw > 255) raw = 255;
        return (uint8_t)raw;
    }
    static inline float decodeVoltage_2_10_01(uint8_t raw)
    {
        const float VMIN = 2.0f, VMAX = 10.0f;
        return VMIN + ((float)raw * (VMAX - VMIN) / 255.0f);
    }

};

#endif
