#ifndef TR_INA230_H
#define TR_INA230_H

#include <Arduino.h>
#include <driver/i2c.h>

// ---------------------------------------------------------------------------
//  Status codes
// ---------------------------------------------------------------------------
typedef enum {
    TR_INA230_OK    =  0,
    TR_INA230_ERROR = -1
} TR_INA230Status;

// ---------------------------------------------------------------------------
//  Register addresses  (Table 7-3)
// ---------------------------------------------------------------------------
namespace INA230_Reg {
    static constexpr uint8_t CONFIGURATION = 0x00;
    static constexpr uint8_t SHUNT_VOLTAGE = 0x01;
    static constexpr uint8_t BUS_VOLTAGE   = 0x02;
    static constexpr uint8_t POWER         = 0x03;
    static constexpr uint8_t CURRENT       = 0x04;
    static constexpr uint8_t CALIBRATION   = 0x05;
    static constexpr uint8_t MASK_ENABLE   = 0x06;
    static constexpr uint8_t ALERT_LIMIT   = 0x07;
    static constexpr uint8_t DIE_ID        = 0xFF;
}

// ---------------------------------------------------------------------------
//  Configuration register bit-field helpers
// ---------------------------------------------------------------------------

// Averaging mode  (AVG bits [11:9])
enum class INA230_Avg : uint8_t {
    AVG_1    = 0b000,
    AVG_4    = 0b001,
    AVG_16   = 0b010,
    AVG_64   = 0b011,
    AVG_128  = 0b100,
    AVG_256  = 0b101,
    AVG_512  = 0b110,
    AVG_1024 = 0b111
};

// Conversion time for bus voltage and shunt voltage (CT bits)
enum class INA230_ConvTime : uint8_t {
    CT_140us  = 0b000,
    CT_204us  = 0b001,
    CT_332us  = 0b010,
    CT_588us  = 0b011,
    CT_1100us = 0b100,   // 1.1 ms  (default)
    CT_2116us = 0b101,
    CT_4156us = 0b110,
    CT_8244us = 0b111
};

// Operating mode (MODE bits [2:0])
enum class INA230_Mode : uint8_t {
    POWER_DOWN          = 0b000,
    SHUNT_TRIG          = 0b001,
    BUS_TRIG            = 0b010,
    SHUNT_BUS_TRIG      = 0b011,
    POWER_DOWN2         = 0b100,
    SHUNT_CONTINUOUS    = 0b101,
    BUS_CONTINUOUS      = 0b110,
    SHUNT_BUS_CONTINUOUS = 0b111   // default
};

// Mask/Enable alert function select (bits [15:11])
enum class INA230_Alert : uint16_t {
    NONE  = 0x0000,
    SOL   = 0x8000,   // Shunt Over-Limit
    SUL   = 0x4000,   // Shunt Under-Limit
    BOL   = 0x2000,   // Bus Over-Limit
    BUL   = 0x1000,   // Bus Under-Limit
    POL   = 0x0800    // Power Over-Limit
};

// ---------------------------------------------------------------------------
//  Measurement result struct
// ---------------------------------------------------------------------------
struct INA230_Data {
    float shunt_voltage_mV;   // across shunt resistor
    float bus_voltage_V;      // at VBUS pin
    float current_A;          // requires calibration
    float power_W;            // requires calibration
};

// ---------------------------------------------------------------------------
//  TR_INA230 driver class  (uses ESP-IDF i2c driver, not Arduino Wire)
// ---------------------------------------------------------------------------
class TR_INA230
{
public:
    /// @param port  ESP-IDF I2C port (I2C_NUM_0 or I2C_NUM_1)
    /// @param addr  7-bit I2C address (default 0x40 = A0=GND, A1=GND)
    TR_INA230(i2c_port_t port, uint8_t addr = 0x40);

    /// Install the I2C master driver and probe the device.
    TR_INA230Status begin(int sda_pin, int scl_pin,
                          uint32_t clock_hz = 400000,
                          bool internal_pullups = false);

    /// Software reset (RST bit in Configuration register).
    TR_INA230Status reset();

    // ---- Configuration ---------------------------------------------------

    TR_INA230Status setAveraging(INA230_Avg avg);
    TR_INA230Status setBusConvTime(INA230_ConvTime ct);
    TR_INA230Status setShuntConvTime(INA230_ConvTime ct);
    TR_INA230Status setMode(INA230_Mode mode);

    TR_INA230Status setConfiguration(INA230_Avg avg,
                                     INA230_ConvTime bus_ct,
                                     INA230_ConvTime shunt_ct,
                                     INA230_Mode mode);

    TR_INA230Status getConfiguration(uint16_t *value);

    // ---- Calibration -----------------------------------------------------

    TR_INA230Status setCalibration(uint16_t cal);

    /// Compute and program CAL from shunt resistance and desired current-LSB.
    ///   r_shunt_ohm   = shunt resistance in ohms  (e.g. 0.002 for 2 mOhm)
    ///   current_lsb_A = desired current LSB in amps (e.g. 0.001 for 1 mA)
    TR_INA230Status calibrate(float r_shunt_ohm, float current_lsb_A);

    // ---- Data read -------------------------------------------------------

    TR_INA230Status readShuntVoltageRaw(int16_t *raw);
    TR_INA230Status readShuntVoltage_mV(float *mV);

    TR_INA230Status readBusVoltageRaw(uint16_t *raw);
    TR_INA230Status readBusVoltage_V(float *volts);

    TR_INA230Status readCurrentRaw(int16_t *raw);
    TR_INA230Status readCurrent_A(float *amps);

    TR_INA230Status readPowerRaw(uint16_t *raw);
    TR_INA230Status readPower_W(float *watts);

    TR_INA230Status readAll(INA230_Data *data);

    // ---- Alert -----------------------------------------------------------

    TR_INA230Status setAlert(INA230_Alert func, uint16_t limit);
    TR_INA230Status enableConversionReadyAlert(bool enable);
    TR_INA230Status readMaskEnable(uint16_t *value);

    // ---- Die ID ----------------------------------------------------------

    TR_INA230Status readDieId(uint16_t *id);

    // ---- Low-level -------------------------------------------------------

    TR_INA230Status writeRegister(uint8_t reg, uint16_t value);
    TR_INA230Status readRegister(uint8_t reg, uint16_t *value);

private:
    i2c_port_t _port;
    uint8_t    _addr;
    float      _current_lsb_A;
};

#endif
