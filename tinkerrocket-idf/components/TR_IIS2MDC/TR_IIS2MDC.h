#ifndef TR_IIS2MDC_H
#define TR_IIS2MDC_H

#include <compat.h>
#include <driver/i2c_master.h>

// ---------------------------------------------------------------------------
//  Status codes
// ---------------------------------------------------------------------------
typedef enum {
    TR_IIS2MDC_OK    =  0,
    TR_IIS2MDC_ERROR = -1
} TR_IIS2MDCStatus;

// ---------------------------------------------------------------------------
//  Register addresses
// ---------------------------------------------------------------------------
namespace IIS2MDC_Reg {
    static constexpr uint8_t OFFSET_X_REG_L = 0x45;
    static constexpr uint8_t OFFSET_X_REG_H = 0x46;
    static constexpr uint8_t OFFSET_Y_REG_L = 0x47;
    static constexpr uint8_t OFFSET_Y_REG_H = 0x48;
    static constexpr uint8_t OFFSET_Z_REG_L = 0x49;
    static constexpr uint8_t OFFSET_Z_REG_H = 0x4A;
    static constexpr uint8_t WHO_AM_I       = 0x4F;
    static constexpr uint8_t CFG_REG_A      = 0x60;
    static constexpr uint8_t CFG_REG_B      = 0x61;
    static constexpr uint8_t CFG_REG_C      = 0x62;
    static constexpr uint8_t INT_CTRL_REG   = 0x63;
    static constexpr uint8_t INT_SOURCE_REG = 0x64;
    static constexpr uint8_t INT_THS_L_REG  = 0x65;
    static constexpr uint8_t INT_THS_H_REG  = 0x66;
    static constexpr uint8_t STATUS_REG     = 0x67;
    static constexpr uint8_t OUTX_L_REG     = 0x68;
    static constexpr uint8_t OUTX_H_REG     = 0x69;
    static constexpr uint8_t OUTY_L_REG     = 0x6A;
    static constexpr uint8_t OUTY_H_REG     = 0x6B;
    static constexpr uint8_t OUTZ_L_REG     = 0x6C;
    static constexpr uint8_t OUTZ_H_REG     = 0x6D;
    static constexpr uint8_t TEMP_OUT_L_REG = 0x6E;
    static constexpr uint8_t TEMP_OUT_H_REG = 0x6F;
}

// WHO_AM_I value (datasheet section 9.7)
static constexpr uint8_t IIS2MDC_WHO_AM_I_VALUE = 0x40;

// Default 7-bit I2C address (fixed; SAD pin not present on this part)
static constexpr uint8_t IIS2MDC_DEFAULT_ADDR = 0x1E;

// Sensitivity: 1.5 milligauss / LSB = 0.15 microtesla / LSB
static constexpr float IIS2MDC_LSB_TO_uT = 0.15f;

// ---------------------------------------------------------------------------
//  Output Data Rate (CFG_REG_A bits [3:2])
// ---------------------------------------------------------------------------
enum class IIS2MDC_ODR : uint8_t {
    ODR_10HZ  = 0b00,
    ODR_20HZ  = 0b01,
    ODR_50HZ  = 0b10,
    ODR_100HZ = 0b11
};

// ---------------------------------------------------------------------------
//  Operating mode (CFG_REG_A bits [1:0])
// ---------------------------------------------------------------------------
enum class IIS2MDC_Mode : uint8_t {
    CONTINUOUS = 0b00,
    SINGLE     = 0b01,
    IDLE_A     = 0b10,
    IDLE_B     = 0b11
};

// ---------------------------------------------------------------------------
//  Raw measurement (signed 16-bit per axis, native sensor frame)
// ---------------------------------------------------------------------------
struct IIS2MDC_RawData {
    int16_t x;
    int16_t y;
    int16_t z;
};

// ---------------------------------------------------------------------------
//  TR_IIS2MDC driver  (uses ESP-IDF new i2c_master API)
// ---------------------------------------------------------------------------
class TR_IIS2MDC
{
public:
    TR_IIS2MDC(uint8_t addr = IIS2MDC_DEFAULT_ADDR);

    /// Add device to an existing I2C master bus and verify WHO_AM_I.
    TR_IIS2MDCStatus begin(i2c_master_bus_handle_t bus,
                           uint32_t clock_hz = 400000);

    /// Probe WHO_AM_I — returns true iff the IIS2MDC ID byte (0x40) is read.
    bool isConnected();

    /// Software reset (CFG_REG_A bit SOFT_RST). Caller should wait ~10 ms after.
    TR_IIS2MDCStatus softReset();

    /// One-shot configuration. Defaults: 100 Hz continuous, high-resolution,
    /// offset cancellation on with periodic Set pulses, low-pass filter off,
    /// BDU on, DRDY pin off, temperature compensation always on.
    ///
    /// `set_pulse_periodic` (CFG_REG_B SET_FREQ): when true, the Set pulse
    /// fires every 63 ODR cycles instead of only at power-on. Required for
    /// OFF_CANC to actively track hard-iron drift; without it, OFF_CANC only
    /// has the single power-on Set/Reset to work with — and if the supply
    /// rail stayed alive across an ESP32 reset, even that pulse never fired.
    TR_IIS2MDCStatus configure(IIS2MDC_ODR odr = IIS2MDC_ODR::ODR_100HZ,
                               IIS2MDC_Mode mode = IIS2MDC_Mode::CONTINUOUS,
                               bool low_power = false,
                               bool offset_cancellation = true,
                               bool set_pulse_periodic = true,
                               bool low_pass_filter = false,
                               bool block_data_update = true,
                               bool drdy_on_int_pin = false);

    /// True when the Zyxda bit in STATUS_REG is set (XYZ all ready).
    bool dataReady();

    /// Read X, Y, Z magnetic field as raw signed 16-bit counts (one 6-byte burst).
    TR_IIS2MDCStatus readRawXYZ(IIS2MDC_RawData *out);

    /// Read X, Y, Z magnetic field in microtesla.
    TR_IIS2MDCStatus readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT);

    /// Read WHO_AM_I (expected 0x40).
    TR_IIS2MDCStatus readWhoAmI(uint8_t *id);

    /// Low-level register access.
    TR_IIS2MDCStatus readRegister(uint8_t reg, uint8_t *value);
    TR_IIS2MDCStatus writeRegister(uint8_t reg, uint8_t value);

private:
    i2c_master_dev_handle_t _dev;
    uint8_t _addr;

    TR_IIS2MDCStatus readRegisters(uint8_t reg, uint8_t *buf, size_t len);
};

#endif
