#pragma once

#include <cstdint>
#include <driver/spi_master.h>
#include <driver/gpio.h>

// Bosch BMP5 driver
extern "C" {
  #include "bmp5.h"
  #include "bmp5_defs.h"
}

class TR_BMP585
{
public:

    // ESP-IDF native constructor.  The SPI bus identified by `host` must be
    // initialised (spi_bus_initialize) before begin() is called.  CS is
    // driven manually by the wrapper via the GPIO HAL.  SPI mode is fixed
    // to SPI_MODE0 (Bosch BMP5 SPI convention).
    TR_BMP585(spi_host_device_t host,
              uint8_t cs_pin,
              uint32_t clock_hz);

    ~TR_BMP585();

    // Startup methods
    bool    begin();
    bool    forceSoftResetRaw();
    uint8_t readChipId();
    uint8_t readChipIdCached() const;

    // Config enums
    enum class Oversampling : uint8_t
    {
        x1, x2, x4, x8, x16, x32, x64, x128
    };

    enum class IirCoeff : uint8_t
    {
        Bypass, x2, x4, x8, x16, x32, x64, x128
    };

  // These are “semantic” ODR choices we map to Bosch constants.
  // Keep / trim as you like, but these match the .cpp mapper.
  enum class OutputDataRate : uint8_t
  {
    ODR_240Hz,
    ODR_199_1Hz,
    ODR_149_3Hz,
    ODR_100_2Hz,
    ODR_50Hz,
    ODR_25Hz,
    ODR_15Hz,
    ODR_10Hz,
    ODR_05Hz,
    ODR_01Hz,
    ODR_0_5Hz,
    ODR_0_250Hz,
    ODR_0_125Hz
  };

  enum class PowerMode : uint8_t
  {
    Standby,
    Normal,
    Forced,
    Continuous
  };

  // Config methods (requested)
  bool setTemperatureOversampling(Oversampling osr);
  bool setPressureOversampling(Oversampling osr);
  bool setIirFilter(IirCoeff t, IirCoeff p);
  bool setOutputDataRate(OutputDataRate odr);

  bool setPowerMode(PowerMode mode);

    struct __attribute__((packed)) BmpCompFrame
    {
      uint32_t t_us;     // micros()
      int32_t  temp_q16; // degC * 65536
      uint32_t press_q6; // Pa * 64
    };
    static_assert(sizeof(BmpCompFrame) == 12, "BmpCompFrame should be 12 bytes");

    bool readCompFrame(BmpCompFrame& out);

    // Interrupts
    // Configures INT pin behavior and selects DRDY as an interrupt source.
    // latched=true   -> INT stays asserted until status is read (latched)
    // latched=false  -> pulsed DRDY (easiest for ISR use)
    // activeHigh=true -> active-high INT, else active-low
    // openDrain=true  -> open-drain, else push-pull
    bool enableDataReadyInterrupt(bool enable,
                                bool latched = false,
                                bool activeHigh = true,
                                bool openDrain = false);

private:
  // Bosch SPI callbacks
  static BMP5_INTF_RET_TYPE spiRead_(uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr);
  static BMP5_INTF_RET_TYPE spiWrite_(uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr);
  static void delayUs_(uint32_t period, void* intf_ptr);

  // Manual CS control via the IDF GPIO HAL.  spics_io_num is set to -1 in
  // the device config, so IDF does not drive CS — this wrapper does.
  inline void csSelect_()   { gpio_set_level(static_cast<gpio_num_t>(cs_pin_), 0); }
  inline void csDeselect_() { gpio_set_level(static_cast<gpio_num_t>(cs_pin_), 1); }

  // Lazily add the SPI device on first begin()/forceSoftResetRaw().  Returns
  // false on add_device failure.  Idempotent across retries.
  bool ensureSpiDevice_();

  // Issue the raw Bosch soft reset (0xB6 -> CMD 0x7E) and let the sensor
  // settle. Caller must already hold a configured CS pin and SPI device.
  // Split out of forceSoftResetRaw() so begin() can re-arm the sensor's
  // post-POR state without also running the verification read.
  void issueSoftReset_();

  // One throwaway register read. The BMP5 SPI interface swallows the first
  // transaction after a power-on or soft reset (Bosch's own bmp5_init() opens
  // with exactly this for the same reason), so any read that is not preceded
  // by one can return garbage.
  void spiDummyRead_();

  // Configure cs_pin_ as a GPIO output, idle HIGH.
  void configureCsPin_();

  // Mappers
  static uint8_t mapOsr_(Oversampling osr);
  static uint8_t mapIir_(IirCoeff c);
  static uint8_t mapOdr_(OutputDataRate odr);
  static bool nearestOdrFromHz_(float hz, OutputDataRate& out);

  // Members
  spi_host_device_t   host_;
  uint8_t             cs_pin_;
  uint32_t            clock_hz_;
  spi_device_handle_t spi_dev_ = nullptr;

  bmp5_dev dev_ {};

  // Cached configuration we pass into Bosch functions
  bmp5_osr_odr_press_config osrOdr_ {};
  bmp5_iir_config iir_ {};

  // Cached chip id
  uint8_t chipIdCached_ = 0;
};
