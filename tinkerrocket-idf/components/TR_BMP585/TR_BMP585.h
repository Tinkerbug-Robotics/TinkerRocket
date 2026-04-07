#pragma once

#include <compat.h>

// Bosch BMP5 driver
extern "C" {
  #include "bmp5.h"
  #include "bmp5_defs.h"
}

class TR_BMP585
{
public:

    // Constructor
    TR_BMP585(SPIClass& spi,
              uint8_t csPin,
              const SPISettings& spiSettings);

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

  // SPI helpers (use spiSettings_ consistently)
  void beginTxn_();
  void endTxn_();
  void csSelect_();
  void csDeselect_();

  // Mappers
  static uint8_t mapOsr_(Oversampling osr);
  static uint8_t mapIir_(IirCoeff c);
  static uint8_t mapOdr_(OutputDataRate odr);
  static bool nearestOdrFromHz_(float hz, OutputDataRate& out);

  // Members
  SPIClass& spi_;
  SPISettings spiSettings_;
  uint8_t cs_;

  bmp5_dev dev_ {};

  // Cached configuration we pass into Bosch functions
  bmp5_osr_odr_press_config osrOdr_ {};
  bmp5_iir_config iir_ {};

  // Cached chip id
  uint8_t chipIdCached_ = 0;
};
