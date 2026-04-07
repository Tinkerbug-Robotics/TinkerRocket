#include "TR_BMP585.h"

// ---------------- Ctor ----------------

TR_BMP585::TR_BMP585(SPIClass& spi,
                     uint8_t csPin,
                     const SPISettings& spiSettings)
  : spi_(spi),
    spiSettings_(spiSettings),
    cs_(csPin)
{
}

// ---------------- SPI helpers ----------------

void TR_BMP585::beginTxn_()
{
  spi_.beginTransaction(spiSettings_);
}

void TR_BMP585::endTxn_()
{
  spi_.endTransaction();
}

void TR_BMP585::csSelect_()
{
  digitalWrite(cs_, LOW);
}

void TR_BMP585::csDeselect_()
{
  digitalWrite(cs_, HIGH);
}

// ---------------- Bosch interface glue ----------------

BMP5_INTF_RET_TYPE TR_BMP585::spiRead_(uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr)
{
  auto* self = reinterpret_cast<TR_BMP585*>(intf_ptr);
  if (!self || !data || len == 0) return BMP5_E_NULL_PTR;

  self->beginTxn_();
  self->csSelect_();

  // Bosch bmp5 SPI convention: bit7=1 for read
  self->spi_.transfer(reg | 0x80);
  self->spi_.transfer(data, len);

  self->csDeselect_();
  self->endTxn_();

  return BMP5_OK;
}

BMP5_INTF_RET_TYPE TR_BMP585::spiWrite_(uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr)
{
  auto* self = reinterpret_cast<TR_BMP585*>(intf_ptr);
  if (!self || !data || len == 0) return BMP5_E_NULL_PTR;

  self->beginTxn_();
  self->csSelect_();

  // write: bit7=0
  self->spi_.transfer(reg & 0x7F);
  for (uint32_t i = 0; i < len; i++) self->spi_.transfer(data[i]);

  self->csDeselect_();
  self->endTxn_();

  return BMP5_OK;
}

void TR_BMP585::delayUs_(uint32_t period, void* /*intf_ptr*/)
{
  delayMicroseconds(period);
}

// ---------------- Mappers ----------------

uint8_t TR_BMP585::mapOsr_(Oversampling osr)
{
  switch (osr)
  {
    case Oversampling::x1:   return BMP5_OVERSAMPLING_1X;
    case Oversampling::x2:   return BMP5_OVERSAMPLING_2X;
    case Oversampling::x4:   return BMP5_OVERSAMPLING_4X;
    case Oversampling::x8:   return BMP5_OVERSAMPLING_8X;
    case Oversampling::x16:  return BMP5_OVERSAMPLING_16X;
    case Oversampling::x32:  return BMP5_OVERSAMPLING_32X;
    case Oversampling::x64:  return BMP5_OVERSAMPLING_64X;
    case Oversampling::x128: return BMP5_OVERSAMPLING_128X;
  }
  return BMP5_OVERSAMPLING_1X;
}

uint8_t TR_BMP585::mapIir_(IirCoeff c)
{
  switch (c)
  {
    case IirCoeff::Bypass: return BMP5_IIR_FILTER_BYPASS;
    case IirCoeff::x2:     return BMP5_IIR_FILTER_COEFF_1;
    case IirCoeff::x4:     return BMP5_IIR_FILTER_COEFF_3;
    case IirCoeff::x8:     return BMP5_IIR_FILTER_COEFF_7;
    case IirCoeff::x16:    return BMP5_IIR_FILTER_COEFF_15;
    case IirCoeff::x32:    return BMP5_IIR_FILTER_COEFF_31;
    case IirCoeff::x64:    return BMP5_IIR_FILTER_COEFF_63;
    case IirCoeff::x128:   return BMP5_IIR_FILTER_COEFF_127;
  }
  return BMP5_IIR_FILTER_BYPASS;
}

uint8_t TR_BMP585::mapOdr_(OutputDataRate odr)
{
  switch (odr)
  {
    case OutputDataRate::ODR_240Hz:    return BMP5_ODR_240_HZ;
    case OutputDataRate::ODR_199_1Hz:  return BMP5_ODR_199_1_HZ;
    case OutputDataRate::ODR_149_3Hz:  return BMP5_ODR_149_3_HZ;
    case OutputDataRate::ODR_100_2Hz:  return BMP5_ODR_100_2_HZ;
    case OutputDataRate::ODR_50Hz:     return BMP5_ODR_50_HZ;
    case OutputDataRate::ODR_25Hz:     return BMP5_ODR_25_HZ;
    case OutputDataRate::ODR_15Hz:     return BMP5_ODR_15_HZ;
    case OutputDataRate::ODR_10Hz:     return BMP5_ODR_10_HZ;
    case OutputDataRate::ODR_05Hz:     return BMP5_ODR_05_HZ;
    case OutputDataRate::ODR_01Hz:     return BMP5_ODR_01_HZ;     // NOTE: Bosch names it 01
    case OutputDataRate::ODR_0_5Hz:    return BMP5_ODR_0_5_HZ;
    case OutputDataRate::ODR_0_250Hz:  return BMP5_ODR_0_250_HZ;
    case OutputDataRate::ODR_0_125Hz:  return BMP5_ODR_0_125_HZ;
  }
  return BMP5_ODR_15_HZ;
}

bool TR_BMP585::nearestOdrFromHz_(float hz, OutputDataRate& out)
{
  if (!(hz > 0.0f)) return false;

  // Keep this simple: choose the closest from your enum set.
  struct Candidate { float hz; OutputDataRate e; };
  static const Candidate cands[] = {
    {240.0f,   OutputDataRate::ODR_240Hz},
    {199.1f,   OutputDataRate::ODR_199_1Hz},
    {149.3f,   OutputDataRate::ODR_149_3Hz},
    {100.2f,   OutputDataRate::ODR_100_2Hz},
    {50.0f,    OutputDataRate::ODR_50Hz},
    {25.0f,    OutputDataRate::ODR_25Hz},
    {15.0f,    OutputDataRate::ODR_15Hz},
    {10.0f,    OutputDataRate::ODR_10Hz},
    {5.0f,     OutputDataRate::ODR_05Hz},
    {1.0f,     OutputDataRate::ODR_01Hz},
    {0.5f,     OutputDataRate::ODR_0_5Hz},
    {0.25f,    OutputDataRate::ODR_0_250Hz},
    {0.125f,   OutputDataRate::ODR_0_125Hz},
  };

  float bestErr = 1e30f;
  OutputDataRate best = cands[0].e;
  for (auto& c : cands)
  {
    float err = (hz > c.hz) ? (hz - c.hz) : (c.hz - hz);
    if (err < bestErr) { bestErr = err; best = c.e; }
  }

  out = best;
  return true;
}

// ---------------- Public API ----------------

bool TR_BMP585::begin()
{
  pinMode(cs_, OUTPUT);
  csDeselect_();
  delay(5);

  // Fill Bosch dev struct
  dev_.intf = BMP5_SPI_INTF;
  dev_.read = &TR_BMP585::spiRead_;
  dev_.write = &TR_BMP585::spiWrite_;
  dev_.delay_us = &TR_BMP585::delayUs_;
  dev_.intf_ptr = this;
    
  int8_t rslt = bmp5_init(&dev_);
  if (rslt != BMP5_OK) return false;

  chipIdCached_ = dev_.chip_id;

  // Safe defaults
  osrOdr_.osr_t = BMP5_OVERSAMPLING_1X;
  osrOdr_.osr_p = BMP5_OVERSAMPLING_1X;
  osrOdr_.odr   = BMP5_ODR_15_HZ;
  osrOdr_.press_en = BMP5_ENABLE;

  iir_.set_iir_t = BMP5_IIR_FILTER_BYPASS;
  iir_.set_iir_p = BMP5_IIR_FILTER_BYPASS;

  (void)bmp5_set_osr_odr_press_config(&osrOdr_, &dev_);
  (void)bmp5_set_iir_config(&iir_, &dev_);

  return true;
}

bool TR_BMP585::forceSoftResetRaw()
{
  pinMode(cs_, OUTPUT);
  csDeselect_();
  delay(2);

  // Raw Bosch soft reset command: write 0xB6 to CMD register 0x7E.
  beginTxn_();
  csSelect_();
  spi_.transfer(BMP5_REG_CMD & 0x7F);
  spi_.transfer(BMP5_SOFT_RESET_CMD);
  csDeselect_();
  endTxn_();

  delay(5);

  // Read chip-id directly to confirm SPI communication is back.
  uint8_t id = 0;
  beginTxn_();
  csSelect_();
  spi_.transfer(BMP5_REG_CHIP_ID | 0x80);
  id = spi_.transfer(0x00);
  csDeselect_();
  endTxn_();

  chipIdCached_ = id;
  return (id == BMP5_CHIP_ID_PRIM) || (id == BMP5_CHIP_ID_SEC);
}

uint8_t TR_BMP585::readChipId()
{
  uint8_t id = 0;
  (void)bmp5_get_regs(BMP5_REG_CHIP_ID, &id, 1, &dev_);
  chipIdCached_ = id;
  return id;
}

uint8_t TR_BMP585::readChipIdCached() const
{
  return chipIdCached_;
}

bool TR_BMP585::setTemperatureOversampling(Oversampling osr)
{
  osrOdr_.osr_t = mapOsr_(osr);
  return (bmp5_set_osr_odr_press_config(&osrOdr_, &dev_) == BMP5_OK);
}

bool TR_BMP585::setPressureOversampling(Oversampling osr)
{
  osrOdr_.osr_p = mapOsr_(osr);
  return (bmp5_set_osr_odr_press_config(&osrOdr_, &dev_) == BMP5_OK);
}

bool TR_BMP585::setIirFilter(IirCoeff t, IirCoeff p)
{
  iir_.set_iir_t = mapIir_(t);
  iir_.set_iir_p = mapIir_(p);
  return (bmp5_set_iir_config(&iir_, &dev_) == BMP5_OK);
}

bool TR_BMP585::setOutputDataRate(OutputDataRate odr)
{
  osrOdr_.odr = mapOdr_(odr);
  return (bmp5_set_osr_odr_press_config(&osrOdr_, &dev_) == BMP5_OK);
}

bool TR_BMP585::setPowerMode(PowerMode mode)
{
  bmp5_powermode pm = BMP5_POWERMODE_STANDBY;
  switch (mode)
  {
    case PowerMode::Standby:    pm = BMP5_POWERMODE_STANDBY; break;
    case PowerMode::Normal:     pm = BMP5_POWERMODE_NORMAL; break;
    case PowerMode::Forced:     pm = BMP5_POWERMODE_FORCED; break;
    case PowerMode::Continuous: pm = BMP5_POWERMODE_CONTINOUS; break; // Bosch spelling
  }
  return (bmp5_set_power_mode(pm, &dev_) == BMP5_OK);
}

bool TR_BMP585::readCompFrame(BmpCompFrame& out)
{
  bmp5_sensor_data d{};

  // NOTE: bmp5_get_sensor_data signature requires osr/odr config pointer too
  int8_t rslt = bmp5_get_sensor_data(&d, &osrOdr_, &dev_);
  if (rslt != BMP5_OK) return false;

  out.t_us = (uint32_t)micros();

#if defined(BMP5_USE_FIXED_POINT)
  // In fixed-point builds, Bosch already uses integer raw formats internally.
  // But many Arduino builds do NOT enable this, so most people hit the float path below.
  // If you *are* using fixed-point, confirm your Bosch config:
  // - If temp is already Q16.16 and pressure already *64, you can assign directly (with width checks).
  out.temp_q16 = (int32_t)d.temperature;   // only correct if d.temperature is already Q16.16
  out.press_q6 = (uint32_t)d.pressure;     // only correct if d.pressure is already Pa*64
#else
  // Normal Bosch build: temperature in degC (float), pressure in Pa (float)
  out.temp_q16 = (int32_t)lroundf(((float)d.temperature) * 65536.0f);
  out.press_q6 = (uint32_t)lroundf(((float)d.pressure) * 64.0f);
#endif

  return true;
}

bool TR_BMP585::enableDataReadyInterrupt(bool enable,
                                        bool latched,
                                        bool activeHigh,
                                        bool openDrain)
{
  const bmp5_intr_mode     mode = latched    ? BMP5_LATCHED     : BMP5_PULSED;
  const bmp5_intr_polarity pol  = activeHigh ? BMP5_ACTIVE_HIGH : BMP5_ACTIVE_LOW;
  const bmp5_intr_drive    drv  = openDrain  ? BMP5_INTR_OPEN_DRAIN : BMP5_INTR_PUSH_PULL;
  const bmp5_intr_en_dis   en   = enable     ? BMP5_INTR_ENABLE : BMP5_INTR_DISABLE;

  // NOTE: per bmp5.h, dev is LAST parameter
  int8_t rslt = bmp5_configure_interrupt(mode, pol, drv, en, &dev_);
  if (rslt != BMP5_OK) return false;

  // Route DRDY to interrupt pin
  // NOTE: this is a struct (not typedef) in the Bosch headers
  struct bmp5_int_source_select src = {};
  src.drdy_en = enable ? BMP5_ENABLE : BMP5_DISABLE;

  // NOTE: per bmp5.h, dev is SECOND parameter
  rslt = bmp5_int_source_select(&src, &dev_);
  return (rslt == BMP5_OK);
}
