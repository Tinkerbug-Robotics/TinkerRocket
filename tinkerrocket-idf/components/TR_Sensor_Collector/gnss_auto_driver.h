#pragma once

// One V9 or V10 image, any GNSS receiver on J1 (TR_GNSS_DRIVER_AUTO). Their
// J1 takes the stock u-blox SAM-M10Q carrier, a Septentrio mosaic-G5 harness
// or TinkerRocket's gnss-sdr-dev board, and until this the receiver was the
// build's choice: an image built for another one searched for 30-60 s and
// flew GNSS-absent. TR_GNSSAuto holds the three drivers. begin() finds out
// which receiver is there (gnss_detect.h), then runs that one's begin()
// exactly as its own image ran it, and every later call goes to that driver.
// It has the call surface the drivers share plus the mosaic's extras (each
// false or a no-op while another is in use), so it slots into
// SensorCollector's GNSS seam the way any driver does, and the way TR_MagAuto
// does into the magnetometer's. type() says which receiver begin() found.
//
// The receiver found is kept in NVS ("gnssauto"/"rx") and probed first next
// boot, so a boot with the same receiver costs one probe (0.1-0.4 s). It is
// written before that receiver's driver starts, so a crash inside the
// mosaic's driver is known for one on the boot that follows (main.cpp keeps
// the mosaic out of a boot after a crash, as the mosaic image did).

#include <cstddef>
#include <cstdint>
#include <driver/uart.h>

#include <RocketComputerTypes.h>
#include <TR_GNSSReceiverMosaic_Serial.h>
#include <TR_GNSSReceiverSdr_Serial.h>
#include <TR_GNSSReceiverUBlox_Serial.h>

#include "gnss_detect.h"

class TR_GNSSAuto
{
public:
    explicit TR_GNSSAuto(uart_port_t uart_port = UART_NUM_1);

    // Before begin(). The mosaic's 1PPS pin (J1 pin 5) is only stored here:
    // the mosaic's driver sets it up once the mosaic is found, and nothing
    // touches it otherwise, because on the PX1105R carrier that pin is a
    // receiver input. Its rate: the mosaic's permission file allows 20 Hz.
    void setPpsPin(int8_t pin) { mosaic_.setPpsPin(pin); }
    void setMosaicRateHz(uint8_t hz) { mosaic_rate_hz_ = hz; }

    // The receiver found last boot (GNSS_TYPE_*), from NVS. Readable before
    // begin().
    uint8_t rememberedType();

    // Find the receiver and bring it up. False when none came up (#557
    // GNSS-absent mode). reset_n_pin / safeboot_n_pin go to the u-blox's
    // driver, as they always have.
    bool begin(uint8_t update_rate_hz,
               uint8_t GNSS_RX,
               uint8_t GNSS_TX,
               int8_t reset_n_pin = -1,
               int8_t safeboot_n_pin = -1);

    uint8_t type() const { return type_; }   // GNSS_TYPE_NONE until begin() brought one up
    // The navigation rate the receiver in use runs at: the GNSS poll task's
    // is twice it (the gnss-sdr-dev board's driver is read every 25 ms, as
    // the mosaic's).
    uint8_t updateRateHz() const;
    // The u-blox's high-performance-clock OTP state; NOT_M10 with the others.
    uint8_t otpState() const;

    void getGNSSData(GNSSData& data);
    bool pollNewPVT(GNSSData& data);
    bool pollNewSat(GNSSSatData& out);   // the u-blox's NAV-SAT; false with the others, as their own drivers
    // The mosaic's extras (TR_GNSSReceiverMosaic_Serial.h); false or a no-op
    // with the u-blox. pollRawFrame gives the gnss-sdr-dev board's RTCM3
    // frames too.
    bool pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us);
    bool takeClockLine(GnssClockData& out);
    bool geoidSeparationM(float& sep_m) const;
    void requestNavResend();
#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG
    void logSatDiag();   // the u-blox's; nothing with the mosaic
#endif

private:
    void runPlan(const gnss_detect::Plan& plan);
    bool uartOpen(uint32_t baud);
    void uartClose();
    void storeHint(uint8_t type);
    // The u-blox driver's own last-good link (its "gnsslink" record), read
    // only: the baud it found its receiver at, and whether on swapped wires.
    void ubloxLastLink(uint32_t& baud, bool& swapped) const;

    uart_port_t port_;
    uint8_t     rx_pin_ = 0;
    uint8_t     tx_pin_ = 0;
    uint8_t     ublox_rate_hz_ = 18;
    uint8_t     mosaic_rate_hz_ = 20;
    uint8_t     type_ = GNSS_TYPE_NONE;
    bool        hint_loaded_ = false;
    uint8_t     hint_ = GNSS_TYPE_NONE;   // what NVS holds

    TR_GNSSReceiverUBloxSerial  ublox_;
    TR_GNSSReceiverMosaicSerial mosaic_;
    TR_GNSSReceiverSdrSerial    sdr_;
    gnss_detect::Detector       detector_;
};
