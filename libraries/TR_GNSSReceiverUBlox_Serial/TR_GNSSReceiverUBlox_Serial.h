// TR_GNSSReceiverUBlox.h (Serial version)
#ifndef TRGNSSRECEIVERUBLOX_H
#define TRGNSSRECEIVERUBLOX_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "TR_SparkFun_u-blox_GNSS_v3.h"
#include <sys/time.h>
#include <string>

#include <RocketComputerTypes.h>

class TR_GNSSReceiverUBloxSerial
{
    public:
    
        // Constructor
        TR_GNSSReceiverUBloxSerial();

        // Initialize GNSS over UART
        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);
               
        // Update a copy of GNSSData
        void getGNSSData(GNSSData &data);
        
    private:

        SFE_UBLOX_GNSS_SERIAL gnss;

        uint8_t update_rate_hz;

        // Staleness detection: if GNSS second+millisecond are unchanged for
        // STALE_THRESHOLD consecutive getGNSSData() calls, the serial link
        // has likely lost sync and the library is returning cached data.
        static constexpr uint16_t STALE_THRESHOLD = 5;  // ~500 ms at 10 Hz
        uint8_t  prev_second      = 0xFF;   // impossible initial value
        uint16_t prev_milli_second = 0xFFFF;
        uint16_t stale_count       = 0;

        void enuToEcefVelocityAndPosition(float v_east,
                                          float v_north,
                                          float v_up,
                                          double lat_rad,
                                          double lon_rad,
                                          double height,
                                          int16_t& ecef_vX,
                                          int16_t& ecef_vY,
                                          int16_t& ecef_vZ,
                                          int32_t& ecef_X,
                                          int32_t& ecef_Y,
                                          int32_t& ecef_Z);

};

#endif
