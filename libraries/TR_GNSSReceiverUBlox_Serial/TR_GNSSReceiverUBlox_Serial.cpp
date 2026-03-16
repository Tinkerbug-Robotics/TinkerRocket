#include "TR_GNSSReceiverUBlox_Serial.h"

// Constructor
TR_GNSSReceiverUBloxSerial::TR_GNSSReceiverUBloxSerial() {}

bool TR_GNSSReceiverUBloxSerial::begin(uint8_t update_rate_hz_in,
                                       uint8_t GNSS_RX,
                                       uint8_t GNSS_TX,
                                       int8_t reset_n_pin,
                                       int8_t safeboot_n_pin) 
{
    update_rate_hz = update_rate_hz_in;

    // Increase Serial1 RX buffer before any .begin() call.
    // Default 256 bytes overflows at 25 Hz UBX PVT (~100 B/epoch @ 460800 baud),
    // causing the SparkFun parser to lose frame sync and silently return
    // cached (stale) position data.
    Serial1.setRxBufferSize(4096);

    Serial.println("Starting SAM-M10Q configuration...");

    const uint32_t preferred_baud = 460800U;
    const uint32_t bootstrap_baud = preferred_baud;
    const uint32_t probe_bauds[] = {9600U, 230400U, 115200U, 460800U, 38400U, 57600U};
    uint32_t connected_baud = 0U;
    uint8_t active_rx = GNSS_RX;
    uint8_t active_tx = GNSS_TX;

    if (safeboot_n_pin >= 0)
    {
        pinMode((uint8_t)safeboot_n_pin, OUTPUT);
        digitalWrite((uint8_t)safeboot_n_pin, HIGH);
        Serial.print("GNSS: SAFEBOOT_N forced HIGH on pin ");
        Serial.println(safeboot_n_pin);
        delay(10);
    }

    auto pulseReset = [&]()
    {
        if (reset_n_pin < 0)
        {
            return;
        }

        pinMode((uint8_t)reset_n_pin, OUTPUT);
        digitalWrite((uint8_t)reset_n_pin, HIGH);
        delay(2);
        digitalWrite((uint8_t)reset_n_pin, LOW);
        delay(20);
        digitalWrite((uint8_t)reset_n_pin, HIGH);
        delay(250);
        Serial.print("GNSS: pulsed RESET_N on pin ");
        Serial.println(reset_n_pin);
    };

    auto hasSerialActivity = [&](uint8_t rx_pin, uint8_t tx_pin, uint32_t baud) -> bool
    {
        Serial1.end();
        delay(10);
        Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

        const uint32_t start_ms = millis();
        while ((millis() - start_ms) < 250U)
        {
            while (Serial1.available() > 0)
            {
                (void)Serial1.read();
            }
            delay(1);
        }

        const uint32_t activity_window_start = millis();
        while ((millis() - activity_window_start) < 400U)
        {
            if (Serial1.available() > 0)
            {
                Serial.print("GNSS: serial activity detected on RX=");
                Serial.print(rx_pin);
                Serial.print(" TX=");
                Serial.print(tx_pin);
                Serial.print(" at ");
                Serial.print(baud);
                Serial.println(" baud");
                return true;
            }
            delay(1);
        }
        return false;
    };

    auto scanAndConnectPins = [&](uint8_t rx_pin, uint8_t tx_pin, uint32_t &found_baud) -> bool
    {
        const size_t n = sizeof(probe_bauds) / sizeof(probe_bauds[0]);
        for (size_t i = 0; i < n; i++)
        {
            const uint32_t baud = probe_bauds[i];
            Serial.print("GNSS: trying ");
            Serial.print(baud);
            Serial.print(" baud (RX=");
            Serial.print(rx_pin);
            Serial.print(", TX=");
            Serial.print(tx_pin);
            Serial.println(")");

            Serial1.end();
            delay(20);
            Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
            delay(100);

            if (gnss.begin(Serial1, 1500) == true)
            {
                found_baud = baud;
                active_rx = rx_pin;
                active_tx = tx_pin;
                return true;
            }

            // If we see bytes but cannot establish full UBX handshake,
            // use assumeSuccess path and attempt to recover configuration.
            if (hasSerialActivity(rx_pin, tx_pin, baud) && gnss.begin(Serial1, 1500, true))
            {
                found_baud = baud;
                active_rx = rx_pin;
                active_tx = tx_pin;
                Serial.println("GNSS: connected using serial signs-of-life");
                return true;
            }
        }
        return false;
    };

    auto scanAndConnect = [&](uint32_t &found_baud) -> bool
    {
        if (scanAndConnectPins(GNSS_RX, GNSS_TX, found_baud))
        {
            return true;
        }

        if ((GNSS_RX != GNSS_TX) && scanAndConnectPins(GNSS_TX, GNSS_RX, found_baud))
        {
            Serial.println("GNSS: detected swapped RX/TX wiring; using swapped pin assignment");
            return true;
        }
        return false;
    };

    // Bootstrap from preferred UART rate first, then fall back as needed.
    Serial.print("GNSS: bootstrap try ");
    Serial.print(bootstrap_baud);
    Serial.println(" baud");
    Serial1.end();
    delay(20);
    Serial1.begin(bootstrap_baud, SERIAL_8N1, GNSS_RX, GNSS_TX);
    delay(80);
    if (gnss.begin(Serial1, 800))
    {
        connected_baud = preferred_baud;
        active_rx = GNSS_RX;
        active_tx = GNSS_TX;
    }

    // Keep trying until we can talk UBX at some baud.
    uint8_t scan_attempt = 0;
    while ((connected_baud == 0U) && !scanAndConnect(connected_baud))
    {
        scan_attempt++;
        Serial.println("GNSS: no response on known bauds, retrying...");
        if ((scan_attempt % 2U) == 0U)
        {
            pulseReset();
        }
        delay(700);
    }
    
    Serial.print("GNSS: connected at ");
    Serial.print(connected_baud);
    Serial.println(" baud");
    Serial.print("GNSS: active UART pins RX=");
    Serial.print(active_rx);
    Serial.print(" TX=");
    Serial.println(active_tx);

    // Standardize to preferred baud for runtime.
    // We require preferred_baud for runtime consistency and throughput.
    if (connected_baud != preferred_baud)
    {
        bool switched = false;
        for (uint8_t attempt = 0; attempt < 6; attempt++)
        {
            bool baud_change_requested = false;
            for (uint8_t n = 0; n < 3; n++)
            {
                if (gnss.setSerialRate(preferred_baud))
                {
                    baud_change_requested = true;
                    break;
                }
                delay(60);
            }

            if (!baud_change_requested)
            {
                Serial.println("GNSS: no ACK on baud switch request; forcing probe on preferred baud");
            }

            Serial1.end();
            delay(20);
            Serial1.begin(preferred_baud, SERIAL_8N1, active_rx, active_tx);
            delay(150);

            if (gnss.begin(Serial1, 1500))
            {
                connected_baud = preferred_baud;
                switched = true;
                break;
            }

            // Recover command path on old baud for next attempt.
            Serial1.end();
            delay(20);
            Serial1.begin(connected_baud, SERIAL_8N1, active_rx, active_tx);
            delay(120);
            (void)gnss.begin(Serial1, 1500, true);
            delay(80);
        }

        if (!switched)
        {
            Serial.println("GNSS: failed to force preferred baud; applying factory default and rescanning...");
            (void)gnss.factoryDefault(5000);
            delay(1500);

            while (!scanAndConnect(connected_baud))
            {
                delay(500);
            }

            if (connected_baud != preferred_baud)
            {
                (void)gnss.setSerialRate(preferred_baud);
                Serial1.end();
                delay(20);
                Serial1.begin(preferred_baud, SERIAL_8N1, active_rx, active_tx);
                delay(200);
                if (gnss.begin(Serial1, 1500))
                {
                    connected_baud = preferred_baud;
                }
                else
                {
                    // Hard requirement: runtime must be preferred_baud.
                    Serial.println("GNSS: device not at preferred baud after recovery");
                    return false;
                }
            }
        }

        Serial.print("GNSS: running at ");
        Serial.print(preferred_baud);
        Serial.println(" baud");
    }

    // Runtime always enforced at preferred_baud; probe list includes fallback
    // rates for modules that are not yet configured.

    Serial.println("GNSS serial connected");
    Serial.print("GNSS: verified runtime baud = ");
    Serial.println(connected_baud);

    if (gnss.getModuleInfo())
    {
        Serial.print(F("The GNSS module is: "));
        Serial.println(gnss.getModuleName());    

        Serial.print(F("The firmware type is: "));
        Serial.println(gnss.getFirmwareType());    

        Serial.print(F("The firmware version is: "));
        Serial.print(gnss.getFirmwareVersionHigh());
        Serial.print(F("."));
        Serial.println(gnss.getFirmwareVersionLow());
        
        Serial.print(F("The protocol version is: "));
        Serial.print(gnss.getProtocolVersionHigh());
        Serial.print(F("."));
        Serial.println(gnss.getProtocolVersionLow());
    }

    auto configureReceiver = [&]() -> bool
    {
        bool ok = false;
        uint8_t i = 0;

        // Accept both UBX and NMEA on input (for bring-up compatibility).
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setUART1Input((uint8_t)(COM_TYPE_UBX | COM_TYPE_NMEA))) { ok = true; break; }
            Serial.println("GNSS: failed to set UART1 input protocol mask");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: UART1 input protocol mask set");

        // UBX-only output: NMEA sentences (GGA, RMC, GSV × 4 constellations)
        // add ~1-2 KB/epoch of serial data that the SparkFun library must parse
        // byte-by-byte, blocking the sensor polling task for ~10 ms per GNSS
        // poll and causing ISM6/BMP/MMC data gaps.  We only need UBX autoPVT.
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setUART1Output(COM_TYPE_UBX)) { ok = true; break; }
            Serial.println("GNSS: failed to set UART1 output protocol mask");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: UART1 output protocol mask set (UBX only)");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            Serial.println("GNSS: failed to enable GPS constellation");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: enabled GPS");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            Serial.println("GNSS: failed to enable Galileo constellation");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: enabled Galileo");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            Serial.println("GNSS: failed to enable Glonass constellation");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: enabled Glonass");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU, VAL_LAYER_RAM_BBR)) { ok = true; break; }
            Serial.println("GNSS: failed to enable Beidou constellation");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: enabled Beidou");

        // Power mode: ensure Full Power (continuous / high performance).
        // CFG-PM-OPERATEMODE: 0 = Full Power, 1 = PSMOO, 2 = PSMCT
        {
            uint8_t cur_mode = 0xFF;
            if (gnss.getVal8(UBLOX_CFG_PM_OPERATEMODE, &cur_mode))
            {
                Serial.printf("GNSS: current power mode = %u\n", cur_mode);
                if (cur_mode != 0)
                {
                    Serial.println("GNSS: switching to Full Power (high performance) mode");
                    ok = false;
                    for (i = 0; i < 8; i++)
                    {
                        if (gnss.setVal8(UBLOX_CFG_PM_OPERATEMODE, 0, VAL_LAYER_RAM_BBR)) { ok = true; break; }
                        Serial.println("GNSS: failed to set Full Power mode");
                        delay(150);
                    }
                    if (!ok) return false;
                    Serial.println("GNSS: Full Power mode set");
                }
                else
                {
                    Serial.println("GNSS: already in Full Power mode");
                }
            }
            else
            {
                // Cannot read → force-write Full Power as a safe default
                Serial.println("GNSS: cannot read power mode, forcing Full Power");
                (void)gnss.setVal8(UBLOX_CFG_PM_OPERATEMODE, 0, VAL_LAYER_RAM_BBR);
            }
        }

        // Dynamic model: Airborne <4g (6)
        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setVal8(UBLOX_CFG_NAVSPG_DYNMODEL, 6)) { ok = true; break; }
            Serial.println("GNSS: failed to set dynamic model");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: dynamic model set to Airborne <4g");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setNavigationFrequency(update_rate_hz)) { ok = true; break; }
            Serial.println("GNSS: failed to set navigation update rate");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: navigation update rate set");

        ok = false;
        for (i = 0; i < 8; i++)
        {
            if (gnss.setAutoPVT(true)) { ok = true; break; }
            Serial.println("GNSS: failed to enable auto PVT");
            delay(150);
        }
        if (!ok) return false;
        Serial.println("GNSS: auto PVT enabled");

        // Save settings to BBR/flash where available.
        (void)gnss.saveConfiguration();
        return true;
    };

    // New modules can be in odd config states. If configuration repeatedly fails,
    // perform a factory default and try one more full configuration pass.
    if (!configureReceiver())
    {
        Serial.println("GNSS: config failed, applying factory default and retrying...");
        (void)gnss.factoryDefault(5000);
        delay(1500);

        while (!scanAndConnect(connected_baud))
        {
            delay(500);
        }

        if (connected_baud != preferred_baud)
        {
            (void)gnss.setSerialRate(preferred_baud);
            Serial1.end();
            delay(20);
            Serial1.begin(preferred_baud, SERIAL_8N1, active_rx, active_tx);
            delay(150);
            (void)gnss.begin(Serial1, 1500);
        }

        if (!configureReceiver())
        {
            Serial.println("GNSS: configuration failed after factory default");
            return false;
        }
    }
    
    Serial.println("GNSS configuration complete.");
    delay(100);
        
    return true;
}

void TR_GNSSReceiverUBloxSerial::getGNSSData(GNSSData &gnss_data)
{
    gnss_data.time_us = micros();
    gnss_data.year = gnss.getYear();
    gnss_data.month = gnss.getMonth();
    gnss_data.day = gnss.getDay();
    gnss_data.hour = gnss.getHour();
    gnss_data.minute = gnss.getMinute();
    gnss_data.second = gnss.getSecond();
    gnss_data.milli_second = gnss.getMillisecond();
    // 0: No Fix, 1: Dead Reckoning, 2: 2D Fix, 3: 3D Fix,
    // 4:GNSS + Dead Reckoning, 5: Time Only
    gnss_data.fix_mode = gnss.getFixType();
    gnss_data.num_sats = gnss.getSIV();

    // SparkFun u-blox returns PDOP as scale 0.01. Convert to x10 for packed type.
    const uint16_t pdop_x100 = gnss.getPDOP();
    uint16_t pdop_x10_u16 = (uint16_t)((pdop_x100 + 5U) / 10U);
    if (pdop_x10_u16 > 255U) pdop_x10_u16 = 255U;
    gnss_data.pdop_x10 = (uint8_t)pdop_x10_u16;

    // Accuracy estimates are reported in mm. Convert to whole meters.
    const uint32_t h_acc_mm = gnss.getHorizontalAccEst();
    const uint32_t v_acc_mm = gnss.getVerticalAccEst();
    uint32_t h_acc_m_u32 = (h_acc_mm + 500U) / 1000U;
    uint32_t v_acc_m_u32 = (v_acc_mm + 500U) / 1000U;
    if (h_acc_m_u32 > 255U) h_acc_m_u32 = 255U;
    if (v_acc_m_u32 > 255U) v_acc_m_u32 = 255U;
    gnss_data.h_acc_m = (uint8_t)h_acc_m_u32;
    gnss_data.v_acc_m = (uint8_t)v_acc_m_u32;

    // Velocity (ENU, mm/s)
    gnss_data.vel_e_mmps = gnss.getNedEastVel();
    gnss_data.vel_n_mmps = gnss.getNedNorthVel();
    gnss_data.vel_u_mmps = -gnss.getNedDownVel();

    // Latitude and Longitude (deg*1e7)
    gnss_data.lat_e7 = gnss.getLatitude();
    gnss_data.lon_e7 = gnss.getLongitude();

    // Altitude relative to mean sea level (mm)
    gnss_data.alt_mm = gnss.getAltitudeMSL();

    // --- Staleness detection ---
    // If the GNSS second+millisecond are unchanged across consecutive reads,
    // the SparkFun library is returning cached data (serial buffer overflow
    // caused the UBX parser to lose frame sync).  After STALE_THRESHOLD
    // consecutive unchanged readings, zero fix_mode so downstream consumers
    // know the position is unreliable.
    if (gnss_data.second == prev_second &&
        gnss_data.milli_second == prev_milli_second)
    {
        if (stale_count < UINT16_MAX) stale_count++;

        if (stale_count == STALE_THRESHOLD)
        {
            Serial.println("[GNSS] WARNING: data stale — no new PVT for "
                           "5 consecutive reads, zeroing fix_mode");
        }
    }
    else
    {
        if (stale_count >= STALE_THRESHOLD)
        {
            Serial.printf("[GNSS] data resumed after %u stale reads\n",
                          (unsigned)stale_count);
        }
        stale_count = 0;
    }

    prev_second = gnss_data.second;
    prev_milli_second = gnss_data.milli_second;

    if (stale_count >= STALE_THRESHOLD)
    {
        gnss_data.fix_mode = 0;  // signal "no fix" to consumers
    }
}
