#pragma once

// Tinker-Base V2 pin map + board topology (hardware/tinker-base, silkscreen V2).
// Selected with TR_BS_BOARD=5:  idf.py -B build_v5 -DTR_BS_BOARD=5 build
//
// The build number continues the base-station sequence, as board_v4.h explains:
// the Tinker-Base's first revision builds as board 4 ("-v4"), so its second
// builds as board 5 and stamps "-v5". The suffix keeps the two apart over OTA,
// which matters more here than usual: a V4 image declares a 16 MB boot NOR and
// this board has 4 MB.
//
// Netlist-verified 2026-11-26 against hardware/tinker-base (kicad-cli netlist,
// U3 pad -> pinfunction -> net, diffed pin by pin against the V1 tag
// tinker-base-v1.0.0). What V2 changed on U3's pins, and nothing else:
//  - U3 is an ESP32-S3FH4R2: 4 MB of flash and 2 MB of PSRAM in the package.
//    V1's WLCSP boot NOR (U1) is gone, and pads 30-35 (SPIHD/WP/CS0/CLK/Q/D)
//    are now no-connects. The CMakeLists selects sdkconfig.defaults.flash4mb
//    and partitions_4mb.csv for this board; main.cpp refuses to compile a
//    board-5 image that declares more than 4 MB.
//  - U23, a GD25Q128ESIGR 16 MB NOR log flash, on GPIO4-7 (pads V1 left
//    unconnected). Logs go there; see Storage below.
//  - An I2C bus: I2C_SCL on GPIO10 (pad 15), I2C_SDA on GPIO12 (pad 17), with
//    5.11 k pull-ups to +3V3 (R63, R64). Its only device is U24, an INA230 on
//    the cell (see below).
// Unchanged from V1, pad for pad: every radio net, Volt_Read on GPIO1, USB on
// GPIO19/20, the crystals, GPIO0/GPIO46 straps.
//
// NOT YET RUN ON HARDWARE: no V2 board has been ordered or built. Every
// constant below is a netlist fact; none is bench-proven.
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V5 Tinker-Base V2 (on-board E220 LoRa, INA230 on the cell)";

    // --- LoRa radio: on-board E220-900MM22S (U16) over SPI — unchanged from V4 ---
    // The E220's RF switch is split: TXEN is looped to the radio's own DIO2 on
    // the module side (L_DI02, no MCU pin), which TR_LoRa_Comms drives through
    // setDio2AsRfSwitch(true); RXEN is the one the MCU drives. DIO3 is left
    // unconnected on purpose — the module carries its own 32 MHz crystal, so
    // it must not be configured for a DIO3-powered TCXO.
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK  = 17;   // L_SCK   -> U16 SCK
    static constexpr int LORA_SPI_MISO = 33;   // L_MISO  <- U16 MISO
    static constexpr int LORA_SPI_MOSI = 21;   // L_MOSI  -> U16 MOSI
    static constexpr int LORA_CS_PIN   = 18;   // L_CS    -> U16 NSS
    static constexpr int LORA_DIO1_PIN = 2;    // L_DI01  <- U16 DIO1
    static constexpr int LORA_RST_PIN  = 38;   // L_RST   -> U16 NRST
    static constexpr int LORA_BUSY_PIN = 34;   // L_BUSY  <- U16 BUSY
    static constexpr int LORA_RXEN_PIN = 35;   // L_RXEN  -> U16 RXEN
    static constexpr int LORA_DIO2_PIN = -1;   // module-local loop to TXEN
    static constexpr int LORA_DIO3_PIN = -1;   // unconnected (own crystal)

    // --- Radio daughterboard host link: not on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN     = -1;

    // --- Storage: U23, a 16 MB SPI NOR log flash ---
    // U23 (GD25Q128ESIGR, JEDEC C8 40 18) sits on GPIO4-7, the same M_* nets
    // and pins as the V2 base station's NAND (legacy/board_v2.h): M_SCK GPIO4,
    // M_MOSI GPIO5, M_FLASH_CS GPIO6, M_MISO GPIO7, on SPI3_HOST (the radio
    // has SPI2_HOST). R58-R60 pull /CS, /WP and /HOLD up to +3V3, so the bus
    // is 1-bit SPI. It is a NOR, not a NAND: main.cpp mounts it through
    // esp_flash + wear levelling + FAT at /extflash (mountExternalNorFat()),
    // never through the NAND driver, which would probe it and fail.
    //
    // If it does not come up, logs fall back to spiffs in the package's 4 MB
    // (partitions_4mb.csv: 1.94 MB after two 1 MB app slots), and that is a
    // demotion: the storage frame says so (BSS_FLAG_FALLBACK).
    static constexpr bool HAS_SDMMC    = false;
    static constexpr bool HAS_EXT_NAND = false;
    static constexpr bool HAS_EXT_NOR  = true;
    static constexpr int FLASH_SCK  = 4;   // M_SCK      -> U23 CLK (pin 6)
    static constexpr int FLASH_MOSI = 5;   // M_MOSI     -> U23 DI  (pin 5)
    static constexpr int FLASH_CS   = 6;   // M_FLASH_CS -> U23 /CS (pin 1)
    static constexpr int FLASH_MISO = 7;   // M_MISO     <- U23 DO  (pin 2)
    static constexpr int SD_CLK = -1;      // no SD slot
    static constexpr int SD_CMD = -1;
    static constexpr int SD_D0  = -1;
    static constexpr int SD_D1  = -1;
    static constexpr int SD_D2  = -1;
    static constexpr int SD_D3  = -1;

    // --- I2C: new on V2, for the cell-current monitor only ---
    // External 5.11 k pull-ups (R63 SCL, R64 SDA). No fuel gauge (DW01A +
    // FS8205A still protect the cell) and no pack charger, so main.cpp probes
    // only the INA230 on this bus.
    static constexpr int I2C_SCL_PIN = 10;   // I2C_SCL, U3 pad 15
    static constexpr int I2C_SDA_PIN = 12;   // I2C_SDA, U3 pad 17
    static constexpr bool EXPECT_MAX17303 = false;
    static constexpr bool HAS_FUEL_GAUGE  = false;

    // --- Cell-current monitor: U24 INA230 (battery.kicad_sch, "1S Battery Current") ---
    // Wired as the Tinker-Beetle V2's U23. A0 = A1 = GND, so 0x40; ALERT is
    // open. R62, a 20 mOhm 0805 shunt (PA0805FRE470R02L; 2 mOhm, the Beetle's
    // part, until 2026-11-26), sits high-side between the 18650's +
    // terminal (net B+, IN+) and VCC (IN- and BUS). VCC is where U5 BQ21040's
    // output and U22's load-switch input meet, so the shunt carries the cell's
    // NET current: discharge flows IN+ -> IN- (the INA230 reads it positive),
    // charge flows IN- -> IN+ (negative). VS is +3V3, so the part only has
    // power while U22 is on, which is whenever this firmware runs.
    // The address, shunt and current LSB are in config.h with the gauges'.
    static constexpr bool HAS_CELL_INA230 = true;

    // --- Own-battery voltage sense (net Volt_Read) — unchanged from V4 ---
    // Now the FALLBACK: used when the INA230 does not answer. BT2, a single
    // 18650 charged by U5 BQ21040 (1S linear), so cells = 1.
    // V_SWITCH -> R44 1M -> Volt_Read -> R46 1M -> GND, C30 100 nF at the pin.
    // 12 dB for the same reason as V3: a full 4.2 V cell divides to 2.1 V,
    // outside 6 dB's ~1.75 V calibrated range.
    static constexpr int   BATT_VSENSE_GPIO     = 1;    // Volt_Read, ADC1_CH0
    static constexpr float BATT_VSENSE_DIVIDER  = 2.0f; // R44 1M / R46 1M
    static constexpr int   BATT_VSENSE_ATTEN_DB = 12;   // ADC_ATTEN_DB_12

    // --- Flight-pack charger and its dividers: not on this board ---
    static constexpr bool HAS_PACK_CHARGER = false;
    static constexpr int   PACK_VSENSE_GPIO         = -1;
    static constexpr float PACK_VSENSE_DIVIDER      = 0.0f;
    static constexpr int   PACK_VSENSE_ATTEN_DB     = -1;
    static constexpr int   PACK_MID_VSENSE_GPIO     = -1;
    static constexpr float PACK_MID_VSENSE_DIVIDER  = 0.0f;
    static constexpr int   PACK_MID_VSENSE_ATTEN_DB = -1;
};
