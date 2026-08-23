#pragma once

// V3 base-station PCB pin map + board topology.
// Selected with TR_BS_BOARD=3:  idf.py -B build_v3 -DTR_BS_BOARD=3 build
//
// What changed vs V2:
//  - Fuel gauge: MAX17303 (1S, ModelGauge m5 + protector) replaces the
//    BQ27Z746. Runtime-probed at 0x36 and disambiguated from the V1
//    MAX17205 (same address) via DevName.
//  - NEW: MP2672 2-cell charger for external rocket flight packs (same I2C
//    bus as the gauge).
//  - I2C moves to SDA=33 / SCL=34 (33/34 were LoRa RST / unused on V2).
//  - Radio: no on-board LLCC68 — the V8 radio daughterboard (#409,
//    projects/radio_board) connects over UART as a transparent modem,
//    same topology as the V8 out computer (#410).
//  - Storage: unchanged from V2 — FORESEE F35SQB004G SPI NAND on the same
//    M_* nets/pins.
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V3 (MAX17303 + MP2672 + radio daughterboard)";

    // --- LoRa radio: UART daughterboard (#409) — no direct LLCC68 ---
    static constexpr bool USE_UART_RADIO_MODEM = true;
    static constexpr int LORA_SPI_SCK  = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN   = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN  = -1;
    static constexpr int LORA_BUSY_PIN = -1;
    static constexpr int LORA_RXEN_PIN = -1;
    static constexpr int LORA_DIO2_PIN = -1;
    static constexpr int LORA_DIO3_PIN = -1;

    // --- Radio daughterboard host link ---
    // CONFIRMED against both PCBs' pad->net maps, not the labels alone: our
    // J6.4 (net LoRa_TX, GPIO35) lands on the daughterboard's J6.4 (its net
    // LoRa_RX, its GPIO5), and our J6.3 (net LoRa_RX, GPIO36) lands on its
    // J6.3 (its net LoRa_TX, its GPIO6). The two boards name the nets from
    // their own perspective, so the same pair of names appears on both ends
    // of a crossed link — the cable pin number is the only unambiguous
    // reference. (This is the trap that #234/RunCam hit; it is resolved here,
    // so do NOT "fix" it by swapping these two on bring-up.)
    static constexpr int LORA_UART_TX_PIN = 35;  // LoRa_TX -> J6.4 (CONFIRMED)
    static constexpr int LORA_UART_RX_PIN = 36;  // LoRa_RX <- J6.3 (CONFIRMED)
    // Daughterboard power gate (#835 item 4). It DOES exist and the old
    // comment here denied it: LoRa_EN = GPIO21 -> R18 1k -> U2 TPS61023 EN,
    // with R17 100k pulling EN to +3V3, so the rail is ON at reset (GPIO21 is
    // high-Z then, and is not an S3 strapping pin). Driving it low puts EN at
    // 3.3 * 1k/101k ~= 33 mV. V_LORA feeds J6.2 to the daughterboard.
    //
    // CAVEAT, unmeasured: the TPS61023 is a synchronous boost with NO output
    // disconnect, so with EN low VIN still reaches VOUT through the high-side
    // body diode. V_LORA may sit at roughly V_SWITCH - Vf rather than 0, and
    // the daughterboard's own CR3 + TPS62913 chain could keep it alive. So
    // treat this as "the gate is wired" — do NOT assume it is the #412
    // recovery hammer until someone measures V_LORA and the daughterboard 3V3
    // with GPIO21 low, on both a full and a half-discharged cell.
    static constexpr int LORA_ACT_PIN = 21;

    // --- Storage: NO external NAND and no SD slot (#835 item 1) ---
    // This header claimed a FORESEE F35SQB004G "same as V2". There is no such
    // part on this board and never was: the only flash is U1, the boot NOR on
    // the S3's DEDICATED SPI0 pins (28-35 + VDD_SPI), and GPIO4/5/6/7 are
    // unconnected pads in the netlist. The false claim made
    // bs_storage_policy::demoted() report a demotion on every boot and log a
    // "**** STORAGE DEMOTED ****" error for hardware that was working as
    // designed. Logging goes to the spiffs partition on U1.
    static constexpr bool HAS_SDMMC    = false;
    static constexpr bool HAS_EXT_NAND = false;
    static constexpr int FLASH_SCK  = -1;
    static constexpr int FLASH_MOSI = -1;
    static constexpr int FLASH_CS   = -1;
    static constexpr int FLASH_MISO = -1;
    static constexpr int SD_CLK = -1;      // no SD slot
    static constexpr int SD_CMD = -1;
    static constexpr int SD_D0  = -1;
    static constexpr int SD_D1  = -1;
    static constexpr int SD_D2  = -1;
    static constexpr int SD_D3  = -1;

    // --- I2C bus (MP2672 pack charger; NO fuel gauge) ---
    static constexpr int I2C_SCL_PIN = 34;
    static constexpr int I2C_SDA_PIN = 33;
    // #835 item 2: there is no fuel gauge on this board. The MAX17303 was
    // DELETED in 15da738 (2026-07-26), two weeks before the fab tag, and
    // replaced by a DW01A + FS8205A protection pair. That commit's own message
    // says state of charge "now reads a 1M/1M divider on GPIO1" — the firmware
    // half of that sentence was never written, so bs_voltage/bs_soc stayed NaN
    // forever behind `if (!fuel_gauge_present) return;`.
    static constexpr bool EXPECT_MAX17303 = false;
    static constexpr bool HAS_FUEL_GAUGE  = false;

    // --- Own-battery voltage sense (net Volt_Read) ---
    // BT2 is a SINGLE 18650 (BH-18650-A5BJ001-2D) charged by U5 BQ21040, a 1S
    // linear charger — so cells = 1 and bs_battery_soc's per-cell LiCoO2 curve
    // applies directly. V_SWITCH -> R44 1M -> Volt_Read -> R46 1M -> GND, with
    // C30 100 nF at the pin as the sampling reservoir.
    //
    // 12 dB attenuation, not the 6 dB used for the pack sense: a full 4.2 V
    // cell divides to 2.1 V, which is outside the ~1.75 V calibrated range at
    // 6 dB. The esp-idf calibration curve is PER-ATTENUATION — the adc_cali
    // handle must be created for this exact setting or every read is silently
    // mis-scaled.
    static constexpr int   BATT_VSENSE_GPIO     = 1;    // Volt_Read, ADC1_CH0
    static constexpr float BATT_VSENSE_DIVIDER  = 2.0f; // R44 1M / R46 1M
    static constexpr int   BATT_VSENSE_ATTEN_DB = 12;   // ADC_ATTEN_DB_12

    // --- Flight-pack charger (MP2672GD-0000-Z, external 2S packs) ---
    static constexpr bool HAS_PACK_CHARGER = true;

    // --- Flight-pack voltage sense (net PosADC, external_charger sheet) ---
    // BatteryPos -> R40 -> PosADC -> R42 -> GND, with C55 100 nF at the pin
    // as the sampling reservoir; read on GPIO8 (ADC1_CH7). Hardware PR #728
    // (2026-08-09) re-ratioed the divider onto fleet resistor values:
    // R40 300 k -> 1 M, R42 100 k -> 180 k. A full 2S pack (8.4 V) now puts
    // 1.281 V at the pin (was 2.1 V); divider drain drops 21 uA -> 7 uA.
    // Nothing reads this yet — constants staged so the first implementation
    // cannot inherit the pre-#728 scale of 4.000.
    static constexpr int   PACK_VSENSE_GPIO       = 8;    // PosADC, ADC1_CH7
    static constexpr float PACK_VSENSE_DIVIDER    = 1180.0f / 180.0f;  // 6.5556 (pre-#728: 4.000)
    // Attenuation chosen deliberately with the new scale: 6 dB. Its ~1.75 V
    // calibrated range covers the 1.281 V full-pack reading with 27% headroom
    // and roughly halves the LSB size vs 12 dB. The esp-idf cal curve is
    // per-attenuation — create the adc_cali handle FOR THIS setting; changing
    // one without the other silently mis-scales every read.
    static constexpr int   PACK_VSENSE_ATTEN_DB   = 6;    // ADC_ATTEN_DB_6
};
