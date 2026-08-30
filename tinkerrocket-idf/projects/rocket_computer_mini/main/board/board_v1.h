#pragma once

#include <stdint.h>

// rev1 PCB pin map + board topology for the rocket-computer-mini.
// Selected with TR_MINI_BOARD=1 (the default):  idf.py build
//
// ONLY pins and part-presence flags live here — the board_pins contract.
// Logic, rates, gains, and protocol constants stay in config.h and must not
// fork per board revision.
//
// Source of truth: kicad-cli netlist export of
// hardware/rocket-computer-mini/rocket-computer-mini.kicad_sch (every pin
// below verified against the U15 pad list — note that pin-budget.md's
// "Assignment — as built" table predates several moves and disagrees with
// the netlist; the netlist wins). Net names in comments are the schematic
// nets. This is one processor doing both of the big board's jobs, so the map
// carries flight_computer's sensor/pyro groups AND out_computer's
// memory/radio/power groups.
struct board_pins
{
    // --- Peripheral rail switch ---
    // Net POWER_SWITCH -> TPS22810 (U30) EN/UVLO, R84 pulldown so the rail
    // idles OFF through reset. High = V_MCU_SWTCH up. Unlike the big board,
    // this rail is not a second processor: it powers the IMU, baro, mag,
    // GNSS (VCC *and* V_BCKP), the LoRa module, and the NAND log flash.
    // Nothing but the INA230 and the boot flash works until this is high —
    // and cycling it cold-starts the GNSS (no battery-backed almanac).
    static constexpr int PWR_PIN = 3;            // POWER_SWITCH (CONFIRMED)

    // --- Power monitoring (INA230 @ 0x40) ---
    // The one always-on peripheral. On the mini the magnetometer shares this
    // bus (pin-budget.md: "no new pins"), so PWR_SDA/PWR_SCL and the mag's
    // I2C pins are the same physical bus — one I2C port, two devices.
    static constexpr int PWR_SDA = 21;           // SEN_SDA (CONFIRMED)
    static constexpr int PWR_SCL = 33;           // SEN_SCL (CONFIRMED)

    // --- Sensor SPI bus (latency-critical, keeps its own host) ---
    // Net names are from the sensors' perspective: SENS_SDI is data INTO the
    // sensors (our MOSI), SENS_SDO is data out of them (our MISO).
    static constexpr int SENS_SPI_SCK = 34;      // SENS_SCLK (CONFIRMED)
    static constexpr int SENS_SPI_MOSI = 1;      // SENS_SDI  (CONFIRMED)
    static constexpr int SENS_SPI_MISO = 2;      // SENS_SDO  (CONFIRMED)

    static constexpr int ISM6HG256_CS = 9;       // ISM6HG256_CS   (CONFIRMED)
    // GPIO47 IS SPICLK_P AND GPIO48 IS SPICLK_N (ESP-IDF io_mux_reg.h:
    // IO_MUX_GPIO47_REG = PERIPHS_IO_MUX_SPICLK_P_U). These two were reversed
    // until 2026-08-30, following pin-budget.md's "GPIO47, GPIO48 = SPICLK_N /
    // SPICLK_P" pairing. The schematic puts BMP585_CS on pad 36 (SPICLK_N) and
    // ISM6HG256_INT1 on pad 37 (SPICLK_P) both before and after the two-MCU
    // split, so the correct numbers are 48 and 47.
    static constexpr int ISM6HG256_INT = 47;     // ISM6HG256_INT1 (pad 37, SPICLK_P)
    static constexpr int BMP585_CS = 48;         // BMP585_CS      (pad 36, SPICLK_N)
    static constexpr int BMP585_INT = 41;        // BMP585_INT     (CONFIRMED)

    // --- Magnetometer (IIS2MDC @ 0x1E, on the PWR_SDA/PWR_SCL bus) ---
    // Its INT/DRDY pin is unconnected on this board — poll only.
    static constexpr int IIS2MDC_INT = -1;       // not wired (CONFIRMED)

    // --- GNSS (Quectel LC86G, UART only) ---
    // Net names are from the module's perspective: GNSS_TX is the module's
    // TXD (we receive on it). No PPS to the MCU (1PPS drives LED D7), no
    // reset line (RESET_N unconnected) — a wedged receiver is recovered only
    // by cycling PWR_PIN, which takes the radio, sensors and NAND with it.
    static constexpr int GNSS_RX = 39;           // GNSS_TX, module->us (CONFIRMED)
    static constexpr int GNSS_TX = 40;           // GNSS_RX, us->module (CONFIRMED)

    // --- Memory SPI bus (long-transaction, shared with the radio) ---
    static constexpr int MEM_SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int MEM_SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int MEM_SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;           // M_FLASH_CS (CONFIRMED)
    // No MRAM on this board — TR_LogToFlash falls back to its RAM ring, and
    // the MRAM crash snapshot does not survive a reboot here.
    static constexpr int MRAM_CS = -1;           // not fitted (CONFIRMED)

    // --- Radio: bare E220-900MM22S (LLCC68) on the memory bus ---
    // Direct-SPI topology (the V7 out-computer path): the module keeps only
    // its own chip select; SCK/MISO/MOSI are the MEM_SPI_* pins above.
    // DIO2->TXEN is shorted on the module (net L_DI02) so TX switching is the
    // radio's own job; only RXEN reaches a GPIO and it MUST be driven — the
    // base station has already been bitten by an RXEN left floating in RX.
    // DIO3 floats: never configure a DIO3-powered TCXO (the module carries a
    // passive 32 MHz crystal).
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK = MEM_SPI_SCK;    // shared bus
    static constexpr int LORA_SPI_MISO = MEM_SPI_MISO;  // shared bus
    static constexpr int LORA_SPI_MOSI = MEM_SPI_MOSI;  // shared bus
    static constexpr int LORA_CS_PIN = 13;       // L_CS   (CONFIRMED)
    static constexpr int LORA_BUSY_PIN = 14;     // L_BUSY (CONFIRMED)
    static constexpr int LORA_DIO1_PIN = 17;     // L_DI01 (CONFIRMED)
    static constexpr int LORA_RST_PIN = 18;      // L_RST  (CONFIRMED)
    static constexpr int LORA_RXEN_PIN = 44;     // L_RXEN (CONFIRMED; U0RXD spent)

    // --- Pyro (4 channels, shared low-side arm FET on the return) ---
    // Safety property from the S3 datasheet Table 2-2, verified in
    // pin-budget.md: every FIRE pin and ARM sit in the GPIO1-14 group, which
    // glitches LOW ONLY (~60 us) at power-up — the chip cannot drive them
    // high before firmware takes control.
    // FIRE is active high (drives a BJT that pulls the high-side P-FET gate
    // down); ARM is active high (low-side N-FET U9 closing the shared pyro
    // return). Continuity: R8-class pull-ups tie each channel to V_MCU_SWTCH,
    // an attached igniter pulls the node toward PYRO_GND through R73 — so
    // CONT reads LOW when an igniter is present, and reads only while
    // PWR_PIN's rail is up.
    static constexpr int PYRO_ARM_PIN = 8;       // PYRO_ARM (CONFIRMED)
    static constexpr int PYRO1_FIRE_PIN = 4;     // PYRO1_FIRE (CONFIRMED)
    static constexpr int PYRO2_FIRE_PIN = 5;     // PYRO2_FIRE (CONFIRMED)
    static constexpr int PYRO3_FIRE_PIN = 6;     // PYRO3_FIRE (CONFIRMED)
    static constexpr int PYRO4_FIRE_PIN = 7;     // PYRO4_FIRE (CONFIRMED)
    static constexpr int PYRO1_CONT_PIN = 10;    // PYRO1_CONT (CONFIRMED)
    static constexpr int PYRO2_CONT_PIN = 11;    // PYRO2_CONT (CONFIRMED)
    static constexpr int PYRO3_CONT_PIN = 12;    // PYRO3_CONT (CONFIRMED)
    static constexpr int PYRO4_CONT_PIN = 42;    // PYRO4_CONT (CONFIRMED; JTAG spent)

    // --- UI ---
    // GPIO45 is a strapping pin (VDD_SPI voltage select) with a weak
    // pull-down at reset — safe as an LED output precisely because the LED
    // load never pulls it high. Drive only after boot.
    static constexpr int LED_PIN = 45;           // -> D8 (CONFIRMED)
    static constexpr int BOOT_BTN_PIN = 0;       // SW3 (CONFIRMED)

    // --- Part presence ---
    static constexpr bool USE_ISM6HG256 = true;
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_IIS2MDC = true;
    static constexpr bool USE_MMC5983MA = false; // not fitted on the mini
    static constexpr bool USE_GNSS = true;

    // Gone with the hardware, kept as sentinels so ported call sites guard
    // cleanly: no servos, no camera, no piezo on this board.
    static constexpr int SERVO_PIN_1 = -1;
    static constexpr int SERVO_PIN_2 = -1;
    static constexpr int SERVO_PIN_3 = -1;
    static constexpr int SERVO_PIN_4 = -1;
    static constexpr int RUNCAM_RX_PIN = -1;
    static constexpr int RUNCAM_TX_PIN = -1;
    static constexpr int RUNCAM_PWR_PIN = -1;
    static constexpr int PIEZO_PIN = -1;
};
