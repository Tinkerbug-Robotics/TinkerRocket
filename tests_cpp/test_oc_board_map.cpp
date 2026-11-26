// The out computer's board map, as config.h ships it for one board flag.
//
// Built once per board (tests_cpp/CMakeLists.txt). Two things are checked for
// every board, because the Tinker-Beetle V2 target (TR_BOARD_M2) added them to
// code that all boards compile:
//   * TR_OC_ANTENNA_OFFSET, which replaced ten per-board checks in main.cpp:
//     the antenna offset is served everywhere but on the Beetle.
//   * config::LINK_NET_ON_MTCK, which decides whether setup_oc() turns off the
//     pull-up the chip puts on GPIO39 at reset. Only the V2 has a link net
//     there, so the block must compile to nothing on every other board.
// The M2 build then pins the V2 map itself: the seven signals the 2026-11-21
// layout moved, everything that stayed, and the rules a moved pin must keep.
#include <gtest/gtest.h>
#include <SPI.h>       // host shim: SPI_MODE0, which the OC config.h names
#include "config.h"    // the SHIPPED pins for this board (TR_BOARD_* from CMake)
#include <BoardIdentity.h>
#include "pyro_fault_trip_policy.h"   // the INA230 Mask/Enable words this board writes
#include <cstdio>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace
{
// Every signal this processor has on a pad, by the net it carries.
std::vector<std::pair<std::string, int>> wiredPins()
{
    std::vector<std::pair<std::string, int>> all = {
        {"FC_EN_OC / P4_EN_S3", config::PWR_PIN},
        {"GPS power",           config::GPS_PWR_PIN},
        {"SEN_SDA",             config::PWR_SDA},
        {"SEN_SCL",             config::PWR_SCL},
        {"M_SCK",               config::SPI_SCK},
        {"M_MISO",              config::SPI_MISO},
        {"M_MOSI",              config::SPI_MOSI},
        {"M_FLASH_CS",          config::NAND_CS},
        {"MRAM_CS",             config::MRAM_CS},
        {"OC_ARM_EN",           config::ARM_CONSENT_PIN},
        {"INA_ALERT",           config::INA_ALERT_PIN},
        {"PG_RAIL",             config::PG_RAIL_PIN},
        {"VBUCK_OK",            config::VBUCK_OK_PIN},
        {"V_SCAP_ADC",          config::SCAP_ADC_PIN},
        {"ESP_SDA",             config::I2C_SDA_PIN},
        {"ESP_SCL",             config::I2C_SCL_PIN},
        {"ESP_I2S_BCLK",        config::I2S_BCLK_PIN},
        {"ESP_I2S_WS",          config::I2S_WS_PIN},
        {"ESP_I2S_SD",          config::I2S_DIN_PIN},
        {"ESP_I2S_FSYNC",       config::I2S_FSYNC_PIN},
        {"L_CS",                config::LORA_CS_PIN},
        {"L_BUSY",              config::LORA_BUSY_PIN},
        {"L_DI01",              config::LORA_DIO1_PIN},
        {"L_RST",               config::LORA_RST_PIN},
        {"LoRa UART TX",        config::LORA_UART_TX_PIN},
        {"LoRa UART RX",        config::LORA_UART_RX_PIN},
        {"LoRa_ACT",            config::LORA_ACT_PIN},
        {"camera IMON",         config::CAM_IMON_GPIO},
        {"servo IMON",          config::SERVO_IMON_GPIO},
#if TR_BOARD_M2
        {"L_RXEN",              config::LORA_RXEN_PIN},   // only the Beetle's header names it
#endif
    };
    std::vector<std::pair<std::string, int>> wired;
    for (const auto& p : all)
        if (p.second >= 0) wired.push_back(p);
    return wired;
}
}   // namespace

TEST(OcBoardMap, TheAntennaOffsetIsServedEverywhereButOnTheBeetle)
{
#if TR_BOARD_M2
    EXPECT_EQ(TR_OC_ANTENNA_OFFSET, 0);
#else
    EXPECT_EQ(TR_OC_ANTENNA_OFFSET, 1);
#endif
}

TEST(OcBoardMap, OnlyTheBeetleV2HasALinkNetOnThePadTheChipPullsUp)
{
    EXPECT_EQ(config::S3_MTCK_GPIO, 39);
#if TR_BOARD_M2
    EXPECT_TRUE(config::LINK_NET_ON_MTCK);
#else
    // setup_oc()'s block is `if constexpr` on this: false means no code.
    EXPECT_FALSE(config::LINK_NET_ON_MTCK);
#endif
}

TEST(OcBoardMap, OnlyTheBeetleV3sAlertDrivesTheArmTrip)
{
    // Q15/Q16 pull U9's gate low on INA_ALERT (branch
    // hardware/pyro-pfet-alert-trip). The Mantis V11 has the trip too, and
    // builds with the V10 target until it has one of its own.
#if TR_BOARD_M2 && TR_BEETLE_V3
    EXPECT_TRUE(config::INA_ALERT_DRIVES_ARM_TRIP);
    EXPECT_EQ(config::INA_ALERT_PIN, -1) << "the V3's ALERT reaches the trip, not an S3 pin";
#else
    EXPECT_FALSE(config::INA_ALERT_DRIVES_ARM_TRIP);
#endif
    if (config::INA_ALERT_DRIVES_ARM_TRIP)
        EXPECT_GE(config::ARM_CONSENT_PIN, 0);
}

TEST(OcBoardMap, TheInaWordsForThisBoardKeepCnvrOffATripAlert)
{
    using namespace PyroFaultTrip;
    constexpr bool wired = config::INA_ALERT_PIN >= 0;
    constexpr bool trips = config::INA_ALERT_DRIVES_ARM_TRIP;
    const uint16_t words[] = {
        solMaskEnable(wired, trips),          // pyroFaultGuard(true)
        idleMaskEnable(wired, trips),         // pyroFaultGuard(false)
        bootMaskEnable(0x0000, wired, trips), // setup_oc, register at power-on
        bootMaskEnable(0xFFFF, wired, trips), // setup_oc, anything left in it
    };
    for (uint16_t w : words)
    {
        if (trips)
            EXPECT_EQ(w & kMeCnvr, 0u) << "CNVR on ALERT would hold U9 off: 0x" << std::hex << w;
    }
    // SOL is what the guard arms on every board with a consent stage.
    if (config::ARM_CONSENT_PIN >= 0)
        EXPECT_NE(solMaskEnable(wired, trips) & kMeSol, 0u);
#if TR_BOARD_M2 && !TR_BEETLE_V3
    // The Beetle V2 as ordered: exactly the words it has always written.
    EXPECT_EQ(solMaskEnable(wired, trips), (uint16_t)(kMeSol | kMeCnvr));
    EXPECT_EQ(idleMaskEnable(wired, trips), kMeCnvr);
    EXPECT_EQ(bootMaskEnable(0x0000, wired, trips), kMeCnvr);
#endif
#if TR_BOARD_V10
    EXPECT_EQ(solMaskEnable(wired, trips), kMeSol);
    EXPECT_EQ(idleMaskEnable(wired, trips), kMeCnvr);
#endif
}

TEST(OcBoardMap, NoTwoSignalsShareAPad)
{
    std::set<int> seen;
    for (const auto& p : wiredPins())
        EXPECT_TRUE(seen.insert(p.second).second) << p.first << " shares GPIO" << p.second;
}

TEST(OcBoardMap, NoSignalSitsOnAPadThatIsNotAGpio)
{
    // The S3 has GPIO0-21 and GPIO26-48. 19/20 are the USB pair on every
    // board, and 15/16 the 32 kHz crystal.
    for (const auto& p : wiredPins())
    {
        const int g = p.second;
        EXPECT_TRUE((g >= 0 && g <= 21) || (g >= 26 && g <= 48)) << p.first << " on GPIO" << g;
        EXPECT_FALSE(g == 19 || g == 20) << p.first << " on the USB pair";
        EXPECT_FALSE(g == 15 || g == 16) << p.first << " on the 32 kHz crystal";
    }
}

#if TR_BOARD_M2

TEST(OcBoardM2, TheSevenSignalsTheV2LayoutMovedAreOnTheirNewPads)
{
    EXPECT_EQ(config::PWR_PIN, 48);          // FC_EN_OC       pad 36  (rev1: 7)
    EXPECT_EQ(config::LORA_CS_PIN, 47);      // L_CS           pad 37  (rev1: 13)
    EXPECT_EQ(config::I2S_FSYNC_PIN, 34);    // ESP_I2S_FSYNC  pad 39  (rev1: 4)
    EXPECT_EQ(config::I2S_WS_PIN, 39);       // ESP_I2S_WS     pad 44  (rev1: 1)
    EXPECT_EQ(config::I2S_BCLK_PIN, 40);     // ESP_I2S_BCLK   pad 45  (rev1: 2)
    EXPECT_EQ(config::LORA_BUSY_PIN, 41);    // L_BUSY         pad 47  (rev1: 14)
    EXPECT_EQ(config::LORA_RXEN_PIN, 42);    // L_RXEN         pad 48  (rev1: 10)
}

TEST(OcBoardM2, EverythingElseIsWhereRev1HasIt)
{
    // rev1's numbers, written out (its header was retired with the V1).
    EXPECT_EQ(config::I2S_DIN_PIN, 3);
    EXPECT_EQ(config::I2C_SDA_PIN, 5);
    EXPECT_EQ(config::I2C_SCL_PIN, 6);
    EXPECT_EQ(config::SCAP_ADC_PIN, 8);
    EXPECT_EQ(config::ARM_CONSENT_PIN, 11);
    EXPECT_EQ(config::LORA_DIO1_PIN, 17);
    EXPECT_EQ(config::LORA_RST_PIN, 18);
    EXPECT_EQ(config::PWR_SDA, 21);
    EXPECT_EQ(config::PWR_SCL, 33);
    EXPECT_EQ(config::SPI_MISO, 35);
    EXPECT_EQ(config::NAND_CS, 36);
    EXPECT_EQ(config::SPI_SCK, 37);
    EXPECT_EQ(config::SPI_MOSI, 38);
    EXPECT_EQ(config::GPS_PWR_PIN, -1);
    EXPECT_EQ(config::MRAM_CS, -1);
    EXPECT_EQ(config::INA_ALERT_PIN, -1);
    EXPECT_EQ(config::PG_RAIL_PIN, -1);
    EXPECT_EQ(config::VBUCK_OK_PIN, -1);
    EXPECT_EQ(config::LORA_UART_TX_PIN, -1);
    EXPECT_EQ(config::LORA_UART_RX_PIN, -1);
    EXPECT_EQ(config::LORA_ACT_PIN, -1);
    EXPECT_EQ(config::CAM_IMON_GPIO, -1);
    EXPECT_EQ(config::SERVO_IMON_GPIO, -1);
    EXPECT_TRUE(config::RING_IN_PSRAM);
    EXPECT_FALSE(config::SCAP_FROM_FC);
    EXPECT_FLOAT_EQ(config::SCAP_DIVIDER_RATIO, 2.0f);
    EXPECT_EQ(config::IMU_RATE_MAX_HZ, 7680);
    EXPECT_TRUE(config::USE_LORA_RADIO);
    EXPECT_FALSE(config::USE_UART_RADIO_MODEM);
    EXPECT_TRUE(config::LORA_ON_MEMORY_BUS);
}

TEST(OcBoardM2, NoSignalSitsOnAPadTheInPackageMemoryOwns)
{
    // The ESP32-S3FH4R2's flash and PSRAM are on GPIO26-32 (pads 28, 30-35).
    for (const auto& p : wiredPins())
        EXPECT_FALSE(p.second >= 26 && p.second <= 32) << p.first << " on GPIO" << p.second;
    // The strapping pads this board leaves alone: GPIO45 sets the flash rail
    // voltage and GPIO46 the boot mode.
    for (const auto& p : wiredPins())
        EXPECT_FALSE(p.second == 45 || p.second == 46 || p.second == 0)
            << p.first << " on a strapping pad";
}

TEST(OcBoardM2, TheRailEnableAndTheReceiveEnableAreNotOnAPadPulledUpAtReset)
{
    // Both sit behind a 100 k pull-down (R84, R142) that a 45 k reset pull-up
    // would beat. ESP32-S3 datasheet v2.2, Table 2-1: GPIO0, GPIO39 (MTCK,
    // while EFUSE_DIS_PAD_JTAG is unburnt), GPIO43 and GPIO44 come up pulled
    // high, as do the in-package memory pads.
    for (int pin : {config::PWR_PIN, config::LORA_RXEN_PIN})
    {
        EXPECT_NE(pin, 0);
        EXPECT_NE(pin, 39);
        EXPECT_NE(pin, 43);
        EXPECT_NE(pin, 44);
        EXPECT_FALSE(pin >= 26 && pin <= 32);
    }
    // And neither is in the power-up glitch table (Table 2-2: GPIO1-14,
    // GPIO17-20 and the 32 kHz pads), which rev1's GPIO7 and GPIO10 are.
    for (int pin : {config::PWR_PIN, config::LORA_RXEN_PIN})
        EXPECT_GT(pin, 20);
}

TEST(OcBoardM2, TheLinkPadTheChipPullsUpIsTheWordSelect)
{
    // The one moved signal on a pad with a reset pull, and the one that
    // tolerates it best: an input the flight computer drives push-pull.
    EXPECT_EQ(config::I2S_WS_PIN, config::S3_MTCK_GPIO);
}

TEST(OcBoardM2, TheHoldUpSenseStaysOnAdc1)
{
    EXPECT_GE(config::SCAP_ADC_PIN, 1);
    EXPECT_LE(config::SCAP_ADC_PIN, 10);
}

#if TR_BEETLE_V3

TEST(OcBoardM2, TheV3NamesItselfM3)
{
    // A V3 image says so in the log and the flight log, and a board provisioned
    // as a V2 does not match it (the OTA suffix "-m3" is CMake's, beside this).
    char norm[board_identity::kMaxRev + 1];
    board_identity::normalizeRev(TR_BOARD_REV_STR, norm, sizeof(norm));
    EXPECT_STREQ(norm, "M3");
    EXPECT_EQ(board_identity::encodeRevCode(TR_BOARD_REV_STR, true), 0xA3);
    EXPECT_TRUE(board_identity::revsMatch(TR_BOARD_REV_STR, "M3"));
    EXPECT_FALSE(board_identity::revsMatch(TR_BOARD_REV_STR, "M2"));
}

#else

TEST(OcBoardM2, TheBoardNameReducesToM2)
{
    char norm[board_identity::kMaxRev + 1];
    board_identity::normalizeRev(TR_BOARD_REV_STR, norm, sizeof(norm));
    EXPECT_STREQ(norm, "M2");
    // The flight log's one-byte code: family M (0x20), number 2, asserted.
    EXPECT_EQ(board_identity::encodeRevCode(TR_BOARD_REV_STR, true), 0xA2);
    EXPECT_EQ(board_identity::encodeRevCode(TR_BOARD_REV_STR, false), 0x22);
    // A V2 provisioned as "M2" matches this image; a rev1 board does not.
    EXPECT_TRUE(board_identity::revsMatch(TR_BOARD_REV_STR, "M2"));
    EXPECT_FALSE(board_identity::revsMatch(TR_BOARD_REV_STR, "M1"));
}

#endif   // TR_BEETLE_V3

#endif   // TR_BOARD_M2

