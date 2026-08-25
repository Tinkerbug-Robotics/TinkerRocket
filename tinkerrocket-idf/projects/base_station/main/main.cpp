// ==========================================================================
// SECTION: Includes, board selection, and compile-time configuration
// ==========================================================================
#include <compat.h>
#include <TR_NVS.h>
#include <algorithm>

#include <esp_app_desc.h>         // esp_app_get_description for firmware version readback (#8)
#include <esp_log.h>
#include <esp_mac.h>              // esp_efuse_mac_get_default for unit_id
#include <esp_ota_ops.h>          // esp_ota_mark_app_valid_cancel_rollback (#8)
#include <esp_partition.h>        // esp_ota_get_running_partition for rollback gate (#8)
#include <esp_rom_sys.h>          // esp_rom_delay_us — sub-tick NAND reset settle (#761)
#include <esp_vfs_fat.h>
#include <esp_spiffs.h>
#include <nvs_flash.h>            // nvs_flash_init — must run before any Preferences use (#500)
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_vfs_fat_nand.h>
#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>   // #835 item 2: own-battery divider read
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include <cstdio>
#include <memory>                // unique_ptr — file-list top-N window (#835)
#include <cstdlib>                // setenv() — UTC TZ for the log-rename helper (#168)
#include <cstring>
#include <cerrno>
#include <ctime>                  // mktime / gmtime_r for the rename-on-time-sync path (#168)
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>               // fsync() — periodic-flush SD commit (#107)

#include "config.h"
#include "bs_log_policy.h"        // parseSequentialFilename() (#137)
#include "bs_uplink_policy.h"     // mayTransmitUplink() scan gate (#379)
#include "bs_uplink_queue.h"      // uplink command FIFO (#502)
#include "bs_uplink_txwin.h"      // TX-in-the-RX-gap window policy (#506)
#include "bs_battery_soc.h"       // voltage-based SoC fallback (#501)
#include "bs_download_policy.h"   // nextChunk()/mayEmitChunk() pacing (#380)
#include "bs_storage_policy.h"    // NAND bring-up retry / demotion policy (#761)
#include "bs_file_list.h"       // top-N file-list window (#835)

#include <TR_LoRa_Comms.h>
#include <LoRaDirectBackend.h>
#include <UartModemBackend.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_I2C_Interface.h>   // binary log framing: packMessage(), same records as the OC
#include <TR_BLE_To_APP.h>
#include <TR_MAX17205G.h>
#include <TR_MAX17303.h>
#include <TR_MP2672.h>
#include <TR_BQ27Z746.h>
#include <RocketComputerTypes.h>

static const char* TAG = "BS";

// OTA rollback gate (#8). True only between boot and the first successful
// telemetry-while-connected event when we booted PENDING_VERIFY (i.e., a
// fresh OTA image). On first hit we call esp_ota_mark_app_valid_cancel_rollback
// once and clear the flag. If we never get there (panic, hang, BLE init
// failure) the bootloader auto-reverts to ota_0 on next boot.
static bool g_ota_pending_verify = false;

// ==========================================================================
// SECTION: OTA boot validation
// ==========================================================================
static inline void maybeMarkOtaValid()
{
    if (!g_ota_pending_verify) return;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "OTA: new image validated, rollback cancelled");
    }
    else
    {
        ESP_LOGE(TAG, "OTA: mark_app_valid_cancel_rollback failed: %s",
                 esp_err_to_name(err));
    }
    g_ota_pending_verify = false;
}

// Forward declarations
static const char* rocketStateToString(uint8_t state);

// Radio backend seam (#410/#414): direct SPI LLCC68 (V1/V2 boards) or the
// UART radio-daughterboard modem (V3), selected by the board header. The
// reference keeps the historical `lora_comms` name so every call site below
// is untouched; the unused backend is never begun.
static LoRaDirectBackend lora_direct_backend;
static UartModemBackend lora_modem_backend;
static IRadioLink& lora_comms =
    config::USE_UART_RADIO_MODEM
        ? static_cast<IRadioLink&>(lora_modem_backend)
        : static_cast<IRadioLink&>(lora_direct_backend);
static SensorConverter sensor_converter;
static TR_Coordinates coord;
static TR_BLE_To_APP ble_app("TinkerBaseStation");

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

// Stats tracking
static uint32_t last_stats_ms = 0;
static uint32_t last_rx_count = 0;
// Last time we heard OUR rocket: bumped only on a network-id-matched beacon
// or telemetry decode (#384). Drives the cmd-10 VERIFYING predicate, silence
// recovery, and hop-liveness — none of which should trust foreign traffic.
static uint32_t last_packet_ms = 0;
// CRC-passing decodes whose SNR was below loraMinValidSnrDb(current_sf)
// — almost certainly noise-floor false positives.  Counted but otherwise
// dropped so they don't update last_packet_ms / hop / recovery state.
// (#90 follow-up; the field-confirmed t=79 -12.8 dB SF8 catch is the
// motivating example.)
static uint32_t lora_low_snr_drops = 0;

// #329: telemetry packets dropped by the network_id filter.  Otherwise fully
// silent (no CSV row, no counter) — a top suspect for a base-station log that
// stays empty while the rocket is clearly transmitting, if the rocket's nid
// drifts from the BS default (forced to 0 at boot, see #136).
static uint32_t lora_netid_mismatch_drops = 0;

// #150 (Seam B finding): the counter is lifetime-cumulative, so the app's
// mismatch banner keyed on "drops > 0" never cleared after the link healed.
// Report drops over BLE only while they are RECENT — the banner then
// self-clears shortly after a fix, and a reconnecting app doesn't
// resurrect a stale warning.  The log/stats lines keep the lifetime count.
static constexpr uint32_t NETID_DROP_REPORT_WINDOW_MS = 30000;
static uint32_t lora_netid_last_drop_ms = 0;

// #570: CRC-valid packets dropped because their length is neither a beacon
// nor either telemetry frame size — the classic mixed-flash trap
// (rocket and BS built from commits with different LoRaData sizes drops 100%
// of telemetry while the app just shows "Searching"). Mirrors the #329
// netid-drop pattern: lifetime count for logs, recency-windowed "szd"
// surface over BLE (same window as nidd).
static uint32_t lora_size_mismatch_drops = 0;
static uint32_t lora_size_last_drop_ms   = 0;

// Base station battery (MAX17205G fuel gauge via I2C)
static float bs_voltage = NAN;
static float bs_soc = NAN;
static float bs_current = NAN;
static float bs_temperature = NAN;
// #501: true when bs_soc came from the voltage curve rather than the gauge's own
// coulomb count, because the gauge's SoC failed the plausibility check.
static bool  bs_soc_estimated = false;
static bs_battery_soc::Estimator bq_soc_estimator(config::SOC_FILTER_ALPHA);
static uint32_t last_battery_ms = 0;
static i2c_master_bus_handle_t i2c_bus = nullptr;
static TR_MAX17205G fuel_gauge(config::MAX17205_ADDR);
static TR_BQ27Z746  bq_gauge(config::BQ27Z746_ADDR);

// ==========================================================================
// SECTION: Peripheral objects and board hardware
// ==========================================================================
static TR_MAX17303  max17303_gauge(config::MAX17303_ADDR);  // V3; shares 0x36 with MAX17205, split by DevName
static TR_MP2672    pack_charger(config::MP2672_ADDR);      // V3 flight-pack charger (HAS_PACK_CHARGER)
// Which gauge runtime detection found (one firmware image, both PCBs).
enum class GaugeKind { None, MAX17205, BQ27Z746, MAX17303 };
static GaugeKind gauge_kind = GaugeKind::None;
static bool fuel_gauge_present = false;   // true if EITHER gauge is present

// Servo/PID config cache (for BLE readback — mirrors what was sent to rocket)
static int16_t cfg_servo_bias1 = 0;
static int16_t cfg_servo_hz    = 50;
static int16_t cfg_servo_min   = 1000;
static int16_t cfg_servo_max   = 2000;
static float   cfg_pid_kp  = 0.10f;
static float   cfg_pid_ki  = 0.0f;
static float   cfg_pid_kd  = 0.0f;
static float   cfg_pid_min = -20.0f;
static float   cfg_pid_max = 20.0f;
static bool    cfg_servo_enabled = true;

// Device identity (loaded from NVS "identity" namespace)
static char    unit_id_hex[9] = {0};              // last 4 bytes of MAC as "a1b2c3d4"
static char    unit_name[24]  = "TinkerBaseStation"; // default until NVS loads
static uint8_t network_id     = config::DEFAULT_NETWORK_ID;

// LoRa uplink state (BaseStation → OutComputer).
// #502: a FIFO, not a single slot — see bs_uplink_queue.h. The uplink is blind
// (no command ACK from the rocket), so a command's retries ARE its delivery;
// overwriting a pending command threw those away.
static bs_uplink_queue::Queue uplink_q;
static uint32_t uplink_last_tx_ms = 0;

// ==========================================================================
// SECTION: Uplink queue state
// ==========================================================================
// "The uplink is busy" — successor to the old `uplink_pending` flag. True while
// anything is queued or mid-retry, so the transaction / recovery / heartbeat /
// mask-drift paths keep their original meaning: don't touch the radio, and
// don't consider a relay finished, until the queue has fully drained.
static inline bool uplinkBusy() { return uplink_q.busy(); }

// #506: the rocket's downlink cadence, learned from RX timestamps, so uplink
// retries can be fired into the quiet stretch between telemetry packets instead
// of straight over the top of them.
static bs_uplink_txwin::RxCadence rx_cadence;
static uint32_t uplink_defer_start_ms = 0;   // when the head command was first held back
static uint32_t uplink_tx_count       = 0;   // TXs actually emitted (stats)
static uint32_t uplink_tx_airtime_ms  = 0;   // cumulative uplink time-on-air (stats)
static uint32_t uplink_defer_count    = 0;   // passes deferred to protect the downlink
static uint32_t uplink_defer_override = 0;   // liveness backstop fired (should be ~0)

// Storage mount state. Preferred backend is SD over SDMMC; if that fails at boot
// we fall back to SPIFFS on internal flash (partitions.csv: "spiffs", ~5 MB).
// The pointer is reassigned to /flash on fallback so all downstream paths
// (logger, BLE file list/delete/download) keep working via VFS unchanged.
static const char* SD_MOUNT_POINT = "/sdcard";
static sdmmc_card_t* sd_card = nullptr;
static bool using_internal_flash = false;
static bool using_external_flash = false;   // V2/V3: FAT on the external SPI flash
// #761: true when this board HAS primary storage and we are on SPIFFS anyway.
// Distinct from using_internal_flash, which on a hypothetical board with no
// primary storage would be the intended backend rather than a failure.
static bool storage_demoted      = false;
static spi_device_handle_t      s_ext_spi = nullptr;
static spi_nand_flash_device_t *s_nand    = nullptr;

// ==========================================================================
// SECTION: External flash and FAT mount
// ==========================================================================
// NAND bring-up diagnostics, latched at boot (#761).  Before these existed, a
// demotion to internal SPIFFS was only inferable from the storage frame's
// `backend` field, which says where we ended up and not why — so a field report
// of "it logged to internal flash again" carried no information about which of
// the five bring-up steps failed, and the boot log had usually scrolled away.
enum : uint8_t
{
    NAND_STEP_NONE      = 0,
    NAND_STEP_BUS_INIT  = 1,   // spi_bus_initialize
    NAND_STEP_ADD_DEV   = 2,   // spi_bus_add_device
    NAND_STEP_READY     = 3,   // RESET + ready poll (below; not in the driver)
    NAND_STEP_NAND_INIT = 4,   // spi_nand_flash_init_device (probe + FTL attach)
    NAND_STEP_FAT_MOUNT = 5,   // esp_vfs_fat_nand_mount
};
static uint8_t   nand_fail_step = NAND_STEP_NONE;  // step that failed on the LAST attempt
static esp_err_t nand_fail_err  = ESP_OK;          // its error code
static uint8_t   nand_attempts  = 0;               // attempts actually made
static uint8_t   nand_mfr_id    = 0x00;            // raw byte from the 9Fh pre-probe

static const char* nandStepName(uint8_t step)
{
    switch (step)
    {
        case NAND_STEP_BUS_INIT:  return "spi_bus_initialize";
        case NAND_STEP_ADD_DEV:   return "spi_bus_add_device";
        case NAND_STEP_READY:     return "NAND reset/ready poll";
        case NAND_STEP_NAND_INIT: return "spi_nand_flash_init_device";
        case NAND_STEP_FAT_MOUNT: return "esp_vfs_fat_nand_mount";
        default:                  return "none";
    }
}

// Opcodes we have to issue ourselves, before the driver is up.
//
// The vendored spi_nand_flash component sends READ ID as the literal first
// transaction on the bus: nand_init_device() -> detect_chip() ->
// spi_nand_read_manufacturer_id().  There is no RESET and no ready poll
// anywhere in the component — grep it, opcode 0xFF does not appear — and no
// post-power-on settle.  That is harmless on a chip sitting idle and wrong in
// exactly the case the field reports: the reset that follows a flash leaves the
// NAND powered and possibly still running the PROGRAM or ERASE that was in
// flight when the flasher took the MCU.  Commands issued while OIP is set are
// unreliable, and a SET FEATURE is simply ignored — so unprotect_chip()'s write
// clearing REG_PROTECT can silently do nothing, leaving the block-protect bits
// armed.  That failure does NOT surface as a detect error; it surfaces two
// steps later when the FTL cannot erase and the FAT mount fails.
static constexpr uint8_t NAND_CMD_RESET       = 0xFF;
static constexpr uint8_t NAND_CMD_GET_FEATURE = 0x0F;
static constexpr uint8_t NAND_CMD_READ_ID     = 0x9F;
static constexpr uint8_t NAND_REG_STATUS      = 0xC0;
static constexpr uint8_t NAND_STAT_OIP        = 0x01;   // operation in progress

// One raw half-duplex command on the not-yet-driver-owned NAND.  Mirrors the
// component's own transaction shape (variable cmd/addr/dummy, 8-bit command),
// so anything that works here works once the driver takes over.  rx_len <= 4:
// SPI_TRANS_USE_RXDATA lands the reply in the transaction's inline buffer.
static esp_err_t nandRawCmd(uint8_t cmd, uint8_t addr, uint8_t addr_bytes,
                            uint8_t* rx, uint8_t rx_len)
{
    spi_transaction_ext_t t = {};
    t.base.flags    = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
    t.base.cmd      = cmd;
    t.base.addr     = addr;
    t.base.length   = 0;              // half-duplex: nothing beyond cmd + addr
    t.base.rxlength = rx_len * 8;
    t.command_bits  = 8;
    t.address_bits  = addr_bytes * 8;
    t.dummy_bits    = 0;
    if (rx_len) t.base.flags |= SPI_TRANS_USE_RXDATA;

    esp_err_t e = spi_device_transmit(s_ext_spi, &t.base);
    if (e == ESP_OK && rx_len) memcpy(rx, t.base.rx_data, rx_len);
    return e;
}

// Put the NAND in a known state and wait for it to report ready, so the
// driver's first command lands on an idle chip.  See the opcode block above for
// why the driver not doing this is the leading suspect for the demotions.
static esp_err_t nandResetAndWaitReady()
{
    esp_err_t e = nandRawCmd(NAND_CMD_RESET, 0, 0, nullptr, 0);
    if (e != ESP_OK) return e;

    // Let OIP actually assert before the first poll. Without this the poll can
    // win the race and read a stale not-busy, which would quietly give up the
    // one thing this function exists to do — wait out an operation that was
    // still running when the MCU reset.
    esp_rom_delay_us(200);

    // tRST depends on what the chip was doing when RESET arrived (idle is a few
    // microseconds, mid-erase is the worst case) — poll rather than guess.
    const uint32_t deadline = millis() + config::NAND_READY_TIMEOUT_MS;
    for (;;)
    {
        uint8_t status = 0xFF;
        e = nandRawCmd(NAND_CMD_GET_FEATURE, NAND_REG_STATUS, 1, &status, 1);
        if (e != ESP_OK) return e;
        if ((status & NAND_STAT_OIP) == 0) return ESP_OK;
        if ((int32_t)(millis() - deadline) >= 0) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Undo whatever this attempt managed to set up, so the next one starts from a
// cold bus rather than a half-initialised one.
//
// s_nand is nulled WITHOUT deinit when the probe itself failed: on that path
// nand_init_device() has already freed the handle but leaves the caller's
// pointer pointing at it, so s_nand is dangling and must not be passed back in.
// (The only other way spi_nand_flash_init_device can fail is a NULL ops table,
// which is a compile-time-constant struct — unreachable — so nulling is safe
// rather than a leak.)
static void tearDownExtFlash(bool bus_up, bool nand_handle_valid)
{
    if (s_nand)
    {
        if (nand_handle_valid) spi_nand_flash_deinit_device(s_nand);
        s_nand = nullptr;
    }
    if (s_ext_spi)
    {
        spi_bus_remove_device(s_ext_spi);
        s_ext_spi = nullptr;
    }
    if (bus_up) spi_bus_free(SPI3_HOST);
}

// One bring-up attempt.  On success repoints SD_MOUNT_POINT and sets
// using_external_flash; on failure records (nand_fail_step, nand_fail_err),
// tears everything back down and returns the error.
//
// allow_format gates esp_vfs_fat_nand_mount's format_if_mount_failed.  Only the
// LAST attempt may format: a transient FTL or mount error that a retry would
// have fixed must not reach f_mkfs, because that trades a recoverable boot for
// every flight log on the chip.  A genuinely blank NAND still gets formatted,
// just one attempt later.
static esp_err_t mountExternalFlashFatAttempt(uint8_t attempt, uint32_t clock_hz, bool allow_format)
{
    const uint8_t attempts = config::NAND_MOUNT_ATTEMPTS;
    bool bus_up = false;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num     = config::FLASH_MOSI;
    buscfg.miso_io_num     = config::FLASH_MISO;
    buscfg.sclk_io_num     = config::FLASH_SCK;
    buscfg.quadwp_io_num   = -1;
    buscfg.quadhd_io_num   = -1;
    buscfg.max_transfer_sz = 4096 + 256;     // page + spare
    esp_err_t e = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "ext-flash [%u/%u] SPI bus init failed (0x%x)", attempt, attempts, (int)e);
        nand_fail_step = NAND_STEP_BUS_INIT; nand_fail_err = e;
        // INVALID_STATE means the host is still claimed — a teardown that did
        // not complete, which every later attempt would trip over identically.
        // Release it so the retry has something to retry into; without this the
        // "bounded retry" would be three copies of the same failure.
        if (e == ESP_ERR_INVALID_STATE) spi_bus_free(SPI3_HOST);
        return e;
    }
    bus_up = true;

    const uint32_t spi_flags = SPI_DEVICE_HALFDUPLEX;
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = clock_hz;
    devcfg.mode           = 0;
    devcfg.spics_io_num   = config::FLASH_CS;
    devcfg.queue_size     = 10;
    devcfg.flags          = spi_flags;
    e = spi_bus_add_device(SPI3_HOST, &devcfg, &s_ext_spi);
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "ext-flash [%u/%u] add SPI device failed (0x%x)", attempt, attempts, (int)e);
        nand_fail_step = NAND_STEP_ADD_DEV; nand_fail_err = e;
        tearDownExtFlash(bus_up, false);
        return e;
    }

    // Let the rail settle, then reset the chip and wait for ready before the
    // driver's first command.
    vTaskDelay(pdMS_TO_TICKS(config::NAND_SETTLE_MS));
    e = nandResetAndWaitReady();
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "ext-flash [%u/%u] NAND reset/ready poll failed (0x%x)%s",
                 attempt, attempts, (int)e,
                 e == ESP_ERR_TIMEOUT ? " -- chip never cleared OIP (absent, unpowered or wedged)" : "");
        nand_fail_step = NAND_STEP_READY; nand_fail_err = e;
        tearDownExtFlash(bus_up, false);
        return e;
    }

    // Read the manufacturer byte ourselves purely so it reaches the log. The
    // driver checks the same byte but reports only "unsupported chip or
    // wiring"; the raw value separates the cases that need different fixes —
    // 0xCD is the FORESEE part, 0x00 / 0xFF is a floating MISO (chip absent,
    // unpowered, or CS/MISO not connected — cf. #714, where board_v4 dropped
    // the NAND entirely), and anything else is a genuinely different chip.
    nandRawCmd(NAND_CMD_READ_ID, 0, 1, &nand_mfr_id, 1);

    spi_nand_flash_config_t nand_cfg = {};
    nand_cfg.device_handle = s_ext_spi;
    nand_cfg.io_mode       = SPI_NAND_IO_MODE_SIO;
    nand_cfg.flags         = spi_flags;
    e = spi_nand_flash_init_device(&nand_cfg, &s_nand);
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "ext-flash [%u/%u] SPI NAND init failed (0x%x) -- unsupported chip or wiring "
                      "(mfr id read back as 0x%02x, expected 0xcd)",
                 attempt, attempts, (int)e, nand_mfr_id);
        nand_fail_step = NAND_STEP_NAND_INIT; nand_fail_err = e;
        tearDownExtFlash(bus_up, false);   // handle already freed by the driver
        return e;
    }

    uint32_t blocks = 0, block_sz = 0, page_sz = 0;
    spi_nand_flash_get_block_num(s_nand, &blocks);
    spi_nand_flash_get_block_size(s_nand, &block_sz);
    spi_nand_flash_get_page_size(s_nand, &page_sz);
    ESP_LOGI(TAG, "SPI NAND up [%u/%u]: mfr 0x%02x, %lu blocks x %lu B (page %lu B) = %llu MB @ %lu MHz",
             attempt, attempts, nand_mfr_id,
             (unsigned long)blocks, (unsigned long)block_sz, (unsigned long)page_sz,
             (unsigned long long)((uint64_t)blocks * block_sz / (1024 * 1024)),
             (unsigned long)(clock_hz / 1000000));

    esp_vfs_fat_mount_config_t mcfg = {};
    mcfg.format_if_mount_failed = allow_format;
    mcfg.max_files              = 5;
    mcfg.allocation_unit_size   = 0;   // FATFS default cluster size
    e = esp_vfs_fat_nand_mount("/extflash", s_nand, &mcfg);
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "ext-flash [%u/%u] FAT mount failed (0x%x)%s", attempt, attempts, (int)e,
                 allow_format ? "" : " -- format withheld until the final attempt");
        nand_fail_step = NAND_STEP_FAT_MOUNT; nand_fail_err = e;
        tearDownExtFlash(bus_up, true);
        return e;
    }

    SD_MOUNT_POINT = "/extflash";
    using_external_flash = true;
    // nand_fail_step/_err are deliberately NOT cleared here: after a recovered
    // boot they are the record of what the retry had to work around, which is
    // the whole point of collecting them.
    ESP_LOGI(TAG, "External-flash FAT mounted at %s", SD_MOUNT_POINT);
    return ESP_OK;
}

// Mount a FAT filesystem (Dhara wear-leveling FTL) on the external SPI NAND
// flash used for logging on the V2/V3 PCBs (M_* pins on SPI3_HOST; LoRa owns
// SPI2_HOST on V2). The chip is a FORESEE F35SQB004G (mfr 0xCD, 512 MB), supported via
// the vendored+patched spi_nand_flash component (components/spi_nand_flash/src/
// devices/nand_foresee.c). On success repoints SD_MOUNT_POINT and sets
// using_external_flash; otherwise returns the error so the caller falls back to
// internal SPIFFS.
//
// #761: this used to be single-shot — any one of the four steps returned
// straight to the caller, which then mounted the ~1.9 MB internal SPIFFS
// partition and logged there for the rest of the boot, so one transient error
// cost 512 MB of flight log and truncated the flight. Now each attempt tears
// itself back down and the next one starts from a cold bus, and the last
// attempt also halves the SPI clock.
static esp_err_t mountExternalFlashFat()
{
    esp_err_t e = ESP_FAIL;
    for (uint8_t attempt = 1; attempt <= config::NAND_MOUNT_ATTEMPTS; ++attempt)
    {
        const auto plan = bs_storage_policy::planAttempt(
            attempt, config::NAND_MOUNT_ATTEMPTS,
            config::NAND_CLOCK_HZ, config::NAND_CLOCK_FALLBACK_HZ);

        nand_attempts = attempt;
        e = mountExternalFlashFatAttempt(attempt, plan.clock_hz, plan.allow_format);
        if (e == ESP_OK)
        {
            if (attempt > 1)
            {
                ESP_LOGW(TAG, "SPI NAND came up on attempt %u/%u (last failure: %s, 0x%x)%s",
                         attempt, config::NAND_MOUNT_ATTEMPTS,
                         nandStepName(nand_fail_step), (int)nand_fail_err,
                         plan.clock_hz != config::NAND_CLOCK_HZ ? " -- on the reduced bring-up clock" : "");
            }
            return ESP_OK;
        }
        if (!plan.is_last) vTaskDelay(pdMS_TO_TICKS(config::NAND_MOUNT_RETRY_DELAY_MS));
    }

    ESP_LOGE(TAG, "SPI NAND bring-up failed on all %u attempts; last failing step: %s (0x%x)",
             config::NAND_MOUNT_ATTEMPTS, nandStepName(nand_fail_step), (int)nand_fail_err);
    return e;
}
static const char* SPIFFS_PARTITION_LABEL = "spiffs";

// CSV logging state
static FILE* log_file = nullptr;
static bool logging_active = false;
static uint32_t log_start_ms = 0;
static uint32_t log_last_write_ms = 0;
static uint32_t log_last_flush_ms = 0;
// Throttle for repeated fopen() attempts when startLogging() fails (#107).
// Auto-start fires on every packet until logging_active is true; without
// a throttle a stuck SD card would spam ESP_LOGE on every downlink.
// Reset to 0 by stopLogging() so the next flight can retry immediately
// rather than waiting out the residual window.
static uint32_t log_last_open_attempt_ms = 0;
// #329: CSV writes/flushes that failed.  fprintf() usually buffers a row and
// returns OK even on a full card, so a full disk surfaces only at fflush/fsync
// (ENOSPC) — counting both makes otherwise-silent row loss visible instead of
// vanishing with no warning. (Declared here so startLogging/stopLogging can
// count header-write and close failures too — the #384 gaps.)
static uint32_t log_write_fail_count = 0;
// State-transition tracking (INFLIGHT safety arm time, last state, boot-edge
// guard, per-rocket freq lock) lives per tracked rocket in
// TrackedRocket::log_state (#381) — one global here made two interleaved
// rockets read as a state transition on every packet, closing and reopening
// the CSV once per interleave cycle. See bs_log_policy.h.
// Sticky inhibit set by a manual cmd 23 stop so auto-start doesn't immediately
// re-open on the next packet (#107).  Cleared on any rocket state change or
// by a manual cmd 23 start, so a real next flight is still captured.
static bool     log_manual_inhibit = false;
static bool     last_known_camera_recording = false;  // Track rocket camera state for idempotent uplink
static bool     last_known_rocket_logging = false;    // Actual rocket logging state from LoRa downlink

// Frequency lock for flight (issue #71).  Set when any tracked rocket
// reports INFLIGHT; cleared on LANDED or READY (matches the rocket-side
// sticky flag).  PRELAUNCH does NOT clear, since a post-flight
// LANDED → PRELAUNCH transition (rocket regains GPS on the ground) would
// otherwise leave the lock stuck on indefinitely.  While set:
//   • Silence recovery is suppressed (we don't hop/scan during flight).
//   • Silence tolerance is extended so momentary SNR dips don't alarm.
//   • Transactional Cmd 10 handler still refuses on the rocket side, but
//     we refuse on the base station side too as belt-and-braces.
// The transition logic is shared with the rocket side via
// computeFreqLockForFlight() in RocketComputerTypes.h.
//
// #381: this is now the AGGREGATE across tracked rockets — locked while any
// fresh rocket's per-rocket lock is latched (bs_log_policy::aggregateFreqLock).
// The old single global was overwritten by whichever rocket's packet arrived
// last, so a second rocket's READY cleared the lock mid-flight of the first,
// once per interleaved packet.
//
// #835 item 6: the aggregate is a CACHE, and a cache only decays if something
// recomputes it.  Until this fix the sole writer was the accepted-telemetry RX
// path, so a rocket that went silent while INFLIGHT froze the last `true` for
// the rest of the power cycle: the freshness window was unreachable and cmd-10,
// silence recovery and fixed-mode heartbeats stayed suppressed until reboot.
// loop_bs now re-evaluates every pass (see the log-lifecycle section), and the
// latch expires reads older than the window even if that call is ever lost.
//
// Two windows — see bs_log_policy::FreqLockLatch for why.  The short one gates
// the passive consumers; the long one gates anything that physically retunes
// the radio, which must stay shut for the whole plausible flight.
static bs_log_policy::FreqLockLatch freq_lock_latch;

// Passive consumers: silence recovery, fixed-mode heartbeats, auto-acquire.
static inline bool freqLockedForFlight()
{
    return freq_lock_latch.flightLockedAt(millis(), config::LOG_SILENCE_TIMEOUT_MS);
}

// Radio-moving consumers: the cmd-10 LoRa reconfigure transaction.
static inline bool freqLockedForRetune()
{
    return freq_lock_latch.retuneLockedAt(millis(), config::LOG_INFLIGHT_SAFETY_MS);
}

// ----------------------------------------------------------------------------
// Per-packet channel-hop state (issues #40 / #41, phase 2a — BS side)
// ----------------------------------------------------------------------------
// The BS is purely reactive: the rocket owns the hop sequence, and the BS
// retunes after each successful RX based on the packet's next_channel_idx.
// hop_active_ tracks whether we're currently following a hopping rocket;
// hop_idx_ is the channel we're (about to be) tuned to for the next RX.
//
// Multi-rocket caveat (deliberately punted to a follow-up): if the BS is
// tracking more than one rocket, only one of them can drive the hop
// sequence at a time.  v2a follows whichever rocket's packet arrived
// most recently, which works fine when there's just one rocket on the
// link — the dominant case for our setup.
static bool     hop_active_       = false;
static uint8_t  hop_idx_          = 0;
static bool     hop_needs_retune_ = false;
static uint32_t hop_last_rx_ms_   = 0;

// Link mode (#106 origin, user-facing since #150): suppress hopping and
// stay on lora_freq_mhz.  BS is the authority; the BLE handler for cmd 17
// (driven by the app's Fixed/Hopping picker) updates this flag, persists
// to NVS, and uplinks the same byte to the rocket via
// LORA_CMD_SET_HOP_DISABLED so both sides stay aligned —
// serviceHopModeResync() re-pushes on evidence of a mismatch.
// Initialized true (fixed mode, the default) so the window between boot
// and the NVS load can never report hop-enabled.
static bool     lora_hop_disabled = true;

// ==========================================================================
// SECTION: LoRa channel hopping
// ==========================================================================
// #150: packets-per-channel dwell for the CURRENT modulation, derived
// from real airtime so the FCC occupancy bound holds at every preset.
// 0 means hopping is not permitted at this (sf, bw, cr): the cmd-17
// enable is refused and the hop-follow path stays inert.  Uses the link
// gate (not the raw config function): CR-only changes are unverifiable
// over the air, so hopping is only offered at the factory CR — see
// loraHopDwellForLink.  Must match the rocket's computation — both call
// the same shared function.
static inline uint8_t currentHopDwell()
{
    return loraHopDwellForLink(lora_sf, lora_bw_khz, SIZE_OF_LORA_BUDGET, lora_cr);
}

// ----------------------------------------------------------------------------
// Channel-set state (#40 / #41 phase 3)
// ----------------------------------------------------------------------------
// Selected by `loraSelectChannelSet()` after the pre-launch scan finishes,
// persisted to NVS, and pushed to the rocket via LORA_CMD_CHANNEL_SET.
// `channel_set_bw_khz_` is the BW the mask was generated against — used
// to detect when a cmd-10 BW change invalidates the mask (we revert to
// "no skips" until the user re-runs the scan on the new BW).
//
// Rendezvous freq used to live here too (NVS-stored, scan-selected) but
// that allowed the BS and rocket to drift apart with no recovery path —
// see issue #105.  It's now compile-time hardcoded to the shared
// LORA_FACTORY_RENDEZVOUS_MHZ on both sides.
static uint8_t skip_mask_[LORA_SKIP_MASK_MAX_BYTES] = {0};
static uint8_t skip_mask_n_        = 0;        // 0 = no mask yet
static float   channel_set_bw_khz_ = 0.0f;     // BW the mask was built for

// #150: effective skip mask for the hop schedule = the scan mask (when
// valid for the current BW) + the implicit home-channel skip — identical
// derivation to the rocket's, so the expected-next sanity check compares
// like with like.  `buf` must hold LORA_SKIP_MASK_MAX_BYTES.
static const uint8_t* effectiveHopMask(uint8_t* buf, uint8_t n_channels)
{
    const bool mask_valid = (skip_mask_n_ == n_channels &&
                              channel_set_bw_khz_ == lora_bw_khz);
    if (mask_valid) memcpy(buf, skip_mask_, LORA_SKIP_MASK_MAX_BYTES);
    else            memset(buf, 0, LORA_SKIP_MASK_MAX_BYTES);
    (void)loraApplyHomeChannelSkip(buf, lora_bw_khz, lora_freq_mhz);
    return buf;
}

// Mask-drift auto-recovery (#105 follow-up).  Set in the hop RX path
// when the rocket's announced next_channel_idx disagrees with the
// BS-side seq-derived channel — that means BS and OC have diverged
// skip-masks (typically: scan ran, cmd-15 push got displaced by a
// follow-on cmd-10, OC never received the new mask).  serviceMaskDriftRepush
// re-queues cmd 15 with a cooldown so it can't spam.
static bool     chset_drift_repush_pending = false;
static uint32_t chset_drift_repush_last_ms_ = 0;
static constexpr uint32_t CHSET_DRIFT_REPUSH_COOLDOWN_MS = 30000;
static uint32_t chset_drift_repush_count   = 0;  // for [STATS]

// Forward declarations — definitions live further down with the other
// channel-set machinery, but a couple of call sites (cmd-10 commit,
// boot-time NVS load) need them earlier in the file.
static void loadChannelSetFromNvs();
static void invalidateSkipMaskForBwChange();
static void analyzeAndPushFromCachedScan();
static void pushCurrentChannelSet();
static void serviceMaskDriftRepush();
static void serviceHopModeResync();
static void serviceHopDisableDrain();

// ==========================================================================
// SECTION: Auto-acquire state
// ==========================================================================
// Auto-acquire + auto-scan state (#136).  Full definitions live just
// before setup_bs(); the enum + state variable are declared here so
// finalizeNoiseScan() can branch on them without forward-decl gymnastics.
enum class AutoAcquireState : uint8_t {
    AWAITING_ROCKET,
    GRACE_DELAY,
    SCANNING,
    COMMITTING,
    DONE,
};
static AutoAcquireState auto_acquire_state = AutoAcquireState::AWAITING_ROCKET;
static void autoAcquireOnScanFinalize();

// Noise-scan pass counter — actual scan machinery lives further down,
// but serviceHeartbeat() needs to see this to skip beats mid-scan.
// Moved up from its original spot near the scan code (#136 follow-up: a
// heartbeat TX during the auto-acquire scan caused subsequent passes
// of the multi-pass scan to refuse to start because tx_ongoing_ was
// still set when startScan() was retried).
static uint8_t scan_passes_remaining_ = 0;

// If we're following a hopping rocket and packets dry up for this long,
// give up and fall back to lora_freq_mhz so the existing silence /
// recovery machinery can take over.  Sized to swallow a handful of
// missed hop windows even at the slowest preset (Long Range = ~2 Hz),
// without holding the radio hostage if the rocket really has vanished.
static constexpr uint32_t HOP_SILENCE_FALLBACK_MS = 3000;

// ----------------------------------------------------------------------------
// Hop-session diagnostics (#105)
// ----------------------------------------------------------------------------
// A "hop session" begins when hop_active_ flips false→true (BS starts
// following the rocket's hop schedule) and ends on either silence-fallback
// or rocket-state-change.  The counters here let the operator see, after a
// flight, how often lock was lost, how long sessions lasted, and how many
// packets the BS *observed* missing during each session — together with
// the seq-derived per-packet loss in the CSV, this is the post-flight
// view that issue #105 was opened to enable.
static uint32_t hop_silence_events_count   = 0;   // lifetime, since boot
static uint32_t hop_session_started_ms     = 0;   // 0 = no active session
static uint32_t hop_session_total_pkts     = 0;   // accepted while hop_active_
static uint32_t hop_session_observed_loss  = 0;   // sum of observed gaps (#105)
static uint32_t lora_total_observed_loss   = 0;   // lifetime, since boot

// ----------------------------------------------------------------------------
// Hop-mode resync (#150)
// ----------------------------------------------------------------------------
// The BS is the link-mode authority, but a rocket that rebooted before a
// cmd-17 change persisted (or missed it entirely) can disagree.  The RX
// path collects evidence and serviceHopModeResync() re-uplinks cmd 17
// with the BS's mode.  Two directions, different bars:
//   • BS hop-enabled, rocket announces NO_HOP in a hop state: legitimate
//     during rendezvous visits / coordinated-scan pauses, so require
//     HOP_MODE_RESYNC_STREAK consecutive frames before acting.
//   • BS fixed, rocket announces a real channel idx: never legitimate —
//     act on first evidence (cooldown still applies).
static constexpr uint8_t  HOP_MODE_RESYNC_STREAK      = 6;
static constexpr uint32_t HOP_MODE_RESYNC_COOLDOWN_MS = 10000;
static uint8_t  hop_mode_mismatch_streak_  = 0;
static bool     hop_mode_resync_pending    = false;
static uint32_t hop_mode_resync_last_ms_   = 0;
static uint32_t hop_mode_resync_count_     = 0;

// #150 (review): graceful hop disable.  A cmd-17 disable taken while we
// are FOLLOWING a hop must drain its uplink retries on the channel the
// rocket is listening on BEFORE we stop following and retune away — the
// naive order (flip, retune, then let serviceUplink fire) put at most one
// blind retry on the hop channel and the rocket essentially never heard
// the disable.  Non-zero deadline = drain in progress; commit happens in
// serviceHopDisableDrain().
static constexpr uint32_t HOP_DISABLE_DRAIN_MAX_MS = 1500;
static uint32_t hop_disable_drain_deadline_ms = 0;

// Per-rocket tracker (replaces single last_decoded for multi-rocket support)
static constexpr int MAX_TRACKED_ROCKETS = 4;

// ==========================================================================
// SECTION: Multi-rocket tracker
// ==========================================================================
struct TrackedRocket {
    bool       active = false;
    uint8_t    rocket_id = 0;
    char       unit_name[24] = {0};
    LoRaDataSI last_data = {};
    float      last_rssi = NAN;
    float      last_snr  = NAN;
    double     last_lat_deg = NAN;
    double     last_lon_deg = NAN;
    double     last_alt_m = NAN;
    uint32_t   last_seen_ms = 0;
    // #835 item 6 residual: last accepted TELEMETRY packet. last_seen_ms
    // above is also stamped by a NAME BEACON, which must not hold the
    // flight frequency lock alive.
    uint32_t   last_telem_ms = 0;
    // Free-running TX seq from the rocket (#105, widened to 16 bits in
    // proto v4).  -1 = no prior packet ("first contact"); on an
    // implausible forward delta we reset to -1 and treat the next
    // packet as fresh contact — most likely the rocket rebooted and
    // lora_tx_seq restarted at 0.  int32_t to hold the full uint16
    // range plus the -1 sentinel.
    int32_t    last_seq = -1;
    // Per-rocket CSV/freq-lock transition state (#381) — replaces the old
    // globals last_rocket_state / have_seen_first_state / inflight_entry_ms.
    bs_log_policy::RocketLogState log_state = {};
};
static TrackedRocket tracked_rockets[MAX_TRACKED_ROCKETS];
static uint8_t active_rocket_idx = 0;  // Which rocket the BLE telemetry currently shows

// ── Radio focus (#390) ──
// One rocket owns this BS's radio: hop-follow, the stale re-push subject
// (active_rocket_idx), and the default target for untargeted uplinks.
// Before #390 all of those keyed off "whichever packet arrived last",
// which flip-flopped at packet rate with two rockets in range.
//   focus_rid_pinned — app's explicit pin (cmd 45); survives until the app
//                      changes it or BLE resets us (RAM-only by design:
//                      the app re-sends on every connect).
//   focus_rid_auto   — sticky FIRST-heard rocket. One-way fallback to a
//                      fresh rocket only after the sticky one has been
//                      silent > FOCUS_AUTO_FALLBACK_MS; never ping-pongs
//                      between two live rockets.
static uint8_t focus_rid_pinned = 0;   // 0 = auto
static uint8_t focus_rid_auto   = 0;   // 0 = nothing heard yet
static constexpr uint32_t FOCUS_AUTO_FALLBACK_MS = 30000;

static uint8_t effectiveFocusRid()
{
    return focus_rid_pinned != 0 ? focus_rid_pinned : focus_rid_auto;
}

/// Tracker slot of a rocket id, or -1.
static int slotOfRid(uint8_t rid)
{
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++)
        if (tracked_rockets[i].active && tracked_rockets[i].rocket_id == rid)
            return i;
    return -1;
}

/// True when `rid` is the rocket this BS's radio is dedicated to.  With no
/// focus at all yet (nothing heard, no pin) every rocket qualifies — the
/// first packet then latches the auto focus.
static bool isFocusedRocket(uint8_t rid)
{
    const uint8_t f = effectiveFocusRid();
    return f == 0 || f == rid;
}

/// Uplink target for rocket-directed commands: the focused rocket, or
/// broadcast while nothing has been heard yet (pre-#390 behaviour).
static uint8_t focusTargetRid()
{
    const uint8_t f = effectiveFocusRid();
    return f != 0 ? f : 0xFF;
}

/// Fold one received packet into the focus state: latch the first-heard
/// rocket, and (auto mode only) fall back one-way to a fresh rocket when
/// the sticky one has gone silent past the fallback window.
static void updateFocusOnPacket(uint8_t rid, uint32_t now_ms)
{
    if (focus_rid_auto == 0)
    {
        focus_rid_auto = rid;
        ESP_LOGI(TAG, "[FOCUS] Auto focus latched on rocket %u (first heard)",
                 (unsigned)rid);
        return;
    }
    if (focus_rid_pinned != 0 || rid == focus_rid_auto) return;
    const int fslot = slotOfRid(focus_rid_auto);
    if (fslot < 0 ||
        (now_ms - tracked_rockets[fslot].last_seen_ms) > FOCUS_AUTO_FALLBACK_MS)
    {
        ESP_LOGW(TAG, "[FOCUS] Auto focus fallback: rocket %u silent > %u s, following rocket %u",
                 (unsigned)focus_rid_auto,
                 (unsigned)(FOCUS_AUTO_FALLBACK_MS / 1000), (unsigned)rid);
        focus_rid_auto = rid;
    }
}

/// Find or allocate a tracker slot for a given rocket_id.
/// Returns index, or -1 if all slots full.
static int findOrAllocRocket(uint8_t rid) {
    int free_slot = -1;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].active && tracked_rockets[i].rocket_id == rid)
            return i;
        if (!tracked_rockets[i].active && free_slot < 0)
            free_slot = i;
    }
    if (free_slot >= 0) {
        tracked_rockets[free_slot].active = true;
        tracked_rockets[free_slot].rocket_id = rid;
        tracked_rockets[free_slot].unit_name[0] = '\0';
        tracked_rockets[free_slot].last_seen_ms = millis();
        return free_slot;
    }
    // All slots full — evict oldest
    uint32_t oldest_ms = UINT32_MAX;
    int oldest_idx = 0;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].last_seen_ms < oldest_ms) {
            oldest_ms = tracked_rockets[i].last_seen_ms;
            oldest_idx = i;
        }
    }
    tracked_rockets[oldest_idx].active = true;
    tracked_rockets[oldest_idx].rocket_id = rid;
    tracked_rockets[oldest_idx].unit_name[0] = '\0';
    tracked_rockets[oldest_idx].last_seen_ms = millis();
    tracked_rockets[oldest_idx].log_state = {};  // evicted slot: fresh transitions
    return oldest_idx;
}

// Snapshot the tracked slots for the aggregate log-close / freq-lock
// decisions (#381). Pure data out; bs_log_policy does the reasoning.
static void buildRocketViews(bs_log_policy::RocketView out[MAX_TRACKED_ROCKETS])
{
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        out[i].active            = tracked_rockets[i].active;
        out[i].last_seen_ms      = tracked_rockets[i].last_seen_ms;
        out[i].state             = tracked_rockets[i].log_state.last_state;
        out[i].inflight_entry_ms = tracked_rockets[i].log_state.inflight_entry_ms;
        out[i].freq_lock         = tracked_rockets[i].log_state.freq_lock;
        out[i].last_telem_ms     = tracked_rockets[i].last_telem_ms;
    }
}

// The single write point for the aggregate flight lock, so every edge is
// logged once with the site that caused it.  Pre-#835 the RX-path write was
// silent, which is much of why a permanently stuck lock went unnoticed: the
// only trace in a capture was a secondary "[TXN] Refused" or an
// recoveryEnd("flight locked") — the symptom, never the state.
//
// The edge guard wraps only the logging.  latch.update() ALWAYS runs, because
// its last_eval_ms stamp is what the read-side backstop measures against; a
// stamp that advanced only on an edge would expire the lock one window after
// the first INFLIGHT packet regardless of ongoing telemetry.
static void updateFreqLock(const bs_log_policy::RocketView* views,
                           uint32_t now_ms, const char* why)
{
    const bool was_flight = freq_lock_latch.flight;
    const bool was_retune = freq_lock_latch.retune;
    freq_lock_latch.update(views, MAX_TRACKED_ROCKETS, now_ms,
                           config::LOG_SILENCE_TIMEOUT_MS,
                           config::LOG_INFLIGHT_SAFETY_MS);

    if (freq_lock_latch.flight != was_flight)
    {
        if (freq_lock_latch.flight)
            ESP_LOGI(TAG, "[LOG] Flight freq lock SET (%s) — silence recovery "
                          "and fixed-channel heartbeats suppressed", why);
        else
            ESP_LOGW(TAG, "[LOG] Flight freq lock CLEARED (%s) — silence "
                          "recovery and heartbeats re-enabled", why);
    }
    if (freq_lock_latch.retune != was_retune)
    {
        if (freq_lock_latch.retune)
            ESP_LOGI(TAG, "[LOG] Retune lock SET (%s) — cmd-10 refused for the "
                          "flight", why);
        else
            ESP_LOGW(TAG, "[LOG] Retune lock CLEARED (%s) — cmd-10 reconfigure "
                          "accepted again", why);
    }
}

// State of the most recently heard tracked rocket (0 = INITIALIZATION when
// none). The single-rocket predecessor was the last_rocket_state global —
// "state of whichever packet arrived last" — which matches the hop-follow
// code's one-rocket-drives-the-hop semantics (#40/#41 caveat).
static uint8_t lastSeenRocketState()
{
    uint32_t newest_ms = 0;
    uint8_t  state     = 0;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].active &&
            tracked_rockets[i].log_state.have_seen_first &&
            tracked_rockets[i].last_seen_ms >= newest_ms) {
            newest_ms = tracked_rockets[i].last_seen_ms;
            state     = tracked_rockets[i].log_state.last_state;
        }
    }
    return state;
}

/// State of the FOCUSED rocket when it is tracked, else fall back to the
/// most recently heard one (#390).  Radio coordination (scan pauses) should
/// reason about the rocket that owns the hop, not whoever spoke last.
static uint8_t focusedRocketState()
{
    const int fslot = slotOfRid(effectiveFocusRid());
    if (fslot >= 0 && tracked_rockets[fslot].log_state.have_seen_first)
        return tracked_rockets[fslot].log_state.last_state;
    return lastSeenRocketState();
}
static char log_filename[64] = "";

// Time sync state (UTC time reference from phone via BLE)
static bool time_synced = false;
static uint32_t time_sync_millis = 0;
static uint16_t sync_year = 0;
static uint8_t  sync_month = 0, sync_day = 0;
static uint8_t  sync_hour = 0, sync_minute = 0, sync_second = 0;

// ── MAX17205G fuel gauge helpers ──
// The chip is read/configured through the TR_MAX17205G component.
// updateBattery() fans the latest readings into the global fields used by
// the BLE telemetry builder.

// ==========================================================================
// SECTION: Battery monitoring and charger FETs
// ==========================================================================
// Keep the new-board (BQ27Z746) battery protection FETs enabled. The gauge
// ships with FET_EN=0 (and a reset reverts to it), so we (re)enable whenever
// FET_EN has dropped and there is no active safety fault -- never overriding a
// real protection event. Also flags the "commanded on but not conducting" case
// (gate-drive / assembly fault, e.g. the EFC8811 open joint we hit). No-op on
// the MAX17205 board. Call right after bq_gauge.update() so the flags are fresh.
static void maintainBatteryFets()
{
    if (gauge_kind != GaugeKind::BQ27Z746) return;
    const TR_BQ27Z746_Data& d = bq_gauge.data();

    if (!d.fet_en)
    {
        // enableFetsIfNeeded() re-reads status and refuses while a fault is
        // active, so this is safe even mid-fault; it logs its own result.
        bq_gauge.enableFetsIfNeeded();
    }
    else if (!d.chg_fet_on || !d.dsg_fet_on)
    {
        static uint8_t throttle = 0;            // rate-limit (~every 15th cycle)
        if ((throttle++ % 15) == 0)
        {
            if (d.safety_active)
                ESP_LOGW(TAG, "BQ27Z746 safety protection holding a FET off (BattStatus=0x%04X)",
                         d.batt_status);
            else
                ESP_LOGW(TAG, "BQ27Z746 FET_EN=1, no fault, but CHG=%d DSG=%d -- FETs not conducting "
                              "(gate-drive/assembly?)", d.chg_fet_on, d.dsg_fet_on);
        }
    }
}

// ── Own-battery voltage via ADC (#835 item 2) ────────────────────────
//
// Boards with no fuel gauge (board_v3: the MAX17303 was deleted in 15da738,
// two weeks before the fab tag) measure the cell through a resistor divider
// instead. That commit said SoC "now reads a 1M/1M divider on GPIO1"; the
// firmware was never written, so bs_voltage/bs_soc sat at NaN forever.
//
// Every failure path here yields NaN rather than a number. A wrong battery
// reading is worse than an admitted unknown: the operator plans a recovery
// walk around it. In particular the esp-idf calibration curve is
// per-attenuation, so if the cali handle cannot be created we report nothing
// rather than scaling a raw count by a nominal full-scale and hoping.
static adc_oneshot_unit_handle_t batt_adc_unit = nullptr;
static adc_cali_handle_t         batt_adc_cali = nullptr;
static bool                      batt_adc_ready = false;

static void initBatteryAdc()
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id  = ADC_UNIT_1;             // GPIO1 = ADC1_CH0
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &batt_adc_unit) != ESP_OK) {
        ESP_LOGE(TAG, "[BATT] ADC unit init failed — battery voltage unavailable");
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = (adc_atten_t)ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_oneshot_config_channel(batt_adc_unit, ADC_CHANNEL_0, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "[BATT] ADC channel config failed — battery voltage unavailable");
        return;
    }

    // Curve fitting is the S3's scheme. Created FOR THIS attenuation — the
    // curve is per-atten and mixing them mis-scales every read silently.
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id  = ADC_UNIT_1;
    cali_cfg.atten    = (adc_atten_t)ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &batt_adc_cali) != ESP_OK) {
        ESP_LOGE(TAG, "[BATT] ADC calibration unavailable (no eFuse cal data?) — "
                      "reporting no voltage rather than an uncalibrated guess");
        return;
    }

    batt_adc_ready = true;
    ESP_LOGI(TAG, "[BATT] ADC ready: GPIO%d, divider %.3f, %d dB",
             config::BATT_VSENSE_GPIO, (double)config::BATT_VSENSE_DIVIDER,
             config::BATT_VSENSE_ATTEN_DB);
}

// Returns the cell terminal voltage, or NaN if it cannot be measured.
static float readBatteryVolts()
{
    if (!batt_adc_ready) return NAN;
    // Average a handful: the divider is high-impedance (1M||1M = 500k) and the
    // 100 nF at the pin is the only reservoir, so single reads are noisy.
    constexpr int kSamples = 8;
    int mv_sum = 0, taken = 0;
    for (int i = 0; i < kSamples; ++i) {
        int raw = 0;
        if (adc_oneshot_read(batt_adc_unit, ADC_CHANNEL_0, &raw) != ESP_OK) continue;
        int mv = 0;
        if (adc_cali_raw_to_voltage(batt_adc_cali, raw, &mv) != ESP_OK) continue;
        mv_sum += mv;
        ++taken;
    }
    if (taken == 0) return NAN;
    return ((float)mv_sum / (float)taken) * 0.001f * config::BATT_VSENSE_DIVIDER;
}
static void updateBattery()
{
    // constexpr, not #if: HAS_FUEL_GAUGE is a config struct member, and an
    // #if on an undefined macro silently evaluates to 0 — which would have
    // run the ADC path on the gauged boards too.
    if constexpr (!config::HAS_FUEL_GAUGE)
    {
        // No gauge on this board — the divider is the only source (#835 item 2).
        bs_voltage = readBatteryVolts();
        if (isnan(bs_voltage)) {
            bs_soc           = NAN;
            bs_soc_estimated = false;
        } else {
            // 1S: BT2 is a single 18650 on a BQ21040 linear charger, so the
            // estimator's per-cell curve applies directly. No current sense
            // on this path, so no IR compensation — pass 0.
            bs_soc           = bq_soc_estimator.update(bs_voltage, 1, 0.0f, 0.0f);
            bs_soc_estimated = true;
        }
        return;
    }

    if (!fuel_gauge_present) return;
    if (gauge_kind == GaugeKind::BQ27Z746)
    {
        bq_gauge.update();
        maintainBatteryFets();
        bs_voltage     = bq_gauge.voltage();
        bs_current     = bq_gauge.current();
        bs_temperature = bq_gauge.temperature();

        // #501: this gauge reported 0% SoC on a 4.17 V pack, with
        // RemainingCapacity=0 and FullChargeCapacity ~10x design. Impedance Track
        // derives all three by integrating current, and this part's CC Gain was
        // never calibrated (and GAUGE_EN ships at 0), so they are garbage — while
        // Voltage() is a direct ADC read and is correct. Prefer the gauge when it
        // is actually believable; otherwise estimate from voltage, which is at
        // least honest and gives us a low-battery warning.
        const bool gauge_ok = bs_battery_soc::gaugeSocPlausible(
            bq_gauge.gaugingEnabled(), bq_gauge.soc(), bq_gauge.fullCapacity(),
            (float)config::BATTERY_DESIGN_MAH, bs_voltage, config::BQ27Z746_CELLS);

        if (gauge_ok)
        {
            bs_soc           = bq_gauge.soc();
            bs_soc_estimated = false;
        }
        else
        {
            bs_soc = bq_soc_estimator.update(bs_voltage, config::BQ27Z746_CELLS,
                                             bs_current, config::BATTERY_INTERNAL_R_OHM);
            bs_soc_estimated = true;
        }

        // Say it once, and again only if the verdict flips — the operator needs to
        // know whether the number on the app is gauged or inferred.
        static int last_verdict = -1;
        const int verdict = gauge_ok ? 1 : 0;
        if (verdict != last_verdict)
        {
            if (gauge_ok)
                ESP_LOGI(TAG, "[BATT] BQ27Z746 gauge SoC is plausible — using it (%.0f%%)",
                         (double)bs_soc);
            else
                ESP_LOGW(TAG, "[BATT] BQ27Z746 SoC not trustworthy "
                              "(GAUGE_EN=%d, raw SOC=%.0f%%, FullCap=%.0f vs design %u mAh) — "
                              "falling back to voltage estimate (#501)",
                         bq_gauge.gaugingEnabled(), (double)bq_gauge.soc(),
                         (double)bq_gauge.fullCapacity(),
                         (unsigned)config::BATTERY_DESIGN_MAH);
            last_verdict = verdict;
        }
    }
    else if (gauge_kind == GaugeKind::MAX17303)
    {
        max17303_gauge.update();
        bs_voltage     = max17303_gauge.voltage();
        bs_soc         = max17303_gauge.soc();
        bs_current     = max17303_gauge.current();
        bs_temperature = max17303_gauge.temperature();

        // Protector observability (BQ FET saga lesson). No enable action
        // exists or is needed on this part — the protector runs the FETs —
        // but log WHY the battery path opened, once per transition.
        static bool last_fets_on = true;
        static uint16_t last_prot = 0;
        const bool fets_on = max17303_gauge.fetsOn();
        const uint16_t prot = max17303_gauge.protStatus();
        if (fets_on != last_fets_on || (prot != last_prot && prot != 0))
        {
            if (!fets_on || prot != 0)
                ESP_LOGW(TAG, "MAX17303 protector: FETs %s, ProtStatus=0x%04X",
                         fets_on ? "on" : "OFF", prot);
            else
                ESP_LOGI(TAG, "MAX17303 protector: FETs back on, faults clear");
        }
        last_fets_on = fets_on;
        last_prot = prot;
    }
    else
    {
        fuel_gauge.update();
        bs_voltage     = fuel_gauge.voltage();
        bs_soc         = fuel_gauge.soc();
        bs_current     = fuel_gauge.current();
        bs_temperature = fuel_gauge.temperature();
    }
}

// ==========================================================================
// SECTION: CSV logging: clock, file lifecycle, and writers
// ==========================================================================

// Days in each month (non-leap / leap year)
static const uint8_t days_in_month[2][12] = {
    {31,28,31,30,31,30,31,31,30,31,30,31},  // non-leap
    {31,29,31,30,31,30,31,31,30,31,30,31}   // leap
};

static bool isLeapYear(uint16_t y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

/// Compute current UTC time from the phone-synced reference + elapsed millis
static void getCurrentTime(uint16_t& year, uint8_t& month, uint8_t& day,
                           uint8_t& hour, uint8_t& minute, uint8_t& second)
{
    uint32_t elapsed_s = (millis() - time_sync_millis) / 1000;

    // Start from sync values
    year   = sync_year;
    month  = sync_month;
    day    = sync_day;
    hour   = sync_hour;
    minute = sync_minute;
    second = sync_second;

    // Add elapsed seconds with carry
    uint32_t total_sec = (uint32_t)second + elapsed_s;
    second = total_sec % 60;

    uint32_t total_min = (uint32_t)minute + total_sec / 60;
    minute = total_min % 60;

    uint32_t total_hr = (uint32_t)hour + total_min / 60;
    hour = total_hr % 24;

    uint32_t extra_days = total_hr / 24;
    while (extra_days > 0)
    {
        uint8_t dim = days_in_month[isLeapYear(year) ? 1 : 0][month - 1];
        if (day + extra_days <= dim)
        {
            day += extra_days;
            extra_days = 0;
        }
        else
        {
            extra_days -= (dim - day + 1);
            day = 1;
            month++;
            if (month > 12)
            {
                month = 1;
                year++;
            }
        }
    }
}

// Sync time → UTC seconds since the Unix epoch.  Relies on TZ=UTC0 being
// set in setup_bs() so mktime() interprets the broken-down time as UTC
// rather than local.  Returns 0 if no sync has happened yet.
static time_t syncedEpoch()
{
    if (!time_synced) return 0;
    struct tm tm_sync = {};
    tm_sync.tm_year = (int)sync_year - 1900;
    tm_sync.tm_mon  = (int)sync_month - 1;
    tm_sync.tm_mday = (int)sync_day;
    tm_sync.tm_hour = (int)sync_hour;
    tm_sync.tm_min  = (int)sync_minute;
    tm_sync.tm_sec  = (int)sync_second;
    return mktime(&tm_sync);
}

// Build "<mount>/lora_YYYYMMDD_HHMMSS.bin" for the given UTC epoch, walking
// _2/_3/.. suffixes on collision until a free path is found.  Mirrors the
// inline collision loop in startLogging() — extracted so the rename-on-
// time-sync path (#168) reuses the same naming + collision rules.
static void buildTimestampedLogPathForEpoch(time_t epoch,
                                             char* out_path, size_t out_len)
{
    struct tm tm_buf = {};
    gmtime_r(&epoch, &tm_buf);
    char basename[40];
    snprintf(basename, sizeof(basename),
             "lora_%04d%02d%02d_%02d%02d%02d.bin",
             tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    snprintf(out_path, out_len, "%s/%s", SD_MOUNT_POINT, basename);

    struct stat st;
    if (stat(out_path, &st) != 0) return;

    char base_no_ext[40];
    const size_t blen = strlen(basename);
    const size_t copy = (blen >= 4) ? (blen - 4) : blen;  // strip ".bin"
    memcpy(base_no_ext, basename, copy);
    base_no_ext[copy] = '\0';
    for (int suffix = 2; suffix < 100; suffix++)
    {
        snprintf(out_path, out_len, "%s/%s_%d.bin",
                 SD_MOUNT_POINT, base_no_ext, suffix);
        if (stat(out_path, &st) != 0) return;
    }
}

static uint16_t findNextFileNumber()
{
    uint16_t max_num = 0;
    DIR* dir = opendir(SD_MOUNT_POINT);
    if (!dir) return 1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        // #137: use the strict parser so timestamped siblings (e.g.
        // lora_20260509_164143.bin) don't get matched as "lora_NNN.bin"
        // with the leading digits truncated to the low 16 bits (= 9885).
        // Pre-fix, that quirk made the BS pick lora_9886.bin after every
        // no-time-sync boot whose SD already held timestamped flights.
        uint16_t num = 0;
        if (bs_log_policy::parseSequentialFilename(entry->d_name, num))
        {
            if (num > max_num) max_num = num;
        }
    }
    closedir(dir);
    return max_num + 1;
}

static void startLogging()
{
    if (logging_active)
    {
        fclose(log_file);
        log_file = nullptr;
        ESP_LOGI(TAG, "[LOG] Closed previous log: %s", log_filename);
    }

    // Build a filename relative to the mount point (log_filename stores VFS path)
    char basename[40];
    if (time_synced)
    {
        // Use timestamped filename (matches rocket's flight_YYYYMMDD_HHMMSS naming)
        uint16_t y; uint8_t mo, d, h, mi, s;
        getCurrentTime(y, mo, d, h, mi, s);
        snprintf(basename, sizeof(basename),
                 "lora_%04u%02u%02u_%02u%02u%02u.bin",
                 y, mo, d, h, mi, s);
    }
    else
    {
        // Fallback to sequential numbering if no time sync
        uint16_t num = findNextFileNumber();
        snprintf(basename, sizeof(basename), "lora_%03u.bin", num);
    }
    snprintf(log_filename, sizeof(log_filename), "%s/%s", SD_MOUNT_POINT, basename);

    // The LANDED-transition close (#107) closes and lets the next packet
    // auto-restart, which can land within the same wall-clock second as
    // the previous open — `fopen("w")` on the same path would truncate
    // the just-finished flight.  If the timestamped name already exists,
    // append _2/_3/.. so each flight stays on its own file.  Sequential
    // names are already unique because findNextFileNumber returns max+1.
    if (time_synced)
    {
        struct stat st;
        if (stat(log_filename, &st) == 0)
        {
            char base_no_ext[40];
            const size_t blen = strlen(basename);
            const size_t copy = (blen >= 4) ? (blen - 4) : blen;  // strip ".bin"
            memcpy(base_no_ext, basename, copy);
            base_no_ext[copy] = '\0';
            for (int suffix = 2; suffix < 100; suffix++)
            {
                snprintf(log_filename, sizeof(log_filename), "%s/%s_%d.bin",
                         SD_MOUNT_POINT, base_no_ext, suffix);
                if (stat(log_filename, &st) != 0) break;
            }
        }
    }

    log_file = fopen(log_filename, "wb");
    if (!log_file)
    {
        ESP_LOGE(TAG, "[LOG] Failed to open %s for writing! errno=%d (%s)",
                 log_filename, errno, strerror(errno));
        logging_active = false;
        return;
    }

    // Write CSV header.  The trailing block (next_ch..event) was added
    // for #105 lock-loss diagnostics: per-packet hop target, the freq the
    // BS was actually tuned to, the rocket's free-running seq and the
    // BS-derived gap to the last RX, plus an `event` column populated
    // for non-telemetry rows (hop_active / hop_inactive / hop_silence).
    // rocket_id (#381) attributes each telemetry row to its source rocket
    // #850 follow-up: this file is BINARY now, not CSV. Instead of a column
    // header it opens with a magic + version so a reader can identify it
    // without trusting the extension, and refuse a file it does not understand
    // rather than walking arbitrary bytes as records.
    //
    // The records that follow use the SAME framing as the rocket computer's log
    // (TR_I2C_Interface::packMessage — SOF/type/len/CRC16), so the same walker
    // reads both. That parity is the point: one binary format, one set of
    // tools, and the CSV is generated wherever it is actually needed.
    const uint8_t magic[8] = {
        'T', 'R', 'B', 'S', 'L', 'O', 'G',        // "TRBSLOG"
        BS_LOG_FORMAT_VERSION,
    };
    size_t hdr_written = fwrite(magic, 1, sizeof(magic), log_file);
    if (hdr_written != sizeof(magic))
    {
        // #384 (#329 residual): a full/wedged card at open used to fail the
        // header silently; every later consumer then sees a headerless file.
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] magic fwrite() failed (%u/%u, errno=%d %s)",
                 (unsigned)hdr_written, (unsigned)sizeof(magic),
                 errno, strerror(errno));
    }

    logging_active = true;
    log_start_ms = millis();
    log_last_write_ms = millis();
    log_last_flush_ms = millis();

    ESP_LOGI(TAG, "[LOG] Started logging: %s", log_filename);
}

static void stopLogging()
{
    if (!logging_active) return;

    if (log_file)
    {
        // #384 (#329 residual): fclose flushes the final buffered rows — the
        // LANDED-close tail of a flight. A card-full here was fully silent.
        if (fclose(log_file) != 0)
        {
            log_write_fail_count++;
            ESP_LOGW(TAG, "[LOG] fclose() failed — final buffered rows may be lost (errno=%d %s)",
                     errno, strerror(errno));
        }
        log_file = nullptr;
    }
    logging_active = false;
    log_last_open_attempt_ms = 0;  // allow the next packet to retry immediately (#107)

    // Disarm every rocket's INFLIGHT safety timer with the file it bounds
    // (#381) — a timer that expired while another rocket kept the log open
    // must not instantly close the NEXT session's file.
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        tracked_rockets[i].log_state.inflight_entry_ms = 0;
    }

    ESP_LOGI(TAG, "[LOG] Closed log: %s", log_filename);
}

// Called from the BLE time-sync command (#9) right after the sync clock is
// established.  If the currently-open log was opened pre-sync and therefore
// carries a sequential `lora_NNN.bin` name, rename it on disk to its proper
// `lora_YYYYMMDD_HHMMSS.bin` form using the wall-clock at which the file
// actually opened (computed as sync_time minus elapsed millis since open).
// Safe no-op when there's no open log, time isn't synced, or the file is
// already timestamped.  This is the root-cause fix for #168 — the 5/17/26
// test day produced `lora_002.bin` / `lora_003.bin` / `lora_004.bin` halves
// of three flights alongside their timestamped post-landing remnants
// because iOS BLE time-sync landed within a few seconds of each launch.
static void renameOpenLogIfSequential()
{
    if (!logging_active || !time_synced || log_file == nullptr) return;

    const char* slash = strrchr(log_filename, '/');
    const char* basename = slash ? (slash + 1) : log_filename;
    uint16_t parsed_num = 0;
    if (!bs_log_policy::parseSequentialFilename(basename, parsed_num)) {
        // Already timestamped (or shape we don't manage) — nothing to do.
        return;
    }

    const time_t epoch_sync = syncedEpoch();
    if (epoch_sync <= 0) return;
    const uint32_t back_s = (time_sync_millis > log_start_ms)
                              ? (time_sync_millis - log_start_ms) / 1000U
                              : 0U;
    const time_t epoch_open = epoch_sync - (time_t)back_s;

    char new_path[64];
    buildTimestampedLogPathForEpoch(epoch_open, new_path, sizeof(new_path));

    char old_path[64];
    strncpy(old_path, log_filename, sizeof(old_path) - 1);
    old_path[sizeof(old_path) - 1] = '\0';

    // Flush + close, rename on disk, then reopen append.  fflush alone
    // wouldn't be enough — FATFS needs the file closed before rename().
    fflush(log_file);
    if (!using_internal_flash) fsync(fileno(log_file));
    fclose(log_file);
    log_file = nullptr;

    if (rename(old_path, new_path) != 0)
    {
        ESP_LOGW(TAG, "[LOG] rename %s -> %s failed (errno=%d %s); reopening original",
                 old_path, new_path, errno, strerror(errno));
        log_file = fopen(old_path, "ab");
        if (!log_file)
        {
            ESP_LOGE(TAG, "[LOG] Could not reopen %s after failed rename — logging stopped",
                     old_path);
            logging_active = false;
        }
        return;
    }

    log_file = fopen(new_path, "ab");
    if (!log_file)
    {
        ESP_LOGE(TAG, "[LOG] Could not open %s after rename — logging stopped",
                 new_path);
        logging_active = false;
        return;
    }
    strncpy(log_filename, new_path, sizeof(log_filename) - 1);
    log_filename[sizeof(log_filename) - 1] = '\0';
    log_last_flush_ms = millis();
    ESP_LOGI(TAG, "[LOG] Time-sync arrived; renamed sequential log -> %s", log_filename);
}

static uint32_t log_write_count = 0;  // Tracks calls for periodic flash check


// Query the active log filesystem (SPIFFS or FAT) for total/used bytes.
// Returns true if a backend is mounted and the query succeeded.
static bool bsQueryStorage(uint64_t& total, uint64_t& used)
{
    total = 0; used = 0;
    if (using_internal_flash)
    {
        size_t t = 0, u = 0;
        if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &t, &u) != ESP_OK) return false;
        total = t; used = u;
        return true;
    }
    // Ask by MOUNT POINT, not by FAT drive number. This used to be
    // f_getfree("0:"), which is only correct as long as our volume happens to
    // land on pdrv 0 — esp_vfs_fat_nand_mount() takes whatever ff_diskio_get_drive()
    // hands it, so nothing guarantees that, and a second FAT volume would make
    // this silently report the WRONG volume's free space. esp_vfs_fat_info()
    // resolves base_path -> drive itself, which also drops the manual
    // cluster/sector arithmetic (the external NAND mounts 4096-byte sectors, so
    // the old hardcoded-512 version of this under-reported by 8x).
    uint64_t total_b = 0, free_b = 0;
    if (esp_vfs_fat_info(SD_MOUNT_POINT, &total_b, &free_b) != ESP_OK) return false;
    total = total_b;
    used  = (total_b > free_b) ? (total_b - free_b) : 0;
    return true;
}

// ----------------------------------------------------------------------------
// Binary LoRa RX record
// ----------------------------------------------------------------------------
// Logs the bytes we actually received, plus the three things only the base
// station knows: when it arrived, how strong it was, and which channel it
// landed on. Everything else the old CSV carried is either inside the frame or
// derived from it, so it is recomputed by whatever renders the log.
//
// Takes the RAW frame, not the decoded LoRaDataSI, deliberately. With FAST and
// SLOW interleaved, the accumulator is a running picture rather than a record
// of this packet — writing it would log the same forward-filled values over and
// over and lose which fields actually arrived when. The raw bytes are the
// evidence; the picture can always be rebuilt from them.
static void logLoRaPacket(const uint8_t* frame, size_t frame_len,
                          float rssi, float snr, float rx_freq_mhz)
{
    if (!logging_active || log_file == nullptr) return;
    if (frame == nullptr || frame_len == 0) return;

    // Periodic storage usage check (every 100 writes)
    if (++log_write_count % 100 == 0)
    {
        uint64_t total = 0, used = 0;
        bsQueryStorage(total, used);
        if (total > 0 && used > (total * 9 / 10))
        {
            ESP_LOGW(TAG, "[LOG] %s nearly full! %llu/%llu bytes (%.0f%%)",
                     using_internal_flash ? "Internal flash" : using_external_flash ? "External NAND" : "SD card",
                     (unsigned long long)used, (unsigned long long)total,
                     (double)used * 100.0 / (double)total);
        }
    }

    BsLoRaRxHeader hdr{};
    hdr.time_ms = millis() - log_start_ms;
    // NaN is what the radio reports when it has no reading; it must not become
    // a plausible-looking 0 dBm in the log.
    hdr.rssi_dbm_x10 = (rssi == rssi) ? (int16_t)lroundf(rssi * 10.0f) : BS_RSSI_UNKNOWN;
    hdr.snr_db_x10   = (snr  == snr)  ? (int16_t)lroundf(snr  * 10.0f) : BS_RSSI_UNKNOWN;
    hdr.rx_freq_hz   = (uint32_t)lroundf(rx_freq_mhz * 1e6f);

    uint8_t payload[sizeof(BsLoRaRxHeader) + SIZE_OF_LORA_FAST];
    if (frame_len > SIZE_OF_LORA_FAST) return;   // cannot happen; never truncate silently
    memcpy(payload, &hdr, sizeof(hdr));
    memcpy(payload + sizeof(hdr), frame, frame_len);

    uint8_t rec[MAX_FRAME];
    size_t  rec_len = 0;
    if (!TR_I2C_Interface::packMessage(BS_LORA_RX_MSG, payload,
                                       sizeof(hdr) + frame_len,
                                       rec, sizeof(rec), rec_len))
    {
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] packMessage(BS_LORA_RX) failed");
        return;
    }

    const size_t wrote = fwrite(rec, 1, rec_len, log_file);
    if (wrote != rec_len)
    {
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] fwrite() short (%u/%u, errno=%d %s)",
                 (unsigned)wrote, (unsigned)rec_len, errno, strerror(errno));
    }

    log_last_write_ms = millis();
}

// ----------------------------------------------------------------------------
// Binary hop-event record (#105)
// ----------------------------------------------------------------------------
// Interleaved with the RX records in the same file and the same framing, so a
// single pass over the log sees telemetry and events in arrival order — the
// property the old CSV got by padding event rows out to the full column count.
// Binary needs no padding: the record type distinguishes them.  The text is
// free-form; downstream scripts match on it to plot session timelines and loss
// histograms.
//
// Skipped when logging_active=false so we don't open a file just to record
// an event with no surrounding telemetry.  Hop transitions still go to
// ESP_LOG via the existing logging at the call site, so nothing is lost.
static void logHopEvent(const char* event_str, float rx_freq_mhz)
{
    if (!logging_active || log_file == nullptr || event_str == nullptr) return;

    size_t text_len = strnlen(event_str, BS_EVENT_TEXT_MAX);

    BsEventHeader hdr{};
    hdr.time_ms    = millis() - log_start_ms;
    hdr.rx_freq_hz = (uint32_t)lroundf(rx_freq_mhz * 1e6f);
    hdr.text_len   = (uint8_t)text_len;

    uint8_t payload[sizeof(BsEventHeader) + BS_EVENT_TEXT_MAX];
    memcpy(payload, &hdr, sizeof(hdr));
    memcpy(payload + sizeof(hdr), event_str, text_len);

    uint8_t rec[MAX_FRAME];
    size_t  rec_len = 0;
    if (!TR_I2C_Interface::packMessage(BS_EVENT_MSG, payload,
                                       sizeof(hdr) + text_len,
                                       rec, sizeof(rec), rec_len))
    {
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] packMessage(BS_EVENT) failed");
        return;
    }

    const size_t wrote = fwrite(rec, 1, rec_len, log_file);
    if (wrote != rec_len)
    {
        log_write_fail_count++;   // #384: event records count toward #329 stats
        ESP_LOGW(TAG, "[LOG] fwrite(event) short (%u/%u)",
                 (unsigned)wrote, (unsigned)rec_len);
    }
    log_last_write_ms = millis();
}

// Returns the radio's currently-tuned RX frequency for the hop diagnostics
// CSV — i.e. the channel the BS was sitting on when this packet arrived
// (or when this hop event fired).  Single source of truth so callers don't
// have to re-derive lora_freq_mhz vs loraChannelMHz(...) inline.
static inline float currentRxFreqMHz()
{
    if (hop_active_) {
        const float f = loraChannelMHz(lora_bw_khz, hop_idx_);
        if (f > 0.0f) return f;
    }
    return lora_freq_mhz;
}

// Record the operator's phone fix as an EVENT row (BLE_BS_CMD_SET_PHONE_FIX).
//
// This is the base station's position, which nothing else in the system
// captures: the BS has no GNSS, so every lat/lon in this CSV is the ROCKET's
// relayed position. Range -- the whole point of a range test -- was therefore
// unreconstructable from the logs alone.
//
// Emitted as an EVENT row rather than a new column because the schema already
// has the mechanism (state == "EVENT", free-text `event`), every existing
// parser already skips those rows, and the fix arrives on its own cadence
// rather than per packet. Range is then a one-line offline computation
// against the rocket lat/lon on the surrounding telemetry rows.
//
// The event text is space-separated: `event` is a CSV field and a comma here
// would silently shift every column to its right.
static void logPhoneFixEvent(int32_t lat_e7, int32_t lon_e7,
                             int16_t alt_m, uint8_t h_acc_m)
{
    if (!logging_active || log_file == nullptr) return;
    char ev[96];
    snprintf(ev, sizeof(ev),
             "phone_fix lat=%.7f lon=%.7f alt=%d hacc=%u",
             (double)lat_e7 * 1e-7, (double)lon_e7 * 1e-7,
             (int)alt_m, (unsigned)h_acc_m);
    logHopEvent(ev, currentRxFreqMHz());
}

// ==========================================================================
// SECTION: BLE file operations (list, delete, download)
// ==========================================================================

// NOTE: All SD card operations (log, list, delete) run on the main loop task.
// No mutex needed as long as this guarantee holds. If BLE callbacks move to
// a separate task, add a mutex around all SD card access.
static void handleFileListCommand()
{
    uint8_t page = ble_app.getFileListPage();

    // Stream the WHOLE directory past a window that keeps only the N greatest
    // names (#835 item 5).  This used to read into `FileEntry entries[64]` and
    // stop at 64 BEFORE sorting; readdir returns on-disk slot order, so the
    // entries it never reached were the newest logs and the just-recorded
    // flight became unlistable and undownloadable once the directory held 64+
    // files.  N is sized to the requested page, so page 0 is correct for any
    // directory size and only paging depth is bounded.
    const size_t want = bs_file_list::windowFor(page, config::FILES_PER_PAGE,
                                                config::FILE_LIST_MAX_WINDOW);
    if (want == 0)
    {
        // Past the deepest page we will build a window for.  An empty page is
        // the "list ended" signal both apps already stop on.
        ESP_LOGW(TAG, "[BLE] File list page %u beyond max window (%u entries)",
                 (unsigned)page, (unsigned)config::FILE_LIST_MAX_WINDOW);
        ble_app.sendFileList(String("[]"));
        return;
    }

    std::unique_ptr<bs_file_list::Entry[]> storage(
        new (std::nothrow) bs_file_list::Entry[want]);
    if (!storage)
    {
        ESP_LOGE(TAG, "[BLE] File list page %u: out of memory for %u entries",
                 (unsigned)page, (unsigned)want);
        ble_app.sendFileList(String("[]"));
        return;
    }
    bs_file_list::TopNames window(storage.get(), want);

    DIR* dir = opendir(SD_MOUNT_POINT);
    if (dir)
    {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            // Skip directories
            if (entry->d_type == DT_DIR) continue;

            const char* fname = entry->d_name;

            // Derive the active basename from log_filename for comparison
            const char* active_basename = log_filename;
            // log_filename is e.g. "/sdcard/lora_001.bin", strip mount prefix
            if (strncmp(active_basename, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) == 0)
                active_basename += strlen(SD_MOUNT_POINT) + 1; // skip "/sdcard/"

            // Skip the currently active log file
            if (logging_active && strcmp(fname, active_basename) == 0)
                continue;

            window.offer(fname);
        }
        closedir(dir);
    }

    // Build JSON.  stat() only the rows we actually emit -- the old code
    // stat()ed every file it collected, which on FAT is a directory walk each.
    const size_t start = (size_t)page * config::FILES_PER_PAGE;
    String json = "[";
    size_t emitted = 0;
    for (size_t i = start; i < window.count() && emitted < config::FILES_PER_PAGE; ++i)
    {
        const char* name = window.at(i);
        if (emitted > 0) json += ",";

        char fullpath[64];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", SD_MOUNT_POINT, name);
        struct stat st;
        const uint32_t fsize = (stat(fullpath, &st) == 0) ? (uint32_t)st.st_size : 0;

        json += "{\"name\":\"";
        json += name;
        json += "\",\"size\":";
        json += std::to_string(fsize);
        json += "}";
        ++emitted;
    }
    json += "]";

    ble_app.sendFileList(json);
    ESP_LOGI(TAG, "[BLE] Sent file list page %u: %u files (total %u)",
             (unsigned)page, (unsigned)emitted, (unsigned)window.total());
}

static void handleDeleteCommand()
{
    String filename = ble_app.getDeleteFilename();
    if (filename.length() == 0) return;

    // Don't delete the active log file — derive basename from log_filename
    const char* active = log_filename;
    if (strncmp(active, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) == 0)
        active += strlen(SD_MOUNT_POINT) + 1;
    if (logging_active && filename == active)
    {
        ESP_LOGW(TAG, "[BLE] Cannot delete active log: %s", filename.c_str());
        return;
    }

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename.c_str());
    if (remove(path) == 0)
    {
        ESP_LOGI(TAG, "[BLE] Deleted: %s", filename.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "[BLE] Delete failed: %s", filename.c_str());
    }

    // Send updated file list (page 0)
    handleFileListCommand();
}

// ---------------------------------------------------------------------------
// Cooperative BLE file download (#380).
//
// The download used to run to completion inside the command dispatch — a
// synchronous fread/sendFileChunk/delay(15 ms) loop on the single bs_loop
// task. A 120 KB CSV blocked the loop ~11 s (a multi-MB log for minutes),
// starving telemetry RX, uplink retries, heartbeats, the LoRa transaction /
// recovery state machines, and CSV flushes; a >5 min download silence-closed
// the active log. It is now a state machine serviced once per loop iteration
// (mirroring coord_scan_state_): startDownload() opens the file and captures
// state, serviceDownload() emits at most one chunk per pass, gated on the
// same 15 ms wall-time spacing the BLE notify path needs to drain its mbufs
// (bs_download_policy::mayEmitChunk — the loop runs ~1 ms, so pacing must be
// wall-time, not loop cadence). Throughput is unchanged (~11 KB/s); the loop
// keeps servicing everything else between chunks.
static FILE*    dl_file_          = nullptr;
static uint32_t dl_file_size_     = 0;
static uint32_t dl_offset_        = 0;
static uint32_t dl_last_chunk_ms_ = 0;
static char     dl_name_[40]      = {0};  // for logging only

static void finishDownload(const char* outcome)
{
    ESP_LOGI(TAG, "[BLE] Download %s: %s (%lu of %lu bytes sent)",
             outcome, dl_name_, (unsigned long)dl_offset_,
             (unsigned long)dl_file_size_);
    if (dl_file_ != nullptr) fclose(dl_file_);
    dl_file_          = nullptr;
    dl_file_size_     = 0;
    dl_offset_        = 0;
    dl_last_chunk_ms_ = 0;
    dl_name_[0]       = '\0';
}

static void startDownload()
{
    // getDownloadFilename() clears on read — capture everything now; the
    // per-iteration service below must not touch the BLE pending slot.
    String filename = ble_app.getDownloadFilename();
    if (filename.length() == 0) return;

    if (dl_file_ != nullptr)
    {
        // A new request supersedes an in-flight transfer (same "latest wins"
        // semantics as the uplink slot). The app only runs one download UI.
        ESP_LOGW(TAG, "[BLE] New download request supersedes active transfer");
        finishDownload("superseded");
    }

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename.c_str());
    FILE* f = fopen(path, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "[BLE] Download failed, file not found: %s", filename.c_str());
        // #526: EOF|ABORT, not a bare EOF.  A bare EOF is indistinguishable
        // from a successful zero-byte transfer, and the app saves it and
        // reports success — so a log rotated or bulk-deleted between the
        // listing and the request lands as a 0-byte "downloaded" file.
        (void)ble_app.sendFileChunk(0, nullptr, 0, true, /*abort=*/true);
        return;
    }

    // Size measured at open bounds the whole transfer — a still-growing active
    // log downloads the bytes that existed at request time and terminates.
    fseek(f, 0, SEEK_END);
    dl_file_size_ = (uint32_t)ftell(f);
    fseek(f, 0, SEEK_SET);

    dl_file_          = f;
    dl_offset_        = 0;
    dl_last_chunk_ms_ = 0;
    snprintf(dl_name_, sizeof(dl_name_), "%s", filename.c_str());

    ESP_LOGI(TAG, "[BLE] Starting download: %s (%lu bytes)",
             dl_name_, (unsigned long)dl_file_size_);
}

static void serviceDownload()
{
    if (dl_file_ == nullptr) return;

    if (!ble_app.isConnected())
    {
        // Abort without an EOF chunk, matching the old behavior — the peer is
        // gone, and a reconnecting app re-requests from scratch.
        ESP_LOGW(TAG, "[BLE] Disconnected during download, aborting");
        finishDownload("aborted");
        return;
    }

    const uint32_t now = millis();
    if (!bs_download_policy::mayEmitChunk(now, dl_last_chunk_ms_,
                                          config::BLE_CHUNK_DELAY_MS))
    {
        return;  // BLE notify path still draining — try next iteration
    }

    const auto plan = bs_download_policy::nextChunk(
        dl_file_size_, dl_offset_, config::BLE_FILE_CHUNK_SIZE);

    uint8_t chunk_buf[config::BLE_FILE_CHUNK_SIZE];
    size_t got = 0;
    if (plan.read_len > 0)
    {
        got = fread(chunk_buf, 1, plan.read_len, dl_file_);
        if (got == 0)
        {
            // Unexpected short read (SD error / file truncated underneath us).
            // Terminate with an empty EOF chunk so the app doesn't hang — the
            // old loop just stopped here without ever flagging EOF.
            ESP_LOGE(TAG, "[BLE] Download read failed at offset %lu of %s",
                     (unsigned long)dl_offset_, dl_name_);
            (void)ble_app.sendFileChunk(dl_offset_, nullptr, 0, true, /*abort=*/true);
            finishDownload("failed");
            return;
        }
    }

    const bool eof = plan.eof && (got == plan.read_len);
    if (!ble_app.sendFileChunk(dl_offset_, got ? chunk_buf : nullptr, got, eof))
    {
        // sendFileChunk only returns false once the whole reliable-send
        // backpressure budget is spent (#524), i.e. the chunk is genuinely
        // lost.  Its contract requires the caller to abort rather than carry
        // on: the app appends chunks sequentially and never inspects the
        // offset field, so continuing past a dropped chunk splices the file
        // silently and hands over a CSV with an invisible hole in the middle
        // that still parses.  The out_computer and the mini already do this;
        // the base station was the one downloader that did not (#827).
        ESP_LOGE(TAG, "[BLE] Download chunk send failed at offset %lu of %s, aborting",
                 (unsigned long)dl_offset_, dl_name_);
        (void)ble_app.sendFileChunk(dl_offset_, nullptr, 0, true, /*abort=*/true);
        finishDownload("failed");
        return;
    }
    dl_offset_ += got;
    dl_last_chunk_ms_ = now;

    if (eof) finishDownload("complete");
}

// ==========================================================================
// SECTION: BLE telemetry frame and console output
// ==========================================================================

static const char* rocketStateToString(uint8_t state)
{
    switch (state)
    {
        case 0:  return "INIT";
        case 1:  return "READY";
        case 2:  return "PRELAUNCH";
        case 3:  return "INFLIGHT";
        case 4:  return "LANDED";
        case 5:  return "MAG_CAL";
        default: return "UNKNOWN";
    }
}

static void buildBLETelemetry(const LoRaDataSI& lora, float rssi, float snr,
                              double lat_deg, double lon_deg, double gnss_alt_m,
                              TR_BLE_To_APP::TelemetryData& out)
{
    memset(&out, 0, sizeof(out));

    // Power
    out.soc = lora.soc;
    out.current = lora.current;
    out.voltage = lora.voltage;

    // GPS (pre-computed lat/lon)
    out.latitude = lat_deg;
    out.longitude = lon_deg;
    out.gdop = lora.pdop;
    out.num_sats = (int)lora.num_sats;

    // Sensor health scorecard bitfield (#303) — relayed straight from the
    // FC/OC LoRa downlink; iOS unpacks the 2-bit-per-sensor states.
    out.sensor_health = lora.sensor_health;

    // State
    out.state = rocketStateToString(lora.rocket_state);

    // Camera recording (from LoRa downlink flags)
    out.camera_recording = lora.camera_recording;
    // Rocket logging state (actual, from LoRa downlink).  #390: read the
    // packet's own flag, not the last_known_* global — the global tracks
    // whichever rocket spoke last, which is wrong for a re-push of a
    // different (focused) rocket's cached frame.
    out.logging_active = lora.logging_active;
    // #835 item 9: without this the memset() above left sim_active=false on
    // EVERY relayed frame — an affirmative "real flight", not an absence.  A
    // pad sim watched through the base station looked identical to a real one,
    // including in the CSV the flight-report tooling reads afterwards.
    out.sim_active     = lora.sim_active;
    // Surface the BS log basename as a heartbeat so the operator can
    // confirm logging is live before each flight (#107).  The rocket-side
    // filename isn't shipped over LoRa, so this slot is otherwise unused
    // when the iOS app is connected via the base station.
    if (logging_active)
    {
        const char* slash = strrchr(log_filename, '/');
        out.active_file = slash ? slash + 1 : log_filename;
    }
    else
    {
        out.active_file = "";
    }

    // Base station's own CSV logging state (shown as separate indicator)
    out.bs_logging_active = logging_active;
    // Seconds remaining until silence-timeout fires (#137 follow-up).  When
    // the log is open, iOS renders a countdown next to the Base Stn Log
    // badge so the operator can see the auto-close approaching during long
    // post-flight idles.  We sit-clamp at 0 rather than going negative
    // because the close runs out of band with this packet build.
    if (logging_active)
    {
        const uint32_t age_ms   = millis() - log_last_write_ms;
        const uint32_t remain_ms = (age_ms >= config::LOG_SILENCE_TIMEOUT_MS)
                                 ? 0
                                 : (config::LOG_SILENCE_TIMEOUT_MS - age_ms);
        out.bs_log_silence_remaining_s = (uint16_t)(remain_ms / 1000U);
    }
    else
    {
        out.bs_log_silence_remaining_s = 0xFFFF;  // sentinel — omit from JSON
    }

    // Data rates -- not applicable for base station
    out.rx_kbs = NAN;
    out.wr_kbs = NAN;
    out.frames_rx = 0;
    out.frames_drop = 0;

    // Performance
    out.max_alt_m = lora.max_alt;
    out.max_speed_mps = lora.max_speed;

    // Altitude
    out.pressure_alt = lora.pressure_alt;
    out.altitude_rate = lora.altitude_rate;
    out.gnss_alt = isnan(gnss_alt_m) ? NAN : (float)gnss_alt_m;

    // #191: EKF ENU velocity + burnout (ascent landing prediction)
    out.vel_e = lora.vel_e;
    out.vel_n = lora.vel_n;
    out.vel_u = lora.vel_u;
    out.burnout_flag = lora.burnout_detected;

    // #390: board→rocket orientation from flags2 (0 = not reported)
    out.imu_orient_code = lora.imu_orient_code;
    out.imu_orient_mode = lora.imu_orient_mode;

    // IMU -- low-g only (high-g not in LoRa packet)
    out.low_g_x = lora.acc_x;
    out.low_g_y = lora.acc_y;
    out.low_g_z = lora.acc_z;
    out.high_g_x = NAN;
    out.high_g_y = NAN;
    out.high_g_z = NAN;
    out.gyro_x = lora.gyro_x;
    out.gyro_y = lora.gyro_y;
    out.gyro_z = lora.gyro_z;

    // Attitude
    out.roll  = lora.roll;
    out.pitch = lora.pitch;
    out.yaw   = lora.yaw;
    out.q0    = lora.q0;
    out.q1    = lora.q1;
    out.q2    = lora.q2;
    out.q3    = lora.q3;

    // LoRa signal quality
    out.rssi = rssi;
    out.snr = snr;

    // #150: hop-state + nid-drop surface (rides the droppable tier-3 tail
    // of the telemetry JSON; both keys are omitted entirely in fixed mode
    // with a healthy nid).
    out.hop_active      = hop_active_;
    out.hop_channel_idx = hop_idx_;
    // Only surface nid drops while they're recent — see the counter's
    // declaration for why (lifetime count kept for logs/stats).
    out.netid_drops     = (lora_netid_mismatch_drops > 0 &&
                           (millis() - lora_netid_last_drop_ms) < NETID_DROP_REPORT_WINDOW_MS)
                          ? lora_netid_mismatch_drops : 0;
    // #570: same recency treatment for size-mismatch drops ("szd") — the
    // mixed-flash trap the netid surface doesn't catch.
    out.size_drops      = (lora_size_mismatch_drops > 0 &&
                           (millis() - lora_size_last_drop_ms) < NETID_DROP_REPORT_WINDOW_MS)
                          ? lora_size_mismatch_drops : 0;

    // Base station battery (local measurement)
    out.bs_soc = bs_soc;
    out.bs_voltage = bs_voltage;
    out.bs_current = bs_current;

    // Flight event flags (from LoRa packet)
    out.launch_flag       = lora.launch_flag;
    out.vel_u_apogee_flag = lora.vel_u_apogee_flag;
    out.alt_apogee_flag   = lora.alt_apogee_flag;
    out.alt_landed_flag   = lora.alt_landed_flag;

    // Source rocket identity (for app-side multi-rocket demux)
    out.source_rocket_id  = lora.rocket_id;
    out.source_unit_name  = nullptr;  // Caller sets from tracker if available
}

static uint32_t last_telem_print_ms = 0;
static uint8_t  last_printed_state  = 0xFF;

static void printTelemetry(const LoRaDataSI& data, float rssi, float snr,
                           double lat_deg, double lon_deg, double alt_m)
{
    const uint32_t now = millis();
    const bool state_changed = (data.rocket_state != last_printed_state);
    const bool timer_elapsed = (now - last_telem_print_ms >= 5000);

    if (!state_changed && !timer_elapsed) return;

    last_telem_print_ms = now;
    last_printed_state  = data.rocket_state;

    // next_channel_idx is logged so we can verify the framing byte is
    // crossing the wire correctly before phase 2 starts using it.  0xFF
    // ("--") is the phase-1 sentinel meaning "no hop"; anything else is
    // already a hop intent.
    char hop_str[8];
    if (data.next_channel_idx == LORA_NEXT_CH_NO_HOP)
        snprintf(hop_str, sizeof(hop_str), "--");
    else
        snprintf(hop_str, sizeof(hop_str), "%u", (unsigned)data.next_channel_idx);

    // #504: spd= used to be fed data.max_speed, so a landed rocket read 99 m/s —
    // alarming to watch, and flatly contradicted by the CSV (speed=1.0,
    // max_speed=99.0). Print the live speed under spd=, and keep the peak under
    // its own max= label, which is what you actually want side by side on the bench.
    ESP_LOGI(TAG, "[RX] %s | alt=%.0fm spd=%.1fm/s max=%.1fm/s | %.0fdBm SNR=%.1f | sats=%u | %.2fV %.0f%% | nextCh=%s",
             rocketStateToString(data.rocket_state),
             (double)data.pressure_alt,
             (double)data.speed,
             (double)data.max_speed,
             (double)rssi,
             (double)snr,
             (unsigned)data.num_sats,
             (double)data.voltage,
             (double)data.soc,
             hop_str);
}

static void printStats()
{
    if (!config::DEBUG)
    {
        return;
    }

    const uint32_t now = millis();
    if ((now - last_stats_ms) < config::STATS_PERIOD_MS)
    {
        return;
    }
    const uint32_t dt = now - last_stats_ms;
    last_stats_ms = now;

    TR_LoRa_Comms::Stats ls = {};
    lora_comms.getStats(ls);

    const uint32_t rx_delta = ls.rx_count - last_rx_count;
    const float rx_hz = (dt > 0) ? ((float)rx_delta * 1000.0f / (float)dt) : 0.0f;
    last_rx_count = ls.rx_count;

    // "Last pkt N ms ago" was misleading when N == 0 (looked like "just
    // received" but actually meant "never received").  Print "never" in
    // that case so a fresh-BS-no-rocket scenario is unambiguous in logs.
    char last_pkt_str[24];
    if (last_packet_ms > 0)
    {
        snprintf(last_pkt_str, sizeof(last_pkt_str), "%lu ms ago",
                 (unsigned long)(now - last_packet_ms));
    }
    else
    {
        snprintf(last_pkt_str, sizeof(last_pkt_str), "never");
    }

    // #520: dup-rx = stale DIO1 latches caught before they could re-read the
    // radio buffer and emit a byte-identical duplicate packet. Nonzero is
    // EXPECTED and healthy — it means the race still occurs and is being caught.
    ESP_LOGI(TAG, "[STATS] RX: %lu pkts (%.1f Hz) | CRC fail: %lu | len drop: %lu | dup-rx caught: %lu | low-SNR drop: %lu | netid drop: %lu | log wr-fail: %lu | ISR: %lu | rx_mode: %d | TX wdog: %lu | Last RSSI: %.0f dBm SNR: %.1f dB | Last pkt %s",
             (unsigned long)ls.rx_count,
             (double)rx_hz,
             (unsigned long)ls.rx_crc_fail,
             (unsigned long)ls.rx_len_drop,
             (unsigned long)ls.rx_spurious,
             (unsigned long)lora_low_snr_drops,
             (unsigned long)lora_netid_mismatch_drops,
             (unsigned long)log_write_fail_count,
             (unsigned long)ls.isr_count,
             (int)ls.rx_mode,
             (unsigned long)ls.tx_watchdog_fires,
             (double)ls.last_rssi,
             (double)ls.last_snr,
             last_pkt_str);

    // Uplink airtime (#506).  The radio is half-duplex, so uplink time-on-air is
    // time we are deaf — and it used to be silently charged to the downlink loss
    // counters above, making a self-inflicted hole look like an RF problem. txwin
    // = retries held back to land in the gap between telemetry packets; forced =
    // the liveness backstop transmitting over the downlink anyway (want ~0).
    ESP_LOGI(TAG, "[UPLINK] TX: %lu sent, %lu ms airtime | txwin: %lu deferred, %lu forced | rocket cadence: %u ms%s",
             (unsigned long)uplink_tx_count,
             (unsigned long)uplink_tx_airtime_ms,
             (unsigned long)uplink_defer_count,
             (unsigned long)uplink_defer_override,
             (unsigned)rx_cadence.periodMs(),
             rx_cadence.valid() ? "" : " (est, not yet learned)");

    // Base-station battery.  The (est) marker says the SoC came from the voltage
    // curve rather than the gauge's coulomb count (#501) — without it, a plausible
    // number gives no hint that the gauge underneath it is untrustworthy.
    if (fuel_gauge_present)
    {
        ESP_LOGI(TAG, "[BATT] %.2f V | %.0f%% SoC (%s) | %.0f mA | %.1f C",
                 (double)bs_voltage, (double)bs_soc,
                 bs_soc_estimated ? "est from voltage" : "gauge",
                 (double)bs_current, (double)bs_temperature);
    }

    // Hop diagnostics (#105).  hop_active=Y/N is the live link state; the
    // session counters reflect the *current* session and reset on each
    // hop_inactive / hop_silence event, so a "0 / 0" reading mid-flight
    // means we just lost lock.  lifetime totals let the operator spot a
    // run with abnormally many silence events at a glance.  drift_repush
    // counts auto re-pushes of cmd 15 in response to BS/OC mask divergence.
    ESP_LOGI(TAG, "[STATS] HOP: active=%c idx=%u session(pkts=%lu loss=%lu) lifetime(silence=%lu loss=%lu drift_repush=%lu)",
             hop_active_ ? 'Y' : 'N',
             (unsigned)hop_idx_,
             (unsigned long)hop_session_total_pkts,
             (unsigned long)hop_session_observed_loss,
             (unsigned long)hop_silence_events_count,
             (unsigned long)lora_total_observed_loss,
             (unsigned long)chset_drift_repush_count);
}

// ==========================================================================
// SECTION: Config readback to the app
// ==========================================================================

static void sendCurrentConfig()
{
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
             "{\"type\":\"config\""
             ",\"sb1\":%d,\"shz\":%d,\"smn\":%d,\"smx\":%d"
             ",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f"
             ",\"pmn\":%.1f,\"pmx\":%.1f"
             ",\"sen\":%s"
             ",\"lf\":%.1f,\"lsf\":%u,\"lbw\":%.0f,\"lcr\":%u,\"lpw\":%d"
             ",\"lhd\":%s,\"lhdw\":%u}",
             (int)cfg_servo_bias1, (int)cfg_servo_hz,
             (int)cfg_servo_min, (int)cfg_servo_max,
             (double)cfg_pid_kp, (double)cfg_pid_ki, (double)cfg_pid_kd,
             (double)cfg_pid_min, (double)cfg_pid_max,
             cfg_servo_enabled ? "true" : "false",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
             lora_hop_disabled ? "true" : "false",
             // #150: airtime-derived hop dwell; 0 = hopping unavailable at
             // this preset (the app greys the picker option).
             (unsigned)currentHopDwell());
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        ESP_LOGW(TAG, "[CFG] Config JSON truncated! (%d bytes, buf=%u)", n, (unsigned)sizeof(buf));
    }
    String j(buf);
    ble_app.sendConfigJSON(j);
    ESP_LOGI(TAG, "[CFG] Sent config readback to app");

    // #570: the fixed delay(50) that used to sit here was legacy pre-#524
    // pacing. It parked the single bs_loop task for 50 ms per config command
    // (no RX decode, no uplink retries, no heartbeat, no CSV flush) — a
    // connect-time burst chained several of those. notify_data now applies
    // MTU-aware backpressure itself, so back-to-back sends are safe.

    // Message 2: device identity ("config_identity" type)
    const esp_app_desc_t* app_desc = esp_app_get_description();
    const char* fw_ver = (app_desc && app_desc->version[0]) ? app_desc->version : "unknown";
    char id_buf[192];
    snprintf(id_buf, sizeof(id_buf),
             "{\"type\":\"config_identity\""
             ",\"uid\":\"%s\""
             ",\"un\":\"%s\""
             ",\"nid\":%u"
             ",\"dt\":\"%s\""
             ",\"fw\":\"%s\"}",
             unit_id_hex, unit_name,
             (unsigned)network_id,
             config::DEVICE_TYPE,
             fw_ver);
    String id_json(id_buf);
    ble_app.sendConfigJSON(id_json);
    ESP_LOGI(TAG, "[CFG] Sent identity readback (%u bytes)", (unsigned)id_json.length());
}

// #285: cacheServoConfig() / cachePIDConfig() were removed along with the
// servo/PID config-relay handlers that were their only callers.  The cfg_servo_*
// / cfg_pid_* globals remain as a read-only readback (loaded from NVS at boot,
// surfaced via sendCurrentConfig) but are no longer written from a relay path,
// consistent with the base station never configuring the rocket.

// ==========================================================================
// SECTION: LoRa uplink to the rocket
// ==========================================================================

/// Build an uplink packet with routing header.
/// target_rid: destination rocket_id (0xFF = broadcast to all rockets in network)
/// retries: number of TX attempts.  Defaults to config::UPLINK_RETRIES (8) for
///   reliability on important commands; heartbeat-style traffic can pass a
///   smaller value to keep airtime low.
static void buildUplinkPacket(uint8_t cmd, const uint8_t* payload, size_t payload_len,
                              uint8_t target_rid = 0xFF,
                              uint8_t retries = config::UPLINK_RETRIES)
{
    // An idle queue transmits immediately; a busy one keeps pacing off the last
    // TX so a queued command can't fire back-to-back with the one ahead of it.
    const bool was_idle = uplink_q.empty();

    const auto res = uplink_q.push(config::UPLINK_SYNC_BYTE,   // 0xCA
                                   network_id, target_rid,
                                   LORA_NEXT_CH_NO_HOP,        // phase 1: hop logic deferred
                                   cmd, payload, payload_len, retries);

    switch (res)
    {
        case bs_uplink_queue::PushResult::RejectedOversized:
            // #286: reject (do NOT truncate) an oversized payload.  Truncating
            // would send a malformed command with a wrong-but-consistent length
            // and no error surfaced.  The queue checks this before anything else,
            // so a rejected command leaves the pending ones untouched.
            ESP_LOGE(TAG, "[UPLINK] cmd=%u payload %u B exceeds max %u — REJECTED, not queued",
                     cmd, (unsigned)payload_len, (unsigned)bs_uplink_queue::kMaxPayload);
            return;

        case bs_uplink_queue::PushResult::RejectedFull:
            // #502: the queue is the fix for silent clobbering, so a genuinely
            // full queue must be loud rather than quietly dropping the oldest.
            ESP_LOGE(TAG, "[UPLINK] queue full (%u) — cmd=%u DROPPED, not queued",
                     (unsigned)bs_uplink_queue::kDepth, cmd);
            return;

        case bs_uplink_queue::PushResult::Queued:
            break;
    }

    if (was_idle) uplink_last_tx_ms = 0;

    ESP_LOGI(TAG, "[UPLINK] Queued cmd=%u -> rid=%u payload=%u bytes, %u retries (queue=%u)",
             cmd, target_rid, (unsigned)payload_len, (unsigned)retries,
             (unsigned)uplink_q.size());
}

static void serviceUplink()
{
    bs_uplink_queue::Entry* tx = uplink_q.head();
    if (tx == nullptr) return;  // nothing queued

    if (tx->retries_left == 0)
    {
        // #285: the uplink is blind fire-and-retry — the rocket sends no
        // command ACK, so completing all retries is NOT proof of delivery.
        // Log it honestly so the operator does not read retry completion as
        // success.  (Safety-relevant rocket state — pyro armed/fired,
        // camera, logging — is confirmed separately via the rocket's own
        // telemetry echo, not via this path.)
        ESP_LOGW(TAG, "[UPLINK] cmd=%u: blind retries exhausted — delivery "
                      "UNCONFIRMED (rocket sends no ACK)",
                 (unsigned)tx->cmd());
        // service() already auto-entered RX after the last TX.  Do NOT call
        // startReceive() here: it would reset rx_done_ and drop any downlink
        // packet that arrived between the TX completion and this point.
        uplink_q.pop();  // next queued command (if any) starts on the following pass
        uplink_defer_start_ms = 0;  // #506: don't carry this command's defer clock over
        return;
    }

    lora_comms.pollDio1();  // Catch TX-done if ISR didn't fire (critical in drain loop)
    lora_comms.service();   // Complete any pending TX

    const uint32_t now = millis();
    if (uplink_last_tx_ms != 0 &&
        (now - uplink_last_tx_ms) < config::UPLINK_RETRY_INTERVAL_MS)
    {
        return;  // Wait between retries
    }

    // #379: gate the transmit on radio-ready AND not-scanning. A TX during a
    // frequency scan goes out on the scan's dwell channel (silently missing the
    // rocket, no ACK to catch it) and corrupts that pass's RSSI — so defer.
    // Returning here before send() preserves retries_left, so the command
    // fires once the scan completes instead of being lost. Mirrors
    // serviceHeartbeat's scan_passes_remaining_ gate.
    if (!bs_uplink_policy::mayTransmitUplink(/*uplink_pending=*/true, tx->retries_left,
                                             scan_passes_remaining_,
                                             lora_comms.canSend()))
    {
        return;  // radio busy, or a scan owns the channel — retry next pass
    }

    // #506: the radio is half-duplex, so this TX is a deaf window. At SF8/BW250 a
    // downlink packet is ~82 ms on air and the gaps between our retries are only
    // ~49 ms, so a blind burst loses essentially EVERY packet that arrives during
    // it (bench-measured: 3 of 3, and they were the run's ONLY losses). Fire into
    // the quiet stretch between the rocket's ~500 ms telemetry packets instead.
    // Transmitting right after the rocket's own TX is also when it is listening,
    // so this should help uplink delivery too.
    // #150: while following a hop, never transmit with a retune still
    // pending — the radio is on the channel of the packet we just decoded,
    // which at dwell-1 the rocket has ALREADY left (it steps to its next
    // channel immediately after its own TX).  One deferred pass lets the
    // top-of-loop retune land the attempt on the channel the rocket is
    // actually listening on.
    if (hop_active_ && hop_needs_retune_)
    {
        return;
    }

    bs_uplink_txwin::Params win;
    win.period_ms     = rx_cadence.periodMs();
    win.tx_airtime_ms = bs_uplink_txwin::timeOnAirMs(tx->len, lora_sf, lora_bw_khz, lora_cr);
    // #150: reserve = the REAL downlink airtime at the live modulation plus
    // margin.  The old flat 140 ms was sized for SF8's ~82 ms downlink and
    // under-reserved at higher SFs, sanctioning attempts that collided with
    // the head of the next downlink (see config.h UPLINK_RX_RESERVE_MARGIN_MS).
    win.rx_reserve_ms = loraTimeOnAirMs(SIZE_OF_LORA_BUDGET, lora_sf, lora_bw_khz,
                                        lora_cr, LORA_TELEM_PREAMBLE_SYMS)
                        + config::UPLINK_RX_RESERVE_MARGIN_MS;
    win.link_stale_ms = config::UPLINK_LINK_STALE_MS;
    win.max_defer_ms  = config::UPLINK_MAX_DEFER_MS;

    // No established telemetry cadence yet => nothing to protect; don't gate.
    const uint32_t rx_anchor = rx_cadence.valid() ? rx_cadence.lastMs() : 0;
    if (uplink_defer_start_ms == 0) uplink_defer_start_ms = now;
    const uint32_t deferred_for = now - uplink_defer_start_ms;

    if (!bs_uplink_txwin::mayStartTx(now, rx_anchor, deferred_for, win))
    {
        uplink_defer_count++;
        return;  // inside the rocket's next downlink slot — wait for the gap
    }
    if (deferred_for >= win.max_defer_ms && rx_anchor != 0)
    {
        // The liveness backstop fired: we transmitted over the downlink rather
        // than starve a blind, safety-relevant command. Should be ~0 in practice;
        // if it climbs, the cadence estimate or the reserve is wrong.
        uplink_defer_override++;
        ESP_LOGW(TAG, "[UPLINK] cmd=%u: TX window never opened for %ums — transmitting anyway",
                 (unsigned)tx->cmd(), (unsigned)deferred_for);
    }

    if (lora_comms.send(tx->buf, tx->len))
    {
        uplink_defer_start_ms = 0;   // this attempt got out; re-arm for the next
        uplink_tx_count++;
        uplink_tx_airtime_ms += win.tx_airtime_ms;
        tx->retries_left--;
        tx->send_failures = 0;  // #565: the radio works again — count CONSECUTIVE failures
        uplink_last_tx_ms = now;
        // #285: "blind" + "unconfirmed" so the log is not mistaken for an ACK.
        ESP_LOGI(TAG, "[UPLINK] blind TX cmd=%u, %u attempt(s) left (unconfirmed, no ACK)",
                 (unsigned)tx->cmd(), tx->retries_left);
    }
    else
    {
        // #565: a failed send() put NOTHING on air (startTransmit error;
        // tx_ongoing_ is never set on this path, so the TX watchdog can't
        // recover it either). This branch used to log-and-return without
        // bounding anything: retries_left never decremented, so a radio whose
        // startTransmit fails persistently wedged the head command — and with
        // it the entire queue — until reboot, RejectedFull-dropping every
        // later command. Count consecutive failures (NOT retries_left: a
        // transient hiccup must not eat the blind delivery budget), pace them
        // like retries via uplink_last_tx_ms (this also stops the old
        // busy-retry-every-pass spin), and at the cap drop the command LOUDLY
        // so the queue stays live for the ones behind it.
        tx->send_failures++;
        uplink_last_tx_ms = now;
        if (tx->send_failures >= config::UPLINK_MAX_SEND_FAILURES)
        {
            ESP_LOGE(TAG, "[UPLINK] cmd=%u DROPPED after %u consecutive send() "
                          "failures (radio TX fault) — command was NEVER transmitted",
                     (unsigned)tx->cmd(), (unsigned)tx->send_failures);
            uplink_q.pop();             // unwedge: next command gets its chance
            uplink_defer_start_ms = 0;  // #506: don't carry this command's defer clock over
        }
        else
        {
            ESP_LOGW(TAG, "[UPLINK] cmd=%u send() failed (%u/%u), will retry",
                     (unsigned)tx->cmd(), (unsigned)tx->send_failures,
                     (unsigned)config::UPLINK_MAX_SEND_FAILURES);
        }
    }
}

// ==========================================================================
// SECTION: Transactional LoRa reconfigure
// ==========================================================================
// Transactional commit of a new LoRa config (freq / bw / sf / cr / pwr).
// Sequence:
//   1. On BLE Cmd 10 the base station takes a rollback snapshot, then queues
//      a broadcast uplink of inner Cmd 10 to every rocket on the OLD channel.
//   2. Once the uplink retries finish (or the upper-bound timer fires), the
//      base station switches its radio to the NEW channel and listens for
//      proof of life (any rocket beacon or telemetry packet bumps
//      last_packet_ms).
//   3. On proof → commit to NVS + send BLE readback.
//      On timeout → reconfigure back to OLD, send BLE readback with OLD
//      values, and leave NVS untouched.  The silence-recovery layer is
//      then responsible for healing any residual divergence.
// The handler is a non-blocking state machine serviced from loop_bs();
// the whole transaction takes ~3 s under normal conditions.

enum class LoRaTxnState : uint8_t {
    IDLE,
    RELAYING,       // Uplink retries in flight on OLD channel
    VERIFYING,      // Listening for rocket beacon/telem on NEW channel
    ROLLING_BACK,   // Restoring OLD channel after verify timed out
};

static LoRaTxnState lora_txn_state = LoRaTxnState::IDLE;

// Target config
static float   txn_new_freq = 0.0f, txn_new_bw = 0.0f;
static uint8_t txn_new_sf = 0, txn_new_cr = 0;
static int8_t  txn_new_pwr = 0;
// Rollback snapshot
static float   txn_old_freq = 0.0f, txn_old_bw = 0.0f;
static uint8_t txn_old_sf = 0, txn_old_cr = 0;
static int8_t  txn_old_pwr = 0;

static uint32_t txn_phase_start_ms = 0;
// last_packet_ms value at the moment we switched to NEW.  Any increase
// during the verify window proves the rocket joined us on NEW.
static uint32_t txn_verify_baseline_packet_ms = 0;

static constexpr uint32_t TXN_VERIFY_WINDOW_MS = 5000;  // Listen 5 s on NEW
static constexpr uint32_t TXN_MAX_RELAY_MS     = 3000;  // Upper bound on relay phase

/// Begin a transactional LoRa reconfigure.  Returns false (and leaves the
/// BS config unchanged, BLE readback sent) if the preconditions fail.
static bool startLoRaTransaction(float new_freq, float new_bw,
                                 uint8_t new_sf, uint8_t new_cr, int8_t new_pwr)
{
    if (freqLockedForRetune() || hop_active_)
    {
        // #835 item 6: deliberately the LONG window.  This is the one consumer
        // that physically retunes the radio, and the relay is a broadcast whose
        // transaction commits on any netid-matching packet — a second rocket on
        // the pad can answer on the new channel and strand an airborne one.
        ESP_LOGW(TAG, "[TXN] Refused: %s",
                 freqLockedForRetune() ? "frequency locked for flight"
                                       : "channel hopping active");
        sendCurrentConfig();
        return false;
    }
    if (lora_txn_state != LoRaTxnState::IDLE)
    {
        ESP_LOGW(TAG, "[TXN] Refused: transaction already in progress");
        sendCurrentConfig();
        return false;
    }

    // Take rollback snapshot BEFORE queuing the uplink, so we can always
    // restore whatever was active when the transaction started.
    txn_old_freq = lora_freq_mhz;
    txn_old_bw   = lora_bw_khz;
    txn_old_sf   = lora_sf;
    txn_old_cr   = lora_cr;
    txn_old_pwr  = lora_tx_power;

    txn_new_freq = new_freq;
    txn_new_bw   = new_bw;
    txn_new_sf   = new_sf;
    txn_new_cr   = new_cr;
    txn_new_pwr  = new_pwr;

    // Broadcast uplink to every rocket in the network on the OLD channel.
    // buildUplinkPacket queues + runs 8 retries on its own (~800 ms total).
    uint8_t loraPayload[11];
    memcpy(loraPayload + 0, &new_freq, 4);
    memcpy(loraPayload + 4, &new_bw,   4);
    loraPayload[8]  = new_sf;
    loraPayload[9]  = new_cr;
    loraPayload[10] = (uint8_t)new_pwr;
    buildUplinkPacket(10, loraPayload, 11, /* target_rid = broadcast */ 0xFF);

    lora_txn_state = LoRaTxnState::RELAYING;
    txn_phase_start_ms = millis();
    ESP_LOGI(TAG, "[TXN] Start: relay %.2f MHz SF%u BW%.0f on OLD, then verify on NEW",
             (double)new_freq, (unsigned)new_sf, (double)new_bw);
    return true;
}

/// Run the transaction state machine — call from loop_bs() every iteration.
static void serviceLoRaTransaction()
{
    switch (lora_txn_state)
    {
        case LoRaTxnState::IDLE:
            return;

        case LoRaTxnState::RELAYING:
        {
            const uint32_t now = millis();
            // serviceUplink() drains the queue once the last retry has
            // fired; at that point the rocket has either joined NEW or not.
            // TXN_MAX_RELAY_MS is a safety net in case uplink state is stuck.
            // Waiting for the whole queue (not just this command) also keeps
            // any command queued behind us from firing after the reconfigure
            // below has already retuned the radio to the NEW channel.
            const bool relay_done   = !uplinkBusy();
            const bool relay_timeout = (now - txn_phase_start_ms) > TXN_MAX_RELAY_MS;
            if (!relay_done && !relay_timeout) return;

            if (!lora_comms.reconfigure(txn_new_freq, txn_new_sf, txn_new_bw,
                                        txn_new_cr, txn_new_pwr))
            {
                ESP_LOGE(TAG, "[TXN] reconfigure(NEW) failed — rolling back");
                lora_txn_state = LoRaTxnState::ROLLING_BACK;
                txn_phase_start_ms = now;
                return;
            }
            lora_comms.startReceive();  // listen on NEW freq

            txn_verify_baseline_packet_ms = last_packet_ms;
            lora_txn_state = LoRaTxnState::VERIFYING;
            txn_phase_start_ms = now;
            ESP_LOGI(TAG, "[TXN] On NEW %.2f MHz; verifying for %u ms",
                     (double)txn_new_freq, (unsigned)TXN_VERIFY_WINDOW_MS);
            break;
        }

        case LoRaTxnState::VERIFYING:
        {
            const uint32_t now = millis();
            // Any packet (beacon or telem) received after we switched proves
            // the rocket joined us on NEW.  The RX path unconditionally
            // bumps last_packet_ms on every successful receive.
            if (last_packet_ms > txn_verify_baseline_packet_ms)
            {
                // COMMIT: radio already on NEW; persist to NVS + update
                // runtime vars so the rest of the firmware sees the new
                // config in readbacks.
                const float old_bw = lora_bw_khz;
                lora_freq_mhz = txn_new_freq;
                lora_bw_khz   = txn_new_bw;
                lora_sf       = txn_new_sf;
                lora_cr       = txn_new_cr;
                lora_tx_power = txn_new_pwr;

                prefs.begin("lora", false);
                prefs.putFloat("freq",  lora_freq_mhz);
                prefs.putFloat("bw",    lora_bw_khz);
                prefs.putUChar("sf",    lora_sf);
                prefs.putUChar("cr",    lora_cr);
                prefs.putChar("txpwr",  lora_tx_power);
                prefs.end();

                // BW change invalidates the skip-mask (it's sized for
                // the old hop table).  Rendezvous freq survives.
                if (old_bw != lora_bw_khz)
                {
                    invalidateSkipMaskForBwChange();
                }

                // Re-push the channel-set if we have a recent scan
                // (#40 / #41 phase 3).  The original post-scan cmd-15
                // would have been displaced from the uplink queue by
                // this very cmd-10 transaction; re-pushing here is the
                // simplest way to guarantee delivery, and it also
                // re-sizes the skip-mask if BW just changed.  Inert if
                // no scan has happened this session.
                analyzeAndPushFromCachedScan();

                ESP_LOGI(TAG, "[TXN] COMMIT: heard rocket on NEW %.2f MHz, saved",
                         (double)lora_freq_mhz);
                lora_txn_state = LoRaTxnState::IDLE;
                sendCurrentConfig();
                return;
            }
            if ((now - txn_phase_start_ms) >= TXN_VERIFY_WINDOW_MS)
            {
                ESP_LOGW(TAG, "[TXN] TIMEOUT: no rocket on NEW %.2f MHz, rolling back",
                         (double)txn_new_freq);
                lora_txn_state = LoRaTxnState::ROLLING_BACK;
                txn_phase_start_ms = now;
            }
            break;
        }

        case LoRaTxnState::ROLLING_BACK:
        {
            // Restore the OLD config.  If this fails the radio may be stuck;
            // log loudly and return to IDLE — silence recovery is the last
            // line of defence.
            if (lora_comms.reconfigure(txn_old_freq, txn_old_sf, txn_old_bw,
                                       txn_old_cr, txn_old_pwr))
            {
                lora_freq_mhz = txn_old_freq;
                lora_bw_khz   = txn_old_bw;
                lora_sf       = txn_old_sf;
                lora_cr       = txn_old_cr;
                lora_tx_power = txn_old_pwr;
                lora_comms.startReceive();
                ESP_LOGI(TAG, "[TXN] ROLLED BACK to %.2f MHz", (double)txn_old_freq);
            }
            else
            {
                ESP_LOGE(TAG, "[TXN] Rollback reconfigure FAILED — radio may be stuck");
            }
            lora_txn_state = LoRaTxnState::IDLE;
            sendCurrentConfig();
            break;
        }
    }
}

// ==========================================================================
// SECTION: Silence recovery
// ==========================================================================
// If the base station hears nothing from any rocket for RECOVERY_SILENCE_MS
// while on the ground (not freqLockedForFlight()), hop through known-good
// frequencies looking for the rocket:
//   Phase A (rendezvous): tune to LORA_FACTORY_RENDEZVOUS_MHZ and listen 3 s.  If a
//     beacon / telem arrives, relay Cmd 10 with the saved NVS config to
//     push the rocket back, then return to the saved NVS freq.
//   Phase B (grid scan): if Phase A was silent, scan the saved NVS freq ±
//     2 MHz in 200 kHz steps (21 channels), dwelling one beacon cycle per
//     step.  On hit, relay Cmd 10 on that channel and return to NVS.
//     If the grid completes with nothing heard, give up this cycle and
//     wait for the next silence trip.
// While in flight (freqLockedForFlight()) recovery is fully disabled —
// momentary silence during flight is expected (SNR dips) and hopping would
// guarantee we lose the rest of the telemetry stream.

enum class RecoveryState : uint8_t {
    IDLE,
    PHASE_A_RENDEZVOUS,
    PHASE_B_SCAN,
    COMPLETE,               // Relay in flight; hop home once it drains
};

static RecoveryState recovery_state = RecoveryState::IDLE;
static uint32_t recovery_phase_start_ms     = 0;
static uint32_t recovery_baseline_packet_ms = 0;
static int      recovery_scan_index         = 0;
static float    recovery_scan_current_mhz   = 0.0f;

static constexpr uint32_t RECOVERY_SILENCE_MS       = 10000; // idle trigger
// Phase A dwells on the rendezvous frequency long enough to (a) catch
// many beacons from a rocket sitting permanently on rendezvous (factory
// default case, ~15 beacons in 30 s) and (b) deterministically overlap
// the rocket's 5 s slow-rendezvous window when the two NVS freqs differ.
// Anything shorter than ~10 s is statistical and was the root of the
// "BS comes up, never sees rocket" symptom from the field test.
static constexpr uint32_t RECOVERY_PHASE_A_DWELL_MS = 30000;
static constexpr uint32_t RECOVERY_PHASE_B_DWELL_MS = 2500;  // one beacon cycle + slack
static constexpr int      RECOVERY_PHASE_B_CHANNELS = 21;    // ±10 steps of 200 kHz
static constexpr float    RECOVERY_PHASE_B_STEP_MHZ = 0.200f;
static constexpr float    RECOVERY_PHASE_B_SPAN_MHZ = 2.0f;  // ±2 MHz around NVS

// Hop to the full rendezvous mode (freq + SF/BW/CR/power).  Used for
// Phase A — both ends agree on this exact config as a known-good
// fallback, regardless of what the user set in NVS.  All five values
// are compile-time constants in RocketComputerTypes.h so BS and OC
// cannot disagree on the meeting place (#105 follow-up).
static void recoveryHopToRendezvousMode()
{
    if (lora_comms.reconfigure(LORA_FACTORY_RENDEZVOUS_MHZ,
                                LORA_FACTORY_RENDEZVOUS_SF,
                                LORA_FACTORY_RENDEZVOUS_BW_KHZ,
                                LORA_FACTORY_RENDEZVOUS_CR,
                                LORA_FACTORY_RENDEZVOUS_TX_DBM))
    {
        lora_comms.startReceive();
    }
    else
    {
        ESP_LOGE(TAG, "[RECOVER] reconfigure to rendezvous mode failed");
    }
}

// Hop to a target frequency keeping the user-configured NVS modulation
// (SF/BW/CR/power).  Used for Phase B local scan and the post-recovery
// return to the saved channel.  This catches the common case of "rocket
// is on a slightly off frequency but same SF/BW as the BS".
static void recoveryHopToFreq(float freq_mhz)
{
    if (lora_comms.reconfigure(freq_mhz, lora_sf, lora_bw_khz, lora_cr, lora_tx_power))
    {
        lora_comms.startReceive();
    }
    else
    {
        ESP_LOGE(TAG, "[RECOVER] reconfigure to %.2f MHz failed", (double)freq_mhz);
    }
}

static void recoveryEnterPhaseA()
{
    recoveryHopToRendezvousMode();
    recovery_baseline_packet_ms = last_packet_ms;
    recovery_phase_start_ms     = millis();
    recovery_state              = RecoveryState::PHASE_A_RENDEZVOUS;
    ESP_LOGW(TAG, "[RECOVER] Silent — Phase A rendezvous mode (%.2f MHz SF%u BW%.0f) for %u ms",
             (double)LORA_FACTORY_RENDEZVOUS_MHZ,
             (unsigned)LORA_FACTORY_RENDEZVOUS_SF,
             (double)LORA_FACTORY_RENDEZVOUS_BW_KHZ,
             (unsigned)RECOVERY_PHASE_A_DWELL_MS);
}

static void recoveryEnterPhaseB()
{
    recovery_scan_index       = 0;
    recovery_scan_current_mhz = lora_freq_mhz - RECOVERY_PHASE_B_SPAN_MHZ;
    recoveryHopToFreq(recovery_scan_current_mhz);
    recovery_baseline_packet_ms = last_packet_ms;
    recovery_phase_start_ms     = millis();
    recovery_state              = RecoveryState::PHASE_B_SCAN;
    ESP_LOGW(TAG, "[RECOVER] Phase B scan: %d channels around %.2f MHz",
             RECOVERY_PHASE_B_CHANNELS, (double)lora_freq_mhz);
}

static void recoveryEnd(const char* why)
{
    // Always come back to the saved NVS frequency so we're either settled
    // on the committed channel or poised to hear the rocket once it
    // returns there.
    recoveryHopToFreq(lora_freq_mhz);
    recovery_state = RecoveryState::IDLE;
    ESP_LOGI(TAG, "[RECOVER] Done (%s). Back on %.2f MHz",
             why, (double)lora_freq_mhz);
}

/// Relay Cmd 10 on the current channel to push the rocket back to the
/// saved NVS config.  Used when we re-locate the rocket during recovery.
static void recoveryPushRocketHome()
{
    uint8_t payload[11];
    memcpy(payload + 0, &lora_freq_mhz, 4);
    memcpy(payload + 4, &lora_bw_khz,   4);
    payload[8]  = lora_sf;
    payload[9]  = lora_cr;
    payload[10] = (uint8_t)lora_tx_power;
    buildUplinkPacket(10, payload, 11, /* target_rid = broadcast */ 0xFF);
    ESP_LOGI(TAG, "[RECOVER] Relay Cmd 10 -> %.2f MHz (saved NVS)",
             (double)lora_freq_mhz);
}

static void serviceRecovery()
{
    // #136/#150: in fixed mode the recovery state machine is redundant —
    // both ends are pinned to LORA_FACTORY_RENDEZVOUS at boot, so Phase
    // A's reconfigure is a no-op and the post-Phase-A "push rocket home"
    // cmd 10 just blasts redundant TX retries at a rocket that's already
    // on the right channel.  Since #150 this gate is live policy, not a
    // placeholder: recovery runs whenever the user selects hopping mode
    // (where the two ends genuinely can lose each other), and stays
    // suppressed in fixed mode.
    if (lora_hop_disabled)
    {
        if (recovery_state != RecoveryState::IDLE)
            recoveryEnd("hop disabled — recovery suppressed");
        return;
    }
    // While locked for flight, or while actively hopping with the rocket,
    // accept silence — neither is a recovery scenario.  In flight, momentary
    // SNR dips look like silence; while hopping (#40 / #41), the recovery
    // hop scan and the hop sequence would fight each other for the radio.
    if (freqLockedForFlight() || hop_active_)
    {
        if (recovery_state != RecoveryState::IDLE)
            recoveryEnd(freqLockedForFlight() ? "flight locked" : "hopping active");
        return;
    }
    // Transactional reconfigure takes priority.  A BLE Cmd 10 arriving
    // mid-recovery aborts the recovery cycle; the transaction then
    // self-rolls-back-or-commits, and any residual divergence is healed
    // by the next recovery pass.
    if (lora_txn_state != LoRaTxnState::IDLE)
    {
        if (recovery_state != RecoveryState::IDLE) recoveryEnd("transaction");
        return;
    }

    const uint32_t now = millis();

    switch (recovery_state)
    {
        case RecoveryState::IDLE:
        {
            // Avoid firing during the first RECOVERY_SILENCE_MS after boot
            // — it's normal to be silent while the rocket is still booting.
            if (last_packet_ms == 0 && now < RECOVERY_SILENCE_MS) return;
            const uint32_t silent_for = (last_packet_ms > 0)
                ? (now - last_packet_ms)
                : now;
            if (silent_for >= RECOVERY_SILENCE_MS) recoveryEnterPhaseA();
            break;
        }

        case RecoveryState::PHASE_A_RENDEZVOUS:
        {
            if (last_packet_ms > recovery_baseline_packet_ms)
            {
                recoveryPushRocketHome();
                recovery_phase_start_ms = now;
                recovery_state = RecoveryState::COMPLETE;
                break;
            }
            if ((now - recovery_phase_start_ms) >= RECOVERY_PHASE_A_DWELL_MS)
            {
                // #136: Phase B scans ±2 MHz around lora_freq_mhz to catch
                // a hopping rocket that drifted to an unknown channel.
                // With hopping disabled, the rocket is fixed on its
                // configured channel — walking the BS off-frequency only
                // hides the rocket from us during the very window it's
                // beaconing.  Just end the recovery cycle; next silence
                // tick will start another Phase A dwell on rendezvous,
                // which is the only place a fixed-channel rocket can be.
                if (lora_hop_disabled)
                {
                    recoveryEnd("hop disabled — skipping Phase B scan");
                }
                else
                {
                    recoveryEnterPhaseB();
                }
            }
            break;
        }

        case RecoveryState::PHASE_B_SCAN:
        {
            if (last_packet_ms > recovery_baseline_packet_ms)
            {
                recoveryPushRocketHome();
                recovery_phase_start_ms = now;
                recovery_state = RecoveryState::COMPLETE;
                break;
            }
            if ((now - recovery_phase_start_ms) >= RECOVERY_PHASE_B_DWELL_MS)
            {
                recovery_scan_index++;
                if (recovery_scan_index >= RECOVERY_PHASE_B_CHANNELS)
                {
                    // Exhausted grid with no hits — give up this cycle.
                    // Silence will trip us again after RECOVERY_SILENCE_MS.
                    recoveryEnd("scan exhausted");
                    return;
                }
                recovery_scan_current_mhz += RECOVERY_PHASE_B_STEP_MHZ;
                recoveryHopToFreq(recovery_scan_current_mhz);
                recovery_baseline_packet_ms = last_packet_ms;
                recovery_phase_start_ms = now;
            }
            break;
        }

        case RecoveryState::COMPLETE:
        {
            // Give uplink retries time to land on the current channel
            // before we hop back to NVS — otherwise the rocket might miss
            // the push command and we'd just rediscover it next cycle.
            if (!uplinkBusy() ||
                (now - recovery_phase_start_ms) > TXN_MAX_RELAY_MS)
            {
                recoveryEnd("pushed rocket home");
            }
            break;
        }
    }
}

// ==========================================================================
// SECTION: Heartbeat to the rocket
// ==========================================================================
// Periodic uplink that gives the rocket positive proof of comms.  Without
// this, a rocket happily streaming telemetry to a base station that's
// receiving fine would still fall into its slow-rendezvous cycle every
// time the user goes a couple of minutes without sending a command —
// because beacons go one way and the rocket has no signal that anyone is
// listening.
//
// Only sent while we ARE hearing the rocket (last_packet_ms recent).  If
// rocket goes silent, the recovery state machine takes over and we stop
// heartbeating until comms are restored.  Uses 2 retries instead of 8 to
// keep airtime per beat small (~100 ms total).
//
// Interval must be < OC RENDEZVOUS_TRIGGER_QUIET_MS (15 s in #105) with
// margin for at least one missed beat — otherwise the rocket fires its
// slow-rendezvous in steady state, drops to the rendezvous channel, and
// the BS spends ~30 s syncing it back via Cmd-10 relay.  10 s gives the
// OC two heartbeats per silence-tolerance window and keeps airtime ~1%.
// (Defined here, after the LoRa transaction & recovery sections, so the
// state-machine enums it depends on are in scope.)

static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 10000;  // every 10 s (#105)

// #150 bench finding: a heartbeat TX can land exactly in the rocket's
// post-enable bootstrap window and eat the handoff (the BS is deaf while
// transmitting).  Hold heartbeats briefly after we send a cmd-17 ENABLE so
// the bootstrap packets meet a listening receiver.
static constexpr uint32_t HEARTBEAT_HOLD_AFTER_ENABLE_MS = 4000;
static uint32_t heartbeat_hold_until_ms = 0;
static constexpr uint32_t HEARTBEAT_RX_FRESH_MS = 5000;   // rocket "alive"
static constexpr uint8_t  HEARTBEAT_RETRIES     = 2;
static uint32_t last_heartbeat_tx_ms = 0;

// Safe-window guard for BS uplinks while the rocket is hopping
// (#40 / #41 phase 2b).  The rocket TXes telemetry at a fixed rate
// (typ. 2 Hz = 500 ms cycle); after we RX a packet, we know the
// rocket is in RX mode for the rest of its slot.  TXing within
// ~150 ms of that RX guarantees we finish well before the rocket's
// next TX, avoiding the desync collision pattern that broke the
// link in phase 2a bench testing.
static constexpr uint32_t HOP_BS_TX_SAFE_WINDOW_MS = 150;

static inline bool inHopSafeWindow()
{
    if (!hop_active_) return true;             // not hopping → always safe
    if (last_packet_ms == 0) return false;     // never RX'd → no anchor
    return (millis() - last_packet_ms) <= HOP_BS_TX_SAFE_WINDOW_MS;
}

static void serviceHeartbeat()
{
    // The flight freq-lock freezes radio PARAMETERS mid-flight (#106); a
    // zero-payload broadcast heartbeat changes none, and while hopping the
    // heartbeats are load-bearing: without them the rocket hears no uplink,
    // its rendezvous fallback fires every 30 s (HOP_FALLBACK_TRIGGER_
    // INITIAL_MS), and each 3 s visit trips our own 3 s silence teardown —
    // ~8 packets lost per ~33 s, ~5% flight loss (2026-07-16 bench, three
    // sim flights; loss <0.2% in every non-flight state). Fixed-channel
    // flights keep the old behavior: no hop fallback to feed, and staying
    // RX-only in flight costs nothing.
    if (freqLockedForFlight() && !hop_active_) return;  // fixed-channel flight only
    if (lora_txn_state != LoRaTxnState::IDLE)  return;  // Don't interfere with txn
    if (recovery_state != RecoveryState::IDLE) return;  // Recovery owns the radio
    if (scan_passes_remaining_ != 0)           return;  // #136: don't TX mid-scan
    if (uplinkBusy())                          return;  // Don't compete with a real cmd
    // #150: stay quiet through the rocket's post-enable bootstrap window
    // so we can't be deaf (mid-TX) when the handoff packets arrive.
    if (heartbeat_hold_until_ms != 0 &&
        (int32_t)(millis() - heartbeat_hold_until_ms) < 0) return;
    // While hopping, only TX in the safe window right after a fresh
    // rocket RX — see HOP_BS_TX_SAFE_WINDOW_MS comment.  With slow-hop
    // dwell=4 (#105 follow-up) the rocket stays on a channel for 2 s,
    // so a 250 ms heartbeat-with-retries fits comfortably inside the
    // dwell.  Without heartbeats during hop, the rocket's
    // hop_session_uplink_count stays 0 and its fallback fires every 30 s
    // (HOP_FALLBACK_TRIGGER_INITIAL_MS), tearing down the session.
    if (!inHopSafeWindow())                    return;

    const uint32_t now = millis();
    // Only heartbeat when we've recently heard the rocket.  If rocket has
    // gone silent, recovery will engage and ramp through rendezvous/scan;
    // beating into the void during that is just wasted airtime.
    if (last_packet_ms == 0)                                  return;
    if ((now - last_packet_ms) > HEARTBEAT_RX_FRESH_MS)       return;
    if ((now - last_heartbeat_tx_ms) < HEARTBEAT_INTERVAL_MS) return;

    buildUplinkPacket(LORA_CMD_HEARTBEAT, nullptr, 0,
                      /* target_rid = broadcast */ 0xFF,
                      /* retries */ HEARTBEAT_RETRIES);
    last_heartbeat_tx_ms = now;
}

// ==========================================================================
// SECTION: Coordinated noise scan and channel set
// ==========================================================================
// On BLE cmd 60 we run a 5-pass noise scan (max-RSSI accumulation per
// channel) instead of a single sweep, so an intermittent jammer that
// only fires every couple of seconds is more likely to be captured.
// When all passes complete, we ship the accumulated grid to the iOS app
// (preserving the existing single-result protocol), run the channel-set
// analyzer locally, persist the result to NVS, and push it to the
// rocket via LORA_CMD_CHANNEL_SET.

static constexpr uint8_t LORA_NOISE_SCAN_PASSES = 5;

static int8_t   scan_peak_rssi_[TR_LoRa_Comms::SCAN_MAX_SAMPLES] = {0};
static size_t   scan_peak_count_ = 0;
static float    scan_peak_start_mhz_ = 0.0f;
static float    scan_peak_step_khz_  = 0.0f;
// scan_passes_remaining_ is declared near the top of the file so
// serviceHeartbeat() can read it.
// Set true after a multi-pass scan completes; used by the cmd-10
// commit path to re-push the channel-set selection (the original
// post-scan push gets clobbered when the iOS app's auto-channel-select
// queues cmd 10 immediately after seeing scan results).
static bool     scan_results_valid_ = false;

// Held across passes so subsequent calls re-use the same parameters.
static float    scan_param_start_mhz_ = 0.0f;
static float    scan_param_stop_mhz_  = 0.0f;
static uint16_t scan_param_step_khz_  = 0;
static uint16_t scan_param_dwell_ms_  = 0;

// ----------------------------------------------------------------------------
// Coordinated hop pause for in-flight scan (#90)
// ----------------------------------------------------------------------------
// When a BLE cmd 60 arrives while we're already following a hopping rocket
// (hop_active_ == true), running the scan directly would silence the link
// for ~9 s and trip the rocket's hop fallback.  Instead, we ask the rocket
// to park on lora_freq_mhz for a known window via cmd 16, then run the
// scan + push cmd 15 inside that window, then both sides re-bootstrap hop.
//
// State flow:
//   IDLE          → AWAITING_PAUSE  (cmd 16 queued; wait for retries done)
//   AWAITING_PAUSE → SCANNING       (reconfigure to lora_freq_mhz, kick scan)
//   SCANNING       → PUSHING_CHSET  (scan finalized, cmd 15 queued)
//   PUSHING_CHSET  → RESUMING       (cmd 15 retries done)
//   RESUMING       → IDLE           (rocket bootstrap RX, or timeout)
enum class CoordScanState : uint8_t {
    IDLE,
    AWAITING_PAUSE,
    SCANNING,
    PUSHING_CHSET,
    RESUMING,
};

static CoordScanState coord_scan_state_       = CoordScanState::IDLE;
static uint32_t       coord_scan_phase_ms_    = 0;
static uint32_t       coord_scan_resume_anchor_ms_ = 0;  // hop_last_rx_ms_ snapshot at RESUMING entry
// Stored scan params so we can re-issue startNoiseScan() after the pause is acked.
static float    coord_scan_start_mhz_ = 0.0f;
static float    coord_scan_stop_mhz_  = 0.0f;
static uint16_t coord_scan_step_khz_  = 0;
static uint16_t coord_scan_dwell_ms_  = 0;

// After cmd 16 retries finish, give the rocket a moment to actually swap
// to lora_freq_mhz before we reconfigure the BS radio.
static constexpr uint32_t COORD_SCAN_PAUSE_GRACE_MS  = 500;
// If the rocket never resumes hopping after we push cmd 15, give up so
// the normal silence/recovery machinery can take over.
static constexpr uint32_t COORD_SCAN_RESUMING_MAX_MS = 5000;
// #570: SCANNING-state budget — the driver scan's own worst case at the
// stored params (steps × (dwell + 250 ms/step of loop-pacing margin) + 5 s
// slack, matching the #567 in-driver backstop sizing), so a healthy slow
// scan can never trip it. Guards the coordinated-scan machine against a
// scan that finishes with nothing pushable (see the SCANNING case).
static uint32_t coordScanScanningBudgetMs()
{
    if (coord_scan_step_khz_ == 0) return 30000u;  // defensive; startScan rejects 0
    const uint32_t steps = (uint32_t)(((coord_scan_stop_mhz_ - coord_scan_start_mhz_) * 1000.0f)
                                      / (float)coord_scan_step_khz_) + 1u;
    return steps * (uint32_t)(coord_scan_dwell_ms_ + 250u) + 5000u;
}

// Slack on top of the computed scan + cmd 15 retry budget so the rocket's
// pause comfortably outlasts our work window.
// #150 Seam B finding: the rocket's pause clock starts at cmd-16 RECEIPT,
// but the BS only starts spending the budget once it has confirmed the
// park (~2 s later: remaining cmd-16 retries + grace).  With 2 s of slack
// the cmd-15 mask push landed exactly at the rocket's resume deadline and
// collided with its resume bootstraps (half-duplex), costing ~46 s of
// fallback healing after every mid-hop scan.  5 s absorbs the start
// latency + the push train with margin; the rocket-side cap is 60 s.
static constexpr uint32_t COORD_SCAN_PAUSE_SLACK_MS  = 5000;
// "Recent enough" window for treating a non-hop_active_ rocket as still
// in a hop state (#90).  A packet within this window showing PRELAUNCH
// or INFLIGHT means the rocket is conceptually hopping (possibly
// bootstrapping or visiting rendezvous) and a direct scan would still
// drop the link — so route through the coordinated-pause path.  Sized
// to match RECOVERY_SILENCE_MS (10 s): beyond that the recovery layer
// has already taken over and we don't presume anything.
static constexpr uint32_t COORD_HOP_RECENT_MS        = 10000;

// Persist channel-set selection to NVS.  Skip-mask is keyed off the BW
// it was generated for so a later cmd-10 BW change invalidates it
// cleanly (loadChannelSetFromNvs detects mismatch and clears).
// Rendezvous freq is no longer persisted — see #105 / LORA_NVS_SCHEMA_VERSION
// v3 in RocketComputerTypes.h.
static void saveChannelSetToNvs(const LoRaChannelSetSelection& sel,
                                float bw_khz)
{
    Preferences p;
    if (!p.begin("lora", false)) return;
    p.putUChar("chset_n", sel.n_channels);
    p.putFloat("chset_bw", bw_khz);
    const size_t bytes_used = (sel.n_channels + 7) / 8;
    p.putBytes("chset_mask", sel.skip_mask, bytes_used);
    p.end();
}

// Restore channel-set selection from NVS.  If the stored BW doesn't
// match the active operating BW, the skip-mask is treated as stale and
// the runtime state is reset to "no skips".  Rendezvous freq is no
// longer NVS-stored (#105) — it's a compile-time constant.
static void loadChannelSetFromNvs()
{
    Preferences p;
    if (!p.begin("lora", true)) return;
    const uint8_t n_stored  = p.getUChar("chset_n", 0);
    const float   bw_stored = p.getFloat("chset_bw", 0.0f);
    if (n_stored > 0 && bw_stored > 0.0f && bw_stored == lora_bw_khz)
    {
        const size_t bytes_used = (n_stored + 7) / 8;
        size_t got = p.getBytes("chset_mask", skip_mask_, bytes_used);
        if (got == bytes_used)
        {
            skip_mask_n_        = n_stored;
            channel_set_bw_khz_ = bw_stored;
        }
    }
    else
    {
        skip_mask_n_        = 0;
        channel_set_bw_khz_ = 0.0f;
        for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    }
    p.end();
}

// Clear stored skip-mask — called on cmd-10 BW change so the rocket
// doesn't follow a stale skip-mask sized for the previous BW.
static void invalidateSkipMaskForBwChange()
{
    skip_mask_n_        = 0;
    channel_set_bw_khz_ = 0.0f;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    Preferences p;
    if (p.begin("lora", false))
    {
        p.remove("chset_n");
        p.remove("chset_bw");
        p.remove("chset_mask");
        p.end();
    }
    ESP_LOGI(TAG, "[CHSET] BW changed — skip-mask invalidated");
}

// Apply a freshly computed selection to runtime state and queue a push
// to the rocket so the rocket's NVS + runtime mirrors ours.
static void applyAndPushChannelSet(const LoRaChannelSetSelection& sel,
                                    float bw_khz)
{
    // Adopt locally
    skip_mask_n_        = sel.n_channels;
    channel_set_bw_khz_ = bw_khz;
    const size_t bytes_used = (sel.n_channels + 7) / 8;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    for (size_t i = 0; i < bytes_used; i++) skip_mask_[i] = sel.skip_mask[i];

    saveChannelSetToNvs(sel, bw_khz);

    // Wire format: [bw:f4][n:u1][mask: ceil(n/8)]
    // Rendezvous freq used to lead this payload but is now hardcoded on
    // both sides (#105) — see LORA_FACTORY_RENDEZVOUS_MHZ.
    uint8_t payload[40] = {0};
    size_t  off = 0;
    memcpy(payload + off, &bw_khz,             4); off += 4;
    payload[off++] = sel.n_channels;
    for (size_t i = 0; i < bytes_used; i++) payload[off++] = sel.skip_mask[i];

    // Count active for the log so user sees how many channels survived
    uint8_t active = 0;
    for (uint8_t i = 0; i < sel.n_channels; i++)
        if (!loraSkipMaskTest(sel.skip_mask, i)) active++;
    ESP_LOGI(TAG, "[CHSET] %u/%u channels active at BW=%.0f kHz "
                  "(min FCC floor %u) — pushing to rocket",
             (unsigned)active,
             (unsigned)sel.n_channels, (double)bw_khz,
             (unsigned)loraFhssMinChannels(bw_khz));

    buildUplinkPacket(LORA_CMD_CHANNEL_SET, payload, off, /*target_rid=*/0xFF,
                      /*retries=*/config::UPLINK_RETRIES);
}

// Called on a fresh BLE cmd-60 to start the multi-pass scan sequence.
// Returns true if the first pass started successfully.
static bool startNoiseScan(float start_mhz, float stop_mhz,
                            uint16_t step_khz, uint16_t dwell_ms)
{
    if (scan_passes_remaining_ != 0) return false;  // already scanning

    scan_param_start_mhz_  = start_mhz;
    scan_param_stop_mhz_   = stop_mhz;
    scan_param_step_khz_   = step_khz;
    scan_param_dwell_ms_   = dwell_ms;
    scan_peak_count_       = 0;  // signals "first pass, copy not max"
    scan_results_valid_    = false;  // becomes true on finalize
    for (size_t i = 0; i < TR_LoRa_Comms::SCAN_MAX_SAMPLES; i++)
        scan_peak_rssi_[i] = INT8_MIN;

    if (!lora_comms.startScan(start_mhz, stop_mhz, step_khz, dwell_ms))
        return false;

    scan_passes_remaining_ = LORA_NOISE_SCAN_PASSES;
    ESP_LOGI(TAG, "[CHSET] Noise scan started: %u passes × %u ms dwell, %.1f..%.1f MHz",
             (unsigned)LORA_NOISE_SCAN_PASSES, (unsigned)dwell_ms,
             (double)start_mhz, (double)stop_mhz);
    return true;
}

// Merge the just-completed pass's samples into scan_peak_rssi_[] (max).
// On the first pass we also capture the geometry (start_mhz, step_khz)
// for the analyzer.  Returns true if more passes are needed.
static bool absorbScanPass()
{
    const auto* samples = lora_comms.getScanSamples();
    const size_t n = lora_comms.getScanSampleCount();

    if (scan_peak_count_ == 0)
    {
        scan_peak_start_mhz_ = lora_comms.getScanStartMHz();
        scan_peak_step_khz_  = lora_comms.getScanStepKHz();
        scan_peak_count_     = (n > TR_LoRa_Comms::SCAN_MAX_SAMPLES)
                                ? TR_LoRa_Comms::SCAN_MAX_SAMPLES : n;
        for (size_t i = 0; i < scan_peak_count_; i++)
            scan_peak_rssi_[i] = samples[i].rssi_dbm;
    }
    else
    {
        const size_t cap = (n < scan_peak_count_) ? n : scan_peak_count_;
        for (size_t i = 0; i < cap; i++)
            if (samples[i].rssi_dbm > scan_peak_rssi_[i])
                scan_peak_rssi_[i] = samples[i].rssi_dbm;
    }
    lora_comms.consumeScanDone();
    return (--scan_passes_remaining_) > 0;
}

// Re-run the channel-set analyzer over the most recent scan grid using
// `lora_bw_khz` (whatever it is right now), then push to the rocket.
// Used by both finalizeNoiseScan() and the cmd-10 commit path — the
// latter to re-push after the auto-select cmd-10 race clobbered the
// initial cmd-15 in the uplink queue.
static void analyzeAndPushFromCachedScan()
{
    if (!scan_results_valid_ || scan_peak_count_ == 0) return;

    float scan_freqs[TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    for (size_t i = 0; i < scan_peak_count_; i++)
    {
        scan_freqs[i] = scan_peak_start_mhz_
                        + (scan_peak_step_khz_ * (float)i) / 1000.0f;
    }
    LoRaChannelSetSelection sel{};
    loraSelectChannelSet(scan_freqs, scan_peak_rssi_, scan_peak_count_,
                          lora_bw_khz, &sel);
    applyAndPushChannelSet(sel, lora_bw_khz);
}

// Re-push the BS's current skip-mask state to the rocket — used by the
// mask-drift auto-recovery (#105 follow-up) when the rocket's announced
// next_channel_idx doesn't match the BS's seq-derived expectation.  No
// re-analysis (no scan needed); just (re-)delivers what we already have.
// Inert if we have no mask to push.
static void pushCurrentChannelSet()
{
    if (skip_mask_n_ == 0) return;  // nothing to push
    LoRaChannelSetSelection sel = {};
    sel.n_channels = skip_mask_n_;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) {
        sel.skip_mask[i] = skip_mask_[i];
    }
    applyAndPushChannelSet(sel, channel_set_bw_khz_);
}

// Mask-drift auto-recovery service (#105 follow-up).  Watches the flag
// set by the hop RX path and re-pushes cmd 15 when the radio state is
// quiet enough to do so safely.  Cooldown prevents spam if drift
// persists across multiple packets.
static void serviceMaskDriftRepush()
{
    if (!chset_drift_repush_pending) return;
    // Only push when nothing else is using the radio for TX.
    if (uplinkBusy())                          return;
    if (lora_txn_state != LoRaTxnState::IDLE)  return;
    if (recovery_state != RecoveryState::IDLE) return;
    // Cooldown: don't re-push more than once every CHSET_DRIFT_REPUSH_COOLDOWN_MS.
    // First-ever push (last_ms_=0) bypasses the cooldown.
    const uint32_t now = millis();
    if (chset_drift_repush_last_ms_ != 0 &&
        (now - chset_drift_repush_last_ms_) < CHSET_DRIFT_REPUSH_COOLDOWN_MS)
    {
        // Cooldown active; clear the flag so we don't spin checking.
        // Next drift event after the cooldown will set it again.
        chset_drift_repush_pending = false;
        return;
    }
    if (skip_mask_n_ == 0) {
        // No mask to push — likely a brand-new BS without scan results.
        // The drift warning is informational only in this case.
        chset_drift_repush_pending = false;
        return;
    }
    chset_drift_repush_pending = false;
    chset_drift_repush_last_ms_ = now;
    chset_drift_repush_count++;
    ESP_LOGI(TAG, "[CHSET] Mask drift detected — re-pushing cmd 15 (count=%lu)",
             (unsigned long)chset_drift_repush_count);
    pushCurrentChannelSet();
}

// Hop-mode resync (#150).  Consumes the evidence flag set by the hop RX
// path (see the streak/direction rules at the state definitions) and
// re-uplinks cmd 17 with the BS's current link mode so a rocket that
// rebooted or missed the change converges back.  Same radio-idle gating
// and cooldown pattern as serviceMaskDriftRepush.
static void serviceHopModeResync()
{
    if (!hop_mode_resync_pending) return;
    if (uplinkBusy())                          return;
    if (lora_txn_state != LoRaTxnState::IDLE)  return;
    if (recovery_state != RecoveryState::IDLE) return;
    const uint32_t now = millis();
    if (hop_mode_resync_last_ms_ != 0 &&
        (now - hop_mode_resync_last_ms_) < HOP_MODE_RESYNC_COOLDOWN_MS)
    {
        // Cooldown active; clear so we don't spin.  Fresh evidence after
        // the cooldown re-arms it.
        hop_mode_resync_pending = false;
        return;
    }
    hop_mode_resync_pending   = false;
    hop_mode_mismatch_streak_ = 0;
    hop_mode_resync_last_ms_  = now;
    hop_mode_resync_count_++;
    const uint8_t desired = lora_hop_disabled ? 1 : 0;
    ESP_LOGW(TAG, "[HOP] Mode mismatch evidence — re-pushing cmd 17 (%s, count=%lu)",
             lora_hop_disabled ? "disable" : "enable",
             (unsigned long)hop_mode_resync_count_);
    if (!lora_hop_disabled)
    {
        // #150: same heartbeat hold as the BLE enable path — the resync
        // enable also triggers a deferred bootstrap on the rocket.
        heartbeat_hold_until_ms = millis() + HEARTBEAT_HOLD_AFTER_ENABLE_MS;
    }
    buildUplinkPacket(LORA_CMD_SET_HOP_DISABLED, &desired, 1);
}

// #150 (review): completes a graceful hop disable — see the cmd-17 BLE
// handler.  The uplink retries were queued while we kept following the
// hop so they'd go out on the channel the rocket is listening on; once
// they drain (or the deadline passes) commit the mode locally.
static void serviceHopDisableDrain()
{
    if (hop_disable_drain_deadline_ms == 0) return;
    if (uplinkBusy() &&
        (int32_t)(millis() - hop_disable_drain_deadline_ms) < 0) return;
    hop_disable_drain_deadline_ms = 0;
    lora_hop_disabled = true;
    prefs.begin("lora", false);
    prefs.putUChar("hopdis", 1);
    prefs.end();
    if (hop_active_)
    {
        hop_active_       = false;
        hop_needs_retune_ = true;
    }
    hop_mode_mismatch_streak_ = 0;
    ESP_LOGI(TAG, "[BLE->UPLINK] Hop disable: drain complete — fixed mode");
    sendCurrentConfig();
}

// All passes done.  Ship results to BLE (preserving the existing
// single-result protocol so the iOS app doesn't need to change), then
// dispatch to either the auto-acquire single-channel picker (#136) or
// the skip-mask push for the hopping path (#150: user-selectable, gated
// off by default).
static void finalizeNoiseScan()
{
    ble_app.sendScanResults(scan_peak_start_mhz_, scan_peak_step_khz_,
                            scan_peak_rssi_, (uint8_t)scan_peak_count_);
    scan_results_valid_ = true;

    // #136: when our auto-acquire flow kicked off this scan, the
    // post-scan action is "pick one channel + cmd-10 move", not
    // "compute skip-mask + cmd-15 push".  Any other scan (BLE cmd 60,
    // coordinated scan) falls through to the existing skip-mask path.
    if (auto_acquire_state == AutoAcquireState::SCANNING)
    {
        autoAcquireOnScanFinalize();
    }
    else
    {
        analyzeAndPushFromCachedScan();
    }
}

// Compute the cmd 16 pause duration (ms) for a coordinated scan.  Sized
// to comfortably cover scan + cmd 15 retries + slack, and capped to the
// rocket-side max so a too-large value doesn't silently get truncated
// to a different value than we accounted for.
static uint16_t computeCoordPauseMs(float start_mhz, float stop_mhz,
                                     uint16_t step_khz, uint16_t dwell_ms)
{
    if (step_khz == 0 || stop_mhz <= start_mhz) return LORA_HOP_PAUSE_MAX_MS;
    const uint32_t span_khz   = (uint32_t)((stop_mhz - start_mhz) * 1000.0f);
    const uint32_t channels   = span_khz / step_khz + 1;
    const uint32_t scan_ms    = (uint32_t)LORA_NOISE_SCAN_PASSES * channels
                                * (uint32_t)dwell_ms;
    const uint32_t cmd15_ms   = (uint32_t)config::UPLINK_RETRIES
                                * (uint32_t)config::UPLINK_RETRY_INTERVAL_MS;
    uint32_t total = scan_ms + cmd15_ms + COORD_SCAN_PAUSE_GRACE_MS
                     + COORD_SCAN_PAUSE_SLACK_MS;
    if (total > LORA_HOP_PAUSE_MAX_MS) total = LORA_HOP_PAUSE_MAX_MS;
    return (uint16_t)total;
}

// Kick off a coordinated scan: queue cmd 16 to the rocket and stash the
// scan params for use once retries finish.  Caller has already verified
// hop_active_ and that no coordinated scan is in progress.
static void startCoordinatedScan(float start_mhz, float stop_mhz,
                                  uint16_t step_khz, uint16_t dwell_ms)
{
    coord_scan_start_mhz_ = start_mhz;
    coord_scan_stop_mhz_  = stop_mhz;
    coord_scan_step_khz_  = step_khz;
    coord_scan_dwell_ms_  = dwell_ms;

    const uint16_t pause_ms = computeCoordPauseMs(start_mhz, stop_mhz,
                                                   step_khz, dwell_ms);
    uint8_t payload[2];
    memcpy(payload, &pause_ms, 2);
    buildUplinkPacket(LORA_CMD_HOP_PAUSE, payload, 2,
                      /*target_rid=*/0xFF, config::UPLINK_RETRIES);
    coord_scan_state_    = CoordScanState::AWAITING_PAUSE;
    coord_scan_phase_ms_ = millis();
    ESP_LOGI(TAG, "[CHSET] Coordinated scan start: pausing rocket for %u ms "
                  "(scan range %.1f..%.1f MHz, %u kHz, %u ms dwell)",
             (unsigned)pause_ms, (double)start_mhz, (double)stop_mhz,
             (unsigned)step_khz, (unsigned)dwell_ms);
}

// Drive the coordinated-scan state machine.  Called every loop iteration
// before the scan-done detection so that AWAITING_PAUSE → SCANNING can
// kick off a fresh scan in the same iteration where it transitions.
static void serviceCoordinatedScan()
{
    if (coord_scan_state_ == CoordScanState::IDLE) return;

    const uint32_t now = millis();
    switch (coord_scan_state_)
    {
        case CoordScanState::IDLE: return;
        case CoordScanState::AWAITING_PAUSE:
        {
            // Cmd 16 retries still going — push out the grace anchor so
            // we wait COORD_SCAN_PAUSE_GRACE_MS after the *last* retry,
            // not after we queued the command.
            if (uplinkBusy())
            {
                coord_scan_phase_ms_ = now;
                return;
            }
            if ((now - coord_scan_phase_ms_) < COORD_SCAN_PAUSE_GRACE_MS) return;

            // Cmd 16 has been pushed (8 retries / ~800 ms).  We don't
            // get an explicit ack — the rocket pause is fire-and-forget.
            // Reconfigure the BS radio to lora_freq_mhz so we (a) can
            // verify the rocket is actually parked there, and (b) the
            // scan starts from a known frequency.
            if (!lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                         lora_cr, lora_tx_power))
            {
                ESP_LOGE(TAG, "[CHSET] Coord scan: reconfigure to %.2f MHz failed — abandoning",
                         (double)lora_freq_mhz);
                coord_scan_state_ = CoordScanState::IDLE;
                return;
            }
            (void)lora_comms.startReceive();
            // Drop hop tracking flags so the post-scan packet from the
            // rocket re-enters hop following cleanly (matching a fresh
            // bootstrap, not a continuation).
            hop_active_       = false;
            hop_needs_retune_ = false;

            if (!startNoiseScan(coord_scan_start_mhz_, coord_scan_stop_mhz_,
                                 coord_scan_step_khz_, coord_scan_dwell_ms_))
            {
                ESP_LOGE(TAG, "[CHSET] Coord scan: startNoiseScan failed — abandoning");
                coord_scan_state_ = CoordScanState::IDLE;
                return;
            }
            coord_scan_state_    = CoordScanState::SCANNING;
            coord_scan_phase_ms_ = now;
            ESP_LOGI(TAG, "[CHSET] Coord scan: rocket should be parked, scan started");
            break;
        }
        case CoordScanState::SCANNING:
        {
            // finalizeNoiseScan() runs from the existing scan-done path
            // and (a) sets scan_results_valid_ = true, (b) queues cmd 15
            // via applyAndPushChannelSet().  Either signal works as a
            // transition trigger; combining them is most precise.
            if (scan_results_valid_ && uplinkBusy())
            {
                coord_scan_state_    = CoordScanState::PUSHING_CHSET;
                coord_scan_phase_ms_ = now;
            }
            // #570: this state had NO timeout — the only exit needed BOTH a
            // valid scan AND a queued cmd 15. A zero-sample pass (quiet /
            // wedged radio: analyzeAndPushFromCachedScan early-returns, no
            // cmd 15) or a RejectedFull cmd 15 left it stuck in SCANNING
            // forever, which suppresses the hop-silence fallback and rejects
            // every future scan until reboot. Budget = the driver scan's own
            // worst case (steps × (dwell+250 ms) + 5 s, the #567 sizing) so
            // it can never fire on a healthy slow scan; on expiry drop to
            // IDLE and let the normal recovery machinery take over.
            else if ((now - coord_scan_phase_ms_) > coordScanScanningBudgetMs())
            {
                ESP_LOGW(TAG, "[CHSET] Coord scan: no pushable result within "
                              "%lu ms (empty scan or cmd-15 not queued) — "
                              "abandoning to normal recovery",
                         (unsigned long)coordScanScanningBudgetMs());
                coord_scan_state_ = CoordScanState::IDLE;
            }
            break;
        }
        case CoordScanState::PUSHING_CHSET:
        {
            if (uplinkBusy()) return;  // cmd 15 retries still going
            // Cmd 15 has been delivered (or all retries exhausted).
            // Rocket should resume hopping when its pause deadline hits.
            // Snapshot hop_last_rx_ms_ so we can detect a fresh RX.
            coord_scan_resume_anchor_ms_ = hop_last_rx_ms_;
            coord_scan_state_            = CoordScanState::RESUMING;
            coord_scan_phase_ms_         = now;
            ESP_LOGI(TAG, "[CHSET] Coord scan: cmd 15 pushed, awaiting rocket hop resume");
            break;
        }
        case CoordScanState::RESUMING:
        {
            // The rocket-side pause expires and a bootstrap packet hits
            // us on lora_freq_mhz.  The existing RX path adopts the new
            // hop_idx_ and sets hop_active_ + hop_last_rx_ms_.  Detect
            // either: a fresh RX (anchor changed) or a timeout.
            if (hop_last_rx_ms_ != coord_scan_resume_anchor_ms_)
            {
                ESP_LOGI(TAG, "[CHSET] Coord scan complete — hop resumed");
                coord_scan_state_ = CoordScanState::IDLE;
            }
            else if ((now - coord_scan_phase_ms_) > COORD_SCAN_RESUMING_MAX_MS)
            {
                ESP_LOGW(TAG, "[CHSET] Coord scan: rocket did not resume hop within %u ms — "
                              "letting normal recovery take over",
                         (unsigned)COORD_SCAN_RESUMING_MAX_MS);
                coord_scan_state_ = CoordScanState::IDLE;
            }
            break;
        }
    }
}

// ----------------------------------------------------------------------------
// Auto-acquire + auto-scan (#136)
// ----------------------------------------------------------------------------
// Every BS power cycle starts on LORA_FACTORY_RENDEZVOUS_MHZ.  Once we
// confirm the rocket is alive on that channel (one RX is enough; the
// rocket beacons at ~2 Hz on the ground), we run a 902..928 MHz noise
// scan, pick the single quietest channel, and drive the existing cmd-10
// transactional reconfigure to move the rocket onto it.  The txn's
// verify/rollback semantics naturally fall back to rendezvous if the
// rocket doesn't follow — exactly the handshake/ack-with-fallback the
// issue calls for.
//
// Why wait for an RX before scanning?  Running the scan immediately on
// boot would race the rocket's own boot.  If the rocket isn't up yet we
// move to a channel it can't hear cmd-10 on, the txn times out and we
// roll back, and we've burned 14 s for no reason.  Acquiring first means
// every scan we run results in a real move (or stays put because the
// rendezvous is already the quietest channel).
//
// One-shot per power cycle.  Re-runs require a BS reboot — keeps the
// behaviour predictable on the bench and means we never drop the link
// mid-flight to chase a different channel.
//
// Enum + auto_acquire_state are forward-declared near the top so
// finalizeNoiseScan() can branch on them; values:
//   AWAITING_ROCKET → No packet seen yet; just listening on rendezvous
//   GRACE_DELAY     → First packet seen; brief settle before we scan
//   SCANNING        → startNoiseScan() running — finalize will dispatch
//   COMMITTING      → cmd-10 transaction in flight — wait for IDLE
//   DONE            → Settled (committed or rolled back); inert
static uint32_t auto_acquire_grace_start_ms = 0;
// Brief settle window after first RX so the rocket has a moment to
// stabilize its beacon cadence and we aren't tearing down the link
// halfway through its first packet.  Short enough to feel responsive,
// long enough to avoid colliding with an in-flight uplink retry.
static constexpr uint32_t AUTO_ACQUIRE_GRACE_MS = 750;
// Scan parameters — matches what the iOS Frequency Scan view used to
// send via BLE cmd 60.  Wide enough to cover the entire US ISM band,
// fine enough that every BW=250 channel has a sample within ±250 kHz.
static constexpr float    AUTO_ACQUIRE_SCAN_START_MHZ = LORA_BAND_LO_MHZ;
static constexpr float    AUTO_ACQUIRE_SCAN_STOP_MHZ  = LORA_BAND_HI_MHZ;
static constexpr uint16_t AUTO_ACQUIRE_SCAN_STEP_KHZ  = 500;
static constexpr uint16_t AUTO_ACQUIRE_SCAN_DWELL_MS  = 30;

// ==========================================================================
// SECTION: Auto-acquire
// ==========================================================================
// Called from finalizeNoiseScan() when the auto-acquire scan completes.
// Walks the scan grid, picks the quietest channel snapped to the BW
// table, and hands off to the existing cmd-10 transaction.  If the
// quietest channel is the rendezvous we're already on, short-circuit
// to DONE rather than burning a verify window on a no-op move.
static void autoAcquireOnScanFinalize()
{
    if (scan_peak_count_ == 0)
    {
        ESP_LOGW(TAG, "[AUTO] Scan finalized with no samples — staying on %.2f MHz",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    float scan_freqs[TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    for (size_t i = 0; i < scan_peak_count_; i++)
    {
        scan_freqs[i] = scan_peak_start_mhz_
                        + (scan_peak_step_khz_ * (float)i) / 1000.0f;
    }
    const float quietest = loraPickQuietestChannelMHz(
        scan_freqs, scan_peak_rssi_, scan_peak_count_, lora_bw_khz);

    // No-op move detection.  loraChannelMHz() returns multiples of the
    // BW spacing offset from LORA_BAND_LO_MHZ + bw/2, so equality here
    // is exact in principle, but compare with a tolerance well below the
    // channel spacing just to be safe against float quirks.
    if (fabsf(quietest - lora_freq_mhz) < 0.05f)
    {
        ESP_LOGI(TAG, "[AUTO] Rendezvous %.2f MHz is already the quietest — staying put",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    if (startLoRaTransaction(quietest, lora_bw_khz, lora_sf, lora_cr, lora_tx_power))
    {
        auto_acquire_state = AutoAcquireState::COMMITTING;
        ESP_LOGI(TAG, "[AUTO] Scan done — moving rocket to %.2f MHz (was %.2f MHz)",
                 (double)quietest, (double)lora_freq_mhz);
    }
    else
    {
        // startLoRaTransaction refuses if freq is locked for flight or a
        // txn is already in progress.  Neither is expected this early in
        // a power cycle, but if it happens we just stay on rendezvous.
        ESP_LOGW(TAG, "[AUTO] startLoRaTransaction refused — staying on %.2f MHz",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
    }
}

// Drive the auto-acquire state machine.  Called every loop iteration.
// Cheap when in AWAITING_ROCKET / DONE — most calls are a single switch.
//
// #136 v2: scan-and-move was removed.  The whole point of the issue is to
// stay on the hardcoded rendezvous (915 MHz SF8 BW250) for the duration
// of the test so we can measure raw link performance.  Picking a
// "quieter" channel and moving via cmd 10 adds a fragile transaction
// where any cmd-10 loss makes the BS roll back to 915 — which is
// exactly the no-op we'd have wanted in the first place.  Field-tested
// on 2026-05-11: the BS picked 923.5, the OC didn't follow, BS rolled
// back, and the user (rightly) flagged the BS as no longer "on a
// single frequency".
//
// State machine now just waits for first RX and declares success.
// #150 decision (2026-07-15): the scan-and-move pathway stays retired
// even with hopping re-enabled — hopping starts over the FULL channel
// set (trivially satisfies the FCC floor) and the operator's manual
// Frequency Scan pushes an optional skip-mask via cmd 15 instead.  The
// scan helpers (autoAcquireOnScanFinalize, AUTO_ACQUIRE_SCAN_*) remain
// in source only as dead code; delete them if they rot.
static void serviceAutoAcquire()
{
    if (auto_acquire_state == AutoAcquireState::DONE) return;

    // Only the AWAITING_ROCKET state is reachable now; the rest of the
    // enum is dead-code preserved for #150's follow-up.  Belt-and-braces:
    // if anything ever leaves us in a non-AWAITING state, flush to DONE
    // so we don't get stuck.
    if (auto_acquire_state != AutoAcquireState::AWAITING_ROCKET)
    {
        ESP_LOGW(TAG, "[AUTO] Unexpected state %d — forcing DONE",
                 (int)auto_acquire_state);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    if (freqLockedForFlight())
    {
        ESP_LOGW(TAG, "[AUTO] Rocket INFLIGHT before first acquire — done");
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }
    if (last_packet_ms != 0)
    {
        ESP_LOGI(TAG, "[AUTO] First rocket packet acquired — locked on %.2f MHz "
                      "for this flight (no scan, no move)",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
    }
}

// ==========================================================================
// SECTION: Boot setup
// ==========================================================================
static void setup_bs()
{
    // NVS first — before BLE/LoRa and before any Preferences use (#500).
    // The BS was missing the block OC/FC have always had, so nvs_open()
    // returned ESP_ERR_NVS_NOT_INITIALIZED: every Preferences read fell
    // back to the caller's default and every write no-op'd, silently. The
    // PHY couldn't cache its RF calibration either, forcing a full recal
    // on every boot.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK)
    {
        ESP_LOGE(TAG, "[NVS] nvs_flash_init failed: %s — settings will NOT persist",
                 esp_err_to_name(nvs_err));
    }

    delay(500);
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  TinkerRocket Base Station");
    // Board banner = the wrong-build-dir flash guard (same lesson as the
    // FC/OC -B build_v8 gotcha): if this line doesn't match the physical
    // board, stop and flash the right variant.
    ESP_LOGI(TAG, "  Board: %s (TR_BS_BOARD=%d)", config::BOARD_NAME, TR_BS_BOARD);
    ESP_LOGI(TAG, "======================================");

    // OTA boot-state check (#8). If this image was just OTA-installed it
    // boots PENDING_VERIFY; we hold off the "valid" mark until we've seen
    // BLE work end-to-end (first telemetry sent while connected). If the
    // app crashes or hangs before that, the bootloader auto-rolls back to
    // ota_0 on the next boot.
    {
        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
        if (running && esp_ota_get_state_partition(running, &state) == ESP_OK)
        {
            if (state == ESP_OTA_IMG_PENDING_VERIFY)
            {
                ESP_LOGW(TAG, "OTA: running on PENDING_VERIFY image (partition '%s' @ 0x%08x); "
                              "rollback armed until first telemetry round-trip",
                         running->label, (unsigned)running->address);
                g_ota_pending_verify = true;
            }
            else
            {
                ESP_LOGI(TAG, "OTA: running partition '%s' state=%d", running->label, (int)state);
            }
        }
    }

    // BLE time-sync arrives as broken-down UTC; pin TZ so mktime() in the
    // log-rename helper (#168) interprets it as UTC seconds, not local.
    setenv("TZ", "UTC0", 1);
    tzset();

    // Initialize storage. V2/V3 -> wear-leveled FAT on the external SPI NAND;
    // V1 -> SD over SDMMC 4-bit. Either way, fall back to SPIFFS on
    // internal flash. Logging is backend-agnostic (uses SD_MOUNT_POINT + the
    // standard file API), so only the mount differs per board.
    {
        esp_err_t ret = ESP_ERR_NOT_FOUND;
        if (config::HAS_EXT_NAND)
        {
            ret = mountExternalFlashFat();   // logs its own info; repoints SD_MOUNT_POINT on success
        }
        else if (config::HAS_SDMMC)
        {
            sdmmc_host_t host = SDMMC_HOST_DEFAULT();
            host.max_freq_khz = SDMMC_FREQ_DEFAULT;

            sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
            slot.width = 4;
            slot.clk = (gpio_num_t)config::SD_CLK;
            slot.cmd = (gpio_num_t)config::SD_CMD;
            slot.d0  = (gpio_num_t)config::SD_D0;
            slot.d1  = (gpio_num_t)config::SD_D1;
            slot.d2  = (gpio_num_t)config::SD_D2;
            slot.d3  = (gpio_num_t)config::SD_D3;
            slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

            esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {};
            mount_cfg.format_if_mount_failed = false;
            mount_cfg.max_files = 5;
            mount_cfg.allocation_unit_size = 16 * 1024;

            ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &sd_card);
            if (ret == ESP_OK)
            {
                sdmmc_card_print_info(stdout, sd_card);

                // By mount point, not FAT drive "0:" — see bsQueryStorage().
                uint64_t total = 0, free_bytes = 0;
                if (esp_vfs_fat_info(SD_MOUNT_POINT, &total, &free_bytes) == ESP_OK)
                {
                    const uint64_t used = (total > free_bytes) ? (total - free_bytes) : 0;
                    ESP_LOGI(TAG, "SD card mounted: %llu MB total, %llu MB used, %llu MB free",
                             (unsigned long long)(total / (1024 * 1024)),
                             (unsigned long long)(used / (1024 * 1024)),
                             (unsigned long long)(free_bytes / (1024 * 1024)));
                }
            }
        }
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Primary storage mount failed (0x%x) — falling back to internal flash (SPIFFS)",
                     (int)ret);
            sd_card = nullptr;

            esp_vfs_spiffs_conf_t spiffs_conf = {};
            spiffs_conf.base_path              = "/flash";
            spiffs_conf.partition_label        = SPIFFS_PARTITION_LABEL;
            spiffs_conf.max_files              = 5;
            spiffs_conf.format_if_mount_failed = true;

            esp_err_t sret = esp_vfs_spiffs_register(&spiffs_conf);
            if (sret != ESP_OK)
            {
                ESP_LOGE(TAG, "SPIFFS fallback mount FAILED (0x%x) — no logging available",
                         (int)sret);
            }
            else
            {
                SD_MOUNT_POINT = "/flash";
                using_internal_flash = true;

                size_t total = 0, used = 0;
                if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &total, &used) == ESP_OK)
                {
                    ESP_LOGI(TAG, "SPIFFS mounted at %s: %u KB total, %u KB used, %u KB free",
                             SD_MOUNT_POINT,
                             (unsigned)(total / 1024),
                             (unsigned)(used / 1024),
                             (unsigned)((total - used) / 1024));
                }
            }
        }

        // #761: on a board fitted with primary storage, landing on SPIFFS is a
        // demotion, not a configuration — this boot logs into ~1.9 MB instead
        // of 512 MB and a flight will truncate. Latch it so the storage frame
        // can say so (BSS_FLAG_FALLBACK) rather than leaving the operator to
        // infer it from a backend label, and say it once, loudly, in the boot
        // log with the step that actually failed.
        storage_demoted = bs_storage_policy::demoted(
            config::HAS_EXT_NAND || config::HAS_SDMMC, using_internal_flash);
        if (storage_demoted)
        {
            ESP_LOGE(TAG, "**** STORAGE DEMOTED: logging to internal flash (~1.9 MB), NOT the "
                          "%s ****", config::HAS_EXT_NAND ? "512 MB NAND" : "SD card");
            if (config::HAS_EXT_NAND)
            {
                ESP_LOGE(TAG, "**** last failing step: %s (0x%x) after %u attempt(s); "
                              "NAND mfr id read back 0x%02x (expect 0xcd) ****",
                         nandStepName(nand_fail_step), (int)nand_fail_err,
                         nand_attempts, nand_mfr_id);
            }
            ESP_LOGE(TAG, "**** a flight logged in this state truncates at the partition size ****");
        }

        // Write test covers both backends
        if (sd_card || using_internal_flash || using_external_flash)
        {
            char test_path[48];
            snprintf(test_path, sizeof(test_path), "%s/.write_test", SD_MOUNT_POINT);
            FILE* test = fopen(test_path, "w");
            if (test)
            {
                fprintf(test, "ok\n");
                fclose(test);
                remove(test_path);
                ESP_LOGI(TAG, "Storage write test: OK (%s)",
                         using_internal_flash ? "internal flash" : using_external_flash ? "external NAND" : "SD card");
            }
            else
            {
                ESP_LOGE(TAG, "Storage write test FAILED! errno=%d (%s)",
                         errno, strerror(errno));
            }
        }
    }

    // Configure I2C and auto-detect the fuel gauge (one firmware, both PCBs):
    // probe the new-PCB BQ27Z746 (0x55) first, then the original MAX17205 (0x36).
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port     = I2C_NUM_0;
        bus_cfg.sda_io_num   = (gpio_num_t)config::I2C_SDA_PIN;
        bus_cfg.scl_io_num   = (gpio_num_t)config::I2C_SCL_PIN;
        bus_cfg.clk_source   = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = false;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "I2C bus init failed (%d) — battery readings unavailable", (int)err);
        }
        else if (i2c_master_probe(i2c_bus, config::BQ27Z746_ADDR, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            // New PCB: BQ27Z746 single-cell gauge + protection.
            TR_BQ27Z746_Config bq_cfg;
            bq_cfg.current_invert   = false;  // TODO: verify SRP/SRN polarity on the new board
            bq_cfg.auto_enable_fets = false;  // FET enable/maintenance owned by maintainBatteryFets()
                                              //   (runs via updateBattery() at boot + every cycle)
            if (bq_gauge.begin(i2c_bus, bq_cfg, config::I2C_FREQ_HZ) == ESP_OK)
            {
                gauge_kind = GaugeKind::BQ27Z746;
                fuel_gauge_present = true;
                ESP_LOGI(TAG, "BQ27Z746 fuel gauge found on I2C (0x%02X), devtype 0x%04X",
                         config::BQ27Z746_ADDR, bq_gauge.deviceType());
                updateBattery();
                ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                         (double)bs_voltage, (double)bs_soc, (double)bs_current);
                bq_gauge.logDiagnostics(TAG);

                // New-PCB gauge bring-up: the BQ27Z746 ships with a 5300 mAh
                // default DesignCapacity and was never configured for our 2800 mAh
                // 18650, so SoC/capacity are meaningless until provisioned. One-shot,
                // self-gated (no-op once correct), read-back verified.
                bq_gauge.provisionDesignCapacity((int16_t)config::BATTERY_DESIGN_MAH);
                // Read-only diagnostic: raw coulomb-counter ADC. If this tracks real
                // battery current (charge vs discharge) the current path is good and
                // only CC Gain needs a known-current calibration; if it's frozen, the
                // shunt/Kelvin sense is the problem.
                int16_t bq_raw_cc = 0;
                (void)bq_gauge.readRawCcCurrent(bq_raw_cc);
            }
            else
            {
                ESP_LOGW(TAG, "BQ27Z746 probe succeeded but begin() failed");
            }
        }
        else if (i2c_master_probe(i2c_bus, config::MAX17205_ADDR, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            // 0x36 is shared by two ModelGauge m5 parts: MAX17303 (V3 PCB,
            // 1S + protector) and MAX17205 (V1 PCB, 2S). Try the MAX17303
            // driver first — its begin() reads DevName and hands the address
            // back (ESP_ERR_NOT_FOUND) when the part isn't a MAX1730x.
            TR_MAX17303_Config m3_cfg;
            m3_cfg.design_mah     = config::BATTERY_DESIGN_MAH;
            m3_cfg.rsense_mohm    = config::RSENSE_MOHM;   // TODO: verify V3 Rsense on the bench
            m3_cfg.current_invert = false;                  // TODO: verify CSP/CSN polarity on V3
            m3_cfg.assume_when_unidentified = config::EXPECT_MAX17303;

            if (max17303_gauge.begin(i2c_bus, m3_cfg, config::I2C_FREQ_HZ) == ESP_OK)
            {
                gauge_kind = GaugeKind::MAX17303;
                fuel_gauge_present = true;
                ESP_LOGI(TAG, "MAX17303 fuel gauge found on I2C (0x%02X), DevName 0x%04X",
                         config::MAX17303_ADDR, max17303_gauge.devName());
                max17303_gauge.initIfNeeded();
                updateBattery();
                ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                         (double)bs_voltage, (double)bs_soc, (double)bs_current);
                max17303_gauge.logDiagnostics(TAG);
            }
            else
            {
                // Original PCB: MAX17205 gauge.
                TR_MAX17205G_Config fg_cfg;
                fg_cfg.design_mah     = config::BATTERY_DESIGN_MAH;
                fg_cfg.rsense_mohm    = config::RSENSE_MOHM;
                fg_cfg.current_invert = true;   // CSP/CSN swapped on schematic (U7) vs.
                                                //   datasheet typical app circuit; flips
                                                //   displayed current so charge reads
                                                //   positive. Driver uses VFSOC for SoC
                                                //   because the chip's m5 can't be told.
                fg_cfg.num_cells      = config::NUM_BATTERY_CELLS;

                if (fuel_gauge.begin(i2c_bus, fg_cfg, config::I2C_FREQ_HZ) == ESP_OK)
                {
                    gauge_kind = GaugeKind::MAX17205;
                    fuel_gauge_present = true;
                    ESP_LOGI(TAG, "MAX17205G fuel gauge found on I2C (0x%02X)", config::MAX17205_ADDR);
                    fuel_gauge.initIfNeeded();
                    updateBattery();
                    ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                             (double)bs_voltage, (double)bs_soc, (double)bs_current);
                    fuel_gauge.logDiagnostics(TAG);
                }
                else
                {
                    ESP_LOGW(TAG, "MAX17205G probe succeeded but begin() failed");
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "No fuel gauge found (neither BQ27Z746 0x55 nor MAX17205/MAX17303 0x36) — battery readings unavailable");
        }
    }

    // #835 item 2: boards with no gauge measure the cell through a divider
    // instead. Unconditional on the board flag rather than on the probe
    // result: a gauged board whose gauge failed to answer should stay silent
    // (its divider does not exist) rather than start reporting a voltage
    // from an unconnected pin.
    if constexpr (!config::HAS_FUEL_GAUGE)
    {
        initBatteryAdc();
        updateBattery();   // seed bs_voltage/bs_soc before the first telemetry
    }

    // V3: external flight-pack charger on the same I2C bus as the gauge.
    // begin() succeeds even with no charge input attached (the chip is
    // unpowered then); the periodic service() detects plug-in and applies
    // the charge config each time.
    if (config::HAS_PACK_CHARGER && i2c_bus != nullptr)
    {
        TR_MP2672_Config chg_cfg;
        chg_cfg.vbatt_reg_code = config::PACK_VBATT_REG_CODE;
        chg_cfg.icc_code       = config::PACK_CHARGE_ICC_CODE;
        chg_cfg.chg_timer_code = config::PACK_CHG_TIMER_CODE;
        if (pack_charger.begin(i2c_bus, chg_cfg, config::I2C_FREQ_HZ) != ESP_OK)
        {
            ESP_LOGW(TAG, "MP2672 pack-charger driver init failed");
        }
    }

    // Load saved LoRa config from NVS (write config.h defaults if empty)
    prefs.begin("lora", false);  // read-write

    // NVS schema gate (#105 follow-up).  If the stored schema version
    // doesn't match the current build, wipe the lora namespace so the
    // device falls back to the shared factory rendezvous and re-syncs
    // with the BS via the standard rendezvous flow.  Without this, a
    // re-flashed device can come up on a stale operating freq the BS
    // doesn't know about — exactly the chicken-and-egg that motivated
    // this gate.  Single namespace clear is fine: every LoRa-related
    // key (freq, sf, bw, cr, txpwr, hopdis, chset_*, plus the now-defunct
    // rdv_mhz from v2 schema) lives under "lora", so one clear() takes
    // them all together.
    {
        const uint8_t stored_v = prefs.getUChar("schemv", 0);
        if (stored_v != LORA_NVS_SCHEMA_VERSION)
        {
            ESP_LOGW(TAG, "[CFG] LoRa NVS schema mismatch (stored=%u, current=%u) — clearing",
                     (unsigned)stored_v, (unsigned)LORA_NVS_SCHEMA_VERSION);
            prefs.clear();
            prefs.putUChar("schemv", LORA_NVS_SCHEMA_VERSION);
        }
    }

    if (!prefs.isKey("freq"))
    {
        // First boot or NVS erased — seed with config.h factory defaults
        prefs.putFloat("freq",  config::LORA_FREQ_MHZ);
        prefs.putUChar("sf",    config::LORA_SF);
        prefs.putFloat("bw",    config::LORA_BW_KHZ);
        prefs.putUChar("cr",    config::LORA_CR);
        prefs.putChar("txpwr",  config::LORA_TX_POWER_DBM);
        prefs.putUChar("hopdis", 1);  // #150: fixed mode is the default
        ESP_LOGI(TAG, "[CFG] LoRa NVS empty -- wrote config.h defaults");
    }
    lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
    lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
    lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
    lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
    lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
    lora_hop_disabled = prefs.getUChar("hopdis", 1) != 0;  // #150: default fixed mode
    prefs.end();
    ESP_LOGI(TAG, "[CFG] LoRa NVS (cached): %.1f MHz SF%u BW%.0f CR%u %d dBm hop_disabled=%d",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
             (int)lora_hop_disabled);

    // Issue #136: every BS power cycle starts on the hardcoded rendezvous
    // frequency + preset, regardless of any NVS values left over from
    // prior sessions.  This guarantees that BS and rocket meet on a known
    // channel.  cmd-10 commits still write through to NVS during a
    // session for visibility and BLE-readback, but those values no longer
    // drive boot config.
    // #150: lora_hop_disabled is deliberately NOT in this override
    // anymore — the link mode is user-selected (app Fixed/Hopping picker
    // via BLE cmd 17) and honors NVS across reboots.  Schema v4 wiped any
    // stale pre-#150 hopdis, and the default is 1 (fixed mode).
    lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
    lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
    lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
    lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
    lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;
    ESP_LOGI(TAG, "[CFG] LoRa boot (forced rendezvous): %.1f MHz SF%u BW%.0f CR%u %d dBm",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

    // Channel-set: skip-mask from the most recent pre-launch scan
    // (#40 / #41 phase 3).  Falls back to no-skips if never scanned, or
    // if the stored mask was generated for a different BW.  Rendezvous
    // freq is no longer scan-selected; both sides use the shared
    // hardcoded LORA_FACTORY_RENDEZVOUS_MHZ (#105).
    loadChannelSetFromNvs();
    if (skip_mask_n_ > 0)
    {
        uint8_t active = 0;
        for (uint8_t i = 0; i < skip_mask_n_; i++)
            if (!loraSkipMaskTest(skip_mask_, i)) active++;
        ESP_LOGI(TAG, "[CHSET] NVS: %u/%u channels active",
                 (unsigned)active, (unsigned)skip_mask_n_);
    }
    else
    {
        ESP_LOGI(TAG, "[CHSET] NVS: no skip-mask");
    }

    // Load cached servo config from NVS
    prefs.begin("servo", true);
    cfg_servo_bias1 = prefs.getShort("b1",  cfg_servo_bias1);
    cfg_servo_hz    = prefs.getShort("hz",  cfg_servo_hz);
    cfg_servo_min   = prefs.getShort("min", cfg_servo_min);
    cfg_servo_max   = prefs.getShort("max", cfg_servo_max);
    prefs.end();

    // Load cached PID config from NVS
    prefs.begin("pid", true);
    cfg_pid_kp  = prefs.getFloat("kp",  cfg_pid_kp);
    cfg_pid_ki  = prefs.getFloat("ki",  cfg_pid_ki);
    cfg_pid_kd  = prefs.getFloat("kd",  cfg_pid_kd);
    cfg_pid_min = prefs.getFloat("mn",  cfg_pid_min);
    cfg_pid_max = prefs.getFloat("mx",  cfg_pid_max);
    prefs.end();
    ESP_LOGI(TAG, "[NVS] Config cache: servo bias=%d hz=%d, PID Kp=%.4f",
             cfg_servo_bias1, cfg_servo_hz, cfg_pid_kp);

    // --- Device Identity (NVS "identity" namespace) ---
    {
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(unit_id_hex, sizeof(unit_id_hex), "%02x%02x%02x%02x",
                 mac[2], mac[3], mac[4], mac[5]);

        prefs.begin("identity", false);

        // #150: the identity namespace versions SEPARATELY from the lora
        // namespace and MIGRATES instead of wiping — a wipe is what caused
        // the #133-era regression (this BS came back nid=0 while the
        // rocket kept nid=180 and the network-id filter silently dropped
        // everything).
        //
        // v0 -> v1 resets nid to the compile-time default ONCE (review
        // finding): the #136 boot-force this branch removes was RAM-only,
        // so fielded devices still store whatever nid the #133 era left
        // behind — masked, divergent, and unreachable for months.  Making
        // those live would kill the link on the first post-flash boot
        // with no over-the-air recovery (both directions are
        // nid-filtered).  The pre-upgrade EFFECTIVE nid was always the
        // default, so resetting to it is the behavior-preserving
        // migration.  `un` (user data) is preserved.
        {
            const uint8_t stored_v = prefs.getUChar("schemv", 0);
            if (stored_v != IDENTITY_NVS_SCHEMA_VERSION)
            {
                if (stored_v == 0)
                {
                    const uint8_t old_nid =
                        prefs.getUChar("nid", config::DEFAULT_NETWORK_ID);
                    if (old_nid != config::DEFAULT_NETWORK_ID)
                    {
                        ESP_LOGW(TAG, "[CFG] Identity migration: stale masked nid=%u reset to default %u",
                                 (unsigned)old_nid, (unsigned)config::DEFAULT_NETWORK_ID);
                        prefs.putUChar("nid", config::DEFAULT_NETWORK_ID);
                    }
                }
                ESP_LOGW(TAG, "[CFG] Identity NVS schema %u -> %u (migrated)",
                         (unsigned)stored_v, (unsigned)IDENTITY_NVS_SCHEMA_VERSION);
                prefs.putUChar("schemv", IDENTITY_NVS_SCHEMA_VERSION);
            }
        }

        if (!prefs.isKey("un"))
        {
            char default_name[24];
            snprintf(default_name, sizeof(default_name), "TR-B-%.4s", &unit_id_hex[4]);
            prefs.putBytes("un", default_name, strlen(default_name) + 1);
            prefs.putUChar("nid", config::DEFAULT_NETWORK_ID);
            ESP_LOGI(TAG, "[CFG] Identity NVS empty — seeded: name=%s nid=%u",
                     default_name, config::DEFAULT_NETWORK_ID);
        }
        char nvs_name_buf[24] = "TinkerBaseStation";
        size_t un_len = prefs.getBytes("un", nvs_name_buf, sizeof(nvs_name_buf) - 1);
        if (un_len > 0) nvs_name_buf[un_len] = '\0';
        strncpy(unit_name, nvs_name_buf, sizeof(unit_name) - 1);
        unit_name[sizeof(unit_name) - 1] = '\0';
        network_id = prefs.getUChar("nid", config::DEFAULT_NETWORK_ID);
        prefs.end();

        // #150: the #136 boot-time force of network_id back to default is
        // gone.  Per-network IDs are user-set again (BLE cmd 41 / the
        // network-name UI), survive reboot, and drift is now VISIBLE
        // instead of silent: the RX filter counts nid-mismatch drops and
        // surfaces them to the app ("nidd"), and identity NVS migrates
        // rather than wipes.

        ESP_LOGI(TAG, "[CFG] Identity: uid=%s name=%s nid=%u",
                 unit_id_hex, unit_name, (unsigned)network_id);

        ble_app.setName(unit_name);
    }

    // Configure LoRa radio (uses NVS-saved config or factory defaults).
    // Backend per board header (#410 pattern): V3 talks to the radio
    // daughterboard over UART; V1/V2 drive the on-board LLCC68 over SPI.
    bool radio_ok = false;
    if (config::USE_UART_RADIO_MODEM)
    {
        UartModemBackend::Config mcfg = {};
        mcfg.uart.tx_pin      = config::LORA_UART_TX_PIN;
        mcfg.uart.rx_pin      = config::LORA_UART_RX_PIN;
        mcfg.act_pin          = config::LORA_ACT_PIN;
        mcfg.preamble_len     = config::LORA_PREAMBLE_LEN;
        mcfg.crc_on           = config::LORA_CRC_ON;
        mcfg.rx_boosted_gain  = config::LORA_RX_BOOSTED_GAIN;
        mcfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
        radio_ok = lora_modem_backend.begin(mcfg, lora_freq_mhz, lora_sf,
                                            lora_bw_khz, lora_cr,
                                            lora_tx_power, config::DEBUG);
        if (!radio_ok)
        {
            // Unlike the direct-SPI boards, a missing daughterboard must not
            // brick the BS: bench bring-up runs gauge/charger/storage/BLE
            // without a radio, and the backend hot-joins a modem that BOOTs
            // later (service() keeps polling the UART).
            ESP_LOGE(TAG, "LoRa modem init FAILED — continuing radio-less "
                          "(daughterboard can hot-join)");
        }
    }
    else
    {
        TR_LoRa_Comms::Config lora_cfg = {};
        lora_cfg.enabled           = true;
        lora_cfg.cs_pin            = config::LORA_CS_PIN;
        lora_cfg.dio1_pin          = config::LORA_DIO1_PIN;
        lora_cfg.rst_pin           = config::LORA_RST_PIN;
        lora_cfg.busy_pin          = config::LORA_BUSY_PIN;
        // V2 PCB: MCU-driven RX half of the RF switch (was defined but never
        // driven — RXEN floated in RX). -1 on the original PCB (no switch).
        lora_cfg.rxen_pin          = config::LORA_RXEN_PIN;
        lora_cfg.spi_sck           = config::LORA_SPI_SCK;
        lora_cfg.spi_miso          = config::LORA_SPI_MISO;
        lora_cfg.spi_mosi          = config::LORA_SPI_MOSI;
        lora_cfg.spi_host          = SPI2_HOST;
        lora_cfg.freq_mhz          = lora_freq_mhz;
        lora_cfg.spreading_factor  = lora_sf;
        lora_cfg.bandwidth_khz     = lora_bw_khz;
        lora_cfg.coding_rate       = lora_cr;
        lora_cfg.preamble_len      = config::LORA_PREAMBLE_LEN;
        lora_cfg.tx_power_dbm      = lora_tx_power;
        lora_cfg.crc_on            = config::LORA_CRC_ON;
        lora_cfg.rx_boosted_gain   = config::LORA_RX_BOOSTED_GAIN;
        lora_cfg.syncword_private  = config::LORA_SYNCWORD_PRIVATE;

        radio_ok = lora_direct_backend.begin(lora_cfg, config::DEBUG);
        if (!radio_ok)
        {
            // On-board radio is soldered to this PCB — a failed init is a
            // hardware fault, keep the historical hard stop.
            ESP_LOGE(TAG, "LoRa init FAILED!");
            while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
        }
    }

    ESP_LOGI(TAG, "LoRa config: %.1f MHz SF%u BW%.0f kHz CR%u %d dBm",
             (double)lora_freq_mhz,
             (unsigned)lora_sf,
             (double)lora_bw_khz,
             (unsigned)lora_cr,
             (int)lora_tx_power);

    // Start continuous receive mode
    if (radio_ok && !lora_comms.startReceive())
    {
        ESP_LOGE(TAG, "LoRa startReceive FAILED!");
        if (!config::USE_UART_RADIO_MODEM)
        {
            while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
        }
    }

    // Initialize BLE app interface
    if (!ble_app.begin())
    {
        ESP_LOGE(TAG, "BLE init FAILED!");
        // Continue anyway - LoRa RX still works without BLE
    }
    else
    {
        // The live unit name, not the compile-time default: this line read
        // "TinkerBaseStation" on a board named "BaseStation V4", which is
        // exactly the wrong thing to print while chasing a naming bug.
        ESP_LOGI(TAG, "BLE advertising as '%s'", unit_name);
    }

    last_stats_ms = millis();
    ESP_LOGI(TAG, "Listening for rocket telemetry...");
}

// ==========================================================================
// SECTION: Main loop
// ==========================================================================
static void loop_bs()
{
    // #289: set when the RX path sends a (fresh) telemetry notification this
    // iteration, so the periodic battery push below can skip a redundant second
    // notify — two back-to-back sends double the notify pressure exactly when
    // the queue may be full, making an RX-path drop more likely.
    bool rx_sent_telem_this_iter = false;

    // ==========================================================================
    // SECTION: LoRa service and hop management
    // ==========================================================================
    // Service LoRa (complete any pending TX before checking for RX)
    lora_comms.service();
    lora_comms.pollDio1();          // Fallback if DIO1 interrupt doesn't fire
    lora_comms.serviceTxWatchdog(); // Force-clear stuck tx_ongoing_ (#105)

    // Drive the coordinated-scan state machine first so AWAITING_PAUSE
    // → SCANNING transitions can kick off a scan in the same iteration
    // serviceUplink finishes the cmd 16 retries (#90).
    serviceCoordinatedScan();

    // Hop-silence fallback: if we've been following a hopping rocket but
    // haven't heard from it in HOP_SILENCE_FALLBACK_MS, drop back to the
    // static channel so the standard silence / recovery machinery can
    // take over.  Without this, hop_active_ would pin the BS to a hop
    // channel forever after a lost rocket — recovery itself is
    // suppressed while hop_active_ for the opposite reason (don't
    // recover during normal hop misses).  Suppressed during a
    // coordinated scan (#90): the BS sweeps off the hop channel and the
    // rocket is parked on lora_freq_mhz, so silence is expected.
    if (hop_active_ && coord_scan_state_ == CoordScanState::IDLE &&
        hop_last_rx_ms_ != 0 &&
        (millis() - hop_last_rx_ms_) > HOP_SILENCE_FALLBACK_MS)
    {
        const uint32_t silence_ms = millis() - hop_last_rx_ms_;
        const uint32_t dur_ms     = (hop_session_started_ms != 0)
            ? (millis() - hop_session_started_ms) : 0;
        ESP_LOGW(TAG, "[HOP] Silence > %u ms — falling back to static channel",
                 (unsigned)HOP_SILENCE_FALLBACK_MS);
        hop_silence_events_count++;
        char ev[96];
        snprintf(ev, sizeof(ev),
                 "hop_silence duration_ms=%u pkts=%u loss=%u silence_ms=%u",
                 (unsigned)dur_ms,
                 (unsigned)hop_session_total_pkts,
                 (unsigned)hop_session_observed_loss,
                 (unsigned)silence_ms);
        // Log the event row BEFORE flipping hop_active_ off so currentRxFreqMHz()
        // still reports the hop channel we were sitting on at the moment of loss.
        logHopEvent(ev, currentRxFreqMHz());
        hop_session_started_ms    = 0;
        hop_session_total_pkts    = 0;
        hop_session_observed_loss = 0;
        hop_active_       = false;
        hop_needs_retune_ = true;
    }

    // Honour any pending hop retune — the RX path defers the actual
    // setFrequency call to here so the radio finishes any in-flight
    // operation first (#40 / #41 phase 2a).
    if (hop_needs_retune_ && lora_comms.canSend())
    {
        const float target = hop_active_
            ? loraChannelMHz(lora_bw_khz, hop_idx_)
            : lora_freq_mhz;
        if (target > 0.0f)
        {
            (void)lora_comms.hopToFrequencyMHz(target);
        }
        hop_needs_retune_ = false;
    }

    // ==========================================================================
    // SECTION: Packet receive: beacon, telemetry, tracker
    // ==========================================================================
    // Check for received packet
    uint8_t rx_buf[256];
    size_t rx_len = 0;

    bool rx_packet_accepted = false;
    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        // SNR floor (#90 follow-up).  Defends against noise-floor
        // false-positive decodes that confuse recovery + hop tracking.
        // The threshold tracks the radio's currently-configured SF
        // because Phase A rendezvous can run at a different SF than
        // operating mode.
        TR_LoRa_Comms::Stats ls_pre = {};
        lora_comms.getStats(ls_pre);
        const float min_snr = loraMinValidSnrDb(lora_comms.currentSpreadingFactor());
        if (ls_pre.last_snr < min_snr)
        {
            lora_low_snr_drops++;
            ESP_LOGW(TAG, "[RX] Drop: SNR %.1f dB < %.1f dB floor "
                          "(SF%u) — likely noise-floor false positive",
                     (double)ls_pre.last_snr, (double)min_snr,
                     (unsigned)lora_comms.currentSpreadingFactor());
            // Fall through; the `if (rx_packet_accepted)` guard below
            // skips state updates while letting the rest of loop_bs
            // (serviceUplink etc.) keep running on schedule.
        }
        else
        {
            rx_packet_accepted = true;
        }
    }

    if (rx_packet_accepted)
    {
        // #384: last_packet_ms deliberately NOT bumped here — this point is
        // before the network-id filter, so foreign-network packets (or any
        // SNR-passing decode of the right shape) counted as proof of life.
        // That suppressed silence recovery and, worse, satisfied the cmd-10
        // VERIFYING predicate ("any last_packet_ms increase = the rocket
        // followed us"), committing a new frequency to NVS on the strength
        // of somebody else's traffic and stranding the BS on a channel our
        // rocket isn't on. The bump now lives in the two netid-matched
        // branches below (beacon + telemetry).

        // --- Name beacon: [0xBE][network_id][rocket_id][unit_name...] ---
        // #384: a telemetry frame whose first byte (network_id) happens to
        // equal LORA_BEACON_SYNC (0xBE = nid 190) would otherwise parse as a
        // beacon and shadow 100% of telemetry. Inert today (boot forces
        // nid 0), so require the length to disambiguate: beacons are short,
        // telemetry is exactly SIZE_OF_LORA_FAST or SIZE_OF_LORA_SLOW.
        if (rx_len >= 3 && rx_len != SIZE_OF_LORA_FAST && rx_len != SIZE_OF_LORA_SLOW
            && rx_buf[0] == LORA_BEACON_SYNC)
        {
            uint8_t bcn_nid = rx_buf[1];
            uint8_t bcn_rid = rx_buf[2];
            if (bcn_nid == network_id)
            {
                last_packet_ms = millis();  // netid-matched proof of life (#384)
                int slot = findOrAllocRocket(bcn_rid);
                if (slot >= 0 && rx_len > 3)
                {
                    size_t name_len = rx_len - 3;
                    if (name_len >= sizeof(tracked_rockets[slot].unit_name))
                        name_len = sizeof(tracked_rockets[slot].unit_name) - 1;
                    memcpy(tracked_rockets[slot].unit_name, &rx_buf[3], name_len);
                    tracked_rockets[slot].unit_name[name_len] = '\0';
                    tracked_rockets[slot].last_seen_ms = millis();
                    ESP_LOGI(TAG, "[RX] Beacon: rid=%u name=%s",
                             bcn_rid, tracked_rockets[slot].unit_name);
                }
            }
        }
        // --- Telemetry packet: FAST (55 B) or SLOW (22 B) ---
        // (#570: no literal here — a stale "(73)" outlived two frame diets.)
        else if (rx_len == SIZE_OF_LORA_FAST || rx_len == SIZE_OF_LORA_SLOW)
        {
            // #850: both frames open with the same 7-byte LoRaFrameHeader, so
            // the routing fields and ver_type can be read before we know which
            // frame this is. That is the whole point of the shared prefix.
            LoRaFrameHeader hdr{};
            memcpy(&hdr, rx_buf, sizeof(hdr));
            const uint8_t frame_ver  = loraFrameVersion(hdr.ver_type);
            const uint8_t frame_type = loraFrameType(hdr.ver_type);

            // #837 item 14: the version is now CHECKED, not decorative. Before
            // this it was never transmitted at all and `rx_len` was the only
            // guard — which two frame sizes would have quietly defeated, since
            // a stale build's 65 B frame is neither of ours but a future
            // layout change at the same length would have been invisible.
            // Type and length must agree. A frame claiming SLOW at 55 bytes is
            // corrupt in a way the CRC did not catch, and decoding it would
            // read 33 bytes past the struct.
            const bool len_ok = (frame_type == LORA_FRAME_SLOW)
                                    ? (rx_len == SIZE_OF_LORA_SLOW)
                                    : (frame_type == LORA_FRAME_FAST && rx_len == SIZE_OF_LORA_FAST);

            // Header only for now — enough for the network filter and the slot
            // lookup below. The payload is merged into the per-rocket
            // accumulator once we know which rocket it belongs to.
            LoRaDataSI decoded = {};
            sensor_converter.unpackLoRaHeader(hdr, decoded);

            // Drop ladder, most-fundamental first: a frame we cannot parse,
            // then one we can parse but is malformed, then one that parses
            // fine but belongs to somebody else's network.
            if (frame_ver != LORA_PROTO_VERSION)
            {
                lora_size_mismatch_drops++;
                lora_size_last_drop_ms = millis();
                if (lora_size_mismatch_drops == 1 || lora_size_mismatch_drops % 100 == 0)
                {
                    ESP_LOGW(TAG, "[RX] Drop: LoRa proto v%u != ours v%u (%lu dropped) "
                                  "— rocket and base station flashed from different builds",
                             (unsigned)frame_ver, (unsigned)LORA_PROTO_VERSION,
                             (unsigned long)lora_size_mismatch_drops);
                }
            }
            else if (!len_ok)
            {
                lora_size_mismatch_drops++;
                lora_size_last_drop_ms = millis();
                ESP_LOGW(TAG, "[RX] Drop: frame type %u with %u bytes (fast=%u slow=%u)",
                         (unsigned)frame_type, (unsigned)rx_len,
                         (unsigned)SIZE_OF_LORA_FAST, (unsigned)SIZE_OF_LORA_SLOW);
            }
            // Filter by network_id
            else if (decoded.network_id != network_id)
            {
                // Not our network — drop.  Previously fully silent (#329): if
                // the rocket's nid drifts from the BS default (0, #136), every
                // packet lands here and the flight log stays empty with no
                // clue why.  Count it + a throttled warning so it's attributable.
                lora_netid_mismatch_drops++;
                lora_netid_last_drop_ms = millis();   // #150: recency for BLE reporting
                if (lora_netid_mismatch_drops == 1 ||
                    lora_netid_mismatch_drops % 100 == 0)
                {
                    ESP_LOGW(TAG, "[RX] Drop: network_id %u != ours %u "
                                  "(%lu dropped) — rocket on a different network?",
                             (unsigned)decoded.network_id, (unsigned)network_id,
                             (unsigned long)lora_netid_mismatch_drops);
                }
            }
            else
            {
                last_packet_ms = millis();  // netid-matched proof of life (#384)
                // #506: learn the downlink cadence from the TELEMETRY stream only.
                // Beacons are sporadic (~2 s) and would corrupt the estimate; this
                // is the steady ~500 ms stream whose gaps the uplink aims for.
                // #570: and ONLY from the FOCUSED rocket — with two rockets up,
                // feeding every packet drove the learned period toward the
                // inter-rocket spacing (~100-400 ms) instead of the true
                // per-rocket ~500 ms, collapsing the #506 TX window until
                // mayStartTx stopped gating and uplinks fired blind over
                // downlinks. (No focus yet → isFocusedRocket is true for all,
                // so single-rocket learning is unchanged.)
                if (isFocusedRocket(decoded.rocket_id))
                {
                    rx_cadence.onPacket(last_packet_ms);
                }

                // Route to per-rocket tracker
                int slot = findOrAllocRocket(decoded.rocket_id);

                // #850: FORWARD-FILL. Each frame carries only its own subset,
                // so seed from this rocket's last known state and let the frame
                // overwrite what it actually contains. `decoded` is therefore
                // always the complete picture — which is what lets every
                // consumer below (CSV row, BLE telemetry, logging policy) stay
                // written against a whole telemetry record, and what makes the
                // CSV a rectangular table with a row per received packet rather
                // than alternating half-empty rows.
                //
                // The unpackers deliberately do not clear the fields they do
                // not carry; see the note in TR_Sensor_Data_Converter.cpp.
                if (slot >= 0)
                {
                    decoded = tracked_rockets[slot].last_data;
                }
                if (frame_type == LORA_FRAME_SLOW)
                {
                    sensor_converter.unpackLoRaSlowBytes(rx_buf, decoded);
                }
                else
                {
                    sensor_converter.unpackLoRaFastBytes(rx_buf, decoded);
                }

                TR_LoRa_Comms::Stats ls = {};
                lora_comms.getStats(ls);

                // Observed-loss attribution (#105).  Use the rocket's free-
                // running seq to compute how many telemetry packets we missed
                // since the last RX from this rocket.  loraComputeObservedLoss
                // returns -1 on first contact, duplicates, or implausible
                // forward deltas (most likely a rocket reboot — lora_tx_seq
                // resets to 0).  We update last_seq unconditionally so the
                // next RX gets a clean baseline either way.
                int32_t observed_gap = -1;
                if (slot >= 0) {
                    observed_gap = loraComputeObservedLoss(
                        tracked_rockets[slot].last_seq, decoded.seq);
                    tracked_rockets[slot].last_seq = (int32_t)decoded.seq;
                    if (observed_gap > 0) {
                        lora_total_observed_loss += (uint32_t)observed_gap;
                        if (hop_active_) {
                            hop_session_observed_loss += (uint32_t)observed_gap;
                        }
                    }
                    if (hop_active_) hop_session_total_pkts++;
                }

                // Convert ECEF to lat/lon — but only when the rocket reports
                // an actual GPS fix.  The rocket sometimes packs nonzero ECEF
                // even with num_sats == 0 (stale GNSS register read), which
                // would otherwise yield a "valid-looking" lat/lon while the
                // fix is invalid (#95).
                double lat_deg = NAN, lon_deg = NAN, alt_m = NAN;
                if (decoded.num_sats > 0 &&
                    (decoded.ecef_x != 0.0 || decoded.ecef_y != 0.0 || decoded.ecef_z != 0.0))
                {
                    coord.ecefToGeodetic(decoded.ecef_x, decoded.ecef_y, decoded.ecef_z,
                                         lat_deg, lon_deg, alt_m);
                }

                printTelemetry(decoded, ls.last_rssi, ls.last_snr, lat_deg, lon_deg, alt_m);

                // Base station CSV logging policy (#107, refined in #168,
                // multi-rocket in #381):
                //   • Start a log whenever a non-LANDED packet arrives and
                //     we aren't already logging.  Every flight state counts
                //     — even READY pre-flight setup is worth keeping.
                //   • Refuse to auto-open while the rocket reports LANDED
                //     (#168).  Without this gate, the close-on-LANDED-
                //     transition (below) would be immediately undone by
                //     the next LANDED packet, generating a remnant file
                //     that only contains post-landing telemetry — that's
                //     the LANDED-only files in the 5/17/26 flight folders.
                //     Operators can still capture LANDED telemetry via the
                //     manual BLE cmd 23 start, which bypasses this gate.
                //   • Close on a rocket's LANDED transition ONLY when it is
                //     the last fresh rocket flying (#381): state edges are
                //     tracked per tracked_rockets slot, so a landed rocket's
                //     repeat LANDED packets interleaved with another rocket's
                //     READY no longer read as close/reopen transitions once
                //     per packet pair (the one-file-per-second shred).
                //   • Throttled by LOG_OPEN_RETRY_MS so a persistent
                //     fopen() failure (wedged SD) doesn't log-spam.
                if (!logging_active && !log_manual_inhibit &&
                    decoded.rocket_state != LANDED &&
                    (log_last_open_attempt_ms == 0 ||
                     (millis() - log_last_open_attempt_ms) >= config::LOG_OPEN_RETRY_MS))
                {
                    log_last_open_attempt_ms = millis();
                    startLogging();
                }

                last_known_rocket_logging = decoded.logging_active;

                if (logging_active)
                {
                    // The RAW frame, not `decoded`: with FAST and SLOW
                    // interleaved, `decoded` is the forward-filled accumulator
                    // and logging it would repeat the same values and lose
                    // which fields actually arrived in this packet. lat/lon,
                    // Euler and the seq gap are all recomputed from these bytes
                    // by whatever renders the log.
                    logLoRaPacket(rx_buf, rx_len, ls.last_rssi, ls.last_snr,
                                  currentRxFreqMHz());
                }

                if (slot >= 0)
                {
                    const uint32_t now_ms = millis();
                    auto& rls = tracked_rockets[slot].log_state;
                    const bool was_inflight_armed = (rls.inflight_entry_ms != 0);

                    // Fold this packet into the rocket's own transition state:
                    // detects edges, arms/disarms its INFLIGHT safety timer,
                    // latches its freq lock (bs_log_policy #381).
                    const auto edges =
                        bs_log_policy::updateRocketLogState(rls,
                                                            decoded.rocket_state,
                                                            now_ms);
                    if (!was_inflight_armed && rls.inflight_entry_ms != 0)
                    {
                        ESP_LOGI(TAG, "[LOG] Rocket %u INFLIGHT entry — safety timer armed (%u min)",
                                 (unsigned)decoded.rocket_id,
                                 (unsigned)(config::LOG_INFLIGHT_SAFETY_MS / 60000));
                    }

                    // Freshness for the aggregate decisions below uses
                    // last_seen_ms; stamp it now (the tracker mirror further
                    // down stamps it again — idempotent).
                    tracked_rockets[slot].last_seen_ms  = now_ms;

                    // #835 item 6 residual: telemetry-only clock for the freq lock.

                    tracked_rockets[slot].last_telem_ms = now_ms;

                    bs_log_policy::RocketView views[MAX_TRACKED_ROCKETS];
                    buildRocketViews(views);   // KEEP: the landed_edge close below uses it
                    updateFreqLock(views, now_ms, "rx");

                    // Close on LANDED transition — but only when no other
                    // fresh rocket is still flying (#381). Boot edge is
                    // handled inside updateRocketLogState (no edge on a
                    // rocket's first packet), so a post-flight BS reboot
                    // seeing LANDED first doesn't produce a zero-byte file.
                    if (edges.landed_edge && logging_active)
                    {
                        if (bs_log_policy::noFreshRocketFlying(
                                views, MAX_TRACKED_ROCKETS, now_ms,
                                config::LOG_SILENCE_TIMEOUT_MS,
                                config::LOG_INFLIGHT_SAFETY_MS))
                        {
                            ESP_LOGI(TAG, "[LOG] Rocket %u LANDED (last fresh rocket) — closing flight log",
                                     (unsigned)decoded.rocket_id);
                            stopLogging();
                        }
                        else
                        {
                            ESP_LOGI(TAG, "[LOG] Rocket %u LANDED — log stays open (another rocket still active)",
                                     (unsigned)decoded.rocket_id);
                        }
                    }

                    // This rocket changing state clears the manual stop
                    // inhibit so the next flight gets logged automatically
                    // (#107). Keyed per rocket: a second rocket's steady
                    // stream no longer fabricates state changes.
                    if (edges.state_changed && log_manual_inhibit)
                    {
                        log_manual_inhibit = false;
                        ESP_LOGI(TAG, "[LOG] State change cleared manual stop inhibit");
                    }
                }

                // Hop state follow (#40 / #41 phase 2a, seq-anchored in
                // #105 follow-up).  The rocket and the BS both compute
                // the channel for any seq from loraHopChannelForSeq() —
                // identical formula, no chained handoff.  This means a
                // single missed packet within a dwell window doesn't
                // desync us: the next received packet's seq tells us
                // exactly where the rocket is.  next_channel_idx in the
                // packet header is now a sanity hint; we still honour
                // the NO_HOP sentinel as the "not currently hopping"
                // signal, but for the actual channel we consult seq.
                //
                // #150 direction B evidence: rocket announces a live hop
                // channel — or the off-schedule marker 0xFE — while our
                // link mode is fixed.  Never legitimate (it rebooted with
                // a stale hopdis=0 or missed our disable).  Queue a
                // cmd-17 re-push immediately; the service applies the
                // cooldown.  The 0xFE case is the valuable one: the
                // rocket is parked on the shared rendezvous LISTENING for
                // the full visit window, so the push queued off this very
                // frame lands while it can still hear us.
                if (lora_hop_disabled &&
                    decoded.next_channel_idx != LORA_NEXT_CH_NO_HOP)
                {
                    hop_mode_resync_pending = true;
                }

                // #106/#150: when the link mode is fixed, ignore the
                // rocket's hop entirely and stay on lora_freq_mhz.  Also
                // stay put when the current modulation can't legally hop
                // (currentHopDwell() == 0) — the cmd-17 refusal should
                // make that unreachable, but the gate keeps a bad state
                // from chasing a non-compliant schedule.
                // #390: ONE radio can follow ONE schedule — only the
                // focused rocket drives hop state.  A second hopping
                // rocket's packets (heard when channels coincide) must not
                // retune us mid-dwell; before this gate, two hopping
                // rockets made the BS chase whichever spoke last.
                if (!lora_hop_disabled && currentHopDwell() > 0 &&
                    isFocusedRocket(decoded.rocket_id) &&
                    shouldHopInState(decoded.rocket_state))
                {
                    if (decoded.next_channel_idx == LORA_NEXT_CH_HOP_OFFSCHEDULE)
                    {
                        // #150 (review): rocket is hopping but momentarily
                        // off-schedule (rendezvous visit / scan pause) —
                        // it is transmitting on the channel we just heard
                        // it on and returns to the schedule on its own.
                        // Hold position; 0xFE is a marker, not a channel
                        // index.  A rebooted BS parked on the rendezvous
                        // sees these frames and simply waits for the
                        // rocket's post-visit bootstrap on lora_freq_mhz.
                        hop_mode_mismatch_streak_ = 0;
                    }
                    else if (decoded.next_channel_idx != LORA_NEXT_CH_NO_HOP)
                    {
                        hop_mode_mismatch_streak_ = 0;
                        const uint8_t n = loraChannelCount(lora_bw_khz);
                        if (n > 0)
                        {
                            uint8_t mask_buf[LORA_SKIP_MASK_MAX_BYTES];
                            const uint8_t* mask = effectiveHopMask(mask_buf, n);
                            // Compute next channel from seq+1 — what the
                            // rocket will be on for the *next* packet.
                            // Trust the rocket's announced next_channel_idx
                            // as the retune target — that's where the rocket
                            // will actually transmit, regardless of whether
                            // our local skip-mask agrees with theirs.  The
                            // seq-derived computation below is a sanity
                            // CHECK, not the source of truth: if it disagrees
                            // with what the rocket announced, the masks have
                            // drifted (cmd-15 push didn't reach this side, or
                            // the BS just ran a fresh scan and the rocket
                            // hasn't received the new mask yet).  In that
                            // case the mask-warning fires but we still follow
                            // the rocket so the link stays up.
                            const uint8_t expected_next = loraHopChannelForSeq(
                                (uint16_t)(decoded.seq + 1),
                                currentHopDwell(), mask, n);
                            // #150: bootstrap frames announce the schedule
                            // ENTRY channel (seq + remaining bootstraps),
                            // deliberately != f(seq+1) — so skip the drift
                            // sanity check on the frame that STARTS a
                            // follow session (it's a bootstrap whenever the
                            // session is new) or a false "mask drift" warn
                            // fires and queues a pointless cmd-15 re-push.
                            if (hop_active_ &&
                                decoded.next_channel_idx != expected_next) {
                                ESP_LOGW(TAG, "[HOP] seq=%u: rocket says next=%u, "
                                              "we'd compute next=%u (mask drift "
                                              "— following rocket)",
                                         (unsigned)decoded.seq,
                                         (unsigned)decoded.next_channel_idx,
                                         (unsigned)expected_next);
                                // Schedule auto re-push of the mask via
                                // cmd 15 — the rocket's mask is out of
                                // sync with ours, fixed by re-sending
                                // ours.  Cooldown enforced in
                                // serviceMaskDriftRepush.
                                chset_drift_repush_pending = true;
                            }
                            const bool was_active = hop_active_;
                            hop_idx_           = decoded.next_channel_idx;
                            hop_active_        = true;
                            hop_needs_retune_  = true;
                            hop_last_rx_ms_    = millis();
                            if (!was_active)
                            {
                                // Start a fresh hop-session counter window
                                // so the eventual hop_inactive / hop_silence
                                // event reports stats for *this* session
                                // (#105 diagnostics).
                                hop_session_started_ms    = millis();
                                hop_session_total_pkts    = 1;  // this packet counts
                                hop_session_observed_loss = 0;
                                ESP_LOGI(TAG, "[HOP] Active: %u channels at BW=%.0f kHz "
                                              "dwell=%u, first hop -> idx=%u (%.3f MHz)",
                                         (unsigned)n, (double)lora_bw_khz,
                                         (unsigned)currentHopDwell(),
                                         (unsigned)hop_idx_,
                                         (double)loraChannelMHz(lora_bw_khz, hop_idx_));
                                char ev[96];
                                snprintf(ev, sizeof(ev),
                                         "hop_active idx=%u nch=%u dwell=%u",
                                         (unsigned)hop_idx_, (unsigned)n,
                                         (unsigned)currentHopDwell());
                                logHopEvent(ev,
                                            loraChannelMHz(lora_bw_khz, hop_idx_));
                            }
                        }
                    }
                    else
                    {
                        // #150 direction A evidence: rocket is in a hop
                        // state but announces NO_HOP.  Legitimate during
                        // rendezvous visits and coordinated-scan pauses,
                        // so require a consecutive run before treating it
                        // as a mode mismatch (e.g. rocket rebooted before
                        // our enable was persisted).
                        if (hop_mode_mismatch_streak_ < 255) hop_mode_mismatch_streak_++;
                        if (hop_mode_mismatch_streak_ >= HOP_MODE_RESYNC_STREAK)
                        {
                            hop_mode_resync_pending = true;
                        }
                    }
                }
                else if (hop_active_ && isFocusedRocket(decoded.rocket_id))
                {
                    // The FOCUSED rocket is no longer in a hop state —
                    // return to the static configured channel so recovery /
                    // ground comms resume on a known frequency.  Gated on
                    // focus (#390): a background rocket's LANDED packet must
                    // not tear down the followed rocket's hop session.
                    hop_active_       = false;
                    hop_needs_retune_ = true;
                    const uint32_t dur_ms = (hop_session_started_ms != 0)
                        ? (millis() - hop_session_started_ms) : 0;
                    ESP_LOGI(TAG, "[HOP] Inactive (rocket state changed): "
                                  "returning to %.2f MHz", (double)lora_freq_mhz);
                    char ev[96];
                    snprintf(ev, sizeof(ev),
                             "hop_inactive duration_ms=%u pkts=%u loss=%u "
                             "reason=state",
                             (unsigned)dur_ms,
                             (unsigned)hop_session_total_pkts,
                             (unsigned)hop_session_observed_loss);
                    logHopEvent(ev, lora_freq_mhz);
                    hop_session_started_ms    = 0;
                    hop_session_total_pkts    = 0;
                    hop_session_observed_loss = 0;
                }

                last_known_camera_recording = decoded.camera_recording;

                // Update per-rocket tracker
                if (slot >= 0)
                {
                    tracked_rockets[slot].last_data = decoded;
                    tracked_rockets[slot].last_rssi = ls.last_rssi;
                    tracked_rockets[slot].last_snr  = ls.last_snr;
                    tracked_rockets[slot].last_lat_deg = lat_deg;
                    tracked_rockets[slot].last_lon_deg = lon_deg;
                    tracked_rockets[slot].last_alt_m   = alt_m;
                    tracked_rockets[slot].last_seen_ms = millis();
                    // #390: the stale re-push subject follows the FOCUSED
                    // rocket, not whichever packet arrived last — two live
                    // rockets used to flip active_rocket_idx per packet.
                    updateFocusOnPacket(decoded.rocket_id, millis());
                    if (decoded.rocket_id == effectiveFocusRid())
                        active_rocket_idx = (uint8_t)slot;
                }

                // Forward telemetry to BLE app (with rocket_id for app-side demux)
                TR_BLE_To_APP::TelemetryData ble_telem = {};
                buildBLETelemetry(decoded, ls.last_rssi, ls.last_snr,
                                  lat_deg, lon_deg, alt_m, ble_telem);
                if (slot >= 0 && tracked_rockets[slot].unit_name[0]) {
                    ble_telem.source_unit_name = tracked_rockets[slot].unit_name;
                }
                ble_app.sendTelemetry(ble_telem);
                rx_sent_telem_this_iter = true;   // #289
                maybeMarkOtaValid();
            }
        }
        else
        {
            // #570: count + surface (was a bare WARN — a mixed-flash size
            // mismatch dropped 100% of telemetry with no counter and nothing
            // over BLE; the netid path got this treatment in #329, this
            // didn't).
            lora_size_mismatch_drops++;
            lora_size_last_drop_ms = millis();
            ESP_LOGW(TAG, "[RX] Unexpected packet size: %u (expected %u fast, %u slow, "
                          "or a beacon) — %lu dropped; mixed-firmware flash? A 65 B "
                          "packet here is a pre-#850 rocket that still speaks the "
                          "single-frame protocol",
                     (unsigned)rx_len, (unsigned)SIZE_OF_LORA_FAST,
                     (unsigned)SIZE_OF_LORA_SLOW,
                     (unsigned long)lora_size_mismatch_drops);
        }
    }

    // ==========================================================================
    // SECTION: Log lifecycle timeouts and flush
    // ==========================================================================
    // INFLIGHT safety timeout (#107, per-rocket in #381): close the log if a
    // rocket has been in INFLIGHT too long without a LANDED — lost-LoRa-
    // during-descent / stuck-rocket-state-machine — UNLESS another fresh
    // rocket is still flying (its own flight keeps the file open; the expired
    // rocket already counts as "presumed down" in that aggregate, so the log
    // closes when the last fresh rocket lands). Timers disarm in
    // stopLogging(). Does NOT inhibit the auto-restart on the next packet —
    // a rocket genuinely stuck INFLIGHT past the cap starts a fresh file and
    // the operator sees two files instead of one.
    // One view snapshot serves BOTH the freq-lock freshness sweep and the
    // INFLIGHT-safety close.
    {
        const uint32_t now_ms = millis();
        bs_log_policy::RocketView views[MAX_TRACKED_ROCKETS];
        buildRocketViews(views);

        // #835 item 6: re-evaluate the aggregate freq lock EVERY pass, not
        // only when a packet arrives.  Deliberately OUTSIDE the
        // logging_active gate below — the lock outlives the log file, and a
        // lock that only decayed while a file happened to be open would be
        // the same bug in a smaller box.  Cheap enough to run unthrottled:
        // buildRocketViews is 4 x 5 scalar copies (MAX_TRACKED_ROCKETS = 4)
        // and each aggregate is at most 4 compares, against a loop_bs that
        // vTaskDelay(1)s at 1 kHz.
        // #835 item 6 residual: also CLEAR the underlying per-rocket latch
        // once its telemetry has gone stale, not just the aggregate.
        // computeFreqLockForFlight() leaves log_state.freq_lock UNCHANGED
        // through INITIALIZATION and PRELAUNCH, so a recovered, rebooted
        // rocket's first packet would otherwise re-latch the aggregate from
        // the stale bit and re-lock the base station until READY arrived —
        // on exactly the path where the operator is trying to get the radio
        // back. Uses the RETUNE window (the longer of the two): a latch is
        // only truly dead once even the radio-moving consumers would release.
        for (int i = 0; i < MAX_TRACKED_ROCKETS; ++i)
        {
            if (bs_log_policy::freqLockExpired(views[i], now_ms,
                                               config::LOG_INFLIGHT_SAFETY_MS))
            {
                tracked_rockets[i].log_state.freq_lock = false;
                views[i].freq_lock                     = false;
                ESP_LOGW(TAG, "[LOG] Per-rocket freq latch cleared: rid=%u "
                              "no telemetry for %lu s",
                         (unsigned)tracked_rockets[i].rocket_id,
                         (unsigned long)((now_ms -
                                          tracked_rockets[i].last_telem_ms) / 1000U));
            }
        }

        updateFreqLock(views, now_ms, "freshness sweep");

        if (logging_active &&
            bs_log_policy::anySafetyExpired(views, MAX_TRACKED_ROCKETS, now_ms,
                                            config::LOG_INFLIGHT_SAFETY_MS) &&
            bs_log_policy::noFreshRocketFlying(views, MAX_TRACKED_ROCKETS, now_ms,
                                               config::LOG_SILENCE_TIMEOUT_MS,
                                               config::LOG_INFLIGHT_SAFETY_MS))
        {
            ESP_LOGW(TAG, "[LOG] INFLIGHT safety timeout (%u min), closing log",
                     (unsigned)(config::LOG_INFLIGHT_SAFETY_MS / 60000));
            stopLogging();
        }
    }

    // Silence timeout: close log if no packets for LOG_SILENCE_TIMEOUT_MS
    // (5 min, #137).  Applies in every rocket state — pre-#137 this was 30 s
    // which closed mid-flight on altitude-driven RX gaps near apogee, but
    // bumping to 5 min handles even the worst-case descent silence on the
    // Estes-class flights we test with while still flushing the log a
    // reasonable interval after the rocket is powered off post-recovery.
    if (logging_active &&
        (millis() - log_last_write_ms) >= config::LOG_SILENCE_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "[LOG] Silence timeout (state=%s), closing log file",
                 rocketStateToString(lastSeenRocketState()));
        stopLogging();
    }

    // Periodic flush to flash.  fflush() only pushes stdio buffers down to
    // the OS — fsync() forces FATFS to commit dirty sectors to the SD card,
    // so a power loss within the flush window doesn't lose buffered
    // telemetry.  Skip fsync on SPIFFS fallback: SPIFFS persists writes
    // synchronously and fsync is a no-op there.  (#107)
    if (logging_active && (millis() - log_last_flush_ms) >= config::LOG_FLUSH_INTERVAL_MS)
    {
        if (log_file)
        {
            // #329: check the return values.  A full card typically doesn't
            // fail at fprintf() (it buffers) — it fails HERE, when fflush/fsync
            // push the buffered rows to FATFS (ENOSPC).  Ignoring these returns
            // is exactly how a flight's telemetry can vanish with no warning.
            bool ok = (fflush(log_file) == 0);
            if (ok && !using_internal_flash && fsync(fileno(log_file)) != 0) ok = false;
            if (!ok)
            {
                log_write_fail_count++;
                ESP_LOGE(TAG, "[LOG] flush/fsync failed (errno=%d %s) — buffered "
                              "rows may be lost (card full?)", errno, strerror(errno));
            }
        }
        log_last_flush_ms = millis();
    }

    // ==========================================================================
    // SECTION: Battery read and standalone BLE update
    // ==========================================================================
    // Periodic battery read + standalone BLE update
    if (millis() - last_battery_ms >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_battery_ms = millis();
        // Skip the I2C gauge poll while an OTA is in flight (#17): the
        // esp_ota_begin() partition erase blocks the SPI flash for ~1-2 s
        // and the gauge transaction collides with it, logging spurious
        // i2c_master_transmit_receive failures. The telemetry push below
        // still runs so the app sees liveness + OTA status during the flash.
        if (!ble_app.isOtaActive())
        {
            updateBattery();
            // V3: reconcile the flight-pack charger (presence detect, config
            // re-apply after plug-in / watchdog, 40 s WD kick, status/fault
            // transition logs). Cheap no-op while no charge input is present.
            if (config::HAS_PACK_CHARGER)
            {
                pack_charger.service();
            }
        }

        // Always push base station stats to BLE, even without LoRa packets,
        // so BS battery / logging state / RSSI stay live.  Tag each push
        // with a freshness status (#95) so the iOS app can distinguish:
        //   • LIVE    — re-pushing telemetry that's still recent
        //   • STALE   — re-pushing cached telemetry older than the threshold
        //               (iOS dims + shows "stale (Ns ago)")
        //   • SYNCING — no rocket has ever been caught
        //               (iOS hides rocket fields, shows "Searching for rocket…")
        // The RX path always sends LIVE because it fires on a fresh decode.
        if (ble_app.isConnected())
        {
            TR_BLE_To_APP::TelemetryData ble_telem = {};
            auto& tr = tracked_rockets[active_rocket_idx];
            if (tr.active)
            {
                buildBLETelemetry(tr.last_data, tr.last_rssi, tr.last_snr,
                                  tr.last_lat_deg, tr.last_lon_deg, tr.last_alt_m, ble_telem);
                if (tr.unit_name[0]) {
                    ble_telem.source_unit_name = tr.unit_name;
                }
                const uint32_t age_ms = millis() - tr.last_seen_ms;
                if (age_ms > config::BLE_TELEMETRY_STALE_MS)
                {
                    ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::STALE;
                    ble_telem.data_age_ms = age_ms;
                }
            }
            else
            {
                // No rocket tracked — publish base-station-only fields with
                // a SYNCING tag so the iOS app shows "Searching for rocket…"
                // rather than rendering the empty zero-init values as if
                // they were a rocket sitting in INIT.
                LoRaDataSI empty = {};
                buildBLETelemetry(empty, NAN, NAN, NAN, NAN, NAN, ble_telem);
                ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::SYNCING;
            }
            // #289: skip this push if the RX path already sent a fresher frame
            // this iteration — the periodic push exists to keep BS stats/liveness
            // current when no packet arrived, so it's redundant right after a
            // fresh decode.  The battery read above still runs every period.
            if (!rx_sent_telem_this_iter)
            {
                ble_app.sendTelemetry(ble_telem);
                maybeMarkOtaValid();
            }

            // Flash-space stats for the app's storage bar (every ~5 s).
            static uint32_t last_bs_storage_ms = 0;
            const uint32_t now_bs = millis();
            if (now_bs - last_bs_storage_ms >= 5000U)
            {
                last_bs_storage_ms = now_bs;
                BaseStationStorageStatsData bss = {};
                uint64_t total = 0, used = 0;
                const bool mounted = bsQueryStorage(total, used);
                bss.total_bytes = total;
                bss.used_bytes  = used;
                bss.free_bytes  = (total > used) ? (total - used) : 0;
                bss.backend = using_internal_flash ? 0 : (using_external_flash ? 2 : 1);
                bss.flags   = (mounted         ? BSS_FLAG_MOUNTED  : 0)
                            | (storage_demoted ? BSS_FLAG_FALLBACK : 0)
                            | (bs_storage_policy::recovered(storage_demoted, nand_attempts)
                                               ? BSS_FLAG_RETRIED  : 0);
                ble_app.sendStorageStats(0xCD, reinterpret_cast<const uint8_t*>(&bss), sizeof(bss));
            }
        }
    }

    // ==========================================================================
    // SECTION: BLE command dispatch
    // ==========================================================================
    // Handle BLE commands (file list, delete, download)
    ble_app.loop();

    // Detect BLE connect (rising edge) — no auto-config send since
    // the base station's cached config may not match the rocket's actual values
    {
        static bool ble_was_connected = false;
        bool ble_now = ble_app.isConnected();
        ble_was_connected = ble_now;
    }

    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd == BLE_BS_CMD_FILE_LIST)
    {
        handleFileListCommand();
    }
    else if (ble_cmd == BLE_BS_CMD_FILE_DELETE)
    {
        handleDeleteCommand();
    }
    else if (ble_cmd == BLE_BS_CMD_FILE_DOWNLOAD)
    {
        // #380: only STARTS the transfer (opens the file, captures state).
        // Chunks go out one per loop pass via serviceDownload() below, so the
        // RX/uplink/heartbeat/flush services keep running during the download.
        startDownload();
    }
    else if (ble_cmd == BLE_BS_CMD_CAMERA_TOGGLE)
    {
        // Camera toggle: send desired state (inverse of last known) so LoRa
        // retries are idempotent — won't toggle back and forth on the rocket.
        // #390: keyed to the FOCUSED rocket (state + target) — the old
        // globals read whichever rocket spoke last and broadcast to all.
        const uint8_t frid  = effectiveFocusRid();
        const int     fslot = frid ? slotOfRid(frid) : -1;
        const bool recording = (fslot >= 0)
            ? tracked_rockets[fslot].last_data.camera_recording
            : last_known_camera_recording;
        uint8_t desired = recording ? 0 : 1;
        buildUplinkPacket(1, &desired, 1, frid ? frid : 0xFF);
        ESP_LOGI(TAG, "[BLE->UPLINK] Camera %s -> rid=%u",
                 desired ? "START" : "STOP", (unsigned)(frid ? frid : 0xFF));
    }
    else if (ble_cmd == BLE_BS_CMD_LOGGING_TOGGLE)
    {
        // Logging toggle: starts/stops BOTH rocket flash recording (via LoRa
        // uplink) and base station SD card logging simultaneously.
        // Base station logging state is the toggle authority — rocket follows.
        // #390: the rocket half keys state + target off the FOCUSED rocket
        // (globals tracked whichever rocket spoke last, and the uplink hit
        // every rocket in range).
        const uint8_t frid  = effectiveFocusRid();
        const int     fslot = frid ? slotOfRid(frid) : -1;
        const bool rocket_logging = (fslot >= 0)
            ? tracked_rockets[fslot].last_data.logging_active
            : last_known_rocket_logging;
        if (!logging_active)
        {
            log_manual_inhibit = false;  // explicit start clears any prior inhibit (#107)
            startLogging();
            ESP_LOGI(TAG, "[LOG] Base station logging started (manual)");

            if (!rocket_logging)
            {
                uint8_t desired = 1;
                buildUplinkPacket(23, &desired, 1, frid ? frid : 0xFF);
                ESP_LOGI(TAG, "[BLE->UPLINK] Rocket logging START -> rid=%u",
                         (unsigned)(frid ? frid : 0xFF));
            }
        }
        else
        {
            stopLogging();
            // Sticky inhibit: don't auto-restart on the next packet.  Cleared
            // on the next rocket state change so a real next flight is still
            // captured automatically.  (#107)
            log_manual_inhibit = true;
            ESP_LOGI(TAG, "[LOG] Base station logging stopped (manual; auto-restart inhibited until state change)");

            if (rocket_logging)
            {
                uint8_t desired = 0;
                buildUplinkPacket(23, &desired, 1, frid ? frid : 0xFF);
                ESP_LOGI(TAG, "[BLE->UPLINK] Rocket logging STOP -> rid=%u",
                         (unsigned)(frid ? frid : 0xFF));
            }
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SERVO_TEST_ANGLES)
    {
        // Servo test angles: relay 8-byte payload to OutComputer via LoRa uplink
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 8)
        {
            buildUplinkPacket(24, payload, 8, focusTargetRid());   // #390: focused rocket
            ESP_LOGI(TAG, "[BLE->UPLINK] Servo test angles -> rid=%u", (unsigned)focusTargetRid());
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SERVO_TEST_STOP)
    {
        // Servo test stop: relay to OutComputer via LoRa uplink
        buildUplinkPacket(25, nullptr, 0, focusTargetRid());   // #390: focused rocket
        ESP_LOGI(TAG, "[BLE->UPLINK] Servo test stop -> rid=%u", (unsigned)focusTargetRid());
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_CONFIG)
    {
        // Sim config: relay to OutComputer via LoRa uplink
        // Payload is 16 bytes: [mass_g:4][thrust_n:4][burn_s:4][descent_rate_mps:4]
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len > 16) payload_len = 16;
        buildUplinkPacket(5, payload, payload_len, focusTargetRid());   // #390: focused rocket

        if (payload_len >= 12)
        {
            float mass_g, thrust_n, burn_s;
            memcpy(&mass_g,   payload + 0, 4);
            memcpy(&thrust_n, payload + 4, 4);
            memcpy(&burn_s,   payload + 8, 4);
            float descent = 0.0f;
            if (payload_len >= 16)
            {
                memcpy(&descent, payload + 12, 4);
            }
            ESP_LOGI(TAG, "[BLE->UPLINK] Sim config: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                     (double)mass_g, (double)thrust_n, (double)burn_s, (double)descent);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_START)
    {
        buildUplinkPacket(6, nullptr, 0, focusTargetRid());   // #390: focused rocket
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim start -> rid=%u", (unsigned)focusTargetRid());
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_STOP)
    {
        buildUplinkPacket(7, nullptr, 0, focusTargetRid());   // #390: focused rocket
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim stop -> rid=%u", (unsigned)focusTargetRid());
    }
    else if (ble_cmd == BLE_BS_CMD_TIME_SYNC)
    {
        // Time sync from phone: [year_lo][year_hi][month][day][hour][minute][second]
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 7)
        {
            sync_year   = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            sync_month  = payload[2];
            sync_day    = payload[3];
            sync_hour   = payload[4];
            sync_minute = payload[5];
            sync_second = payload[6];
            time_sync_millis = millis();
            time_synced = true;

            ESP_LOGI(TAG, "[BLE] Time synced: %04u-%02u-%02u %02u:%02u:%02u UTC",
                     sync_year, sync_month, sync_day,
                     sync_hour, sync_minute, sync_second);

            // If a sequential-named log was opened before sync arrived,
            // promote it to its proper timestamped name now (#168).
            renameOpenLogIfSequential();
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SET_PHONE_FIX)
    {
        // Where the base station actually is.  Logged, never acted on --
        // nothing in the BS behaviour depends on this, it exists purely so
        // the CSV records the range.
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 11)
        {
            int32_t lat_e7, lon_e7;
            int16_t alt_m;
            memcpy(&lat_e7, payload + 0, 4);
            memcpy(&lon_e7, payload + 4, 4);
            memcpy(&alt_m,  payload + 8, 2);
            const uint8_t h_acc_m = payload[10];
            logPhoneFixEvent(lat_e7, lon_e7, alt_m, h_acc_m);
            ESP_LOGI(TAG, "[LOG] phone fix %.7f, %.7f alt=%d hacc=%u%s",
                     (double)lat_e7 * 1e-7, (double)lon_e7 * 1e-7,
                     (int)alt_m, (unsigned)h_acc_m,
                     logging_active ? "" : " (no session open -- not logged)");
        }
    }
    else if (ble_cmd == BLE_BS_CMD_LORA_RECONFIG)
    {
        // LoRa reconfiguration — transactional (issue #71).  The base
        // station relays the new config to every rocket on the OLD channel,
        // then switches to NEW and verifies the rocket joined before
        // committing to NVS.  On timeout, both sides stay on OLD and the
        // silence-recovery layer heals any residual divergence.
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 11)
        {
            float new_freq, new_bw;
            memcpy(&new_freq, payload + 0, 4);
            memcpy(&new_bw,   payload + 4, 4);
            uint8_t new_sf   = payload[8];
            uint8_t new_cr   = payload[9];
            int8_t  new_pwr  = (int8_t)payload[10];
            startLoRaTransaction(new_freq, new_bw, new_sf, new_cr, new_pwr);
        }
    }
    // #285: ble_cmd 12/13/14/65/66 (servo / PID / servo-enable / guidance /
    // fin-layout config relay to the rocket) were removed.  The base station
    // is a read-only display of the active rocket and never configures it: the
    // iOS app gates all config editing to a direct rocket connection
    // (SettingsView renders base stations read-only; ActiveRocketSyncer never
    // pushes a profile to a BS).  These relays were therefore unreachable dead
    // paths — and a foot-gun if a future app build ever sent config to a BS —
    // so they are intentionally absent.  Rocket config flows app->rocket over
    // a direct BLE link only.
    else if (ble_cmd == LORA_CMD_SET_HOP_DISABLED)
    {
        // BS-controlled hop enable/disable (#106).  Persist locally,
        // re-tune to lora_freq_mhz immediately if we were tracking a hop,
        // and uplink the same byte to the rocket so both sides agree.
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 1)
        {
            const bool new_disabled = (payload[0] != 0);
            if (!new_disabled && currentHopDwell() == 0)
            {
                // #150: the current modulation can't fit one packet inside
                // the FCC dwell budget — refuse the enable and re-send
                // config so the app's picker snaps back.  The app also
                // greys the option via "lhdw" == 0; this is the belt to
                // that brace.
                ESP_LOGW(TAG, "[BLE] Hop enable REFUSED: dwell=0 at SF%u/BW%.0f",
                         (unsigned)lora_sf, (double)lora_bw_khz);
                sendCurrentConfig();
            }
            else if (new_disabled && !lora_hop_disabled && hop_active_)
            {
                // #150 (review): we are FOLLOWING a hop — drain the
                // disable retries on the channel the rocket is listening
                // on before we stop following and retune away.  Keep
                // tracking the hop while the retries run; the flag flip,
                // NVS persist, retune, and config readback happen in
                // serviceHopDisableDrain() once the queue empties (or the
                // deadline passes).  No sendCurrentConfig() here: the app
                // keeps its optimistic picker state and gets one clean
                // readback when the mode actually commits (~1 s).
                hop_disable_drain_deadline_ms = millis() + HOP_DISABLE_DRAIN_MAX_MS;
                buildUplinkPacket(LORA_CMD_SET_HOP_DISABLED, payload, 1);
                ESP_LOGI(TAG, "[BLE->UPLINK] Hop disable: draining retries on the hop channel first");
            }
            else
            {
                hop_disable_drain_deadline_ms = 0;  // an enable cancels a pending drain
                if (new_disabled != lora_hop_disabled)
                {
                    lora_hop_disabled = new_disabled;
                    prefs.begin("lora", false);
                    prefs.putUChar("hopdis", lora_hop_disabled ? 1 : 0);
                    prefs.end();
                    if (lora_hop_disabled && hop_active_)
                    {
                        // Drop hop tracking so the next radio service tick
                        // returns us to lora_freq_mhz.
                        hop_active_       = false;
                        hop_needs_retune_ = true;
                    }
                    hop_mode_mismatch_streak_ = 0;  // fresh mode, fresh evidence
                }
                if (!new_disabled)
                {
                    // #150: hold heartbeats through the rocket's deferred
                    // activation + bootstrap so we're listening when the
                    // handoff packets arrive.
                    heartbeat_hold_until_ms = millis() + HEARTBEAT_HOLD_AFTER_ENABLE_MS;
                }
                buildUplinkPacket(LORA_CMD_SET_HOP_DISABLED, payload, 1);
                ESP_LOGI(TAG, "[BLE->UPLINK] Hop disable: %s",
                         lora_hop_disabled ? "DISABLED (fixed freq)" : "ENABLED");
                sendCurrentConfig();
            }
        }
    }
    else if (ble_cmd == BLE_BS_CMD_CONFIG_READBACK)
    {
        // Config readback request
        sendCurrentConfig();
    }
    // ---- Device Identity Commands ----
    else if (ble_cmd == BLE_BS_CMD_SET_UNIT_NAME)
    {
        // Set unit name — payload is UTF-8 string, max 20 bytes
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen > 0 && plen <= 20)
        {
            char new_name[24];
            memcpy(new_name, payload, plen);
            new_name[plen] = '\0';
            strncpy(unit_name, new_name, sizeof(unit_name) - 1);
            unit_name[sizeof(unit_name) - 1] = '\0';
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putBytes("un", new_name, strlen(new_name) + 1);
            id_prefs.end();
            ble_app.setName(unit_name);
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Unit name set: %s", unit_name);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SET_NETWORK_ID)
    {
        // Set network_id — payload: [nid:1]
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 1)
        {
            network_id = payload[0];
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putUChar("nid", network_id);
            id_prefs.end();
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Network ID set: %u", (unsigned)network_id);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SET_FOCUS_ROCKET)
    {
        // #390: pin the radio focus to one rocket. Payload: [rid], 0 = back
        // to auto (sticky first-heard). RAM-only on purpose — the app owns
        // the choice and re-sends it on every BLE connect, so a BS reboot
        // can't resurrect a stale pin.
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 1)
        {
            focus_rid_pinned = payload[0];
            const int fslot = focus_rid_pinned ? slotOfRid(focus_rid_pinned) : -1;
            if (fslot >= 0)
            {
                // Re-point the BLE re-push subject immediately so the app's
                // stale banner describes the newly focused rocket without
                // waiting for its next packet.
                active_rocket_idx = (uint8_t)fslot;
            }
            // Log the EFFECTIVE focus: on a pin-clear the subject falls back
            // to the auto rid, whose tracked-ness is what matters (checking
            // the cleared pin's slot printed "not heard yet" for a rocket
            // the BS was actively receiving).
            const uint8_t log_rid = focus_rid_pinned ? focus_rid_pinned : focus_rid_auto;
            const int log_slot = log_rid ? slotOfRid(log_rid) : -1;
            ESP_LOGI(TAG, "[FOCUS] %s rid=%u (%s)",
                     focus_rid_pinned ? "Pinned to" : "Auto (pin cleared),",
                     (unsigned)log_rid,
                     log_slot >= 0 ? "tracked" : "not heard yet");
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SET_BS_LOGGING)
    {
        // #390: BS-only CSV logging control for the app's base-station
        // screen. Payload: [on]. Unlike legacy cmd 23 this never uplinks a
        // rocket-logging command; same manual-inhibit semantics (#107).
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 1)
        {
            const bool want_on = (payload[0] != 0);
            if (want_on && !logging_active)
            {
                log_manual_inhibit = false;
                startLogging();
                ESP_LOGI(TAG, "[LOG] Base station logging started (manual, BS-only cmd)");
            }
            else if (!want_on && logging_active)
            {
                stopLogging();
                log_manual_inhibit = true;
                ESP_LOGI(TAG, "[LOG] Base station logging stopped (manual, BS-only cmd; auto-restart inhibited until state change)");
            }
        }
    }
    else if (ble_cmd == BLE_BS_CMD_RELAY_TO_ROCKET)
    {
        // Relay command to a specific rocket via LoRa uplink
        // Payload: [target_rid:1][inner_cmd:1][inner_payload:0..33 (bs_uplink_queue::kMaxPayload)]
        // (was documented as 0..18 — stale; the queue accepts 33, and #435's
        // cmd-28 guidance point needs a 20-byte inner payload.)
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 2)
        {
            uint8_t target_rid = payload[0];
            uint8_t inner_cmd  = payload[1];
            const uint8_t* inner_payload = (plen > 2) ? &payload[2] : nullptr;
            size_t inner_len = (plen > 2) ? (plen - 2) : 0;
            buildUplinkPacket(inner_cmd, inner_payload, inner_len, target_rid);
            ESP_LOGI(TAG, "[BLE->UPLINK] Relay cmd=%u -> rid=%u (%u bytes)",
                     inner_cmd, target_rid, (unsigned)inner_len);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_FREQ_SCAN)
    {
        // Frequency scan (base-station radio, pre-launch collision avoidance).
        // Payload: [start_mhz f32][stop_mhz f32][step_khz u16][dwell_ms u16]
        // All fields are little-endian.
        //
        // Two paths (#90):
        //   • Direct: rocket isn't presumed to be hopping (no recent RX, or
        //     last seen in READY/INIT/LANDED) — scan immediately.
        //   • Coordinated: rocket is presumed hopping → cmd 16 to park it
        //     on lora_freq_mhz, scan, push cmd 15, then re-bootstrap hop.
        //
        // The coordinated trigger uses rocketLikelyHopping() so it also
        // fires when the BS recently caught the rocket in a hop state but
        // the packet's next_channel_idx was NO_HOP (bootstrap, visiting
        // rendezvous, or paused).  In all those cases a direct scan would
        // still drop the link, even though hop_active_ is currently false.
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 12)
        {
            float start_mhz, stop_mhz;
            uint16_t step_khz, dwell_ms;
            memcpy(&start_mhz, payload + 0, 4);
            memcpy(&stop_mhz,  payload + 4, 4);
            memcpy(&step_khz,  payload + 8, 2);
            memcpy(&dwell_ms,  payload + 10, 2);

            // #150 (review): in fixed mode the rocket cannot legitimately
            // be hopping, and shouldHopInState(LANDED) is now true — a
            // recently-heard landed rocket would otherwise force the
            // coordinated-pause path (pointless cmd 16 + a RESUMING
            // stall) where pre-#150 the scan ran directly.
            const bool need_coord = !lora_hop_disabled && rocketLikelyHopping(
                hop_active_, last_packet_ms, millis(),
                focusedRocketState(), COORD_HOP_RECENT_MS);   // #390

            if (!need_coord)
            {
                if (startNoiseScan(start_mhz, stop_mhz, step_khz, dwell_ms))
                {
                    ESP_LOGI(TAG, "[BLE] Scan started: %.1f..%.1f MHz, %u kHz, %u ms (×%u passes)",
                             (double)start_mhz, (double)stop_mhz,
                             (unsigned)step_khz, (unsigned)dwell_ms,
                             (unsigned)LORA_NOISE_SCAN_PASSES);
                }
                else
                {
                    ESP_LOGW(TAG, "[BLE] Scan start rejected (busy or invalid range)");
                }
            }
            else if (coord_scan_state_ != CoordScanState::IDLE)
            {
                ESP_LOGW(TAG, "[BLE] Scan rejected: coordinated scan already in progress");
            }
            else if (uplinkBusy())
            {
                ESP_LOGW(TAG, "[BLE] Scan rejected: uplink busy — retry shortly");
            }
            else
            {
                startCoordinatedScan(start_mhz, stop_mhz, step_khz, dwell_ms);
            }
        }
    }

    // ==========================================================================
    // SECTION: Scan completion and periodic service chain
    // ==========================================================================
    // Service scan state machine (no-op when idle).  Must come before
    // serviceUplink so a TX retry doesn't fire while we're mid-scan —
    // the scan temporarily owns the radio's frequency.
    lora_comms.serviceScan();
    if (lora_comms.isScanDone())
    {
        if (scan_passes_remaining_ > 0)
        {
            // Multi-pass mode (#40 / #41 phase 3): merge this pass's
            // samples into the running max-RSSI accumulator, then
            // either kick off the next pass or finalize.
            const bool more = absorbScanPass();
            if (more)
            {
                (void)lora_comms.startScan(scan_param_start_mhz_,
                                           scan_param_stop_mhz_,
                                           scan_param_step_khz_,
                                           scan_param_dwell_ms_);
            }
            else
            {
                finalizeNoiseScan();
            }
        }
        else
        {
            // Defensive: a scan finished but we have no pass count
            // (shouldn't happen — startNoiseScan is the only path that
            // starts a scan).  Drain to keep the radio sane.
            (void)lora_comms.getScanSampleCount();
            lora_comms.consumeScanDone();
        }
    }

    // Service LoRa uplink retries (TX commands, then resume RX)
    serviceUplink();

    // Advance the transactional reconfigure state machine (issue #71).
    // Must run after serviceUplink so we observe the uplink queue draining
    // to empty in the same loop iteration it happens.
    serviceLoRaTransaction();

    // Silence recovery runs after the transaction service so a freshly
    // started transaction always wins over any pending recovery cycle.
    serviceRecovery();

    // Mask-drift auto-recovery (#105 follow-up).  Re-pushes cmd 15 if
    // the hop RX path detected a divergence between our skip-mask and
    // the rocket's.  Cheap when no drift is pending; only acts when
    // the radio is otherwise idle.
    serviceMaskDriftRepush();

    // Hop-mode resync (#150).  Re-pushes cmd 17 if frame evidence says
    // the rocket's link mode disagrees with ours.  Same idle-only /
    // cooldown discipline as the mask repush.
    serviceHopModeResync();

    // Graceful hop-disable commit (#150 review): flips to fixed mode once
    // the disable retries have drained on the hop channel.
    serviceHopDisableDrain();

    // Auto-acquire (#136): one-shot per power cycle.  Waits to hear the
    // rocket on rendezvous, runs a noise scan, picks the quietest
    // channel, and uses the existing cmd-10 transactional flow to move
    // both ends onto it.  Inert once DONE.
    serviceAutoAcquire();

    // Heartbeat — quietly tells the rocket "we're hearing you" so its
    // slow-rendezvous timer doesn't expire during normal idle operation.
    // Gates itself on recovery/transaction state internally.
    serviceHeartbeat();

    // BLE file download — at most one 170 B chunk per pass, paced to the BLE
    // notify drain interval. No-op unless a transfer is active (#380).
    serviceDownload();

    printStats();

    // Must use vTaskDelay(1) instead of yield() on ESP-IDF.
    // yield() only yields to equal-or-higher priority tasks, so IDLE
    // (priority 0) never runs and the task watchdog fires.
    vTaskDelay(1);
}

// ==========================================================================
// SECTION: FreeRTOS entry point
// ==========================================================================

extern "C" void app_main(void)
{
    setup_bs();
    xTaskCreatePinnedToCore([](void*) { while (true) { loop_bs(); } },
                            "bs_loop", 8 * 1024, NULL, 5, NULL, 1);
}
