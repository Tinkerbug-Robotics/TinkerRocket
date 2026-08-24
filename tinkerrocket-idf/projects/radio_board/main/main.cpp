// Radio daughterboard firmware (#409): a transparent UART<->LoRa modem.
//
// The host (OC on the rocket, BS on the ground — this firmware never knows
// which) sends fully-formed air frames over the UART link; we radiate them
// unmodified and tunnel received frames back with RSSI/SNR metadata. All
// protocol knowledge (LORA_PROTO_VERSION, network ids, hop schedules) stays
// on the hosts, so the on-air format is byte-identical to direct-SPI boards
// and deployed base stations interoperate unchanged.
//
// Protocol: components/TR_UART_Link/RadioModemProtocol.h (v1).
// Flow control: modem-side TX queue of TX_QUEUE_CAPACITY frames; every
// accepted TX_FRAME seq is eventually answered with a TX_RESULT (the credit
// return) — never silently dropped.

#include <cstring>

#include <driver/gpio.h>
#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <RadioModemProtocol.h>
#include <TR_LoRa_Comms.h>
#include <TR_UART_Link.h>

#include "config.h"

#ifdef TR_BENCH_SELFTEST
#include "bench_selftest.h"
#endif

// ---- Board constraints -------------------------------------------------------
// LoRa V3 hardware facts that a future sdkconfig edit could violate silently.
// The two that map to a settable Kconfig symbol fail the build rather than the
// bench; the third has no such symbol and is stated here so it is at least on
// the page next to the code it constrains.
//
// NO 2.4 GHz, EVER. The S3's LNA_IN (pin 1) is left floating on this board
// with no matching network, and the RF supply is a 2 nH 0402 + 100 nF. Nothing
// in this firmware may call esp_wifi_init() or esp_bt_controller_init().
// (CONFIG_ESP_WIFI_ENABLED cannot be asserted on: it is a non-prompt SoC
// capability symbol, always y on an S3 — the BT half below is the settable one.)

#ifdef CONFIG_BT_ENABLED
#error "radio_board: the daughterboard leaves the S3's LNA_IN floating with no \
matching network — no 2.4 GHz radio may be brought up on this board."
#endif
#ifdef CONFIG_SPIRAM
#error "radio_board: the as-built board's S3 has no in-package PSRAM at all, \
and on the S3RH2 revision the PSRAM shares VDD_SPI (fed through ~14 ohm) with \
the boot flash — both active sags that rail under the 2.7 V minimum of each. \
See sdkconfig.defaults."
#endif

static const char* TAG = "radio_board";

using namespace radio_modem;

static TR_UART_Link uart_link;
static TR_LoRa_Comms radio;
static bool radio_up = false;

// ---- TX queue + in-flight tracking -----------------------------------------

struct TxEntry
{
    uint8_t seq;
    uint8_t len;
    uint8_t data[MAX_AIR_FRAME];
};

static TxEntry tx_queue[TX_QUEUE_CAPACITY];
static uint8_t txq_head = 0;  // next to send
static uint8_t txq_used = 0;

struct InFlight
{
    bool active = false;
    uint8_t seq = 0;
    uint32_t ok_before = 0;
};
static InFlight in_flight;

// ---- Indicator LEDs ----------------------------------------------------------
// TX LED tracks the transmission (airtime-length flash); RX LED pulses
// LED_RX_PULSE_MS per received air packet.

static uint32_t rx_led_off_at_ms = 0;

static uint32_t uptimeMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

static void ledWrite(int pin, bool lit)
{
    gpio_set_level(static_cast<gpio_num_t>(pin),
                   lit ? config::LED_ACTIVE_LEVEL : !config::LED_ACTIVE_LEVEL);
}

static void initLeds()
{
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << config::LED_RX_PIN) |
                      (1ULL << config::LED_TX_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    ledWrite(config::LED_RX_PIN, false);
    ledWrite(config::LED_TX_PIN, false);
}

// One visible blink at boot. On a board powered from J6 with no USB attached
// this is the only sign the S3 is running at all, which matters when the
// question is "is anything on this board alive". Blocking is fine here —
// nothing is listening yet.
static void blinkBoot()
{
    for (int i = 0; i < 2; i++)
    {
        ledWrite(config::LED_RX_PIN, true);
        ledWrite(config::LED_TX_PIN, true);
        vTaskDelay(pdMS_TO_TICKS(120));
        ledWrite(config::LED_RX_PIN, false);
        ledWrite(config::LED_TX_PIN, false);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void pulseRxLed()
{
    ledWrite(config::LED_RX_PIN, true);
    rx_led_off_at_ms = uptimeMs() + config::LED_RX_PULSE_MS;
}

static void serviceLeds()
{
    if (rx_led_off_at_ms != 0 &&
        static_cast<int32_t>(uptimeMs() - rx_led_off_at_ms) >= 0)
    {
        ledWrite(config::LED_RX_PIN, false);
        rx_led_off_at_ms = 0;
    }
}

// ---- Radio hardware probe ----------------------------------------------------
// Runs BEFORE TR_LoRa_Comms claims the pins, and answers a question begin()
// cannot: is the E220 module electrically there at all? A failed begin() is
// ambiguous -- dead module, dead SPI, or a config the chip rejected all look
// the same from the return code. This separates "the module is not answering
// at the pin level" from "the module answers but the link setup failed", which
// is the first thing worth knowing on a board that has seen reverse polarity.
//
// The test is a pull fight. U14's BUSY (pad 11) and DIO1 (pad 20) are
// push-pull outputs of the LLCC68 with no board pull resistors, so:
//   * level follows our internal pull  -> nothing is driving it (module
//     unpowered, dead, or the joint is open)
//   * level holds against our pull     -> the module is driving it, alive
// After a reset the LLCC68 holds BUSY high through its internal boot and
// releases it in standby (~1 ms typical), so a BUSY stuck high past the
// timeout is the same verdict as a floating one, reported separately because
// the fix differs (dead die vs. no VCC / open joint).

struct RadioProbe
{
    bool busy_driven = false;    // something is holding BUSY against our pull
    bool busy_released = false;  // BUSY went low after reset, i.e. booted
    bool dio1_driven = false;
    uint32_t busy_release_us = 0;
};

static bool pinHoldsAgainstPull(gpio_num_t pin)
{
    // Read with an internal pull-up, then a pull-down. A floating pin follows
    // both; a driven pin reads the same level twice.
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(200);
    const int with_pu = gpio_get_level(pin);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(200);
    const int with_pd = gpio_get_level(pin);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    return with_pu == with_pd;
}

static RadioProbe probeRadioHardware()
{
    RadioProbe p = {};
    const gpio_num_t rst = static_cast<gpio_num_t>(config::LORA_RST_PIN);
    const gpio_num_t busy = static_cast<gpio_num_t>(config::LORA_BUSY_PIN);
    const gpio_num_t dio1 = static_cast<gpio_num_t>(config::LORA_DIO1_PIN);

    gpio_config_t out = {};
    out.pin_bit_mask = 1ULL << config::LORA_RST_PIN;
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);

    gpio_config_t in = {};
    in.pin_bit_mask = (1ULL << config::LORA_BUSY_PIN) |
                      (1ULL << config::LORA_DIO1_PIN);
    in.mode = GPIO_MODE_INPUT;
    gpio_config(&in);

    // Same reset shape TR_LoRa_Comms::begin() uses, so a module that survives
    // this one will see nothing new from the driver.
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(rst, 1);

    const int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 50'000)
    {
        if (gpio_get_level(busy) == 0)
        {
            p.busy_released = true;
            p.busy_release_us = static_cast<uint32_t>(esp_timer_get_time() - t0);
            break;
        }
        esp_rom_delay_us(100);
    }

    p.busy_driven = pinHoldsAgainstPull(busy);
    p.dio1_driven = pinHoldsAgainstPull(dio1);
    return p;
}

static void logRadioProbe(const RadioProbe& p)
{
    if (p.busy_released && p.busy_driven)
    {
        ESP_LOGI(TAG, "radio probe: module ALIVE — BUSY released %u us after "
                      "reset and is driven (DIO1 %s)",
                 p.busy_release_us, p.dio1_driven ? "driven" : "floating");
        return;
    }
    if (!p.busy_driven)
    {
        ESP_LOGE(TAG, "radio probe: BUSY is FLOATING — nothing is driving U14 "
                      "pad 11. The module is unpowered, dead, or its joints are "
                      "open. Check +3V3 at U14 pad 1 before suspecting "
                      "firmware.");
        return;
    }
    ESP_LOGE(TAG, "radio probe: BUSY driven but STUCK HIGH past 50 ms — the "
                  "module has power and is driving the pin, but never finished "
                  "its boot. Suspect a damaged die or a bad NRST/SPI joint.");
}

static void sendTxResult(uint8_t seq, bool ok)
{
    const TxResultData res = {seq, static_cast<uint8_t>(ok ? 1 : 0)};
    uart_link.sendFrame(MSG_TX_RESULT,
                        reinterpret_cast<const uint8_t*>(&res), sizeof(res));
}

// ---- Outbound status/identity ----------------------------------------------

static void sendIdentity(uint8_t msg_type)
{
    ModemIdentityData id = {};
    id.protocol_version = PROTOCOL_VERSION;
    id.chip = CHIP_LLCC68;
    id.max_tx_power_dbm = config::RADIO_MAX_TX_POWER_DBM;
    id.freq_min_mhz = config::RADIO_FREQ_MIN_MHZ;
    id.freq_max_mhz = config::RADIO_FREQ_MAX_MHZ;
    const esp_app_desc_t* app = esp_app_get_description();
    strncpy(id.fw_version, app->version, sizeof(id.fw_version) - 1);
    uart_link.sendFrame(msg_type, reinterpret_cast<const uint8_t*>(&id),
                        sizeof(id));
}

// #835 item 7: result of the most recent SET_CONFIG.  Starts true so a STATUS
// polled before any config is not reported as a failure.
static bool last_set_config_ok = true;

static void sendStatus()
{
    TR_LoRa_Comms::Stats rs = {};
    radio.getStats(rs);
    TR_UART_Link::Stats us = {};
    uart_link.getStats(us);

    ModemStatusData st = {};
    st.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    st.radio_enabled = radio_up ? 1 : 0;
    st.rx_mode = rs.rx_mode ? 1 : 0;
    st.tx_queue_used = txq_used + (in_flight.active ? 1 : 0);
    st.tx_queue_capacity = TX_QUEUE_CAPACITY;
    st.tx_ok = rs.tx_ok;
    st.tx_fail = rs.tx_fail;
    st.rx_count = rs.rx_count;
    st.rx_crc_fail = rs.rx_crc_fail;
    st.tx_watchdog_fires = rs.tx_watchdog_fires;
    st.uart_rx_crc_fails = us.rx_crc_fails;
    st.uart_rx_resync_bytes = us.rx_resync_bytes;
    st.last_rssi_dbm = rs.last_rssi;
    st.last_snr_db = rs.last_snr;
    st.current_freq_mhz = radio.currentFrequencyMHz();
    st.current_sf = radio.currentSpreadingFactor();
    st.config_ok = last_set_config_ok ? CFG_ACK_APPLIED : CFG_ACK_REJECTED;
    uart_link.sendFrame(MSG_STATUS, reinterpret_cast<const uint8_t*>(&st),
                        sizeof(st));
}

// ---- Inbound message handling ------------------------------------------------

static TR_LoRa_Comms::Config radioConfigFromMsg(const RadioConfigData& d)
{
    TR_LoRa_Comms::Config cfg = {};
    cfg.enabled = true;
    cfg.cs_pin = config::LORA_CS_PIN;
    cfg.dio1_pin = config::LORA_DIO1_PIN;
    cfg.rst_pin = config::LORA_RST_PIN;
    cfg.busy_pin = config::LORA_BUSY_PIN;
    cfg.rxen_pin = config::LORA_RXEN_PIN;
    cfg.spi_sck = config::LORA_SPI_SCK;
    cfg.spi_miso = config::LORA_SPI_MISO;
    cfg.spi_mosi = config::LORA_SPI_MOSI;
    cfg.freq_mhz = d.freq_mhz;
    cfg.spreading_factor = d.spreading_factor;
    cfg.bandwidth_khz = d.bandwidth_khz;
    cfg.coding_rate = d.coding_rate;
    cfg.preamble_len = d.preamble_len;
    cfg.tx_power_dbm = d.tx_power_dbm;
    cfg.crc_on = (d.flags & CFG_FLAG_CRC_ON) != 0;
    cfg.rx_boosted_gain = (d.flags & CFG_FLAG_RX_BOOSTED_GAIN) != 0;
    cfg.syncword_private = (d.flags & CFG_FLAG_SYNCWORD_PRIVATE) != 0;
    return cfg;
}

static void handleTxFrame(const uint8_t* payload, size_t len)
{
    if (len < sizeof(TxFrameHeader) + 1)
    {
        return;  // no seq to answer — malformed, drop
    }
    const uint8_t seq = payload[0];
    const uint8_t* air = payload + sizeof(TxFrameHeader);
    const size_t air_len = len - sizeof(TxFrameHeader);

    if (air_len > MAX_AIR_FRAME || !radio_up)
    {
        sendTxResult(seq, false);
        return;
    }
    if (txq_used >= TX_QUEUE_CAPACITY)
    {
        // Host exceeded its credit window (or credits desynced) — reject so
        // the credit is returned rather than leaked.
        sendTxResult(seq, false);
        return;
    }

    TxEntry& e = tx_queue[(txq_head + txq_used) % TX_QUEUE_CAPACITY];
    e.seq = seq;
    e.len = static_cast<uint8_t>(air_len);
    memcpy(e.data, air, air_len);
    txq_used++;
}

static void handleSetConfig(const uint8_t* payload, size_t len)
{
    if (len != sizeof(RadioConfigData))
    {
        ESP_LOGW(TAG, "SET_CONFIG bad length %u", (unsigned)len);
        return;
    }
    RadioConfigData d;
    memcpy(&d, payload, sizeof(d));
    // #835 item 7: tracks whether this SET_CONFIG actually reached the air.
    bool cfg_ok = true;

    // Clamp to module capability — hosts should already respect IDENTITY,
    // but a matched-pair mismatch must degrade, not transmit out of spec.
    if (d.tx_power_dbm > config::RADIO_MAX_TX_POWER_DBM)
    {
        d.tx_power_dbm = config::RADIO_MAX_TX_POWER_DBM;
    }

    if (!radio_up)
    {
        // Radio never came up (or wasn't wired at boot) — full begin(), which
        // applies every field including preamble/flags.
        radio_up = radio.begin(radioConfigFromMsg(d), config::DEBUG);
        cfg_ok = radio_up;
    }
    else
    {
        // SET_CONFIG is fully authoritative: modulation via reconfigure(),
        // then the frame-format fields (preamble/CRC/gain/syncword) via
        // applyFrameParams(). Without the second half, a host whose preamble
        // differs from our boot default would silently keep ours — its
        // airtime model (FCC dwell accounting, RocketComputerTypes.h) counts
        // preamble symbols it never actually got.
        if (radio.reconfigure(d.freq_mhz, d.spreading_factor, d.bandwidth_khz,
                              d.coding_rate, d.tx_power_dbm))
        {
            // #835 item 7: the frame-format half counts too.  A host whose
            // preamble silently stayed at our boot default has an airtime
            // model that counts symbols never sent, so a failure here is a
            // config failure, not a cosmetic one.
            cfg_ok = radio.applyFrameParams(
                d.preamble_len, (d.flags & CFG_FLAG_CRC_ON) != 0,
                (d.flags & CFG_FLAG_RX_BOOSTED_GAIN) != 0,
                (d.flags & CFG_FLAG_SYNCWORD_PRIVATE) != 0);
        }
        else
        {
            // reconfigure() rolled the radio back to the PREVIOUS modulation
            // and left it up.  Without this the STATUS below would ack with
            // radio_enabled=1 and the host would cache a modulation that is
            // not on the air (#835 item 7).
            cfg_ok = false;
        }
    }
    if (radio_up && d.start_rx)
    {
        radio.startReceive();
    }
    // #835 item 7: publish the result BEFORE the ack, so the STATUS the host
    // is waiting on carries this config's outcome rather than the previous.
    last_set_config_ok = cfg_ok;
    if (!cfg_ok)
    {
        ESP_LOGW(TAG, "SET_CONFIG REJECTED (%.3f MHz SF%u BW%u CR%u) — radio "
                      "rolled back; acking with config_ok=0",
                 (double)d.freq_mhz, (unsigned)d.spreading_factor,
                 (unsigned)d.bandwidth_khz, (unsigned)d.coding_rate);
    }
    sendStatus();  // config paths always get a status echo as the ack
}

static void onHostFrame(void* /*ctx*/, uint8_t type, const uint8_t* payload,
                        size_t len)
{
    switch (type)
    {
        case MSG_TX_FRAME:
            handleTxFrame(payload, len);
            break;

        case MSG_SET_CONFIG:
            handleSetConfig(payload, len);
            break;

        case MSG_HOP_FREQ:
            if (len == sizeof(HopFreqData) && radio_up)
            {
                HopFreqData d;
                memcpy(&d, payload, sizeof(d));
                // Fire-and-forget by design (per-packet hop latency path):
                // a mid-TX refusal keeps the current channel, matching the
                // direct driver's can't-retune-mid-TX contract.
                (void)radio.hopToFrequencyMHz(d.freq_mhz);
            }
            break;

        case MSG_START_RX:
            if (radio_up)
            {
                radio.startReceive();
            }
            break;

        case MSG_GET_STATUS:
            sendStatus();
            break;

        case MSG_GET_IDENTITY:
            sendIdentity(MSG_IDENTITY);
            break;

        case MSG_START_SCAN:
            if (len == sizeof(ScanRequestData) && radio_up)
            {
                ScanRequestData d;
                memcpy(&d, payload, sizeof(d));
                radio.startScan(d.start_mhz, d.stop_mhz, d.step_khz, d.dwell_ms);
            }
            break;

        default:
            ESP_LOGW(TAG, "unknown host msg type 0x%02X (%u B)", type,
                     (unsigned)len);
            break;
    }
}

// ---- Radio-side servicing ----------------------------------------------------

static void serviceTx()
{
    if (!radio_up)
    {
        return;
    }

    // Close out a completed transmission: service() has run finishTransmit
    // (or the TX watchdog force-cleared a wedge) once canSend() goes true.
    if (in_flight.active && radio.canSend())
    {
        TR_LoRa_Comms::Stats rs = {};
        radio.getStats(rs);
        sendTxResult(in_flight.seq, rs.tx_ok > in_flight.ok_before);
        in_flight.active = false;
        ledWrite(config::LED_TX_PIN, false);
    }

    if (!in_flight.active && txq_used > 0 && radio.canSend() &&
        !radio.isScanActive())
    {
        TxEntry& e = tx_queue[txq_head];
        TR_LoRa_Comms::Stats rs = {};
        radio.getStats(rs);
        if (radio.send(e.data, e.len))
        {
            in_flight.active = true;
            in_flight.seq = e.seq;
            in_flight.ok_before = rs.tx_ok;
            ledWrite(config::LED_TX_PIN, true);
        }
        else
        {
            sendTxResult(e.seq, false);
        }
        txq_head = (txq_head + 1) % TX_QUEUE_CAPACITY;
        txq_used--;
    }
}

static void serviceRx()
{
    if (!radio_up)
    {
        return;
    }
    // RX_FRAME payload = RxFrameHeader + air bytes, packed in one buffer.
    static uint8_t out[sizeof(RxFrameHeader) + MAX_AIR_FRAME];
    size_t air_len = 0;
    if (!radio.readPacket(out + sizeof(RxFrameHeader), MAX_AIR_FRAME, air_len))
    {
        return;
    }
    TR_LoRa_Comms::Stats rs = {};
    radio.getStats(rs);
    RxFrameHeader hdr = {rs.last_rssi, rs.last_snr};
    memcpy(out, &hdr, sizeof(hdr));
    uart_link.sendFrame(MSG_RX_FRAME, out, sizeof(hdr) + air_len);
    pulseRxLed();
}

static void serviceScanResult()
{
    if (!radio_up || !radio.isScanDone())
    {
        return;
    }

    const size_t n = radio.getScanSampleCount();
    const TR_LoRa_Comms::ScanSample* samples = radio.getScanSamples();

    static uint8_t out[sizeof(ScanResultHeader) + TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    ScanResultHeader hdr = {};
    hdr.count = static_cast<uint8_t>(n);
    hdr.start_mhz = radio.getScanStartMHz();
    hdr.step_khz = radio.getScanStepKHz();
    memcpy(out, &hdr, sizeof(hdr));
    for (size_t i = 0; i < n; i++)
    {
        out[sizeof(hdr) + i] = static_cast<uint8_t>(samples[i].rssi_dbm);
    }
    uart_link.sendFrame(MSG_SCAN_RESULT, out, sizeof(hdr) + n);
    radio.consumeScanDone();
    radio.startReceive();  // resume listening after the scan sweep
}

// ---- Entry -------------------------------------------------------------------

extern "C" void app_main(void)
{
    const esp_app_desc_t* app = esp_app_get_description();
    ESP_LOGI(TAG, "radio_board fw %s (protocol v%u)", app->version,
             PROTOCOL_VERSION);

    initLeds();
    blinkBoot();

    TR_UART_Link::Config ucfg = {};
    ucfg.port = config::HOST_UART_PORT;
    ucfg.tx_pin = config::HOST_UART_TX;
    ucfg.rx_pin = config::HOST_UART_RX;
    ucfg.baud = config::HOST_UART_BAUD;
    ESP_ERROR_CHECK(uart_link.begin(ucfg));

    // Bring the radio up on boot defaults so the modem listens immediately;
    // the host's SET_CONFIG replaces these (and is also the recovery path if
    // the radio fails here — see handleSetConfig).
    RadioConfigData boot = {};
    boot.freq_mhz = config::BOOT_FREQ_MHZ;
    boot.bandwidth_khz = config::BOOT_BW_KHZ;
    boot.spreading_factor = config::BOOT_SF;
    boot.coding_rate = config::BOOT_CR;
    boot.tx_power_dbm = config::BOOT_TX_POWER_DBM;
    // 12, matching both hosts' LORA_PREAMBLE_LEN. This mattered more than it
    // looks: the hosts' airtime model (LORA_TELEM_PREAMBLE_SYMS,
    // RocketComputerTypes.h) assumes their preamble is what's on the air,
    // and the FCC dwell budget on the tightest hop rung has ~2 ms of margin
    // — a 16-symbol boot default that survived (pre-applyFrameParams) blew
    // that budget silently.
    boot.preamble_len = 12;
    boot.flags = CFG_FLAG_CRC_ON | CFG_FLAG_RX_BOOSTED_GAIN |
                 CFG_FLAG_SYNCWORD_PRIVATE;
    // Probe first: if the module is not answering at the pin level, the
    // begin() failure below is a symptom, not the finding.
    logRadioProbe(probeRadioHardware());

    radio_up = radio.begin(radioConfigFromMsg(boot), config::DEBUG);
    if (radio_up)
    {
        radio.startReceive();
        ESP_LOGI(TAG, "radio up, listening at %.1f MHz SF%u",
                 (double)config::BOOT_FREQ_MHZ, config::BOOT_SF);
    }
    else
    {
        TR_LoRa_Comms::Stats rs = {};
        radio.getStats(rs);
        ESP_LOGE(TAG, "radio init FAILED (RadioLib %d) — modem alive, RF dead "
                      "(host sees radio_enabled=0)", rs.last_error);
    }

#ifdef TR_BENCH_SELFTEST
    // Bench build only (see main/CMakeLists.txt). Runs before BOOT is
    // announced so a host can never see the loopback traffic ahead of the
    // handshake — though a bench image should not be plugged into a host at
    // all.
    bench::runSelfTest(uart_link, radio, config::HOST_UART_PORT, radio_up);
#endif

    // Announce boot: identity payload doubles as the host's signal to reset
    // its credit window (our TX queue is empty now).
    sendIdentity(MSG_BOOT);

    for (;;)
    {
        uart_link.poll(onHostFrame, nullptr);

        if (radio_up)
        {
            radio.pollDio1();
            radio.service();
            radio.serviceTxWatchdog();
            if (radio.isScanActive())
            {
                radio.serviceScan();
            }
        }

        serviceTx();
        serviceRx();
        serviceScanResult();
        serviceLeds();

        vTaskDelay(1);  // 1 ms at CONFIG_FREERTOS_HZ=1000
    }
}
