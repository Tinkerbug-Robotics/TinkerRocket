#include "bench_selftest.h"

#ifdef TR_BENCH_SELFTEST

#include <cstring>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <RadioModemProtocol.h>
#include <TR_MsgCodec.h>

namespace bench
{

static const char* TAG = "bench";

using namespace radio_modem;

// ---------------------------------------------------------------------------
// Capture harness: collect frames the deframer hands back
// ---------------------------------------------------------------------------

namespace
{

struct Capture
{
    static constexpr size_t MAX_FRAMES = 64;
    struct Frame
    {
        uint8_t type;
        size_t len;
        uint8_t payload[tr_msg::MAX_PAYLOAD];
    };
    Frame frames[MAX_FRAMES];
    size_t count = 0;
    size_t overflow = 0;

    void reset()
    {
        count = 0;
        overflow = 0;
    }
};

Capture g_cap;

void onFrame(void* /*ctx*/, uint8_t type, const uint8_t* payload, size_t len)
{
    if (g_cap.count >= Capture::MAX_FRAMES)
    {
        g_cap.overflow++;
        return;
    }
    Capture::Frame& f = g_cap.frames[g_cap.count++];
    f.type = type;
    f.len = len;
    if (len > 0)
    {
        memcpy(f.payload, payload, len);
    }
}

// Pump the link for up to timeout_ms, stopping early once `want` frames are in.
void pumpUntil(TR_UART_Link& link, size_t want, uint32_t timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline)
    {
        link.poll(onFrame, nullptr);
        if (g_cap.count >= want)
        {
            // One more poll: a trailing frame may already be in the FIFO, and
            // "we got what we asked for" must not hide an extra arrival.
            link.poll(onFrame, nullptr);
            return;
        }
        vTaskDelay(1);
    }
}

struct Results
{
    int checks = 0;
    int failures = 0;

    void check(bool ok, const char* what)
    {
        checks++;
        if (ok)
        {
            ESP_LOGI(TAG, "  ok   %s", what);
        }
        else
        {
            failures++;
            ESP_LOGE(TAG, "  FAIL %s", what);
        }
    }
};

// Write raw bytes straight at the UART, bypassing the codec — needed to feed
// the deframer garbage and deliberately corrupted frames.
void writeRaw(uart_port_t port, const uint8_t* data, size_t len)
{
    uart_write_bytes(port, reinterpret_cast<const char*>(data), len);
}

}  // namespace

// ---------------------------------------------------------------------------
// UART link tests (internal loopback — no adapter, no host)
// ---------------------------------------------------------------------------

static void testUartLink(TR_UART_Link& link, uart_port_t port, Results& r)
{
    ESP_LOGI(TAG, "UART link (internal loopback @ real baud):");

    ESP_ERROR_CHECK(uart_set_loop_back(port, true));
    // Anything already in flight would land in the middle of test 1.
    uart_flush_input(port);

    TR_UART_Link::Stats before = {};
    link.getStats(before);

    // --- 1. Round trip across the payload-size range, including the edges ---
    // 0 is the common case on this link (GET_STATUS et al) and 255 is the
    // codec's ceiling; both are where a length bug shows up first.
    {
        const size_t sizes[] = {0, 1, 16, 64, sizeof(ModemStatusData),
                                MAX_AIR_FRAME, tr_msg::MAX_PAYLOAD};
        const size_t n = sizeof(sizes) / sizeof(sizes[0]);
        static uint8_t payload[tr_msg::MAX_PAYLOAD];
        for (size_t i = 0; i < sizeof(payload); i++)
        {
            payload[i] = static_cast<uint8_t>(i * 7 + 3);  // non-trivial pattern
        }

        g_cap.reset();
        for (size_t i = 0; i < n; i++)
        {
            link.sendFrame(static_cast<uint8_t>(0x40 + i), payload, sizes[i]);
        }
        pumpUntil(link, n, 500);

        bool ok = (g_cap.count == n);
        for (size_t i = 0; ok && i < n; i++)
        {
            ok = g_cap.frames[i].type == static_cast<uint8_t>(0x40 + i) &&
                 g_cap.frames[i].len == sizes[i] &&
                 memcmp(g_cap.frames[i].payload, payload, sizes[i]) == 0;
        }
        if (!ok)
        {
            ESP_LOGE(TAG, "   got %u/%u frames back", (unsigned)g_cap.count,
                     (unsigned)n);
        }
        r.check(ok, "round trip, 7 payload sizes 0..255 B, bytes identical");
    }

    // --- 2. Back-to-back burst: does the ring buffer hold at 921600? --------
    // TR_UART_Link sizes rx_ring at 4096; 24 full frames is ~6.3 kB, so this
    // only passes if poll() is draining as fast as the wire fills.
    {
        static uint8_t payload[MAX_AIR_FRAME];
        memset(payload, 0xA5, sizeof(payload));
        const size_t burst = 24;

        g_cap.reset();
        const int64_t t0 = esp_timer_get_time();
        for (size_t i = 0; i < burst; i++)
        {
            // Interleave polling: this mirrors the real loop, which is the
            // only configuration whose ring-buffer behaviour we care about.
            link.sendFrame(0x51, payload, sizeof(payload));
            link.poll(onFrame, nullptr);
        }
        pumpUntil(link, burst, 1000);
        const int64_t dt = esp_timer_get_time() - t0;

        const size_t wire_bytes = burst * (tr_msg::FRAME_OVERHEAD + sizeof(payload));
        ESP_LOGI(TAG, "   %u frames (%u B on the wire) in %lld us -> %.0f kB/s",
                 (unsigned)burst, (unsigned)wire_bytes, dt,
                 (double)wire_bytes / ((double)dt / 1000.0));
        r.check(g_cap.count == burst && g_cap.overflow == 0,
                "24-frame back-to-back burst, none dropped");
    }

    // --- 3. Resynchronize through garbage ----------------------------------
    // Includes a false SOF prefix (AA AA 13 55), which is the case a naive
    // "restart at byte 0" hunt gets wrong.
    {
        TR_UART_Link::Stats s0 = {};
        link.getStats(s0);
        const uint8_t garbage[] = {0x00, 0xFF, 0xAA, 0xAA, 0x13, 0x55};

        g_cap.reset();
        writeRaw(port, garbage, sizeof(garbage));
        const uint8_t payload[] = {0x01, 0x02};
        link.sendFrame(0x33, payload, sizeof(payload));
        pumpUntil(link, 1, 500);

        TR_UART_Link::Stats s1 = {};
        link.getStats(s1);
        const uint32_t resynced = s1.rx_resync_bytes - s0.rx_resync_bytes;
        ESP_LOGI(TAG, "   discarded %u B hunting for SOF (6 B of garbage in)",
                 resynced);
        // 5, not 6: the counter tracks what the state machine DISCARDS, and
        // one of the two 0xAAs is consumed as a SOF candidate rather than
        // dropped. Pinned at 5 because this number goes to hosts in
        // STATUS.uart_rx_resync_bytes and anything that recomputes it (the
        // bench host did) must agree.
        r.check(g_cap.count == 1 && g_cap.frames[0].type == 0x33 &&
                    g_cap.frames[0].len == 2 && resynced == 5,
                "resyncs past garbage incl. a false SOF, counts 5 of 6 B");
    }

    // --- 4. A corrupted frame is dropped, counted, and recovered from -------
    {
        TR_UART_Link::Stats s0 = {};
        link.getStats(s0);

        uint8_t frame[tr_msg::MAX_FRAME];
        size_t frame_len = 0;
        const uint8_t bad_payload[] = {0x0A, 0x14, 0x1E};
        tr_msg::pack(0x44, bad_payload, sizeof(bad_payload), frame,
                     sizeof(frame), frame_len);
        frame[7] ^= 0xFF;  // flip a payload byte -> CRC mismatch

        g_cap.reset();
        writeRaw(port, frame, frame_len);
        const uint8_t good[] = {0x2A};
        link.sendFrame(0x55, good, sizeof(good));
        pumpUntil(link, 1, 500);

        TR_UART_Link::Stats s1 = {};
        link.getStats(s1);
        r.check(g_cap.count == 1 && g_cap.frames[0].type == 0x55 &&
                    (s1.rx_crc_fails - s0.rx_crc_fails) == 1,
                "CRC-corrupted frame dropped + counted, next frame survives");
    }

    // --- 5. A truncated frame costs exactly two frames, then recovers ------
    // The deframer is length-driven: a frame cut short keeps consuming, so it
    // eats the NEXT frame's SOF as its own payload/CRC, fails the CRC, and
    // only recovers on the frame after that. That is the documented contract
    // ("a rare drop is recovered by the next frame", TR_MsgCodec.h), not a
    // defect — but it is worth pinning, because the obvious host-side
    // implementation (scan forward to the next SOF) loses only one frame and
    // would therefore report better link quality than the modem really has.
    {
        TR_UART_Link::Stats s0 = {};
        link.getStats(s0);

        uint8_t frame[tr_msg::MAX_FRAME];
        size_t frame_len = 0;
        const uint8_t payload[] = {1, 2, 3, 4, 5, 6, 7, 8};
        tr_msg::pack(0x66, payload, sizeof(payload), frame, sizeof(frame),
                     frame_len);

        g_cap.reset();
        writeRaw(port, frame, frame_len - 4);  // cut it short
        const uint8_t eaten[] = {0xEE};
        link.sendFrame(0x77, eaten, sizeof(eaten));   // expected casualty
        const uint8_t good[] = {0xEF};
        link.sendFrame(0x78, good, sizeof(good));     // must get through
        pumpUntil(link, 1, 500);

        TR_UART_Link::Stats s1 = {};
        link.getStats(s1);
        r.check(g_cap.count == 1 && g_cap.frames[0].type == 0x78 &&
                    g_cap.frames[0].payload[0] == 0xEF &&
                    (s1.rx_crc_fails - s0.rx_crc_fails) == 1,
                "truncated frame swallows exactly one follower, then recovers");
    }

    // --- 6. Codec agrees with the host-side golden bytes --------------------
    // Same literal frame asserted in tests_cpp and tools/bench_radio_modem.py.
    // Cheap to check here too, and it proves the ON-CHIP build produces it —
    // the host tests compile the same source with a different toolchain.
    {
        uint8_t frame[tr_msg::MAX_FRAME];
        size_t frame_len = 0;
        const uint8_t payload[] = {0x01, 0x02, 0x03};
        tr_msg::pack(0x42, payload, sizeof(payload), frame, sizeof(frame),
                     frame_len);
        const uint8_t golden[] = {0xAA, 0x55, 0xAA, 0x55, 0x42, 0x03,
                                  0x01, 0x02, 0x03, 0x00, 0x66};
        r.check(frame_len == sizeof(golden) &&
                    memcmp(frame, golden, sizeof(golden)) == 0,
                "on-chip codec matches the host golden frame");
    }

    ESP_ERROR_CHECK(uart_set_loop_back(port, false));
    uart_flush_input(port);
}

// ---------------------------------------------------------------------------
// Radio receive-path test (no transmission — safe with no antenna)
// ---------------------------------------------------------------------------

static void testRadioRxPath(TR_LoRa_Comms& radio, Results& r)
{
    ESP_LOGI(TAG, "Radio RX path (spectrum scan, receive-only):");

    // 902-928 MHz at 500 kHz = 53 samples, inside SCAN_MAX_SAMPLES (128).
    // The sweep drives the SPI command path, retunes the synthesiser 53
    // times, enters RX at each step (toggling RXEN, i.e. the half of the RF
    // switch this MCU actually owns) and reads RSSI back out of the chip.
    if (!radio.startScan(902.0f, 928.0f, 500, 30))
    {
        r.check(false, "startScan accepted");
        return;
    }

    const int64_t deadline = esp_timer_get_time() + 15'000'000;
    while (!radio.isScanDone() && esp_timer_get_time() < deadline)
    {
        radio.serviceScan();
        vTaskDelay(1);
    }
    if (!radio.isScanDone())
    {
        r.check(false, "scan completed within 15 s");
        return;
    }

    const size_t n = radio.getScanSampleCount();
    const TR_LoRa_Comms::ScanSample* s = radio.getScanSamples();
    r.check(n > 0, "scan returned samples");
    if (n == 0)
    {
        radio.consumeScanDone();
        return;
    }

    int lo = 127, hi = -128;
    long sum = 0;
    size_t pegged = 0;
    for (size_t i = 0; i < n; i++)
    {
        const int v = s[i].rssi_dbm;
        lo = v < lo ? v : lo;
        hi = v > hi ? v : hi;
        sum += v;
        // A dead receive path reads as a rail, not as noise.
        if (v == 0 || v <= -127)
        {
            pegged++;
        }
    }
    const int mean = static_cast<int>(sum / (long)n);
    ESP_LOGI(TAG, "   %u samples 902-928 MHz: min %d, mean %d, max %d dBm",
             (unsigned)n, lo, mean, hi);

    // Coarse profile so a real signal or a stuck reading is visible by eye.
    char bar[80];
    const size_t width = n < sizeof(bar) - 1 ? n : sizeof(bar) - 1;
    for (size_t i = 0; i < width; i++)
    {
        const int v = s[i * n / width].rssi_dbm;
        bar[i] = v > -60 ? '#' : v > -80 ? '+' : v > -100 ? '.' : ' ';
    }
    bar[width] = '\0';
    ESP_LOGI(TAG, "   902 MHz [%s] 928 MHz   (' '<-100  '.'<-80  '+'<-60  '#')",
             bar);

    r.check(pegged < n,
            "RSSI is a noise floor, not a stuck rail (0 or -127 dBm)");
    // A healthy 900 MHz front end sits well below -80 dBm on a quiet bench;
    // anything above -50 across the whole sweep means the reading is not noise.
    r.check(mean < -50, "mean noise floor is plausible for a quiet bench");

    radio.consumeScanDone();
    radio.startReceive();  // leave the radio as we found it
}

// ---------------------------------------------------------------------------

int runSelfTest(TR_UART_Link& link, TR_LoRa_Comms& radio, uart_port_t port,
                bool radio_up)
{
    Results r;
    ESP_LOGW(TAG, "==== bench self-test (TR_BENCH_SELFTEST build) ====");

    testUartLink(link, port, r);

    if (radio_up)
    {
        testRadioRxPath(radio, r);
    }
    else
    {
        ESP_LOGW(TAG, "Radio RX path: SKIPPED — radio never initialised");
    }

    if (r.failures == 0)
    {
        ESP_LOGW(TAG, "==== self-test PASSED (%d checks) ====", r.checks);
    }
    else
    {
        ESP_LOGE(TAG, "==== self-test FAILED: %d of %d checks ====", r.failures,
                 r.checks);
    }
    return r.failures;
}

}  // namespace bench

#endif  // TR_BENCH_SELFTEST
