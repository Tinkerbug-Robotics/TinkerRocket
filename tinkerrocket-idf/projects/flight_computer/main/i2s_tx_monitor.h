#pragma once

// Bench diagnostic, TR_I2S_TX_MONITOR=1 builds only: what the FC's I2S TX
// really puts on its pins, frame by frame. It found the Beetle's IMU batch
// gaps (2026-10-12, i2s_tx_fill_policy.h). Not for flight.
//
//   idf.py -B build_m1_mon -DTR_BOARD_M1=1 -DTR_I2S_TX_MONITOR=1 build
//
// Two views of the link, parsed the way the OC parses it (i2s_link_check.h)
// and matched in order against the sender's own log of what it wrote:
//   tx  on_sent fires as each DMA buffer finishes, before auto_clear wipes
//       it, so a copy taken there is the buffer as the DMA left it. A byte
//       copied in after the DMA had read past it shows here but went out as
//       zero, so this view cannot see the race; it gives each frame's margin,
//       the estimated read time of its first byte minus the time its write
//       began (from the EOF clock: the GDMA's read-ahead, and a preemption
//       between the start of a write and its copy, both make the true margin
//       smaller).
//   rx  I2S1 as a slave receiver on the TX pins (input path only; I2S0 keeps
//       driving them): the FC's own bitstream, as the OC gets it less the
//       board traces. A frame this view never sees was lost on the FC.
//
// Console, once a second (counts for that second):
//   I2SMON: 1s rx frames= crc= miss=(imu) late= unm= gaps= lost= | tx ... mmin= | ovf tx/rx= | lost: TT:m ...
//     miss   written, skipped over and never seen: lost (rx) or never sent (tx)
//     late   seen after frames written later
//     unm    seen, but nothing written matches
//     gaps   ISM6 batch-time gaps as received, lost = the samples in them
//     mmin   the smallest tx margin that second, us
//     ovf    the monitor's own snapshot rings overflowed (it fell behind):
//            a second whose ovf moved is not evidence
//     lost:  type:tx-margin of each rx loss (-999999: no tx record)
// and every 10 s the totals and three margin histograms (all frames, lost
// frames, lost IMU batches).
//
// Costs: ~80 KB of internal RAM (on the Beetle 32-deep rings left the flight
// task's stack no room, and it never started), two 1 KB copies per DMA
// buffer in ISRs, and a priority-1 task on the sensor core. Uses I2S_NUM_1.
// After an OTA revert the tx view re-hooks the new channel; the rx view
// keeps listening (during the flip it sees the OC's image frames as unm).

#include <driver/i2s_std.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdio>
#include <cstring>
#include <new>

#include <RocketComputerTypes.h>

#include "i2s_link_check.h"

namespace i2smon
{

static constexpr size_t   BUF_MAX = 1024;    // both DMA rings: 256 frames x 4 B
static constexpr uint32_t SNAPS   = 24;      // per ring (16 overflowed at 7680 Hz)
static constexpr uint32_t WRITES  = 2048;    // the sender's write log
static constexpr uint32_t MARGINS = 2048;    // tx margins by frame key (power of two)
static constexpr uint32_t HOLD_US = 100000;  // a skipped write is missing after this
static const char *const  TAG     = "I2SMON";

struct Snap
{
    uint32_t t_eof;
    uint32_t len;
    uint8_t data[BUF_MAX];
};

struct Ring
{
    Snap *snaps = nullptr;
    volatile uint32_t head = 0;
    volatile uint32_t tail = 0;
    volatile uint32_t ovf = 0;
};

static Ring tx_ring, rx_ring;
static i2slink::WriteLog<WRITES> *wlog = nullptr;
static TaskHandle_t task = nullptr;
static i2s_chan_handle_t loop_chan = nullptr;
static volatile bool tx_restarted = false;
static float us_per_byte = 1e6f / (88200.0f * 4.0f);

// The sender, just before each writeFrame().
static inline void noteWrite(uint8_t type, const uint8_t *payload, size_t len)
{
    if (wlog) wlog->add(type, (uint8_t)len, payload, (uint32_t)esp_timer_get_time());
}

static IRAM_ATTR bool push(Ring &r, const i2s_event_data_t *e)
{
    const uint32_t h = r.head;
    if (h - r.tail >= SNAPS)
    {
        r.ovf = r.ovf + 1;
        return false;
    }
    Snap &s = r.snaps[h % SNAPS];
    s.t_eof = (uint32_t)esp_timer_get_time();
    s.len = e->size < BUF_MAX ? e->size : BUF_MAX;
    memcpy(s.data, e->dma_buf, s.len);
    r.head = h + 1;
    BaseType_t woke = pdFALSE;
    if (task) vTaskNotifyGiveFromISR(task, &woke);
    return woke == pdTRUE;
}
static IRAM_ATTR bool onSent(i2s_chan_handle_t, i2s_event_data_t *e, void *) { return push(tx_ring, e); }
static IRAM_ATTR bool onRecv(i2s_chan_handle_t, i2s_event_data_t *e, void *) { return push(rx_ring, e); }

// ---- the parser task's state ----------------------------------------------

static constexpr int BINS = 10;
static const int32_t kEdgeUs[BINS - 1] = { 0, 200, 400, 600, 800, 1000, 1500, 2000, 3000 };
static int binOf(int32_t m)
{
    int b = 0;
    while (b < BINS - 1 && m >= kEdgeUs[b]) ++b;
    return b;
}

struct Margin
{
    uint32_t key;
    int32_t us;
};

struct Counts
{
    uint32_t frames, miss, miss_imu, late, unm;
};

struct View
{
    i2slink::FrameStream<BUF_MAX> stream{ MAX_PAYLOAD };
    i2slink::Matcher<WRITES> match;
    i2slink::ImuGapTracker imu{ SIZE_OF_ISM6HG256_DATA };
    Counts n{};
    // the last two buffers: a frame's first byte is in one of them
    uint32_t id[2] = { 0, 0 };
    uint32_t t_eof[2] = { 0, 0 };
    uint32_t len[2] = { 0, 0 };
};

static View *vtx = nullptr;
static View *vrx = nullptr;
static Margin *margins = nullptr;
static uint32_t hist_all[BINS], hist_lost[BINS], hist_lost_imu[BINS], lost_no_tx = 0;
static int32_t mmin_sec = INT32_MAX;
struct LostNote
{
    uint8_t type;
    int32_t margin;
};
static LostNote lost_sec[6];
static uint32_t n_lost_sec = 0;

static void putMargin(uint32_t key, int32_t us)
{
    uint32_t i = key & (MARGINS - 1);
    for (uint32_t p = 0; p < 8; ++p, i = (i + 1) & (MARGINS - 1))
    {
        if (margins[i].key == 0 || margins[i].key == key)
        {
            margins[i] = Margin{ key, us };
            return;
        }
    }
    margins[key & (MARGINS - 1)] = Margin{ key, us };
}
static bool getMargin(uint32_t key, int32_t &us)
{
    uint32_t i = key & (MARGINS - 1);
    for (uint32_t p = 0; p < 8; ++p, i = (i + 1) & (MARGINS - 1))
    {
        if (margins[i].key == key)
        {
            us = margins[i].us;
            return true;
        }
        if (margins[i].key == 0) return false;
    }
    return false;
}

static void feedView(View &v, const Snap &s, uint32_t id, bool is_tx)
{
    v.id[0] = v.id[1];       v.id[1] = id;
    v.t_eof[0] = v.t_eof[1]; v.t_eof[1] = s.t_eof;
    v.len[0] = v.len[1];     v.len[1] = s.len;
    v.stream.feed(s.data, s.len, id, [&](const i2slink::FrameAt &f) {
        v.n.frames++;
        if (f.type == ISM6_BATCH_MSG) v.imu.batch(f.payload, f.len);
        const uint32_t key = i2slink::frameKey(f.type, f.len, f.payload);
        i2slink::Write w{};
        switch (v.match.seen(*wlog, f.type, key, s.t_eof, &w))
        {
            case i2slink::Matcher<WRITES>::Kind::Matched:
                if (is_tx)
                {
                    const int k = (f.buf_id == v.id[1]) ? 1 : 0;
                    const uint32_t t_read = v.t_eof[k] - (uint32_t)((float)(v.len[k] - f.offset) * us_per_byte);
                    const int32_t m = (int32_t)(t_read - w.t_us);
                    putMargin(key, m);
                    hist_all[binOf(m)]++;
                    if (m < mmin_sec) mmin_sec = m;
                }
                break;
            case i2slink::Matcher<WRITES>::Kind::Late:
                v.n.late++;
                break;
            case i2slink::Matcher<WRITES>::Kind::NotInLog:
                v.n.unm++;
                break;
        }
    });
    v.match.expire(s.t_eof, HOLD_US, [&](const i2slink::Write &w) {
        v.n.miss++;
        const bool imu = (w.type == ISM6_BATCH_MSG);
        if (imu) v.n.miss_imu++;
        if (is_tx) return;
        int32_t m = 0;
        if (getMargin(w.key, m))
        {
            hist_lost[binOf(m)]++;
            if (imu) hist_lost_imu[binOf(m)]++;
        }
        else
        {
            lost_no_tx++;
            m = -999999;
        }
        if (n_lost_sec < 6) lost_sec[n_lost_sec++] = LostNote{ w.type, m };
    });
}

static void drain(Ring &r, View &v, bool is_tx)
{
    while (r.tail != r.head)
    {
        const uint32_t id = r.tail;
        feedView(v, r.snaps[id % SNAPS], id, is_tx);
        r.tail = id + 1;
    }
}

static void printHist(const char *what, const uint32_t *h)
{
    ESP_LOGI(TAG, "%s margin us <0:%lu 0-200:%lu 200-400:%lu 400-600:%lu 600-800:%lu 800-1k:%lu "
                  "1-1.5k:%lu 1.5-2k:%lu 2-3k:%lu >3k:%lu",
             what, (unsigned long)h[0], (unsigned long)h[1], (unsigned long)h[2], (unsigned long)h[3],
             (unsigned long)h[4], (unsigned long)h[5], (unsigned long)h[6], (unsigned long)h[7],
             (unsigned long)h[8], (unsigned long)h[9]);
}

static void monitorTask(void *)
{
    int64_t last_print = esp_timer_get_time();
    uint32_t sec = 0;
    Counts prx{}, ptx{};
    uint32_t prx_crc = 0, ptx_crc = 0, prx_gaps = 0, prx_lost = 0, ptx_gaps = 0;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        if (tx_restarted)
        {
            tx_restarted = false;
            vtx->stream.reset();
        }
        drain(tx_ring, *vtx, true);
        drain(rx_ring, *vrx, false);

        const int64_t now = esp_timer_get_time();
        if (now - last_print < 1000000) continue;
        last_print = now;
        ++sec;

        char lost_s[96] = { 0 };
        size_t o = 0;
        for (uint32_t k = 0; k < n_lost_sec && o + 16 < sizeof lost_s; ++k)
            o += (size_t)snprintf(lost_s + o, sizeof lost_s - o, " %02X:%ld",
                                  lost_sec[k].type, (long)lost_sec[k].margin);
        const Counts &r = vrx->n, &t = vtx->n;
        const uint32_t rx_miss = r.miss + vrx->match.untracked();
        const uint32_t tx_miss = t.miss + vtx->match.untracked();
        ESP_LOGI(TAG, "1s rx frames=%lu crc=%lu miss=%lu(imu %lu) late=%lu unm=%lu gaps=%lu lost=%lu | "
                      "tx frames=%lu crc=%lu miss=%lu(imu %lu) late=%lu gaps=%lu mmin=%ld | ovf tx/rx=%lu/%lu | lost:%s",
                 (unsigned long)(r.frames - prx.frames), (unsigned long)(vrx->stream.crcFailures() - prx_crc),
                 (unsigned long)(rx_miss - prx.miss), (unsigned long)(r.miss_imu - prx.miss_imu),
                 (unsigned long)(r.late - prx.late), (unsigned long)(r.unm - prx.unm),
                 (unsigned long)(vrx->imu.gaps - prx_gaps), (unsigned long)(vrx->imu.lost - prx_lost),
                 (unsigned long)(t.frames - ptx.frames), (unsigned long)(vtx->stream.crcFailures() - ptx_crc),
                 (unsigned long)(tx_miss - ptx.miss), (unsigned long)(t.miss_imu - ptx.miss_imu),
                 (unsigned long)(t.late - ptx.late), (unsigned long)(vtx->imu.gaps - ptx_gaps),
                 (long)(mmin_sec == INT32_MAX ? -1 : mmin_sec),
                 (unsigned long)tx_ring.ovf, (unsigned long)rx_ring.ovf, lost_s);
        if (sec % 10 == 0)
        {
            ESP_LOGI(TAG, "total %lus: rx frames=%lu miss=%lu(imu %lu) late=%lu unm=%lu gaps=%lu lost=%lu | "
                          "tx frames=%lu miss=%lu(imu %lu) late=%lu gaps=%lu",
                     (unsigned long)sec, (unsigned long)r.frames, (unsigned long)rx_miss,
                     (unsigned long)r.miss_imu, (unsigned long)r.late, (unsigned long)r.unm,
                     (unsigned long)vrx->imu.gaps, (unsigned long)vrx->imu.lost,
                     (unsigned long)t.frames, (unsigned long)tx_miss, (unsigned long)t.miss_imu,
                     (unsigned long)t.late, (unsigned long)vtx->imu.gaps);
            printHist("all tx frames ", hist_all);
            printHist("lost on wire  ", hist_lost);
            printHist("lost imu      ", hist_lost_imu);
            ESP_LOGI(TAG, "lost with no tx record: %lu", (unsigned long)lost_no_tx);
        }
        prx = r;
        prx.miss = rx_miss;
        ptx = t;
        ptx.miss = tx_miss;
        prx_crc = vrx->stream.crcFailures();
        ptx_crc = vtx->stream.crcFailures();
        prx_gaps = vrx->imu.gaps;
        prx_lost = vrx->imu.lost;
        ptx_gaps = vtx->imu.gaps;
        n_lost_sec = 0;
        mmin_sec = INT32_MAX;
    }
}

// A new TX channel (setup, or the revert after an OTA): hook its on_sent.
static inline void onTxChannel(i2s_chan_handle_t h)
{
    if (!h || !task) return;
    i2s_event_callbacks_t cbs = {};
    cbs.on_sent = onSent;
    i2s_channel_disable(h);
    const esp_err_t e = i2s_channel_register_event_callback(h, &cbs, nullptr);
    i2s_channel_enable(h);
    tx_restarted = true;
    if (e != ESP_OK) ESP_LOGW(TAG, "on_sent hook: %s", esp_err_to_name(e));
}

// After beginMasterTx(): the parser task, the tx hook and the loopback.
static inline void begin(i2s_chan_handle_t h, int bclk, int ws, int dout, uint32_t sample_rate, int core)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    tx_ring.snaps = static_cast<Snap *>(heap_caps_malloc(sizeof(Snap) * SNAPS, caps));
    rx_ring.snaps = static_cast<Snap *>(heap_caps_malloc(sizeof(Snap) * SNAPS, caps));
    wlog = static_cast<i2slink::WriteLog<WRITES> *>(heap_caps_calloc(1, sizeof(i2slink::WriteLog<WRITES>), caps));
    margins = static_cast<Margin *>(heap_caps_calloc(MARGINS, sizeof(Margin), caps));
    void *vt = heap_caps_calloc(1, sizeof(View), caps);
    void *vr = heap_caps_calloc(1, sizeof(View), caps);
    if (!tx_ring.snaps || !rx_ring.snaps || !wlog || !margins || !vt || !vr || !h)
    {
        ESP_LOGE(TAG, "not started: no memory or no channel");
        wlog = nullptr;
        return;
    }
    vtx = new (vt) View();
    vrx = new (vr) View();
    us_per_byte = 1e6f / ((float)sample_rate * 4.0f);
    xTaskCreatePinnedToCore(monitorTask, "I2S Mon", 4096, nullptr, 1, &task, core);
    onTxChannel(h);

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    cc.dma_desc_num = 4;
    cc.dma_frame_num = 256;
    esp_err_t e = i2s_new_channel(&cc, nullptr, &loop_chan);
    if (e == ESP_OK)
    {
        i2s_std_config_t sc = {};
        sc.clk_cfg.sample_rate_hz = sample_rate;
        sc.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
        sc.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
        sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
        sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
        sc.gpio_cfg.bclk = (gpio_num_t)bclk;
        sc.gpio_cfg.ws = (gpio_num_t)ws;
        sc.gpio_cfg.dout = I2S_GPIO_UNUSED;
        sc.gpio_cfg.din = (gpio_num_t)dout;
        e = i2s_channel_init_std_mode(loop_chan, &sc);
        if (e == ESP_OK)
        {
            i2s_event_callbacks_t rcb = {};
            rcb.on_recv = onRecv;
            e = i2s_channel_register_event_callback(loop_chan, &rcb, nullptr);
        }
        if (e == ESP_OK) e = i2s_channel_enable(loop_chan);
    }
    ESP_LOGW(TAG, "I2S TX monitor on (bench build): loopback %s, internal heap left %u B",
             esp_err_to_name(e), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

}  // namespace i2smon
