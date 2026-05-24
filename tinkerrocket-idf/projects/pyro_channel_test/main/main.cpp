/**
 * Pyro Channel Interactive Debug Tool — ESP32-P4
 *
 * Manual debug REPL over USB Serial JTAG for the new rocket-computer PCB.
 * One shared ARM FET feeds four squib drivers; per-channel CONT/FIRE.
 * Walk the chain (ARM → upstream FET → channel FIRE → load → CONT readback)
 * one step at a time while probing with a multimeter.
 *
 * Pins (new PCB):
 *   PYRO_ARM = 14   (shared by all four channels)
 *   Channel 1: CONT=15  FIRE=16
 *   Channel 2: CONT=18  FIRE=19
 *   Channel 3: CONT=38  FIRE=34
 *   Channel 4: CONT=51  FIRE=50
 *
 * Continuity polarity (new PCB, inverted vs old board): raw GPIO 0 = closed
 * (load present), raw 1 = open. The "cont" line in printouts shows the
 * logical bit (1 = good continuity); the raw GPIO level is also shown so
 * you can verify on the multimeter. TODO: confirm polarity on bench before
 * trusting.
 *
 * Commands (one char, no Enter needed):
 *   ?   help
 *   s   show state
 *   1   select channel 1
 *   2   select channel 2
 *   3   select channel 3
 *   4   select channel 4
 *   a   toggle shared ARM (latching, affects ALL channels)
 *   f   pulse FIRE 500 ms on active channel (only if armed)
 *   F   pulse FIRE 2 s    on active channel (only if armed)
 *   c   read continuity on active channel (meaningful only when ARMed)
 *   C   momentary continuity test (arm 200 ms, read, disarm)
 *   r   read continuity raw on ALL channels (compare)
 *   j   jam FIRE high indefinitely on active channel (must be armed)
 *   b   blink FIRE @ 1 Hz for 30 s on active channel (multimeter swing test)
 *   d   disarm + release jammed FIRE pins (safety)
 */

#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <fcntl.h>
#include <unistd.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
// ESP32-P4 in IDF 5.3.2 has no ADC calibration scheme yet (per
// components/esp_adc/esp32p4/include/adc_cali_schemes.h), so we skip cali
// entirely and approximate mV from raw counts using the nominal 12-bit /
// 12 dB atten range. Rough — but plenty for "is this 2.2 V or 3.3 V?".
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PYRO_DBG";

static constexpr gpio_num_t PYRO_ARM_PIN = GPIO_NUM_14;

struct PyroChannel {
    const char*  name;
    gpio_num_t   fire;
    gpio_num_t   cont;
};

static PyroChannel ch[4] = {
    { "PYRO1", GPIO_NUM_16, GPIO_NUM_15 },
    { "PYRO2", GPIO_NUM_19, GPIO_NUM_18 },
    { "PYRO3", GPIO_NUM_34, GPIO_NUM_38 },
    { "PYRO4", GPIO_NUM_50, GPIO_NUM_51 },
};

static int  active     = 0;      // 0..3
static bool global_arm = false;  // mirrors PYRO_ARM_PIN level

// ADC handles for the two CONT pins that are ADC-capable on ESP32-P4:
//   PYRO2_CONT = GPIO 18 -> ADC1 channel 2
//   PYRO4_CONT = GPIO 51 -> ADC2 channel 2
// PYRO1_CONT (GPIO 15) and PYRO3_CONT (GPIO 38) have no ADC mapping; those
// channels stay on digital reads only.
static adc_oneshot_unit_handle_t adc1_h = nullptr;
static adc_oneshot_unit_handle_t adc2_h = nullptr;

struct ChannelAdc {
    adc_oneshot_unit_handle_t* unit;  // nullptr if no ADC on this channel
    adc_channel_t              channel;
};
static ChannelAdc adc_map[4] = {
    { nullptr,  (adc_channel_t)0 },   // PYRO1 GPIO15: no ADC
    { &adc1_h,  ADC_CHANNEL_2 },      // PYRO2 GPIO18: ADC1 ch2
    { nullptr,  (adc_channel_t)0 },   // PYRO3 GPIO38: no ADC
    { &adc2_h,  ADC_CHANNEL_2 },      // PYRO4 GPIO51: ADC2 ch2
};
static inline bool has_adc(int i) { return adc_map[i].unit != nullptr; }

// 12-bit ADC, ADC_ATTEN_DB_12 → ~0-3100 mV linear range. Real chip has some
// non-linearity at the rails; for our "2.2 V vs 3.3 V" question this is fine.
static inline int adc_raw_to_mv_rough(int raw) { return (raw * 3100) / 4095; }

// New-PCB inverted continuity (digital sense): raw 0 = closed (load present).
static inline bool cont_from_raw(int raw) { return raw == 0; }

// ADC-based continuity threshold. With the current divider (R29 50k pull-up,
// R85 100k isolation) and VPP ~3.3 V, expect ~3.3 V open and ~2.2 V with a
// shorted load. Threshold at 2.7 V (mid-band) — tune empirically once we see
// real numbers.
static constexpr int CONT_THRESHOLD_MV = 2700;
static inline bool cont_from_mv(int mv) { return mv > 0 && mv < CONT_THRESHOLD_MV; }

// Returns true if ADC read succeeded. Populates raw_out (0..4095) and
// mv_out (rough mV from the linear approximation above).
static bool read_adc_mv(int i, int* raw_out, int* mv_out)
{
    *raw_out = 0;
    *mv_out  = 0;
    if (!has_adc(i)) return false;
    if (adc_oneshot_read(*adc_map[i].unit, adc_map[i].channel, raw_out) != ESP_OK) {
        return false;
    }
    *mv_out = adc_raw_to_mv_rough(*raw_out);
    return true;
}

// Print a uniform CONT line for one channel, automatically picking digital
// or ADC read based on the pin's capability.
static void print_cont_for(int i, const char* prefix = "  ", const char* suffix = "")
{
    if (has_adc(i)) {
        int raw = 0, mv = 0;
        bool ok = read_adc_mv(i, &raw, &mv);
        if (ok) {
            printf("%s%s  ADC raw=%4d (%4d mV) -> cont=%d%s\n",
                   prefix, ch[i].name, raw, mv, cont_from_mv(mv) ? 1 : 0, suffix);
        } else {
            printf("%s%s  ADC read FAILED%s\n", prefix, ch[i].name, suffix);
        }
    } else {
        int raw = gpio_get_level(ch[i].cont);
        printf("%s%s  CONT raw=%d -> cont=%d%s\n",
               prefix, ch[i].name, raw, cont_from_raw(raw) ? 1 : 0, suffix);
    }
}

static void init_pins()
{
    // gpio_reset_pin() forces IO MUX back to GPIO function — required on
    // ESP32-P4 because pins 14-19 default to SPI2/SPI3 IO MUX. Without it,
    // gpio_config() silently no-ops because the pad is stuck in its
    // peripheral default function.
    gpio_reset_pin(PYRO_ARM_PIN);
    {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << PYRO_ARM_PIN;
        // INPUT_OUTPUT lets gpio_get_level() read back the actual pad state —
        // useful for "I set it high but multimeter shows 0V" diagnostics.
        cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(PYRO_ARM_PIN, 0);
    }

    for (int i = 0; i < 4; ++i) {
        auto& c = ch[i];
        gpio_reset_pin(c.fire);
        gpio_reset_pin(c.cont);

        gpio_config_t out_cfg = {};
        out_cfg.pin_bit_mask = 1ULL << c.fire;
        out_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
        out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&out_cfg);
        gpio_set_level(c.fire, 0);

        // CONT: only configure as a digital input on channels without ADC.
        // ADC-capable channels (2 and 4) get their pad set up by
        // adc_oneshot_config_channel() in init_adc().
        if (!has_adc(i)) {
            gpio_config_t in_cfg = {};
            in_cfg.pin_bit_mask  = 1ULL << c.cont;
            in_cfg.mode          = GPIO_MODE_INPUT;
            in_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
            in_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
            gpio_config(&in_cfg);
        }
    }

    global_arm = false;
}

static void init_adc()
{
    adc_oneshot_unit_init_cfg_t init1 = {};
    init1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init1, &adc1_h));

    adc_oneshot_unit_init_cfg_t init2 = {};
    init2.unit_id = ADC_UNIT_2;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init2, &adc2_h));

    adc_oneshot_chan_cfg_t chan = {};
    chan.atten    = ADC_ATTEN_DB_12;          // ~0-3.1 V input range
    chan.bitwidth = ADC_BITWIDTH_DEFAULT;     // 12-bit on ESP32-P4
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_h, ADC_CHANNEL_2, &chan));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_h, ADC_CHANNEL_2, &chan));
}

static void set_arm(bool on)
{
    gpio_set_level(PYRO_ARM_PIN, on ? 1 : 0);
    global_arm = on;
}

static void disarm_all()
{
    for (auto& c : ch) gpio_set_level(c.fire, 0);
    set_arm(false);
}

static void print_help()
{
    printf("\n=== Pyro Channel Debug (new PCB, shared ARM) ===\n");
    printf("  CONT readings: digital on ch1/ch3 (GPIO 15/38, no ADC).\n");
    printf("  Analog (ADC1/ADC2) on ch2 (GPIO18) and ch4 (GPIO51) -\n");
    printf("  use those to validate the divider topology.\n");
    printf("\n");
    printf("  ?   help\n");
    printf("  s   show state\n");
    printf("  1   select channel 1 (CONT=15 FIRE=16, digital)\n");
    printf("  2   select channel 2 (CONT=18 FIRE=19, ADC1)\n");
    printf("  3   select channel 3 (CONT=38 FIRE=34, digital)\n");
    printf("  4   select channel 4 (CONT=51 FIRE=50, ADC2)\n");
    printf("  a   toggle shared ARM (pin 14, affects ALL channels)\n");
    printf("  f   pulse FIRE 500 ms on active channel (must be armed)\n");
    printf("  F   pulse FIRE 2 s    on active channel (must be armed)\n");
    printf("  c   read continuity on active channel\n");
    printf("  C   momentary continuity test (arm 200 ms, read, disarm)\n");
    printf("  r   read continuity raw on ALL channels\n");
    printf("  j   jam FIRE high indefinitely on active channel\n");
    printf("  b   blink FIRE @ 1 Hz for 30 s on active channel\n");
    printf("  d   disarm + release jammed FIRE pins\n");
    printf("================================================\n");
}

static void print_state()
{
    printf("\nactive: %s   ARM(GPIO%d)=%d\n",
           ch[active].name, PYRO_ARM_PIN, global_arm ? 1 : 0);
    for (int i = 0; i < 4; ++i) {
        char suffix[64];
        snprintf(suffix, sizeof(suffix), "  FIRE(GPIO%d) rb=%d",
                 ch[i].fire, gpio_get_level(ch[i].fire));
        print_cont_for(i, "  ", suffix);
    }
    printf("  (cont meaningful only when ARM=1)\n\n");
}

static void cmd_toggle_arm()
{
    set_arm(!global_arm);
    printf("ARM=%d  (pin %d readback=%d)\n",
           global_arm ? 1 : 0, PYRO_ARM_PIN, gpio_get_level(PYRO_ARM_PIN));
    if (global_arm) {
        vTaskDelay(pdMS_TO_TICKS(50));
        for (int i = 0; i < 4; ++i) print_cont_for(i);
    }
}

static void cmd_fire(uint32_t ms)
{
    auto& c = ch[active];
    if (!global_arm) {
        printf("[%s] REFUSED — ARM not asserted (press 'a' first)\n", c.name);
        return;
    }
    printf("[%s] FIRE %lu ms...\n", c.name, (unsigned long)ms);
    gpio_set_level(c.fire, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(c.fire, 0);
    printf("[%s] FIRE done.\n", c.name);
    print_cont_for(active);
}

static void cmd_cont_raw()
{
    char suffix[40];
    snprintf(suffix, sizeof(suffix), "  (ARM=%d)", global_arm ? 1 : 0);
    print_cont_for(active, "[active] ", suffix);
}

static void cmd_cont_momentary()
{
    bool was_armed = global_arm;
    if (!was_armed) {
        set_arm(true);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    printf("[%s] momentary cont test:\n", ch[active].name);
    print_cont_for(active, "  ");
    if (!was_armed) set_arm(false);
    printf("  (new PCB: digital raw 0 = closed; ADC mV < %d -> closed)\n",
           CONT_THRESHOLD_MV);
}

static void cmd_read_all()
{
    printf("ARM=%d  (pin %d)\n", global_arm ? 1 : 0, PYRO_ARM_PIN);
    for (int i = 0; i < 4; ++i) {
        char suffix[40];
        snprintf(suffix, sizeof(suffix), "  FIRE rb=%d", gpio_get_level(ch[i].fire));
        print_cont_for(i, "  ", suffix);
    }
}

static void cmd_jam_fire()
{
    auto& c = ch[active];
    if (!global_arm) {
        printf("[%s] REFUSED — ARM not asserted (press 'a' first)\n", c.name);
        return;
    }
    gpio_set_level(c.fire, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    int arm_rb  = gpio_get_level(PYRO_ARM_PIN);
    int fire_rb = gpio_get_level(c.fire);
    printf("[%s] FIRE JAMMED — readback ARM(GPIO%d)=%d  FIRE(GPIO%d)=%d  (1 = pad reads high)\n",
           c.name, PYRO_ARM_PIN, arm_rb, c.fire, fire_rb);
    printf("       Probe with multimeter now. Press 'd' to release.\n");
}

// Blink FIRE at 1 Hz for ~30 s. Lets you watch a multimeter and see the
// trace voltage swing — useful when 'j' static probing gives ambiguous reads.
static volatile bool blink_running = false;
static void cmd_blink_fire()
{
    auto& c = ch[active];
    if (!global_arm) {
        printf("[%s] REFUSED — ARM not asserted (press 'a' first)\n", c.name);
        return;
    }
    printf("[%s] BLINKING FIRE @ 1 Hz for 30 s — watch the multimeter, expect 0V<->~3V swing\n", c.name);
    blink_running = true;
    for (int i = 0; i < 30 && blink_running; i++) {
        gpio_set_level(c.fire, 1);
        printf("  t=%2ds  FIRE=1  readback=%d\n", i, gpio_get_level(c.fire));
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(c.fire, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    gpio_set_level(c.fire, 0);
    printf("[%s] blink done\n", c.name);
}

static void handle(char k)
{
    switch (k) {
        case '?': case 'h': case 'H':  print_help(); break;
        case 's': case 'S':            print_state(); break;
        case '1': case '2': case '3': case '4':
            active = k - '1';
            printf("active = %s\n", ch[active].name);
            break;
        case 'a': case 'A':            cmd_toggle_arm(); break;
        case 'f':                      cmd_fire(500); break;
        case 'F':                      cmd_fire(2000); break;
        case 'c':                      cmd_cont_raw(); break;
        case 'C':                      cmd_cont_momentary(); break;
        case 'r': case 'R':            cmd_read_all(); break;
        case 'j': case 'J':            cmd_jam_fire(); break;
        case 'b': case 'B':            cmd_blink_fire(); break;
        case 'd': case 'D':
            disarm_all();
            printf("ALL DISARMED\n");
            break;
        case '\r': case '\n': case ' ': case '\t':
            break;  // ignore whitespace
        default:
            printf("unknown '%c' — '?' for help\n", k);
            break;
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "=== Pyro Channel Interactive Debug (new PCB) ===");
    vTaskDelay(pdMS_TO_TICKS(500));

    init_pins();
    init_adc();

    // Non-blocking stdin so we can poll without freezing the task.
    int flags = fcntl(fileno(stdin), F_GETFL, 0);
    fcntl(fileno(stdin), F_SETFL, flags | O_NONBLOCK);
    // Disable line buffering so individual keystrokes arrive immediately.
    setvbuf(stdin,  nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    print_help();
    print_state();

    while (true) {
        int c = getchar();
        if (c != EOF) {
            handle((char)c);
        } else {
            clearerr(stdin);  // some VFS impls latch EOF in nonblocking mode
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
