/**
 * Pyro Channel Interactive Debug Tool — ESP32-P4
 *
 * Manual debug REPL over USB Serial JTAG for the new rocket-computer PCB.
 * One shared ARM FET feeds four squib drivers; per-channel CONT/FIRE.
 * Walk the chain (ARM → upstream FET → channel FIRE → load → CONT readback)
 * one step at a time while probing with a multimeter.
 *
 * Pins are board-revision dependent (#411):
 *   V7 (default):            ARM=14, CONT/FIRE = 15/16 18/19 38/34 51/50
 *   V8 (-DTR_BOARD_V8=1):    ARM=5,  CONT/FIRE = 7/6 10/9 12/11 14/13
 *   V9/V10 (-DTR_BOARD_V9=1):ARM=16, CONT/FIRE = 7/6 10/11 12/9 14/13
 * Build the V9 map with:  idf.py -B build_v9 -DTR_BOARD_V9=1 build
 *
 * V8 vs V9 is not cosmetic: ARM moves (5 -> 16) and channels 2 and 3 swap
 * their FIRE pins while keeping their CONT pins, so the wrong build fires the
 * wrong connector with continuity still reading correctly. hardware/
 * rocket-computer/ is the V9/V10 board; the V8 boards are the older bench
 * units. The selected map is printed at startup — read it before arming.
 *
 * V8 NOTE: the CONT sense path is powered through the arming FET (test-board
 * circuit) — use 'C' (momentary arm) or arm first; unarmed 'c'/'r' reads are
 * meaningless on V8.
 *
 * Continuity polarity (both boards since V7): raw GPIO 0 = closed
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
 *   c   read continuity on active channel
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PYRO_DBG";

#ifndef TR_BOARD_V8
#define TR_BOARD_V8 0
#endif
#ifndef TR_BOARD_V9
#define TR_BOARD_V9 0
#endif
#if TR_BOARD_V8 && TR_BOARD_V9
#error "Pick one board: -DTR_BOARD_V8=1 or -DTR_BOARD_V9=1, not both."
#endif

#if TR_BOARD_V9
#define TR_BOARD_REV_STR "V9/V10"
#elif TR_BOARD_V8
#define TR_BOARD_REV_STR "V8"
#else
#define TR_BOARD_REV_STR "V7"
#endif

struct PyroChannel {
    const char*  name;
    gpio_num_t   fire;
    gpio_num_t   cont;
};

#if TR_BOARD_V9
// V9/V10 rocket computer (P4): mirror of projects/flight_computer/main/board/board_v9.h
static constexpr gpio_num_t PYRO_ARM_PIN = GPIO_NUM_16;
static PyroChannel ch[4] = {
    { "PYRO1", GPIO_NUM_6,  GPIO_NUM_7  },
    { "PYRO2", GPIO_NUM_11, GPIO_NUM_10 },
    { "PYRO3", GPIO_NUM_9,  GPIO_NUM_12 },
    { "PYRO4", GPIO_NUM_13, GPIO_NUM_14 },
};
#elif TR_BOARD_V8
// V8 rocket computer (P4): mirror of projects/flight_computer/main/board/board_v8.h
static constexpr gpio_num_t PYRO_ARM_PIN = GPIO_NUM_5;
static PyroChannel ch[4] = {
    { "PYRO1", GPIO_NUM_6,  GPIO_NUM_7  },
    { "PYRO2", GPIO_NUM_9,  GPIO_NUM_10 },
    { "PYRO3", GPIO_NUM_11, GPIO_NUM_12 },
    { "PYRO4", GPIO_NUM_13, GPIO_NUM_14 },
};
#else
// V7 rocket computer: mirror of .../board/board_v7.h
static constexpr gpio_num_t PYRO_ARM_PIN = GPIO_NUM_14;
static PyroChannel ch[4] = {
    { "PYRO1", GPIO_NUM_16, GPIO_NUM_15 },
    { "PYRO2", GPIO_NUM_19, GPIO_NUM_18 },
    { "PYRO3", GPIO_NUM_34, GPIO_NUM_38 },
    { "PYRO4", GPIO_NUM_50, GPIO_NUM_51 },
};
#endif

static int  active     = 0;      // 0..3
static bool global_arm = false;  // mirrors PYRO_ARM_PIN level

// New-PCB inverted continuity: raw 0 = closed (load present).
static inline bool cont_from_raw(int raw) { return raw == 0; }

#include <esp_private/gpio.h>      // gpio_func_sel
#include <rom/gpio.h>              // esp_rom_gpio_connect_out_signal
#include <soc/gpio_sig_map.h>      // SIG_GPIO_OUT_IDX (implicit before IDF v6)

// Safe ARM/FIRE pad init — same recipe as the flight firmware's
// safePyroOutputInit (see FC main.cpp / commit 421dd63): NEVER
// gpio_reset_pin() an output pad wired to a DTC123J gate driver — its brief
// internal pull-up twitches the driver hard enough to flash the pyro rail.
// Stage 0 first, detach any peripheral matrix route, force plain GPIO mux,
// then enable drive.
static void safe_output_init(gpio_num_t pin)
{
    gpio_set_level(pin, 0);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    gpio_func_sel(pin, PIN_FUNC_GPIO);
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    // INPUT_OUTPUT lets gpio_get_level() read back the actual pad state —
    // useful for "I set it high but multimeter shows 0V" diagnostics.
    cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(pin, 0);
}

static void init_pins()
{
    safe_output_init(PYRO_ARM_PIN);

    for (auto& c : ch) {
        safe_output_init(c.fire);

        // CONT is an input pad (no gate driver downstream) — the brief
        // pull-up from gpio_reset_pin() is harmless here.
        gpio_reset_pin(c.cont);

        gpio_config_t in_cfg = {};
        in_cfg.pin_bit_mask  = 1ULL << c.cont;
        in_cfg.mode          = GPIO_MODE_INPUT;
        in_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;   // PCB has external pull on CONT
        in_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
        gpio_config(&in_cfg);
    }

    global_arm = false;
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
    printf("\n=== Pyro Channel Debug (%s pin map, shared ARM) ===\n",
           TR_BOARD_REV_STR);
    printf("  ?   help\n");
    printf("  s   show state\n");
    for (int i = 0; i < 4; i++) {
        printf("  %d   select channel %d (CONT=%d FIRE=%d)\n",
               i + 1, i + 1, (int)ch[i].cont, (int)ch[i].fire);
    }
    printf("  a   toggle shared ARM (pin %d, affects ALL channels)\n",
           (int)PYRO_ARM_PIN);
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
    for (auto& c : ch) {
        int raw = gpio_get_level(c.cont);
        printf("  %s  CONT raw=%d -> cont=%d  FIRE(GPIO%d) rb=%d\n",
               c.name, raw, cont_from_raw(raw) ? 1 : 0,
               c.fire, gpio_get_level(c.fire));
    }
    printf("\n");
}

static void cmd_toggle_arm()
{
    set_arm(!global_arm);
    printf("ARM=%d  (pin %d readback=%d)\n",
           global_arm ? 1 : 0, PYRO_ARM_PIN, gpio_get_level(PYRO_ARM_PIN));
    if (global_arm) {
        vTaskDelay(pdMS_TO_TICKS(50));
        for (auto& c : ch) {
            int raw = gpio_get_level(c.cont);
            printf("  %s  CONT raw=%d -> cont=%d\n",
                   c.name, raw, cont_from_raw(raw) ? 1 : 0);
        }
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
    int raw = gpio_get_level(c.cont);
    printf("[%s] FIRE done. CONT raw=%d -> cont=%d\n",
           c.name, raw, cont_from_raw(raw) ? 1 : 0);
}

static void cmd_cont_raw()
{
    auto& c = ch[active];
    int raw = gpio_get_level(c.cont);
    printf("[%s] CONT raw=%d -> cont=%d\n",
           c.name, raw, cont_from_raw(raw) ? 1 : 0);
}

static void cmd_cont_momentary()
{
    auto& c = ch[active];
    bool was_armed = global_arm;
    if (!was_armed) {
        set_arm(true);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    int raw = gpio_get_level(c.cont);
    if (!was_armed) set_arm(false);
    printf("[%s] momentary CONT raw=%d -> cont=%d  "
           "(new PCB: raw 0 = closed/load present, raw 1 = open)\n",
           c.name, raw, cont_from_raw(raw) ? 1 : 0);
}

static void cmd_read_all()
{
    printf("ARM=%d  (pin %d)\n", global_arm ? 1 : 0, PYRO_ARM_PIN);
    for (auto& c : ch) {
        int raw = gpio_get_level(c.cont);
        printf("  %s  CONT raw=%d -> cont=%d  FIRE rb=%d\n",
               c.name, raw, cont_from_raw(raw) ? 1 : 0, gpio_get_level(c.fire));
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
