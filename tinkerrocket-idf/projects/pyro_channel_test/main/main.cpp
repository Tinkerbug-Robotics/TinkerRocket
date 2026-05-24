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

// New-PCB inverted continuity: raw 0 = closed (load present).
static inline bool cont_from_raw(int raw) { return raw == 0; }

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

    for (auto& c : ch) {
        gpio_reset_pin(c.fire);
        gpio_reset_pin(c.cont);

        gpio_config_t out_cfg = {};
        out_cfg.pin_bit_mask = 1ULL << c.fire;
        out_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
        out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&out_cfg);
        gpio_set_level(c.fire, 0);

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
    printf("\n=== Pyro Channel Debug (new PCB, shared ARM) ===\n");
    printf("  ?   help\n");
    printf("  s   show state\n");
    printf("  1   select channel 1 (CONT=15 FIRE=16)\n");
    printf("  2   select channel 2 (CONT=18 FIRE=19)\n");
    printf("  3   select channel 3 (CONT=38 FIRE=34)\n");
    printf("  4   select channel 4 (CONT=51 FIRE=50)\n");
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
