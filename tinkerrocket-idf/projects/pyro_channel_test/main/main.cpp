/**
 * Pyro Channel Interactive Debug Tool — ESP32-P4
 *
 * Manual single-channel debug REPL over USB Serial JTAG. Lets you walk the
 * chain (ARM → P-FET → VN5E160 VCC → FIRE → OUTPUT → load → STATUS) one
 * step at a time while probing with a multimeter.
 *
 * Channels:
 *   1: ARM=14  FIRE=15  CONT=16
 *   2: ARM=22  FIRE=18  CONT=19
 *   3: ARM=37  FIRE=38  CONT=34
 *   4: ARM=49  FIRE=50  CONT=51
 *
 * Commands (one char, no Enter needed):
 *   ?   help
 *   s   show state
 *   1   select channel 1
 *   2   select channel 2
 *   3   select channel 3
 *   4   select channel 4
 *   a   toggle ARM on active channel (latching)
 *   f   pulse FIRE 500 ms (only if armed)
 *   F   pulse FIRE 2 s    (only if armed)
 *   c   read continuity (raw GPIO level — meaningful only when ARMed)
 *   C   momentary continuity test (arm 200 ms, read, disarm)
 *   r   read continuity raw on BOTH channels (compare)
 *   j   jam FIRE high indefinitely (must be armed — for DC probing)
 *   d   disarm everything + release jammed FIRE (safety)
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

struct PyroChannel {
    const char*  name;
    gpio_num_t   arm;
    gpio_num_t   fire;
    gpio_num_t   cont;
    bool         armed;
};

static PyroChannel ch[4] = {
    { "PYRO1", GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16, false },
    { "PYRO2", GPIO_NUM_22, GPIO_NUM_18, GPIO_NUM_19, false },
    { "PYRO3", GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_34, false },
    { "PYRO4", GPIO_NUM_49, GPIO_NUM_50, GPIO_NUM_51, false },
};

static int active = 0;  // 0..3

static void init_channel(PyroChannel& c)
{
    // gpio_reset_pin() forces IO MUX back to GPIO function — required on
    // ESP32-P4 because SPI2 default pins (14-16) and SPI3 default pins (17-19)
    // overlap with pyro pins. Without this, gpio_config() silently no-ops
    // because the pad is stuck in its peripheral default function.
    for (gpio_num_t pin : {c.arm, c.fire, c.cont}) {
        gpio_reset_pin(pin);
    }

    // INPUT_OUTPUT mode so gpio_get_level() reads back the actual pad state —
    // useful for diagnosing "I set it high but multimeter shows 0V" mysteries.
    gpio_config_t out_cfg = {};
    out_cfg.pin_bit_mask = (1ULL << c.arm) | (1ULL << c.fire);
    out_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
    out_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&out_cfg);
    gpio_set_level(c.arm,  0);
    gpio_set_level(c.fire, 0);

    gpio_config_t in_cfg = {};
    in_cfg.pin_bit_mask  = 1ULL << c.cont;
    in_cfg.mode          = GPIO_MODE_INPUT;
    in_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;   // PCB has 10k pullup to 3V3
    in_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    gpio_config(&in_cfg);

    c.armed = false;
}

static void disarm_all()
{
    for (auto& c : ch) {
        gpio_set_level(c.fire, 0);
        gpio_set_level(c.arm,  0);
        c.armed = false;
    }
}

static void print_help()
{
    printf("\n=== Pyro Channel Debug ===\n");
    printf("  ?   help\n");
    printf("  s   show state\n");
    printf("  1   select channel 1 (ARM=14 FIRE=15 CONT=16)\n");
    printf("  2   select channel 2 (ARM=22 FIRE=18 CONT=19)\n");
    printf("  3   select channel 3 (ARM=37 FIRE=38 CONT=34)\n");
    printf("  4   select channel 4 (ARM=49 FIRE=50 CONT=51)\n");
    printf("  a   toggle ARM on active channel\n");
    printf("  f   pulse FIRE 500 ms (must be armed)\n");
    printf("  F   pulse FIRE 2 s    (must be armed)\n");
    printf("  c   read continuity raw (active channel)\n");
    printf("  C   momentary continuity test (arm 200 ms, read, disarm)\n");
    printf("  r   read continuity raw on ALL channels\n");
    printf("  j   jam FIRE high indefinitely (must be armed — for DC probing)\n");
    printf("  b   blink FIRE at 1 Hz for 30 s (must be armed — for multimeter swing test)\n");
    printf("  d   disarm everything + release jammed FIRE\n");
    printf("============================\n");
}

static void print_state()
{
    printf("\nactive: %s\n", ch[active].name);
    for (auto& c : ch) {
        int cont = gpio_get_level(c.cont);
        printf("  %s  ARM=%d  CONT=%d  (cont meaningful only when ARM=1)\n",
               c.name, c.armed ? 1 : 0, cont);
    }
    printf("\n");
}

static void cmd_toggle_arm()
{
    auto& c = ch[active];
    c.armed = !c.armed;
    gpio_set_level(c.arm, c.armed ? 1 : 0);
    printf("[%s] ARM=%d\n", c.name, c.armed ? 1 : 0);
    if (c.armed) {
        vTaskDelay(pdMS_TO_TICKS(50));
        printf("[%s] CONT=%d (after arm settle)\n", c.name, gpio_get_level(c.cont));
    }
}

static void cmd_fire(uint32_t ms)
{
    auto& c = ch[active];
    if (!c.armed) {
        printf("[%s] REFUSED — not armed (press 'a' first)\n", c.name);
        return;
    }
    printf("[%s] FIRE %lu ms...\n", c.name, (unsigned long)ms);
    gpio_set_level(c.fire, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(c.fire, 0);
    printf("[%s] FIRE done. CONT=%d\n", c.name, gpio_get_level(c.cont));
}

static void cmd_cont_raw()
{
    auto& c = ch[active];
    printf("[%s] CONT=%d  (ARM=%d — only meaningful when ARM=1)\n",
           c.name, gpio_get_level(c.cont), c.armed ? 1 : 0);
}

static void cmd_cont_momentary()
{
    auto& c = ch[active];
    bool was_armed = c.armed;
    if (!was_armed) {
        gpio_set_level(c.arm, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    int v = gpio_get_level(c.cont);
    if (!was_armed) {
        gpio_set_level(c.arm, 0);
    }
    printf("[%s] momentary CONT=%d  (1=load, 0=open per VN5E160 STATUS)\n", c.name, v);
}

static void cmd_read_both()
{
    for (auto& c : ch) {
        printf("  %s  ARM=%d  CONT=%d\n", c.name, c.armed ? 1 : 0, gpio_get_level(c.cont));
    }
}

static void cmd_jam_fire()
{
    auto& c = ch[active];
    if (!c.armed) {
        printf("[%s] REFUSED — not armed (press 'a' first)\n", c.name);
        return;
    }
    gpio_set_level(c.fire, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    int arm_rb  = gpio_get_level(c.arm);
    int fire_rb = gpio_get_level(c.fire);
    printf("[%s] FIRE JAMMED — readback ARM(GPIO%d)=%d  FIRE(GPIO%d)=%d  (1 = pad reads high)\n",
           c.name, c.arm, arm_rb, c.fire, fire_rb);
    printf("       Probe with multimeter now. Press 'd' to release.\n");
}

// Blink FIRE at 1 Hz for ~30 s. Lets you watch a multimeter and see the
// trace voltage swing — useful when 'j' static probing gives ambiguous reads.
static volatile bool blink_running = false;
static void cmd_blink_fire()
{
    auto& c = ch[active];
    if (!c.armed) {
        printf("[%s] REFUSED — not armed (press 'a' first)\n", c.name);
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
        case 'r': case 'R':            cmd_read_both(); break;
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
    ESP_LOGI(TAG, "=== Pyro Channel Interactive Debug ===");
    vTaskDelay(pdMS_TO_TICKS(500));

    for (auto& c : ch) init_channel(c);

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
