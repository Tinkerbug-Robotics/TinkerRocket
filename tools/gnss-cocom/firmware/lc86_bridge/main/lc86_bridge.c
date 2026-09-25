// lc86_bridge.c -- the Tinker-Beetle's LC86G on USB, for the COCOM rig (#491).
//
// The Beetle (rocket-computer-mini, M1) keeps its GNSS on the flight
// computer's UART: U5, a Quectel LC86G, on GPIO39 (module TXD -> us) and
// GPIO40 (us -> module RXD), per projects/flight_computer/main/board/board_m1.h.
// Nothing raw ever reaches USB in the flight image -- the driver parses the
// stream and keeps only what the EKF needs -- so a bench run needs this image
// instead. It copies bytes both ways between that UART and USB-Serial-JTAG and
// does nothing else, so the host sees the module's own sentences unparsed and
// can send it PAIR/PQTM commands. tools/gnss-cocom/sdr/lc86_config.py applies
// the same configuration the flight driver applies at boot.
//
// The receiver is not reset by flashing or by this image: it sits on
// V_MCU_SWTCH, which the OUT computer switches (cmd 8), not on this chip's
// reset. Whatever was configured before stays configured until that rail
// cycles -- and so does whatever was NOT.
//
// SAFETY: the pyro ARM and FIRE outputs are parked LOW before anything else,
// exactly as the flight image's safePyroOutputInit() does. Left at their reset
// state the pads can carry the ~50 k internal pull-up (GPIO44 is U0RXD), which
// biases the DTC123J gate drivers' bases -- the flight image documents the
// pyro rail twitching at every boot from exactly that. This image runs for
// hours, not milliseconds, so it holds them low for the whole session.
//
// Build and flash (ESP-IDF v6.0), identifying the chip by MAC first:
//
//     idf.py set-target esp32s3 && idf.py build
//     python -m esptool --port PORT --after no-reset read-mac   # 9C:13:9E:28:9E:8C
//     idf.py -p PORT flash
//
// Put the flight image back afterwards (idf.py -B build_m1 -p PORT flash in
// tinkerrocket-idf/projects/flight_computer); it rewrites the bootloader, the
// partition table and otadata, and leaves NVS alone, as this image does.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_private/gpio.h"      // gpio_func_sel
#include "rom/gpio.h"              // esp_rom_gpio_connect_out_signal
#include "soc/gpio_sig_map.h"      // SIG_GPIO_OUT_IDX
#include "soc/io_mux_reg.h"        // PIN_FUNC_GPIO

// board_m1.h, net names from the module's side: GNSS_TX is the module's
// output, so it is our receive pin.
#define GNSS_UART      UART_NUM_1
#define GNSS_RX_PIN    39   // net GNSS_TX, module -> us
#define GNSS_TX_PIN    40   // net GNSS_RX, us -> module
#define GNSS_BAUD      115200  // LC86G power-on default; the flight driver never re-bauds

// board_m1.h pyro outputs: FC_ARM and the four FIRE lines.
static const gpio_num_t kPyroOutputs[] = {
    (gpio_num_t)44,  // PYRO_ARM_PIN   (FC_ARM, U0RXD)
    (gpio_num_t)38,  // PYRO1_FIRE_PIN
    (gpio_num_t)35,  // PYRO2_FIRE_PIN
    (gpio_num_t)34,  // PYRO3_FIRE_PIN
    (gpio_num_t)33,  // PYRO4_FIRE_PIN
};

// The flight image's safePyroOutputInit(), step for step: stage a 0, detach
// any peripheral, select plain GPIO, then enable the driver with no pulls, so
// the pad goes from high-Z straight to driving 0 with no pull-up window.
// gpio_reset_pin() must not be used here -- it enables the pull-up first.
static void park_low(gpio_num_t pin)
{
    gpio_set_level(pin, 0);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    gpio_func_sel(pin, PIN_FUNC_GPIO);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(pin, 0);
}

// A module that has gone completely quiet must look different from a bridge
// that has died. Quectel documents exactly that silence for this part: above
// 50 km in every navigation mode but Balloon (80 km in Balloon), the LC86G
// stops ALL output, NMEA and acks included. So after SILENT_MS without a
// byte, the bridge says so once a second. The fix interval is 100-1000 ms, so
// two seconds of nothing is never an ordinary gap between bursts.
#define SILENT_MS 2000

// Module -> host. A short USB write timeout on purpose: with no host attached
// the USB side never drains, and blocking here would let the UART overflow
// instead. Bytes nobody is listening for are simply dropped.
static void gnss_to_usb(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    TickType_t last_rx = xTaskGetTickCount();
    TickType_t last_note = last_rx;
    for (;;)
    {
        const int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), pdMS_TO_TICKS(5));
        const TickType_t now = xTaskGetTickCount();
        if (n > 0)
        {
            usb_serial_jtag_write_bytes(buf, (size_t)n, pdMS_TO_TICKS(20));
            last_rx = now;
            continue;
        }
        if (now - last_rx >= pdMS_TO_TICKS(SILENT_MS) &&
            now - last_note >= pdMS_TO_TICKS(1000))
        {
            // Leading CRLF ends any half-received sentence first, so the note
            // can never be glued onto NMEA. It starts with '#', which the
            // rig's parser skips but its captures keep, with a timestamp.
            char line[64];
            const int len = snprintf(line, sizeof(line),
                                     "\r\n# lc86_bridge: LC86G silent %lu ms\r\n",
                                     (unsigned long)pdTICKS_TO_MS(now - last_rx));
            usb_serial_jtag_write_bytes(line, (size_t)len, pdMS_TO_TICKS(20));
            last_note = now;
        }
    }
}

// Host -> module. Commands are a few tens of bytes; nothing to pace.
static void usb_to_gnss(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;)
    {
        const int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0)
        {
            uart_write_bytes(GNSS_UART, buf, (size_t)n);
        }
    }
}

void app_main(void)
{
    for (size_t i = 0; i < sizeof(kPyroOutputs) / sizeof(kPyroOutputs[0]); i++)
    {
        park_low(kPyroOutputs[i]);
    }

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_cfg.rx_buffer_size = 1024;
    usb_cfg.tx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    // Route any stray console write through the driver too, so it cannot
    // race the bridge for the FIFO. The log level is WARN, so in practice
    // nothing is written after boot.
    usb_serial_jtag_vfs_use_driver();

    const uart_config_t uart_cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART, 4096, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // One line so a boot capture says what is running. It starts with '#', so
    // the rig's NMEA demuxer skips it like any other non-sentence text.
    static const char banner[] =
        "\r\n# lc86_bridge: LC86G on UART1 rx=GPIO39 tx=GPIO40 @115200, "
        "pyro ARM/FIRE parked low\r\n";
    usb_serial_jtag_write_bytes(banner, sizeof(banner) - 1, 0);

    xTaskCreate(gnss_to_usb, "gnss_to_usb", 3072, NULL, 10, NULL);
    xTaskCreate(usb_to_gnss, "usb_to_gnss", 3072, NULL, 9, NULL);
}
