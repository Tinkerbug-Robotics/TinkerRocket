// px1105r_p4_bridge.c -- the PX1105R board's receiver on USB and on the host UART: the board's first P4 image.
//
// hardware/legacy/gnss-px1105r-18mm-highpower-ext-ant puts a SkyTraq PX1105R (U1) behind an ESP32-P4 (U6). Every J4
// signal and the receiver's UART go only to the P4 (design review F1), so with a blank P4 the host hears nothing.
// This image copies bytes between the receiver and two ports and does nothing else:
//
//     receiver TXD                       -> USB-Serial-JTAG and the host's RX (J4.2)
//     USB-Serial-JTAG, host's TX (J4.1)  -> receiver RXD
//
// Both ports see the receiver's own NMEA / SkyTraq binary, unparsed. Today's Mantis driver speaks only UBX and will
// not parse this stream (F1): the host side is for the bench and for the next step, not for flight.
//
// Pins (schematic nets; the review's F2 UART map):
//     UART2  GPIO3  RX  <- GNSS_TX   receiver TXD
//            GPIO4  TX  -> GNSS_RX   receiver RXD
//     UART1  GPIO11 RX  <- HOST_TX   host TX, J4.1 -> R9
//            GPIO10 TX  -> HOST_RX   host RX, R8 -> J4.2
// Left alone on purpose:
//     GPIO8   GNSS_RSTN: open-drain only, R12 is the pull-up. Never drive it high.
//     GPIO15  GNSS_BOOT: drives Q1 (DTC123J), which pulls the receiver's BOOT_SEL low. Left floating, Q1's 47 k keeps
//             it off. Driving it high and then pulsing RSTN low boots the receiver into its ROM loader.
//     GPIO6   GNSS_PPS, GPIO12 (host TX2 via R11), GPIO2 (receiver RXD2, pulled up from reset): unused here.
//     GPIO24/25: USB-Serial-JTAG, the only programming path once the board is mounted.
//
// GPIO4 and GPIO10 float after reset. bootloader_components/uart_idle_pullups pulls both up at the start of the
// bootloader, so the receiver and the host see idle-high lines instead of noise while the P4 boots (review S3).
//
// Power budget: keep +3V3_MCU at or below about 0.2 A. The input choke FL2 is rated 320 mA per line, and the board
// draws 0.20-0.25 A with the P4 at its 150 mA dual-core maximum (review S6). This image idles far below that.
//
// Baud: 115200 on both UARTs, the receiver's factory rate. V_BCKP is tied to VCC (review F4), so every power cycle
// brings the receiver back to it. A baud change sent through the bridge (SkyTraq 0x05) loses the receiver until the
// next power cycle; a change written to its flash (attribute 1) loses it for good, so use SRAM only (attribute 0).
//
// Build and flash (ESP-IDF v6.0). The board has no button: download mode is TP4 (BOOT) shorted to TP2 (GND) plus a
// pulse on TP3 (EN), then flash over USB (review F3).
//
//     idf.py set-target esp32p4 && idf.py build
//     idf.py -p PORT flash

#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#define GNSS_UART      UART_NUM_2
#define GNSS_RX_PIN    3        // net GNSS_TX, receiver -> us
#define GNSS_TX_PIN    4        // net GNSS_RX, us -> receiver

#define HOST_UART      UART_NUM_1
#define HOST_RX_PIN    11       // net HOST_TX, host -> R9 -> us
#define HOST_TX_PIN    10       // net HOST_RX, us -> R8 -> host

#define BRIDGE_BAUD    115200   // PX1105R factory rate; the host side matches so the bridge stays transparent

// Receiver -> USB and host. A short USB write timeout on purpose: with no USB host attached that side never drains,
// and blocking here would let the receiver UART overflow instead. Bytes nobody is listening for are dropped.
static void gnss_out(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;)
    {
        const int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), pdMS_TO_TICKS(5));
        if (n > 0)
        {
            usb_serial_jtag_write_bytes(buf, (size_t)n, pdMS_TO_TICKS(20));
            uart_write_bytes(HOST_UART, buf, (size_t)n);
        }
    }
}

// USB -> receiver. Commands are a few tens of bytes; nothing to pace.
static void usb_in(void *arg)
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

// Host -> receiver.
static void host_in(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;)
    {
        const int n = uart_read_bytes(HOST_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0)
        {
            uart_write_bytes(GNSS_UART, buf, (size_t)n);
        }
    }
}

static void uart_start(uart_port_t port, int tx_pin, int rx_pin)
{
    const uart_config_t cfg = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, 4096, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void app_main(void)
{
    // The UARTs first: once a TX pad belongs to its UART it idles high on its own, and the bootloader's pull-ups
    // only have to cover the time until here.
    uart_start(GNSS_UART, GNSS_TX_PIN, GNSS_RX_PIN);
    uart_start(HOST_UART, HOST_TX_PIN, HOST_RX_PIN);

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_cfg.rx_buffer_size = 1024;
    usb_cfg.tx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    // Route any stray console write through the driver too, so it cannot race the bridge for the FIFO. The log
    // level is WARN, so in practice nothing is written after boot.
    usb_serial_jtag_vfs_use_driver();

    // One line so a boot capture says what is running. It starts with '#', which NMEA parsers skip. USB only: the
    // host's parser gets nothing but receiver bytes.
    static const char banner[] =
        "\r\n# px1105r_p4_bridge: receiver UART2 rx=GPIO3 tx=GPIO4, host UART1 rx=GPIO11 tx=GPIO10, 115200\r\n";
    usb_serial_jtag_write_bytes(banner, sizeof(banner) - 1, 0);

    xTaskCreate(gnss_out, "gnss_out", 3072, NULL, 10, NULL);
    xTaskCreate(usb_in, "usb_in", 3072, NULL, 9, NULL);
    xTaskCreate(host_in, "host_in", 3072, NULL, 9, NULL);
}
