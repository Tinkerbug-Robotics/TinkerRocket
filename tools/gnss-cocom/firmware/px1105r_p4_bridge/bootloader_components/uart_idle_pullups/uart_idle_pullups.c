// uart_idle_pullups.c -- hold the P4's two UART outputs idle-high from the start of the 2nd-stage bootloader.
//
// After reset the P4 leaves GPIO4 as an input with no pull and GPIO10 fully high-impedance (datasheet v0.7,
// Table 2-1). GPIO4 drives the receiver's RXD (net GNSS_RX), which the PX1105R wants held high when idle; GPIO10
// drives the host's RX through R8 and J4.2 (net HOST_RX), which has no pull on either board. Both lines would float
// until the app starts its UARTs, 0.1-0.5 s into every boot (design review S3), and feed noise to both parsers.
//
// This hook runs before anything else in the bootloader and turns on both pads' weak pull-ups, so the lines idle
// high a few milliseconds after reset. The app's UARTs take the pads over later and leave the pull-ups on, which is
// harmless on push-pull outputs. Not covered: the ROM phase before this hook, download mode, and a blank flash.
//
// GPIO2 (net GNSS_RXD2, receiver RXD2) already comes out of reset with its pull-up on, so it needs nothing here.

#include "hal/gpio_ll.h"

#define GNSS_RX_PAD 4    // net GNSS_RX: us -> receiver RXD
#define HOST_RX_PAD 10   // net HOST_RX: us -> R8 -> J4.2 -> host RX

// Referenced by the bootloader's link so this file is kept.
void bootloader_hooks_include(void)
{
}

void bootloader_before_init(void)
{
    gpio_ll_pulldown_dis(&GPIO, GNSS_RX_PAD);
    gpio_ll_pullup_en(&GPIO, GNSS_RX_PAD);
    gpio_ll_pulldown_dis(&GPIO, HOST_RX_PAD);
    gpio_ll_pullup_en(&GPIO, HOST_RX_PAD);
}
