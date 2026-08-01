#pragma once

#include <TR_LoRa_Comms.h>
#include <TR_UART_Link.h>

// On-hardware bench self-test for the radio daughterboard (#409).
//
// Built only with `idf.py -B build_bench -DTR_BENCH_SELFTEST=1 build`, never
// in a production image: the UART tests put the peripheral in internal
// loopback, and while loopback routes TX back to RX inside the chip, the TX
// pin keeps driving — a host on J6 would see the test frames.
//
// It exists because the two halves of this firmware were verifiable in
// completely different places. The framing codec has host tests
// (tests_cpp/test_uart_link_codec.cpp) but had never run on a real UART at a
// real baud rate, and the radio had never been asked to do anything beyond
// initialise. Neither gap needs a second board or even a UART adapter to
// close, so neither should have waited for one.
//
// Everything here is receive-only on the RF side. Nothing transmits, so it is
// safe to run with no antenna fitted.

namespace bench
{

// Runs the whole suite, logging results to the console. Returns the number of
// failed checks (0 = all good). Safe to call once, after uart_link.begin()
// and the radio's begin() attempt; leaves both in their normal state.
int runSelfTest(TR_UART_Link& link, TR_LoRa_Comms& radio, uart_port_t port,
                bool radio_up);

}  // namespace bench
