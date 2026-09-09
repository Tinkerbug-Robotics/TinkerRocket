#pragma once

// #1155 item 7: who a DIO1 edge belongs to.
//
// Both the DIO1 ISR and pollDio1() used to decide between rx_done_ and
// tx_done_ from rx_mode_ alone. A spectrum scan sets rx_mode_ = false while
// serviceScan() puts the chip INTO receive on every dwell channel, so a packet
// decoded during a dwell was booked as a transmit completion: tx_ok++ for a
// transmission that never happened, finishTransmit() dropped the receiver to
// standby mid-dwell, and startReceive() flipped rx_mode_ back on under the
// scan. The state that cannot lie is tx_ongoing_ — DIO1 is masked to TxDone
// for the whole time it is set — so: a transmit in flight owns the edge;
// otherwise an armed receiver does; otherwise (scan dwell, retune window)
// nobody does and the edge is ignored. Host-tested in
// tests_cpp/test_lora_dio1_policy.cpp.
namespace lora_dio1 {

enum class Owner : unsigned char { None = 0, Tx, Rx };

inline Owner classify(bool tx_ongoing, bool rx_mode)
{
    if (tx_ongoing) return Owner::Tx;
    if (rx_mode)    return Owner::Rx;
    return Owner::None;
}

}  // namespace lora_dio1
